#include "airlive-connection.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <vector>

#include <obs-module.h>

#include "auth.hpp"
#include "net-compat.hpp"
#include "wire.hpp"

namespace airlive {

namespace {
// How long accept()/recv() poll before re-checking the running flag. Short
// enough that stop() returns promptly, long enough not to spin the CPU.
constexpr int kPollTimeoutMs = 250;
constexpr size_t kRecvChunk = 64 * 1024;

// A live feed delivers frames continuously. If no bytes arrive for this long,
// the peer is gone — even if it never sent a TCP FIN (app backgrounded, Wi-Fi
// dropped, walked out of range). Reaping the dead socket frees the single slot
// so the iPhone can reconnect. Without this the worker would loop forever on a
// half-open connection and block every reconnect.
constexpr int kStallTimeoutMs = 8000;

// Auth handshake budget: how long we wait for a valid response before FINning a
// connection that never authenticates. Pinned to 15 s across ALL receivers
// (STREAM-AUTH-SPEC §4) so behavior is consistent with the Bridge / Studio.
constexpr int kAuthStallTimeoutMs = 15000;

// Anti-bruteforce: after this many failed attempts from one source, ban it for
// an exponentially growing window (base × 2^over), capped.
constexpr int kAuthMaxFailures = 5;
constexpr int64_t kAuthBanBaseMs = 30000;
constexpr int64_t kAuthBanMaxMs = 600000;

void setNonBlocking(socket_t s) {
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(s, FIONBIO, &mode);
#else
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
}

// Monotonic milliseconds for ban deadlines (never walks backward on a clock set).
int64_t nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

// Human-readable remote IP for the anti-bruteforce key. The listen socket is
// dual-stack, so peers arrive as AF_INET6 (IPv4 shows as "::ffff:a.b.c.d").
std::string peerKeyFromSockaddr(const sockaddr_storage &ss) {
    char buf[INET6_ADDRSTRLEN] = {0};
    if (ss.ss_family == AF_INET6) {
        auto *a = reinterpret_cast<const sockaddr_in6 *>(&ss);
        inet_ntop(AF_INET6, &a->sin6_addr, buf, sizeof(buf));
    } else if (ss.ss_family == AF_INET) {
        auto *a = reinterpret_cast<const sockaddr_in *>(&ss);
        inet_ntop(AF_INET, &a->sin_addr, buf, sizeof(buf));
    }
    return buf[0] ? std::string(buf) : std::string("?");
}
} // namespace

AirliveConnection::AirliveConnection(ServiceIdentity identity, FrameSink sink, ControlSink control)
    : identity_(std::move(identity)),
      decoder_(std::move(sink)),
      controlSink_(std::move(control)) {}

AirliveConnection::~AirliveConnection() { stop(); }

void AirliveConnection::start() {
    if (running_.exchange(true))
        return;
    thread_ = std::thread([this] { run(); });
}

void AirliveConnection::stop() {
    if (!running_.exchange(false))
        return;
    if (thread_.joinable())
        thread_.join();
}

// Bind a dual-stack IPv6 socket on port 0 so the OS assigns a free port (and a
// dual-stack iPhone reaches us over either IPv4 or IPv6). Returns the chosen
// port via outPort, or kInvalidSocket on failure.
socket_t AirliveConnection::openListenSocket(uint16_t &outPort) {
    socket_t fd = ::socket(AF_INET6, SOCK_STREAM, 0);
    if (fd == kInvalidSocket) {
        blog(LOG_ERROR, "[airlive] socket() failed");
        return kInvalidSocket;
    }

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes), sizeof(yes));
    int no = 0; // accept both IPv6 and IPv4-mapped clients
    setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, reinterpret_cast<const char *>(&no), sizeof(no));

    sockaddr_in6 addr{};
    addr.sin6_family = AF_INET6;
    addr.sin6_addr = in6addr_any;
    addr.sin6_port = 0; // ephemeral

    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        blog(LOG_ERROR, "[airlive] bind() failed");
        close_socket(fd);
        return kInvalidSocket;
    }
    if (::listen(fd, 1) != 0) {
        blog(LOG_ERROR, "[airlive] listen() failed");
        close_socket(fd);
        return kInvalidSocket;
    }

    sockaddr_in6 bound{};
    socklen_t len = sizeof(bound);
    if (::getsockname(fd, reinterpret_cast<sockaddr *>(&bound), &len) != 0) {
        blog(LOG_ERROR, "[airlive] getsockname() failed");
        close_socket(fd);
        return kInvalidSocket;
    }
    outPort = ntohs(bound.sin6_port);
    setNonBlocking(fd);
    return fd;
}

void AirliveConnection::run() {
    // Winsock is started once per module (obs_module_load), not per worker —
    // WSACleanup is reference-counted, so doing it per source could tear Winsock
    // out from under the rest of OBS.
    uint16_t port = 0;
    socket_t listenFd = openListenSocket(port);
    if (listenFd == kInvalidSocket) {
        running_ = false;
        return;
    }

    blog(LOG_INFO, "[airlive] listening on ephemeral port %u", port);
    bonjour_.start(port, identity_);

    while (running_.load()) {
        // Poll the listen socket and, when present, the dns_sd socket so the
        // mDNS registration/conflict callbacks actually get serviced.
        poll_fd_t pfds[2]{};
        pfds[0].fd = listenFd;
        pfds[0].events = POLLIN;
        unsigned long nfds = 1;
        const int bonjourFd = bonjour_.fd();
        if (bonjourFd >= 0) {
            pfds[1].fd = (socket_t)bonjourFd;
            pfds[1].events = POLLIN;
            nfds = 2;
        }
        const int pr = poll_sockets(pfds, nfds, kPollTimeoutMs);
        if (pr <= 0)
            continue; // timeout (re-check running_) or transient error

        if (nfds == 2 && (pfds[1].revents & POLLIN))
            bonjour_.process(); // drain a dns_sd callback (e.g. "advertised" / conflict)
        if (!(pfds[0].revents & POLLIN))
            continue; // only the bonjour fd was ready

        sockaddr_storage peer{};
        socklen_t peerLen = sizeof(peer);
        socket_t client = ::accept(listenFd, reinterpret_cast<sockaddr *>(&peer), &peerLen);
        if (client == kInvalidSocket)
            continue;

        const std::string peerKey = peerKeyFromSockaddr(peer);
        // Anti-bruteforce: a source that just burned its auth attempts is refused
        // outright (no slot, no challenge) until its ban expires.
        if (isBanned(peerKey)) {
            blog(LOG_INFO, "[airlive] refused banned source %s", peerKey.c_str());
            close_socket(client);
            continue;
        }

        blog(LOG_INFO, "[airlive] iPhone connected (%s)", peerKey.c_str());
#ifdef SO_NOSIGPIPE
        // macOS/BSD: never raise SIGPIPE if the iPhone vanishes mid-send.
        int one = 1;
        setsockopt(client, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
        // Keepalive as a backstop for a peer that dies without a FIN; the
        // stall timeout in serveClient is the primary, faster detector.
        int ka = 1;
        setsockopt(client, SOL_SOCKET, SO_KEEPALIVE, reinterpret_cast<const char *>(&ka), sizeof(ka));
        int idle = 10; // seconds idle before the first keepalive probe
#if defined(TCP_KEEPALIVE)
        setsockopt(client, IPPROTO_TCP, TCP_KEEPALIVE, &idle, sizeof(idle)); // macOS/BSD
#elif defined(TCP_KEEPIDLE)
        setsockopt(client, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));  // Linux
#endif
        {
            std::lock_guard<std::mutex> lk(sendMutex_);
            clientFd_ = client;
        }
        // NOTE: connected_ / busy are NOT set here — serveClient flips them only
        // AFTER auth passes (or immediately when auth is off), so an
        // unauthenticated peer never reserves the slot (STREAM-AUTH-SPEC §4).

        serveClient(client, peerKey);

        {
            std::lock_guard<std::mutex> lk(sendMutex_);
            clientFd_ = kInvalidSocket;
        }
        close_socket(client);
        connected_ = false;
        bonjour_.setBusy(false);
        parser_.reset();
        decoder_.reset();
        blog(LOG_INFO, "[airlive] iPhone disconnected — listening for reconnect");
    }

    bonjour_.stop();
    close_socket(listenFd);
}

void AirliveConnection::serveClient(socket_t client, const std::string &peerKey) {
    setNonBlocking(client);

    // Snapshot the auth config once for this whole connection so a settings
    // change from the UI thread mid-handshake can't race the verify.
    bool requireAuth;
    std::string password;
    {
        std::lock_guard<std::mutex> lk(authMutex_);
        requireAuth = authRequire_ && !authPassword_.empty();
        password = authPassword_;
    }
    if (requireAuth && !runAuthHandshake(client, peerKey, password))
        return; // rejected / timed out — run() closes the socket

    // Authorized (or auth off): NOW reserve the slot + advertise busy. An
    // unauthenticated peer never got this far, so it never held the channel.
    connected_ = true;
    bonjour_.setBusy(true);

    std::vector<uint8_t> chunk(kRecvChunk);
    int idleMs = 0; // time since the last byte — resets on every recv

    while (running_.load()) {
        poll_fd_t pfd{};
        pfd.fd = client;
        pfd.events = POLLIN;
        const int pr = poll_sockets(&pfd, 1, kPollTimeoutMs);
        if (pr < 0)
            return;
        if (pr == 0) {
            // No data this interval. A live feed never goes quiet for long, so a
            // sustained silence means the peer vanished without a FIN.
            idleMs += kPollTimeoutMs;
            if (idleMs >= kStallTimeoutMs) {
                blog(LOG_INFO, "[airlive] no data for %d ms — assuming iPhone gone", idleMs);
                return;
            }
            continue;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            return;

#ifdef _WIN32
        const int n = ::recv(client, reinterpret_cast<char *>(chunk.data()), int(chunk.size()), 0);
#else
        const ssize_t n = ::recv(client, chunk.data(), chunk.size(), 0);
#endif
        if (n == 0)
            return; // peer closed
        if (n < 0) {
#ifdef _WIN32
            if (WSAGetLastError() == WSAEWOULDBLOCK)
                continue;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
#endif
            return; // real error
        }

        idleMs = 0; // got bytes — peer is alive
        parser_.feed(chunk.data(), size_t(n), [this](const Packet &pkt) {
            switch (pkt.type) {
            case PacketType::FormatDescription:
                decoder_.setParameterSets(pkt.payload, pkt.payload_len);
                break;
            case PacketType::Sample:
                decoder_.decodeSample(pkt.payload, pkt.payload_len, pkt.timestamp_us);
                break;
            case PacketType::Control:
                // The iPhone sends a JSON state snapshot here on connect and
                // after camera-side changes. We surface it for status display;
                // sending set-commands back (tally) is the only write we do.
                if (controlSink_)
                    controlSink_(reinterpret_cast<const char *>(pkt.payload), pkt.payload_len);
                break;
            case PacketType::AuthChallenge:
            case PacketType::AuthResponse:
            case PacketType::AuthResult:
                // Auth packets matter only during the handshake; once authorized
                // we never act on them and never re-verify per packet (the
                // thermal-critical invariant). Ignore.
                break;
            }
        });
    }
}

// ---- auth handshake (worker thread) ---------------------------------------
//
// The camera's send path is UNCHANGED: it streams formatDescription + samples
// immediately, so those arrive interleaved with (or before) the response. We
// buffer the latest format, DROP samples, and process ONLY the response until it
// verifies or the stall timeout fires. We authorize ONLY on our own constant-
// time verify of a type-4 — never because we received a type-5 (a forged
// authResult) or any other packet (STREAM-AUTH-SPEC §3).
bool AirliveConnection::runAuthHandshake(socket_t client, const std::string &peerKey,
                                         const std::string &password) {
    uint8_t nonce[kAuthTagLength];
    fillNonce(nonce);
    if (!sendRaw(PacketType::AuthChallenge, nonce, sizeof(nonce)))
        return false; // couldn't even send the challenge — peer already gone
    blog(LOG_INFO, "[airlive] auth challenge sent — awaiting response");

    std::vector<uint8_t> chunk(kRecvChunk);
    std::vector<uint8_t> pendingFormat; // latest format buffered while pending
    int idleMs = 0;
    bool decided = false; // first decision wins; ignore later packets
    bool authorized = false;

    while (running_.load() && !decided) {
        poll_fd_t pfd{};
        pfd.fd = client;
        pfd.events = POLLIN;
        const int pr = poll_sockets(&pfd, 1, kPollTimeoutMs);
        if (pr < 0)
            return false;
        if (pr == 0) {
            idleMs += kPollTimeoutMs;
            if (idleMs >= kAuthStallTimeoutMs) {
                blog(LOG_INFO, "[airlive] auth stall — no valid response in %d ms", idleMs);
                sendAuthResult(false, "auth_required");
                return false;
            }
            continue;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            return false;

#ifdef _WIN32
        const int n = ::recv(client, reinterpret_cast<char *>(chunk.data()), int(chunk.size()), 0);
#else
        const ssize_t n = ::recv(client, chunk.data(), chunk.size(), 0);
#endif
        if (n == 0)
            return false; // peer closed
        if (n < 0) {
#ifdef _WIN32
            if (WSAGetLastError() == WSAEWOULDBLOCK)
                continue;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
#endif
            return false;
        }
        idleMs = 0;

        bool violation = false;
        parser_.feed(chunk.data(), size_t(n), [&](const Packet &pkt) {
            if (decided)
                return; // a decision was already made this feed — ignore the rest
            switch (pkt.type) {
            case PacketType::AuthResponse:
                // Length-check BEFORE verify (a non-32-byte tag is a failure
                // outright), then constant-time verify of THIS nonce.
                if (pkt.payload_len == size_t(kAuthTagLength) &&
                    authVerify(password, nonce, sizeof(nonce), pkt.payload, pkt.payload_len)) {
                    decided = true;
                    authorized = true;
                } else {
                    decided = true; // wrong password
                }
                break;
            case PacketType::FormatDescription:
                pendingFormat.assign(pkt.payload, pkt.payload + pkt.payload_len);
                break;
            case PacketType::Sample:
            case PacketType::Control:
                break; // dropped while pending — process ONLY the response
            case PacketType::AuthChallenge:
            case PacketType::AuthResult:
                // Receiver→camera only — a peer sending one is forging the
                // handshake. Protocol violation → reject (never authorize on it).
                decided = true;
                violation = true;
                break;
            }
        });

        if (decided) {
            if (authorized) {
                sendAuthResult(true);
                if (!pendingFormat.empty())
                    decoder_.setParameterSets(pendingFormat.data(), pendingFormat.size());
                clearBan(peerKey);
                blog(LOG_INFO, "[airlive] authorized");
                return true;
            }
            registerAuthFailure(peerKey);
            sendAuthResult(false, "auth_failed");
            blog(LOG_INFO, "[airlive] auth %s", violation ? "protocol violation" : "failed (wrong password)");
            return false;
        }
    }
    return false; // running_ went false mid-handshake
}

// Convenience: frame + send an AuthResult JSON packet (uses the live clientFd_).
void AirliveConnection::sendAuthResult(bool ok, const char *reason) {
    const std::string json = authResultJSON(ok, reason);
    sendRaw(PacketType::AuthResult, reinterpret_cast<const uint8_t *>(json.data()), json.size());
}

// ---- anti-bruteforce ledger (worker-thread only) --------------------------

bool AirliveConnection::isBanned(const std::string &peerKey) const {
    auto it = authBans_.find(peerKey);
    return it != authBans_.end() && it->second.second > nowMs();
}

void AirliveConnection::registerAuthFailure(const std::string &peerKey) {
    auto &entry = authBans_[peerKey]; // {failures, banUntilMs}
    entry.first += 1;
    if (entry.first >= kAuthMaxFailures) {
        const int over = entry.first - kAuthMaxFailures;
        int64_t ban = kAuthBanBaseMs << (over > 20 ? 20 : over); // base × 2^over, guarded
        if (ban > kAuthBanMaxMs || ban <= 0)
            ban = kAuthBanMaxMs;
        entry.second = nowMs() + ban;
        blog(LOG_INFO, "[airlive] %s banned %lld ms after %d failed attempts",
             peerKey.c_str(), (long long)ban, entry.first);
    }
}

void AirliveConnection::clearBan(const std::string &peerKey) { authBans_.erase(peerKey); }

void AirliveConnection::setAuth(bool require, std::string password) {
    std::lock_guard<std::mutex> lk(authMutex_);
    authRequire_ = require;
    authPassword_ = std::move(password);
}

bool AirliveConnection::sendControl(const std::string &json) {
    return sendRaw(PacketType::Control,
                   reinterpret_cast<const uint8_t *>(json.data()), json.size());
}

bool AirliveConnection::sendRaw(PacketType type, const uint8_t *payload, size_t len) {
    // Frame the packet: 18-byte header (big-endian) + payload.
    std::vector<uint8_t> out;
    out.reserve(kHeaderSize + len);
    const uint32_t m = kMagic;
    out.push_back(uint8_t(m >> 24));
    out.push_back(uint8_t(m >> 16));
    out.push_back(uint8_t(m >> 8));
    out.push_back(uint8_t(m));
    out.push_back(kProtocolVersion);
    out.push_back(uint8_t(type));
    const uint32_t plen = uint32_t(len);
    out.push_back(uint8_t(plen >> 24));
    out.push_back(uint8_t(plen >> 16));
    out.push_back(uint8_t(plen >> 8));
    out.push_back(uint8_t(plen));
    for (int i = 0; i < 8; ++i)
        out.push_back(0); // timestamp_us = 0 for control / auth messages
    if (len)
        out.insert(out.end(), payload, payload + len);

#ifdef MSG_NOSIGNAL
    const int flags = MSG_NOSIGNAL; // Linux: suppress SIGPIPE on a dead peer
#else
    const int flags = 0; // macOS uses SO_NOSIGPIPE (set on accept)
#endif

    std::lock_guard<std::mutex> lk(sendMutex_);
    if (clientFd_ == kInvalidSocket)
        return false; // nobody connected — drop silently

    // The client socket is NON-BLOCKING (set in serveClient), and we run on the
    // OBS UI thread under sendMutex_. So we must neither spin forever NOR bail
    // out mid-packet (a half-sent control frame would corrupt the wire). On
    // EAGAIN we poll briefly for writability and finish the packet; a peer that
    // can't take ~50 bytes within the budget is wedged (the stall timeout reaps
    // it), and we drop the WHOLE packet cleanly. Tally is cheap to re-send on the
    // next state change, so a dropped cue self-heals.
    constexpr int kSendBudgetMs = 500;
    int spentMs = 0;
    size_t sent = 0;
    while (sent < out.size()) {
#ifdef _WIN32
        const int n = ::send(clientFd_, reinterpret_cast<const char *>(out.data() + sent),
                             int(out.size() - sent), flags);
#else
        const ssize_t n = ::send(clientFd_, out.data() + sent, out.size() - sent, flags);
#endif
        if (n > 0) {
            sent += size_t(n);
            continue;
        }
#ifdef _WIN32
        const bool wouldBlock = (n < 0 && WSAGetLastError() == WSAEWOULDBLOCK);
#else
        const bool wouldBlock =
            (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR));
#endif
        if (!wouldBlock || spentMs >= kSendBudgetMs)
            return false; // real error, or budget spent — peer gone/wedged; read loop reaps it
        poll_fd_t pfd{};
        pfd.fd = clientFd_;
        pfd.events = POLLOUT;
        constexpr int kSliceMs = 50;
        if (poll_sockets(&pfd, 1, kSliceMs) < 0)
            return false;
        spentMs += kSliceMs;
    }
    return true; // whole packet written
}

} // namespace airlive
