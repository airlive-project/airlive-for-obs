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

// The "OBS Airlive Bridge" source and the Airlive Bridge app run on the SAME Mac, so their link is
// a plain loopback socket on this fixed port — NO Bonjour, NO LAN exposure (so the iPhone never sees
// it and no local-network permission is involved). The Bridge app connects to 127.0.0.1:this port.
// Must match kBridgeLocalPort in airlive-bridge/Sources/Output/AirliveRelayOutput.swift.
constexpr uint16_t kBridgeLocalPort = 47788;

// No bytes for this long = the camera's link is silent. The camera goes completely quiet when its
// route is lost and RESUMES ON THE SAME CONNECTION when it returns, so silence is not a reason to
// close: OBS paints black (source_video_tick) and the socket is kept. The first bytes after the
// silence mark the return (tally re-sent, the stale tail dropped). Same threshold as the Bridge:
// a 5x lens switch keeps the camera quiet for up to ~1.2 s and must not count as a lost link.
constexpr int64_t kSilenceMs = 1500;

// While the picture waits for its keyframe after a silence, ask again at most this often. The
// camera makes keyframes only on request (natural ones are 15-30 s apart) and ignores a request
// that lands inside its own spacing window, so one lost request must not mean a long black; a
// keyframe is also the heaviest frame, so never more than one a second on a weak link.
constexpr int64_t kResumeKeyframeIntervalMs = 1000;

// A new connection while the slot is held usually means the phone came back on a NEW socket (app
// relaunched, phone rebooted) and the held one is a dead corpse keepalive has not reaped yet: the
// newcomer takes over at once. Not within this age, though: one phone opens an IPv6 and an IPv4
// socket together, and the second must not evict the first (the Bridge's rule, same numbers).
constexpr int64_t kTakeoverMinAgeMs = 3000;

// A peer that stops acknowledging what WE send is dead as surely as one that stops answering
// keepalive probes; keepalive does not cover it. Same bound as the Bridge.
constexpr int kConnectionDropSeconds = 30;

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

// Whether an AVCC access unit (4-byte big-endian NAL lengths) holds an IDR slice (NAL type 5).
// The camera's keyframe is the first live picture after it comes back: it does NOT re-send its
// format then (only once per connection, or when a receiver asks for a keyframe), so the keyframe
// itself has to be recognised. Malformed lengths end the walk - "no IDR" is the safe answer.
bool containsIdr(const uint8_t *p, size_t len) {
    size_t pos = 0;
    while (pos + 4 < len) {
        const uint32_t n = (uint32_t(p[pos]) << 24) | (uint32_t(p[pos + 1]) << 16) |
                           (uint32_t(p[pos + 2]) << 8) | uint32_t(p[pos + 3]);
        pos += 4;
        if (n == 0 || uint64_t(pos) + n > uint64_t(len))
            return false;
        if ((p[pos] & 0x1F) == 5)
            return true;
        pos += n;
    }
    return false;
}
} // namespace

AirliveConnection::AirliveConnection(ServiceIdentity identity, FrameSink sink, ControlSink control,
                                     DisconnectSink disconnect, LinkResumedSink resumed,
                                     ResumeGate resumeGate)
    : identity_(std::move(identity)),
      decoder_(std::move(sink)),
      controlSink_(std::move(control)),
      disconnectSink_(std::move(disconnect)),
      linkResumedSink_(std::move(resumed)),
      resumeGate_(std::move(resumeGate)) {}

AirliveConnection::~AirliveConnection() { stop(); }

void AirliveConnection::start() {
    if (running_.exchange(true))
        return;
    thread_ = std::thread([this] { run(); });
}

void AirliveConnection::stop() {
    // The FLAG IS NOT THE JOIN CONDITION, and treating it as one aborted OBS.
    //
    // A worker that returned on its own has already cleared `running_` - run() does exactly
    // that when it cannot open its listen socket, which is the everyday case of a SECOND
    // Bridge source losing the race for the fixed loopback port. A std::thread whose function
    // has RETURNED is still joinable, so the early return skipped the join precisely when it
    // was needed, and ~std::thread on a joinable thread calls std::terminate(). Removing that
    // source took the whole of OBS down with it, from a stack with no hint of a socket in it.
    //
    // Joining unconditionally is also what makes stop() idempotent: after a join the thread is
    // no longer joinable, so a second call does nothing. Only the destructor calls this, and
    // never from the worker itself, so there is no self-join to fear.
    running_.store(false);
    if (thread_.joinable())
        thread_.join();
}

// Bind a dual-stack IPv6 socket on port 0 so the OS assigns a free port (and a
// dual-stack iPhone reaches us over either IPv4 or IPv6). Returns the chosen
// port via outPort, or kInvalidSocket on failure.
socket_t AirliveConnection::openListenSocket(uint16_t &outPort) {
    // Bridge-program source: same-machine link → bind IPv4 LOOPBACK on a FIXED port. Not reachable
    // from the LAN (iPhone can't see it, no Bonjour, no local-network permission), and the Bridge app
    // connects straight to 127.0.0.1:kBridgeLocalPort.
    if (identity_.role == "obs-bridge") {
        socket_t fd = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd == kInvalidSocket) {
            blog(LOG_ERROR, "[airlive] socket() failed (bridge)");
            return kInvalidSocket;
        }
        int yes = 1;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes), sizeof(yes));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); // 127.0.0.1 only
        addr.sin_port = htons(kBridgeLocalPort);
        if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
            blog(LOG_ERROR, "[airlive] bind() 127.0.0.1:%u failed — is another OBS Airlive Bridge source running?",
                 kBridgeLocalPort);
            close_socket(fd);
            return kInvalidSocket;
        }
        if (::listen(fd, 1) != 0) {
            blog(LOG_ERROR, "[airlive] listen() failed (bridge)");
            close_socket(fd);
            return kInvalidSocket;
        }
        outPort = kBridgeLocalPort;
        setNonBlocking(fd);
        return fd;
    }

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

    if (identity_.role == "obs-bridge") {
        // Same-machine loopback link — no Bonjour advert at all (the Bridge app connects directly).
        blog(LOG_INFO, "[airlive] OBS Airlive Bridge source listening on 127.0.0.1:%u (loopback, no Bonjour)", port);
    } else {
        blog(LOG_INFO, "[airlive] listening on ephemeral port %u", port);
        bonjour_.start(port, identity_);
    }

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

        const char *peerName = (identity_.role == "obs-bridge") ? "Airlive Bridge" : "iPhone";
        blog(LOG_INFO, "[airlive] %s connected (%s)", peerName, peerKey.c_str());
#ifdef SO_NOSIGPIPE
        // macOS/BSD: never raise SIGPIPE if the iPhone vanishes mid-send.
        int one = 1;
        setsockopt(client, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
        // Keepalive is what frees the slot from a peer that died without a FIN (silence alone no
        // longer closes: a camera with no route is silent and comes back on this same socket).
        set_int_option(client, SOL_SOCKET, SO_KEEPALIVE, 1);
        constexpr int kKeepaliveIdleSeconds = 10; // idle before the first keepalive probe
#if defined(TCP_KEEPALIVE)
        set_int_option(client, IPPROTO_TCP, TCP_KEEPALIVE, kKeepaliveIdleSeconds); // macOS, Windows
#elif defined(TCP_KEEPIDLE)
        set_int_option(client, IPPROTO_TCP, TCP_KEEPIDLE, kKeepaliveIdleSeconds);  // Linux
#endif
        // 10 s idle + 3 probes 10 s apart ≈ 40 s, the Bridge's numbers: well inside a service, and
        // gentle on the phone's radio (probing every few seconds wakes it for nothing). A phone that
        // returns on a new socket never waits for this - it takes over (see serveClient).
#if defined(TCP_KEEPINTVL)
        set_int_option(client, IPPROTO_TCP, TCP_KEEPINTVL, 10);
#endif
#if defined(TCP_KEEPCNT)
        set_int_option(client, IPPROTO_TCP, TCP_KEEPCNT, 3);
#endif
        // Nagle OFF. Inbound camera->OBS video is one-way bulk (Nagle on the receiver never delays
        // it), but everything OBS SENDS BACK on this socket — the auth handshake + tally setCue cues
        // — is a lone small write that Nagle + delayed-ACK can sit on for ~40ms. Camera (noDelay=true)
        // and Bridge (tcp.noDelay=true) already disable it; make all three ends consistent.
        set_int_option(client, IPPROTO_TCP, TCP_NODELAY, 1);
        // Give up on data the peer never acknowledges (keepalive does not cover that case).
#if defined(TCP_RXT_CONNDROPTIME)
        set_int_option(client, IPPROTO_TCP, TCP_RXT_CONNDROPTIME, kConnectionDropSeconds); // macOS
#elif defined(TCP_MAXRT)
        set_int_option(client, IPPROTO_TCP, TCP_MAXRT, kConnectionDropSeconds);            // Windows
#endif
        {
            std::lock_guard<std::mutex> lk(sendMutex_);
            clientFd_ = client;
        }
        // NOTE: connected_ / busy are NOT set here — serveClient flips them only
        // AFTER auth passes (or immediately when auth is off), so an
        // unauthenticated peer never reserves the slot (STREAM-AUTH-SPEC §4).

        const int64_t startedMs = nowMs();
        const char *reason = serveClient(client, peerKey, listenFd);

        {
            std::lock_guard<std::mutex> lk(sendMutex_);
            clientFd_ = kInvalidSocket;
        }
        close_socket(client);
        connected_ = false;
        {
            // Nothing queued for this phone outlives it (see sendControl).
            std::lock_guard<std::mutex> lk(outMutex_);
            outControl_.clear();
        }
        awaitingResumeKeyframe_ = false;
        bonjour_.setBusy(false);
        parser_.reset();
        decoder_.reset();
        if (disconnectSink_)
            disconnectSink_(); // reset the source's peer-hello epoch (PROTOCOL-COMPAT-SPEC invariant)
        blog(LOG_INFO, "[airlive] %s %s disconnected (%s) after %lld s - listening for reconnect",
             peerName, peerKey.c_str(), reason, (long long)((nowMs() - startedMs) / 1000));
    }

    bonjour_.stop();
    close_socket(listenFd);
}

const char *AirliveConnection::serveClient(socket_t client, const std::string &peerKey,
                                          socket_t listenFd) {
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
        return "not authorised"; // rejected / timed out — run() closes the socket

    // Authorized (or auth off): NOW reserve the slot + advertise busy. An
    // unauthenticated peer never got this far, so it never held the channel.
    {
        // A clean slate: nothing queued before this phone was here is sent to it.
        std::lock_guard<std::mutex> lk(outMutex_);
        outControl_.clear();
    }
    connected_ = true;
    bonjour_.setBusy(true);
    peerVideoActive_.store(true); // assume video until a control-only snapshot says otherwise
    awaitingResumeKeyframe_ = false;
    const int64_t committedMs = nowMs();

    // One-shot hello — FIRST control message after accept/auth (PROTOCOL-COMPAT-SPEC §2).
    // Enqueued before setDeliveryMode so it goes out first (drainOutgoing preserves order).
    // An old camera hits its verb switch `default:` and ignores it — harmless. caps = the
    // protocol surface this plugin implements; new verbs/types stay gated on the PEER's caps.
    // appVersion is display-only (tracks CMake project() 1.0.0); proto/minProto are ints.
    sendControl(
        "{\"type\":\"hello\",\"hello\":{\"app\":\"obs\",\"appVersion\":\"1.0.0\",\"proto\":2,\"minProto\":1,"
        "\"caps\":[\"auth\",\"deliveryMode\",\"rotation\",\"capabilities\",\"tally\",\"updateRequired\"]}}");

    // Mirror the Bridge: tell the phone to enable its encoder. Without this a phone left STICKY in
    // control-only mode (encoder off) is permanently video-dead in OBS. setEncoderEnabled is
    // idempotent, so a phone already in video+control just re-confirms. Enqueued — drained below.
    // Not for the Bridge-program peer: it's the Bridge app relaying, not a camera to command.
    if (identity_.role != "obs-bridge")
        sendControl("{\"type\":\"setDeliveryMode\",\"stringValue\":\"videoAndControl\"}");

    // A returning phone is watched for only on the camera source: the Bridge-program peer is a
    // loopback link from one app, with a reliable FIN.
    bool watchListen = identity_.role != "obs-bridge" && listenFd != kInvalidSocket;

    std::vector<uint8_t> chunk(kRecvChunk);
    int64_t lastRxMs = nowMs(); // when the last byte arrived
    bool silent = false;        // past kSilenceMs with nothing - logged once per silence

    while (running_.load()) {
        drainOutgoing(); // flush queued tally / delivery-mode on the worker thread (never inline)
        poll_fd_t pfds[2]{};
        pfds[0].fd = client;
        pfds[0].events = POLLIN;
        unsigned long nfds = 1;
        if (watchListen) {
            pfds[1].fd = listenFd;
            pfds[1].events = POLLIN;
            nfds = 2;
        }
        const int pr = poll_sockets(pfds, nfds, kPollTimeoutMs);
        if (pr < 0)
            return "poll error";
        if (pr == 0) {
            // Silence is NOT a reason to close: a camera that lost its route is quiet and comes back
            // on this socket (OBS shows black meanwhile). Keepalive and the drop time free the slot
            // from a peer that is really gone.
            if (!silent && nowMs() - lastRxMs >= kSilenceMs) {
                silent = true;
                if (peerVideoActive_.load())
                    blog(LOG_INFO, "[airlive] nothing from the iPhone for %lld ms - keeping the connection",
                         (long long)kSilenceMs);
            }
            continue;
        }

        if (nfds == 2 && (pfds[1].revents & POLLIN)) {
            // Another phone (or this one, back on a new socket) is connecting while the slot is held.
            if (nowMs() - committedMs >= kTakeoverMinAgeMs && !requireAuth) {
                // The held socket is a corpse keepalive has not reaped yet: let the newcomer in now.
                // run() closes this one and accepts the waiting connection on its next turn.
                blog(LOG_INFO, "[airlive] new connect while the slot is held - taking over");
                return "superseded by a new connection";
            }
            // A sibling of the connection just made (IPv6 + IPv4 from one phone), or a stranger
            // while a password is set (it must not kick a live camera): refuse it.
            sockaddr_storage peer{};
            socklen_t peerLen = sizeof(peer);
            socket_t other = ::accept(listenFd, reinterpret_cast<sockaddr *>(&peer), &peerLen);
            if (other != kInvalidSocket) {
                blog(LOG_INFO, "[airlive] refused %s: slot held%s", peerKeyFromSockaddr(peer).c_str(),
                     requireAuth ? ", password on (no takeover)" : " (sibling of the connection just made)");
                close_socket(other);
            } else {
                // accept() itself failing (out of descriptors) would leave the listen socket
                // readable and spin this loop: stop watching it for the rest of this connection.
                watchListen = false;
                blog(LOG_WARNING, "[airlive] could not refuse a waiting connection - not watching for one any more");
            }
        }

        if (!(pfds[0].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)))
            continue;
        // A peer that closed reports POLLHUP together with POLLIN: read on, recv() says FIN and
        // delivers whatever came before it.
        if (!(pfds[0].revents & POLLIN))
            return "socket error";

#ifdef _WIN32
        const int n = ::recv(client, reinterpret_cast<char *>(chunk.data()), int(chunk.size()), 0);
#else
        const ssize_t n = ::recv(client, chunk.data(), chunk.size(), 0);
#endif
        if (n == 0)
            return "peer closed (FIN)";
        if (n < 0) {
#ifdef _WIN32
            if (WSAGetLastError() == WSAEWOULDBLOCK)
                continue;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            if (errno == ETIMEDOUT)
                return "timed out (no answer from the iPhone)";
#endif
            return "receive error";
        }

        const int64_t now = nowMs();
        if (now - lastRxMs >= kSilenceMs) {
            // Back on the same connection after a silence: its route returned. What its socket held
            // arrives first (old frames), then a keyframe; drop the old ones until then.
            // Not the retired Bridge link: an idle program is quiet by nature and may resume on any
            // frame, and nothing times it here.
            if (peerVideoActive_.load() && identity_.role != "obs-bridge") {
                awaitingResumeKeyframe_ = true;
                if (resumeGate_.begin)
                    resumeGate_.begin();
                // ASK for it: the camera makes a keyframe by itself only when iOS reported its route
                // gone and back; a Wi-Fi stall that kept the route leaves it sending P-frames, and
                // its next natural keyframe is up to 15 s away. The same request the Bridge makes.
                sendControl("{\"type\":\"forceKeyframe\"}");
                lastResumeKeyframeAskMs_ = now;
                blog(LOG_INFO, "[airlive] iPhone back after %lld ms - asked for a keyframe",
                     (long long)(now - lastRxMs));
            }
            // Its lamp went dark meanwhile: one current cue. A Control-only camera is quiet by
            // nature, so for it any message after a quiet second counts - one tiny, idempotent cue,
            // and the only way to catch its real return (the camera sends one snapshot then).
            if (linkResumedSink_)
                linkResumedSink_();
        }
        silent = false;
        parser_.feed(chunk.data(), size_t(n), [this](const Packet &pkt) {
            switch (pkt.type) {
            case PacketType::FormatDescription:
                decoder_.setParameterSets(pkt.payload, pkt.payload_len);
                break;
            case PacketType::Sample:
                if (awaitingResumeKeyframe_) {
                    const bool idr = containsIdr(pkt.payload, pkt.payload_len);
                    const bool show = resumeGate_.check ? resumeGate_.check(pkt.timestamp_us, idr) : idr;
                    if (!show) {
                        // The tail, or a keyframe too late for its slot - see awaitingResumeKeyframe_.
                        const int64_t sampleMs = nowMs();
                        if (sampleMs - lastResumeKeyframeAskMs_ >= kResumeKeyframeIntervalMs) {
                            lastResumeKeyframeAskMs_ = sampleMs;
                            sendControl("{\"type\":\"forceKeyframe\"}");
                        }
                        break;
                    }
                    awaitingResumeKeyframe_ = false; // its keyframe: live video again
                    blog(LOG_INFO, "[airlive] picture back");
                }
                peerVideoActive_.store(true);
                decoder_.decodeSample(pkt.payload, pkt.payload_len, pkt.timestamp_us);
                break;
            case PacketType::Control:
                // The iPhone sends a JSON state snapshot here on connect and after camera-side
                // changes. Track videoActive (a Control-only camera is quiet by nature, so its
                // silence is neither logged nor treated as a lost picture). Lightweight substring
                // scan of the JSON (Swift JSONEncoder emits no spaces around ':'); absent key (older
                // camera) leaves the prior value.
                if (pkt.payload_len) {
                    const std::string s(reinterpret_cast<const char *>(pkt.payload), pkt.payload_len);
                    if (s.find("\"videoActive\":false") != std::string::npos)
                        peerVideoActive_.store(false);
                    else if (s.find("\"videoActive\":true") != std::string::npos)
                        peerVideoActive_.store(true);
                }
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
        // Taken AFTER decoding, so the time spent decoding is never mistaken for silence.
        lastRxMs = nowMs();
    }
    return "stopped";
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
    // Bound the ledger. It's only pruned on a successful auth (clearBan), so a spoofed-source flood
    // of many distinct IPs each failing once would grow it for the whole OBS session. When it gets
    // large, drop entries whose ban has already expired (banUntilMs <= now, incl. sub-threshold
    // failures with banUntilMs==0) — never a currently-active ban, never the peer being recorded now.
    if (authBans_.size() > 256) {
        const int64_t now = nowMs();
        for (auto it = authBans_.begin(); it != authBans_.end();) {
            if (it->first != peerKey && it->second.second <= now)
                it = authBans_.erase(it);
            else
                ++it;
        }
    }
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
    // ENQUEUE only — never send inline. sendControl is reachable from the OBS render/video thread
    // (ensureTally via source_activate/deactivate/show/hide), and sendRaw can block up to its budget
    // on a half-dead peer; blocking there freezes the compositor on scene switches. The worker thread
    // drains this (drainOutgoing) between polls.
    //
    // Nothing is kept for a phone that is not here: a queue filled while it was away was flushed
    // into it on the next connect - cues of scene changes long past, settings nobody meant any
    // more, sometimes to a different phone. The connect sends the current cue (first state snapshot).
    if (!connected_.load())
        return false;
    std::lock_guard<std::mutex> lk(outMutex_);
    if (outControl_.size() >= 64)
        outControl_.erase(outControl_.begin()); // drop oldest — newest cue supersedes it anyway
    outControl_.push_back(json);
    return true;
}

// Worker-thread only: flush everything sendControl queued. sendRaw may block here (worker thread),
// which is fine — the render/UI thread already returned from sendControl without waiting.
void AirliveConnection::drainOutgoing() {
    std::vector<std::string> pending;
    {
        std::lock_guard<std::mutex> lk(outMutex_);
        pending.swap(outControl_);
    }
    for (const auto &json : pending)
        // Stop on the first failure: a false means the peer is gone/wedged, and each further sendRaw
        // could burn its full send budget too — draining 64 of them would stall the worker for many
        // seconds. The rest is dropped with the connection; the next connect sends the current cue.
        if (!sendRaw(PacketType::Control, reinterpret_cast<const uint8_t *>(json.data()), json.size()))
            break;
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
        // Teardown asked us to stop: abandon the packet NOW rather than spend the rest of the
        // budget on a peer nobody is waiting for any more. stop() joins this thread, and the
        // caller of stop() is OBS's graphics thread (a rename or a reorder rebuilds the
        // connection; so does deleting the source) - that thread also drives video_tick for
        // every other source, so half a second spent here is half a second of stutter on the
        // whole output. A control packet dropped while shutting down costs nothing.
        if (!running_.load())
            return false;
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
