#include "airlive-connection.hpp"

#include <cerrno>
#include <cstring>
#include <vector>

#include <obs-module.h>

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

void setNonBlocking(socket_t s) {
#ifdef _WIN32
    u_long mode = 1;
    ioctlsocket(s, FIONBIO, &mode);
#else
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
#endif
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

        socket_t client = ::accept(listenFd, nullptr, nullptr);
        if (client == kInvalidSocket)
            continue;

        blog(LOG_INFO, "[airlive] iPhone connected");
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
        connected_ = true;
        bonjour_.setBusy(true);

        serveClient(client);

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

void AirliveConnection::serveClient(socket_t client) {
    setNonBlocking(client);
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
            }
        });
    }
}

void AirliveConnection::sendControl(const std::string &json) {
    // Frame a type-2 packet: 18-byte header (big-endian) + JSON payload.
    std::vector<uint8_t> out;
    out.reserve(kHeaderSize + json.size());
    const uint32_t m = kMagic;
    out.push_back(uint8_t(m >> 24));
    out.push_back(uint8_t(m >> 16));
    out.push_back(uint8_t(m >> 8));
    out.push_back(uint8_t(m));
    out.push_back(kProtocolVersion);
    out.push_back(uint8_t(PacketType::Control));
    const uint32_t len = uint32_t(json.size());
    out.push_back(uint8_t(len >> 24));
    out.push_back(uint8_t(len >> 16));
    out.push_back(uint8_t(len >> 8));
    out.push_back(uint8_t(len));
    for (int i = 0; i < 8; ++i)
        out.push_back(0); // timestamp_us = 0 for control messages
    out.insert(out.end(), json.begin(), json.end());

#ifdef MSG_NOSIGNAL
    const int flags = MSG_NOSIGNAL; // Linux: suppress SIGPIPE on a dead peer
#else
    const int flags = 0; // macOS uses SO_NOSIGPIPE (set on accept)
#endif

    std::lock_guard<std::mutex> lk(sendMutex_);
    if (clientFd_ == kInvalidSocket)
        return; // nobody connected — drop silently

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
            return; // real error, or budget spent — peer gone/wedged; read loop reaps it
        poll_fd_t pfd{};
        pfd.fd = clientFd_;
        pfd.events = POLLOUT;
        constexpr int kSliceMs = 50;
        if (poll_sockets(&pfd, 1, kSliceMs) < 0)
            return;
        spentMs += kSliceMs;
    }
}

} // namespace airlive
