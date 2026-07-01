// airlive-connection.hpp — the worker thread: TCP server on an ephemeral port,
// Bonjour advert, single-client accept loop, framing parser, H.264 decode.
//
// WHY a dedicated thread: the socket and decoder must never block OBS's
// graphics/UI thread. The source creates one of these in create() and tears it
// down in destroy(). Decoded frames are handed back via a callback the source
// turns into obs_source_output_video().
//
// Connection policy (per spec): bind port 0 so the OS picks a free port (never
// a hardcoded 7777 — many receivers coexist on one LAN). Accept exactly one
// iPhone at a time; flip the Bonjour busy flag on connect/disconnect; on
// disconnect keep listening so the phone can reconnect.

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "bonjour-service.hpp"
#include "h264-decoder.hpp"
#include "net-compat.hpp" // socket_t (Winsock first; correct include order)
#include "packet-parser.hpp"

struct AVFrame;

namespace airlive {

class AirliveConnection {
public:
    using FrameSink = H264Decoder::FrameSink;
    // Fired on every inbound type-2 (control) packet — a JSON ControlMessage,
    // e.g. the iPhone's full camera-state snapshot. Used for status display.
    using ControlSink = std::function<void(const char *json, size_t len)>;

    AirliveConnection(ServiceIdentity identity, FrameSink sink, ControlSink control = nullptr);
    ~AirliveConnection();

    AirliveConnection(const AirliveConnection &) = delete;
    AirliveConnection &operator=(const AirliveConnection &) = delete;

    void start();
    void stop();

    bool connected() const { return connected_.load(); }

    // Send a control message (type-2) back to the iPhone over the same socket.
    // Thread-safe. Returns true only if the whole packet was written; false if
    // no iPhone is connected or the send was dropped (so the caller can retry).
    bool sendControl(const std::string &json);

    // Apply the receiver-password auth config (called from the OBS UI thread when
    // the source's properties change).  `require && !password.empty()` turns the
    // challenge-response on for the NEXT connection.  Thread-safe; snapshotted by
    // the worker when a client connects, so a mid-handshake change can't race.
    void setAuth(bool require, std::string password);

private:
    void run();                        // thread body
    socket_t openListenSocket(uint16_t &outPort);
    // Blocking read loop for one iPhone.  `peerKey` (remote IP) keys the
    // anti-bruteforce ledger.  Runs the auth handshake first when required.
    void serveClient(socket_t client, const std::string &peerKey);

    // Auth handshake (worker thread).  Sends one challenge, processes ONLY the
    // response (buffering the latest format, dropping samples) until it verifies
    // or times out.  Returns true ⇒ authorized (caller goes live), false ⇒ the
    // connection was rejected/closed.
    bool runAuthHandshake(socket_t client, const std::string &peerKey,
                          const std::string &password);

    // Frame + send any packet type back to the iPhone (the generic primitive
    // behind sendControl and the auth packets).  Thread-safe.  BLOCKS up to a send
    // budget on a wedged peer — must run on the WORKER thread only (auth handshake
    // + drainOutgoing), never on an OBS render/UI thread (see sendControl).
    bool sendRaw(PacketType type, const uint8_t *payload, size_t len);

    // Flush queued control packets (enqueued by sendControl from any thread) on the
    // worker thread.  Called each serveClient loop iteration.
    void drainOutgoing();

    // Frame + send an AuthResult JSON packet over the live client fd.
    void sendAuthResult(bool ok, const char *reason = nullptr);

    // Anti-bruteforce (worker-thread only — no lock needed): per-source failure
    // count + ban deadline.  isBanned/registerAuthFailure/clearBan operate on it.
    bool isBanned(const std::string &peerKey) const;
    void registerAuthFailure(const std::string &peerKey);
    void clearBan(const std::string &peerKey);

    ServiceIdentity identity_;
    BonjourService bonjour_;
    PacketParser parser_;
    H264Decoder decoder_;
    ControlSink controlSink_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    // Guards the live client fd so sendControl() (called from OBS threads) never
    // writes to a fd the worker thread is closing.
    std::mutex sendMutex_;
    socket_t clientFd_{kInvalidSocket};

    // Outgoing control packets are ENQUEUED here by sendControl (callable from the OBS render/UI
    // thread) and drained on the worker thread — so a blocking send on a half-dead peer can never
    // freeze the compositor. Bounded; tally/mode are self-healing so dropping the oldest is safe.
    std::mutex outMutex_;
    std::vector<std::string> outControl_;

    // The phone's last-reported video-active state (from its control-JSON snapshot / any sample).
    // False for a control-only camera → exempt it from the sample-stall reaper (TCP keepalive still
    // catches a truly dead peer).  Written on the worker thread, read there too.
    std::atomic<bool> peerVideoActive_{true};

    // Auth config — written from the OBS UI thread, read by the worker.  Guarded
    // by authMutex_; the worker snapshots it once per connection.
    mutable std::mutex authMutex_;
    bool authRequire_{false};
    std::string authPassword_;

    // Per-source failure ledger (IP → {failures, banUntilMs}).  Worker-only.
    std::map<std::string, std::pair<int, int64_t>> authBans_;
};

} // namespace airlive
