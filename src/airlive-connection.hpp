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
#include <mutex>
#include <string>
#include <thread>

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
    // Thread-safe; a no-op when no iPhone is connected. Used for tally (setCue).
    void sendControl(const std::string &json);

private:
    void run();                        // thread body
    socket_t openListenSocket(uint16_t &outPort);
    void serveClient(socket_t client); // blocking read loop for one iPhone

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
};

} // namespace airlive
