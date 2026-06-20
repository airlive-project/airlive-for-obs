// packet-parser.hpp — stateful framing parser for the Airlive TCP byte stream.
//
// WHY: TCP is a byte stream, not a message stream. One recv() may carry half a
// packet, several packets, or a packet split across reads. This class buffers
// bytes and emits whole packets only when fully present, resyncing past any
// header that fails validation. It is a direct port of PacketParser in
// AirliveCore/Packet.swift — keep the state machine identical.
//
// Not thread-safe: feed() is called from the single connection worker thread.

#pragma once

#include <cstdint>
#include <functional>
#include <vector>

#include "wire.hpp"

namespace airlive {

struct Packet {
    PacketType type;
    int64_t timestamp_us;     // capture PTS, microseconds
    const uint8_t *payload;   // points into the parser's internal buffer
    size_t payload_len;       // valid only for the duration of the emit callback
};

class PacketParser {
public:
    using Emit = std::function<void(const Packet &)>;

    // Append freshly received bytes and emit every whole packet now available.
    // `emit` borrows the payload pointer; copy out anything you need to keep.
    void feed(const uint8_t *data, size_t len, const Emit &emit);

    // Drop all buffered bytes (call on disconnect so a reconnect starts clean).
    void reset() { buffer_.clear(); }

private:
    std::vector<uint8_t> buffer_;
};

} // namespace airlive
