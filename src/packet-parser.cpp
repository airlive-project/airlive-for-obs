#include "packet-parser.hpp"

namespace airlive {

namespace {
// Read a big-endian integer from a byte pointer. The wire is big-endian
// regardless of host byte order, so we assemble explicitly rather than memcpy.
inline uint32_t be32(const uint8_t *p) {
    return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | uint32_t(p[3]);
}
inline int64_t be64(const uint8_t *p) {
    int64_t v = 0;
    for (int i = 0; i < 8; ++i)
        v = (v << 8) | int64_t(p[i]);
    return v;
}
} // namespace

void PacketParser::feed(const uint8_t *data, size_t len, const Emit &emit) {
    // Defensive cap: a peer that keeps sending valid-looking headers with a
    // near-max payload_len but never the bodies could otherwise grow buffer_
    // without bound. A real stream never holds more than one in-flight packet,
    // so anything past 2× the payload ceiling is junk — drop and resync clean.
    if (buffer_.size() > size_t(2) * kMaxPayloadLength)
        buffer_.clear();
    buffer_.insert(buffer_.end(), data, data + len);

    size_t pos = 0; // read cursor; we erase consumed bytes once at the end
    while (buffer_.size() - pos >= size_t(kHeaderSize)) {
        const uint8_t *h = buffer_.data() + pos;

        // Validate the header; on any failure drop a single byte and resync.
        // Advancing by one (not by a guessed length) is what lets us recover
        // from a desynced stream without discarding a real packet boundary.
        if (be32(h) != kMagic) { ++pos; continue; }
        if (h[4] != kProtocolVersion) { ++pos; continue; }

        const uint32_t payloadLen = be32(h + 6);
        if (payloadLen > kMaxPayloadLength) { ++pos; continue; }
        const size_t total = size_t(kHeaderSize) + payloadLen;

        const uint8_t rawType = h[5];
        // FORWARD-COMPAT (PROTOCOL-COMPAT-SPEC §3): a type ABOVE AuthResult, on an otherwise
        // valid header (good magic+version, sane length), is a packet from a NEWER peer — skip
        // it WHOLE (header + payload), not one byte. Byte-resync used to walk THROUGH the unknown
        // packet's payload, where embedded "ARLV" bytes mis-framed as packets. New types stay
        // gated on the hello caps; this skip degrades a gating bug to "ignored", not "corrupted".
        if (rawType > uint8_t(PacketType::AuthResult)) {
            if (buffer_.size() - pos < total) break; // whole packet not here yet — wait
            pos += total;
            continue;
        }

        if (buffer_.size() - pos < total) break; // wait for the rest of this packet

        Packet pkt;
        pkt.type = static_cast<PacketType>(rawType);
        pkt.timestamp_us = be64(h + 10);
        pkt.payload = h + kHeaderSize;
        pkt.payload_len = payloadLen;
        emit(pkt);

        pos += total;
    }

    // Erase everything we consumed in one shot — cheaper than removeFirst()
    // per packet, and keeps the buffer from growing without bound.
    if (pos > 0)
        buffer_.erase(buffer_.begin(), buffer_.begin() + pos);
}

} // namespace airlive
