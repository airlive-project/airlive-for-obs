// wire.hpp — the Airlive TCP wire format, shared by parser and decoder.
//
// WHY this file exists: the framing and the H.264 payload layout are the one
// contract this plugin must match byte-for-byte with the iPhone. Keeping every
// magic number, offset and limit in one header means a reader never has to
// reverse-engineer the protocol from the parsing code.
//
// Ground truth: AirliveCore/Packet.swift in the camera/studio repo.
//   [4 magic="ARLV"][1 version][1 type][4 payload_length BE][8 timestamp_us BE][N payload]
// All multi-byte integers are BIG-ENDIAN.

#pragma once

#include <cstdint>

namespace airlive {

// "ARLV" — first four bytes of every packet header.
constexpr uint32_t kMagic = 0x41524C56;

// Bumped only when the binary framing changes incompatibly. A header whose
// version != this is from a peer we cannot frame; the parser resyncs past it.
constexpr uint8_t kProtocolVersion = 1;

// 4 magic + 1 version + 1 type + 4 length + 8 timestamp.
constexpr int kHeaderSize = 18;

// Sanity ceiling on a network-sourced payload length. A 1080p H.264 keyframe is
// far under 1 MiB; a length above this means a corrupt/desynced header, so the
// parser resyncs rather than waiting forever for bytes that never arrive.
constexpr uint32_t kMaxPayloadLength = 16u * 1024u * 1024u;

enum class PacketType : uint8_t {
    FormatDescription = 0, // length-prefixed SPS/PPS parameter sets
    Sample            = 1, // one encoded access unit, AVCC (4-byte length-prefixed NALs)
    Control           = 2, // JSON ControlMessage — ignored in v1 (video-only)
};

// AVCC NAL length prefix is 4 bytes (nalUnitHeaderLength = 4).
constexpr int kAvccLengthBytes = 4;

// H.264 NAL unit type for an IDR (instantaneous decoder refresh) slice — the
// keyframe in front of which we prepend SPS/PPS. nal_type = firstByte & 0x1F.
constexpr uint8_t kNalTypeIDR = 5;

} // namespace airlive
