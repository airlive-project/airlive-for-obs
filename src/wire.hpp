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
    Control           = 2, // JSON ControlMessage — iPhone state snapshot / set-cmds
    // ── Receiver-password auth (challenge-response, HMAC) ──────────────────
    // Additive; kProtocolVersion is NOT bumped (an auth-OFF receiver never sends
    // these, so old camera ↔ new plugin is byte-identical).  See the FROZEN
    // docs/STREAM-AUTH-SPEC.md.  Payloads:
    //   AuthChallenge (receiver→camera): 32 raw bytes, single-use nonce.
    //   AuthResponse  (camera→receiver): 32 raw bytes, HMAC-SHA256 tag.
    //   AuthResult    (receiver→camera): JSON {"ok":bool,"reason"?:string}.
    AuthChallenge     = 3,
    AuthResponse      = 4,
    AuthResult        = 5,
};

// Protocol GENERATION ladder (PROTOCOL-COMPAT-SPEC.md) — DISTINCT from kProtocolVersion (the
// binary framing epoch, which never bumps). Counts additive protocol surface (verbs/fields/
// types/TXT keys); used ONLY for the update-prompt UX (peer.proto < my.minProto → "update"),
// NEVER for feature gating (that's the hello caps). Mirrors AirliveCore's AirliveProto.
constexpr int kProtoGeneration = 2;    // this build: 1 = pre-hello, 2 = hello
constexpr int kProtoMinGeneration = 1; // oldest peer generation we fully support (stays 1)

// SHA-256 output size: a challenge nonce and a response tag are both 32 bytes.
constexpr int kAuthTagLength = 32;

// AVCC NAL length prefix is 4 bytes (nalUnitHeaderLength = 4).
constexpr int kAvccLengthBytes = 4;

// H.264 NAL unit type for an IDR (instantaneous decoder refresh) slice — the
// keyframe in front of which we prepend SPS/PPS. nal_type = firstByte & 0x1F.
constexpr uint8_t kNalTypeIDR = 5;

} // namespace airlive
