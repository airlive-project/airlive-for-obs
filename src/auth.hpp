// auth.hpp — receiver-password challenge-response crypto (HMAC-SHA256).
//
// Replicates AirliveCore's AirliveAuth (the FROZEN canonical definition) so the
// camera, Studio, Bridge and this plugin all compute the SAME tag bit-for-bit —
// see docs/STREAM-AUTH-SPEC.md.  The HMAC and the nonce come from the OS (platform/crypto.hpp).
//
// THE tag (replicate EXACTLY): HMAC-SHA256(key = UTF-8 bytes of the password,
// message = the 32 raw nonce bytes) → raw 32-byte tag.  No normalization, no KDF,
// no salt; raw bytes on the wire (never hex/base64).  Verified against the same
// known-answer vector the Bridge's Swift uses (Tests/auth-kat.cpp).
//
// Threat model is ACCESS control, not confidentiality (no TLS): keep a same-LAN
// prankster from grabbing the slot or injecting a fake feed.  Exactly one HMAC
// per connection, before any video — zero per-frame cost.

#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <string>

#include "platform/crypto.hpp"
#include "wire.hpp" // kAuthTagLength

namespace airlive {

// The canonical response tag for a (password, nonce) pair.
inline void authResponseTag(const std::string &password,
                            const uint8_t *nonce, size_t nonceLen,
                            uint8_t out[kAuthTagLength]) {
    hmacSha256(reinterpret_cast<const uint8_t *>(password.data()), password.size(),
               nonce, nonceLen, out);
}

// Constant-time equality over a fixed length — no early-out, so a wrong tag
// leaks no timing signal an attacker could use to recover bytes one at a time.
inline bool constantTimeEquals(const uint8_t *a, const uint8_t *b, size_t n) {
    uint8_t diff = 0;
    for (size_t i = 0; i < n; ++i)
        diff = uint8_t(diff | (a[i] ^ b[i]));
    return diff == 0;
}

// Verify a camera's response (RECEIVER side).  Length is checked by the CALLER
// before this (a non-32-byte tag is auth_failed); here we recompute and compare
// in constant time.
inline bool authVerify(const std::string &password,
                       const uint8_t *nonce, size_t nonceLen,
                       const uint8_t *tag, size_t tagLen) {
    if (tagLen != size_t(kAuthTagLength))
        return false;
    uint8_t expected[kAuthTagLength];
    authResponseTag(password, nonce, nonceLen, expected);
    return constantTimeEquals(expected, tag, kAuthTagLength);
}

// Fill a single-use challenge nonce from the system CSPRNG.
inline void fillNonce(uint8_t out[kAuthTagLength]) {
    secureRandom(out, kAuthTagLength);
}

// Build the JSON for an AuthResult packet (receiver→camera).  snake_case reason
// must match AirliveCore's AuthReason rawValues so the camera decodes it.
inline std::string authResultJSON(bool ok, const char *reason = nullptr) {
    if (ok)
        return "{\"ok\":true}";
    return std::string("{\"ok\":false,\"reason\":\"") + (reason ? reason : "auth_failed") + "\"}";
}

} // namespace airlive
