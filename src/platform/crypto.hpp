// crypto.hpp — the two primitives the receiver password needs, one implementation per OS.
//
// Declared here, implemented in platform/<os>/crypto.cpp with the OS's own crypto library
// (CommonCrypto on macOS, CNG on Windows), so the plugin ships no crypto library of its own.

#pragma once

#include <cstddef>
#include <cstdint>

namespace airlive {

/// HMAC-SHA256 of `msg` under `key`, into out[32].
void hmacSha256(const uint8_t *key, size_t keyLen, const uint8_t *msg, size_t msgLen, uint8_t out[32]);

/// Fill `out` with `len` bytes from the system CSPRNG.
void secureRandom(uint8_t *out, size_t len);

} // namespace airlive
