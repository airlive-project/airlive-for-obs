// crypto.cpp (macOS) — CommonCrypto + arc4random. See platform/crypto.hpp.

#include "../crypto.hpp"

#include <CommonCrypto/CommonHMAC.h>
#include <cstdlib>

namespace airlive {

void hmacSha256(const uint8_t *key, size_t keyLen, const uint8_t *msg, size_t msgLen, uint8_t out[32]) {
    CCHmac(kCCHmacAlgSHA256, key, keyLen, msg, msgLen, out);
}

void secureRandom(uint8_t *out, size_t len) { arc4random_buf(out, len); }

} // namespace airlive
