// crypto.cpp (Windows) — CNG (bcrypt). See platform/crypto.hpp.

#include "../crypto.hpp"

#include <windows.h>
#include <bcrypt.h>

#include <obs-module.h>

namespace airlive {

void hmacSha256(const uint8_t *key, size_t keyLen, const uint8_t *msg, size_t msgLen, uint8_t out[32]) {
    BCRYPT_ALG_HANDLE alg = nullptr;
    BCRYPT_HASH_HANDLE hash = nullptr;
    NTSTATUS st = BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr,
                                              BCRYPT_ALG_HANDLE_HMAC_FLAG);
    if (BCRYPT_SUCCESS(st))
        st = BCryptCreateHash(alg, &hash, nullptr, 0, const_cast<PUCHAR>(key), ULONG(keyLen), 0);
    if (BCRYPT_SUCCESS(st))
        st = BCryptHashData(hash, const_cast<PUCHAR>(msg), ULONG(msgLen), 0);
    if (BCRYPT_SUCCESS(st))
        st = BCryptFinishHash(hash, out, 32, 0);
    if (!BCRYPT_SUCCESS(st)) {
        // Never a silent pass: a zeroed tag cannot match a real one, so auth fails closed.
        memset(out, 0, 32);
        blog(LOG_ERROR, "[airlive] HMAC-SHA256 failed (0x%08lx) - the password check will refuse", st);
    }
    if (hash)
        BCryptDestroyHash(hash);
    if (alg)
        BCryptCloseAlgorithmProvider(alg, 0);
}

void secureRandom(uint8_t *out, size_t len) {
    const NTSTATUS st = BCryptGenRandom(nullptr, out, ULONG(len), BCRYPT_USE_SYSTEM_PREFERRED_RNG);
    if (!BCRYPT_SUCCESS(st))
        blog(LOG_ERROR, "[airlive] system random generator failed (0x%08lx)", st);
}

} // namespace airlive
