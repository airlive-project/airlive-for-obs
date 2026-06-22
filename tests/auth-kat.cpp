// auth-kat.cpp — known-answer + behaviour test for the receiver-password crypto.
//
// Self-contained (no test framework). Build + run via tests/run-auth-kat.sh.
// The KAT vector is IDENTICAL to the Bridge's (Swift) and an independent Python
// reference, so a pass here proves the OBS plugin computes the same HMAC tag the
// camera/Studio/Bridge do — bit-for-bit interop (STREAM-AUTH-SPEC §2).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "../src/auth.hpp"

using namespace airlive;

int main() {
    const std::string pw = "airlive-test";
    uint8_t nonce[32];
    for (int i = 0; i < 32; ++i)
        nonce[i] = uint8_t(i); // 0x00..0x1f
    // python: hmac.new(b"airlive-test", bytes(range(32)), hashlib.sha256).hexdigest()
    const char *expected = "f5708e4ebcf85a651f5f897323533dcf543add52d651179fbbd390124b1f4ab1";

    uint8_t tag[32];
    authResponseTag(pw, nonce, 32, tag);
    char hex[65];
    for (int i = 0; i < 32; ++i)
        snprintf(hex + i * 2, 3, "%02x", tag[i]);

    int fails = 0;
    auto check = [&](bool ok, const char *name) {
        printf("  %s %s\n", ok ? "[PASS]" : "[FAIL]", name);
        if (!ok)
            ++fails;
    };

    check(strcmp(hex, expected) == 0, "KAT — wire bytes match the independent HMAC reference");
    check(authVerify(pw, nonce, 32, tag, 32), "round-trip verify");
    check(!authVerify("wrong", nonce, 32, tag, 32), "wrong password rejected");
    check(!authVerify(pw, nonce, 32, tag, 31), "short (31-byte) tag rejected");
    uint8_t wrongNonce[32];
    memset(wrongNonce, 0xAA, 32);
    check(!authVerify(pw, wrongNonce, 32, tag, 32), "wrong nonce rejected");

    uint8_t a[4] = {1, 2, 3, 4}, b[4] = {1, 2, 3, 4}, c[4] = {1, 2, 3, 5};
    check(constantTimeEquals(a, b, 4) && !constantTimeEquals(a, c, 4), "constant-time compare");

    uint8_t n1[32], n2[32];
    fillNonce(n1);
    fillNonce(n2);
    check(memcmp(n1, n2, 32) != 0, "nonce is non-constant (single-use)");

    check(authResultJSON(true) == "{\"ok\":true}", "AuthResult success JSON");
    check(authResultJSON(false, "auth_failed").find("auth_failed") != std::string::npos,
          "AuthResult failure reason JSON");

    printf("%s\n", fails == 0 ? "ALL AUTH KAT TESTS PASSED" : "AUTH KAT TEST(S) FAILED");
    return fails;
}
