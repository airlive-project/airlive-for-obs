#!/bin/bash
# Compile + run the receiver-password KAT against src/auth.hpp.
# Self-contained (CommonCrypto on macOS / OpenSSL elsewhere). Exit 0 = pass.
set -euo pipefail
cd "$(dirname "$0")/.."
BIN="$(mktemp -t airlive-obs-auth-kat)"
trap 'rm -f "$BIN"' EXIT
# On non-Apple, link OpenSSL (auth.hpp uses it there); macOS uses CommonCrypto.
# The KAT runs on the Mac against the macOS crypto; Windows CI runs it against CNG.
clang++ -std=c++17 -Isrc tests/auth-kat.cpp src/platform/macos/crypto.cpp -o "$BIN"
"$BIN"
