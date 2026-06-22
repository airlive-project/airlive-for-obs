#!/bin/bash
# Compile + run the receiver-password KAT against src/auth.hpp.
# Self-contained (CommonCrypto on macOS / OpenSSL elsewhere). Exit 0 = pass.
set -euo pipefail
cd "$(dirname "$0")/.."
BIN="$(mktemp -t airlive-obs-auth-kat)"
trap 'rm -f "$BIN"' EXIT
# On non-Apple, link OpenSSL (auth.hpp uses it there); macOS uses CommonCrypto.
EXTRA=""
[ "$(uname)" != "Darwin" ] && EXTRA="-lcrypto"
clang++ -std=c++17 -Isrc tests/auth-kat.cpp -o "$BIN" $EXTRA
"$BIN"
