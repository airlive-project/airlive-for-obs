#!/bin/bash
# Compile + run the playout-ladder KAT against src/delay-ladder.hpp.
# Header-only, no OBS and no FFmpeg needed. Exit 0 = pass.
set -euo pipefail
cd "$(dirname "$0")/.."
BIN="$(mktemp -t airlive-obs-delay-kat)"
trap 'rm -f "$BIN"' EXIT
clang++ -std=c++17 -Isrc tests/delay-ladder-kat.cpp -o "$BIN"
"$BIN"
