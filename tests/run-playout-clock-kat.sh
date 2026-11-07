#!/bin/bash
# Compile + run the playout-clock KAT against src/playout-clock.hpp.
# Header-only, no OBS and no FFmpeg needed. Exit 0 = pass.
set -euo pipefail
cd "$(dirname "$0")/.."
BIN="$(mktemp -t airlive-obs-clock-kat)"
trap 'rm -f "$BIN"' EXIT
clang++ -std=c++17 -Isrc tests/playout-clock-kat.cpp -o "$BIN"
"$BIN"
