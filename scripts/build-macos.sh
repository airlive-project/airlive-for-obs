#!/bin/bash
# build-macos.sh — compile, link, ad-hoc sign, and (optionally) install the
# Airlive OBS plugin on macOS without CMake/Xcode.
#
#   ./scripts/build-macos.sh           # build only  → build/airlive-obs.plugin
#   ./scripts/build-macos.sh install   # build + install into the user plugins dir
#
# Header/lib locations are overridable via env (defaults below). libobs headers
# come from an obsproject/obs-studio checkout; simde is a submodule of that repo.
set -euo pipefail
cd "$(dirname "$0")/.."

OBS_INC="${OBS_INC:-/tmp/obs321/libobs}"        # obs-studio/libobs (obs-module.h …)
SIMDE_INC="${SIMDE_INC:-/tmp/simde}"            # obs-studio/deps/simde
FFMPEG_INC="${FFMPEG_INC:-/opt/homebrew/include}"
FFMPEG_LIB="${FFMPEG_LIB:-/opt/homebrew/opt/ffmpeg/lib}"
OBS_APP="${OBS_APP:-/Applications/OBS.app}"
PLUGIN="airlive-obs"

BUILD="build"
rm -rf "$BUILD"; mkdir -p "$BUILD/obj"

echo "== compile =="
objs=()
for f in src/*.cpp; do
  o="$BUILD/obj/$(basename "${f%.cpp}").o"
  clang++ -std=c++17 -O2 -arch arm64 \
    -Isrc -I"$OBS_INC" -I"$SIMDE_INC" -I"$FFMPEG_INC" -c "$f" -o "$o"
  objs+=("$o")
  echo "  ok $(basename "$f")"
done

echo "== link =="
# -framework Security: the macOS Keychain (secret-store.hpp). libobs is linked by
# its in-framework dylib path so the recorded load command is @rpath-relative.
clang++ -bundle -arch arm64 -o "$BUILD/$PLUGIN" "${objs[@]}" \
  "$OBS_APP/Contents/Frameworks/libobs.framework/Versions/A/libobs" \
  -L"$FFMPEG_LIB" -lavcodec -lavutil -lswscale \
  -framework Security -framework CoreFoundation \
  -Wl,-rpath,"$OBS_APP/Contents/Frameworks" -Wl,-rpath,/opt/homebrew/lib

echo "== bundle + ad-hoc sign =="
APP="$BUILD/$PLUGIN.plugin"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Resources"
cp "$BUILD/$PLUGIN" "$APP/Contents/MacOS/$PLUGIN"
codesign --force --deep --sign - "$APP"
codesign --verify --verbose "$APP" && echo "  signed OK"

if [ "${1:-}" = "install" ]; then
  DEST="$HOME/Library/Application Support/obs-studio/plugins/$PLUGIN.plugin"
  echo "== install -> $DEST =="
  mkdir -p "$DEST/Contents/MacOS"
  cp "$APP/Contents/MacOS/$PLUGIN" "$DEST/Contents/MacOS/$PLUGIN"
  codesign --force --deep --sign - "$DEST"
  codesign --verify --verbose "$DEST" && echo "  installed + signed OK"
  pgrep -x OBS >/dev/null && echo "  ⚠️ OBS is running — quit & relaunch to load it" || true
fi
echo "done."
