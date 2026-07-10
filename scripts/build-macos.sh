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
MIN_MACOS="${MIN_MACOS:-13.0}"                  # deployment target — ≤ OBS's own (12), matches Bridge (13)
PLUGIN="airlive-obs"
PLUGIN_VERSION="${PLUGIN_VERSION:-1.0}"

BUILD="build"
rm -rf "$BUILD"; mkdir -p "$BUILD/obj"

echo "== compile =="
objs=()
for f in src/*.cpp; do
  o="$BUILD/obj/$(basename "${f%.cpp}").o"
  clang++ -std=c++17 -O2 -arch arm64 -mmacosx-version-min="$MIN_MACOS" \
    -Isrc -I"$OBS_INC" -I"$SIMDE_INC" -I"$FFMPEG_INC" -c "$f" -o "$o"
  objs+=("$o")
  echo "  ok $(basename "$f")"
done

echo "== link =="
# -framework Security: the macOS Keychain (secret-store.hpp). libobs is linked by
# its in-framework dylib path so the recorded load command is @rpath-relative.
# -headerpad_max_install_names: leave room so dylibbundler can rewrite the
#   FFmpeg load commands to @loader_path (longer than the linked paths).
# rpath @executable_path/../Frameworks: @executable_path is OBS's own binary
#   (obs64), so this resolves @rpath/libobs.framework to OBS.app/Contents/
#   Frameworks WHEREVER OBS is installed — the standard OBS-plugin idiom. NOT a
#   build-machine-absolute /Applications/OBS.app path. (Our bundled FFmpeg uses
#   @loader_path, the plugin's own dir, so it needs no rpath.)
clang++ -bundle -arch arm64 -mmacosx-version-min="$MIN_MACOS" \
  -Wl,-headerpad_max_install_names -o "$BUILD/$PLUGIN" "${objs[@]}" \
  "$OBS_APP/Contents/Frameworks/libobs.framework/Versions/A/libobs" \
  -L"$FFMPEG_LIB" -lavcodec -lavutil -lswscale \
  -framework Security -framework CoreFoundation \
  -Wl,-rpath,@executable_path/../Frameworks

echo "== assemble .plugin bundle =="
APP="$BUILD/$PLUGIN.plugin"
rm -rf "$APP"
mkdir -p "$APP/Contents/MacOS" "$APP/Contents/Frameworks" "$APP/Contents/Resources"
cp "$BUILD/$PLUGIN" "$APP/Contents/MacOS/$PLUGIN"

# Info.plist — macOS won't load a bundle without it (CFBundleExecutable).
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>CFBundleDevelopmentRegion</key><string>en</string>
  <key>CFBundleExecutable</key><string>$PLUGIN</string>
  <key>CFBundleIdentifier</key><string>com.airlive.$PLUGIN</string>
  <key>CFBundleInfoDictionaryVersion</key><string>6.0</string>
  <key>CFBundleName</key><string>$PLUGIN</string>
  <key>CFBundlePackageType</key><string>BNDL</string>
  <key>CFBundleShortVersionString</key><string>$PLUGIN_VERSION</string>
  <key>CFBundleVersion</key><string>$PLUGIN_VERSION</string>
  <key>LSMinimumSystemVersion</key><string>$MIN_MACOS</string>
  <key>NSHumanReadableCopyright</key><string>Airlive</string>
</dict></plist>
PLIST

# OBS_MODULE_USE_DEFAULT_LOCALE reads locale/*.ini from the bundle's Resources.
[ -d data ] && cp -R data/. "$APP/Contents/Resources/"

# Bundle FFmpeg (+ its dependency closure) INTO the plugin at
# @loader_path/../Frameworks so it is self-contained — no /opt/homebrew at
# runtime. Point FFMPEG_LIB at a LOW-deployment-target FFmpeg (obs-deps) so the
# copied dylibs' minos ≤ MIN_MACOS; Homebrew's target the host OS and would
# raise the floor. dylibbundler recursively copies + rewrites install names.
echo "== bundle FFmpeg (dylibbundler) =="
dylibbundler -od -b -x "$APP/Contents/MacOS/$PLUGIN" \
  -d "$APP/Contents/Frameworks" -p "@loader_path/../Frameworks" \
  -s "$FFMPEG_LIB" >/dev/null

# dylibbundler rewrites our LC_RPATH to @loader_path; re-assert @executable_path/
# ../Frameworks (= OBS's own Frameworks, since @executable_path is obs64) so
# @rpath/libobs.framework resolves without relying on obs64's inherited rpath.
otool -l "$APP/Contents/MacOS/$PLUGIN" | grep -q "@executable_path/../Frameworks" \
  || install_name_tool -add_rpath @executable_path/../Frameworks "$APP/Contents/MacOS/$PLUGIN"

# Fail loud (no silent regressions): non-portable path leak, or a bundled
# binary whose minos exceeds the deployment target (would break launch on
# every macOS below it).
LEAK=$(otool -L "$APP/Contents/MacOS/$PLUGIN" | grep -E '/opt/homebrew|/usr/local/(Cellar|opt)' || true)
[ -n "$LEAK" ] && { echo "✗ non-portable path leaked into the plugin:"; echo "$LEAK"; exit 1; }
for d in "$APP/Contents/MacOS/$PLUGIN" "$APP"/Contents/Frameworks/*.dylib; do
  [ -e "$d" ] || continue
  mv=$(otool -l "$d" 2>/dev/null | awk '/LC_BUILD_VERSION/{f=1} f&&/minos/{print $2; exit}')
  [ -n "$mv" ] && awk "BEGIN{exit !($mv > $MIN_MACOS)}" \
    && { echo "✗ $(basename "$d") minos=$mv > target $MIN_MACOS — use obs-deps FFmpeg (FFMPEG_LIB/FFMPEG_INC)"; exit 1; }
done

echo "== sign (inside-out, hardened runtime) =="
# Sign nested dylibs FIRST, then the bundle once WITHOUT --deep (deprecated for
# signing since macOS 13 — it can't selectively harden the main binary while
# leaving third-party dylibs alone). --options runtime turns on the hardened
# runtime even for ad-hoc dev builds (matches obs-plugintemplate Release; a real
# Developer ID identity would add --timestamp here).
SIGN_ID="${SIGN_ID:--}"                          # ad-hoc by default; override with a Developer ID
for dylib in "$APP"/Contents/Frameworks/*.dylib; do
  [ -e "$dylib" ] && codesign --force --options runtime --sign "$SIGN_ID" "$dylib"
done
codesign --force --options runtime --sign "$SIGN_ID" "$APP"
codesign --verify --verbose "$APP" && echo "  signed OK"

if [ "${1:-}" = "install" ]; then
  DEST="$HOME/Library/Application Support/obs-studio/plugins/$PLUGIN.plugin"
  echo "== install -> $DEST =="
  rm -rf "$DEST"; mkdir -p "$(dirname "$DEST")"
  cp -R "$APP" "$DEST"          # nested signatures are preserved by cp -R
  codesign --force --options runtime --sign "$SIGN_ID" "$DEST"
  codesign --verify --verbose "$DEST" && echo "  installed + signed OK"
  pgrep -x OBS >/dev/null && echo "  ⚠️ OBS is running — quit & relaunch to load it" || true
fi
echo "done."
