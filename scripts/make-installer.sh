#!/usr/bin/env bash
# make-installer.sh — package the already-built "Airlive for OBS" plugin (Airlive Camera +
# Airlive Bridge sources) into a double-click .pkg for OBS Studio.
#
# Run AFTER scripts/build-macos.sh (which produces a self-contained, minos-restamped, signed
# build/airlive-obs.plugin). This step only wraps that bundle in a pkg — no re-bundling.
#
# Signed release: set INSTALLER_ID="Developer ID Installer: … (TEAMID)" then notarize + staple
# the pkg (see airlive-bridge/docs/APPLE-DEVELOPER-RELEASE.md). Unset → unsigned pkg (right-click → Open).
set -euo pipefail

cd "$(dirname "$0")/.."
ROOT="$(pwd)"
PRODUCT="Airlive for OBS"
VERSION="${VERSION:-1.0.0}"
PKG_ID="studio.airlive.obs.airlive-for-obs"
BUNDLE_NAME="airlive-obs.plugin"
SRC_PLUGIN="$ROOT/build/$BUNDLE_NAME"
MIN_MACOS="${MIN_MACOS:-14.0}"; MIN_MACOS_MAJOR="${MIN_MACOS%%.*}"

DIST="$ROOT/dist"
PAYLOAD="$DIST/payload"
SCRIPTS="$DIST/scripts"
RESOURCES="$DIST/resources"
INSTALL_DIR="Library/Application Support/obs-studio/plugins"
PKG_OUT="$DIST/Airlive-for-OBS.pkg"

[ -d "$SRC_PLUGIN" ] || { echo "✗ $SRC_PLUGIN not found — run scripts/build-macos.sh first" >&2; exit 1; }

echo "==> Cleaning previous build"
rm -rf "$DIST"; mkdir -p "$PAYLOAD/$INSTALL_DIR" "$SCRIPTS" "$RESOURCES"

echo "==> Staging $BUNDLE_NAME (already self-contained from build-macos.sh)"
COPYFILE_DISABLE=1 cp -R "$SRC_PLUGIN" "$PAYLOAD/$INSTALL_DIR/"

echo "==> Verifying deployment target (build-macos.sh should already guarantee this)"
FW="$PAYLOAD/$INSTALL_DIR/$BUNDLE_NAME/Contents/Frameworks"
shopt -s nullglob
for macho in "$FW"/*.dylib "$PAYLOAD/$INSTALL_DIR/$BUNDLE_NAME/Contents/MacOS/"*; do
  leak="$(otool -L "$macho" 2>/dev/null | grep -E '/opt/homebrew|/usr/local/Cellar' || true)"
  [ -z "$leak" ] || { echo "✗ $(basename "$macho") leaks a Homebrew path:"; echo "$leak"; exit 1; }
  while read -r minos; do maj=${minos%%.*}
    [ -n "$maj" ] && [ "$maj" -gt "$MIN_MACOS_MAJOR" ] && { echo "✗ $(basename "$macho") minos=$minos > $MIN_MACOS"; exit 1; }
  done < <(vtool -show-build "$macho" 2>/dev/null | awk '/minos/{print $2}')
done
shopt -u nullglob

echo "==> Writing postinstall (strip quarantine; NO force-quit of OBS)"
cat > "$SCRIPTS/postinstall" <<POST
#!/bin/bash
# Runs as the USER (currentUserHome install, no root); \$HOME expands at run time to their home.
xattr -dr com.apple.quarantine "\$HOME/$INSTALL_DIR/$BUNDLE_NAME" 2>/dev/null || true
# Deliberately does NOT quit OBS — force-quitting can lose unsaved scenes. The plugin loads
# on OBS's next launch (the conclusion screen tells the user to restart OBS).
exit 0
POST
chmod +x "$SCRIPTS/postinstall"

printf '%s\n' \
  "Airlive for OBS — iPhone camera sources for OBS Studio" "" \
  "После установки перезапусти OBS, затем Sources (+):" \
  "  • Airlive Camera — телефон с приложением Airlive Camera напрямую" \
  "  • Airlive Bridge — приём из приложения Airlive Bridge" "" \
  "Требования: macOS $MIN_MACOS+, OBS Studio." > "$RESOURCES/welcome.txt"
printf '%s\n' "Готово. Перезапусти OBS — источники Airlive Camera / Airlive Bridge появятся в меню +." \
  "" "Удаление: запусти uninstall.command рядом с этим пакетом." > "$RESOURCES/conclusion.txt"

echo "==> Building component .pkg"
COMPONENT="$DIST/component.pkg"
pkgbuild --root "$PAYLOAD" --identifier "$PKG_ID" --version "$VERSION" \
         --scripts "$SCRIPTS" --install-location "/" "$COMPONENT" >/dev/null

echo "==> Wrapping into distribution .pkg"
DIST_XML="$DIST/distribution.xml"
cat > "$DIST_XML" <<XML
<?xml version="1.0" encoding="utf-8"?>
<installer-gui-script minSpecVersion="2">
  <title>$PRODUCT</title>
  <organization>studio.airlive</organization>
  <welcome    file="welcome.txt"    mime-type="text/plain"/>
  <conclusion file="conclusion.txt" mime-type="text/plain"/>
  <options customize="never" require-scripts="false" hostArchitectures="arm64,x86_64"/>
  <!-- USER-home install, no admin — the OBS-plugin standard (DistroAV); /Library is a bug upstream. -->
  <domains enable_anywhere="false" enable_currentUserHome="true" enable_localSystem="false"/>
  <volume-check><allowed-os-versions><os-version min="$MIN_MACOS"/></allowed-os-versions></volume-check>
  <choices-outline><line choice="default"><line choice="$PKG_ID"/></line></choices-outline>
  <choice id="default"/>
  <choice id="$PKG_ID" visible="false"><pkg-ref id="$PKG_ID"/></choice>
  <pkg-ref id="$PKG_ID" version="$VERSION" onConclusion="none">component.pkg</pkg-ref>
</installer-gui-script>
XML

# if/else, not a "${arr[@]}" expansion — an empty array under `set -u` throws on bash 3.2.
if [ -n "${INSTALLER_ID:-}" ]; then
  productbuild --distribution "$DIST_XML" --resources "$RESOURCES" --package-path "$DIST" \
    --sign "$INSTALLER_ID" "$PKG_OUT" >/dev/null
else
  productbuild --distribution "$DIST_XML" --resources "$RESOURCES" --package-path "$DIST" \
    "$PKG_OUT" >/dev/null
fi

rm -rf "$PAYLOAD" "$SCRIPTS" "$RESOURCES" "$COMPONENT" "$DIST_XML"
cp "$ROOT/scripts/uninstall.command" "$DIST/uninstall.command" 2>/dev/null && chmod +x "$DIST/uninstall.command" || true

echo ""
echo "==> Done!  $PKG_OUT ($(du -h "$PKG_OUT" | cut -f1))"
echo "    + $DIST/uninstall.command (ship it alongside)"
[ -z "${INSTALLER_ID:-}" ] && echo "    UNSIGNED — recipient: right-click → Open. (Set INSTALLER_ID + notarize for a clean install.)"
