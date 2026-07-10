#!/bin/bash
# Uninstall "Airlive for OBS" — removes JUST this plugin (Airlive Camera + Airlive Bridge
# sources) and its install receipt. Other OBS plugins, incl. Airlive Screen Mirroring, are
# left alone. Double-click to run; no admin unless an old system-wide copy is found.
set -u
USER_DIR="$HOME/Library/Application Support/obs-studio/plugins"
SYS_DIR="/Library/Application Support/obs-studio/plugins"
PLUGIN="airlive-obs.plugin"
RECEIPTS=("studio.airlive.obs.airlive-for-obs")

echo "This removes \"Airlive for OBS\" ($PLUGIN — the Airlive Camera + Airlive Bridge sources)."
echo "Airlive Screen Mirroring and every other OBS plugin are left untouched."
read -r -p "Continue? [y/N] " ans; case "$ans" in y|Y|yes|YES) ;; *) echo "Cancelled."; exit 0;; esac
osascript -e 'tell application "OBS" to quit' 2>/dev/null || true; sleep 1

removed=0
[ -e "$USER_DIR/$PLUGIN" ] && rm -rf "$USER_DIR/$PLUGIN" && { echo "  removed $USER_DIR/$PLUGIN"; removed=1; }
if [ -e "$SYS_DIR/$PLUGIN" ]; then
  echo "Found an older system-wide copy in $SYS_DIR — removing it needs your password."
  sudo rm -rf "$SYS_DIR/$PLUGIN" && { echo "  removed $SYS_DIR/$PLUGIN"; removed=1; }
  for id in "${RECEIPTS[@]}"; do sudo pkgutil --forget "$id" >/dev/null 2>&1 || true; done
fi
for id in "${RECEIPTS[@]}"; do pkgutil --forget "$id" >/dev/null 2>&1 || true; done

[ "$removed" = 1 ] && echo "✅ Airlive for OBS removed. Restart OBS." || echo "Nothing to remove — it wasn't installed."
