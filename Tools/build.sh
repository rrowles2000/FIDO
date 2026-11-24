#!/usr/bin/env bash
# Builds Fido.dll (FIDO, formerly K2D2) with plain csc against the installed KSP2 Redux runtime (no Unity editor).
# Branch redux-stable-0.2.8.5 targets Redux stable 0.2.8.5.103184 (Unity 6000.4.1f1).
# Output: Deploy/obj/Fido.dll
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
KSP2="${KSP2:-$HOME/.steam/steam/steamapps/common/Kerbal Space Program 2}"
MANAGED="$KSP2/KSP2_x64_Data/Managed"
CSC="${CSC:-$(find -L "$HOME/.steam/steam/steamapps/common" -path '*wine/mono*Roslyn/csc.exe' 2>/dev/null | sort | tail -1 || true)}"
[ -f "$CSC" ] || { echo "csc.exe not found; set CSC=" >&2; exit 1; }
mkdir -p Deploy/obj
{
  echo "-target:library -unsafe -nostdlib+ -langversion:9.0 -optimize+ -nologo"
  echo "-nowarn:0618,0612,0672,0169,0649,0414,0108,0114"
  echo "-out:Deploy/obj/Fido.dll"
  for dll in "$MANAGED"/*.dll; do echo "-r:\"$dll\""; done
  find Assets/FIDO/Code -name '*.cs' | sort | sed 's/.*/"&"/'
} > Deploy/obj/csc.rsp
mono "$CSC" @Deploy/obj/csc.rsp
if monodis --typeref Deploy/obj/Fido.dll | grep -q PanelRenderer; then
  echo "built DLL references PanelRenderer - not loadable on stable" >&2; exit 1
fi
echo "Built Deploy/obj/Fido.dll"
# --install: copy the DLL and the Patch Manager patches into the game (never Deploy's bundles)
if [ "${1:-}" = "--install" ]; then
  DEST="$KSP2/mods/FIDO"
  [ -d "$KSP2/mods/K2D2" ] && { echo "mods/K2D2 still exists: run Tools/migrate_install.py first" >&2; exit 1; }
  # copy then rename: overwriting a DLL the running game has loaded crashes it
  cp Deploy/obj/Fido.dll "$DEST/.Fido.dll.new" && mv -f "$DEST/.Fido.dll.new" "$DEST/Fido.dll"
  cp Assets/FIDO/swinfo.json "$DEST/swinfo.json"
  mkdir -p "$DEST/patches"
  cp Tools/install/patches/*.lua "$DEST/patches/"
  mkdir -p "$DEST/localizations"
  cp Tools/install/localizations/*.csv "$DEST/localizations/"
  # Patch Manager only rebuilds its cache when the mod list changes - clear it so patch edits apply
  rm -rf "$KSP2/pm_cache"
  echo "Installed to $DEST"
fi
