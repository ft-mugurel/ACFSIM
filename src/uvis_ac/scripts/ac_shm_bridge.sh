#!/bin/bash
# Copy Assetto Corsa physics and graphics pages out of the Proton prefix.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXE="${AC_SHM_EXE:-${HERE}/../tools/ac_shm_copy.exe}"
if [[ -n "${AC_STEAM_HOME:-}" ]]; then
  STEAM_HOME="$AC_STEAM_HOME"
elif [[ -d "${HOME}/.local/share/Steam/steamapps/common/assettocorsa" ]]; then
  STEAM_HOME="$HOME"
elif [[ -n "${DISTROBOX_HOST_HOME:-}" && -d "${DISTROBOX_HOST_HOME}/.local/share/Steam/steamapps/common/assettocorsa" ]]; then
  STEAM_HOME="$DISTROBOX_HOST_HOME"
else
  STEAM_HOME="$HOME"
fi
APP_ID="${AC_APP_ID:-244210}"
COMPAT="${STEAM_HOME}/.local/share/Steam/steamapps/compatdata/${APP_ID}"
PROTON="${AC_PROTON:-${STEAM_HOME}/.local/share/Steam/compatibilitytools.d/GE-Proton9-20/proton}"
WIN_EXE="c:\\uvis\\ac_shm_copy.exe"
if [[ ! -f "$EXE" ]]; then
  echo "Missing $EXE"
  echo "Build it with: x86_64-w64-mingw32-gcc -O2 -o tools/ac_shm_copy.exe tools/ac_shm_copy.c"
  exit 1
fi
if [[ ! -x "$PROTON" && ! -f "$PROTON" ]]; then
  echo "Proton not found at $PROTON"
  echo "Set AC_PROTON to your proton executable."
  exit 1
fi
mkdir -p "${COMPAT}/pfx/drive_c/uvis"
cp -f "$EXE" "${COMPAT}/pfx/drive_c/uvis/ac_shm_copy.exe"
export STEAM_COMPAT_CLIENT_INSTALL_PATH="${STEAM_HOME}/.local/share/Steam"
export STEAM_COMPAT_DATA_PATH="$COMPAT"
# Skip Proton's steam.exe wrapper. A unix path makes Wine pop "file not found".
export UMU_ID="umu-${APP_ID}"
exec "$PROTON" run "$WIN_EXE"
