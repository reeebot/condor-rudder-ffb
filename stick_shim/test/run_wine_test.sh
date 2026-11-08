#!/bin/sh
# Runs test_shim.exe under 32-bit wine with mock_dinput8.dll as the "system" DirectInput.
set -e
cd "$(dirname "$0")"
../build.sh >/dev/null
python3 -m ziglang cc -target x86-windows-gnu -O1 -o test_shim.exe test_shim.c -ldxguid -lws2_32
P=${WINEPREFIX:-/tmp/shim_wineprefix}
export WINEPREFIX=$P WINEARCH=win32 WINEDEBUG=-all WINEDLLOVERRIDES="dinput8=n,b;mscoree=;mshtml="
[ -d "$P/drive_c/windows/system32" ] || wineboot -i >/dev/null 2>&1
W=$(mktemp -d)
cp ../dinput8.dll test_shim.exe "$W/"
cp mock_dinput8.dll "$P/drive_c/windows/system32/dinput8.dll"
set +e
(cd "$W" && timeout 120 wine test_shim.exe)
rc=$?
echo "--- ffb_shim.log"
cat "$W/ffb_shim.log"
exit $rc
