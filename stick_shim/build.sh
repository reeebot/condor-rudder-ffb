#!/bin/sh
# Builds the 32-bit dinput8.dll shim (Condor.exe is x86) with zig's bundled clang + mingw-w64.
#   pip install ziglang
set -e
cd "$(dirname "$0")"
python3 -m ziglang cc -target x86-windows-gnu -shared -O2 -s -Wall -Wextra -Wno-unused-parameter \
    -o dinput8.dll dinput8.c dinput8.def -lws2_32
rm -f dinput8.lib dinput8.pdb
ls -l dinput8.dll
