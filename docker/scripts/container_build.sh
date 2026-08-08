#!/usr/bin/env bash
# Build ORB_SLAM3_Modern inside the dev container into the /build named volume.
# Records the exact compile flags next to the binaries (flag-parity evidence).
set -euo pipefail

SRC=/workspace
BUILD=/build/cmake
NPROC="$(nproc)"
JOBS=$(( NPROC < 10 ? NPROC : 10 ))

export PATH="/usr/lib/ccache:${PATH}"

cmake -S "$SRC" -B "$BUILD" -GNinja -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" -j"$JOBS"

if [[ ! -f /build/ORBvoc.txt ]]; then
  echo ">> extracting vocabulary"
  tar -xzf "$SRC/Vocabulary/ORBvoc.txt.tar.gz" -C /build
fi

# Flag evidence must come from the BUILD RULES, not CMakeCache: g2o appends
# flags at directory scope, which never reaches the cache (DIVERGENCES #18).
{
  grep -E "CMAKE_CXX_FLAGS|CMAKE_BUILD_TYPE" "$BUILD/CMakeCache.txt" | sort -u
  echo "--- actual compile rules (unique FLAGS lines) ---"
  grep -h "^  FLAGS = " "$BUILD/build.ninja" | sort -u
  echo "--- forbidden flag scan ---"
  if grep -qE "march=native|ffast-math|-msse" "$BUILD/build.ninja"; then
    echo "FAIL: forbidden numeric flag present"; grep -oE "march=native|ffast-math|-msse[0-9._a-z]*" "$BUILD/build.ninja" | sort -u
  else
    echo "OK: no march=native / ffast-math / -msse in any build rule"
  fi
} > /build/build-flags.txt
echo ">> build complete:"
ls -l "$BUILD/bin"
cat /build/build-flags.txt
