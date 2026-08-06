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

grep -E "CMAKE_CXX_FLAGS|CMAKE_BUILD_TYPE" "$BUILD/CMakeCache.txt" | sort -u > /build/build-flags.txt
echo ">> build complete:"
ls -l "$BUILD/bin"
cat /build/build-flags.txt
