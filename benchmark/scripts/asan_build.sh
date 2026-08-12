#!/usr/bin/env bash
# P12-L0-a ASan diagnostic tree (docs/P12_L0_DESIGN.md). Container-side:
#   docker compose run --rm dev benchmark/scripts/asan_build.sh
# Builds /build/asan BESIDE the gate tree /build/cmake (untouched; ccache
# keys differ, no pollution). Command-line flag injection only — zero
# CMakeLists changes. CMAKE_CXX_FLAGS propagates to every subdirectory, so
# DBoW2 and g2o are instrumented too (an uninstrumented side would hide
# reports whose access or free site lives inside those libraries).
#
# -fsanitize-recover=address is REQUIRED: default ASan aborts on the first
# report, and the ledger needs the full signature set from one run
# (runtime side: halt_on_error=0 in asan_smoke.sh).
#
# The ASan tree is diagnostic-only and NEVER ATE-judged (same policy as
# /build/tsan). Idempotent: re-running reconfigures + rebuilds incrementally.
set -euo pipefail

SRC=/workspace
BUILD=/build/asan
NPROC="$(nproc)"
JOBS=$(( NPROC < 10 ? NPROC : 10 ))

export PATH="/usr/lib/ccache:${PATH}"

cmake -S "$SRC" -B "$BUILD" -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_FLAGS="-fsanitize=address -fsanitize-recover=address -fno-omit-frame-pointer -g -O1" \
  -DCMAKE_CXX_FLAGS="-fsanitize=address -fsanitize-recover=address -fno-omit-frame-pointer -g -O1" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address" \
  -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=address"
cmake --build "$BUILD" -j"$JOBS"

if [[ ! -f /build/ORBvoc.txt ]]; then
  echo ">> extracting vocabulary"
  tar -xzf "$SRC/Vocabulary/ORBvoc.txt.tar.gz" -C /build
fi

echo ">> ASan build complete: $BUILD"
ls -l "$BUILD/bin"
