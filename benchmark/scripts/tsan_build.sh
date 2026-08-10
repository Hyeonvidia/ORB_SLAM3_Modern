#!/usr/bin/env bash
# P10 TSAN diagnostic tree (docs/P10_RECON.md 3부 §1). Container-side:
#   docker compose run --rm dev benchmark/scripts/tsan_build.sh
# Builds /build/tsan BESIDE the gate tree /build/cmake (which stays
# untouched; ccache keys differ, no pollution). Command-line flag injection
# only — zero CMakeLists changes. CMAKE_CXX_FLAGS propagates to every
# subdirectory, so DBoW2 and g2o are instrumented too (required: the
# mbAbortBA/mbStopGBA bool* polling happens inside g2o, and ComputeBoW
# writes happen inside DBoW2 — an uninstrumented side hides the race).
# The TSAN tree is diagnostic-only and never ATE-judged; the P1 "don't
# touch Thirdparty flags" gate constraint does not apply here.
# Idempotent: re-running reconfigures + rebuilds incrementally.
set -euo pipefail

SRC=/workspace
BUILD=/build/tsan
NPROC="$(nproc)"
JOBS=$(( NPROC < 10 ? NPROC : 10 ))

export PATH="/usr/lib/ccache:${PATH}"

cmake -S "$SRC" -B "$BUILD" -GNinja -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_C_FLAGS="-fsanitize=thread -fno-omit-frame-pointer -g -O1" \
  -DCMAKE_CXX_FLAGS="-fsanitize=thread -fno-omit-frame-pointer -g -O1" \
  -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=thread" \
  -DCMAKE_SHARED_LINKER_FLAGS="-fsanitize=thread"
cmake --build "$BUILD" -j"$JOBS"

if [[ ! -f /build/ORBvoc.txt ]]; then
  echo ">> extracting vocabulary"
  tar -xzf "$SRC/Vocabulary/ORBvoc.txt.tar.gz" -C /build
fi

echo ">> TSAN build complete: $BUILD"
ls -l "$BUILD/bin"
