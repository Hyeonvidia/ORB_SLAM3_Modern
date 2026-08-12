#!/usr/bin/env bash
# P12-G0-1 timing-profile diagnostic tree (docs/P12_PLAN.md §3 G0-1).
# Container-side:
#   docker compose run --rm dev benchmark/scripts/times_build.sh
# Builds /build/times with upstream's preserved REGISTER_TIMES
# instrumentation armed (per-stage timing vectors + PrintTimeStats at
# Shutdown — Tracking/LocalMapping/LoopClosing sections). Gate Release
# flags otherwise, so the numbers are representative of the gate binary.
# Purpose: measure whether g2o optimization time is actually the
# bottleneck on arm64 — the GTSAM go/no-go evidence (P12-D4: container
# proxy first). Diagnostic-only, never ATE-judged.
set -euo pipefail

SRC=/workspace
BUILD=/build/times
NPROC="$(nproc)"
JOBS=$(( NPROC < 10 ? NPROC : 10 ))

export PATH="/usr/lib/ccache:${PATH}"

cmake -S "$SRC" -B "$BUILD" -GNinja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="-DREGISTER_TIMES" \
  -DCMAKE_CXX_FLAGS="-DREGISTER_TIMES"
cmake --build "$BUILD" -j"$JOBS"

if [[ ! -f /build/ORBvoc.txt ]]; then
  echo ">> extracting vocabulary"
  tar -xzf "$SRC/Vocabulary/ORBvoc.txt.tar.gz" -C /build
fi

echo ">> REGISTER_TIMES build complete: $BUILD"
ls -l "$BUILD/bin"
