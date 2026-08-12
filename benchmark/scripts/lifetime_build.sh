#!/usr/bin/env bash
# P12-L0-b lifetime-trace diagnostic tree (docs/P12_L0_DESIGN.md).
# Container-side:
#   docker compose run --rm dev benchmark/scripts/lifetime_build.sh
# Builds /build/lifetime BESIDE the gate tree with -DLIFETIME_TRACE=1 armed
# (probes and the LifetimeLedger compile in; gate builds get ((void)0) —
# inertness proven by md5-identical gate binaries). Uses the gate's exact
# Release flags so probe timing perturbation is the ONLY delta vs a gate
# binary. Diagnostic-only, never ATE-judged.
set -euo pipefail

SRC=/workspace
BUILD=/build/lifetime
NPROC="$(nproc)"
JOBS=$(( NPROC < 10 ? NPROC : 10 ))

export PATH="/usr/lib/ccache:${PATH}"

cmake -S "$SRC" -B "$BUILD" -GNinja -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_FLAGS="-DLIFETIME_TRACE=1" \
  -DCMAKE_CXX_FLAGS="-DLIFETIME_TRACE=1"
cmake --build "$BUILD" -j"$JOBS"

if [[ ! -f /build/ORBvoc.txt ]]; then
  echo ">> extracting vocabulary"
  tar -xzf "$SRC/Vocabulary/ORBvoc.txt.tar.gz" -C /build
fi

echo ">> lifetime-trace build complete: $BUILD"
ls -l "$BUILD/bin"
