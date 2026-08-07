#!/usr/bin/env bash
# P6 backend-equivalence harness driver (docs/P6_DESIGN.md §B).
#
# P6-0 scope: vendored (frozen golden) variant only.
#   1. configure + build tests/backend_equiv with EQUIV_BACKEND=vendored in
#      the dev container (/build/equiv_vendored named-volume build tree,
#      docker/scripts/container_build.sh pattern)
#   2. run equiv_runner pose_optimization mono_grid (prints TWO records from
#      independently built fixtures)
#   3. self-determinism gate: the two records must be byte-identical
#      (SHA256 printed as evidence)
#   4. compare.py --self-test on the record (comparator sanity gate)
#   5. fixture contract: inlier count must equal the designed 36 of 40
#
# P6-2 will add the modern variant build + cross-variant compare.py runs.
#
# Run from anywhere; requires the orbslam3-modern-dev compose service.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT_DIR="$SCRIPT_DIR/out"
mkdir -p "$OUT_DIR"

cd "$REPO_ROOT"

echo ">> [1/5] configure + build (vendored) in container"
docker compose run --rm dev bash -c '
  set -euo pipefail
  export PATH="/usr/lib/ccache:${PATH}"
  NPROC="$(nproc)"; JOBS=$(( NPROC < 8 ? NPROC : 8 ))
  cmake -S /workspace/tests/backend_equiv -B /build/equiv_vendored -GNinja \
        -DEQUIV_BACKEND=vendored -DCMAKE_BUILD_TYPE=Release
  cmake --build /build/equiv_vendored -j"$JOBS"
  grep -E "CMAKE_CXX_FLAGS|CMAKE_BUILD_TYPE|EQUIV_BACKEND" \
       /build/equiv_vendored/CMakeCache.txt | sort -u \
       > /workspace/tests/backend_equiv/out/build-flags-vendored.txt
' </dev/null

echo ">> [2/5] run equiv_runner pose_optimization mono_grid"
docker compose run --rm dev bash -c '
  set -euo pipefail
  /build/equiv_vendored/equiv_runner pose_optimization mono_grid \
      > /workspace/tests/backend_equiv/out/records_vendored.txt
' </dev/null

RECORDS="$OUT_DIR/records_vendored.txt"
REC1="$OUT_DIR/record_vendored_run1.txt"
REC2="$OUT_DIR/record_vendored_run2.txt"

# Split the two BEGIN_RECORD/END_RECORD blocks into separate files.
awk -v r1="$REC1" -v r2="$REC2" '
  /^BEGIN_RECORD$/ { n++; inrec=1; next }
  /^END_RECORD$/   { inrec=0; next }
  inrec && n==1 { print > r1 }
  inrec && n==2 { print > r2 }
' "$RECORDS"

sha256() {  # portable: macOS shasum / Linux sha256sum
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d" " -f1
  else shasum -a 256 "$1" | cut -d" " -f1; fi
}

H1="$(sha256 "$REC1")"
H2="$(sha256 "$REC2")"
echo ">> [3/5] self-determinism gate"
echo "   record run1 SHA256: $H1"
echo "   record run2 SHA256: $H2"
if [[ "$H1" != "$H2" ]]; then
  echo "FAIL: records differ across runs (nondeterminism)" >&2
  exit 1
fi
echo "   OK: records identical"

echo ">> [4/5] compare.py self-test"
python3 "$SCRIPT_DIR/compare.py" --self-test "$REC1"

echo ">> [5/5] fixture inlier contract (designed: 36 of 40)"
INLIERS="$(awk '$1=="inliers"{print $2; exit}' "$REC1")"
if [[ "$INLIERS" != "36" ]]; then
  echo "FAIL: inlier count $INLIERS != designed 36" >&2
  exit 1
fi
echo "   OK: inliers = $INLIERS"

echo ""
echo "PASS backend_equiv vendored bootstrap"
echo "  record hash: $H1"
