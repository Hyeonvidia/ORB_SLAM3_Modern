#!/usr/bin/env bash
# P6 backend-equivalence harness driver (docs/P6_DESIGN.md §B).
#
# P6-4 scope: BOTH variants x ALL five function/fixture pairs.
#   1. configure + build tests/backend_equiv twice in the dev container:
#        EQUIV_BACKEND=vendored -> /build/equiv_vendored (frozen golden
#                                  reference_backend + Thirdparty/g2o)
#        EQUIV_BACKEND=modern   -> /build/equiv_modern (live src/backend +
#                                  third_party/g2o @ 20241228_git)
#   2. run the function matrix in each (PAIRS below); every invocation prints
#      TWO records built from independently constructed fixtures
#   3. self-determinism gate per variant x function: the two records must be
#      byte-identical (SHA256 printed as evidence)
#   4. compare.py --self-test on each record (comparator sanity)
#   5. per-function fixture contract:
#        pose_optimization:          inlier count == designed 36 of 40
#        every other pair:           compare.py --gt-check — analytic GT
#                                    recovery (per-record gates, see compare.py)
#   6. CROSS-VARIANT GATE per function: compare.py vendored vs modern
#      (tolerance table in compare.py — design §B.2 layer 1)
#
# Run from anywhere; requires the orbslam3-modern-dev compose service.
#
# KNOWN EXPECTED MISMATCH since R4a (534c739, 2026-08-20): the cross-variant
# gate for inertial_optimization_full FAILS on scale (rel ~1e-5) and ba
# (~5e-9). R4a promoted the #9 EdgeInertialGS scale-Jacobian chain-rule fix
# unconditionally into the live backend, while reference_backend/G2oTypes.cpp
# keeps the upstream bug BY DESIGN (it is the pre-fix golden reference).
# The buggy Jacobian stalls that variant's LM at ~1e-5 from GT while the
# fixed one converges to ~1e-8 — both inside their documented --gt-check
# gates, which are the operative acceptance for this pair post-R4a. The
# mismatch went unobserved between R4a and R4b slice 2 because the vendored
# build tree had been unbuildable since R2 (system-Eigen removal); slice 2
# repaired it. The other four cross gates remain EQUIVALENT.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUT_DIR="$SCRIPT_DIR/out"
mkdir -p "$OUT_DIR"

cd "$REPO_ROOT"

# function:fixture matrix (1:1 pairing; the runner rejects any other combo)
PAIRS=(
  "pose_optimization:mono_grid"
  "inertial_optimization:imu_chain"
  "inertial_optimization_full:imu_chain_bias"
  "inertial_optimization_bias:imu_chain_metric"
  "pose_inertial_lastkf:mono_imu_link"
)
FUNCTIONS=()
for P in "${PAIRS[@]}"; do FUNCTIONS+=("${P%%:*}"); done

sha256() {  # portable: macOS shasum / Linux sha256sum
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$1" | cut -d" " -f1
  else shasum -a 256 "$1" | cut -d" " -f1; fi
}

# ---- per-variant: build, then per function: run, self-determinism,
#      self-test, fixture contract ---------------------------------------------
run_variant() {  # $1 = vendored | modern
  local V="$1"

  echo ">> [$V build] configure + build (EQUIV_BACKEND=$V) in container"
  docker compose run --rm dev bash -c '
    set -euo pipefail
    export PATH="/usr/lib/ccache:${PATH}"
    NPROC="$(nproc)"; JOBS=$(( NPROC < 8 ? NPROC : 8 ))
    cmake -S /workspace/tests/backend_equiv -B /build/equiv_'"$V"' -GNinja \
          -DEQUIV_BACKEND='"$V"' -DCMAKE_BUILD_TYPE=Release
    cmake --build /build/equiv_'"$V"' -j"$JOBS"
    grep -E "CMAKE_CXX_FLAGS|CMAKE_BUILD_TYPE|EQUIV_BACKEND" \
         /build/equiv_'"$V"'/CMakeCache.txt | sort -u \
         > /workspace/tests/backend_equiv/out/build-flags-'"$V"'.txt
  ' </dev/null

  echo ">> [$V run] function matrix"
  local CMDS=""
  local P FN FX
  for P in "${PAIRS[@]}"; do
    FN="${P%%:*}"; FX="${P##*:}"
    CMDS+="/build/equiv_${V}/equiv_runner ${FN} ${FX} > /workspace/tests/backend_equiv/out/records_${V}_${FN}.txt"$'\n'
  done
  docker compose run --rm dev bash -c "set -euo pipefail
$CMDS" </dev/null

  for FN in "${FUNCTIONS[@]}"; do
    local RECORDS="$OUT_DIR/records_${V}_${FN}.txt"
    local REC1="$OUT_DIR/record_${V}_${FN}_run1.txt"
    local REC2="$OUT_DIR/record_${V}_${FN}_run2.txt"
    rm -f "$REC1" "$REC2"

    # Split the two BEGIN_RECORD/END_RECORD blocks into separate files.
    awk -v r1="$REC1" -v r2="$REC2" '
      /^BEGIN_RECORD$/ { n++; inrec=1; next }
      /^END_RECORD$/   { inrec=0; next }
      inrec && n==1 { print > r1 }
      inrec && n==2 { print > r2 }
    ' "$RECORDS"

    local H1 H2
    H1="$(sha256 "$REC1")"
    H2="$(sha256 "$REC2")"
    echo ">> [$V $FN] self-determinism gate"
    echo "   record run1 SHA256: $H1"
    echo "   record run2 SHA256: $H2"
    if [[ "$H1" != "$H2" ]]; then
      echo "FAIL: $V $FN records differ across runs (nondeterminism)" >&2
      exit 1
    fi
    echo "   OK: records identical -> PASS $V $FN self-determinism"

    echo ">> [$V $FN] compare.py self-test"
    python3 "$SCRIPT_DIR/compare.py" --self-test "$REC1"

    case "$FN" in
      pose_optimization)
        echo ">> [$V $FN] fixture inlier contract (designed: 36 of 40)"
        local INLIERS
        INLIERS="$(awk '$1=="inliers"{print $2; exit}' "$REC1")"
        if [[ "$INLIERS" != "36" ]]; then
          echo "FAIL: $V inlier count $INLIERS != designed 36" >&2
          exit 1
        fi
        echo "   OK: inliers = $INLIERS"
        ;;
      *)
        echo ">> [$V $FN] analytic GT recovery gate (compare.py --gt-check)"
        python3 "$SCRIPT_DIR/compare.py" --gt-check "$REC1"
        ;;
    esac
  done
}

run_variant vendored
run_variant modern

echo ""
for FN in "${FUNCTIONS[@]}"; do
  echo ">> [cross] compare.py vendored vs modern ($FN)"
  python3 "$SCRIPT_DIR/compare.py" \
      "$OUT_DIR/record_vendored_${FN}_run1.txt" \
      "$OUT_DIR/record_modern_${FN}_run1.txt"
done

echo ""
echo "PASS backend_equiv: vendored + modern self-determinism, GT recovery, cross-variant equivalence"
for FN in "${FUNCTIONS[@]}"; do
  echo "  [$FN] vendored record hash: $(sha256 "$OUT_DIR/record_vendored_${FN}_run1.txt")"
  echo "  [$FN] modern   record hash: $(sha256 "$OUT_DIR/record_modern_${FN}_run1.txt")"
done
