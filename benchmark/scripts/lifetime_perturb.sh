#!/usr/bin/env bash
# P12-L2 perturbation gate (docs/P12_L0_DESIGN.md, P7 golden-trace method).
# Container-side:
#   docker compose run --rm -e HEADLESS=1 dev benchmark/scripts/lifetime_perturb.sh
#
# Runs the SAME scenario (mono-inertial MH01 — the transition-richest mode:
# resets, RECENTLY_LOST, IMU init) on the gate binary (/build/cmake, no
# probes) and the lifetime-probed binary (/build/lifetime), both with
# ORB_TRACE_STATE=1, and compares the TRANSITION-TYPE SETS (distinct
# "FROM -> TO reason" triples, frame ids stripped — P7-1a trace format).
# Identical sets = the probes did not change which state-machine behaviors
# occur. Sequence/counts are NOT compared (multithread nondeterminism).
# On MISMATCH: judge stochastic-vs-instrumentation by rerunning (a type
# appearing/vanishing across reruns of the SAME arm is stochastic).
# MANDATORY before probes enter the hot Optimizer class; pilot classes
# (shutdown/solver) sit outside the realtime loop but are certified here
# too. Quiet machine required. Diagnostic-only.
set -uo pipefail

WS=/workspace
VOC=/build/ORBvoc.txt
EUROC=/datasets/EuRoc

require() { [[ -e "$1" ]] || { echo "ERROR: not found: $1" >&2; exit 1; }; }
require /build/cmake/bin/mono_inertial_euroc
require /build/lifetime/bin/mono_inertial_euroc
require "$VOC"
require "$EUROC/MH01/mav0/imu0/data.csv"

OUT="/results/perturb_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

run_arm() {  # $1 = tree (cmake|lifetime), $2 = tag (gate|probed)
  local D="$OUT/$2"
  mkdir -p "$D"
  cd "$D"
  echo ">> [$2] mono_inertial MH01 on /build/$1 $(date +%H:%M:%S)"
  ORB_TRACE_STATE=1 ORB_TRACE_STATE_FILE="$D/state_trace.txt" \
  LIFETIME_TRACE_OUT="$D/lifetime_trace.csv" \
  xvfb-run -a timeout -k 60 2700 "/build/$1/bin/mono_inertial_euroc" "$VOC" \
    "$WS/Examples/Monocular-Inertial/EuRoC.yaml" "$EUROC/MH01" \
    "$WS/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt" \
    > run.log 2>&1 || true
  require "$D/state_trace.txt"
  # Type = "FROM -> TO reason", with ONE documented normalization: the
  # cross-thread InitializeIMU line (P7_RECON T22) keeps its identity by
  # OCCURRENCE, not landing state — the golden state-trace README declares
  # its from-state pure LM-thread timing ("usually lands while already
  # OK"), so the from-state is masked to '*'. Everything else compares
  # strictly. Measured motivation: probe stamps systematically delay LM
  # enough to land T22 during RECENTLY_LOST windows (2/2 probed vs 0/2
  # gate, 2026-08-12) — a timing shift of a declared-nondeterministic
  # landing, not a novel state-machine behavior.
  cut -d' ' -f2- "$D/state_trace.txt" \
    | sed -E 's/^[A-Z_]+ (-> [A-Z_]+ LocalMapping::InitializeIMU \(cross-thread\))$/* \1/' \
    | sort -u > "$D/types.txt"
}

run_arm cmake gate
run_arm lifetime probed

echo "== transition-type sets (gate / probed) =="
paste <(wc -l < "$OUT/gate/types.txt") <(wc -l < "$OUT/probed/types.txt")
if diff -u "$OUT/gate/types.txt" "$OUT/probed/types.txt"; then
  echo "PERTURB GATE: PASS — transition-type sets identical"
else
  echo "PERTURB GATE: MISMATCH — rerun to separate stochastic from instrumentation (P7 method)" >&2
  exit 1
fi
