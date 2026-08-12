#!/usr/bin/env bash
# P12-L0-a ASan smoke scenarios (docs/P12_L0_DESIGN.md). Container-side:
#   docker compose run --rm -e HEADLESS=1 dev benchmark/scripts/asan_smoke.sh <A1|A2|A3>
# Runs the /build/asan binaries on full sequences (ASan is only ~2x slower,
# unlike TSAN's 5-20x — no timestamp truncation needed).
#
#   A1  euroc_stereo MH01           -> mapping/culling/tombstone hot paths
#   A2  euroc_mono_inertial MH01    -> IMU init + reset storm (map churn)
#   A3  kitti_stereo 00             -> loop closing + GBA (add when L2
#                                      reaches the LoopClosing site class,
#                                      L0-D3)
#
# ASAN_OPTIONS policy (do not change without updating P12_L0_DESIGN.md):
#   detect_leaks=0   MANDATORY — the tombstone is an intentional-leak
#                    contract (OWNERSHIP rule 1); LSan output would drown
#                    the real signal in thousands of by-design leaks.
#   halt_on_error=0  collect every report in one run (pairs with the
#                    -fsanitize-recover=address compile flag).
#
# BASELINE CLAIM: on the current tree these runs must produce ZERO reports —
# map-resident KF/MP are never freed, so UAF is impossible by construction
# (the only real deletes are the #19-guarded un-admitted-queue paths,
# unreached in these workloads). The empty baseline is the attribution
# anchor: any report appearing during an L2 migration commit belongs to
# that commit. Raw reports land in <rundir>/asan.<pid>; feed them to
# asan_ledger.py. Diagnostic-only — NEVER ATE-judged.
set -euo pipefail

SCEN="${1:?usage: asan_smoke.sh <A1|A2|A3>}"

WS=/workspace
BIN=/build/asan/bin
VOC=/build/ORBvoc.txt
EUROC=/datasets/EuRoc
KITTI=/datasets/kitti_dataset/data_odometry_gray/dataset/sequences

require() { [[ -e "$1" ]] || { echo "ERROR: not found: $1" >&2; exit 1; }; }
require "$BIN"
require "$VOC"

OUT="/results/asan_${SCEN}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

case "$SCEN" in
  A1)
    require "$EUROC/MH01/mav0/cam1/data"
    CMD=("$BIN/stereo_euroc" "$VOC" "$WS/Examples/Stereo/EuRoC.yaml" "$EUROC/MH01" "$WS/Examples/Stereo/EuRoC_TimeStamps/MH01.txt") ;;
  A2)
    require "$EUROC/MH01/mav0/imu0/data.csv"
    CMD=("$BIN/mono_inertial_euroc" "$VOC" "$WS/Examples/Monocular-Inertial/EuRoC.yaml" "$EUROC/MH01" "$WS/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt") ;;
  A3)
    require "$KITTI/00/image_1"
    CMD=("$BIN/stereo_kitti" "$VOC" "$WS/Examples/Stereo/KITTI00-02.yaml" "$KITTI/00") ;;
  *) echo "ERROR: unknown scenario '$SCEN' (A1|A2|A3)" >&2; exit 1 ;;
esac

export ASAN_OPTIONS="detect_leaks=0:halt_on_error=0:log_path=$OUT/asan"

cd "$OUT"
echo ">> ASan $SCEN -> results/$(basename "$OUT")"
echo ">> ASAN_OPTIONS=$ASAN_OPTIONS"

# 45 min hard bound (ASan ~2x; teardown crash tolerated — judge by asan.*).
CMD=(timeout -k 60 2700 "${CMD[@]}")
RC=0
if [[ "${HEADLESS:-1}" == "1" ]]; then
  xvfb-run -a "${CMD[@]}" 2>&1 | tee run.log || RC=$?
else
  "${CMD[@]}" 2>&1 | tee run.log || RC=$?
fi
[[ $RC -eq 124 ]] && echo "!! TIMEOUT (exit 124): possible hang" >&2

echo ">> exit=$RC; ASan report files:"
ls -l "$OUT"/asan.* 2>/dev/null || echo "(no asan.* files — zero reports: baseline claim holds)"
echo ">> ledger: python3 benchmark/scripts/asan_ledger.py results/$(basename "$OUT")"
