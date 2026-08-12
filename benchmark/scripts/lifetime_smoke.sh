#!/usr/bin/env bash
# P12-L0-b lifetime-trace pilot scenarios (docs/P12_L0_DESIGN.md).
# Container-side:
#   docker compose run --rm -e HEADLESS=1 dev benchmark/scripts/lifetime_smoke.sh <L1|L2>
#
#   L1  euroc_stereo MH01         -> culling churn; trajectory-save walks
#   L2  euroc_mono_inertial MH01  -> reset storm; heaviest bad-KF traffic
#   L3  kitti_stereo 00           -> loop closing: Sim3Solver candidate
#                                    loops over possibly-culled MPs (the
#                                    solver-local class evidence)
#
# The run writes <rundir>/lifetime_trace.csv at shutdown (LifetimeLedger
# flush; clean exit required — P10-5 join chain provides it). Feed the CSV
# to lifetime_report.py. Pilot probe coverage: solver-local class (2 sites)
# + shutdown/serialization class (18 sites) ONLY — wider classes arm
# class-by-class alongside L2 (see P12_L0_DESIGN.md).
set -euo pipefail

SCEN="${1:?usage: lifetime_smoke.sh <L1|L2|L3>}"

WS=/workspace
BIN=/build/lifetime/bin
VOC=/build/ORBvoc.txt
EUROC=/datasets/EuRoc
KITTI=/datasets/kitti_dataset/data_odometry_gray/dataset/sequences

require() { [[ -e "$1" ]] || { echo "ERROR: not found: $1" >&2; exit 1; }; }
require "$BIN"
require "$VOC"

OUT="/results/lifetime_${SCEN}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

case "$SCEN" in
  L1)
    require "$EUROC/MH01/mav0/cam0/data"
    CMD=("$BIN/stereo_euroc" "$VOC" "$WS/Examples/Stereo/EuRoC.yaml" "$EUROC/MH01" "$WS/Examples/Stereo/EuRoC_TimeStamps/MH01.txt") ;;
  L2)
    require "$EUROC/MH01/mav0/imu0/data.csv"
    CMD=("$BIN/mono_inertial_euroc" "$VOC" "$WS/Examples/Monocular-Inertial/EuRoC.yaml" "$EUROC/MH01" "$WS/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt") ;;
  L3)
    require "$KITTI/00/image_1"
    CMD=("$BIN/stereo_kitti" "$VOC" "$WS/Examples/Stereo/KITTI00-02.yaml" "$KITTI/00") ;;
  *) echo "ERROR: unknown scenario '$SCEN' (L1|L2|L3)" >&2; exit 1 ;;
esac

export LIFETIME_TRACE_OUT="$OUT/lifetime_trace.csv"

cd "$OUT"
echo ">> lifetime $SCEN -> results/$(basename "$OUT")"

CMD=(timeout -k 60 2700 "${CMD[@]}")
RC=0
if [[ "${HEADLESS:-1}" == "1" ]]; then
  xvfb-run -a "${CMD[@]}" 2>&1 | tee run.log || RC=$?
else
  "${CMD[@]}" 2>&1 | tee run.log || RC=$?
fi
[[ $RC -eq 124 ]] && echo "!! TIMEOUT (exit 124): possible hang" >&2

echo ">> exit=$RC; trace:"
tail -1 "$OUT/lifetime_trace.csv" 2>/dev/null || echo "(no trace file — crash before flush?)"
echo ">> report: python3 benchmark/scripts/lifetime_report.py results/$(basename "$OUT")/lifetime_trace.csv"
