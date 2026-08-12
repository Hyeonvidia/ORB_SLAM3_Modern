#!/usr/bin/env bash
# P12-G0-1 timing-profile scenarios. Container-side:
#   docker compose run --rm -e HEADLESS=1 dev benchmark/scripts/times_run.sh <P1|P2|P3>
#
#   P1  euroc_stereo MH01          -> vision-only LBA share
#   P2  euroc_mono_inertial MH01   -> inertial LBA/IMU-init share
#   P3  kitti_stereo 00            -> loop closing + essential graph + GBA
#
# PrintTimeStats output (per-stage means/std for Tracking, LocalMapping,
# LoopClosing) lands in run.log at Shutdown. Run on a QUIET machine —
# the numbers are wall-clock. Diagnostic-only.
set -euo pipefail

SCEN="${1:?usage: times_run.sh <P1|P2|P3>}"

WS=/workspace
BIN=/build/times/bin
VOC=/build/ORBvoc.txt
EUROC=/datasets/EuRoc
KITTI=/datasets/kitti_dataset/data_odometry_gray/dataset/sequences

require() { [[ -e "$1" ]] || { echo "ERROR: not found: $1" >&2; exit 1; }; }
require "$BIN"
require "$VOC"

OUT="/results/times_${SCEN}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

case "$SCEN" in
  P1)
    require "$EUROC/MH01/mav0/cam1/data"
    CMD=("$BIN/stereo_euroc" "$VOC" "$WS/Examples/Stereo/EuRoC.yaml" "$EUROC/MH01" "$WS/Examples/Stereo/EuRoC_TimeStamps/MH01.txt") ;;
  P2)
    require "$EUROC/MH01/mav0/imu0/data.csv"
    CMD=("$BIN/mono_inertial_euroc" "$VOC" "$WS/Examples/Monocular-Inertial/EuRoC.yaml" "$EUROC/MH01" "$WS/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt") ;;
  P3)
    require "$KITTI/00/image_1"
    CMD=("$BIN/stereo_kitti" "$VOC" "$WS/Examples/Stereo/KITTI00-02.yaml" "$KITTI/00") ;;
  *) echo "ERROR: unknown scenario '$SCEN' (P1|P2|P3)" >&2; exit 1 ;;
esac

cd "$OUT"
echo ">> times $SCEN -> results/$(basename "$OUT")"

CMD=(timeout -k 60 2700 "${CMD[@]}")
RC=0
if [[ "${HEADLESS:-1}" == "1" ]]; then
  xvfb-run -a "${CMD[@]}" 2>&1 | tee run.log || RC=$?
else
  "${CMD[@]}" 2>&1 | tee run.log || RC=$?
fi
[[ $RC -eq 124 ]] && echo "!! TIMEOUT (exit 124): possible hang" >&2

echo ">> exit=$RC; PrintTimeStats tail:"
grep -A200 "Time Stats" run.log | tail -60 || echo "(no time stats in run.log?)"
