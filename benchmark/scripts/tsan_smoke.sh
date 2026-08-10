#!/usr/bin/env bash
# P10 TSAN smoke scenarios (docs/P10_RECON.md 3부 §1 table). Container-side:
#   docker compose run --rm -e HEADLESS=1 dev benchmark/scripts/tsan_smoke.sh <T1|T2|T3|T4>
# Runs the /build/tsan binaries DIRECTLY (not via run_slam.sh) so truncated
# EuRoC timestamp files can be passed. NEVER run scenarios concurrently with
# each other or with gate rounds (16-core contention, DIVERGENCES).
#
#   T1  euroc_stereo MH01, ~45 s (first 900 timestamp lines)  -> R4, R1-window
#   T2  kitti_stereo 07, full (1101 frames, 1 loop)           -> R2,R-c,R-d,R-f,GBA
#   T3  euroc_stereo_inertial MH01, ~75 s (first 1500 lines)  -> R3, R5
#   T4  euroc_mono_inertial MH01, ~75 s (reset storm)         -> R-b, reset protocol
#
# Raw TSAN reports land in <rundir>/tsan.<pid>; feed them to tsan_ledger.py.
set -euo pipefail

SCEN="${1:?usage: tsan_smoke.sh <T1|T2|T3|T4>}"

WS=/workspace
BIN=/build/tsan/bin
VOC=/build/ORBvoc.txt
EUROC=/datasets/EuRoc
KITTI=/datasets/kitti_dataset/data_odometry_gray/dataset/sequences

require() { [[ -e "$1" ]] || { echo "ERROR: not found: $1" >&2; exit 1; }; }
require "$BIN"
require "$VOC"

OUT="/results/tsan_${SCEN}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"

truncate_ts() {  # <src> <lines> <dst>
  require "$1"
  head -n "$2" "$1" > "$3"
}

case "$SCEN" in
  T1)
    require "$EUROC/MH01/mav0/cam1/data"
    truncate_ts "$WS/Examples/Stereo/EuRoC_TimeStamps/MH01.txt" 900 "$OUT/MH01_ts.txt"
    CMD=("$BIN/stereo_euroc" "$VOC" "$WS/Examples/Stereo/EuRoC.yaml" "$EUROC/MH01" "$OUT/MH01_ts.txt") ;;
  T2)
    require "$KITTI/07/image_1"
    CMD=("$BIN/stereo_kitti" "$VOC" "$WS/Examples/Stereo/KITTI04-12.yaml" "$KITTI/07") ;;
  T3)
    require "$EUROC/MH01/mav0/imu0/data.csv"
    truncate_ts "$WS/Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt" 1500 "$OUT/MH01_ts.txt"
    CMD=("$BIN/stereo_inertial_euroc" "$VOC" "$WS/Examples/Stereo-Inertial/EuRoC.yaml" "$EUROC/MH01" "$OUT/MH01_ts.txt") ;;
  T4)
    require "$EUROC/MH01/mav0/imu0/data.csv"
    truncate_ts "$WS/Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt" 1500 "$OUT/MH01_ts.txt"
    CMD=("$BIN/mono_inertial_euroc" "$VOC" "$WS/Examples/Monocular-Inertial/EuRoC.yaml" "$EUROC/MH01" "$OUT/MH01_ts.txt") ;;
  *) echo "ERROR: unknown scenario '$SCEN' (T1|T2|T3|T4)" >&2; exit 1 ;;
esac

# called_from_lib suppressions MUST each match exactly one library --
# a multi-library match (e.g. 'libpango*') is a FATAL error in gcc-11 TSAN,
# and with exitcode=0 it dies silently (P10-0 attempt-2 postmortem).
# Generate one line per actually-present library at runtime.
SUPP="$OUT/suppressions.txt"
{
  for so in /usr/local/lib/libpango*.so; do
    [[ -e "$so" ]] && echo "called_from_lib:$(basename "$so")"
  done
  ldd "$BIN/stereo_euroc" | grep -oE 'libopencv[a-z_0-9]*\.so' | sort -u | \
    while read -r so; do echo "called_from_lib:$so"; done
} > "$SUPP"

export TSAN_OPTIONS="history_size=7 second_deadlock_stack=1 exitcode=0 log_path=$OUT/tsan suppressions=$SUPP"
# Kill the uninstrumented-OpenCV worker-pool noise (recon §1 mitigation);
# timing distortion is irrelevant in TSAN runs.
export OPENCV_FOR_THREADS_NUM=1

cd "$OUT"
echo ">> TSAN $SCEN -> results/$(basename "$OUT")"
echo ">> TSAN_OPTIONS=$TSAN_OPTIONS"

# coreutils timeout: 45 min hard bound per run (TSAN is 5-20x slower).
# Teardown crash is tolerated (result-file policy); judge by tsan.* output.
CMD=(timeout -k 60 2700 "${CMD[@]}")
RC=0
if [[ "${HEADLESS:-1}" == "1" ]]; then
  xvfb-run -a "${CMD[@]}" 2>&1 | tee run.log || RC=$?
else
  "${CMD[@]}" 2>&1 | tee run.log || RC=$?
fi
[[ $RC -eq 124 ]] && echo "!! TIMEOUT (exit 124): possible hang" >&2

echo ">> exit=$RC; TSAN report files:"
ls -l "$OUT"/tsan.* 2>/dev/null || echo "(no tsan.* files — zero reports?)"
echo ">> ledger: python3 benchmark/scripts/tsan_ledger.py results/$(basename "$OUT")"
