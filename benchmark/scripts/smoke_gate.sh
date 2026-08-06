#!/usr/bin/env bash
# Fast smoke gate (~4 min): one euroc_stereo MH01 run with the current build.
# PASS = run completes with a well-formed trajectory AND ATE RMSE below a
# generous sanity bound (2x the golden baseline median) — catches gross
# breakage during iteration. Precision equivalence is the full gate's job
# (5 modes x N=3 medians at phase completion). See docs/PROJECT_PLAN.md §4.
set -euo pipefail
cd "$(dirname "$0")/../.."

BOUND="${SMOKE_BOUND:-0.084}"   # 2x euroc_stereo baseline median (see BASELINE.md)

docker compose run --rm -e HEADLESS=1 dev docker/scripts/run_slam.sh euroc_stereo MH01 </dev/null >/dev/null
D=$(ls -td results/euroc_stereo_MH01_* | head -1)

LINES=$(awk 'NF==8' "$D/CameraTrajectory.txt" | wc -l | tr -d ' ')
[ "$LINES" -ge 3600 ] || { echo "SMOKE FAIL: trajectory only $LINES well-formed lines (expect ~3682)"; exit 1; }

OUT=$(./benchmark/scripts/evaluate.sh euroc_stereo "$D/CameraTrajectory.txt" MH01)
RMSE=$(echo "$OUT" | awk -F= '{print $2}')
echo "$OUT (bound $BOUND, run $D)"
awk -v r="$RMSE" -v b="$BOUND" 'BEGIN{exit !(r<b)}' \
  && echo "SMOKE PASS" \
  || { echo "SMOKE FAIL: ATE $RMSE >= $BOUND"; exit 1; }
