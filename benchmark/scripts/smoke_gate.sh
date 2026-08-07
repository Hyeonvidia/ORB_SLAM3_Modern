#!/usr/bin/env bash
# Fast smoke gate (~4 min): one euroc_stereo MH01 run with the current build.
# PASS = run completes with a well-formed trajectory AND ATE RMSE below a
# generous sanity bound (2x the golden baseline median) — catches gross
# breakage during iteration. Precision equivalence is the full gate's job
# (5 modes x N=3 medians at phase completion). See docs/PROJECT_PLAN.md §4.
set -euo pipefail
cd "$(dirname "$0")/../.."

BOUND="${SMOKE_BOUND:-0.0758}"  # 2x euroc_stereo N=5 baseline MEDIAN 0.0379 (baseline_ate_distribution.csv)

# Upstream crashes in teardown AFTER saving trajectories (known SIGABRT/SIGSEGV);
# judge by the result files, never by the process exit code.
START_EPOCH=$(date +%s)
docker compose run --rm -e HEADLESS=1 dev docker/scripts/run_slam.sh euroc_stereo MH01 </dev/null >/dev/null || true
D=$(ls -td results/euroc_stereo_MH01_* | head -1)
# Freshness guard: judge only a run created by THIS invocation (checkpoint
# review: a totally failed run must not silently re-judge yesterday's dir).
DIR_EPOCH=$(stat -f %B "$D" 2>/dev/null || stat -c %Y "$D")
[ "$DIR_EPOCH" -ge "$START_EPOCH" ] || { echo "SMOKE FAIL: no fresh run produced (latest: $D)"; exit 1; }

LINES=$(awk 'NF==8' "$D/CameraTrajectory.txt" | wc -l | tr -d ' ')
[ "$LINES" -ge 3600 ] || { echo "SMOKE FAIL: trajectory only $LINES well-formed lines (expect ~3682)"; exit 1; }

OUT=$(./benchmark/scripts/evaluate.sh euroc_stereo "$D/CameraTrajectory.txt" MH01)
RMSE=$(echo "$OUT" | awk -F= '{print $2}')
echo "$OUT (bound $BOUND, run $D)"
awk -v r="$RMSE" -v b="$BOUND" 'BEGIN{exit !(r<b)}' \
  && echo "SMOKE PASS" \
  || { echo "SMOKE FAIL: ATE $RMSE >= $BOUND"; exit 1; }
