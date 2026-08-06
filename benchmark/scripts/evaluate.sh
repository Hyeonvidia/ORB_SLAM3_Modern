#!/usr/bin/env bash
# Compute ATE RMSE for one run: ./evaluate.sh <mode> <trajectory_file>
# Prints "<mode> ATE_RMSE=<m>". Runs evo inside the orbslam3-modern-eval image.
#
# Alignment policy (fixed — same for baseline and regression runs):
#   euroc_mono            evo_ape tum -as   (scale unobservable in pure mono)
#   euroc_stereo          evo_ape tum -a    vs left-cam GT (in-repo)
#   euroc_mono_inertial   evo_ape tum -a    vs mav0 body GT (dataset)
#   euroc_stereo_inertial evo_ape tum -a    vs mav0 body GT (dataset)
#   kitti_stereo          evo_ape kitti -a  vs odometry poses (dataset)
set -euo pipefail
MODE="${1:?usage: evaluate.sh <mode> <trajectory_file> [seq]}"
TRAJ="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"
SEQ="${3:-}"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
DATASETS="${DATASETS:-$REPO/../Datasets}"
IMG=orbslam3-modern-eval

docker image inspect "$IMG" >/dev/null 2>&1 || \
  docker build -q -t "$IMG" -f "$REPO/benchmark/docker/eval.Dockerfile" "$REPO/benchmark/docker" >/dev/null

run_eval() { docker run --rm -v "$REPO:/repo:ro" -v "$DATASETS:/datasets:ro" \
             -v "$(dirname "$TRAJ"):/traj:ro" "$IMG" bash -c "$1"; }

T="/traj/$(basename "$TRAJ")"
PY=/repo/benchmark/scripts/to_tum.py

case "$MODE" in
  euroc_mono|euroc_stereo)
    S="${SEQ:-MH01}"
    FLAGS=$([ "$MODE" = euroc_mono ] && echo "-as" || echo "-a")
    OUT=$(run_eval "python $PY traj $T /tmp/est.tum >/dev/null &&
      python $PY euroc_csv /repo/evaluation/Ground_truth/EuRoC_left_cam/${S}_GT.txt /tmp/gt.tum >/dev/null &&
      evo_ape tum /tmp/gt.tum /tmp/est.tum $FLAGS") ;;
  euroc_mono_inertial|euroc_stereo_inertial)
    S="${SEQ:-MH01}"
    OUT=$(run_eval "python $PY traj $T /tmp/est.tum >/dev/null &&
      python $PY euroc_csv /datasets/EuRoc/${S}/mav0/state_groundtruth_estimate0/data.csv /tmp/gt.tum >/dev/null &&
      evo_ape tum /tmp/gt.tum /tmp/est.tum -a") ;;
  kitti_stereo)
    S="${SEQ:-00}"
    OUT=$(run_eval "evo_ape kitti /datasets/kitti_dataset/data_odometry_poses/dataset/poses/${S}.txt $T -a") ;;
  *) echo "ERROR: unknown mode '$MODE'" >&2; exit 1 ;;
esac

RMSE=$(echo "$OUT" | awk '/rmse/{print $2}')
[ -n "$RMSE" ] || { echo "$OUT" >&2; echo "ERROR: rmse not found" >&2; exit 1; }
echo "$MODE ATE_RMSE=${RMSE}"
