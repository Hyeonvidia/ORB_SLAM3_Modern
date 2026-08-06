#!/usr/bin/env bash
# Batch-evaluate every run directory under a results root in ONE container.
#   ./benchmark/scripts/evaluate_all.sh [results_dir]   (default: ./results)
# Prints CSV lines "mode,ate_rmse,run_dir" — pipe to a file to record.
# Recognizes: euroc_{mono,stereo,mono_inertial,stereo_inertial}_MH01_*,
# kitti_stereo_00_*. Same GT/alignment policy as evaluate.sh.
set -euo pipefail
cd "$(dirname "$0")/../.."
RUNS="$(cd "${1:-results}" && pwd)"
DATASETS="${DATASETS:-$PWD/../Datasets}"
IMG=orbslam3-modern-eval
docker image inspect "$IMG" >/dev/null 2>&1 || \
  docker build -q -t "$IMG" -f benchmark/docker/eval.Dockerfile benchmark/docker >/dev/null

docker run --rm -v "$PWD:/repo:ro" -v "$DATASETS:/datasets:ro" -v "$RUNS:/runs:ro" "$IMG" bash -c '
PY=/repo/benchmark/scripts/to_tum.py
python $PY euroc_csv /repo/evaluation/Ground_truth/EuRoC_left_cam/MH01_GT.txt /tmp/gt_cam.tum >/dev/null
python $PY euroc_csv /datasets/EuRoc/MH01/mav0/state_groundtruth_estimate0/data.csv /tmp/gt_body.tum >/dev/null
for d in /runs/*/; do
  name=$(basename "$d"); traj="${d}CameraTrajectory.txt"
  [ -f "$traj" ] || continue
  case "$name" in
    euroc_mono_MH01_*)            mode=euroc_mono;            gt=/tmp/gt_cam.tum;  flags="-as";;
    euroc_stereo_MH01_*)          mode=euroc_stereo;          gt=/tmp/gt_cam.tum;  flags="-a";;
    euroc_mono_inertial_MH01_*)   mode=euroc_mono_inertial;   gt=/tmp/gt_body.tum; flags="-a";;
    euroc_stereo_inertial_MH01_*) mode=euroc_stereo_inertial; gt=/tmp/gt_body.tum; flags="-a";;
    kitti_stereo_00_*)            mode=kitti_stereo;;
    *) continue;;
  esac
  if [ "$mode" = kitti_stereo ]; then
    rmse=$(evo_ape kitti /datasets/kitti_dataset/data_odometry_poses/dataset/poses/00.txt "$traj" -a 2>/dev/null | awk "/rmse/{print \$2}")
  else
    python $PY traj "$traj" /tmp/est.tum >/dev/null 2>&1 || continue
    rmse=$(evo_ape tum "$gt" /tmp/est.tum $flags 2>/dev/null | awk "/rmse/{print \$2}")
  fi
  [ -n "$rmse" ] && echo "$mode,$rmse,$name"
done'
