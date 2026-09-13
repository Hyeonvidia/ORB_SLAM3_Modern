#!/usr/bin/env bash
# Host wrapper: run one SLAM mode on one dataset sequence in the dev container,
# then print (and evaluate) the result.
#
#   ./run.sh <mode> <sequence> [--headless|--xquartz] [--timeout <sec>] [--no-eval]
#   ./run.sh --list
#
# modes: euroc_mono | euroc_stereo | euroc_mono_inertial | euroc_stereo_inertial
#        | kitti_mono | kitti_stereo
# examples:
#   ./run.sh euroc_stereo MH01                 # GUI: viewer window in XQuartz
#   ./run.sh euroc_mono_inertial V102 --headless
#   ./run.sh kitti_stereo 07 --timeout 300     # bounded run (exit 124 = timeout)
#
# GUI default shows a single "ORB_SLAM3_Modern viewer" window in XQuartz:
# a nested X server (Xephyr) drawn as plain 2D, with the SLAM GUI rendered
# into it in-container via llvmpipe. XQuartz's own GLX is bypassed — it
# currently renders nothing for container clients (even indirect glxgears
# is blank; measured 2026-09-13). --vnc uses macOS Screen Sharing instead.
# kitti_mono saves only KeyFrameTrajectory.txt (upstream behavior), no ATE.
set -uo pipefail
cd "$(dirname "$0")"

DATASETS="${DATASETS:-$(pwd)/../Datasets}"
EUROC="$DATASETS/EuRoc"
KITTI="$DATASETS/kitti_dataset/data_odometry_gray/dataset/sequences"
MODES="euroc_mono euroc_stereo euroc_mono_inertial euroc_stereo_inertial kitti_mono kitti_stereo"
VNC_PORT="${VNC_PORT:-5901}"

usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 1; }

list_sequences() {
  echo "modes : $MODES"
  echo -n "EuRoC : "
  for d in "$EUROC"/*/; do
    s=$(basename "$d")
    [[ -f "Examples/Monocular/EuRoC_TimeStamps/$s.txt" ]] && printf '%s ' "$s"
  done; echo
  echo -n "KITTI : "
  for d in "$KITTI"/*/; do printf '%s ' "$(basename "$d")"; done; echo
  exit 0
}

[[ "${1:-}" == "--list" ]] && list_sequences
MODE="${1:-}"; SEQ="${2:-}"
[[ -n "$MODE" && -n "$SEQ" ]] || usage
shift 2

GUI=xquartz; TIMEOUT=""; EVAL=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) GUI=none ;;
    --xquartz)  GUI=xquartz ;;
    --vnc)      GUI=vnc ;;
    --timeout)  TIMEOUT="${2:?--timeout needs seconds}"; shift ;;
    --no-eval)  EVAL=0 ;;
    *) echo "unknown option: $1" >&2; usage ;;
  esac
  shift
done

# --- validate mode + sequence on the host, before touching Docker ----------
case " $MODES " in *" $MODE "*) ;; *) echo "ERROR: unknown mode '$MODE'" >&2; usage ;; esac
case "$MODE" in
  euroc_*)
    [[ -d "$EUROC/$SEQ" ]] || { echo "ERROR: no EuRoC sequence $EUROC/$SEQ (./run.sh --list)" >&2; exit 1; }
    [[ -f "Examples/Monocular/EuRoC_TimeStamps/$SEQ.txt" ]] || { echo "ERROR: no timestamps for '$SEQ'" >&2; exit 1; } ;;
  kitti_*)
    [[ -d "$KITTI/$SEQ" ]] || { echo "ERROR: no KITTI sequence $KITTI/$SEQ (./run.sh --list)" >&2; exit 1; }
    [[ "$SEQ" =~ ^(1[3-9]|2[01])$ ]] && echo "WARN: KITTI $SEQ has no dedicated yaml; using KITTI04-12.yaml" >&2 ;;
esac

# --- per-GUI-mode setup ------------------------------------------------------
RUNFLAGS=(); ENVS=()
case "$GUI" in
  none)
    ENVS+=(-e HEADLESS=1) ;;
  vnc)
    ENVS+=(-e VNC=1); RUNFLAGS+=(-p "$VNC_PORT:5900")
    # Open macOS Screen Sharing as soon as the in-container VNC server is up.
    ( for _ in $(seq 1 90); do
        nc -z 127.0.0.1 "$VNC_PORT" 2>/dev/null && { sleep 1; open "vnc://127.0.0.1:$VNC_PORT"; exit; }
        sleep 1
      done ) & WATCHER=$! ;;
  xquartz)
    if ! lsof -nP -iTCP:6000 -sTCP:LISTEN >/dev/null 2>&1; then
      echo ">> starting XQuartz..."
      open -a XQuartz || { echo "ERROR: XQuartz not installed (brew install --cask xquartz)" >&2; exit 1; }
      for _ in $(seq 1 20); do
        lsof -nP -iTCP:6000 -sTCP:LISTEN >/dev/null 2>&1 && break; sleep 1
      done
      lsof -nP -iTCP:6000 -sTCP:LISTEN >/dev/null 2>&1 || {
        echo "ERROR: XQuartz did not open TCP 6000. Enable 'Allow connections from network clients'" >&2
        echo "       in XQuartz > Settings > Security, then restart XQuartz." >&2; exit 1; }
    fi
    LDPY="$(launchctl getenv DISPLAY || true)"
    [[ -n "$LDPY" ]] && DISPLAY="$LDPY" /opt/X11/bin/xhost +localhost >/dev/null 2>&1 || true
    ENVS+=(-e XEPHYR=1) ;;
esac
[[ -n "$TIMEOUT" ]] && ENVS+=(-e RUN_TIMEOUT="$TIMEOUT")

# --- run ---------------------------------------------------------------------
START_EPOCH=$(date +%s)
docker compose run --rm ${RUNFLAGS[@]+"${RUNFLAGS[@]}"} ${ENVS[@]+"${ENVS[@]}"} dev docker/scripts/run_slam.sh "$MODE" "$SEQ"
RC=$?
[[ "$RC" == 124 ]] && echo ">> run stopped by --timeout $TIMEOUT (exit 124)"
[[ -n "${WATCHER:-}" ]] && kill "$WATCHER" 2>/dev/null

# --- locate this invocation's result dir (freshness-guarded) -----------------
D=$(ls -td results/"${MODE}_${SEQ}"_* 2>/dev/null | head -1)
[[ -n "$D" ]] || { echo "ERROR: no result dir produced" >&2; exit 1; }
DIR_EPOCH=$(stat -f %B "$D" 2>/dev/null || stat -c %Y "$D")
[[ "$DIR_EPOCH" -ge "$START_EPOCH" ]] || { echo "ERROR: no fresh result (latest: $D)" >&2; exit 1; }
echo ">> results: $D"

# --- ATE (skipped for kitti_mono / timeout-killed / --no-eval) ----------------
case "$MODE" in
  euroc_mono|euroc_mono_inertial) TRAJ="$D/KeyFrameTrajectory.txt" ;;
  kitti_mono)                     TRAJ="" ;;
  *)                              TRAJ="$D/CameraTrajectory.txt" ;;
esac
if [[ "$EVAL" == 1 && -n "$TRAJ" && -s "$TRAJ" ]]; then
  ./benchmark/scripts/evaluate.sh "$MODE" "$TRAJ" "$SEQ" \
    || echo "WARN: ATE evaluation failed (missing ground truth for $SEQ?)" >&2
elif [[ -n "$TRAJ" && ! -s "$TRAJ" ]]; then
  echo ">> no trajectory saved (timeout/abort) — ATE skipped"
fi
