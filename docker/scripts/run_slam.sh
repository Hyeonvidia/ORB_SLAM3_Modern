#!/usr/bin/env bash
# Container-side dispatcher (same interface as orb_slam3_docker):
#   run_slam.sh <mode> <sequence>     modes: euroc_mono|euroc_stereo|
#   euroc_mono_inertial|euroc_stereo_inertial|kitti_mono|kitti_stereo
# HEADLESS=1 wraps with xvfb-run. Trajectories land in /results/<run>/.
set -euo pipefail

MODE="${1:?usage: run_slam.sh <mode> <sequence>}"
SEQ="${2:?usage: run_slam.sh <mode> <sequence>}"

WS=/workspace
BIN=/build/cmake/bin
VOC=/build/ORBvoc.txt
EUROC=/datasets/EuRoc
KITTI=/datasets/kitti_dataset/data_odometry_gray/dataset/sequences

kitti_yaml() {
  case "$1" in
    00|01|02) echo "KITTI00-02.yaml" ;;
    03)       echo "KITTI03.yaml" ;;
    *)        echo "KITTI04-12.yaml" ;;
  esac
}
require() { [[ -e "$1" ]] || { echo "ERROR: not found: $1" >&2; exit 1; }; }

case "$MODE" in
  euroc_mono)
    EX="$WS/Examples/Monocular"
    require "$EUROC/$SEQ/mav0/cam0/data"
    CMD=("$BIN/mono_euroc" "$VOC" "$EX/EuRoC.yaml" "$EUROC/$SEQ" "$EX/EuRoC_TimeStamps/$SEQ.txt") ;;
  euroc_stereo)
    EX="$WS/Examples/Stereo"
    require "$EUROC/$SEQ/mav0/cam1/data"
    CMD=("$BIN/stereo_euroc" "$VOC" "$EX/EuRoC.yaml" "$EUROC/$SEQ" "$EX/EuRoC_TimeStamps/$SEQ.txt") ;;
  euroc_mono_inertial)
    EX="$WS/Examples/Monocular-Inertial"
    require "$EUROC/$SEQ/mav0/imu0/data.csv"
    CMD=("$BIN/mono_inertial_euroc" "$VOC" "$EX/EuRoC.yaml" "$EUROC/$SEQ" "$EX/EuRoC_TimeStamps/$SEQ.txt") ;;
  euroc_stereo_inertial)
    EX="$WS/Examples/Stereo-Inertial"
    require "$EUROC/$SEQ/mav0/imu0/data.csv"
    CMD=("$BIN/stereo_inertial_euroc" "$VOC" "$EX/EuRoC.yaml" "$EUROC/$SEQ" "$EX/EuRoC_TimeStamps/$SEQ.txt") ;;
  kitti_mono)
    EX="$WS/Examples/Monocular"
    YAML="$(kitti_yaml "$SEQ")"
    require "$KITTI/$SEQ/image_0"
    CMD=("$BIN/mono_kitti" "$VOC" "$EX/$YAML" "$KITTI/$SEQ") ;;
  kitti_stereo)
    EX="$WS/Examples/Stereo"
    YAML="$(kitti_yaml "$SEQ")"
    require "$KITTI/$SEQ/image_1"
    CMD=("$BIN/stereo_kitti" "$VOC" "$EX/$YAML" "$KITTI/$SEQ") ;;
  *) echo "ERROR: unknown mode '$MODE'" >&2; exit 1 ;;
esac

OUT="/results/${MODE}_${SEQ}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"; cd "$OUT"
echo ">> $MODE $SEQ -> results/$(basename "$OUT")"
# P10-0: optional bounded-time guard (exit 124 = hang, distinct from the
# tolerated teardown crash). Unset = behavior unchanged.
if [[ -n "${RUN_TIMEOUT:-}" ]]; then
  CMD=(timeout "$RUN_TIMEOUT" "${CMD[@]}")
fi
# Lay out the two SLAM windows on the :99 display once both appear: the
# Pangolin Map Viewer is the main window (fills the screen), the OpenCV
# Current Frame sits as a small overlay in the top-right corner (kept out
# of the Map Viewer's left control panel). Runs in the background against a
# window manager (openbox) so the user can still drag/resize afterwards.
arrange_windows_99() {
  # Cosmetic, best-effort: never let a transient xdotool non-zero (empty
  # search before the windows exist, under the script's pipefail) abort it.
  set +e +o pipefail
  export DISPLAY=:99
  local geo sw sh mv cf
  geo="$(xdotool getdisplaygeometry 2>/dev/null)" || return
  sw="${geo% *}"; sh="${geo#* }"
  for _ in $(seq 1 90); do
    mv="$(xdotool search --name '^ORB-SLAM3: Map Viewer$' 2>/dev/null | head -1)"
    cf="$(xdotool search --name '^ORB-SLAM3: Current Frame$' 2>/dev/null | head -1)"
    [[ -n "$mv" && -n "$cf" ]] && break
    sleep 1
  done
  [[ -n "$mv" ]] || return
  # Let openbox finish its own initial placement first, otherwise it overrides
  # ours on the map event (the moves would silently not stick).
  sleep 3
  # Map Viewer = main window, fills the screen.
  xdotool windowsize "$mv" "$sw" "$sh"; xdotool windowmove "$mv" 0 0
  if [[ -n "$cf" ]]; then
    # Current Frame = small overlay docked top-right (~30% width, aspect kept).
    local W=0 H=0; eval "$(xdotool getwindowgeometry --shell "$cf" 2>/dev/null | grep -E '^(WIDTH|HEIGHT)=')"
    local tw=$(( sw * 30 / 100 )) th=360
    [[ "$W" -gt 0 && "$H" -gt 0 ]] && th=$(( tw * H / W ))
    xdotool windowsize "$cf" "$tw" "$th"
    xdotool windowmove "$cf" "$(( sw - tw - 12 ))" 12
    xdotool windowraise "$cf"
  fi
}

if [[ "${XEPHYR:-0}" == "1" || "${VNC:-0}" == "1" ]]; then
  if [[ "${XEPHYR:-0}" == "1" ]]; then
    # XQuartz-native viewer: Xephyr is a nested X server shown as a plain 2D
    # window on the host XQuartz (DISPLAY from compose); the SLAM binaries
    # render GL into it via llvmpipe on :99. XQuartz's own GLX — which renders
    # nothing for container clients — is never involved.
    Xephyr :99 -screen "${VNC_GEOMETRY:-1600x1000}" -title "ORB_SLAM3_Modern viewer" &
    DISP_PID=$!
    for _ in $(seq 1 50); do [[ -S /tmp/.X11-unix/X99 ]] && break; sleep 0.1; done
    [[ -S /tmp/.X11-unix/X99 ]] || { echo "ERROR: Xephyr failed to start (is XQuartz reachable?)" >&2; exit 1; }
  else
    # VNC path: render into Xvfb via llvmpipe, export over VNC/Screen Sharing.
    Xvfb :99 -screen 0 "${VNC_GEOMETRY:-1600x1000}x24" &
    DISP_PID=$!
    for _ in $(seq 1 50); do [[ -S /tmp/.X11-unix/X99 ]] && break; sleep 0.1; done
    x11vnc -display :99 -rfbport 5900 -forever -shared -nopw -quiet -bg >/dev/null 2>&1
  fi
  DISPLAY=:99 openbox &
  OPENBOX_PID=$!
  arrange_windows_99 &
  DISPLAY=:99 "${CMD[@]}" 2>&1 | tee run.log
  kill "$OPENBOX_PID" "$DISP_PID" 2>/dev/null || true
elif [[ "${HEADLESS:-0}" == "1" ]]; then
  xvfb-run -a "${CMD[@]}" 2>&1 | tee run.log
else
  "${CMD[@]}" 2>&1 | tee run.log
fi
ls -l "$OUT"
