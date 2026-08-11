#!/usr/bin/env bash
# GUI SLAM viewer launcher (macOS + XQuartz + docker compose).
#
#   ./view_slam.sh                      # interactive: pick mode + sequence
#   ./view_slam.sh euroc_stereo MH01    # direct
#   ./view_slam.sh kitti_stereo 00
#   ./view_slam.sh --list               # show available modes/sequences
#
# Runs docker/scripts/run_slam.sh WITHOUT HEADLESS so the Pangolin viewer
# and OpenCV frame windows appear via XQuartz. Handles first-time XQuartz
# setup (network clients + xhost) automatically.
#
# NOTE: GUI runs are for eyeballing only — the viewer perturbs timing, so
# anything ATE-judged (gates, probes) must use the HEADLESS=1 path instead.
# Policy reminder: never set LIBGL_ALWAYS_INDIRECT (breaks Pangolin on
# macOS Docker — see docker-compose.yml header).
set -uo pipefail
cd "$(dirname "$0")"

DATASETS="$(cd .. && pwd)/Datasets"
EUROC_DIR="$DATASETS/EuRoc"
KITTI_DIR="$DATASETS/kitti_dataset/data_odometry_gray/dataset/sequences"
MODES=(euroc_mono euroc_stereo euroc_mono_inertial euroc_stereo_inertial kitti_mono kitti_stereo)

list_euroc() { [ -d "$EUROC_DIR" ] && ls "$EUROC_DIR" 2>/dev/null | sort || true; }
list_kitti() { [ -d "$KITTI_DIR" ] && ls "$KITTI_DIR" 2>/dev/null | sort || true; }

if [[ "${1:-}" == "--list" ]]; then
  echo "modes : ${MODES[*]}"
  echo "EuRoC : $(list_euroc | tr '\n' ' ')"
  echo "KITTI : $(list_kitti | tr '\n' ' ')"
  exit 0
fi

MODE="${1:-}"
SEQ="${2:-}"

# ---- interactive selection when args are missing ----------------------------
if [[ -z "$MODE" ]]; then
  echo "== mode 선택 =="
  select m in "${MODES[@]}"; do [[ -n "$m" ]] && MODE="$m" && break; done
fi
case " ${MODES[*]} " in
  *" $MODE "*) ;;
  *) echo "ERROR: unknown mode '$MODE' (modes: ${MODES[*]})" >&2; exit 1 ;;
esac

if [[ -z "$SEQ" ]]; then
  if [[ "$MODE" == kitti_* ]]; then CANDS=($(list_kitti)); else CANDS=($(list_euroc)); fi
  [[ ${#CANDS[@]} -eq 0 ]] && { echo "ERROR: no sequences under $DATASETS" >&2; exit 1; }
  if [[ ${#CANDS[@]} -eq 1 ]]; then SEQ="${CANDS[0]}"; echo ">> sequence: $SEQ (유일)"
  else
    echo "== sequence 선택 =="
    select s in "${CANDS[@]}"; do [[ -n "$s" ]] && SEQ="$s" && break; done
  fi
fi

# ---- dataset sanity (host-side mirror of run_slam.sh require()) -------------
if [[ "$MODE" == kitti_* ]]; then REQ="$KITTI_DIR/$SEQ"; else REQ="$EUROC_DIR/$SEQ"; fi
[[ -d "$REQ" ]] || { echo "ERROR: dataset not found: $REQ" >&2; exit 1; }

# ---- XQuartz ----------------------------------------------------------------
if [[ "$(uname)" == "Darwin" ]]; then
  if ! open -Ra XQuartz 2>/dev/null; then
    echo "ERROR: XQuartz가 없습니다. 설치: brew install --cask xquartz (설치 후 재로그인)" >&2
    exit 1
  fi
  # network clients must be allowed (nolisten_tcp=0); fix + restart if not
  for DOM in org.xquartz.X11 org.macosforge.xquartz.X11; do
    if defaults read "$DOM" nolisten_tcp >/dev/null 2>&1; then
      if [[ "$(defaults read "$DOM" nolisten_tcp)" == "1" ]]; then
        echo ">> XQuartz 'Allow network clients' 활성화 ($DOM) 후 재시작"
        defaults write "$DOM" nolisten_tcp -bool false
        osascript -e 'quit app "XQuartz"' 2>/dev/null; sleep 1
      fi
    fi
  done
  open -a XQuartz
  # wait until the X server answers, then allow local clients
  XH_OK=0
  for _ in $(seq 1 20); do
    if DISPLAY="${DISPLAY:-:0}" xhost +localhost >/dev/null 2>&1; then XH_OK=1; break; fi
    sleep 0.5
  done
  [[ $XH_OK -eq 1 ]] || { echo "ERROR: X 서버 응답 없음 — XQuartz를 열고 다시 실행하세요" >&2; exit 1; }
fi

if [[ -n "${LIBGL_ALWAYS_INDIRECT:-}" ]]; then
  echo ">> WARNING: LIBGL_ALWAYS_INDIRECT 해제 (macOS Docker에서 Pangolin 파손 금기)"
  unset LIBGL_ALWAYS_INDIRECT
fi

# ---- binaries preflight (build once if missing) -----------------------------
if ! docker compose run --rm dev bash -c 'test -x /build/cmake/bin/mono_euroc' </dev/null >/dev/null 2>&1; then
  echo ">> 바이너리 없음 — 컨테이너 빌드 실행 (최초 1회, 수 분 소요)"
  docker compose run --rm dev docker/scripts/container_build.sh </dev/null || exit 1
fi

# ---- run (no HEADLESS => GUI) -----------------------------------------------
echo ">> $MODE $SEQ — Pangolin/OpenCV 창이 XQuartz로 뜹니다 (종료: 뷰어 닫기/Ctrl-C)"
docker compose run --rm dev docker/scripts/run_slam.sh "$MODE" "$SEQ"
echo ">> 궤적: results/ 최신 디렉터리 (관찰용 런 — 게이트 수치는 HEADLESS=1로 측정)"
