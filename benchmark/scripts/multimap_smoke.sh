#!/usr/bin/env bash
# P11-F3 multimap smoke: MH01 twice, single-session STEREO via the
# example's num_seq support. Stereo deliberately: non-inertial detection has
# no IMU/BA2 entry gates (12-KF minimum only) and mono-inertial's init reset
# storm wipes young maps+KFDB before merge windows open (measured: 24 resets,
# 0 merge seeds). Exercises multi-map creation + visual cross-map merge
# (the path the standard gates never reach — DIVERGENCES #21/L1/L2/D5
# territory). Asserts via the [loopclosing] channel trace that at least one
# merge was consumed. Runs with whatever yaml is passed (default = gate yaml
# = level-0); pass a FixLevel yaml copy to screen the ON config.
#   ./benchmark/scripts/multimap_smoke.sh [settings_yaml_in_container]
set -uo pipefail
cd "$(dirname "$0")/../.."
YAML="${1:-/workspace/Examples/Stereo/EuRoC.yaml}"
docker compose run --rm -e HEADLESS=1 -e RUN_TIMEOUT=900 dev bash -c "
  set -uo pipefail
  OUT=/results/multimap_\$(date +%Y%m%d_%H%M%S)
  mkdir -p \"\$OUT\"; cd \"\$OUT\"
  timeout 900 xvfb-run -a /build/cmake/bin/stereo_euroc /build/ORBvoc.txt $YAML \
    /datasets/EuRoc/MH01 /workspace/Examples/Stereo/EuRoC_TimeStamps/MH01.txt \
    /datasets/EuRoc/MH01 /workspace/Examples/Stereo/EuRoC_TimeStamps/MH01.txt \
    2>&1 | tee run.log >/dev/null
  echo \"run dir: \$OUT\"
" </dev/null >/dev/null 2>&1
D=$(ls -td results/multimap_* | head -1)
MERGES=$(grep -c "\[loopclosing\] merge wipe-consume" "$D/run.log" 2>/dev/null || true)
MAPS=$(grep -cE "Creation of new map|CreateMapInAtlas" "$D/run.log" 2>/dev/null || true)
echo "multimap_smoke: dir=$(basename "$D") maps_created=$MAPS merge_markers=$MERGES"
grep -E "\[loopclosing\] (merge|loop)" "$D/run.log" | tail -6
if [ "${MERGES:-0}" -ge 1 ]; then echo "MULTIMAP SMOKE PASS"; else echo "MULTIMAP SMOKE FAIL (no merge observed)"; exit 1; fi
