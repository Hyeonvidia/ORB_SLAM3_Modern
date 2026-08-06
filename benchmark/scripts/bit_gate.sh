#!/usr/bin/env bash
# P4 bit-identity gate: extraction hashes on 3 EuRoC MH01 images x 3 cases
# must match benchmark/golden/bit_hashes.txt exactly.
#   ./benchmark/scripts/bit_gate.sh            # verify against golden
#   ./benchmark/scripts/bit_gate.sh --record   # (re)record golden hashes
# Hashes are machine+container specific by design (OpenCV SIMD dispatch);
# golden values are only meaningful within the pinned dev image on this host.
set -euo pipefail
cd "$(dirname "$0")/../.."
GOLDEN=benchmark/golden/bit_hashes.txt
MODE="${1:-verify}"

OUT=$(docker compose run --rm dev bash -c '
  set -e
  BIN=/build/cmake/bin/extract_hash
  IMDIR=/datasets/EuRoc/MH01/mav0/cam0/data
  IMGS=$(ls "$IMDIR" | sed -n "1p;1800p;3600p")
  for img in $IMGS; do
    for case in "1000 1.2 8 20 7 0 0" "1000 1.2 8 20 7 0 1000" "5000 1.2 8 20 7 0 1000"; do
      H=$("$BIN" "$IMDIR/$img" $case | head -1)
      echo "$img [$case] $H"
    done
  done' </dev/null)

if [ "$MODE" = "--record" ]; then
  echo "$OUT" > "$GOLDEN"
  echo "recorded $(wc -l < "$GOLDEN" | tr -d ' ') golden hashes:"
  cat "$GOLDEN"
else
  if diff <(echo "$OUT") "$GOLDEN" >/dev/null; then
    echo "BIT GATE PASS ($(wc -l < "$GOLDEN" | tr -d ' ') hashes identical)"
  else
    echo "BIT GATE FAIL:"
    diff <(echo "$OUT") "$GOLDEN" || true
    exit 1
  fi
fi
