#!/usr/bin/env bash
# Full gate v2.1 (docs/phase_reports/P2.md): interleaved paired original-vs-new
# runs, judged by exact rank-sum. Run on a RESTED machine (daytime policy).
#
#   ./benchmark/scripts/full_gate_paired.sh [rounds]   (default 4 — the minimum with formal statistical power)
#
# Requires the sibling golden harness at ../orb_slam3_docker (original image).
# Writes verdict + per-run CSV to benchmark/gate_runs/<timestamp>/.
set -uo pipefail
cd "$(dirname "$0")/../.."
ROUNDS="${1:-4}"
# Statistical power: with R rounds/side the exact rank-sum minimum p is
# 1/C(2R,R); p<0.05 needs R>=4 (checkpoint review finding). Fewer rounds
# produce an UNDERPOWERED verdict, never a formal PASS.
ORIG_DIR="$(cd ../orb_slam3_docker && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="benchmark/gate_runs/$STAMP"
mkdir -p "$OUT"
CUT_UTC="$(date -u +%Y%m%d_%H%M%S)"
echo "gate start $STAMP (UTC cut $CUT_UTC), rounds=$ROUNDS" | tee "$OUT/gate.log"

run_pair() { # mode seq
  local m="$1" s="$2"
  echo "== $m $s ORIG $(date +%H:%M:%S)" | tee -a "$OUT/gate.log"
  (cd "$ORIG_DIR" && docker compose run --rm -e HEADLESS=1 orbslam3 /opt/scripts/run_slam.sh "$m" "$s" >/dev/null 2>&1 </dev/null)
  echo "== $m $s NEW  $(date +%H:%M:%S)" | tee -a "$OUT/gate.log"
  docker compose run --rm -e HEADLESS=1 dev docker/scripts/run_slam.sh "$m" "$s" >/dev/null 2>&1 </dev/null
}

for r in $(seq 1 "$ROUNDS"); do
  for m in euroc_mono euroc_stereo euroc_mono_inertial euroc_stereo_inertial; do
    run_pair "$m" MH01
  done
  run_pair kitti_stereo 00
done

./benchmark/scripts/evaluate_all.sh "$ORIG_DIR/results" > "$OUT/orig.csv"
./benchmark/scripts/evaluate_all.sh results > "$OUT/new.csv"

python3 - "$OUT" "$CUT_UTC" <<'EOF'
import csv, statistics, itertools, sys, pathlib
out, cut = pathlib.Path(sys.argv[1]), sys.argv[2]
cut_date, cut_ts = cut.split('_')
def load(p):
    d={}
    for row in csv.reader(open(p)):
        if len(row)!=3: continue
        mode,rmse,name=row
        date=name.rsplit('_',2)[-2]; ts=name.rsplit('_',1)[-1]
        if date>cut_date or (date==cut_date and ts>=cut_ts):
            d.setdefault(mode,[]).append(float(rmse))
    return d
orig,new = load(out/"orig.csv"), load(out/"new.csv")
def p_worse(a,b):
    allv=a+b; nb=len(b)
    obs=sum(sorted(allv).index(v)+1 for v in b)
    tot=cnt=0
    for c in itertools.combinations(range(len(allv)),nb):
        rs=sum(sorted(allv).index(allv[i])+1 for i in c); tot+=1
        if rs>=obs: cnt+=1
    return cnt/tot
lines=[f"{'mode':24s} {'orig_med(n)':>13s} {'new_med(n)':>13s} {'d%':>7s} {'p':>6s} verdict"]
ok=True
for m in sorted(set(orig)|set(new)):
    a,b=orig.get(m,[]),new.get(m,[])
    if not a or not b:
        lines.append(f"{m:24s} MISSING RUNS"); ok=False; continue
    am,bm=statistics.median(a),statistics.median(b)
    d=(bm-am)/am*100; p=p_worse(a,b)
    fail = p<0.05 and d>10
    ok &= not fail
    lines.append(f"{m:24s} {am:>8.4f}({len(a)}) {bm:>8.4f}({len(b)}) {d:>+6.1f}% {p:>6.3f} {'FAIL' if fail else 'PASS'}")
lines.append("")
import math
minp = 1.0/math.comb(2*min(len(v) for d in (orig,new) for v in d.values() if v), min(len(v) for d in (orig,new) for v in d.values() if v)) if orig and new else 1.0
if minp >= 0.05:
    lines.append(f"FULL GATE: UNDERPOWERED (min achievable p={minp:.3f} >= 0.05) — run >=4 rounds for a formal verdict")
else:
    lines.append("FULL GATE: " + ("PASS" if ok else "FAIL — bisect-replicate before acting (gate v2.1)"))
report="\n".join(lines)
(out/"verdict.txt").write_text(report+"\n")
print(report)
EOF
