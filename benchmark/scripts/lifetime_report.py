#!/usr/bin/env python3
"""P12-L0-b lifetime distribution report (docs/P12_L0_DESIGN.md).

Aggregates lifetime_trace.csv (site,kind,id,seq_delta,ms_delta) into a
per-site bad-then-read distribution — the per-site expiry-decision
evidence:

  site                      n_events  n_objs  seq_delta(min/med/max)  ms_delta(min/med/max)

Sites absent from the trace produced ZERO bad-object reads in that run —
list them via --expected <file> (one site label per line) to print the
zero-event sites explicitly (a site with empirically zero bad reads +
a static reachability argument is an expiry-safe candidate; a heavy tail
marks a load-bearing tombstone site).

Usage:
    lifetime_report.py <lifetime_trace.csv> [more ...] [--expected sites.txt]
"""

import statistics
import sys
from collections import defaultdict


def main(argv):
    files, expected = [], None
    it = iter(argv[1:])
    for a in it:
        if a == "--expected":
            expected = next(it, None)
        else:
            files.append(a)
    if not files:
        print(__doc__, file=sys.stderr)
        return 2

    events = defaultdict(list)   # site -> [(seq_delta, ms_delta, id)]
    unknown = defaultdict(int)   # site -> count of pre-ledger (PostLoad) reads
    for path in files:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith(("site,", "#")):
                    continue
                parts = line.split(",")
                if len(parts) != 5:
                    continue
                site, kind, oid, seq_d, ms_d = parts
                if ms_d == "-1.000":
                    unknown[site] += 1
                    continue
                events[site].append((int(seq_d), float(ms_d), kind + oid))

    def dist(vals):
        return f"{min(vals):.0f}/{statistics.median(vals):.0f}/{max(vals):.0f}"

    print(f"{'site':28s} {'n_ev':>6s} {'n_obj':>6s} {'seq min/med/max':>18s} {'ms min/med/max':>22s}")
    for site in sorted(events, key=lambda s: -len(events[s])):
        ev = events[site]
        seqs = [e[0] for e in ev]
        mss = [e[1] for e in ev]
        nobj = len({e[2] for e in ev})
        print(f"{site:28s} {len(ev):>6d} {nobj:>6d} {dist(seqs):>18s} {dist(mss):>22s}")
    for site, cnt in sorted(unknown.items()):
        print(f"{site:28s} {cnt:>6d}      -  (pre-ledger objects, e.g. PostLoad)")

    if expected:
        with open(expected) as f:
            exp = [l.strip() for l in f if l.strip() and not l.startswith("#")]
        zero = [s for s in exp if s not in events and s not in unknown]
        if zero:
            print("\n# zero-event sites (expiry-safe candidates pending reachability argument):")
            for s in zero:
                print(f"#   {s}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
