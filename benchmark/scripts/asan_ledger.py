#!/usr/bin/env python3
"""P12-L0-a ASan signature ledger (docs/P12_L0_DESIGN.md).

Fork of tsan_ledger.py for AddressSanitizer output. Deduplicates ASan
reports into signatures so L2 lifetime-migration steps can be judged by
signature-set DELTA against the (expected-empty) baseline. Signature =
(report type, top frame of the access stack, top frame of the second
stack — for heap-use-after-free that is the "freed by" stack).

Usage:
    asan_ledger.py <asan-log-file-or-dir> [more ...]

Directories are scanned for asan.* files. Output (stdout), sorted by count
descending then alphabetically:

    <count>  <type>  <frame1> | <frame2>

Frames are normalized like the TSAN ledger: trailing "(module+0x...)"
dropped, column number dropped (file:line kept), /workspace/ prefix
stripped. Exit status: 0 always (the DELTA judgment is the caller's job —
an empty ledger prints "0 reports").
"""

import re
import sys
from collections import Counter
from pathlib import Path

ERROR_RE = re.compile(r"==\d+==\s*ERROR: AddressSanitizer: ([A-Za-z0-9_-]+)")
FRAME0_RE = re.compile(r"^\s*#0\s+(.*)$")
MODULE_RE = re.compile(r"\s+\(\S+\+0x[0-9a-fA-F]+\)$")
COLUMN_RE = re.compile(r"(:\d+):\d+$")
# ASan (unlike TSAN) prefixes every frame with the ASLR-randomized pc:
# "#0 0x4008b9 in func file:line" — strip it or identical sites never dedup.
PC_RE = re.compile(r"^0x[0-9a-fA-F]+ in ")


def normalize_frame(raw: str) -> str:
    f = PC_RE.sub("", raw.strip())
    f = MODULE_RE.sub("", f)
    f = COLUMN_RE.sub(r"\1", f)
    f = f.replace("/workspace/", "")
    return f.strip() or "?"


def parse_reports(text: str):
    """Yield (type, frame1, frame2) per report."""
    lines = text.splitlines()
    i, n = 0, len(lines)
    while i < n:
        m = ERROR_RE.search(lines[i])
        if not m:
            i += 1
            continue
        rtype = m.group(1).strip()
        frames = []
        i += 1
        while i < n:
            if ERROR_RE.search(lines[i]) or lines[i].startswith("SUMMARY: AddressSanitizer"):
                break
            fm = FRAME0_RE.match(lines[i])
            if fm and len(frames) < 2:
                frames.append(normalize_frame(fm.group(1)))
            i += 1
        f1 = frames[0] if frames else "?"
        f2 = frames[1] if len(frames) > 1 else "-"
        yield (rtype, f1, f2)


def collect_files(args):
    files = []
    for a in args:
        p = Path(a)
        if p.is_dir():
            files.extend(sorted(p.glob("asan.*")))
        elif p.is_file():
            files.append(p)
        else:
            print(f"WARNING: skipping missing path {a}", file=sys.stderr)
    return files


def main(argv):
    if len(argv) < 2:
        print(__doc__, file=sys.stderr)
        return 2
    files = collect_files(argv[1:])
    if not files:
        print("(no asan.* log files — 0 reports)")
        return 0

    sigs = Counter()
    total = 0
    for f in files:
        text = f.read_text(errors="replace")
        for sig in parse_reports(text):
            sigs[sig] += 1
            total += 1

    if not sigs:
        print("(0 reports across %d file(s))" % len(files))
        return 0
    for (rtype, f1, f2), cnt in sorted(sigs.items(), key=lambda kv: (-kv[1], kv[0])):
        print(f"{cnt:6d}  {rtype}  {f1} | {f2}")
    print(f"# total {total} report(s), {len(sigs)} signature(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
