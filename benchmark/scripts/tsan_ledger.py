#!/usr/bin/env python3
"""P10 TSAN signature ledger (docs/P10_RECON.md 3부 §1).

Deduplicates ThreadSanitizer reports into signatures so migration steps can
be judged by signature-set DELTA (tombstone-contract noise makes TSAN-clean
unreachable by design). Signature = (report type, top frame of the first
stack, top frame of the second stack).

Usage:
    tsan_ledger.py <tsan-log-file-or-dir> [more ...]

Directories are scanned for tsan.* files. Output (stdout), sorted by count
descending then alphabetically:

    <count>  <type>  <frame1> | <frame2>

Frames are normalized: the trailing "(module+0x...)" is dropped, the column
number is dropped (file:line kept), and the /workspace/ prefix is stripped.
"""

import re
import sys
from collections import Counter
from pathlib import Path

WARN_RE = re.compile(r"WARNING: ThreadSanitizer: (.+?) \(pid=\d+\)")
FRAME0_RE = re.compile(r"^\s*#0\s+(.*)$")
MODULE_RE = re.compile(r"\s+\(\S+\+0x[0-9a-fA-F]+\)$")
COLUMN_RE = re.compile(r"(:\d+):\d+$")


def normalize_frame(raw: str) -> str:
    f = MODULE_RE.sub("", raw.strip())
    f = COLUMN_RE.sub(r"\1", f)
    f = f.replace("/workspace/", "")
    return f.strip() or "?"


def parse_reports(text: str):
    """Yield (type, frame1, frame2) per report."""
    lines = text.splitlines()
    i, n = 0, len(lines)
    while i < n:
        m = WARN_RE.search(lines[i])
        if not m:
            i += 1
            continue
        rtype = m.group(1).strip()
        frames = []
        i += 1
        while i < n:
            if WARN_RE.search(lines[i]) or lines[i].startswith("SUMMARY: ThreadSanitizer"):
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
            files.extend(sorted(p.glob("tsan.*")))
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
        print("WARNING: no tsan.* log files found", file=sys.stderr)

    sigs = Counter()
    total = 0
    for f in files:
        text = f.read_text(errors="replace")
        for sig in parse_reports(text):
            sigs[sig] += 1
            total += 1

    for (rtype, f1, f2), cnt in sorted(
        sigs.items(), key=lambda kv: (-kv[1], kv[0])
    ):
        print(f"{cnt:5d}  {rtype}  {f1} | {f2}")

    print(
        f"# {total} reports, {len(sigs)} unique signatures, "
        f"{len(files)} log file(s)",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
