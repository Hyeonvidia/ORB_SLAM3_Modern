#!/usr/bin/env python3
"""P6 backend-equivalence comparator (docs/P6_DESIGN.md SS B, layer-1 gates).

Usage:
    compare.py <record_A> <record_B>     # vendored vs modern (P6-2)
    compare.py --self-test <record>      # record vs itself, sanity gate

Each file contains one canonical record (the first EQUIV_RECORD found is
used; BEGIN_RECORD/END_RECORD wrappers, if present, are ignored).

Tolerance table for pose_optimization (design SS B.2 — the Frame pose is
stored as float, so 1e-9 gates would be meaningless):

    field            gate
    ---------------  ----------------------------------------
    INPUT_HASH       exact (mismatch aborts the comparison)
    inliers          exact
    pose translation abs <= 1e-6 m, per component
    pose rotation    geodesic <= 1e-6 rad (via quaternion dot)
    outlier flags    exact, per index

Exit code 0 = equivalent within gates, 1 = mismatch, 2 = usage/parse error.
"""

import math
import sys

TRANS_ABS_TOL = 1e-6      # meters, per component
ROT_GEODESIC_TOL = 1e-6   # radians


def parse_record(path):
    """Parse the first record in the file into a dict."""
    rec = {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            lines = [ln.rstrip("\n") for ln in f]
    except OSError as e:
        print(f"compare.py: cannot read {path}: {e}", file=sys.stderr)
        sys.exit(2)

    in_record = False
    for ln in lines:
        if ln.startswith("EQUIV_RECORD"):
            if in_record:  # only the first record
                break
            in_record = True
            rec["version"] = ln
            continue
        if not in_record:
            continue
        if ln == "END_RECORD":
            break
        parts = ln.split()
        if not parts:
            continue
        key = parts[0]
        if key in ("function", "fixture", "INPUT_HASH"):
            rec[key] = parts[1]
        elif key == "inliers":
            rec["inliers"] = int(parts[1])
        elif key == "pose_t":
            rec["pose_t"] = [float(x) for x in parts[1:4]]
        elif key == "pose_q":
            rec["pose_q"] = [float(x) for x in parts[1:5]]  # w x y z
        elif key == "outliers":
            rec["outliers"] = [int(x) for x in parts[1:]]

    required = ["version", "function", "fixture", "INPUT_HASH",
                "inliers", "pose_t", "pose_q", "outliers"]
    missing = [k for k in required if k not in rec]
    if missing:
        print(f"compare.py: {path}: missing fields {missing}", file=sys.stderr)
        sys.exit(2)
    return rec


def quat_geodesic(qa, qb):
    """Geodesic angle (rad) between unit quaternions [w,x,y,z]."""
    dot = abs(sum(a * b for a, b in zip(qa, qb)))
    dot = min(1.0, dot)
    return 2.0 * math.acos(dot)


def compare(rec_a, rec_b, name_a, name_b):
    failures = []

    for key in ("version", "function", "fixture"):
        if rec_a[key] != rec_b[key]:
            failures.append(f"{key}: {rec_a[key]!r} != {rec_b[key]!r}")
    if failures:
        for f in failures:
            print(f"MISMATCH {f}")
        return False

    # Input-hash gate: fixtures are shared code — a differing input dump
    # voids the whole comparison (design SS B.2).
    if rec_a["INPUT_HASH"] != rec_b["INPUT_HASH"]:
        print(f"MISMATCH INPUT_HASH: {rec_a['INPUT_HASH']} != "
              f"{rec_b['INPUT_HASH']} — comparison void, fix fixtures first")
        return False

    # inlier count: exact (discrete contract guaranteed by fixture margins)
    if rec_a["inliers"] != rec_b["inliers"]:
        failures.append(
            f"inliers: {rec_a['inliers']} != {rec_b['inliers']} (exact gate)")

    # translation: per-component absolute
    dt = [abs(a - b) for a, b in zip(rec_a["pose_t"], rec_b["pose_t"])]
    if max(dt) > TRANS_ABS_TOL:
        failures.append(
            f"pose translation: max abs delta {max(dt):.3e} > {TRANS_ABS_TOL:g}"
            f" ({name_a}={rec_a['pose_t']}, {name_b}={rec_b['pose_t']})")

    # rotation: geodesic distance via quaternion dot
    ang = quat_geodesic(rec_a["pose_q"], rec_b["pose_q"])
    if ang > ROT_GEODESIC_TOL:
        failures.append(
            f"pose rotation: geodesic delta {ang:.3e} rad > {ROT_GEODESIC_TOL:g}"
            f" ({name_a}={rec_a['pose_q']}, {name_b}={rec_b['pose_q']})")

    # outlier flags: exact, per index
    if len(rec_a["outliers"]) != len(rec_b["outliers"]):
        failures.append(f"outlier flag count: {len(rec_a['outliers'])} != "
                        f"{len(rec_b['outliers'])}")
    else:
        diff = [i for i, (a, b) in
                enumerate(zip(rec_a["outliers"], rec_b["outliers"])) if a != b]
        if diff:
            failures.append(f"outlier flags differ at indices {diff} (exact gate)")

    if failures:
        for f in failures:
            print(f"MISMATCH {f}")
        return False

    print(f"EQUIVALENT {rec_a['function']}/{rec_a['fixture']}: "
          f"inliers={rec_a['inliers']}, "
          f"max |dt|={max(dt):.3e} m (gate {TRANS_ABS_TOL:g}), "
          f"geodesic dR={ang:.3e} rad (gate {ROT_GEODESIC_TOL:g}), "
          f"outlier flags identical")
    return True


def main(argv):
    if len(argv) == 3 and argv[1] == "--self-test":
        path_a = path_b = argv[2]
        name_a, name_b = "self", "self"
    elif len(argv) == 3:
        path_a, path_b = argv[1], argv[2]
        name_a, name_b = "A", "B"
    else:
        print(__doc__, file=sys.stderr)
        return 2

    rec_a = parse_record(path_a)
    rec_b = parse_record(path_b)
    return 0 if compare(rec_a, rec_b, name_a, name_b) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
