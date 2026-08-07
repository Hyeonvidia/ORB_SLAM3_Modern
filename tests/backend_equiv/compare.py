#!/usr/bin/env python3
"""P6 backend-equivalence comparator (docs/P6_DESIGN.md SS B, layer-1 gates).

Usage:
    compare.py <record_A> <record_B>     # vendored vs modern (P6-2)
    compare.py --self-test <record>      # record vs itself, sanity gate
    compare.py --gt-check <record>       # inertial only: analytic GT recovery

Each file contains one canonical record (the first EQUIV_RECORD found is
used; BEGIN_RECORD/END_RECORD wrappers, if present, are ignored).

Tolerance table (design SS B.2):

  pose_optimization (Frame pose is stored as float -> 1e-9 gates would be
  meaningless):

    field            gate
    ---------------  ----------------------------------------
    INPUT_HASH       exact (mismatch aborts the comparison)
    inliers          exact
    pose translation abs <= 1e-6 m, per component
    pose rotation    geodesic <= 1e-6 rad (via quaternion dot)
    outlier flags    exact, per index

  inertial_optimization (scale/Rwg are returned as double -> tight gates):

    field            vendored<->modern gate       GT gate (--gt-check)
    ---------------  ---------------------------  -----------------------
    INPUT_HASH       exact                        --
    GT_scale/GT_gdir exact (fixture constants)    --
    scale            relative <= 1e-9             rel |s - s_true| < 1e-7
    Rwg              geodesic <= 1e-9 rad         gravity-direction
    gdir             angle    <= 1e-9 rad           angle to GT < 1e-7 rad
    velocities       abs <= 1e-8, per component   --
                     (fixed vertices: also an "inputs untouched" check)

Exit code 0 = equivalent within gates, 1 = mismatch, 2 = usage/parse error.
"""

import math
import sys

TRANS_ABS_TOL = 1e-6        # meters, per component
ROT_GEODESIC_TOL = 1e-6     # radians

SCALE_REL_TOL = 1e-9        # vendored <-> modern, relative
RWG_GEODESIC_TOL = 1e-9     # radians
GDIR_ANGLE_TOL = 1e-9       # radians
VEL_ABS_TOL = 1e-8          # m/s, per component

GT_SCALE_REL_TOL = 1e-7     # |scale - s_true| / s_true
GT_GDIR_ANGLE_TOL = 1e-7    # radians, estimated vs true gravity direction

REQUIRED_FIELDS = {
    "pose_optimization": ["version", "function", "fixture", "INPUT_HASH",
                          "inliers", "pose_t", "pose_q", "outliers"],
    "inertial_optimization": ["version", "function", "fixture", "INPUT_HASH",
                              "GT_scale", "GT_gdir", "scale", "rwg_q",
                              "gdir", "vels"],
}


def parse_record(path):
    """Parse the first record in the file into a dict."""
    rec = {"vels": {}}
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
        elif key == "GT_scale":
            rec["GT_scale"] = float(parts[1])
        elif key == "GT_gdir":
            rec["GT_gdir"] = [float(x) for x in parts[1:4]]
        elif key == "scale":
            rec["scale"] = float(parts[1])
        elif key == "rwg_q":
            rec["rwg_q"] = [float(x) for x in parts[1:5]]  # w x y z
        elif key == "gdir":
            rec["gdir"] = [float(x) for x in parts[1:4]]
        elif key == "vel":
            rec["vels"][int(parts[1])] = [float(x) for x in parts[2:5]]

    fn = rec.get("function")
    required = REQUIRED_FIELDS.get(fn)
    if required is None:
        print(f"compare.py: {path}: unknown/missing function {fn!r}",
              file=sys.stderr)
        sys.exit(2)
    missing = [k for k in required
               if k not in rec or (k == "vels" and not rec["vels"])]
    if missing:
        print(f"compare.py: {path}: missing fields {missing}", file=sys.stderr)
        sys.exit(2)
    return rec


def quat_geodesic(qa, qb):
    """Geodesic angle (rad) between unit quaternions [w,x,y,z].

    Chord-based (4*asin(d/2) with d = min-sign chord distance): acos(dot)
    is ill-conditioned near dot=1 (floor ~4e-8 even for identical inputs),
    which would swamp the 1e-9 gates; asin near 0 is exact.
    """
    d_minus = math.sqrt(sum((a - b) ** 2 for a, b in zip(qa, qb)))
    d_plus = math.sqrt(sum((a + b) ** 2 for a, b in zip(qa, qb)))
    d = min(d_minus, d_plus)          # quaternion double cover
    d = min(d, 2.0)
    return 4.0 * math.asin(d / 2.0)


def vec_angle(va, vb):
    """Angle (rad) between two 3-vectors — atan2(|a x b|, a.b), which is
    well-conditioned near 0 (acos(dot) has a ~1e-8 floor)."""
    cx = va[1] * vb[2] - va[2] * vb[1]
    cy = va[2] * vb[0] - va[0] * vb[2]
    cz = va[0] * vb[1] - va[1] * vb[0]
    cross = math.sqrt(cx * cx + cy * cy + cz * cz)
    dot = sum(a * b for a, b in zip(va, vb))
    return math.atan2(cross, dot)


def compare_pose(rec_a, rec_b, name_a, name_b, failures):
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
        return None
    return (f"inliers={rec_a['inliers']}, "
            f"max |dt|={max(dt):.3e} m (gate {TRANS_ABS_TOL:g}), "
            f"geodesic dR={ang:.3e} rad (gate {ROT_GEODESIC_TOL:g}), "
            f"outlier flags identical")


def compare_inertial(rec_a, rec_b, name_a, name_b, failures):
    # GT header fields are fixture constants: exact match required.
    if rec_a["GT_scale"] != rec_b["GT_scale"]:
        failures.append(f"GT_scale: {rec_a['GT_scale']!r} != "
                        f"{rec_b['GT_scale']!r} (fixture constant)")
    if rec_a["GT_gdir"] != rec_b["GT_gdir"]:
        failures.append(f"GT_gdir: {rec_a['GT_gdir']!r} != "
                        f"{rec_b['GT_gdir']!r} (fixture constant)")

    # scale: relative (double-precision output path, design SS B.2)
    denom = max(abs(rec_a["scale"]), abs(rec_b["scale"]))
    rel = abs(rec_a["scale"] - rec_b["scale"]) / denom
    if rel > SCALE_REL_TOL:
        failures.append(
            f"scale: relative delta {rel:.3e} > {SCALE_REL_TOL:g}"
            f" ({name_a}={rec_a['scale']!r}, {name_b}={rec_b['scale']!r})")

    # Rwg: geodesic distance via quaternion dot
    ang = quat_geodesic(rec_a["rwg_q"], rec_b["rwg_q"])
    if ang > RWG_GEODESIC_TOL:
        failures.append(
            f"Rwg: geodesic delta {ang:.3e} rad > {RWG_GEODESIC_TOL:g}"
            f" ({name_a}={rec_a['rwg_q']}, {name_b}={rec_b['rwg_q']})")

    # gravity direction (redundant with Rwg, cheap and readable)
    gang = vec_angle(rec_a["gdir"], rec_b["gdir"])
    if gang > GDIR_ANGLE_TOL:
        failures.append(
            f"gdir: angle {gang:.3e} rad > {GDIR_ANGLE_TOL:g}"
            f" ({name_a}={rec_a['gdir']}, {name_b}={rec_b['gdir']})")

    # velocities: per-component absolute (fixed vertices in this variant)
    if sorted(rec_a["vels"]) != sorted(rec_b["vels"]):
        failures.append(f"velocity KF index sets differ: "
                        f"{sorted(rec_a['vels'])} != {sorted(rec_b['vels'])}")
        max_dv = float("nan")
    else:
        max_dv = 0.0
        for k in sorted(rec_a["vels"]):
            dv = [abs(a - b) for a, b in
                  zip(rec_a["vels"][k], rec_b["vels"][k])]
            max_dv = max(max_dv, max(dv))
            if max(dv) > VEL_ABS_TOL:
                failures.append(
                    f"vel[{k}]: max abs delta {max(dv):.3e} > {VEL_ABS_TOL:g}"
                    f" ({name_a}={rec_a['vels'][k]}, {name_b}={rec_b['vels'][k]})")

    if failures:
        return None
    return (f"scale rel delta={rel:.3e} (gate {SCALE_REL_TOL:g}), "
            f"Rwg geodesic={ang:.3e} rad (gate {RWG_GEODESIC_TOL:g}), "
            f"gdir angle={gang:.3e} rad (gate {GDIR_ANGLE_TOL:g}), "
            f"max |dvel|={max_dv:.3e} (gate {VEL_ABS_TOL:g})")


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

    if rec_a["function"] == "pose_optimization":
        summary = compare_pose(rec_a, rec_b, name_a, name_b, failures)
    else:
        summary = compare_inertial(rec_a, rec_b, name_a, name_b, failures)

    if failures:
        for f in failures:
            print(f"MISMATCH {f}")
        return False

    print(f"EQUIVALENT {rec_a['function']}/{rec_a['fixture']}: {summary}")
    return True


def gt_check(rec):
    """Analytic ground-truth recovery gate (inertial_optimization only)."""
    if rec["function"] != "inertial_optimization":
        print(f"compare.py: --gt-check only applies to inertial_optimization "
              f"records (got {rec['function']!r})", file=sys.stderr)
        return 2

    failures = []

    s_true = rec["GT_scale"]
    rel = abs(rec["scale"] - s_true) / s_true
    if rel > GT_SCALE_REL_TOL:
        failures.append(
            f"scale recovery: |{rec['scale']!r} - {s_true!r}|/{s_true!r} = "
            f"{rel:.3e} > {GT_SCALE_REL_TOL:g}")

    gang = vec_angle(rec["gdir"], rec["GT_gdir"])
    if gang > GT_GDIR_ANGLE_TOL:
        failures.append(
            f"gravity-direction recovery: angle {gang:.3e} rad > "
            f"{GT_GDIR_ANGLE_TOL:g} (est={rec['gdir']}, gt={rec['GT_gdir']})")

    if failures:
        for f in failures:
            print(f"GT_MISMATCH {f}")
        return 1

    print(f"GT_RECOVERED {rec['function']}/{rec['fixture']}: "
          f"scale={rec['scale']!r} vs s_true={s_true!r} "
          f"(rel {rel:.3e}, gate {GT_SCALE_REL_TOL:g}), "
          f"gravity-direction angle={gang:.3e} rad "
          f"(gate {GT_GDIR_ANGLE_TOL:g})")
    return 0


def main(argv):
    if len(argv) == 3 and argv[1] == "--gt-check":
        return gt_check(parse_record(argv[2]))
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
