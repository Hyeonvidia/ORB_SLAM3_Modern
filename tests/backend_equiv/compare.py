#!/usr/bin/env python3
"""P6 backend-equivalence comparator (docs/P6_DESIGN.md SS B, layer-1 gates).

Usage:
    compare.py <record_A> <record_B>     # vendored vs modern (P6-2)
    compare.py --self-test <record>      # record vs itself, sanity gate
    compare.py --gt-check <record>       # analytic GT recovery (where defined)

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

  inertial_optimization_full (P6-4a) / inertial_optimization_bias (P6-4b):

    field            vendored<->modern gate       GT gate (--gt-check)
    ---------------  ---------------------------  -----------------------
    scale (full)     relative <= 1e-9             rel <= 1e-4  (see below)
    Rwg / gdir(full) geodesic <= 1e-9 rad         angle <= 1e-6 rad
    bg, ba           abs <= 1e-9                  derived, see below
    velocities       abs <= 1e-6, per component   --
                     (OPTIMIZED and written back through SetVelocity(float),
                      so the float storage path sets the gate, not 1e-8)

    GT gates for the two bias overloads are NOT the flat 1e-7 of the design
    table. The loosening is backed by a measurement carried IN the record
    (GT_resid_er / GT_resid_ev / GT_resid_ep / GT_resid_dT, plus priorG and
    priorA) and by the two diagnostics wired into EquivMain.cpp.

    WHY. Both overloads are Levenberg-Marquardt with setUserLambdaInit(1e3);
    the P6-3 scale overload is plain Gauss-Newton, which is why THAT pair keeps
    its 1e-7 gate (measured 1.57e-08). Under LM both new pairs stop at a
    non-stationary point:

      EQUIV_FULL_DEBUG=1  (inertial_optimization_full)
        chi2 at the analytic optimum        7.53e-08
        chi2 where LM parks from (I, 1.0)   5.76e-05
        8 chained calls do not move it (hard fixed point of the solver)
        initializing AT the analytic optimum keeps it there (scale drift
          1.0e-11, gdir 3.3e-08 rad, |dbg| 7.8e-10)
        => scale 1.12e-05 relative off, |dbg| 3.97e-07, |dba| 5.62e-08
        => identical for priorG = 1e2 and priorG = 1e-3, so it is NOT prior
           shrinkage

      EQUIV_BIAS_DEBUG=1  (inertial_optimization_bias)
        chi2 at the analytic optimum        4.1419e-08
        chi2 where LM parks from bias 0     4.3864e-08   (only 5.9% higher)
        GT-init stays put (|dbg| 8.5e-11, |dba| 6.7e-10)
        => |dbg| 4.59e-07, |dba| 4.64e-07

    The residual rows explain the magnitude: the fixture stores KF states as
    float32 (KeyFrame::SetPose/SetVelocity), so the chi2 at the analytic
    optimum is already ~4e-08 rather than 0, and LM abandons a descent worth
    only ~6% of that floor. So the analytic optimum is genuine (the GT-init
    probes prove it) and the shortfall belongs to the solver — the same family
    as docs/DIVERGENCES.md #9 (EdgeInertialGS pairs an additive scale Jacobian
    with a multiplicative VertexScale update), preserved bug-for-bug. BOTH
    backends must therefore reproduce the SAME stall, which is precisely what
    the 1e-9 cross-variant gate below tests; the GT gate only has to be loose
    enough to admit the stall and tight enough to catch a broken fixture (the
    errors it must not admit are the 2.5e-02 scale / 3.7e-02 ba miss seen
    while the fixture was still mis-designed, i.e. 4 orders of margin).

    Gates used:  scale rel <= 1e-4, gdir <= 1e-6 rad, bias <= 1e-6 abs.

  pose_inertial_lastkf (P6-4, class B — the GetHessian contract):

    field            vendored<->modern gate       GT gate (--gt-check)
    ---------------  ---------------------------  -----------------------
    inliers          exact                        == 37 designed
    pose t / q       abs 1e-6 m / 1e-6 rad        1e-5 / 1e-5
    velocity         abs 1e-6                     1e-5
    bg, ba           abs 1e-9 (float, GT is 0)    1e-7
    outlier flags    exact, per index             designed set {7,19,31}
    mpcpi 15x15 H    element-wise RELATIVE 1e-6   --
                     with an absolute floor of 1e-16 * max|H| — DELIBERATELY
                     below the eps*max|H| roundoff of the SelfAdjointEigenSolver
                     reconstruction in ConstraintPoseImu's ctor, so the gate
                     cannot hide a real difference. It exists only so the
                     structurally-zero blocks (rows/cols 9..14 decouple from
                     0..8) are not compared relatively against exact 0.

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

# --- P6-4 -----------------------------------------------------------------
BIAS_ABS_TOL = 1e-9         # vendored <-> modern, bg/ba (double return path)
OPT_VEL_ABS_TOL = 1e-6      # optimized velocities: SetVelocity(float) path

# LM-stall gates for the two bias overloads (see docstring). The
# Gauss-Newton scale overload keeps GT_SCALE_REL_TOL / GT_GDIR_ANGLE_TOL.
GT_LM_BIAS_ABS_TOL = 1e-6             # measured 3.97e-7 / 4.64e-7
GT_FULL_SCALE_REL_TOL = 1e-4          # measured 1.12e-5
GT_FULL_GDIR_ANGLE_TOL = 1e-6         # measured 2.39e-8

PI_TRANS_ABS_TOL = 1e-6     # pose_inertial_lastkf, float storage path
PI_ROT_GEODESIC_TOL = 1e-6
PI_VEL_ABS_TOL = 1e-6
PI_BIAS_ABS_TOL = 1e-9
HESS_REL_TOL = 1e-6         # element-wise relative
# Absolute floor as a fraction of max|H|. ConstraintPoseImu's ctor rebuilds H
# from a SelfAdjointEigenSolver, whose absolute roundoff is ~eps*max|H|
# (= 6.1e-06 here), so elements far below that carry no relative information at
# all. 1e-16 is DELIBERATELY tighter than eps (2.2e-16): the gate must not be
# able to hide a real difference, and it does not have to — measured, the
# largest absolute element-wise delta over the whole matrix is 5.7e-06, i.e.
# under one eps*max|H|, and only H[6][7] / H[7][6] (value 7.4e-02, i.e. 2.7e-12
# of the matrix norm; absolute delta 8.6e-08, seventy times below the
# reconstruction floor) sit above 1e-6 relative.
HESS_ABS_FLOOR_REL = 1e-16

GT_PI_TRANS_ABS_TOL = 1e-5
GT_PI_ROT_GEODESIC_TOL = 1e-5
GT_PI_VEL_ABS_TOL = 1e-5
GT_PI_BIAS_ABS_TOL = 1e-7
PI_DESIGNED_INLIERS = 37
PI_DESIGNED_OUTLIERS = [7, 19, 31]

REQUIRED_FIELDS = {
    "pose_optimization": ["version", "function", "fixture", "INPUT_HASH",
                          "inliers", "pose_t", "pose_q", "outliers"],
    "inertial_optimization": ["version", "function", "fixture", "INPUT_HASH",
                              "GT_scale", "GT_gdir", "scale", "rwg_q",
                              "gdir", "vels"],
    "inertial_optimization_full": ["version", "function", "fixture",
                                   "INPUT_HASH", "priorG", "priorA",
                                   "GT_resid_er", "GT_resid_ev", "GT_resid_dT",
                                   "GT_scale", "GT_gdir", "GT_bg", "GT_ba",
                                   "scale", "rwg_q", "gdir", "bg", "ba",
                                   "vels"],
    "inertial_optimization_bias": ["version", "function", "fixture",
                                   "INPUT_HASH", "priorG", "priorA",
                                   "GT_resid_er", "GT_resid_ev", "GT_resid_dT",
                                   "GT_bg", "GT_ba", "bg", "ba", "vels"],
    "pose_inertial_lastkf": ["version", "function", "fixture", "INPUT_HASH",
                             "GT_pose_t", "GT_pose_q", "GT_vel", "inliers",
                             "pose_t", "pose_q", "vel", "bg", "ba",
                             "outliers", "H"],
}


def parse_record(path):
    """Parse the first record in the file into a dict."""
    rec = {"vels": {}, "H": {}}
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
            # "vel <k> x y z" in the KF-chain records; bare "vel x y z" in the
            # pose_inertial record (a single Frame velocity).
            if len(parts) == 5:
                rec["vels"][int(parts[1])] = [float(x) for x in parts[2:5]]
            else:
                rec["vel"] = [float(x) for x in parts[1:4]]
        elif key in ("priorG", "priorA", "GT_resid_er", "GT_resid_ev",
                     "GT_resid_ep", "GT_resid_dT"):
            rec[key] = float(parts[1])
        elif key in ("GT_bg", "GT_ba", "bg", "ba", "GT_vel"):
            rec[key] = [float(x) for x in parts[1:4]]
        elif key in ("GT_pose_t",):
            rec["GT_pose_t"] = [float(x) for x in parts[1:4]]
        elif key in ("GT_pose_q",):
            rec["GT_pose_q"] = [float(x) for x in parts[1:5]]
        elif key == "H":
            rec["H"][int(parts[1])] = [float(x) for x in parts[2:17]]

    fn = rec.get("function")
    required = REQUIRED_FIELDS.get(fn)
    if required is None:
        print(f"compare.py: {path}: unknown/missing function {fn!r}",
              file=sys.stderr)
        sys.exit(2)
    missing = [k for k in required
               if k not in rec
               or (k in ("vels", "H") and not rec[k])]
    if "H" in required and len(rec.get("H", {})) != 15:
        missing.append("H(15 rows)")
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


def _vel_block(rec_a, rec_b, name_a, name_b, failures, tol):
    """Per-KF velocity comparison shared by the three KF-chain records."""
    if sorted(rec_a["vels"]) != sorted(rec_b["vels"]):
        failures.append(f"velocity KF index sets differ: "
                        f"{sorted(rec_a['vels'])} != {sorted(rec_b['vels'])}")
        return float("nan")
    max_dv = 0.0
    for k in sorted(rec_a["vels"]):
        dv = [abs(a - b) for a, b in zip(rec_a["vels"][k], rec_b["vels"][k])]
        max_dv = max(max_dv, max(dv))
        if max(dv) > tol:
            failures.append(
                f"vel[{k}]: max abs delta {max(dv):.3e} > {tol:g}"
                f" ({name_a}={rec_a['vels'][k]}, {name_b}={rec_b['vels'][k]})")
    return max_dv


def _vec_abs(rec_a, rec_b, key, tol, failures, label):
    d = max(abs(a - b) for a, b in zip(rec_a[key], rec_b[key]))
    if d > tol:
        failures.append(f"{label}: max abs delta {d:.3e} > {tol:g}"
                        f" (A={rec_a[key]}, B={rec_b[key]})")
    return d


def _header_exact(rec_a, rec_b, keys, failures):
    for k in keys:
        if rec_a[k] != rec_b[k]:
            failures.append(f"{k}: {rec_a[k]!r} != {rec_b[k]!r} "
                            f"(fixture/run constant)")


def compare_inertial_full(rec_a, rec_b, name_a, name_b, failures):
    _header_exact(rec_a, rec_b,
                  ["priorG", "priorA", "GT_scale", "GT_gdir", "GT_bg", "GT_ba"],
                  failures)

    denom = max(abs(rec_a["scale"]), abs(rec_b["scale"]))
    rel = abs(rec_a["scale"] - rec_b["scale"]) / denom
    if rel > SCALE_REL_TOL:
        failures.append(
            f"scale: relative delta {rel:.3e} > {SCALE_REL_TOL:g}"
            f" ({name_a}={rec_a['scale']!r}, {name_b}={rec_b['scale']!r})")

    ang = quat_geodesic(rec_a["rwg_q"], rec_b["rwg_q"])
    if ang > RWG_GEODESIC_TOL:
        failures.append(f"Rwg: geodesic delta {ang:.3e} rad > "
                        f"{RWG_GEODESIC_TOL:g}")
    gang = vec_angle(rec_a["gdir"], rec_b["gdir"])
    if gang > GDIR_ANGLE_TOL:
        failures.append(f"gdir: angle {gang:.3e} rad > {GDIR_ANGLE_TOL:g}")

    dbg = _vec_abs(rec_a, rec_b, "bg", BIAS_ABS_TOL, failures, "bg")
    dba = _vec_abs(rec_a, rec_b, "ba", BIAS_ABS_TOL, failures, "ba")
    max_dv = _vel_block(rec_a, rec_b, name_a, name_b, failures,
                        OPT_VEL_ABS_TOL)

    if failures:
        return None
    return (f"scale rel delta={rel:.3e} (gate {SCALE_REL_TOL:g}), "
            f"Rwg geodesic={ang:.3e} rad, gdir angle={gang:.3e} rad, "
            f"max |dbg|={dbg:.3e}, max |dba|={dba:.3e} "
            f"(gate {BIAS_ABS_TOL:g}), max |dvel|={max_dv:.3e} "
            f"(gate {OPT_VEL_ABS_TOL:g})")


def compare_inertial_bias(rec_a, rec_b, name_a, name_b, failures):
    _header_exact(rec_a, rec_b, ["priorG", "priorA", "GT_bg", "GT_ba"],
                  failures)
    dbg = _vec_abs(rec_a, rec_b, "bg", BIAS_ABS_TOL, failures, "bg")
    dba = _vec_abs(rec_a, rec_b, "ba", BIAS_ABS_TOL, failures, "ba")
    max_dv = _vel_block(rec_a, rec_b, name_a, name_b, failures,
                        OPT_VEL_ABS_TOL)
    if failures:
        return None
    return (f"max |dbg|={dbg:.3e}, max |dba|={dba:.3e} "
            f"(gate {BIAS_ABS_TOL:g}), max |dvel|={max_dv:.3e} "
            f"(gate {OPT_VEL_ABS_TOL:g})")


def compare_hessian(rec_a, rec_b, failures):
    """15x15 ConstraintPoseImu Hessian: element-wise relative, with an absolute
    floor at HESS_ABS_FLOOR_REL * max|H| (see the constant's comment)."""
    max_abs = max(abs(v) for row in rec_a["H"].values() for v in row)
    floor = HESS_ABS_FLOOR_REL * max_abs
    worst_rel, worst_at = 0.0, None
    worst_abs, worst_abs_at = 0.0, None
    floored_rel, floored_at = 0.0, None
    n_floored = 0
    bad = []
    for r in range(15):
        for c in range(15):
            a, b = rec_a["H"][r][c], rec_b["H"][r][c]
            d = abs(a - b)
            if d > worst_abs:
                worst_abs, worst_abs_at = d, (r, c)
            denom = max(abs(a), abs(b))
            rel = d / denom if denom > 0.0 else float("inf")
            if d <= floor:
                # Reported, never gated: full transparency about what the
                # floor absorbs.
                if d > 0.0:
                    n_floored += 1
                    if rel > floored_rel:
                        floored_rel, floored_at = rel, (r, c)
                continue
            if rel > worst_rel:
                worst_rel, worst_at = rel, (r, c)
            if rel > HESS_REL_TOL:
                bad.append(f"H[{r}][{c}]: rel {rel:.3e} abs {d:.3e} "
                           f"(A={a!r}, B={b!r})")
    if bad:
        failures.append(f"mpcpi Hessian: {len(bad)} element(s) over "
                        f"{HESS_REL_TOL:g} relative; first: {bad[0]}")
    return {"rel": worst_rel, "rel_at": worst_at, "abs": worst_abs,
            "abs_at": worst_abs_at, "max_abs": max_abs, "floor": floor,
            "floored": n_floored, "floored_rel": floored_rel,
            "floored_at": floored_at}


def compare_pose_inertial(rec_a, rec_b, name_a, name_b, failures):
    _header_exact(rec_a, rec_b, ["GT_pose_t", "GT_pose_q", "GT_vel"], failures)

    if rec_a["inliers"] != rec_b["inliers"]:
        failures.append(
            f"inliers: {rec_a['inliers']} != {rec_b['inliers']} (exact gate)")

    dt = [abs(a - b) for a, b in zip(rec_a["pose_t"], rec_b["pose_t"])]
    if max(dt) > PI_TRANS_ABS_TOL:
        failures.append(f"pose translation: max abs delta {max(dt):.3e} > "
                        f"{PI_TRANS_ABS_TOL:g}")
    ang = quat_geodesic(rec_a["pose_q"], rec_b["pose_q"])
    if ang > PI_ROT_GEODESIC_TOL:
        failures.append(f"pose rotation: geodesic delta {ang:.3e} rad > "
                        f"{PI_ROT_GEODESIC_TOL:g}")

    dv = _vec_abs(rec_a, rec_b, "vel", PI_VEL_ABS_TOL, failures, "velocity")
    dbg = _vec_abs(rec_a, rec_b, "bg", PI_BIAS_ABS_TOL, failures, "bg")
    dba = _vec_abs(rec_a, rec_b, "ba", PI_BIAS_ABS_TOL, failures, "ba")

    if len(rec_a["outliers"]) != len(rec_b["outliers"]):
        failures.append(f"outlier flag count: {len(rec_a['outliers'])} != "
                        f"{len(rec_b['outliers'])}")
    else:
        diff = [i for i, (a, b) in
                enumerate(zip(rec_a["outliers"], rec_b["outliers"])) if a != b]
        if diff:
            failures.append(
                f"outlier flags differ at indices {diff} (exact gate)")

    h = compare_hessian(rec_a, rec_b, failures)
    if failures:
        return None
    return (f"inliers={rec_a['inliers']}, max |dt|={max(dt):.3e} m, "
            f"geodesic dR={ang:.3e} rad, max |dvel|={dv:.3e}, "
            f"max |dbg|={dbg:.3e}, max |dba|={dba:.3e}, "
            f"outlier flags identical; mpcpi 15x15 H: worst element-wise "
            f"relative delta={h['rel']:.3e} at {h['rel_at']} "
            f"(gate {HESS_REL_TOL:g}), worst ABSOLUTE delta={h['abs']:.3e} at "
            f"{h['abs_at']} vs max|H|={h['max_abs']:.6e} "
            f"(eps*max|H|={2.220446049250313e-16 * h['max_abs']:.3e}), "
            f"{h['floored']} element(s) below the {h['floor']:.3e} abs "
            f"floor (worst relative among those: {h['floored_rel']:.3e} at "
            f"{h['floored_at']}, ungated)")


COMPARATORS = {
    "pose_optimization": compare_pose,
    "inertial_optimization": compare_inertial,
    "inertial_optimization_full": compare_inertial_full,
    "inertial_optimization_bias": compare_inertial_bias,
    "pose_inertial_lastkf": compare_pose_inertial,
}


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

    summary = COMPARATORS[rec_a["function"]](rec_a, rec_b, name_a, name_b,
                                             failures)

    if failures:
        for f in failures:
            print(f"MISMATCH {f}")
        return False

    print(f"EQUIVALENT {rec_a['function']}/{rec_a['fixture']}: {summary}")
    return True


def _bias_gt_block(rec, failures):
    """bg/ba vs the injected ground truth, under the LM-stall gate."""
    dbg = max(abs(a - b) for a, b in zip(rec["bg"], rec["GT_bg"]))
    dba = max(abs(a - b) for a, b in zip(rec["ba"], rec["GT_ba"]))
    if dbg > GT_LM_BIAS_ABS_TOL:
        failures.append(f"gyro-bias recovery: max abs {dbg:.3e} > "
                        f"{GT_LM_BIAS_ABS_TOL:g} "
                        f"(est={rec['bg']}, gt={rec['GT_bg']})")
    if dba > GT_LM_BIAS_ABS_TOL:
        failures.append(f"acc-bias recovery: max abs {dba:.3e} > "
                        f"{GT_LM_BIAS_ABS_TOL:g} "
                        f"(est={rec['ba']}, gt={rec['GT_ba']})")
    return dbg, dba


def gt_check_inertial_full(rec):
    failures = []
    s_true = rec["GT_scale"]
    rel = abs(rec["scale"] - s_true) / s_true
    if rel > GT_FULL_SCALE_REL_TOL:
        failures.append(f"scale recovery: rel {rel:.3e} > "
                        f"{GT_FULL_SCALE_REL_TOL:g} "
                        f"(est={rec['scale']!r}, gt={s_true!r})")
    gang = vec_angle(rec["gdir"], rec["GT_gdir"])
    if gang > GT_FULL_GDIR_ANGLE_TOL:
        failures.append(f"gravity-direction recovery: angle {gang:.3e} rad > "
                        f"{GT_FULL_GDIR_ANGLE_TOL:g}")
    dbg, dba = _bias_gt_block(rec, failures)
    if failures:
        for f in failures:
            print(f"GT_MISMATCH {f}")
        return 1
    print(f"GT_RECOVERED {rec['function']}/{rec['fixture']}: "
          f"scale={rec['scale']!r} vs s_true={s_true!r} (rel {rel:.3e}, "
          f"gate {GT_FULL_SCALE_REL_TOL:g} — LM stall, see docstring), "
          f"gravity-direction angle={gang:.3e} rad "
          f"(gate {GT_FULL_GDIR_ANGLE_TOL:g}), "
          f"|dbg|={dbg:.3e}, |dba|={dba:.3e} "
          f"(gate {GT_LM_BIAS_ABS_TOL:g}); fixture float32 residual floor "
          f"er={rec['GT_resid_er']:.3e} ev={rec['GT_resid_ev']:.3e} "
          f"over dT={rec['GT_resid_dT']:.6f}, priors ({rec['priorG']:g}, "
          f"{rec['priorA']:g})")
    return 0


def gt_check_inertial_bias(rec):
    failures = []
    dbg, dba = _bias_gt_block(rec, failures)
    if failures:
        for f in failures:
            print(f"GT_MISMATCH {f}")
        return 1
    print(f"GT_RECOVERED {rec['function']}/{rec['fixture']}: "
          f"bg={rec['bg']} vs GT {rec['GT_bg']} (max abs {dbg:.3e}), "
          f"ba={rec['ba']} vs GT {rec['GT_ba']} (max abs {dba:.3e}); "
          f"gate {GT_LM_BIAS_ABS_TOL:g} (LM stall, see docstring); "
          f"fixture float32 residual floor er={rec['GT_resid_er']:.3e} "
          f"ev={rec['GT_resid_ev']:.3e} over dT={rec['GT_resid_dT']:.6f}, "
          f"priors ({rec['priorG']:g}, {rec['priorA']:g})")
    return 0


def gt_check_pose_inertial(rec):
    failures = []
    if rec["inliers"] != PI_DESIGNED_INLIERS:
        failures.append(f"inlier count {rec['inliers']} != designed "
                        f"{PI_DESIGNED_INLIERS}")
    flagged = [i for i, v in enumerate(rec["outliers"]) if v]
    if flagged != PI_DESIGNED_OUTLIERS:
        failures.append(f"outlier set {flagged} != designed "
                        f"{PI_DESIGNED_OUTLIERS}")

    dt = max(abs(a - b) for a, b in zip(rec["pose_t"], rec["GT_pose_t"]))
    if dt > GT_PI_TRANS_ABS_TOL:
        failures.append(f"pose translation recovery: max abs {dt:.3e} > "
                        f"{GT_PI_TRANS_ABS_TOL:g}")
    ang = quat_geodesic(rec["pose_q"], rec["GT_pose_q"])
    if ang > GT_PI_ROT_GEODESIC_TOL:
        failures.append(f"pose rotation recovery: geodesic {ang:.3e} rad > "
                        f"{GT_PI_ROT_GEODESIC_TOL:g}")
    dv = max(abs(a - b) for a, b in zip(rec["vel"], rec["GT_vel"]))
    if dv > GT_PI_VEL_ABS_TOL:
        failures.append(f"velocity recovery: max abs {dv:.3e} > "
                        f"{GT_PI_VEL_ABS_TOL:g}")
    db = max(abs(v) for v in rec["bg"] + rec["ba"])   # GT bias is zero
    if db > GT_PI_BIAS_ABS_TOL:
        failures.append(f"bias recovery: max abs {db:.3e} > "
                        f"{GT_PI_BIAS_ABS_TOL:g} (GT bias is zero)")

    if failures:
        for f in failures:
            print(f"GT_MISMATCH {f}")
        return 1
    print(f"GT_RECOVERED {rec['function']}/{rec['fixture']}: "
          f"inliers={rec['inliers']} (designed {PI_DESIGNED_INLIERS}), "
          f"outlier set {flagged} as designed, "
          f"pose |dt|={dt:.3e} m (gate {GT_PI_TRANS_ABS_TOL:g}), "
          f"geodesic dR={ang:.3e} rad (gate {GT_PI_ROT_GEODESIC_TOL:g}), "
          f"|dvel|={dv:.3e} (gate {GT_PI_VEL_ABS_TOL:g}), "
          f"|bias|={db:.3e} (gate {GT_PI_BIAS_ABS_TOL:g})")
    return 0


def gt_check(rec):
    """Analytic ground-truth recovery gate."""
    fn = rec["function"]
    if fn == "inertial_optimization_full":
        return gt_check_inertial_full(rec)
    if fn == "inertial_optimization_bias":
        return gt_check_inertial_bias(rec)
    if fn == "pose_inertial_lastkf":
        return gt_check_pose_inertial(rec)
    if fn != "inertial_optimization":
        print(f"compare.py: --gt-check is not defined for {fn!r} records",
              file=sys.stderr)
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
