/**
 * P11-F1 FL-9 scale-convergence property test (DIVERGENCES #9,
 * Fix.ScaleJacobian; docs/P11_RECON.md 2부 FL-9).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure.
 *
 * The bug: EdgeInertialGS::linearizeOplus ships the ADDITIVE scale Jacobian
 * dr/ds while VertexScale applies a MULTIPLICATIVE update s <- s*exp(w), so
 * the GN fixed-point iteration is s <- s*exp(s_true - s): contracting only
 * for s_true < 2, marginally stable (stall) at exactly 2.0, divergent above.
 * The chain-rule fix (dr/dw = dr/ds * s, armed by Fix.ScaleJacobian) makes
 * the fixed point superlinear (multiplier 0 at the optimum) for any s_true.
 *
 * What is asserted, on the backend_equiv imu_chain fixture (kScaleGravity)
 * with the P11-F1 sTrueOverride, driving the production
 * Optimizer::InertialOptimization(Map*, Rwg&, scale&) from the canonical
 * (Rwg = I, scale = 1) start — ONE call = the fixed 10 GN iterations,
 * exactly LocalMapping::ScaleRefinement's usage:
 *
 *   (a) OFF s_true=1.1: converges (the OFF contract of the backend_equiv
 *       twin-equivalence pair, re-checked here as the phase baseline);
 *   (b) OFF s_true=2.0: STALLS — relative scale error stays above 5e-3
 *       (P6-3 measured the stall at ~2.3e-2). This is the #9 bug
 *       reproduced; if a change makes this converge with the flag OFF,
 *       the OFF contract drifted and this check fails the build;
 *   (c) ON  s_true=2.0: converges to ground truth — THE fix property;
 *   (d) ON  s_true=1.1: still converges (no regression in the contracting
 *       regime the production ScaleRefinement path lives in).
 *
 * Convergence gate: rel |s - s_true|/s_true < 1e-6 and gravity-direction
 * angle error < 1e-6 rad — same floor class as the backend_equiv GT gate
 * (1e-7 with measured 1.57e-08 at s_true=1.1; the float32-stored fixture
 * states set the floor, see EquivFixtures.hpp).
 *
 * ON semantics are armed through the PUBLIC write-once installer
 * FixFlags::Set, called ONCE between the OFF and ON phases: this binary is
 * single-threaded and never constructs a System, so the header's
 * Set-before-any-SLAM-thread contract holds trivially, and no test-only
 * ctor parameter or setter has to be added to the production edge (the
 * least invasive wiring — the edge caches the flag at construction, and
 * every optimizer call below constructs fresh edges). The frozen reference
 * backend cannot arm the flag, which is why this test lives here and not
 * in the backend_equiv variant matrix.
 */

#include "EquivFixtures.hpp"

#include "backend/Optimizer.hpp"
#include "core/FixFlags.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using ORB_SLAM3::FixFlags;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #cond,        \
                         __FILE__, __LINE__);                                \
            std::_Exit(1);                                                   \
        }                                                                    \
    } while (0)

namespace {

struct RunResult {
    double scaleRelErr;
    double gdirAngleErr;  // rad
};

// One production InertialOptimization(Map*, Rwg&, scale&) call from the
// canonical (I, 1.0) start on a fresh s_true-override fixture.
RunResult RunOnce(double sTrue)
{
    equiv::ImuChainFixture fx = equiv::MakeImuChainFixture(
        equiv::ImuChainVariant::kScaleGravity, sTrue);

    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();
    double scale = 1.0;
    ORB_SLAM3::Optimizer::InertialOptimization(fx.map.get(), Rwg, scale);

    RunResult r;
    r.scaleRelErr = std::abs(scale - fx.sTrue) / fx.sTrue;
    r.gdirAngleErr = std::acos(std::min(
        1.0, (Rwg * Eigen::Vector3d(0, 0, -1)).dot(fx.gDirTrue)));
    return r;
}

constexpr double kConvergedScaleRelTol = 1e-6;   // measured floor ~1.6e-8
constexpr double kConvergedGdirTol = 1e-6;       // rad
constexpr double kStallScaleRelMin = 5e-3;       // P6-3 measured stall ~2.3e-2

}  // namespace

int main()
{
    // ---- OFF phase: the untouched process-global default is level 0 ----
    CHECK(!FixFlags::I().scaleJacobianChainRule);

    // (a) OFF, s_true = 1.1 — contracting regime, must converge (OFF
    // contract baseline, same property the backend_equiv GT gate pins).
    {
        const RunResult r = RunOnce(1.1);
        std::fprintf(stderr,
                     "off s=1.1: rel_scale_err=%.3e gdir_err=%.3e rad\n",
                     r.scaleRelErr, r.gdirAngleErr);
        CHECK(r.scaleRelErr < kConvergedScaleRelTol);
        CHECK(r.gdirAngleErr < kConvergedGdirTol);
        std::fprintf(stderr, "ok  a_off_s1.1_converges\n");
    }

    // (b) OFF, s_true = 2.0 — the #9 stall reproduced: NOT converged.
    {
        const RunResult r = RunOnce(2.0);
        std::fprintf(stderr,
                     "off s=2.0: rel_scale_err=%.3e gdir_err=%.3e rad\n",
                     r.scaleRelErr, r.gdirAngleErr);
        CHECK(r.scaleRelErr > kStallScaleRelMin);
        std::fprintf(stderr, "ok  b_off_s2.0_stalls (bug reproduced)\n");
    }

    // ---- ON phase: arm Fix.ScaleJacobian via the write-once installer ----
    {
        FixFlags f;
        f.scaleJacobianChainRule = true;
        FixFlags::Set(f);
        CHECK(FixFlags::I().scaleJacobianChainRule);
    }

    // (c) ON, s_true = 2.0 — the fix property: converges to ground truth.
    {
        const RunResult r = RunOnce(2.0);
        std::fprintf(stderr,
                     "on  s=2.0: rel_scale_err=%.3e gdir_err=%.3e rad\n",
                     r.scaleRelErr, r.gdirAngleErr);
        CHECK(r.scaleRelErr < kConvergedScaleRelTol);
        CHECK(r.gdirAngleErr < kConvergedGdirTol);
        std::fprintf(stderr, "ok  c_on_s2.0_converges (fix property)\n");
    }

    // (d) ON, s_true = 1.1 — no regression in the contracting regime.
    {
        const RunResult r = RunOnce(1.1);
        std::fprintf(stderr,
                     "on  s=1.1: rel_scale_err=%.3e gdir_err=%.3e rad\n",
                     r.scaleRelErr, r.gdirAngleErr);
        CHECK(r.scaleRelErr < kConvergedScaleRelTol);
        CHECK(r.gdirAngleErr < kConvergedGdirTol);
        std::fprintf(stderr, "ok  d_on_s1.1_converges\n");
    }

    std::fprintf(stderr, "fl9_scale_convergence: ALL PASS\n");
    return 0;
}
