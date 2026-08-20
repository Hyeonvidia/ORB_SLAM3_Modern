/**
 * FL-9 scale-convergence property test (DIVERGENCES #9; chain-rule fix
 * promoted to unconditional in R4a — this is now a pure convergence
 * property test, no flag phases).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure.
 *
 * The historical bug: EdgeInertialGS::linearizeOplus shipped the ADDITIVE
 * scale Jacobian dr/ds while VertexScale applies a MULTIPLICATIVE update
 * s <- s*exp(w), so the GN fixed-point iteration was s <- s*exp(s_true - s):
 * contracting only for s_true < 2, marginally stable (stall) at exactly 2.0,
 * divergent above. The chain-rule fix (dr/dw = dr/ds * s), unconditional
 * since R4a, makes the fixed point superlinear (multiplier 0 at the optimum)
 * for any s_true.
 *
 * What is asserted, on the backend_equiv imu_chain fixture (kScaleGravity)
 * with the sTrueOverride, driving the production
 * Optimizer::InertialOptimization(Map*, Rwg&, scale&) from the canonical
 * (Rwg = I, scale = 1) start — ONE call = the fixed 10 GN iterations,
 * exactly LocalMapping::ScaleRefinement's usage:
 *
 *   (a) s_true=2.0: converges to ground truth — THE fix property (the
 *       upstream additive Jacobian stalled here at ~2.3e-2 rel error);
 *   (b) s_true=1.1: converges (no regression in the contracting regime
 *       the production ScaleRefinement path lives in).
 *
 * Convergence gate: rel |s - s_true|/s_true < 1e-6 and gravity-direction
 * angle error < 1e-6 rad — same floor class as the backend_equiv GT gate
 * (1e-7 with measured 1.57e-08 at s_true=1.1; the float32-stored fixture
 * states set the floor, see EquivFixtures.hpp).
 */

#include "EquivFixtures.hpp"

#include "backend/Optimizer.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

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

}  // namespace

int main()
{
    // (a) s_true = 2.0 — the fix property: the upstream additive Jacobian
    // stalled exactly here; the chain rule must converge to ground truth.
    {
        const RunResult r = RunOnce(2.0);
        std::fprintf(stderr,
                     "s=2.0: rel_scale_err=%.3e gdir_err=%.3e rad\n",
                     r.scaleRelErr, r.gdirAngleErr);
        CHECK(r.scaleRelErr < kConvergedScaleRelTol);
        CHECK(r.gdirAngleErr < kConvergedGdirTol);
        std::fprintf(stderr, "ok  a_s2.0_converges (fix property)\n");
    }

    // (b) s_true = 1.1 — no regression in the contracting regime.
    {
        const RunResult r = RunOnce(1.1);
        std::fprintf(stderr,
                     "s=1.1: rel_scale_err=%.3e gdir_err=%.3e rad\n",
                     r.scaleRelErr, r.gdirAngleErr);
        CHECK(r.scaleRelErr < kConvergedScaleRelTol);
        CHECK(r.gdirAngleErr < kConvergedGdirTol);
        std::fprintf(stderr, "ok  b_s1.1_converges\n");
    }

    std::fprintf(stderr, "fl9_scale_convergence: ALL PASS\n");
    return 0;
}
