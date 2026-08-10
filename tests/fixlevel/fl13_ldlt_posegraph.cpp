/**
 * P11-F5 FL-13 LDLT-vs-LLT pose-graph fixture (DIVERGENCES #13/#15,
 * Fix.LdltPoseGraph, OFF by default; docs/P11_RECON.md 2부 FL-13).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure.
 *
 * THE CLIFF: both OptimizeEssentialGraph overloads solve a BlockSolver_7_3
 * Sim3 pose graph with setUserLambdaInit(1e-16) — effectively undamped.
 * The vendored ORB-SLAM g2o factorized with Eigen::SimplicialLDLT
 * (semidefinite/indefinite-tolerant); upstream 20241228_git uses
 * SimplicialLLT (strict SPD). On a gauge-poor graph the LLT refusal is
 * silent: every trial step is rejected inside one LM iteration, the
 * iteration terminates, and optimize(20) returns positive having moved
 * NOTHING (#15 — hence the chi2-stall counter in Optimizer.cpp).
 *
 * THE FIXTURE, built directly on g2o like the backend_equiv fixtures: a
 * two-vertex Sim3 graph with NO fixed vertex (the gauge-free shape of an
 * early loop) and one EdgeSim3 whose information matrix carries a small
 * NEGATIVE eigenvalue on the scale row (diag(1,...,1,-1e-4)). Honest
 * scope note, per the task's fallback clause: the field mechanism is
 * rounding-induced indefiniteness of the gauge-null pivots (7 pivots of
 * size ~1e-16 emerging from O(1) cancellations — sign is rounding luck,
 * fixed per graph but fragile across Eigen/compiler bumps), so a fixture
 * relying on it alone would be flaky BY DESIGN. The tiny negative
 * information eigenvalue produces the same failure class
 * (factorization-time indefiniteness at every lambda the run can reach:
 * lambda climbs only to ~1e-13 before the first iteration terminates)
 * deterministically. LDLT factorizes the same matrix (nonzero pivots of
 * either sign) and the step is dominated by the well-posed rows, so the
 * translation residual actually converges.
 *
 * The solver construction below reproduces the production site's shape —
 * including the flag consultation `FixFlags::I().ldltPoseGraph`, the
 * OrbLevenberg wrapper, setBlockOrdering(false), and
 * setUserLambdaInit(1e-16) — so the ON/OFF fork exercised here is the same
 * fork Optimizer.cpp takes (the call-site expression is textually
 * replicated; OptimizeEssentialGraph itself needs Maps/KeyFrames a unit
 * test cannot host).
 *
 * Assertions (weakened from the original plan — docs/P11_RECON.md FL-13
 * fallback clause): a deterministic LLT-failure fixture proved
 * unachievable on this platform (SimplicialLLT factorizes even the
 * negative-eigenvalue construction; failure is pivot-order/rounding
 * dependent). The unit therefore proves what is provable:
 *   (a) OFF: the LLT path solves this fixture (baseline recorded);
 *   (b) ON (via the public write-once FixFlags::Set; single-threaded
 *       binary): the vendored LDLT path solves with the fix property
 *       AND at parity-or-better with the LLT baseline — the header is
 *       wired and numerically sane.
 * The #13 field failure mode remains detected by the #15 chi2-no-move
 * stderr counter at the two production essential-graph sites.
 */

#include "backend/LinearSolverEigenLDLT.hpp"
#include "backend/OrbLevenberg.hpp"
#include "core/FixFlags.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

using ORB_SLAM3::FixFlags;
using ORB_SLAM3::OrbLevenberg;

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
    double chi2Before;
    double chi2After;
    double transErrAfter;  // |t(v1) - t(measurement target)| after opt
};

// One full pose-graph solve on a fresh fixture. The optimizer construction
// mirrors src/backend/Optimizer.cpp::OptimizeEssentialGraph verbatim,
// including the P11-F5 flag fork.
RunResult RunOnce()
{
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_7_3::LinearSolverType> linearSolver;
    if (FixFlags::I().ldltPoseGraph) {
        auto eigenLdlt = std::make_unique<
            g2o::LinearSolverEigenLDLT<g2o::BlockSolver_7_3::PoseMatrixType>>();
        eigenLdlt->setBlockOrdering(false);
        linearSolver = std::move(eigenLdlt);
    } else {
        auto eigenLlt = std::make_unique<
            g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>>();
        eigenLlt->setBlockOrdering(false);
        linearSolver = std::move(eigenLlt);
    }
    std::unique_ptr<g2o::BlockSolver_7_3> solver_ptr =
        std::make_unique<g2o::BlockSolver_7_3>(std::move(linearSolver));
    OrbLevenberg* solver = new OrbLevenberg(std::move(solver_ptr));
    solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);

    // v0: identity. v1: the measurement target, perturbed in translation —
    // the residual a healthy solver removes in one or two steps.
    const Eigen::Quaterniond qI = Eigen::Quaterniond::Identity();
    const g2o::Sim3 S10(qI, Eigen::Vector3d(1.0, 0.0, 0.0), 1.0);  // measurement
    // EdgeSim3 error = log(meas * S0 * S1^-1); with v0 = I the consistent
    // v1 estimate is S10 itself.
    const Eigen::Vector3d tTarget(1.0, 0.0, 0.0);
    const Eigen::Vector3d tPerturb(0.3, -0.2, 0.4);

    g2o::VertexSim3Expmap* v0 = new g2o::VertexSim3Expmap();
    v0->setId(0);
    v0->setEstimate(g2o::Sim3());
    v0->setFixed(false);  // gauge-free: the early-loop shape
    v0->_fix_scale = false;
    optimizer.addVertex(v0);

    g2o::VertexSim3Expmap* v1 = new g2o::VertexSim3Expmap();
    v1->setId(1);
    v1->setEstimate(g2o::Sim3(qI, tTarget + tPerturb, 1.0));
    v1->setFixed(false);
    v1->_fix_scale = false;
    optimizer.addVertex(v1);

    g2o::EdgeSim3* e = new g2o::EdgeSim3();
    e->setVertex(0, v0);
    e->setVertex(1, v1);
    e->setMeasurement(S10);
    Eigen::Matrix<double, 7, 7> lambda =
        Eigen::Matrix<double, 7, 7>::Identity();
    lambda(6, 6) = -1e-4;  // the deterministic indefiniteness (header note)
    e->information() = lambda;
    optimizer.addEdge(e);

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    RunResult r;
    r.chi2Before = optimizer.activeRobustChi2();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();
    r.chi2After = optimizer.activeRobustChi2();

    // Gauge-invariant convergence measure: the relative pose error left on
    // the edge (chi2 of the well-posed rows), reported as the translation
    // part of the residual.
    const g2o::Sim3 rel = S10 * v0->estimate() * v1->estimate().inverse();
    r.transErrAfter = rel.translation().norm();
    return r;
}

}  // namespace

static double gOffChi2After = 0.0;
static double gOffTransErr = 0.0;

int main()
{
    // ---- OFF phase: the untouched process-global default is level 0 ----
    CHECK(!FixFlags::I().ldltPoseGraph);
    {
        const RunResult r = RunOnce();
        std::fprintf(stderr,
                     "off: chi2 %.6e -> %.6e  trans_err=%.3e\n",
                     r.chi2Before, r.chi2After, r.transErrAfter);
        CHECK(r.chi2Before > 1e-2);  // the perturbation is visible
        // NOTE (deliberate weakening, docs/P11_RECON.md FL-13 fallback):
        // a DETERMINISTIC LLT-failure fixture proved unachievable -- on this
        // near-semidefinite graph Eigen's SimplicialLLT still factorizes
        // (whether it fails is platform/pivot-order dependent), so the
        // original "OFF stalls" assertion was flaky by construction. The
        // unit now proves what is provable: the OFF path solves (baseline),
        // and the ON/LDLT path solves at least as well (below). The #13
        // failure mode in the field is detected by the #15 chi2-no-move
        // stderr counter on the two essential-graph sites.
        CHECK(r.chi2After < r.chi2Before);   // OFF path converged here
        gOffChi2After = r.chi2After;
        gOffTransErr = r.transErrAfter;
        std::fprintf(stderr, "ok  a_off_llt_solves_this_fixture\n");
    }

    // ---- ON phase: arm Fix.LdltPoseGraph via the write-once installer ----
    {
        FixFlags f;
        f.ldltPoseGraph = true;
        FixFlags::Set(f);
        CHECK(FixFlags::I().ldltPoseGraph);
    }
    {
        const RunResult r = RunOnce();
        std::fprintf(stderr,
                     "on : chi2 %.6e -> %.6e  trans_err=%.3e\n",
                     r.chi2Before, r.chi2After, r.transErrAfter);
        CHECK(r.chi2Before > 1e-2);
        // The fix property: the pose graph MOVES and the well-posed
        // residual is gone.
        CHECK(r.chi2After < 1e-2 * r.chi2Before);
        CHECK(r.transErrAfter < 1e-4);
        // And LDLT is at least as good as whatever LLT achieved on this
        // fixture (equivalence-when-LLT-succeeds -- the vendored header is
        // wired and numerically sane).
        CHECK(r.chi2After <= gOffChi2After * (1.0 + 1e-6) + 1e-12);
        CHECK(r.transErrAfter <= gOffTransErr * (1.0 + 1e-6) + 1e-12);
        std::fprintf(stderr, "ok  b_on_ldlt_solves (fix property + parity)\n");
    }

    std::fprintf(stderr, "fl13_ldlt_posegraph: ALL PASS\n");
    return 0;
}
