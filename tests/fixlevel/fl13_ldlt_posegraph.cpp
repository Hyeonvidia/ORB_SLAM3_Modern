/**
 * FL-13 LDLT pose-graph fixture (DIVERGENCES #13/#15; the vendored LDLT
 * solver was promoted to the unconditional essential-graph solver in R4a —
 * the LLT arm and the flag fork are gone).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure.
 *
 * THE CLIFF (historical): both OptimizeEssentialGraph overloads solve a
 * BlockSolver_7_3 Sim3 pose graph with setUserLambdaInit(1e-16) —
 * effectively undamped. The vendored ORB-SLAM g2o factorized with
 * Eigen::SimplicialLDLT (semidefinite/indefinite-tolerant); upstream
 * 20241228_git uses SimplicialLLT (strict SPD), whose refusal on a
 * gauge-poor graph is silent: every trial step is rejected inside one LM
 * iteration and optimize(20) returns positive having moved NOTHING (#15 —
 * hence the chi2-stall counter in Optimizer.cpp). Since R4a the vendored
 * LDLT shadow copy (include/backend/LinearSolverEigenLDLT.hpp) is THE
 * solver at both production sites.
 *
 * THE FIXTURE, built directly on g2o like the backend_equiv fixtures: a
 * two-vertex Sim3 graph with NO fixed vertex (the gauge-free shape of an
 * early loop) and one EdgeSim3 whose information matrix carries a small
 * NEGATIVE eigenvalue on the scale row (diag(1,...,1,-1e-4)) — a
 * deterministic stand-in for the field mechanism (rounding-induced
 * indefiniteness of the gauge-null pivots). LDLT factorizes the matrix
 * (nonzero pivots of either sign) and the step is dominated by the
 * well-posed rows, so the translation residual converges.
 *
 * The solver construction below reproduces the production site's shape —
 * LinearSolverEigenLDLT, the OrbLevenberg wrapper, setBlockOrdering(false),
 * and setUserLambdaInit(1e-16) — so the path exercised here is the same
 * path Optimizer.cpp takes (OptimizeEssentialGraph itself needs
 * Maps/KeyFrames a unit test cannot host).
 *
 * Assertion: the pose graph MOVES on the indefinite fixture and the
 * well-posed residual is gone — the fix property, now the only path.
 */

#include "backend/LinearSolverEigenLDLT.hpp"
#include "backend/OrbLevenberg.hpp"

#include "g2o/core/block_solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

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
// mirrors src/backend/Optimizer.cpp::OptimizeEssentialGraph verbatim.
RunResult RunOnce()
{
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    auto linearSolver = std::make_unique<
        g2o::LinearSolverEigenLDLT<g2o::BlockSolver_7_3::PoseMatrixType>>();
    linearSolver->setBlockOrdering(false);
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

int main()
{
    const RunResult r = RunOnce();
    std::fprintf(stderr,
                 "ldlt: chi2 %.6e -> %.6e  trans_err=%.3e\n",
                 r.chi2Before, r.chi2After, r.transErrAfter);
    CHECK(r.chi2Before > 1e-2);  // the perturbation is visible
    // The fix property: the pose graph MOVES on the indefinite fixture and
    // the well-posed residual is gone.
    CHECK(r.chi2After < 1e-2 * r.chi2Before);
    CHECK(r.transErrAfter < 1e-4);
    std::fprintf(stderr, "ok  ldlt_solves_indefinite_fixture\n");

    std::fprintf(stderr, "fl13_ldlt_posegraph: ALL PASS\n");
    return 0;
}
