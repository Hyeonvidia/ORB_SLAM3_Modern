/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORB_SLAM3_BACKEND_OPTIMIZERCOMMON_HPP
#define ORB_SLAM3_BACKEND_OPTIMIZERCOMMON_HPP

// -----------------------------------------------------------------------------
// R5: shared internals of the Optimizer function-family TUs (OptimizerGlobal /
// OptimizerPose / OptimizerLocal / OptimizerInertialInit / OptimizerSim3Graph,
// all split out of the former ~5,800-line monolithic Optimizer.cpp).
// Deliberately lives in src/backend/, not include/backend/:
// include/backend/Optimizer.hpp remains the single public declaration surface.
// -----------------------------------------------------------------------------

// R4b slice 1 (pin-set convention): every optimizer function pins the
// MapPoints it touches by holding them in local std::vector/set/map of
// MapPointPtr built ONCE at graph-construction time (GetAllMapPoints /
// GetMapPointMatches copies and the per-edge side tables such as
// vpMapPointEdge*). Refcounting therefore happens only at graph build and
// recovery — never inside the per-iteration LM loops, which run on raw g2o
// vertices. Do not introduce shared_ptr copies inside per-iteration loops.

#include "core/Verbose.hpp"  // P7-1b: was transitive via Tracking.hpp -> System.hpp

#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "g2o/core/sparse_optimizer.h"  // R5: explicit (was transitive in the monolithic TU)
#include "g2o/core/sparse_block_matrix.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/dense/linear_solver_dense.h"

#include <memory>
#include <optional>
#include <utility>
#include "backend/G2oTypes.hpp"
#include "backend/LinearSolverEigenLDLT.hpp"  // vendored LDLT solver for the essential-graph sites (DIVERGENCES #13)
#include "backend/OrbLevenberg.hpp"
#include "io/Converter.hpp"

#include<mutex>

#include "backend/OptimizableTypes.hpp"

// -----------------------------------------------------------------------------
// g2o-fork parity (docs/DIVERGENCES.md items 10 & 11). The vendored
// Thirdparty/g2o was Raúl Mur-Artal's ORB-SLAM fork, and the P6-2 migration to
// upstream 20241228_git silently dropped two of its deliberate design decisions:
//   F1  LM early-stop ("Stop criterium (Raul)") -> restored by constructing
//       ORB_SLAM3::OrbLevenberg instead of g2o::OptimizationAlgorithmLevenberg
//       at every LM site below (14 sites; the 3 GaussNewton sites are untouched,
//       the fork never modified GN).
//   F2  LinearSolverEigen block ordering: fork default false (scalar AMD),
//       upstream default true (block AMD, via LinearSolverCCS) -> restored by an
//       explicit setBlockOrdering(false) at every LinearSolverEigen site
//       (12 sites). LinearSolverDense derives straight from LinearSolver<M> and
//       has no ordering knob at all, so the 4 dense sites are untouched.
// The pinned submodule is never patched; both restorations live in OUR code.
// -----------------------------------------------------------------------------

namespace ORB_SLAM3
{

// -----------------------------------------------------------------------------
// Failure observability for the g2o backend (docs/DIVERGENCES.md items 13 & 15).
//
// OBSERVABILITY ONLY: nothing below changes an estimate, an iteration count, a
// threshold or the control flow. It only reads what optimize() reported and
// prints to std::cerr (System forces Verbose::VERBOSITY_QUIET, so
// Verbose::PrintMess would be swallowed; stderr also keeps the
// tests/backend_equiv record stream on stdout clean).
//
// RETURN-VALUE SEMANTICS of SparseOptimizer::optimize() at the pinned tag
// third_party/g2o @ 20241228_git (core/sparse_optimizer.cpp:393-455):
//
//   < 0  (only ever -1)  HARD FAILURE, zero iterations ran, estimates are
//        bit-identical to the input. Two causes:
//          * `_ivMap.size() == 0` — no active vertices (initializeOptimization()
//            not called, or it selected an empty active set);
//          * `_algorithm->init(online)` returned false.
//        Both log via G2O_WARN/G2O_ERROR, which G2O_USE_LOGGING=OFF compiles to
//        a no-op — hence item 15: without this helper the caller writes the
//        UNCHANGED estimates back into the map and reports success.
//
//   == 0  Two disjoint causes, distinguished below:
//          (a) the loop body never executed — `iterations <= 0`, or
//              `terminate()` was already true on entry, i.e. the caller's
//              force-stop flag was set before the call. This is a LEGITIMATE
//              abort (LocalMapping raises that flag whenever a new KF arrives)
//              and must not warn;
//          (b) the last executed iteration returned SolverResult::Fail
//              (`optimization_algorithm.h:48`, Fail = -1) — CCS structure build
//              failure in LM/GN, or a failed linear solve in GaussNewton. Real
//              failure -> warn.
//        We separate them by asking the optimizer whether its force-stop flag is
//        currently raised; when it is, we stay silent (conservative: a genuine
//        Fail that coincides with a stop request is not reported, which is the
//        right trade for a per-KF path).
//
//   > 0  The number of iterations that actually ran. LESS THAN REQUESTED IS
//        NORMAL and must NOT warn: it is either the restored ORB-SLAM LM
//        early-stop (src/backend/OrbLevenberg.cpp -> SolverResult::Terminate,
//        item 10) or a mid-run force-stop. Never a failure indication.
//
// LIMITATION worth stating, because it is exactly item 13: an Eigen
// SimplicialLLT refusal inside LM does NOT surface in the return value at all.
// `_solver.solve()` returning false is treated by LM as a rejected trial step
// (tempChi = DBL_MAX, lambda inflated); after `maxTrialsAfterFailure` trials LM
// returns Terminate, so optimize() returns a POSITIVE count while the estimate
// never moved. The return code cannot see it — only a chi2 comparison can,
// which is why the two OptimizeEssentialGraph overloads (the setUserLambdaInit
// (1e-16) BlockSolver_7_3 pose graphs, run once per loop closure) additionally
// bracket the call with ReportIfPoseGraphStalled().
// -----------------------------------------------------------------------------
// Definitions in OptimizerCommon.cpp — moved verbatim from the pre-R5
// monolithic Optimizer.cpp anonymous namespace (now shared across the
// function-family TUs, hence external linkage).
int RunOptimization(g2o::SparseOptimizer& optimizer, int its, const char* where, int pass = -1);
void ReportIfPoseGraphStalled(const char* where, int nRan, double chi2Before, double chi2After);

// -----------------------------------------------------------------------------
// R5 g2o graph-builder helpers: sugar over the exact construction sequences the
// monolithic Optimizer.cpp repeated at every site — same allocations, same call
// order, same values. Each site's solver/lambda configuration is preserved via
// the template/argument choices; NOTHING numeric is decided in here. The goal
// is that every optimizer function reads as the four-section skeleton
// "vertices -> edges -> solve -> recover" at a glance.
// -----------------------------------------------------------------------------

// LM over LinearSolverEigen (scalar-AMD ordering, F2 above) wrapped in
// BlockSolverT; algorithm = OrbLevenberg (F1). Returns the algorithm so the
// site can bind the abort flag or tune lambda afterwards (g2o's optimizer
// keeps the pointer; lifetime as before — allocated per optimization).
// lambdaInit empty = g2o's own initial-lambda heuristic, exactly the sites
// that never called setUserLambdaInit.
template <typename BlockSolverT>
OrbLevenberg* MakeLmOptimizerEigen(g2o::SparseOptimizer& optimizer,
                                   std::optional<double> lambdaInit = std::nullopt)
{
    auto linearSolver = std::make_unique<g2o::LinearSolverEigen<typename BlockSolverT::PoseMatrixType>>();
    linearSolver->setBlockOrdering(false);  // fork parity: vendored ORB-SLAM g2o default (docs/DIVERGENCES.md 11)
    auto solver_ptr = std::make_unique<BlockSolverT>(std::move(linearSolver));
    OrbLevenberg* solver = new OrbLevenberg(std::move(solver_ptr));
    if(lambdaInit)
        solver->setUserLambdaInit(*lambdaInit);
    optimizer.setAlgorithm(solver);
    return solver;
}

// Essential-graph twin of MakeLmOptimizerEigen: the vendored LDLT linear
// solver (semidefinite-tolerant, DIVERGENCES #13) instead of upstream's
// strict-SPD SimplicialLLT. Both pose-graph sites run effectively undamped
// (lambdaInit 1e-16), which is why LLT's silent refusal mattered there.
template <typename BlockSolverT>
OrbLevenberg* MakeLmOptimizerEigenLDLT(g2o::SparseOptimizer& optimizer, double lambdaInit)
{
    auto linearSolver = std::make_unique<g2o::LinearSolverEigenLDLT<typename BlockSolverT::PoseMatrixType>>();
    linearSolver->setBlockOrdering(false);  // fork parity: vendored ORB-SLAM g2o default (docs/DIVERGENCES.md 11)
    auto solver_ptr = std::make_unique<BlockSolverT>(std::move(linearSolver));
    OrbLevenberg* solver = new OrbLevenberg(std::move(solver_ptr));
    solver->setUserLambdaInit(lambdaInit);
    optimizer.setAlgorithm(solver);
    return solver;
}

// LM over LinearSolverDense (no ordering knob at all — F2 does not apply).
template <typename BlockSolverT>
OrbLevenberg* MakeLmOptimizerDense(g2o::SparseOptimizer& optimizer)
{
    auto linearSolver = std::make_unique<g2o::LinearSolverDense<typename BlockSolverT::PoseMatrixType>>();
    auto solver_ptr = std::make_unique<BlockSolverT>(std::move(linearSolver));
    OrbLevenberg* solver = new OrbLevenberg(std::move(solver_ptr));
    optimizer.setAlgorithm(solver);
    return solver;
}

// GaussNewton over LinearSolverDense — the fork never modified GN (F1 note),
// so these sites use the upstream algorithm directly.
template <typename BlockSolverT>
g2o::OptimizationAlgorithmGaussNewton* MakeGnOptimizerDense(g2o::SparseOptimizer& optimizer)
{
    auto linearSolver = std::make_unique<g2o::LinearSolverDense<typename BlockSolverT::PoseMatrixType>>();
    auto solver_ptr = std::make_unique<BlockSolverT>(std::move(linearSolver));
    auto* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    optimizer.setAlgorithm(solver);
    return solver;
}

// GaussNewton over LinearSolverEigen (scalar-AMD, F2) — the
// InertialOptimization(Rwg,scale) site.
template <typename BlockSolverT>
g2o::OptimizationAlgorithmGaussNewton* MakeGnOptimizerEigen(g2o::SparseOptimizer& optimizer)
{
    auto linearSolver = std::make_unique<g2o::LinearSolverEigen<typename BlockSolverT::PoseMatrixType>>();
    linearSolver->setBlockOrdering(false);  // fork parity: vendored ORB-SLAM g2o default (docs/DIVERGENCES.md 11)
    auto solver_ptr = std::make_unique<BlockSolverT>(std::move(linearSolver));
    auto* solver = new g2o::OptimizationAlgorithmGaussNewton(std::move(solver_ptr));
    optimizer.setAlgorithm(solver);
    return solver;
}

// P10-1 shadow bridge: g2o polls a plain bool*; hand it the solver-local
// shadow refreshed from the atomic inside OrbLevenberg::solve(). No-op with a
// null flag — exactly the old `if(pbStopFlag) { bindAbortFlag;
// setForceStopFlag(shadowPtr) }` site blocks. Call-site PLACEMENT is
// deliberately preserved per site (LocalInertialBA binds only after its
// solve — an upstream ordering quirk kept verbatim).
inline void BindAbortFlag(g2o::SparseOptimizer& optimizer, OrbLevenberg* solver,
                          const std::atomic<bool>* pbStopFlag)
{
    if(!pbStopFlag)
        return;
    solver->bindAbortFlag(pbStopFlag);
    optimizer.setForceStopFlag(solver->shadowPtr());
}

// Attach a fresh Huber robust kernel with the site's delta (the ubiquitous
// new/setRobustKernel/setDelta triple).
inline void AddHuberKernel(g2o::OptimizableGraph::Edge* e, double delta)
{
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    e->setRobustKernel(rk);
    rk->setDelta(delta);
}

// SE3 keyframe vertex at the KF's current pose; id = pKF->mnId.
inline g2o::VertexSE3Expmap* AddSE3Vertex(g2o::SparseOptimizer& optimizer,
                                          const KeyFramePtr& pKF, bool bFixed)
{
    g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();
    Sophus::SE3<float> Tcw = pKF->GetPose();
    vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(), Tcw.translation().cast<double>()));
    vSE3->setId(pKF->mnId);
    vSE3->setFixed(bFixed);
    optimizer.addVertex(vSE3);
    return vSE3;
}

// Marginalized XYZ landmark vertex at the MapPoint's current position. The
// caller keeps computing the id — every function has its own id layout, and
// hiding that arithmetic would obscure the vertex-id contract.
inline g2o::VertexPointXYZ* AddLandmarkVertex(g2o::SparseOptimizer& optimizer,
                                              const MapPointPtr& pMP, int id)
{
    g2o::VertexPointXYZ* vPoint = new g2o::VertexPointXYZ();
    vPoint->setEstimate(pMP->GetWorldPos().cast<double>());
    vPoint->setId(id);
    vPoint->setMarginalized(true);
    optimizer.addVertex(vPoint);
    return vPoint;
}

// Inertial keyframe vertex block: VertexPose at id mnId, plus (when the KF
// carries IMU state) VertexVelocity/VertexGyroBias/VertexAccBias at the
// standard maxKFid+3*mnId+{1,2,3} id layout — all sharing one fixed flag.
// (FullInertialBA keeps its own bespoke loop: per-vertex fixing via BAEpochs
// and the bInit shared-bias variant do not fit this shape.)
inline void AddInertialKFVertices(g2o::SparseOptimizer& optimizer, const KeyFramePtr& pKFi,
                                  unsigned long maxKFid, bool bFixed)
{
    VertexPose* VP = new VertexPose(pKFi);
    VP->setId(pKFi->mnId);
    VP->setFixed(bFixed);
    optimizer.addVertex(VP);

    if(pKFi->bImu)
    {
        VertexVelocity* VV = new VertexVelocity(pKFi);
        VV->setId(maxKFid+3*(pKFi->mnId)+1);
        VV->setFixed(bFixed);
        optimizer.addVertex(VV);
        VertexGyroBias* VG = new VertexGyroBias(pKFi);
        VG->setId(maxKFid+3*(pKFi->mnId)+2);
        VG->setFixed(bFixed);
        optimizer.addVertex(VG);
        VertexAccBias* VA = new VertexAccBias(pKFi);
        VA->setId(maxKFid+3*(pKFi->mnId)+3);
        VA->setFixed(bFixed);
        optimizer.addVertex(VA);
    }
}

} //namespace ORB_SLAM

#endif // ORB_SLAM3_BACKEND_OPTIMIZERCOMMON_HPP
