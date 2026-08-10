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

// =============================================================================
// Restoration of the ORB-SLAM g2o fork's LM early-stop ("Stop criterium (Raul)").
// Reference (do not patch, read only):
//   Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp
//     :27-28  fork notice
//     :54     `_nBad = 0` (constructor)
//     :96     `_nBad = 0` inside `if (iteration == 0)` — the per-optimize() reset
//     :85     `iniChi = currentChi` (chi2 of the graph on entry)
//     :141    `currentChi = tempChi` on an ACCEPTED trial step
//     :152-155 early `return Terminate` (qmax exhausted / rho == 0) — this path
//              deliberately does NOT touch `_nBad`
//     :157-166 the criterion itself
//     :168    `return OK`
//
// WHY A WRAPPER AND NOT A FORK OF solve()
// ---------------------------------------
// The criterion only needs three things from solve(): the chi2 BEFORE the
// iteration, the chi2 AFTER it, and whether the iteration ended on the normal
// (non-Terminate) path. All three are observable from outside, so we call the
// upstream solve() and bracket it, instead of copying its ~120-line body and
// inheriting the maintenance burden of a second LM implementation.
//
// Semantic equivalence rests on one invariant of BOTH the vendored and the
// upstream loop: the tracked `currentChi` always equals the robust chi2 of the
// CURRENT graph estimate.
//   * entry:          currentChi = activeRobustChi2() of the entry estimate.
//   * accepted trial: `discardTop()` keeps the updated estimate and
//                     `currentChi = tempChi`, which was computed by
//                     `computeActiveErrors()` on exactly that estimate.
//   * rejected trial: `pop()` restores the pre-trial estimate and `currentChi`
//                     is left untouched — i.e. still the chi2 of that estimate.
// Therefore recomputing `computeActiveErrors(); activeRobustChi2();` right after
// solve() returns reproduces the vendored `currentChi` bit-for-bit: chi2 is a
// pure, deterministic function of the estimates (SparseOptimizer::
// computeActiveErrors just calls Edge::computeError per active edge;
// activeRobustChi2 just folds Edge::chi2 through the robust kernel), and
// push()/pop() restore estimates exactly.
//
// The same argument gives iniChi: computeActiveErrors() before calling solve()
// is idempotent with the computeActiveErrors() solve() itself performs first
// (buildStructure() in between only maps Hessian memory; it never touches an
// estimate), so our iniChi is bit-identical to the fork's line-85 `iniChi`.
//
// Early-Terminate parity: the criterion sits AFTER every non-OK return in the
// fork, so "skip the criterion whenever the result is not OK" is exactly the
// fork's control flow — `if (r != OK) return r;`. Upstream's Terminate test
// (`qmax == maxTrials || rho == 0 || !isfinite(_currentLambda)`) carries one
// extra disjunct over the fork's (`qmax == maxTrials || rho == 0`), but that
// disjunct belongs to the upstream loop we are calling, not to the criterion:
// a verbatim copy of upstream's solve() with the criterion spliced in at the
// fork's position would behave identically. The non-finite-lambda guard is a
// pre-existing, deliberate part of the P6-2 upstream migration, not something
// this file changes.
//
// COST: one extra computeActiveErrors() per LM iteration (O(#active edges)).
// It is unavoidable — the chi2 cached in the optimizer at solve() entry belongs
// to the last TRIAL of the previous iteration, which may have been rolled back,
// and is undefined before the first iteration.
// =============================================================================

#include "backend/OrbLevenberg.hpp"

#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"

namespace ORB_SLAM3
{

OrbLevenberg::OrbLevenberg(std::unique_ptr<g2o::Solver> solver)
    : g2o::OptimizationAlgorithmLevenberg(std::move(solver))
{
    // fork parity: `_nBad = 0` in the constructor (vendored :54).
}

OrbLevenberg::~OrbLevenberg() = default;

g2o::OptimizationAlgorithm::SolverResult OrbLevenberg::solve(int iteration, bool online)
{
    // P10-1 shadow-bridge entry refresh: the upstream retrial loop polls the
    // shadow through _forceStopFlag during this solve() call.
    refreshShadowStop();

    // fork parity: `_nBad = 0` inside `if (iteration == 0)` (vendored :96).
    // SparseOptimizer::optimize() always starts its loop at iteration 0, so the
    // counter is per-optimize() and never leaks across calls.
    if(iteration == 0)
        mnBad = 0;

    _optimizer->computeActiveErrors();
    const double iniChi = _optimizer->activeRobustChi2();

    const SolverResult result = g2o::OptimizationAlgorithmLevenberg::solve(iteration, online);

    // P10-1 shadow-bridge exit refresh: the outer terminate() check in
    // SparseOptimizer::optimize() reads the shadow right after solve() returns
    // (all return paths below).
    refreshShadowStop();

    if(result != OK)
        return result;   // Fail / early Terminate: the fork leaves _nBad alone here

    _optimizer->computeActiveErrors();
    const double currentChi = _optimizer->activeRobustChi2();

    // Stop criterium (Raul)
    if((iniChi - currentChi) * 1e3 < iniChi)
        mnBad++;
    else
        mnBad = 0;

    if(mnBad >= 3)
        return Terminate;

    return OK;
}

} // namespace ORB_SLAM3
