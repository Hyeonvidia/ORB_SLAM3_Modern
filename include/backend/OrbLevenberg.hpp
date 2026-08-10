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

#ifndef ORB_SLAM3_BACKEND_ORBLEVENBERG_HPP
#define ORB_SLAM3_BACKEND_ORBLEVENBERG_HPP

#include <atomic>
#include <memory>

#include "g2o/core/optimization_algorithm_levenberg.h"

namespace g2o { class Solver; }

namespace ORB_SLAM3
{

/**
 * @brief Levenberg-Marquardt with ORB-SLAM's "Stop criterium (Raul)".
 *
 * The vendored Thirdparty/g2o is Raúl Mur-Artal's ORB-SLAM fork, not stock g2o
 * ("Modified Raul Mur Artal (2014) - Stop criterium (solve function)",
 * Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp:27-28). The fork
 * adds an early-stop to solve(): when the RELATIVE robust-chi2 improvement of an
 * accepted LM iteration stays below 0.1% for three CONSECUTIVE iterations, solve()
 * returns Terminate, which makes SparseOptimizer::optimize() break out of its
 * iteration loop early (sparse_optimizer.cpp:415/428 — `ok = (result == OK)`).
 *
 * Upstream g2o 20241228_git has no `_nBad`, so the P6-2 migration silently made
 * every LM optimization burn its full requested iteration budget. That is a
 * behavior change on two axes: (a) the extra steps move the estimate, and
 * (b) LocalBA wall-clock grows, which shifts the KF insertion rate of the
 * LocalMapping thread. The criterion is a deliberate ORB-SLAM design decision,
 * so the behavior-preservation policy requires restoring it in OUR code — the
 * pinned submodule is never patched (docs/DIVERGENCES.md item 10).
 *
 * Implemented as a thin wrapper around the upstream solve() rather than a fork of
 * its ~120-line body; see OrbLevenberg.cpp for the semantic-equivalence argument.
 */
class OrbLevenberg : public g2o::OptimizationAlgorithmLevenberg
{
public:
    explicit OrbLevenberg(std::unique_ptr<g2o::Solver> solver);
    ~OrbLevenberg() override;

    /**
     * One LM iteration, plus the fork's early-stop criterion.
     * @return Terminate once three consecutive iterations each improved the
     *         robust chi2 by less than 0.1%; otherwise the upstream result.
     */
    SolverResult solve(int iteration, bool online = false) override;

    //! consecutive low-improvement iterations seen so far (the fork's `_nBad`)
    int badIterations() const { return mnBad; }

    // -------------------------------------------------------------------------
    // P10-1 abort-flag shadow bridge (docs/P10_RECON.md 2부 item 4).
    //
    // The producers of the BA-abort request (Tracking::InterruptBA /
    // InsertKeyFrame, LoopClosing's GBA abort) live on OTHER threads than the
    // optimization, so the source of truth is a std::atomic<bool>
    // (LocalMapping::mbAbortBA, LoopClosing::mbStopGBA). g2o, however, polls a
    // plain `bool*` (`SparseOptimizer::setForceStopFlag(bool*)`; derefs in
    // sparse_optimizer.h terminate() and the LM retrial loop), and the pinned
    // submodule is never patched (docs/DIVERGENCES.md item 10).
    //
    // Rejected alternatives (both remain data races / UB):
    //  * std::atomic_ref (C++20): [atomics.ref] requires ALL concurrent
    //    accesses to go through atomic_ref while one exists — g2o's plain
    //    `*_forceStopFlag` read concurrent with an atomic_ref store is still a
    //    data race. C++20 does not solve this.
    //  * reinterpret_cast<bool*>(&atomic<bool>): works on our ABIs but is UB.
    //
    // Sound route: bind the atomic here and hand g2o `&mShadowStop`, a plain
    // bool touched ONLY by the optimizing thread. solve() refreshes the shadow
    // from the atomic at entry AND exit (bindAbortFlag() also samples once, so
    // an abort raised before optimize() starts skips all iterations, matching
    // the pre-bridge semantics). The single cross-thread edge is the relaxed
    // atomic load. Abort granularity vs the raw pointer: the outer terminate()
    // check reads the value sampled at the previous solve() exit (µs-stale);
    // the retrial-loop poll sees the solve()-entry sample — worst case the
    // remaining trials of one LM iteration of extra latency. Timing-only.
    // -------------------------------------------------------------------------

    //! Bind the cross-thread abort flag; samples it once immediately.
    void bindAbortFlag(const std::atomic<bool>* pAbortFlag)
    {
        mpAbortFlag = pAbortFlag;
        refreshShadowStop();
    }

    //! Plain-bool alias for g2o's setForceStopFlag(); optimizing-thread-only.
    bool* shadowPtr() { return &mShadowStop; }

private:
    //! mirrors the fork's `_nBad`; reset on iteration 0 of every optimize() call
    int mnBad = 0;

    //! P10-1 shadow bridge: atomic source of truth (nullptr = no abort flag)
    const std::atomic<bool>* mpAbortFlag = nullptr;
    //! P10-1 shadow bridge: what g2o's bool* actually points at
    bool mShadowStop = false;

    void refreshShadowStop()
    {
        if(mpAbortFlag)
            mShadowStop = mpAbortFlag->load(std::memory_order_relaxed);
    }
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_BACKEND_ORBLEVENBERG_HPP
