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

private:
    //! mirrors the fork's `_nBad`; reset on iteration 0 of every optimize() call
    int mnBad = 0;
};

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_BACKEND_ORBLEVENBERG_HPP
