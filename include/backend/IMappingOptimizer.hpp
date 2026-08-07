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

#ifndef IMAPPINGOPTIMIZER_H
#define IMAPPINGOPTIMIZER_H

#include <Eigen/Core>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
struct BAEpochs;
struct GBAResult;

// P6-A ISP split (docs/P6_DESIGN.md §C): the optimizer surface LocalMapping
// actually consumes. Forward declarations + Eigen only — no g2o headers, no
// LoopClosing dependency.
//
// All methods are const (stateless backend, safe cross-thread — see
// ITrackingOptimizer.hpp). Default arguments only here, never on overrides.
//
// FullInertialBA is intentionally redeclared in ILoopOptimizer with an
// identical signature: one override in the concrete backend satisfies both
// bases (design rule 2 — no common-base diamond needed).
class IMappingOptimizer
{
public:
    virtual ~IMappingOptimizer() = default;

    virtual void LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                       int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges,
                                       BAEpochs& epochs) const = 0;

    virtual void LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                 int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges,
                                 bool bLarge, bool bRecInit, BAEpochs& epochs) const = 0;

    // Full variant (IMU initialization).
    virtual void InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale,
                                      Eigen::Vector3d& bg, Eigen::Vector3d& ba, bool bMono,
                                      Eigen::MatrixXd& covInertial, bool bFixedVel = false,
                                      bool bGauss = false, float priorG = 1e2, float priorA = 1e6) const = 0;

    // Scale/gravity-only variant (scale refinement).
    virtual void InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale) const = 0;

    virtual void FullInertialBA(Map* pMap, int its, bool bFixLocal, unsigned long nLoopKF,
                                bool* pbStopFlag, bool bInit, float priorG, float priorA,
                                Eigen::VectorXd* vSingVal, bool* bHess,
                                GBAResult* pResult, BAEpochs& epochs) const = 0;
};

} // namespace ORB_SLAM3

#endif // IMAPPINGOPTIMIZER_H
