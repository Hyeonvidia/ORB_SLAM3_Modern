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

#ifndef IMUINITIALIZER_H
#define IMUINITIALIZER_H

#include <Eigen/Core>

namespace ORB_SLAM3
{

class LocalMapping;

// P8-4: the IMU initialization machinery, extracted verbatim from
// LocalMapping (docs/P8_RECON.md §6). This class owns only the estimator
// outputs of the inertial-only optimization -- gravity-aligning rotation,
// gyro/acc biases, scale. Everything the outside world touches stays on the
// host LocalMapping and is reached through it: the keyframe queue and its
// drain/purge semantics (incl. DIVERGENCES #19), bInitializing / mFirstTs /
// mTinit (shared with Run()'s staging gates, Tracking and System), and the
// Tracking cross-thread touches (UpdateFrameIMU, t0IMU,
// NotifyImuInitialized). Both entry points run on the LocalMapping thread
// exactly as before, so the memory-model status of every access is unchanged
// (docs/OWNERSHIP.md R3/R5).
class ImuInitializer
{
public:
    explicit ImuInitializer(LocalMapping& host);

    // Staged entry points, called from LocalMapping::Run() only: first init
    // (priorG=1e2, priorA=1e10 mono / 1e5 otherwise), VIBA1 (1, 1e5), VIBA2
    // (0, 0). bFIBA gates the FullInertialBA pass; every caller passes true.
    void InitializeIMU(float priorG, float priorA, bool bFIBA);

    // Mono-only periodic scale/gravity refinement (the reachable mTinit
    // windows are 25/35/45s; see LocalMapping::Run).
    void ScaleRefinement();

private:
    LocalMapping& mHost;

    // Estimator outputs of the last InertialOptimization call.
    Eigen::Matrix3d mRwg;
    Eigen::Vector3d mbg;
    Eigen::Vector3d mba;
    double mScale;
};

} // namespace ORB_SLAM3

#endif // IMUINITIALIZER_H
