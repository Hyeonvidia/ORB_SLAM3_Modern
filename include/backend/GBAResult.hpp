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

#ifndef GBARESULT_H
#define GBARESULT_H

#include <map>
#include "map/MapTypes.hpp"  // R4b: MapPointPtr

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include "backend/ImuTypes.hpp"

namespace ORB_SLAM3
{

class KeyFrame;
class MapPoint;

// Result buffers of one Global Bundle Adjustment invocation, externalized from
// KeyFrame/MapPoint member fields (P5 group D). A GBAResult is owned by the
// caller that afterwards propagates the optimized values into the map
// (LoopClosing::RunGlobalBundleAdjustment, LocalMapping::InitializeIMU) and is
// threaded into Optimizer::BundleAdjustment / Optimizer::FullInertialBA.
// Membership in the maps replaces the old mnBAGlobalForKF==nLoopKF stamps, so
// each invocation naturally starts from a clean slate.

struct KeyFrameGBAResult
{
    Sophus::SE3f Tcw;        // was KeyFrame::mTcwGBA
    Eigen::Vector3f Vwb;     // was KeyFrame::mVwbGBA
    IMU::Bias Bias;          // was KeyFrame::mBiasGBA
    bool hasVwb = false;     // Vwb only set by inertial GBA / propagation
    bool hasBias = false;    // Bias only set by inertial GBA / propagation
};

struct GBAResult
{
    std::map<KeyFrame*, KeyFrameGBAResult> kfs;
    std::map<MapPointPtr, Eigen::Vector3f> mps;   // was MapPoint::mPosGBA
};

} // namespace ORB_SLAM3

#endif // GBARESULT_H
