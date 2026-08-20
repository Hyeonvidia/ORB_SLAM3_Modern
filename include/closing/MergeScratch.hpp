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

#ifndef MERGESCRATCH_H
#define MERGESCRATCH_H

#include <map>
#include "map/MapTypes.hpp"  // R4b: MapPointPtr

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace ORB_SLAM3
{

class KeyFrame;
class MapPoint;

// Per-merge scratch state, externalized from KeyFrame/MapPoint member fields
// (P5 group G). A MergeScratch is local to one LoopClosing::MergeLocal run and
// is threaded into the merge variant of Optimizer::OptimizeEssentialGraph.

struct KeyFrameMergeScratch
{
    Sophus::SE3f Tcw;        // was KeyFrame::mTcwMerge
    Sophus::SE3f TcwBef;     // was KeyFrame::mTcwBefMerge
    Sophus::SE3f TwcBef;     // was KeyFrame::mTwcBefMerge
    Eigen::Vector3f Vwb;     // was KeyFrame::mVwbMerge
};

struct MapPointMergeScratch
{
    Eigen::Vector3f Pos;     // was MapPoint::mPosMerge
    Eigen::Vector3f Normal;  // was MapPoint::mNormalVectorMerge
};

struct MergeScratch
{
    std::map<KeyFramePtr, KeyFrameMergeScratch> kfs;
    std::map<MapPointPtr, MapPointMergeScratch> mps;
};

} // namespace ORB_SLAM3

#endif // MERGESCRATCH_H
