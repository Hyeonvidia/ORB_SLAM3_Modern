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

#ifndef ORB_SLAM3_TWO_VIEW_INITIALIZER_HPP
#define ORB_SLAM3_TWO_VIEW_INITIALIZER_HPP

#include "camera/GeometricCamera.hpp"
#include "geometry/TwoViewReconstruction.hpp"

#include <memory>

namespace ORB_SLAM3
{

// Monocular two-view initialization orchestrator (P3-3). Extracted from the
// camera models, which used to each own a lazily created raw
// TwoViewReconstruction* (leaked in KannalaBrandt8, uninitialized in some
// constructors). Semantics preserved bit-for-bit: same lazy construction
// from the camera K (TwoViewReconstruction defaults sigma=1.0, iters=200),
// same model-specific keypoint undistortion (via
// GeometricCamera::UndistortKeyPoints) before reconstruction.
class TwoViewInitializer
{
public:
    bool Reconstruct(GeometricCamera* pCamera,
                     const std::vector<cv::KeyPoint>& vKeys1,
                     const std::vector<cv::KeyPoint>& vKeys2,
                     const std::vector<int>& vMatches12,
                     Sophus::SE3f& T21, std::vector<cv::Point3f>& vP3D,
                     std::vector<bool>& vbTriangulated);

private:
    std::unique_ptr<TwoViewReconstruction> mpTvr;
};

}

#endif // ORB_SLAM3_TWO_VIEW_INITIALIZER_HPP
