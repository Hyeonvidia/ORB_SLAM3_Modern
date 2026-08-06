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

#include "geometry/TwoViewInitializer.hpp"

namespace ORB_SLAM3
{

bool TwoViewInitializer::Reconstruct(GeometricCamera* pCamera,
                                     const std::vector<cv::KeyPoint>& vKeys1,
                                     const std::vector<cv::KeyPoint>& vKeys2,
                                     const std::vector<int>& vMatches12,
                                     Sophus::SE3f& T21, std::vector<cv::Point3f>& vP3D,
                                     std::vector<bool>& vbTriangulated)
{
    if(!mpTvr){
        Eigen::Matrix3f K = pCamera->toK_();
        mpTvr.reset(new TwoViewReconstruction(K));
    }

    const std::vector<cv::KeyPoint> vKeysUn1 = pCamera->UndistortKeyPoints(vKeys1);
    const std::vector<cv::KeyPoint> vKeysUn2 = pCamera->UndistortKeyPoints(vKeys2);

    return mpTvr->Reconstruct(vKeysUn1,vKeysUn2,vMatches12,T21,vP3D,vbTriangulated);
}

}
