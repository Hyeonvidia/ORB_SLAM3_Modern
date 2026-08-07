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

#ifndef CONVERTER_H
#define CONVERTER_H

#include <opencv2/core/core.hpp>

#include <Eigen/Dense>
#include "Thirdparty/g2o/g2o/types/sim3.h"
#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"

namespace ORB_SLAM3
{

// cv::Mat <-> Eigen/Sophus conversion utilities.
// P2-3 trimmed this to the surface the codebase actually calls (16 dead
// functions removed, including every g2o::SE3Quat conversion); the remaining
// g2o dependency is toSophus(g2o::Sim3) only, which the P6 backend swap
// will retire together with LoopClosing's g2o::Sim3 value type.
class Converter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    static cv::Mat toCvMat(const Eigen::Matrix<float,4,4> &m);
    static cv::Mat toCvMat(const Eigen::Matrix3f &m);
    static cv::Mat toCvMat(const Eigen::Matrix<float,3,1> &m);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    static Eigen::Matrix<float,3,1> toVector3f(const cv::Mat &cvVector);
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    static Eigen::Matrix<float,3,3> toMatrix3f(const cv::Mat &cvMat3);
    static std::vector<float> toQuaternion(const cv::Mat &M);

    static Sophus::SE3<float> toSophus(const cv::Mat& T);
    static Sophus::Sim3f toSophus(const g2o::Sim3& S);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
