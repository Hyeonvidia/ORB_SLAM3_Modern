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

// P6 Phase A (docs/P6_DESIGN.md §C): every method is a 1:1 delegation to the
// static Optimizer:: functions (vendored g2o) — zero behavior change. This TU
// and Optimizer.cpp are the only in-tree consumers of backend/Optimizer.hpp.

#include "backend/G2oBackend.hpp"

#include "backend/Optimizer.hpp"

namespace ORB_SLAM3
{

// ---- ITrackingOptimizer -----------------------------------------------------

int G2oBackend::PoseOptimization(Frame* pFrame) const
{
    return Optimizer::PoseOptimization(pFrame);
}

int G2oBackend::PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit) const
{
    return Optimizer::PoseInertialOptimizationLastKeyFrame(pFrame, bRecInit);
}

int G2oBackend::PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit) const
{
    return Optimizer::PoseInertialOptimizationLastFrame(pFrame, bRecInit);
}

// ---- ITrackingOptimizer + ILoopOptimizer ------------------------------------

void G2oBackend::GlobalBundleAdjustment(Map* pMap, int nIterations, bool* pbStopFlag,
                                        unsigned long nLoopKF, bool bRobust,
                                        GBAResult* pResult) const
{
    Optimizer::GlobalBundleAdjustemnt(pMap, nIterations, pbStopFlag, nLoopKF, bRobust, pResult);
}

// ---- IMappingOptimizer ------------------------------------------------------

void G2oBackend::LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                       int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges,
                                       BAEpochs& epochs) const
{
    Optimizer::LocalBundleAdjustment(pKF, pbStopFlag, pMap, num_fixedKF, num_OptKF, num_MPs, num_edges, epochs);
}

void G2oBackend::LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                                 int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges,
                                 bool bLarge, bool bRecInit, BAEpochs& epochs) const
{
    Optimizer::LocalInertialBA(pKF, pbStopFlag, pMap, num_fixedKF, num_OptKF, num_MPs, num_edges, bLarge, bRecInit, epochs);
}

void G2oBackend::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale,
                                      Eigen::Vector3d& bg, Eigen::Vector3d& ba, bool bMono,
                                      Eigen::MatrixXd& covInertial, bool bFixedVel,
                                      bool bGauss, float priorG, float priorA) const
{
    Optimizer::InertialOptimization(pMap, Rwg, scale, bg, ba, bMono, covInertial, bFixedVel, bGauss, priorG, priorA);
}

void G2oBackend::InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale) const
{
    Optimizer::InertialOptimization(pMap, Rwg, scale);
}

// ---- IMappingOptimizer + ILoopOptimizer -------------------------------------

void G2oBackend::FullInertialBA(Map* pMap, int its, bool bFixLocal, unsigned long nLoopKF,
                                bool* pbStopFlag, bool bInit, float priorG, float priorA,
                                Eigen::VectorXd* vSingVal, bool* bHess,
                                GBAResult* pResult, BAEpochs& epochs) const
{
    Optimizer::FullInertialBA(pMap, its, bFixLocal, nLoopKF, pbStopFlag, bInit, priorG, priorA, vSingVal, bHess, pResult, epochs);
}

// ---- ILoopOptimizer ---------------------------------------------------------

int G2oBackend::OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches1,
                             g2o::Sim3& g2oS12, float th2, bool bFixScale,
                             Eigen::Matrix<double,7,7>& mAcumHessian, bool bAllPoints) const
{
    return Optimizer::OptimizeSim3(pKF1, pKF2, vpMatches1, g2oS12, th2, bFixScale, mAcumHessian, bAllPoints);
}

void G2oBackend::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                        const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
                                        const LoopClosing::KeyFrameAndPose& CorrectedSim3,
                                        const std::map<KeyFrame*, std::set<KeyFrame*> >& LoopConnections,
                                        const bool& bFixScale,
                                        const std::map<MapPoint*, unsigned long>& correctedRefs) const
{
    Optimizer::OptimizeEssentialGraph(pMap, pLoopKF, pCurKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixScale, correctedRefs);
}

void G2oBackend::OptimizeEssentialGraph(KeyFrame* pCurKF, std::vector<KeyFrame*>& vpFixedKFs,
                                        std::vector<KeyFrame*>& vpFixedCorrectedKFs,
                                        std::vector<KeyFrame*>& vpNonFixedKFs,
                                        std::vector<MapPoint*>& vpNonCorrectedMPs,
                                        MergeScratch& scratch) const
{
    Optimizer::OptimizeEssentialGraph(pCurKF, vpFixedKFs, vpFixedCorrectedKFs, vpNonFixedKFs, vpNonCorrectedMPs, scratch);
}

void G2oBackend::OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                            const LoopClosing::KeyFrameAndPose& NonCorrectedSim3,
                                            const LoopClosing::KeyFrameAndPose& CorrectedSim3,
                                            const std::map<KeyFrame*, std::set<KeyFrame*> >& LoopConnections) const
{
    Optimizer::OptimizeEssentialGraph4DoF(pMap, pLoopKF, pCurKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
}

void G2oBackend::MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool* pbStopFlag, Map* pMap,
                                 LoopClosing::KeyFrameAndPose& corrPoses, BAEpochs& epochs) const
{
    Optimizer::MergeInertialBA(pCurrKF, pMergeKF, pbStopFlag, pMap, corrPoses, epochs);
}

void G2oBackend::LocalBundleAdjustment(KeyFrame* pMainKF, std::vector<KeyFrame*> vpAdjustKF,
                                       std::vector<KeyFrame*> vpFixedKF, bool* pbStopFlag,
                                       const BAEpochs& epochs) const
{
    Optimizer::LocalBundleAdjustment(pMainKF, vpAdjustKF, vpFixedKF, pbStopFlag, epochs);
}

void G2oBackend::InertialOptimization(Map* pMap, Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                                      float priorG, float priorA) const
{
    Optimizer::InertialOptimization(pMap, bg, ba, priorG, priorA);
}

} // namespace ORB_SLAM3
