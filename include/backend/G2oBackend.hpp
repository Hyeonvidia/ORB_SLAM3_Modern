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

#ifndef G2OBACKEND_H
#define G2OBACKEND_H

#include "backend/ITrackingOptimizer.hpp"
#include "backend/IMappingOptimizer.hpp"
#include "backend/ILoopOptimizer.hpp"

namespace ORB_SLAM3
{

// P6 Phase A (docs/P6_DESIGN.md §C): the single concrete backend implementing
// all three ISP interfaces. Stateless (zero data members); every method
// delegates 1:1 to the corresponding static Optimizer:: function over the
// vendored g2o — zero behavior change. Phase B replaces this implementation
// (build-time ORB_BACKEND switch) with the upstream-g2o port.
//
// Owned by value in System (BAEpochs precedent) and injected as narrow
// interface pointers into Tracking / LocalMapping / LoopClosing.
//
// No default arguments on any override (they live on the interface
// declarations only — virtual default args bind statically).
class G2oBackend : public ITrackingOptimizer, public IMappingOptimizer, public ILoopOptimizer
{
public:
    G2oBackend() = default;

    // ---- ITrackingOptimizer -------------------------------------------------
    int PoseOptimization(Frame* pFrame) const override;
    int PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit) const override;
    int PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit) const override;

    // ---- ITrackingOptimizer + ILoopOptimizer (single override, both bases) --
    void GlobalBundleAdjustment(Map* pMap, int nIterations, bool* pbStopFlag,
                                unsigned long nLoopKF, bool bRobust,
                                GBAResult* pResult) const override;

    // ---- IMappingOptimizer --------------------------------------------------
    void LocalBundleAdjustment(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                               int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges,
                               BAEpochs& epochs) const override;
    void LocalInertialBA(KeyFrame* pKF, bool* pbStopFlag, Map* pMap,
                         int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges,
                         bool bLarge, bool bRecInit, BAEpochs& epochs) const override;
    void InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale,
                              Eigen::Vector3d& bg, Eigen::Vector3d& ba, bool bMono,
                              Eigen::MatrixXd& covInertial, bool bFixedVel,
                              bool bGauss, float priorG, float priorA) const override;
    void InertialOptimization(Map* pMap, Eigen::Matrix3d& Rwg, double& scale) const override;

    // ---- IMappingOptimizer + ILoopOptimizer (single override, both bases) ---
    void FullInertialBA(Map* pMap, int its, bool bFixLocal, unsigned long nLoopKF,
                        bool* pbStopFlag, bool bInit, float priorG, float priorA,
                        Eigen::VectorXd* vSingVal, bool* bHess,
                        GBAResult* pResult, BAEpochs& epochs) const override;

    // ---- ILoopOptimizer -----------------------------------------------------
    int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches1,
                     g2o::Sim3& g2oS12, float th2, bool bFixScale,
                     Eigen::Matrix<double,7,7>& mAcumHessian, bool bAllPoints) const override;
    void OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                const KeyFrameAndPose& NonCorrectedSim3,
                                const KeyFrameAndPose& CorrectedSim3,
                                const std::map<KeyFrame*, std::set<KeyFrame*> >& LoopConnections,
                                const bool& bFixScale,
                                const std::map<MapPoint*, unsigned long>& correctedRefs) const override;
    void OptimizeEssentialGraph(KeyFrame* pCurKF, std::vector<KeyFrame*>& vpFixedKFs,
                                std::vector<KeyFrame*>& vpFixedCorrectedKFs,
                                std::vector<KeyFrame*>& vpNonFixedKFs,
                                std::vector<MapPoint*>& vpNonCorrectedMPs,
                                MergeScratch& scratch) const override;
    void OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                    const KeyFrameAndPose& NonCorrectedSim3,
                                    const KeyFrameAndPose& CorrectedSim3,
                                    const std::map<KeyFrame*, std::set<KeyFrame*> >& LoopConnections) const override;
    void MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool* pbStopFlag, Map* pMap,
                         KeyFrameAndPose& corrPoses, BAEpochs& epochs) const override;
    void LocalBundleAdjustment(KeyFrame* pMainKF, std::vector<KeyFrame*> vpAdjustKF,
                               std::vector<KeyFrame*> vpFixedKF, bool* pbStopFlag,
                               const BAEpochs& epochs) const override;
    void InertialOptimization(Map* pMap, Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                              float priorG, float priorA) const override;
};

} // namespace ORB_SLAM3

#endif // G2OBACKEND_H
