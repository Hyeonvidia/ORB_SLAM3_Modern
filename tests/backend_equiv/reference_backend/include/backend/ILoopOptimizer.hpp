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

#ifndef ILOOPOPTIMIZER_H
#define ILOOPOPTIMIZER_H

#include <map>
#include <set>
#include <utility>
#include <vector>

#include <Eigen/Core>

// P6-A ISP split (docs/P6_DESIGN.md §C): the optimizer surface LoopClosing
// (including its detached GBA thread) consumes. This is the ONLY interface
// that depends on g2o::Sim3 — the Tracking/LocalMapping TUs stay clean.
//
// The narrow sim3.h include follows the io/Converter.hpp precedent; the full
// closing/LoopClosing.hpp cannot be included here because System.hpp (which
// carries G2oBackend and therefore this header) is itself reached from inside
// LoopClosing.hpp's include chain (Tracking.hpp -> Viewer.hpp -> System.hpp).
#include "Thirdparty/g2o/g2o/types/sim3.h"

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class MapPoint;
struct BAEpochs;
struct GBAResult;
struct MergeScratch;

// All methods are const (stateless backend, safe cross-thread — see
// ITrackingOptimizer.hpp); the GBA thread calls GlobalBundleAdjustment /
// FullInertialBA through the same pointer with the same thread semantics as
// the previous static calls. Default arguments only here, never on overrides.
//
// GlobalBundleAdjustment / FullInertialBA are intentionally redeclared with
// signatures identical to ITrackingOptimizer / IMappingOptimizer: one
// override in the concrete backend satisfies all bases (design rule 2).
class ILoopOptimizer
{
public:
    // Same underlying type as LoopClosing::KeyFrameAndPose (a class-nested
    // typedef cannot be forward-declared; redeclaring the identical std::map
    // instantiation keeps this header outside the LoopClosing.hpp cycle).
    typedef std::map<KeyFrame*, g2o::Sim3, std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

    virtual ~ILoopOptimizer() = default;

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    virtual int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint*>& vpMatches1,
                             g2o::Sim3& g2oS12, float th2, bool bFixScale,
                             Eigen::Matrix<double,7,7>& mAcumHessian, bool bAllPoints = false) const = 0;

    // Loop variant. if bFixScale is true, 6DoF optimization (stereo,rgbd),
    // 7DoF otherwise (mono). correctedRefs: map points corrected during the
    // loop closure, mapped to the id of their corrected reference keyframe.
    virtual void OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                        const KeyFrameAndPose& NonCorrectedSim3,
                                        const KeyFrameAndPose& CorrectedSim3,
                                        const std::map<KeyFrame*, std::set<KeyFrame*> >& LoopConnections,
                                        const bool& bFixScale,
                                        const std::map<MapPoint*, unsigned long>& correctedRefs) const = 0;

    // Merge variant (per-call MergeScratch, P5-G).
    virtual void OptimizeEssentialGraph(KeyFrame* pCurKF, std::vector<KeyFrame*>& vpFixedKFs,
                                        std::vector<KeyFrame*>& vpFixedCorrectedKFs,
                                        std::vector<KeyFrame*>& vpNonFixedKFs,
                                        std::vector<MapPoint*>& vpNonCorrectedMPs,
                                        MergeScratch& scratch) const = 0;

    // For inertial loopclosing.
    virtual void OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                            const KeyFrameAndPose& NonCorrectedSim3,
                                            const KeyFrameAndPose& CorrectedSim3,
                                            const std::map<KeyFrame*, std::set<KeyFrame*> >& LoopConnections) const = 0;

    virtual void MergeInertialBA(KeyFrame* pCurrKF, KeyFrame* pMergeKF, bool* pbStopFlag, Map* pMap,
                                 KeyFrameAndPose& corrPoses, BAEpochs& epochs) const = 0;

    // Welding variant: local BA in the welding area when two maps are merged
    // (epochs is read-only here).
    virtual void LocalBundleAdjustment(KeyFrame* pMainKF, std::vector<KeyFrame*> vpAdjustKF,
                                       std::vector<KeyFrame*> vpFixedKF, bool* pbStopFlag,
                                       const BAEpochs& epochs) const = 0;

    // Bias variant.
    virtual void InertialOptimization(Map* pMap, Eigen::Vector3d& bg, Eigen::Vector3d& ba,
                                      float priorG = 1e2, float priorA = 1e6) const = 0;

    // Spelling fixed at the interface level (upstream static is the typo'd
    // Optimizer::GlobalBundleAdjustemnt, kept as-is internally).
    virtual void GlobalBundleAdjustment(Map* pMap, int nIterations, bool* pbStopFlag,
                                        unsigned long nLoopKF, bool bRobust,
                                        GBAResult* pResult) const = 0;

    virtual void FullInertialBA(Map* pMap, int its, bool bFixLocal, unsigned long nLoopKF,
                                bool* pbStopFlag, bool bInit, float priorG, float priorA,
                                Eigen::VectorXd* vSingVal, bool* bHess,
                                GBAResult* pResult, BAEpochs& epochs) const = 0;
};

} // namespace ORB_SLAM3

#endif // ILOOPOPTIMIZER_H
