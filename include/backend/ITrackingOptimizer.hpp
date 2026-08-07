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

#ifndef ITRACKINGOPTIMIZER_H
#define ITRACKINGOPTIMIZER_H

namespace ORB_SLAM3
{

class Frame;
class Map;
struct GBAResult;

// P6-A ISP split (docs/P6_DESIGN.md §C): the optimizer surface Tracking
// actually consumes. Deliberately free of g2o headers — forward declarations
// only — so the Tracking TU no longer inherits the backend's include cost.
//
// All methods are const: the backend is stateless (P5 extracted the hidden
// state into BAEpochs/MergeScratch/GBAResult), which makes sharing one
// instance across the Tracking/LocalMapping/LoopClosing(+GBA) threads safe
// with the same semantics as the previous static calls.
//
// Default arguments live ONLY on these interface declarations, never on
// overrides (virtual default args bind statically); all call sites go through
// the interface pointer, so this is safe.
class ITrackingOptimizer
{
public:
    virtual ~ITrackingOptimizer() = default;

    virtual int PoseOptimization(Frame* pFrame) const = 0;
    virtual int PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false) const = 0;
    virtual int PoseInertialOptimizationLastFrame(Frame* pFrame, bool bRecInit = false) const = 0;

    // Spelling fixed at the interface level (upstream static is the typo'd
    // Optimizer::GlobalBundleAdjustemnt, kept as-is internally).
    virtual void GlobalBundleAdjustment(Map* pMap, int nIterations = 5, bool* pbStopFlag = nullptr,
                                        unsigned long nLoopKF = 0, bool bRobust = true,
                                        GBAResult* pResult = nullptr) const = 0;
};

} // namespace ORB_SLAM3

#endif // ITRACKINGOPTIMIZER_H
