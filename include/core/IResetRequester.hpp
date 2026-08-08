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

#ifndef IRESETREQUESTER_H
#define IRESETREQUESTER_H

// P7-1b: the only System surface Tracking actually consumes (docs/P7_RECON.md
// §B). Deliberately ZERO includes and zero forward declarations: no ORB_SLAM3
// type appears in the signature, so this header can never re-enter the
// System.hpp -> Tracking.hpp include cycle the way ILoopOptimizer did in P6-1.

namespace ORB_SLAM3
{

// Narrow reset-request interface, implemented by System.
//
// Naming follows the existing LocalMapping::RequestResetActiveMap /
// LoopClosing::RequestResetActiveMap convention: "Request" = non-blocking
// latch, not the work itself. (Tracking::ResetActiveMap() is the EXECUTOR of
// the request; keeping the interface verb distinct makes the latch/execute
// split greppable.)
class IResetRequester
{
public:
    virtual ~IResetRequester() = default;

    // Non-blocking. Latches a request (System: mbResetActiveMap under
    // mMutexReset) that is consumed by the tracking thread at the top of the
    // NEXT System::TrackStereo/TrackRGBD/TrackMonocular call, which then runs
    // Tracking::ResetActiveMap() synchronously. The requesting frame is NOT
    // reset — 7 of the 8 Tracking call sites return immediately, and the
    // eighth (TrackLocalMap-failure, inertial) deliberately finishes the
    // whole frame in RECENTLY_LOST before the reset lands one frame later.
    // MUST NOT be made synchronous: that deferred semantics is load-bearing
    // for the EuRoC/TUM-VI baselines (docs/P7_RECON.md §B.1, §A T10).
    virtual void RequestResetActiveMap() = 0;
};

} // namespace ORB_SLAM3

#endif // IRESETREQUESTER_H
