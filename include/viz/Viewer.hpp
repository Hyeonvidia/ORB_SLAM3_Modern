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


#ifndef VIEWER_H
#define VIEWER_H

#include "viz/FrameDrawer.hpp"
#include "viz/MapDrawer.hpp"
#include "tracking/Tracking.hpp"
// P10-6: core/System.hpp is GONE (it re-included this header — the
// System.hpp <-> Viewer.hpp cycle of P10 recon §4). The Viewer consumes
// only the narrow IViewerHost surface, and the sensor-enum constants it
// used to read from System are a ctor-injected bool.
#include "core/IViewerHost.hpp"
#include "core/Settings.hpp"

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace ORB_SLAM3
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class Settings;

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // P10-6: was (System* pSystem, ...). bNonInertialSensor replaces the
    // menu-setup read of mpSystem->MONOCULAR/STEREO/RGBD (true for the
    // three non-inertial sensors; gates the ShowGraph default).
    Viewer(IViewerHost* pHost, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const std::string &strSettingPath, Settings* settings, bool bNonInertialSensor);

    void newParameterLoader(Settings* settings);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    // P10-6: CV wait for mbStopped (replaces Tracking's
    // `while(!isStopped()) usleep(3000)` reset-path spins). Woken by
    // Stop(). Deliberately NOT finish-aware — preserves the upstream
    // liveness quirk (recon W6): after RequestFinish, Stop() never sets
    // mbStopped and the old spin never terminated either.
    void WaitUntilStopped();

    bool isStepByStep();

    void Release();

    bool both;
private:


    bool Stop();

    IViewerHost* mpHost;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // P10-6: replaces Run()'s read of the System sensor enum (menu setup).
    bool mbNonInertialSensor;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageViewerScale;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    // P10-6: atomic — read lock-free by the park predicate under mMutexStop
    // (taking mMutexFinish there would nest Stop->Finish; LM discipline)
    // and by Stop(), whose old second mMutexFinish lock was the only
    // Stop->Finish nesting in the Viewer. Writer keeps mMutexFinish (orders
    // against the mbFinished handshake).
    std::atomic<bool> mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
    // P10-6: paired with mMutexStop. Waiters: Run()'s park (predicate
    // !mbStopped || finish-requested, UNTIMED — pthread_cond_wait is
    // TSAN-intercepted; the P10-4 clockwait gap applies only to timed
    // waits) and WaitUntilStopped() (predicate mbStopped). Notified by
    // Stop(), Release(), RequestFinish().
    std::condition_variable mCondStop;

    // P10-6: mbStopTrack DELETED — its only writer was the commented-out
    // SetTrackingPause(); the Run()-loop read/clear could never see true.

};

}


#endif // VIEWER_H
	

