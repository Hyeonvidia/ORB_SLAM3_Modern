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


#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

// P8-3: this header uses only KeyFrame/Atlas(->Map) as complete types;
// Tracking/LoopClosing are forward-declared below, cutting the two mutual
// include cycles LocalMapping.hpp <-> Tracking.hpp / LoopClosing.hpp.
#include "map/KeyFrame.hpp"
#include "map/Atlas.hpp"
#include "mapping/ImuInitializer.hpp"

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>


namespace ORB_SLAM3
{

class Tracking;
class LoopClosing;
class Atlas;
struct BAEpochs;
class IMappingOptimizer;

// =============================================================================
// Queue/backpressure contract (P8-2, from the docs/P8_RECON.md survey).
//
// mlNewKeyFrames is an unbounded producer/consumer queue with no condition
// variable; Run() polls at 3ms. Producer is always the Tracking thread via
// InsertKeyFrame (which also preempts any running BA through mbAbortBA).
// KeyFrames in the queue are NOT yet in the map -- Atlas::AddKeyFrame happens
// inside ProcessNewKeyFrame, and only KFs consumed by Run()'s main path are
// ever handed to LoopClosing.
//
// Protocol invariants (behavioral contract, not aspiration):
//  1. Backpressure is advisory. mbAcceptKeyFrames is a duty-cycle hint
//     (false for the whole iteration body); Tracking inserts regardless when
//     its c3/c4 conditions hold, via InterruptBA + queue-depth<3 (stereo).
//  2. Queue drain beats stop. Run() will not park while the queue is
//     non-empty, so a RequestStop -> isStopped() wait implicitly waits for
//     full drain unless the caller drains it itself with EmptyQueue().
//  3. STOPPED is exited only externally (Release() or shutdown).
//     SetNotStop(true) failing (already stopped) tells Tracking to drop the
//     keyframe it was about to insert.
//  4. mbBadImu quirk: Stop() sets mbStopped even when mbBadImu skips the park
//     loop -- external isStopped() waiters proceed while Run() keeps looping;
//     recovery is the reset Run() self-requested.
//  5. The park loop does not service ResetIfRequested; a Tracking reset
//     request spins until some LoopClosing/System path calls Release().
//  6. Resets are producer-synchronous: RequestReset* spins the calling
//     thread until consumed, so ResetIfRequested's lock-free queue clear is
//     protected by the protocol, not the mutex.
//
// Known inherited races (R1-R5) and the three drain-disposal asymmetries are
// cataloged in docs/OWNERSHIP.md; fixes are deferred to P10 (threading).
// =============================================================================
class LocalMapping
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocalMapping(Atlas* pAtlas, bool bMonocular, bool bInertial, BAEpochs* pBAEpochs, IMappingOptimizer* pOptimizer);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);
    void EmptyQueue();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();

    double mFirstTs;

    bool mbBadImu;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

#ifdef REGISTER_TIMES
    vector<double> vdKFInsert_ms;
    vector<double> vdMPCulling_ms;
    vector<double> vdMPCreation_ms;
    vector<double> vdLBA_ms;
    vector<double> vdKFCulling_ms;
    vector<double> vdLMTotal_ms;


    vector<double> vdLBASync_ms;
    vector<double> vdKFCullingSync_ms;
    vector<int> vnLBA_edges;
    vector<int> vnLBA_KFopt;
    vector<int> vnLBA_KFfixed;
    vector<int> vnLBA_MPs;
    int nLBA_exec;
    int nLBA_abort;
#endif
protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();
    void KeyFrameCulling();

    bool mbMonocular;
    bool mbInertial;

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;

    // Persistent local-BA epoch marks, owned by System (P5-C).
    BAEpochs* mpBAEpochs;

    // Narrow optimizer interface, owned by System (G2oBackend value member,
    // P6-A ISP split — see include/backend/IMappingOptimizer.hpp).
    IMappingOptimizer* mpOptimizer;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    // P10-0: opt-in queue-latency tracing (ORB_TRACE_QUEUE=1, P7-1a env-gate
    // pattern; docs/P10_RECON.md 3부 §4). Dormant unless the env var is set:
    // every touch point costs one cached static-bool test when off (no clock
    // calls, no allocation). The enqueue-stamp deque shadows mlNewKeyFrames
    // under the SAME mMutexNewKFs (cleared wherever the queue is destructively
    // drained); latency samples and iteration counters are LM-thread-only.
    // One summary line is emitted from SetFinish(). Behavior-neutral by
    // construction: no locking changes, no extra wakeups.
    std::deque<std::chrono::steady_clock::time_point> mdqTraceEnqueueTs;
    std::vector<long> mvTraceDequeueUs;
    std::size_t mnTraceMaxDepth = 0;
    unsigned long mnTraceIters = 0;
    unsigned long mnTraceEmptyIters = 0;

    // P10-1: atomic (was plain bool, ledger race R4). Written lock-free by
    // Tracking (InterruptBA) and under mMutexNewKFs by InsertKeyFrame; read
    // by the LM thread and, through the OrbLevenberg shadow bridge, polled
    // during BA. All accesses relaxed — it is a lone advisory flag, no
    // ordering contract with other data.
    std::atomic<bool> mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    // P8-4: the guarded end-of-init queue purge (DIVERGENCES #19) lives
    // here so every mlNewKeyFrames mutation stays inside LocalMapping;
    // called by ImuInitializer at the end of both entry points.
    void PurgeNewKeyFramesAfterInertialInit();

    bool bInitializing;

    float mTinit;

    // P8-4: IMU initialization machinery (InitializeIMU / ScaleRefinement)
    // extracted to a collaborator; it reaches host state through the friend
    // grant. Run() staging gates and mTinit bookkeeping stay here.
    friend class ImuInitializer;
    ImuInitializer mImuInit;

    };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
