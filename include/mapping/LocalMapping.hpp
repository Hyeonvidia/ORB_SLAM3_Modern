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
#include "map/MapTypes.hpp"  // R4b: MapPointPtr
#include "map/Atlas.hpp"
#include "mapping/ImuInitializer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
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
// mlNewKeyFrames is an unbounded producer/consumer queue. Since P10-4 it is
// condition-variable-driven: InsertKeyFrame notifies mCondNewKFs, and Run()'s
// tail is a 3ms-capped timed wait (the timed net is PERMANENT -- see the
// mCondNewKFs comment below). Producer is always the Tracking thread via
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
//     request blocks (CV wait since P10-4, spin before) until some
//     LoopClosing/System path calls Release().
//  6. Resets are producer-synchronous: RequestReset* blocks the calling
//     thread until consumed (mCondReset handshake since P10-4), so
//     ResetIfRequested's queue clear is protected by the protocol on top
//     of the P10-2 mMutexNewKFs closure.
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
    //
    // P10-2 canonical lock order (also in docs/OWNERSHIP.md):
    //   mMutexFinish -> mMutexStop -> mMutexNewKFs
    //   mMutexReset  -> mMutexNewKFs      (LM ResetIfRequested)
    //   LoopClosing: mMutexReset -> mMutexLoopQueue; mMutexGBA standalone
    //   mMutexAccept standalone
    // SetFinish() and Release() both follow Finish -> Stop (Release was
    // Stop -> Finish until P10-2 -- the R6 ABBA deadlock found in the P10
    // recon). Never take an earlier mutex while holding a later one.
    void RequestStop();
    void RequestReset();
    void RequestResetActiveMap(Map* pMap);
    bool Stop();
    void Release();
    bool isStopped();
    // P10-4: CV replacement for the external `while(!isStopped()) usleep`
    // spin (LoopClosing CorrectLoop/MergeLocal/MergeLocal2, GBA, System
    // localization-mode). Blocks until mbStopped == true; woken by Stop()
    // and SetFinish() (which also sets mbStopped, so finish-aware waiters
    // converge on the same predicate). isStopped()/stopRequested() remain
    // for non-blocking callers (Tracking::NeedNewKeyFrame).
    void WaitUntilStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    bool IsInitializing();
    double GetCurrKFTime();

    // P10-2: atomic (ledger race R5 -- a torn double was possible). TWO
    // writer threads (Tracking at monocular-inertial map creation,
    // ImuInitializer on the LM thread) plus cross-thread readers
    // (System::GetTimeFromIMUInit). Relaxed everywhere: lone timestamp,
    // no ordering contract with other data.
    std::atomic<double> mFirstTs;
    static_assert(std::atomic<double>::is_always_lock_free,
                  "atomic<double> must be lock-free on target ABIs (arm64/x86-64)");

    // P10-2: atomic (ledger race R5). Written by the LM thread (Run
    // no-motion branch, ResetIfRequested); read lock-free by Tracking
    // (Track entry gate) and the LM Run gates. Relaxed -- advisory flag.
    std::atomic<bool> mbBadImu;

    // not consider far points (clouds)
    bool mbFarPoints;
    float mThFarPoints;

#ifdef REGISTER_TIMES
    std::vector<double> vdKFInsert_ms;
    std::vector<double> vdMPCulling_ms;
    std::vector<double> vdMPCreation_ms;
    std::vector<double> vdLBA_ms;
    std::vector<double> vdKFCulling_ms;
    std::vector<double> vdLMTotal_ms;


    std::vector<double> vdLBASync_ms;
    std::vector<double> vdKFCullingSync_ms;
    std::vector<int> vnLBA_edges;
    std::vector<int> vnLBA_KFopt;
    std::vector<int> vnLBA_KFfixed;
    std::vector<int> vnLBA_MPs;
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
    // P10-2: request flags atomic (ledger race R5) -- the ImuInitializer
    // early guards read them lock-free from the LM thread. Writes stay
    // inside the existing mMutexReset scopes (the requester spin handshake
    // is unchanged); mpMapToReset stays mutex-guarded.
    std::atomic<bool> mbResetRequested;
    std::atomic<bool> mbResetRequestedActiveMap;
    Map* mpMapToReset;
    std::mutex mMutexReset;
    // P10-4: reset handshake CV (paired with mMutexReset). RequestReset*
    // wait on "my flag cleared"; ResetIfRequested notifies after clearing.
    // Preserved upstream quirks (see the function comments): a reset
    // requested while Run() is parked is serviced only after Release(),
    // and a reset requested after SetFinish() never returns.
    std::condition_variable mCondReset;

    bool CheckFinish();
    void SetFinish();
    // P10-2: atomic (prep for the P10-4 park predicate). The store keeps
    // the existing mMutexFinish scope in RequestFinish; CheckFinish reads
    // lock-free relaxed. mbFinished stays under mMutexFinish because
    // SetFinish couples it with mbStopped.
    std::atomic<bool> mbFinishRequested;
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

    // P10-2: atomic (ledger race R5). Store-release in ProcessNewKeyFrame
    // (pairs with the mMutexNewKFs handoff to make the Tracking-thread KF
    // construction visible), load-acquire in GetCurrKFTime (the lock-free
    // cross-thread reader via System::GetTimeFromIMUInit); all same-thread
    // (LM) uses are relaxed loads.
    std::atomic<KeyFrame*> mpCurrentKeyFrame;

    std::list<MapPointPtr> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;
    // P10-4: queue CV (paired with mMutexNewKFs). InsertKeyFrame notifies
    // after unlocking; Run()'s tail waits on "queue non-empty" with a 3ms
    // timed net. The net is PERMANENT by design (docs/P10_RECON.md 2부
    // item 1): the loop's other wake reasons -- stop/finish/reset/mbBadImu
    // -- live under three OTHER mutexes, and a pure untimed wait would need
    // them all in this predicate (lock nesting against the canonical order)
    // while turning the invariant-4 mbBadImu quirk (queue non-empty, work
    // branch skipped) into an instant-wakeup busy loop where upstream burnt
    // a 3ms sleep. The timed net keeps every non-queue latency at exactly
    // the historical <=3ms bound while making KF pickup immediate.
    std::condition_variable mCondNewKFs;

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
    // P10-4: stop/park CV (paired with mMutexStop). Two predicate families:
    //   - Run()'s park waits on "!mbStopped || mbFinishRequested" (the
    //     atomic finish flag keeps mMutexFinish OUT of the predicate --
    //     taking it here would nest Stop->Finish against the canonical
    //     Finish->Stop order). The predicate deliberately EXCLUDES the
    //     reset flags (invariant 5: a reset during park waits for Release).
    //   - WaitUntilStopped() waits on "mbStopped".
    // Notifiers: Stop() on success, Release(), SetFinish(), RequestFinish()
    // (the latter via a mMutexStop hand-off, see its comment).
    std::condition_variable mCondStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    // P8-4: the guarded end-of-init queue purge (DIVERGENCES #19) lives
    // here so every mlNewKeyFrames mutation stays inside LocalMapping;
    // called by ImuInitializer at the end of both entry points.
    void PurgeNewKeyFramesAfterInertialInit();

    // P10-2: atomic (ledger race R5). Written by ImuInitializer (LM
    // thread), read lock-free by Tracking via IsInitializing. Relaxed.
    std::atomic<bool> bInitializing;

    float mTinit;

    // P8-4: IMU initialization machinery (InitializeIMU / ScaleRefinement)
    // extracted to a collaborator; it reaches host state through the friend
    // grant. Run() staging gates and mTinit bookkeeping stay here.
    friend class ImuInitializer;
    ImuInitializer mImuInit;

    };

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
