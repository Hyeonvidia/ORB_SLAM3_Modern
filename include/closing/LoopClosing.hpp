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


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "map/KeyFrame.hpp"
#include "map/MapTypes.hpp"  // R4b: MapPointPtr
#include "mapping/LocalMapping.hpp"
#include "map/Atlas.hpp"
#include "recognition/OrbVocabulary.hpp"

#include "recognition/KeyFrameDatabase.hpp"
#include "closing/PlaceRecognition.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <thread>
#include <mutex>
#include "g2o/types/sim3/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;
struct BAEpochs;
class ILoopOptimizer;


class LoopClosing
{
public:

    typedef std::map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, const bool bFixScale, const bool bActiveLC, BAEpochs* pBAEpochs, ILoopOptimizer* pOptimizer);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();
    void RequestResetActiveMap(Map* pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        std::unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

    void RequestFinish();

    // P10-5: GBA custody transfer + reap, called by System::Shutdown ONLY,
    // strictly AFTER the LC thread has been joined (custody chain,
    // docs/OWNERSHIP.md GBA section: mThreadGBA is LC-thread-confined while
    // LC lives, System-confined once LC is joined). Sets mbStopGBA and bumps
    // the epoch under mMutexGBA, then joins OUTSIDE the lock (the GBA tail
    // takes mMutexGBA -- joining under it could deadlock).
    void StopAndJoinGBA();

    bool isFinished();

#ifdef REGISTER_TIMES

    std::vector<double> vdDataQuery_ms;
    std::vector<double> vdEstSim3_ms;
    std::vector<double> vdPRTotal_ms;

    std::vector<double> vdMergeMaps_ms;
    std::vector<double> vdWeldingBA_ms;
    std::vector<double> vdMergeOptEss_ms;
    std::vector<double> vdMergeTotal_ms;
    std::vector<int> vnMergeKFs;
    std::vector<int> vnMergeMPs;
    int nMerges;

    std::vector<double> vdLoopFusion_ms;
    std::vector<double> vdLoopOptEss_ms;
    std::vector<double> vdLoopTotal_ms;
    int nLoop;

    std::vector<double> vdGBA_ms;
    std::vector<double> vdUpdateMap_ms;
    std::vector<double> vdFGBATotal_ms;
    std::vector<int> vnGBAKFs;
    std::vector<int> vnGBAMPs;
    int nFGBA_exec;
    int nFGBA_abort;

#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();

    // P9-5: the queue pop + SetNotErase block of NewDetectCommonRegions,
    // kept here so EVERY mlpLoopKeyFrameQueue mutation stays inside
    // LoopClosing (mirror of P8-4's PurgeNewKeyFramesAfterInertialInit);
    // called by PlaceRecognition::NewDetectCommonRegions via mHost.
    void PopNewKeyFrame();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, std::vector<MapPointPtr> &vpMapPoints);
    void SearchAndFuse(const std::vector<KeyFrame*> &vConectedKFs, std::vector<MapPointPtr> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map* mpMapToReset;
    std::mutex mMutexReset;
    // P10-5: reset handshake CV (paired with mMutexReset), the LC twin of
    // LocalMapping::mCondReset. RequestReset* wait on "my flag cleared";
    // ResetIfRequested notifies after clearing. Preserved upstream quirk
    // (docs/P10_RECON.md 2부 item 2, R-f second half): a reset requested
    // after SetFinish() never returns -- Run() has exited and nothing
    // services the flag. Documented, NOT fixed (no finished-escape).
    std::condition_variable mCondReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;

    // Persistent local-BA epoch marks, owned by System (P5-C).
    BAEpochs* mpBAEpochs;

    // Narrow optimizer interface, owned by System (G2oBackend value member,
    // P6-A ISP split — see include/backend/ILoopOptimizer.hpp). Also used by
    // the detached GBA thread (stateless const contract, same thread
    // semantics as the previous static calls).
    ILoopOptimizer* mpOptimizer;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;
    // P10-5: queue CV (paired with mMutexLoopQueue), the LC twin of
    // LocalMapping::mCondNewKFs. InsertKeyFrame notifies after unlocking;
    // Run()'s tail waits on "queue non-empty" with a 5ms timed net. The net
    // is PERMANENT by design (docs/P10_RECON.md 2부 item 2, same rationale
    // as the LM 3ms net): the loop's other wake reasons -- finish and reset
    // -- live under OTHER mutexes (mMutexFinish/mMutexReset), and a pure
    // untimed wait would need them in this predicate (lock nesting or more
    // atomics). The timed net keeps their historical <=5ms latency bound
    // while making loop/merge-detection pickup of a queued KF immediate.
    std::condition_variable mCondLoopQueue;

    // P10-0: opt-in queue-latency tracing (ORB_TRACE_QUEUE=1), the LC twin
    // of LocalMapping's instrumentation — see the comment there. Stamp deque
    // shadows mlpLoopKeyFrameQueue under the SAME mMutexLoopQueue; samples
    // and counters are LC-thread-only; summary emitted from SetFinish().
    std::deque<std::chrono::steady_clock::time_point> mdqTraceEnqueueTs;
    std::vector<long> mvTraceDequeueUs;
    std::size_t mnTraceMaxDepth = 0;
    unsigned long mnTraceIters = 0;
    unsigned long mnTraceEmptyIters = 0;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpLastCurrentKF;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPointPtr> mvpLoopMapPoints;

    //-------
    Map* mpLastMap;

    // Consume-side members, NOT machine state (written once per consume
    // from the channel, read by CorrectLoop/MergeLocal/MergeLocal2).
    g2o::Sim3 mg2oLoopScw;
    g2o::Sim3 mg2oMergeScw;
    std::vector<KeyFrame*> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    // P10-1: atomic (was plain bool, ledger race R-e). LC writes it (abort
    // paths under mMutexGBA, spawn resets lock-free as before); the GBA
    // thread reads it and, through the OrbLevenberg shadow bridge, the
    // optimizer polls it. All accesses relaxed — advisory flag only.
    std::atomic<bool> mbStopGBA;
    std::mutex mMutexGBA;
    // P10-5: owned VALUE member (was `std::thread* mpThreadGBA`, docs/
    // P10_RECON.md 2부 item 6). Custody chain (docs/OWNERSHIP.md): LC-thread
    // -confined while LC lives; System-confined after System::Shutdown joins
    // the LC thread; reaped by StopAndJoinGBA before SaveAtlas. The abort
    // paths NO LONGER detach (flag + epoch only) -- every GBA stays
    // joinable, so the next spawn's reap-join (or Shutdown) always accounts
    // for it. The reap-join always executes OUTSIDE any mMutexGBA scope:
    // the exiting GBA's tail takes mMutexGBA.
    std::thread mThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    // P10-2: atomic (ledger race R-d). The GBA thread's first epoch read
    // is lock-free (relaxed); increments stay inside the existing
    // mMutexGBA scopes, and the under-lock recheck pairs with them.
    std::atomic<int> mnFullBAIdx;


    // To (de)activate LC
    bool mbActiveLC = true;

    // P9-5: detection machinery (NewDetectCommonRegions + the P9-4 channel
    // machine) extracted to a collaborator; it reaches host state through
    // the friend grant (P8-4 ImuInitializer pattern). Run() consumes its
    // hypotheses via the const LoopCh()/MergeCh() accessors and the named
    // wipe entry points. Declared last; initialized last in the ctor list.
    friend class PlaceRecognition;
    // P11-F3: test-only access grant (twin of the PlaceRecognition.hpp
    // declaration) -- lets tests/fixlevel/detection_machine_checks.cpp set
    // mpCurrentKF and drive ResetIfRequested on a minimally-constructed
    // host. Defined only in that test binary.
    friend struct DetectionMachineTestAccess;
    PlaceRecognition mPlaceRec;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
