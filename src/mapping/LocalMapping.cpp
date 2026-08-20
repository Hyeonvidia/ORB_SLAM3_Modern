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


#include "mapping/LocalMapping.hpp"
#include "closing/LoopClosing.hpp"
#include "tracking/Tracking.hpp"  // P8-3: was transitive via LocalMapping.hpp
#include "features/ORBmatcher.hpp"
#include "backend/IMappingOptimizer.hpp"
#include "backend/BAEpochs.hpp"
#include "backend/GBAResult.hpp"
#include "io/Converter.hpp"
#include "geometry/GeometricTools.hpp"
#include "core/Verbose.hpp"  // P7-1b: was transitive via Tracking.hpp -> System.hpp

#include<mutex>
#include<chrono>
#include<algorithm>
#include<cstdio>
#include<cstdlib>

namespace ORB_SLAM3
{

namespace
{
// P10-0: env gate for the opt-in queue-latency trace (ORB_TRACE_QUEUE,
// P7-1a pattern). getenv runs once; afterwards every call is one static
// bool test. Anything other than unset/empty/"0" enables tracing.
bool TraceQueueOn()
{
    static const bool bOn = []{
        const char* p = std::getenv("ORB_TRACE_QUEUE");
        return p && p[0] != '\0' && std::string(p) != "0";
    }();
    return bOn;
}
} // anonymous namespace

LocalMapping::LocalMapping(Atlas *pAtlas, bool bMonocular, bool bInertial, BAEpochs* pBAEpochs, IMappingOptimizer* pOptimizer):
    // P10-2: mFirstTs/mpCurrentKeyFrame need determinate init as atomics
    // (both were uninitialized until their first store); listed in
    // declaration order.
    mFirstTs(0.0),
    mbMonocular(bMonocular), mbInertial(bInertial), mbResetRequested(false), mbResetRequestedActiveMap(false),
    mbFinishRequested(false), mbFinished(true), mpAtlas(pAtlas), mpBAEpochs(pBAEpochs), mpOptimizer(pOptimizer),
    mpCurrentKeyFrame(nullptr),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), bInitializing(false),
    mImuInit(*this)
{
    mbBadImu = false;

    mTinit = 0.f;

#ifdef REGISTER_TIMES
    nLBA_exec = 0;
    nLBA_abort = 0;
#endif

}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LocalMapping::Run()
{
    {
        // P10-2 (R-a): the entry write raced isFinished readers; now under
        // the owning mutex (NOT atomic -- SetFinish couples it with mbStopped).
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinished = false;
    }

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        if(TraceQueueOn())  // P10-0 queue trace, diagnostic only
            ++mnTraceIters;

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames() && !mbBadImu.load(std::memory_order_relaxed))
        {
#ifdef REGISTER_TIMES
            double timeLBA_ms = 0;
            double timeKFCulling_ms = 0;

            std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
#endif
            // BoW conversion and insertion in Map
            ProcessNewKeyFrame();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

            double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
            vdKFInsert_ms.push_back(timeProcessKF);
#endif

            // Check recent MapPoints
            MapPointCulling();
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

            double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
            vdMPCulling_ms.push_back(timeMPCulling);
#endif

            // Triangulate new MapPoints
            CreateNewMapPoints();

            mbAbortBA.store(false, std::memory_order_relaxed);

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                SearchInNeighbors();
            }

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

            double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
            vdMPCreation_ms.push_back(timeMPCreation);
#endif

            // Consumed only by the REGISTER_TIMES accounting below.
            [[maybe_unused]] bool b_doneLBA = false;
            int num_FixedKF_BA = 0;
            int num_OptKF_BA = 0;
            int num_MPs_BA = 0;
            int num_edges_BA = 0;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                if(mpAtlas->KeyFramesInMap()>2)
                {

                    if(mbInertial && mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->isImuInitialized())
                    {
                        float dist = (mpCurrentKeyFrame.load(std::memory_order_relaxed)->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetCameraCenter()).norm() +
                                (mpCurrentKeyFrame.load(std::memory_order_relaxed)->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame.load(std::memory_order_relaxed)->mPrevKF->GetCameraCenter()).norm();

                        if(dist>0.05)
                            mTinit += mpCurrentKeyFrame.load(std::memory_order_relaxed)->mTimeStamp - mpCurrentKeyFrame.load(std::memory_order_relaxed)->mPrevKF->mTimeStamp;
                        if(!mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->GetIniertialBA2())
                        {
                            if((mTinit<10.f) && (dist<0.02))
                            {
                                std::cout << "Not enough motion for initializing. Reseting..." << std::endl;
                                std::unique_lock<std::mutex> lock(mMutexReset);
                                mbResetRequestedActiveMap.store(true, std::memory_order_relaxed);
                                mpMapToReset = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap();
                                mbBadImu.store(true, std::memory_order_relaxed);
                            }
                        }

                        bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
                        mpOptimizer->LocalInertialBA(mpCurrentKeyFrame.load(std::memory_order_relaxed), &mbAbortBA, mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->GetIniertialBA2(), *mpBAEpochs);
                        b_doneLBA = true;
                    }
                    else
                    {
                        mpOptimizer->LocalBundleAdjustment(mpCurrentKeyFrame.load(std::memory_order_relaxed),&mbAbortBA, mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, *mpBAEpochs);
                        b_doneLBA = true;
                    }

                }
#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

                if(b_doneLBA)
                {
                    timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLBA - time_EndMPCreation).count();
                    vdLBA_ms.push_back(timeLBA_ms);

                    nLBA_exec += 1;
                    if(mbAbortBA.load(std::memory_order_relaxed))
                    {
                        nLBA_abort += 1;
                    }
                    vnLBA_edges.push_back(num_edges_BA);
                    vnLBA_KFopt.push_back(num_OptKF_BA);
                    vnLBA_KFfixed.push_back(num_FixedKF_BA);
                    vnLBA_MPs.push_back(num_MPs_BA);
                }

#endif

                // Initialize IMU here
                if(!mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->isImuInitialized() && mbInertial)
                {
                    if (mbMonocular)
                        mImuInit.InitializeIMU(1e2, 1e10, true);
                    else
                        mImuInit.InitializeIMU(1e2, 1e5, true);
                }


                // Check redundant local Keyframes
                KeyFrameCulling();

#ifdef REGISTER_TIMES
                std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

                timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
                vdKFCulling_ms.push_back(timeKFCulling_ms);
#endif

                if ((mTinit<50.0f) && mbInertial)
                {
                    if(mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->isImuInitialized() && mpTracker->GetState()==Tracking::OK) // Enter here everytime local-mapping is called
                    {
                        if(!mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->GetIniertialBA1()){
                            if (mTinit>5.0f)
                            {
                                std::cout << "start VIBA 1" << std::endl;
                                mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->SetIniertialBA1();
                                mImuInit.InitializeIMU(1.f, 1e5, true);
                                std::cout << "end VIBA 1" << std::endl;
                            }
                        }
                        else if(!mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->GetIniertialBA2()){
                            if (mTinit>15.0f){
                                std::cout << "start VIBA 2" << std::endl;
                                mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->SetIniertialBA2();
                                mImuInit.InitializeIMU(0.f, 0.f, true);
                                std::cout << "end VIBA 2" << std::endl;
                            }
                        }

                        // scale refinement
                        if (((mpAtlas->KeyFramesInMap())<=200) &&
                                ((mTinit>25.0f && mTinit<25.5f)||
                                (mTinit>35.0f && mTinit<35.5f)||
                                (mTinit>45.0f && mTinit<45.5f)||
                                (mTinit>55.0f && mTinit<55.5f)||
                                (mTinit>65.0f && mTinit<65.5f)||
                                (mTinit>75.0f && mTinit<75.5f))){
                            if (mbMonocular)
                                mImuInit.ScaleRefinement();
                        }
                    }
                }
            }

#ifdef REGISTER_TIMES
            vdLBASync_ms.push_back(timeKFCulling_ms);
            vdKFCullingSync_ms.push_back(timeKFCulling_ms);
#endif

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame.load(std::memory_order_relaxed));

#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

            double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
            vdLMTotal_ms.push_back(timeLocalMap);
#endif
        }
        else if(Stop() && !mbBadImu.load(std::memory_order_relaxed))
        {
            // Safe area to stop
            //
            // P10-4: CV park (was `while(isStopped() && !CheckFinish())
            // usleep(3000)`). Woken by Release() (clears mbStopped),
            // RequestFinish()/SetFinish(). The predicate reads the ATOMIC
            // finish flag -- taking mMutexFinish here would nest
            // Stop->Finish against the canonical Finish->Stop order.
            // Invariant 5 preserved: reset flags are deliberately absent
            // from the predicate (a reset during park waits for Release).
            // Invariant 4 preserved: the mbBadImu quirk skips this branch
            // via the same enclosing condition as before.
            {
                std::unique_lock<std::mutex> lock(mMutexStop);
                mCondStop.wait(lock, [&]{
                    return !mbStopped ||
                           mbFinishRequested.load(std::memory_order_relaxed);
                });
            }
            if(CheckFinish())
                break;
        }
        else if(TraceQueueOn())  // P10-0: neither processed nor parked
            ++mnTraceEmptyIters;

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        // P10-4: CV'd queue wait (was `usleep(3000)`). The 3ms timed net is
        // PERMANENT by design (docs/P10_RECON.md 2부 item 1, rationale at
        // the mCondNewKFs declaration): stop/finish/reset/mbBadImu wakeups
        // live under other mutexes and keep their historical <=3ms bound via
        // the timeout; a queued KF now wakes the loop immediately instead.
        // An empty-queue timeout still runs a full iteration, so the P10-0
        // trace counters (mnTraceIters/mnTraceEmptyIters) keep counting
        // iterations exactly as under the old poll.
        // TSAN visibility (P10-4 postmortem): wait_for compiles to
        // pthread_cond_clockwait (CLOCK_MONOTONIC), which gcc-11 libtsan
        // does NOT intercept -- every timeout would silently break TSAN's
        // mutex modeling for mMutexNewKFs and spray spurious race reports
        // on everything the queue mutex protects. system_clock wait_until
        // maps to the intercepted pthread_cond_timedwait; a realtime clock
        // jump can only stretch or clip one 3ms net (harmless -- the
        // predicate re-check handles it). Do not "modernize" this back.
        {
            std::unique_lock<std::mutex> lock(mMutexNewKFs);
            mCondNewKFs.wait_until(lock,
                                   std::chrono::system_clock::now() + std::chrono::milliseconds(3),
                                   [&]{ return !mlNewKeyFrames.empty(); });
        }
    }

    SetFinish();
}

void LocalMapping::InsertKeyFrame(KeyFramePtr pKF)
{
    {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        mlNewKeyFrames.push_back(pKF);
        mbAbortBA.store(true, std::memory_order_relaxed);
        if(TraceQueueOn())  // P10-0: enqueue stamp, same lock as the queue
        {
            mdqTraceEnqueueTs.push_back(std::chrono::steady_clock::now());
            if(mlNewKeyFrames.size() > mnTraceMaxDepth)
                mnTraceMaxDepth = mlNewKeyFrames.size();
        }
    }
    // P10-4: wake Run()'s queue wait. Notify AFTER unlock so the woken
    // consumer never immediately blocks on the mutex we still hold.
    mCondNewKFs.notify_one();
}


bool LocalMapping::CheckNewKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

void LocalMapping::ProcessNewKeyFrame()
{
    {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        // P10-2 (R5): store-release -- with the mMutexNewKFs handoff this
        // makes the Tracking-thread KF construction visible to the
        // lock-free load-acquire in GetCurrKFTime.
        mpCurrentKeyFrame.store(mlNewKeyFrames.front(), std::memory_order_release);
        mlNewKeyFrames.pop_front();
        if(TraceQueueOn() && !mdqTraceEnqueueTs.empty())  // P10-0 dequeue
        {
            mvTraceDequeueUs.push_back(static_cast<long>(std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - mdqTraceEnqueueTs.front()).count()));
            mdqTraceEnqueueTs.pop_front();
        }
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame.load(std::memory_order_relaxed)->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const std::vector<MapPointPtr> vpMapPointMatches = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPointPtr pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->IsInKeyFrame(mpCurrentKeyFrame.load(std::memory_order_relaxed)))
                {
                    pMP->AddObservation(mpCurrentKeyFrame.load(std::memory_order_relaxed), i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame.load(std::memory_order_relaxed)->UpdateConnections();

    // Insert Keyframe in Map
    mpAtlas->AddKeyFrame(mpCurrentKeyFrame.load(std::memory_order_relaxed));
}

void LocalMapping::EmptyQueue()
{
    while(CheckNewKeyFrames())
        ProcessNewKeyFrame();
}

void LocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    std::list<MapPointPtr>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mnId;

    int nThObs;
    if(mbMonocular)
        nThObs = 2;
    else
        nThObs = 3;
    const int cnThObs = nThObs;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        MapPointPtr pMP = *lit;

        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if((static_cast<int>(nCurrentKFid)-static_cast<int>(pMP->mnFirstKFid))>=2 && pMP->Observations()<=cnThObs)
        {
            pMP->SetBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if((static_cast<int>(nCurrentKFid)-static_cast<int>(pMP->mnFirstKFid))>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
        {
            lit++;
        }
    }
}


void LocalMapping::CreateNewMapPoints()
{
    // Retrieve neighbor keyframes in covisibility graph
    int nn = 10;
    // For stereo inertial case
    if(mbMonocular)
        nn=30;
    std::vector<KeyFramePtr> vpNeighKFs = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetBestCovisibilityKeyFrames(nn);

    if (mbInertial)
    {
        KeyFramePtr pKF = mpCurrentKeyFrame.load(std::memory_order_relaxed);
        int count=0;
        while((vpNeighKFs.size()<=static_cast<size_t>(nn))&&(pKF->mPrevKF)&&(count++<nn))
        {
            std::vector<KeyFramePtr>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
            if(it==vpNeighKFs.end())
                vpNeighKFs.push_back(pKF->mPrevKF);
            pKF = pKF->mPrevKF;
        }
    }

    float th = 0.6f;

    ORBmatcher matcher(th,false);

    Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetPose();
    Eigen::Matrix<float,3,4> eigTcw1 = sophTcw1.matrix3x4();
    Eigen::Matrix<float,3,3> Rcw1 = eigTcw1.block<3,3>(0,0);
    Eigen::Matrix<float,3,3> Rwc1 = Rcw1.transpose();
    Eigen::Vector3f tcw1 = sophTcw1.translation();
    Eigen::Vector3f Ow1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->fx;
    const float &fy1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->fy;
    const float &cx1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->cx;
    const float &cy1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->cy;

    const float ratioFactor = 1.5f*mpCurrentKeyFrame.load(std::memory_order_relaxed)->mfScaleFactor;
    // Search matches with epipolar restriction and triangulate
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        KeyFramePtr pKF2 = vpNeighKFs[i];

        GeometricCamera* pCamera1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mpCamera, *pCamera2 = pKF2->mpCamera;

        // Check first that baseline is not too short
        Eigen::Vector3f Ow2 = pKF2->GetCameraCenter();
        Eigen::Vector3f vBaseline = Ow2-Ow1;
        const float baseline = vBaseline.norm();

        if(!mbMonocular)
        {
            if(baseline<pKF2->mb)
                continue;
        }
        else
        {
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            const float ratioBaselineDepth = baseline/medianDepthKF2;

            if(ratioBaselineDepth<0.01)
                continue;
        }

        // Search matches that fullfil epipolar constraint
        std::vector<std::pair<size_t,size_t> > vMatchedIndices;
        bool bCoarse = mbInertial && mpTracker->GetState()==Tracking::RECENTLY_LOST && mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->GetIniertialBA2();

        matcher.SearchForTriangulation(mpCurrentKeyFrame.load(std::memory_order_relaxed),pKF2,vMatchedIndices,false,bCoarse);

        Sophus::SE3<float> sophTcw2 = pKF2->GetPose();
        Eigen::Matrix<float,3,4> eigTcw2 = sophTcw2.matrix3x4();
        Eigen::Matrix<float,3,3> Rcw2 = eigTcw2.block<3,3>(0,0);
        Eigen::Matrix<float,3,3> Rwc2 = Rcw2.transpose();
        Eigen::Vector3f tcw2 = sophTcw2.translation();

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = (mpCurrentKeyFrame.load(std::memory_order_relaxed) -> NLeft == -1) ? mpCurrentKeyFrame.load(std::memory_order_relaxed)->mvKeysUn[idx1]
                                                                         : (idx1 < mpCurrentKeyFrame.load(std::memory_order_relaxed) -> NLeft) ? mpCurrentKeyFrame.load(std::memory_order_relaxed) -> mvKeys[idx1]
                                                                                                               : mpCurrentKeyFrame.load(std::memory_order_relaxed) -> mvKeysRight[idx1 - mpCurrentKeyFrame.load(std::memory_order_relaxed) -> NLeft];
            const float kp1_ur=mpCurrentKeyFrame.load(std::memory_order_relaxed)->mvuRight[idx1];
            bool bStereo1 = (!mpCurrentKeyFrame.load(std::memory_order_relaxed)->mpCamera2 && kp1_ur>=0);
            const bool bRight1 = (mpCurrentKeyFrame.load(std::memory_order_relaxed) -> NLeft == -1 || idx1 < mpCurrentKeyFrame.load(std::memory_order_relaxed) -> NLeft) ? false
                                                                                                         : true;

            const cv::KeyPoint &kp2 = (pKF2 -> NLeft == -1) ? pKF2->mvKeysUn[idx2]
                                                            : (idx2 < pKF2 -> NLeft) ? pKF2 -> mvKeys[idx2]
                                                                                     : pKF2 -> mvKeysRight[idx2 - pKF2 -> NLeft];

            const float kp2_ur = pKF2->mvuRight[idx2];
            bool bStereo2 = (!pKF2->mpCamera2 && kp2_ur>=0);
            const bool bRight2 = (pKF2 -> NLeft == -1 || idx2 < pKF2 -> NLeft) ? false
                                                                               : true;

            if(mpCurrentKeyFrame.load(std::memory_order_relaxed)->mpCamera2 && pKF2->mpCamera2){
                if(bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetRightPose();
                    Ow1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mpCamera2;
                    pCamera2 = pKF2->mpCamera2;
                }
                else if(bRight1 && !bRight2){
                    sophTcw1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetRightPose();
                    Ow1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetRightCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mpCamera2;
                    pCamera2 = pKF2->mpCamera;
                }
                else if(!bRight1 && bRight2){
                    sophTcw1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetPose();
                    Ow1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetCameraCenter();

                    sophTcw2 = pKF2->GetRightPose();
                    Ow2 = pKF2->GetRightCameraCenter();

                    pCamera1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mpCamera;
                    pCamera2 = pKF2->mpCamera2;
                }
                else{
                    sophTcw1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetPose();
                    Ow1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetCameraCenter();

                    sophTcw2 = pKF2->GetPose();
                    Ow2 = pKF2->GetCameraCenter();

                    pCamera1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mpCamera;
                    pCamera2 = pKF2->mpCamera;
                }
                eigTcw1 = sophTcw1.matrix3x4();
                Rcw1 = eigTcw1.block<3,3>(0,0);
                Rwc1 = Rcw1.transpose();
                tcw1 = sophTcw1.translation();

                eigTcw2 = sophTcw2.matrix3x4();
                Rcw2 = eigTcw2.block<3,3>(0,0);
                Rwc2 = Rcw2.transpose();
                tcw2 = sophTcw2.translation();
            }

            // Check parallax between rays
            Eigen::Vector3f xn1 = pCamera1->unprojectEig(kp1.pt);
            Eigen::Vector3f xn2 = pCamera2->unprojectEig(kp2.pt);

            Eigen::Vector3f ray1 = Rwc1 * xn1;
            Eigen::Vector3f ray2 = Rwc2 * xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

            float cosParallaxStereo = cosParallaxRays+1;
            float cosParallaxStereo1 = cosParallaxStereo;
            float cosParallaxStereo2 = cosParallaxStereo;

            if(bStereo1)
                cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame.load(std::memory_order_relaxed)->mb/2,mpCurrentKeyFrame.load(std::memory_order_relaxed)->mvDepth[idx1]));
            else if(bStereo2)
                cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

            cosParallaxStereo = std::min(cosParallaxStereo1,cosParallaxStereo2);

            Eigen::Vector3f x3D;

            bool goodProj = false;
            if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
                                                                          (cosParallaxRays<0.9996 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
            {
                goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                if(!goodProj)
                    continue;
            }
            else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
            {
                goodProj = mpCurrentKeyFrame.load(std::memory_order_relaxed)->UnprojectStereo(idx1, x3D);
            }
            else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
            {
                goodProj = pKF2->UnprojectStereo(idx2, x3D);
            }
            else
            {
                continue; //No stereo and very low parallax
            }

            if(!goodProj)
                continue;

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3D)+tcw1(0);
            const float y1 = Rcw1.row(1).dot(x3D)+tcw1(1);
            const float invz1 = 1.0/z1;

            if(!bStereo1)
            {
                cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                float errX1 = uv1.x - kp1.pt.x;
                float errY1 = uv1.y - kp1.pt.y;

                if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                    continue;

            }
            else
            {
                float u1 = fx1*x1*invz1+cx1;
                float u1_r = u1 - mpCurrentKeyFrame.load(std::memory_order_relaxed)->mbf*invz1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                float errX1_r = u1_r - kp1_ur;
                if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3D)+tcw2(0);
            const float y2 = Rcw2.row(1).dot(x3D)+tcw2(1);
            const float invz2 = 1.0/z2;
            if(!bStereo2)
            {
                cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                float errX2 = uv2.x - kp2.pt.x;
                float errY2 = uv2.y - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                    continue;
            }
            else
            {
                float u2 = fx2*x2*invz2+cx2;
                float u2_r = u2 - mpCurrentKeyFrame.load(std::memory_order_relaxed)->mbf*invz2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                float errX2_r = u2_r - kp2_ur;
                if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            Eigen::Vector3f normal1 = x3D - Ow1;
            float dist1 = normal1.norm();

            Eigen::Vector3f normal2 = x3D - Ow2;
            float dist2 = normal2.norm();

            if(dist1==0 || dist2==0)
                continue;

            if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                continue;

            const float ratioDist = dist2/dist1;
            const float ratioOctave = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            MapPointPtr pMP = std::make_shared<MapPoint>(x3D, mpCurrentKeyFrame.load(std::memory_order_relaxed), mpAtlas->GetCurrentMap());

            pMP->AddObservation(mpCurrentKeyFrame.load(std::memory_order_relaxed),idx1);
            pMP->AddObservation(pKF2,idx2);

            mpCurrentKeyFrame.load(std::memory_order_relaxed)->AddMapPoint(pMP,idx1);
            pKF2->AddMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpAtlas->AddMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);
        }
    }    
}

void LocalMapping::SearchInNeighbors()
{
    // P5-2: per-call dedup scratch, externalized from KeyFrame::mnFuseTargetForKF
    // and MapPoint::mnFuseCandidateForKF (their lifetime was one call anyway).
    std::set<KeyFramePtr> sFuseTargets;
    std::set<MapPointPtr> sFuseCandidates;
    // Retrieve neighbor keyframes
    int nn = 10;
    if(mbMonocular)
        nn=30;
    const std::vector<KeyFramePtr> vpNeighKFs = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetBestCovisibilityKeyFrames(nn);
    std::vector<KeyFramePtr> vpTargetKFs;
    for(KeyFramePtr pKFi : vpNeighKFs)
    {
        if(pKFi->isBad() || sFuseTargets.count(pKFi))
        {
            continue;
        }
        vpTargetKFs.push_back(pKFi);
        sFuseTargets.insert(pKFi);
    }

    // Add some covisible of covisible
    // Extend to some second neighbors if abort is not requested
    for(int i=0, imax=vpTargetKFs.size(); i<imax; i++)
    {
        const std::vector<KeyFramePtr> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        for(KeyFramePtr pKFi2 : vpSecondNeighKFs)
        {
            if(pKFi2->isBad() || sFuseTargets.count(pKFi2) || pKFi2->mnId==mpCurrentKeyFrame.load(std::memory_order_relaxed)->mnId)
            {
                continue;
            }
            vpTargetKFs.push_back(pKFi2);
            sFuseTargets.insert(pKFi2);
        }
        if (mbAbortBA.load(std::memory_order_relaxed))
            break;
    }

    // Extend to temporal neighbors
    if(mbInertial)
    {
        KeyFramePtr pKFi = mpCurrentKeyFrame.load(std::memory_order_relaxed)->mPrevKF;
        while(vpTargetKFs.size()<20 && pKFi)
        {
            if(pKFi->isBad() || sFuseTargets.count(pKFi))
            {
                pKFi = pKFi->mPrevKF;
                continue;
            }
            vpTargetKFs.push_back(pKFi);
            sFuseTargets.insert(pKFi);
            pKFi = pKFi->mPrevKF;
        }
    }

    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    std::vector<MapPointPtr> vpMapPointMatches = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMapPointMatches();
    for(KeyFramePtr pKFi : vpTargetKFs)
    {

        matcher.Fuse(pKFi,vpMapPointMatches);
        if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,true);
    }


    if (mbAbortBA.load(std::memory_order_relaxed))
        return;

    // Search matches by projection from target KFs in current KF
    std::vector<MapPointPtr> vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

    for(KeyFramePtr pKFi : vpTargetKFs)
    {

        std::vector<MapPointPtr> vpMapPointsKFi = pKFi->GetMapPointMatches();

        for(std::vector<MapPointPtr>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            MapPointPtr pMP = *vitMP;
            if(!pMP)
                continue;
            if(pMP->isBad() || sFuseCandidates.count(pMP))
            {
                continue;
            }
            sFuseCandidates.insert(pMP);
            vpFuseCandidates.push_back(pMP);
        }
    }

    matcher.Fuse(mpCurrentKeyFrame.load(std::memory_order_relaxed),vpFuseCandidates);
    if(mpCurrentKeyFrame.load(std::memory_order_relaxed)->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame.load(std::memory_order_relaxed),vpFuseCandidates,true);


    // Update points
    vpMapPointMatches = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMapPointMatches();
    for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
    {
        MapPointPtr pMP=vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame.load(std::memory_order_relaxed)->UpdateConnections();
}

void LocalMapping::RequestStop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopRequested = true;
    // P10-1: the mMutexNewKFs nest existed only to guard this write; with
    // mbAbortBA atomic it is gone — which also removes the codebase's only
    // Stop->NewKFs lock-order edge (docs/P10_RECON.md 2부 item 4).
    mbAbortBA.store(true, std::memory_order_relaxed);
}

bool LocalMapping::Stop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        std::cout << "Local Mapping STOP" << std::endl;
        // P10-4: wake WaitUntilStopped() waiters (LoopClosing windows, GBA,
        // System localization-mode). mbStopped is written under mMutexStop,
        // so notifying under the same lock cannot lose a wakeup.
        mCondStop.notify_all();
        return true;
    }

    return false;
}

bool LocalMapping::isStopped()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopped;
}

void LocalMapping::WaitUntilStopped()
{
    // P10-4: replaces the 7 external `while(!isStopped()) usleep` spins.
    // Woken by Stop() and by SetFinish() -- SetFinish also sets mbStopped,
    // so the old finish-aware GBA variant (`!isStopped() && !isFinished()`)
    // converges on this same predicate (identical to the polling semantics,
    // where SetFinish made isStopped() true). Invariant 4 preserved: with
    // mbBadImu, Stop() still sets mbStopped and notifies while Run() keeps
    // looping -- external waiters proceed exactly as under the old spin.
    std::unique_lock<std::mutex> lock(mMutexStop);
    mCondStop.wait(lock, [&]{ return mbStopped; });
}

bool LocalMapping::stopRequested()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalMapping::Release()
{
    {
        // P10-2 (R6): Finish -> Stop, matching SetFinish. The old Stop -> Finish
        // order was an ABBA deadlock against a concurrent SetFinish (found in
        // the P10 recon; canonical order documented in LocalMapping.hpp).
        std::unique_lock<std::mutex> lock(mMutexFinish);
        std::unique_lock<std::mutex> lock2(mMutexStop);
        if(mbFinished)
            return;
        mbStopped = false;
        mbStopRequested = false;
        {
            // P10-2 (R1): the delete-drain now holds mMutexNewKFs (innermost in
            // the canonical order) -- it raced push_back from the SetNotStop-less
            // initialization producers. Trace deque cleared under the same lock
            // (P10-0 shadow-deque contract).
            //
            // P11-A (B3, DIVERGENCES #29) demoted the raw delete to the #19
            // pattern. R4b slice 2 dissolves that leak-vs-UAF reasoning
            // entirely: the queue holds strong KeyFramePtrs, so disposal is
            // SetBadFlag (graph detach; no-op Map/KFDB erases for the
            // never-admitted) + dropping the reference — the KF is freed
            // when Tracking's own aliases (mpLastKeyFrame slot, frame
            // members) let go, and simply lives on if SetBadFlag's early
            // returns fired (the old "leak" arm, now just a pinned object).
            // SetBadFlag under mMutexNewKFs is safe:
            // it takes only KeyFrame/Map/KFDB-layer mutexes, and no path
            // acquires mMutexNewKFs while holding those (P10-2 R3 precedent).
            std::unique_lock<std::mutex> lock3(mMutexNewKFs);
            for(std::list<KeyFramePtr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
            {
                (*lit)->SetBadFlag();
            }
            mlNewKeyFrames.clear();
            if(TraceQueueOn())
                mdqTraceEnqueueTs.clear();
        }
    }

    // P10-4: wake the parked Run() (park predicate: !mbStopped). Notify
    // after unlock; mbStopped was cleared under mMutexStop above.
    mCondStop.notify_all();

    std::cout << "Local Mapping RELEASE" << std::endl;
}

bool LocalMapping::AcceptKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    std::unique_lock<std::mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool LocalMapping::SetNotStop(bool flag)
{
    std::unique_lock<std::mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void LocalMapping::InterruptBA()
{
    mbAbortBA.store(true, std::memory_order_relaxed);
}

void LocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    const int Nd = 21;
    mpCurrentKeyFrame.load(std::memory_order_relaxed)->UpdateBestCovisibles();
    std::vector<KeyFramePtr> vpLocalKeyFrames = mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetVectorCovisibleKeyFrames();

    float redundant_th;
    if(!mbInertial)
        redundant_th = 0.9;
    else if (mbMonocular)
        redundant_th = 0.9;
    else
        redundant_th = 0.5;

    const bool bInitImu = mpAtlas->isImuInitialized();
    int count=0;

    // Compoute last KF from optimizable window:
    // R1: initialized — only assigned on the mbInertial path; the read below is
    // short-circuit-guarded by bInitImu but gcc-13 flags it (-Wmaybe-uninitialized).
    unsigned int last_ID = 0;
    if (mbInertial)
    {
        int count = 0;
        KeyFramePtr aux_KF = mpCurrentKeyFrame.load(std::memory_order_relaxed);
        while(count<Nd && aux_KF->mPrevKF)
        {
            aux_KF = aux_KF->mPrevKF;
            count++;
        }
        last_ID = aux_KF->mnId;
    }



    for(std::vector<KeyFramePtr>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        count++;
        KeyFramePtr pKF = *vit;

        if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
        {
            continue;
        }
        const std::vector<MapPointPtr> vpMapPoints = pKF->GetMapPointMatches();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPointPtr pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(!mbMonocular)
                    {
                        if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                            continue;
                    }

                    nMPs++;
                    if(pMP->Observations()>thObs)
                    {
                        const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
                                                                     : (i < static_cast<size_t>(pKF -> NLeft)) ? pKF -> mvKeys[i].octave
                                                                                          : pKF -> mvKeysRight[i].octave;
                        const std::map<KeyFramePtr, std::tuple<int,int>> observations = pMP->GetObservations();
                        int nObs=0;
                        for(std::map<KeyFramePtr, std::tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            KeyFramePtr pKFi = mit->first;
                            if(pKFi==pKF)
                                continue;
                            std::tuple<int,int> indexes = mit->second;
                            int leftIndex = std::get<0>(indexes), rightIndex = get<1>(indexes);
                            int scaleLeveli = -1;
                            if(pKFi -> NLeft == -1)
                                scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
                            else {
                                if (leftIndex != -1) {
                                    scaleLeveli = pKFi->mvKeys[leftIndex].octave;
                                }
                                if (rightIndex != -1) {
                                    int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
                                    scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
                                                                                                  : scaleLeveli;
                                }
                            }

                            if(scaleLeveli<=scaleLevel+1)
                            {
                                nObs++;
                                if(nObs>thObs)
                                    break;
                            }
                        }
                        if(nObs>thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }

        if(nRedundantObservations>redundant_th*nMPs)
        {
            if (mbInertial)
            {
                if (mpAtlas->KeyFramesInMap()<=Nd)
                    continue;

                if(pKF->mnId>(mpCurrentKeyFrame.load(std::memory_order_relaxed)->mnId-2))
                    continue;

                // R4b slice 2: mNextKF is weak — pin it once for the whole
                // rewire (an expired next == "already rewired away": skip).
                KeyFramePtr pNextKF = pKF->mNextKF.lock();
                if(pKF->mPrevKF && pNextKF)
                {
                    const float t = pNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                    if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                    {
                        pNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pNextKF;
                        pKF->mNextKF.reset();
                        pKF->mPrevKF = nullptr;
                        pKF->SetBadFlag();
                    }
                    else if(!mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->GetIniertialBA2() && ((pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition()).norm()<0.02) && (t<3))
                    {
                        pNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                        pNextKF->mPrevKF = pKF->mPrevKF;
                        pKF->mPrevKF->mNextKF = pNextKF;
                        pKF->mNextKF.reset();
                        pKF->mPrevKF = nullptr;
                        pKF->SetBadFlag();
                    }
                }
            }
            else
            {
                pKF->SetBadFlag();
            }
        }
        if((count > 20 && mbAbortBA.load(std::memory_order_relaxed)) || count>100)
        {
            break;
        }
    }
}

void LocalMapping::RequestReset()
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        std::cout << "LM: Map reset recieved" << std::endl;
        mbResetRequested.store(true, std::memory_order_relaxed);
    }
    std::cout << "LM: Map reset, waiting..." << std::endl;

    // P10-4: CV handshake (was a 3ms spin). ResetIfRequested notifies
    // mCondReset after clearing the flag. Preserved upstream quirks, on
    // purpose (docs/P10_RECON.md 2부 item 1):
    //   - reset-while-parked still blocks here until some Release() lets
    //     Run() reach ResetIfRequested (invariant 5);
    //   - reset-after-SetFinish still NEVER returns (Run() has exited and
    //     nothing services the flag -- upstream behavior, documented; do
    //     NOT add a finished-escape, that would change observable protocol).
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mCondReset.wait(lock, [&]{
            return !mbResetRequested.load(std::memory_order_relaxed);
        });
    }
    std::cout << "LM: Map reset, Done!!!" << std::endl;
}

void LocalMapping::RequestResetActiveMap(Map* pMap)
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        std::cout << "LM: Active map reset recieved" << std::endl;
        mbResetRequestedActiveMap.store(true, std::memory_order_relaxed);
        mpMapToReset = pMap;
    }
    std::cout << "LM: Active map reset, waiting..." << std::endl;

    // P10-4: CV handshake (was a 3ms spin); same preserved quirks as
    // RequestReset above (park blocks until Release; after SetFinish this
    // never returns -- upstream, documented, no finished-escape).
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mCondReset.wait(lock, [&]{
            return !mbResetRequestedActiveMap.load(std::memory_order_relaxed);
        });
    }
    std::cout << "LM: Active map reset, Done!!!" << std::endl;
}

void LocalMapping::ResetIfRequested()
{
    bool executed_reset = false;
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if(mbResetRequested.load(std::memory_order_relaxed))
        {
            executed_reset = true;

            std::cout << "LM: Reseting Atlas in Local Mapping..." << std::endl;
            {
                // P10-2: queue clear closed under mMutexNewKFs (inner to
                // mMutexReset; was protocol-protected by the requester spin
                // -- closed for uniformity while in the area). Trace deque
                // cleared under the same lock (P10-0 shadow-deque contract).
                std::unique_lock<std::mutex> lockQueue(mMutexNewKFs);
                mlNewKeyFrames.clear();
                if(TraceQueueOn())
                    mdqTraceEnqueueTs.clear();
            }
            mlpRecentAddedMapPoints.clear();
            mbResetRequested.store(false, std::memory_order_relaxed);
            mbResetRequestedActiveMap.store(false, std::memory_order_relaxed);

            // Inertial parameters
            mTinit = 0.f;
            mbBadImu.store(false, std::memory_order_relaxed);

            std::cout << "LM: End reseting Local Mapping..." << std::endl;
        }

        if(mbResetRequestedActiveMap.load(std::memory_order_relaxed)) {
            executed_reset = true;
            std::cout << "LM: Reseting current map in Local Mapping..." << std::endl;
            {
                // P10-2: same closure as the full-reset branch above.
                std::unique_lock<std::mutex> lockQueue(mMutexNewKFs);
                mlNewKeyFrames.clear();
                if(TraceQueueOn())
                    mdqTraceEnqueueTs.clear();
            }
            mlpRecentAddedMapPoints.clear();

            // Inertial parameters
            mTinit = 0.f;
            mbBadImu.store(false, std::memory_order_relaxed);

            mbResetRequested.store(false, std::memory_order_relaxed);
            mbResetRequestedActiveMap.store(false, std::memory_order_relaxed);
            std::cout << "LM: End reseting Local Mapping..." << std::endl;
        }
    }
    if(executed_reset)
    {
        std::cout << "LM: Reset free the mutex" << std::endl;
        // P10-4: wake the RequestReset/RequestResetActiveMap handshake
        // waiters. The flags were cleared under mMutexReset above; notify
        // after unlock.
        mCondReset.notify_all();
    }

}

void LocalMapping::RequestFinish()
{
    {
        // P10-2: the store keeps the lock (it still orders against the
        // mbFinished handshake); readers are lock-free.
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinishRequested.store(true, std::memory_order_relaxed);
    }
    // P10-4: wake a parked Run() (park predicate reads the atomic finish
    // flag, which is NOT guarded by mMutexStop). The empty mMutexStop
    // acquisition is required: it orders the store above against the park
    // wait's predicate check -- without it the notify could land in the
    // window between the waiter's predicate evaluation and its block
    // (lost wakeup, and the park wait has no timeout). Lock order is
    // canonical (mMutexFinish released before mMutexStop is taken).
    { std::unique_lock<std::mutex> lock(mMutexStop); }
    mCondStop.notify_all();
}

bool LocalMapping::CheckFinish()
{
    // P10-2: lock-free (atomic flag; prep for the P10-4 park predicate).
    return mbFinishRequested.load(std::memory_order_relaxed);
}

void LocalMapping::SetFinish()
{
    if(TraceQueueOn())  // P10-0: one summary line, before any finish locks
    {
        std::size_t nMaxDepth;
        {
            std::unique_lock<std::mutex> lock(mMutexNewKFs);
            nMaxDepth = mnTraceMaxDepth;
        }
        std::vector<long> v = mvTraceDequeueUs;
        std::sort(v.begin(), v.end());
        const std::size_t n = v.size();
        std::fprintf(stderr,
            "[queue-trace] lm n=%zu p50_us=%ld p95_us=%ld max_us=%ld max_depth=%zu iters=%lu empty_iters=%lu\n",
            n, n ? v[(n-1)/2] : 0L, n ? v[(95*(n-1))/100] : 0L, n ? v.back() : 0L,
            nMaxDepth, mnTraceIters, mnTraceEmptyIters);
    }
    {
        std::unique_lock<std::mutex> lock(mMutexFinish);
        mbFinished = true;
        std::unique_lock<std::mutex> lock2(mMutexStop);
        mbStopped = true;
    }
    // P10-4: SetFinish sets mbStopped, so external WaitUntilStopped()
    // waiters (including the old finish-aware GBA wait) must be woken here
    // too. Notify after unlock; mbStopped was written under mMutexStop.
    mCondStop.notify_all();
}

bool LocalMapping::isFinished()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    return mbFinished;
}

void LocalMapping::PurgeNewKeyFramesAfterInertialInit()
{
    // P10-2 (R3): the purge now holds mMutexNewKFs -- Tracking's insertion
    // gate passes during bInitializing (IsInitializing bypass), so the
    // lock-free traversal raced InsertKeyFrame. SetBadFlag under this lock
    // is safe: it takes only KeyFrame/Map/KFDB-layer mutexes, and no code
    // path acquires mMutexNewKFs while holding those. Trace deque cleared
    // under the same lock (P10-0 shadow-deque contract).
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    for(std::list<KeyFramePtr>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
    {
        // DIVERGENCES #19 guarded a raw delete here; R4b slice 2 retires it:
        // disposal is SetBadFlag + dropping the strong queue reference. A
        // SetBadFlag early-return survivor (map-origin KF, mbNotErase) just
        // stays alive through its remaining pins — no leak-vs-UAF choice.
        (*lit)->SetBadFlag();
    }
    mlNewKeyFrames.clear();
    if(TraceQueueOn())
        mdqTraceEnqueueTs.clear();
}



bool LocalMapping::IsInitializing()
{
    // P10-2 (R5): lock-free cross-thread read, now atomic.
    return bInitializing.load(std::memory_order_relaxed);
}


double LocalMapping::GetCurrKFTime()
{
    // P10-2 (R5): called lock-free from System::GetTimeFromIMUInit;
    // load-acquire pairs with ProcessNewKeyFrame's store-release.
    KeyFramePtr pKF = mpCurrentKeyFrame.load(std::memory_order_acquire);
    if (pKF)
    {
        return pKF->mTimeStamp;
    }
    else
        return 0.0;
}

} //namespace ORB_SLAM
