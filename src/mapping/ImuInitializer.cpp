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

#include "mapping/ImuInitializer.hpp"
#include "mapping/LocalMapping.hpp"
#include "tracking/Tracking.hpp"
#include "backend/IMappingOptimizer.hpp"
#include "backend/BAEpochs.hpp"
#include "backend/GBAResult.hpp"
#include "core/Verbose.hpp"

#include <mutex>

// R3: this TU used to inherit a global `using namespace std;` from the vendored
// DBoW2 fork header; restored per-TU until the R4 modern-C++ sweep qualifies it.
using namespace std;

namespace ORB_SLAM3
{

ImuInitializer::ImuInitializer(LocalMapping& host):
    mHost(host), mScale(1.0)
{
}

void ImuInitializer::InitializeIMU(float priorG, float priorA, bool bFIBA)
{
    if (mHost.mbResetRequested.load(std::memory_order_relaxed))
        return;

    float minTime;
    int nMinKF;
    if (mHost.mbMonocular)
    {
        minTime = 2.0;
        nMinKF = 10;
    }
    else
    {
        minTime = 1.0;
        nMinKF = 10;
    }


    if(mHost.mpAtlas->KeyFramesInMap()<nMinKF)
        return;

    // Retrieve all keyframe in temporal order
    list<KeyFramePtr> lpKF;
    KeyFramePtr pKF = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed);
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFramePtr> vpKF(lpKF.begin(),lpKF.end());

    if(vpKF.size()<nMinKF)
        return;

    mHost.mFirstTs.store(vpKF.front()->mTimeStamp, std::memory_order_relaxed);
    if(mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->mTimeStamp-mHost.mFirstTs.load(std::memory_order_relaxed)<minTime)
        return;

    mHost.bInitializing.store(true, std::memory_order_relaxed);

    while(mHost.CheckNewKeyFrames())
    {
        mHost.ProcessNewKeyFrame();
        vpKF.push_back(mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed));
        lpKF.push_back(mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed));
    }

    const int N = vpKF.size();

    // Compute and KF velocities mRwg estimation
    if (!mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->isImuInitialized())
    {
        Eigen::Matrix3f Rwg;
        Eigen::Vector3f dirG;
        dirG.setZero();
        for(vector<KeyFramePtr>::iterator itKF = vpKF.begin(); itKF!=vpKF.end(); itKF++)
        {
            if (!(*itKF)->mpImuPreintegrated)
                continue;
            if (!(*itKF)->mPrevKF)
                continue;

            dirG -= (*itKF)->mPrevKF->GetImuRotation() * (*itKF)->mpImuPreintegrated->GetUpdatedDeltaVelocity();
            Eigen::Vector3f _vel = ((*itKF)->GetImuPosition() - (*itKF)->mPrevKF->GetImuPosition())/(*itKF)->mpImuPreintegrated->dT;
            (*itKF)->SetVelocity(_vel);
            (*itKF)->mPrevKF->SetVelocity(_vel);
        }

        dirG = dirG/dirG.norm();
        Eigen::Vector3f gI(0.0f, 0.0f, -1.0f);
        Eigen::Vector3f v = gI.cross(dirG);
        const float nv = v.norm();
        const float cosg = gI.dot(dirG);
        const float ang = acos(cosg);
        Eigen::Vector3f vzg = v*ang/nv;
        Rwg = Sophus::SO3f::exp(vzg).matrix();
        mRwg = Rwg.cast<double>();
        mHost.mTinit = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->mTimeStamp-mHost.mFirstTs.load(std::memory_order_relaxed);
    }
    else
    {
        mRwg = Eigen::Matrix3d::Identity();
        mbg = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetGyroBias().cast<double>();
        mba = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetAccBias().cast<double>();
    }

    mScale=1.0;

    // Dead out-param of the interface signature (never touched by the
    // backend); was a persistent member until P8-1.
    Eigen::MatrixXd infoInertial = Eigen::MatrixXd::Zero(9,9);
    mHost.mpOptimizer->InertialOptimization(mHost.mpAtlas->GetCurrentMap(), mRwg, mScale, mbg, mba, mHost.mbMonocular, infoInertial, false, false, priorG, priorA);

    if (mScale<1e-1)
    {
        cout << "scale too small" << endl;
        mHost.bInitializing.store(false, std::memory_order_relaxed);
        return;
    }

    // Before this line we are not changing the map
    {
        unique_lock<mutex> lock(mHost.mpAtlas->GetCurrentMap()->mMutexMapUpdate);
        if ((fabs(mScale - 1.f) > 0.00001) || !mHost.mbMonocular) {
            Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
            mHost.mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
            // P11-A: was a direct cross-thread UpdateFrameIMU call (TSAN T3
            // Frame::operator= race class). Posted as a message; Tracking
            // applies it under its next mMutexMapUpdate acquisition, which
            // this very lock scope makes atomic with ApplyScaledRotation
            // from Tracking's point of view. DIVERGENCES #28.
            Tracking::ImuUpdateMsg msg;
            msg.scale = mScale;
            msg.bias = vpKF[0]->GetImuBias();
            msg.pBaseKF = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed);
            msg.bFirstInit = false;
            msg.t0Imu = msg.pBaseKF->mTimeStamp;
            mHost.mpTracker->PostImuUpdate(msg);
        }

        // Check if initialization OK
        if (!mHost.mpAtlas->isImuInitialized())
            for (int i = 0; i < N; i++) {
                KeyFramePtr pKF2 = vpKF[i];
                pKF2->bImu = true;
            }
    }

    // P11-A: was `UpdateFrameIMU(1.0, ...)` followed by a raw cross-thread
    // write of mpTracker->t0IMU from a racy read of Tracking's live
    // mCurrentFrame.mTimeStamp (the ImuInitializer:160 TSAN signature). The
    // rebias rides the message (scale 1.0 composes as a no-op on top of a
    // still-pending rescale); the first-init t0IMU stamp rides bFirstInit —
    // Tracking stamps its OWN current frame time at application (bounded
    // semantic shift, DIVERGENCES #28). SetImuInitialized/bImu stay here:
    // they are Atlas/KeyFrame state, not Tracking state.
    {
        Tracking::ImuUpdateMsg msg;
        msg.scale = 1.0f;
        msg.bias = vpKF[0]->GetImuBias();
        msg.pBaseKF = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed);
        msg.bFirstInit = !mHost.mpAtlas->isImuInitialized();
        msg.t0Imu = msg.pBaseKF->mTimeStamp;
        mHost.mpTracker->PostImuUpdate(msg);
    }
    if (!mHost.mpAtlas->isImuInitialized())
    {
        mHost.mpAtlas->SetImuInitialized();
        mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->bImu = true;
    }

    // Result buffers of this GBA invocation (P5-D, was the KeyFrame/MapPoint
    // *GBA scribble fields stamped with mnBAGlobalForKF==mpCurrentKeyFrame->mnId)
    GBAResult gbaResult;

    if (bFIBA)
    {
        if (priorA!=0.f)
            mHost.mpOptimizer->FullInertialBA(mHost.mpAtlas->GetCurrentMap(), 100, false, mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->mnId, NULL, true, priorG, priorA, NULL, NULL, &gbaResult, *mHost.mpBAEpochs);
        else
            mHost.mpOptimizer->FullInertialBA(mHost.mpAtlas->GetCurrentMap(), 100, false, mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->mnId, NULL, false, 1e2, 1e6, NULL, NULL, &gbaResult, *mHost.mpBAEpochs);
    }

    Verbose::PrintMess("Global Bundle Adjustment finished\nUpdating map ...", Verbose::VERBOSITY_NORMAL);

    // Get Map Mutex
    unique_lock<mutex> lock(mHost.mpAtlas->GetCurrentMap()->mMutexMapUpdate);

    // Process keyframes in the queue
    while(mHost.CheckNewKeyFrames())
    {
        mHost.ProcessNewKeyFrame();
        vpKF.push_back(mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed));
        lpKF.push_back(mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed));
    }

    // Correct keyframes starting at map first keyframe
    list<KeyFramePtr> lpKFtoCheck(mHost.mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.begin(),mHost.mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.end());

    // Pre-correction pose of every keyframe updated in this pass
    // (was KeyFrame::mTcwBefGBA), read below to re-anchor map points
    std::map<KeyFramePtr, Sophus::SE3f> tcwBefGBA;

    while(!lpKFtoCheck.empty())
    {
        KeyFramePtr pKF = lpKFtoCheck.front();
        const set<KeyFramePtr> sChilds = pKF->GetChilds();
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        KeyFrameGBAResult& rKF = gbaResult.kfs[pKF];
        for(set<KeyFramePtr>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
        {
            KeyFramePtr pChild = *sit;
            if(!pChild || pChild->isBad())
            {
                continue;
            }

            if(!gbaResult.kfs.count(pChild))
            {
                Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                KeyFrameGBAResult& rChild = gbaResult.kfs[pChild];
                rChild.Tcw = Tchildc * rKF.Tcw;

                Sophus::SO3f Rcor = rChild.Tcw.so3().inverse() * pChild->GetPose().so3();
                if(pChild->isVelocitySet()){
                    rChild.Vwb = Rcor * pChild->GetVelocity();
                    rChild.hasVwb = true;
                }
                else {
                    Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);
                }

                rChild.Bias = pChild->GetImuBias();
                rChild.hasBias = true;

            }
            lpKFtoCheck.push_back(pChild);
        }

        tcwBefGBA[pKF] = pKF->GetPose();
        pKF->SetPose(rKF.Tcw);

        if(pKF->bImu)
        {
            if(rKF.hasVwb)
                pKF->SetVelocity(rKF.Vwb);
            if(rKF.hasBias)
                pKF->SetNewBias(rKF.Bias);
        } else {
            cout << "KF " << pKF->mnId << " not set to inertial!! \n";
        }

        lpKFtoCheck.pop_front();
    }

    // Correct MapPoints
    const vector<MapPointPtr> vpMPs = mHost.mpAtlas->GetCurrentMap()->GetAllMapPoints();

    for(size_t i=0; i<vpMPs.size(); i++)
    {
        MapPointPtr pMP = vpMPs[i];

        if(pMP->isBad())
        {
            continue;
        }

        std::map<MapPointPtr, Eigen::Vector3f>::const_iterator itMP = gbaResult.mps.find(pMP);
        if(itMP != gbaResult.mps.end())
        {
            // If optimized by Global BA, just update
            pMP->SetWorldPos(itMP->second);
        }
        else
        {
            // Update according to the correction of its reference keyframe
            KeyFramePtr pRefKF = pMP->GetReferenceKeyFrame();

            std::map<KeyFramePtr, Sophus::SE3f>::const_iterator itBef = tcwBefGBA.find(pRefKF);
            if(itBef == tcwBefGBA.end())
                continue;

            // Map to non-corrected camera
            Eigen::Vector3f Xc = itBef->second * pMP->GetWorldPos();

            // Backproject using corrected camera
            pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
        }
    }

    Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);

    mHost.PurgeNewKeyFramesAfterInertialInit();

    // P7-1a: was `mpTracker->mState=Tracking::OK;`. Same write, same thread
    // (local mapping), same absence of synchronization -- now routed through a
    // named entry point instead of a public data member. This is the one
    // legitimate cross-thread writer of the tracking state; see the warning
    // block at the top of docs/P7_RECON.md and finding F11 there.
    mHost.mpTracker->NotifyImuInitialized();
    mHost.bInitializing.store(false, std::memory_order_relaxed);

    mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->IncreaseChangeIndex();

    return;
}

void ImuInitializer::ScaleRefinement()
{
    // Minimum number of keyframes to compute a solution
    // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
    if (mHost.mbResetRequested.load(std::memory_order_relaxed))
        return;

    // Retrieve all keyframes in temporal order
    list<KeyFramePtr> lpKF;
    KeyFramePtr pKF = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed);
    while(pKF->mPrevKF)
    {
        lpKF.push_front(pKF);
        pKF = pKF->mPrevKF;
    }
    lpKF.push_front(pKF);
    vector<KeyFramePtr> vpKF(lpKF.begin(),lpKF.end());

    while(mHost.CheckNewKeyFrames())
    {
        mHost.ProcessNewKeyFrame();
        vpKF.push_back(mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed));
        lpKF.push_back(mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed));
    }

    mRwg = Eigen::Matrix3d::Identity();
    mScale=1.0;

    mHost.mpOptimizer->InertialOptimization(mHost.mpAtlas->GetCurrentMap(), mRwg, mScale);

    if (mScale<1e-1) // 1e-1
    {
        cout << "scale too small" << endl;
        mHost.bInitializing.store(false, std::memory_order_relaxed);
        return;
    }

    // Before this line we are not changing the map
    unique_lock<mutex> lock(mHost.mpAtlas->GetCurrentMap()->mMutexMapUpdate);
    if ((fabs(mScale-1.f)>0.002)||!mHost.mbMonocular)
    {
        Sophus::SE3f Tgw(mRwg.cast<float>().transpose(),Eigen::Vector3f::Zero());
        mHost.mpAtlas->GetCurrentMap()->ApplyScaledRotation(Tgw,mScale,true);
        // P11-A: message-passing (see InitializeIMU). DIVERGENCES #28.
        Tracking::ImuUpdateMsg msg;
        msg.scale = mScale;
        msg.pBaseKF = mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed);
        msg.bias = msg.pBaseKF->GetImuBias();
        msg.bFirstInit = false;
        msg.t0Imu = msg.pBaseKF->mTimeStamp;
        mHost.mpTracker->PostImuUpdate(msg);
    }

    mHost.PurgeNewKeyFramesAfterInertialInit();

    // To perform pose-inertial opt w.r.t. last keyframe
    mHost.mpCurrentKeyFrame.load(std::memory_order_relaxed)->GetMap()->IncreaseChangeIndex();

    return;
}

} //namespace ORB_SLAM
