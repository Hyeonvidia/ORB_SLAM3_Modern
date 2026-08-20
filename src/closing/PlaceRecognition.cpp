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


#include "closing/PlaceRecognition.hpp"

#include "closing/LoopClosing.hpp"       // host state via the friend grant (P8-4 pattern)
#include "core/System.hpp"               // System::eSensor enumerators
#include "tracking/Tracking.hpp"         // mHost.mpTracker->mSensor reads
#include "geometry/Sim3Solver.hpp"
#include "io/Converter.hpp"
#include "backend/ILoopOptimizer.hpp"    // OptimizeSim3
#include "features/ORBmatcher.hpp"

#include<cstdio>   // P9-4: TraceChannel


// R3: this TU used to inherit a global `using namespace std;` from the vendored
// DBoW2 fork header; restored per-TU until the R4 modern-C++ sweep qualifies it.
using namespace std;

namespace ORB_SLAM3
{

PlaceRecognition::PlaceRecognition(LoopClosing& host):
    mHost(host)
{
    // The channel counters/flags are DetectionChannel default member
    // initializers (P9-4); nothing else to construct.
}

bool PlaceRecognition::NewDetectCommonRegions()
{
    // To deactivate placerecognition. No loopclosing nor merging will be performed
    if(!mHost.mbActiveLC)
        return false;

    mHost.PopNewKeyFrame();

    if(mHost.mpLastMap->IsInertial() && !mHost.mpLastMap->GetIniertialBA2())
    {
        mHost.mpKeyFrameDB->add(mHost.mpCurrentKF);
        mHost.mpCurrentKF->SetErase();
        return false;
    }

    if(mHost.mpTracker->mSensor == System::STEREO && mHost.mpLastMap->GetAllKeyFrames().size() < 5) //12
    {
        mHost.mpKeyFrameDB->add(mHost.mpCurrentKF);
        mHost.mpCurrentKF->SetErase();
        return false;
    }

    if(mHost.mpLastMap->GetAllKeyFrames().size() < 12)
    {
        mHost.mpKeyFrameDB->add(mHost.mpCurrentKF);
        mHost.mpCurrentKF->SetErase();
        return false;
    }


    //Check the last candidates with geometric validation
    // Loop candidates
    bool bLoopDetectedInKF = false;

#ifdef REGISTER_TIMES
    std::chrono::steady_clock::time_point time_StartEstSim3_1 = std::chrono::steady_clock::now();
#endif
    if(mLoopCh.numCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3d mTcl = (mHost.mpCurrentKF->GetPose() * mLoopCh.lastCurrentKF->GetPoseInverse()).cast<double>();
        g2o::Sim3 gScl(mTcl.unit_quaternion(),mTcl.translation(),1.0);
        g2o::Sim3 gScw = gScl * mLoopCh.slw;
        int numProjMatches = 0;
        vector<MapPointPtr> vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mHost.mpCurrentKF, mLoopCh.matchedKF, gScw, numProjMatches, mLoopCh.mps, vpMatchedMPs);
        if(bCommonRegion)
        {

            bLoopDetectedInKF = true;

            // loop channel DOES reset numNotFound on success
            ChannelAdvance("loop", mLoopCh, gScw, vpMatchedMPs, /*bResetNotFound=*/true);

            if(!mLoopCh.detected)
            {
                cout << "PR: Loop detected with Reffine Sim3" << endl;
            }
        }
        else
        {
            bLoopDetectedInKF = false;

            // Clear detected on reffine failure, like the merge twin below:
            // a reffine-failed loop hypothesis must not stay DETECTED and be
            // consumed off a stale anchor (DIVERGENCES #21, promoted
            // unconditional in R4a — upstream left the loop channel latched).
            ChannelDecayStep("loop", mLoopCh, /*bClearDetected=*/true);
        }
    }

    //Merge candidates
    bool bMergeDetectedInKF = false;
    if(mMergeCh.numCoincidences > 0)
    {
        // Find from the last KF candidates
        Sophus::SE3d mTcl = (mHost.mpCurrentKF->GetPose() * mMergeCh.lastCurrentKF->GetPoseInverse()).cast<double>();

        g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
        g2o::Sim3 gScw = gScl * mMergeCh.slw;
        int numProjMatches = 0;
        vector<MapPointPtr> vpMatchedMPs;
        bool bCommonRegion = DetectAndReffineSim3FromLastKF(mHost.mpCurrentKF, mMergeCh.matchedKF, gScw, numProjMatches, mMergeCh.mps, vpMatchedMPs);
        if(bCommonRegion)
        {
            bMergeDetectedInKF = true;

            // merge channel does NOT reset numNotFound on success
            // (docs/DIVERGENCES.md #21)
            ChannelAdvance("merge", mMergeCh, gScw, vpMatchedMPs, /*bResetNotFound=*/false);
        }
        else
        {
            bMergeDetectedInKF = false;

            // merge channel DOES clear detected on failure
            ChannelDecayStep("merge", mMergeCh, /*bClearDetected=*/true);
        }
    }
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_1 = std::chrono::steady_clock::now();

        double timeEstSim3 = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndEstSim3_1 - time_StartEstSim3_1).count();
#endif

    if(mMergeCh.detected || mLoopCh.detected)
    {
#ifdef REGISTER_TIMES
        mHost.vdEstSim3_ms.push_back(timeEstSim3);
#endif
        mHost.mpKeyFrameDB->add(mHost.mpCurrentKF);
        return true;
    }

    // Extract candidates from the bag of words
    vector<KeyFramePtr> vpMergeBowCand, vpLoopBowCand;
    if(!bMergeDetectedInKF || !bLoopDetectedInKF)
    {
        // Search in BoW
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartQuery = std::chrono::steady_clock::now();
#endif
        mHost.mpKeyFrameDB->DetectNBestCandidates(mHost.mpCurrentKF, vpLoopBowCand, vpMergeBowCand,3);
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndQuery = std::chrono::steady_clock::now();

        double timeDataQuery = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndQuery - time_StartQuery).count();
        mHost.vdDataQuery_ms.push_back(timeDataQuery);
#endif
    }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_StartEstSim3_2 = std::chrono::steady_clock::now();
#endif
    // Check the BoW candidates if the geometric candidate list is empty
    // P9-4: the channel's detected flag is now written inside ChannelBoWSeed
    // (the upstream `mbLoopDetected = DetectCommonRegionsFromBoW(...)` also
    // wrote `false` on a BoW miss, but both flags are provably false here --
    // the early-return above fired otherwise -- so dropping that no-op
    // assignment is unobservable).
    //Loop candidates
    if(!bLoopDetectedInKF && !vpLoopBowCand.empty())
    {
        DetectCommonRegionsFromBoW(vpLoopBowCand, "loop", mLoopCh);
    }
    // Merge candidates
    if(!bMergeDetectedInKF && !vpMergeBowCand.empty())
    {
        DetectCommonRegionsFromBoW(vpMergeBowCand, "merge", mMergeCh);
    }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point time_EndEstSim3_2 = std::chrono::steady_clock::now();

        timeEstSim3 += std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndEstSim3_2 - time_StartEstSim3_2).count();
        mHost.vdEstSim3_ms.push_back(timeEstSim3);
#endif

    mHost.mpKeyFrameDB->add(mHost.mpCurrentKF);

    if(mMergeCh.detected || mLoopCh.detected)
    {
        return true;
    }

    mHost.mpCurrentKF->SetErase();
    

    return false;
}

// ---------------------------------------------------------------------------
// P9-4: DetectionChannel mutators. These are the ONLY writers of channel
// state after construction; each maps 1:1 onto an upstream mutation block
// and emits one stderr trace line (P7 SetState-with-reason precedent).
// The load-bearing asymmetries between the loop and merge channels are
// parameterized, NOT unified -- see docs/DIVERGENCES.md #21 and the header
// doc block.
// ---------------------------------------------------------------------------

void PlaceRecognition::TraceChannel(const char* ch, const char* event, const DetectionChannel& c, const char* extra)
{
    fprintf(stderr, "[loopclosing] %s %s cnt=%d notFound=%d detected=%d kf=%ld %s\n",
            ch, event, c.numCoincidences, c.numNotFound, c.detected ? 1 : 0,
            mHost.mpCurrentKF ? (long)mHost.mpCurrentKF->mnId : -1, extra);
}

// Reffine success block (upstream Run-loop LC:389-408 / merge twin :439-450).
void PlaceRecognition::ChannelAdvance(const char* ch, DetectionChannel& c, const g2o::Sim3& gScw,
                                 const std::vector<MapPointPtr>& vpMatchedMPs, bool bResetNotFound)
{
    c.numCoincidences++;
    c.lastCurrentKF->SetErase();     // anchor hand-off: release the old anchor's latch...
    c.lastCurrentKF = mHost.mpCurrentKF;   // ...and adopt the current KF (already SetNotErase'd at pop)
    c.slw = gScw;
    c.matchedMps = vpMatchedMPs;

    c.detected = c.numCoincidences >= 3;
    if(bResetNotFound)               // loop only; merge success keeps its NotFound count
        c.numNotFound = 0;
    TraceChannel(ch, "reffine-advance", c);
}

// Reffine failure step (upstream LC:409-424 / :451-465), including the decay
// wipe at the second consecutive failure. The decay wipe does NOT clear
// `detected`; instead both call sites clear it up front on EVERY failure
// (bClearDetected=true — the loop channel joined the merge channel's shape
// when DIVERGENCES #21's second half was fixed, promoted unconditional in
// R4a).
void PlaceRecognition::ChannelDecayStep(const char* ch, DetectionChannel& c, bool bClearDetected)
{
    if(bClearDetected)
        c.detected = false;

    c.numNotFound++;
    TraceChannel(ch, "reffine-fail", c);
    if(c.numNotFound >= 2)
    {
        c.lastCurrentKF->SetErase();
        c.matchedKF->SetErase();
        c.numCoincidences = 0;
        c.matchedMps.clear();
        c.mps.clear();
        c.numNotFound = 0;
        // NOTE: c.detected deliberately untouched (upstream decay-wipe shape)
        TraceChannel(ch, "wipe-decay", c);
    }
}

// Full wipe (upstream consume :204-211/:290-297, merge-priority discard
// :213-223, scale abort :150-156 -- all three clear the exact same field
// set, including `detected`). Pointers/Sim3 stay stale by design.
void PlaceRecognition::ChannelWipe(const char* ch, DetectionChannel& c, const char* reason)
{
    c.lastCurrentKF->SetErase();
    c.matchedKF->SetErase();
    c.numCoincidences = 0;
    c.matchedMps.clear();
    c.mps.clear();
    c.numNotFound = 0;
    c.detected = false;
    TraceChannel(ch, reason, c);
}

// BoW commit block (upstream LC:875-886, the tail of
// DetectCommonRegionsFromBoW). Seeds -- or overwrites -- the hypothesis.
// Upstream did no SetErase on the previously latched anchor/matched KF
// (leak L1), could seed numCoincidences == 0 (orphan latch L2), and carried
// numNotFound over from the dead hypothesis; the hygiene block below fixes
// all three (promoted unconditional in R4a).
bool PlaceRecognition::ChannelBoWSeed(const char* ch, DetectionChannel& c, KeyFramePtr pBestMatchedKF,
                                 int nBestNumCoincidences, const g2o::Sim3& g2oBestScw,
                                 const std::vector<MapPointPtr>& vpBestMapPoints,
                                 const std::vector<MapPointPtr>& vpBestMatchedMapPoints)
{
    // Latch hygiene (OWNERSHIP L1/L2, docs/P9_RECON.md D2/D3; promoted
    // unconditional in R4a):
    //   L2 -- a seed with zero coincidences would latch a KF the decay path
    //   (guarded numCoincidences > 0 upstream) can never release: permanent
    //   orphan latch + stale-anchor carryover. Refuse the seed outright and
    //   keep the existing hypothesis (and its legitimate latches) intact; a
    //   cnt==0 seed could never set `detected`, and the BoW callers ignore
    //   the return value beyond the channel flags, so refusing is the
    //   minimal divergence.
    //   L1 -- seeding over a live hypothesis overwrites matchedKF/
    //   lastCurrentKF without SetErase: the old latches pin those KFs
    //   against culling forever. Release them first (the current KF keeps
    //   its queue-pop latch), and zero the numNotFound carried over from
    //   the dead hypothesis so the fresh one starts clean.
    if(nBestNumCoincidences == 0)
    {
        TraceChannel(ch, "bow-seed-refused-cnt0", c);
        return false;
    }
    if(c.matchedKF)
        c.matchedKF->SetErase();
    if(c.lastCurrentKF && c.lastCurrentKF != mHost.mpCurrentKF)
        c.lastCurrentKF->SetErase();
    c.numNotFound = 0;

    c.lastCurrentKF = mHost.mpCurrentKF;
    c.numCoincidences = nBestNumCoincidences;
    c.matchedKF = pBestMatchedKF;
    c.matchedKF->SetNotErase();
    c.slw = g2oBestScw;
    c.mps = vpBestMapPoints;
    c.matchedMps = vpBestMatchedMapPoints;

    c.detected = c.numCoincidences >= 3;
    TraceChannel(ch, "bow-seed", c);
    return c.detected;
}

bool PlaceRecognition::DetectAndReffineSim3FromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                 std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs)
{
    set<MapPointPtr> spAlreadyMatchedMPs;
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    int nProjOptMatches = 50;
    int nProjMatchesRep = 100;

    if(nNumProjMatches >= nProjMatches)
    {
        Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
        g2o::Sim3 gSwm(mTwm.unit_quaternion(),mTwm.translation(),1.0);
        g2o::Sim3 gScm = gScw * gSwm;
        Eigen::Matrix<double, 7, 7> mHessian7x7;

        bool bFixedScale = mHost.mbFixScale;       // TODO CHECK; Solo para el monocular inertial
        if(mHost.mpTracker->mSensor==System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
            bFixedScale=false;
        int numOptMatches = mHost.mpOptimizer->OptimizeSim3(mHost.mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);


        if(numOptMatches > nProjOptMatches)
        {
            g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(),1.0);

            nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
            if(nNumProjMatches >= nProjMatchesRep)
            {
                gScw = gScw_estimation;
                return true;
            }
        }
    }
    return false;
}

bool PlaceRecognition::DetectCommonRegionsFromBoW(std::vector<KeyFramePtr> &vpBowCand, const char* chName, DetectionChannel &ch)
{
    int nBoWMatches = 20;
    int nBoWInliers = 15;
    int nSim3Inliers = 20;
    int nProjMatches = 50;
    int nProjOptMatches = 80;

    set<KeyFramePtr> spConnectedKeyFrames = mHost.mpCurrentKF->GetConnectedKeyFrames();

    int nNumCovisibles = 10;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);

    // Varibles to select the best numbe
    KeyFramePtr pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    std::vector<MapPointPtr> vpBestMapPoints;
    std::vector<MapPointPtr> vpBestMatchedMapPoints;

    for(KeyFramePtr pKFi : vpBowCand)
    {
        if(!pKFi || pKFi->isBad())
        {
            continue;
        }

        // Current KF against KF with covisibles version
        std::vector<KeyFramePtr> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
        if(vpCovKFi.empty())
        {
            std::cout << "Covisible list empty" << std::endl;
            vpCovKFi.push_back(pKFi);
        }
        else
        {
            vpCovKFi.push_back(vpCovKFi[0]);
            vpCovKFi[0] = pKFi;
        }


        bool bAbortByNearKF = false;
        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            {
                bAbortByNearKF = true;
                break;
            }
        }
        if(bAbortByNearKF)
        {
            continue;
        }


        std::vector<std::vector<MapPointPtr> > vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPointPtr> spMatchedMPi;
        int numBoWMatches = 0;

        KeyFramePtr pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;

        std::vector<MapPointPtr> vpMatchedPoints = std::vector<MapPointPtr>(mHost.mpCurrentKF->GetMapPointMatches().size(), nullptr);
        std::vector<KeyFramePtr> vpKeyFrameMatchedMP = std::vector<KeyFramePtr>(mHost.mpCurrentKF->GetMapPointMatches().size(), nullptr);

        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
            {
                continue;
            }

            int num = matcherBoW.SearchByBoW(mHost.mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            if (num > nMostBoWNumMatches)
            {
                nMostBoWNumMatches = num;
            }
        }

        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            for(int k=0; k < vvpMatchedMPs[j].size(); ++k)
            {
                MapPointPtr pMPi_j = vvpMatchedMPs[j][k];
                if(!pMPi_j || pMPi_j->isBad())
                {
                    continue;
                }

                if(spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                {
                    spMatchedMPi.insert(pMPi_j);
                    numBoWMatches++;

                    vpMatchedPoints[k]= pMPi_j;
                    vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                }
            }
        }

        if(numBoWMatches >= nBoWMatches) // TODO pick a good threshold
        {
            // Geometric validation
            bool bFixedScale = mHost.mbFixScale;
            if(mHost.mpTracker->mSensor==System::IMU_MONOCULAR && !mHost.mpCurrentKF->GetMap()->GetIniertialBA2())
                bFixedScale=false;

            Sim3Solver solver = Sim3Solver(mHost.mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

            bool bNoMore = false;
            vector<bool> vbInliers;
            int nInliers;
            bool bConverge = false;
            Eigen::Matrix4f mTcm;
            while(!bConverge && !bNoMore)
            {
                mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);
            }

            if(bConverge)
            {

                // Match by reprojection
                vpCovKFi.clear();
                vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                vpCovKFi.push_back(pMostBoWMatchesKF);

                set<MapPointPtr> spMapPoints;
                vector<MapPointPtr> vpMapPoints;
                vector<KeyFramePtr> vpKeyFrames;
                for(KeyFramePtr pCovKFi : vpCovKFi)
                {
                    for(const MapPointPtr& pCovMPij : pCovKFi->GetMapPointMatches())
                    {
                        if(!pCovMPij || pCovMPij->isBad())
                        {
                            continue;
                        }

                        if(spMapPoints.find(pCovMPij) == spMapPoints.end())
                        {
                            spMapPoints.insert(pCovMPij);
                            vpMapPoints.push_back(pCovMPij);
                            vpKeyFrames.push_back(pCovKFi);
                        }
                    }
                }


                g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),solver.GetEstimatedTranslation().cast<double>(), (double) solver.GetEstimatedScale());
                g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
                g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                Sophus::Sim3f mScw = Converter::toSophus(gScw);

                vector<MapPointPtr> vpMatchedMP;
                vpMatchedMP.resize(mHost.mpCurrentKF->GetMapPointMatches().size(), nullptr);
                vector<KeyFramePtr> vpMatchedKF;
                vpMatchedKF.resize(mHost.mpCurrentKF->GetMapPointMatches().size(), nullptr);
                int numProjMatches = matcher.SearchByProjection(mHost.mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);

                if(numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    // Pass the IMU_MONOCULAR pre-BA2 relaxed bFixedScale
                    // computed above, like the other three OptimizeSim3
                    // sites — upstream passed raw mHost.mbFixScale here
                    // (DIVERGENCES #24, docs/P9_RECON.md D6; promoted
                    // unconditional in R4a).
                    int numOptMatches = mHost.mpOptimizer->OptimizeSim3(mHost.mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, bFixedScale, mHessian7x7, true);

                    if(numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
                        g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                        Sophus::Sim3f mScw = Converter::toSophus(gScw);

                        vector<MapPointPtr> vpMatchedMP;
                        vpMatchedMP.resize(mHost.mpCurrentKF->GetMapPointMatches().size(), nullptr);
                        int numProjOptMatches = matcher.SearchByProjection(mHost.mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                        if(numProjOptMatches >= nProjOptMatches)
                        {
                            int nNumKFs = 0;
                            //vpMatchedMPs = vpMatchedMP;
                            //vpMPs = vpMapPoints;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            vector<KeyFramePtr> vpCurrentCovKFs = mHost.mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

                            int j = 0;
                            while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                            {
                                KeyFramePtr pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc = (pKFj->GetPose() * mHost.mpCurrentKF->GetPoseInverse()).cast<double>();
                                g2o::Sim3 gSjc(mTjc.unit_quaternion(),mTjc.translation(),1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPointPtr> vpMatchedMPs_j;
                                bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                if(bValid)
                                {
                                    nNumKFs++;
                                }
                                j++;
                            }

                            if(nBestMatchesReproj < numProjOptMatches)
                            {
                                nBestMatchesReproj = numProjOptMatches;
                                nBestNumCoindicendes = nNumKFs;
                                pBestMatchedKF = pMostBoWMatchesKF;
                                g2oBestScw = gScw;
                                vpBestMapPoints = vpMapPoints;
                                vpBestMatchedMapPoints = vpMatchedMP;
                            }
                        }
                    }
                }
            }
        }
    }

    if(nBestMatchesReproj > 0)
    {
        return ChannelBoWSeed(chName, ch, pBestMatchedKF, nBestNumCoindicendes,
                              g2oBestScw, vpBestMapPoints, vpBestMatchedMapPoints);
    }
    return false;
}

bool PlaceRecognition::DetectCommonRegionsFromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs)
{
    set<MapPointPtr> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    if(nNumProjMatches >= nProjMatches)
    {
        return true;
    }

    return false;
}

int PlaceRecognition::FindMatchesByProjection(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKFw, g2o::Sim3 &g2oScw,
                                         set<MapPointPtr> &spMatchedMPinOrigin, vector<MapPointPtr> &vpMapPoints,
                                         vector<MapPointPtr> &vpMatchedMapPoints)
{
    int nNumCovisibles = 10;
    vector<KeyFramePtr> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFramePtr> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFramePtr> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    if(nInitialCov < nNumCovisibles)
    {
        for(int i=0; i<nInitialCov; ++i)
        {
            vector<KeyFramePtr> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
            int nInserted = 0;
            int j = 0;
            while(j < vpKFs.size() && nInserted < nNumCovisibles)
            {
                if(spCheckKFs.find(vpKFs[j]) == spCheckKFs.end() && spCurrentCovisbles.find(vpKFs[j]) == spCurrentCovisbles.end())
                {
                    spCheckKFs.insert(vpKFs[j]);
                    ++nInserted;
                }
                ++j;
            }
            vpCovKFm.insert(vpCovKFm.end(), vpKFs.begin(), vpKFs.end());
        }
    }
    set<MapPointPtr> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for(KeyFramePtr pKFi : vpCovKFm)
    {
        for(const MapPointPtr& pMPij : pKFi->GetMapPointMatches())
        {
            if(!pMPij || pMPij->isBad())
            {
                continue;
            }

            if(spMapPoints.find(pMPij) == spMapPoints.end())
            {
                spMapPoints.insert(pMPij);
                vpMapPoints.push_back(pMPij);
            }
        }
    }

    Sophus::Sim3f mScw = Converter::toSophus(g2oScw);
    ORBmatcher matcher(0.9, true);

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), nullptr);
    int num_matches = matcher.SearchByProjection(pCurrentKF, mScw, vpMapPoints, vpMatchedMapPoints, 3, 1.5);

    return num_matches;
}

// ---------------------------------------------------------------------------
// P9-5: named consume-side entry points for LoopClosing::Run. Each is a thin
// funnel onto the P9-4 mutators so the per-site trace reasons (and the wipe
// shapes they name) stay exactly as before; Run gets no mutable channel
// access.
// ---------------------------------------------------------------------------

void PlaceRecognition::WipeMergeAfterConsume()
{
    ChannelWipe("merge", mMergeCh, "wipe-consume");
}

void PlaceRecognition::WipeLoopOnMergePriority()
{
    ChannelWipe("loop", mLoopCh, "wipe-merge-priority-discard");
}

void PlaceRecognition::WipeLoopAfterConsume()
{
    ChannelWipe("loop", mLoopCh, "wipe-consume");
}

void PlaceRecognition::WipeMergeOnScaleAbort()
{
    ChannelWipe("merge", mMergeCh, "wipe-scale-abort");
}

// Full machine wipe for LoopClosing::ResetIfRequested -- both channels,
// called from both reset branches (OWNERSHIP D5; promoted unconditional in
// R4a -- upstream only cleared the queue and let channels survive pointing
// into the torn-down map). ChannelWipe's shape derefs the KF latches
// unconditionally (all its upstream call sites hold a live hypothesis), but
// a reset can fire before any BoW seed ever ran, so guard on lastCurrentKF:
// the seed writes lastCurrentKF and matchedKF together, so a null
// lastCurrentKF means a never-seeded channel with nothing latched. A
// previously wiped channel keeps stale non-null pointers by design; the
// repeated SetErase there is idempotent latch release (and the #19
// deferred-SetBadFlag completion path).
void PlaceRecognition::ResetChannels()
{
    if(mLoopCh.lastCurrentKF)
        ChannelWipe("loop", mLoopCh, "wipe-reset");
    if(mMergeCh.lastCurrentKF)
        ChannelWipe("merge", mMergeCh, "wipe-reset");
}

void PlaceRecognition::TraceReset(const char* reason)
{
    TraceChannel("loop", reason, mLoopCh);
    TraceChannel("merge", reason, mMergeCh);
}

} //namespace ORB_SLAM
