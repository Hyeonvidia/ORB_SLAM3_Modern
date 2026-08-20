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
#include "core/LifetimeLedger.hpp"  // P12-L2 class-4 probes (no-op unless LIFETIME_TRACE)

#include "closing/LoopClosing.hpp"       // host state via the friend grant (P8-4 pattern)
#include "core/FixFlags.hpp"             // P11-F2: Fix.ScaleJacobian rides into OptimizeSim3
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
        vector<MapPoint*> vpMatchedMPs;
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

            // Level 0: the loop channel does NOT clear detected on failure —
            // the second half of the DIVERGENCES #21 poisoned-consume
            // composite (the merge twin below passes true). P11-F3
            // (Fix.LoopStateHygiene, OFF by default): clear it like the
            // merge channel does, so a reffine-failed loop hypothesis cannot
            // stay DETECTED and be consumed off a stale anchor. Per-LC-KF
            // site, direct flag read (no ctor caching needed).
            ChannelDecayStep("loop", mLoopCh,
                             /*bClearDetected=*/FixFlags::I().loopStateHygiene);
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
        vector<MapPoint*> vpMatchedMPs;
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
    vector<KeyFrame*> vpMergeBowCand, vpLoopBowCand;
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
                                 const std::vector<MapPoint*>& vpMatchedMPs, bool bResetNotFound)
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
// `detected` in either channel; the merge channel instead clears it up front
// on EVERY failure, the loop channel never does (that latch is DIVERGENCES
// #21's second half).
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
// DetectCommonRegionsFromBoW). Seeds -- or overwrites -- the hypothesis:
// no SetErase on the previously latched anchor/matched KF (leak L1), can
// seed numCoincidences == 0 (orphan latch L2), and numNotFound is carried
// over from the dead hypothesis. All upstream, all preserved at level 0;
// P11-F3 (Fix.LatchHygiene) arms the hygiene block below.
bool PlaceRecognition::ChannelBoWSeed(const char* ch, DetectionChannel& c, KeyFrame* pBestMatchedKF,
                                 int nBestNumCoincidences, const g2o::Sim3& g2oBestScw,
                                 const std::vector<MapPoint*>& vpBestMapPoints,
                                 const std::vector<MapPoint*>& vpBestMatchedMapPoints)
{
    // P11-F3 (OWNERSHIP L1/L2, docs/P9_RECON.md D2/D3; Fix.LatchHygiene,
    // OFF by default; per-seed site, direct flag read):
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
    if(FixFlags::I().latchHygiene)
    {
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
    }

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

bool PlaceRecognition::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                 std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
{
    set<MapPoint*> spAlreadyMatchedMPs;
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

bool PlaceRecognition::DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, const char* chName, DetectionChannel &ch)
{
    int nBoWMatches = 20;
    int nBoWInliers = 15;
    int nSim3Inliers = 20;
    int nProjMatches = 50;
    int nProjOptMatches = 80;

    set<KeyFrame*> spConnectedKeyFrames = mHost.mpCurrentKF->GetConnectedKeyFrames();

    int nNumCovisibles = 10;

    ORBmatcher matcherBoW(0.9, true);
    ORBmatcher matcher(0.75, true);

    // Varibles to select the best numbe
    KeyFrame* pBestMatchedKF;
    int nBestMatchesReproj = 0;
    int nBestNumCoindicendes = 0;
    g2o::Sim3 g2oBestScw;
    std::vector<MapPoint*> vpBestMapPoints;
    std::vector<MapPoint*> vpBestMatchedMapPoints;

    for(KeyFrame* pKFi : vpBowCand)
    {
        if(!pKFi || pKFi->isBad())
        {
            if(pKFi && pKFi->isBad()) LT_PROBE_BAD("PRDetectCommon.406", 'K', pKFi->mnId);
            continue;
        }

        // Current KF against KF with covisibles version
        std::vector<KeyFrame*> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
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


        std::vector<std::vector<MapPoint*> > vvpMatchedMPs;
        vvpMatchedMPs.resize(vpCovKFi.size());
        std::set<MapPoint*> spMatchedMPi;
        int numBoWMatches = 0;

        KeyFrame* pMostBoWMatchesKF = pKFi;
        int nMostBoWNumMatches = 0;

        std::vector<MapPoint*> vpMatchedPoints = std::vector<MapPoint*>(mHost.mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
        std::vector<KeyFrame*> vpKeyFrameMatchedMP = std::vector<KeyFrame*>(mHost.mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));

        for(int j=0; j<vpCovKFi.size(); ++j)
        {
            if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
            {
                if(vpCovKFi[j] && vpCovKFi[j]->isBad()) LT_PROBE_BAD("PRDetectCommon.451", 'K', vpCovKFi[j]->mnId);
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
                MapPoint* pMPi_j = vvpMatchedMPs[j][k];
                if(!pMPi_j || pMPi_j->isBad())
                {
                    if(pMPi_j && pMPi_j->isBad()) LT_PROBE_BAD("PRDetectCommon.466", 'M', pMPi_j->mnId);
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

                set<MapPoint*> spMapPoints;
                vector<MapPoint*> vpMapPoints;
                vector<KeyFrame*> vpKeyFrames;
                for(KeyFrame* pCovKFi : vpCovKFi)
                {
                    for(MapPoint* pCovMPij : pCovKFi->GetMapPointMatches())
                    {
                        if(!pCovMPij || pCovMPij->isBad())
                        {
                            if(pCovMPij && pCovMPij->isBad()) LT_PROBE_BAD("PRDetectCommon.515", 'M', pCovMPij->mnId);
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

                vector<MapPoint*> vpMatchedMP;
                vpMatchedMP.resize(mHost.mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                vector<KeyFrame*> vpMatchedKF;
                vpMatchedKF.resize(mHost.mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));
                int numProjMatches = matcher.SearchByProjection(mHost.mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);

                if(numProjMatches >= nProjMatches)
                {
                    // Optimize Sim3 transformation with every matches
                    Eigen::Matrix<double, 7, 7> mHessian7x7;

                    // Upstream passes raw mHost.mbFixScale here, without the
                    // IMU_MONOCULAR pre-BA2 relaxation already computed above
                    // (bFixedScale) and used by the other three OptimizeSim3
                    // sites — preserved bug-for-bug at level 0 (DIVERGENCES
                    // #24, docs/P9_RECON.md D6). P11-F2: Fix.ScaleJacobian
                    // (#9's flag — same MI early-scale-window semantics, same
                    // validation runs) passes the relaxed value instead.
                    // Config-class site, not hot: direct FixFlags read, no
                    // ctor caching needed (contrast EdgeInertialGS, P11-F1).
                    const bool bSim3FixScale =
                        FixFlags::I().scaleJacobianChainRule ? bFixedScale
                                                             : mHost.mbFixScale;
                    int numOptMatches = mHost.mpOptimizer->OptimizeSim3(mHost.mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, bSim3FixScale, mHessian7x7, true);

                    if(numOptMatches >= nSim3Inliers)
                    {
                        g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
                        g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                        Sophus::Sim3f mScw = Converter::toSophus(gScw);

                        vector<MapPoint*> vpMatchedMP;
                        vpMatchedMP.resize(mHost.mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                        int numProjOptMatches = matcher.SearchByProjection(mHost.mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

                        if(numProjOptMatches >= nProjOptMatches)
                        {
                            int nNumKFs = 0;
                            //vpMatchedMPs = vpMatchedMP;
                            //vpMPs = vpMapPoints;
                            // Check the Sim3 transformation with the current KeyFrame covisibles
                            vector<KeyFrame*> vpCurrentCovKFs = mHost.mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

                            int j = 0;
                            while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                            {
                                KeyFrame* pKFj = vpCurrentCovKFs[j];
                                Sophus::SE3d mTjc = (pKFj->GetPose() * mHost.mpCurrentKF->GetPoseInverse()).cast<double>();
                                g2o::Sim3 gSjc(mTjc.unit_quaternion(),mTjc.translation(),1.0);
                                g2o::Sim3 gSjw = gSjc * gScw;
                                int numProjMatches_j = 0;
                                vector<MapPoint*> vpMatchedMPs_j;
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

bool PlaceRecognition::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                                std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
{
    set<MapPoint*> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
    nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

    int nProjMatches = 30;
    if(nNumProjMatches >= nProjMatches)
    {
        return true;
    }

    return false;
}

int PlaceRecognition::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                         set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                         vector<MapPoint*> &vpMatchedMapPoints)
{
    int nNumCovisibles = 10;
    vector<KeyFrame*> vpCovKFm = pMatchedKFw->GetBestCovisibilityKeyFrames(nNumCovisibles);
    int nInitialCov = vpCovKFm.size();
    vpCovKFm.push_back(pMatchedKFw);
    set<KeyFrame*> spCheckKFs(vpCovKFm.begin(), vpCovKFm.end());
    set<KeyFrame*> spCurrentCovisbles = pCurrentKF->GetConnectedKeyFrames();
    if(nInitialCov < nNumCovisibles)
    {
        for(int i=0; i<nInitialCov; ++i)
        {
            vector<KeyFrame*> vpKFs = vpCovKFm[i]->GetBestCovisibilityKeyFrames(nNumCovisibles);
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
    set<MapPoint*> spMapPoints;
    vpMapPoints.clear();
    vpMatchedMapPoints.clear();
    for(KeyFrame* pKFi : vpCovKFm)
    {
        for(MapPoint* pMPij : pKFi->GetMapPointMatches())
        {
            if(!pMPij || pMPij->isBad())
            {
                if(pMPij && pMPij->isBad()) LT_PROBE_BAD("PRFindMatchesB.669", 'M', pMPij->mnId);
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

    vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
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

// P11-F3 (OWNERSHIP D5, Fix.LcResetWipe): full machine wipe for
// LoopClosing::ResetIfRequested -- both channels, called from both reset
// branches, ONLY with the flag armed (level 0 preserves the upstream leak:
// queue cleared, channels survive pointing into the torn-down map for up to
// two KFs). ChannelWipe's shape derefs the KF latches unconditionally (all
// its upstream call sites hold a live hypothesis), but a reset can fire
// before any BoW seed ever ran, so guard on lastCurrentKF: the seed writes
// lastCurrentKF and matchedKF together, so a null lastCurrentKF means a
// never-seeded channel with nothing latched. A previously wiped channel
// keeps stale non-null pointers by design; the repeated SetErase there is
// idempotent latch release (and the #19 deferred-SetBadFlag completion
// path), which is exactly the hygiene this flag buys.
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
