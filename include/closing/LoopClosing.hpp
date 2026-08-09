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
#include "mapping/LocalMapping.hpp"
#include "map/Atlas.hpp"
#include "features/ORBVocabulary.hpp"

#include "recognition/KeyFrameDatabase.hpp"

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

    typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
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
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }

    void RequestFinish();

    bool isFinished();

#ifdef REGISTER_TIMES

    vector<double> vdDataQuery_ms;
    vector<double> vdEstSim3_ms;
    vector<double> vdPRTotal_ms;

    vector<double> vdMergeMaps_ms;
    vector<double> vdWeldingBA_ms;
    vector<double> vdMergeOptEss_ms;
    vector<double> vdMergeTotal_ms;
    vector<int> vnMergeKFs;
    vector<int> vnMergeMPs;
    int nMerges;

    vector<double> vdLoopFusion_ms;
    vector<double> vdLoopOptEss_ms;
    vector<double> vdLoopTotal_ms;
    int nLoop;

    vector<double> vdGBA_ms;
    vector<double> vdUpdateMap_ms;
    vector<double> vdFGBATotal_ms;
    vector<int> vnGBAKFs;
    vector<int> vnGBAMPs;
    int nFGBA_exec;
    int nFGBA_abort;

#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();


    // ------------------------------------------------------------------
    // P9-4: one detection channel of the place-recognition machine
    // (docs/P9_RECON.md §1). States are implied exactly as upstream:
    // IDLE (numCoincidences==0, !detected), ACCUM (1-2), DETECTED
    // (detected). Pointers/Sim3 are NOT cleared on wipes (upstream
    // semantics -- validity is implied by numCoincidences/detected, see
    // P9_RECON D5/L2).
    //
    // Wipe shapes (each has its own trace event; do NOT unify):
    //   wipe-consume               Run() after a merge / after a loop
    //                              (good or BAD) -- full clear incl.
    //                              detected.
    //   wipe-merge-priority-discard  Run(), loop hypothesis discarded
    //                              unconsumed because merge won -- full
    //                              clear incl. detected.
    //   wipe-scale-abort           Run(), inertial merge scale gate
    //                              failed -- full clear incl. detected;
    //                              the `continue` then SKIPS the loop
    //                              branch and the mpLastCurrentKF update.
    //   wipe-decay                 NewDetectCommonRegions(), 2nd reffine
    //                              failure -- clears counters/vectors but
    //                              NOT detected.
    //
    // Load-bearing asymmetries preserved verbatim (docs/DIVERGENCES.md
    // #21, docs/P9_RECON.md D1-D5): loop reffine-fail does NOT clear
    // detected (merge's does); merge reffine-success does NOT reset
    // numNotFound (loop's does); BoW seeding does NOT reset numNotFound;
    // ResetIfRequested does NOT clear the machine at all (D5 -- it only
    // traces both channels).
    // ------------------------------------------------------------------
    struct DetectionChannel {
        int  numCoincidences = 0;
        int  numNotFound = 0;
        bool detected = false;
        KeyFrame* lastCurrentKF = nullptr;   // upstream left these uninitialized;
        KeyFrame* matchedKF = nullptr;       // nullptr init is strictly safer and unobservable
        g2o::Sim3 slw;
        std::vector<MapPoint*> mps;
        std::vector<MapPoint*> matchedMps;
    };

    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, const char* chName, DetectionChannel &ch);
    bool DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs);

    // P9-4: the ONLY mutators of DetectionChannel state after construction
    // (1:1 with the upstream mutation sites; each emits one stderr trace
    // line, P7 SetState-with-reason precedent). They live on LoopClosing
    // (not the struct) because they need mpCurrentKF for the anchor
    // hand-off and the trace.
    void TraceChannel(const char* ch, const char* event, const DetectionChannel& c, const char* extra = "");
    // Reffine success ("reffine-advance"): cnt++, anchor SetErase hand-off
    // to mpCurrentKF, hypothesis update, detected = cnt>=3. bResetNotFound
    // is true only for the loop channel (merge success does NOT reset it).
    void ChannelAdvance(const char* ch, DetectionChannel& c, const g2o::Sim3& gScw,
                        const std::vector<MapPoint*>& vpMatchedMPs, bool bResetNotFound);
    // Reffine failure ("reffine-fail"): notFound++, then the decay wipe at
    // >=2 ("wipe-decay", does NOT clear detected). bClearDetected is true
    // only for the merge channel (loop failure does NOT clear the flag).
    void ChannelDecayStep(const char* ch, DetectionChannel& c, bool bClearDetected);
    // Full wipe (consume / merge-priority-discard / scale-abort): SetErase
    // both KFs, clear counters+flag+vectors. `reason` is the trace event.
    void ChannelWipe(const char* ch, DetectionChannel& c, const char* reason);
    // BoW commit block ("bow-seed"): seeds/overwrites the hypothesis. Does
    // NOT SetErase the previous anchor/matched KF (latch leak L1, upstream)
    // and does NOT reset numNotFound. Returns the new detected flag.
    bool ChannelBoWSeed(const char* ch, DetectionChannel& c, KeyFrame* pBestMatchedKF,
                        int nBestNumCoincidences, const g2o::Sim3& g2oBestScw,
                        const std::vector<MapPoint*>& vpBestMapPoints,
                        const std::vector<MapPoint*>& vpBestMatchedMapPoints);
    int FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
                                vector<MapPoint*> &vpMatchedMapPoints);


    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints);
    void SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map* mpMapToReset;
    std::mutex mMutexReset;

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

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpLastCurrentKF;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpLoopMapPoints;

    //-------
    Map* mpLastMap;

    // P9-4: the two detection channels of the place-recognition machine
    // (struct + mutator docs above, near DetectCommonRegionsFromBoW).
    DetectionChannel mLoopCh, mMergeCh;

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
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    int mnFullBAIdx;


    // To (de)activate LC
    bool mbActiveLC = true;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
