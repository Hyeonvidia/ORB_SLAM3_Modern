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


#ifndef PLACERECOGNITION_H
#define PLACERECOGNITION_H

#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "map/MapTypes.hpp"  // R4b: MapPointPtr

#include <set>
#include <vector>

namespace ORB_SLAM3
{

class LoopClosing;
class KeyFrame;
class MapPoint;

// P9-5: the place-recognition (detection) half of loop closing, extracted
// VERBATIM from LoopClosing (docs/P9_RECON.md §1-2 and §7 P9-5; P8-4
// ImuInitializer precedent -- mechanical mHost. prefixing, no logic edits).
//
// Ownership boundary:
//   here -- the two-channel detection state machine (DetectionChannel +
//           mLoopCh/mMergeCh, P9-4) and the five detection functions
//           (NewDetectCommonRegions and its four helpers).
//   host -- the keyframe queue and EVERY mutation of it (the pop +
//           SetNotErase block became LoopClosing::PopNewKeyFrame, called
//           from NewDetectCommonRegions -- the P8-4
//           PurgeNewKeyFramesAfterInertialInit mirror), mpCurrentKF /
//           mpLastMap, all consume-side state (mg2oLoopScw / mg2oMergeScw /
//           mSold_new / mvpLoopMapPoints / mvpMergeConnectedKFs),
//           CorrectLoop / MergeLocal / MergeLocal2 / SearchAndFuse / GBA,
//           Run() and the reset/finish protocol.
//
// LoopClosing::Run consumes hypotheses through the const channel accessors
// (reads only) and mutates the machine solely through the named wipe entry
// points below, which keep the per-site trace reasons (docs/DIVERGENCES.md
// #21, docs/P9_RECON.md D1-D5).
//
// Everything here runs on the LoopClosing thread exactly as before -- the
// memory-model status of every access is unchanged (docs/OWNERSHIP.md,
// LC races R-a..f). Host state is reached through the friend grant on
// LoopClosing (P8-4 pattern).
class PlaceRecognition
{
public:
    explicit PlaceRecognition(LoopClosing& host);

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
    //
    // P11-F3: three of those asymmetries are now FIXABLE behind runtime
    // flags, all OFF by default (level 0 = the verbatim behavior above):
    //   Fix.LoopStateHygiene  #21 -- loop reffine-fail clears detected;
    //                         the Run() scale-abort discards an escaping
    //                         DETECTED loop hypothesis (arm A, in
    //                         LoopClosing.cpp).
    //   Fix.LatchHygiene      L1/L2 -- ChannelBoWSeed releases the old
    //                         latches, zeroes the carried numNotFound, and
    //                         refuses cnt==0 seeds ("bow-seed-refused-cnt0"
    //                         trace).
    //   Fix.LcResetWipe       D5 -- ResetIfRequested wipes both channels
    //                         via ResetChannels() ("wipe-reset" trace).
    // ------------------------------------------------------------------
    struct DetectionChannel {
        int  numCoincidences = 0;
        int  numNotFound = 0;
        bool detected = false;
        KeyFramePtr lastCurrentKF = nullptr;   // upstream left these uninitialized;
        KeyFramePtr matchedKF = nullptr;       // nullptr init is strictly safer and unobservable
        g2o::Sim3 slw;
        std::vector<MapPointPtr> mps;
        std::vector<MapPointPtr> matchedMps;
    };

    // Detection entry point, called from LoopClosing::Run() only (still the
    // LoopClosing thread).
    bool NewDetectCommonRegions();

    // Consume-side reads for LoopClosing::Run/CorrectLoop/MergeLocal[2]
    // (hypothesis Sim3/KF/MP payloads). Const: Run's four machine
    // mutations go through the named wipes below, nothing else writes.
    const DetectionChannel& LoopCh() const { return mLoopCh; }
    const DetectionChannel& MergeCh() const { return mMergeCh; }

    // P9-5: named consume-site wipes for LoopClosing::Run (1:1 with the
    // four Run-side ChannelWipe call sites, per-site trace reasons kept).
    void WipeMergeAfterConsume();     // "wipe-consume" (merge)
    void WipeLoopOnMergePriority();   // "wipe-merge-priority-discard"
    void WipeLoopAfterConsume();      // "wipe-consume" (loop)
    void WipeMergeOnScaleAbort();     // "wipe-scale-abort"
    // P11-F3 (Fix.LcResetWipe, OWNERSHIP D5): full both-channel wipe for
    // ResetIfRequested ("wipe-reset"); null-guarded for never-seeded
    // channels. Called ONLY with the flag armed.
    void ResetChannels();
    // P9-4 D5 visibility traces for ResetIfRequested ("reset-full" /
    // "reset-active-map"): traces BOTH channels, mutates nothing.
    void TraceReset(const char* reason);

private:
    LoopClosing& mHost;

    // P11-F3: test-only access grant so tests/fixlevel/
    // detection_machine_checks.cpp can drive the REAL channel mutators
    // headlessly (seed/advance/decay/wipe/reset sequences) instead of a
    // replica -- the friend-grant precedent is LoopClosing's own grant to
    // this class (P8-4 pattern). The struct is defined only inside that
    // test binary; no production code defines or uses it.
    friend struct DetectionMachineTestAccess;

    //Methods to implement the new place recognition algorithm
    bool DetectAndReffineSim3FromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<KeyFramePtr> &vpBowCand, const char* chName, DetectionChannel &ch);
    bool DetectCommonRegionsFromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs);

    // P9-4: the ONLY mutators of DetectionChannel state after construction
    // (1:1 with the upstream mutation sites; each emits one stderr trace
    // line, P7 SetState-with-reason precedent). They live on
    // PlaceRecognition (not the struct) because they need mHost.mpCurrentKF
    // for the anchor hand-off and the trace.
    void TraceChannel(const char* ch, const char* event, const DetectionChannel& c, const char* extra = "");
    // Reffine success ("reffine-advance"): cnt++, anchor SetErase hand-off
    // to mpCurrentKF, hypothesis update, detected = cnt>=3. bResetNotFound
    // is true only for the loop channel (merge success does NOT reset it).
    void ChannelAdvance(const char* ch, DetectionChannel& c, const g2o::Sim3& gScw,
                        const std::vector<MapPointPtr>& vpMatchedMPs, bool bResetNotFound);
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
    bool ChannelBoWSeed(const char* ch, DetectionChannel& c, KeyFramePtr pBestMatchedKF,
                        int nBestNumCoincidences, const g2o::Sim3& g2oBestScw,
                        const std::vector<MapPointPtr>& vpBestMapPoints,
                        const std::vector<MapPointPtr>& vpBestMatchedMapPoints);
    int FindMatchesByProjection(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKFw, g2o::Sim3 &g2oScw,
                                std::set<MapPointPtr> &spMatchedMPinOrigin, std::vector<MapPointPtr> &vpMapPoints,
                                std::vector<MapPointPtr> &vpMatchedMapPoints);

    // P9-4: the two detection channels of the place-recognition machine
    // (struct + mutator docs above).
    DetectionChannel mLoopCh, mMergeCh;
};

} //namespace ORB_SLAM

#endif // PLACERECOGNITION_H
