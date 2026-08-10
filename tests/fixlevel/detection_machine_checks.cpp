/**
 * P11-F3 detection-machine checks (DIVERGENCES #21, OWNERSHIP L1/L2/D5;
 * flags Fix.LoopStateHygiene / Fix.LatchHygiene / Fix.LcResetWipe, all OFF
 * by default; docs/P11_RECON.md 2부 FL-21, FL-L1/L2, FL-D5).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure,
 * bounded well under 2 s.
 *
 * WHAT IS DRIVEN — the REAL machine, not a replica: a minimally constructed
 * LoopClosing host (null collaborators; its ctor only stores them) and its
 * own PlaceRecognition value member, reached through the
 * DetectionMachineTestAccess friend grant (P11-F3; the precedent is
 * LoopClosing's own friend grant to PlaceRecognition). KeyFrames are real
 * default-constructed KeyFrame objects (public ctor, boost-serialization
 * requirement) via a subclass that exposes the protected mbNotErase latch
 * bit; SetNotErase/SetErase run the production latch protocol (SetBadFlag
 * is unreachable: mbToBeErased is never set here).
 *
 * TWO deliberate replications, stated per the task contract:
 *   - the loop reffine-fail CALL-SITE ARGUMENT
 *     (bClearDetected = FixFlags::I().loopStateHygiene,
 *     PlaceRecognition.cpp NewDetectCommonRegions) is reproduced textually
 *     when driving ChannelDecayStep, because NewDetectCommonRegions itself
 *     needs BoW machinery a unit test cannot host;
 *   - the scale-abort GUARD EXPRESSION
 *     (FixFlags::I().loopStateHygiene && LoopCh().detected,
 *     LoopClosing.cpp Run) is reproduced for the same reason; the wipe it
 *     invokes (WipeLoopOnMergePriority) is the real public entry point.
 * Everything else (seed / advance / decay / wipe / ResetIfRequested) is the
 * production code path, including the real flag consultations inside
 * ChannelBoWSeed and ResetIfRequested.
 *
 * Flag phases are armed through the PUBLIC write-once installer
 * FixFlags::Set. It is called more than once here, which the header
 * contract tolerates in exactly this setting: the binary is single-threaded
 * and never spawns a SLAM thread, so the Set-before-any-thread
 * happens-before requirement is vacuous (one step beyond the FL-9 test's
 * single mid-binary Set, same wiring). Each phase installs EXACTLY ONE
 * flag, so cross-flag independence is asserted, not assumed.
 *
 * Assertion map (task contract):
 *   (a) OFF preserves the #21 asymmetries verbatim ....... offPhase()
 *   (b) ON clears detected on loop decay / wipes on
 *       scale-abort (Fix.LoopStateHygiene) ................ loopHygienePhase()
 *   (c) ON releases old latches on re-seed and refuses
 *       cnt==0 latch (Fix.LatchHygiene) ................... latchHygienePhase()
 *   (d) ON reset wipes channels (Fix.LcResetWipe) ......... resetWipePhase()
 */

#include "closing/LoopClosing.hpp"

#include "core/FixFlags.hpp"

#include <cstdio>
#include <cstdlib>
#include <mutex>

using ORB_SLAM3::FixFlags;
using ORB_SLAM3::KeyFrame;
using ORB_SLAM3::LoopClosing;
using ORB_SLAM3::PlaceRecognition;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #cond,        \
                         __FILE__, __LINE__);                                \
            std::_Exit(1);                                                   \
        }                                                                    \
    } while (0)

namespace ORB_SLAM3 {

// Real KeyFrame with the protected latch state made observable. The default
// KeyFrame ctor is public (boost deserialization entry) and touches no
// collaborator; mnId is public but indeterminate there, so pin it.
struct TestKF : public KeyFrame {
    explicit TestKF(long unsigned int id) { mnId = id; }
    bool Latched() const { return mbNotErase; }
};

// P11-F3 friend grant (declared in PlaceRecognition.hpp / LoopClosing.hpp).
// Thin static funnels only — no logic of its own.
struct DetectionMachineTestAccess {
    using Channel = PlaceRecognition::DetectionChannel;

    static PlaceRecognition& PR(LoopClosing& lc) { return lc.mPlaceRec; }
    static Channel& LoopCh(LoopClosing& lc) { return lc.mPlaceRec.mLoopCh; }
    static Channel& MergeCh(LoopClosing& lc) { return lc.mPlaceRec.mMergeCh; }

    // Mirrors LoopClosing::PopNewKeyFrame's observable effect on the
    // machine's inputs: current-KF adoption + its SetNotErase latch.
    static void PopKF(LoopClosing& lc, KeyFrame* pKF)
    {
        lc.mpCurrentKF = pKF;
        pKF->SetNotErase();
    }

    static bool Seed(LoopClosing& lc, const char* ch, Channel& c,
                     KeyFrame* pMatched, int nCoincidences)
    {
        return lc.mPlaceRec.ChannelBoWSeed(
            ch, c, pMatched, nCoincidences, g2o::Sim3(),
            std::vector<MapPoint*>(), std::vector<MapPoint*>());
    }

    static void Decay(LoopClosing& lc, const char* ch, Channel& c,
                      bool bClearDetected)
    {
        lc.mPlaceRec.ChannelDecayStep(ch, c, bClearDetected);
    }

    static void FireResetFull(LoopClosing& lc)
    {
        {
            std::unique_lock<std::mutex> lock(lc.mMutexReset);
            lc.mbResetRequested = true;
        }
        lc.ResetIfRequested();
    }

    static void FireResetActiveMap(LoopClosing& lc)
    {
        {
            std::unique_lock<std::mutex> lock(lc.mMutexReset);
            lc.mbResetActiveMapRequested = true;
            lc.mpMapToReset = nullptr;  // queue is empty: never dereferenced
        }
        lc.ResetIfRequested();
    }
};

}  // namespace ORB_SLAM3

using Access = ORB_SLAM3::DetectionMachineTestAccess;
using ORB_SLAM3::TestKF;

namespace {

// Minimal host: the LoopClosing ctor only stores its collaborators, so null
// injection is safe as long as the test never enters queue/consume paths
// (it doesn't — it drives the machine and ResetIfRequested only).
LoopClosing* MakeHost()
{
    return new LoopClosing(/*pAtlas=*/nullptr, /*pDB=*/nullptr,
                           /*bFixScale=*/true, /*bActiveLC=*/true,
                           /*pBAEpochs=*/nullptr, /*pOptimizer=*/nullptr);
}

// ---------------------------------------------------------------------------
// (a) OFF: the #21 / L1 / L2 / D5 shapes, verbatim.
// ---------------------------------------------------------------------------
void offPhase()
{
    CHECK(!FixFlags::I().loopStateHygiene);
    CHECK(!FixFlags::I().latchHygiene);
    CHECK(!FixFlags::I().lcResetWipe);

    LoopClosing* host = MakeHost();
    auto& loopCh = Access::LoopCh(*host);
    auto& mergeCh = Access::MergeCh(*host);

    TestKF kfC1(101), kfC2(102), kfC3(103);
    TestKF kfM1(201), kfM2(202), kfM3(203), kfM4(204), kfM5(205);

    // a1: BoW seed latches the matched KF and sets detected at cnt>=3.
    Access::PopKF(*host, &kfC1);
    CHECK(Access::Seed(*host, "loop", loopCh, &kfM1, 3));
    CHECK(loopCh.detected);
    CHECK(loopCh.numCoincidences == 3);
    CHECK(kfM1.Latched());
    std::fprintf(stderr, "ok  a1_off_seed_latches_and_detects\n");

    // a2: loop reffine-fail does NOT clear detected (#21 arm B; the
    // bClearDetected argument reproduces the production call site verbatim:
    // /*bClearDetected=*/FixFlags::I().loopStateHygiene).
    Access::Decay(*host, "loop", loopCh,
                  /*bClearDetected=*/FixFlags::I().loopStateHygiene);
    CHECK(loopCh.numNotFound == 1);
    CHECK(loopCh.detected);  // the #21 latch, preserved
    std::fprintf(stderr, "ok  a2_off_loop_decay_keeps_detected\n");

    // a3: merge twin DOES clear detected on every failure (the asymmetry).
    CHECK(Access::Seed(*host, "merge", mergeCh, &kfM2, 3));
    CHECK(mergeCh.detected);
    Access::Decay(*host, "merge", mergeCh, /*bClearDetected=*/true);
    CHECK(!mergeCh.detected);
    std::fprintf(stderr, "ok  a3_off_merge_decay_clears_detected\n");

    // a4: scale-abort guard replica (LoopClosing.cpp Run): OFF never wipes,
    // so the DETECTED loop hypothesis escapes the iteration intact — the
    // stale-anchor consume shape of #21 arm A.
    const bool bWouldWipe =
        FixFlags::I().loopStateHygiene && Access::PR(*host).LoopCh().detected;
    CHECK(!bWouldWipe);
    CHECK(loopCh.detected);
    std::fprintf(stderr, "ok  a4_off_scale_abort_leaves_detected\n");

    // a5: L1 — re-seed over the live hypothesis neither releases the old
    // matched-KF latch nor resets numNotFound (both carried over).
    Access::PopKF(*host, &kfC2);
    Access::Seed(*host, "loop", loopCh, &kfM3, 1);
    CHECK(kfM1.Latched());            // leak L1: old latch pinned forever
    CHECK(kfM3.Latched());
    CHECK(loopCh.numNotFound == 1);   // carried from the dead hypothesis
    CHECK(loopCh.numCoincidences == 1);
    CHECK(!loopCh.detected);          // a fresh cnt=1 seed clears detected
    std::fprintf(stderr, "ok  a5_off_reseed_leaks_latch_and_notfound\n");

    // a6: L2 — a cnt==0 seed goes through and latches an orphan the decay
    // path (numCoincidences > 0 guard) can never release.
    Access::PopKF(*host, &kfC3);
    CHECK(!Access::Seed(*host, "loop", loopCh, &kfM4, 0));
    CHECK(loopCh.numCoincidences == 0);
    CHECK(kfM4.Latched());            // the orphan latch
    std::fprintf(stderr, "ok  a6_off_cnt0_seed_latches_orphan\n");

    // a7: D5 — a reset clears the queue only; the machine (counters,
    // latches, detected) survives untouched.
    Access::Seed(*host, "loop", loopCh, &kfM5, 3);  // live DETECTED state
    CHECK(loopCh.detected);
    Access::FireResetFull(*host);
    CHECK(loopCh.numCoincidences == 3);
    CHECK(loopCh.detected);
    CHECK(kfM5.Latched());
    std::fprintf(stderr, "ok  a7_off_reset_leaves_machine\n");
}

// ---------------------------------------------------------------------------
// (b) ON Fix.LoopStateHygiene only.
// ---------------------------------------------------------------------------
void loopHygienePhase()
{
    {
        FixFlags f;
        f.loopStateHygiene = true;
        FixFlags::Set(f);
    }
    CHECK(FixFlags::I().loopStateHygiene);
    CHECK(!FixFlags::I().latchHygiene);
    CHECK(!FixFlags::I().lcResetWipe);

    LoopClosing* host = MakeHost();
    auto& loopCh = Access::LoopCh(*host);

    TestKF kfC1(111), kfC2(112);
    TestKF kfM1(211), kfM2(212), kfM3(213);

    // b1: loop reffine-fail now clears detected (production call-site
    // argument, now true).
    Access::PopKF(*host, &kfC1);
    Access::Seed(*host, "loop", loopCh, &kfM1, 3);
    CHECK(loopCh.detected);
    Access::Decay(*host, "loop", loopCh,
                  /*bClearDetected=*/FixFlags::I().loopStateHygiene);
    CHECK(!loopCh.detected);
    CHECK(loopCh.numNotFound == 1);
    std::fprintf(stderr, "ok  b1_on_loop_decay_clears_detected\n");

    // b2: scale-abort guard replica now fires and the real wipe runs: the
    // escaping DETECTED hypothesis is discarded (counters, flag, latches).
    Access::PopKF(*host, &kfC2);
    Access::Seed(*host, "loop", loopCh, &kfM2, 3);
    CHECK(loopCh.detected);
    if (FixFlags::I().loopStateHygiene && Access::PR(*host).LoopCh().detected)
        Access::PR(*host).WipeLoopOnMergePriority();
    CHECK(loopCh.numCoincidences == 0);
    CHECK(!loopCh.detected);
    CHECK(!kfM2.Latched());           // wipe released the matched latch
    CHECK(!kfC2.Latched());           // ...and the anchor latch
    std::fprintf(stderr, "ok  b2_on_scale_abort_wipes_loop\n");

    // b3: flag independence — latchHygiene is OFF in this phase, so a
    // re-seed still leaks the old latch (L1 untouched by this flag).
    Access::Seed(*host, "loop", loopCh, &kfM1, 1);
    Access::Seed(*host, "loop", loopCh, &kfM3, 1);
    CHECK(kfM1.Latched());
    std::fprintf(stderr, "ok  b3_on_loop_hygiene_does_not_touch_latches\n");
}

// ---------------------------------------------------------------------------
// (c) ON Fix.LatchHygiene only.
// ---------------------------------------------------------------------------
void latchHygienePhase()
{
    {
        FixFlags f;
        f.latchHygiene = true;
        FixFlags::Set(f);
    }
    CHECK(FixFlags::I().latchHygiene);
    CHECK(!FixFlags::I().loopStateHygiene);
    CHECK(!FixFlags::I().lcResetWipe);

    LoopClosing* host = MakeHost();
    auto& loopCh = Access::LoopCh(*host);

    TestKF kfC1(121), kfC2(122), kfC3(123);
    TestKF kfA(221), kfB(222), kfC(223);

    // c1: re-seed releases the old matched latch AND the old anchor latch,
    // and zeroes the carried numNotFound.
    Access::PopKF(*host, &kfC1);
    Access::Seed(*host, "loop", loopCh, &kfA, 2);
    CHECK(kfA.Latched());
    CHECK(kfC1.Latched());            // pop latch = anchor latch
    Access::Decay(*host, "loop", loopCh, /*bClearDetected=*/false);
    CHECK(loopCh.numNotFound == 1);
    Access::PopKF(*host, &kfC2);
    Access::Seed(*host, "loop", loopCh, &kfB, 2);
    CHECK(!kfA.Latched());            // L1 fixed: old matched released
    CHECK(!kfC1.Latched());           // old anchor released
    CHECK(kfB.Latched());
    CHECK(loopCh.numNotFound == 0);   // carryover zeroed
    CHECK(loopCh.numCoincidences == 2);
    std::fprintf(stderr, "ok  c1_on_reseed_releases_old_latches\n");

    // c2: cnt==0 seed is refused — no orphan latch, existing hypothesis
    // (and its latches) intact.
    Access::PopKF(*host, &kfC3);
    CHECK(!Access::Seed(*host, "loop", loopCh, &kfC, 0));
    CHECK(!kfC.Latched());            // L2 fixed: no latch taken
    CHECK(loopCh.matchedKF == &kfB);  // prior hypothesis survives
    CHECK(kfB.Latched());
    CHECK(loopCh.numCoincidences == 2);
    std::fprintf(stderr, "ok  c2_on_cnt0_seed_refused\n");
}

// ---------------------------------------------------------------------------
// (d) ON Fix.LcResetWipe only.
// ---------------------------------------------------------------------------
void resetWipePhase()
{
    {
        FixFlags f;
        f.lcResetWipe = true;
        FixFlags::Set(f);
    }
    CHECK(FixFlags::I().lcResetWipe);
    CHECK(!FixFlags::I().loopStateHygiene);
    CHECK(!FixFlags::I().latchHygiene);

    // d1: a never-seeded machine resets without touching the (null) latches
    // — the ResetChannels null-guard.
    {
        LoopClosing* fresh = MakeHost();
        Access::FireResetFull(*fresh);
        CHECK(Access::LoopCh(*fresh).numCoincidences == 0);
        std::fprintf(stderr, "ok  d1_on_reset_fresh_machine_safe\n");
    }

    LoopClosing* host = MakeHost();
    auto& loopCh = Access::LoopCh(*host);
    auto& mergeCh = Access::MergeCh(*host);

    TestKF kfC1(131), kfC2(132);
    TestKF kfM1(231), kfM2(232), kfM3(233);

    // d2: full reset wipes BOTH channels — counters, detected, latches.
    Access::PopKF(*host, &kfC1);
    Access::Seed(*host, "loop", loopCh, &kfM1, 3);
    Access::Seed(*host, "merge", mergeCh, &kfM2, 1);
    CHECK(loopCh.detected);
    CHECK(kfM1.Latched() && kfM2.Latched());
    Access::FireResetFull(*host);
    CHECK(loopCh.numCoincidences == 0);
    CHECK(!loopCh.detected);
    CHECK(mergeCh.numCoincidences == 0);
    CHECK(!mergeCh.detected);
    CHECK(!kfM1.Latched());
    CHECK(!kfM2.Latched());
    CHECK(!kfC1.Latched());
    std::fprintf(stderr, "ok  d2_on_full_reset_wipes_channels\n");

    // d3: active-map reset wipes the same way (both branches armed).
    Access::PopKF(*host, &kfC2);
    Access::Seed(*host, "loop", loopCh, &kfM3, 2);
    CHECK(kfM3.Latched());
    Access::FireResetActiveMap(*host);
    CHECK(loopCh.numCoincidences == 0);
    CHECK(!loopCh.detected);
    CHECK(!kfM3.Latched());
    std::fprintf(stderr, "ok  d3_on_active_map_reset_wipes_channels\n");
}

}  // namespace

int main()
{
    offPhase();
    loopHygienePhase();
    latchHygienePhase();
    resetWipePhase();

    std::fprintf(stderr, "detection_machine_checks: ALL PASS\n");
    return 0;
}
