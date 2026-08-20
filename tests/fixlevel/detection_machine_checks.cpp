/**
 * Detection-machine checks (DIVERGENCES #21, OWNERSHIP L1/L2/D5; the
 * state-hygiene fixes were promoted to unconditional in R4a — this asserts
 * the hygiene contract as THE behavior, no flag phases).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure,
 * bounded well under 2 s.
 *
 * WHAT IS DRIVEN — the REAL machine, not a replica: a minimally constructed
 * LoopClosing host (null collaborators; its ctor only stores them) and its
 * own PlaceRecognition value member, reached through the
 * DetectionMachineTestAccess friend grant (the precedent is LoopClosing's
 * own friend grant to PlaceRecognition). KeyFrames are real
 * default-constructed KeyFrame objects (public ctor, boost-serialization
 * requirement) via a subclass that exposes the protected mbNotErase latch
 * bit; SetNotErase/SetErase run the production latch protocol (SetBadFlag
 * is unreachable: mbToBeErased is never set here).
 *
 * TWO deliberate replications, stated per the original task contract:
 *   - the loop reffine-fail CALL-SITE ARGUMENT (bClearDetected = true,
 *     PlaceRecognition.cpp NewDetectCommonRegions) is reproduced textually
 *     when driving ChannelDecayStep, because NewDetectCommonRegions itself
 *     needs BoW machinery a unit test cannot host;
 *   - the scale-abort GUARD EXPRESSION (LoopCh().detected,
 *     LoopClosing.cpp Run) is reproduced for the same reason; the wipe it
 *     invokes (WipeLoopOnMergePriority) is the real public entry point.
 * Everything else (seed / advance / decay / wipe / ResetIfRequested) is the
 * production code path.
 *
 * Assertion map:
 *   (a) loop decay clears detected / scale-abort wipes (#21) . loopHygiene()
 *   (b) re-seed releases old latches; cnt==0 seed refused
 *       (L1/L2) ........................................... latchHygiene()
 *   (c) reset wipes both channels (D5) .................... resetWipe()
 */

#include "closing/LoopClosing.hpp"

#include <cstdio>
#include <cstdlib>
#include <mutex>

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

// Friend grant (declared in PlaceRecognition.hpp / LoopClosing.hpp).
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
            std::vector<MapPointPtr>(), std::vector<MapPointPtr>());
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
// (a) Loop-state hygiene (#21): decay clears detected, scale-abort wipes.
// ---------------------------------------------------------------------------
void loopHygiene()
{
    LoopClosing* host = MakeHost();
    auto& loopCh = Access::LoopCh(*host);
    auto& mergeCh = Access::MergeCh(*host);

    TestKF kfC1(111), kfC2(112);
    TestKF kfM1(211), kfM2(212);

    // a1: BoW seed latches the matched KF and sets detected at cnt>=3.
    Access::PopKF(*host, &kfC1);
    CHECK(Access::Seed(*host, "loop", loopCh, &kfM1, 3));
    CHECK(loopCh.detected);
    CHECK(loopCh.numCoincidences == 3);
    CHECK(kfM1.Latched());
    std::fprintf(stderr, "ok  a1_seed_latches_and_detects\n");

    // a2: loop reffine-fail clears detected — the production call-site
    // argument (bClearDetected = true, same as the merge twin) so a failed
    // hypothesis cannot stay DETECTED off a stale anchor.
    Access::Decay(*host, "loop", loopCh, /*bClearDetected=*/true);
    CHECK(loopCh.numNotFound == 1);
    CHECK(!loopCh.detected);
    std::fprintf(stderr, "ok  a2_loop_decay_clears_detected\n");

    // a3: merge twin behaves identically.
    CHECK(Access::Seed(*host, "merge", mergeCh, &kfM2, 3));
    CHECK(mergeCh.detected);
    Access::Decay(*host, "merge", mergeCh, /*bClearDetected=*/true);
    CHECK(!mergeCh.detected);
    std::fprintf(stderr, "ok  a3_merge_decay_clears_detected\n");

    // a4: scale-abort guard replica (LoopClosing.cpp Run) fires on a
    // DETECTED hypothesis and the real wipe runs: the escaping hypothesis
    // is discarded (counters, flag, latches).
    Access::PopKF(*host, &kfC2);
    Access::Seed(*host, "loop", loopCh, &kfM1, 3);
    CHECK(loopCh.detected);
    if (Access::PR(*host).LoopCh().detected)
        Access::PR(*host).WipeLoopOnMergePriority();
    CHECK(loopCh.numCoincidences == 0);
    CHECK(!loopCh.detected);
    CHECK(!kfM1.Latched());           // wipe released the matched latch
    CHECK(!kfC2.Latched());           // ...and the anchor latch
    std::fprintf(stderr, "ok  a4_scale_abort_wipes_loop\n");
}

// ---------------------------------------------------------------------------
// (b) Latch hygiene (L1/L2): re-seed releases, cnt==0 refused.
// ---------------------------------------------------------------------------
void latchHygiene()
{
    LoopClosing* host = MakeHost();
    auto& loopCh = Access::LoopCh(*host);

    TestKF kfC1(121), kfC2(122), kfC3(123);
    TestKF kfA(221), kfB(222), kfC(223);

    // b1: re-seed releases the old matched latch AND the old anchor latch,
    // and zeroes the carried numNotFound (L1).
    Access::PopKF(*host, &kfC1);
    Access::Seed(*host, "loop", loopCh, &kfA, 2);
    CHECK(kfA.Latched());
    CHECK(kfC1.Latched());            // pop latch = anchor latch
    Access::Decay(*host, "loop", loopCh, /*bClearDetected=*/true);
    CHECK(loopCh.numNotFound == 1);
    Access::PopKF(*host, &kfC2);
    Access::Seed(*host, "loop", loopCh, &kfB, 2);
    CHECK(!kfA.Latched());            // old matched released
    CHECK(!kfC1.Latched());           // old anchor released
    CHECK(kfB.Latched());
    CHECK(loopCh.numNotFound == 0);   // carryover zeroed
    CHECK(loopCh.numCoincidences == 2);
    std::fprintf(stderr, "ok  b1_reseed_releases_old_latches\n");

    // b2: cnt==0 seed is refused — no orphan latch, existing hypothesis
    // (and its latches) intact (L2).
    Access::PopKF(*host, &kfC3);
    CHECK(!Access::Seed(*host, "loop", loopCh, &kfC, 0));
    CHECK(!kfC.Latched());            // no latch taken
    CHECK(loopCh.matchedKF == &kfB);  // prior hypothesis survives
    CHECK(kfB.Latched());
    CHECK(loopCh.numCoincidences == 2);
    std::fprintf(stderr, "ok  b2_cnt0_seed_refused\n");
}

// ---------------------------------------------------------------------------
// (c) Reset wipe (D5).
// ---------------------------------------------------------------------------
void resetWipe()
{
    // c1: a never-seeded machine resets without touching the (null) latches
    // — the ResetChannels null-guard.
    {
        LoopClosing* fresh = MakeHost();
        Access::FireResetFull(*fresh);
        CHECK(Access::LoopCh(*fresh).numCoincidences == 0);
        std::fprintf(stderr, "ok  c1_reset_fresh_machine_safe\n");
    }

    LoopClosing* host = MakeHost();
    auto& loopCh = Access::LoopCh(*host);
    auto& mergeCh = Access::MergeCh(*host);

    TestKF kfC1(131), kfC2(132);
    TestKF kfM1(231), kfM2(232), kfM3(233);

    // c2: full reset wipes BOTH channels — counters, detected, latches.
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
    std::fprintf(stderr, "ok  c2_full_reset_wipes_channels\n");

    // c3: active-map reset wipes the same way (both branches).
    Access::PopKF(*host, &kfC2);
    Access::Seed(*host, "loop", loopCh, &kfM3, 2);
    CHECK(kfM3.Latched());
    Access::FireResetActiveMap(*host);
    CHECK(loopCh.numCoincidences == 0);
    CHECK(!loopCh.detected);
    CHECK(!kfM3.Latched());
    std::fprintf(stderr, "ok  c3_active_map_reset_wipes_channels\n");
}

}  // namespace

int main()
{
    loopHygiene();
    latchHygiene();
    resetWipe();

    std::fprintf(stderr, "detection_machine_checks: ALL PASS\n");
    return 0;
}
