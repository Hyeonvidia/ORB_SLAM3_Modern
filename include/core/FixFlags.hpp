/**
 * P11-F0: FixLevel runtime-flag infrastructure (docs/P11_RECON.md 2부 §2/§4,
 * registry: docs/DIVERGENCES.md "FixLevel 레지스트리").
 *
 * Design driver (DIVERGENCES #20): compile-time ifdefs would make ON-vs-OFF
 * different binaries, polluting every ON validation with the binary-layout
 * confound #20 measured (+9..13% stereo-inertial ATE from semantically inert
 * layout churn). Runtime flags keep ON and OFF the SAME binary, so a paired
 * gate measures only the fix's semantic effect.
 *
 * CONCURRENCY / WRITE-ONCE CONTRACT (no atomics by design):
 *   The process-global instance behind I() is written exactly once, by the
 *   System constructor (FixFlags::Set, from Settings), BEFORE any SLAM
 *   thread is spawned (LocalMapping/LoopClosing/Viewer threads are created
 *   later in the same constructor body). std::thread creation synchronizes-
 *   with the start of the new thread (C++17 [thread.thread.constr]), so
 *   every worker thread observes the fully-written flags without atomics or
 *   locks. After that single Set the instance is treated as immutable —
 *   readers may cache `const bool` copies freely (hot paths should: cache
 *   at construction, e.g. one bool member per optimizer edge).
 *   Corollaries:
 *     - Set MUST NOT be called while SLAM threads are running. Sequential
 *       System lifetimes (construct → Shutdown → construct) are fine;
 *       concurrent System instances are not (the codebase does not support
 *       them anyway).
 *     - Code that runs before System's Set (e.g. standalone Settings use in
 *       tests) sees the all-false default = level 0, upstream behavior.
 *
 * Flag semantics: false = bug-for-bug upstream-equivalent behavior (level 0,
 * the permanent gate configuration). true = the corresponding documented fix
 * is armed. P11-F0 ships the infrastructure only — NO fix code reads these
 * flags yet; every flag is a no-op by construction until its P11-F1..F6
 * commit lands.
 *
 * Yaml surface (parsed in FixFlags::Parse, called from Settings):
 *   FixLevel: 0|1|2            preset (absent = 0 = all false)
 *     level 1 = state-hygiene set: loopStateHygiene, latchHygiene,
 *               lcResetWipe, stereoMbInit, rectifiedResizeCal2
 *     level 2 = all eight flags
 *   Fix.<Name>: 0|1            per-fix override, wins over the preset
 * Gate yamls carry none of these keys → level 0 by absence, and startup is
 * silent (the provenance banner prints only when a non-default flag is set).
 */

#ifndef ORB_SLAM3_FIXFLAGS_H
#define ORB_SLAM3_FIXFLAGS_H

#include <string>

namespace cv {
    class FileStorage;
}

namespace ORB_SLAM3 {

    struct FixFlags {
        // -- level-2-only fixes (numeric-behavior changes) ------------------
        // DIVERGENCES #9: EdgeInertialGS::linearizeOplus chain-rule factor s
        // for the multiplicative VertexScale update (also carries #24, the
        // relaxed bFixedScale into OptimizeSim3 — same MI early-scale-window
        // semantics, same validation runs; lands P11-F1/F2). yaml:
        // Fix.ScaleJacobian
        bool scaleJacobianChainRule = false;

        // -- level-1 state-hygiene set --------------------------------------
        // DIVERGENCES #21: loop-detection poisoned-consume composite (merge
        // scale-abort leaves detected+stale slw; reffine-fail keeps
        // detected). yaml: Fix.LoopStateHygiene (P11-F3)
        bool loopStateHygiene = false;
        // OWNERSHIP L1/L2 (P9_RECON D2/D3): SetNotErase latch leaks in
        // ChannelBoWSeed. yaml: Fix.LatchHygiene (P11-F3)
        bool latchHygiene = false;
        // OWNERSHIP D5: LC reset leaves detection channels pointing into the
        // torn-down map. yaml: Fix.LcResetWipe (P11-F3)
        bool lcResetWipe = false;

        // -- level-2-only (numeric) -----------------------------------------
        // DIVERGENCES #13: SimplicialLDLT vendored solver for the two
        // OptimizeEssentialGraph 7_3 sites (LLT robustness cliff). yaml:
        // Fix.LdltPoseGraph (P11-F5)
        bool ldltPoseGraph = false;

        // -- level-1 state-hygiene set --------------------------------------
        // DIVERGENCES #5: Frame::mb read-before-init in the stereo ctor
        // (ComputeStereoMatches minZ garbage — genuine UB). yaml:
        // Fix.StereoMbInit (P11-F4)
        bool stereoMbInit = false;

        // -- level-2-only (legacy semantics) --------------------------------
        // DIVERGENCES #3: restore the legacy parser's fps-sized post-reloc
        // IMU-reset window (mnFramesToResetIMU = mMaxFrames instead of 0).
        // yaml: Fix.LegacyImuResetWindow (P11-F6)
        bool legacyImuResetWindow = false;

        // -- level-1 state-hygiene set --------------------------------------
        // DIVERGENCES #6: Rectified calibration2_ synthesized from
        // pre-resize calibration1_. yaml: Fix.RectifiedResizeCal2 (P11-F6)
        bool rectifiedResizeCal2 = false;

        // Process-global read accessor. All-false (level 0) until System's
        // single Set — see the write-once contract above.
        static const FixFlags& I();

        // Write-once installer. Called by the System constructor, from
        // Settings, before any thread spawn. See the contract above.
        static void Set(const FixFlags& flags);

        // Parse the yaml surface (FixLevel preset, then per-fix overrides)
        // from an open FileStorage. Absent keys = level 0 (all false).
        // Malformed values (non-integer, FixLevel outside 0..2) abort with a
        // diagnostic, matching Settings' readParameter policy.
        static FixFlags Parse(const cv::FileStorage& fSettings);

        // Space-separated list of active (true) flag names; empty string at
        // level 0. Drives the one-line stderr provenance banner in System.
        std::string ActiveList() const;
    };

}  // namespace ORB_SLAM3

#endif  // ORB_SLAM3_FIXFLAGS_H
