/**
 * P11-F0 FixFlags parse/preset checks (docs/P11_RECON.md 2부 §2/§4).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure,
 * total runtime well under 1 s.
 *
 * What is asserted (the level-0 inertness contract's config half):
 *   (a) aggregate default and the process-global I() are ALL FALSE before
 *       any Set — level 0 is the built-in state, not a parsed one;
 *   (b) a yaml with no Fix keys parses to all-false (gate yamls carry no
 *       keys, so this IS the gate configuration);
 *   (c) FixLevel: 1 arms exactly the state-hygiene set {loopStateHygiene,
 *       latchHygiene, lcResetWipe, stereoMbInit, rectifiedResizeCal2};
 *   (d) FixLevel: 2 arms all eight flags;
 *   (e) individual Fix.* keys override the preset in BOTH directions
 *       (force-off a preset-armed flag, force-on a preset-dark flag);
 *   (f) an individual key with no preset arms only itself;
 *   (g) Set/I round trip: System's installer publishes the parsed struct.
 *
 * Temp yamls are written by the test itself (unique per-PID paths under
 * /tmp) and removed on the success path.
 */

#include "core/FixFlags.hpp"

#include <opencv2/core/persistence.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

using ORB_SLAM3::FixFlags;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #cond,        \
                         __FILE__, __LINE__);                                \
            std::_Exit(1);                                                   \
        }                                                                    \
    } while (0)

static bool allFalse(const FixFlags& f)
{
    return !f.scaleJacobianChainRule && !f.loopStateHygiene &&
           !f.latchHygiene && !f.lcResetWipe && !f.ldltPoseGraph &&
           !f.stereoMbInit && !f.legacyImuResetWindow &&
           !f.rectifiedResizeCal2;
}

// Write `body` (appended to the mandatory %YAML directive) to a unique temp
// path and parse it through the production FixFlags::Parse.
static FixFlags parseYaml(const std::string& body, int idx)
{
    const std::string path = "/tmp/fixflags_checks_" +
                             std::to_string(getpid()) + "_" +
                             std::to_string(idx) + ".yaml";
    {
        std::ofstream out(path);
        CHECK(out.good());
        out << "%YAML:1.0\n---\n" << body;
        CHECK(out.good());
    }
    cv::FileStorage fs(path, cv::FileStorage::READ);
    CHECK(fs.isOpened());
    const FixFlags f = FixFlags::Parse(fs);
    fs.release();
    std::remove(path.c_str());
    return f;
}

int main()
{
    // (a) Defaults: aggregate and the untouched process-global are level 0.
    {
        const FixFlags d{};
        CHECK(allFalse(d));
        CHECK(allFalse(FixFlags::I()));
        CHECK(FixFlags::I().ActiveList().empty());
        std::fprintf(stderr, "ok  a_defaults_all_false\n");
    }

    // (b) Absent keys = level 0 (a yaml with unrelated content only).
    {
        const FixFlags f = parseYaml("File.version: \"1.0\"\n", 0);
        CHECK(allFalse(f));
        std::fprintf(stderr, "ok  b_absent_keys_level0\n");
    }

    // (c) FixLevel: 1 = exactly the state-hygiene set.
    {
        const FixFlags f = parseYaml("FixLevel: 1\n", 1);
        CHECK(f.loopStateHygiene);
        CHECK(f.latchHygiene);
        CHECK(f.lcResetWipe);
        CHECK(f.stereoMbInit);
        CHECK(f.rectifiedResizeCal2);
        CHECK(!f.scaleJacobianChainRule);
        CHECK(!f.ldltPoseGraph);
        CHECK(!f.legacyImuResetWindow);
        std::fprintf(stderr, "ok  c_level1_hygiene_set\n");
    }

    // (d) FixLevel: 2 = all eight.
    {
        const FixFlags f = parseYaml("FixLevel: 2\n", 2);
        CHECK(f.scaleJacobianChainRule);
        CHECK(f.loopStateHygiene);
        CHECK(f.latchHygiene);
        CHECK(f.lcResetWipe);
        CHECK(f.ldltPoseGraph);
        CHECK(f.stereoMbInit);
        CHECK(f.legacyImuResetWindow);
        CHECK(f.rectifiedResizeCal2);
        std::fprintf(stderr, "ok  d_level2_all\n");
    }

    // (e) Individual keys override the preset in both directions:
    // FixLevel:1 arms loopStateHygiene (forced off here) and leaves
    // scaleJacobianChainRule dark (forced on here); untouched preset
    // members keep their preset values.
    {
        const FixFlags f = parseYaml(
            "FixLevel: 1\n"
            "Fix.LoopStateHygiene: 0\n"
            "Fix.ScaleJacobian: 1\n",
            3);
        CHECK(!f.loopStateHygiene);       // preset-on, key forces OFF
        CHECK(f.scaleJacobianChainRule);  // preset-off, key forces ON
        CHECK(f.latchHygiene);            // preset survives untouched
        CHECK(f.lcResetWipe);
        CHECK(f.stereoMbInit);
        CHECK(f.rectifiedResizeCal2);
        CHECK(!f.ldltPoseGraph);          // stays dark at level 1
        CHECK(!f.legacyImuResetWindow);
        std::fprintf(stderr, "ok  e_override_beats_preset\n");
    }

    // (f) Individual key with no preset arms only itself.
    {
        const FixFlags f = parseYaml("Fix.LdltPoseGraph: 1\n", 4);
        CHECK(f.ldltPoseGraph);
        CHECK(!f.scaleJacobianChainRule);
        CHECK(!f.loopStateHygiene);
        CHECK(!f.latchHygiene);
        CHECK(!f.lcResetWipe);
        CHECK(!f.stereoMbInit);
        CHECK(!f.legacyImuResetWindow);
        CHECK(!f.rectifiedResizeCal2);
        CHECK(f.ActiveList() == "ldltPoseGraph");
        std::fprintf(stderr, "ok  f_single_key_no_preset\n");
    }

    // (g) Set/I round trip — LAST: mutates the process-global the earlier
    // checks assert is still default.
    {
        const FixFlags f = parseYaml("FixLevel: 1\n", 5);
        FixFlags::Set(f);
        CHECK(FixFlags::I().loopStateHygiene);
        CHECK(FixFlags::I().stereoMbInit);
        CHECK(!FixFlags::I().scaleJacobianChainRule);
        CHECK(FixFlags::I().ActiveList() ==
              "loopStateHygiene latchHygiene lcResetWipe stereoMbInit "
              "rectifiedResizeCal2");
        std::fprintf(stderr, "ok  g_set_publishes\n");
    }

    std::fprintf(stderr, "fixflags_checks: ALL PASS\n");
    return 0;
}
