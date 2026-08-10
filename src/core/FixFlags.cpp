/**
 * P11-F0: FixLevel runtime-flag infrastructure — storage, yaml parse,
 * banner text. See include/core/FixFlags.hpp for the write-once contract
 * and docs/DIVERGENCES.md "FixLevel 레지스트리" for the flag registry.
 */

#include "core/FixFlags.hpp"

#include <opencv2/core/persistence.hpp>

#include <cstdlib>
#include <iostream>

namespace ORB_SLAM3 {

    namespace {
        // The process-global instance. Plain (non-atomic) by design: written
        // exactly once by FixFlags::Set before any SLAM thread exists;
        // thread creation provides the happens-before edge (header contract).
        FixFlags g_fixFlags;

        // Read an optional 0/1 integer key; absent keys leave `dst`
        // untouched (preset value survives), present keys override.
        // Deliberately NOT Settings::readParameter: that helper prints a
        // stderr line for every absent optional key, and level 0 (all keys
        // absent — every gate yaml) must be completely silent.
        void overrideFromKey(const cv::FileStorage& fSettings,
                             const char* name, bool& dst)
        {
            const cv::FileNode node = fSettings[name];
            if (node.empty())
                return;
            if (!node.isInt()) {
                std::cerr << name
                          << " parameter must be an integer (0|1), aborting..."
                          << std::endl;
                exit(-1);
            }
            dst = (static_cast<int>(node) != 0);
        }
    }  // namespace

    const FixFlags& FixFlags::I()
    {
        return g_fixFlags;
    }

    void FixFlags::Set(const FixFlags& flags)
    {
        g_fixFlags = flags;
    }

    FixFlags FixFlags::Parse(const cv::FileStorage& fSettings)
    {
        FixFlags f;  // aggregate defaults: all false = level 0

        // 1) FixLevel preset (absent = 0).
        int level = 0;
        {
            const cv::FileNode node = fSettings["FixLevel"];
            if (!node.empty()) {
                if (!node.isInt()) {
                    std::cerr << "FixLevel parameter must be an integer "
                                 "(0|1|2), aborting..."
                              << std::endl;
                    exit(-1);
                }
                level = static_cast<int>(node);
                if (level < 0 || level > 2) {
                    std::cerr << "FixLevel must be 0, 1 or 2 (got " << level
                              << "), aborting..." << std::endl;
                    exit(-1);
                }
            }
        }
        if (level >= 1) {
            // state-hygiene set
            f.loopStateHygiene = true;
            f.latchHygiene = true;
            f.lcResetWipe = true;
            f.stereoMbInit = true;
            f.rectifiedResizeCal2 = true;
        }
        if (level >= 2) {
            // all remaining (numeric / legacy-semantics) fixes
            f.scaleJacobianChainRule = true;
            f.ldltPoseGraph = true;
            f.legacyImuResetWindow = true;
        }

        // 2) Per-fix overrides (win over the preset in BOTH directions).
        overrideFromKey(fSettings, "Fix.ScaleJacobian", f.scaleJacobianChainRule);
        overrideFromKey(fSettings, "Fix.LoopStateHygiene", f.loopStateHygiene);
        overrideFromKey(fSettings, "Fix.LatchHygiene", f.latchHygiene);
        overrideFromKey(fSettings, "Fix.LcResetWipe", f.lcResetWipe);
        overrideFromKey(fSettings, "Fix.LdltPoseGraph", f.ldltPoseGraph);
        overrideFromKey(fSettings, "Fix.StereoMbInit", f.stereoMbInit);
        overrideFromKey(fSettings, "Fix.LegacyImuResetWindow", f.legacyImuResetWindow);
        overrideFromKey(fSettings, "Fix.RectifiedResizeCal2", f.rectifiedResizeCal2);

        return f;
    }

    std::string FixFlags::ActiveList() const
    {
        std::string s;
        auto add = [&s](bool on, const char* name) {
            if (!on)
                return;
            if (!s.empty())
                s += ' ';
            s += name;
        };
        add(scaleJacobianChainRule, "scaleJacobianChainRule");
        add(loopStateHygiene, "loopStateHygiene");
        add(latchHygiene, "latchHygiene");
        add(lcResetWipe, "lcResetWipe");
        add(ldltPoseGraph, "ldltPoseGraph");
        add(stereoMbInit, "stereoMbInit");
        add(legacyImuResetWindow, "legacyImuResetWindow");
        add(rectifiedResizeCal2, "rectifiedResizeCal2");
        return s;
    }

}  // namespace ORB_SLAM3
