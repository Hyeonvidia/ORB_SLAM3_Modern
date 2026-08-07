// P6 backend-equivalence harness — CLI runner (docs/P6_DESIGN.md §B).
//
//   equiv_runner <function> <fixture>
//
// Currently: function = pose_optimization, fixture = mono_grid.
//
// The requested function is run TWICE, each time on an independently built
// fixture (fresh Frame/MapPoints/camera), and both full records are printed
// to stdout delimited by BEGIN_RECORD / END_RECORD lines. Identical records
// are the in-binary self-determinism gate (bit_identity pattern, P4);
// run_equiv.sh / compare.py consume the records.
//
// Exit codes: 0 = records identical, 1 = nondeterminism detected, 2 = usage.

#include "EquivFixtures.hpp"
#include "EquivSerialize.hpp"

#include "backend/Optimizer.hpp"
#include "core/Verbose.hpp"

#include <cstdio>
#include <string>

// Verbose::th lives in src/core/System.cpp, which drags Tracking/LoopClosing/
// Viewer into the link — out of scope for the harness. Define it here instead
// (System.cpp is not part of this binary, so no ODR conflict).
ORB_SLAM3::Verbose::eLevel ORB_SLAM3::Verbose::th =
    ORB_SLAM3::Verbose::VERBOSITY_QUIET;

namespace {

std::string RunPoseOptimizationOnce(const std::string& function,
                                    const std::string& fixture)
{
    equiv::FrameFixture fx = equiv::MakeMonoGridFixture();

    const std::string inputDump =
        equiv::SerializePoseOptimizationInput(fx.frame);
    const std::string inputHash = equiv::Sha256Hex(inputDump);

    const int inliers = ORB_SLAM3::Optimizer::PoseOptimization(&fx.frame);

    if (inliers != fx.expectedInliers) {
        std::fprintf(stderr,
                     "WARNING: inlier count %d != designed %d "
                     "(fixture margin violated?)\n",
                     inliers, fx.expectedInliers);
    }

    return equiv::MakePoseOptimizationRecord(function, fixture, inputHash,
                                             inliers, fx.frame);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::fprintf(stderr, "usage: %s <function> <fixture>\n"
                             "  functions: pose_optimization\n"
                             "  fixtures:  mono_grid\n", argv[0]);
        return 2;
    }
    const std::string function = argv[1];
    const std::string fixture = argv[2];

    if (function != "pose_optimization" || fixture != "mono_grid") {
        std::fprintf(stderr, "unknown function/fixture: %s %s\n",
                     function.c_str(), fixture.c_str());
        return 2;
    }

    const std::string rec1 = RunPoseOptimizationOnce(function, fixture);
    const std::string rec2 = RunPoseOptimizationOnce(function, fixture);

    std::printf("BEGIN_RECORD\n%sEND_RECORD\n", rec1.c_str());
    std::printf("BEGIN_RECORD\n%sEND_RECORD\n", rec2.c_str());

    if (rec1 != rec2) {
        std::fprintf(stderr, "SELF-DETERMINISM FAILURE: records differ\n");
        return 1;
    }
    return 0;
}
