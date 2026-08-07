// P6 backend-equivalence harness — synthetic fixtures (docs/P6_DESIGN.md §B).
//
// Fixtures use only map/camera/imu types; they never call g2o APIs directly,
// so the same fixture code compiles identically in the vendored and modern
// variants of the harness.

#ifndef EQUIV_FIXTURES_HPP
#define EQUIV_FIXTURES_HPP

#include "map/Frame.hpp"
#include "map/MapPoint.hpp"
#include "camera/GeometricCamera.hpp"

#include <memory>
#include <vector>

namespace equiv {

// FrameFixture (design §B 1.1, class A) — input for Optimizer::PoseOptimization.
//
// mono_grid layout:
//   * 40 MapPoints on an 8x5 grid (depth staggered 3.0..3.6 m in camera frame)
//   * ground-truth pose TcwGT; observations are exact pinhole projections
//     (EuRoC-like intrinsics) -> inlier chi2 ~ 0 at the optimum
//   * 4 deliberate outliers (indices kOutlierIndices): observation offset by
//     (+25, -20) px -> chi2 >= 1025 / 1.2^14 ~ 79.8 even at octave 7,
//     comfortably above the design margin (outliers > 30, inliers < 0.5;
//     nothing near the 5.991 mono threshold)
//   * monocular: mpCamera2 = nullptr, mvuRight all -1, Nleft = -1
//   * 8-level pyramid mvInvLevelSigma2 = 1.2^{-2k}, octave = i % 8
//   * initial pose = TcwGT perturbed by ~2 deg rotation, ~5 cm translation
struct FrameFixture {
    ORB_SLAM3::Frame frame;

    // Owned resources (Frame stores raw pointers only).
    std::unique_ptr<ORB_SLAM3::GeometricCamera> camera;
    std::vector<std::unique_ptr<ORB_SLAM3::MapPoint>> mapPoints;

    // Design bookkeeping (asserted by the runner / run_equiv.sh).
    int expectedInliers = 0;                 // 36 for mono_grid
    std::vector<int> outlierIndices;         // designed outlier indices

    FrameFixture() = default;
    FrameFixture(const FrameFixture&) = delete;
    FrameFixture& operator=(const FrameFixture&) = delete;
    FrameFixture(FrameFixture&&) = default;
    FrameFixture& operator=(FrameFixture&&) = default;
};

// Builds the deterministic mono_grid FrameFixture described above.
// Every call constructs a fully independent fixture (fresh MapPoints,
// fresh camera, fresh Frame) so two invocations exercise the
// self-determinism gate end to end.
FrameFixture MakeMonoGridFixture();

}  // namespace equiv

#endif  // EQUIV_FIXTURES_HPP
