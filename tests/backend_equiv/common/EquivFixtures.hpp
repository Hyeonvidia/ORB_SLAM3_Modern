// P6 backend-equivalence harness — synthetic fixtures (docs/P6_DESIGN.md §B).
//
// Fixtures use only map/camera/imu types; they never call g2o APIs directly,
// so the same fixture code compiles identically in the vendored and modern
// variants of the harness.

#ifndef EQUIV_FIXTURES_HPP
#define EQUIV_FIXTURES_HPP

#include "map/Frame.hpp"
#include "map/KeyFrame.hpp"
#include "map/Map.hpp"
#include "map/MapPoint.hpp"
#include "camera/GeometricCamera.hpp"
#include "backend/ImuTypes.hpp"

#include <memory>
#include <new>
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

// KfChainBlock — placement-new KeyFrames into ONE contiguous aligned buffer,
// constructed in mnId order, so ascending address == ascending mnId.
//
// Why: Map stores KeyFrame* in a std::set (pointer-ordered), and
// InertialOptimization iterates Map::GetAllKeyFrames() to insert g2o
// vertices/edges. g2o processes them in insertion (internalId) order, so the
// floating-point summation order — and therefore the last-ULP content of the
// serialized record — would depend on heap addresses if the KFs came from
// individual `new` calls. The contiguous block pins set order to mnId order
// in every run and both harness variants.
class KfChainBlock {
public:
    explicit KfChainBlock(int count) : count_(count) {
        raw_ = ::operator new(sizeof(ORB_SLAM3::KeyFrame) * count_,
                              std::align_val_t(alignof(ORB_SLAM3::KeyFrame)));
        auto* base = static_cast<ORB_SLAM3::KeyFrame*>(raw_);
        for (int i = 0; i < count_; i++)
            new (base + i) ORB_SLAM3::KeyFrame();
    }
    ~KfChainBlock() {
        auto* base = static_cast<ORB_SLAM3::KeyFrame*>(raw_);
        for (int i = count_ - 1; i >= 0; i--)
            (base + i)->~KeyFrame();
        ::operator delete(raw_,
                          std::align_val_t(alignof(ORB_SLAM3::KeyFrame)));
    }
    KfChainBlock(const KfChainBlock&) = delete;
    KfChainBlock& operator=(const KfChainBlock&) = delete;

    ORB_SLAM3::KeyFrame* at(int i) {
        return static_cast<ORB_SLAM3::KeyFrame*>(raw_) + i;
    }
    int count() const { return count_; }

private:
    void* raw_;
    int count_;
};

// ImuKfChainFixture (design §B 1.2, class C) — input for
// Optimizer::InertialOptimization(Map*, Rwg&, scale&) (scale/gravity variant).
//
// imu_chain layout:
//   * 5 KFs (mnId 0..4) at 0.5 s spacing, 200 Hz synthetic IMU
//   * constant-ACCELERATION trajectory a_world = (0.3, 0, 0.9) m/s² — constant
//     velocity would make scale unobservable (design §B 1.2 trick 1). The
//     vertical (z) component is mandatory: a first-order gravity-direction
//     tilt Δg spans exactly the horizontal plane, so a purely horizontal
//     constant acceleration makes the scale column of the Jacobian parallel
//     to a tilt column (both are [c·dt; c·dt²/2] per edge with parallel
//     constant world vectors c) — a singular Hessian. Vertical excitation is
//     orthogonal to every tilt direction and pins the scale.
//   * small constant body rate (0.02, -0.015, 0.01) rad/s; a_meas =
//     R_wb^T (a_world - g_world), g_world = Rwg_perturb * (0,0,-9.81)
//   * GT defined by the discrete preintegration recursion ITSELF (trick 2):
//     measurements go through IMU::Preintegrated::IntegrateNewMeasurement
//     (the shared live float path), and KF k+1 state is back-derived as
//        R2 = R1·ΔR,  v2 = v1 + (g dt + R1·ΔV)/s,
//        p2 = p1 + v1 dt + (g dt²/2 + R1·ΔP)/s
//     — the already-1/s-scaled recursion, evaluated on the READ-BACK
//     (float-quantized) predecessor state via the exact accessors the edge
//     uses, so each edge residual at GT is a single float quantization
//     (~1e-7), not an accumulated error.
//   * KF states are stored 1/s_true-scaled (s_true = 1.1) in a world frame
//     whose gravity is tilted ~10° (Rwg_perturb) — the optimizer, started
//     from Rwg = I / scale = 1, must recover (s_true, gravity direction).
//     s_true = 1.1 (not the design's example 2.0) is forced by an upstream
//     ORB-SLAM3 solver quirk found during P6-3: EdgeInertialGS provides an
//     ADDITIVE scale Jacobian (d r/d s) while VertexScale applies a
//     MULTIPLICATIVE update (s*exp(u)), making the GN iteration
//     s <- s*exp(s_true - s), with multiplier -(s_true - 1) at the optimum.
//     It diverges for s_true > 2, oscillates without contracting at exactly
//     s_true = 2.0 (measured: stall at 2.3% error, creep ~4e-4 per extra
//     10-iteration call), and contracts fast only for s_true near 1 — which
//     also matches production usage (LocalMapping::ScaleRefinement calls
//     this variant with a nearly-correct map). A non-contracting fixture
//     would additionally let solver-path differences between the two g2o
//     builds survive to the output, defeating the 1e-9 cross-variant gate
//     (design §B.5).
//   * EuRoC-class noise (1.7e-4, 2e-3, 1.9e-5, 3e-3) with the sqrt(freq)
//     convention (Tracking's Calib construction) so EdgeInertialGS's
//     information inversion is well-conditioned.
//   * mImuCalib is assigned BEFORE SetPose (KeyFrame::SetPose only computes
//     mOwb when mImuCalib.mbIsSet — design §B fixture trap).
struct ImuChainFixture {
    static constexpr int kNumKFs = 5;

    std::unique_ptr<ORB_SLAM3::Map> map;
    std::unique_ptr<ORB_SLAM3::GeometricCamera> camera;
    // preints[k] covers interval (k-1 -> k); preints[0] is null.
    std::vector<std::unique_ptr<ORB_SLAM3::IMU::Preintegrated>> preints;
    std::unique_ptr<KfChainBlock> kfBlock;

    // Ground truth the optimizer must recover.
    double sTrue = 0.0;
    Eigen::Vector3d gDirTrue = Eigen::Vector3d::Zero();  // unit, map frame
    // Debug affordances (EQUIV_INERTIAL_DEBUG path in EquivMain): the full
    // perturbation rotation and gravity vector behind gDirTrue.
    Eigen::Matrix3d RwgTrue = Eigen::Matrix3d::Identity();
    Eigen::Vector3d gWorldTrue = Eigen::Vector3d::Zero();

    ORB_SLAM3::KeyFrame* kf(int i) const { return kfBlock->at(i); }

    ImuChainFixture() = default;
    ImuChainFixture(const ImuChainFixture&) = delete;
    ImuChainFixture& operator=(const ImuChainFixture&) = delete;
    ImuChainFixture(ImuChainFixture&&) = default;
    ImuChainFixture& operator=(ImuChainFixture&&) = default;
};

// Builds the deterministic imu_chain fixture described above. Every call
// constructs a fully independent fixture (fresh Map/KeyFrames/Preintegrated/
// camera) so two invocations exercise the self-determinism gate end to end.
ImuChainFixture MakeImuChainFixture();

}  // namespace equiv

#endif  // EQUIV_FIXTURES_HPP
