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

#include <sophus/se3.hpp>

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

// P6-4: which distortion the imu_chain fixture bakes into the stored state,
// i.e. which InertialOptimization overload the fixture is built for.
//
//   kScaleGravity — P6-3 pair, InertialOptimization(Map*, Rwg&, scale&).
//                   States 1/s_true-scaled (s_true = 1.1) in a ~10 deg
//                   gravity-tilted world frame; TRUE bias is zero.
//   kFullInit     — P6-4 (a), the 11-argument IMU-init overload. Same
//                   scale/gravity distortion PLUS a known bias injected into
//                   the raw measurements (see below).
//   kBiasOnly     — P6-4 (b), InertialOptimization(Map*, bg, ba, priorG,
//                   priorA). That overload PINS VertexGDir at identity and
//                   VertexScale at 1.0 (Optimizer.cpp:3295-3302), so the
//                   fixture must store the METRIC trajectory (s_true = 1.0)
//                   in an UPRIGHT world frame (g = (0,0,-G)); the only thing
//                   left to recover is the injected bias.
//
// Bias injection (kFullInit / kBiasOnly), design §B 1.2 trick 2 extended:
//   * raw measurements carry the bias:  a_meas = R_wb^T (a_w - g) + ba_true,
//                                       w_meas = w_body + bg_true
//   * the Preintegrated object is constructed with linearization bias
//     b = (ba_true, bg_true), so IntegrateNewMeasurement subtracts exactly
//     the injected bias and dR/dV/dP describe the TRUE (bias-free) motion
//   * KF states are back-derived from those same deltas, so the edge residual
//     at bias = b_true is a single float quantization — NOT a first-order
//     bias-correction truncation. b_true is therefore the exact minimizer of
//     the inertial term, giving an analytic GT for bg/ba.
//   * KF stored bias remains ZERO, which is what VertexGyroBias/VertexAccBias
//     read as their initial estimate — the optimizer starts at 0 and must
//     travel to b_true.
enum class ImuChainVariant {
    kScaleGravity,
    kFullInit,
    kBiasOnly,
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

    ImuChainVariant variant = ImuChainVariant::kScaleGravity;

    // Ground truth the optimizer must recover.
    double sTrue = 0.0;
    Eigen::Vector3d gDirTrue = Eigen::Vector3d::Zero();  // unit, map frame
    Eigen::Vector3d bgTrue = Eigen::Vector3d::Zero();    // rad/s
    Eigen::Vector3d baTrue = Eigen::Vector3d::Zero();    // m/s^2

    // Irreducible EdgeInertialGS residual AT the analytic optimum, measured by
    // the fixture itself in plain double Eigen. It is not zero because
    // KeyFrame::SetPose / SetVelocity store the state as float32: the last
    // rounding of each derived state survives into the residual. The GT gates
    // in compare.py are DERIVED from these numbers (a residual dr in the
    // ev/er block is absorbed by a bias error dr/|J| with |JVa| = |JRg| = dT),
    // which is why they are emitted into the record header.
    double residErMax = 0.0;   // rad
    double residEvMax = 0.0;   // m/s
    double residEpMax = 0.0;   // m
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
// The default arguments keep the P6-3 pair byte-identical.
//
// sTrueOverride (P11-F1): > 0 replaces the variant's default s_true (kSTrue
// for the distorted variants, 1.0 for kBiasOnly). Everything downstream is
// already parameterized on fx.sTrue (state scaling, residual floor), so the
// override needs no other change. Meaningful for kScaleGravity only; used by
// tests/fixlevel/fl9_scale_convergence.cpp to build the s_true = 2.0 fixture
// that sits exactly at the DIVERGENCES #9 stall point — OUTSIDE the solver
// convergence region the harness fixtures must stay in (see the kSTrue
// comment in EquivFixtures.cpp), which is why the equivalence pairs keep the
// default. 0.0 = no override.
ImuChainFixture MakeImuChainFixture(
    ImuChainVariant variant = ImuChainVariant::kScaleGravity,
    double sTrueOverride = 0.0);

// PoseInertialFixture (design §B 1.1 + §B class B) — input for
// Optimizer::PoseInertialOptimizationLastKeyFrame(Frame*, bool).
//
// mono_imu_link layout:
//   * ONE KeyFrame (all four of its vertices are fixed in the optimizer) at
//     t = 0 and ONE Frame at t = 0.1 s, linked by a 200 Hz preintegration
//     (20 samples) of the SAME constant-acceleration / constant-body-rate
//     model the imu_chain fixture uses. GT bias is zero; the frame's bias
//     vertices are tied to the KF's fixed zero bias by EdgeGyroRW/EdgeAccRW
//     whose information is C(9:11,9:11)^-1 / C(12:14,12:14)^-1.
//   * the Frame's GT state is back-derived from the discrete recursion
//     evaluated on the READ-BACK (float-quantized) KeyFrame state, exactly as
//     in MakeImuChainFixture, with s = 1 and g = (0,0,-G) — EdgeInertial
//     hard-codes that gravity (G2oTypes.cpp:498) — so the inertial residual
//     at GT is a single float quantization.
//   * 40 MapPoints on the same 8x5 staggered-depth grid as mono_grid,
//     projected exactly through the Frame's GT camera pose, of which
//     kPoseInertialOutliers are displaced by (+25, -20) px.
//     Margin rule (design §B 1.1): mTrackDepth is the true 3.0-3.6 m depth,
//     so every observation takes the bClose branch and the active thresholds
//     are chi2close = 1.5*{12, 7.5, 5.991, 5.991} = {18, 11.25, 8.99, 8.99}.
//     Designed inlier chi2 ~ 0, designed outlier chi2 >= 1025/1.2^14 ~ 79.8:
//     no edge lands within a decade of any of the four round thresholds, so
//     the 4-round inlier reclassification is a deterministic discrete
//     contract in both harness variants.
//   * 37 designed inliers >= 30, so the "recover not too bad points" salvage
//     branch (Optimizer.cpp:4800) is deterministically NOT taken and the
//     return value is nInitialCorrespondences - nBad = 40 - 3 = 37.
//   * initial Frame state = GT perturbed by ~1 deg / 3 cm / 5 cm/s, i.e. a
//     tracking-realistic IMU prediction error.
//   * mImuCalib is assigned BEFORE SetPose on the KeyFrame (KeyFrame::SetPose
//     only computes mOwb when mImuCalib.mbIsSet — design §B fixture trap) and
//     before SetImuPoseVelocity on the Frame (which reads mImuCalib.mTcb to
//     build mTcw).
//
// NOTE: PoseInertialOptimizationLastKeyFrame assigns a freshly allocated
// ConstraintPoseImu to frame.mpcpi and never frees the previous one. The
// fixture leaves mpcpi null; the CALLER owns the object the optimizer stores
// there (EquivMain deletes it after serializing the Hessian).
struct PoseInertialFixture {
    ORB_SLAM3::Frame frame;

    std::unique_ptr<ORB_SLAM3::GeometricCamera> camera;
    std::vector<std::unique_ptr<ORB_SLAM3::MapPoint>> mapPoints;
    std::unique_ptr<KfChainBlock> kfBlock;   // exactly one KeyFrame
    std::unique_ptr<ORB_SLAM3::IMU::Preintegrated> preint;

    int expectedInliers = 0;
    std::vector<int> outlierIndices;

    // Ground truth the optimizer must recover (analytic, see above).
    Sophus::SE3d TcwGT;                                  // camera pose
    Eigen::Vector3d velGT = Eigen::Vector3d::Zero();     // world velocity

    ORB_SLAM3::KeyFrame* kf() const { return kfBlock->at(0); }

    PoseInertialFixture() = default;
    PoseInertialFixture(const PoseInertialFixture&) = delete;
    PoseInertialFixture& operator=(const PoseInertialFixture&) = delete;
    PoseInertialFixture(PoseInertialFixture&&) = default;
    PoseInertialFixture& operator=(PoseInertialFixture&&) = default;
};

// Number of deliberate visual outliers in mono_imu_link (of 40 points).
constexpr int kPoseInertialOutliers = 3;

// Builds the deterministic mono_imu_link fixture described above.
PoseInertialFixture MakePoseInertialFixture();

}  // namespace equiv

#endif  // EQUIV_FIXTURES_HPP
