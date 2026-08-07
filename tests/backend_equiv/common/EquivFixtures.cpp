// P6 backend-equivalence harness — fixture construction (docs/P6_DESIGN.md §B).

#include "EquivFixtures.hpp"

#include "camera/Pinhole.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

#include <cmath>

namespace equiv {

namespace {

// EuRoC MH-like pinhole intrinsics (cam0).
constexpr double kFx = 458.654;
constexpr double kFy = 457.296;
constexpr double kCx = 367.215;
constexpr double kCy = 248.375;

constexpr int kGridCols = 8;
constexpr int kGridRows = 5;
constexpr int kNumPoints = kGridCols * kGridRows;  // 40
constexpr int kNumLevels = 8;
constexpr float kScaleFactor = 1.2f;

// 4 deliberate outliers (design: 36 of 40 inliers).
constexpr int kOutlierIndices[4] = {5, 13, 21, 34};

// Pixel offset applied to outlier observations. chi2 = invSigma2 * (25^2+20^2)
// = 1025 * 1.2^{-2*octave}; worst case (octave 7) ~ 79.8 >> 30.
constexpr double kOutlierDu = 25.0;
constexpr double kOutlierDv = -20.0;

bool IsOutlierIndex(int i)
{
    for (int idx : kOutlierIndices)
        if (idx == i) return true;
    return false;
}

}  // namespace

FrameFixture MakeMonoGridFixture()
{
    FrameFixture fx;
    ORB_SLAM3::Frame& F = fx.frame;  // default ctor + manual field population

    // ---- ground-truth pose (world -> camera), deliberately non-identity ----
    const Eigen::Vector3d axisGT = Eigen::Vector3d(0.3, -0.2, 0.5).normalized();
    const Eigen::AngleAxisd aaGT(0.08, axisGT);  // ~4.6 deg
    const Sophus::SE3d TcwGT(Eigen::Quaterniond(aaGT),
                             Eigen::Vector3d(0.05, -0.03, 0.20));
    const Sophus::SE3d TwcGT = TcwGT.inverse();

    // ---- camera ----------------------------------------------------------
    fx.camera.reset(new ORB_SLAM3::Pinhole(
        {static_cast<float>(kFx), static_cast<float>(kFy),
         static_cast<float>(kCx), static_cast<float>(kCy)}));

    // ---- Frame fields touched by PoseOptimization (mono path) ------------
    F.mnId = 1;
    F.N = kNumPoints;
    F.mpCamera = fx.camera.get();
    F.mpCamera2 = nullptr;   // conventional (non-fisheye-rig) path
    F.Nleft = -1;            // monocular
    F.Nright = -1;

    // Calibration mirror fields (stereo path only in PoseOptimization, but
    // populated for completeness / future fixtures).
    F.fx = static_cast<float>(kFx);
    F.fy = static_cast<float>(kFy);
    F.cx = static_cast<float>(kCx);
    F.cy = static_cast<float>(kCy);
    F.invfx = 1.0f / F.fx;
    F.invfy = 1.0f / F.fy;
    F.mbf = 0.0f;
    F.mb = 0.0f;
    F.mThDepth = 35.0f;

    // ---- 8-level scale pyramid -------------------------------------------
    F.mnScaleLevels = kNumLevels;
    F.mfScaleFactor = kScaleFactor;
    F.mfLogScaleFactor = std::log(kScaleFactor);
    F.mvScaleFactors.resize(kNumLevels);
    F.mvInvScaleFactors.resize(kNumLevels);
    F.mvLevelSigma2.resize(kNumLevels);
    F.mvInvLevelSigma2.resize(kNumLevels);
    for (int k = 0; k < kNumLevels; k++) {
        F.mvScaleFactors[k] = std::pow(kScaleFactor, static_cast<float>(k));
        F.mvInvScaleFactors[k] = 1.0f / F.mvScaleFactors[k];
        F.mvLevelSigma2[k] = F.mvScaleFactors[k] * F.mvScaleFactors[k];
        F.mvInvLevelSigma2[k] = 1.0f / F.mvLevelSigma2[k];
    }

    // ---- 3D grid of MapPoints + exact projections ------------------------
    F.mvpMapPoints.assign(kNumPoints, nullptr);
    F.mvKeysUn.resize(kNumPoints);
    F.mvuRight.assign(kNumPoints, -1.0f);   // all monocular
    F.mvDepth.assign(kNumPoints, -1.0f);
    F.mvbOutlier.assign(kNumPoints, false);
    fx.mapPoints.reserve(kNumPoints);

    for (int i = 0; i < kNumPoints; i++) {
        const int col = i % kGridCols;
        const int row = i / kGridCols;

        // Point in CAMERA(GT) coordinates: x in [-1.4, 1.4], y in [-0.9, 0.9],
        // depth staggered 3.0 .. 3.6 m for conditioning.
        const double x = -1.4 + 2.8 * col / (kGridCols - 1);
        const double y = -0.9 + 1.8 * row / (kGridRows - 1);
        const double z = 3.0 + 0.15 * ((i * 7) % 5);
        const Eigen::Vector3d Pc(x, y, z);
        const Eigen::Vector3d Pw = TwcGT * Pc;

        std::unique_ptr<ORB_SLAM3::MapPoint> pMP(new ORB_SLAM3::MapPoint());
        pMP->SetWorldPos(Pw.cast<float>());
        F.mvpMapPoints[i] = pMP.get();
        fx.mapPoints.push_back(std::move(pMP));

        // Exact pinhole projection of the GT geometry (inlier chi2 ~ 0).
        double u = kFx * Pc.x() / Pc.z() + kCx;
        double v = kFy * Pc.y() / Pc.z() + kCy;
        if (IsOutlierIndex(i)) {
            u += kOutlierDu;
            v += kOutlierDv;
        }

        cv::KeyPoint kp;
        kp.pt = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
        kp.octave = i % kNumLevels;
        kp.size = 31.0f * F.mvScaleFactors[kp.octave];
        kp.angle = 0.0f;
        kp.response = 0.0f;
        kp.class_id = -1;
        F.mvKeysUn[i] = kp;
    }
    F.mvKeys = F.mvKeysUn;  // mono path reads mvKeysUn only; keep consistent

    // ---- perturbed initial pose (~2 deg rotation, ~5 cm translation) -----
    const Eigen::Vector3d axisP = Eigen::Vector3d(0.2, 0.5, -0.3).normalized();
    const Eigen::AngleAxisd aaP(0.035, axisP);  // ~2.0 deg
    const Sophus::SE3d dT(Eigen::Quaterniond(aaP),
                          Eigen::Vector3d(0.03, -0.02, 0.04));
    const Sophus::SE3d Tcw0 = dT * TcwGT;
    F.SetPose(Tcw0.cast<float>());

    fx.expectedInliers = kNumPoints - 4;  // 36
    fx.outlierIndices.assign(kOutlierIndices, kOutlierIndices + 4);
    return fx;
}

// ===========================================================================
// imu_chain — ImuKfChainFixture (design §B 1.2, class C)
// ===========================================================================

namespace {

// IMU / trajectory constants (see EquivFixtures.hpp for the rationale).
constexpr double kKfSpacing = 0.5;                  // s between KFs
constexpr int kImuHz = 200;
constexpr int kSamplesPerInterval = 100;            // 0.5 s * 200 Hz
constexpr float kImuSampleDt = 1.0f / kImuHz;
// s_true must lie in the SOLVER's convergence region (see the P6-3 finding
// documented in EquivFixtures.hpp): upstream ORB-SLAM3 pairs an ADDITIVE
// scale Jacobian (EdgeInertialGS::linearizeOplus, d r/d s) with a
// MULTIPLICATIVE vertex update (VertexScale::oplusImpl, s*exp(u)), so the
// GN fixed-point iteration is s <- s*exp(s_true - s) with multiplier
// -(s_true - 1) at the optimum: it contracts only for 0 < s_true < 2 and is
// marginally stable (pure oscillation) at exactly 2.0. s_true = 1.1 gives a
// 0.1x contraction per iteration -> machine precision within the function's
// fixed 10 GN iterations.
constexpr double kSTrue = 1.1;
constexpr double kGravityTiltRad = 10.0 * M_PI / 180.0;  // ~10 deg

}  // namespace

ImuChainFixture MakeImuChainFixture()
{
    using ORB_SLAM3::IMU::Bias;
    using ORB_SLAM3::IMU::Calib;
    using ORB_SLAM3::IMU::Preintegrated;

    ImuChainFixture fx;
    fx.sTrue = kSTrue;

    // ---- IMU calibration --------------------------------------------------
    // Non-trivial Tbc so the Tcw <-> Twb conversion path (GetImuRotation /
    // GetImuPosition / mOwb) is exercised with non-identity values.
    const Eigen::Vector3f axisTbc = Eigen::Vector3f(0.2f, -0.1f, 0.3f).normalized();
    const Sophus::SE3f Tbc(Eigen::Quaternionf(Eigen::AngleAxisf(0.3f, axisTbc)),
                           Eigen::Vector3f(0.05f, -0.02f, 0.03f));
    // EuRoC-class continuous-time densities, discretized exactly as
    // Tracking::ParseIMUParamFile does: noise * sqrt(freq), walk / sqrt(freq).
    const float sf = std::sqrt(static_cast<float>(kImuHz));
    const Calib calib(Tbc, 1.7e-4f * sf, 2e-3f * sf, 1.9e-5f / sf, 3e-3f / sf);

    // ---- ground truth: gravity direction + scale --------------------------
    // Map-frame gravity is the canonical gI = (0,0,-G) tilted by ~10 deg
    // about a horizontal axis; the optimizer must recover this direction.
    // G must be the same float->double value EdgeInertialGS uses.
    const double G = static_cast<double>(ORB_SLAM3::IMU::GRAVITY_VALUE);
    const Eigen::Vector3d tiltAxis = Eigen::Vector3d(0.6, 0.8, 0.0).normalized();
    const Eigen::Matrix3d RwgPerturb =
        Eigen::AngleAxisd(kGravityTiltRad, tiltAxis).toRotationMatrix();
    const Eigen::Vector3d gWorld = RwgPerturb * Eigen::Vector3d(0.0, 0.0, -G);
    fx.gDirTrue = gWorld.normalized();
    fx.RwgTrue = RwgPerturb;
    fx.gWorldTrue = gWorld;

    // ---- true (metric) trajectory -----------------------------------------
    // Constant world acceleration with a mandatory vertical component (see
    // header: horizontal-only excitation makes scale/tilt degenerate) and a
    // small constant body rate. v0 centers the velocities around zero to
    // minimize float-quantization noise in the stored states.
    const Eigen::Vector3d aWorld(0.3, 0.0, 0.9);
    const Eigen::Vector3d omegaBody(0.02, -0.015, 0.01);
    const double totalT = (ImuChainFixture::kNumKFs - 1) * kKfSpacing;  // 2 s
    const Eigen::Vector3d v0 = -aWorld * (totalT / 2.0);
    const Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();

    // ---- preintegrate each interval through the SHARED float path ---------
    fx.preints.resize(ImuChainFixture::kNumKFs);  // [0] stays null
    for (int k = 1; k < ImuChainFixture::kNumKFs; k++) {
        auto pre = std::make_unique<Preintegrated>(Bias(), calib);
        for (int j = 0; j < kSamplesPerInterval; j++) {
            // Midpoint sampling of the continuous model; the recursion below
            // defines GT from whatever these measurements integrate to, so
            // the sampling rule only affects realism, not consistency.
            const double tMid =
                (k - 1) * kKfSpacing + (j + 0.5) / static_cast<double>(kImuHz);
            const Eigen::Matrix3d Rwb =
                R0 * Sophus::SO3d::exp(omegaBody * tMid).matrix();
            const Eigen::Vector3d aMeas = Rwb.transpose() * (aWorld - gWorld);
            pre->IntegrateNewMeasurement(aMeas.cast<float>(),
                                         omegaBody.cast<float>(),
                                         kImuSampleDt);
        }
        fx.preints[k] = std::move(pre);
    }

    // ---- KFs: contiguous block (deterministic std::set order) -------------
    fx.camera.reset(new ORB_SLAM3::Pinhole(
        {458.654f, 457.296f, 367.215f, 248.375f}));
    fx.kfBlock.reset(new KfChainBlock(ImuChainFixture::kNumKFs));

    // KeyFrame convention: GetImuPose() = Twc * mTcb (mTcb = mTbc.inverse()),
    // i.e. Twb = Twc * Tcb  =>  Tcw = Tcb * Twb^{-1}  (NOT Tbc * Twb^{-1} —
    // the classic Tcw/Twb trap called out in design §B).
    const Sophus::SE3d TcbD = Tbc.cast<double>().inverse();
    auto setupKF = [&](int k, const Eigen::Matrix3d& Rwb,
                       const Eigen::Vector3d& pwb, const Eigen::Vector3d& vw) {
        ORB_SLAM3::KeyFrame* pKF = fx.kfBlock->at(k);
        pKF->mImuCalib = calib;  // BEFORE SetPose: mOwb needs mbIsSet
        pKF->mnId = static_cast<unsigned long>(k);
        pKF->bImu = true;
        pKF->mpCamera = fx.camera.get();
        pKF->mpCamera2 = nullptr;  // default ctor leaves it uninitialized
        pKF->mPrevKF = (k == 0) ? nullptr : fx.kfBlock->at(k - 1);
        pKF->mNextKF = nullptr;
        if (k > 0)
            fx.kfBlock->at(k - 1)->mNextKF = pKF;
        pKF->mpImuPreintegrated = (k == 0) ? nullptr : fx.preints[k].get();

        Eigen::Quaterniond qwb(Rwb);
        qwb.normalize();
        const Sophus::SE3d Twb(qwb, pwb);
        const Sophus::SE3d Tcw = TcbD * Twb.inverse();  // Tcw = Tcb * Tbw
        pKF->SetPose(Tcw.cast<float>());
        pKF->SetVelocity(vw.cast<float>());
        pKF->SetNewBias(Bias());  // GT biases are zero
    };

    // Stored (distorted) chain: states are 1/s_true-scaled, gravity-tilted.
    // KF0 from the analytic initial conditions; KF k>=1 from the discrete
    // recursion evaluated on the READ-BACK (float-quantized) predecessor
    // state via the exact accessors EdgeInertialGS/VertexPose use, so each
    // edge residual at (s_true, gDirTrue) is a single float quantization.
    setupKF(0, R0, p0 / kSTrue, v0 / kSTrue);
    for (int k = 1; k < ImuChainFixture::kNumKFs; k++) {
        ORB_SLAM3::KeyFrame* pPrev = fx.kfBlock->at(k - 1);
        const Eigen::Matrix3d R1 = pPrev->GetImuRotation().cast<double>();
        const Eigen::Vector3d t1 = pPrev->GetImuPosition().cast<double>();
        const Eigen::Vector3d v1 = pPrev->GetVelocity().cast<double>();

        Preintegrated* pre = fx.preints[k].get();
        const Bias b0;  // zero — matches the (fixed, zero) bias vertices
        const double dt = static_cast<double>(pre->dT);
        const Eigen::Matrix3d dR = pre->GetDeltaRotation(b0).cast<double>();
        const Eigen::Vector3d dV = pre->GetDeltaVelocity(b0).cast<double>();
        const Eigen::Vector3d dP = pre->GetDeltaPosition(b0).cast<double>();

        // The edge computes R1^T * (...) - dX, so exact zero requires the
        // recursion to apply R1^{-T}, NOT R1: the read-back R1 is a
        // float-quantized rotation whose orthonormality defect is O(1e-7),
        // and R1 * dX vs R1^{-T} * dX differ by defect * |dX| — with
        // |dV| ~ 5 m/s that alone costs ~2e-7 per residual (measured), which
        // pushed the recovered scale just past the 1e-7 GT gate.
        const Eigen::Matrix3d R1invT = R1.transpose().inverse();
        const Eigen::Matrix3d R2 = R1invT * dR;
        const Eigen::Vector3d v2 = v1 + (gWorld * dt + R1invT * dV) / kSTrue;
        const Eigen::Vector3d p2 =
            t1 + v1 * dt + (0.5 * gWorld * dt * dt + R1invT * dP) / kSTrue;
        setupKF(k, R2, p2, v2);
    }

    // ---- Map --------------------------------------------------------------
    fx.map.reset(new ORB_SLAM3::Map(0));
    for (int k = 0; k < ImuChainFixture::kNumKFs; k++)
        fx.map->AddKeyFrame(fx.kfBlock->at(k));

    return fx;
}

}  // namespace equiv
