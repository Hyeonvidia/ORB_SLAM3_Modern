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

        auto pMP = std::make_shared<ORB_SLAM3::MapPoint>();
        pMP->SetWorldPos(Pw.cast<float>());
        F.mvpMapPoints[i] = pMP;
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

// P6-4 injected bias (kFullInit / kBiasOnly). Small enough to stay inside the
// regime where a metre-scale trajectory is still recovered, large enough that
// the estimate is far outside every gate from its zero starting point:
// |bg| = 2.7e-3 rad/s and |ba| = 3.7e-2 m/s^2 are ~1e5x the 1e-7 GT gate.
constexpr double kBgTrue[3] = {0.002, -0.001, 0.0015};
constexpr double kBaTrue[3] = {0.02, -0.03, 0.01};

}  // namespace

ImuChainFixture MakeImuChainFixture(ImuChainVariant variant,
                                    double sTrueOverride)
{
    using ORB_SLAM3::IMU::Bias;
    using ORB_SLAM3::IMU::Calib;
    using ORB_SLAM3::IMU::Preintegrated;

    ImuChainFixture fx;
    fx.variant = variant;
    // kBiasOnly targets the overload that PINS gravity to identity and scale
    // to 1.0, so its stored trajectory must be the undistorted metric one.
    const bool bDistort = (variant != ImuChainVariant::kBiasOnly);
    const bool bInjectBias = (variant != ImuChainVariant::kScaleGravity);
    // P11-F1: sTrueOverride > 0 replaces the variant default (see the header
    // comment); everything below reads fx.sTrue, so no other change.
    fx.sTrue = (sTrueOverride > 0.0) ? sTrueOverride
                                     : (bDistort ? kSTrue : 1.0);
    if (bInjectBias)
        fx.bgTrue = Eigen::Vector3d(kBgTrue[0], kBgTrue[1], kBgTrue[2]);
    // Accelerometer bias is injected ONLY in kBiasOnly. Measured reason
    // (P6-4): in the 11-argument overload the velocity vertices are FREE, and
    // then (scale, gravity tilt, u0 = s*v0, ba) span a valley whose curvature
    // is below the fixture's own float32 residual floor — with priorA = 1e-3
    // LM parks at s = 1.0746 / ba = (0.046,-0.051,0.026) whose chi2
    // (1.727e-06) is HIGHER than the analytic optimum's (1.396e-06), and eight
    // chained calls do not move it. Raising the body rate 25x (breaking the
    // body-frame-ba vs world-frame-g coupling) did NOT fix it. Upstream itself
    // regularizes this direction: LocalMapping.cpp:186 initializes with
    // priorA = 1e10, i.e. ba is pinned to ZERO for the first IMU init. So the
    // kFullInit fixture uses ba_true = 0, which is exactly consistent with the
    // production prior, and keeps a nonzero analytic GT on bg / scale / gdir.
    // kBiasOnly pins gravity and scale internally, so no such valley exists
    // there and a two-sided nonzero bias GT is recoverable.
    if (variant == ImuChainVariant::kBiasOnly)
        fx.baTrue = Eigen::Vector3d(kBaTrue[0], kBaTrue[1], kBaTrue[2]);
    const Bias bTrue(static_cast<float>(fx.baTrue.x()),
                     static_cast<float>(fx.baTrue.y()),
                     static_cast<float>(fx.baTrue.z()),
                     static_cast<float>(fx.bgTrue.x()),
                     static_cast<float>(fx.bgTrue.y()),
                     static_cast<float>(fx.bgTrue.z()));

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
        bDistort ? Eigen::AngleAxisd(kGravityTiltRad, tiltAxis).toRotationMatrix()
                 : Eigen::Matrix3d::Identity();
    const Eigen::Vector3d gWorld = RwgPerturb * Eigen::Vector3d(0.0, 0.0, -G);
    fx.gDirTrue = gWorld.normalized();
    fx.RwgTrue = RwgPerturb;
    fx.gWorldTrue = gWorld;

    // ---- true (metric) trajectory -----------------------------------------
    // Constant world acceleration with a mandatory vertical component (see
    // header: horizontal-only excitation makes scale/tilt degenerate) and a
    // small constant body rate. v0 centers the velocities around zero to
    // minimize float-quantization noise in the stored states.
    // The trajectory is IDENTICAL in all three variants (maximum fixture
    // reuse); only the stored-state distortion and the injected bias differ.
    const Eigen::Vector3d aWorld(0.3, 0.0, 0.9);
    const Eigen::Vector3d omegaBody(0.02, -0.015, 0.01);
    const double totalT = (ImuChainFixture::kNumKFs - 1) * kKfSpacing;  // 2 s
    const Eigen::Vector3d v0 = -aWorld * (totalT / 2.0);
    const Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    const Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();

    // ---- preintegrate each interval through the SHARED float path ---------
    // The measurements carry bTrue and the Preintegrated linearization bias IS
    // bTrue, so IntegrateNewMeasurement subtracts it exactly: dR/dV/dP describe
    // the bias-free motion and GetDelta*(bTrue) needs no first-order
    // correction (dbg = dba = 0). See the header for why that matters.
    fx.preints.resize(ImuChainFixture::kNumKFs);  // [0] stays null
    for (int k = 1; k < ImuChainFixture::kNumKFs; k++) {
        auto pre = std::make_unique<Preintegrated>(bTrue, calib);
        for (int j = 0; j < kSamplesPerInterval; j++) {
            // Midpoint sampling of the continuous model; the recursion below
            // defines GT from whatever these measurements integrate to, so
            // the sampling rule only affects realism, not consistency.
            const double tMid =
                (k - 1) * kKfSpacing + (j + 0.5) / static_cast<double>(kImuHz);
            const Eigen::Matrix3d Rwb =
                R0 * Sophus::SO3d::exp(omegaBody * tMid).matrix();
            const Eigen::Vector3d aMeas =
                Rwb.transpose() * (aWorld - gWorld) + fx.baTrue;
            const Eigen::Vector3d wMeas = omegaBody + fx.bgTrue;
            pre->IntegrateNewMeasurement(aMeas.cast<float>(),
                                         wMeas.cast<float>(),
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
    setupKF(0, R0, p0 / fx.sTrue, v0 / fx.sTrue);
    for (int k = 1; k < ImuChainFixture::kNumKFs; k++) {
        ORB_SLAM3::KeyFrame* pPrev = fx.kfBlock->at(k - 1);
        const Eigen::Matrix3d R1 = pPrev->GetImuRotation().cast<double>();
        const Eigen::Vector3d t1 = pPrev->GetImuPosition().cast<double>();
        const Eigen::Vector3d v1 = pPrev->GetVelocity().cast<double>();

        Preintegrated* pre = fx.preints[k].get();
        // GT bias: the deltas are read back at the SAME bias the edge will
        // see at the analytic optimum (zero for kScaleGravity, the injected
        // bias otherwise), so dbg = dba = 0 and no first-order correction is
        // involved on either side of the residual.
        const double dt = static_cast<double>(pre->dT);
        const Eigen::Matrix3d dR = pre->GetDeltaRotation(bTrue).cast<double>();
        const Eigen::Vector3d dV = pre->GetDeltaVelocity(bTrue).cast<double>();
        const Eigen::Vector3d dP = pre->GetDeltaPosition(bTrue).cast<double>();

        // The edge computes R1^T * (...) - dX, so exact zero requires the
        // recursion to apply R1^{-T}, NOT R1: the read-back R1 is a
        // float-quantized rotation whose orthonormality defect is O(1e-7),
        // and R1 * dX vs R1^{-T} * dX differ by defect * |dX| — with
        // |dV| ~ 5 m/s that alone costs ~2e-7 per residual (measured), which
        // pushed the recovered scale just past the 1e-7 GT gate.
        const Eigen::Matrix3d R1invT = R1.transpose().inverse();
        const Eigen::Matrix3d R2 = R1invT * dR;
        const Eigen::Vector3d v2 = v1 + (gWorld * dt + R1invT * dV) / fx.sTrue;
        const Eigen::Vector3d p2 =
            t1 + v1 * dt + (0.5 * gWorld * dt * dt + R1invT * dP) / fx.sTrue;
        setupKF(k, R2, p2, v2);
    }

    // ---- measured residual at the analytic optimum ------------------------
    // Same algebra as EdgeInertialGS::computeError (G2oTypes.cpp:635-637),
    // evaluated in double at (sTrue, RwgTrue, bTrue). Everything that is left
    // is the float32 storage rounding of the KF states; compare.py turns it
    // into the GT tolerances.
    for (int k = 1; k < ImuChainFixture::kNumKFs; k++) {
        ORB_SLAM3::KeyFrame* p1 = fx.kfBlock->at(k - 1);
        ORB_SLAM3::KeyFrame* p2 = fx.kfBlock->at(k);
        Preintegrated* pre = fx.preints[k].get();
        const double dt = static_cast<double>(pre->dT);
        const Eigen::Matrix3d R1 = p1->GetImuRotation().cast<double>();
        const Eigen::Vector3d t1 = p1->GetImuPosition().cast<double>();
        const Eigen::Vector3d v1 = p1->GetVelocity().cast<double>();
        const Eigen::Matrix3d R2 = p2->GetImuRotation().cast<double>();
        const Eigen::Vector3d t2 = p2->GetImuPosition().cast<double>();
        const Eigen::Vector3d v2 = p2->GetVelocity().cast<double>();
        const Eigen::Matrix3d dR = pre->GetDeltaRotation(bTrue).cast<double>();
        const Eigen::Vector3d dV = pre->GetDeltaVelocity(bTrue).cast<double>();
        const Eigen::Vector3d dP = pre->GetDeltaPosition(bTrue).cast<double>();

        Eigen::Quaterniond qe(dR.transpose() * R1.transpose() * R2);
        qe.normalize();
        const double er = std::abs(Eigen::AngleAxisd(qe).angle());
        const Eigen::Vector3d ev =
            R1.transpose() * (fx.sTrue * (v2 - v1) - gWorld * dt) - dV;
        const Eigen::Vector3d ep =
            R1.transpose() *
                (fx.sTrue * (t2 - t1 - v1 * dt) - gWorld * dt * dt / 2.0) - dP;
        fx.residErMax = std::max(fx.residErMax, er);
        fx.residEvMax = std::max(fx.residEvMax, ev.norm());
        fx.residEpMax = std::max(fx.residEpMax, ep.norm());
    }

    // ---- Map --------------------------------------------------------------
    fx.map.reset(new ORB_SLAM3::Map(0));
    for (int k = 0; k < ImuChainFixture::kNumKFs; k++)
        fx.map->AddKeyFrame(fx.kfBlock->at(k));

    return fx;
}

// ===========================================================================
// mono_imu_link — PoseInertialFixture (design §B class B)
// ===========================================================================

namespace {

// 3 deliberate visual outliers of 40 (37 designed inliers >= 30 keeps the
// salvage branch at Optimizer.cpp:4800 out of the picture).
constexpr int kPiOutlierIndices[kPoseInertialOutliers] = {7, 19, 31};

// KF -> Frame link: 0.1 s at 200 Hz.
constexpr double kPiLinkDt = 0.1;
constexpr int kPiSamples = 20;

bool IsPiOutlierIndex(int i)
{
    for (int idx : kPiOutlierIndices)
        if (idx == i) return true;
    return false;
}

}  // namespace

PoseInertialFixture MakePoseInertialFixture()
{
    using ORB_SLAM3::IMU::Bias;
    using ORB_SLAM3::IMU::Calib;
    using ORB_SLAM3::IMU::Preintegrated;

    PoseInertialFixture fx;
    ORB_SLAM3::Frame& F = fx.frame;

    // ---- shared camera + IMU calibration ----------------------------------
    fx.camera.reset(new ORB_SLAM3::Pinhole(
        {static_cast<float>(kFx), static_cast<float>(kFy),
         static_cast<float>(kCx), static_cast<float>(kCy)}));

    const Eigen::Vector3f axisTbc = Eigen::Vector3f(0.2f, -0.1f, 0.3f).normalized();
    const Sophus::SE3f Tbc(Eigen::Quaternionf(Eigen::AngleAxisf(0.3f, axisTbc)),
                           Eigen::Vector3f(0.05f, -0.02f, 0.03f));
    const float sf = std::sqrt(static_cast<float>(kImuHz));
    const Calib calib(Tbc, 1.7e-4f * sf, 2e-3f * sf, 1.9e-5f / sf, 3e-3f / sf);
    const Sophus::SE3d TcbD = Tbc.cast<double>().inverse();

    // ---- true motion over the link ----------------------------------------
    // EdgeInertial hard-codes g = (0,0,-GRAVITY_VALUE) (G2oTypes.cpp:498), so
    // this fixture lives in the upright metric world frame — no scale/gravity
    // distortion, unlike imu_chain.
    const double G = static_cast<double>(ORB_SLAM3::IMU::GRAVITY_VALUE);
    const Eigen::Vector3d gWorld(0.0, 0.0, -G);
    const Eigen::Vector3d aWorld(0.3, 0.0, 0.9);
    const Eigen::Vector3d omegaBody(0.02, -0.015, 0.01);

    const Eigen::Vector3d axisKf = Eigen::Vector3d(0.1, 0.3, -0.2).normalized();
    const Eigen::Matrix3d Rwb1 =
        Eigen::AngleAxisd(0.12, axisKf).toRotationMatrix();
    const Eigen::Vector3d pwb1(0.20, -0.10, 0.05);
    const Eigen::Vector3d vwb1(0.40, -0.20, 0.10);

    // ---- KeyFrame (all four of its vertices are fixed in the optimizer) ----
    fx.kfBlock.reset(new KfChainBlock(1));
    ORB_SLAM3::KeyFrame* pKF = fx.kfBlock->at(0);
    pKF->mImuCalib = calib;   // BEFORE SetPose: mOwb needs mbIsSet
    pKF->mnId = 0;
    pKF->bImu = true;
    pKF->mpCamera = fx.camera.get();
    pKF->mpCamera2 = nullptr;
    pKF->mPrevKF = nullptr;
    pKF->mNextKF = nullptr;
    pKF->mpImuPreintegrated = nullptr;
    // KeyFrame::mbf is const and the default ctor already zeroes it (mono).
    {
        Eigen::Quaterniond q1(Rwb1);
        q1.normalize();
        const Sophus::SE3d Twb1(q1, pwb1);
        pKF->SetPose((TcbD * Twb1.inverse()).cast<float>());  // Tcw = Tcb * Tbw
    }
    pKF->SetVelocity(vwb1.cast<float>());
    pKF->SetNewBias(Bias());   // GT bias is zero on both ends of the link

    // ---- preintegrate the link through the SHARED float path ---------------
    fx.preint = std::make_unique<Preintegrated>(Bias(), calib);
    for (int j = 0; j < kPiSamples; j++) {
        const double tMid = (j + 0.5) / static_cast<double>(kImuHz);
        const Eigen::Matrix3d Rwb =
            Rwb1 * Sophus::SO3d::exp(omegaBody * tMid).matrix();
        const Eigen::Vector3d aMeas = Rwb.transpose() * (aWorld - gWorld);
        fx.preint->IntegrateNewMeasurement(aMeas.cast<float>(),
                                           omegaBody.cast<float>(),
                                           kImuSampleDt);
    }

    // ---- Frame GT state from the discrete recursion ------------------------
    // Evaluated on the READ-BACK (float-quantized) KeyFrame state through the
    // exact accessors VertexPose/VertexVelocity use, with R1^{-T} rather than
    // R1 for the same reason as imu_chain (float orthonormality defect).
    const Eigen::Matrix3d R1 = pKF->GetImuRotation().cast<double>();
    const Eigen::Vector3d t1 = pKF->GetImuPosition().cast<double>();
    const Eigen::Vector3d v1 = pKF->GetVelocity().cast<double>();
    const double dt = static_cast<double>(fx.preint->dT);
    const Bias b0;
    const Eigen::Matrix3d dR = fx.preint->GetDeltaRotation(b0).cast<double>();
    const Eigen::Vector3d dV = fx.preint->GetDeltaVelocity(b0).cast<double>();
    const Eigen::Vector3d dP = fx.preint->GetDeltaPosition(b0).cast<double>();
    const Eigen::Matrix3d R1invT = R1.transpose().inverse();

    const Eigen::Matrix3d Rwb2 = R1invT * dR;
    const Eigen::Vector3d vwb2 = v1 + gWorld * dt + R1invT * dV;
    const Eigen::Vector3d pwb2 =
        t1 + v1 * dt + 0.5 * gWorld * dt * dt + R1invT * dP;

    Eigen::Quaterniond q2(Rwb2);
    q2.normalize();
    const Sophus::SE3d Twb2GT(q2, pwb2);
    fx.TcwGT = TcbD * Twb2GT.inverse();
    fx.velGT = vwb2;
    const Sophus::SE3d TwcGT = fx.TcwGT.inverse();

    // ---- Frame fields touched by PoseInertialOptimizationLastKeyFrame ------
    F.mnId = 1;
    F.N = kNumPoints;
    F.mpCamera = fx.camera.get();
    F.mpCamera2 = nullptr;
    F.Nleft = -1;               // monocular: bRight == false
    F.Nright = -1;
    F.fx = static_cast<float>(kFx);
    F.fy = static_cast<float>(kFy);
    F.cx = static_cast<float>(kCx);
    F.cy = static_cast<float>(kCy);
    F.invfx = 1.0f / F.fx;
    F.invfy = 1.0f / F.fy;
    F.mbf = 0.0f;
    F.mb = 0.0f;
    F.mThDepth = 35.0f;
    F.mImuCalib = calib;        // BEFORE SetImuPoseVelocity (reads mTcb)
    F.mpImuPreintegrated = fx.preint.get();
    F.mpImuPreintegratedFrame = nullptr;
    F.mpLastKeyFrame = pKF;
    F.mpPrevFrame = nullptr;
    F.mpcpi = nullptr;          // the optimizer allocates it; caller frees
    F.mImuBias = Bias();        // initial bias estimate (VertexGyro/AccBias)
    F.mPredBias = Bias();
    // Frame::mbImuPreintegrated is private and unread by this optimizer path.

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

    // ---- 3D grid of MapPoints + exact projections through TcwGT -----------
    F.mvpMapPoints.assign(kNumPoints, nullptr);
    F.mvKeysUn.resize(kNumPoints);
    F.mvuRight.assign(kNumPoints, -1.0f);   // monocular branch
    F.mvDepth.assign(kNumPoints, -1.0f);
    F.mvbOutlier.assign(kNumPoints, false);
    fx.mapPoints.reserve(kNumPoints);

    for (int i = 0; i < kNumPoints; i++) {
        const int col = i % kGridCols;
        const int row = i / kGridCols;
        const double x = -1.4 + 2.8 * col / (kGridCols - 1);
        const double y = -0.9 + 1.8 * row / (kGridRows - 1);
        const double z = 3.0 + 0.15 * ((i * 7) % 5);
        const Eigen::Vector3d Pc(x, y, z);
        const Eigen::Vector3d Pw = TwcGT * Pc;

        auto pMP = std::make_shared<ORB_SLAM3::MapPoint>();
        pMP->SetWorldPos(Pw.cast<float>());
        // MapPoint's default ctor leaves mTrackDepth uninitialized and the
        // optimizer reads it (bClose branch, Optimizer.cpp:4739): pin it to
        // the true camera depth so every observation takes the same branch.
        pMP->mTrackDepth = static_cast<float>(z);
        pMP->mTrackDepthR = static_cast<float>(z);
        F.mvpMapPoints[i] = pMP;
        fx.mapPoints.push_back(std::move(pMP));

        double u = kFx * Pc.x() / Pc.z() + kCx;
        double v = kFy * Pc.y() / Pc.z() + kCy;
        if (IsPiOutlierIndex(i)) {
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
    F.mvKeys = F.mvKeysUn;

    // ---- perturbed initial state (tracking-realistic IMU prediction error) -
    const Eigen::Vector3d axisP = Eigen::Vector3d(0.2, 0.5, -0.3).normalized();
    const Eigen::Matrix3d Rwb2Init =
        (Eigen::AngleAxisd(0.0175, axisP).toRotationMatrix() * Rwb2).eval();
    const Eigen::Vector3d pwb2Init = pwb2 + Eigen::Vector3d(0.02, -0.015, 0.01);
    const Eigen::Vector3d vwb2Init = vwb2 + Eigen::Vector3d(0.03, 0.04, -0.02);
    F.SetImuPoseVelocity(Rwb2Init.cast<float>(), pwb2Init.cast<float>(),
                         vwb2Init.cast<float>());

    fx.expectedInliers = kNumPoints - kPoseInertialOutliers;  // 37
    fx.outlierIndices.assign(kPiOutlierIndices,
                             kPiOutlierIndices + kPoseInertialOutliers);
    return fx;
}

}  // namespace equiv
