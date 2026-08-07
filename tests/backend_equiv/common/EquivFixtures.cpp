// P6 backend-equivalence harness — fixture construction (docs/P6_DESIGN.md §B).

#include "EquivFixtures.hpp"

#include "camera/Pinhole.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

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

}  // namespace equiv
