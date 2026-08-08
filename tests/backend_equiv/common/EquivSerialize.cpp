// P6 backend-equivalence harness — canonical record serialization.

#include "EquivSerialize.hpp"

#include "backend/G2oTypes.hpp"   // ConstraintPoseImu (Frame::mpcpi target)
#include "map/MapPoint.hpp"

#include <openssl/sha.h>

#include <Eigen/Geometry>

#include <cstdio>
#include <sstream>

namespace equiv {

namespace {

// Sign normalization: canonical representative of the double-cover,
// w >= 0 (fixtures keep |w| far from 0, so the w == 0 tie never occurs).
Eigen::Quaterniond NormalizedSign(const Eigen::Quaterniond& q)
{
    if (q.w() < 0.0)
        return Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}

void AppendPose(std::ostringstream& os, const char* prefix,
                const Sophus::SE3d& T)
{
    const Eigen::Vector3d t = T.translation();
    const Eigen::Quaterniond q = NormalizedSign(T.unit_quaternion());
    os << prefix << "_t " << FormatDouble(t.x()) << ' ' << FormatDouble(t.y())
       << ' ' << FormatDouble(t.z()) << '\n';
    os << prefix << "_q " << FormatDouble(q.w()) << ' ' << FormatDouble(q.x())
       << ' ' << FormatDouble(q.y()) << ' ' << FormatDouble(q.z()) << '\n';
}

void AppendQuat(std::ostringstream& os, const char* label,
                const Eigen::Quaterniond& qIn)
{
    const Eigen::Quaterniond q = NormalizedSign(qIn);
    os << label << ' ' << FormatDouble(q.w()) << ' ' << FormatDouble(q.x())
       << ' ' << FormatDouble(q.y()) << ' ' << FormatDouble(q.z()) << '\n';
}

void AppendVec3(std::ostringstream& os, const char* label,
                const Eigen::Vector3d& v)
{
    os << label << ' ' << FormatDouble(v.x()) << ' ' << FormatDouble(v.y())
       << ' ' << FormatDouble(v.z()) << '\n';
}

template <typename Derived>
void AppendMatRowMajor(std::ostringstream& os, const char* label,
                       const Eigen::MatrixBase<Derived>& m)
{
    os << label;
    for (int r = 0; r < m.rows(); r++)
        for (int c = 0; c < m.cols(); c++)
            os << ' ' << FormatDouble(static_cast<double>(m(r, c)));
    os << '\n';
}

}  // namespace

std::string FormatDouble(double v)
{
    char buf[40];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return std::string(buf);
}

std::string Sha256Hex(const std::string& data)
{
    unsigned char md[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(data.data()), data.size(), md);
    char hex[2 * SHA256_DIGEST_LENGTH + 1];
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++)
        std::snprintf(hex + 2 * i, 3, "%02x", md[i]);
    return std::string(hex);
}

std::string SerializePoseOptimizationInput(const ORB_SLAM3::Frame& F)
{
    std::ostringstream os;
    os << "INPUT pose_optimization v1\n";
    os << "N " << F.N << '\n';
    os << "camera pinhole " << FormatDouble(F.mpCamera->getParameter(0)) << ' '
       << FormatDouble(F.mpCamera->getParameter(1)) << ' '
       << FormatDouble(F.mpCamera->getParameter(2)) << ' '
       << FormatDouble(F.mpCamera->getParameter(3)) << '\n';
    AppendPose(os, "pose0", F.GetPose().cast<double>());
    for (int i = 0; i < F.N; i++) {
        const Eigen::Vector3d Xw =
            F.mvpMapPoints[i]->GetWorldPos().cast<double>();
        const cv::KeyPoint& kp = F.mvKeysUn[i];
        os << "obs " << i << ' ' << FormatDouble(Xw.x()) << ' '
           << FormatDouble(Xw.y()) << ' ' << FormatDouble(Xw.z()) << ' '
           << FormatDouble(static_cast<double>(kp.pt.x)) << ' '
           << FormatDouble(static_cast<double>(kp.pt.y)) << ' ' << kp.octave
           << ' ' << FormatDouble(static_cast<double>(F.mvuRight[i])) << '\n';
    }
    return os.str();
}

std::string SerializeStage0Edges()
{
    // Stage-0 (per-edge error/chi2/information before any solver iteration)
    // is a follow-up: it will instantiate the projection edges directly from
    // the fixture and serialize errors in fixed index order. Emitting nothing
    // keeps the record schema forward-compatible (section simply absent).
    return std::string();
}

std::string SerializePoseOptimizationResult(int inliers,
                                            const ORB_SLAM3::Frame& F)
{
    std::ostringstream os;
    os << "RESULT\n";
    os << "inliers " << inliers << '\n';
    AppendPose(os, "pose", F.GetPose().cast<double>());
    os << "outliers";
    for (int i = 0; i < F.N; i++)
        os << ' ' << (F.mvbOutlier[i] ? 1 : 0);
    os << '\n';
    os << "END_RESULT\n";
    return os.str();
}

std::string MakePoseOptimizationRecord(const std::string& function,
                                       const std::string& fixture,
                                       const std::string& inputHash,
                                       int inliers,
                                       const ORB_SLAM3::Frame& F)
{
    std::ostringstream os;
    os << "EQUIV_RECORD v1\n";
    os << "function " << function << '\n';
    os << "fixture " << fixture << '\n';
    os << "INPUT_HASH " << inputHash << '\n';
    os << SerializeStage0Edges();
    os << SerializePoseOptimizationResult(inliers, F);
    return os.str();
}

std::string SerializeInertialOptimizationInput(const ImuChainFixture& fx,
                                               const Eigen::Matrix3d& Rwg0,
                                               double scale0)
{
    std::ostringstream os;
    os << "INPUT inertial_optimization v1\n";
    os << "nkfs " << ImuChainFixture::kNumKFs << '\n';

    // IMU calibration (identical on every KF; read from KF 0).
    const ORB_SLAM3::IMU::Calib& calib = fx.kf(0)->mImuCalib;
    AppendPose(os, "calib_tbc", calib.mTbc.cast<double>());
    AppendMatRowMajor(os, "calib_cov", calib.Cov.diagonal());
    AppendMatRowMajor(os, "calib_covwalk", calib.CovWalk.diagonal());

    // Initial estimate handed to the optimizer.
    AppendQuat(os, "init_rwg_q", Eigen::Quaterniond(Rwg0));
    os << "init_scale " << FormatDouble(scale0) << '\n';

    // Per-KF stored state, read back through the optimizer's accessors.
    for (int k = 0; k < ImuChainFixture::kNumKFs; k++) {
        ORB_SLAM3::KeyFrame* pKF = fx.kf(k);
        os << "kf " << pKF->mnId << '\n';
        AppendPose(os, "tcw", pKF->GetPose().cast<double>());
        AppendVec3(os, "twb", pKF->GetImuPosition().cast<double>());
        AppendQuat(os, "rwb_q",
                   Eigen::Quaterniond(
                       pKF->GetImuRotation().cast<double>()));
        AppendVec3(os, "vel", pKF->GetVelocity().cast<double>());
        AppendVec3(os, "bg", pKF->GetGyroBias().cast<double>());
        AppendVec3(os, "ba", pKF->GetAccBias().cast<double>());
    }

    // Per-interval preintegration (everything EdgeInertialGS reads).
    for (int k = 1; k < ImuChainFixture::kNumKFs; k++) {
        ORB_SLAM3::IMU::Preintegrated* pre = fx.preints[k].get();
        const ORB_SLAM3::IMU::Bias b0;
        os << "preint " << k << '\n';
        // Linearization bias: only a non-trivial input in the P6-4 bias
        // variants (it is identically zero for kScaleGravity), so it is
        // emitted only there — that keeps the P6-3 record byte-stable.
        if (fx.variant != ImuChainVariant::kScaleGravity) {
            os << "preint_b " << FormatDouble(pre->b.bwx) << ' '
               << FormatDouble(pre->b.bwy) << ' ' << FormatDouble(pre->b.bwz)
               << ' ' << FormatDouble(pre->b.bax) << ' '
               << FormatDouble(pre->b.bay) << ' ' << FormatDouble(pre->b.baz)
               << '\n';
        }
        os << "dT " << FormatDouble(static_cast<double>(pre->dT)) << '\n';
        AppendQuat(os, "dR_q",
                   Eigen::Quaterniond(
                       pre->GetDeltaRotation(b0).cast<double>()));
        AppendVec3(os, "dV", pre->GetDeltaVelocity(b0).cast<double>());
        AppendVec3(os, "dP", pre->GetDeltaPosition(b0).cast<double>());
        AppendMatRowMajor(os, "JRg", pre->JRg);
        AppendMatRowMajor(os, "JVg", pre->JVg);
        AppendMatRowMajor(os, "JVa", pre->JVa);
        AppendMatRowMajor(os, "JPg", pre->JPg);
        AppendMatRowMajor(os, "JPa", pre->JPa);
        AppendMatRowMajor(os, "C9", pre->C.block<9, 9>(0, 0));
    }
    return os.str();
}

std::string SerializeInertialOptimizationResult(double scale,
                                                const Eigen::Matrix3d& Rwg,
                                                const ImuChainFixture& fx)
{
    std::ostringstream os;
    os << "RESULT\n";
    os << "scale " << FormatDouble(scale) << '\n';
    AppendQuat(os, "rwg_q", Eigen::Quaterniond(Rwg));
    AppendVec3(os, "gdir", Rwg * Eigen::Vector3d(0.0, 0.0, -1.0));
    for (int k = 0; k < ImuChainFixture::kNumKFs; k++) {
        const Eigen::Vector3d v = fx.kf(k)->GetVelocity().cast<double>();
        os << "vel " << k << ' ' << FormatDouble(v.x()) << ' '
           << FormatDouble(v.y()) << ' ' << FormatDouble(v.z()) << '\n';
    }
    os << "END_RESULT\n";
    return os.str();
}

std::string MakeInertialOptimizationRecord(const std::string& function,
                                           const std::string& fixture,
                                           const std::string& inputHash,
                                           const ImuChainFixture& fx,
                                           double scale,
                                           const Eigen::Matrix3d& Rwg)
{
    std::ostringstream os;
    os << "EQUIV_RECORD v1\n";
    os << "function " << function << '\n';
    os << "fixture " << fixture << '\n';
    os << "INPUT_HASH " << inputHash << '\n';
    os << "GT_scale " << FormatDouble(fx.sTrue) << '\n';
    AppendVec3(os, "GT_gdir", fx.gDirTrue);
    os << SerializeInertialOptimizationResult(scale, Rwg, fx);
    return os.str();
}

// ---------------------------------------------------------------------------
// P6-4 (a)/(b) — bias-carrying InertialOptimization overloads.
// ---------------------------------------------------------------------------

namespace {

// Measured irreducible residual at the analytic optimum (float32 state
// storage). compare.py DERIVES the GT bias tolerances from these rows, so the
// only "loosening" of the 1e-7 design gate is a measured quantity carried in
// the record itself (design §B.2: "record the measured delta in the report and
// tighten the gate empirically").
void AppendGtResidual(std::ostringstream& os, const ImuChainFixture& fx)
{
    os << "GT_resid_er " << FormatDouble(fx.residErMax) << '\n';
    os << "GT_resid_ev " << FormatDouble(fx.residEvMax) << '\n';
    os << "GT_resid_ep " << FormatDouble(fx.residEpMax) << '\n';
    os << "GT_resid_dT " << FormatDouble(static_cast<double>(
              fx.preints[1]->dT)) << '\n';
}

// Velocities are OPTIMIZED outputs in both bias overloads (VertexVelocity is
// free), written back with SetVelocity(Vw.cast<float>()) — float storage path.
void AppendOptimizedVelocities(std::ostringstream& os,
                               const ImuChainFixture& fx)
{
    for (int k = 0; k < ImuChainFixture::kNumKFs; k++) {
        const Eigen::Vector3d v = fx.kf(k)->GetVelocity().cast<double>();
        os << "vel " << k << ' ' << FormatDouble(v.x()) << ' '
           << FormatDouble(v.y()) << ' ' << FormatDouble(v.z()) << '\n';
    }
}

}  // namespace

std::string MakeInertialFullRecord(const std::string& function,
                                   const std::string& fixture,
                                   const std::string& inputHash,
                                   const ImuChainFixture& fx,
                                   double priorG, double priorA,
                                   double scale, const Eigen::Matrix3d& Rwg,
                                   const Eigen::Vector3d& bg,
                                   const Eigen::Vector3d& ba)
{
    std::ostringstream os;
    os << "EQUIV_RECORD v1\n";
    os << "function " << function << '\n';
    os << "fixture " << fixture << '\n';
    os << "INPUT_HASH " << inputHash << '\n';
    os << "priorG " << FormatDouble(priorG) << '\n';
    os << "priorA " << FormatDouble(priorA) << '\n';
    AppendGtResidual(os, fx);
    os << "GT_scale " << FormatDouble(fx.sTrue) << '\n';
    AppendVec3(os, "GT_gdir", fx.gDirTrue);
    AppendVec3(os, "GT_bg", fx.bgTrue);
    AppendVec3(os, "GT_ba", fx.baTrue);
    os << "RESULT\n";
    os << "scale " << FormatDouble(scale) << '\n';
    AppendQuat(os, "rwg_q", Eigen::Quaterniond(Rwg));
    AppendVec3(os, "gdir", Rwg * Eigen::Vector3d(0.0, 0.0, -1.0));
    AppendVec3(os, "bg", bg);
    AppendVec3(os, "ba", ba);
    AppendOptimizedVelocities(os, fx);
    os << "END_RESULT\n";
    return os.str();
}

std::string MakeInertialBiasRecord(const std::string& function,
                                   const std::string& fixture,
                                   const std::string& inputHash,
                                   const ImuChainFixture& fx,
                                   double priorG, double priorA,
                                   const Eigen::Vector3d& bg,
                                   const Eigen::Vector3d& ba)
{
    std::ostringstream os;
    os << "EQUIV_RECORD v1\n";
    os << "function " << function << '\n';
    os << "fixture " << fixture << '\n';
    os << "INPUT_HASH " << inputHash << '\n';
    os << "priorG " << FormatDouble(priorG) << '\n';
    os << "priorA " << FormatDouble(priorA) << '\n';
    AppendGtResidual(os, fx);
    AppendVec3(os, "GT_bg", fx.bgTrue);
    AppendVec3(os, "GT_ba", fx.baTrue);
    os << "RESULT\n";
    AppendVec3(os, "bg", bg);
    AppendVec3(os, "ba", ba);
    AppendOptimizedVelocities(os, fx);
    os << "END_RESULT\n";
    return os.str();
}

// ---------------------------------------------------------------------------
// P6-4 (TASK 2) — PoseInertialOptimizationLastKeyFrame.
// ---------------------------------------------------------------------------

std::string SerializePoseInertialInput(const PoseInertialFixture& fx)
{
    const ORB_SLAM3::Frame& F = fx.frame;
    ORB_SLAM3::KeyFrame* pKF = fx.kf();

    std::ostringstream os;
    os << "INPUT pose_inertial_lastkf v1\n";
    os << "N " << F.N << '\n';
    os << "Nleft " << F.Nleft << '\n';
    os << "camera pinhole " << FormatDouble(F.mpCamera->getParameter(0)) << ' '
       << FormatDouble(F.mpCamera->getParameter(1)) << ' '
       << FormatDouble(F.mpCamera->getParameter(2)) << ' '
       << FormatDouble(F.mpCamera->getParameter(3)) << '\n';

    // IMU calibration (shared by Frame and KeyFrame).
    AppendPose(os, "calib_tbc", F.mImuCalib.mTbc.cast<double>());
    AppendMatRowMajor(os, "calib_cov", F.mImuCalib.Cov.diagonal());
    AppendMatRowMajor(os, "calib_covwalk", F.mImuCalib.CovWalk.diagonal());

    // Inverse level sigma^2 table (drives every visual information matrix).
    os << "invLevelSigma2";
    for (int k = 0; k < F.mnScaleLevels; k++)
        os << ' ' << FormatDouble(static_cast<double>(F.mvInvLevelSigma2[k]));
    os << '\n';

    // Fixed KeyFrame state, read back through the optimizer's accessors.
    AppendPose(os, "kf_tcw", pKF->GetPose().cast<double>());
    AppendVec3(os, "kf_twb", pKF->GetImuPosition().cast<double>());
    AppendQuat(os, "kf_rwb_q",
               Eigen::Quaterniond(pKF->GetImuRotation().cast<double>()));
    AppendVec3(os, "kf_vel", pKF->GetVelocity().cast<double>());
    AppendVec3(os, "kf_bg", pKF->GetGyroBias().cast<double>());
    AppendVec3(os, "kf_ba", pKF->GetAccBias().cast<double>());

    // Initial Frame state (the optimizer's starting point).
    AppendPose(os, "f_tcw0", F.GetPose().cast<double>());
    AppendVec3(os, "f_vel0", F.GetVelocity().cast<double>());
    os << "f_bias0 " << FormatDouble(F.mImuBias.bwx) << ' '
       << FormatDouble(F.mImuBias.bwy) << ' ' << FormatDouble(F.mImuBias.bwz)
       << ' ' << FormatDouble(F.mImuBias.bax) << ' '
       << FormatDouble(F.mImuBias.bay) << ' ' << FormatDouble(F.mImuBias.baz)
       << '\n';

    // Observations, ascending index.
    for (int i = 0; i < F.N; i++) {
        ORB_SLAM3::MapPoint* pMP = F.mvpMapPoints[i];
        const Eigen::Vector3d Xw = pMP->GetWorldPos().cast<double>();
        const cv::KeyPoint& kp = F.mvKeysUn[i];
        os << "obs " << i << ' ' << FormatDouble(Xw.x()) << ' '
           << FormatDouble(Xw.y()) << ' ' << FormatDouble(Xw.z()) << ' '
           << FormatDouble(static_cast<double>(kp.pt.x)) << ' '
           << FormatDouble(static_cast<double>(kp.pt.y)) << ' ' << kp.octave
           << ' ' << FormatDouble(static_cast<double>(F.mvuRight[i])) << ' '
           << FormatDouble(static_cast<double>(pMP->mTrackDepth)) << '\n';
    }

    // Preintegration over the KF -> Frame link. The FULL 15x15 covariance is
    // dumped: (0,0) feeds EdgeInertial's information, (9,9) and (12,12) feed
    // EdgeGyroRW / EdgeAccRW (Optimizer.cpp:4689,4696).
    ORB_SLAM3::IMU::Preintegrated* pre = fx.preint.get();
    const ORB_SLAM3::IMU::Bias b0;
    os << "preint 1\n";
    os << "dT " << FormatDouble(static_cast<double>(pre->dT)) << '\n';
    AppendQuat(os, "dR_q",
               Eigen::Quaterniond(pre->GetDeltaRotation(b0).cast<double>()));
    AppendVec3(os, "dV", pre->GetDeltaVelocity(b0).cast<double>());
    AppendVec3(os, "dP", pre->GetDeltaPosition(b0).cast<double>());
    AppendMatRowMajor(os, "JRg", pre->JRg);
    AppendMatRowMajor(os, "JVg", pre->JVg);
    AppendMatRowMajor(os, "JVa", pre->JVa);
    AppendMatRowMajor(os, "JPg", pre->JPg);
    AppendMatRowMajor(os, "JPa", pre->JPa);
    AppendMatRowMajor(os, "C15", pre->C);
    return os.str();
}

std::string SerializePoseInertialResult(int inliers,
                                        const ORB_SLAM3::Frame& F)
{
    std::ostringstream os;
    os << "RESULT\n";
    os << "inliers " << inliers << '\n';
    AppendPose(os, "pose", F.GetPose().cast<double>());
    AppendVec3(os, "vel", F.GetVelocity().cast<double>());
    AppendVec3(os, "bg",
               Eigen::Vector3d(F.mImuBias.bwx, F.mImuBias.bwy, F.mImuBias.bwz));
    AppendVec3(os, "ba",
               Eigen::Vector3d(F.mImuBias.bax, F.mImuBias.bay, F.mImuBias.baz));
    os << "outliers";
    for (int i = 0; i < F.N; i++)
        os << ' ' << (F.mvbOutlier[i] ? 1 : 0);
    os << '\n';

    // The marginalization contract: the 15x15 prior Hessian the next frame
    // will consume through EdgePriorPoseImu. One line per row so the record
    // stays diff-readable.
    const Eigen::Matrix<double, 15, 15>& H = F.mpcpi->H;
    for (int r = 0; r < 15; r++) {
        os << "H " << r;
        for (int c = 0; c < 15; c++)
            os << ' ' << FormatDouble(H(r, c));
        os << '\n';
    }
    os << "END_RESULT\n";
    return os.str();
}

std::string MakePoseInertialRecord(const std::string& function,
                                   const std::string& fixture,
                                   const std::string& inputHash,
                                   const PoseInertialFixture& fx,
                                   int inliers)
{
    std::ostringstream os;
    os << "EQUIV_RECORD v1\n";
    os << "function " << function << '\n';
    os << "fixture " << fixture << '\n';
    os << "INPUT_HASH " << inputHash << '\n';
    AppendPose(os, "GT_pose", fx.TcwGT);
    AppendVec3(os, "GT_vel", fx.velGT);
    os << SerializePoseInertialResult(inliers, fx.frame);
    return os.str();
}

}  // namespace equiv
