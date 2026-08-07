// P6 backend-equivalence harness — canonical record serialization.

#include "EquivSerialize.hpp"

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

}  // namespace equiv
