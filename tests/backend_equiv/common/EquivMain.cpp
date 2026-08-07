// P6 backend-equivalence harness — CLI runner (docs/P6_DESIGN.md §B).
//
//   equiv_runner <function> <fixture>
//
// Pairs: pose_optimization/mono_grid, inertial_optimization/imu_chain.
//
// The requested function is run TWICE, each time on an independently built
// fixture, and both full records are printed to stdout delimited by
// BEGIN_RECORD / END_RECORD lines. Identical records are the in-binary
// self-determinism gate (bit_identity pattern, P4); run_equiv.sh /
// compare.py consume the records.
//
// Exit codes: 0 = records identical, 1 = nondeterminism detected, 2 = usage.

#include "EquivFixtures.hpp"
#include "EquivSerialize.hpp"

#include "backend/G2oTypes.hpp"
#include "backend/Optimizer.hpp"
#include "core/Verbose.hpp"

#include <sophus/so3.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// EQUIV_INERTIAL_DEBUG=1 diagnostics (design §B: "debug the FIXTURE first —
// print residuals at GT"): per-edge EdgeInertialGS residuals evaluated in
// plain Eigen at (s_true, Rwg_true), plus a GT-initialized optimization to
// show whether GT is a fixed point of the solver.
void DebugInertialAtGT(equiv::ImuChainFixture& fx)
{
    const double s = fx.sTrue;
    const Eigen::Vector3d g = fx.gWorldTrue;
    std::fprintf(stderr, "DEBUG residuals at GT (s=%.17g):\n", s);
    for (int k = 1; k < equiv::ImuChainFixture::kNumKFs; k++) {
        ORB_SLAM3::KeyFrame* p1 = fx.kf(k - 1);
        ORB_SLAM3::KeyFrame* p2 = fx.kf(k);
        const Eigen::Matrix3d R1 = p1->GetImuRotation().cast<double>();
        const Eigen::Vector3d t1 = p1->GetImuPosition().cast<double>();
        const Eigen::Vector3d v1 = p1->GetVelocity().cast<double>();
        const Eigen::Matrix3d R2 = p2->GetImuRotation().cast<double>();
        const Eigen::Vector3d t2 = p2->GetImuPosition().cast<double>();
        const Eigen::Vector3d v2 = p2->GetVelocity().cast<double>();
        ORB_SLAM3::IMU::Preintegrated* pre = fx.preints[k].get();
        const ORB_SLAM3::IMU::Bias b0;
        const double dt = static_cast<double>(pre->dT);
        const Eigen::Matrix3d dR = pre->GetDeltaRotation(b0).cast<double>();
        const Eigen::Vector3d dV = pre->GetDeltaVelocity(b0).cast<double>();
        const Eigen::Vector3d dP = pre->GetDeltaPosition(b0).cast<double>();

        const Eigen::Quaterniond qe(dR.transpose() * R1.transpose() * R2);
        const double er = Eigen::AngleAxisd(qe.normalized()).angle();
        const Eigen::Vector3d ev =
            R1.transpose() * (s * (v2 - v1) - g * dt) - dV;
        const Eigen::Vector3d ep =
            R1.transpose() *
                (s * (t2 - t1 - v1 * dt) - g * dt * dt / 2.0) - dP;
        std::fprintf(stderr,
                     "  edge %d: |er|=%.3e rad |ev|=%.3e |ep|=%.3e\n",
                     k, er, ev.norm(), ep.norm());
    }

    Eigen::Matrix3d Rwg = fx.RwgTrue;
    double scale = fx.sTrue;
    ORB_SLAM3::Optimizer::InertialOptimization(fx.map.get(), Rwg, scale);
    const double gtAngle = std::acos(std::min(
        1.0, (Rwg * Eigen::Vector3d(0, 0, -1)).dot(fx.gDirTrue)));
    std::fprintf(stderr,
                 "DEBUG GT-init optimize: scale %.17g -> %.17g "
                 "(rel drift %.3e), gdir drift %.3e rad\n",
                 fx.sTrue, scale, std::abs(scale - fx.sTrue) / fx.sTrue,
                 gtAngle);

    // Chained-call convergence trace from the canonical (I, 1.0) start.
    Rwg = Eigen::Matrix3d::Identity();
    scale = 1.0;
    for (int c = 1; c <= 5; c++) {
        ORB_SLAM3::Optimizer::InertialOptimization(fx.map.get(), Rwg, scale);
        const double ang = std::acos(std::min(
            1.0, (Rwg * Eigen::Vector3d(0, 0, -1)).dot(fx.gDirTrue)));
        std::fprintf(stderr,
                     "DEBUG chain call %d: scale=%.17g (rel err %.3e), "
                     "gdir err %.3e rad\n",
                     c, scale, std::abs(scale - fx.sTrue) / fx.sTrue, ang);
    }

    // 1-D robustified-cost sweep over scale at Rwg = RwgTrue: solver-free
    // edge evaluation, distinguishes "shifted robust minimum" from "solver
    // pathology".
    for (double sSweep : {1.0, 1.9, 1.99, 2.0, 2.01, 2.0475, 2.1}) {
        double totalRaw = 0.0, totalRobust = 0.0;
        for (int k = 1; k < equiv::ImuChainFixture::kNumKFs; k++) {
            auto* VP1 = new ORB_SLAM3::VertexPose(fx.kf(k - 1));
            auto* VV1 = new ORB_SLAM3::VertexVelocity(fx.kf(k - 1));
            auto* VG = new ORB_SLAM3::VertexGyroBias(fx.kf(0));
            auto* VA = new ORB_SLAM3::VertexAccBias(fx.kf(0));
            auto* VP2 = new ORB_SLAM3::VertexPose(fx.kf(k));
            auto* VV2 = new ORB_SLAM3::VertexVelocity(fx.kf(k));
            Eigen::Matrix3d RwgT = fx.RwgTrue;
            auto* VGD = new ORB_SLAM3::VertexGDir(RwgT);
            auto* VS = new ORB_SLAM3::VertexScale(sSweep);
            auto* e = new ORB_SLAM3::EdgeInertialGS(fx.preints[k].get());
            e->setVertex(0, VP1);
            e->setVertex(1, VV1);
            e->setVertex(2, VG);
            e->setVertex(3, VA);
            e->setVertex(4, VP2);
            e->setVertex(5, VV2);
            e->setVertex(6, VGD);
            e->setVertex(7, VS);
            e->computeError();
            const double chi2 = e->chi2();
            totalRaw += chi2;
            totalRobust +=
                (chi2 > 1.0) ? (2.0 * std::sqrt(chi2) - 1.0) : chi2;
            delete e;  // edge does not own vertices
            delete VP1; delete VV1; delete VG; delete VA;
            delete VP2; delete VV2; delete VGD; delete VS;
        }
        std::fprintf(stderr,
                     "DEBUG sweep s=%.6g: chi2 raw=%.6e robust=%.6e\n",
                     sSweep, totalRaw, totalRobust);
    }

    // Exact robustified free-block (theta1,theta2,u) GN system at a probe
    // state, Jacobians by central finite differences of the edges' public
    // computeError()/error() — version-agnostic, no g2o internals.
    auto edgeError = [&fx](int k, const Eigen::Matrix3d& RwgProbe,
                           double sProbe) -> Eigen::Matrix<double, 9, 1> {
        auto* VP1 = new ORB_SLAM3::VertexPose(fx.kf(k - 1));
        auto* VV1 = new ORB_SLAM3::VertexVelocity(fx.kf(k - 1));
        auto* VG = new ORB_SLAM3::VertexGyroBias(fx.kf(0));
        auto* VA = new ORB_SLAM3::VertexAccBias(fx.kf(0));
        auto* VP2 = new ORB_SLAM3::VertexPose(fx.kf(k));
        auto* VV2 = new ORB_SLAM3::VertexVelocity(fx.kf(k));
        Eigen::Matrix3d RwgT = RwgProbe;
        auto* VGD = new ORB_SLAM3::VertexGDir(RwgT);
        auto* VS = new ORB_SLAM3::VertexScale(sProbe);
        auto* e = new ORB_SLAM3::EdgeInertialGS(fx.preints[k].get());
        e->setVertex(0, VP1);
        e->setVertex(1, VV1);
        e->setVertex(2, VG);
        e->setVertex(3, VA);
        e->setVertex(4, VP2);
        e->setVertex(5, VV2);
        e->setVertex(6, VGD);
        e->setVertex(7, VS);
        e->computeError();
        const Eigen::Matrix<double, 9, 1> r = e->error();
        delete e;
        delete VP1; delete VV1; delete VG; delete VA;
        delete VP2; delete VV2; delete VGD; delete VS;
        return r;
    };
    auto edgeInfo = [&fx](int k) -> Eigen::Matrix<double, 9, 9> {
        auto* e = new ORB_SLAM3::EdgeInertialGS(fx.preints[k].get());
        const Eigen::Matrix<double, 9, 9> W = e->information();
        delete e;
        return W;
    };
    for (double sProbe : {2.0475123895540701, 1.0}) {
        const double h = 1e-6;
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d gvec = Eigen::Vector3d::Zero();
        for (int k = 1; k < equiv::ImuChainFixture::kNumKFs; k++) {
            const Eigen::Matrix<double, 9, 9> W = edgeInfo(k);
            const Eigen::Matrix<double, 9, 1> r0 =
                edgeError(k, fx.RwgTrue, sProbe);
            Eigen::Matrix<double, 9, 3> J;
            for (int d = 0; d < 2; d++) {  // gravity-dir dofs (Exp(u0,u1,0))
                Eigen::Vector3d u = Eigen::Vector3d::Zero();
                u[d] = h;
                const Eigen::Matrix3d Rp =
                    fx.RwgTrue * Sophus::SO3d::exp(u).matrix();
                const Eigen::Matrix3d Rm =
                    fx.RwgTrue * Sophus::SO3d::exp(-u).matrix();
                J.col(d) = (edgeError(k, Rp, sProbe) -
                            edgeError(k, Rm, sProbe)) / (2.0 * h);
            }
            J.col(2) = (edgeError(k, fx.RwgTrue, sProbe * std::exp(h)) -
                        edgeError(k, fx.RwgTrue, sProbe * std::exp(-h))) /
                       (2.0 * h);
            const double chi2 = r0.dot(W * r0);
            const double rho1 =
                (chi2 > 1.0) ? 1.0 / std::sqrt(chi2) : 1.0;  // Huber d=1
            H += rho1 * J.transpose() * W * J;
            gvec += rho1 * J.transpose() * W * r0;
        }
        const Eigen::Vector3d step = -H.ldlt().solve(gvec);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(H);
        std::fprintf(stderr,
                     "DEBUG GN system at s=%.6g (Rwg=true): "
                     "step=(%.3e, %.3e, %.3e)  g=(%.3e, %.3e, %.3e)  "
                     "H eig=(%.3e, %.3e, %.3e)\n",
                     sProbe, step[0], step[1], step[2],
                     gvec[0], gvec[1], gvec[2],
                     es.eigenvalues()[0], es.eigenvalues()[1],
                     es.eigenvalues()[2]);
    }
}

std::string RunInertialOptimizationOnce(const std::string& function,
                                        const std::string& fixture)
{
    equiv::ImuChainFixture fx = equiv::MakeImuChainFixture();

    if (std::getenv("EQUIV_INERTIAL_DEBUG")) {
        equiv::ImuChainFixture fxDbg = equiv::MakeImuChainFixture();
        DebugInertialAtGT(fxDbg);
    }

    // Deliberately perturbed initial estimate: the fixture states are
    // 1/s_true-scaled with a ~10 deg gravity tilt, so starting from the
    // identity/unit initialization the optimizer must recover
    // (s_true, gDirTrue) — the analytic optimum (design §B 1.2).
    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();
    double scale = 1.0;

    const std::string inputDump =
        equiv::SerializeInertialOptimizationInput(fx, Rwg, scale);
    const std::string inputHash = equiv::Sha256Hex(inputDump);

    ORB_SLAM3::Optimizer::InertialOptimization(fx.map.get(), Rwg, scale);

    return equiv::MakeInertialOptimizationRecord(function, fixture, inputHash,
                                                 fx, scale, Rwg);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::fprintf(stderr,
                     "usage: %s <function> <fixture>\n"
                     "  pairs: pose_optimization mono_grid\n"
                     "         inertial_optimization imu_chain\n",
                     argv[0]);
        return 2;
    }
    const std::string function = argv[1];
    const std::string fixture = argv[2];

    std::string rec1, rec2;
    if (function == "pose_optimization" && fixture == "mono_grid") {
        rec1 = RunPoseOptimizationOnce(function, fixture);
        rec2 = RunPoseOptimizationOnce(function, fixture);
    } else if (function == "inertial_optimization" && fixture == "imu_chain") {
        rec1 = RunInertialOptimizationOnce(function, fixture);
        rec2 = RunInertialOptimizationOnce(function, fixture);
    } else {
        std::fprintf(stderr, "unknown function/fixture: %s %s\n",
                     function.c_str(), fixture.c_str());
        return 2;
    }

    std::printf("BEGIN_RECORD\n%sEND_RECORD\n", rec1.c_str());
    std::printf("BEGIN_RECORD\n%sEND_RECORD\n", rec2.c_str());

    if (rec1 != rec2) {
        std::fprintf(stderr, "SELF-DETERMINISM FAILURE: records differ\n");
        return 1;
    }
    return 0;
}
