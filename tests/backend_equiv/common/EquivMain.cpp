// P6 backend-equivalence harness — CLI runner (docs/P6_DESIGN.md §B).
//
//   equiv_runner <function> <fixture>
//
// Pairs:
//   pose_optimization          mono_grid         PoseOptimization
//   inertial_optimization      imu_chain         InertialOptimization(Rwg,s)
//   inertial_optimization_full imu_chain_bias    InertialOptimization(11 args)
//   inertial_optimization_bias imu_chain_metric  InertialOptimization(bg,ba)
//   pose_inertial_lastkf       mono_imu_link     PoseInertialOptimization-
//                                                LastKeyFrame
//
// Fixture diagnostics (stderr, opt-in): EQUIV_INERTIAL_DEBUG=1 (scale/gravity
// overload), EQUIV_FULL_DEBUG=1 and EQUIV_BIAS_DEBUG=1 (the two P6-4 bias
// overloads: chi2 at the analytic optimum, a GT-initialized fixed-point probe
// and a chained-call trace — the evidence behind the LM-stall gates recorded
// in compare.py).
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

// P6-4 prior weights.
//
// (a) the 11-argument IMU-init overload runs with the PRODUCTION first-init
//     weights (LocalMapping.cpp:186 — InitializeIMU(1e2, 1e10)). priorA = 1e10
//     pins the accelerometer bias to zero, which is exactly what regularizes
//     the (scale, gravity tilt, u0, ba) valley documented in
//     EquivFixtures.cpp; the kFullInit fixture therefore injects ba_true = 0
//     and keeps a nonzero analytic GT on bg / scale / gravity direction.
//     priorG = 1e2 against a gyro-bias information block of ~1e8 shrinks bg by
//     ~1e-6 relative, i.e. ~3e-9 absolute — three orders below the GT gate.
//
// (b) the bias-only overload pins gravity and scale internally, so nothing is
//     degenerate and BOTH biases have a genuine nonzero analytic optimum — but
//     only if the zero-mean prior is weak: production's priorA = 1e6 is the
//     same order as this fixture's accelerometer information (~5e5) and would
//     shrink ba by ~2/3, destroying the analytic GT. 1e-3 keeps the shrinkage
//     ~1e-9 while staying NON-ZERO so the
//         if (priorG != 0.f) solver->setUserLambdaInit(1e3)
//     branch (Optimizer.cpp:3246 always sets it here, 3063 in the full
//     overload) stays on the production path. Recorded in the record header.
constexpr double kEquivFullPriorG = 1e2;
constexpr double kEquivFullPriorA = 1e10;
constexpr double kEquivBiasPriorG = 1e-3;
constexpr double kEquivBiasPriorA = 1e-3;

// Total EdgeInertialGS chi2 of the chain at an arbitrary probe state, built
// from directly instantiated vertices/edges (no g2o solver involved), so it is
// version-agnostic. Used by the EQUIV_FULL_DEBUG diagnostics to tell
// "optimizer stopped short" from "GT is not the minimum".
double InertialGsChi2(equiv::ImuChainFixture& fx, double s,
                      const Eigen::Matrix3d& RwgIn,
                      const Eigen::Vector3d& bg, const Eigen::Vector3d& ba,
                      const std::vector<Eigen::Vector3d>& vels)
{
    double total = 0.0;
    for (int k = 1; k < equiv::ImuChainFixture::kNumKFs; k++) {
        auto* VP1 = new ORB_SLAM3::VertexPose(fx.kf(k - 1));
        auto* VV1 = new ORB_SLAM3::VertexVelocity(fx.kf(k - 1));
        auto* VG = new ORB_SLAM3::VertexGyroBias(fx.kf(0));
        auto* VA = new ORB_SLAM3::VertexAccBias(fx.kf(0));
        auto* VP2 = new ORB_SLAM3::VertexPose(fx.kf(k));
        auto* VV2 = new ORB_SLAM3::VertexVelocity(fx.kf(k));
        Eigen::Matrix3d Rwg = RwgIn;
        auto* VGD = new ORB_SLAM3::VertexGDir(Rwg);
        auto* VS = new ORB_SLAM3::VertexScale(s);
        VV1->setEstimate(vels[k - 1]);
        VV2->setEstimate(vels[k]);
        VG->setEstimate(bg);
        VA->setEstimate(ba);
        auto* e = new ORB_SLAM3::EdgeInertialGS(fx.preints[k].get());
        e->setVertex(0, VP1); e->setVertex(1, VV1); e->setVertex(2, VG);
        e->setVertex(3, VA);  e->setVertex(4, VP2); e->setVertex(5, VV2);
        e->setVertex(6, VGD); e->setVertex(7, VS);
        e->computeError();
        total += e->chi2();
        delete e;
        delete VP1; delete VV1; delete VG; delete VA;
        delete VP2; delete VV2; delete VGD; delete VS;
    }
    return total;
}

std::vector<Eigen::Vector3d> StoredVelocities(const equiv::ImuChainFixture& fx)
{
    std::vector<Eigen::Vector3d> v;
    for (int k = 0; k < equiv::ImuChainFixture::kNumKFs; k++)
        v.push_back(fx.kf(k)->GetVelocity().cast<double>());
    return v;
}

// EQUIV_FULL_DEBUG=1: is the 11-argument overload stopping short of the
// analytic optimum, or is the analytic optimum not the minimum?
void DebugInertialFull(double priorG, double priorA)
{
    equiv::ImuChainFixture fx =
        equiv::MakeImuChainFixture(equiv::ImuChainVariant::kFullInit);

    const std::vector<Eigen::Vector3d> velGT = StoredVelocities(fx);
    const double chi2GT = InertialGsChi2(fx, fx.sTrue, fx.RwgTrue, fx.bgTrue,
                                         fx.baTrue, velGT);
    std::fprintf(stderr, "DEBUG(full) chi2 at analytic GT = %.6e\n", chi2GT);

    // GT-init fixed-point probe: does the analytic optimum stay put?
    {
        equiv::ImuChainFixture fg =
            equiv::MakeImuChainFixture(equiv::ImuChainVariant::kFullInit);
        const ORB_SLAM3::IMU::Bias bT(
            static_cast<float>(fg.baTrue.x()), static_cast<float>(fg.baTrue.y()),
            static_cast<float>(fg.baTrue.z()), static_cast<float>(fg.bgTrue.x()),
            static_cast<float>(fg.bgTrue.y()), static_cast<float>(fg.bgTrue.z()));
        for (int k = 0; k < equiv::ImuChainFixture::kNumKFs; k++)
            fg.kf(k)->SetNewBias(bT);
        Eigen::Matrix3d Rwg = fg.RwgTrue;
        double scale = fg.sTrue;
        Eigen::Vector3d bg = fg.bgTrue, ba = fg.baTrue;
        Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(9, 9);
        ORB_SLAM3::Optimizer::InertialOptimization(
            fg.map.get(), Rwg, scale, bg, ba, true, cov, false, false,
            static_cast<float>(priorG), static_cast<float>(priorA));
        const double ang = std::acos(std::min(
            1.0, (Rwg * Eigen::Vector3d(0, 0, -1)).dot(fg.gDirTrue)));
        std::fprintf(stderr,
                     "DEBUG(full) GT-init: s=%.9f (drift %.3e) gdir drift "
                     "%.3e rad |dbg|=%.3e |dba|=%.3e\n",
                     scale, std::abs(scale - fg.sTrue), ang,
                     (bg - fg.bgTrue).norm(), (ba - fg.baTrue).norm());
    }

    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();
    double scale = 1.0;
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(9, 9);
    for (int c = 1; c <= 8; c++) {
        ORB_SLAM3::Optimizer::InertialOptimization(
            fx.map.get(), Rwg, scale, bg, ba, true, cov, false, false,
            static_cast<float>(priorG), static_cast<float>(priorA));
        const double ang = std::acos(std::min(
            1.0, (Rwg * Eigen::Vector3d(0, 0, -1)).dot(fx.gDirTrue)));
        const double chi2 = InertialGsChi2(fx, scale, Rwg, bg, ba,
                                           StoredVelocities(fx));
        std::fprintf(stderr,
                     "DEBUG(full) call %d: s=%.9f (err %.3e) gdir err %.3e rad "
                     "|dbg|=%.3e |dba|=%.3e chi2=%.6e\n",
                     c, scale, std::abs(scale - fx.sTrue), ang,
                     (bg - fx.bgTrue).norm(), (ba - fx.baTrue).norm(), chi2);
    }
}

// P6-4 (a) — InertialOptimization(Map*, Rwg, scale, bg, ba, bMono,
// covInertial, bFixedVel, bGauss, priorG, priorA).
std::string RunInertialFullOnce(const std::string& function,
                                const std::string& fixture)
{
    if (std::getenv("EQUIV_FULL_DEBUG"))
        DebugInertialFull(kEquivFullPriorG, kEquivFullPriorA);

    equiv::ImuChainFixture fx =
        equiv::MakeImuChainFixture(equiv::ImuChainVariant::kFullInit);

    Eigen::Matrix3d Rwg = Eigen::Matrix3d::Identity();
    double scale = 1.0;
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    // Never read or written by the overload (Optimizer.cpp:3046-3228) but part
    // of the signature; sized as LocalMapping does.
    Eigen::MatrixXd covInertial = Eigen::MatrixXd::Zero(9, 9);

    const std::string inputDump =
        equiv::SerializeInertialOptimizationInput(fx, Rwg, scale);
    const std::string inputHash = equiv::Sha256Hex(inputDump);

    ORB_SLAM3::Optimizer::InertialOptimization(
        fx.map.get(), Rwg, scale, bg, ba, /*bMono=*/true, covInertial,
        /*bFixedVel=*/false, /*bGauss=*/false,
        static_cast<float>(kEquivFullPriorG),
        static_cast<float>(kEquivFullPriorA));

    return equiv::MakeInertialFullRecord(function, fixture, inputHash, fx,
                                         kEquivFullPriorG, kEquivFullPriorA,
                                         scale, Rwg, bg, ba);
}

// EQUIV_BIAS_DEBUG=1: same question for the bias-only overload — is the
// residual bias error the fixture's float32 floor or a solver stop?
void DebugInertialBias(double priorG, double priorA)
{
    equiv::ImuChainFixture fx =
        equiv::MakeImuChainFixture(equiv::ImuChainVariant::kBiasOnly);
    const ORB_SLAM3::IMU::Bias bT(
        static_cast<float>(fx.baTrue.x()), static_cast<float>(fx.baTrue.y()),
        static_cast<float>(fx.baTrue.z()), static_cast<float>(fx.bgTrue.x()),
        static_cast<float>(fx.bgTrue.y()), static_cast<float>(fx.bgTrue.z()));

    const double chi2GT = InertialGsChi2(fx, fx.sTrue, fx.RwgTrue, fx.bgTrue,
                                         fx.baTrue, StoredVelocities(fx));
    std::fprintf(stderr, "DEBUG(bias) chi2 at analytic GT = %.6e\n", chi2GT);

    {   // GT-init fixed-point probe
        equiv::ImuChainFixture fg =
            equiv::MakeImuChainFixture(equiv::ImuChainVariant::kBiasOnly);
        for (int k = 0; k < equiv::ImuChainFixture::kNumKFs; k++)
            fg.kf(k)->SetNewBias(bT);
        Eigen::Vector3d bg = fg.bgTrue, ba = fg.baTrue;
        ORB_SLAM3::Optimizer::InertialOptimization(
            fg.map.get(), bg, ba, static_cast<float>(priorG),
            static_cast<float>(priorA));
        std::fprintf(stderr,
                     "DEBUG(bias) GT-init: |dbg|=%.3e |dba|=%.3e chi2=%.6e\n",
                     (bg - fg.bgTrue).cwiseAbs().maxCoeff(),
                     (ba - fg.baTrue).cwiseAbs().maxCoeff(),
                     InertialGsChi2(fg, fg.sTrue, fg.RwgTrue, bg, ba,
                                    StoredVelocities(fg)));
    }

    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();
    for (int c = 1; c <= 5; c++) {
        ORB_SLAM3::Optimizer::InertialOptimization(
            fx.map.get(), bg, ba, static_cast<float>(priorG),
            static_cast<float>(priorA));
        std::fprintf(stderr,
                     "DEBUG(bias) call %d: |dbg|=%.3e |dba|=%.3e chi2=%.6e\n",
                     c, (bg - fx.bgTrue).cwiseAbs().maxCoeff(),
                     (ba - fx.baTrue).cwiseAbs().maxCoeff(),
                     InertialGsChi2(fx, fx.sTrue, fx.RwgTrue, bg, ba,
                                    StoredVelocities(fx)));
    }
}

// P6-4 (b) — InertialOptimization(Map*, bg, ba, priorG, priorA).
std::string RunInertialBiasOnce(const std::string& function,
                                const std::string& fixture)
{
    if (std::getenv("EQUIV_BIAS_DEBUG"))
        DebugInertialBias(kEquivBiasPriorG, kEquivBiasPriorA);

    equiv::ImuChainFixture fx =
        equiv::MakeImuChainFixture(equiv::ImuChainVariant::kBiasOnly);

    Eigen::Vector3d bg = Eigen::Vector3d::Zero();
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();

    // This overload pins gravity/scale internally; the dump records the values
    // it hard-codes so the input fingerprint stays complete.
    const Eigen::Matrix3d Rwg0 = Eigen::Matrix3d::Identity();
    const std::string inputDump =
        equiv::SerializeInertialOptimizationInput(fx, Rwg0, 1.0);
    const std::string inputHash = equiv::Sha256Hex(inputDump);

    ORB_SLAM3::Optimizer::InertialOptimization(
        fx.map.get(), bg, ba, static_cast<float>(kEquivBiasPriorG),
        static_cast<float>(kEquivBiasPriorA));

    return equiv::MakeInertialBiasRecord(function, fixture, inputHash, fx,
                                         kEquivBiasPriorG, kEquivBiasPriorA,
                                         bg, ba);
}

// P6-4 (TASK 2) — PoseInertialOptimizationLastKeyFrame.
std::string RunPoseInertialLastKfOnce(const std::string& function,
                                      const std::string& fixture)
{
    equiv::PoseInertialFixture fx = equiv::MakePoseInertialFixture();

    const std::string inputDump = equiv::SerializePoseInertialInput(fx);
    const std::string inputHash = equiv::Sha256Hex(inputDump);

    const int inliers =
        ORB_SLAM3::Optimizer::PoseInertialOptimizationLastKeyFrame(
            &fx.frame, /*bRecInit=*/false);

    if (inliers != fx.expectedInliers) {
        std::fprintf(stderr,
                     "WARNING: inlier count %d != designed %d "
                     "(fixture margin violated?)\n",
                     inliers, fx.expectedInliers);
    }
    if (!fx.frame.mpcpi) {
        std::fprintf(stderr, "FATAL: mpcpi not produced\n");
        std::abort();
    }

    const std::string rec = equiv::MakePoseInertialRecord(function, fixture,
                                                          inputHash, fx,
                                                          inliers);
    // The optimizer allocates mpcpi and the Frame never owns it (see
    // EquivFixtures.hpp); free it here now that the Hessian is serialized.
    delete fx.frame.mpcpi;
    fx.frame.mpcpi = nullptr;
    return rec;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::fprintf(stderr,
                     "usage: %s <function> <fixture>\n"
                     "  pairs: pose_optimization           mono_grid\n"
                     "         inertial_optimization       imu_chain\n"
                     "         inertial_optimization_full  imu_chain_bias\n"
                     "         inertial_optimization_bias  imu_chain_metric\n"
                     "         pose_inertial_lastkf        mono_imu_link\n",
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
    } else if (function == "inertial_optimization_full" &&
               fixture == "imu_chain_bias") {
        rec1 = RunInertialFullOnce(function, fixture);
        rec2 = RunInertialFullOnce(function, fixture);
    } else if (function == "inertial_optimization_bias" &&
               fixture == "imu_chain_metric") {
        rec1 = RunInertialBiasOnce(function, fixture);
        rec2 = RunInertialBiasOnce(function, fixture);
    } else if (function == "pose_inertial_lastkf" &&
               fixture == "mono_imu_link") {
        rec1 = RunPoseInertialLastKfOnce(function, fixture);
        rec2 = RunPoseInertialLastKfOnce(function, fixture);
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
