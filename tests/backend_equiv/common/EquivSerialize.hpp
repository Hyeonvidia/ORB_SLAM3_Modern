// P6 backend-equivalence harness — canonical record serialization
// (docs/P6_DESIGN.md §B: "%.17g" floats, quaternion sign normalization
//  (w >= 0), fixed id-ascending order, sections INPUT_HASH / STAGE0_EDGES /
//  RESULT).

#ifndef EQUIV_SERIALIZE_HPP
#define EQUIV_SERIALIZE_HPP

#include "EquivFixtures.hpp"

#include "map/Frame.hpp"

#include <string>

namespace equiv {

// %.17g — round-trip exact for IEEE754 double.
std::string FormatDouble(double v);

// SHA256 hex digest of a byte string (OpenSSL).
std::string Sha256Hex(const std::string& data);

// Canonical dump of everything PoseOptimization reads from the Frame:
// N, camera intrinsics, initial pose (translation + sign-normalized
// quaternion), and per index i (ascending): world position, undistorted
// keypoint (u, v, octave) and mvuRight. Hash of this dump is the
// INPUT_HASH gate: both harness variants must serialize bit-identical
// dumps or the comparison is void.
std::string SerializePoseOptimizationInput(const ORB_SLAM3::Frame& F);

// Stage-0 (graph-construction equivalence) section. Placeholder for now:
// emits nothing. Follow-up task wires per-edge error/chi2 serialization
// through directly-instantiated edges (design §B layer 0).
std::string SerializeStage0Edges();

// RESULT section for PoseOptimization: returned inlier count, final pose
// (translation + sign-normalized quaternion, read back from the Frame),
// per-index mvbOutlier flags in ascending index order.
std::string SerializePoseOptimizationResult(int inliers,
                                            const ORB_SLAM3::Frame& F);

// Full record:
//   EQUIV_RECORD v1
//   function <function>
//   fixture <fixture>
//   INPUT_HASH <sha256 of input dump>
//   [STAGE0_EDGES section — empty for now]
//   RESULT
//   ...
//   END_RESULT
std::string MakePoseOptimizationRecord(const std::string& function,
                                       const std::string& fixture,
                                       const std::string& inputHash,
                                       int inliers,
                                       const ORB_SLAM3::Frame& F);

// Canonical dump of everything InertialOptimization(Map*, Rwg&, scale&)
// reads from the fixture: per-KF stored state (Tcw, velocity, biases — read
// back through the same accessors the optimizer uses), the IMU calibration,
// the initial (Rwg, scale) handed to the optimizer, and per-interval
// preintegration output (dT, ΔR/ΔV/ΔP, bias Jacobians, 9x9 covariance
// block). Hash of this dump is the INPUT_HASH gate.
std::string SerializeInertialOptimizationInput(const ImuChainFixture& fx,
                                               const Eigen::Matrix3d& Rwg0,
                                               double scale0);

// RESULT section for InertialOptimization: final scale (double, %.17g),
// final Rwg (sign-normalized quaternion) plus the gravity direction
// Rwg*(0,0,-1), and per-KF velocities in mnId order (fixed vertices in this
// variant — doubles as an "inputs not clobbered" check).
std::string SerializeInertialOptimizationResult(double scale,
                                                const Eigen::Matrix3d& Rwg,
                                                const ImuChainFixture& fx);

// Full record. GT_scale / GT_gdir header lines (fixture ground truth) let
// compare.py --gt-check assert analytic recovery without re-deriving the
// fixture.
std::string MakeInertialOptimizationRecord(const std::string& function,
                                           const std::string& fixture,
                                           const std::string& inputHash,
                                           const ImuChainFixture& fx,
                                           double scale,
                                           const Eigen::Matrix3d& Rwg);

// ---------------------------------------------------------------------------
// P6-4 (a): InertialOptimization(Map*, Rwg, scale, bg, ba, bMono, covInertial,
// bFixedVel, bGauss, priorG, priorA) — the full IMU-init overload.
// RESULT adds bg/ba (double, %.17g) to the scale/Rwg rows; the per-KF
// velocities are now OPTIMIZED outputs (VertexVelocity is free), stored back
// as float by the optimizer.
// Header carries GT_scale / GT_gdir / GT_bg / GT_ba plus the prior weights
// actually used, since the priors bias the analytic optimum (see run_equiv.sh
// / compare.py notes).
std::string MakeInertialFullRecord(const std::string& function,
                                   const std::string& fixture,
                                   const std::string& inputHash,
                                   const ImuChainFixture& fx,
                                   double priorG, double priorA,
                                   double scale, const Eigen::Matrix3d& Rwg,
                                   const Eigen::Vector3d& bg,
                                   const Eigen::Vector3d& ba);

// P6-4 (b): InertialOptimization(Map*, bg, ba, priorG, priorA) — the bias-only
// refinement overload (gravity pinned to identity, scale pinned to 1.0).
std::string MakeInertialBiasRecord(const std::string& function,
                                   const std::string& fixture,
                                   const std::string& inputHash,
                                   const ImuChainFixture& fx,
                                   double priorG, double priorA,
                                   const Eigen::Vector3d& bg,
                                   const Eigen::Vector3d& ba);

// ---------------------------------------------------------------------------
// P6-4 (TASK 2): PoseInertialOptimizationLastKeyFrame.

// Canonical dump of everything the function reads: Frame geometry and initial
// IMU state, per-index observation (world point, undistorted keypoint, octave,
// mvuRight, mTrackDepth), the inverse level-sigma table, the fixed KeyFrame
// state, and the full preintegration (including the 15x15 covariance, because
// blocks (9,9) and (12,12) drive the EdgeGyroRW / EdgeAccRW information).
std::string SerializePoseInertialInput(const PoseInertialFixture& fx);

// RESULT section: returned inlier count, optimized Frame pose / velocity /
// bias (all read back through the float storage path the optimizer writes),
// per-index outlier flags, and the 15x15 ConstraintPoseImu Hessian one row per
// line — the GetHessian marginalization contract (PROJECT_PLAN risk #4).
std::string SerializePoseInertialResult(int inliers,
                                        const ORB_SLAM3::Frame& F);

// Full record; GT_pose_* / GT_vel header lines carry the analytic optimum.
std::string MakePoseInertialRecord(const std::string& function,
                                   const std::string& fixture,
                                   const std::string& inputHash,
                                   const PoseInertialFixture& fx,
                                   int inliers);

}  // namespace equiv

#endif  // EQUIV_SERIALIZE_HPP
