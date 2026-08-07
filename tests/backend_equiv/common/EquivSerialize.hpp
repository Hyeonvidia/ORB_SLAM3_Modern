// P6 backend-equivalence harness — canonical record serialization
// (docs/P6_DESIGN.md §B: "%.17g" floats, quaternion sign normalization
//  (w >= 0), fixed id-ascending order, sections INPUT_HASH / STAGE0_EDGES /
//  RESULT).

#ifndef EQUIV_SERIALIZE_HPP
#define EQUIV_SERIALIZE_HPP

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

}  // namespace equiv

#endif  // EQUIV_SERIALIZE_HPP
