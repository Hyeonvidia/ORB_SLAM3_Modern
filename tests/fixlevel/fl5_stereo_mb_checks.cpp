/**
 * FL-5 stereo Frame::mb determinism check (DIVERGENCES #5; the
 * mb-before-ComputeStereoMatches init was promoted to unconditional in
 * R4a — this now asserts the fixed behavior only).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure.
 *
 * THE HISTORICAL BUG: the pinhole-stereo Frame ctor called
 * ComputeStereoMatches() BEFORE assigning mb = mbf/fx.
 * ComputeStereoMatches derives its disparity search range from that read
 * (minZ = mb; maxD = mbf/minZ; minU = uL - maxD), so every stereo frame's
 * match set depended on whatever bytes the fresh Frame object happened to
 * sit on — genuine UB and a latent nondeterminism source. Since R4a mb is
 * initialized before the call, unconditionally.
 *
 * DETERMINISTIC PROBE OF THE OLD INDETERMINACY: the test placement-news the
 * Frame into a caller-owned buffer prefilled with a chosen byte pattern
 * (0x00 -> a pre-fix mb of 0.0; 0xFF -> a pre-fix mb of NaN, which made the
 * frame silently stereo-blind). With the fix, both fill patterns must
 * produce IDENTICAL mvuRight/mvDepth, a healthy match count, and
 * mb == mbf/fx already during the matching.
 */

#include "map/Frame.hpp"

#include "camera/Pinhole.hpp"
#include "features/ORBextractor.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <random>
#include <vector>

using ORB_SLAM3::Frame;
using ORB_SLAM3::ORBextractor;
using ORB_SLAM3::Pinhole;

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::fprintf(stderr, "CHECK FAILED: %s (%s:%d)\n", #cond,        \
                         __FILE__, __LINE__);                                \
            std::_Exit(1);                                                   \
        }                                                                    \
    } while (0)

namespace {

constexpr int kW = 640;
constexpr int kH = 480;
constexpr int kDisparity = 12;   // px; well inside the healthy search range
constexpr float kFx = 400.0f;
constexpr float kBf = 40.0f;     // -> mb = 0.1 m, maxD = mbf/mb = fx = 400 px

// Deterministic textured stereo pair: blocky noise (FAST-corner rich), right
// image = left shifted kDisparity px to the LEFT (uR = uL - d, positive
// disparity), horizontal wrap at the seam.
void MakeStereoPair(cv::Mat& imL, cv::Mat& imR)
{
    imL.create(kH, kW, CV_8UC1);
    std::mt19937 rng(42);
    std::uniform_int_distribution<int> dist(0, 255);
    const int kBlock = 4;
    for (int y = 0; y < kH; y += kBlock) {
        for (int x = 0; x < kW; x += kBlock) {
            const unsigned char v = static_cast<unsigned char>(dist(rng));
            for (int yy = y; yy < std::min(y + kBlock, kH); ++yy)
                for (int xx = x; xx < std::min(x + kBlock, kW); ++xx)
                    imL.at<unsigned char>(yy, xx) = v;
        }
    }
    imR.create(kH, kW, CV_8UC1);
    for (int y = 0; y < kH; ++y)
        for (int x = 0; x < kW; ++x)
            imR.at<unsigned char>(y, x) =
                imL.at<unsigned char>(y, (x + kDisparity) % kW);
}

struct RunResult {
    int nKeys = 0;
    int nMatches = 0;
    float mb = 0.f;
    std::vector<float> uRight;
    std::vector<float> depth;
};

// Construct the stereo Frame inside a buffer prefilled with `fill`, harvest
// the observables, destroy, free. The extractors are shared across calls
// (production reuses them across frames too); their pyramid state is
// rewritten by each extraction of the SAME image, so sharing is inert.
RunResult ConstructOnPattern(unsigned char fill, const cv::Mat& imL,
                             const cv::Mat& imR, ORBextractor* extL,
                             ORBextractor* extR, cv::Mat& K, cv::Mat& dist,
                             Pinhole* cam)
{
    void* raw = nullptr;
    if (posix_memalign(&raw, 64, sizeof(Frame)) != 0) {
        std::fprintf(stderr, "posix_memalign failed\n");
        std::_Exit(1);
    }
    std::memset(raw, fill, sizeof(Frame));

    Frame* f = new (raw) Frame(imL, imR, /*timeStamp=*/0.0, extL, extR,
                               /*voc=*/nullptr, K, dist, kBf,
                               /*thDepth=*/35.0f, cam);

    RunResult r;
    r.nKeys = f->N;
    r.mb = f->mb;
    r.uRight = f->mvuRight;
    r.depth = f->mvDepth;
    for (float u : r.uRight)
        if (u >= 0.f)
            ++r.nMatches;

    f->~Frame();
    std::free(raw);
    return r;
}

}  // namespace

int main()
{
    cv::Mat imL, imR;
    MakeStereoPair(imL, imR);

    ORBextractor extL(1000, 1.2f, 8, 20, 7);
    ORBextractor extR(1000, 1.2f, 8, 20, 7);

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = kFx;
    K.at<float>(1, 1) = kFx;
    K.at<float>(0, 2) = kW / 2.0f;
    K.at<float>(1, 2) = kH / 2.0f;
    cv::Mat dist = cv::Mat::zeros(4, 1, CV_32F);
    Pinhole cam(std::vector<float>{kFx, kFx, kW / 2.0f, kH / 2.0f});

    const RunResult zero =
        ConstructOnPattern(0x00, imL, imR, &extL, &extR, K, dist, &cam);
    const RunResult ones =
        ConstructOnPattern(0xFF, imL, imR, &extL, &extR, K, dist, &cam);
    std::fprintf(stderr, "keys=%d matches=%d/%d mb=%g (expect %g)\n",
                 zero.nKeys, zero.nMatches, ones.nMatches, zero.mb,
                 kBf / kFx);
    CHECK(zero.nKeys == ones.nKeys);
    CHECK(zero.nKeys > 100);              // synthetic texture is ORB-rich
    CHECK(zero.nMatches > 20);            // healthy match count
    CHECK(zero.nMatches == ones.nMatches);
    CHECK(zero.uRight == ones.uRight);    // bit-identical match sets
    CHECK(zero.depth == ones.depth);
    CHECK(zero.mb == kBf / kFx);          // set before ComputeStereoMatches
    CHECK(ones.mb == kBf / kFx);
    std::fprintf(stderr, "ok  pattern_independent (determinism property)\n");

    std::fprintf(stderr, "fl5_stereo_mb_checks: ALL PASS\n");
    return 0;
}
