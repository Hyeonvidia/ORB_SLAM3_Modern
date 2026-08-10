/**
 * P11-F4 FL-5 stereo Frame::mb read-before-init check (DIVERGENCES #5,
 * Fix.StereoMbInit, OFF by default; docs/P11_RECON.md 2부 FL-5).
 *
 * Dependency-free assert-style binary in the tests/bit_identity/
 * extract_hash.cpp mold: no gtest, exits nonzero on the first failure.
 *
 * THE BUG: the pinhole-stereo Frame ctor calls ComputeStereoMatches()
 * (Frame.cpp:~140 upstream shape) BEFORE assigning mb = mbf/fx (:~167).
 * ComputeStereoMatches derives its disparity search range from that
 * indeterminate read (minZ = mb; maxD = mbf/minZ; minU = uL - maxD), so
 * every stereo frame's match set depends on whatever bytes the fresh Frame
 * object happens to sit on — genuine UB and a latent nondeterminism source
 * the golden baseline merely got lucky with.
 *
 * DETERMINISTIC CONSTRUCTION OF THE INDETERMINACY: the test placement-news
 * the Frame into a caller-owned buffer prefilled with a chosen byte
 * pattern, so the "indeterminate" mb read becomes a chosen value:
 *   0x00-fill -> mb = 0.0f  -> maxD = +inf -> full search range (the lucky
 *                 stack-garbage class the baseline lives on: matches found);
 *   0xFF-fill -> mb = NaN   -> maxD/minU = NaN -> every range test false ->
 *                 ZERO stereo matches (the unlucky class: silent stereo
 *                 blindness on the frame).
 * Identical synthetic stereo input both times; only the object's prior
 * memory differs — which is exactly the #5 mechanism.
 *
 * Assertions:
 *   OFF (a): 0x00-fill yields a healthy match count; 0xFF-fill yields zero
 *            matches on the same input — the two constructions DISAGREE
 *            (bug reproduced; if this starts failing, the OFF contract
 *            drifted);
 *   ON  (b): with Fix.StereoMbInit armed (public write-once FixFlags::Set,
 *            single-threaded binary — the extraction threads the ctor
 *            spawns start after Set, satisfying the header contract), both
 *            fill patterns produce IDENTICAL mvuRight/mvDepth and a healthy
 *            match count, and mb was already mbf/fx during the matching.
 */

#include "map/Frame.hpp"

#include "camera/Pinhole.hpp"
#include "core/FixFlags.hpp"
#include "features/ORBextractor.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>
#include <random>
#include <vector>

using ORB_SLAM3::FixFlags;
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

    // ---- OFF phase: the untouched process-global default is level 0 ----
    CHECK(!FixFlags::I().stereoMbInit);
    {
        const RunResult zero =
            ConstructOnPattern(0x00, imL, imR, &extL, &extR, K, dist, &cam);
        const RunResult ones =
            ConstructOnPattern(0xFF, imL, imR, &extL, &extR, K, dist, &cam);
        std::fprintf(stderr,
                     "off: keys=%d/%d matches(0x00)=%d matches(0xFF)=%d\n",
                     zero.nKeys, ones.nKeys, zero.nMatches, ones.nMatches);
        CHECK(zero.nKeys == ones.nKeys);      // extraction ignores mb
        CHECK(zero.nKeys > 100);              // synthetic texture is ORB-rich
        CHECK(zero.nMatches > 20);            // lucky-garbage class matches
        CHECK(ones.nMatches == 0);            // NaN garbage: stereo-blind
        CHECK(zero.nMatches != ones.nMatches);  // the #5 nondeterminism
        std::fprintf(stderr, "ok  a_off_pattern_dependent (bug reproduced)\n");
    }

    // ---- ON phase: arm Fix.StereoMbInit via the write-once installer ----
    {
        FixFlags f;
        f.stereoMbInit = true;
        FixFlags::Set(f);
        CHECK(FixFlags::I().stereoMbInit);
    }
    {
        const RunResult zero =
            ConstructOnPattern(0x00, imL, imR, &extL, &extR, K, dist, &cam);
        const RunResult ones =
            ConstructOnPattern(0xFF, imL, imR, &extL, &extR, K, dist, &cam);
        std::fprintf(stderr, "on : keys=%d matches=%d/%d mb=%g (expect %g)\n",
                     zero.nKeys, zero.nMatches, ones.nMatches, zero.mb,
                     kBf / kFx);
        CHECK(zero.nKeys == ones.nKeys);
        CHECK(zero.nMatches > 20);
        CHECK(zero.nMatches == ones.nMatches);
        CHECK(zero.uRight == ones.uRight);    // bit-identical match sets
        CHECK(zero.depth == ones.depth);
        CHECK(zero.mb == kBf / kFx);          // hoisted value == upstream :167
        CHECK(ones.mb == kBf / kFx);
        std::fprintf(stderr, "ok  b_on_pattern_independent (fix property)\n");
    }

    std::fprintf(stderr, "fl5_stereo_mb_checks: ALL PASS\n");
    return 0;
}
