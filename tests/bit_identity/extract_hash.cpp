/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

// P4 bit-identity harness: hashes the exact extraction output of
// ORBextractor for a fixed image + parameters. Refactors of the feature
// layer must keep these hashes unchanged (same machine, same container
// image — OpenCV SIMD dispatch makes hashes non-portable across builds).
//
// Serialized stream (native endianness, same-machine comparison only):
//   int32 nkeypoints, int32 monoIndex(return value of operator()),
//   per keypoint: pt.x pt.y size angle response as raw IEEE754 float bits,
//                 int32 octave, int32 class_id,
//   descriptor Mat bytes (N x 32, CV_8UC1).
// The keypoint ORDER is part of the contract (vLapping reorders in-place).
//
// Usage: extract_hash <image> <nfeatures> <scale> <nlevels> <iniTh> <minTh> <lap0> <lap1>
// Prints two hash lines (run twice on one instance — determinism self-check).

#include "features/ORBextractor.hpp"

#include <opencv2/imgcodecs.hpp>
#include <openssl/sha.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

using namespace ORB_SLAM3;

static void put(std::vector<unsigned char>& buf, const void* p, size_t n)
{
    const unsigned char* c = static_cast<const unsigned char*>(p);
    buf.insert(buf.end(), c, c + n);
}

static std::string runOnce(ORBextractor& ext, const cv::Mat& im, std::vector<int> vLapping)
{
    std::vector<cv::KeyPoint> vKeys;
    cv::Mat desc;
    int monoIndex = ext(im, cv::Mat(), vKeys, desc, vLapping);

    std::vector<unsigned char> buf;
    int32_t n = static_cast<int32_t>(vKeys.size());
    put(buf, &n, 4);
    int32_t mi = monoIndex;
    put(buf, &mi, 4);
    for(const cv::KeyPoint& kp : vKeys){
        put(buf, &kp.pt.x, 4); put(buf, &kp.pt.y, 4);
        put(buf, &kp.size, 4); put(buf, &kp.angle, 4);
        put(buf, &kp.response, 4);
        int32_t oc = kp.octave, ci = kp.class_id;
        put(buf, &oc, 4); put(buf, &ci, 4);
    }
    if(!desc.empty()){
        CV_Assert(desc.type() == CV_8UC1 && desc.isContinuous());
        put(buf, desc.data, static_cast<size_t>(desc.rows) * desc.cols);
    }

    unsigned char md[SHA256_DIGEST_LENGTH];
    SHA256(buf.data(), buf.size(), md);
    char hex[2*SHA256_DIGEST_LENGTH+1];
    for(int i = 0; i < SHA256_DIGEST_LENGTH; i++) std::snprintf(hex+2*i, 3, "%02x", md[i]);
    return std::string(hex);
}

int main(int argc, char** argv)
{
    if(argc != 9){
        std::fprintf(stderr, "usage: %s <image> <nfeatures> <scale> <nlevels> <iniTh> <minTh> <lap0> <lap1>\n", argv[0]);
        return 2;
    }
    cv::Mat im = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if(im.empty()){ std::fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }

    ORBextractor ext(std::atoi(argv[2]), std::atof(argv[3]), std::atoi(argv[4]),
                     std::atoi(argv[5]), std::atoi(argv[6]));
    std::vector<int> vLapping = {std::atoi(argv[7]), std::atoi(argv[8])};

    std::string h1 = runOnce(ext, im, vLapping);
    std::string h2 = runOnce(ext, im, vLapping);
    std::printf("%s\n%s\n", h1.c_str(), h2.c_str());
    return h1 == h2 ? 0 : 1;   // nonzero = nondeterminism detected
}
