/*
 * File: Random.cpp (trimmed)
 * Project: DUtils library, DLib — Dorian Galvez-Lopez, April 2010
 * Description: manages pseudo-random numbers
 * License: see the original DLib distribution (BSD-style); this subset was
 *          shipped by ORB-SLAM inside its DBoW2 fork.
 *
 * R3: relocated from the retired vendored Thirdparty/DBoW2/DUtils (see
 * include/util/Random.h). The seeded path and RandomInt are bit-for-bit
 * the fork's code; only the time-based SeedRand() — unused by this
 * codebase, every call site seeds with SeedRandOnce(0) or relies on the
 * process-global rand() state — now reads the clock via <chrono> instead
 * of dragging DUtils::Timestamp along.
 */

#include "util/Random.h"

#include <chrono>
#include <cstdlib>

namespace DUtils {

bool Random::m_already_seeded = false;

void Random::SeedRand(){
    // seconds since epoch, matching the old Timestamp::getFloatTime() seed
    const double now = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    srand(static_cast<unsigned>(now));
}

void Random::SeedRandOnce()
{
    if(!m_already_seeded)
    {
        Random::SeedRand();
        m_already_seeded = true;
    }
}

void Random::SeedRand(int seed)
{
    srand(seed);
}

void Random::SeedRandOnce(int seed)
{
    if(!m_already_seeded)
    {
        Random::SeedRand(seed);
        m_already_seeded = true;
    }
}

int Random::RandomInt(int min, int max){
    int d = max - min + 1;
    return int((static_cast<double>(rand())/(static_cast<double>(RAND_MAX) + 1.0)) * d) + min;
}

} // namespace DUtils
