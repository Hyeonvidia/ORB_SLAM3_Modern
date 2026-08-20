/*
 * File: Random.h (trimmed)
 * Project: DUtils library, DLib — Dorian Galvez-Lopez, April 2010
 * Description: manages pseudo-random numbers
 * License: see the original DLib distribution (BSD-style); this subset was
 *          shipped by ORB-SLAM inside its DBoW2 fork.
 *
 * R3 (docs/REFACTOR_PLAN.md): relocated from the retired vendored
 * Thirdparty/DBoW2/DUtils into our tree — the upstream DBoW2 submodule does
 * not ship DUtils (it lives in the separate DLib repo), and the only users
 * are OUR RANSAC solvers (Sim3Solver, TwoViewReconstruction, MLPnPsolver).
 * The rand()-based generator is kept bit-for-bit: replacing it with <random>
 * would change every RANSAC draw sequence. Trimmed to what the codebase
 * uses: the unused UnrepeatedRandomizer and RandomGaussianValue are gone
 * (git history of the vendored tree preserves them).
 */

#ifndef ORB_SLAM3_UTIL_RANDOM_H
#define ORB_SLAM3_UTIL_RANDOM_H

#include <cstdlib>

namespace DUtils {

/// Functions to generate pseudo-random numbers
class Random
{
public:
    /**
     * Sets the random number seed to the current time
     */
    static void SeedRand();

    /**
     * Sets the random number seed to the current time only the first
     * time this function is called
     */
    static void SeedRandOnce();

    /**
     * Sets the given random number seed
     * @param seed
     */
    static void SeedRand(int seed);

    /**
     * Sets the given random number seed only the first time this function
     * is called
     * @param seed
     */
    static void SeedRandOnce(int seed);

    /**
     * Returns a random number in the range [0..1]
     * @return random T number in [0..1]
     */
    template <class T>
    static T RandomValue(){
        return (T)rand()/(T)RAND_MAX;
    }

    /**
     * Returns a random number in the range [min..max]
     * @param min
     * @param max
     * @return random T number in [min..max]
     */
    template <class T>
    static T RandomValue(T min, T max){
        return Random::RandomValue<T>() * (max - min) + min;
    }

    /**
     * Returns a random int in the range [min..max]
     * @param min
     * @param max
     * @return random int in [min..max]
     */
    static int RandomInt(int min, int max);

private:
    /// If SeedRandOnce() or SeedRandOnce(int) have already been called
    static bool m_already_seeded;
};

} // namespace DUtils

#endif // ORB_SLAM3_UTIL_RANDOM_H
