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

#ifndef ORB_SLAM3_RECOGNITION_ORBVOCABULARY_HPP
#define ORB_SLAM3_RECOGNITION_ORBVOCABULARY_HPP

#include <string>

#include "DBoW2/FORB.h"
#include "DBoW2/TemplatedVocabulary.h"

// R3 one-TU instantiation: every member of the ORB vocabulary template is
// instantiated once in src/recognition/OrbVocabulary.cpp; the many TUs that
// include this header (Frame, KeyFrame, Tracking, ...) only reference it.
extern template class DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor,
                                                 DBoW2::FORB>;

namespace ORB_SLAM3
{

/**
 * @brief ORB visual vocabulary: upstream DBoW2 template + the ORB-SLAM
 *        fork's file formats as a wrapper layer.
 *
 * R3 (docs/REFACTOR_PLAN.md): DBoW2 now builds from the pinned upstream
 * dorian3d submodule (third_party/DBoW2 @ master 3924753), which is never
 * patched — the g2o/OrbLevenberg precedent. The fork deltas found by the
 * R3 audit live HERE instead of in a patched vendored tree:
 *
 *  - loadFromTextFile()/saveToTextFile(): the plain-text vocabulary format
 *    is an ORB-SLAM addition (Raúl Mur-Artal, Aug 2015) — upstream DBoW2
 *    only has the cv::FileStorage save/load. ORBvoc.txt is this format,
 *    and SaveAtlas checksums the TEXT file, so its semantics (including
 *    the trailing-blank-line quirk node) are preserved verbatim.
 *  - loadFromBinaryFile()/saveToBinaryFile(): our P11-V host-local binary
 *    cache of the text file (fast startup, DIVERGENCES #27). Format v1,
 *    byte-identical to the pre-R3 vendored implementation, so existing
 *    .bin caches remain valid.
 *
 * The audit found no other behavioral fork deltas on the load/transform
 * path: the fork's FORB tweaks (int vs double distance return, 32-bit vs
 * 64-bit popcount) yield identical Hamming values, and the fork's
 * DUtils-seeded kmeans++ only affects vocabulary TRAINING, which ORB-SLAM
 * never does at runtime. Hence upstream FORB is used as-is.
 */
class OrbVocabulary
    : public DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
{
public:
    /**
     * Loads the vocabulary from an ORB-SLAM text file (e.g. ORBvoc.txt).
     * @return true iff the file was accepted
     */
    bool loadFromTextFile(const std::string &filename);

    /** Saves the vocabulary into an ORB-SLAM text file. */
    void saveToTextFile(const std::string &filename) const;

    /**
     * Loads the vocabulary from a binary cache file written by
     * saveToBinaryFile. On any failure the vocabulary contents are
     * unspecified; callers must fall back to loadFromTextFile (which
     * clears and reloads everything).
     * @return true iff the whole file was read and accepted
     */
    bool loadFromBinaryFile(const std::string &filename);

    /**
     * Saves the vocabulary into a binary cache file (format note in the
     * implementation). Best-effort: returns false instead of throwing
     * when the file cannot be written.
     * @return true iff the whole file was written
     */
    bool saveToBinaryFile(const std::string &filename) const;
};

// Historical name used throughout the codebase; OrbVocabulary is the R3 class.
using ORBVocabulary = OrbVocabulary;

} // namespace ORB_SLAM3

#endif // ORB_SLAM3_RECOGNITION_ORBVOCABULARY_HPP
