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

// P11-V vocabulary binary-cache round-trip check (extract_hash mold:
// dependency-light assert-style binary, nonzero exit = FAIL).
//
// Loads the canonical text vocabulary into instance A, writes the binary
// cache (saveToBinaryFile), loads it into a fresh instance B
// (loadFromBinaryFile), then compares the two in-memory vocabularies
// node-by-node: k/L/scoring/weighting, node count, word count, and every
// node's id/parent/children/word-registration/descriptor bytes/weight
// (weight compared bitwise on the IEEE754 representation). This is the
// "bit-identical by construction" evidence for DIVERGENCES #27: the
// binary file stores the exact values the text parser produced, including
// the text loader's trailing-blank-line quirk node if present.
//
// Usage: vocab_roundtrip <ORBvoc.txt> [cache.bin]
//   cache.bin defaults to <ORBvoc.txt>.roundtrip.bin (removed on PASS).

// R3: the loaders under test moved from the vendored DBoW2 fork into the
// OrbVocabulary wrapper over the upstream submodule; same checks apply.
#include "recognition/OrbVocabulary.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

namespace {

// Test-only subclass: the node tree is protected, and the whole point of
// this check is exact node-level state, so expose it read-only here.
class VocUnderTest : public ORB_SLAM3::OrbVocabulary
{
public:
    typedef ORB_SLAM3::OrbVocabulary::Node Node;
    const std::vector<Node>& nodes() const { return m_nodes; }
    const std::vector<Node*>& words() const { return m_words; }
};

int gFailures = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        if(!(cond)) {                                                       \
            std::fprintf(stderr, "FAIL(%d): %s — ", __LINE__, #cond);       \
            std::fprintf(stderr, __VA_ARGS__);                              \
            std::fprintf(stderr, "\n");                                     \
            ++gFailures;                                                    \
        }                                                                   \
    } while(0)

double seconds_since(std::chrono::steady_clock::time_point t0)
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(
        std::chrono::steady_clock::now() - t0).count();
}

bool word_registered(const VocUnderTest& v, const VocUnderTest::Node& n)
{
    return !v.words().empty() && n.word_id < (DBoW2::WordId)v.words().size()
           && v.words()[n.word_id] == &n;
}

} // namespace

int main(int argc, char** argv)
{
    if(argc < 2 || argc > 3) {
        std::fprintf(stderr, "usage: %s <ORBvoc.txt> [cache.bin]\n", argv[0]);
        return 2;
    }
    const std::string txtPath = argv[1];
    const std::string binPath =
        (argc == 3) ? argv[2] : txtPath + ".roundtrip.bin";

    // --- A: canonical text load -------------------------------------------
    VocUnderTest vocA;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    if(!vocA.loadFromTextFile(txtPath)) {
        std::fprintf(stderr, "FAIL: cannot load text vocabulary %s\n",
                     txtPath.c_str());
        return 1;
    }
    std::printf("text load:   %.3fs  (%zu nodes incl. root, %u words)\n",
                seconds_since(t0), vocA.nodes().size(), vocA.size());

    // --- save binary cache ------------------------------------------------
    t0 = std::chrono::steady_clock::now();
    if(!vocA.saveToBinaryFile(binPath)) {
        std::fprintf(stderr, "FAIL: cannot write binary cache %s\n",
                     binPath.c_str());
        return 1;
    }
    std::printf("binary save: %.3fs  (%s)\n", seconds_since(t0),
                binPath.c_str());

    // --- B: binary load into a fresh instance -----------------------------
    VocUnderTest vocB;
    t0 = std::chrono::steady_clock::now();
    if(!vocB.loadFromBinaryFile(binPath)) {
        std::fprintf(stderr, "FAIL: cannot load binary cache %s\n",
                     binPath.c_str());
        return 1;
    }
    std::printf("binary load: %.3fs\n", seconds_since(t0));

    // --- compare ----------------------------------------------------------
    CHECK(vocA.getBranchingFactor() == vocB.getBranchingFactor(),
          "k %d vs %d", vocA.getBranchingFactor(), vocB.getBranchingFactor());
    CHECK(vocA.getDepthLevels() == vocB.getDepthLevels(),
          "L %d vs %d", vocA.getDepthLevels(), vocB.getDepthLevels());
    CHECK(vocA.getScoringType() == vocB.getScoringType(),
          "scoring %d vs %d", (int)vocA.getScoringType(),
          (int)vocB.getScoringType());
    CHECK(vocA.getWeightingType() == vocB.getWeightingType(),
          "weighting %d vs %d", (int)vocA.getWeightingType(),
          (int)vocB.getWeightingType());
    CHECK(vocA.size() == vocB.size(), "word count %u vs %u",
          vocA.size(), vocB.size());
    CHECK(vocA.nodes().size() == vocB.nodes().size(),
          "node count %zu vs %zu", vocA.nodes().size(), vocB.nodes().size());
    CHECK(vocA.words().size() == vocB.words().size(),
          "m_words size %zu vs %zu", vocA.words().size(), vocB.words().size());

    const size_t nNodes =
        vocA.nodes().size() < vocB.nodes().size() ? vocA.nodes().size()
                                                  : vocB.nodes().size();
    size_t nWordNodes = 0;
    for(size_t i = 0; i < nNodes && gFailures < 20; ++i) {
        const VocUnderTest::Node& a = vocA.nodes()[i];
        const VocUnderTest::Node& b = vocB.nodes()[i];

        CHECK(a.id == b.id, "node %zu: id %u vs %u", i, a.id, b.id);
        CHECK(a.parent == b.parent, "node %zu: parent %u vs %u",
              i, a.parent, b.parent);
        CHECK(a.children == b.children,
              "node %zu: children differ (%zu vs %zu entries)",
              i, a.children.size(), b.children.size());

        // weight: bitwise on the IEEE754 representation (exact, and safe
        // for any NaN a corrupt parse could have produced)
        uint64_t wa, wb;
        std::memcpy(&wa, &a.weight, 8);
        std::memcpy(&wb, &b.weight, 8);
        CHECK(wa == wb, "node %zu: weight %.17g vs %.17g", i, a.weight,
              b.weight);

        // descriptor: byte-exact (empty allowed for the root only)
        const bool aEmpty = a.descriptor.empty();
        const bool bEmpty = b.descriptor.empty();
        if(i == 0) {
            CHECK(aEmpty == bEmpty, "root: descriptor emptiness %d vs %d",
                  (int)aEmpty, (int)bEmpty);
        } else {
            CHECK(!aEmpty && !bEmpty, "node %zu: empty descriptor (%d/%d)",
                  i, (int)aEmpty, (int)bEmpty);
            if(!aEmpty && !bEmpty) {
                CHECK(a.descriptor.rows == 1 && b.descriptor.rows == 1 &&
                          a.descriptor.cols == DBoW2::FORB::L &&
                          b.descriptor.cols == DBoW2::FORB::L &&
                          a.descriptor.type() == CV_8U &&
                          b.descriptor.type() == CV_8U,
                      "node %zu: descriptor shape/type mismatch", i);
                CHECK(std::memcmp(a.descriptor.ptr<unsigned char>(),
                                  b.descriptor.ptr<unsigned char>(),
                                  DBoW2::FORB::L) == 0,
                      "node %zu: descriptor bytes differ", i);
            }
        }

        // word registration (m_words membership), not children.empty():
        // the text loader's trailing-blank-line quirk node is childless
        // but must NOT be a word — see TemplatedVocabulary.h format note
        const bool aWord = word_registered(vocA, a);
        const bool bWord = word_registered(vocB, b);
        CHECK(aWord == bWord, "node %zu: word registration %d vs %d",
              i, (int)aWord, (int)bWord);
        if(aWord && bWord) {
            ++nWordNodes;
            CHECK(a.word_id == b.word_id, "node %zu: word_id %u vs %u",
                  i, a.word_id, b.word_id);
        }
    }

    // m_words must index the same node ids in the same order
    const size_t nWords =
        vocA.words().size() < vocB.words().size() ? vocA.words().size()
                                                  : vocB.words().size();
    for(size_t w = 0; w < nWords && gFailures < 20; ++w) {
        CHECK(vocA.words()[w] != NULL && vocB.words()[w] != NULL,
              "word %zu: null entry", w);
        if(vocA.words()[w] && vocB.words()[w])
            CHECK(vocA.words()[w]->id == vocB.words()[w]->id,
                  "word %zu: node id %u vs %u", w, vocA.words()[w]->id,
                  vocB.words()[w]->id);
    }

    if(gFailures) {
        std::fprintf(stderr, "vocab_roundtrip FAIL (%d mismatches%s)\n",
                     gFailures, gFailures >= 20 ? ", capped" : "");
        return 1;
    }

    std::printf("vocab_roundtrip PASS (%zu nodes, %zu word nodes, "
                "%u words)\n", nNodes, nWordNodes, vocA.size());
    std::remove(binPath.c_str());
    return 0;
}
