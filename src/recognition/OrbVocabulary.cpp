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

// R3: ORB-SLAM's vocabulary file formats, extracted from the retired
// vendored DBoW2 fork (Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h) into
// this wrapper layer over the unpatched upstream submodule. The text
// loader/saver are ported verbatim (Raúl Mur-Artal's Aug 2015 additions,
// quirks included — they define what ORBvoc.txt means); the binary cache
// is our P11-V code, format v1 unchanged. See include/recognition/
// OrbVocabulary.hpp for the audit summary and rationale.

#include "recognition/OrbVocabulary.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// The single instantiation of the vocabulary template for the whole build
// (declared extern in the header).
template class DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor,
                                          DBoW2::FORB>;

namespace ORB_SLAM3
{

using DBoW2::FORB;
using DBoW2::WordId;

// ---------------------------------------------------------------------------
// Text format (ORB-SLAM fork addition, ported verbatim).
//
// Line 1: "k L scoring weighting"; then one line per node in node-id order:
// "parent isLeaf d0 .. d31 weight". Quirks preserved on purpose:
//  * a missing file is rejected through the parameter sanity window (the
//    failed stream extracts zeros), not through an is_open() check;
//  * the while(!eof) loop turns a trailing blank line into one childless
//    non-word node — the binary cache round-trips that quirk node, so the
//    two load paths stay bit-identical (tests/vocab/vocab_roundtrip.cpp).
// ---------------------------------------------------------------------------

bool OrbVocabulary::loadFromTextFile(const std::string &filename)
{
    std::ifstream f;
    f.open(filename.c_str());

    if(f.eof())
        return false;

    m_words.clear();
    m_nodes.clear();

    std::string s;
    getline(f,s);
    std::stringstream ss;
    ss << s;
    ss >> m_k;
    ss >> m_L;
    int n1, n2;
    ss >> n1;
    ss >> n2;

    if(m_k<0 || m_k>20 || m_L<1 || m_L>10 || n1<0 || n1>5 || n2<0 || n2>3)
    {
        std::cerr << "Vocabulary loading failure: This is not a correct text file!" << std::endl;
        return false;
    }

    m_scoring = (DBoW2::ScoringType)n1;
    m_weighting = (DBoW2::WeightingType)n2;
    createScoringObject();

    // nodes
    int expected_nodes =
    static_cast<int>(((std::pow(static_cast<double>(m_k), static_cast<double>(m_L) + 1) - 1)/(m_k - 1)));
    m_nodes.reserve(expected_nodes);

    m_words.reserve(std::pow(static_cast<double>(m_k), static_cast<double>(m_L) + 1));

    m_nodes.resize(1);
    m_nodes[0].id = 0;
    while(!f.eof())
    {
        std::string snode;
        getline(f,snode);
        std::stringstream ssnode;
        ssnode << snode;

        int nid = m_nodes.size();
        m_nodes.resize(m_nodes.size()+1);
        m_nodes[nid].id = nid;

        int pid;
        ssnode >> pid;
        m_nodes[nid].parent = pid;
        m_nodes[pid].children.push_back(nid);

        int nIsLeaf;
        ssnode >> nIsLeaf;

        std::stringstream ssd;
        for(int iD=0;iD<FORB::L;iD++)
        {
            std::string sElement;
            ssnode >> sElement;
            ssd << sElement << " ";
        }
        FORB::fromString(m_nodes[nid].descriptor, ssd.str());

        ssnode >> m_nodes[nid].weight;

        if(nIsLeaf>0)
        {
            int wid = m_words.size();
            m_words.resize(wid+1);

            m_nodes[nid].word_id = wid;
            m_words[wid] = &m_nodes[nid];
        }
        else
        {
            m_nodes[nid].children.reserve(m_k);
        }
    }

    return true;
}

// ---------------------------------------------------------------------------

void OrbVocabulary::saveToTextFile(const std::string &filename) const
{
    std::fstream f;
    f.open(filename.c_str(), std::ios_base::out);
    // fork quirk kept: an extra space token after L (harmless — the loader's
    // stream extraction is whitespace-insensitive)
    f << m_k << " " << m_L << " " << " " << m_scoring << " " << m_weighting << std::endl;

    for(size_t i=1; i<m_nodes.size(); i++)
    {
        const Node& node = m_nodes[i];

        f << node.parent << " ";
        if(node.isLeaf())
            f << 1 << " ";
        else
            f << 0 << " ";

        f << FORB::toString(node.descriptor) << " " << static_cast<double>(node.weight) << std::endl;
    }

    f.close();
}

// ---------------------------------------------------------------------------
// Binary vocabulary cache format (P11-V, DIVERGENCES #27) — format v1,
// unchanged by R3 so pre-R3 .bin caches keep loading.
// Native endianness, no padding — a host-local derived cache of the
// canonical text file, NOT a portable interchange format.
//
//   header (40 bytes):
//     char[8]   magic "DBOW2BIN"
//     int32     format version (1)
//     int32     m_k
//     int32     m_L
//     int32     scoring type
//     int32     weighting type
//     uint64    node record count (m_nodes.size()-1, root excluded)
//     int32     F::L (descriptor length in bytes)
//   per node record (13 + F::L bytes), in node id order (ids 1..count):
//     int32     parent node id
//     uint8     1 if the node is a word (leaf), 0 otherwise
//     uint8[F::L] descriptor bytes (assumes TDescriptor = cv::Mat 1xF::L
//               CV_8U, as created by F::fromString)
//     float64   weight
//
// The word flag records "registered in m_words", NOT children.empty():
// loadFromTextFile can append a childless non-word node from a trailing
// blank line, and that in-memory quirk must survive the round trip so a
// binary load is bit-identical to the text load that produced the cache.
// ---------------------------------------------------------------------------

bool OrbVocabulary::saveToBinaryFile(const std::string &filename) const
{
    std::ofstream f;
    f.open(filename.c_str(), std::ios_base::out | std::ios_base::binary);
    if(!f.is_open())
        return false;

    const char magic[8] = {'D','B','O','W','2','B','I','N'};
    const int32_t version = 1;
    const int32_t k = m_k;
    const int32_t L = m_L;
    const int32_t scoring = (int32_t)m_scoring;
    const int32_t weighting = (int32_t)m_weighting;
    const uint64_t count = m_nodes.empty() ? 0 : (uint64_t)(m_nodes.size()-1);
    const int32_t FL = FORB::L;

    f.write(magic, 8);
    f.write((const char*)&version, 4);
    f.write((const char*)&k, 4);
    f.write((const char*)&L, 4);
    f.write((const char*)&scoring, 4);
    f.write((const char*)&weighting, 4);
    f.write((const char*)&count, 8);
    f.write((const char*)&FL, 4);

    std::vector<char> record(13 + FORB::L);
    for(size_t i=1; i<m_nodes.size(); i++)
    {
        const Node& node = m_nodes[i];
        char *p = &record[0];

        const int32_t pid = (int32_t)node.parent;
        std::memcpy(p, &pid, 4); p += 4;

        // word membership, not children.empty() (see format note above)
        unsigned char isWord = 0;
        if(!m_words.empty() && node.word_id < (WordId)m_words.size()
           && m_words[node.word_id] == &node)
            isWord = 1;
        *p++ = (char)isWord;

        if(node.descriptor.empty())
            std::memset(p, 0, FORB::L);
        else
            std::memcpy(p, node.descriptor.ptr<unsigned char>(), FORB::L);
        p += FORB::L;

        const double w = static_cast<double>(node.weight);
        std::memcpy(p, &w, 8);

        f.write(&record[0], record.size());
    }

    f.flush();
    const bool ok = f.good();
    f.close();
    return ok;
}

// ---------------------------------------------------------------------------

bool OrbVocabulary::loadFromBinaryFile(const std::string &filename)
{
    std::ifstream f;
    f.open(filename.c_str(), std::ios_base::in | std::ios_base::binary);
    if(!f.is_open())
        return false;

    f.seekg(0, std::ios_base::end);
    const uint64_t file_size = (uint64_t)f.tellg();
    f.seekg(0, std::ios_base::beg);

    char magic[8];
    int32_t version, k, L, scoring, weighting, FL;
    uint64_t count;
    f.read(magic, 8);
    f.read((char*)&version, 4);
    f.read((char*)&k, 4);
    f.read((char*)&L, 4);
    f.read((char*)&scoring, 4);
    f.read((char*)&weighting, 4);
    f.read((char*)&count, 8);
    f.read((char*)&FL, 4);
    if(!f.good())
        return false;

    if(std::memcmp(magic, "DBOW2BIN", 8) != 0 || version != 1 || FL != FORB::L)
        return false;

    // same parameter sanity window as loadFromTextFile
    if(k<0 || k>20 || L<1 || L>10 || scoring<0 || scoring>5
       || weighting<0 || weighting>3)
        return false;

    // the file must be exactly header + count fixed-size records; this
    // also rejects absurd counts before any allocation happens
    const size_t record_size = 13 + static_cast<size_t>(FORB::L);
    if(count == 0 || file_size != 40 + count * (uint64_t)record_size)
        return false;

    m_k = k;
    m_L = L;
    m_scoring = (DBoW2::ScoringType)scoring;
    m_weighting = (DBoW2::WeightingType)weighting;
    createScoringObject();

    m_words.clear();
    m_nodes.clear();

    // one buffered read of all fixed-size records, then a linear fill
    const size_t total = static_cast<size_t>(count) * record_size;
    std::vector<char> buf(total);
    f.read(&buf[0], total);
    if(static_cast<size_t>(f.gcount()) != total)
        return false;

    // exact reserve: m_words stores pointers into m_nodes, so m_nodes
    // must never reallocate during the fill
    m_nodes.reserve(count + 1);
    m_words.reserve(count);

    m_nodes.resize(1);
    m_nodes[0].id = 0;

    const char *p = &buf[0];
    for(uint64_t i=0; i<count; i++)
    {
        const int nid = static_cast<int>(m_nodes.size());
        m_nodes.resize(m_nodes.size()+1);
        m_nodes[nid].id = nid;

        int32_t pid;
        std::memcpy(&pid, p, 4); p += 4;
        if(pid < 0 || pid >= nid)   // parents always precede children
            return false;
        m_nodes[nid].parent = pid;
        m_nodes[pid].children.push_back(nid);

        const unsigned char isWord = (unsigned char)*p; p += 1;

        // same construction as FORB::fromString (cv::Mat 1xF::L CV_8U)
        m_nodes[nid].descriptor.create(1, FORB::L, CV_8U);
        std::memcpy(m_nodes[nid].descriptor.ptr<unsigned char>(), p, FORB::L);
        p += FORB::L;

        double w;
        std::memcpy(&w, p, 8); p += 8;
        m_nodes[nid].weight = w;

        if(isWord)
        {
            const int wid = static_cast<int>(m_words.size());
            m_words.resize(wid+1);
            m_nodes[nid].word_id = wid;
            m_words[wid] = &m_nodes[nid];
        }
        else
        {
            m_nodes[nid].children.reserve(m_k);
        }
    }

    return true;
}

} // namespace ORB_SLAM3
