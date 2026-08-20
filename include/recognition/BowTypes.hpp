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

#ifndef ORB_SLAM3_RECOGNITION_BOWTYPES_HPP
#define ORB_SLAM3_RECOGNITION_BOWTYPES_HPP

// R3 (docs/REFACTOR_PLAN.md): DBoW2 comes from the pinned upstream dorian3d
// submodule (third_party/DBoW2), which is never patched — the OrbLevenberg
// precedent. The ORB-SLAM fork of DBoW2 added intrusive Boost serialization
// to BowVector/FeatureVector (used by the Atlas/KeyFrame map save/load);
// upstream lacks it, so it is restored here NON-intrusively. Both classes
// publicly derive from std::map, so base_object<> needs no friend access,
// and the archive layout matches the fork's intrusive member serialize()
// (same base-map payload, same implicit version/tracking traits) — .osa
// files stay compatible.

#include "DBoW2/BowVector.h"
#include "DBoW2/FeatureVector.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>

namespace boost
{
namespace serialization
{

template<class Archive>
void serialize(Archive &ar, DBoW2::BowVector &v, const unsigned int /*version*/)
{
    ar & base_object<std::map<DBoW2::WordId, DBoW2::WordValue> >(v);
}

template<class Archive>
void serialize(Archive &ar, DBoW2::FeatureVector &v, const unsigned int /*version*/)
{
    ar & base_object<std::map<DBoW2::NodeId, std::vector<unsigned int> > >(v);
}

} // namespace serialization
} // namespace boost

#endif // ORB_SLAM3_RECOGNITION_BOWTYPES_HPP
