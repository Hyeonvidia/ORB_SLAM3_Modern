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


#ifndef KEYFRAMEDATABASE_H
#define KEYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>

#include "map/KeyFrame.hpp"
#include "map/Frame.hpp"
#include "recognition/OrbVocabulary.hpp"
#include "map/Map.hpp"

#include<mutex>


namespace ORB_SLAM3
{

class KeyFrame;
class Frame;
class Map;


class KeyFrameDatabase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyFrameDatabase(const ORBVocabulary &voc);

    void add(const KeyFramePtr& pKF);

    // Raw on purpose: called from KeyFrame::SetBadFlag with `this` (the
    // caller pins the KF); pure key-scrub of the inverted file.
    void erase(KeyFrame* pKF);

    void clear();
    void clearMap(Map* pMap);

    // Loop and Merge Detection
    void DetectNBestCandidates(const KeyFramePtr& pKF, std::vector<KeyFramePtr> &vpLoopCand, std::vector<KeyFramePtr> &vpMergeCand, int nNumCandidates);

    // Relocalization
    std::vector<KeyFramePtr> DetectRelocalizationCandidates(Frame* F, Map* pMap);

protected:

   // Associated vocabulary
   const ORBVocabulary* mpVoc;

   // Inverted file. R4b slice 2: RAW entries on purpose (~1M word lists —
   // weak_ptr would double the entry size and add refcount churn to the
   // hottest query loops). Contract: KeyFrame::SetBadFlag scrubs its entries
   // via erase() before releasing the KF, so any listed entry is alive; the
   // Detect* queries wrap candidates into KeyFramePtr pins under mMutex
   // before scoring them outside the lock.
   std::vector<std::list<KeyFrame*> > mvInvertedFile;

   // Mutex
   std::mutex mMutex;

};

} //namespace ORB_SLAM

#endif
