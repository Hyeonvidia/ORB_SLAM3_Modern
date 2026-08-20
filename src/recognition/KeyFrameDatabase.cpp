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


#include "recognition/KeyFrameDatabase.hpp"

#include "map/KeyFrame.hpp"
#include "recognition/BowTypes.hpp"  // R3: upstream DBoW2 BowVector

#include<mutex>
#include<unordered_map>

namespace ORB_SLAM3
{

namespace {
// P5-3: per-query scratch, externalized from KeyFrame's mn*Query/mn*Words/m*Score
// scribble fields. Lifetime is one Detect* call; 'candidate' mirrors the old
// "query id was stamped" condition (i.e. the KF entered the sharing-words list).
struct QueryScratch {
    int nWords = 0;
    float score = 0.f;
    bool seen = false;
    bool candidate = false;
};
using QueryScratchMap = std::unordered_map<KeyFramePtr , QueryScratch>;
}

KeyFrameDatabase::KeyFrameDatabase (const ORBVocabulary &voc):
    mpVoc(&voc)
{
    mvInvertedFile.resize(voc.size());
}


void KeyFrameDatabase::add(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // Raw entry (see header comment): the KF is Map-pinned until its
    // SetBadFlag scrubs these entries back out via erase().
    for(DBoW2::BowVector::const_iterator vit= pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
        mvInvertedFile[vit->first].push_back(pKF.get());
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        std::list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

        for(std::list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        {
            if(pKF==*lit)
            {
                lKFs.erase(lit);
                break;
            }
        }
    }
}

void KeyFrameDatabase::clear()
{
    mvInvertedFile.clear();
    mvInvertedFile.resize(mpVoc->size());
}

void KeyFrameDatabase::clearMap(Map* pMap)
{
    std::unique_lock<std::mutex> lock(mMutex);

    // Erase elements in the Inverse File for the entry
    for(std::vector<std::list<KeyFrame*> >::iterator vit=mvInvertedFile.begin(), vend=mvInvertedFile.end(); vit!=vend; vit++)
    {
        // List of keyframes that share the word
        std::list<KeyFrame*> &lKFs =  *vit;

        for(std::list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend;)
        {
            KeyFrame* pKFi = *lit;
            if(pMap == pKFi->GetMap())
            {
                lit = lKFs.erase(lit);
                // Dont delete the KF because the class Map clean all the KF when it is destroyed
            }
            else
            {
                ++lit;
            }
        }
    }
}

bool compFirst(const std::pair<float, KeyFramePtr> & a, const std::pair<float, KeyFramePtr> & b)
{
    return a.first > b.first;
}


void KeyFrameDatabase::DetectNBestCandidates(const KeyFramePtr& pKF, std::vector<KeyFramePtr> &vpLoopCand, std::vector<KeyFramePtr> &vpMergeCand, int nNumCandidates)
{
    std::list<KeyFramePtr> lKFsSharingWords;
    std::set<KeyFramePtr> spConnectedKF;
    QueryScratchMap qsPlace;

    // Search all keyframes that share a word with current frame
    {
        std::unique_lock<std::mutex> lock(mMutex);

        spConnectedKF = pKF->GetConnectedKeyFrames();

        for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        {
            std::list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(std::list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                // Pin under mMutex: a listed entry cannot complete its
                // SetBadFlag scrub (erase() takes this mutex) while we hold
                // the lock, so it is alive and shared_from_this() is valid.
                KeyFramePtr pKFi=(*lit)->shared_from_this();

                QueryScratch& e = qsPlace[pKFi];
                if(!e.seen)
                {
                    e.seen = true;
                    if(!spConnectedKF.count(pKFi))
                    {

                        e.candidate = true;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                e.nWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return;

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(std::list<KeyFramePtr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if(qsPlace[*lit].nWords>maxCommonWords)
            maxCommonWords=qsPlace[*lit].nWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    std::list<std::pair<float,KeyFramePtr> > lScoreAndMatch;

    // Compute similarity score.
    for(KeyFramePtr pKFi : lKFsSharingWords)
    {

        if(qsPlace[pKFi].nWords>minCommonWords)
        {
            float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);
            qsPlace[pKFi].score=si;
            lScoreAndMatch.emplace_back(si,pKFi);
        }
    }

    if(lScoreAndMatch.empty())
        return;

    std::list<std::pair<float,KeyFramePtr> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(std::list<std::pair<float,KeyFramePtr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFramePtr pKFi = it->second;
        std::vector<KeyFramePtr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFramePtr pBestKF = pKFi;
        for(KeyFramePtr pKF2 : vpNeighs)
        {
            QueryScratchMap::iterator itQS = qsPlace.find(pKF2);
            if(itQS==qsPlace.end() || !itQS->second.candidate)
                continue;

            accScore+=itQS->second.score;
            if(itQS->second.score>bestScore)
            {
                pBestKF=pKF2;
                bestScore = itQS->second.score;
            }

        }
        lAccScoreAndMatch.emplace_back(accScore,pBestKF);
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    lAccScoreAndMatch.sort(compFirst);

    vpLoopCand.reserve(nNumCandidates);
    vpMergeCand.reserve(nNumCandidates);
    std::set<KeyFramePtr> spAlreadyAddedKF;
    size_t i = 0;
    std::list<std::pair<float,KeyFramePtr> >::iterator it=lAccScoreAndMatch.begin();
    while(i < lAccScoreAndMatch.size() && (vpLoopCand.size() < static_cast<size_t>(nNumCandidates) || vpMergeCand.size() < static_cast<size_t>(nNumCandidates)))
    {
        KeyFramePtr pKFi = it->second;
        if(pKFi->isBad())
        {
            // DIVERGENCES #23: upstream `continue`d here without advancing
            // i/it, spinning forever if a bad KF reached this list. Skip the
            // candidate instead of hanging.
            i++;
            it++;
            continue;
        }

        if(!spAlreadyAddedKF.count(pKFi))
        {
            if(pKF->GetMap() == pKFi->GetMap() && vpLoopCand.size() < static_cast<size_t>(nNumCandidates))
            {
                vpLoopCand.push_back(pKFi);
            }
            else if(pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < static_cast<size_t>(nNumCandidates) && !pKFi->GetMap()->IsBad())
            {
                vpMergeCand.push_back(pKFi);
            }
            spAlreadyAddedKF.insert(pKFi);
        }
        i++;
        it++;
    }
}


std::vector<KeyFramePtr> KeyFrameDatabase::DetectRelocalizationCandidates(Frame *F, Map* pMap)
{
    std::list<KeyFramePtr> lKFsSharingWords;
    QueryScratchMap qsReloc;

    // Search all keyframes that share a word with current frame
    {
        std::unique_lock<std::mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit=F->mBowVec.begin(), vend=F->mBowVec.end(); vit != vend; vit++)
        {
            std::list<KeyFrame*> &lKFs =   mvInvertedFile[vit->first];

            for(std::list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
            {
                // Pin under mMutex: a listed entry cannot complete its
                // SetBadFlag scrub (erase() takes this mutex) while we hold
                // the lock, so it is alive and shared_from_this() is valid.
                KeyFramePtr pKFi=(*lit)->shared_from_this();
                QueryScratch& e = qsReloc[pKFi];
                if(!e.seen)
                {
                    e.seen = true;
                    e.candidate = true;
                    lKFsSharingWords.push_back(pKFi);
                }
                e.nWords++;
            }
        }
    }
    if(lKFsSharingWords.empty())
        return std::vector<KeyFramePtr>();

    // Only compare against those keyframes that share enough words
    int maxCommonWords=0;
    for(std::list<KeyFramePtr>::iterator lit=lKFsSharingWords.begin(), lend= lKFsSharingWords.end(); lit!=lend; lit++)
    {
        if(qsReloc[*lit].nWords>maxCommonWords)
            maxCommonWords=qsReloc[*lit].nWords;
    }

    int minCommonWords = maxCommonWords*0.8f;

    std::list<std::pair<float,KeyFramePtr> > lScoreAndMatch;

    // Compute similarity score.
    for(KeyFramePtr pKFi : lKFsSharingWords)
    {

        if(qsReloc[pKFi].nWords>minCommonWords)
        {
            float si = mpVoc->score(F->mBowVec,pKFi->mBowVec);
            qsReloc[pKFi].score=si;
            lScoreAndMatch.emplace_back(si,pKFi);
        }
    }

    if(lScoreAndMatch.empty())
        return std::vector<KeyFramePtr>();

    std::list<std::pair<float,KeyFramePtr> > lAccScoreAndMatch;
    float bestAccScore = 0;

    // Lets now accumulate score by covisibility
    for(std::list<std::pair<float,KeyFramePtr> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
    {
        KeyFramePtr pKFi = it->second;
        std::vector<KeyFramePtr> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        float bestScore = it->first;
        float accScore = bestScore;
        KeyFramePtr pBestKF = pKFi;
        for(KeyFramePtr pKF2 : vpNeighs)
        {
            QueryScratchMap::iterator itQS = qsReloc.find(pKF2);
            if(itQS==qsReloc.end() || !itQS->second.candidate)
                continue;

            accScore+=itQS->second.score;
            if(itQS->second.score>bestScore)
            {
                pBestKF=pKF2;
                bestScore = itQS->second.score;
            }

        }
        lAccScoreAndMatch.emplace_back(accScore,pBestKF);
        if(accScore>bestAccScore)
            bestAccScore=accScore;
    }

    // Return all those keyframes with a score higher than 0.75*bestScore
    float minScoreToRetain = 0.75f*bestAccScore;
    std::set<KeyFramePtr> spAlreadyAddedKF;
    std::vector<KeyFramePtr> vpRelocCandidates;
    vpRelocCandidates.reserve(lAccScoreAndMatch.size());
    for(std::list<std::pair<float,KeyFramePtr> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
    {
        const float &si = it->first;
        if(si>minScoreToRetain)
        {
            KeyFramePtr pKFi = it->second;
            if (pKFi->GetMap() != pMap)
                continue;
            if(!spAlreadyAddedKF.count(pKFi))
            {
                vpRelocCandidates.push_back(pKFi);
                spAlreadyAddedKF.insert(pKFi);
            }
        }
    }

    return vpRelocCandidates;
}

} //namespace ORB_SLAM
