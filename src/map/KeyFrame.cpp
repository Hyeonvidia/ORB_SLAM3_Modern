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

#include "map/KeyFrame.hpp"
#include "io/Converter.hpp"
#include "backend/ImuTypes.hpp"
#include<mutex>

namespace ORB_SLAM3
{

long unsigned int KeyFrame::nNextId=0;

KeyFrame::KeyFrame():
      mnFrameId(0),
      mTimeStamp(0),
      mnGridCols(FRAME_GRID_COLS),
      mnGridRows(FRAME_GRID_ROWS),
      mfGridElementWidthInv(0),
      mfGridElementHeightInv(0),
      mnTrackReferenceForFrame(0),
      fx(0),
      fy(0),
      cx(0),
      cy(0),
      invfx(0),
      invfy(0),
      mbf(0),
      mb(0),
      mThDepth(0),
      N(0),
      // R1: the vectors were "initialized" by casting nullptr to the vector's
      // size_type (gcc-13 -Wconversion-null); plain value-init is identical.
      mvKeys(),
      mvKeysUn(),
      mvuRight(),
      mvDepth(),
      mnScaleLevels(0),
      mfScaleFactor(0),
      mfLogScaleFactor(0),
      mvScaleFactors(0),
      mvLevelSigma2(0),
      mvInvLevelSigma2(0),
      mnMinX(0),
      mnMinY(0),
      mnMaxX(0),
      mnMaxY(0),
      mPrevKF(nullptr),
      mNextKF(),
      mbHasVelocity(false),
      mbFirstConnection(true),
      mpParent(nullptr),
      mbNotErase(false),
      mbToBeErased(false),
      mbBad(false),
      mHalfBaseline(0),
      NLeft(0),
      NRight(0)
{

}

KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB):
      bImu(pMap->isImuInitialized()),
      mnFrameId(F.mnId),
      mTimeStamp(F.mTimeStamp),
      mnGridCols(FRAME_GRID_COLS),
      mnGridRows(FRAME_GRID_ROWS),
      mfGridElementWidthInv(F.mfGridElementWidthInv),
      mfGridElementHeightInv(F.mfGridElementHeightInv),
      mnTrackReferenceForFrame(0),
      fx(F.fx),
      fy(F.fy),
      cx(F.cx),
      cy(F.cy),
      invfx(F.invfx),
      invfy(F.invfy),
      mbf(F.mbf),
      mb(F.mb),
      mThDepth(F.mThDepth),
      mDistCoef(F.mDistCoef),
      N(F.N),
      mvKeys(F.mvKeys),
      mvKeysUn(F.mvKeysUn),
      mvuRight(F.mvuRight),
      mvDepth(F.mvDepth),
      mDescriptors(F.mDescriptors.clone()),
      mBowVec(F.mBowVec),
      mFeatVec(F.mFeatVec),
      mnScaleLevels(F.mnScaleLevels),
      mfScaleFactor(F.mfScaleFactor),
      mfLogScaleFactor(F.mfLogScaleFactor),
      mvScaleFactors(F.mvScaleFactors),
      mvLevelSigma2(F.mvLevelSigma2),
      mvInvLevelSigma2(F.mvInvLevelSigma2),
      mnMinX(F.mnMinX),
      mnMinY(F.mnMinY),
      mnMaxX(F.mnMaxX),
      mnMaxY(F.mnMaxY),
      mPrevKF(nullptr),
      mNextKF(),
      mpImuPreintegrated(F.mpImuPreintegrated),
      mImuCalib(F.mImuCalib),
      mNameFile(F.mNameFile),
      mnDataset(F.mnDataset),
      mbHasVelocity(false),
      mTlr(F.GetRelativePoseTlr()),
      mTrl(F.GetRelativePoseTrl()),
      mvpMapPoints(F.mvpMapPoints),
      mpKeyFrameDB(pKFDB),
      mpORBvocabulary(F.mpORBvocabulary),
      mbFirstConnection(true),
      mpParent(nullptr),
      mbNotErase(false),
      mbToBeErased(false),
      mbBad(false),
      mHalfBaseline(F.mb/2),
      mpMap(pMap),
      mK_(F.mK_),
      mpCamera(F.mpCamera),
      mpCamera2(F.mpCamera2),
      mvLeftToRightMatch(F.mvLeftToRightMatch),
      mvRightToLeftMatch(F.mvRightToLeftMatch),
      mvKeysRight(F.mvKeysRight),
      NLeft(F.Nleft),
      NRight(F.Nright)
{
    mnId=nNextId++;

    mGrid.resize(mnGridCols);
    if(F.Nleft != -1)  mGridRight.resize(mnGridCols);
    for(int i=0; i<mnGridCols;i++)
    {
        mGrid[i].resize(mnGridRows);
        if(F.Nleft != -1) mGridRight[i].resize(mnGridRows);
        for(int j=0; j<mnGridRows; j++){
            mGrid[i][j] = F.mGrid[i][j];
            if(F.Nleft != -1){
                mGridRight[i][j] = F.mGridRight[i][j];
            }
        }
    }



    if(!F.HasVelocity()) {
        mVw.setZero();
        mbHasVelocity = false;
    }
    else
    {
        mVw = F.GetVelocity();
        mbHasVelocity = true;
    }

    mImuBias = F.mImuBias;
    SetPose(F.GetPose());

    mnOriginMapId = pMap->GetId();
}

void KeyFrame::ComputeBoW()
{
    if(mBowVec.empty() || mFeatVec.empty())
    {
        std::vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        // Feature vector associate features with nodes in the 4th level (from leaves up)
        // We assume the vocabulary tree has 6 levels, change the 4 otherwise
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void KeyFrame::SetPose(const Sophus::SE3f &Tcw)
{
    std::unique_lock<std::mutex> lock(mMutexPose);

    mTcw = Tcw;
    mRcw = mTcw.rotationMatrix();
    mTwc = mTcw.inverse();
    mRwc = mTwc.rotationMatrix();

    if (mImuCalib.mbIsSet) // TODO Use a flag instead of the OpenCV matrix
    {
        mOwb = mRwc * mImuCalib.mTcb.translation() + mTwc.translation();
    }
}

void KeyFrame::SetVelocity(const Eigen::Vector3f &Vw)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    mVw = Vw;
    mbHasVelocity = true;
}

Sophus::SE3f KeyFrame::GetPose()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTcw;
}

Sophus::SE3f KeyFrame::GetPoseInverse()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTwc;
}

Eigen::Vector3f KeyFrame::GetCameraCenter(){
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTwc.translation();
}

Eigen::Vector3f KeyFrame::GetImuPosition()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mOwb;
}

Eigen::Matrix3f KeyFrame::GetImuRotation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return (mTwc * mImuCalib.mTcb).rotationMatrix();
}

Sophus::SE3f KeyFrame::GetImuPose()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTwc * mImuCalib.mTcb;
}

Eigen::Matrix3f KeyFrame::GetRotation(){
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mRcw;
}

Eigen::Vector3f KeyFrame::GetTranslation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTcw.translation();
}

Eigen::Vector3f KeyFrame::GetVelocity()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mVw;
}

bool KeyFrame::isVelocitySet()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mbHasVelocity;
}

void KeyFrame::AddConnection(KeyFrame *pKF, const int &weight)
{
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(!mConnectedKeyFrameWeights.count(pKF))
            mConnectedKeyFrameWeights[pKF]=weight;
        else if(mConnectedKeyFrameWeights[pKF]!=weight)
            mConnectedKeyFrameWeights[pKF]=weight;
        else
            return;
    }

    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::vector<std::pair<int,KeyFrame*> > vPairs;
    vPairs.reserve(mConnectedKeyFrameWeights.size());
    for(std::map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
       vPairs.emplace_back(mit->second,mit->first);

    std::sort(vPairs.begin(),vPairs.end());
    std::list<KeyFrame*> lKFs;
    std::list<int> lWs;
    for(size_t i=0, iend=vPairs.size(); i<iend;i++)
    {
        if(!vPairs[i].second->isBad())
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }
    }

    mvpOrderedConnectedKeyFrames = std::vector<KeyFrame*>(lKFs.begin(),lKFs.end());
    mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());
}

// R4b slice 2: the covisibility accessors wrap the raw storage into
// KeyFramePtr pins UNDER mMutexConnections. While we hold this lock, any
// partner still present in the maps has not completed its SetBadFlag scrub
// (EraseConnection takes this same mutex), so it is still strongly held by
// its Map and shared_from_this() is valid — the caller's copy is a pin.
std::set<KeyFramePtr> KeyFrame::GetConnectedKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::set<KeyFramePtr> s;
    for(std::map<KeyFrame*,int>::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
        s.insert(mit->first->shared_from_this());
    return s;
}

std::vector<KeyFramePtr> KeyFrame::GetVectorCovisibleKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    std::vector<KeyFramePtr> v;
    v.reserve(mvpOrderedConnectedKeyFrames.size());
    for(KeyFrame* pKFi : mvpOrderedConnectedKeyFrames)
        v.push_back(pKFi->shared_from_this());
    return v;
}

std::vector<KeyFramePtr> KeyFrame::GetBestCovisibilityKeyFrames(const int &N)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    const size_t n = (static_cast<int>(mvpOrderedConnectedKeyFrames.size())<N) ? mvpOrderedConnectedKeyFrames.size() : static_cast<size_t>(N);
    std::vector<KeyFramePtr> v;
    v.reserve(n);
    for(size_t i=0; i<n; ++i)
        v.push_back(mvpOrderedConnectedKeyFrames[i]->shared_from_this());
    return v;
}

std::vector<KeyFramePtr> KeyFrame::GetCovisiblesByWeight(const int &w)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);

    if(mvpOrderedConnectedKeyFrames.empty())
    {
        return std::vector<KeyFramePtr>();
    }

    std::vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,KeyFrame::weightComp);

    if(it==mvOrderedWeights.end() && mvOrderedWeights.back() < w)
    {
        return std::vector<KeyFramePtr>();
    }
    else
    {
        int n = it-mvOrderedWeights.begin();
        std::vector<KeyFramePtr> v;
        v.reserve(n);
        for(int i=0; i<n; ++i)
            v.push_back(mvpOrderedConnectedKeyFrames[i]->shared_from_this());
        return v;
    }
}

int KeyFrame::GetWeight(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    if(mConnectedKeyFrameWeights.count(pKF.get()))
        return mConnectedKeyFrameWeights[pKF.get()];
    else
        return 0;
}

int KeyFrame::GetNumberMPs()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    int numberMPs = 0;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        numberMPs++;
    }
    return numberMPs;
}

void KeyFrame::AddMapPoint(const MapPointPtr& pMP, const size_t &idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=pMP;
}

void KeyFrame::EraseMapPointMatch(const int &idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mvpMapPoints[idx]=nullptr;
}

void KeyFrame::EraseMapPointMatch(const MapPointPtr& pMP)
{
    std::tuple<size_t,size_t> indexes = pMP->GetIndexInKeyFrame(shared_from_this());
    size_t leftIndex = std::get<0>(indexes), rightIndex = std::get<1>(indexes);
    // Sentinel: GetIndexInKeyFrame packs int -1 into size_t (== size_t(-1)).
    if(leftIndex != static_cast<size_t>(-1))
        mvpMapPoints[leftIndex]=nullptr;
    if(rightIndex != static_cast<size_t>(-1))
        mvpMapPoints[rightIndex]=nullptr;
}


void KeyFrame::ReplaceMapPointMatch(const int &idx, const MapPointPtr& pMP)
{
    mvpMapPoints[idx]=pMP;
}

std::set<MapPointPtr> KeyFrame::GetMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    std::set<MapPointPtr> s;
    for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
    {
        if(!mvpMapPoints[i])
            continue;
        MapPointPtr pMP = mvpMapPoints[i];
        if(!pMP->isBad())
            s.insert(pMP);
    }
    return s;
}

int KeyFrame::TrackedMapPoints(const int &minObs)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);

    int nPoints=0;
    const bool bCheckObs = minObs>0;
    for(int i=0; i<N; i++)
    {
        MapPointPtr pMP = mvpMapPoints[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(bCheckObs)
                {
                    if(mvpMapPoints[i]->Observations()>=minObs)
                        nPoints++;
                }
                else
                    nPoints++;
            }
        }
    }

    return nPoints;
}

std::vector<MapPointPtr> KeyFrame::GetMapPointMatches()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

MapPointPtr KeyFrame::GetMapPoint(const size_t &idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mvpMapPoints[idx];
}

void KeyFrame::UpdateConnections(bool upParent)
{
    // R4b slice 2: the counter works in KeyFramePtr space (the observation
    // accessor hands out pins); the raw member containers are written once
    // under the lock at the end (raw storage = deliberate cycle break).
    std::map<KeyFramePtr,int> KFcounter;

    std::vector<MapPointPtr> vpMP;

    {
        std::unique_lock<std::mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for(MapPointPtr pMP : vpMP)
    {

        if(!pMP)
            continue;

        if(pMP->isBad())
        {
            continue;
        }

        std::map<KeyFramePtr,std::tuple<int,int>> observations = pMP->GetObservations();

        for(std::map<KeyFramePtr,std::tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            if(mit->first->mnId==mnId || mit->first->isBad() || mit->first->GetMap() != mpMap)
            {
                continue;
            }
            KFcounter[mit->first]++;

        }
    }

    // This should not happen
    if(KFcounter.empty())
        return;

    //If the counter is greater than threshold add connection
    //In case no keyframe counter is over threshold add the one with maximum counter
    int nmax=0;
    KeyFramePtr pKFmax=nullptr;
    int th = 15;

    std::vector<std::pair<int,KeyFramePtr> > vPairs;
    vPairs.reserve(KFcounter.size());
    if(!upParent)
        std::cout << "UPDATE_CONN: current KF " << mnId << std::endl;
    for(std::map<KeyFramePtr,int>::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
    {
        if(!upParent)
            std::cout << "  UPDATE_CONN: KF " << mit->first->mnId << " ; num matches: " << mit->second << std::endl;
        if(mit->second>nmax)
        {
            nmax=mit->second;
            pKFmax=mit->first;
        }
        if(mit->second>=th)
        {
            vPairs.emplace_back(mit->second,mit->first);
            (mit->first)->AddConnection(this,mit->second);
        }
    }

    if(vPairs.empty())
    {
        vPairs.emplace_back(nmax,pKFmax);
        pKFmax->AddConnection(this,nmax);
    }

    std::sort(vPairs.begin(),vPairs.end());
    std::list<KeyFramePtr> lKFs;
    std::list<int> lWs;
    for(size_t i=0; i<vPairs.size();i++)
    {
        lKFs.push_front(vPairs[i].second);
        lWs.push_front(vPairs[i].first);
    }

    {
        std::unique_lock<std::mutex> lockCon(mMutexConnections);

        mConnectedKeyFrameWeights.clear();
        for(const auto& kv : KFcounter)
            mConnectedKeyFrameWeights[kv.first.get()] = kv.second;
        mvpOrderedConnectedKeyFrames.clear();
        mvpOrderedConnectedKeyFrames.reserve(lKFs.size());
        for(const KeyFramePtr& pKFi : lKFs)
            mvpOrderedConnectedKeyFrames.push_back(pKFi.get());
        mvOrderedWeights = std::vector<int>(lWs.begin(), lWs.end());


        if(mbFirstConnection && mnId!=mpMap->GetInitKFid())
        {
            mpParent = lKFs.front();
            mpParent->AddChild(shared_from_this());
            mbFirstConnection = false;
        }

    }
}

void KeyFrame::AddChild(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mspChildrens.insert(KeyFrameWeakPtr(pKF));
}

void KeyFrame::EraseChild(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mspChildrens.erase(KeyFrameWeakPtr(pKF));
}

void KeyFrame::ChangeParent(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    if(pKF.get() == this)
    {
        std::cout << "ERROR: Change parent KF, the parent and child are the same KF" << std::endl;
        throw std::invalid_argument("The parent and child can not be the same");
    }

    mpParent = pKF;
    pKF->AddChild(shared_from_this());
}

std::set<KeyFramePtr> KeyFrame::GetChilds()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    std::set<KeyFramePtr> s;
    for(const KeyFrameWeakPtr& w : mspChildrens)
        if(KeyFramePtr p = w.lock())
            s.insert(p);
    return s;
}

KeyFramePtr KeyFrame::GetParent()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mpParent;
}

bool KeyFrame::hasChild(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mspChildrens.count(KeyFrameWeakPtr(pKF));
}

void KeyFrame::SetFirstConnection(bool bFirst)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mbFirstConnection=bFirst;
}

void KeyFrame::AddLoopEdge(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(KeyFrameWeakPtr(pKF));
}

std::set<KeyFramePtr> KeyFrame::GetLoopEdges()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    std::set<KeyFramePtr> s;
    for(const KeyFrameWeakPtr& w : mspLoopEdges)
        if(KeyFramePtr p = w.lock())
            s.insert(p);
    return s;
}

void KeyFrame::AddMergeEdge(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspMergeEdges.insert(KeyFrameWeakPtr(pKF));
}

std::set<KeyFramePtr> KeyFrame::GetMergeEdges()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    std::set<KeyFramePtr> s;
    for(const KeyFrameWeakPtr& w : mspMergeEdges)
        if(KeyFramePtr p = w.lock())
            s.insert(p);
    return s;
}

void KeyFrame::SetNotErase()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    mbNotErase = true;
}

void KeyFrame::SetErase()
{
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(mspLoopEdges.empty())
        {
            mbNotErase = false;
        }
    }

    if(mbToBeErased)
    {
        SetBadFlag();
    }
}

void KeyFrame::SetBadFlag()
{
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(mnId==mpMap->GetInitKFid())
        {
            return;
        }
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
    }

    // R4b slice 2: pin self once — after Map::EraseKeyFrame below drops the
    // owning reference, this pin (plus the caller's handle) keeps the
    // tombstone alive through the tail of this function.
    const KeyFramePtr pSelf = shared_from_this();

    for(std::map<KeyFrame*,int>::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
    {
        mit->first->EraseConnection(this);
    }

    for(size_t i=0; i<mvpMapPoints.size(); i++)
    {
        if(mvpMapPoints[i])
        {
            mvpMapPoints[i]->EraseObservation(pSelf);
        }
    }

    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        std::unique_lock<std::mutex> lock1(mMutexFeatures);

        mConnectedKeyFrameWeights.clear();
        mvpOrderedConnectedKeyFrames.clear();

        // Update Spanning Tree (children are weak now: lock() each; an
        // expired entry means the child was already reclaimed — skip it,
        // exactly as a rewired-away raw child would have been unreachable)
        std::set<KeyFramePtr> sParentCandidates;
        if(mpParent)
            sParentCandidates.insert(mpParent);

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        while(!mspChildrens.empty())
        {
            bool bContinue = false;

            int max = -1;
            KeyFramePtr pC;
            KeyFramePtr pP;

            for(auto sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
            {
                KeyFramePtr pKF = sit->lock();
                if(!pKF || pKF->isBad())
                {
                    continue;
                }

                // Check if a parent candidate is connected to the keyframe
                std::vector<KeyFramePtr> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                {
                    for(std::set<KeyFramePtr>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                    {
                        if(vpConnected[i]->mnId == (*spcit)->mnId)
                        {
                            int w = pKF->GetWeight(vpConnected[i]);
                            if(w>max)
                            {
                                pC = pKF;
                                pP = vpConnected[i];
                                max = w;
                                bContinue = true;
                            }
                        }
                    }
                }
            }

            if(bContinue)
            {
                pC->ChangeParent(pP);
                sParentCandidates.insert(pC);
                mspChildrens.erase(KeyFrameWeakPtr(pC));
            }
            else
                break;
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        if(!mspChildrens.empty())
        {
            for(auto sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
            {
                if(KeyFramePtr pChild = sit->lock())
                    pChild->ChangeParent(mpParent);
            }
        }

        if(mpParent){
            mpParent->EraseChild(pSelf);
            mTcp = mTcw * mpParent->GetPoseInverse();
        }
        mbBad = true;
    }


    mpMap->EraseKeyFrame(pSelf);
    mpKeyFrameDB->erase(this);
}

bool KeyFrame::isBad()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    return mbBad;
}

void KeyFrame::EraseConnection(KeyFrame* pKF)
{
    bool bUpdate = false;
    {
        std::unique_lock<std::mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
        {
            mConnectedKeyFrameWeights.erase(pKF);
            bUpdate=true;
        }
    }

    if(bUpdate)
        UpdateBestCovisibles();
}


std::vector<size_t> KeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r, const bool bRight) const
{
    std::vector<size_t> vIndices;
    vIndices.reserve(N);

    float factorX = r;
    float factorY = r;

    const int nMinCellX = std::max(0,static_cast<int>(floor((x-mnMinX-factorX)*mfGridElementWidthInv)));
    if(nMinCellX>=mnGridCols)
        return vIndices;

    const int nMaxCellX = std::min(static_cast<int>(mnGridCols)-1,static_cast<int>(ceil((x-mnMinX+factorX)*mfGridElementWidthInv)));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = std::max(0,static_cast<int>(floor((y-mnMinY-factorY)*mfGridElementHeightInv)));
    if(nMinCellY>=mnGridRows)
        return vIndices;

    const int nMaxCellY = std::min(static_cast<int>(mnGridRows)-1,static_cast<int>(ceil((y-mnMinY+factorY)*mfGridElementHeightInv)));
    if(nMaxCellY<0)
        return vIndices;

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const std::vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];
            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = (NLeft == -1) ? mvKeysUn[vCell[j]]
                                                         : (!bRight) ? mvKeys[vCell[j]]
                                                                     : mvKeysRight[vCell[j]];
                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool KeyFrame::IsInImage(const float &x, const float &y) const
{
    return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
}

bool KeyFrame::UnprojectStereo(int i, Eigen::Vector3f &x3D)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeys[i].pt.x;
        const float v = mvKeys[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        Eigen::Vector3f x3Dc(x, y, z);

        std::unique_lock<std::mutex> lock(mMutexPose);
        x3D = mRwc * x3Dc + mTwc.translation();
        return true;
    }
    else
        return false;
}

float KeyFrame::ComputeSceneMedianDepth(const int q)
{
    if(N==0)
        return -1.0;

    std::vector<MapPointPtr> vpMapPoints;
    Eigen::Matrix3f Rcw;
    Eigen::Vector3f tcw;
    {
        std::unique_lock<std::mutex> lock(mMutexFeatures);
        std::unique_lock<std::mutex> lock2(mMutexPose);
        vpMapPoints = mvpMapPoints;
        tcw = mTcw.translation();
        Rcw = mRcw;
    }

    std::vector<float> vDepths;
    vDepths.reserve(N);
    Eigen::Matrix<float,1,3> Rcw2 = Rcw.row(2);
    float zcw = tcw(2);
    for(int i=0; i<N; i++) {
        if(mvpMapPoints[i])
        {
            MapPointPtr pMP = mvpMapPoints[i];
            Eigen::Vector3f x3Dw = pMP->GetWorldPos();
            float z = Rcw2.dot(x3Dw) + zcw;
            vDepths.push_back(z);
        }
    }

    std::sort(vDepths.begin(),vDepths.end());

    return vDepths[(vDepths.size()-1)/q];
}

void KeyFrame::SetNewBias(const IMU::Bias &b)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    mImuBias = b;
    if(mpImuPreintegrated)
        mpImuPreintegrated->SetNewBias(b);
}

Eigen::Vector3f KeyFrame::GetGyroBias()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bwx, mImuBias.bwy, mImuBias.bwz);
}

Eigen::Vector3f KeyFrame::GetAccBias()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Eigen::Vector3f(mImuBias.bax, mImuBias.bay, mImuBias.baz);
}

IMU::Bias KeyFrame::GetImuBias()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mImuBias;
}

Map* KeyFrame::GetMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mpMap;
}

void KeyFrame::UpdateMap(Map* pMap)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mpMap = pMap;
}

void KeyFrame::PreSave(std::set<KeyFramePtr>& spKF,std::set<MapPointPtr>& spMP, std::set<GeometricCamera*>& spCam)
{
    // Save the id of each MapPoint in this KF, there can be null pointer in the vector
    mvBackupMapPointsId.clear();
    mvBackupMapPointsId.reserve(N);
    for(int i = 0; i < N; ++i)
    {

        if(mvpMapPoints[i] && spMP.find(mvpMapPoints[i]) != spMP.end()) // Checks if the element is not null
            mvBackupMapPointsId.push_back(mvpMapPoints[i]->mnId);
        else // If the element is null his value is -1 because all the id are positives
            mvBackupMapPointsId.push_back(-1);
    }
    // Save the id of each connected KF with it weight
    // (PreSave runs single-threaded after the join chain; the raw covis keys
    // are alive by the scrub contract, so wrapping them here is safe.)
    mBackupConnectedKeyFrameIdWeights.clear();
    for(std::map<KeyFrame*,int>::const_iterator it = mConnectedKeyFrameWeights.begin(), end = mConnectedKeyFrameWeights.end(); it != end; ++it)
    {
        if(spKF.find(it->first->shared_from_this()) != spKF.end())
            mBackupConnectedKeyFrameIdWeights[it->first->mnId] = it->second;
    }

    // Save the parent id
    mBackupParentId = -1;
    if(mpParent && spKF.find(mpParent) != spKF.end())
        mBackupParentId = mpParent->mnId;

    // Save the id of the childrens KF
    mvBackupChildrensId.clear();
    mvBackupChildrensId.reserve(mspChildrens.size());
    for(const KeyFrameWeakPtr& wKFi : mspChildrens)
    {
        KeyFramePtr pKFi = wKFi.lock();
        if(pKFi && spKF.find(pKFi) != spKF.end())
            mvBackupChildrensId.push_back(pKFi->mnId);
    }

    // Save the id of the loop edge KF
    mvBackupLoopEdgesId.clear();
    mvBackupLoopEdgesId.reserve(mspLoopEdges.size());
    for(const KeyFrameWeakPtr& wKFi : mspLoopEdges)
    {
        KeyFramePtr pKFi = wKFi.lock();
        if(pKFi && spKF.find(pKFi) != spKF.end())
            mvBackupLoopEdgesId.push_back(pKFi->mnId);
    }

    // Save the id of the merge edge KF
    mvBackupMergeEdgesId.clear();
    mvBackupMergeEdgesId.reserve(mspMergeEdges.size());
    for(const KeyFrameWeakPtr& wKFi : mspMergeEdges)
    {
        KeyFramePtr pKFi = wKFi.lock();
        if(pKFi && spKF.find(pKFi) != spKF.end())
            mvBackupMergeEdgesId.push_back(pKFi->mnId);
    }

    //Camera data
    mnBackupIdCamera = -1;
    if(mpCamera && spCam.find(mpCamera) != spCam.end())
        mnBackupIdCamera = mpCamera->GetId();

    mnBackupIdCamera2 = -1;
    if(mpCamera2 && spCam.find(mpCamera2) != spCam.end())
        mnBackupIdCamera2 = mpCamera2->GetId();

    //Inertial data
    mBackupPrevKFId = -1;
    if(mPrevKF && spKF.find(mPrevKF) != spKF.end())
        mBackupPrevKFId = mPrevKF->mnId;

    mBackupNextKFId = -1;
    if(KeyFramePtr pNextKF = mNextKF.lock())
        if(spKF.find(pNextKF) != spKF.end())
            mBackupNextKFId = pNextKF->mnId;

    if(mpImuPreintegrated)
        mBackupImuPreintegrated.CopyFrom(mpImuPreintegrated);
}

void KeyFrame::PostLoad(std::map<long unsigned int, KeyFramePtr>& mpKFid, std::map<long unsigned int, MapPointPtr>& mpMPid, std::map<unsigned int, GeometricCamera*>& mpCamId){
    // Rebuild the empty variables

    // Pose
    SetPose(mTcw);

    mTrl = mTlr.inverse();

    // Reference reconstruction
    // Each MapPoint sight from this KeyFrame
    mvpMapPoints.clear();
    mvpMapPoints.resize(N);
    for(int i=0; i<N; ++i)
    {
        if(mvBackupMapPointsId[i] != -1)
            mvpMapPoints[i] = mpMPid[mvBackupMapPointsId[i]];
        else
            mvpMapPoints[i] = nullptr;
    }

    // Conected KeyFrames with him weight
    mConnectedKeyFrameWeights.clear();
    for(std::map<long unsigned int, int>::const_iterator it = mBackupConnectedKeyFrameIdWeights.begin(), end = mBackupConnectedKeyFrameIdWeights.end();
        it != end; ++it)
    {
        const KeyFramePtr& pKFi = mpKFid[it->first];
        mConnectedKeyFrameWeights[pKFi.get()] = it->second;
    }

    // Restore parent KeyFrame
    if(mBackupParentId>=0)
        mpParent = mpKFid[mBackupParentId];

    // KeyFrame childrens
    mspChildrens.clear();
    for(std::vector<long unsigned int>::const_iterator it = mvBackupChildrensId.begin(), end = mvBackupChildrensId.end(); it!=end; ++it)
    {
        mspChildrens.insert(KeyFrameWeakPtr(mpKFid[*it]));
    }

    // Loop edge KeyFrame
    mspLoopEdges.clear();
    for(std::vector<long unsigned int>::const_iterator it = mvBackupLoopEdgesId.begin(), end = mvBackupLoopEdgesId.end(); it != end; ++it)
    {
        mspLoopEdges.insert(KeyFrameWeakPtr(mpKFid[*it]));
    }

    // Merge edge KeyFrame
    mspMergeEdges.clear();
    for(std::vector<long unsigned int>::const_iterator it = mvBackupMergeEdgesId.begin(), end = mvBackupMergeEdgesId.end(); it != end; ++it)
    {
        mspMergeEdges.insert(KeyFrameWeakPtr(mpKFid[*it]));
    }

    //Camera data
    if(mnBackupIdCamera >= 0)
    {
        mpCamera = mpCamId[mnBackupIdCamera];
    }
    else
    {
        std::cout << "ERROR: There is not a main camera in KF " << mnId << std::endl;
    }
    if(mnBackupIdCamera2 >= 0)
    {
        mpCamera2 = mpCamId[mnBackupIdCamera2];
    }

    //Inertial data
    if(mBackupPrevKFId != -1)
    {
        mPrevKF = mpKFid[mBackupPrevKFId];
    }
    if(mBackupNextKFId != -1)
    {
        mNextKF = mpKFid[mBackupNextKFId];
    }
    mpImuPreintegrated = &mBackupImuPreintegrated;


    // Remove all backup container
    mvBackupMapPointsId.clear();
    mBackupConnectedKeyFrameIdWeights.clear();
    mvBackupChildrensId.clear();
    mvBackupLoopEdgesId.clear();

    UpdateBestCovisibles();
}

bool KeyFrame::ProjectPointDistort(const MapPointPtr& pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY = Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        std::cout << "Negative depth: " << PcZ << std::endl;
        return false;
    }

    // Project in image and check it is not outside
    float invz = 1.0f/PcZ;
    u=fx*PcX*invz+cx;
    v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    float x = (u - cx) * invfx;
    float y = (v - cy) * invfy;
    float r2 = x * x + y * y;
    float k1 = mDistCoef.at<float>(0);
    float k2 = mDistCoef.at<float>(1);
    float p1 = mDistCoef.at<float>(2);
    float p2 = mDistCoef.at<float>(3);
    float k3 = 0;
    if(mDistCoef.total() == 5)
    {
        k3 = mDistCoef.at<float>(4);
    }

    // Radial distorsion
    float x_distort = x * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
    float y_distort = y * (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

    // Tangential distorsion
    x_distort = x_distort + (2 * p1 * x * y + p2 * (r2 + 2 * x * x));
    y_distort = y_distort + (p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);

    float u_distort = x_distort * fx + cx;
    float v_distort = y_distort * fy + cy;

    u = u_distort;
    v = v_distort;

    kp = cv::Point2f(u, v);

    return true;
}

bool KeyFrame::ProjectPointUnDistort(const MapPointPtr& pMP, cv::Point2f &kp, float &u, float &v)
{

    // 3D in absolute coordinates
    Eigen::Vector3f P = pMP->GetWorldPos();

    // 3D in camera coordinates
    Eigen::Vector3f Pc = mRcw * P + mTcw.translation();
    float &PcX = Pc(0);
    float &PcY= Pc(1);
    float &PcZ = Pc(2);

    // Check positive depth
    if(PcZ<0.0f)
    {
        std::cout << "Negative depth: " << PcZ << std::endl;
        return false;
    }

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    u = fx * PcX * invz + cx;
    v = fy * PcY * invz + cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    kp = cv::Point2f(u, v);

    return true;
}

Sophus::SE3f KeyFrame::GetRelativePoseTrl()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTrl;
}

Sophus::SE3f KeyFrame::GetRelativePoseTlr()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return mTlr;
}

Sophus::SE3<float> KeyFrame::GetRightPose() {
    std::unique_lock<std::mutex> lock(mMutexPose);

    return mTrl * mTcw;
}

Sophus::SE3<float> KeyFrame::GetRightPoseInverse() {
    std::unique_lock<std::mutex> lock(mMutexPose);

    return mTwc * mTlr;
}

Eigen::Vector3f KeyFrame::GetRightCameraCenter() {
    std::unique_lock<std::mutex> lock(mMutexPose);

    return (mTwc * mTlr).translation();
}

Eigen::Matrix<float,3,3> KeyFrame::GetRightRotation() {
    std::unique_lock<std::mutex> lock(mMutexPose);

    return (mTrl.so3() * mTcw.so3()).matrix();
}

Eigen::Vector3f KeyFrame::GetRightTranslation() {
    std::unique_lock<std::mutex> lock(mMutexPose);
    return (mTrl * mTcw).translation();
}

void KeyFrame::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBvocabulary = pORBVoc;
}

void KeyFrame::SetKeyFrameDatabase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

} //namespace ORB_SLAM
