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


#include "map/Map.hpp"

#include<mutex>

namespace ORB_SLAM3
{

long unsigned int Map::nNextId=0;

Map::Map():
      mpFirstRegionKF(nullptr),
      mbFail(false),
      mbImuInitialized(false),
      mnMapChange(0),
      mnMapChangeNotified(0),
      mnMaxKFid(0),
      mnBigChangeIdx(0),
      mIsInUse(false),
      mHasTumbnail(false),
      mbBad(false),
      mbIsInertial(false),
      mbIMU_BA1(false),
      mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = nullptr;
}

Map::Map(int initKFid):
      mpFirstRegionKF(nullptr),
      mbFail(false),
      mbImuInitialized(false),
      mnMapChange(0),
      mnMapChangeNotified(0),
      mnInitKFid(initKFid),
      mnMaxKFid(initKFid),
      mnBigChangeIdx(0),
      mIsInUse(false),
      mHasTumbnail(false),
      mbBad(false),
      mbIsInertial(false),
      mbIMU_BA1(false),
      mbIMU_BA2(false)
{
    mnId=nNextId++;
    mThumbnail = nullptr;
}

Map::~Map()
{
    // R4b: dropping the owning sets releases every MapPoint/KeyFrame that no
    // other strong holder (frames, queues, pins, side tables) references.
    // KF match slots hold MPs strongly and each KF holds mPrevKF/mpParent
    // strongly, so the release cascades through the graph in dependency
    // order once the external pins are gone (the cycle-bearing edges —
    // observations, covisibility, children, mNextKF, loop/merge — are
    // raw/weak by design and cannot keep the graph alive).
    mspMapPoints.clear();
    mspKeyFrames.clear();

    if(mThumbnail)
        delete mThumbnail;
    mThumbnail = nullptr;

    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AddKeyFrame(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    if(mspKeyFrames.empty()){
        std::cout << "First KF:" << pKF->mnId << "; Map init KF:" << mnInitKFid << std::endl;
        mnInitKFid = pKF->mnId;
        mpKFinitial = pKF;
        mpKFlowerID = pKF;
    }
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
    {
        mnMaxKFid=pKF->mnId;
    }
    if(pKF->mnId<mpKFlowerID->mnId)
    {
        mpKFlowerID = pKF;
    }
}

void Map::AddMapPoint(const MapPointPtr& pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::SetImuInitialized()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mbImuInitialized = true;
}

bool Map::isImuInitialized()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mbImuInitialized;
}

void Map::EraseMapPoint(const MapPointPtr& pMP)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(const KeyFramePtr& pKF)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);
    if(mspKeyFrames.size()>0)
    {
        if(pKF->mnId == mpKFlowerID->mnId)
        {
            std::vector<KeyFramePtr> vpKFs = std::vector<KeyFramePtr>(mspKeyFrames.begin(),mspKeyFrames.end());
            sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
            mpKFlowerID = vpKFs[0];
        }
    }
    else
    {
        mpKFlowerID = nullptr;
    }

    // R4b: dropping the owning reference IS the erase; the tombstone lives
    // on through whatever other strong holders remain.
}

void Map::SetReferenceMapPoints(const std::vector<MapPointPtr> &vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

std::vector<KeyFramePtr> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<KeyFramePtr>(mspKeyFrames.begin(),mspKeyFrames.end());
}

std::vector<MapPointPtr> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<MapPointPtr>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

std::vector<MapPointPtr> Map::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetId()
{
    return mnId;
}
long unsigned int Map::GetInitKFid()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnInitKFid;
}

void Map::SetInitKFid(long unsigned int initKFif)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnInitKFid = initKFif;
}

long unsigned int Map::GetMaxKFid()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMaxKFid;
}

KeyFramePtr Map::GetOriginKF()
{
    return mpKFinitial;
}

void Map::SetCurrentMap()
{
    mIsInUse = true;
}

void Map::SetStoredMap()
{
    mIsInUse = false;
}

void Map::clear()
{
    // R4b: no manual delete — releasing the owning sets is the drop. With
    // both classes shared and the cycle-bearing edges raw/weak, this now
    // genuinely frees every KF/MP that no external strong holder (Tracking's
    // trajectory references and frames, queues, LC channels, side tables)
    // still pins; pinned tombstones are reclaimed later when those holders
    // let go.

    for(const KeyFramePtr& pKF : mspKeyFrames)
    {
        pKF->UpdateMap(nullptr);
    }

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = mnInitKFid;
    mbImuInitialized = false;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
    mbIMU_BA1 = false;
    mbIMU_BA2 = false;
}

bool Map::IsInUse()
{
    return mIsInUse;
}

void Map::SetBad()
{
    mbBad = true;
}

bool Map::IsBad()
{
    return mbBad;
}


void Map::ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel)
{
    std::unique_lock<std::mutex> lock(mMutexMap);

    // Body position (IMU) of first keyframe is fixed to (0,0,0)
    Sophus::SE3f Tyw = T;
    Eigen::Matrix3f Ryw = Tyw.rotationMatrix();
    Eigen::Vector3f tyw = Tyw.translation();

    for(std::set<KeyFramePtr>::iterator sit=mspKeyFrames.begin(); sit!=mspKeyFrames.end(); sit++)
    {
        const KeyFramePtr& pKF = *sit;
        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Twc.translation() *= s;
        Sophus::SE3f Tyc = Tyw*Twc;
        Sophus::SE3f Tcy = Tyc.inverse();
        pKF->SetPose(Tcy);
        Eigen::Vector3f Vw = pKF->GetVelocity();
        if(!bScaledVel)
            pKF->SetVelocity(Ryw*Vw);
        else
            pKF->SetVelocity(Ryw*Vw*s);

    }
    for(std::set<MapPointPtr>::iterator sit=mspMapPoints.begin(); sit!=mspMapPoints.end(); sit++)
    {
        MapPointPtr pMP = *sit;
        pMP->SetWorldPos(s * Ryw * pMP->GetWorldPos() + tyw);
        pMP->UpdateNormalAndDepth();
    }
    mnMapChange++;
}

void Map::SetInertialSensor()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mbIsInertial = true;
}

bool Map::IsInertial()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mbIsInertial;
}

void Map::SetIniertialBA1()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mbIMU_BA1 = true;
}

void Map::SetIniertialBA2()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mbIMU_BA2 = true;
}

bool Map::GetIniertialBA1()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mbIMU_BA1;
}

bool Map::GetIniertialBA2()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mbIMU_BA2;
}

void Map::ChangeId(long unsigned int nId)
{
    mnId = nId;
}

unsigned int Map::GetLowerKFID()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    if (mpKFlowerID) {
        return mpKFlowerID->mnId;
    }
    return 0;
}

int Map::GetMapChangeIndex()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMapChange;
}

void Map::IncreaseChangeIndex()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnMapChange++;
}

int Map::GetLastMapChange()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mnMapChangeNotified;
}

void Map::SetLastMapChange(int currentChangeId)
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    mnMapChangeNotified = currentChangeId;
}

void Map::PreSave(std::set<GeometricCamera*> &spCams)
{
    int nMPWithoutObs = 0;
    for(const MapPointPtr& pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
        {
            continue;
        }

        if(pMPi->GetObservations().size() == 0)
        {
            nMPWithoutObs++;
        }
        std::map<KeyFramePtr, std::tuple<int,int>> mpObs = pMPi->GetObservations();
        for(std::map<KeyFramePtr, std::tuple<int,int>>::iterator it= mpObs.begin(), end=mpObs.end(); it!=end; ++it)
        {
            if(it->first->GetMap() != this || it->first->isBad())
            {
                pMPi->EraseObservation(it->first);
            }

        }
    }

    // Saves the id of KF origins
    mvBackupKeyFrameOriginsId.clear();
    mvBackupKeyFrameOriginsId.reserve(mvpKeyFrameOrigins.size());
    for(int i = 0, numEl = mvpKeyFrameOrigins.size(); i < numEl; ++i)
    {
        mvBackupKeyFrameOriginsId.push_back(mvpKeyFrameOrigins[i]->mnId);
    }


    // Backup of MapPoints
    mvpBackupMapPoints.clear();
    for(const MapPointPtr& pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
        {
            continue;
        }

        // Raw .get(): the backup vector keeps the pre-R4b archive layout
        // (see Map.hpp); the owning shared_ptr lives in mspMapPoints.
        mvpBackupMapPoints.push_back(pMPi.get());
        pMPi->PreSave(mspKeyFrames,mspMapPoints);
    }

    // Backup of KeyFrames
    mvpBackupKeyFrames.clear();
    for(const KeyFramePtr& pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
        {
            continue;
        }

        // Raw .get(): the backup vector keeps the pre-R4b archive layout
        // (see Map.hpp); the owning shared_ptr lives in mspKeyFrames.
        mvpBackupKeyFrames.push_back(pKFi.get());
        pKFi->PreSave(mspKeyFrames,mspMapPoints, spCams);
    }

    mnBackupKFinitialID = -1;
    if(mpKFinitial)
    {
        mnBackupKFinitialID = mpKFinitial->mnId;
    }

    mnBackupKFlowerID = -1;
    if(mpKFlowerID)
    {
        mnBackupKFlowerID = mpKFlowerID->mnId;
    }

}

void Map::PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, std::map<unsigned int, GeometricCamera*> &mpCams)
{
    // R4b: boost allocated the MapPoints/KeyFrames raw into the backup
    // vectors; wrap each exactly ONCE into its owning shared_ptr here (this
    // is the single ownership hand-off point of the load path — it runs
    // before any graph wiring, so shared_from_this() is valid everywhere
    // downstream).
    for(MapPoint* pMPraw : mvpBackupMapPoints)
        mspMapPoints.insert(MapPointPtr(pMPraw));
    for(KeyFrame* pKFraw : mvpBackupKeyFrames)
        mspKeyFrames.insert(KeyFramePtr(pKFraw));

    std::map<long unsigned int,MapPointPtr> mpMapPointId;
    for(const MapPointPtr& pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
        {
            continue;
        }

        pMPi->UpdateMap(this);
        mpMapPointId[pMPi->mnId] = pMPi;
    }

    std::map<long unsigned int, KeyFramePtr> mpKeyFrameId;
    for(const KeyFramePtr& pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
        {
            continue;
        }

        pKFi->UpdateMap(this);
        pKFi->SetORBVocabulary(pORBVoc);
        pKFi->SetKeyFrameDatabase(pKFDB);
        mpKeyFrameId[pKFi->mnId] = pKFi;
    }

    // References reconstruction between different instances
    for(const MapPointPtr& pMPi : mspMapPoints)
    {
        if(!pMPi || pMPi->isBad())
        {
            continue;
        }

        pMPi->PostLoad(mpKeyFrameId, mpMapPointId);
    }

    for(const KeyFramePtr& pKFi : mspKeyFrames)
    {
        if(!pKFi || pKFi->isBad())
        {
            continue;
        }

        pKFi->PostLoad(mpKeyFrameId, mpMapPointId, mpCams);
        pKFDB->add(pKFi);
    }


    if(mnBackupKFinitialID != static_cast<unsigned long int>(-1))
    {
        mpKFinitial = mpKeyFrameId[mnBackupKFinitialID];
    }

    if(mnBackupKFlowerID != static_cast<unsigned long int>(-1))
    {
        mpKFlowerID = mpKeyFrameId[mnBackupKFlowerID];
    }

    mvpKeyFrameOrigins.clear();
    mvpKeyFrameOrigins.reserve(mvBackupKeyFrameOriginsId.size());
    for(size_t i = 0; i < mvBackupKeyFrameOriginsId.size(); ++i)
    {
        mvpKeyFrameOrigins.push_back(mpKeyFrameId[mvBackupKeyFrameOriginsId[i]]);
    }

    mvpBackupMapPoints.clear();
}


} //namespace ORB_SLAM3
