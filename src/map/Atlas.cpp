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

#include "map/Atlas.hpp"
#include "viz/Viewer.hpp"

#include "camera/GeometricCamera.hpp"
#include "camera/Pinhole.hpp"
#include "camera/KannalaBrandt8.hpp"

namespace ORB_SLAM3
{

Atlas::Atlas(){
    mpCurrentMap = nullptr;
}

Atlas::Atlas(int initKFid): mnLastInitKFidMap(initKFid), mHasViewer(false)
{
    mpCurrentMap = nullptr;
    CreateNewMap();
}

Atlas::~Atlas()
{
    for(std::set<Map*>::iterator it = mspMaps.begin(), end = mspMaps.end(); it != end;)
    {
        Map* pMi = *it;

        if(pMi)
        {
            delete pMi;
            pMi = nullptr;

            it = mspMaps.erase(it);
        }
        else
            ++it;

    }
}

void Atlas::CreateNewMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    CreateNewMap_impl();
}

// Precondition: caller holds mMutexAtlas. Split out so GetCurrentMap can
// create the first map without re-locking the non-recursive mutex — the
// old GetCurrentMap()->CreateNewMap() path self-deadlocked whenever another
// thread called GetCurrentMap during Tracking's reset window (P5-1; same
// defect the Remastered audit observed live as E4b).
void Atlas::CreateNewMap_impl()
{
    std::cout << "Creation of new map with id: " << Map::nNextId << std::endl;
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum

        mpCurrentMap->SetStoredMap();
        std::cout << "Stored map with ID: " << mpCurrentMap->GetId() << std::endl;

    }
    std::cout << "Creation of new map with last KF id: " << mnLastInitKFidMap << std::endl;

    mpCurrentMap = new Map(mnLastInitKFidMap);
    mpCurrentMap->SetCurrentMap();
    mspMaps.insert(mpCurrentMap);
}

void Atlas::ChangeMap(Map* pMap)
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    std::cout << "Change to map with id: " << pMap->GetId() << std::endl;
    if(mpCurrentMap){
        mpCurrentMap->SetStoredMap();
    }

    mpCurrentMap = pMap;
    mpCurrentMap->SetCurrentMap();
}

unsigned long int Atlas::GetLastInitKFid()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mnLastInitKFidMap;
}

void Atlas::SetViewer(Viewer* pViewer)
{
    mpViewer = pViewer;
    mHasViewer = true;
}

// P11-A (Atlas lock batch): the upstream-inherited lock-free method group
// (AddKeyFrame/AddMapPoint/AddCamera/GetAllCameras/SetMapBad/RemoveBadMaps,
// docs/OWNERSHIP.md "Atlas 무락 메서드군") now takes mMutexAtlas like the
// rest of the class. Pure threading safety, numerics-invisible. Lock-order
// note: this adds only Atlas -> {Map,KeyFrame}-layer edges (already
// established by clearMap/GetCurrentMap); no code path acquires mMutexAtlas
// while holding a Map/KeyFrame-layer mutex, and callers holding
// mMutexMapUpdate (e.g. CreateNewKeyFrame -> AddKeyFrame) follow the
// existing MapUpdate -> Atlas order (ImuInitializer precedent).
void Atlas::AddKeyFrame(KeyFramePtr pKF)
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    Map* pMapKF = pKF->GetMap();
    pMapKF->AddKeyFrame(pKF);
}

void Atlas::AddMapPoint(const MapPointPtr& pMP)
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    Map* pMapMP = pMP->GetMap();
    pMapMP->AddMapPoint(pMP);
}

GeometricCamera* Atlas::AddCamera(GeometricCamera* pCam)
{
    // P11-A: guards mvpCameras (also read by GetAllCameras from other
    // threads and by the serialization path).
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    //Check if the camera already exists
    bool bAlreadyInMap = false;
    int index_cam = -1;
    for(size_t i=0; i < mvpCameras.size(); ++i)
    {
        GeometricCamera* pCam_i = mvpCameras[i];
        if(!pCam) std::cout << "Not pCam" << std::endl;
        if(!pCam_i) std::cout << "Not pCam_i" << std::endl;
        if(pCam->GetType() != pCam_i->GetType())
            continue;

        if(pCam->GetType() == GeometricCamera::CAM_PINHOLE)
        {
            if(((Pinhole*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
        else if(pCam->GetType() == GeometricCamera::CAM_FISHEYE)
        {
            if(((KannalaBrandt8*)pCam_i)->IsEqual(pCam))
            {
                bAlreadyInMap = true;
                index_cam = i;
            }
        }
    }

    if(bAlreadyInMap)
    {
        return mvpCameras[index_cam];
    }
    else{
        mvpCameras.push_back(pCam);
        return pCam;
    }
}

std::vector<GeometricCamera*> Atlas::GetAllCameras()
{
    // P11-A: copy taken under the lock (see AddCamera).
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mvpCameras;
}

void Atlas::SetReferenceMapPoints(const std::vector<MapPointPtr> &vpMPs)
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetReferenceMapPoints(vpMPs);
}

void Atlas::InformNewBigChange()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->InformNewBigChange();
}

int Atlas::GetLastBigChangeIdx()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetLastBigChangeIdx();
}

long unsigned int Atlas::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->MapPointsInMap();
}

long unsigned Atlas::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->KeyFramesInMap();
}

std::vector<KeyFramePtr> Atlas::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllKeyFrames();
}

std::vector<MapPointPtr> Atlas::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetAllMapPoints();
}

std::vector<MapPointPtr> Atlas::GetReferenceMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->GetReferenceMapPoints();
}

std::vector<Map*> Atlas::GetAllMaps()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::vector<Map*> vMaps(mspMaps.begin(),mspMaps.end());
    std::sort(vMaps.begin(), vMaps.end(), compFunctor());
    return vMaps;
}

int Atlas::CountMaps()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mspMaps.size();
}

void Atlas::clearMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->clear();
}

void Atlas::clearAtlas()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    // R6: upstream's commented per-map clear()+delete loop stays retired —
    // Map* is still a raw pointer other threads may hold, so deleting here
    // would risk use-after-free. Maps are deliberately leaked until Map
    // ownership gets its own migration (same policy as RemoveBadMaps).
    mspMaps.clear();
    mpCurrentMap = nullptr;
    mnLastInitKFidMap = 0;
}

Map* Atlas::GetCurrentMap()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    if(!mpCurrentMap)
        CreateNewMap_impl();
    while(mpCurrentMap->IsBad())
        usleep(3000);

    return mpCurrentMap;
}

void Atlas::SetMapBad(Map* pMap)
{
    // P11-A: mutates mspMaps/mspBadMaps — called from the LC thread
    // (MergeLocal) concurrently with Tracking's GetCurrentMap/CreateNewMap.
    // PreSave also calls this, WITHOUT holding mMutexAtlas (single-threaded
    // post-Shutdown path), so no self-deadlock.
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mspMaps.erase(pMap);
    pMap->SetBad();

    mspBadMaps.insert(pMap);
}

void Atlas::RemoveBadMaps()
{
    // P11-A: see SetMapBad.
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    // R6: bad maps are deliberately not deleted (upstream kept this delete
    // loop commented out): raw Map* references may still be live elsewhere.
    mspBadMaps.clear();
}

bool Atlas::isInertial()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->IsInertial();
}

void Atlas::SetInertialSensor()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetInertialSensor();
}

void Atlas::SetImuInitialized()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    mpCurrentMap->SetImuInitialized();
}

bool Atlas::isImuInitialized()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    return mpCurrentMap->isImuInitialized();
}

// P11-A audit note: PreSave/PostLoad/GetAtlasKeyframes and the
// KFDB/vocabulary/viewer setter-getters below stay UNLOCKED deliberately —
// the save/load surface runs single-threaded (SaveAtlas after the P10-5
// Shutdown join chain; LoadAtlas before thread spawn), and the wiring
// setters run in the System constructor before any thread exists
// (happens-before via thread creation). PreSave calls SetMapBad/
// RemoveBadMaps, which now lock — it must NOT hold mMutexAtlas itself.
void Atlas::PreSave()
{
    if(mpCurrentMap){
        if(!mspMaps.empty() && mnLastInitKFidMap < mpCurrentMap->GetMaxKFid())
            mnLastInitKFidMap = mpCurrentMap->GetMaxKFid()+1; //The init KF is the next of current maximum
    }

    struct compFunctor
    {
        inline bool operator()(Map* elem1 ,Map* elem2)
        {
            return elem1->GetId() < elem2->GetId();
        }
    };
    std::copy(mspMaps.begin(), mspMaps.end(), std::back_inserter(mvpBackupMaps));
    sort(mvpBackupMaps.begin(), mvpBackupMaps.end(), compFunctor());

    std::set<GeometricCamera*> spCams(mvpCameras.begin(), mvpCameras.end());
    for(Map* pMi : mvpBackupMaps)
    {
        if(!pMi || pMi->IsBad())
            continue;

        if(pMi->GetAllKeyFrames().size() == 0) {
            // Empty map, erase before of save it.
            SetMapBad(pMi);
            continue;
        }
        pMi->PreSave(spCams);
    }
    RemoveBadMaps();
}

void Atlas::PostLoad()
{
    std::map<unsigned int,GeometricCamera*> mpCams;
    for(GeometricCamera* pCam : mvpCameras)
    {
        mpCams[pCam->GetId()] = pCam;
    }

    mspMaps.clear();
    unsigned long int numKF = 0, numMP = 0;
    for(Map* pMi : mvpBackupMaps)
    {
        mspMaps.insert(pMi);
        pMi->PostLoad(mpKeyFrameDB, mpORBVocabulary, mpCams);
        numKF += pMi->GetAllKeyFrames().size();
        numMP += pMi->GetAllMapPoints().size();
    }
    mvpBackupMaps.clear();
}

void Atlas::SetKeyFrameDababase(KeyFrameDatabase* pKFDB)
{
    mpKeyFrameDB = pKFDB;
}

KeyFrameDatabase* Atlas::GetKeyFrameDatabase()
{
    return mpKeyFrameDB;
}

void Atlas::SetORBVocabulary(ORBVocabulary* pORBVoc)
{
    mpORBVocabulary = pORBVoc;
}

ORBVocabulary* Atlas::GetORBVocabulary()
{
    return mpORBVocabulary;
}

long unsigned int Atlas::GetNumLivedKF()
{
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for(Map* pMap_i : mspMaps)
    {
        num += pMap_i->GetAllKeyFrames().size();
    }

    return num;
}

long unsigned int Atlas::GetNumLivedMP() {
    std::unique_lock<std::mutex> lock(mMutexAtlas);
    long unsigned int num = 0;
    for (Map* pMap_i : mspMaps) {
        num += pMap_i->GetAllMapPoints().size();
    }

    return num;
}

std::map<long unsigned int, KeyFramePtr> Atlas::GetAtlasKeyframes()
{
    std::map<long unsigned int, KeyFramePtr> mpIdKFs;
    for(Map* pMap_i : mvpBackupMaps)
    {
        std::vector<KeyFramePtr> vpKFs_Mi = pMap_i->GetAllKeyFrames();

        for(KeyFramePtr pKF_j_Mi : vpKFs_Mi)
        {
            mpIdKFs[pKF_j_Mi->mnId] = pKF_j_Mi;
        }
    }

    return mpIdKFs;
}

} //namespace ORB_SLAM3
