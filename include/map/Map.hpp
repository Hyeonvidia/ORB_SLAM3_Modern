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


/**
 * Ownership/lifetime (R4b): Map is THE owner of both KeyFrames and MapPoints
 * (mspKeyFrames / mspMapPoints hold the owning shared_ptrs). Removal is still
 * SetBadFlag() — a tombstoned object survives for as long as any other strong
 * holder (frames, queues, pins, side tables) references it, and is reclaimed
 * when the last one lets go. See docs/OWNERSHIP.md before changing any
 * lifetime or lock-order behavior here.
 */

#ifndef MAP_H
#define MAP_H

#include "map/MapPoint.hpp"
#include "map/KeyFrame.hpp"

#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM3
{

class MapPoint;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;

        // Save/load a set structure, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
        //ar & mspKeyFrames;
        //ar & mspMapPoints;
        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(const KeyFramePtr& pKF);
    void AddMapPoint(const MapPointPtr& pMP);
    void EraseMapPoint(const MapPointPtr& pMP);
    void EraseKeyFrame(const KeyFramePtr& pKF);
    void SetReferenceMapPoints(const std::vector<MapPointPtr> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFramePtr> GetAllKeyFrames();
    std::vector<MapPointPtr> GetAllMapPoints();
    std::vector<MapPointPtr> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFramePtr GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void ApplyScaledRotation(const Sophus::SE3f &T, const float s, const bool bScaledVel=false);

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc/*, std::map<long unsigned int, KeyFrame*>& mpKeyFrameId*/, std::map<unsigned int, GeometricCamera*> &mpCams);

    void printReprojectionError(std::list<KeyFramePtr> &lpLocalWindowKFs, const KeyFramePtr& mpCurrentKF, std::string &name, std::string &name_folder);

    std::vector<KeyFramePtr> mvpKeyFrameOrigins;
    std::vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFramePtr mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // DEBUG: show KFs which are used in LBA
    std::set<long unsigned int> msOptKFs;
    std::set<long unsigned int> msFixedKFs;

protected:

    long unsigned int mnId;

    // R4b slices 1+2: THE strong owners of all live MapPoints/KeyFrames in
    // this map. SetBadFlag() erases the entry; a bad object then survives
    // only as long as other strong holders (frames, queues, pins) reference
    // it, and is genuinely reclaimed when the last one lets go.
    std::set<MapPointPtr> mspMapPoints;
    std::set<KeyFramePtr> mspKeyFrames;

    // Save/load, the set structure is broken in libboost 1.58 for ubuntu 16.04, a vector is serializated
    // R4b (2026-08-20): deliberately kept as RAW pointers so the boost
    // archive layout (pointer-tracked vector) is byte-compatible with pre-R4b
    // .osa sessions. PreSave() fills them with .get() of the owning
    // shared_ptrs; on load, boost news the objects here and PostLoad() wraps
    // each exactly once into the owning smart pointer, then clears the
    // vectors (non-owning).
    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFramePtr mpKFinitial;
    KeyFramePtr mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPointPtr> mvpReferenceMapPoints;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    //long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    // Mutex
    std::mutex mMutexMap;

};

} //namespace ORB_SLAM3

#endif // MAP_H
