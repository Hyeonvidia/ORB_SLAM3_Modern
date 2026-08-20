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
 * Ownership/lifetime (R4b slices 1+2, 2026-08-20): MapPoint is
 * shared_ptr-managed (MapPointPtr, see map/MapTypes.hpp). Map::mspMapPoints is
 * the owner; every other MapPoint holder is deliberately strong so tombstone
 * liveness is preserved. SetBadFlag() is still the removal protocol.
 * Slice 2: the MP -> KF back-references are the deliberate KF<->MP cycle
 * break — mObservations keys stay RAW KeyFrame* (scrub contract: a key
 * present in the map has not completed KeyFrame::SetBadFlag and is Map-pinned;
 * accessors wrap keys into KeyFramePtr pins under mMutexFeatures), and
 * mpRefKF/mpHostKF are WEAK (a bad MapPoint may outlive its reference KF —
 * GetReferenceKeyFrame() then returns null instead of dangling).
 * See docs/OWNERSHIP.md before changing any lifetime or lock-order behavior.
 */

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "map/MapTypes.hpp"
#include "map/KeyFrame.hpp"
#include "map/Frame.hpp"
#include "map/Map.hpp"
#include "io/Converter.hpp"

#include "io/SerializationUtils.hpp"

#include <opencv2/core/core.hpp>
#include <memory>
#include <mutex>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/array.hpp>
#include <boost/serialization/map.hpp>

namespace ORB_SLAM3
{

class KeyFrame;
class Map;
class Frame;

// enable_shared_from_this: SetBadFlag()/Replace() must hand `this` to
// Map::EraseMapPoint as a MapPointPtr. Every MapPoint is created through
// std::make_shared (or wrapped exactly once in Map::PostLoad on the boost
// load path) before either can run, so shared_from_this() is always valid.
class MapPoint : public std::enable_shared_from_this<MapPoint>
{

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnFirstKFid;
        ar & mnFirstFrame;
        ar & nObs;
        // Per-frame tracking scratch (mTrackProj*/mbTrackInView*/scale/
        // view-cos/mnTrackReferenceForFrame/mnLastFrameSeen) is deliberately
        // not serialized — .osa layout unchanged (upstream format).

        // Protected variables
        ar & boost::serialization::make_array(mWorldPos.data(), mWorldPos.size());
        ar & boost::serialization::make_array(mNormalVector.data(), mNormalVector.size());
        ar & mBackupObservationsId1;
        ar & mBackupObservationsId2;
        serializeMatrix(ar,mDescriptor,version);
        ar & mBackupRefKFId;

        ar & mbBad;
        ar & mBackupReplacedId;

        ar & mfMinDistance;
        ar & mfMaxDistance;

    }


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint();

    // R4c: noncopyable (identity node with per-instance mutexes; same as KeyFrame).
    MapPoint(const MapPoint&) = delete;
    MapPoint& operator=(const MapPoint&) = delete;

    MapPoint(const Eigen::Vector3f &Pos, const KeyFramePtr& pRefKF, Map* pMap);
    MapPoint(const double invDepth, cv::Point2f uv_init, const KeyFramePtr& pRefKF, const KeyFramePtr& pHostKF, Map* pMap);
    MapPoint(const Eigen::Vector3f &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const Eigen::Vector3f &Pos);
    Eigen::Vector3f GetWorldPos();

    Eigen::Vector3f GetNormal();
    void SetNormalVector(const Eigen::Vector3f& normal);

    KeyFramePtr GetReferenceKeyFrame();

    // Keys are wrapped into KeyFramePtr pins under mMutexFeatures (see the
    // ownership comment above) — the caller's copy keeps every observing
    // KeyFrame alive for the duration of its use.
    std::map<KeyFramePtr,std::tuple<int,int>> GetObservations();
    int Observations();

    void AddObservation(const KeyFramePtr& pKF,int idx);
    void EraseObservation(const KeyFramePtr& pKF);

    std::tuple<int,int> GetIndexInKeyFrame(const KeyFramePtr& pKF);
    bool IsInKeyFrame(const KeyFramePtr& pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(const MapPointPtr& pMP);    
    MapPointPtr GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, const KeyFramePtr& pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void PrintObservations();

    void PreSave(std::set<KeyFramePtr>& spKF,std::set<MapPointPtr>& spMP);
    void PostLoad(std::map<long unsigned int, KeyFramePtr>& mpKFid, std::map<long unsigned int, MapPointPtr>& mpMPid);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackDepth;
    float mTrackDepthR;
    float mTrackProjXR;
    float mTrackProjYR;
    bool mbTrackInView, mbTrackInViewR;
    int mnTrackScaleLevel, mnTrackScaleLevelR;
    float mTrackViewCos, mTrackViewCosR;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // P5-C: the local-BA epoch mark (mnBALocalForKF) was externalized into
    // BAEpochs (include/backend/BAEpochs.hpp), a persistent table owned by
    // System and threaded through LocalMapping/LoopClosing into the Optimizer
    // local-BA functions.

    // P5-D: the per-GBA result fields (mPosGBA/mnBAGlobalForKF) were
    // externalized into GBAResult (include/backend/GBAResult.hpp).

    // P5-G: the per-merge scribble fields (mnBALocalForMerge/mPosMerge/
    // mNormalVectorMerge) were externalized into MergeScratch
    // (include/closing/MergeScratch.hpp) and function-local sets in Optimizer.cpp.

    // Fopr inverse depth optimization
    double mInvDepth;
    double mInitU;
    double mInitV;
    // R4b slice 2: WEAK (write-only member; a strong ref would feed the
    // KF<->MP cycle for no benefit)
    KeyFrameWeakPtr mpHostKF;

    static std::mutex mGlobalMutex;

    unsigned int mnOriginMapId;

protected:    

     // Position in absolute coordinates
     Eigen::Vector3f mWorldPos;

     // Keyframes observing the point and associated index in keyframe.
     // R4b slice 2: RAW keys on purpose — the KF<->MP cycle break. Scrub
     // contract: KeyFrame::SetBadFlag erases its key from here (under
     // mMutexFeatures) before the Map's strong pin on that KF can drop, so
     // a present key is always alive; GetObservations() wraps keys into
     // pins under the same lock.
     std::map<KeyFrame*,std::tuple<int,int> > mObservations;
     // For save relation without pointer, this is necessary for save/load function
     std::map<long unsigned int, int> mBackupObservationsId1;
     std::map<long unsigned int, int> mBackupObservationsId2;

     // Mean viewing direction
     Eigen::Vector3f mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference KeyFrame. R4b slice 2: WEAK — a bad MapPoint keeps its last
     // mpRefKF value with no scrub, so a strong ref would leak and a raw one
     // could dangle once the KF is reclaimed; lock() returns null instead.
     // For a LIVE MapPoint the invariant "mpRefKF is an observation key (or
     // null)" holds (EraseObservation reassigns it), so lock() succeeds.
     KeyFrameWeakPtr mpRefKF;
     long unsigned int mBackupRefKFId;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (tombstone marker; the object lives while referenced)
     bool mbBad;
     // R4b slice 1: STRONG — Replace() chains must stay walkable for
     // CheckReplacedInLastFrame and trajectory recovery. A hypothetical
     // replace cycle would leak, which is exactly the pre-migration
     // tombstone behavior (leak-equivalent, accepted).
     MapPointPtr mpReplaced;
     // For save relation without pointer, this is necessary for save/load function
     long long int mBackupReplacedId;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     // Mutex
     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
     std::mutex mMutexMap;

};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
