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
 * Ownership/lifetime (R4b slice 2, 2026-08-20): KeyFrame is shared_ptr-managed
 * (KeyFramePtr, see map/MapTypes.hpp). Map::mspKeyFrames is the owner; removal
 * is still SetBadFlag() (tombstone protocol). Strong-by-default with deliberate
 * cycle breaks:
 *   - child->parent (mpParent) and mPrevKF are STRONG: bad-KF parent-chain
 *     walks (trajectory saving, UpdateFrameIMU) and IMU-chain MergePrevious
 *     need those targets alive.
 *   - parent->children, mNextKF, loop/merge edges are WEAK (lock() to use).
 *   - covisibility (mConnectedKeyFrameWeights / mvpOrderedConnectedKeyFrames)
 *     stays RAW KeyFrame*: mutual strong edges would leak the whole graph.
 *     Contract: SetBadFlag() erases this KF from every partner's maps before
 *     the Map's strong pin is dropped, so a raw entry present in the map is
 *     alive; the public accessors wrap entries into KeyFramePtr pins under
 *     mMutexConnections before handing them out.
 * See docs/OWNERSHIP.md before changing any lifetime or lock-order behavior.
 */

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "map/MapTypes.hpp"
#include "map/MapPoint.hpp"
// R3: upstream DBoW2 submodule + our serialization shim / wrapper layer
#include "recognition/BowTypes.hpp"
#include "recognition/OrbVocabulary.hpp"
#include "features/ORBextractor.hpp"
#include "map/Frame.hpp"
#include "recognition/KeyFrameDatabase.hpp"
#include "backend/ImuTypes.hpp"

#include "camera/GeometricCamera.hpp"
#include "io/SerializationUtils.hpp"

#include <mutex>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>


namespace ORB_SLAM3
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class GeometricCamera;

// enable_shared_from_this: graph maintenance (UpdateConnections, SetBadFlag,
// ChangeParent, EraseMapPointMatch) must hand `this` out as a KeyFramePtr.
// Every KeyFrame is created through std::make_shared (3 Tracking sites) or
// wrapped exactly once in Map::PostLoad on the boost load path, before any
// of those can run — shared_from_this() is never reached from a constructor.
class KeyFrame : public std::enable_shared_from_this<KeyFrame>
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & mnId;
        ar & const_cast<long unsigned int&>(mnFrameId);
        ar & const_cast<double&>(mTimeStamp);
        // Grid
        ar & const_cast<int&>(mnGridCols);
        ar & const_cast<int&>(mnGridRows);
        ar & const_cast<float&>(mfGridElementWidthInv);
        ar & const_cast<float&>(mfGridElementHeightInv);

        // Tracking-only scratch (mnTrackReferenceForFrame) is deliberately
        // not serialized — .osa layout unchanged (upstream format).

        // Scale
        ar & mfScale;
        // Calibration parameters
        ar & const_cast<float&>(fx);
        ar & const_cast<float&>(fy);
        ar & const_cast<float&>(invfx);
        ar & const_cast<float&>(invfy);
        ar & const_cast<float&>(cx);
        ar & const_cast<float&>(cy);
        ar & const_cast<float&>(mbf);
        ar & const_cast<float&>(mb);
        ar & const_cast<float&>(mThDepth);
        serializeMatrix(ar, mDistCoef, version);
        // Number of Keypoints
        ar & const_cast<int&>(N);
        // KeyPoints
        serializeVectorKeyPoints<Archive>(ar, mvKeys, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysUn, version);
        ar & const_cast<std::vector<float>& >(mvuRight);
        ar & const_cast<std::vector<float>& >(mvDepth);
        serializeMatrix<Archive>(ar,mDescriptors,version);
        // BOW
        ar & mBowVec;
        ar & mFeatVec;
        // Pose relative to parent
        serializeSophusSE3<Archive>(ar, mTcp, version);
        // Scale
        ar & const_cast<int&>(mnScaleLevels);
        ar & const_cast<float&>(mfScaleFactor);
        ar & const_cast<float&>(mfLogScaleFactor);
        ar & const_cast<std::vector<float>& >(mvScaleFactors);
        ar & const_cast<std::vector<float>& >(mvLevelSigma2);
        ar & const_cast<std::vector<float>& >(mvInvLevelSigma2);
        // Image bounds and calibration
        ar & const_cast<int&>(mnMinX);
        ar & const_cast<int&>(mnMinY);
        ar & const_cast<int&>(mnMaxX);
        ar & const_cast<int&>(mnMaxY);
        ar & boost::serialization::make_array(mK_.data(), mK_.size());
        // Pose
        serializeSophusSE3<Archive>(ar, mTcw, version);
        // MapPointsId associated to keypoints
        ar & mvBackupMapPointsId;
        // Grid
        ar & mGrid;
        // Connected KeyFrameWeight
        ar & mBackupConnectedKeyFrameIdWeights;
        // Spanning Tree and Loop Edges
        ar & mbFirstConnection;
        ar & mBackupParentId;
        ar & mvBackupChildrensId;
        ar & mvBackupLoopEdgesId;
        ar & mvBackupMergeEdgesId;
        // Bad flags
        ar & mbNotErase;
        ar & mbToBeErased;
        ar & mbBad;

        ar & mHalfBaseline;

        ar & mnOriginMapId;

        // Camera variables
        ar & mnBackupIdCamera;
        ar & mnBackupIdCamera2;

        // Fisheye variables
        ar & mvLeftToRightMatch;
        ar & mvRightToLeftMatch;
        ar & const_cast<int&>(NLeft);
        ar & const_cast<int&>(NRight);
        serializeSophusSE3<Archive>(ar, mTlr, version);
        serializeVectorKeyPoints<Archive>(ar, mvKeysRight, version);
        ar & mGridRight;

        // Inertial variables
        ar & mImuBias;
        ar & mBackupImuPreintegrated;
        ar & mImuCalib;
        ar & mBackupPrevKFId;
        ar & mBackupNextKFId;
        ar & bImu;
        ar & boost::serialization::make_array(mVw.data(), mVw.size());
        ar & boost::serialization::make_array(mOwb.data(), mOwb.size());
        ar & mbHasVelocity;
    }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrame();

    // R4c: noncopyable (identity node in the covisibility graph; per-instance
    // mutexes already forbade copies implicitly — now documented).
    KeyFrame(const KeyFrame&) = delete;
    KeyFrame& operator=(const KeyFrame&) = delete;
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const Sophus::SE3f &Tcw);
    void SetVelocity(const Eigen::Vector3f &Vw_);

    Sophus::SE3f GetPose();

    Sophus::SE3f GetPoseInverse();
    Eigen::Vector3f GetCameraCenter();

    Eigen::Vector3f GetImuPosition();
    Eigen::Matrix3f GetImuRotation();
    Sophus::SE3f GetImuPose();
    Eigen::Matrix3f GetRotation();
    Eigen::Vector3f GetTranslation();
    Eigen::Vector3f GetVelocity();
    bool isVelocitySet();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions.
    // Internal mutators take raw KeyFrame* (storage is raw — cycle break);
    // the accessors return KeyFramePtr PINS wrapped under mMutexConnections
    // (safe: an entry still present in the maps has not completed SetBadFlag,
    // hence is still strongly held by its Map).
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);

    void UpdateConnections(bool upParent=true);
    void UpdateBestCovisibles();
    std::set<KeyFramePtr> GetConnectedKeyFrames();
    std::vector<KeyFramePtr> GetVectorCovisibleKeyFrames();
    std::vector<KeyFramePtr> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFramePtr> GetCovisiblesByWeight(const int &w);
    int GetWeight(const KeyFramePtr& pKF);

    // Spanning tree functions (child->parent strong, parent->children weak)
    void AddChild(const KeyFramePtr& pKF);
    void EraseChild(const KeyFramePtr& pKF);
    void ChangeParent(const KeyFramePtr& pKF);
    std::set<KeyFramePtr> GetChilds();
    KeyFramePtr GetParent();
    bool hasChild(const KeyFramePtr& pKF);
    void SetFirstConnection(bool bFirst);

    // Loop Edges (weak storage; loop partners are latch-pinned in the Map)
    void AddLoopEdge(const KeyFramePtr& pKF);
    std::set<KeyFramePtr> GetLoopEdges();

    // Merge Edges (weak storage; culled partners simply expire)
    void AddMergeEdge(const KeyFramePtr& pKF);
    std::set<KeyFramePtr> GetMergeEdges();

    // MapPoint observation functions
    int GetNumberMPs();
    void AddMapPoint(const MapPointPtr& pMP, const size_t &idx);
    void EraseMapPointMatch(const int &idx);
    void EraseMapPointMatch(const MapPointPtr& pMP);
    void ReplaceMapPointMatch(const int &idx, const MapPointPtr& pMP);
    std::set<MapPointPtr> GetMapPoints();
    std::vector<MapPointPtr> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    MapPointPtr GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const bool bRight = false) const;
    bool UnprojectStereo(int i, Eigen::Vector3f &x3D);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(const KeyFramePtr& pKF1, const KeyFramePtr& pKF2){
        return pKF1->mnId<pKF2->mnId;
    }

    Map* GetMap();
    void UpdateMap(Map* pMap);

    void SetNewBias(const IMU::Bias &b);
    Eigen::Vector3f GetGyroBias();

    Eigen::Vector3f GetAccBias();

    IMU::Bias GetImuBias();

    bool ProjectPointDistort(const MapPointPtr& pMP, cv::Point2f &kp, float &u, float &v);
    bool ProjectPointUnDistort(const MapPointPtr& pMP, cv::Point2f &kp, float &u, float &v);

    void PreSave(std::set<KeyFramePtr>& spKF,std::set<MapPointPtr>& spMP, std::set<GeometricCamera*>& spCam);
    void PostLoad(std::map<long unsigned int, KeyFramePtr>& mpKFid, std::map<long unsigned int, MapPointPtr>& mpMPid, std::map<unsigned int, GeometricCamera*>& mpCamId);


    void SetORBVocabulary(ORBVocabulary* pORBVoc);
    void SetKeyFrameDatabase(KeyFrameDatabase* pKFDB);

    bool bImu;

    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;

    // P5-C: the local-BA epoch marks (mnBALocalForKF/mnBAFixedForKF) were
    // externalized into BAEpochs (include/backend/BAEpochs.hpp), a persistent
    // table owned by System and threaded through LocalMapping/LoopClosing
    // into the Optimizer local-BA functions (values cross call boundaries:
    // FullInertialBA(bFixLocal=true) reads marks left by a previous call).

    //Number of optimizations by BA(amount of iterations in BA)

    // P5-3: the per-query keyframe-database scribble fields (mn*Query/mn*Words/
    // m*Score for Loop/Merge/PlaceRecognition/Reloc) were externalized into
    // function-local scratch maps in KeyFrameDatabase.cpp.

    // P5-D: the per-GBA result fields (mTcwGBA/mTcwBefGBA/mVwbGBA/mBiasGBA/
    // mnBAGlobalForKF) were externalized into GBAResult
    // (include/backend/GBAResult.hpp) and a propagation-local tcwBefGBA map
    // in LoopClosing.cpp/LocalMapping.cpp.

    // P5-G: the per-merge scribble fields (mTcwMerge/mTcwBefMerge/mTwcBefMerge/
    // mVwbMerge/mnBALocalForMerge) were externalized into MergeScratch
    // (include/closing/MergeScratch.hpp) and function-local sets in Optimizer.cpp.

    float mfScale;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;
    cv::Mat mDistCoef;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    Sophus::SE3f mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;

    // IMU chain (R4b slice 2): prev STRONG — culling's MergePrevious and the
    // propagation walks need the predecessor alive even after it goes bad;
    // next WEAK — the reverse edge would close a cycle, and culling rewires
    // both directions anyway. lock() mNextKF before use (null == "rewired
    // away or reclaimed", the same null the raw pointer produced).
    KeyFramePtr mPrevKF;
    KeyFrameWeakPtr mNextKF;

    // IMU CONTRACT (docs/IMU_CONTRACT.md §2): preintegration covering
    // mPrevKF -> this. Ownership arrives from Tracking (this ctor copies the
    // Frame's pointer; Tracking then re-points its member without delete).
    // Never freed — KeyFrames are tombstoned, not deleted (docs/OWNERSHIP.md).
    // Mutated in place by LocalMapping/Optimizer (Reintegrate/SetNewBias/
    // MergePrevious). May be nullptr if PreintegrateIMU early-exited (hazard B6).
    IMU::Preintegrated* mpImuPreintegrated;
    IMU::Calib mImuCalib;

    unsigned int mnOriginMapId;

    std::string mNameFile;

    int mnDataset;

    // The following variables need to be accessed trough a mutex to be thread safe.
protected:
    // sophus poses
    Sophus::SE3<float> mTcw;
    Eigen::Matrix3f mRcw;
    Sophus::SE3<float> mTwc;
    Eigen::Matrix3f mRwc;

    // IMU position
    Eigen::Vector3f mOwb;
    // Velocity (Only used for inertial SLAM)
    Eigen::Vector3f mVw;
    bool mbHasVelocity;

    //Transformation matrix between cameras in stereo fisheye
    Sophus::SE3<float> mTlr;
    Sophus::SE3<float> mTrl;

    // Imu bias
    IMU::Bias mImuBias;

    // MapPoints associated to keypoints.
    // R4b slice 1: STRONG on purpose (tombstone semantics) — a slot can hold
    // a bad MapPoint between its SetBadFlag() and this KF's EraseMapPointMatch,
    // and that point must stay alive for the lock-free readers that copied it.
    std::vector<MapPointPtr> mvpMapPoints;
    // For save relation without pointer, this is necessary for save/load function
    std::vector<long long int> mvBackupMapPointsId;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    // Covisibility storage stays RAW on purpose (R4b slice 2): the edges are
    // mutual, so strong pointers would make the whole covisibility graph one
    // big cycle and Map::clear() would free nothing. SetBadFlag() scrubs this
    // KF out of every partner before the Map pin drops (see class comment).
    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;
    // For save relation without pointer, this is necessary for save/load function
    std::map<long unsigned int, int> mBackupConnectedKeyFrameIdWeights;

    // Spanning Tree and Loop Edges (R4b slice 2): child->parent STRONG
    // (bad-KF parent chains must stay walkable), parent->children WEAK.
    // Loop/merge edges WEAK: loop partners are permanently Map-pinned by the
    // SetNotErase latch (mspLoopEdges entries never expire while the map
    // lives, so SetErase's empty() test is unchanged); merge partners may be
    // culled — an expired entry reads as gone instead of dangling.
    bool mbFirstConnection;
    KeyFramePtr mpParent;
    std::set<KeyFrameWeakPtr, std::owner_less<KeyFrameWeakPtr>> mspChildrens;
    std::set<KeyFrameWeakPtr, std::owner_less<KeyFrameWeakPtr>> mspLoopEdges;
    std::set<KeyFrameWeakPtr, std::owner_less<KeyFrameWeakPtr>> mspMergeEdges;
    // For save relation without pointer, this is necessary for save/load function
    long long int mBackupParentId;
    std::vector<long unsigned int> mvBackupChildrensId;
    std::vector<long unsigned int> mvBackupLoopEdgesId;
    std::vector<long unsigned int> mvBackupMergeEdgesId;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    // Backup variables for inertial
    long long int mBackupPrevKFId;
    long long int mBackupNextKFId;
    IMU::Preintegrated mBackupImuPreintegrated;

    // Backup for Cameras
    unsigned int mnBackupIdCamera, mnBackupIdCamera2;

    // Calibration
    Eigen::Matrix3f mK_;

    // Mutex
    std::mutex mMutexPose; // for pose, velocity and biases
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
    std::mutex mMutexMap;

public:
    GeometricCamera* mpCamera, *mpCamera2;

    //Indexes of stereo observations correspondences
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;

    Sophus::SE3f GetRelativePoseTrl();
    Sophus::SE3f GetRelativePoseTlr();

    //KeyPoints in the right image (for stereo fisheye, coordinates are needed)
    const std::vector<cv::KeyPoint> mvKeysRight;

    const int NLeft, NRight;

    std::vector< std::vector <std::vector<size_t> > > mGridRight;

    Sophus::SE3<float> GetRightPose();
    Sophus::SE3<float> GetRightPoseInverse();

    Eigen::Vector3f GetRightCameraCenter();
    Eigen::Matrix<float,3,3> GetRightRotation();
    Eigen::Vector3f GetRightTranslation();

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (NLeft != -1) ? NLeft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        std::cout << "Point distribution in KeyFrame: left-> " << left << " --- right-> " << right << std::endl;
    }


};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
