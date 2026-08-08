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


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "viz/Viewer.hpp"
#include "viz/FrameDrawer.hpp"
#include "map/Atlas.hpp"
#include "mapping/LocalMapping.hpp"
#include "closing/LoopClosing.hpp"
#include "map/Frame.hpp"
#include "features/ORBVocabulary.hpp"
#include "recognition/KeyFrameDatabase.hpp"
#include "features/ORBextractor.hpp"
#include "viz/MapDrawer.hpp"
#include "core/System.hpp"
#include "backend/ImuTypes.hpp"
#include "core/Settings.hpp"
#include "geometry/TwoViewInitializer.hpp"

#include "camera/GeometricCamera.hpp"

#include <mutex>
#include <unordered_set>
#include <fstream>
#include <ostream>

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;
class ITrackingOptimizer;

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings,
             ITrackingOptimizer* pOptimizer, const string &_nameSeq=std::string());

    ~Tracking();

    // Parse the config file

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        RECENTLY_LOST=3,
        LOST=4,
        OK_KLT=5
    };

private:

    // The state machine itself. Declared HERE, at the exact position the old
    // public `mState` / `mLastProcessedState` members occupied, so that the
    // constructor's member-initialisation ORDER is bit-for-bit unchanged.
    eTrackingState mState_;
    eTrackingState mLastProcessedState_;

public:

    // ------------------------------------------------------------------
    // P7-1a: the tracking state machine.
    //
    // `mState` and `mLastProcessedState` used to be public data members
    // written and read from three different threads. They are now private
    // (trailing underscore) and reached through the accessors below. The
    // exhaustive site census and transition table this refactor preserves
    // live in docs/P7_RECON.md §A -- 15 writes and 25 comparisons in
    // Tracking.cpp, 4 reads in System.cpp, 2 reads plus ONE cross-thread
    // write in LocalMapping.cpp, 3 reads in FrameDrawer.cpp.
    //
    // This is a pure encapsulation change: same transitions, same order,
    // same values. No locking is introduced -- see GetState().
    // ------------------------------------------------------------------

    // Plain, deliberately UNSYNCHRONIZED read of the tracking state.
    //
    // Every pre-P7 read of `mState` was unsynchronized, including the
    // cross-thread ones (docs/P7_RECON.md F11). Adding a lock here would
    // change the visibility/timing semantics the current golden baselines
    // were measured against, so it is intentionally omitted. Making the
    // state machine properly thread-safe is P10's job, not P7's.
    eTrackingState GetState() const { return mState_; }

    // The tracking state as it was at the top of the current Track() call,
    // snapshotted once per frame (Tracking.cpp, right after the
    // NO_IMAGES_YET -> NOT_INITIALIZED promotion). It is NOT a second state
    // machine; its sole consumer is FrameDrawer::Update. See P7_RECON §3.
    eTrackingState GetLastProcessedState() const { return mLastProcessedState_; }

    // Cross-thread state write, preserved verbatim from upstream.
    //
    // LocalMapping::InitializeIMU() ends by forcing the tracker back to OK
    // (upstream `mpTracker->mState = Tracking::OK;`). That assignment is
    // issued from the LOCAL MAPPING thread, not the tracking thread, and is
    // unsynchronized with respect to the tracking thread's own reads --
    // by design, inherited from upstream. It is called while LocalMapping
    // holds pCurrentMap->mMutexMapUpdate, which happens to cover the middle
    // of Track() but NOT its prologue. See the warning block at the top of
    // docs/P7_RECON.md and finding F11 there.
    //
    // This method exists so the one legitimate external writer has a named,
    // greppable entry point instead of a raw public data member; it must
    // keep exactly these semantics until P10 revisits threading.
    void NotifyImuInitialized();

    // Human-readable enumerator name, used by the transition trace.
    static const char* StateName(eTrackingState s);

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;
    Frame mLastFrame;

    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;


    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;
    vector<double> vdORBExtract_ms;
    vector<double> vdStereoMatch_ms;
    vector<double> vdIMUInteg_ms;
    vector<double> vdPosePred_ms;
    vector<double> vdLMTrack_ms;
    vector<double> vdNewKF_ms;
    vector<double> vdTrackTotal_ms;
#endif

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    //void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();
    bool PredictStateIMU();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // Imu preintegration from last frame
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    // Narrow optimizer interface, owned by System (G2oBackend value member,
    // P6-A ISP split — see include/backend/ITrackingOptimizer.hpp).
    ITrackingOptimizer* mpOptimizer;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;
    float mbf;
    float mImageScale;

    float mImuFreq;
    double mImuPer;
    bool mInsertKFsLost;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    int mnFirstImuFrameId;
    int mnFramesToResetIMU = 0;  // upstream new-format path never set this (see P2-2 note)

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    double mTimeStampLost;
    double time_recently_lost;

    unsigned int mnFirstFrameId;
    unsigned int mnInitialFrameId;
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;

    //Motion Model
    bool mbVelocity{false};
    Sophus::SE3f mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;

    // P3-3: monocular two-view init orchestrator (moved out of camera models)
    TwoViewInitializer mTwoViewInit;

    int initID, lastID;

    Sophus::SE3f mTlr;

    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

private:

    // ------------------------------------------------------------------
    // P7-1a: the single mutator for the tracking state, plus opt-in
    // transition instrumentation.
    //
    // SetState() is the ONLY place mState_ is written after construction.
    // It is intentionally trivial: assign, then (only when tracing is on)
    // record. No lock, no validation, no side effects -- anything else
    // would be a behavior change, and P7-1a is a no-behavior-change edit.
    //
    // `reason` is a literal describing the call site; it lands verbatim in
    // the trace line. `bLogEvenIfUnchanged` forces a no-op self-assignment
    // (from == to) to be traced anyway. Exactly one caller sets it:
    // NotifyImuInitialized(), because the LocalMapping -> Tracking handoff
    // usually fires while the tracker is already OK, and dropping it would
    // make the cross-thread write invisible in the trace even though it
    // definitely happened. Every other site's self-assignment (the OK->OK
    // at the top of the normal tracking path, which fires on essentially
    // every frame) is pure noise and is dropped -- verified against
    // docs/P7_RECON.md §A: no site depends on re-assigning the same value,
    // since the assignment has no observable effect when from == to.
    //
    // Cost when tracing is off: one enum load and one bool test.
    // ------------------------------------------------------------------
    void SetState(eTrackingState s, const char* reason, bool bLogEvenIfUnchanged = false)
    {
        const eTrackingState prev = mState_;
        mState_ = s;
        if(mbTraceState && (prev != s || bLogEvenIfUnchanged))
            TraceStateTransition(prev, s, reason);
    }

    void TraceStateTransition(eTrackingState from, eTrackingState to, const char* reason);

    // Returns the trace sink; the caller must hold mMutexTraceState.
    std::ostream& TraceStateSink();

    // Enabled by the environment variable ORB_TRACE_STATE (default OFF).
    bool mbTraceState{false};
    // Destination, from ORB_TRACE_STATE_FILE; falls back to stderr.
    std::ofstream mTraceStateFile;
    // Guards the SINK ONLY (the trace is written from both the tracking and
    // the local-mapping thread). It is never taken when tracing is off, so
    // the production path keeps exactly the unsynchronized semantics that
    // docs/P7_RECON.md F11 describes.
    std::mutex mMutexTraceState;

public:
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
