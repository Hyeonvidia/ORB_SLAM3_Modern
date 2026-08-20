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
#include "map/MapTypes.hpp"  // R4b: MapPointPtr
#include "core/SensorType.hpp"      // R4c: Sensor
#include "tracking/TrackingState.hpp"  // R4c: TrackingState
#include <opencv2/features2d/features2d.hpp>

// P7-1b: this header no longer includes core/System.hpp (Tracking now holds
// an IResetRequester*, not a System*) nor viz/Viewer.hpp (Viewer is used only
// through a pointer; the forward declaration below suffices, and Tracking.cpp
// includes the full header). Together with the existing forward declarations
// this breaks the System.hpp -> Tracking.hpp -> {System.hpp, Viewer.hpp ->
// System.hpp} include cycles documented in P6-1/P7_RECON §B.5.3.
#include "viz/FrameDrawer.hpp"
#include "map/Atlas.hpp"
#include "mapping/LocalMapping.hpp"
#include "closing/LoopClosing.hpp"
#include "map/Frame.hpp"
#include "recognition/OrbVocabulary.hpp"
#include "recognition/KeyFrameDatabase.hpp"
#include "features/ORBextractor.hpp"
#include "viz/MapDrawer.hpp"
#include "backend/ImuTypes.hpp"
#include "core/Settings.hpp"
#include "geometry/TwoViewInitializer.hpp"

#include "camera/GeometricCamera.hpp"

#include <atomic>
#include <mutex>
#include <optional>
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
class Settings;
class ITrackingOptimizer;
class IResetRequester;

class Tracking
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // P7-1b: the former `System* pSys` back-reference is replaced by the
    // narrow IResetRequester* (constructor injection, same pattern as
    // ITrackingOptimizer/BAEpochs). System passes itself; Tracking can only
    // request a deferred active-map reset through it.
    Tracking(IResetRequester* pResetRequester, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const std::string &strSettingPath, const Sensor sensor, Settings* settings,
             ITrackingOptimizer* pOptimizer, const std::string &_nameSeq=std::string());

    ~Tracking();

    // R4c: noncopyable by design (thread-owned state machine).
    Tracking(const Tracking&) = delete;
    Tracking& operator=(const Tracking&) = delete;

    // Parse the config file

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, std::string filename);
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, std::string filename);
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, std::string filename);

    // IMU CONTRACT (docs/IMU_CONTRACT.md §1): sole producer of mlQueueImuData.
    // Called only from System::Track{Stereo,RGBD,Monocular} on the tracking
    // thread, with the batch of samples t <= t_frame, just before GrabImage*.
    // No validation: non-monotonic push timestamps silently corrupt
    // preintegration downstream.
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

    // ------------------------------------------------------------------
    // P11-A: IMU rescale/rebias message passing (docs/P11_RECON.md 1부 (1c)).
    //
    // UpdateFrameIMU used to be CALLED on the LocalMapping thread
    // (ImuInitializer, 3 sites) and the LoopClosing thread (MergeLocal2,
    // 2 sites) — a cross-thread mutation of Tracking's Frame objects
    // (mCurrentFrame/mLastFrame have ~100 unguarded members each) that
    // raced Tracking's pre-lock frame pipeline even at the call sites that
    // held mMutexMapUpdate (TSAN T3's surviving Frame::operator= class,
    // benchmark/tsan/step_P10-2_T3.ledger). LM/LC now post an ImuUpdateMsg
    // into a single mutex'd slot; the tracking thread applies it itself at
    // the top of Track(), immediately after acquiring mMutexMapUpdate and
    // before any frame state is consumed (ApplyPendingImuUpdate). Frame
    // objects are tracking-thread-confined from P11-A on (the FrameDrawer
    // copy is already under its own mutex). DIVERGENCES #28.
    //
    // Slot semantics — one pending message, composed on overwrite:
    //   * scale COMPOSES MULTIPLICATIVELY. It is not state but a delta: it
    //     rescales mlRelativeFramePoses translations to follow a map-side
    //     ApplyScaledRotation, and every posted rescale must land exactly
    //     once (upstream ran them synchronously in call order). Plain
    //     last-writer-wins would drop the s=mScale rescale whenever the
    //     always-following s=1.0 rebias post (ImuInitializer) overwrote it
    //     before Tracking got to apply.
    //   * bias/pBaseKF are absolute state anchored at the newest base KF —
    //     for these, last-writer-wins is correct: a newer message carries
    //     the full rebias/re-anchor state and supersedes the older one.
    //   * bFirstInit ORs (a first-init flag must never be lost).
    // ------------------------------------------------------------------
    struct ImuUpdateMsg
    {
        float scale = 1.0f;          // mlRelativeFramePoses rescale (delta; composes)
        IMU::Bias bias;              // absolute rebias state
        KeyFramePtr pBaseKF = nullptr; // anchor: becomes mpLastKeyFrame; frames re-anchored on it
        bool bFirstInit = false;     // first IMU init: T stamps t0IMU at application
        // Poster-side anchor timestamp (pBaseKF->mTimeStamp, an immutable
        // field — race-free to read on LM/LC). Diagnostic only: the applied
        // t0IMU is Tracking's OWN current-frame timestamp at application
        // (bounded semantic shift, documented in DIVERGENCES #28).
        double t0Imu = 0.0;
    };

    // Called from the LM/LC threads (the former cross-thread UpdateFrameIMU
    // call sites). Never blocks on Tracking; only the slot mutex is taken.
    void PostImuUpdate(const ImuUpdateMsg &msg);

    KeyFramePtr GetLastKeyFrame()
    {
        // P10-2 (ledger race R-f) made this an atomic; R4b slice 2 upgrades
        // the slot to a mutex-guarded KeyFramePtr (SharedSlot) — the copy
        // LoopClosing/GBA receive is now a strong pin, not a bare pointer.
        return mpLastKeyFrame.load(std::memory_order_acquire);
    }

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    // P7-1b: the two `SaveSubTrajectory` debug overloads were deleted. They
    // had zero callers here AND upstream (docs/P7_RECON.md §B.3), and their
    // only effect was a Tracking -> System -> Tracking round trip (System
    // reading Tracking's own mlRelativeFramePoses/mlpReferences back out).
    // Dead-code removal, no behavior change.

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states (R4c: scoped enum at namespace scope — see
    // tracking/TrackingState.hpp; the re-exports keep Tracking::OK and
    // Tracking::eTrackingState valid at every external site).
    using eTrackingState = TrackingState;
    using enum TrackingState;

private:

    // The state machine itself. Declared HERE, at the exact position the old
    // public `mState` / `mLastProcessedState` members occupied, so that the
    // constructor's member-initialisation ORDER is bit-for-bit unchanged.
    //
    // P11-A: mState_'s storage is now a (lock-free, same-size) atomic. The
    // state is pull-read from the LocalMapping thread (GetState at
    // LocalMapping.cpp:233/536) and written cross-thread by exactly one
    // site (NotifyImuInitialized, LM thread); those were plain-load/store
    // races since upstream. Relaxed ordering everywhere: the reads are
    // heuristic pull-reads, not a synchronization protocol — no ordering
    // guarantee is added, only the data race (UB) is removed.
    std::atomic<eTrackingState> mState_;
    static_assert(std::atomic<eTrackingState>::is_always_lock_free,
                  "mState_ must stay a lock-free scalar");
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

    // Relaxed atomic read of the tracking state.
    //
    // Every pre-P7 read of `mState` was unsynchronized, including the
    // cross-thread ones (docs/P7_RECON.md F11). P11-A keeps exactly those
    // visibility/timing semantics (no lock, no ordering) but moves the
    // storage to an atomic so the cross-thread reads stop being data races
    // (UB). Pull-read semantics unchanged.
    eTrackingState GetState() const { return mState_.load(std::memory_order_relaxed); }

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
    // greppable entry point instead of a raw public data member. P11-A: the
    // underlying storage is atomic now, so this cross-thread write is no
    // longer a data race; its unsynchronized timing semantics are unchanged.
    void NotifyImuInitialized();

    // Human-readable enumerator name, used by the transition trace.
    static const char* StateName(eTrackingState s);

    // Input sensor (R4c: scoped Sensor enum, core/SensorType.hpp; was int)
    Sensor mSensor;

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
    std::list<Sophus::SE3f> mlRelativeFramePoses;
    std::list<KeyFramePtr> mlpReferences;
    std::list<double> mlFrameTimes;
    std::list<bool> mlbLost;

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
    // Time-stamp of IMU initialization. P11-A: tracking-thread-confined —
    // the former cross-thread writer (ImuInitializer.cpp, LM thread) now
    // rides the ImuUpdateMsg bFirstInit flag and Tracking stamps this with
    // its own mCurrentFrame.mTimeStamp at message application. Write-only
    // member (no reader in the repo), kept for upstream parity.
    double t0IMU;
    bool mFastInit = false;


    std::vector<MapPointPtr> GetLocalMapMPS();

    bool mbWriteStats;

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    std::vector<double> vdRectStereo_ms;
    std::vector<double> vdResizeImage_ms;
    std::vector<double> vdORBExtract_ms;
    std::vector<double> vdStereoMatch_ms;
    std::vector<double> vdIMUInteg_ms;
    std::vector<double> vdPosePred_ms;
    std::vector<double> vdLMTrack_ms;
    std::vector<double> vdNewKF_ms;
    std::vector<double> vdTrackTotal_ms;
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

    // ------------------------------------------------------------------
    // P11-A: tracking-thread side of the IMU message protocol.
    //
    // ApplyPendingImuUpdate() is called at exactly ONE point: the top of
    // Track(), immediately after acquiring pCurrentMap->mMutexMapUpdate and
    // before any frame state is consumed. That point is naturally AFTER
    // this frame's PreintegrateIMU() (which runs pre-lock), so the old
    // cross-thread wait for imuIsPreintegrated() — the W9 spin, with its
    // B1 infinite-hang risk on PreintegrateIMU's setIntegrated-less early
    // return — dissolves (docs/IMU_CONTRACT.md §5/§6 B1, retired).
    //
    // DiscardPendingImuUpdate() drops the slot on the three paths that
    // re-default mCurrentFrame/mLastFrame and retire the message's anchor
    // state (Reset, ResetActiveMap, CreateMapInAtlas). Upstream ordering
    // equivalence: on the reset paths the LM handshake (RequestReset*)
    // means no upstream UpdateFrameIMU could land after the frames were
    // re-defaulted; on the CreateMapInAtlas fork upstream's concurrent
    // call could land on default-constructed frames — a documented crash
    // window (IMU_CONTRACT B10/B11) that discarding demotes to a dropped
    // (already map-retired) update. DIVERGENCES #28.
    //
    // UpdateFrameIMU is the unchanged upstream mutation body, now
    // tracking-thread-only (minus the W9 spin).
    // ------------------------------------------------------------------
    void ApplyPendingImuUpdate();
    void DiscardPendingImuUpdate();
    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFramePtr pCurrentKeyFrame);

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();

    bool mbMapUpdated;

    // IMU CONTRACT (docs/IMU_CONTRACT.md §2-§3): preintegration accumulated
    // since the last KeyFrame (upstream's "from last frame" comment was wrong).
    // Owned by Tracking until a KeyFrame is built from the frame aliasing it
    // (KeyFrame's ctor copies the raw pointer); then re-pointed WITHOUT delete —
    // ownership transfer, not a leak. Deleted only in the init paths and
    // CreateMapInAtlas; Reset()/ResetActiveMap() leave it stale (INHERITED).
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;

    // IMU CONTRACT (docs/IMU_CONTRACT.md §1): FIFO of raw IMU samples. Sole
    // producer GrabImuData; sole consumer PreintegrateIMU, which pops samples
    // older than the current frame and leaves the first "future" sample as the
    // next interval's left endpoint. Unbounded and unsorted: monotonic push
    // timestamps are an UNCHECKED precondition. Cleared in exactly one place
    // (Track(), timestamp-regression branch).
    std::list<IMU::Point> mlQueueImuData;

    // Scratch for PreintegrateIMU: tracking-thread-private, NOT guarded by
    // mMutexImuQueue (only mlQueueImuData is). docs/IMU_CONTRACT.md §1.
    std::vector<IMU::Point> mvImuFromLastFrame;
    // Guards mlQueueImuData only. Currently uncontended — every shipped driver
    // pushes from the tracking thread; it permits an async producer in principle,
    // but PreintegrateIMU reads size() unlocked (B2). docs/IMU_CONTRACT.md §1.
    std::mutex mMutexImuQueue;

    // P11-A: the IMU rescale/rebias mailbox (see ImuUpdateMsg above).
    // Guarded by its own DEDICATED leaf mutex rather than reusing
    // mMutexImuQueue: the queue mutex implements a different protocol
    // (per-iteration acquire/release inside PreintegrateIMU's drain loop,
    // sample FIFO custody) and entangling the message slot with it would
    // add cross-thread contention to that hot drain for no protection
    // gain. mMutexImuUpdate is held only for a struct copy in/out and is
    // NEVER held while taking any other lock (a leaf by construction), so
    // it introduces no lock-order edges.
    std::optional<ImuUpdateMsg> mPendingImuUpdate;
    std::mutex mMutexImuUpdate;

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
    KeyFramePtr mpReferenceKF;
    std::vector<KeyFramePtr> mvpLocalKeyFrames;
    std::vector<MapPointPtr> mvpLocalMapPoints;
    
    // P7-1b: narrow reset-request interface, implemented by System (which
    // injects itself in System.cpp). Replaces the former `System* mpSystem`
    // back-reference — the only System method Tracking ever called was
    // ResetActiveMap() (docs/P7_RECON.md §B.4). Declared in the exact slot
    // mpSystem occupied so the ctor member-init order is unchanged.
    IResetRequester* mpResetRequester;


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
    int mnFramesToResetIMU = 0;  // upstream new-format path never set this (see P2-2
                                 // note); P11-F6: newParameterLoader assigns
                                 // Fix.LegacyImuResetWindow ? mMaxFrames : 0
                                 // (DIVERGENCES #3, legacy fps-sized window, OFF default)

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    // P11-A: atomic — pull-read lock-free from the LocalMapping thread
    // (GetMatchesInliers -> bLarge heuristic, LocalMapping.cpp:179), which
    // was a plain-int data race since upstream. Tracking is the only
    // writer; TrackLocalMap counts into a local and publishes ONCE with a
    // relaxed store (LM now sees previous-frame-or-new value instead of
    // upstream's torn mid-count garbage — both are timing lottery, ours is
    // at least a well-defined value). Zero-initialized: upstream let LM
    // read an indeterminate int before the first TrackLocalMap.
    std::atomic<int> mnMatchesInliers{0};

    //Last Frame, KeyFrame and Relocalisation Info
    // P10-2 made this atomic (ledger race R-f); R4b slice 2 upgrades it to
    // a mutex-guarded KeyFramePtr slot (SharedSlot, map/MapTypes.hpp): the
    // .load()/.store() call sites survive verbatim, every load returns a
    // strong pin, and the slot itself keeps the last KF alive for Tracking's
    // dereferences even after culling tombstones it.
    SharedSlot<KeyFrame> mpLastKeyFrame;
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

    // R4b slice 1: temporal points are refcount-owned (no manual delete).
    // They are never registered in the map; they die when the last holder
    // (this list, mLastFrame's slots, FrameDrawer's copy) lets go.
    std::list<MapPointPtr> mlpTemporalPoints;

    //int nMapChangeIndex;

    int mnNumDataset;

    std::ofstream f_track_stats;

    std::ofstream f_track_times;
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
        // P11-A: relaxed atomic accesses (storage change only — same
        // assign-then-trace shape, same values, no added ordering).
        const eTrackingState prev = mState_.load(std::memory_order_relaxed);
        mState_.store(s, std::memory_order_relaxed);
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
