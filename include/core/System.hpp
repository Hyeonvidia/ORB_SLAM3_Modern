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


#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include "core/SensorType.hpp"
#include "map/MapTypes.hpp"  // R4b: MapPointPtr
#include<stdio.h>
#include<stdlib.h>
#include<atomic>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "tracking/Tracking.hpp"
#include "viz/FrameDrawer.hpp"
#include "viz/MapDrawer.hpp"
#include "map/Atlas.hpp"
#include "mapping/LocalMapping.hpp"
#include "closing/LoopClosing.hpp"
#include "recognition/KeyFrameDatabase.hpp"
#include "recognition/OrbVocabulary.hpp"
#include "viz/Viewer.hpp"
#include "backend/BAEpochs.hpp"
#include "backend/G2oBackend.hpp"
#include "backend/ImuTypes.hpp"
#include "core/IResetRequester.hpp"
#include "core/IViewerHost.hpp"
#include "core/Settings.hpp"
#include "core/Verbose.hpp"


namespace ORB_SLAM3
{


class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

// P7-1b: System implements IResetRequester so Tracking can request a deferred
// active-map reset without holding a back-pointer to the full System facade
// (26 reachable public methods, of which Tracking used exactly this one —
// docs/P7_RECON.md §B.4). The flag/mutex machinery (mMutexReset,
// mbResetActiveMap, consumed at the top of the next TrackStereo/TrackRGBD/
// TrackMonocular) is unchanged; the interface is a pure narrowing.
//
// P10-6: System also implements IViewerHost — the 6-method surface the
// Viewer consumes (docs/P10_RECON.md 2부 item 7). This cuts the
// System.hpp <-> Viewer.hpp include cycle. The single
// RequestResetActiveMap() override below is the final overrider for BOTH
// interfaces' identically-named pure virtual.
class System : public IResetRequester, public IViewerHost
{
public:
    // Input sensor (R4c: scoped enum at namespace scope — core/SensorType.hpp;
    // the re-exports below keep System::eSensor / System::MONOCULAR working).
    using eSensor = Sensor;
    using enum Sensor;

    // File type
    enum class FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };
    using enum FileType;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const std::string &strSequence = std::string());

    // P10-5: joins all owned threads. Calls Shutdown() (a no-op if the
    // teardown already ran) and then reaps any thread object still
    // joinable. P10-6: menuStop no longer runs Shutdown on the viewer
    // thread (it latches via RequestShutdown), so in practice Shutdown()
    // joins everything; the joinable() reaping here is the backstop for
    // a System destroyed without any Shutdown() call. A joinable
    // std::thread value member would otherwise std::terminate here.
    ~System();

    // R4c: noncopyable by design (owns threads, mutexes, module graph).
    System(const System&) = delete;
    System& operator=(const System&) = delete;

    // Proccess the given stereo frame. Images must be synchronized and rectified.
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(), std::string filename="");

    // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
    // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Input depthmap: Float (CV_32F).
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(), std::string filename="");

    // Proccess the given monocular frame and optionally imu data
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    Sophus::SE3f TrackMonocular(const cv::Mat &im, const double &timestamp, const std::vector<IMU::Point>& vImuMeas = std::vector<IMU::Point>(), std::string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode() override;
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode() override;

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // IResetRequester (P7-1b) AND IViewerHost (P10-6): the narrow reset
    // entry point for Tracking and the Viewer alike. Forwards to
    // ResetActiveMap() — same lock (mMutexReset), same flag (mbResetActiveMap),
    // same one-frame-deferred consumption; the indirection exists only so
    // the callers do not need the System type.
    void RequestResetActiveMap() override { ResetActiveMap(); }

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

    // IViewerHost (P10-6): latch ONLY — NEVER joins. Sets the mbShutDown
    // request latch and RequestFinishes the Viewer/LM/LC threads; the joins
    // and SaveAtlas stay with Shutdown(), whose own idempotence latch
    // (mbShutdownDone) is deliberately separate so the end-of-example
    // Shutdown() still tears down after a menuStop RequestShutdown.
    void RequestShutdown() override;

    bool isShutDown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const std::string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const std::string &filename);

    // P10-6: the single-argument forms are the IViewerHost overrides (the
    // menuStop path saves synchronously on the viewer thread, upstream
    // behavior); the Map* overloads below are plain System API.
    void SaveTrajectoryEuRoC(const std::string &filename) override;
    void SaveKeyFrameTrajectoryEuRoC(const std::string &filename) override;

    void SaveTrajectoryEuRoC(const std::string &filename, Map* pMap);
    void SaveKeyFrameTrajectoryEuRoC(const std::string &filename, Map* pMap);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const std::string &filename);


    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPointPtr> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:

    // P11-V (DIVERGENCES #27): loads the vocabulary into mpVocabulary,
    // preferring the derived binary cache <strVocFile>.bin; on a cache
    // miss/parse failure it loads the canonical text file and best-effort
    // writes the cache beside it (write failures ignored — read-only
    // mounts). mStrVocabularyFilePath must keep pointing at the TEXT file:
    // SaveAtlas/LoadAtlas MD5-checksum the text file for .osa session
    // compatibility.
    bool LoadVocabulary(const std::string &strVocFile);

    void SaveAtlas(FileType type);
    bool LoadAtlas(FileType type);

    std::string CalculateCheckSum(std::string filename, FileType type);

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    // Persistent local-BA epoch marks shared by LocalMapping and LoopClosing
    // (P5-C, was KeyFrame::mnBALocalForKF/mnBAFixedForKF and
    // MapPoint::mnBALocalForKF). See include/backend/BAEpochs.hpp.
    BAEpochs mBAEpochs;

    // Concrete optimizer backend (P6-A ISP split): implements
    // ITrackingOptimizer + IMappingOptimizer + ILoopOptimizer; each module
    // receives only its narrow interface pointer (constructor injection,
    // same ownership pattern as mBAEpochs above).
    G2oBackend mBackend;

    // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
    // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
    LoopClosing* mpLoopCloser;

    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    // P10-5: owned VALUE members (were raw `std::thread*`, never joined --
    // docs/P10_RECON.md 2부 item 6). Joined by Shutdown() in a fixed order
    // (Viewer -> LM -> LC -> GBA custody via LoopClosing::StopAndJoinGBA),
    // with ~System as the backstop reaper. mptViewer is default-constructed
    // (not joinable) when the viewer is disabled.
    std::thread mptLocalMapping;
    std::thread mptLoopClosing;
    std::thread mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Shutdown flags
    // P10-5/P10-6: mbShutDown is the REQUEST latch — set by RequestShutdown
    // (viewer menuStop) and by Shutdown(); read by the TrackMonocular gate
    // and isShutDown(). Writes happen inside the existing mMutexReset scope
    // so those readers keep their lock discipline unchanged.
    std::atomic<bool> mbShutDown;
    // P10-6: mbShutdownDone is the TEARDOWN latch — exactly one Shutdown()
    // caller runs the joins + SaveAtlas. Separate from mbShutDown by
    // design: a menuStop RequestShutdown latches the request WITHOUT
    // performing teardown (never joins on the viewer thread), and the main
    // thread's end-of-example Shutdown() must still tear down afterwards.
    std::atomic<bool> mbShutdownDone;

    // Tracking state
    int mTrackingState;
    std::vector<MapPointPtr> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    //
    std::string mStrLoadAtlasFromFile;
    std::string mStrSaveAtlasToFile;

    std::string mStrVocabularyFilePath;

    Settings* settings_;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
