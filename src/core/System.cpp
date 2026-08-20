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



#include "core/System.hpp"
#include "io/Converter.hpp"
#include <chrono>
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <openssl/evp.h>  // R1: EVP digest API replaces the MD5_* API deprecated since OpenSSL 3.0
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/string.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

namespace ORB_SLAM3
{

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

System::System(const std::string &strVocFile, const std::string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, const int initFr, const std::string &strSequence):
    mSensor(sensor), mpViewer(nullptr), mbReset(false), mbResetActiveMap(false),
    mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false), mbShutDown(false),
    mbShutdownDone(false)
{
    // Output welcome message
    std::cout << std::endl <<
    "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << std::endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza." << std::endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << std::endl  <<
    "This is free software, and you are welcome to redistribute it" << std::endl <<
    "under certain conditions. See LICENSE.txt." << std::endl << std::endl;

    std::cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        std::cout << "Monocular" << std::endl;
    else if(mSensor==STEREO)
        std::cout << "Stereo" << std::endl;
    else if(mSensor==RGBD)
        std::cout << "RGB-D" << std::endl;
    else if(mSensor==IMU_MONOCULAR)
        std::cout << "Monocular-Inertial" << std::endl;
    else if(mSensor==IMU_STEREO)
        std::cout << "Stereo-Inertial" << std::endl;
    else if(mSensor==IMU_RGBD)
        std::cout << "RGB-D-Inertial" << std::endl;

    //Check settings file (P2-2: Settings is the single parsing path; the
    //legacy flat-key format — Examples_old era, no File.version — is no
    //longer accepted)
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       std::cerr << "Failed to open settings file at: " << strSettingsFile << std::endl;
       exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(node.empty() || !node.isString() || node.string() != "1.0"){
        std::cerr << "Settings file " << strSettingsFile << " lacks File.version: \"1.0\".\n"
             << "Legacy flat-key calibration files are not supported; migrate to the "
             << "V1.0 format (see Examples/*/EuRoC.yaml)." << std::endl;
        exit(-1);
    }
    settings_ = new Settings(strSettingsFile,mSensor);

    mStrLoadAtlasFromFile = settings_->atlasLoadFile();
    mStrSaveAtlasToFile = settings_->atlasSaveFile();

    std::cout << (*settings_) << std::endl;

    bool activeLC = settings_->activeLoopClosing();

    mStrVocabularyFilePath = strVocFile;


    //Load ORB Vocabulary (P11-V: one loader for both atlas branches;
    //binary-cache fast path, text canonical — see LoadVocabulary)
    std::cout << std::endl << "Loading ORB Vocabulary. This could take a while..." << std::endl;

    if(!LoadVocabulary(strVocFile))
    {
        std::cerr << "Wrong path to vocabulary. " << std::endl;
        std::cerr << "Falied to open at: " << strVocFile << std::endl;
        exit(-1);
    }
    std::cout << "Vocabulary loaded!" << std::endl << std::endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    if(mStrLoadAtlasFromFile.empty())
    {
        //Create the Atlas
        std::cout << "Initialization of Atlas from scratch " << std::endl;
        mpAtlas = new Atlas(0);
    }
    else
    {
        std::cout << "Load File" << std::endl;

        // Load the file with an earlier session
        std::cout << "Initialization of Atlas from file: " << mStrLoadAtlasFromFile << std::endl;
        bool isRead = LoadAtlas(FileType::BINARY_FILE);

        if(!isRead)
        {
            std::cout << "Error to load the file, please try with other session file or vocabulary file" << std::endl;
            exit(-1);
        }


        mpAtlas->CreateNewMap();

    }


    if (mSensor==IMU_STEREO || mSensor==IMU_MONOCULAR || mSensor==IMU_RGBD)
        mpAtlas->SetInertialSensor();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpAtlas);
    mpMapDrawer = new MapDrawer(mpAtlas, strSettingsFile, settings_);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    std::cout << "Seq. Name: " << strSequence << std::endl;
    // P7-1b: Tracking receives only the narrow IResetRequester surface of
    // System (constructor injection, same pattern as ITrackingOptimizer).
    mpTracker = new Tracking(static_cast<IResetRequester*>(this), mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpAtlas, mpKeyFrameDatabase, strSettingsFile, mSensor, settings_,
                             static_cast<ITrackingOptimizer*>(&mBackend), strSequence);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpAtlas, mSensor==MONOCULAR || mSensor==IMU_MONOCULAR,
                                     mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD, &mBAEpochs,
                                     static_cast<IMappingOptimizer*>(&mBackend));
    mptLocalMapping = std::thread(&ORB_SLAM3::LocalMapping::Run,mpLocalMapper);
    mpLocalMapper->mThFarPoints = settings_->thFarPoints();
    if(mpLocalMapper->mThFarPoints!=0)
    {
        std::cout << "Discard points further than " << mpLocalMapper->mThFarPoints << " m from current camera" << std::endl;
        mpLocalMapper->mbFarPoints = true;
    }
    else
        mpLocalMapper->mbFarPoints = false;

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpAtlas, mpKeyFrameDatabase, mSensor!=MONOCULAR, activeLC, &mBAEpochs,
                                   static_cast<ILoopOptimizer*>(&mBackend));
    mptLoopClosing = std::thread(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        // P10-6: the Viewer consumes only the narrow IViewerHost surface;
        // the sensor-enum read it used to make (mpSystem->MONOCULAR/...)
        // is a ctor-injected bool.
        mpViewer = new Viewer(static_cast<IViewerHost*>(this), mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile,settings_,
                              mSensor==MONOCULAR || mSensor==STEREO || mSensor==RGBD);
        mptViewer = std::thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
        mpViewer->both = mpFrameDrawer->both;
    }

    // Fix verbosity
    Verbose::SetTh(Verbose::VERBOSITY_QUIET);

}

Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const std::vector<IMU::Point>& vImuMeas, std::string filename)
{
    if(mSensor!=STEREO && mSensor!=IMU_STEREO)
    {
        std::cerr << "ERROR: you called TrackStereo but input sensor was not set to Stereo nor Stereo-Inertial." << std::endl;
        exit(-1);
    }

    cv::Mat imLeftToFeed, imRightToFeed;
    if(settings_ && settings_->needToRectify()){
        cv::Mat M1l = settings_->M1l();
        cv::Mat M2l = settings_->M2l();
        cv::Mat M1r = settings_->M1r();
        cv::Mat M2r = settings_->M2r();

        cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
    }
    else if(settings_ && settings_->needToResize()){
        cv::resize(imLeft,imLeftToFeed,settings_->newImSize());
        cv::resize(imRight,imRightToFeed,settings_->newImSize());
    }
    else{
        imLeftToFeed = imLeft.clone();
        imRightToFeed = imRight.clone();
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped (P10-4: CV
            // wait, was a 1ms isStopped() poll)
            mpLocalMapper->WaitUntilStopped();

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_STEREO)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed,imRightToFeed,timestamp,filename);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = static_cast<int>(mpTracker->GetState());
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp, const std::vector<IMU::Point>& vImuMeas, std::string filename)
{
    if(mSensor!=RGBD  && mSensor!=IMU_RGBD)
    {
        std::cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << std::endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    cv::Mat imDepthToFeed = depthmap.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;

        cv::resize(depthmap,imDepthToFeed,settings_->newImSize());
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped (P10-4: CV
            // wait, was a 1ms isStopped() poll)
            mpLocalMapper->WaitUntilStopped();

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_RGBD)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageRGBD(imToFeed,imDepthToFeed,timestamp,filename);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = static_cast<int>(mpTracker->GetState());
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp, const std::vector<IMU::Point>& vImuMeas, std::string filename)
{

    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if(mbShutDown)
            return Sophus::SE3f();
    }

    if(mSensor!=MONOCULAR && mSensor!=IMU_MONOCULAR)
    {
        std::cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular nor Monocular-Inertial." << std::endl;
        exit(-1);
    }

    cv::Mat imToFeed = im.clone();
    if(settings_ && settings_->needToResize()){
        cv::Mat resizedIm;
        cv::resize(im,resizedIm,settings_->newImSize());
        imToFeed = resizedIm;
    }

    // Check mode change
    {
        std::unique_lock<std::mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped (P10-4: CV
            // wait, was a 1ms isStopped() poll)
            mpLocalMapper->WaitUntilStopped();

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
            mbResetActiveMap = false;
        }
        else if(mbResetActiveMap)
        {
            std::cout << "SYSTEM-> Reseting active map in monocular case" << std::endl;
            mpTracker->ResetActiveMap();
            mbResetActiveMap = false;
        }
    }

    if (mSensor == System::IMU_MONOCULAR)
        for(size_t i_imu = 0; i_imu < vImuMeas.size(); i_imu++)
            mpTracker->GrabImuData(vImuMeas[i_imu]);

    Sophus::SE3f Tcw = mpTracker->GrabImageMonocular(imToFeed,timestamp,filename);

    std::unique_lock<std::mutex> lock2(mMutexState);
    mTrackingState = static_cast<int>(mpTracker->GetState());
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}



void System::ActivateLocalizationMode()
{
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    std::unique_lock<std::mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpAtlas->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbReset = true;
}

void System::ResetActiveMap()
{
    std::unique_lock<std::mutex> lock(mMutexReset);
    mbResetActiveMap = true;
}

// P10-6 (IViewerHost): latch ONLY -- NEVER joins. Called from the viewer
// thread (menuStop), where any join would self-deadlock -- the hazard that
// made upstream comment out Shutdown's viewer wait, and that P10-5 dodged
// with a get_id() guard, is removed structurally here: the viewer only
// requests. Sets the mbShutDown request latch (TrackMonocular gate +
// isShutDown readers) and RequestFinishes all workers so they start
// winding down; the joins and SaveAtlas remain with Shutdown(), whose OWN
// idempotence latch (mbShutdownDone) is separate, so the main thread's
// end-of-example Shutdown() still performs the teardown afterwards.
void System::RequestShutdown()
{
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbShutDown.store(true);
    }

    std::cout << "Shutdown requested" << std::endl;

    if(mpViewer)
        mpViewer->RequestFinish();
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
}

void System::Shutdown()
{
    // P10-5 (docs/P10_RECON.md 2부 item 6): the upstream wait block below
    // this point was entirely commented out -- SaveAtlas raced live LM/LC/
    // GBA threads and nothing was ever joined. Restored as a strict join
    // ordering (comments per step); upstream's commented-out remains were
    // removed with it.
    //
    // Step 1 -- teardown idempotence latch: exactly ONE caller runs the
    // joins (a second join is UB). P10-6: this is a SEPARATE latch from
    // the mbShutDown REQUEST latch -- a menuStop RequestShutdown has
    // already set mbShutDown, and the end-of-example Shutdown() must still
    // tear down; only a second Shutdown() call (e.g. ~System after an
    // explicit Shutdown) skips.
    if(mbShutdownDone.exchange(true))
        return;

    // Request latch (idempotent when RequestShutdown already set it). The
    // mMutexReset scope keeps the Track* readers' lock discipline.
    {
        std::unique_lock<std::mutex> lock(mMutexReset);
        mbShutDown.store(true);
    }

    std::cout << "Shutdown" << std::endl;

    // Step 2 -- Viewer: RequestFinish, then join ONLY from another thread.
    // P10-6: no caller runs Shutdown on the viewer thread anymore
    // (menuStop latches via RequestShutdown), so the get_id() guard is
    // belt-and-braces; if it ever fires, the thread object stays joinable
    // and ~System reaps it after Viewer::Run returns. A parked viewer is
    // woken by RequestFinish (P10-6 park predicate includes the finish
    // flag), so this join cannot hang on a stopped viewer.
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        if(mptViewer.joinable() &&
           std::this_thread::get_id() != mptViewer.get_id())
            mptViewer.join();
    }

    // Step 3 -- ask LM and LC to finish. Both notify their P10-4/P10-5 CVs,
    // so a parked LM / net-waiting LC wakes promptly.
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();

    // Step 4 -- join LM FIRST: its SetFinish() sets mbStopped and notifies
    // mCondStop, so any LC/GBA WaitUntilStopped converges on a dead LM
    // instead of hanging.
    if(mptLocalMapping.joinable())
        mptLocalMapping.join();

    // Step 5 -- join LC: an in-flight CorrectLoop/Merge completes; its
    // Release() is a no-op on the finished LM (upstream semantics).
    if(mptLoopClosing.joinable())
        mptLoopClosing.join();

    // Step 6 -- GBA custody transfer + reap: with LC dead, System has
    // exclusive access to the GBA thread object (custody chain,
    // docs/OWNERSHIP.md GBA section). Stop flag + epoch bump under
    // mMutexGBA, join outside it.
    mpLoopCloser->StopAndJoinGBA();

    // Step 7 -- ONLY NOW SaveAtlas: provably zero live LM/LC/GBA threads,
    // so boost serialization cannot race a map mutation. Achieved by join
    // ordering, not by locks.
    if(!mStrSaveAtlasToFile.empty())
    {
        Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile, Verbose::VERBOSITY_NORMAL);
        SaveAtlas(FileType::BINARY_FILE);
    }

#ifdef REGISTER_TIMES
    mpTracker->PrintTimeStats();
#endif


}

System::~System()
{
    // P10-5: backstop teardown. Shutdown() is a no-op if the teardown
    // already ran; the joinable() guards then reap anything left. P10-6:
    // with menuStop reduced to a RequestShutdown latch, no Shutdown ever
    // runs on the viewer thread anymore -- after a normal end-of-example
    // Shutdown() everything below is already joined, and this destructor
    // only matters for a System destroyed without any Shutdown() call.
    // This destructor never runs on the viewer thread (the System object
    // lives in the example main's scope), so the joins here are safe;
    // without them a joinable std::thread member would std::terminate on
    // destruction. The viewer join stays FIRST (belt-and-braces mirror of
    // Shutdown's own ordering).
    Shutdown();
    if(mptViewer.joinable())
        mptViewer.join();
    if(mptLocalMapping.joinable())
        mptLocalMapping.join();
    if(mptLoopClosing.joinable())
        mptLoopClosing.join();
}

bool System::isShutDown() {
    std::unique_lock<std::mutex> lock(mMutexReset);
    return mbShutDown;
}

void System::SaveTrajectoryTUM(const std::string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
    if(mSensor==MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << std::endl;
        return;
    }

    std::vector<KeyFramePtr> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFramePtr>::iterator lRit = mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(std::list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFramePtr pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Two;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        f << std::setprecision(6) << *lT << " " <<  std::setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryTUM(const std::string &filename)
{
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." << std::endl;

    std::vector<KeyFramePtr> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKF = vpKFs[i];

        if(pKF->isBad())
        {
            continue;
        }

        Sophus::SE3f Twc = pKF->GetPoseInverse();
        Eigen::Quaternionf q = Twc.unit_quaternion();
        Eigen::Vector3f t = Twc.translation();
        f << std::setprecision(6) << pKF->mTimeStamp << std::setprecision(7) << " " << t(0) << " " << t(1) << " " << t(2)
          << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

    }

    f.close();
}

void System::SaveTrajectoryEuRoC(const std::string &filename)
{

    std::cout << std::endl << "Saving trajectory to " << filename << " ..." << std::endl;

    int numMaxKFs = 0;
    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;  // R1: was read uninitialized when every map is empty (gcc-13 -Wmaybe-uninitialized)
    std::cout << "There are " << std::to_string(vpMaps.size()) << " maps in the atlas" << std::endl;
    for(Map* pMap :vpMaps)
    {
        std::cout << "  Map " << std::to_string(pMap->GetId()) << " has " << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs" << std::endl;
        if(pMap->GetAllKeyFrames().size() > static_cast<size_t>(numMaxKFs))
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)  // R1: guard the dereference below (mirrors SaveKeyFrameTrajectoryEuRoC)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    std::vector<KeyFramePtr> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b depending on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFramePtr>::iterator lRit = mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;


        KeyFramePtr pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            continue;
        }

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }

    }
    f.close();
    std::cout << std::endl << "End of saving trajectory to " << filename << " ..." << std::endl;
}

void System::SaveTrajectoryEuRoC(const std::string &filename, Map* pMap)
{

    std::cout << std::endl << "Saving trajectory of map " << pMap->GetId() << " to " << filename << " ..." << std::endl;


    std::vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or not.
    if (mSensor==IMU_MONOCULAR || mSensor==IMU_STEREO || mSensor==IMU_RGBD)
        Twb = vpKFs[0]->GetImuPose();
    else
        Twb = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFramePtr>::iterator lRit = mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    std::list<bool>::iterator lbL = mpTracker->mlbLost.begin();


    for(auto lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;


        KeyFramePtr pKF = *lRit;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        if (!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        if(!pKF || pKF->GetMap() != pMap)
        {
            continue;
        }

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
        else
        {
            Sophus::SE3f Twc = ((*lit)*Trw).inverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f twc = Twc.translation();
            f << std::setprecision(6) << 1e9*(*lT) << " " <<  std::setprecision(9) << twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }

    }
    f.close();
    std::cout << std::endl << "End of saving trajectory to " << filename << " ..." << std::endl;
}


void System::SaveKeyFrameTrajectoryEuRoC(const std::string &filename)
{
    std::cout << std::endl << "Saving keyframe trajectory to " << filename << " ..." << std::endl;

    std::vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap = nullptr;  // R1: the !pBiggerMap guard below read it uninitialized (gcc-13 -Wmaybe-uninitialized)
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap && pMap->GetAllKeyFrames().size() > static_cast<size_t>(numMaxKFs))
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    if(!pBiggerMap)
    {
        std::cout << "There is not a map!!" << std::endl;
        return;
    }

    std::vector<KeyFramePtr> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKF = vpKFs[i];

        if(!pKF || pKF->isBad())
        {
            continue;
        }
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  std::setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  std::setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
    }
    f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const std::string &filename, Map* pMap)
{
    std::cout << std::endl << "Saving keyframe trajectory of map " << pMap->GetId() << " to " << filename << " ..." << std::endl;

    std::vector<KeyFramePtr> vpKFs = pMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFramePtr pKF = vpKFs[i];

        if(!pKF || pKF->isBad())
        {
            continue;
        }
        if (mSensor == IMU_MONOCULAR || mSensor == IMU_STEREO || mSensor==IMU_RGBD)
        {
            Sophus::SE3f Twb = pKF->GetImuPose();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            Eigen::Vector3f twb = Twb.translation();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<  std::setprecision(9) << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;

        }
        else
        {
            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            Eigen::Vector3f t = Twc.translation();
            f << std::setprecision(6) << 1e9*pKF->mTimeStamp << " " <<  std::setprecision(9) << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
        }
    }
    f.close();
}

void System::SaveTrajectoryKITTI(const std::string &filename)
{
    std::cout << std::endl << "Saving camera trajectory to " << filename << " ..." << std::endl;
    if(mSensor==MONOCULAR)
    {
        std::cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << std::endl;
        return;
    }

    std::vector<KeyFramePtr> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

    std::ofstream f;
    f.open(filename.c_str());
    f << std::fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    std::list<ORB_SLAM3::KeyFramePtr>::iterator lRit = mpTracker->mlpReferences.begin();
    std::list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(std::list<Sophus::SE3f>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFramePtr pKF = *lRit;

        Sophus::SE3f Trw;

        if(!pKF)
            continue;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPose() * Tow;

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Matrix3f Rwc = Twc.rotationMatrix();
        Eigen::Vector3f twc = Twc.translation();

        f << std::setprecision(9) << Rwc(0,0) << " " << Rwc(0,1)  << " " << Rwc(0,2) << " "  << twc(0) << " " <<
             Rwc(1,0) << " " << Rwc(1,1)  << " " << Rwc(1,2) << " "  << twc(1) << " " <<
             Rwc(2,0) << " " << Rwc(2,1)  << " " << Rwc(2,2) << " "  << twc(2) << std::endl;
    }
    f.close();
}


int System::GetTrackingState()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackingState;
}

std::vector<MapPointPtr> System::GetTrackedMapPoints()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

std::vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    std::unique_lock<std::mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit()
{
    // P10-2 (R5): mFirstTs is atomic now (this lock-free cross-thread read
    // could tear a double before).
    double aux = mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs.load(std::memory_order_relaxed);
    if ((aux>0.) && mpAtlas->isImuInitialized())
        return mpLocalMapper->GetCurrKFTime()-mpLocalMapper->mFirstTs.load(std::memory_order_relaxed);
    else
        return 0.f;
}

bool System::isLost()
{
    if (!mpAtlas->isImuInitialized())
        return false;
    else
    {
        if (mpTracker->GetState()==Tracking::LOST)
            return true;
        else
            return false;
    }
}


bool System::isFinished()
{
    return (GetTimeFromIMUInit()>0.1);
}

void System::ChangeDataset()
{
    if(mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12)
    {
        mpTracker->ResetActiveMap();
    }
    else
    {
        mpTracker->CreateMapInAtlas();
    }

    mpTracker->NewDataset();
}

float System::GetImageScale()
{
    return mpTracker->GetImageScale();
}

#ifdef REGISTER_TIMES
void System::InsertRectTime(double& time)
{
    mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double& time)
{
    mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double& time)
{
    mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

bool System::LoadVocabulary(const std::string &strVocFile)
{
    // P11-V (DIVERGENCES #27): the .bin beside the .txt is a derived,
    // host-local cache (native endianness, no portability contract). The
    // text file stays canonical: mStrVocabularyFilePath is never pointed
    // at the .bin, so the SaveAtlas/LoadAtlas MD5 checksum of the TEXT
    // file (.osa compatibility) is unaffected.
    mpVocabulary = new ORBVocabulary();

    const std::string strBinFile = strVocFile + ".bin";

    char szTime[32];
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    if(mpVocabulary->loadFromBinaryFile(strBinFile))
    {
        snprintf(szTime, sizeof(szTime), "%.3f",
            std::chrono::duration_cast<std::chrono::duration<double>>(
                std::chrono::steady_clock::now() - t0).count());
        std::cout << "Vocabulary loaded from binary cache " << strBinFile
             << " in " << szTime << "s" << std::endl;
        return true;
    }

    t0 = std::chrono::steady_clock::now();
    if(!mpVocabulary->loadFromTextFile(strVocFile))
        return false;
    snprintf(szTime, sizeof(szTime), "%.3f",
        std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - t0).count());
    std::cout << "Vocabulary loaded from text file " << strVocFile
         << " in " << szTime << "s" << std::endl;

    // best-effort cache write: failures (e.g. read-only mounts) are benign
    if(mpVocabulary->saveToBinaryFile(strBinFile))
        std::cout << "Saved binary vocabulary cache to " << strBinFile << std::endl;

    return true;
}

// R4b slice 1 (2026-08-20) .osa disposition: the archive layout is
// deliberately UNCHANGED by the MapPoint shared_ptr migration. Map still
// serializes a raw-pointer backup vector (filled with .get() in PreSave;
// wrapped exactly once into the owning MapPointPtr in Map::PostLoad), and
// MapPoint's field list is untouched, so pre-R4b session files remain
// format-compatible. If serialization ever moves to
// boost/serialization/shared_ptr.hpp, that IS an archive break — bump then.
void System::SaveAtlas(FileType type){
    if(!mStrSaveAtlasToFile.empty())
    {

        // Save the current session
        mpAtlas->PreSave();

        std::string pathSaveFileName = "./";
        pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
        pathSaveFileName = pathSaveFileName.append(".osa");

        std::string strVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);
        std::size_t found = mStrVocabularyFilePath.find_last_of("/\\");
        std::string strVocabularyName = mStrVocabularyFilePath.substr(found+1);

        if(type == TEXT_FILE) // File text
        {
            std::cout << "Starting to write the save text file " << std::endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::text_oarchive oa(ofs);

            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            std::cout << "End to write the save text file" << std::endl;
        }
        else if(type == BINARY_FILE) // File binary
        {
            std::cout << "Starting to write the save binary file" << std::endl;
            std::remove(pathSaveFileName.c_str());
            std::ofstream ofs(pathSaveFileName, std::ios::binary);
            boost::archive::binary_oarchive oa(ofs);
            oa << strVocabularyName;
            oa << strVocabularyChecksum;
            oa << mpAtlas;
            std::cout << "End to write save binary file" << std::endl;
        }
    }
}

bool System::LoadAtlas(FileType type)
{
    std::string strFileVoc, strVocChecksum;
    bool isRead = false;

    std::string pathLoadFileName = "./";
    pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
    pathLoadFileName = pathLoadFileName.append(".osa");

    if(type == TEXT_FILE) // File text
    {
        std::cout << "Starting to read the save text file " << std::endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            std::cout << "Load file not found" << std::endl;
            return false;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        std::cout << "End to load the save text file " << std::endl;
        isRead = true;
    }
    else if(type == BINARY_FILE) // File binary
    {
        std::cout << "Starting to read the save binary file"  << std::endl;
        std::ifstream ifs(pathLoadFileName, std::ios::binary);
        if(!ifs.good())
        {
            std::cout << "Load file not found" << std::endl;
            return false;
        }
        boost::archive::binary_iarchive ia(ifs);
        ia >> strFileVoc;
        ia >> strVocChecksum;
        ia >> mpAtlas;
        std::cout << "End to load the save binary file" << std::endl;
        isRead = true;
    }

    if(isRead)
    {
        //Check if the vocabulary is the same
        std::string strInputVocabularyChecksum = CalculateCheckSum(mStrVocabularyFilePath,TEXT_FILE);

        if(strInputVocabularyChecksum.compare(strVocChecksum) != 0)
        {
            std::cout << "The vocabulary load isn't the same which the load session was created " << std::endl;
            std::cout << "-Vocabulary name: " << strFileVoc << std::endl;
            return false; // Both are differents
        }

        mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
        mpAtlas->SetORBVocabulary(mpVocabulary);
        mpAtlas->PostLoad();

        return true;
    }
    return false;
}

std::string System::CalculateCheckSum(std::string filename, FileType type)
{
    std::string checksum = "";

    // R1: same MD5 digest through the EVP interface — the low-level MD5_*
    // functions are deprecated since OpenSSL 3.0. Checksum output (and the
    // .osa session-file compatibility that depends on it) is unchanged.
    unsigned char c[EVP_MAX_MD_SIZE];
    unsigned int digestLen = 0;

    std::ios_base::openmode flags = std::ios::in;
    if(type == BINARY_FILE) // Binary file
        flags = std::ios::in | std::ios::binary;

    std::ifstream f(filename.c_str(), flags);
    if ( !f.is_open() )
    {
        std::cout << "[E] Unable to open the in file " << filename << " for Md5 hash." << std::endl;
        return checksum;
    }

    EVP_MD_CTX* md5Context = EVP_MD_CTX_new();
    char buffer[1024];

    EVP_DigestInit_ex(md5Context, EVP_md5(), nullptr);
    while ( int count = f.readsome(buffer, sizeof(buffer)))
    {
        EVP_DigestUpdate(md5Context, buffer, count);
    }

    f.close();

    EVP_DigestFinal_ex(md5Context, c, &digestLen);
    EVP_MD_CTX_free(md5Context);

    for(unsigned int i = 0; i < digestLen; i++)
    {
        char aux[10];
        sprintf(aux,"%02x", c[i]);
        checksum = checksum + aux;
    }

    return checksum;
}

} //namespace ORB_SLAM

