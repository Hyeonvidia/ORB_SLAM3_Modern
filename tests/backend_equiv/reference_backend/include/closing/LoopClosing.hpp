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


#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "map/KeyFrame.hpp"
#include "map/MapTypes.hpp"  // R4b: MapPointPtr
#include "mapping/LocalMapping.hpp"
#include "map/Atlas.hpp"
#include "features/ORBVocabulary.hpp"
#include "tracking/Tracking.hpp"

#include "recognition/KeyFrameDatabase.hpp"

#include <boost/algorithm/string.hpp>
#include <thread>
#include <mutex>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM3
{
// R4b slice 2 build repair (2026-08-20): this frozen snapshot predates R3,
// when the vendored DBoW2 fork header still leaked a global
// `using namespace std;` into every include chain. R3 retired that header,
// so the snapshot's bare std names no longer resolve. Restore the names it
// relied on with scoped using-declarations — name-visibility only, zero
// semantic change to the frozen code.
using std::pair;
using std::set;
using std::map;
using std::vector;
using std::list;
using std::string;
using std::mutex;
using std::unique_lock;
using std::thread;


class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;
struct BAEpochs;
class ILoopOptimizer;


class LoopClosing
{
public:

    typedef pair<set<KeyFramePtr>,int> ConsistentGroup;    
    typedef map<KeyFramePtr,g2o::Sim3,std::less<KeyFramePtr>,
        Eigen::aligned_allocator<std::pair<KeyFramePtr const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Atlas* pAtlas, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale, const bool bActiveLC, BAEpochs* pBAEpochs, ILoopOptimizer* pOptimizer);

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFramePtr pKF);

    void RequestReset();
    void RequestResetActiveMap(Map* pMap);

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }   

    void RequestFinish();

    bool isFinished();

    Viewer* mpViewer;

#ifdef REGISTER_TIMES

    vector<double> vdDataQuery_ms;
    vector<double> vdEstSim3_ms;
    vector<double> vdPRTotal_ms;

    vector<double> vdMergeMaps_ms;
    vector<double> vdWeldingBA_ms;
    vector<double> vdMergeOptEss_ms;
    vector<double> vdMergeTotal_ms;
    vector<int> vnMergeKFs;
    vector<int> vnMergeMPs;
    int nMerges;

    vector<double> vdLoopFusion_ms;
    vector<double> vdLoopOptEss_ms;
    vector<double> vdLoopTotal_ms;
    vector<int> vnLoopKFs;
    int nLoop;

    vector<double> vdGBA_ms;
    vector<double> vdUpdateMap_ms;
    vector<double> vdFGBATotal_ms;
    vector<int> vnGBAKFs;
    vector<int> vnGBAMPs;
    int nFGBA_exec;
    int nFGBA_abort;

#endif

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();


    //Methods to implement the new place recognition algorithm
    bool NewDetectCommonRegions();
    bool DetectAndReffineSim3FromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                        std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs);
    bool DetectCommonRegionsFromBoW(std::vector<KeyFramePtr> &vpBowCand, KeyFramePtr &pMatchedKF, KeyFramePtr &pLastCurrentKF, g2o::Sim3 &g2oScw,
                                     int &nNumCoincidences, std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs);
    bool DetectCommonRegionsFromLastKF(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
                                            std::vector<MapPointPtr> &vpMPs, std::vector<MapPointPtr> &vpMatchedMPs);
    int FindMatchesByProjection(KeyFramePtr pCurrentKF, KeyFramePtr pMatchedKFw, g2o::Sim3 &g2oScw,
                                set<MapPointPtr> &spMatchedMPinOrigin, vector<MapPointPtr> &vpMapPoints,
                                vector<MapPointPtr> &vpMatchedMapPoints);


    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPointPtr> &vpMapPoints);
    void SearchAndFuse(const vector<KeyFramePtr> &vConectedKFs, vector<MapPointPtr> &vpMapPoints);

    void CorrectLoop();

    void MergeLocal();
    void MergeLocal2();

    void CheckObservations(set<KeyFramePtr> &spKFsMap1, set<KeyFramePtr> &spKFsMap2);

    void ResetIfRequested();
    bool mbResetRequested;
    bool mbResetActiveMapRequested;
    Map* mpMapToReset;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Atlas* mpAtlas;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    // Persistent local-BA epoch marks, owned by System (P5-C).
    BAEpochs* mpBAEpochs;

    // Narrow optimizer interface, owned by System (G2oBackend value member,
    // P6-A ISP split — see include/backend/ILoopOptimizer.hpp). Also used by
    // the detached GBA thread (stateless const contract, same thread
    // semantics as the previous static calls).
    ILoopOptimizer* mpOptimizer;

    LocalMapping *mpLocalMapper;

    std::list<KeyFramePtr> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFramePtr mpCurrentKF;
    KeyFramePtr mpLastCurrentKF;
    KeyFramePtr mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFramePtr> mvpEnoughConsistentCandidates;
    std::vector<KeyFramePtr> mvpCurrentConnectedKFs;
    std::vector<MapPointPtr> mvpCurrentMatchedPoints;
    std::vector<MapPointPtr> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    //-------
    Map* mpLastMap;

    bool mbLoopDetected;
    int mnLoopNumCoincidences;
    int mnLoopNumNotFound;
    KeyFramePtr mpLoopLastCurrentKF;
    g2o::Sim3 mg2oLoopSlw;
    g2o::Sim3 mg2oLoopScw;
    KeyFramePtr mpLoopMatchedKF;
    std::vector<MapPointPtr> mvpLoopMPs;
    std::vector<MapPointPtr> mvpLoopMatchedMPs;
    bool mbMergeDetected;
    int mnMergeNumCoincidences;
    int mnMergeNumNotFound;
    KeyFramePtr mpMergeLastCurrentKF;
    g2o::Sim3 mg2oMergeSlw;
    g2o::Sim3 mg2oMergeSmw;
    g2o::Sim3 mg2oMergeScw;
    KeyFramePtr mpMergeMatchedKF;
    std::vector<MapPointPtr> mvpMergeMPs;
    std::vector<MapPointPtr> mvpMergeMatchedMPs;
    std::vector<KeyFramePtr> mvpMergeConnectedKFs;

    g2o::Sim3 mSold_new;
    //-------

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    int mnFullBAIdx;



    vector<double> vdPR_CurrentTime;
    vector<double> vdPR_MatchedTime;
    vector<int> vnPR_TypeRecogn;

    //DEBUG
    string mstrFolderSubTraj;
    int mnNumCorrection;
    int mnCorrectionGBA;


    // To (de)activate LC
    bool mbActiveLC = true;

#ifdef REGISTER_LOOP
    string mstrFolderLoop;
#endif
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
