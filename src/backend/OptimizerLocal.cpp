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


#include "backend/Optimizer.hpp"

#include "OptimizerCommon.hpp"

namespace ORB_SLAM3
{

void Optimizer::LocalBundleAdjustment(KeyFramePtr pKF, const std::atomic<bool>* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, BAEpochs& epochs)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    std::list<KeyFramePtr> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    epochs.localForKF[pKF] = pKF->mnId;
    Map* pCurrentMap = pKF->GetMap();

    const std::vector<KeyFramePtr> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFramePtr pKFi = vNeighKFs[i];
        epochs.localForKF[pKFi] = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    num_fixedKF = 0;
    std::list<MapPointPtr> lLocalMapPoints;
    std::set<MapPointPtr> sNumObsMP;
    for(KeyFramePtr pKFi : lLocalKeyFrames)
    {
        if(pKFi->mnId==pMap->GetInitKFid())
        {
            num_fixedKF = 1;
        }
        std::vector<MapPointPtr> vpMPs = pKFi->GetMapPointMatches();
        for(std::vector<MapPointPtr>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPointPtr pMP = *vit;
            if(pMP)
                if(!pMP->isBad() && pMP->GetMap() == pCurrentMap)
                {

                    if(BAEpochs::get(epochs.mpLocalForKF,pMP)!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        epochs.mpLocalForKF[pMP]=pKF->mnId;
                    }
                }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    std::list<KeyFramePtr> lFixedCameras;
    for(std::list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::map<KeyFramePtr,std::tuple<int,int>> observations = (*lit)->GetObservations();
        for(std::map<KeyFramePtr,std::tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(BAEpochs::get(epochs.localForKF,pKFi)!=pKF->mnId && BAEpochs::get(epochs.fixedForKF,pKFi)!=pKF->mnId )
            {
                epochs.fixedForKF[pKFi]=pKF->mnId;
                if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
                    lFixedCameras.push_back(pKFi);
            }
        }
    }
    num_fixedKF = lFixedCameras.size() + num_fixedKF;


    if(num_fixedKF == 0)
    {
        Verbose::PrintMess("LM-LBA: There are 0 fixed KF in the optimizations, LBA aborted", Verbose::VERBOSITY_NORMAL);
        return;
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    OrbLevenberg* solver = MakeLmOptimizerEigen<g2o::BlockSolver_6_3>(optimizer);
    if (pMap->IsInertial())
        solver->setUserLambdaInit(100.0);
    optimizer.setVerbose(false);
    BindAbortFlag(optimizer, solver, pbStopFlag);

    unsigned long maxKFid = 0;

    // DEBUG LBA
    pCurrentMap->msOptKFs.clear();
    pCurrentMap->msFixedKFs.clear();

    // ---- vertices ----
    // Set Local KeyFrame vertices
    for(KeyFramePtr pKFi : lLocalKeyFrames)
    {
        AddSE3Vertex(optimizer, pKFi, pKFi->mnId==pMap->GetInitKFid());
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msOptKFs.insert(pKFi->mnId);
    }
    num_OptKF = lLocalKeyFrames.size();

    // Set Fixed KeyFrame vertices
    for(KeyFramePtr pKFi : lFixedCameras)
    {
        AddSE3Vertex(optimizer, pKFi, true);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
        // DEBUG LBA
        pCurrentMap->msFixedKFs.insert(pKFi->mnId);
    }

    // ---- edges ---- (landmark vertices interleaved)
    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    std::vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    std::vector<ORB_SLAM3::EdgeSE3ProjectXYZToBody*> vpEdgesBody;
    vpEdgesBody.reserve(nExpectedSize);

    std::vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    std::vector<KeyFramePtr> vpEdgeKFBody;
    vpEdgeKFBody.reserve(nExpectedSize);

    std::vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    std::vector<MapPointPtr> vpMapPointEdgeBody;
    vpMapPointEdgeBody.reserve(nExpectedSize);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    std::vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    std::vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    int nPoints = 0;

    int nEdges = 0;

    for(MapPointPtr pMP : lLocalMapPoints)
    {
        int id = pMP->mnId+maxKFid+1;
        AddLandmarkVertex(optimizer, pMP, id);
        nPoints++;

        const std::map<KeyFramePtr,std::tuple<int,int>> observations = pMP->GetObservations();

        //Set edges
        for(std::map<KeyFramePtr,std::tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = std::get<0>(mit->second);

                // Monocular observation
                if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]<0)
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    AddHuberKernel(e, thHuberMono);

                    e->pCamera = pKFi->mpCamera;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    nEdges++;
                }
                else if(leftIndex != -1 && pKFi->mvuRight[get<0>(mit->second)]>=0)// Stereo observation
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    AddHuberKernel(e, thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    nEdges++;
                }

                if(pKFi->mpCamera2){
                    int rightIndex = get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave];
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        AddHuberKernel(e, thHuberMono);

                        Sophus::SE3f Trl = pKFi-> GetRelativePoseTrl();
                        e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        e->pCamera = pKFi->mpCamera2;

                        optimizer.addEdge(e);
                        vpEdgesBody.push_back(e);
                        vpEdgeKFBody.push_back(pKFi);
                        vpMapPointEdgeBody.push_back(pMP);

                        nEdges++;
                    }
                }
            }
        }
    }
    num_edges = nEdges;

    // ---- solve ----
    if(pbStopFlag)
        if(pbStopFlag->load(std::memory_order_relaxed))
            return;

    optimizer.initializeOptimization();
    RunOptimization(optimizer, 10, "LocalBundleAdjustment");

    // ---- recover ---- (outlier cull + write-back)
    std::vector<std::pair<KeyFramePtr,MapPointPtr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesBody.size()+vpEdgesStereo.size());

    // Check inlier observations       
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
        {
            continue;
        }

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMono[i];
            vToErase.emplace_back(pKFi,pMP);
        }
    }

    for(size_t i=0, iend=vpEdgesBody.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
        MapPointPtr pMP = vpMapPointEdgeBody[i];

        if(pMP->isBad())
        {
            continue;
        }

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFBody[i];
            vToErase.emplace_back(pKFi,pMP);
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPointPtr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
        {
            continue;
        }

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFStereo[i];
            vToErase.emplace_back(pKFi,pMP);
        }
    }


    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFramePtr pKFi = vToErase[i].first;
            MapPointPtr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data
    //Keyframes
    for(KeyFramePtr pKFi : lLocalKeyFrames)
    {
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());
        pKFi->SetPose(Tiw);
    }

    //Points
    for(MapPointPtr pMP : lLocalMapPoints)
    {
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}

void Optimizer::LocalInertialBA(KeyFramePtr pKF, const std::atomic<bool> *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit, BAEpochs& epochs)
{
    Map* pCurrentMap = pKF->GetMap();

    int maxOpt=10;
    int opt_it=10;
    if(bLarge)
    {
        maxOpt=25;
        opt_it=4;
    }
    const int Nd = std::min(static_cast<int>(pCurrentMap->KeyFramesInMap())-2,maxOpt);
    const unsigned long maxKFid = pKF->mnId;

    std::vector<KeyFramePtr> vpOptimizableKFs;
    const std::vector<KeyFramePtr> vpNeighsKFs = pKF->GetVectorCovisibleKeyFrames();
    std::list<KeyFramePtr> lpOptVisKFs;

    vpOptimizableKFs.reserve(Nd);
    vpOptimizableKFs.push_back(pKF);
    epochs.localForKF[pKF] = pKF->mnId;
    for(int i=1; i<Nd; i++)
    {
        if(vpOptimizableKFs.back()->mPrevKF)
        {
            vpOptimizableKFs.push_back(vpOptimizableKFs.back()->mPrevKF);
            epochs.localForKF[vpOptimizableKFs.back()] = pKF->mnId;
        }
        else
            break;
    }

    int N = vpOptimizableKFs.size();

    // Optimizable points seen by temporal optimizable keyframes
    std::list<MapPointPtr> lLocalMapPoints;
    for(int i=0; i<N; i++)
    {
        std::vector<MapPointPtr> vpMPs = vpOptimizableKFs[i]->GetMapPointMatches();
        for(MapPointPtr pMP : vpMPs)
        {
            if(pMP)
                if(!pMP->isBad())
                    if(BAEpochs::get(epochs.mpLocalForKF,pMP)!=pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        epochs.mpLocalForKF[pMP]=pKF->mnId;
                    }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    std::list<KeyFramePtr> lFixedKeyFrames;
    if(vpOptimizableKFs.back()->mPrevKF)
    {
        lFixedKeyFrames.push_back(vpOptimizableKFs.back()->mPrevKF);
        epochs.fixedForKF[vpOptimizableKFs.back()->mPrevKF]=pKF->mnId;
    }
    else
    {
        epochs.localForKF[vpOptimizableKFs.back()]=0;
        epochs.fixedForKF[vpOptimizableKFs.back()]=pKF->mnId;
        lFixedKeyFrames.push_back(vpOptimizableKFs.back());
        vpOptimizableKFs.pop_back();
    }

    // Optimizable visual KFs
    const int maxCovKF = 0;
    for(int i=0, iend=vpNeighsKFs.size(); i<iend; i++)
    {
        if(lpOptVisKFs.size() >= maxCovKF)
            break;

        KeyFramePtr pKFi = vpNeighsKFs[i];
        if(BAEpochs::get(epochs.localForKF,pKFi) == pKF->mnId || BAEpochs::get(epochs.fixedForKF,pKFi) == pKF->mnId)
            continue;
        epochs.localForKF[pKFi] = pKF->mnId;
        if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        {
            lpOptVisKFs.push_back(pKFi);

            std::vector<MapPointPtr> vpMPs = pKFi->GetMapPointMatches();
            for(MapPointPtr pMP : vpMPs)
            {
                if(pMP)
                    if(!pMP->isBad())
                        if(BAEpochs::get(epochs.mpLocalForKF,pMP)!=pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            epochs.mpLocalForKF[pMP]=pKF->mnId;
                        }
            }
        }
    }

    // Fixed KFs which are not covisible optimizable
    const int maxFixKF = 200;

    for(std::list<MapPointPtr>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        std::map<KeyFramePtr,std::tuple<int,int>> observations = (*lit)->GetObservations();
        for(std::map<KeyFramePtr,std::tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(BAEpochs::get(epochs.localForKF,pKFi)!=pKF->mnId && BAEpochs::get(epochs.fixedForKF,pKFi)!=pKF->mnId)
            {
                epochs.fixedForKF[pKFi]=pKF->mnId;
                if(!pKFi->isBad())
                {
                    lFixedKeyFrames.push_back(pKFi);
                    break;
                }
            }
        }
        if(lFixedKeyFrames.size()>=maxFixKF)
            break;
    }


    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    // P10-1: solver hoisted out of the branches (shadow bridge below needs it)
    OrbLevenberg* solver = MakeLmOptimizerEigen<g2o::BlockSolverX>(optimizer);
    if(bLarge)
        solver->setUserLambdaInit(1e-2); // to avoid iterating for finding optimal lambda
    else
        solver->setUserLambdaInit(1e0);


    // ---- vertices ----
    // Set Local temporal KeyFrame vertices
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        AddInertialKFVertices(optimizer, pKFi, maxKFid, false);
    }

    // Set Local visual KeyFrame vertices
    for(KeyFramePtr pKFi : lpOptVisKFs)
    {
        VertexPose * VP = new VertexPose(pKFi);
        VP->setId(pKFi->mnId);
        VP->setFixed(false);
        optimizer.addVertex(VP);
    }

    // Set Fixed KeyFrame vertices
    for(KeyFramePtr pKFi : lFixedKeyFrames)
    {
        // IMU states "should be done only for keyframe just before temporal window" (upstream note)
        AddInertialKFVertices(optimizer, pKFi, maxKFid, true);
    }

    // ---- edges ----
    // Create intertial constraints
    std::vector<EdgeInertial*> vei(N,nullptr);
    std::vector<EdgeGyroRW*> vegr(N,nullptr);
    std::vector<EdgeAccRW*> vear(N,nullptr);

    for(int i=0;i<N;i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        if(!pKFi->mPrevKF)
        {
            std::cout << "NOT INERTIAL LINK TO PREVIOUS FRAME!!!!" << std::endl;
            continue;
        }
        if(pKFi->bImu && pKFi->mPrevKF->bImu && pKFi->mpImuPreintegrated)
        {
            pKFi->mpImuPreintegrated->SetNewBias(pKFi->mPrevKF->GetImuBias());
            g2o::HyperGraph::Vertex* VP1 = optimizer.vertex(pKFi->mPrevKF->mnId);
            g2o::HyperGraph::Vertex* VV1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+1);
            g2o::HyperGraph::Vertex* VG1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+2);
            g2o::HyperGraph::Vertex* VA1 = optimizer.vertex(maxKFid+3*(pKFi->mPrevKF->mnId)+3);
            g2o::HyperGraph::Vertex* VP2 =  optimizer.vertex(pKFi->mnId);
            g2o::HyperGraph::Vertex* VV2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+1);
            g2o::HyperGraph::Vertex* VG2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+2);
            g2o::HyperGraph::Vertex* VA2 = optimizer.vertex(maxKFid+3*(pKFi->mnId)+3);

            if(!VP1 || !VV1 || !VG1 || !VA1 || !VP2 || !VV2 || !VG2 || !VA2)
            {
                std::cerr << "Error " << VP1 << ", "<< VV1 << ", "<< VG1 << ", "<< VA1 << ", " << VP2 << ", " << VV2 <<  ", "<< VG2 << ", "<< VA2 <<std::endl;
                continue;
            }

            vei[i] = new EdgeInertial(pKFi->mpImuPreintegrated);

            vei[i]->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP1));
            vei[i]->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV1));
            vei[i]->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VG1));
            vei[i]->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VA1));
            vei[i]->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VP2));
            vei[i]->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(VV2));

            if(i==N-1 || bRecInit)
            {
                // All inertial residuals are included without robust cost function, but not that one linking the
                // last optimizable keyframe inside of the local window and the first fixed keyframe out. The
                // information matrix for this measurement is also downweighted. This is done to avoid accumulating
                // error due to fixing variables.
                g2o::RobustKernelHuber* rki = new g2o::RobustKernelHuber;
                vei[i]->setRobustKernel(rki);
                if(i==N-1)
                    vei[i]->setInformation(vei[i]->information()*1e-2);
                rki->setDelta(sqrt(16.92));
            }
            optimizer.addEdge(vei[i]);

            vegr[i] = new EdgeGyroRW();
            vegr[i]->setVertex(0,VG1);
            vegr[i]->setVertex(1,VG2);
            Eigen::Matrix3d InfoG = pKFi->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
            vegr[i]->setInformation(InfoG);
            optimizer.addEdge(vegr[i]);

            vear[i] = new EdgeAccRW();
            vear[i]->setVertex(0,VA1);
            vear[i]->setVertex(1,VA2);
            Eigen::Matrix3d InfoA = pKFi->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
            vear[i]->setInformation(InfoA);           

            optimizer.addEdge(vear[i]);
        }
        else
            std::cout << "ERROR building inertial edge" << std::endl;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (N+lFixedKeyFrames.size())*lLocalMapPoints.size();

    // Mono
    std::vector<EdgeMono*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    std::vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    std::vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    // Stereo
    std::vector<EdgeStereo*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    std::vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    std::vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);



    const float thHuberMono = sqrt(5.991);
    const float chi2Mono2 = 5.991;
    const float thHuberStereo = sqrt(7.815);
    const float chi2Stereo2 = 7.815;

    const unsigned long iniMPid = maxKFid*5;

    std::map<int,int> mVisEdges;
    for(int i=0;i<N;i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];
        mVisEdges[pKFi->mnId] = 0;
    }
    for(std::list<KeyFramePtr>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    {
        mVisEdges[(*lit)->mnId] = 0;
    }

    for(MapPointPtr pMP : lLocalMapPoints)
    {
        unsigned long id = pMP->mnId+iniMPid+1;
        AddLandmarkVertex(optimizer, pMP, id);
        const std::map<KeyFramePtr,std::tuple<int,int>> observations = pMP->GetObservations();

        // Create visual constraints
        for(std::map<KeyFramePtr,std::tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFramePtr pKFi = mit->first;

            if(BAEpochs::get(epochs.localForKF,pKFi)!=pKF->mnId && BAEpochs::get(epochs.fixedForKF,pKFi)!=pKF->mnId)
                continue;

            if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
            {
                const int leftIndex = std::get<0>(mit->second);

                cv::KeyPoint kpUn;

                // Monocular left observation
                if(leftIndex != -1 && pKFi->mvuRight[leftIndex]<0)
                {
                    mVisEdges[pKFi->mnId]++;

                    kpUn = pKFi->mvKeysUn[leftIndex];
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    EdgeMono* e = new EdgeMono(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    AddHuberKernel(e, thHuberMono);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                // Stereo-observation
                else if(leftIndex != -1)// Stereo observation
                {
                    kpUn = pKFi->mvKeysUn[leftIndex];
                    mVisEdges[pKFi->mnId]++;

                    const float kp_ur = pKFi->mvuRight[leftIndex];
                    Eigen::Matrix<double,3,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    EdgeStereo* e = new EdgeStereo(0);

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);

                    // Add here uncerteinty
                    const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                    e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    AddHuberKernel(e, thHuberStereo);

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }

                // Monocular right observation
                if(pKFi->mpCamera2){
                    int rightIndex = std::get<1>(mit->second);

                    if(rightIndex != -1 ){
                        rightIndex -= pKFi->NLeft;
                        mVisEdges[pKFi->mnId]++;

                        Eigen::Matrix<double,2,1> obs;
                        cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        obs << kp.pt.x, kp.pt.y;

                        EdgeMono* e = new EdgeMono(1);

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);

                        // Add here uncerteinty
                        const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        AddHuberKernel(e, thHuberMono);

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                }
            }
        }
    }

    for(std::map<int,int>::iterator mit=mVisEdges.begin(), mend=mVisEdges.end(); mit!=mend; mit++)
    {
        assert(mit->second>=3);
    }

    // ---- solve ----
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    float err = optimizer.activeRobustChi2();
    RunOptimization(optimizer, opt_it, "LocalInertialBA"); // Originally to 2
    float err_end = optimizer.activeRobustChi2();
    // Upstream ordering quirk kept verbatim: the abort bridge is attached only
    // AFTER the solve above, so it cannot abort this call's optimization.
    BindAbortFlag(optimizer, solver, pbStopFlag);

    // ---- recover ---- (outlier cull + write-back)
    std::vector<std::pair<KeyFramePtr,MapPointPtr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());

    // Check inlier observations
    // Mono
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        EdgeMono* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];
        bool bClose = pMP->mTrackDepth<10.f;

        if(pMP->isBad())
        {
            continue;
        }

        if((e->chi2()>chi2Mono2 && !bClose) || (e->chi2()>1.5f*chi2Mono2 && bClose) || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMono[i];
            vToErase.emplace_back(pKFi,pMP);
        }
    }


    // Stereo
    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        EdgeStereo* e = vpEdgesStereo[i];
        MapPointPtr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
        {
            continue;
        }

        if(e->chi2()>chi2Stereo2)
        {
            KeyFramePtr pKFi = vpEdgeKFStereo[i];
            vToErase.emplace_back(pKFi,pMP);
        }
    }

    // Get Map Mutex and erase outliers
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);


    // TODO: Some convergence problems have been detected here
    if((2*err < err_end || isnan(err) || isnan(err_end)) && !bLarge) //bGN)
    {
        std::cout << "FAIL LOCAL-INERTIAL BA!!!!" << std::endl;
        return;
    }



    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFramePtr pKFi = vToErase[i].first;
            MapPointPtr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    for(std::list<KeyFramePtr>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
        epochs.fixedForKF[*lit] = 0;

    // Recover optimized data
    // Local temporal Keyframes
    N=vpOptimizableKFs.size();
    for(int i=0; i<N; i++)
    {
        KeyFramePtr pKFi = vpOptimizableKFs[i];

        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        epochs.localForKF[pKFi]=0;

        if(pKFi->bImu)
        {
            VertexVelocity* VV = static_cast<VertexVelocity*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+1));
            pKFi->SetVelocity(VV->estimate().cast<float>());
            VertexGyroBias* VG = static_cast<VertexGyroBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+2));
            VertexAccBias* VA = static_cast<VertexAccBias*>(optimizer.vertex(maxKFid+3*(pKFi->mnId)+3));
            Vector6d b;
            b << VG->estimate(), VA->estimate();
            pKFi->SetNewBias(IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]));

        }
    }

    // Local visual KeyFrame
    for(KeyFramePtr pKFi : lpOptVisKFs)
    {
        VertexPose* VP = static_cast<VertexPose*>(optimizer.vertex(pKFi->mnId));
        Sophus::SE3f Tcw(VP->estimate().Rcw[0].cast<float>(), VP->estimate().tcw[0].cast<float>());
        pKFi->SetPose(Tcw);
        epochs.localForKF[pKFi]=0;
    }

    //Points
    for(MapPointPtr pMP : lLocalMapPoints)
    {
        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(pMP->mnId+iniMPid+1));
        pMP->SetWorldPos(vPoint->estimate().cast<float>());
        pMP->UpdateNormalAndDepth();
    }

    pMap->IncreaseChangeIndex();
}

void Optimizer::LocalBundleAdjustment(KeyFramePtr pMainKF,std::vector<KeyFramePtr> vpAdjustKF, std::vector<KeyFramePtr> vpFixedKF, const std::atomic<bool> *pbStopFlag, const BAEpochs& epochs)
{

    std::vector<MapPointPtr> vpMPs;

    g2o::SparseOptimizer optimizer;
    OrbLevenberg* solver = MakeLmOptimizerEigen<g2o::BlockSolver_6_3>(optimizer);
    optimizer.setVerbose(false);
    BindAbortFlag(optimizer, solver, pbStopFlag);

    long unsigned int maxKFid = 0;
    // Per-call membership sets (replace the old KeyFrame/MapPoint mnBALocalForMerge epoch stamps, P5-G)
    std::set<KeyFramePtr> spKeyFrameBA;
    std::set<MapPointPtr> spMapPointBA;

    Map* pCurrentMap = pMainKF->GetMap();

    // ---- vertices ----
    // Set fixed KeyFrame vertices
    int numInsertedPoints = 0;
    for(KeyFramePtr pKFi : vpFixedKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            Verbose::PrintMess("ERROR LBA: KF is bad or is not in the current map", Verbose::VERBOSITY_NORMAL);
            continue;
        }

        AddSE3Vertex(optimizer, pKFi, true);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        std::set<MapPointPtr> spViewMPs = pKFi->GetMapPoints();
        for(const MapPointPtr& pMPi : spViewMPs)
        {
            if(pMPi)
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)

                    if(!spMapPointBA.count(pMPi))
                    {
                        vpMPs.push_back(pMPi);
                        spMapPointBA.insert(pMPi);
                        numInsertedPoints++;
                    }
        }

        spKeyFrameBA.insert(pKFi);
    }

    // Set non fixed Keyframe vertices
    std::set<KeyFramePtr> spAdjustKF(vpAdjustKF.begin(), vpAdjustKF.end());
    numInsertedPoints = 0;
    for(KeyFramePtr pKFi : vpAdjustKF)
    {
        if(pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
        {
            continue;
        }

        // Fresh g2o vertex default: not fixed (upstream omitted the setFixed call)
        AddSE3Vertex(optimizer, pKFi, false);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;

        std::set<MapPointPtr> spViewMPs = pKFi->GetMapPoints();
        for(const MapPointPtr& pMPi : spViewMPs)
        {
            if(pMPi)
            {
                if(!pMPi->isBad() && pMPi->GetMap() == pCurrentMap)
                {
                    if(!spMapPointBA.count(pMPi))
                    {
                        vpMPs.push_back(pMPi);
                        spMapPointBA.insert(pMPi);
                        numInsertedPoints++;
                    }
                }
            }
        }

        spKeyFrameBA.insert(pKFi);
    }

    const int nExpectedSize = (vpAdjustKF.size()+vpFixedKF.size())*vpMPs.size();

    std::vector<ORB_SLAM3::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    std::vector<KeyFramePtr> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    std::vector<MapPointPtr> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    std::vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    std::vector<KeyFramePtr> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    std::vector<MapPointPtr> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);

    // ---- edges ---- (landmark vertices interleaved)
    // Set MapPoint vertices
    std::map<KeyFramePtr, int> mpObsKFs;
    std::map<KeyFramePtr, int> mpObsFinalKFs;
    std::map<MapPointPtr, int> mpObsMPs;
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPointPtr pMPi = vpMPs[i];
        if(pMPi->isBad())
        {
            continue;
        }

        const int id = pMPi->mnId+maxKFid+1;
        AddLandmarkVertex(optimizer, pMPi, id);


        const std::map<KeyFramePtr,std::tuple<int,int>> observations = pMPi->GetObservations();
        int nEdges = 0;
        //SET EDGES
        for(std::map<KeyFramePtr,std::tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            if(pKF->isBad() || pKF->mnId>maxKFid || !spKeyFrameBA.count(pKF) || !pKF->GetMapPoint(std::get<0>(mit->second)))
            {
                continue;
            }

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[std::get<0>(mit->second)];

            if(pKF->mvuRight[std::get<0>(mit->second)]<0) //Monocular
            {
                mpObsMPs[pMPi]++;
                Eigen::Matrix<double,2,1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                ORB_SLAM3::EdgeSE3ProjectXYZ* e = new ORB_SLAM3::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                AddHuberKernel(e, thHuber2D);

                e->pCamera = pKF->mpCamera;

                optimizer.addEdge(e);

                vpEdgesMono.push_back(e);
                vpEdgeKFMono.push_back(pKF);
                vpMapPointEdgeMono.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsMPs[pMPi]+=2;
                Eigen::Matrix<double,3,1> obs;
                const float kp_ur = pKF->mvuRight[std::get<0>(mit->second)];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                e->setInformation(Info);

                AddHuberKernel(e, thHuber3D);

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);

                vpEdgesStereo.push_back(e);
                vpEdgeKFStereo.push_back(pKF);
                vpMapPointEdgeStereo.push_back(pMPi);

                mpObsKFs[pKF]++;
            }
        }
    }

    // ---- solve ---- (two rounds: Huber, then inliers-only)
    if(pbStopFlag)
        if(pbStopFlag->load(std::memory_order_relaxed))
            return;

    optimizer.initializeOptimization();
    RunOptimization(optimizer, 5, "LocalBundleAdjustment(welding)", 0);

    bool bDoMore= true;

    if(pbStopFlag)
        if(pbStopFlag->load(std::memory_order_relaxed))
            bDoMore = false;

    std::map<unsigned long int, int> mWrongObsKF;
    if(bDoMore)
    {
        // Check inlier observations
        int badMonoMP = 0, badStereoMP = 0;
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPointPtr pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
            {
                continue;
            }

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badMonoMP++;
            }
            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPointPtr pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
            {
                continue;
            }

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
                badStereoMP++;
            }

            e->setRobustKernel(0);
        }
        Verbose::PrintMess("[BA]: First optimization(Huber), there are " + std::to_string(badMonoMP) + " monocular and " + std::to_string(badStereoMP) + " stereo bad edges", Verbose::VERBOSITY_DEBUG);

    optimizer.initializeOptimization(0);
    RunOptimization(optimizer, 10, "LocalBundleAdjustment(welding)", 1);
    }

    // ---- recover ---- (outlier cull + write-back)
    std::vector<std::pair<KeyFramePtr,MapPointPtr> > vToErase;
    vToErase.reserve(vpEdgesMono.size()+vpEdgesStereo.size());
    std::set<MapPointPtr> spErasedMPs;
    std::set<KeyFramePtr> spErasedKFs;

    // Check inlier observations
    int badMonoMP = 0, badStereoMP = 0;
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPointPtr pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
        {
            continue;
        }

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFMono[i];
            vToErase.emplace_back(pKFi,pMP);
            mWrongObsKF[pKFi->mnId]++;
            badMonoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPointPtr pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
        {
            continue;
        }

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFramePtr pKFi = vpEdgeKFStereo[i];
            vToErase.emplace_back(pKFi,pMP);
            mWrongObsKF[pKFi->mnId]++;
            badStereoMP++;

            spErasedMPs.insert(pMP);
            spErasedKFs.insert(pKFi);
        }
    }

    Verbose::PrintMess("[BA]: Second optimization, there are " + std::to_string(badMonoMP) + " monocular and " + std::to_string(badStereoMP) + " sterero bad edges", Verbose::VERBOSITY_DEBUG);

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMainKF->GetMap()->mMutexMapUpdate);

    if(!vToErase.empty())
    {
        for(size_t i=0;i<vToErase.size();i++)
        {
            KeyFramePtr pKFi = vToErase[i].first;
            MapPointPtr pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }
    for(unsigned int i=0; i < vpMPs.size(); ++i)
    {
        MapPointPtr pMPi = vpMPs[i];
        if(pMPi->isBad())
        {
            continue;
        }

        const std::map<KeyFramePtr,std::tuple<int,int>> observations = pMPi->GetObservations();
        for(std::map<KeyFramePtr,std::tuple<int,int>>::const_iterator mit=observations.begin(); mit!=observations.end(); mit++)
        {
            KeyFramePtr pKF = mit->first;
            // Leftover upstream comparison against localForKF (this function
            // never stamps it) -- feeds the mpObsFinalKFs statistics only
            if(pKF->isBad() || pKF->mnId>maxKFid || BAEpochs::get(epochs.localForKF,pKF) != pMainKF->mnId || !pKF->GetMapPoint(std::get<0>(mit->second)))
            {
                continue;
            }

            if(pKF->mvuRight[std::get<0>(mit->second)]<0) //Monocular
            {
                mpObsFinalKFs[pKF]++;
            }
            else // RGBD or Stereo
            {
                mpObsFinalKFs[pKF]++;
            }
        }
    }

    // Recover optimized data
    // Keyframes
    for(KeyFramePtr pKFi : vpAdjustKF)
    {
        if(pKFi->isBad())
        {
            continue;
        }

        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKFi->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        Sophus::SE3f Tiw(SE3quat.rotation().cast<float>(), SE3quat.translation().cast<float>());

        int numMonoBadPoints = 0, numMonoOptPoints = 0;
        int numStereoBadPoints = 0, numStereoOptPoints = 0;
        std::vector<MapPointPtr> vpMonoMPsOpt, vpStereoMPsOpt;
        std::vector<MapPointPtr> vpMonoMPsBad, vpStereoMPsBad;

        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            ORB_SLAM3::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPointPtr pMP = vpMapPointEdgeMono[i];
            KeyFramePtr pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
            {
                continue;
            }

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                numMonoBadPoints++;
                vpMonoMPsBad.push_back(pMP);

            }
            else
            {
                numMonoOptPoints++;
                vpMonoMPsOpt.push_back(pMP);
            }

        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPointPtr pMP = vpMapPointEdgeStereo[i];
            KeyFramePtr pKFedge = vpEdgeKFMono[i];

            if(pKFi != pKFedge)
            {
                continue;
            }

            if(pMP->isBad())
            {
                continue;
            }

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                numStereoBadPoints++;
                vpStereoMPsBad.push_back(pMP);
            }
            else
            {
                numStereoOptPoints++;
                vpStereoMPsOpt.push_back(pMP);
            }
        }

        pKFi->SetPose(Tiw);
    }

    //Points
    for(const MapPointPtr& pMPi : vpMPs)
    {
        if(pMPi->isBad())
        {
            continue;
        }

        g2o::VertexPointXYZ* vPoint = static_cast<g2o::VertexPointXYZ*>(optimizer.vertex(pMPi->mnId+maxKFid+1));
        pMPi->SetWorldPos(vPoint->estimate().cast<float>());
        pMPi->UpdateNormalAndDepth();

    }
}
} //namespace ORB_SLAM
