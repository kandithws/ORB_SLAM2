//
// Created by kandithws on 21/4/2562.
//

#include "Optimizer.h"
#include "g2o_types/g2o_Object.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "g2o_types/g2o_Object.h"

#include <unordered_set>

using namespace cv;
using namespace std;

namespace ORB_SLAM2 {

// Unary on 3rd constraint
void Optimizer::LocalBundleAdjustmentWithObjects(KeyFrame *pKF, bool* pbStopFlag, Map* pMap)
{
    auto start_graphcon = utils::time::time_now();
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;
    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (auto pKFi : vNeighKFs) {
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad()){
            lLocalKeyFrames.push_back(pKFi);
        }
    }


    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    unordered_set<MapPoint*> sLocalMapPoints;
    for (auto& lLocalKeyFrame : lLocalKeyFrames) {
        vector<MapPoint*> vpMPs = lLocalKeyFrame->GetMapPointMatches();
        for (auto pMP : vpMPs) {
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        sLocalMapPoints.insert(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for (auto& lLocalMapPoint : lLocalMapPoints) {
        auto observations = lLocalMapPoint->GetObservations();
        for (auto& observation : observations) {
            KeyFrame* pKFi = observation.first;
            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad()){
                    lFixedCameras.push_back(pKFi);
                }

            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;

    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFId = 0;
    unsigned long  maxLandmarkId = 0;


    // Set Local KeyFrame vertices
    unordered_set<uint32_t> sLandmarkIds;
    list<MapObject* > lLocalLandmarks;

    for (auto pKFi : lLocalKeyFrames) {
        auto objects = pKFi->GetMapObjects();
        for (auto& obj : objects){
            if (obj){
                if(!obj->IsReady())
                    continue;

                if (sLandmarkIds.find(obj->mnId) == sLandmarkIds.end()) {
                    sLandmarkIds.insert(obj->mnId);
                    lLocalLandmarks.push_back(obj);
                }
            }
        }

        // Add Keyframe vertices
        auto* vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFId)
            maxKFId = pKFi->mnId;
    }

    // Getting More fixed Keyframes
    bool update_rollpitch = Config::getInstance().OptimizationParams().update_rollpitch;
    bool update_scale = Config::getInstance().OptimizationParams().update_scale;
    for (const auto& pMO : lLocalLandmarks) {
        if(!pMO->IsReady())
            continue;

        Cuboid pInitCuboidGlobalPose;
        pMO->GetCuboid(pInitCuboidGlobalPose);
        auto* vCuboid = new g2o::VertexCuboid(update_rollpitch, update_scale);
        vCuboid->setEstimate(pInitCuboidGlobalPose);
        vCuboid->setId(maxKFId + 1 + pMO->mnId);
        vCuboid->setFixed(false);
        optimizer.addVertex(vCuboid);
        if (pMO->mnId > maxLandmarkId)
            maxLandmarkId = pMO->mnId;

        // Find More fixed KF that is observing this landmark !

        auto vObservingKFs = pMO->GetObservations();
        for (auto& observation : vObservingKFs){
            KeyFrame* pKFi = observation.first;
            if (pKFi){
                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad()){
                        lFixedCameras.push_back(pKFi);
                    }
                }
            }
        }
    }

    // Set Fixed KeyFrame vertices
    for (auto pKFi : lFixedCameras) {
        auto* vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFId)
            maxKFId = pKFi->mnId;
    }


    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrtf(5.991f);
    const float thHuberStereo = sqrtf(7.815f);
    const float thHuberObject = sqrt(900);

    // -----------------------------------------------------------
    for (auto pMP : lLocalMapPoints) {
        auto* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        unsigned long id = pMP->mnId + maxKFId + maxLandmarkId + 2;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const auto observations = pMP->GetObservations();

        //Set edges
        for (auto& observation : observations) {
            KeyFrame* pKFi = observation.first;

            if (!pKFi->isBad()) {
                const cv::KeyPoint& kpUn = pKFi->mvKeysUn[observation.second];

                // Monocular observation
                if (pKFi->mvuRight[observation.second] < 0) {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    auto* edgeSE3ProjectXYZX = new g2o::EdgeSE3ProjectXYZ();

                    edgeSE3ProjectXYZX->setVertex(0, optimizer.vertex(id));
                    edgeSE3ProjectXYZX->setVertex(1, optimizer.vertex(pKFi->mnId));
                    edgeSE3ProjectXYZX->setMeasurement(obs);
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    edgeSE3ProjectXYZX->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    auto* rk = new g2o::RobustKernelHuber;
                    edgeSE3ProjectXYZX->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    edgeSE3ProjectXYZX->fx = pKFi->fx;
                    edgeSE3ProjectXYZX->fy = pKFi->fy;
                    edgeSE3ProjectXYZX->cx = pKFi->cx;
                    edgeSE3ProjectXYZX->cy = pKFi->cy;

                    optimizer.addEdge(edgeSE3ProjectXYZX);
                    vpEdgesMono.push_back(edgeSE3ProjectXYZX);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[observation.second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    auto* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, optimizer.vertex(id));
                    e->setVertex(1, optimizer.vertex(pKFi->mnId));
                    e->setMeasurement(obs);
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    auto* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }


    // Adding Object Unary constraint here!
    double info_factor = 1.0;
    if (lLocalLandmarks.size() > 5){
        info_factor = 0.5;
    }
    for (const auto& pMO : lLocalLandmarks) {
        unsigned long landmark_id = maxKFId + 1 + pMO->mnId;
        // MapObject - KF

        auto vObservingKFs = pMO->GetObservations();
        for (auto& observation : vObservingKFs){

            KeyFrame* pKFi = observation.first;
            if (pKFi){
                if (pKFi->isBad())
                    continue;

                //auto vObservations = pKFi->GetObjectPredictions();
                //auto vObservationsMap = pKFi->GetMapObjectObservationsMap();
                auto obs = pKFi->FindObservation(pMO);
                if (!obs)
                    continue;
                auto K = Converter::toMatrix3d(pKFi->mK);
                //if (vObservationsMap.find(pMO) == vObservationsMap.end())
                //    continue;

                //auto obs = vObservations[vObservationsMap[pMO]];

                unsigned long landmark_id = maxKFId + 1 + pMO->mnId;
                auto* edgeSE3CuboidProj = new g2o::EdgeSE3CuboidProj(K);
                edgeSE3CuboidProj->setVertex(0, optimizer.vertex(pKFi->mnId));
                edgeSE3CuboidProj->setVertex(1, optimizer.vertex(landmark_id));

                edgeSE3CuboidProj->setMeasurement(Converter::toVector4d(obs->box()));
                Eigen::Matrix4d info = Eigen::Matrix4d::Identity() * obs->_confidence * obs->_confidence * info_factor;
                edgeSE3CuboidProj->setInformation(info);
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                edgeSE3CuboidProj->setRobustKernel(rk);
                rk->setDelta(thHuberObject);
                optimizer.addEdge(edgeSE3CuboidProj);
            }
        }

        // ------------ UNARY ---------------
        // Adding Mappoint Constraints

        auto vMapPointObservations = pMO->GetMapPoints();
        ORB_SLAM2::utils::eigen_aligned_vector<Eigen::Vector3d> vMPWorldPosHomo;
        vMPWorldPosHomo.reserve(vMapPointObservations.size());
        for (auto &pMP : vMapPointObservations) {
            if (pMP->isBad())
                continue;

            vMPWorldPosHomo.push_back(Converter::toVector3d(pMP->GetWorldPos()));
        }

        // Batch MP constriant
        auto* edgeMP = new g2o::EdgeCuboidMapPointUnaryBatch(vMPWorldPosHomo);
        edgeMP->setVertex(0, optimizer.vertex(landmark_id));
        edgeMP->setInformation(Eigen::Matrix3d::Identity() * 2.0);
        optimizer.addEdge(edgeMP);
    }

    auto graphcon_time = utils::time::time_diff_from_now_second(start_graphcon);

    if (pbStopFlag)
        if (*pbStopFlag)
            return;
    auto start_opt = utils::time::time_now();
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore) {

        // Check inlier observations
        for (size_t i = 0; i < vpEdgesMono.size(); i++) {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(nullptr);
        }

        for (size_t i = 0; i < vpEdgesStereo.size(); i++) {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(nullptr);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    SPDLOG_DEBUG("[OptTime] Optimization: {}, GraphConstruction: {}",
                 utils::time::time_diff_from_now_second(start_opt),
                 graphcon_time);

    vector<pair<KeyFrame*, MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0; i < vpEdgesMono.size(); i++) {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.emplace_back(pKFi, pMP);
        }
    }

    for (size_t i = 0; i < vpEdgesStereo.size(); i++) {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.emplace_back(pKFi, pMP);
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (auto& i : vToErase) {
            KeyFrame* pKFi = i.first;
            MapPoint* pMPi = i.second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for (auto pLocalKF : lLocalKeyFrames) {
        auto* vSE3 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pLocalKF->mnId));
        const g2o::SE3Quat& SE3quat = vSE3->estimate();
        pLocalKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (auto pMP : lLocalMapPoints) {
        auto* vPoint = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFId + maxLandmarkId + 2));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }


    //Landmarks
    for (const auto& pMO : lLocalLandmarks) {
        auto* vCube = dynamic_cast<g2o::VertexCuboid*>(optimizer.vertex(maxKFId + 1 + pMO->mnId));
        const g2o::Cuboid& cuboid = vCube->estimate();
        pMO->SetCuboid(cuboid);
    }


}

void Optimizer::LocalBundleAdjustmentWithObjects(KeyFrame *pKF, bool *pbStopFlag,
        Map *pMap, const cv::Mat& gravity){
    auto start_graphcon = utils::time::time_now();
    Eigen::Vector3d gNormalized = Converter::toVector3d(gravity / cv::norm(gravity));

    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;
    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (auto pKFi : vNeighKFs) {
        pKFi->mnBALocalForKF = pKF->mnId;
        if (!pKFi->isBad()){
            lLocalKeyFrames.push_back(pKFi);
        }
    }


    // Local MapPoints seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    unordered_set<MapPoint*> sLocalMapPoints;
    for (auto& lLocalKeyFrame : lLocalKeyFrames) {
        vector<MapPoint*> vpMPs = lLocalKeyFrame->GetMapPointMatches();
        for (auto pMP : vpMPs) {
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        sLocalMapPoints.insert(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for (auto& lLocalMapPoint : lLocalMapPoints) {
        auto observations = lLocalMapPoint->GetObservations();
        for (auto& observation : observations) {
            KeyFrame* pKFi = observation.first;
            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad()){
                    lFixedCameras.push_back(pKFi);
                }

            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;

    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX* solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFId = 0;
    unsigned long  maxLandmarkId = 0;

    // Set Local KeyFrame vertices
    unordered_set<uint32_t> sLandmarkIds;
    list<MapObject* > lLocalLandmarks;

    for (auto pKFi : lLocalKeyFrames) {
        auto objects = pKFi->GetMapObjects();
        for (auto& obj : objects){
            if (obj){
                if(!obj->IsReady())
                    continue;

                if (sLandmarkIds.find(obj->mnId) == sLandmarkIds.end()) {
                    sLandmarkIds.insert(obj->mnId);
                    lLocalLandmarks.push_back(obj);
                }
            }
        }

        // Add Keyframe vertices
        auto* vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFId)
            maxKFId = pKFi->mnId;
    }

    // Getting More fixed Keyframes

    bool update_rollpitch = Config::getInstance().OptimizationParams().update_rollpitch;
    bool update_scale = Config::getInstance().OptimizationParams().update_scale;

    for (const auto& pMO : lLocalLandmarks) {
        if(!pMO->IsReady())
            continue;

        Cuboid pInitCuboidGlobalPose;
        pMO->GetCuboid(pInitCuboidGlobalPose);
        auto* vCuboid = new g2o::VertexCuboid(update_rollpitch, update_scale);
        vCuboid->setEstimate(pInitCuboidGlobalPose);
        vCuboid->setId(maxKFId + 1 + pMO->mnId);
        vCuboid->setFixed(false);
        optimizer.addVertex(vCuboid);
        if (pMO->mnId > maxLandmarkId)
            maxLandmarkId = pMO->mnId;

        // Find More fixed KF that is observing this landmark !

        auto vObservingKFs = pMO->GetObservations();
        for (auto& observation : vObservingKFs){
            KeyFrame* pKFi = observation.first;
            if (pKFi){
                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad()){
                        lFixedCameras.push_back(pKFi);
                    }
                }
            }
        }
    }

    // Set Fixed KeyFrame vertices
    for (auto pKFi : lFixedCameras) {
        auto* vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFId)
            maxKFId = pKFi->mnId;
    }


    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrtf(5.991f);
    const float thHuberStereo = sqrtf(7.815f);
    const float thHuberObject = sqrt(900);

    // -----------------------------------------------------------
    for (auto pMP : lLocalMapPoints) {
        auto* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        unsigned long id = pMP->mnId + maxKFId + maxLandmarkId + 2;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const auto observations = pMP->GetObservations();

        //Set edges
        for (auto& observation : observations) {
            KeyFrame* pKFi = observation.first;

            if (!pKFi->isBad()) {
                const cv::KeyPoint& kpUn = pKFi->mvKeysUn[observation.second];

                // Monocular observation
                if (pKFi->mvuRight[observation.second] < 0) {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    auto* edgeSE3ProjectXYZX = new g2o::EdgeSE3ProjectXYZ();

                    edgeSE3ProjectXYZX->setVertex(0, optimizer.vertex(id));
                    edgeSE3ProjectXYZX->setVertex(1, optimizer.vertex(pKFi->mnId));
                    edgeSE3ProjectXYZX->setMeasurement(obs);
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    edgeSE3ProjectXYZX->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    auto* rk = new g2o::RobustKernelHuber;
                    edgeSE3ProjectXYZX->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    edgeSE3ProjectXYZX->fx = pKFi->fx;
                    edgeSE3ProjectXYZX->fy = pKFi->fy;
                    edgeSE3ProjectXYZX->cx = pKFi->cx;
                    edgeSE3ProjectXYZX->cy = pKFi->cy;

                    optimizer.addEdge(edgeSE3ProjectXYZX);
                    vpEdgesMono.push_back(edgeSE3ProjectXYZX);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[observation.second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    auto* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, optimizer.vertex(id));
                    e->setVertex(1, optimizer.vertex(pKFi->mnId));
                    e->setMeasurement(obs);
                    const float& invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    auto* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);
                }
            }
        }
    }


    // Adding Object Unary constraint here!
    double info_factor = 1.0;
    if (lLocalLandmarks.size() > 5){
        info_factor = 0.5;
    }
    for (const auto& pMO : lLocalLandmarks) {
        unsigned long landmark_id = maxKFId + 1 + pMO->mnId;
        // MapObject - KF

        auto vObservingKFs = pMO->GetObservations();
        for (auto& observation : vObservingKFs){

            KeyFrame* pKFi = observation.first;
            if (pKFi){
                if (pKFi->isBad())
                    continue;

                //auto vObservations = pKFi->GetObjectPredictions();
                //auto vObservationsMap = pKFi->GetMapObjectObservationsMap();
                auto obs = pKFi->FindObservation(pMO);
                if (!obs)
                    continue;
                auto K = Converter::toMatrix3d(pKFi->mK);

                //if (vObservationsMap.find(pMO) == vObservationsMap.end())
                //    continue;


                unsigned long landmark_id = maxKFId + 1 + pMO->mnId;
                auto* edgeSE3CuboidProj = new g2o::EdgeSE3CuboidProj(K);
                edgeSE3CuboidProj->setVertex(0, optimizer.vertex(pKFi->mnId));
                edgeSE3CuboidProj->setVertex(1, optimizer.vertex(landmark_id));
                // auto obs = vObservations[vObservationsMap[pMO]];
                edgeSE3CuboidProj->setMeasurement(Converter::toVector4d(obs->box()));
                Eigen::Matrix4d info = Eigen::Matrix4d::Identity() * obs->_confidence * obs->_confidence * info_factor;
                edgeSE3CuboidProj->setInformation(info);
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                edgeSE3CuboidProj->setRobustKernel(rk);
                rk->setDelta(thHuberObject);
                optimizer.addEdge(edgeSE3CuboidProj);

            }
        }

        // ------------ UNARY ---------------
        // Adding Mappoint Constraints

        auto vMapPointObservations = pMO->GetMapPoints();
        ORB_SLAM2::utils::eigen_aligned_vector<Eigen::Vector3d> vMPWorldPosHomo;
        vMPWorldPosHomo.reserve(vMapPointObservations.size());
        for (auto &pMP : vMapPointObservations) {
            if (pMP->isBad())
                continue;

            vMPWorldPosHomo.push_back(Converter::toVector3d(pMP->GetWorldPos()));
        }

        // Batch MP constriant
        auto* edgeMP = new g2o::EdgeCuboidMapPointUnaryBatch(vMPWorldPosHomo);
        edgeMP->setVertex(0, optimizer.vertex(landmark_id));
        edgeMP->setInformation(Eigen::Matrix3d::Identity() * 2.0);
        optimizer.addEdge(edgeMP);

        // Gravity Constriant
        auto* edgeGravity = new g2o::EdgeCuboidGravityConstraint2(gNormalized);
        edgeGravity->setVertex(0, optimizer.vertex(landmark_id));
        optimizer.addEdge(edgeGravity);
    }

    auto graphcon_time = utils::time::time_diff_from_now_second(start_graphcon);

    if (pbStopFlag)
        if (*pbStopFlag)
            return;
    auto start_opt = utils::time::time_now();
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore) {

        // Check inlier observations
        for (size_t i = 0; i < vpEdgesMono.size(); i++) {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(nullptr);
        }

        for (size_t i = 0; i < vpEdgesStereo.size(); i++) {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(nullptr);
        }

        // Optimize again without the outliers

        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    SPDLOG_DEBUG("[OptTime] Optimization: {}, GraphConstruction: {}",
                 utils::time::time_diff_from_now_second(start_opt),
                 graphcon_time);

    vector<pair<KeyFrame*, MapPoint*> > vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0; i < vpEdgesMono.size(); i++) {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToErase.emplace_back(pKFi, pMP);
        }
    }

    for (size_t i = 0; i < vpEdgesStereo.size(); i++) {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToErase.emplace_back(pKFi, pMP);
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (auto& i : vToErase) {
            KeyFrame* pKFi = i.first;
            MapPoint* pMPi = i.second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for (auto pLocalKF : lLocalKeyFrames) {
        auto* vSE3 = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pLocalKF->mnId));
        const g2o::SE3Quat& SE3quat = vSE3->estimate();
        pLocalKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (auto pMP : lLocalMapPoints) {
        auto* vPoint = dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId + maxKFId + maxLandmarkId + 2));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }


    //Landmarks
    for (const auto& pMO : lLocalLandmarks) {
        auto* vCube = dynamic_cast<g2o::VertexCuboid*>(optimizer.vertex(maxKFId + 1 + pMO->mnId));
        const g2o::Cuboid& cuboid = vCube->estimate();
        // pMO->SetPoseAndDimension(cuboid);
        pMO->SetCuboid(cuboid);
    }
}

}
