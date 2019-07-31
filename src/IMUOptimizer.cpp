//
// Created by kandithws on 23/6/2562.
//
#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <Eigen/StdVector>

#include "Converter.h"

#include<mutex>

//#include "imu/configparam.h"
#include "utils/Config.h"
#include "imu/g2o_types.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_cholmod.h"
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

namespace ORB_SLAM2 {

void
Optimizer::LocalBAPRVIDP(KeyFrame *pCurKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag, Map *pMap,
                         cv::Mat &gw, LocalMapping *pLM) {
    // Check current KeyFrame in local window
    if (pCurKF != lLocalKeyFrames.back())
        cerr << "pCurKF != lLocalKeyFrames.back. check" << endl;

    auto start_graphcon = utils::time::time_now();
    // Extrinsics
    Matrix4d Tcb = Config::getInstance().IMUParams().GetEigTcb();
    Matrix3d Rcb = Tcb.topLeftCorner(3, 3);
    Vector3d Pcb = Tcb.topRightCorner(3, 1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    // All KeyFrames in Local window are optimized
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        if (!pKFi) cerr << "!pKFi. why??????" << endl;
        pKFi->mnBALocalForKF = pCurKF->mnId;
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pCurKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pCurKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    // Add the KeyFrame before local window.
    KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
    if (pKFPrevLocal) {
        // Test log
        if (pKFPrevLocal->isBad()) cerr << "KeyFrame before local window is bad?" << endl;
        if (pKFPrevLocal->mnBAFixedForKF == pCurKF->mnId)
            cerr << "KeyFrame before local, has been added to lFixedKF?" << endl;
        if (pKFPrevLocal->mnBALocalForKF == pCurKF->mnId)
            cerr << "KeyFrame before local, has been added to lLocalKF?" << endl;

        pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
        if (!pKFPrevLocal->isBad())
            lFixedCameras.push_back(pKFPrevLocal);
        else
            cerr << "pKFPrevLocal is Bad?" << endl;
    }
        // Test log
    else { cerr << "pKFPrevLocal is NULL?" << endl; }
    // Covisible KeyFrames
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pCurKF->mnId && pKFi->mnBAFixedForKF != pCurKF->mnId) {
                pKFi->mnBAFixedForKF = pCurKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    //linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);
//optimizer.setVerbose(true);
    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int maxKFid = 0;

    // Extrinsic vertex
    int ExtrinsicVertexId = KeyFrame::nNextId * 3 + MapPoint::nNextId + 1;
    {
        g2o::VertexNavStatePR *vTcb = new g2o::VertexNavStatePR();
        NavState tmpNSTcb;
        tmpNSTcb.Set_Pos(Pcb);
        tmpNSTcb.Set_Rot(Rcb);
        vTcb->setEstimate(tmpNSTcb);
        vTcb->setId(ExtrinsicVertexId);
        vTcb->setFixed(true); //todo
        optimizer.addVertex(vTcb);
    }

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        if (!pKFi) cerr << "!pKFi when setting local KF vertex???" << endl;
        int idKF = pKFi->mnId * 3;
        // Vertex of PR/V
        {
            g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
            vNSPR->setEstimate(pKFi->GetNavState());
            vNSPR->setId(idKF);
            vNSPR->setFixed(false);
            optimizer.addVertex(vNSPR);

            g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
            vNSV->setEstimate(pKFi->GetNavState());
            vNSV->setId(idKF + 1);
            vNSV->setFixed(false);
            optimizer.addVertex(vNSV);
        }
        // Vertex of Bias
        {
            g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF + 2);
            vNSBias->setFixed(false);
            optimizer.addVertex(vNSBias);
        }

        if (idKF + 2 > maxKFid)
            maxKFid = idKF + 2;

    }

    // Set Fixed KeyFrame vertices. Including the pKFPrevLocal.
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        int idKF = pKFi->mnId * 3;
        // For common fixed KeyFrames, only add PR vertex
        {
            g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
            vNSPR->setEstimate(pKFi->GetNavState());
            vNSPR->setId(idKF);
            vNSPR->setFixed(true);
            optimizer.addVertex(vNSPR);
        }
        // For Local-Window-Previous KeyFrame, add V and Bias vertex
        if (pKFi == pKFPrevLocal) {
            g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
            vNSV->setEstimate(pKFi->GetNavState());
            vNSV->setId(idKF + 1);
            vNSV->setFixed(true);  //todo
            optimizer.addVertex(vNSV);

            g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF + 2);
            vNSBias->setFixed(true);    //todo. true or false
            optimizer.addVertex(vNSBias);
        }

        if (idKF + 2 > maxKFid)
            maxKFid = idKF + 2;

    }

    // Edges between KeyFrames in Local Window
    // and
    // Edges between 1st KeyFrame of Local Window and its previous (fixed)KeyFrame - pKFPrevLocal
    vector<g2o::EdgeNavStatePRV *> vpEdgesNavStatePRV;
    vector<g2o::EdgeNavStateBias *> vpEdgesNavStateBias;
    // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
    // 12.592/16.812 for 0.95/0.99 6DoF
    // 16.919/21.666 for 0.95/0.99 9DoF
    //const float thHuberNavState = sqrt(30.5779);
    const float thHuberNavStatePRV = sqrt(/*1e4*/100 * 21.666);
    const float thHuberNavStateBias = sqrt(/*1e4*/100 * 16.812);
    // Inverse covariance of bias random walk

    Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
    InvCovBgaRW.topLeftCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKF1 = *lit;                      // Current KF, store the IMU pre-integration between previous-current
        if (!pKF1) cerr << "pKF1" << endl;
        KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();   // Previous KF

        if (!pKF1 || !pKF0) cerr << "pKF1=" << (size_t) pKF1 << ", pKF0=" << (size_t) pKF0 << endl;
        // PVR edge
        {
            // PR0, PR1, V0, V1, B0
            g2o::EdgeNavStatePRV *epvr = new g2o::EdgeNavStatePRV();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId)));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 1)));
            epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId + 1)));
            epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 2)));
            epvr->setMeasurement(pKF1->GetIMUPreInt());

            //Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            Matrix9d CovPRV = pKF1->GetIMUPreInt().getCovPVPhi();
            CovPRV.col(3).swap(CovPRV.col(6));
            CovPRV.col(4).swap(CovPRV.col(7));
            CovPRV.col(5).swap(CovPRV.col(8));
            CovPRV.row(3).swap(CovPRV.row(6));
            CovPRV.row(4).swap(CovPRV.row(7));
            CovPRV.row(5).swap(CovPRV.row(8));
            epvr->setInformation(CovPRV.inverse());

            epvr->SetParams(GravityVec);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            epvr->setRobustKernel(rk);
            rk->setDelta(thHuberNavStatePRV);

            optimizer.addEdge(epvr);
            vpEdgesNavStatePRV.push_back(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 2)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId + 2)));
            ebias->setMeasurement(pKF1->GetIMUPreInt());

            ebias->setInformation(InvCovBgaRW / pKF1->GetIMUPreInt().getDeltaTime());

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            ebias->setRobustKernel(rk);
            rk->setDelta(thHuberNavStateBias);

            optimizer.addEdge(ebias);
            vpEdgesNavStateBias.push_back(ebias);
        }

    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    //vector<g2o::EdgeNavStatePRPointXYZ*> vpEdgesMono;
    vector<g2o::EdgePRIDP *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

    vector<Vector2d, Eigen::aligned_allocator<Vector2d> > vRefNormXY;
    vRefNormXY.resize(lLocalMapPoints.size());
    vector<bool> vMPGood;
    vMPGood.resize(lLocalMapPoints.size(), false);
    int mpcnt = 0;

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++, mpcnt++) {
        MapPoint *pMP = *lit;

        Vector3d Pw = Converter::toVector3d(pMP->GetWorldPos());
        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
        if (!pRefKF)
            cerr << ("!pRefKF???");
        if (/*!sLocalFixKFs.count(pRefKF) */pRefKF->isBad()) {
            cerr << pRefKF->mnId << " pRefKF not in local or fixed window, bBad=" << pRefKF->isBad() << endl;
            continue;
        }
        Matrix3d Rcw = Converter::toMatrix3d(pRefKF->GetRotation());
        Vector3d tcw = Converter::toVector3d(pRefKF->GetTranslation());
        Vector3d Pc = Rcw * Pw + tcw;
        double dc = Pc[2];  //depth
        if (dc < 0.01) {
            std::cerr << "depth < 0.01, = " << dc << std::endl;
            dc = 0.01;
            continue;
        }

        int mpVertexId = pMP->mnId + maxKFid + 1;
        g2o::VertexIDP *vPoint = new g2o::VertexIDP();
        vPoint->setEstimate(1.0 / dc);
        vPoint->setId(mpVertexId);
        vPoint->setMarginalized(true);
        //optimizer.addVertex(vPoint);

        mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();
        // compute normalized image coordinate [x,y]
        if (!observations.count(pRefKF))
            cerr << "!observations.count(pRefKF), why???" << endl;
        size_t kpIdxInRefKF = observations[pRefKF];
        //    cerr<<pMP->mnId<<" refKFid: "<<kpIdxInRefKF<<endl;
        const cv::KeyPoint &kpRefUn = pRefKF->mvKeysUn[kpIdxInRefKF];
        double normx = (kpRefUn.pt.x - pRefKF->cx) / pRefKF->fx;
        double normy = (kpRefUn.pt.y - pRefKF->cy) / pRefKF->fy;
        vRefNormXY[mpcnt] << normx, normy;

        //        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        //        vPoint->setEstimate(Pw);
        //        int id = pMP->mnId+maxKFid+1;
        //        vPoint->setId(id);
        //        vPoint->setMarginalized(true);
        //        optimizer.addVertex(vPoint);

        bool baddmpvertex = false;

        // Set edges between KeyFrame and MapPoint
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;
            if (pKFi == pRefKF) {
                continue;   // no edge for reference keyframe
            }

            if (!pKFi->isBad()) {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0) {
                    // There may be no available observation. So when there's observation then add vertex of mappoint
                    if (!baddmpvertex) {
                        baddmpvertex = true;
                        optimizer.addVertex(vPoint);
                        vMPGood[mpcnt] = true;
                    }

                    // 0-idp, 1-refPR, 2-curPR, 3-TcbPR
                    g2o::EdgePRIDP *e = new g2o::EdgePRIDP();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(mpVertexId)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pRefKF->mnId)));
                    e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKFi->mnId)));
                    e->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(ExtrinsicVertexId)));
                    e->SetParams(normx, normy, pRefKF->fx, pRefKF->fy, pRefKF->cx, pRefKF->cy);

                    //g2o::EdgeNavStatePRPointXYZ* e = new g2o::EdgeNavStatePRPointXYZ();
                    //e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    //e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(3*pKFi->mnId)));
                    //e->SetParams(pKFi->fx,pKFi->fy,pKFi->cx,pKFi->cy,Rcb.transpose(),-Rcb.transpose()*Pcb);

                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;
                    e->setMeasurement(obs);

                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                } else {
                    // Test log
                    cerr << "Stereo not supported yet, why here?? check." << endl;
                }

                baddmpvertex = true;
            }
        }
    }

    auto graphcon_time = utils::time::time_diff_from_now_second(start_graphcon);
    if (pbStopFlag)
        if (*pbStopFlag)
            return;
    auto start_opt = utils::time::time_now();
    // First try
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();

    optimizer.optimize(5);


    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;
    if (!bDoMore) cerr << "Hint: local mapping optimize only 5 iter. Need more computation resource." << endl;
    if (bDoMore) {
        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            //g2o::EdgeNavStatePRPointXYZ* e = vpEdgesMono[i];
            g2o::EdgePRIDP *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive() ||
                static_cast<g2o::VertexIDP *>(e->vertex(0))->estimate() < 2e-6) {
                e->setLevel(1);
                //cerr<<"e->chi2(): "<<e->chi2()<<", inverse depth rho: "<<static_cast<g2o::VertexIDP*>(e->vertex(0))->estimate()<<endl;
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    SPDLOG_DEBUG("[OptTime] Optimization: {}, GraphConstruction: {}",
                 utils::time::time_diff_from_now_second(start_opt),
                 graphcon_time);

    //
    vector<pair<KeyFrame *, MapPoint *> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    double PosePointchi2 = 0;
    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        //g2o::EdgeNavStatePRPointXYZ* e = vpEdgesMono[i];
        g2o::EdgePRIDP *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive() ||
            static_cast<g2o::VertexIDP *>(e->vertex(0))->estimate() < 2e-6 || e->level() != 0) {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }

        PosePointchi2 += e->chi2();
    }

//    // Debug log
//    // Check inlier observations
//    int tmpcnt2=0;
//    for(size_t i=0, iend=vpEdgesNavStatePRV.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStatePRV* e = vpEdgesNavStatePRV[i];
//        if(e->chi2()>21.666)
//        {
//                        cout<<"2 PRVedge "<<i<<", chi2 "<<e->chi2()<<". ";
//                        tmpcnt2++;
//        }
//    }
//        //cout<<endl<<"edge bias ns bad: ";
//    for(size_t i=0, iend=vpEdgesNavStateBias.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];
//        if(e->chi2()>16.812)
//        {
//                        cout<<"2 Bias edge "<<i<<", chi2 "<<e->chi2()<<". ";
//                        tmpcnt2++;
//        }
//    }
//    if(tmpcnt2>0)
//        cout<<endl;

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    //Keyframes
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexNavStatePR *vNSPR = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(3 * pKFi->mnId));
        g2o::VertexNavStateV *vNSV = static_cast<g2o::VertexNavStateV *>(optimizer.vertex(3 * pKFi->mnId + 1));
        g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(3 * pKFi->mnId + 2));
        // In optimized navstate, bias not changed, delta_bias not zero, should be added to bias
        const NavState &optPRns = vNSPR->estimate();
        const NavState &optVns = vNSV->estimate();
        const NavState &optBiasns = vNSBias->estimate();
        NavState primaryns = pKFi->GetNavState();
        // Update NavState
        pKFi->SetNavStatePos(optPRns.Get_P());
        pKFi->SetNavStateRot(optPRns.Get_R());
        pKFi->SetNavStateVel(optVns.Get_V());
        //if(optBiasns.Get_dBias_Acc().norm()<1e-2 && optBiasns.Get_BiasGyr().norm()<1e-4)
        //{
        pKFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
        pKFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
        //}

        // Update pose Tcw
        pKFi->UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());

        // Test log
        if ((primaryns.Get_BiasGyr() - optPRns.Get_BiasGyr()).norm() > 1e-6 ||
            (primaryns.Get_BiasGyr() - optBiasns.Get_BiasGyr()).norm() > 1e-6)
            cerr << "gyr bias change in optimization?" << endl;
        if ((primaryns.Get_BiasAcc() - optPRns.Get_BiasAcc()).norm() > 1e-6 ||
            (primaryns.Get_BiasAcc() - optBiasns.Get_BiasAcc()).norm() > 1e-6)
            cerr << "acc bias change in optimization?" << endl;

    }

    //Points
    mpcnt = 0;
    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++, mpcnt++) {
        MapPoint *pMP = *lit;
        if (!pMP) cerr << ("!pMP????");
        KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
        if (!pRefKF) cerr << ("!pRefKF???");
        if (/*!sLocalFixKFs.count(pRefKF)*/pRefKF->isBad()) {
            continue;
        }
        if (!vMPGood[mpcnt])
            continue;
        //g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        g2o::VertexIDP *vPoint = static_cast<g2o::VertexIDP *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
        const Vector2d &refXY = vRefNormXY[mpcnt];
        Vector3d Pref;
        Pref << refXY[0], refXY[1], 1.0;
        double rho = vPoint->estimate();
        //if(rho<1e-4) {cerr<<"rho="<<rho<<". ";rho = 1e-4;}

        Pref *= 1.0 / rho;    // point coordinate in reference frame

        cv::Mat Rwr = pRefKF->GetRotation().t();       // Rwr = Rrw^T
        cv::Mat twr = pRefKF->GetCameraCenter();       // twr = - Rrw^T * trw

        pMP->SetWorldPos(Rwr * Converter::toCvMat(Pref) + twr);   // point coordinate in world frame
        pMP->UpdateNormalAndDepth();
    }

    // update Tcb
    {
        g2o::VertexNavStatePR *vTcb = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(ExtrinsicVertexId));
        Matrix3d Rcb = vTcb->estimate().Get_RotMatrix();
        Vector3d tcb = vTcb->estimate().Get_P();
        if (!vTcb->fixed())
            cerr << "opt Rcb/tcb:" << endl << Rcb << endl << tcb.transpose() << endl;
        //todo
    }

    if (pLM) {
        pLM->SetMapUpdateFlagInTracking(true);
    }

}

void Optimizer::GlobalBundleAdjustmentNavStatePRV(Map *pMap, const cv::Mat &gw, int nIterations, bool *pbStopFlag,
                                                  const unsigned long nLoopKF, const bool bRobust) {
    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint *> vpMP = pMap->GetAllMapPoints();

    // Extrinsics
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3, 3);
    Vector3d Pbc = Tbc.topRightCorner(3, 1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    //linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;

        // PR
        g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
        vNSPR->setEstimate(pKF->GetNavState());
        vNSPR->setId(pKF->mnId * 3);
        vNSPR->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vNSPR);
        // V
        g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
        vNSV->setEstimate(pKF->GetNavState());
        vNSV->setId(pKF->mnId * 3 + 1);
        vNSV->setFixed(false);
        optimizer.addVertex(vNSV);
        // Bias
        g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
        vNSBias->setEstimate(pKF->GetNavState());
        vNSBias->setId(pKF->mnId * 3 + 2);
        vNSBias->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vNSBias);

        if (pKF->mnId * 3 + 2 > maxKFid)
            maxKFid = pKF->mnId * 3 + 2;
    }

    // Add NavState PRV/Bias edges
    const float thHuberNavStatePRV = sqrt(100 * 21.666);
    const float thHuberNavStateBias = sqrt(100 * 16.812);
    // Inverse covariance of bias random walk

    Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
    InvCovBgaRW.topLeftCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE


    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF1 = vpKFs[i];
        if (pKF1->isBad()) {
            cout << "pKF is bad in gBA, id " << pKF1->mnId << endl;   //Debug log
            continue;
        }

        KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();
        if (!pKF0) {
            if (pKF1->mnId != 0) cerr << "Previous KeyFrame is NULL?" << endl;  //Test log
            continue;
        }

        // PVR edge
        {
            // PR0, PR1, V0, V1, B0
            g2o::EdgeNavStatePRV *epvr = new g2o::EdgeNavStatePRV();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId)));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 1)));
            epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId + 1)));
            epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 2)));
            epvr->setMeasurement(pKF1->GetIMUPreInt());

            //Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            Matrix9d CovPRV = pKF1->GetIMUPreInt().getCovPVPhi();
            CovPRV.col(3).swap(CovPRV.col(6));
            CovPRV.col(4).swap(CovPRV.col(7));
            CovPRV.col(5).swap(CovPRV.col(8));
            CovPRV.row(3).swap(CovPRV.row(6));
            CovPRV.row(4).swap(CovPRV.row(7));
            CovPRV.row(5).swap(CovPRV.row(8));
            epvr->setInformation(CovPRV.inverse());

            epvr->SetParams(GravityVec);

            if (bRobust) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                epvr->setRobustKernel(rk);
                rk->setDelta(thHuberNavStatePRV);
            }

            optimizer.addEdge(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 2)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId + 2)));
            ebias->setMeasurement(pKF1->GetIMUPreInt());

            ebias->setInformation(InvCovBgaRW / pKF1->GetIMUPreInt().getDeltaTime());

            if (bRobust) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                ebias->setRobustKernel(rk);
                rk->setDelta(thHuberNavStateBias);
            }

            optimizer.addEdge(ebias);
        }

    }

    const float thHuber2D = sqrt(5.99);

    // Set MapPoint vertices
    for (size_t i = 0; i < vpMP.size(); i++) {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit = observations.begin();
             mit != observations.end(); mit++) {

            KeyFrame *pKF = mit->first;
            if (pKF->isBad() || 3 * pKF->mnId > maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if (pKF->mvuRight[mit->second] < 0) {
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeNavStatePRPointXYZ *e = new g2o::EdgeNavStatePRPointXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->SetParams(pKF->fx, pKF->fy, pKF->cx, pKF->cy, Rbc, Pbc);

                optimizer.addEdge(e);
            } else {
                cerr << "Stereo not supported" << endl;
            }
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        } else {
            vbNotIncludedMP[i] = false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        //g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        //g2o::SE3Quat SE3quat = vSE3->estimate();
        g2o::VertexNavStatePR *vNSPR = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(3 * pKF->mnId));
        g2o::VertexNavStateV *vNSV = static_cast<g2o::VertexNavStateV *>(optimizer.vertex(3 * pKF->mnId + 1));
        g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(3 * pKF->mnId + 2));
        const NavState &nspr = vNSPR->estimate();
        const NavState &nsv = vNSV->estimate();
        const NavState &nsbias = vNSBias->estimate();
        NavState ns_recov = nspr;
        ns_recov.Set_Pos(nspr.Get_P());
        ns_recov.Set_Rot(nspr.Get_R());
        ns_recov.Set_Vel(nsv.Get_V());
        ns_recov.Set_DeltaBiasGyr(nsbias.Get_dBias_Gyr());
        ns_recov.Set_DeltaBiasAcc(nsbias.Get_dBias_Acc());

        if (nLoopKF == 0) {
            //pKF->SetPose(Converter::toCvMat(SE3quat));
            pKF->SetNavState(ns_recov);
            pKF->UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());
        } else {
            pKF->mNavStateGBA = ns_recov;

            pKF->mTcwGBA.create(4, 4, CV_32F);
            //Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            cv::Mat Twb_ = cv::Mat::eye(4, 4, CV_32F);
            Converter::toCvMat(pKF->mNavStateGBA.Get_RotMatrix()).copyTo(Twb_.rowRange(0, 3).colRange(0, 3));
            Converter::toCvMat(pKF->mNavStateGBA.Get_P()).copyTo(Twb_.rowRange(0, 3).col(3));
            cv::Mat Twc_ = Twb_ * Config::getInstance().IMUParams().GetMatTbc();
            pKF->mTcwGBA = Converter::toCvMatInverse(Twc_);

            pKF->mnBAGlobalForKF = nLoopKF;
        }

    }

    //Points
    for (size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint *pMP = vpMP[i];

        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0) {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        } else {
            pMP->mPosGBA.create(3, 1, CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

void Optimizer::LocalBundleAdjustmentNavStatePRV(KeyFrame *pCurKF, const std::list<KeyFrame *> &lLocalKeyFrames,
                                                 bool *pbStopFlag, Map *pMap, cv::Mat &gw, LocalMapping *pLM) {
    // Check current KeyFrame in local window
    if (pCurKF != lLocalKeyFrames.back())
        cerr << "pCurKF != lLocalKeyFrames.back. check" << endl;

    // Extrinsics
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3, 3);
    Vector3d Pbc = Tbc.topRightCorner(3, 1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    // All KeyFrames in Local window are optimized
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        if (!pKFi) cerr << "!pKFi. why??????" << endl;
        pKFi->mnBALocalForKF = pCurKF->mnId;
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pCurKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pCurKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    // Add the KeyFrame before local window.
    KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
    if (pKFPrevLocal) {
        // Test log
        if (pKFPrevLocal->isBad()) cerr << "KeyFrame before local window is bad?" << endl;
        if (pKFPrevLocal->mnBAFixedForKF == pCurKF->mnId)
            cerr << "KeyFrame before local, has been added to lFixedKF?" << endl;
        if (pKFPrevLocal->mnBALocalForKF == pCurKF->mnId)
            cerr << "KeyFrame before local, has been added to lLocalKF?" << endl;

        pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
        if (!pKFPrevLocal->isBad())
            lFixedCameras.push_back(pKFPrevLocal);
        else
            cerr << "pKFPrevLocal is Bad?" << endl;
    }
        // Test log
    else { cerr << "pKFPrevLocal is NULL?" << endl; }
    // Covisible KeyFrames
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pCurKF->mnId && pKFi->mnBAFixedForKF != pCurKF->mnId) {
                pKFi->mnBAFixedForKF = pCurKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    //linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        if (!pKFi) cerr << "!pKFi when setting local KF vertex???" << endl;
        int idKF = pKFi->mnId * 3;
        // Vertex of PR/V
        {
            g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
            vNSPR->setEstimate(pKFi->GetNavState());
            vNSPR->setId(idKF);
            vNSPR->setFixed(false);
            optimizer.addVertex(vNSPR);

            g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
            vNSV->setEstimate(pKFi->GetNavState());
            vNSV->setId(idKF + 1);
            vNSV->setFixed(false);
            optimizer.addVertex(vNSV);
        }
        // Vertex of Bias
        {
            g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF + 2);
            vNSBias->setFixed(false);
            optimizer.addVertex(vNSBias);
        }

        if (idKF + 2 > maxKFid)
            maxKFid = idKF + 2;
    }

    // Set Fixed KeyFrame vertices. Including the pKFPrevLocal.
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        int idKF = pKFi->mnId * 3;
        // For common fixed KeyFrames, only add PR vertex
        {
            g2o::VertexNavStatePR *vNSPR = new g2o::VertexNavStatePR();
            vNSPR->setEstimate(pKFi->GetNavState());
            vNSPR->setId(idKF);
            vNSPR->setFixed(true);
            optimizer.addVertex(vNSPR);
        }
        // For Local-Window-Previous KeyFrame, add V and Bias vertex
        if (pKFi == pKFPrevLocal) {
            g2o::VertexNavStateV *vNSV = new g2o::VertexNavStateV();
            vNSV->setEstimate(pKFi->GetNavState());
            vNSV->setId(idKF + 1);
            vNSV->setFixed(true);
            optimizer.addVertex(vNSV);

            g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF + 2);
            vNSBias->setFixed(true);
            optimizer.addVertex(vNSBias);
        }

        if (idKF + 2 > maxKFid)
            maxKFid = idKF + 2;
    }

    // Edges between KeyFrames in Local Window
    // and
    // Edges between 1st KeyFrame of Local Window and its previous (fixed)KeyFrame - pKFPrevLocal
    vector<g2o::EdgeNavStatePRV *> vpEdgesNavStatePRV;
    vector<g2o::EdgeNavStateBias *> vpEdgesNavStateBias;
    // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
    // 12.592/16.812 for 0.95/0.99 6DoF
    // 16.919/21.666 for 0.95/0.99 9DoF
    //const float thHuberNavState = sqrt(30.5779);
    const float thHuberNavStatePRV = sqrt(100 * 21.666);
    const float thHuberNavStateBias = sqrt(100 * 16.812);
    // Inverse covariance of bias random walk

    Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
    InvCovBgaRW.topLeftCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKF1 = *lit;                      // Current KF, store the IMU pre-integration between previous-current
        if (!pKF1) cerr << "pKF1" << endl;
        KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();   // Previous KF

        if (!pKF1 || !pKF0) cerr << "pKF1=" << (size_t) pKF1 << ", pKF0=" << (size_t) pKF0 << endl;
        // PVR edge
        {
            // PR0, PR1, V0, V1, B0
            g2o::EdgeNavStatePRV *epvr = new g2o::EdgeNavStatePRV();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId)));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 1)));
            epvr->setVertex(3, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId + 1)));
            epvr->setVertex(4, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 2)));
            epvr->setMeasurement(pKF1->GetIMUPreInt());

            //Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            Matrix9d CovPRV = pKF1->GetIMUPreInt().getCovPVPhi();
            CovPRV.col(3).swap(CovPRV.col(6));
            CovPRV.col(4).swap(CovPRV.col(7));
            CovPRV.col(5).swap(CovPRV.col(8));
            CovPRV.row(3).swap(CovPRV.row(6));
            CovPRV.row(4).swap(CovPRV.row(7));
            CovPRV.row(5).swap(CovPRV.row(8));
            epvr->setInformation(CovPRV.inverse());

            epvr->SetParams(GravityVec);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            epvr->setRobustKernel(rk);
            rk->setDelta(thHuberNavStatePRV);

            optimizer.addEdge(epvr);
            vpEdgesNavStatePRV.push_back(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF0->mnId + 2)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKF1->mnId + 2)));
            ebias->setMeasurement(pKF1->GetIMUPreInt());

            ebias->setInformation(InvCovBgaRW / pKF1->GetIMUPreInt().getDeltaTime());

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            ebias->setRobustKernel(rk);
            rk->setDelta(thHuberNavStateBias);

            optimizer.addEdge(ebias);
            vpEdgesNavStateBias.push_back(ebias);
        }

//        // Test log
//        if(pKF1->GetIMUPreInt().getDeltaTime() < 1e-3)
//        {
//            cerr<<"IMU pre-integrator delta time between 2 KFs too small: "<<pKF1->GetIMUPreInt().getDeltaTime()<<endl;
//            cerr<<"No EdgeNavState added"<<endl;
//            continue;
//        }
//        if(lit == lLocalKeyFrames.begin())
//        {
//            // First KF in Local Window, link (fixed) pKFPrevLocal
//            if(pKF0 != pKFPrevLocal) cerr<<"pKF0 != pKFPrevLocal for 1st KF in Local Window, id: "<<pKF0->mnId<<","<<pKFPrevLocal->mnId<<endl;
//        }
//        else
//        {
//            // KFs in Local Window, link another local KF
//        }
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeNavStatePRPointXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        cv::Mat Pw = pMP->GetWorldPos();

        const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();

        // Set edges between KeyFrame and MapPoint
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad()) {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0) {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeNavStatePRPointXYZ *e = new g2o::EdgeNavStatePRPointXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(3 * pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->SetParams(pKFi->fx, pKFi->fy, pKFi->cx, pKFi->cy, Rbc, Pbc);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                } else {
                    // Test log
                    cerr << "Stereo not supported yet, why here?? check." << endl;
                }
            }
        }
    }


    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    // First try
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();

    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore) {
        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeNavStatePRPointXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

//    // Check inlier observations
//    int tmpcnt=0;
//    for(size_t i=0, iend=vpEdgesNavStatePRV.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStatePRV* e = vpEdgesNavStatePRV[i];
//        if(e->chi2()>21.666)
//        {
//            //e->setLevel(1);
//                        cout<<"1 PRVedge "<<i<<", chi2 "<<e->chi2()<<". ";
//                        tmpcnt++;
//        }
//        //e->setRobustKernel(0);
//    }
//        //cout<<endl<<"edge bias ns bad: ";
//    for(size_t i=0, iend=vpEdgesNavStateBias.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];
//        if(e->chi2()>16.812)
//        {
//            //e->setLevel(1);
//                        cout<<"1 Bias edge "<<i<<", chi2 "<<e->chi2()<<". ";
//                        tmpcnt++;
//        }
//        //e->setRobustKernel(0);
//    }
//    if(tmpcnt>0)
//        cout<<endl;

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    //
    vector<pair<KeyFrame *, MapPoint *> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    double PosePointchi2 = 0;
    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeNavStatePRPointXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }

        PosePointchi2 += e->chi2();
    }

//    // Debug log
//    // Check inlier observations
//    int tmpcnt2=0;
//    for(size_t i=0, iend=vpEdgesNavStatePRV.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStatePRV* e = vpEdgesNavStatePRV[i];
//        if(e->chi2()>21.666)
//        {
//                        cout<<"2 PRVedge "<<i<<", chi2 "<<e->chi2()<<". ";
//                        tmpcnt2++;
//        }
//    }
//        //cout<<endl<<"edge bias ns bad: ";
//    for(size_t i=0, iend=vpEdgesNavStateBias.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];
//        if(e->chi2()>16.812)
//        {
//                        cout<<"2 Bias edge "<<i<<", chi2 "<<e->chi2()<<". ";
//                        tmpcnt2++;
//        }
//    }
//    if(tmpcnt2>0)
//        cout<<endl;

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    //Keyframes
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexNavStatePR *vNSPR = static_cast<g2o::VertexNavStatePR *>(optimizer.vertex(3 * pKFi->mnId));
        g2o::VertexNavStateV *vNSV = static_cast<g2o::VertexNavStateV *>(optimizer.vertex(3 * pKFi->mnId + 1));
        g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(3 * pKFi->mnId + 2));
        // In optimized navstate, bias not changed, delta_bias not zero, should be added to bias
        const NavState &optPRns = vNSPR->estimate();
        const NavState &optVns = vNSV->estimate();
        const NavState &optBiasns = vNSBias->estimate();
        NavState primaryns = pKFi->GetNavState();
        // Update NavState
        pKFi->SetNavStatePos(optPRns.Get_P());
        pKFi->SetNavStateRot(optPRns.Get_R());
        pKFi->SetNavStateVel(optVns.Get_V());
        //if(optBiasns.Get_dBias_Acc().norm()<1e-2 && optBiasns.Get_BiasGyr().norm()<1e-4)
        //{
        pKFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
        pKFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
        //}

        // Update pose Tcw
        pKFi->UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());

        // Test log
        if ((primaryns.Get_BiasGyr() - optPRns.Get_BiasGyr()).norm() > 1e-6 ||
            (primaryns.Get_BiasGyr() - optBiasns.Get_BiasGyr()).norm() > 1e-6)
            cerr << "gyr bias change in optimization?" << endl;
        if ((primaryns.Get_BiasAcc() - optPRns.Get_BiasAcc()).norm() > 1e-6 ||
            (primaryns.Get_BiasAcc() - optBiasns.Get_BiasAcc()).norm() > 1e-6)
            cerr << "acc bias change in optimization?" << endl;

    }

    //Points
    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if (pLM) {
        pLM->SetMapUpdateFlagInTracking(true);
    }

}

void Optimizer::GlobalBundleAdjustmentNavState(Map *pMap, const cv::Mat &gw, int nIterations, bool *pbStopFlag,
                                               const unsigned long nLoopKF, const bool bRobust) {
    vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    vector<MapPoint *> vpMP = pMap->GetAllMapPoints();

    // Extrinsics
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3, 3);
    Vector3d Pbc = Tbc.topRightCorner(3, 1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Set KeyFrame vertices
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;

        // PVR
        g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
        vNSPVR->setEstimate(pKF->GetNavState());
        vNSPVR->setId(pKF->mnId * 2);
        vNSPVR->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vNSPVR);
        // Bias
        g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
        vNSBias->setEstimate(pKF->GetNavState());
        vNSBias->setId(pKF->mnId * 2 + 1);
        vNSBias->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vNSBias);

        if (pKF->mnId * 2 + 1 > maxKFid)
            maxKFid = pKF->mnId * 2 + 1;
    }

    // Add NavState PVR/Bias edges
    const float thHuberNavStatePVR = sqrt(21.666);
    const float thHuberNavStateBias = sqrt(16.812);
    // Inverse covariance of bias random walk
    Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
    InvCovBgaRW.topLeftCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF1 = vpKFs[i];
        if (pKF1->isBad()) {
            cout << "pKF is bad in gBA, id " << pKF1->mnId << endl;   //Debug log
            continue;
        }

        KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();
        if (!pKF0) {
            if (pKF1->mnId != 0) cerr << "Previous KeyFrame is NULL?" << endl;  //Test log
            continue;
        }

        // PVR edge
        {
            g2o::EdgeNavStatePVR *epvr = new g2o::EdgeNavStatePVR();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF0->mnId)));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF1->mnId)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF0->mnId + 1)));
            epvr->setMeasurement(pKF1->GetIMUPreInt());

            Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            epvr->setInformation(InvCovPVR);
            epvr->SetParams(GravityVec);

            if (bRobust) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                epvr->setRobustKernel(rk);
                rk->setDelta(thHuberNavStatePVR);
            }

            optimizer.addEdge(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF0->mnId + 1)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF1->mnId + 1)));
            ebias->setMeasurement(pKF1->GetIMUPreInt());

            ebias->setInformation(InvCovBgaRW / pKF1->GetIMUPreInt().getDeltaTime());

            if (bRobust) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                ebias->setRobustKernel(rk);
                rk->setDelta(thHuberNavStateBias);
            }

            optimizer.addEdge(ebias);
        }

    }

    const float thHuber2D = sqrt(5.99);

    // Set MapPoint vertices
    for (size_t i = 0; i < vpMP.size(); i++) {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit = observations.begin();
             mit != observations.end(); mit++) {

            KeyFrame *pKF = mit->first;
            if (pKF->isBad() || 2 * pKF->mnId > maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if (pKF->mvuRight[mit->second] < 0) {
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeNavStatePVRPointXYZ *e = new g2o::EdgeNavStatePVRPointXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->SetParams(pKF->fx, pKF->fy, pKF->cx, pKF->cy, Rbc, Pbc);

                optimizer.addEdge(e);
            } else {
                cerr << "Stereo not supported" << endl;
            }
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        } else {
            vbNotIncludedMP[i] = false;
        }
    }

    // Optimize!
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        //g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        //g2o::SE3Quat SE3quat = vSE3->estimate();
        g2o::VertexNavStatePVR *vNSPVR = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(2 * pKF->mnId));
        g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(2 * pKF->mnId + 1));
        const NavState &nspvr = vNSPVR->estimate();
        const NavState &nsbias = vNSBias->estimate();
        NavState ns_recov = nspvr;
        ns_recov.Set_DeltaBiasGyr(nsbias.Get_dBias_Gyr());
        ns_recov.Set_DeltaBiasAcc(nsbias.Get_dBias_Acc());

        if (nLoopKF == 0) {
            //pKF->SetPose(Converter::toCvMat(SE3quat));
            pKF->SetNavState(ns_recov);
            pKF->UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());
        } else {
            pKF->mNavStateGBA = ns_recov;

            pKF->mTcwGBA.create(4, 4, CV_32F);
            //Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            cv::Mat Twb_ = cv::Mat::eye(4, 4, CV_32F);
            Converter::toCvMat(pKF->mNavStateGBA.Get_RotMatrix()).copyTo(Twb_.rowRange(0, 3).colRange(0, 3));
            Converter::toCvMat(pKF->mNavStateGBA.Get_P()).copyTo(Twb_.rowRange(0, 3).col(3));
            cv::Mat Twc_ = Twb_ * Config::getInstance().IMUParams().GetMatTbc();
            pKF->mTcwGBA = Converter::toCvMatInverse(Twc_);

            pKF->mnBAGlobalForKF = nLoopKF;
        }

//        //Debug log
//        cv::Mat tTwb1 = pKF->GetPoseInverse()*ConfigParam::GetMatT_cb();
//        if((Converter::toVector3d(tTwb1.rowRange(0,3).col(3))-pKF->GetNavState().Get_P()).norm()>1e-6)
//            cout<<"in gBA, Twc*Tcb != NavState for GBA KFs, id "<<pKF->mnId<<": "<<tTwb1.rowRange(0,3).col(3).t()<<"/"<<pKF->GetNavState().Get_P().transpose()<<endl;

    }

    //Points
    for (size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint *pMP = vpMP[i];

        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0) {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        } else {
            pMP->mPosGBA.create(3, 1, CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

}

int Optimizer::PoseOptimization(Frame *pFrame, Frame *pLastFrame, const IMUPreintegrator &imupreint, const cv::Mat &gw,
                                const bool &bComputeMarg) {
    // Extrinsics
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3, 3);
    Vector3d Pbc = Tbc.topRightCorner(3, 1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    const int FramePVRId = 0;
    const int FrameBiasId = 1;
    const int LastFramePVRId = 2;
    const int LastFrameBiasId = 3;

    // Set Frame vertex PVR/Bias
    g2o::VertexNavStatePVR *vNSFPVR = new g2o::VertexNavStatePVR();
    {
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFPVR->setId(FramePVRId);
        vNSFPVR->setFixed(false);
        optimizer.addVertex(vNSFPVR);
    }
    g2o::VertexNavStateBias *vNSFBias = new g2o::VertexNavStateBias();
    {
        vNSFBias->setEstimate(pFrame->GetNavState());
        vNSFBias->setId(FrameBiasId);
        vNSFBias->setFixed(false);
        optimizer.addVertex(vNSFBias);
    }

    // Set LastFrame vertex
    g2o::VertexNavStatePVR *vNSFPVRlast = new g2o::VertexNavStatePVR();
    {
        vNSFPVRlast->setEstimate(pLastFrame->GetNavState());
        vNSFPVRlast->setId(LastFramePVRId);
        vNSFPVRlast->setFixed(false);
        optimizer.addVertex(vNSFPVRlast);
    }
    g2o::VertexNavStateBias *vNSFBiaslast = new g2o::VertexNavStateBias();
    {
        vNSFBiaslast->setEstimate(pLastFrame->GetNavState());
        vNSFBiaslast->setId(LastFrameBiasId);
        vNSFBiaslast->setFixed(false);
        optimizer.addVertex(vNSFBiaslast);
    }

    // Set prior edge for Last Frame, from mMargCovInv
    g2o::EdgeNavStatePriorPVRBias *eNSPrior = new g2o::EdgeNavStatePriorPVRBias();
    {
        eNSPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFramePVRId)));
        eNSPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFrameBiasId)));
        eNSPrior->setMeasurement(pLastFrame->mNavStatePrior);

        eNSPrior->setInformation(pLastFrame->mMargCovInv);

        const float thHuberNavState = sqrt(30.5779);
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        eNSPrior->setRobustKernel(rk);
        rk->setDelta(thHuberNavState);

        optimizer.addEdge(eNSPrior);
    }

    // Set PVR edge between LastFrame-Frame
    g2o::EdgeNavStatePVR *eNSPVR = new g2o::EdgeNavStatePVR();
    {
        eNSPVR->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFramePVRId)));
        eNSPVR->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
        eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFrameBiasId)));
        eNSPVR->setMeasurement(imupreint);

        Matrix9d InvCovPVR = imupreint.getCovPVPhi().inverse();
        eNSPVR->setInformation(InvCovPVR);

        eNSPVR->SetParams(GravityVec);

        const float thHuberNavStatePVR = sqrt(21.666);
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        eNSPVR->setRobustKernel(rk);
        rk->setDelta(thHuberNavStatePVR);

        optimizer.addEdge(eNSPVR);
    }
    // Set Bias edge between LastFrame-Frame
    g2o::EdgeNavStateBias *eNSBias = new g2o::EdgeNavStateBias();
    {
        eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFrameBiasId)));
        eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FrameBiasId)));
        eNSBias->setMeasurement(imupreint);

        Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
        InvCovBgaRW.topLeftCorner(3, 3) =
                Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
        InvCovBgaRW.bottomRightCorner(3, 3) =
                Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
        eNSBias->setInformation(InvCovBgaRW / imupreint.getDeltaTime());

        const float thHuberNavStateBias = sqrt(16.812);
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        eNSBias->setRobustKernel(rk);
        rk->setDelta(thHuberNavStateBias);

        optimizer.addEdge(eNSBias);
    }

    // Set MapPoint vertices
    const int Ncur = pFrame->N;
    const int Nlast = pLastFrame->N;

    vector<g2o::EdgeNavStatePVRPointXYZOnlyPose *> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(Ncur);
    vnIndexEdgeMono.reserve(Ncur);

    vector<g2o::EdgeNavStatePVRPointXYZOnlyPose *> vpEdgesMonoLast;
    vector<size_t> vnIndexEdgeMonoLast;
    vpEdgesMonoLast.reserve(Nlast);
    vnIndexEdgeMonoLast.reserve(Nlast);


    const float deltaMono = sqrt(5.991);

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < Ncur; i++) {
            MapPoint *pMP = pFrame->mvpMapPoints[i];
            if (pMP) {
                // Monocular observation
                if (pFrame->mvuRight[i] < 0) {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                    g2o::EdgeNavStatePVRPointXYZOnlyPose *e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->SetParams(pFrame->fx, pFrame->fy, pFrame->cx, pFrame->cy, Rbc, Pbc,
                                 Converter::toVector3d(pMP->GetWorldPos()));

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                } else  // Stereo observation
                {
                    cerr << "stereo shouldn't in poseoptimization" << endl;
                }
            }

        }

        // Add Point-Pose edges for last frame
        for (int i = 0; i < Nlast; i++) {
            MapPoint *pMP = pLastFrame->mvpMapPoints[i];
            if (pMP) {
                // Monocular observation
                if (pLastFrame->mvuRight[i] < 0) {
                    //nInitialCorrespondences++;
                    pLastFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    const cv::KeyPoint &kpUn = pLastFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                    g2o::EdgeNavStatePVRPointXYZOnlyPose *e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastFramePVRId)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pLastFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->SetParams(pLastFrame->fx, pLastFrame->fy, pLastFrame->cx, pLastFrame->cy, Rbc, Pbc,
                                 Converter::toVector3d(pMP->GetWorldPos()));

                    optimizer.addEdge(e);

                    vpEdgesMonoLast.push_back(e);
                    vnIndexEdgeMonoLast.push_back(i);
                } else  // Stereo observation
                {
                    cerr << "stereo shouldn't in poseoptimization" << endl;
                }
            }
        }
    }


    if (nInitialCorrespondences < 3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    //const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};

    //    //Debug log
    //    cout<<"total Points: "<<vpEdgesMono.size()<<endl;

    int nBad = 0;
    int nBadLast = 0;
    for (size_t it = 0; it < 4; it++) {
        // Reset estimates
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFBias->setEstimate(pFrame->GetNavState());
        vNSFPVRlast->setEstimate(pLastFrame->GetNavState());
        vNSFBiaslast->setEstimate(pLastFrame->GetNavState());

        //optimizer.setVerbose(true);
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeNavStatePVRPointXYZOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        nBadLast = 0;
        for (size_t i = 0, iend = vpEdgesMonoLast.size(); i < iend; i++) {
            g2o::EdgeNavStatePVRPointXYZOnlyPose *e = vpEdgesMonoLast[i];

            const size_t idx = vnIndexEdgeMonoLast[i];

            if (pLastFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it]) {
                pLastFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBadLast++;
            } else {
                pLastFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        //        //Debug log
        //        cout<<nBad<<" bad Points in iter "<<it<<", rest points: "<<optimizer.edges().size()<<endl;
        //        cout<<nBadLast<<" bad Points of last Frame in iter "<<it<<endl;
        //        cout<<"NavState edge chi2: "<<eNS->chi2()<<endl;

        //if(vpEdgesMono.size() - nBad < 10)
        if (optimizer.edges().size() < 10)
            break;
    }

    // Debug log
    //if(eNSPVR->chi2()>21.666) cout<<"F-F PVR edge chi2:"<<eNSPVR->chi2()<<endl;
    //if(eNSBias->chi2()>16.812) cout<<"F-F Bias edge chi2:"<<eNSBias->chi2()<<endl;
    //if(eNSPrior->chi2()>30.5779) cout<<"F-F Prior edge chi2:"<<eNSPrior->chi2()<<endl;

    // Recover optimized pose and return number of inliers
    g2o::VertexNavStatePVR *vNSPVR_recov = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(FramePVRId));
    const NavState &nsPVR_recov = vNSPVR_recov->estimate();
    g2o::VertexNavStateBias *vNSBias_recov = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(FrameBiasId));
    const NavState &nsBias_recov = vNSBias_recov->estimate();
    NavState ns_recov = nsPVR_recov;
    ns_recov.Set_DeltaBiasGyr(nsBias_recov.Get_dBias_Gyr());
    ns_recov.Set_DeltaBiasAcc(nsBias_recov.Get_dBias_Acc());
    pFrame->SetNavState(ns_recov);
    pFrame->UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());

    // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization
    if (bComputeMarg) {
        std::vector<g2o::OptimizableGraph::Vertex *> margVerteces;
        margVerteces.push_back(optimizer.vertex(FramePVRId));
        margVerteces.push_back(optimizer.vertex(FrameBiasId));

        g2o::SparseBlockMatrixXd spinv;
        optimizer.computeMarginals(spinv, margVerteces);
        // spinv include 2 blocks, 9x9-(0,0) for PVR, 6x6-(1,1) for Bias
        Matrix<double, 15, 15> margCov = Matrix<double, 15, 15>::Zero();
        margCov.topLeftCorner(9, 9) = spinv.block(0, 0)->eval();
        margCov.topRightCorner(9, 6) = spinv.block(0, 1)->eval();
        margCov.bottomLeftCorner(6, 9) = spinv.block(1, 0)->eval();
        margCov.bottomRightCorner(6, 6) = spinv.block(1, 1)->eval();
        pFrame->mMargCovInv = margCov.inverse();
        pFrame->mNavStatePrior = ns_recov;

        //Debug log
        //cout<<"inv MargCov 2: "<<endl<<pFrame->mMargCovInv<<endl;
    }

    //Test log
    if ((nsPVR_recov.Get_BiasGyr() - nsBias_recov.Get_BiasGyr()).norm() > 1e-6 ||
        (nsPVR_recov.Get_BiasAcc() - nsBias_recov.Get_BiasAcc()).norm() > 1e-6) {
        std::cerr << "1 recovered bias gyr not equal for PVR/Bias vertex" << std::endl
                  << nsPVR_recov.Get_BiasGyr().transpose() << " / " << nsBias_recov.Get_BiasGyr().transpose()
                  << std::endl;
        std::cerr << "1 recovered bias acc not equal for PVR/Bias vertex" << std::endl
                  << nsPVR_recov.Get_BiasAcc().transpose() << " / " << nsBias_recov.Get_BiasAcc().transpose()
                  << std::endl;
    }
    if ((ns_recov.Get_dBias_Gyr() - nsBias_recov.Get_dBias_Gyr()).norm() > 1e-6 ||
        (ns_recov.Get_dBias_Acc() - nsBias_recov.Get_dBias_Acc()).norm() > 1e-6) {
        std::cerr << "1 recovered delta bias gyr not equal to Bias vertex" << std::endl
                  << ns_recov.Get_dBias_Gyr().transpose() << " / " << nsBias_recov.Get_dBias_Gyr().transpose()
                  << std::endl;
        std::cerr << "1 recovered delta bias acc not equal to Bias vertex" << std::endl
                  << ns_recov.Get_dBias_Acc().transpose() << " / " << nsBias_recov.Get_dBias_Acc().transpose()
                  << std::endl;
    }

    return nInitialCorrespondences - nBad;
}

int Optimizer::PoseOptimization(Frame *pFrame, KeyFrame *pLastKF, const IMUPreintegrator &imupreint, const cv::Mat &gw,
                                const bool &bComputeMarg) {
    // Extrinsics
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3, 3);
    Vector3d Pbc = Tbc.topRightCorner(3, 1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    optimizer.setAlgorithm(solver);

    int nInitialCorrespondences = 0;

    const int FramePVRId = 0;
    const int FrameBiasId = 1;
    const int LastKFPVRId = 2;
    const int LastKFBiasId = 3;

    // Set Frame vertex PVR/Bias
    g2o::VertexNavStatePVR *vNSFPVR = new g2o::VertexNavStatePVR();
    {
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFPVR->setId(FramePVRId);
        vNSFPVR->setFixed(false);
        optimizer.addVertex(vNSFPVR);
    }
    g2o::VertexNavStateBias *vNSFBias = new g2o::VertexNavStateBias();
    {
        vNSFBias->setEstimate(pFrame->GetNavState());
        vNSFBias->setId(FrameBiasId);
        vNSFBias->setFixed(false);
        optimizer.addVertex(vNSFBias);
    }

    // Set KeyFrame vertex PVR/Bias
    g2o::VertexNavStatePVR *vNSKFPVR = new g2o::VertexNavStatePVR();
    {
        vNSKFPVR->setEstimate(pLastKF->GetNavState());
        vNSKFPVR->setId(LastKFPVRId);
        vNSKFPVR->setFixed(true);
        optimizer.addVertex(vNSKFPVR);
    }
    g2o::VertexNavStateBias *vNSKFBias = new g2o::VertexNavStateBias();
    {
        vNSKFBias->setEstimate(pLastKF->GetNavState());
        vNSKFBias->setId(LastKFBiasId);
        vNSKFBias->setFixed(true);
        optimizer.addVertex(vNSKFBias);
    }

    // Set PVR edge between LastKF-Frame
    g2o::EdgeNavStatePVR *eNSPVR = new g2o::EdgeNavStatePVR();
    {
        eNSPVR->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFPVRId)));
        eNSPVR->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
        eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFBiasId)));
        eNSPVR->setMeasurement(imupreint);

        Matrix9d InvCovPVR = imupreint.getCovPVPhi().inverse();
        eNSPVR->setInformation(InvCovPVR);

        eNSPVR->SetParams(GravityVec);

        const float thHuberNavStatePVR = sqrt(21.666);
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        eNSPVR->setRobustKernel(rk);
        rk->setDelta(thHuberNavStatePVR);

        optimizer.addEdge(eNSPVR);
    }
    // Set Bias edge between LastKF-Frame
    g2o::EdgeNavStateBias *eNSBias = new g2o::EdgeNavStateBias();
    {
        eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(LastKFBiasId)));
        eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FrameBiasId)));
        eNSBias->setMeasurement(imupreint);

        Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
        InvCovBgaRW.topLeftCorner(3, 3) =
                Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
        InvCovBgaRW.bottomRightCorner(3, 3) =
                Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE
        eNSBias->setInformation(InvCovBgaRW / imupreint.getDeltaTime());

        const float thHuberNavStateBias = sqrt(16.812);
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
        eNSBias->setRobustKernel(rk);
        rk->setDelta(thHuberNavStateBias);

        optimizer.addEdge(eNSBias);
    }

    // Set MapPoint vertices
    const int N = pFrame->N;

    vector<g2o::EdgeNavStatePVRPointXYZOnlyPose *> vpEdgesMono;
    vector<size_t> vnIndexEdgeMono;
    vpEdgesMono.reserve(N);
    vnIndexEdgeMono.reserve(N);

    const float deltaMono = sqrt(5.991);

    {
        unique_lock<mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++) {
            MapPoint *pMP = pFrame->mvpMapPoints[i];
            if (pMP) {
                // Monocular observation
                if (pFrame->mvuRight[i] < 0) {
                    nInitialCorrespondences++;
                    pFrame->mvbOutlier[i] = false;

                    Eigen::Matrix<double, 2, 1> obs;
                    const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                    obs << kpUn.pt.x, kpUn.pt.y;

                    //g2o::EdgeSE3ProjectXYZOnlyPose* e = new g2o::EdgeSE3ProjectXYZOnlyPose();
                    g2o::EdgeNavStatePVRPointXYZOnlyPose *e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(FramePVRId)));
                    e->setMeasurement(obs);
                    const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(deltaMono);

                    e->SetParams(pFrame->fx, pFrame->fy, pFrame->cx, pFrame->cy, Rbc, Pbc,
                                 Converter::toVector3d(pMP->GetWorldPos()));

                    optimizer.addEdge(e);

                    vpEdgesMono.push_back(e);
                    vnIndexEdgeMono.push_back(i);
                } else  // Stereo observation
                {
                    cerr << "stereo shouldn't in poseoptimization" << endl;
                }
            }

        }
    }
    //cout<<endl;


    if (nInitialCorrespondences < 3)
        return 0;

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
    //const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};

    //    //Debug log
    //    cout<<"total Points: "<<vpEdgesMono.size()<<endl;

    int nBad = 0;
    for (size_t it = 0; it < 4; it++) {
        // Reset estimate for vertex
        vNSFPVR->setEstimate(pFrame->GetNavState());
        vNSFBias->setEstimate(pFrame->GetNavState());

        optimizer.initializeOptimization(0);
        optimizer.optimize(its[it]);

        nBad = 0;
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeNavStatePVRPointXYZOnlyPose *e = vpEdgesMono[i];

            const size_t idx = vnIndexEdgeMono[i];

            if (pFrame->mvbOutlier[idx]) {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if (chi2 > chi2Mono[it]) {
                pFrame->mvbOutlier[idx] = true;
                e->setLevel(1);
                nBad++;
            } else {
                pFrame->mvbOutlier[idx] = false;
                e->setLevel(0);
            }

            if (it == 2)
                e->setRobustKernel(0);
        }

        //        //Debug log
        //        cout<<nBad<<" bad Points in iter "<<it<<", rest points: "<<optimizer.edges().size()<<endl;

        if (optimizer.edges().size() < 10)
            break;
    }

    // Debug log
    //if(eNSPVR->chi2()>21.666) cout<<"KF-F PVR edge chi2:"<<eNSPVR->chi2()<<endl;
    //if(eNSBias->chi2()>16.812) cout<<"KF-F Bias edge chi2:"<<eNSBias->chi2()<<endl;

    // Recover optimized pose and return number of inliers
    g2o::VertexNavStatePVR *vNSPVR_recov = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(FramePVRId));
    const NavState &nsPVR_recov = vNSPVR_recov->estimate();
    g2o::VertexNavStateBias *vNSBias_recov = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(FrameBiasId));
    const NavState &nsBias_recov = vNSBias_recov->estimate();
    NavState ns_recov = nsPVR_recov;
    ns_recov.Set_DeltaBiasGyr(nsBias_recov.Get_dBias_Gyr());
    ns_recov.Set_DeltaBiasAcc(nsBias_recov.Get_dBias_Acc());
    pFrame->SetNavState(ns_recov);
    pFrame->UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());

    // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization
    if (bComputeMarg) {
        std::vector<g2o::OptimizableGraph::Vertex *> margVerteces;
        margVerteces.push_back(optimizer.vertex(FramePVRId));
        margVerteces.push_back(optimizer.vertex(FrameBiasId));

        //TODO: how to get the joint marginalized covariance of PVR&Bias
        g2o::SparseBlockMatrixXd spinv;
        optimizer.computeMarginals(spinv, margVerteces);
        // spinv include 2 blocks, 9x9-(0,0) for PVR, 6x6-(1,1) for Bias
        Matrix<double, 15, 15> margCovInv = Matrix<double, 15, 15>::Zero();
        margCovInv.topLeftCorner(9, 9) = spinv.block(0, 0)->inverse();
        margCovInv.bottomRightCorner(6, 6) = spinv.block(1, 1)->inverse();
        pFrame->mMargCovInv = margCovInv;
        pFrame->mNavStatePrior = ns_recov;
    }

    //Test log
    if ((nsPVR_recov.Get_BiasGyr() - nsBias_recov.Get_BiasGyr()).norm() > 1e-6 ||
        (nsPVR_recov.Get_BiasAcc() - nsBias_recov.Get_BiasAcc()).norm() > 1e-6) {
        std::cerr << "recovered bias gyr not equal for PVR/Bias vertex" << std::endl
                  << nsPVR_recov.Get_BiasGyr().transpose() << " / " << nsBias_recov.Get_BiasGyr().transpose()
                  << std::endl;
        std::cerr << "recovered bias acc not equal for PVR/Bias vertex" << std::endl
                  << nsPVR_recov.Get_BiasAcc().transpose() << " / " << nsBias_recov.Get_BiasAcc().transpose()
                  << std::endl;
    }
    if ((ns_recov.Get_dBias_Gyr() - nsBias_recov.Get_dBias_Gyr()).norm() > 1e-6 ||
        (ns_recov.Get_dBias_Acc() - nsBias_recov.Get_dBias_Acc()).norm() > 1e-6) {
        std::cerr << "recovered delta bias gyr not equal to Bias vertex" << std::endl
                  << ns_recov.Get_dBias_Gyr().transpose() << " / " << nsBias_recov.Get_dBias_Gyr().transpose()
                  << std::endl;
        std::cerr << "recovered delta bias acc not equal to Bias vertex" << std::endl
                  << ns_recov.Get_dBias_Acc().transpose() << " / " << nsBias_recov.Get_dBias_Acc().transpose()
                  << std::endl;
    }

    return nInitialCorrespondences - nBad;
}

void Optimizer::LocalBundleAdjustmentNavState(KeyFrame *pCurKF, const std::list<KeyFrame *> &lLocalKeyFrames,
                                              bool *pbStopFlag, Map *pMap, cv::Mat &gw, LocalMapping *pLM) {
    // Check current KeyFrame in local window
    if (pCurKF != lLocalKeyFrames.back())
        cerr << "pCurKF != lLocalKeyFrames.back. check" << endl;

    // Extrinsics
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rbc = Tbc.topLeftCorner(3, 3);
    Vector3d Pbc = Tbc.topRightCorner(3, 1);
    // Gravity vector in world frame
    Vector3d GravityVec = Converter::toVector3d(gw);

    // All KeyFrames in Local window are optimized
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        pKFi->mnBALocalForKF = pCurKF->mnId;
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pCurKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pCurKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    // Add the KeyFrame before local window.
    KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
    if (pKFPrevLocal) {
        // Test log
        if (pKFPrevLocal->isBad()) cerr << "KeyFrame before local window is bad?" << endl;
        if (pKFPrevLocal->mnBAFixedForKF == pCurKF->mnId)
            cerr << "KeyFrame before local, has been added to lFixedKF?" << endl;
        if (pKFPrevLocal->mnBALocalForKF == pCurKF->mnId)
            cerr << "KeyFrame before local, has been added to lLocalKF?" << endl;

        pKFPrevLocal->mnBAFixedForKF = pCurKF->mnId;
        if (!pKFPrevLocal->isBad())
            lFixedCameras.push_back(pKFPrevLocal);
        else
            cerr << "pKFPrevLocal is Bad?" << endl;
    }
        // Test log
    else { cerr << "pKFPrevLocal is NULL?" << endl; }
    // Covisible KeyFrames
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pCurKF->mnId && pKFi->mnBAFixedForKF != pCurKF->mnId) {
                pKFi->mnBAFixedForKF = pCurKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    int maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        int idKF = pKFi->mnId * 2;
        // Vertex of PVR
        {
            g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
            vNSPVR->setEstimate(pKFi->GetNavState());
            vNSPVR->setId(idKF);
            vNSPVR->setFixed(false);
            optimizer.addVertex(vNSPVR);
        }
        // Vertex of Bias
        {
            g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF + 1);
            vNSBias->setFixed(false);
            optimizer.addVertex(vNSBias);
        }

        if (idKF + 1 > maxKFid)
            maxKFid = idKF + 1;
        // Test log
        if (pKFi->mnId == 0) cerr << "pKFi->mnId == 0, shouldn't in LocalBA of NavState" << endl;
    }

    // Set Fixed KeyFrame vertices. Including the pKFPrevLocal.
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        int idKF = pKFi->mnId * 2;
        // For common fixed KeyFrames, only add PVR vertex
        {
            g2o::VertexNavStatePVR *vNSPVR = new g2o::VertexNavStatePVR();
            vNSPVR->setEstimate(pKFi->GetNavState());
            vNSPVR->setId(idKF);
            vNSPVR->setFixed(true);
            optimizer.addVertex(vNSPVR);
        }
        // For Local-Window-Previous KeyFrame, add Bias vertex
        if (pKFi == pKFPrevLocal) {
            g2o::VertexNavStateBias *vNSBias = new g2o::VertexNavStateBias();
            vNSBias->setEstimate(pKFi->GetNavState());
            vNSBias->setId(idKF + 1);
            vNSBias->setFixed(true);
            optimizer.addVertex(vNSBias);
        }

        if (idKF + 1 > maxKFid)
            maxKFid = idKF + 1;
    }

    // Edges between KeyFrames in Local Window
    // and
    // Edges between 1st KeyFrame of Local Window and its previous (fixed)KeyFrame - pKFPrevLocal
    vector<g2o::EdgeNavStatePVR *> vpEdgesNavStatePVR;
    vector<g2o::EdgeNavStateBias *> vpEdgesNavStateBias;
    // Use chi2inv() in MATLAB to compute the value corresponding to 0.95/0.99 prob. w.r.t 15DOF: 24.9958/30.5779
    // 12.592/16.812 for 0.95/0.99 6DoF
    // 16.919/21.666 for 0.95/0.99 9DoF
    //const float thHuberNavState = sqrt(30.5779);
    const float thHuberNavStatePVR = sqrt(100 * 21.666);
    const float thHuberNavStateBias = sqrt(100 * 16.812);
    // Inverse covariance of bias random walk
    Matrix<double, 6, 6> InvCovBgaRW = Matrix<double, 6, 6>::Identity();
    InvCovBgaRW.topLeftCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getGyrBiasRW2();       // Gyroscope bias random walk, covariance INVERSE
    InvCovBgaRW.bottomRightCorner(3, 3) =
            Matrix3d::Identity() / IMUData::getAccBiasRW2();   // Accelerometer bias random walk, covariance INVERSE

    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKF1 = *lit;                      // Current KF, store the IMU pre-integration between previous-current
        KeyFrame *pKF0 = pKF1->GetPrevKeyFrame();   // Previous KF

        // PVR edge
        {
            g2o::EdgeNavStatePVR *epvr = new g2o::EdgeNavStatePVR();
            epvr->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF0->mnId)));
            epvr->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF1->mnId)));
            epvr->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF0->mnId + 1)));
            epvr->setMeasurement(pKF1->GetIMUPreInt());

            Matrix9d InvCovPVR = pKF1->GetIMUPreInt().getCovPVPhi().inverse();
            epvr->setInformation(InvCovPVR);
            epvr->SetParams(GravityVec);

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            epvr->setRobustKernel(rk);
            rk->setDelta(thHuberNavStatePVR);

            optimizer.addEdge(epvr);
            vpEdgesNavStatePVR.push_back(epvr);
        }
        // Bias edge
        {
            g2o::EdgeNavStateBias *ebias = new g2o::EdgeNavStateBias();
            ebias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF0->mnId + 1)));
            ebias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKF1->mnId + 1)));
            ebias->setMeasurement(pKF1->GetIMUPreInt());

            ebias->setInformation(InvCovBgaRW / pKF1->GetIMUPreInt().getDeltaTime());

            g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
            ebias->setRobustKernel(rk);
            rk->setDelta(thHuberNavStateBias);

            optimizer.addEdge(ebias);
            vpEdgesNavStateBias.push_back(ebias);
        }

        // Test log
        if (pKF1->GetIMUPreInt().getDeltaTime() < 1e-3) {
            cerr << "IMU pre-integrator delta time between 2 KFs too small: " << pKF1->GetIMUPreInt().getDeltaTime()
                 << endl;
            cerr << "No EdgeNavState added" << endl;
            continue;
        }
        if (lit == lLocalKeyFrames.begin()) {
            // First KF in Local Window, link (fixed) pKFPrevLocal
            if (pKF0 != pKFPrevLocal)
                cerr << "pKF0 != pKFPrevLocal for 1st KF in Local Window, id: " << pKF0->mnId << ","
                     << pKFPrevLocal->mnId << endl;
        } else {
            // KFs in Local Window, link another local KF
        }
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeNavStatePVRPointXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        cv::Mat Pw = pMP->GetWorldPos();

        const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();

        // Set edges between KeyFrame and MapPoint
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad()) {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0) {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeNavStatePVRPointXYZ *e = new g2o::EdgeNavStatePVRPointXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(2 * pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->SetParams(pKFi->fx, pKFi->fy, pKFi->cx, pKFi->cy, Rbc, Pbc);

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                } else {
                    // Test log
                    cerr << "Stereo not supported yet, why here?? check." << endl;
                }
            }
        }
    }

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    // First try
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore) {
        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeNavStatePVRPointXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

//    // Check inlier observations
//    for(size_t i=0, iend=vpEdgesNavStatePVR.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStatePVR* e = vpEdgesNavStatePVR[i];
//        if(e->chi2()>21.666)
//        {
//            //e->setLevel(1);
//            //cout<<"1 PVRedge "<<i<<", chi2 "<<e->chi2()<<". ";
//        }
//        //e->setRobustKernel(0);
//    }
//    for(size_t i=0, iend=vpEdgesNavStateBias.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];
//        if(e->chi2()>16.812)
//        {
//            //e->setLevel(1);
//            //cout<<"1 Bias edge "<<i<<", chi2 "<<e->chi2()<<". ";
//        }
//        //e->setRobustKernel(0);
//    }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    //
    vector<pair<KeyFrame *, MapPoint *> > vToErase;
    vToErase.reserve(vpEdgesMono.size());

    double PosePointchi2 = 0;
    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeNavStatePVRPointXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }

        PosePointchi2 += e->chi2();
    }

//    // Debug log
//    // Check inlier observations
//    for(size_t i=0, iend=vpEdgesNavStatePVR.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStatePVR* e = vpEdgesNavStatePVR[i];
//        if(e->chi2()>21.666)
//        {
//            //cout<<"2 PVRedge "<<i<<", chi2 "<<e->chi2()<<". ";
//        }
//    }
//    for(size_t i=0, iend=vpEdgesNavStateBias.size(); i<iend; i++)
//    {
//        g2o::EdgeNavStateBias* e = vpEdgesNavStateBias[i];
//        if(e->chi2()>16.812)
//        {
//            //cout<<"2 Bias edge "<<i<<", chi2 "<<e->chi2()<<". ";
//        }
//    }
//    //cout<<"pose-point chi2: "<<PosePointchi2<<", pose-pose chi2: "<<PosePosechi2<<endl;

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    //Keyframes
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexNavStatePVR *vNSPVR = static_cast<g2o::VertexNavStatePVR *>(optimizer.vertex(2 * pKFi->mnId));
        g2o::VertexNavStateBias *vNSBias = static_cast<g2o::VertexNavStateBias *>(optimizer.vertex(2 * pKFi->mnId + 1));
        // In optimized navstate, bias not changed, delta_bias not zero, should be added to bias
        const NavState &optPVRns = vNSPVR->estimate();
        const NavState &optBiasns = vNSBias->estimate();
        NavState primaryns = pKFi->GetNavState();
        // Update NavState
        pKFi->SetNavStatePos(optPVRns.Get_P());
        pKFi->SetNavStateVel(optPVRns.Get_V());
        pKFi->SetNavStateRot(optPVRns.Get_R());
        //if(optBiasns.Get_dBias_Acc().norm()<1e-2 && optBiasns.Get_BiasGyr().norm()<1e-4)
        //{
        pKFi->SetNavStateDeltaBg(optBiasns.Get_dBias_Gyr());
        pKFi->SetNavStateDeltaBa(optBiasns.Get_dBias_Acc());
        //}

        // Update pose Tcw
        pKFi->UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());

        // Test log
        if ((primaryns.Get_BiasGyr() - optPVRns.Get_BiasGyr()).norm() > 1e-6 ||
            (primaryns.Get_BiasGyr() - optBiasns.Get_BiasGyr()).norm() > 1e-6)
            cerr << "gyr bias change in optimization?" << endl;
        if ((primaryns.Get_BiasAcc() - optPVRns.Get_BiasAcc()).norm() > 1e-6 ||
            (primaryns.Get_BiasAcc() - optBiasns.Get_BiasAcc()).norm() > 1e-6)
            cerr << "acc bias change in optimization?" << endl;
        // Debug log
        //cout<<"updated delta bias gyr: "<<optns.Get_dBias_Gyr().transpose()<<endl;
        //cout<<"updated delta bias acc: "<<optns.Get_dBias_Acc().transpose()<<endl;
        //cout<<"before and after opt, navstate.P: "<<primaryns.Get_P().transpose()<<" vs "<<optns.Get_P().transpose()<<endl;
        //cout<<"before and after opt, navstate.V: "<<primaryns.Get_V().transpose()<<" vs "<<optns.Get_V().transpose()<<endl;
        //cout<<"before and after opt, navstate.R: "<<primaryns.Get_RotMatrix()<<" vs "<<optns.Get_RotMatrix()<<endl;

    }

    //Points
    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if (pLM) {
        pLM->SetMapUpdateFlagInTracking(true);
    }

}

Vector3d Optimizer::OptimizeInitialGyroBias(const utils::eigen_aligned_vector<Frame> &vFrames) {
    //size_t N = vpKFs.size();
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rcb = Tbc.topLeftCorner(3, 3).transpose();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Add vertex of gyro bias, to optimizer graph
    g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias();
    vBiasg->setEstimate(Eigen::Vector3d::Zero());
    vBiasg->setId(0);
    optimizer.addVertex(vBiasg);

    // Add unary edges for gyro bias vertex
    for (size_t i = 0; i < vFrames.size(); i++) {
        // Only 19 edges between 20 Frames
        if (i == 0)
            continue;

        const Frame &Fi = vFrames[i - 1];
        const Frame &Fj = vFrames[i];

        cv::Mat Tiw = Fi.mTcw;      // pose of previous KF
        Eigen::Matrix3d Rwci = Converter::toMatrix3d(Tiw.rowRange(0, 3).colRange(0, 3).t());
        cv::Mat Tjw = Fj.mTcw;      // pose of this KF
        Eigen::Matrix3d Rwcj = Converter::toMatrix3d(Tjw.rowRange(0, 3).colRange(0, 3).t());

        //
        IMUPreintegrator imupreint;
        Fj.ComputeIMUPreIntSinceLastFrame(&Fi, imupreint);

        g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
        eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        // measurement is not used in EdgeGyrBias
        eBiasg->dRbij = imupreint.getDeltaR();
        eBiasg->J_dR_bg = imupreint.getJRBiasg();
        eBiasg->Rwbi = Rwci * Rcb;
        eBiasg->Rwbj = Rwcj * Rcb;
        eBiasg->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(eBiasg);
    }

    // It's actualy a linear estimator, so 1 iteration is enough.
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));

    return vBgEst->estimate();
}

Vector3d Optimizer::OptimizeInitialGyroBias(const std::list<KeyFrame *> &lLocalKeyFrames) {
    return OptimizeInitialGyroBias(std::vector<KeyFrame *>(lLocalKeyFrames.begin(), lLocalKeyFrames.end()));
}

Vector3d Optimizer::OptimizeInitialGyroBias(const std::vector<KeyFrame *> &vpKFs) {
    //size_t N = vpKFs.size();
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rcb = Tbc.topLeftCorner(3, 3).transpose();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Add vertex of gyro bias, to optimizer graph
    g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias();
    vBiasg->setEstimate(Eigen::Vector3d::Zero());
    vBiasg->setId(0);
    optimizer.addVertex(vBiasg);

    // Add unary edges for gyro bias vertex
    KeyFrame *pPrevKF0 = vpKFs.front();
    for (std::vector<KeyFrame *>::const_iterator lit = vpKFs.begin(), lend = vpKFs.end(); lit != lend; lit++) {
        KeyFrame *pKF = *lit;
        // Ignore the first KF
        if (pKF == vpKFs.front())
            continue;

        KeyFrame *pPrevKF = pKF->GetPrevKeyFrame();
        cv::Mat Twi = pPrevKF->GetPoseInverse();    // pose of previous KF
        Eigen::Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0, 3).colRange(0, 3));
        cv::Mat Twj = pKF->GetPoseInverse();        // pose of this KF
        Eigen::Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0, 3).colRange(0, 3));

        //
        const IMUPreintegrator &imupreint = pKF->GetIMUPreInt();
        g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
        eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        // measurement is not used in EdgeGyrBias
        eBiasg->dRbij = imupreint.getDeltaR();
        eBiasg->J_dR_bg = imupreint.getJRBiasg();
        eBiasg->Rwbi = Rwci * Rcb;
        eBiasg->Rwbj = Rwcj * Rcb;
        eBiasg->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(eBiasg);

        // Test log
        if (pPrevKF0 != pPrevKF) cerr << "pPrevKF in list != pKF->pPrevKF? in OptimizeInitialGyroBias" << endl;
        pPrevKF0 = pKF;

        // Debug log
        //cout<<"dRbij in pre-int: "<<endl<<eBiasg->dRbij<<endl;
        //cout<<"Rwbi'*Rwbj by ORBSLAM: "<<endl<<eBiasg->Rwbi.transpose()*eBiasg->Rwbj<<endl;
    }

    // It's actualy a linear estimator, so 1 iteration is enough.
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));

    return vBgEst->estimate();
}

//Vector3d Optimizer::OptimizeInitialGyroBias(const vector<cv::Mat> &vTwc, const vector<IMUPreintegrator> &vImuPreInt) {
Vector3d Optimizer::OptimizeInitialGyroBias(const vector<cv::Mat> &vTwc, const utils::eigen_aligned_vector<IMUPreintegrator> &vImuPreInt) {
    int N = vTwc.size();
    if (vTwc.size() != vImuPreInt.size()) cerr << "vTwc.size()!=vImuPreInt.size()" << endl;
    Matrix4d Tbc = Config::getInstance().IMUParams().GetEigTbc();
    Matrix3d Rcb = Tbc.topLeftCorner(3, 3).transpose();

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    //g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    // Add vertex of gyro bias, to optimizer graph
    g2o::VertexGyrBias *vBiasg = new g2o::VertexGyrBias();
    vBiasg->setEstimate(Eigen::Vector3d::Zero());
    vBiasg->setId(0);
    optimizer.addVertex(vBiasg);

    // Add unary edges for gyro bias vertex
    //for(std::vector<KeyFrame*>::const_iterator lit=vpKFs.begin(), lend=vpKFs.end(); lit!=lend; lit++)
    for (int i = 0; i < N; i++) {
        // Ignore the first KF
        if (i == 0)
            continue;

        const cv::Mat &Twi = vTwc[i - 1];    // pose of previous KF
        Matrix3d Rwci = Converter::toMatrix3d(Twi.rowRange(0, 3).colRange(0, 3));
        //Matrix3d Rwci = Twi.rotation_matrix();
        const cv::Mat &Twj = vTwc[i];        // pose of this KF
        Matrix3d Rwcj = Converter::toMatrix3d(Twj.rowRange(0, 3).colRange(0, 3));
        //Matrix3d Rwcj =Twj.rotation_matrix();

        const IMUPreintegrator &imupreint = vImuPreInt[i];

        g2o::EdgeGyrBias *eBiasg = new g2o::EdgeGyrBias();
        eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
        // measurement is not used in EdgeGyrBias
        eBiasg->dRbij = imupreint.getDeltaR();
        eBiasg->J_dR_bg = imupreint.getJRBiasg();
        eBiasg->Rwbi = Rwci * Rcb;
        eBiasg->Rwbj = Rwcj * Rcb;
        //eBiasg->setInformation(Eigen::Matrix3d::Identity());
        eBiasg->setInformation(imupreint.getCovPVPhi().bottomRightCorner(3, 3).inverse());
        optimizer.addEdge(eBiasg);
    }

    // It's actualy a linear estimator, so 1 iteration is enough.
    //optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(1);

    g2o::VertexGyrBias *vBgEst = static_cast<g2o::VertexGyrBias *>(optimizer.vertex(0));

    return vBgEst->estimate();
}

void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag,
                                      Map *pMap, LocalMapping *pLM) {
    // Check current KeyFrame in local window
    if (pKF != lLocalKeyFrames.back())
        cerr << "pKF != lLocalKeyFrames.back. check" << endl;


    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        pKFi->mnBALocalForKF = pKF->mnId;
    }

    // Local MapPoints seen in Local KeyFrames
    list<MapPoint *> lLocalMapPoints;
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++) {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId) {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame *> lFixedCameras;
    // Add the KeyFrame before local window
    KeyFrame *pKFPrevLocal = lLocalKeyFrames.front()->GetPrevKeyFrame();
    if (pKFPrevLocal) {
        // Test log
        if (pKFPrevLocal->isBad()) cerr << "KeyFrame before local window is bad?" << endl;
        if (pKFPrevLocal->mnBAFixedForKF == pKF->mnId)
            cerr << "KeyFrame before local, has been added to lFixedKF?" << endl;
        if (pKFPrevLocal->mnBALocalForKF == pKF->mnId)
            cerr << "KeyFrame before local, has been added to lLocalKF?" << endl;

        pKFPrevLocal->mnBAFixedForKF = pKF->mnId;
        if (!pKFPrevLocal->isBad())
            lFixedCameras.push_back(pKFPrevLocal);
        else
            cerr << "pKFPrevLocal is Bad?" << endl;
    }
    // Covisible KeyFrames
    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = (*lit)->GetObservations();
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;

            if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId) {
                pKFi->mnBAFixedForKF = pKF->mnId;
                if (!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Set Local KeyFrame vertices
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++) {
        KeyFrame *pKFi = *lit;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set MapPoint vertices
    const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);

    vector<KeyFrame *> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);

    vector<MapPoint *> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);

    for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const mapMapPointObs/*map<KeyFrame*,size_t>*/ observations = pMP->GetObservations();

        //Set edges
        for (mapMapPointObs/*map<KeyFrame*,size_t>*/::const_iterator mit = observations.begin(), mend = observations.end();
             mit != mend; mit++) {
            KeyFrame *pKFi = mit->first;

            if (!pKFi->isBad()) {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if (pKFi->mvuRight[mit->second] < 0) {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);
                } else // Stereo observation
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
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

    if (pbStopFlag)
        if (*pbStopFlag)
            return;

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore = true;

    if (pbStopFlag)
        if (*pbStopFlag)
            bDoMore = false;

    if (bDoMore) {

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive()) {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    vector<pair<KeyFrame *, MapPoint *> > vToErase;
    vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

    // Check inlier observations
    for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++) {
        g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
        MapPoint *pMP = vpMapPointEdgeMono[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 5.991 || !e->isDepthPositive()) {
            KeyFrame *pKFi = vpEdgeKFMono[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++) {
        g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
        MapPoint *pMP = vpMapPointEdgeStereo[i];

        if (pMP->isBad())
            continue;

        if (e->chi2() > 7.815 || !e->isDepthPositive()) {
            KeyFrame *pKFi = vpEdgeKFStereo[i];
            vToErase.push_back(make_pair(pKFi, pMP));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if (!vToErase.empty()) {
        for (size_t i = 0; i < vToErase.size(); i++) {
            KeyFrame *pKFi = vToErase[i].first;
            MapPoint *pMPi = vToErase[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for (list<KeyFrame *>::const_iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end();
         lit != lend; lit++) {
        KeyFrame *pKF = *lit;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for (list<MapPoint *>::const_iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end();
         lit != lend; lit++) {
        MapPoint *pMP = *lit;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMP->mnId + maxKFid + 1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    if (pLM) {
        pLM->SetMapUpdateFlagInTracking(true);
    }
}

}


//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------