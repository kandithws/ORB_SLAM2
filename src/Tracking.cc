/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include "Map.h"
#include "Initializer.h"

#include "Optimizer.h"
#include "PnPsolver.h"

#include <iostream>

#include <mutex>
#include <boost/filesystem.hpp>

#define TRACK_WITH_IMU

using namespace std;

namespace ORB_SLAM2 {
//-------------------------------------------------------------------------------------------
//-------------------------------FROM LEARN VIORB----------------------------------------------------------
//-------------------------------------------------------------------------------------------

void Tracking::RecomputeIMUBiasAndCurrentNavstate(NavState &nscur) {
    size_t N = mv20FramesReloc.size();

    //Test log
    if (N != 20)
        cerr << "Frame vector size not 20 to compute bias after reloc??? size: " << mv20FramesReloc.size() << endl;

    // Estimate gyr bias
    Vector3d bg = Optimizer::OptimizeInitialGyroBias(mv20FramesReloc);
    // Update gyr bias of Frames
    for (size_t i = 0; i < N; i++) {
        Frame &frame = mv20FramesReloc[i];
        //Test log
        if (frame.GetNavState().Get_BiasGyr().norm() != 0 || frame.GetNavState().Get_dBias_Gyr().norm() != 0)
            cerr << "Frame " << frame.mnId << " gyr bias or delta bias not zero???" << endl;

        frame.SetNavStateBiasGyr(bg);
    }
    // Re-compute IMU pre-integration
    vector<IMUPreintegrator> v19IMUPreint;
    v19IMUPreint.reserve(20 - 1);
    for (size_t i = 0; i < N; i++) {
        if (i == 0)
            continue;

        const Frame &Fi = mv20FramesReloc[i - 1];
        const Frame &Fj = mv20FramesReloc[i];

        IMUPreintegrator imupreint;
        Fj.ComputeIMUPreIntSinceLastFrame(&Fi, imupreint);
        v19IMUPreint.push_back(imupreint);
    }
    // Construct [A1;A2;...;AN] * ba = [B1;B2;.../BN], solve ba
    cv::Mat A = cv::Mat::zeros(3 * (N - 2), 3, CV_32F);
    cv::Mat B = cv::Mat::zeros(3 * (N - 2), 1, CV_32F);
    const cv::Mat &gw = mpLocalMapper->GetGravityVec();
    const cv::Mat &Tcb = Config::getInstance().IMUParams().GetMatTcb();
    for (int i = 0; i < N - 2; i++) {
        const Frame &F1 = mv20FramesReloc[i];
        const Frame &F2 = mv20FramesReloc[i + 1];
        const Frame &F3 = mv20FramesReloc[i + 2];
        const IMUPreintegrator &PreInt12 = v19IMUPreint[i];
        const IMUPreintegrator &PreInt23 = v19IMUPreint[i + 1];
        // Delta time between frames
        double dt12 = PreInt12.getDeltaTime();
        double dt23 = PreInt23.getDeltaTime();
        // Pre-integrated measurements
        cv::Mat dp12 = Converter::toCvMat(PreInt12.getDeltaP());
        cv::Mat dv12 = Converter::toCvMat(PreInt12.getDeltaV());
        cv::Mat dp23 = Converter::toCvMat(PreInt23.getDeltaP());
        cv::Mat Jpba12 = Converter::toCvMat(PreInt12.getJPBiasa());
        cv::Mat Jvba12 = Converter::toCvMat(PreInt12.getJVBiasa());
        cv::Mat Jpba23 = Converter::toCvMat(PreInt23.getJPBiasa());
        // Pose of body in world frame
        cv::Mat Twb1 = Converter::toCvMatInverse(F1.mTcw) * Tcb;
        cv::Mat Twb2 = Converter::toCvMatInverse(F2.mTcw) * Tcb;
        cv::Mat Twb3 = Converter::toCvMatInverse(F3.mTcw) * Tcb;
        // Position of body, Pwb
        cv::Mat pb1 = Twb1.rowRange(0, 3).col(3);
        cv::Mat pb2 = Twb2.rowRange(0, 3).col(3);
        cv::Mat pb3 = Twb3.rowRange(0, 3).col(3);
        // Rotation of body, Rwb
        cv::Mat Rb1 = Twb1.rowRange(0, 3).colRange(0, 3);
        cv::Mat Rb2 = Twb2.rowRange(0, 3).colRange(0, 3);
        //cv::Mat Rb3 = Twb3.rowRange(0,3).colRange(0,3);
        // Stack to A/B matrix
        // Ai * ba = Bi
        cv::Mat Ai = Rb1 * Jpba12 * dt23 - Rb2 * Jpba23 * dt12 - Rb1 * Jvba12 * dt12 * dt23;
        cv::Mat Bi = (pb2 - pb3) * dt12 + (pb2 - pb1) * dt23 + Rb2 * dp23 * dt12 - Rb1 * dp12 * dt23 +
                     Rb1 * dv12 * dt12 * dt23 + 0.5 * gw * (dt12 * dt12 * dt23 + dt12 * dt23 * dt23);
        Ai.copyTo(A.rowRange(3 * i + 0, 3 * i + 3));
        Bi.copyTo(B.rowRange(3 * i + 0, 3 * i + 3));

        //Test log
        if (fabs(F2.mTimeStamp - F1.mTimeStamp - dt12) > 1e-6 || fabs(F3.mTimeStamp - F2.mTimeStamp - dt23) > 1e-6)
            cerr << "delta time not right." << endl;

        //        // lambda*s + phi*dthetaxy + zeta*ba = psi
        //        cv::Mat lambda = (pc2-pc1)*dt23 + (pc2-pc3)*dt12;
        //        cv::Mat phi = - 0.5*(dt12*dt12*dt23 + dt12*dt23*dt23)*Rwi*SkewSymmetricMatrix(GI);  // note: this has a '-', different to paper
        //        cv::Mat zeta = Rc2*Rcb*Jpba23*dt12 + Rc1*Rcb*Jvba12*dt12*dt23 - Rc1*Rcb*Jpba12*dt23;
        //        cv::Mat psi = (Rc1-Rc2)*pcb*dt23 + Rc1*Rcb*dp12*dt23 - (Rc2-Rc3)*pcb*dt12
        //                     - Rc2*Rcb*dp23*dt12 - Rc1*Rcb*dv12*dt23*dt12 - 0.5*Rwi*GI*(dt12*dt12*dt23 + dt12*dt23*dt23); // note:  - paper
        //        lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
        //        phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3)); //only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
        //        zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
        //        psi.copyTo(D.rowRange(3*i+0,3*i+3));

    }

    // Use svd to compute A*x=B, x=ba 3x1 vector
    // A = u*w*vt, u*w*vt*x=B
    // Then x = vt'*winv*u'*B
    cv::Mat w2, u2, vt2;
    // Note w2 is 3x1 vector by SVDecomp()
    // A is changed in SVDecomp() with cv::SVD::MODIFY_A for speed
    cv::SVDecomp(A, w2, u2, vt2, cv::SVD::MODIFY_A);
    // Compute winv
    cv::Mat w2inv = cv::Mat::eye(3, 3, CV_32F);
    for (int i = 0; i < 3; i++) {
        if (fabs(w2.at<float>(i)) < 1e-10) {
            w2.at<float>(i) += 1e-10;
            // Test log
            cerr << "w2(i) < 1e-10, w=" << endl << w2 << endl;
        }
        w2inv.at<float>(i, i) = 1. / w2.at<float>(i);
    }
    // Then y = vt'*winv*u'*B
    cv::Mat ba_cv = vt2.t() * w2inv * u2.t() * B;
    Vector3d ba = Converter::toVector3d(ba_cv);

    // Update acc bias
    for (size_t i = 0; i < N; i++) {
        Frame &frame = mv20FramesReloc[i];
        //Test log
        if (frame.GetNavState().Get_BiasAcc().norm() != 0 || frame.GetNavState().Get_dBias_Gyr().norm() != 0 ||
            frame.GetNavState().Get_dBias_Acc().norm() != 0)
            cerr << "Frame " << frame.mnId << " acc bias or delta bias not zero???" << endl;

        frame.SetNavStateBiasAcc(ba);
    }

    // Compute Velocity of the last 2 Frames
    Vector3d Pcur;
    Vector3d Vcur;
    Matrix3d Rcur;
    {
        Frame &F1 = mv20FramesReloc[N - 2];
        Frame &F2 = mv20FramesReloc[N - 1];
        const IMUPreintegrator &imupreint = v19IMUPreint.back();
        const double dt12 = imupreint.getDeltaTime();
        const Vector3d dp12 = imupreint.getDeltaP();
        const Vector3d gweig = Converter::toVector3d(gw);
        const Matrix3d Jpba12 = imupreint.getJPBiasa();
        const Vector3d dv12 = imupreint.getDeltaV();
        const Matrix3d Jvba12 = imupreint.getJVBiasa();

        // Velocity of Previous Frame
        // P2 = P1 + V1*dt12 + 0.5*gw*dt12*dt12 + R1*(dP12 + Jpba*ba + Jpbg*0)
        cv::Mat Twb1 = Converter::toCvMatInverse(F1.mTcw) * Tcb;
        cv::Mat Twb2 = Converter::toCvMatInverse(F2.mTcw) * Tcb;
        Vector3d P1 = Converter::toVector3d(Twb1.rowRange(0, 3).col(3));
        /*Vector3d */Pcur = Converter::toVector3d(Twb2.rowRange(0, 3).col(3));
        Matrix3d R1 = Converter::toMatrix3d(Twb1.rowRange(0, 3).colRange(0, 3));
        /*Matrix3d */Rcur = Converter::toMatrix3d(Twb2.rowRange(0, 3).colRange(0, 3));
        Vector3d V1 = 1. / dt12 * (Pcur - P1 - 0.5 * gweig * dt12 * dt12 - R1 * (dp12 + Jpba12 * ba));

        // Velocity of Current Frame
        Vcur = V1 + gweig * dt12 + R1 * (dv12 + Jvba12 * ba);

        // Test log
        if (F2.mnId != mCurrentFrame.mnId) cerr << "framecur.mnId != mCurrentFrame.mnId. why??" << endl;
        if (fabs(F2.mTimeStamp - F1.mTimeStamp - dt12) > 1e-6)
            cerr << "timestamp not right?? in compute vel" << endl;
    }

    // Set NavState of Current Frame, P/V/R/bg/ba/dbg/dba
    nscur.Set_Pos(Pcur);
    nscur.Set_Vel(Vcur);
    nscur.Set_Rot(Rcur);
    nscur.Set_BiasGyr(bg);
    nscur.Set_BiasAcc(ba);
    nscur.Set_DeltaBiasGyr(Vector3d::Zero());
    nscur.Set_DeltaBiasAcc(Vector3d::Zero());

    //mv20FramesReloc
}

bool Tracking::TrackLocalMapWithIMU(bool bMapUpdated) {
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Map updated, optimize with last KeyFrame
    if (mpLocalMapper->GetFirstVINSInited() || bMapUpdated) {
        // Get initial pose from Last KeyFrame
        IMUPreintegrator imupreint = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF);

        // Test log
        //if(mpLocalMapper->GetFirstVINSInited() && !bMapUpdated) cerr<<"1-FirstVinsInit, but not bMapUpdated. shouldn't"<<endl;
        if (mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6)
            cerr << "TrackLocalMapWithIMU current Frame dBias acc not zero" << endl;
        if (mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6)
            cerr << "TrackLocalMapWithIMU current Frame dBias gyr not zero" << endl;

        //
        Optimizer::PoseOptimization(&mCurrentFrame, mpLastKeyFrame, imupreint, mpLocalMapper->GetGravityVec(),
                                    true);
    }
        // Map not updated, optimize with last Frame
    else {
        // Get initial pose from Last Frame
        IMUPreintegrator imupreint = GetIMUPreIntSinceLastFrame(&mCurrentFrame, &mLastFrame);

        Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame, imupreint, mpLocalMapper->GetGravityVec(), true);
    }

    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking) {
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                } else
                    mnMatchesInliers++;
            } else if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
        return false;

    if (mnMatchesInliers < 6/*30*/)
        return false;
    else
        return true;
}

void Tracking::PredictNavStateByIMU(bool bMapUpdated) {
    if (!mpLocalMapper->GetVINSInited())
        cerr << "mpLocalMapper->GetVINSInited() not, shouldn't in PredictNavStateByIMU" << endl;

    // Map updated, optimize with last KeyFrame
    if (mpLocalMapper->GetFirstVINSInited() || bMapUpdated) {
        //if(mpLocalMapper->GetFirstVINSInited() && !bMapUpdated) cerr<<"2-FirstVinsInit, but not bMapUpdated. shouldn't"<<endl;

        // Compute IMU Pre-integration
        mIMUPreIntInTrack = GetIMUPreIntSinceLastKF(&mCurrentFrame, mpLastKeyFrame, mvIMUSinceLastKF);

        // Get initial NavState&pose from Last KeyFrame
        mCurrentFrame.SetInitialNavStateAndBias(mpLastKeyFrame->GetNavState());
        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack, Converter::toVector3d(mpLocalMapper->GetGravityVec()));
        mCurrentFrame.UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());

        // Test log
        // Updated KF by Local Mapping. Should be the same as mpLastKeyFrame
        if (mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6)
            cerr << "PredictNavStateByIMU1 current Frame dBias acc not zero" << endl;
        if (mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6)
            cerr << "PredictNavStateByIMU1 current Frame dBias gyr not zero" << endl;
    }
        // Map not updated, optimize with last Frame
    else {
        // Compute IMU Pre-integration
        mIMUPreIntInTrack = GetIMUPreIntSinceLastFrame(&mCurrentFrame, &mLastFrame);

        // Get initial pose from Last Frame
        mCurrentFrame.SetInitialNavStateAndBias(mLastFrame.GetNavState());
        mCurrentFrame.UpdateNavState(mIMUPreIntInTrack, Converter::toVector3d(mpLocalMapper->GetGravityVec()));
        mCurrentFrame.UpdatePoseFromNS(Config::getInstance().IMUParams().GetMatTbc());

        // Test log
        if (mCurrentFrame.GetNavState().Get_dBias_Acc().norm() > 1e-6)
            cerr << "PredictNavStateByIMU2 current Frame dBias acc not zero" << endl;
        if (mCurrentFrame.GetNavState().Get_dBias_Gyr().norm() > 1e-6)
            cerr << "PredictNavStateByIMU2 current Frame dBias gyr not zero" << endl;
    }
}

bool Tracking::TrackWithIMU(bool bMapUpdated) {
    ORBmatcher matcher(0.9, true);

    // VINS has been inited in this function
    if (!mpLocalMapper->GetVINSInited()) cerr << "local mapping VINS not inited. why call TrackWithIMU?" << endl;

    // Predict NavState&Pose by IMU
    // And compute the IMU pre-integration for PoseOptimization
    PredictNavStateByIMU(bMapUpdated);

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

    // Project points seen in previous frame
    int th;
    if (mSensor != System::STEREO)
        th = 15;
    else
        th = 7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

    // If few matches, uses a wider window search
    if (nmatches < 20) {
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
    }

    if (nmatches </*20*/10)
        return false;


    // Pose optimization. false: no need to compute marginalized for current Frame
    if (mpLocalMapper->GetFirstVINSInited() || bMapUpdated) {
        Optimizer::PoseOptimization(&mCurrentFrame, mpLastKeyFrame, mIMUPreIntInTrack,
                                    mpLocalMapper->GetGravityVec(), false);
    } else {
        Optimizer::PoseOptimization(&mCurrentFrame, &mLastFrame, mIMUPreIntInTrack, mpLocalMapper->GetGravityVec(),
                                    false);
    }


    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    return nmatchesMap >=/*10*/6;
}

IMUPreintegrator
Tracking::GetIMUPreIntSinceLastKF(Frame *pCurF, KeyFrame *pLastKF, const utils::eigen_aligned_vector<IMUData> &vIMUSInceLastKF) {
    // Reset pre-integrator first
    IMUPreintegrator IMUPreInt;
    IMUPreInt.reset();

    Vector3d bg = pLastKF->GetNavState().Get_BiasGyr();
    Vector3d ba = pLastKF->GetNavState().Get_BiasAcc();

    // remember to consider the gap between the last KF and the first IMU
    {
        const IMUData &imu = vIMUSInceLastKF.front();
        double dt = imu._t - pLastKF->mTimeStamp;
        IMUPreInt.update(imu._g - bg, imu._a - ba, dt);

        // Test log
        if (dt < 0) {
            cerr << std::fixed << std::setprecision(3) << "dt = " << dt << ", this KF vs last imu time: "
                 << pLastKF->mTimeStamp << " vs " << imu._t << endl;
            std::cerr.unsetf(std::ios::showbase);                // deactivate showbase
        }
    }
    // integrate each imu
    for (size_t i = 0; i < vIMUSInceLastKF.size(); i++) {
        const IMUData &imu = vIMUSInceLastKF[i];
        double nextt;
        if (i == vIMUSInceLastKF.size() - 1)
            nextt = pCurF->mTimeStamp;         // last IMU, next is this KeyFrame
        else
            nextt = vIMUSInceLastKF[i + 1]._t;  // regular condition, next is imu data

        // delta time
        double dt = nextt - imu._t;
        // update pre-integrator
        IMUPreInt.update(imu._g - bg, imu._a - ba, dt);


        // Test log
        if (dt <= 0) {
            cerr << std::fixed << std::setprecision(3) << "dt = " << dt << ", this vs next time: " << imu._t
                 << " vs " << nextt << endl;
            std::cerr.unsetf(std::ios::showbase);                // deactivate showbase
        }
    }

    return IMUPreInt;
}

IMUPreintegrator Tracking::GetIMUPreIntSinceLastFrame(Frame *pCurF, Frame *pLastF) {
    // Reset pre-integrator first
    IMUPreintegrator IMUPreInt;
    IMUPreInt.reset();

    pCurF->ComputeIMUPreIntSinceLastFrame(pLastF, IMUPreInt);

    return IMUPreInt;
}

cv::Mat Tracking::GrabImageMonoVI(const cv::Mat &im, const utils::eigen_aligned_vector<IMUData> &vimu, const double &timestamp) {

    mvIMUSinceLastKF.insert(mvIMUSinceLastKF.end(), vimu.begin(), vimu.end());
    mImGray = im;

    if (mImGray.channels() == 3) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGB2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGR2GRAY);
    } else if (mImGray.channels() == 4) {
        if (mbRGB)
            cvtColor(mImGray, mImGray, CV_RGBA2GRAY);
        else
            cvtColor(mImGray, mImGray, CV_BGRA2GRAY);
    }

    if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray, timestamp, vimu, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth);
    else
        mCurrentFrame = Frame(mImGray, timestamp, vimu, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth, mpLastKeyFrame);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageStereoVI(const cv::Mat &imRectLeft, const cv::Mat &imRectRight,
                          const ORB_SLAM2::utils::eigen_aligned_vector<IMUData> &vimu, const double &timestamp){

    mvIMUSinceLastKF.insert(mvIMUSinceLastKF.end(), vimu.begin(), vimu.end());

    {
        std::unique_lock<std::mutex> lock(mMutexImColor);
        mImColor = imRectLeft;
        mImColorRight = imRectRight;


        cv::Mat imGrayRight;

        if (mImColor.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImColor, mImGray, CV_RGB2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImColor, mImGray, CV_BGR2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImColor.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImColor, mImGray, CV_RGBA2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImColor, mImGray, CV_BGRA2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_BGRA2GRAY);
            }
        }


        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, vimu, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK,
                              mDistCoef, mbf, mThDepth);
    }
    Track();
    //TrackIMUStereo();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageRGBDVI(const cv::Mat &imRGB, const cv::Mat &imD,
                                  const ORB_SLAM2::utils::eigen_aligned_vector<ORB_SLAM2::IMUData> &vimu,
                                  const double &timestamp) {

    mvIMUSinceLastKF.insert(mvIMUSinceLastKF.end(), vimu.begin(), vimu.end());

    {
        std::unique_lock<std::mutex> lock(mMutexImColor);
        mImColor = imRGB;

        cv::Mat imDepth = imD;

        assert(mImColor.channels() >= 3);

        if (mImColor.channels() == 3) {
            if (mbRGB)
                cvtColor(mImColor, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImColor, mImGray, CV_BGR2GRAY);
        } else if (mImColor.channels() == 4) {
            if (mbRGB)
                cvtColor(mImColor, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImColor, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, vimu, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth);
    }
    Track();
    // TrackIMUStereo();

    return mCurrentFrame.mTcw.clone();
}

//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------

Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap,
                   KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor) :
        mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
        mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer *>(NULL)), mpSystem(pSys), mpViewer(NULL),
        mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
        mbCreateNewKFAfterReloc(false), mbRelocBiasPrepare(false) {
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if (fps == 0)
        fps = 30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if (DistCoef.rows == 5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if (mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    if (sensor == System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

    cout << endl << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if (sensor == System::STEREO || sensor == System::RGBD) {
        mThDepth = mbf * (float) fSettings["ThDepth"] / fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if (sensor == System::RGBD) {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if (fabs(mDepthMapFactor) < 1e-5)
            mDepthMapFactor = 1;
        else
            mDepthMapFactor = 1.0f / mDepthMapFactor;
    }

    if (mbUseObject) {
        mtCleanDetectionThread = std::make_shared<std::thread>(std::bind(&Tracking::CleanDetectionThread, this));
    }

}


Tracking::Tracking(System *pSys,
                   ORBVocabulary *pVoc,
                   FrameDrawer *pFrameDrawer,
                   MapDrawer *pMapDrawer,
                   Map *pMap,
                   KeyFrameDatabase *pKFDB,
                   const string &strSettingPath,
                   const int sensor,
                   const std::shared_ptr<BaseObjectDetector> &pObjectDetector)
        : Tracking(pSys, pVoc, pFrameDrawer, pMapDrawer, pMap, pKFDB, strSettingPath, sensor) {
    mpObjectDetector = pObjectDetector;
    mbUseObject = Config::getInstance().SystemParams().use_object;

    if (mbUseObject){
        for (const auto l : Config::getInstance().ObjectDetectionParams().selected_labels){
            msSelectedDetectionLabels.insert(l);
        }
    }

    if (Config::getInstance().EvalParams().enable){
        auto logp = boost::filesystem::path(Config::getInstance().RuntimeParams().log_file_path);
        if (!boost::filesystem::exists(logp)){
            boost::filesystem::create_directory(logp);
        }

        auto imgdir = logp / boost::filesystem::path("keyframe_images/");
        if (boost::filesystem::exists(imgdir) && boost::filesystem::is_directory(imgdir)){
            boost::filesystem::remove_all(imgdir);
        }

        boost::filesystem::create_directory(imgdir);

        mImageLogDir = imgdir.string();
    }

}

Tracking::~Tracking() {
    mbRequestReset = true;
    if (mtCleanDetectionThread) {
        mcvDetectionThreads.notify_all();
        if (mtCleanDetectionThread->joinable())
            mtCleanDetectionThread->join();
    }
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper) {
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing) {
    mpLoopClosing = pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer) {
    mpViewer = pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp) {

    {
        std::unique_lock<std::mutex> lock(mMutexImColor);
        mImColor = imRectLeft;
        mImColorRight = imRectRight;


        cv::Mat imGrayRight;

        if (mImColor.channels() == 3) {
            if (mbRGB) {
                cvtColor(mImColor, mImGray, CV_RGB2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_RGB2GRAY);
            } else {
                cvtColor(mImColor, mImGray, CV_BGR2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_BGR2GRAY);
            }
        } else if (mImColor.channels() == 4) {
            if (mbRGB) {
                cvtColor(mImColor, mImGray, CV_RGBA2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_RGBA2GRAY);
            } else {
                cvtColor(mImColor, mImGray, CV_BGRA2GRAY);
                cvtColor(mImColorRight, imGrayRight, CV_BGRA2GRAY);
            }
        }

        mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mK,
                              mDistCoef, mbf, mThDepth);
    }
    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp) {
    {
        std::unique_lock<std::mutex> lock(mMutexImColor);
        mImColor = imRGB;

        cv::Mat imDepth = imD;

        assert(mImColor.channels() >= 3);

        if (mImColor.channels() == 3) {
            if (mbRGB)
                cvtColor(mImColor, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImColor, mImGray, CV_BGR2GRAY);
        } else if (mImColor.channels() == 4) {
            if (mbRGB)
                cvtColor(mImColor, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImColor, mImGray, CV_BGRA2GRAY);
        }

        if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
            imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

        mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf,
                              mThDepth);
    }
    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp) {
    //mImGray = im;
    {
        std::unique_lock<std::mutex> lock(mMutexImColor);
        mImColor = im;

        if (mImColor.channels() == 3) {
            if (mbRGB)
                cvtColor(mImColor, mImGray, CV_RGB2GRAY);
            else
                cvtColor(mImColor, mImGray, CV_BGR2GRAY);
        } else if (mImColor.channels() == 4) {
            if (mbRGB)
                cvtColor(mImColor, mImGray, CV_RGBA2GRAY);
            else
                cvtColor(mImColor, mImGray, CV_BGRA2GRAY);
        }

        if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
            mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);
        else
            mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mK, mDistCoef, mbf, mThDepth);

    }

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track() {
    if (mState == NO_IMAGES_YET) {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState = mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    //auto lock = Map::CreateUpdateLock(mpMap);
    bool bUseIMU = mpLocalMapper->GetUseIMUFlag();
    // Different operation, according to whether the map is updated
    bool bMapUpdated = false;
    if (bUseIMU) {
        if (mpLocalMapper->GetMapUpdateFlagForTracking()) {
            bMapUpdated = true;
            mpLocalMapper->SetMapUpdateFlagInTracking(false);
        }
        if (mpLoopClosing->GetMapUpdateFlagForTracking()) {
            bMapUpdated = true;
            mpLoopClosing->SetMapUpdateFlagInTracking(false);
        }
        if (mCurrentFrame.mnId == mnLastRelocFrameId + 20) {
            bMapUpdated = true;
        }
    }

    if (mState == NOT_INITIALIZED) {
        if (mSensor == System::STEREO || mSensor == System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if (mState != OK)
            return;
    } else {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if (!mbOnlyTracking) {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if (mState == OK) {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();
                if (bUseIMU) {
                    // If Visual-Inertial is initialized
                    if (mpLocalMapper->GetVINSInited()) {
                        // 20 Frames after reloc, track with only vision
                        if (mbRelocBiasPrepare) {
                            bOK = TrackReferenceKeyFrame();
                        } else {
                            bOK = TrackWithIMU(bMapUpdated);
                            if (!bOK)
                                bOK = TrackReferenceKeyFrame();

                        }
                    }
                    // If Vi not initialized, use pure SLAM
                    if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                        bOK = TrackReferenceKeyFrame();
                    } else {
                        bOK = TrackWithMotionModel();
                        if (!bOK)
                            bOK = TrackReferenceKeyFrame();
                    }

                } else {
                    // Visual only
                    if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2) {
                        bOK = TrackReferenceKeyFrame();
                    } else {
                        bOK = TrackWithMotionModel();
                        if (!bOK)
                            bOK = TrackReferenceKeyFrame();
                    }
                }

            } else {
                bOK = Relocalization();
            }
        } else {
            // Localization Mode: Local Mapping is deactivated

            if (mState == LOST) {
                bOK = Relocalization();
            } else {
                if (!mbVO) {
                    // In last frame we tracked enough MapPoints in the map

                    if (!mVelocity.empty()) {
                        bOK = TrackWithMotionModel();
                    } else {
                        bOK = TrackReferenceKeyFrame();
                    }
                } else {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint *> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if (!mVelocity.empty()) {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if (bOKMM && !bOKReloc) {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if (mbVO) {
                            for (int i = 0; i < mCurrentFrame.N; i++) {
                                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i]) {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    } else if (bOKReloc) {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if (!mbOnlyTracking) {
            if (bOK) {
                if (bUseIMU) {
                    if (!mpLocalMapper->GetVINSInited())
                        bOK = TrackLocalMap();
                    else {
                        if (mbRelocBiasPrepare) {
                            // 20 Frames after reloc, track with only vision
                            bOK = TrackLocalMap();
                        } else {
                            bOK = TrackLocalMapWithIMU(bMapUpdated);
                        }
                    }
                } else {
                    bOK = TrackLocalMap();
                }
            }
        } else {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if (bUseIMU){
                SPDLOG_ERROR("Localization with IMU not supported yet");
                throw std::runtime_error("Not Implemented");
            }

            if (bOK && !mbVO)
                bOK = TrackLocalMap();
        }

//        if (bOK)
//            mState = OK;
//        else
//            mState = LOST;

        if (bOK) {
            mState = OK;

            // Add Frames to re-compute IMU bias after reloc
            if (bUseIMU && mbRelocBiasPrepare) {
                mv20FramesReloc.push_back(mCurrentFrame);

                // Before creating new keyframe
                // Use 20 consecutive frames to re-compute IMU bias
                if (mCurrentFrame.mnId == mnLastRelocFrameId + 20 - 1) {
                    NavState nscur;
                    RecomputeIMUBiasAndCurrentNavstate(nscur);
                    // Update NavState of CurrentFrame
                    mCurrentFrame.SetNavState(nscur);
                    // Clear flag and Frame buffer
                    mbRelocBiasPrepare = false;
                    mv20FramesReloc.clear();

                    // Release LocalMapping. To ensure to insert new keyframe.
                    mpLocalMapper->Release();
                    // Create new KeyFrame
                    mbCreateNewKFAfterReloc = true;

                    //Debug log
                    cout << "NavState recomputed." << endl;
                    cout << "V:" << mCurrentFrame.GetNavState().Get_V().transpose() << endl;
                    cout << "bg:" << mCurrentFrame.GetNavState().Get_BiasGyr().transpose() << endl;
                    cout << "ba:" << mCurrentFrame.GetNavState().Get_BiasAcc().transpose() << endl;
                    cout << "dbg:" << mCurrentFrame.GetNavState().Get_dBias_Gyr().transpose() << endl;
                    cout << "dba:" << mCurrentFrame.GetNavState().Get_dBias_Acc().transpose() << endl;
                }
            }
        } else {
            mState = LOST;

            // Clear Frame vectors for reloc bias computation
            if (bUseIMU && mv20FramesReloc.size() > 0)
                mv20FramesReloc.clear();
        }

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if (bOK) {
            // Update motion model
            if (!mLastFrame.mTcw.empty()) {
                cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
                mVelocity = mCurrentFrame.mTcw * LastTwc;
            } else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for (int i = 0; i < mCurrentFrame.N; i++) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (pMP)
                    if (pMP->Observations() < 1) {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for (list<MapPoint *>::iterator lit = mlpTemporalPoints.begin(), lend = mlpTemporalPoints.end();
                 lit != lend; lit++) {
                MapPoint *pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe

            if (NeedNewKeyFrame() || (bUseIMU && mbCreateNewKFAfterReloc))
                CreateNewKeyFrame();


            // Clear flag
            if (bUseIMU && mbCreateNewKFAfterReloc)
                mbCreateNewKFAfterReloc = false;

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for (int i = 0; i < mCurrentFrame.N; i++) {
                if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
            }


            // Clear First-Init flag
            if (bUseIMU && mpLocalMapper->GetFirstVINSInited()) {
                mpLocalMapper->SetFirstVINSInited(false);
            }
        }

        // Reset if the camera get lost soon after initialization
        if (mState == LOST) {
            auto cond = bUseIMU ? !mpLocalMapper->GetVINSInited() : mpMap->KeyFramesInMap() <= 5;

            if (cond) {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }

        }

        if (!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if (!mCurrentFrame.mTcw.empty()) {
        cv::Mat Tcr = mCurrentFrame.mTcw * mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState == LOST);
    } else {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState == LOST);
    }
}

void Tracking::StereoInitialization() {
    if (mCurrentFrame.N > 500) {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

        KeyFrame *pKFini;
        // Create KeyFrame
        if (Config::getInstance().SystemParams().use_imu) {
            utils::eigen_aligned_vector<IMUData> vimu;
            auto it = mvIMUSinceLastKF.begin();
            for (auto end = mvIMUSinceLastKF.end(); it != end; it++){
                if ( it->_t < mCurrentFrame.mTimeStamp){
                    vimu.push_back(*it);
                }
                else{
                    break;
                }
            }

            pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper, vimu, NULL);
            if (!mpLocalMapper->GetUseIMUFastInit()){
                pKFini->ComputePreInt();
            }

            mvIMUSinceLastKF.erase(mvIMUSinceLastKF.begin(), it);
        }
        else {
            pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper);
        }


        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for (int i = 0; i < mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpMap);
                pNewMP->AddObservation(pKFini, i);
                pKFini->AddMapPoint(pNewMP, i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i] = pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId = mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints = mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState = OK;
    }
}

void Tracking::MonocularInitialization() {

    if (!mpInitializer) {
        // Clear imu data
        mvIMUSinceLastKF.clear();

        // Set Reference Frame
        if (mCurrentFrame.mvKeys.size() > 100) {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

            if (mpInitializer)
                delete mpInitializer;

            mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

            return;
        }
    } else {
        // Try to initialize
        if ((int) mCurrentFrame.mvKeys.size() <= 100) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
            fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9, true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

        // Check if there are enough correspondences
        if (nmatches < 100) {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer *>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated)) {
            for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++) {
                if (mvIniMatches[i] >= 0 && !vbTriangulated[i]) {
                    mvIniMatches[i] = -1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
            Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
            tcw.copyTo(Tcw.rowRange(0, 3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular() {

    // The first imu package include 2 parts for KF1 and KF2
    utils::eigen_aligned_vector<IMUData> vimu1, vimu2;
    for (size_t i = 0; i < mvIMUSinceLastKF.size(); i++) {
        IMUData imu = mvIMUSinceLastKF[i];
        if (imu._t < mInitialFrame.mTimeStamp)
            vimu1.push_back(imu);
        else
            vimu2.push_back(imu);
    }

    // Create KeyFrames
    KeyFrame *pKFini;
    KeyFrame *pKFcur;

    if (Config::getInstance().SystemParams().use_imu) {
        pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB, mpLocalMapper, vimu1, NULL);
        pKFini->ComputePreInt();
        pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper, vimu2, pKFini);
        pKFcur->ComputePreInt();
        // Clear IMUData buffer
        mvIMUSinceLastKF.clear();
    } else {
        pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB, mpLocalMapper);
        pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper);
    }

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for (size_t i = 0; i < mvIniMatches.size(); i++) {
        if (mvIniMatches[i] < 0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpMap);

        pKFini->AddMapPoint(pMP, i);
        pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

        pMP->AddObservation(pKFini, i);
        pMP->AddObservation(pKFcur, mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f / medianDepth;

    if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100) {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3) * invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint *> vpAllMapPoints = pKFini->GetMapPointMatches();
    for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++) {
        if (vpAllMapPoints[iMP]) {
            MapPoint *pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos() * invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints = mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState = OK;
}

void Tracking::CheckReplacedInLastFrame() {
    for (int i = 0; i < mLastFrame.N; i++) {
        MapPoint *pMP = mLastFrame.mvpMapPoints[i];

        if (pMP) {
            MapPoint *pRep = pMP->GetReplaced();
            if (pRep) {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame() {
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7, true);
    vector<MapPoint *> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

    if (nmatches < 15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    return nmatchesMap >= 10;
}

void Tracking::UpdateLastFrame() {
    // Update pose according to reference keyframe
    KeyFrame *pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr * pRef->GetPose());

    if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float, int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for (int i = 0; i < mLastFrame.N; i++) {
        float z = mLastFrame.mvDepth[i];
        if (z > 0) {
            vDepthIdx.push_back(make_pair(z, i));
        }
    }

    if (vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(), vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for (size_t j = 0; j < vDepthIdx.size(); j++) {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint *pMP = mLastFrame.mvpMapPoints[i];
        if (!pMP)
            bCreateNew = true;
        else if (pMP->Observations() < 1) {
            bCreateNew = true;
        }

        if (bCreateNew) {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint *pNewMP = new MapPoint(x3D, mpMap, &mLastFrame, i);

            mLastFrame.mvpMapPoints[i] = pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        } else {
            nPoints++;
        }

        if (vDepthIdx[j].first > mThDepth && nPoints > 100)
            break;
    }
}

bool Tracking::TrackWithMotionModel() {
    ORBmatcher matcher(0.9, true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity * mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));

    // Project points seen in previous frame
    int th;
    if (mSensor != System::STEREO)
        th = 15;
    else
        th = 7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

    // If few matches, uses a wider window search
    if (nmatches < 20) {
        fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), static_cast<MapPoint *>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
    }

    if (nmatches < 20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (mCurrentFrame.mvbOutlier[i]) {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                mCurrentFrame.mvbOutlier[i] = false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            } else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                nmatchesMap++;
        }
    }

    if (mbOnlyTracking) {
        mbVO = nmatchesMap < 10;
        return nmatches > 20;
    }

    return nmatchesMap >= 10;
}

bool Tracking::TrackLocalMap() {
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            if (!mCurrentFrame.mvbOutlier[i]) {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if (!mbOnlyTracking) {
                    if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
                        mnMatchesInliers++;
                } else
                    mnMatchesInliers++;
            } else if (mSensor == System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);

        }
    }

    // Decide if the tracking was successful
    // More restrictive if there was a relocalization recently
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50) {
        if (mbUseObject)
            SPDLOG_WARN("Lost Track: Try to reduce prediction model size");
        return false;
    }

    if (mnMatchesInliers < 30) {
        return false;
    } else {
        return true;
    }

}


bool Tracking::NeedNewKeyFrame() {
    if (mbOnlyTracking)
        return false;

    bool bUseIMU = mpLocalMapper->GetUseIMUFlag();

    // While updating initial poses
    if (bUseIMU && mpLocalMapper->GetUpdatingInitPoses()) {
        cerr << "mpLocalMapper->GetUpdatingInitPoses, no new KF" << endl;
        return false;
    }

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && nKFs > mMaxFrames)
        return false;


    // Do not insert keyframes if bias is not computed in VINS mode
    if (bUseIMU && mbRelocBiasPrepare/* && mpLocalMapper->GetVINSInited()*/)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if (nKFs <= 2)
        nMinObs = 2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose = 0;
    if (mSensor != System::MONOCULAR) {
        for (int i = 0; i < mCurrentFrame.N; i++) {
            if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth) {
                if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

    // Thresholds
    float thRefRatio = 0.75f;
    if (nKFs < 2)
        thRefRatio = 0.4f;

    if (mSensor == System::MONOCULAR)
        thRefRatio = bUseIMU ? 0.8f : 0.9f;


    bool cTimeGap = false;
    if (bUseIMU) {
        double timegap = 0.1;

        if (mpLocalMapper->GetVINSInited())
            timegap = 0.5;
        //const bool cTimeGap = (fabs(mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp)>=0.3 && mnMatchesInliers>15);
        cTimeGap =
                ((mCurrentFrame.mTimeStamp - mpLastKeyFrame->mTimeStamp) >= timegap) && bLocalMappingIdle &&
                mnMatchesInliers > 15;
    }

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    //const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // TODO -- check VIORB paper for this condition
    const bool c1a = bUseIMU ?
            mCurrentFrame.mTimeStamp >= mpLastKeyFrame->mTimeStamp + 3.0 : mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches * 0.25 || bNeedToInsertClose);
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers < nRefMatches * thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

    if (((c1a || c1b || c1c) && c2) || (bUseIMU && cTimeGap)) {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if (bLocalMappingIdle) {
            return true;
        } else {
            mpLocalMapper->InterruptBA();
            if (mSensor != System::MONOCULAR) {
                if (mpLocalMapper->KeyframesInQueue() < 3)
                    return true;
                else
                    return false;
            } else
                return false;
        }
    } else
        return false;
}

void Tracking::CreateNewKeyFrame() {
    if (!mpLocalMapper->SetNotStop(true))
        return;
    static auto last_time = utils::time::time_now();
    auto time_now = utils::time::time_now();
    SPDLOG_INFO("KeyFrame Create Time diff {}", utils::time::time_diff_second(last_time, time_now));
    last_time = time_now;
    //TODO: is it necessary to clear IMU buffers if this is the first KeyFrame after relocalization (also no prevKF)?
    KeyFrame *pKF;
    bool bUseIMU = Config::getInstance().SystemParams().use_imu;
    if (mbUseObject) {

        std::unique_lock<std::mutex> lock(mMutexImColor);
        if (bUseIMU && !mpLocalMapper->GetVINSInited()) {
            assert (mSensor != System::MONOCULAR);
            pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper, mvIMUSinceLastKF, mpLastKeyFrame);
            // mCurrentFrame.mvIMUDataSinceLastFrame; ?? or use this

            if (!Config::getInstance().IMUParams().fast_init){
                // Set initial NavState for KeyFrame
                pKF->SetInitialNavStateAndBias(mCurrentFrame.GetNavState());
                // Compute pre-integrator
                pKF->ComputePreInt();
            }


        } else {
            pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper);

            if (KeyFrame::nInitId < 0) {
                KeyFrame::nInitId = pKF->mnId; // First KF to consider objects
            }

            // Make a copy of current imcolor;
            if (mpLocalMapper->DetectWaitQueueAvaliable()){
                auto start_time2 = utils::time::time_now();
                QueueDetectionThread(pKF,
                                     mImColor.clone(),
                                     Config::getInstance().ObjectDetectionParams().allow_skip);
                //SPDLOG_INFO("RGB Image Clone time {}", utils::time::time_diff_from_now_second(start_time2));
            }
        }

        // Note that eval logic should be used on mbUseObject==true only!
        if (Config::getInstance().EvalParams().enable){
            auto eval_params = Config::getInstance().EvalParams();

            if (mCurrentNumSaveKFImages < eval_params.num_initial_skip){
                mCurrentNumSaveKFImages++;
            }

            if  ((mCurrentNumSaveKFImages >= eval_params.num_initial_skip)
                && (mCurrentNumSaveKFImages <= eval_params.num_save_kf_images + eval_params.num_initial_skip)) {
                auto tmp_img = mImColor.clone();
                char outfile[100];
                auto nowstamp = utils::time::time_now().time_since_epoch().count();
                sprintf(outfile, "%s/kf-%ld-%ld.png", mImageLogDir.c_str(), pKF->mnId, nowstamp);
                cv::imwrite(outfile, tmp_img);
                mCurrentNumSaveKFImages++;
            }

            if (mCurrentNumSaveKFImages <= eval_params.num_save_kf_images + eval_params.num_initial_skip + 1) {
                SPDLOG_INFO("Evaluation Image Saving Done!!!!!!!!!!!!!!!!!!!!!!");
                mCurrentNumSaveKFImages++;
            }
        }
    } else {
        if (bUseIMU) {
            pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper, mvIMUSinceLastKF, mpLastKeyFrame);
            // Set initial NavState for KeyFrame
            pKF->SetInitialNavStateAndBias(mCurrentFrame.GetNavState());
            // Compute pre-integrator
            pKF->ComputePreInt();
            // Clear IMUData buffer
            // mvIMUSinceLastKF.clear();
        } else {
            pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB, mpLocalMapper);
        }
    }

    // Clear IMUData buffer
    mvIMUSinceLastKF.clear();

    // TODO -- Add Keypoint Color Rendering (or perform as a Thread)
    // AddColorToKeyPoints(pKF);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if (mSensor != System::MONOCULAR) {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float, int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for (int i = 0; i < mCurrentFrame.N; i++) {
            float z = mCurrentFrame.mvDepth[i];
            if (z > 0) {
                vDepthIdx.push_back(make_pair(z, i));
            }
        }

        if (!vDepthIdx.empty()) {
            sort(vDepthIdx.begin(), vDepthIdx.end());

            int nPoints = 0;
            for (size_t j = 0; j < vDepthIdx.size(); j++) {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if (!pMP)
                    bCreateNew = true;
                else if (pMP->Observations() < 1) {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint *>(NULL);
                }

                if (bCreateNew) {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint *pNewMP = new MapPoint(x3D, pKF, mpMap);
                    pNewMP->AddObservation(pKF, i);
                    pKF->AddMapPoint(pNewMP, i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i] = pNewMP;
                    nPoints++;
                } else {
                    nPoints++;
                }

                if (vDepthIdx[j].first > mThDepth && nPoints > 100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints() {
    // Do not search map points already matched
    for (vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end();
         vit != vend; vit++) {
        MapPoint *pMP = *vit;
        if (pMP) {
            if (pMP->isBad()) {
                *vit = static_cast<MapPoint *>(NULL);
            } else {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch = 0;

    // Project points in frame and check its visibility
    for (vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end();
         vit != vend; vit++) {
        MapPoint *pMP = *vit;
        if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if (pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if (mCurrentFrame.isInFrustum(pMP, 0.5)) {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if (nToMatch > 0) {
        ORBmatcher matcher(0.8);
        int th = 1;
        if (mSensor == System::RGBD)
            th = 3;
        // If the camera has been relocalised recently, perform a coarser search
        if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
            th = 5;
        matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
    }
}

void Tracking::UpdateLocalMap() {
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints() {
    mvpLocalMapPoints.clear();

    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++) {
        KeyFrame *pKF = *itKF;
        const vector<MapPoint *> vpMPs = pKF->GetMapPointMatches();

        for (vector<MapPoint *>::const_iterator itMP = vpMPs.begin(), itEndMP = vpMPs.end(); itMP != itEndMP; itMP++) {
            MapPoint *pMP = *itMP;
            if (!pMP)
                continue;
            if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
                continue;
            if (!pMP->isBad()) {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames() {
    bool bUseIMU = mpLocalMapper->GetUseIMUFlag();
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame *, int> keyframeCounter;
    for (int i = 0; i < mCurrentFrame.N; i++) {
        if (mCurrentFrame.mvpMapPoints[i]) {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (!pMP->isBad()) {
                const auto observations = pMP->GetObservations();
                for (auto it = observations.cbegin(), itend = observations.cend(); it != itend; it++)
                    keyframeCounter[it->first]++;
            } else {
                mCurrentFrame.mvpMapPoints[i] = NULL;
            }
        }
    }

    if (keyframeCounter.empty())
        return;

    int max = 0;
    KeyFrame *pKFmax = static_cast<KeyFrame *>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for (map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end();
         it != itEnd; it++) {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        if (it->second > max) {
            max = it->second;
            pKFmax = pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for (vector<KeyFrame *>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end();
         itKF != itEndKF; itKF++) {
        // Limit the number of keyframes
        if (mvpLocalKeyFrames.size() > 80)
            break;

        KeyFrame *pKF = *itKF;

        const vector<KeyFrame *> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for (vector<KeyFrame *>::const_iterator itNeighKF = vNeighs.begin(), itEndNeighKF = vNeighs.end();
             itNeighKF != itEndNeighKF; itNeighKF++) {
            KeyFrame *pNeighKF = *itNeighKF;
            if (!pNeighKF->isBad()) {
                if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame *> spChilds = pKF->GetChilds();
        for (set<KeyFrame *>::const_iterator sit = spChilds.begin(), send = spChilds.end(); sit != send; sit++) {
            KeyFrame *pChildKF = *sit;
            if (!pChildKF->isBad()) {
                if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame *pParent = pKF->GetParent();
        if (pParent) {
            if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                if (!bUseIMU)
                    break;
            }
        }

        if (bUseIMU) {
            KeyFrame *pPrevKF = pKF->GetPrevKeyFrame();
            if (pPrevKF) {
                if (pPrevKF->isBad()) cerr << "pPrevKF is bad in UpdateLocalKeyFrames()?????" << endl;
                if (pPrevKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pPrevKF);
                    pPrevKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }

            KeyFrame *pNextKF = pKF->GetNextKeyFrame();
            if (pNextKF) {
                if (pNextKF->isBad()) cerr << "pNextKF is bad in UpdateLocalKeyFrames()?????" << endl;
                if (pNextKF->mnTrackReferenceForFrame != mCurrentFrame.mnId) {
                    mvpLocalKeyFrames.push_back(pNextKF);
                    pNextKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
                }
            }
        }
    }

    if (pKFmax) {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization() {
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame *> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if (vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75, true);

    vector<PnPsolver *> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint *> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates = 0;

    for (int i = 0; i < nKFs; i++) {
        KeyFrame *pKF = vpCandidateKFs[i];
        if (pKF->isBad())
            vbDiscarded[i] = true;
        else {
            int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
            if (nmatches < 15) {
                vbDiscarded[i] = true;
                continue;
            } else {
                PnPsolver *pSolver = new PnPsolver(mCurrentFrame, vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9, true);

    while (nCandidates > 0 && !bMatch) {
        for (int i = 0; i < nKFs; i++) {
            if (vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver *pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if (bNoMore) {
                vbDiscarded[i] = true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if (!Tcw.empty()) {
                Tcw.copyTo(mCurrentFrame.mTcw);
                set<MapPoint *> sFound;

                const int np = vbInliers.size();

                for (int j = 0; j < np; j++) {
                    if (vbInliers[j]) {
                        mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    } else
                        mCurrentFrame.mvpMapPoints[j] = NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if (nGood < 10)
                    continue;

                for (int io = 0; io < mCurrentFrame.N; io++)
                    if (mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint *>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if (nGood < 50) {
                    int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

                    if (nadditional + nGood >= 50) {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if (nGood > 30 && nGood < 50) {
                            sFound.clear();
                            for (int ip = 0; ip < mCurrentFrame.N; ip++)
                                if (mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

                            // Final optimization
                            if (nGood + nadditional >= 50) {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for (int io = 0; io < mCurrentFrame.N; io++)
                                    if (mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io] = NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if (nGood >= 50) {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if (!bMatch) {
        return false;
    } else {
        mbRelocBiasPrepare = true; // no need to check use IMU,
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset() {

    cout << "System Reseting" << endl;
    if (mpViewer) {
        mpViewer->RequestStop();
        while (!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mCurrentNumSaveKFImages = 0;
    mState = NO_IMAGES_YET;

    if (mpInitializer) {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer *>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if (mpViewer)
        mpViewer->Release();

    mbRequestReset = true;
    if (mbUseObject) {
        if (mqDetectionThreads.size() > 0) {
            if (mtCleanDetectionThread) {
                mcvDetectionThreads.notify_all();
                if (mtCleanDetectionThread->joinable())
                    mtCleanDetectionThread->join();
            }
        }

        KeyFrame::nInitId = -1;
    }


    mbRequestReset = false;
}

void Tracking::ChangeCalibration(const string &strSettingPath) {
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = fx;
    K.at<float>(1, 1) = fy;
    K.at<float>(0, 2) = cx;
    K.at<float>(1, 2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if (k3 != 0) {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag) {
    mbOnlyTracking = flag;
}

void Tracking::DetectObjectInKeyFrame(KeyFrame *pKeyFrame, const cv::Mat &ImColor) {
    SPDLOG_DEBUG("START DETECT: DetectionThread Invoked! KeyframeID={}", pKeyFrame->mnId);
    auto time_point = utils::time::time_now();
    if (msSelectedDetectionLabels.empty()){
        std::lock_guard<std::mutex> lock(pKeyFrame->mMutexObject);
        mpObjectDetector->detectObject(ImColor, pKeyFrame->mvObjectPrediction, false);
        pKeyFrame->mvpMapObjects.resize(pKeyFrame->mvObjectPrediction.size(), static_cast<MapObject *>(NULL));
        pKeyFrame->mvObjectPredictionCuboidEst.resize(pKeyFrame->mvObjectPrediction.size(),
                static_cast<Cuboid *>(NULL));
    }
    else{

        std::vector<std::shared_ptr<PredictedObject> > detections;
        mpObjectDetector->detectObject(ImColor, detections, false);

        {
            std::lock_guard<std::mutex> lock(pKeyFrame->mMutexObject);
            pKeyFrame->mvObjectPrediction.reserve(detections.size());
            for (const auto &d : detections ){
                if (msSelectedDetectionLabels.count(d->_label)){
                    pKeyFrame->mvObjectPrediction.push_back(d);
                }
            }
            pKeyFrame->mvpMapObjects.resize(pKeyFrame->mvObjectPrediction.size(), static_cast<MapObject *>(NULL));
            pKeyFrame->mvObjectPredictionCuboidEst.resize(pKeyFrame->mvObjectPrediction.size(),
                                                          static_cast<Cuboid *>(NULL));

        }
    }

    {
        std::lock_guard<std::mutex> lock(pKeyFrame->mMutexbObjectReady);
        pKeyFrame->mbObjectReady = true;
    }

    pKeyFrame->mcvObjectReady.notify_all(); // in case others is waiting

    if (mpFrameDrawer)
        mpFrameDrawer->UpdateObjectFrame(ImColor, pKeyFrame);

    mcvDetectionThreads.notify_all();
    SPDLOG_DEBUG("END DETECT: Detect {} Objects in KeyframeID={}, time={}",
                 pKeyFrame->mvObjectPrediction.size(),
                 pKeyFrame->mnId,
                 utils::time::time_diff_from_now_second(time_point));
}

void Tracking::QueueDetectionThread(KeyFrame *pKeyframe, const cv::Mat &ImColor, bool skip_if_running) {
    std::lock_guard<std::mutex> lock(mMutexDetectionThreads);
    //t->detach();
    SPDLOG_INFO("DETECTION QUEUE SIZE {}", mqDetectionThreads.size());
    if (skip_if_running && mqDetectionThreads.size() > 0){
        SPDLOG_WARN("Skip detection for Keyframe {}", pKeyframe->mnId);

        {
            std::lock_guard<std::mutex> lock(pKeyframe->mMutexbObjectReady);
            pKeyframe->mbObjectReady = true;
        }

        pKeyframe->mcvObjectReady.notify_all(); // in case others is waiting
    }
    else {
        SPDLOG_WARN("Running Object detection for KF: {}", pKeyframe->mnId);
        std::shared_ptr<std::thread> t = std::make_shared<std::thread>(
                std::bind(&Tracking::DetectObjectInKeyFrame,
                          this, std::placeholders::_1, std::placeholders::_2),
                pKeyframe, ImColor);
        t->detach();
        mqDetectionThreads.push(t);
    }
}

void Tracking::CleanDetectionThread() {
    while(!mbRequestReset){
        std::unique_lock<std::mutex> lock(mMutexDetectionThreads);
        mcvDetectionThreads.wait(lock);
        //auto t = mqDetectionThreads.front();
        mqDetectionThreads.pop();
        SPDLOG_INFO("Detection Thread After Clean {}", mqDetectionThreads.size());

        //        if (t->joinable())
//            t->join();
    }
}


} //namespace ORB_SLAM
