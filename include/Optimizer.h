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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "utils/vector_utils.h"

namespace ORB_SLAM2 {

class LoopClosing;

class Optimizer {
  public:
    void static
    LocalBAPRVIDP(KeyFrame *pKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag, Map *pMap, cv::Mat &gw,
                  LocalMapping *pLM = NULL);

    void static GlobalBundleAdjustmentNavStatePRV(Map *pMap, const cv::Mat &gw, int nIterations, bool *pbStopFlag,
                                                  const unsigned long nLoopKF, const bool bRobust);

    void static
    LocalBundleAdjustmentNavStatePRV(KeyFrame *pKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag,
                                     Map *pMap, cv::Mat &gw, LocalMapping *pLM = NULL);

    void static GlobalBundleAdjustmentNavState(Map *pMap, const cv::Mat &gw, int nIterations, bool *pbStopFlag,
                                               const unsigned long nLoopKF, const bool bRobust);

    int static PoseOptimization(Frame *pFrame, KeyFrame *pLastKF, const IMUPreintegrator &imupreint, const cv::Mat &gw,
                                const bool &bComputeMarg = false);

    int static PoseOptimization(Frame *pFrame, Frame *pLastFrame, const IMUPreintegrator &imupreint, const cv::Mat &gw,
                                const bool &bComputeMarg = false);

    void static
    LocalBundleAdjustmentNavState(KeyFrame *pKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag,
                                  Map *pMap, cv::Mat &gw, LocalMapping *pLM = NULL);

    Vector3d static OptimizeInitialGyroBias(const std::list<KeyFrame *> &lLocalKeyFrames);

    Vector3d static OptimizeInitialGyroBias(const std::vector<KeyFrame *> &vLocalKeyFrames);

    //Vector3d static OptimizeInitialGyroBias(const std::vector<Frame> &vFrames);
    Vector3d static OptimizeInitialGyroBias(const utils::eigen_aligned_vector<Frame> &vFrames);

    Vector3d static OptimizeInitialGyroBias(const vector<cv::Mat> &vTwc, const utils::eigen_aligned_vector<IMUPreintegrator> &vImuPreInt);

    void static // with IMU
    LocalBundleAdjustment(KeyFrame *pKF, const std::list<KeyFrame *> &lLocalKeyFrames, bool *pbStopFlag, Map *pMap,
                          LocalMapping *pLM = NULL);

  public:
    void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                 const bool bRobust = true);

    void static GlobalBundleAdjustemnt(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                       const unsigned long nLoopKF = 0, const bool bRobust = true);

    void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, LocalMapping* pLM=NULL);

    void static LocalBundleAdjustmentWithObjects(KeyFrame *pKF, bool *pbStopFlag, Map *pMap);
    void static LocalBundleAdjustmentWithObjects2(KeyFrame *pKF, bool *pbStopFlag, Map *pMap);
    void static LocalBundleAdjustmentWithObjects(KeyFrame *pKF, bool *pbStopFlag,
            Map *pMap, const cv::Mat& gravity);

    int static PoseOptimization(Frame *pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale, LoopClosing* pLC=NULL);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
