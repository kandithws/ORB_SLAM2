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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "MapObject.h"
#include <set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <spdlog/spdlog.h>

namespace ORB_SLAM2 {

class MapPoint;
class KeyFrame;
class MapObject;

class KFIdComapre{
  public:
    bool operator()(const KeyFrame* kfleft,const KeyFrame* kfright) const;
};

class Map {
  public:
    // Update after an absolute scale is available
    void UpdateScale(const double &scale);

    //-----------------------------------------
  public:
    typedef pcl::PointXYZRGBL PCLPointT;
    Map();
    ~Map();
    void AddKeyFrame(KeyFrame *pKF);
    void AddMapPoint(MapPoint *pMP);
    void AddMapObject(MapObject *pMO);
    void EraseMapPoint(MapPoint *pMP);
    void EraseKeyFrame(KeyFrame *pKF);
    void EraseMapObject(MapObject *pMO);
    void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();
    KeyFrame* GetInitialKeyFrame();

    std::vector<KeyFrame *> GetAllKeyFrames();
    std::vector<MapPoint *> GetAllMapPoints();
    std::vector<MapObject *> GetAllMapObjects();
    std::vector<MapPoint *> GetReferenceMapPoints();

    long unsigned int MapPointsInMap();
    long unsigned KeyFramesInMap();

    long unsigned int GetMaxKFid();

    void clear();

    vector<KeyFrame *> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;


    pcl::PointCloud<PCLPointT>::Ptr GetCloudPtr();
    void NotifyMapUpdated();

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    std::mutex mMutexObjectCreation;

    void ShutDown();

    void SetGravityVec(const cv::Mat& g);
    bool GetGravityVec(cv::Mat& g);

  protected:
    std::set<MapPoint *> mspMapPoints;
    std::set<KeyFrame *, KFIdComapre> mspKeyFrames;
    std::set<MapObject *> mspMapObjects;

    std::vector<MapPoint *> mvpReferenceMapPoints;

    // Notify Condition variable wheter map is updated

    bool mbMapUpdate = false; // Trigger RenderPointCloud
    std::condition_variable mcvMapUpdate;

    // PCL Related FN
    void InitPointCloudThread();
    void RenderPointCloudThread();
    void RenderPointCloud();

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    // Point Cloud related attributes
    bool mbIsShutdown = false;
    std::mutex mMutexCloud;
    pcl::PointCloud<PCLPointT>::Ptr mpCloudMap;
    std::shared_ptr<std::thread> mtPointcloudRendererThread;

    bool mbRenderReady = false; // for PCLViewer

    std::mutex mMutexGravityVec;
    cv::Mat mGravityVec;
    bool mbGravityVecInited = false;
    float mfGravityVecScale = 1.0;
    KeyFrame* mpInitialKeyFrame;
};
} //namespace ORB_SLAM

#endif // MAP_H
