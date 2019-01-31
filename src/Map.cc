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

#include "Map.h"
#include "utils/smart_ptr_make_macro.h"
#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
    InitPointCloudThread();
}

void Map::AddKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(std::shared_ptr<MapPoint> pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    auto st = mspMapPoints.insert(pMP);
    if(st.second){
        // Signal Add new Pointcloud
    }
}

void Map::EraseMapPoint(std::shared_ptr<MapPoint> pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(std::shared_ptr<KeyFrame> pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<std::shared_ptr<MapPoint> > &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<std::shared_ptr<KeyFrame> > Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<std::shared_ptr<KeyFrame> >(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<std::shared_ptr<MapPoint> > Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<std::shared_ptr<MapPoint> >(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<std::shared_ptr<MapPoint> > Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<std::shared_ptr<MapPoint> >::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<std::shared_ptr<KeyFrame> >::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// ----------------------------- Pointcloud Related

pcl::PointCloud<Map::PCLPointT>::Ptr Map::GetCloudPtr() {
    std::lock_guard<std::mutex> lock(mMutexCloud);
    return mpCloudMap;
}

void Map::InitPointCloudThread() {
    mbIsShutdown = false;
    mpCloudMap = BOOST_MAKE_SHARED(pcl::PointCloud<PCLPointT >);
    mtPointcloudRendererThread = std::make_shared<std::thread>(std::bind(&Map::RenderPointCloudThread, this));
}

void Map::ShutDown() {
    if(mtPointcloudRendererThread){
        mbIsShutdown = true;
        NotifyMapUpdated();
        mtPointcloudRendererThread->join();
    }
}

Map::~Map() {
    ShutDown();
}

void Map::RenderPointCloudThread() {
    while (!mbIsShutdown){
        std::unique_lock<std::mutex> lock(mMutexMapUpdate);
        mcvMapUpdate.wait(lock, [this]{ return mbMapUpdate || mbIsShutdown;});
        mbMapUpdate = false;
        lock.unlock(); //unlock update signal
        RenderPointCloud(); //dispatch event
    }
}

void Map::RenderPointCloud() {
    SPDLOG_DEBUG("Rendering PointCloud");
    {
        std::lock_guard<std::mutex> lock(mMutexCloud);
        if(mbRenderReady)
            return;
    }

    pcl::PointCloud<PCLPointT>::Ptr map_cloud_ptr = BOOST_MAKE_SHARED(pcl::PointCloud<PCLPointT>);
    {
        const std::vector<std::shared_ptr<MapPoint> > &map_points = GetAllMapPoints();
        if(map_points.empty())
            return;

        const vector<std::shared_ptr<MapPoint> > &ref_map_points = GetReferenceMapPoints();
        std::set<std::shared_ptr<MapPoint> > set_ref_map_points(ref_map_points.begin(), ref_map_points.end());

        for(size_t i=0, iend=map_points.size(); i < iend; i++){

            if(map_points[i]->isBad() || set_ref_map_points.count(map_points[i]))
                continue;

            cv::Mat pos = map_points[i]->GetWorldPos();
            PCLPointT point;
            point.x = pos.at<float>(0);
            point.y = pos.at<float>(1);
            point.z = pos.at<float>(2);
            map_cloud_ptr->push_back(point);
        }

        for (std::set<std::shared_ptr<MapPoint> >::iterator sit=set_ref_map_points.begin(),
                     send=set_ref_map_points.end(); sit != send; sit++){

            if((*sit)->isBad())
                continue;

            cv::Mat pos = (*sit)->GetWorldPos();
            PCLPointT point;
            point.x = pos.at<float>(0);
            point.y = pos.at<float>(1);
            point.z = pos.at<float>(2);
            map_cloud_ptr->push_back(point);
        }
    }

    {
        // TODO -- CV notify spin thread to update, naive impl for now
        std::lock_guard<std::mutex> lock(mMutexCloud);
        mpCloudMap = map_cloud_ptr;
        mbRenderReady = true;
    }


}

void Map::NotifyMapUpdated() {
    mbMapUpdate = true;
    mcvMapUpdate.notify_all();
}

} //namespace ORB_SLAM
