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


bool KFIdComapre::operator ()(const KeyFrame* kfleft,const KeyFrame* kfright) const {
    return kfleft->mnId < kfright->mnId;
}

void Map::UpdateScale(const double &scale) {
    unique_lock<mutex> lock(mMutexMapUpdate);
    for(std::set<KeyFrame*,KFIdComapre>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++){
        KeyFrame* pKF = *sit;
        cv::Mat Tcw = pKF->GetPose();
        cv::Mat tcw = Tcw.rowRange(0,3).col(3)*scale;
        tcw.copyTo(Tcw.rowRange(0,3).col(3));
        pKF->SetPose(Tcw);
    }
    for(std::set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++){
        MapPoint* pMP = *sit;
        //pMP->SetWorldPos(pMP->GetWorldPos()*scale);
        pMP->UpdateScale(scale);
    }
    SPDLOG_DEBUG("Map scale updated");
}

//---------------------------------------

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    auto st = mspMapPoints.insert(pMP);
    // mmapMapPoints[pMP->mnId] = pMP;
    if(st.second){
        // Signal Add new Pointcloud
    }
}

void Map::AddMapObject(MapObject *pMO) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.insert(pMO);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    // mmapMapPoints[pMP->mnId] = static_cast<MapPoint*>(NULL);
    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseMapObject(MapObject *pMO) {
    unique_lock<mutex> lock(mMutexMap);
    mspMapObjects.erase(pMO);
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
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

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapObject *> Map::GetAllMapObjects(){
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapObject* >(mspMapObjects.begin(),mspMapObjects.end());
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

vector<MapPoint*> Map::GetReferenceMapPoints()
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
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
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
    //mtPointcloudRendererThread = std::make_shared<std::thread>(std::bind(&Map::RenderPointCloudThread, this));
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
        const std::vector<MapPoint*> &map_points = GetAllMapPoints();
        if(map_points.empty())
            return;

        const vector<MapPoint*> &ref_map_points = GetReferenceMapPoints();
        std::set<MapPoint*> set_ref_map_points(ref_map_points.begin(), ref_map_points.end());
        for(size_t i=0, iend=map_points.size(); i < iend; i++){

            if(map_points[i]->isBad() || set_ref_map_points.count(map_points[i])) // if map point is bad or it is a ref
                continue;

            map_cloud_ptr->push_back(map_points[i]->GetPCLPoint());
        }

        for (std::set<MapPoint*>::iterator sit=set_ref_map_points.begin(),
                     send=set_ref_map_points.end(); sit != send; sit++){

            if((*sit)->isBad())
                continue;

            map_cloud_ptr->push_back((*sit)->GetPCLPoint());
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
