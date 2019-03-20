//
// Created by kandithws on 9/3/2562.
//

#include "MapObject.h"

namespace ORB_SLAM2 {

uint32_t MapObject::nNextId=0;

MapObject::MapObject(const cv::Mat &Cuboid, ORB_SLAM2::KeyFrame *pRefKF, ORB_SLAM2::Map *pMap) :
        mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), mpMap(pMap)
{
    Cuboid.copyTo(mCuboid);

    // Avoid id conflict when create
    unique_lock<mutex> lock(mpMap->mMutexObjectCreation);
    mnId=nNextId++;
}

void MapObject::SetWorldCuboid(const cv::Mat &Cuboid) {
    unique_lock<mutex> lock(mMutexCuboid);
    Cuboid.copyTo(mCuboid);
}

cv::Mat MapObject::GetWorldCuboid() {
    unique_lock<mutex> lock(mMutexCuboid);
    return mCuboid.clone();
}

}