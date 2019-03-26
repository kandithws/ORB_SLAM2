//
// Created by kandithws on 9/3/2562.
//

#include "MapObject.h"

namespace ORB_SLAM2 {

uint32_t MapObject::nNextId=0;

MapObject::MapObject(const Cuboid& cuboid, ORB_SLAM2::KeyFrame *pRefKF, ORB_SLAM2::Map *pMap) :
        mCuboid(cuboid), mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), mpMap(pMap){

    // Avoid id conflict when create
    unique_lock<mutex> lock(mpMap->mMutexObjectCreation);
    mnId=nNextId++;
}

void MapObject::SetCuboid(const Cuboid &cuboid) {
    unique_lock<mutex> lock(mMutexCuboid);
    mCuboid = cuboid;
}

Cuboid MapObject::GetCuboid() {
    unique_lock<mutex> lock(mMutexCuboid);
    return mCuboid;
}


}