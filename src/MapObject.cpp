//
// Created by kandithws on 9/3/2562.
//

#include "MapObject.h"

namespace ORB_SLAM2 {

uint32_t MapObject::nNextId=0;

MapObject::MapObject(Cuboid& cuboid, int label, ORB_SLAM2::KeyFrame *pRefKF, ORB_SLAM2::Map *pMap) :
        mLabel(label), mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), mpMap(pMap){

    mCuboid = new Cuboid();
    mCuboid->mPose = cuboid.mPose;
    mCuboid->mScale = cuboid.mScale;

    // Avoid id conflict when create
    unique_lock<mutex> lock(mpMap->mMutexObjectCreation);
    mnId=nNextId++;
}

void MapObject::SetCuboid(Cuboid &cuboid) {
    lock_guard<mutex> lock(mMutexCuboid);
    mCuboid->mScale = cuboid.mScale;
    mCuboid->mPose = cuboid.mPose;
}

void MapObject::GetCuboid(Cuboid &cuboid) {
    lock_guard<mutex> lock(mMutexCuboid);
    cuboid.mScale = mCuboid->mScale;
    cuboid.mPose = mCuboid->mPose;
}

void MapObject::AddObservation(KeyFrame* pKF, size_t idx)
{
    unique_lock<mutex> lock(mMutexObservations);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;

    nObs++;
//    if(pKF->mvuRight[idx]>=0)
//        nObs+=2;
//    else
//        nObs++;
}

void MapObject::EraseObservation(ORB_SLAM2::KeyFrame *pKF) {

    // TODO Implement
    throw std::runtime_error("NOT IMPLEMENTED");
}

map<KeyFrame*, size_t> MapObject::GetObservations()
{
    unique_lock<mutex> lock(mMutexObservations);
    return mObservations;
}

int MapObject::Observations()
{
    unique_lock<mutex> lock(mMutexObservations);
    return nObs;
}


}