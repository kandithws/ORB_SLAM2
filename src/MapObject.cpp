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
    unique_lock<mutex> lock(mMutexPose);
    mCuboid->mScale = cuboid.mScale;
    mCuboid->mPose = cuboid.mPose;
}

void MapObject::GetCuboid(Cuboid &cuboid) {
    unique_lock<mutex> lock(mMutexPose);
    cuboid.mScale = mCuboid->mScale;
    cuboid.mPose = mCuboid->mPose;
}

cv::Mat MapObject::GetPose() {
    lock_guard<mutex> lock(mMutexPose);
    return Converter::toCvMat(mCuboid->mPose);
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


cv::Mat MapObject::GetCentroid() {
    unique_lock<mutex> lock(mMutexPose);
    Eigen::Vector3d translation = mCuboid->getTranslation();
    return Converter::toCvMat(translation);
}

cv::Point2f MapObject::GetProjectedCentroid(KeyFrame *pTargetKF) {
    cv::Mat Tcw = pTargetKF->GetPose();
    cv::Mat K = pTargetKF->mK;
    cv::Mat centroidHomo = K * (Tcw.rowRange(0, 3).colRange(0, 3) * GetCentroid() + Tcw.rowRange(0, 3).col(3));
    return cv::Point2f(centroidHomo.at<float>(0) / centroidHomo.at<float>(2),
                   centroidHomo.at<float>(1) / centroidHomo.at<float>(2));
}

bool MapObject::GetProjectedBoundingBox(ORB_SLAM2::KeyFrame *pTargetKF, cv::Rect &bb) {
    SPDLOG_INFO("I AM HERE 0");
    cv::Mat Tcw = pTargetKF->GetPose();
    SPDLOG_INFO("I AM HERE 1");
    unique_lock<mutex> lock(mMutexPose);
    SPDLOG_INFO("I AM HERE 1.5");
    auto bbox_eigen = mCuboid->projectOntoImageRect(Converter::toSE3Quat(Tcw),
            Converter::toMatrix3d(pTargetKF->mK));
    bb = cv::Rect(cv::Point2f(bbox_eigen[0], bbox_eigen[1]), cv::Point2f(bbox_eigen[2], bbox_eigen[3]));
    SPDLOG_INFO("I AM HERE 2");
    // Check corners visibility
    bool st = false;
    st |= pTargetKF->IsInImage(bb.tl().x, bb.tl().y);
    st |= pTargetKF->IsInImage(bb.tl().x + bb.width, bb.tl().y);
    st |= pTargetKF->IsInImage(bb.tl().x, bb.tl().y + bb.height);
    st |= pTargetKF->IsInImage(bb.br().x, bb.br().y);

    if(!st){
        SPDLOG_WARN("Object {}, bounding boxes is not in Keyframe {}", mnId, pTargetKF->mnId);
    }

    return st;
}

bool MapObject::IsPositiveToKeyFrame(ORB_SLAM2::KeyFrame *pTargetKF) {
    SPDLOG_INFO("I AM HERE 0");
    cv::Mat Tcw = pTargetKF->GetPose(); // 4x4 homogeneous TF
    SPDLOG_INFO("I AM HERE 0.5");
    cv::Mat centroid_w = GetCentroid(); // 3x1 x,y,z point
    SPDLOG_INFO("I AM HERE 1");
    SPDLOG_INFO("Tcw shape {}, {}", Tcw.rows, Tcw.cols);
    SPDLOG_INFO("centroid shape {}, {}", centroid_w.rows, centroid_w.cols);
    cv::Mat centroid_c = Tcw.rowRange(0, 3).colRange(0, 3) * centroid_w + Tcw.rowRange(0, 3).col(3);
    // HOMOGENEOUS pt tf
    SPDLOG_INFO("centroid in KF {} = [{}, {}, {}] ", pTargetKF->mnId,
            centroid_c.at<float>(0), centroid_c.at<float>(1), centroid_c.at<float>(2));
    return centroid_c.at<float>(2) > 0;
}

}