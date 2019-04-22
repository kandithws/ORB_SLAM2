//
// Created by kandithws on 9/3/2562.
//

#include "MapObject.h"

namespace ORB_SLAM2 {

uint32_t MapObject::nNextId=0;

MapObject::MapObject(Cuboid& cuboid, int label, ORB_SLAM2::KeyFrame *pRefKF, ORB_SLAM2::Map *pMap) :
        mnFirstKFid(pRefKF->mnId),
        mnFirstFrame(pRefKF->mnFrameId), mLabel(label), mpMap(pMap){

    mCuboid = new Cuboid();
    mCuboid->mPose = cuboid.mPose;
    mCuboid->mScale = cuboid.mScale;
    unique_lock<mutex> lock(mMutexPose);
    mTwo = Converter::toCvMat(cuboid.mPose);
    mScale = Converter::toCvMat(cuboid.mScale);
    // Avoid id conflict when create
    unique_lock<mutex> lock2(mpMap->mMutexObjectCreation);
    mnId=nNextId++;
}


MapObject::MapObject(const cv::Mat &pose, const cv::Mat &scale, int label, ORB_SLAM2::KeyFrame *pRefKF, ORB_SLAM2::Map *pMap) :
        mnFirstKFid(pRefKF->mnId),
        mnFirstFrame(pRefKF->mnFrameId), mLabel(label), mpMap(pMap){

    mCuboid = new Cuboid();
    mCuboid->mPose = Converter::toSE3Quat(pose);
    mCuboid->mScale = Converter::toVector3d(scale);
    unique_lock<mutex> lock(mMutexPose);
    mTwo = pose.clone();
    mScale = scale.clone();
    // Avoid id conflict when create
    unique_lock<mutex> lock2(mpMap->mMutexObjectCreation);
    mnId=nNextId++;
}

MapObject::~MapObject() {
    delete mCuboid;
}

void MapObject::SetCuboid(const Cuboid &cuboid) {
    unique_lock<mutex> lock(mMutexPose);
    mTwo = Converter::toCvMat(cuboid.mPose);
    mScale = Converter::toCvMat(cuboid.mScale);
    mCuboid->mPose = cuboid.mPose;
    mCuboid->mScale = cuboid.mScale;
}

void MapObject::GetCuboid(Cuboid &cuboid) {
    unique_lock<mutex> lock(mMutexPose);

    cuboid.mPose = mCuboid->mPose;
    cuboid.mScale = mCuboid->mScale;
}

const Cuboid* MapObject::GetCuboidPtr()
{
    unique_lock<mutex> lock(mMutexPose);
    return mCuboid;
}

cv::Mat MapObject::GetPose() {
    unique_lock<mutex> lock(mMutexPose);
    return mTwo.clone();
}

cv::Mat MapObject::GetScale() {
    unique_lock<mutex> lock(mMutexPose);
    return mScale.clone();
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

int MapObject::Observations(){
    unique_lock<mutex> lock(mMutexObservations);
    return nObs;
}


cv::Mat MapObject::GetCentroid() {
    unique_lock<mutex> lock(mMutexPose);
    return mTwo.rowRange(0, 3).col(3).clone();
}

cv::Point2f MapObject::GetProjectedCentroid(KeyFrame *pTargetKF) {
    cv::Mat Tcw = pTargetKF->GetPose();
    cv::Mat K = pTargetKF->mK;
    cv::Mat centroidHomo = K * (Tcw.rowRange(0, 3).colRange(0, 3) * GetCentroid() + Tcw.rowRange(0, 3).col(3));
    return cv::Point2f(centroidHomo.at<float>(0) / centroidHomo.at<float>(2),
                   centroidHomo.at<float>(1) / centroidHomo.at<float>(2));
}

bool MapObject::GetProjectedBoundingBox(ORB_SLAM2::KeyFrame *pTargetKF, cv::Rect &bb) {
    cv::Mat Tcw = pTargetKF->GetPose();
    unique_lock<mutex> lock(mMutexPose);

    auto bbox_eigen = ProjectOntoImageRect(Converter::toSE3Quat(Tcw),
            Converter::toMatrix3d(pTargetKF->mK));
    bb = cv::Rect(cv::Point2f(bbox_eigen[0], bbox_eigen[1]), cv::Point2f(bbox_eigen[2], bbox_eigen[3]));

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

Eigen::Vector4d MapObject::ProjectOntoImageRect(const SE3Quat& campose_cw, const Eigen::Matrix3d& Kalib) {
    Eigen::Matrix4d res = mCuboid->mPose.to_homogeneous_matrix();
    Eigen::Matrix3d scale_mat = mCuboid->mScale.asDiagonal();
    res.topLeftCorner<3, 3>() = res.topLeftCorner<3, 3>() * scale_mat;
    Eigen::Matrix3Xd corners_body;
    corners_body.resize(3, 8);
    corners_body << 1, 1, -1, -1, 1, 1, -1, -1,
            1, -1, -1, 1, 1, -1, -1, 1,
            -1, -1, -1, -1, 1, 1, 1, 1;
    Eigen::Matrix3Xd corners_3d_world = utils::homo_to_real_coord<double>(
            res * utils::real_to_homo_coord<double>(corners_body));

    // Eigen::Matrix3Xd corners_3d_world = compute3D_BoxCorner();
    Eigen::Matrix2Xd corner_2d = utils::homo_to_real_coord<double>(Kalib * utils::homo_to_real_coord<double>(
            campose_cw.to_homogeneous_matrix() * utils::real_to_homo_coord<double>(corners_3d_world)));
    Eigen::Vector2d bottomright = corner_2d.rowwise().maxCoeff(); // x y
    Eigen::Vector2d topleft = corner_2d.rowwise().minCoeff();
    return {topleft(0), topleft(1), bottomright(0), bottomright(1)};
}

bool MapObject::IsPositiveToKeyFrame(ORB_SLAM2::KeyFrame *pTargetKF) {
    //unique_lock<mutex> lock(mMutexPose);

    cv::Mat Tcw = pTargetKF->GetPose(); // 4x4 homogeneous TF

    cv::Mat centroid_w = GetCentroid(); // 3x1 x,y,z point
    cv::Mat centroid_c = Tcw.rowRange(0, 3).colRange(0, 3) * centroid_w + Tcw.rowRange(0, 3).col(3);
    return centroid_c.at<float>(2) > 0;
}

}