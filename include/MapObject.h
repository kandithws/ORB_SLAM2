//
// Created by kandithws on 6/3/2562.
//

#ifndef ORB_SLAM2_MAPOBJECT_H
#define ORB_SLAM2_MAPOBJECT_H

#include"KeyFrame.h"
#include "Map.h"
#include "dnn/BaseObjectDetector.h"
#include "Cuboid.h"
#include "Converter.h"

#include <boost/thread/locks.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <unordered_set>

namespace ORB_SLAM2 {
// TODO: Mimic pattern from MapPoint
class MapPoint;
class KeyFrame;
class Map;

class MapObject {
  public:
    MapObject(Cuboid& cuboid, int label, KeyFrame *pRefKF, Map *pMap); // Get "3D BB" from Object Initializer
    MapObject(const cv::Mat &pose, const cv::Mat &scale, int label, KeyFrame *pRefKF, Map *pMap);
    virtual ~MapObject();
    void SetCuboid(const Cuboid& cuboid);
    void GetCuboid(Cuboid& cuboid); // due to eigen

    const Cuboid* GetCuboidPtr();

    cv::Mat GetPose();
    cv::Mat GetScale();

    void AddObservation(KeyFrame *pKF, size_t idx);
    void EraseObservation(KeyFrame *pKF);

    void AddObservation(MapPoint *pMP);

    std::map<KeyFrame *, size_t> GetObservations();
    //std::vector<KeyFrame*> GetObservingKeyFrames();
    int Observations();

    std::vector<MapPoint*> GetMapPoints(); // TODO -- add counts

    cv::Mat GetCentroid();
    cv::Point2f GetProjectedCentroid(KeyFrame *pTargetKF);
    // if check centroid = true, will check if the centroid is in image, otherwise any corner
    bool GetProjectedBoundingBox(KeyFrame *pTargetKF, cv::Rect& bb, bool check_centroid=true);
    bool IsPositiveToKeyFrame(KeyFrame *pTargetKF);

  public:
    uint32_t mnId;
    static uint32_t nNextId;
    long unsigned int mnFirstKFid;
    long unsigned int mnFirstFrame;

    static std::mutex mGlobalMutex;

    const int mLabel;

  protected:
    //Copy from Cuboid
    Eigen::Vector4d ProjectOntoImageRect(const SE3Quat& campose_cw, const Eigen::Matrix3d& Kalib);

    // Bad flag (we do not currently erase MapObject from memory)
    bool mbBad;

    // std::mutex mMutexPose;
    std::mutex mMutexPose;
    Cuboid* mCuboid; // Due to eigen operator alignment
    cv::Mat mTwo; // 4x4 Homogeneous matrix
    cv::Mat mScale; // 3x1 half scale vector

    // Keyframes observing the object and associated index in keyframe
    std::mutex mMutexObservations;
    std::map<KeyFrame *, size_t> mObservations;
    int nObs = 0;

    // Object has_many map points, map point belongs to one object at a time
    std::set<MapPoint *> mspMPObservations;

    KeyFrame* mpRefKeyframe;
    Map *mpMap;
};
}

#endif //ORB_SLAM2_MAPOBJECT_H
