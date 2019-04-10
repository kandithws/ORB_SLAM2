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

namespace ORB_SLAM2 {
// TODO: Mimic pattern from MapPoint
class MapPoint;
class KeyFrame;
class Map;

class MapObject {
  public:
    MapObject(Cuboid& cuboid, int label, KeyFrame *pRefKF, Map *pMap); // Get "3D BB" from Object Initializer

    void SetCuboid(Cuboid& cuboid);
    void GetCuboid(Cuboid& cuboid); // due to eigen

    cv::Mat GetPose();

    void AddObservation(KeyFrame *pKF, size_t idx);
    void EraseObservation(KeyFrame *pKF);

    std::map<KeyFrame *, size_t> GetObservations();
    int Observations();

    cv::Mat GetCentroid();
    cv::Point2f GetProjectedCentroid(KeyFrame *pTargetKF);
    bool GetProjectedBoundingBox(KeyFrame *pTargetKF, cv::Rect& bb);
    bool IsPositiveToKeyFrame(KeyFrame *pTargetKF);

  public:
    uint32_t mnId;
    static uint32_t nNextId;
    long unsigned int mnFirstKFid;
    long unsigned int mnFirstFrame;

    static std::mutex mGlobalMutex;

    const int mLabel;

  protected:
    // Bad flag (we do not currently erase MapObject from memory)
    bool mbBad;

    std::mutex mMutexPose;
    Cuboid* mCuboid; // Due to eigen operator alignment

    // Keyframes observing the object and associated index in keyframe
    std::mutex mMutexObservations;
    std::map<KeyFrame *, size_t> mObservations;
    int nObs = 0;

    // Object has_many map points, map point belongs to one object at a time
    std::set<MapPoint *> mspMapPoints;

    KeyFrame* mpRefKeyframe;
    Map *mpMap;
};
}

#endif //ORB_SLAM2_MAPOBJECT_H
