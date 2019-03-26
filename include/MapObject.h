//
// Created by kandithws on 6/3/2562.
//

#ifndef ORB_SLAM2_MAPOBJECT_H
#define ORB_SLAM2_MAPOBJECT_H

#include"KeyFrame.h"
#include "Map.h"
#include "dnn/BaseObjectDetector.h"
#include "Cuboid.h"

namespace ORB_SLAM2 {
// TODO: Mimic pattern from MapPoint
class MapPoint;
class KeyFrame;
class Map;

class MapObject {
  public:
    MapObject(const Cuboid& cuboid, KeyFrame *pRefKF, Map *pMap); // Get "3D BB" from Object Initializer

    void SetCuboid(const Cuboid& cuboid);
    Cuboid GetCuboid();

//    void AddObservation(KeyFrame *pKF, size_t idx);
//    void EraseObservation(KeyFrame *pKF);
//
//    void AddMapPoint(MapPoint *pMp);
//    void EraseMapPoint(MapPoint *pMp);
//
//    int GetIndexInKeyFrame(KeyFrame *pKF);
//    bool IsInKeyFrame(KeyFrame *pKF);

    // Add debug pointcloud -- save object pcd

  public:
    uint32_t mnId;
    static uint32_t nNextId;
    long unsigned int mnFirstKFid;
    long unsigned int mnFirstFrame;

    static std::mutex mGlobalMutex;

  protected:
    // Bad flag (we do not currently erase MapObject from memory)
    bool mbBad;

    std::mutex mMutexCuboid;
    Cuboid mCuboid;

    // Keyframes observing the object and associated index in keyframe
    std::map<KeyFrame *, size_t> mObservations;

    // Object has_many map points, map point belongs to one object at a time
    std::set<MapPoint *> mspMapPoints;

    KeyFrame* mpRefKeyframe;
    Map *mpMap;
};
}

#endif //ORB_SLAM2_MAPOBJECT_H
