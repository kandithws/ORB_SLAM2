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
    MapObject(Cuboid& cuboid, int label, KeyFrame *pRefKF, Map *pMap); // Get "3D BB" from Object Initializer

    void SetCuboid(Cuboid& cuboid);
    void GetCuboid(Cuboid& cuboid); // due to eigen

    void AddObservation(KeyFrame *pKF, size_t idx);
    void EraseObservation(KeyFrame *pKF);

    std::map<KeyFrame *, size_t> GetObservations();
    int Observations();

    int GetIndexInKeyFrame(KeyFrame *pKF);
    bool IsInKeyFrame(KeyFrame *pKF);
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

    const int mLabel;

  protected:
    // Bad flag (we do not currently erase MapObject from memory)
    bool mbBad;

    std::mutex mMutexCuboid;
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
