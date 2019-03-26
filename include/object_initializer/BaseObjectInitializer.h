//
// Created by kandithws on 20/2/2562.
//

#ifndef ORB_SLAM2_BASEOBJECTINITIALIZER_H
#define ORB_SLAM2_BASEOBJECTINITIALIZER_H
#include "KeyFrame.h"
#include "Map.h"
#include "MapObject.h"

namespace ORB_SLAM2 {
class BaseObjectInitializer {
  public:
    BaseObjectInitializer() = default;

    virtual void InitializeObjects(KeyFrame* pKeyframe, Map* pMap) = 0;
};
}

#endif //ORB_SLAM2_BASEOBJECTINITIALIZER_H
