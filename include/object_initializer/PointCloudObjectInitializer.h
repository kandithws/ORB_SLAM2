#ifndef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H
#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H

#include <object_initializer/BaseObjectInitializer.h>
#include <spdlog/spdlog.h>

namespace ORB_SLAM2{
class PointCloudObjectInitializer : public BaseObjectInitializer {
  public:
    PointCloudObjectInitializer();
    virtual void InitializeObjects(KeyFrame* pKeyframe, Map* pMap);

    bool MatchObject();
};
}


#endif //ORB_SLAM2_BASEOBJECTINITIALIZER_H