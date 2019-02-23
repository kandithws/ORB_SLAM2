#ifndef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H
#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H

#include <object_initializer/BaseObjectInitializer.h>
namespace ORB_SLAM2{
class PointCloudObjectInitializer : public BaseObjectInitializer {
  public:
    PointCloudObjectInitializer() = default;
    void InitializeObjects(KeyFrame* pKeyframe, Map* pMap) {}
};
}


#endif //ORB_SLAM2_BASEOBJECTINITIALIZER_H