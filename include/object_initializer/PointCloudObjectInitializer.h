#ifndef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H
#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H

#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG

#include <object_initializer/BaseObjectInitializer.h>
#include <spdlog/spdlog.h>
#include <pcl/io/pcd_io.h>

namespace ORB_SLAM2{
class PointCloudObjectInitializer : public BaseObjectInitializer {
  public:
    PointCloudObjectInitializer();
    virtual void InitializeObjects(KeyFrame* pKeyframe, Map* pMap);

    bool MatchObject();

#ifdef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG
    pcl::PCDWriter mCloudDebugWriter;
#endif
};
}


#endif //ORB_SLAM2_BASEOBJECTINITIALIZER_H