#ifndef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H
#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H

//#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG

#include <object_initializer/BaseObjectInitializer.h>
#include <spdlog/spdlog.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "utils/PCLConverter.h"
#include "Cuboid.h"
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <utils/time.h>

namespace ORB_SLAM2{
class Cuboid;

class PointCloudObjectInitializer : public BaseObjectInitializer {
  public:
    PointCloudObjectInitializer();
    virtual void InitializeObjects(KeyFrame* pKeyframe, Map* pMap);

    Cuboid CuboidFromPointCloud(pcl::PointCloud<PCLConverter::PCLPointT>::Ptr cloud);

#ifdef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG
    pcl::PCDWriter mCloudDebugWriter;
#endif
  private:
    bool mbProject2d;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> mCloudSORFilter;
};
}


#endif //ORB_SLAM2_BASEOBJECTINITIALIZER_H