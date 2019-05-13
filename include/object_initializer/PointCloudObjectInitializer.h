#ifndef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H
#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_H

//#define ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG

#include <object_initializer/IObjectInitializer.h>
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

class PointCloudObjectInitializer : public IObjectInitializer {
  public:
    PointCloudObjectInitializer();
    void InitializeObjects(KeyFrame* pKeyframe, Map* pMap);

#ifdef ORB_SLAM2_POINTCLOUDOBJECTINITIALIZER_DEBUG
    pcl::PCDWriter mCloudDebugWriter;
#endif
  private:
    bool mbProject2d;
    bool mbUseMask;
    bool mbUseStatRemoveOutlier;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBL> mCloudSORFilter;
    inline double Point2DDistance(const cv::Point2f& p1, const cv::Point2f& p2){
        return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    Cuboid CuboidFromPointCloud(pcl::PointCloud<PCLConverter::PCLPointT>::Ptr cloud);
    void FilterMapPointsSOR(const std::vector<MapPoint *> &vObjMapPoints,
            std::vector<MapPoint *> &vFilteredMapPoints, bool bProject2D = false, KeyFrame* pKeyFrame= static_cast<KeyFrame*>(NULL));


};
}


#endif //ORB_SLAM2_BASEOBJECTINITIALIZER_H