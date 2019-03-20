//
// Created by kandithws on 6/3/2562.
//

#ifndef ORB_SLAM2_PCLCONVERTER_H
#define ORB_SLAM2_PCLCONVERTER_H

#include "../MapPoint.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ORB_SLAM2 {
class PCLConverter{
  public:
    typedef pcl::PointXYZRGBL PCLPointT;
    static pcl::PointCloud<PCLPointT> toPointCloud(const std::vector<MapPoint>& vMp);
};
}
#endif //ORB_SLAM2_PCLCONVERTER_H
