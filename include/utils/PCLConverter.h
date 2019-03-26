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
    static pcl::PointCloud<PCLPointT>::Ptr toPointCloud(const std::vector<MapPoint*>& vMp);

    template <typename T>
    static void filterVector(const std::vector<int>& vIndices, const std::vector<T>& vIn, std::vector<T>& vOut){
        vOut.reserve(vIndices.size());
        for (const int &idx : vIndices){
            vOut.push_back(vIn[idx]);
        }
    }
};
}
#endif //ORB_SLAM2_PCLCONVERTER_H
