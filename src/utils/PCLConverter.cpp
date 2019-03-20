//
// Created by kandithws on 20/3/2562.
//

#include "utils/PCLConverter.h"

namespace ORB_SLAM2 {

pcl::PointCloud<PCLConverter::PCLPointT>::Ptr PCLConverter::toPointCloud(const vector<MapPoint *> &vMP) {
    pcl::PointCloud<PCLConverter::PCLPointT>::Ptr cloud =
            pcl::PointCloud<PCLConverter::PCLPointT>::Ptr(new pcl::PointCloud<PCLConverter::PCLPointT>());

    for (const auto &pMP : vMP){
        cloud->push_back(pMP->GetPCLPoint());
    }

    return cloud;
}

}
