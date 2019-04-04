//
// Created by kandithws on 22/1/2562.
//

#ifndef ORB_SLAM2_PCLVIEWER_H
#define ORB_SLAM2_PCLVIEWER_H
#include <pcl_slam_visualizer/pcl_slam_visualizer.h>
#include <utils/PCLConverter.h>
//#include <pcl_slam_visualizer/customized_pcl_visualizer/customized_pcl_visualizer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <include/utils/Config.h>
#include <include/utils/time.h>
#include <string>
#include <memory>
#include <thread>
#include "Map.h"
#include "MapObject.h"

#define DEFAULT_CLOUD_COLOR 0x0FFF
#define DEFAULT_CLOUD_COLOR_RED 0x0F00

namespace ORB_SLAM2 {
/**
 *  TODO: Implement PCLViewer as a Queueing basis
 */
class PCLViewer {
  typedef Map::PCLPointT PointT;
  //typedef pcl::PointXYZ PointT;
  public:
    PCLViewer(Map *pMap, const std::string &window_name = "");
    ~PCLViewer();
    void setCurrentCameraPose(const cv::Mat& pose);
    void run();
    void shutdown();
  private:
    cv::Mat current_cam_pose_;
    std::mutex current_cam_pose_mutex_;
    double tracking_render_period_ = 1.0/10.0;
    double object_render_period_ = 1.0;
    void getCurrentCamPose(Eigen::Affine3f& pose);
    void getObjectCubeData(MapObject* pObj,
            Eigen::Vector3f& t,
            Eigen::Quaternionf& q,
            Eigen::Vector3f& scale,
            Eigen::Affine3f& tf);

    std::shared_ptr<std::thread> spin_thread_;
    Map* map_;
    std::string window_name_;
    bool is_shutdown_ = true;
    void spin();
    void spinRenderer();

};

}

#endif //ORB_SLAM2_PCLVIEWER_H
