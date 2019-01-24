//
// Created by kandithws on 22/1/2562.
//

#ifndef ORB_SLAM2_PCLVIEWER_H
#define ORB_SLAM2_PCLVIEWER_H
#include <pcl_slam_visualizer/pcl_slam_visualizer.h>
//#include <pcl_slam_visualizer/customized_pcl_visualizer/customized_pcl_visualizer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <memory>
#include <thread>
#include "Map.h"

#define DEFAULT_CLOUD_COLOR 0x0FFF
#define DEFAULT_CLOUD_COLOR_RED 0x0F00

namespace ORB_SLAM2 {
/**
 *  TODO: Implement PCLViewer as a Queueing basis
 */
class PCLViewer {
  typedef pcl::PointXYZRGBA PointT;
  //typedef pcl::PointXYZ PointT;
  public:
    PCLViewer(const std::string &window_name = "");
    ~PCLViewer();
    void setMapPtr(Map* pMap);
    void run();
    void shutdown();
  private:
    void renderPointCloudMap();
    // std::shared_ptr<pcl::visualization::PCLSLAMVisualizer> slam_visualizer_;
    std::shared_ptr<std::thread> spin_thread_;
    std::shared_ptr<std::thread> render_thread_; // render could come as a signal from LocalBA + Loop Closing
    int render_period_ms_ = 200; // 200 ms
    Map* map_;

    pcl::PointCloud<PointT>::Ptr map_cloud_;
    std::mutex map_mutex_;
    std::mutex map_cloud_mutex_;
    bool cloud_updated_ = false;
    std::string window_name_;
    bool is_shutdown_ = true;
    float cloud_color_;
    void spin();
    void spinRenderer();

};

}

#endif //ORB_SLAM2_PCLVIEWER_H
