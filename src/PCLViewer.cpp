//
// Created by kandithws on 22/1/2562.
//

#include "PCLViewer.h"
#include <utils/smart_ptr_make_macro.h>
#include <chrono>
#include <include/spdlog/spdlog.h>

namespace ORB_SLAM2 {
PCLViewer::PCLViewer(Map *pMap, const std::string &window_name) {
    map_ = pMap;
    window_name_ = window_name;
}

PCLViewer::~PCLViewer() {
    shutdown();
}


void PCLViewer::run() {
    SPDLOG_DEBUG("START RUNNING");
    is_shutdown_ = false;
    spin_thread_ = std::make_shared<std::thread>(std::bind(&PCLViewer::spin, this));
}

void PCLViewer::shutdown(){
    if(!is_shutdown_){
        is_shutdown_ = true;
        if(spin_thread_){
            spin_thread_->join();
            spin_thread_.reset();
        }
    }
}


void PCLViewer::spin() {
    SPDLOG_DEBUG("START PCLViewer SPIN THREAD");
    // Note: Visualizer must be in the same thread with spin()!
    boost::shared_ptr<pcl::visualization::PCLSLAMVisualizer> slam_visualizer= BOOST_MAKE_SHARED(pcl::visualization::PCLSLAMVisualizer, window_name_);
    slam_visualizer->setSize(1280,720);
    slam_visualizer->setBackgroundColor (0, 0, 0);
    slam_visualizer->initCameraParameters();
    slam_visualizer->setCameraPosition(0,0,-0.5,0,-1,0,0);
    slam_visualizer->addCoordinateSystem(0.5);
    slam_visualizer->addPointCloud(map_->GetCloudPtr(), "map_cloud");
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,"map_cloud");
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map_cloud");
    // Force white color for now, will use other fields to store meta data
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1, "map_cloud");

    while(!slam_visualizer->wasStopped() && !is_shutdown_){

        slam_visualizer->spinOnce(100);

        if(map_->mbRenderReady){
            std::lock_guard<std::mutex> lock(map_->mMutexCloud);
            slam_visualizer->updatePointCloud(map_->mpCloudMap, "map_cloud");
            map_->mbRenderReady = false;
        }
        // TODO -- Add camera actor pose update!
        // TODO -- Add 3D Bounding Boxes object update
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

}