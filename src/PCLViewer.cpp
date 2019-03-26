//
// Created by kandithws on 22/1/2562.
//

#include "PCLViewer.h"
#include <utils/smart_ptr_make_macro.h>
#include <chrono>
#include <include/spdlog/spdlog.h>
#include <opencv2/core/eigen.hpp>

namespace ORB_SLAM2 {
PCLViewer::PCLViewer(Map *pMap, const std::string &window_name) {
    map_ = pMap;
    window_name_ = window_name;
    tracking_render_period_ = 1.0 / Config::getInstance().cameraParams().fps;

}

PCLViewer::~PCLViewer() {
    shutdown();
}

void PCLViewer::setCurrentCameraPose(const cv::Mat &pose) {
    std::lock_guard<std::mutex> lock(current_cam_pose_mutex_);
    current_cam_pose_ = pose.clone();
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

void PCLViewer::getCurrentCamPose(Eigen::Affine3f& pose) {
    if(!current_cam_pose_.empty()){
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            std::lock_guard<std::mutex> lock(current_cam_pose_mutex_);
            Rwc = current_cam_pose_.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*current_cam_pose_.rowRange(0,3).col(3);
        }
        pose.matrix() << Rwc.at<float>(0,0) , Rwc.at<float>(0,1), Rwc.at<float>(0,2), twc.at<float>(0),
                Rwc.at<float>(1,0) , Rwc.at<float>(1,1), Rwc.at<float>(1,2), twc.at<float>(1),
                Rwc.at<float>(2,0) , Rwc.at<float>(2,1), Rwc.at<float>(2,2), twc.at<float>(2),
                0, 0, 0, 1;
    }
    else {
        pose.setIdentity();
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
    PointT origin;
    slam_visualizer->addText3D("world", origin, 0.05);
    slam_visualizer->addCoordinateSystem(0.25, "current_pose");
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> init_rgb_handler(map_->GetCloudPtr());
    slam_visualizer->addPointCloud<PointT>(map_->GetCloudPtr(), init_rgb_handler, "map_cloud");
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,"map_cloud");
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map_cloud");
    // Force white color for now, will use other fields to store meta data
    //slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1, "map_cloud");
    auto last_tracking_render_time = utils::time::time_now();
    while(!slam_visualizer->wasStopped() && !is_shutdown_){

        slam_visualizer->spinOnce(10);

        if(map_->mbRenderReady){
            std::lock_guard<std::mutex> lock(map_->mMutexCloud);
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(map_->mpCloudMap);
            slam_visualizer->updatePointCloud<PointT>(map_->mpCloudMap, rgb_handler, "map_cloud");
            map_->mbRenderReady = false;
        }

        if(utils::time::time_diff_from_now_second(last_tracking_render_time) > tracking_render_period_){
            Eigen::Affine3f pose;
            getCurrentCamPose(pose);
            slam_visualizer->updateCoordinateSystemPose("current_pose", pose);
            //SPDLOG_DEBUG("Render Pose {}, {}, {}", pose.translation().x(), pose.translation().y(), pose.translation().z());
            last_tracking_render_time = utils::time::time_now();
        }
        // TODO -- Add camera actor pose update!
        // TODO -- Add 3D Bounding Boxes object update
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

}
