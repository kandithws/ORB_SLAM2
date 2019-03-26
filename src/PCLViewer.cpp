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

void PCLViewer::getObjectCubeData(MapObject *pObj, Eigen::Vector3f &t, Eigen::Quaternionf &q,
                                  Eigen::Vector3f &scale, Eigen::Affine3f &tf) {
    SPDLOG_INFO("Test0");
    if(!pObj)
        SPDLOG_WARN("POINTER NULL");

    Cuboid cuboid; pObj->GetCuboid(cuboid);
    SPDLOG_INFO("Test1");
    t = cuboid.mPose.translation().cast<float>();
    q = cuboid.mPose.rotation().cast<float>();
    scale = cuboid.mScale.cast<float>();
    SPDLOG_INFO("Test2");
    Eigen::Matrix4f tf_mat(Eigen::Matrix4f::Identity());
    tf_mat.block<3,3>(0,0) = q.toRotationMatrix();
    tf_mat.block<3,1>(0,3) = t;
    tf.matrix() = tf_mat;
    SPDLOG_INFO("Test3");
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
    auto last_object_render_time = utils::time::time_now();
    unsigned long int last_object_id = 0;
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

        if (utils::time::time_diff_from_now_second(last_object_render_time) > object_render_period_){
            std::vector<MapObject* > objects = map_->GetAllMapObjects();

            for (MapObject* pObj : objects){
                Eigen::Vector3f t; Eigen::Quaternionf q;
                Eigen::Vector3f scale; Eigen::Affine3f tf;
                SPDLOG_INFO("HELLO WORLD!");
                getObjectCubeData(pObj, t, q, scale, tf);
                SPDLOG_INFO("HELLO WORLD1");
                std::string cube_label_str = "obj_cube" + std::to_string(pObj->mnId);
                std::string tf_label_str = "obj_tf" + std::to_string(pObj->mnId);

                if(slam_visualizer->updateCube(t, q, scale(0), scale(1), scale(2), cube_label_str)){
                    SPDLOG_INFO("HELLO WORLD5");
                    slam_visualizer->updateCoordinateSystemPose(tf_label_str, tf);
                }
                else{

                    slam_visualizer->addCube(t,q, scale(0), scale(1), scale(2), cube_label_str);
                    SPDLOG_INFO("HELLO WORLD2");
                    slam_visualizer->setShapeRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                            cube_label_str);
                    slam_visualizer->setShapeRenderingProperties(
                            pcl::visualization::PCL_VISUALIZER_COLOR,
                            0.0,
                            1.0,
                            0.0,
                            cube_label_str);
                    SPDLOG_INFO("HELLO WORLD3");
                    slam_visualizer->addCoordinateSystem(0.15, tf, tf_label_str);
                    SPDLOG_INFO("HELLO WORLD4");
                }


                //if(slam_visualizer->updateCube()){
                    //slam_visualizer->addCube();
                //}

            }
            last_object_render_time = utils::time::time_now();
        }

        // TODO -- Add 3D Bounding Boxes object update
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

}
