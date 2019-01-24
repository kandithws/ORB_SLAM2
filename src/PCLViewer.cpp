//
// Created by kandithws on 22/1/2562.
//

#include "PCLViewer.h"
#include <utils/smart_ptr_make_macro.h>
#include <chrono>
#include <include/spdlog/spdlog.h>

namespace ORB_SLAM2 {
PCLViewer::PCLViewer(const std::string &window_name) {
    map_cloud_ = BOOST_MAKE_SHARED(pcl::PointCloud<PointT>);
    window_name_ = window_name;
    map_cloud_;
    //pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb_handler(map_cloud_);
    //slam_visualizer_->addPointCloud(map_cloud_, rgb_handler,"map_cloud");
    //slam_visualizer_->addPointCloud(map_cloud_,"map_cloud");
    //slam_visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map_cloud");
    uint32_t rgb = DEFAULT_CLOUD_COLOR;
    cloud_color_ = *reinterpret_cast<float*>(&rgb);
}

PCLViewer::~PCLViewer() {
    shutdown();
}

void PCLViewer::setMapPtr(Map *pMap) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = pMap;
    //map_ = std::shared_ptr<Map>(pMap);
}



void PCLViewer::run() {
    SPDLOG_INFO("START RUNNING");
    is_shutdown_ = false;
    spin_thread_ = std::make_shared<std::thread>(std::bind(&PCLViewer::spin, this));
    render_thread_ = std::make_shared<std::thread>(std::bind(&PCLViewer::spinRenderer, this));
}

void PCLViewer::shutdown(){
    if(!is_shutdown_){
        is_shutdown_ = true;
        spin_thread_->join();
        render_thread_->join();
        spin_thread_.reset();
        render_thread_.reset();
    }
}

void PCLViewer::renderPointCloudMap() {
    // TODO -- add Rendering period, and move to PCL cloud to global context
    {
        std::lock_guard<std::mutex> lock(map_cloud_mutex_);
        if(cloud_updated_)
            return;
    }

    pcl::PointCloud<PointT>::Ptr map_cloud_ptr = BOOST_MAKE_SHARED(pcl::PointCloud<PointT>);
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        const std::vector<MapPoint*> &map_points = map_->GetAllMapPoints();
        if(map_points.empty())
            return;

        const vector<MapPoint*> &ref_map_points = map_->GetReferenceMapPoints();
        std::set<MapPoint*> set_ref_map_points(ref_map_points.begin(), ref_map_points.end());

        for(size_t i=0, iend=map_points.size(); i < iend; i++){

            if(map_points[i]->isBad() || set_ref_map_points.count(map_points[i]))
                continue;

            cv::Mat pos = map_points[i]->GetWorldPos();
            PointT point;
            point.x = pos.at<float>(0);
            point.y = pos.at<float>(1);
            point.z = pos.at<float>(2);
            map_cloud_ptr->push_back(point);
        }

        for (std::set<MapPoint*>::iterator sit=set_ref_map_points.begin(),
                     send=set_ref_map_points.end(); sit != send; sit++){

            if((*sit)->isBad())
                continue;

            cv::Mat pos = (*sit)->GetWorldPos();
            PointT point;
            point.x = pos.at<float>(0);
            point.y = pos.at<float>(1);
            point.z = pos.at<float>(2);
            map_cloud_ptr->push_back(point);
        }
    }

    {
        // TODO -- CV notify spin thread to update, naive impl for now
        std::lock_guard<std::mutex> lock(map_cloud_mutex_);
        map_cloud_ = map_cloud_ptr;
        cloud_updated_ = true;

    }
}

void PCLViewer::spin() {
    SPDLOG_INFO("START PCLViewer SPIN THREAD");
    // Note: Visualizer must be in the same thread with spin()!
    boost::shared_ptr<pcl::visualization::PCLSLAMVisualizer> slam_visualizer= BOOST_MAKE_SHARED(pcl::visualization::PCLSLAMVisualizer, window_name_);
    slam_visualizer->setSize(1280,720);
    slam_visualizer->setBackgroundColor (0, 0, 0);
    slam_visualizer->initCameraParameters();
    slam_visualizer->setCameraPosition(0,0,-0.5,0,-1,0,0);
    slam_visualizer->addCoordinateSystem(0.5);
    slam_visualizer->addPointCloud(map_cloud_, "map_cloud");
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,1,"map_cloud");
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "map_cloud");
    // Force white color for now, will use other fields to store meta data
    slam_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,1,1,1, "map_cloud");

    while(!slam_visualizer->wasStopped() && !is_shutdown_){

        //slam_visualizer_->spinOnce(1);
        slam_visualizer->spinOnce(100);
        if(cloud_updated_){
            std::lock_guard<std::mutex> lock(map_cloud_mutex_);
            bool st  = slam_visualizer->updatePointCloud(map_cloud_, "map_cloud");
            //slam_visualizer_->showCloud();
            cloud_updated_ = false;
        }
        // TODO -- Add camera actor pose update!
        // TODO -- Add 3D Bounding Boxes object update
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void PCLViewer::spinRenderer() {
    while(!is_shutdown_){
        renderPointCloudMap();
        std::this_thread::sleep_for(std::chrono::milliseconds(render_period_ms_));
    }
}
}