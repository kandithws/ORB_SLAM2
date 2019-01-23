//
// Created by kandithws on 22/1/2562.
//

#include "PCLViewer.h"
#include <utils/smart_ptr_make_macro.h>
#include <chrono>


namespace ORB_SLAM2 {
PCLViewer::PCLViewer(const std::string &window_name) {
    slam_visualizer_ = STD_MAKE_SHARED(pcl::visualization::PCLSLAMVisualizer, window_name);
    slam_visualizer_->setBackgroundColor (0, 0, 0);
    slam_visualizer_->initCameraParameters();
    slam_visualizer_->addCoordinateSystem(1.0);
    map_cloud_ = BOOST_MAKE_SHARED(pcl::PointCloud<PointT>);
    slam_visualizer_->addPointCloud(map_cloud_, "map_cloud");
    slam_visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "map_cloud");
    uint32_t rgb = DEFAULT_CLOUD_COLOR;
    cloud_color_ = *reinterpret_cast<float*>(&rgb);
}

void PCLViewer::setMapPtr(Map *pMap) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_ = std::shared_ptr<Map>(pMap);
}



void PCLViewer::run() {
    is_shutdown_ = false;
    if (slam_visualizer_->wasStopped())
        slam_visualizer_->resetStoppedFlag ();
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
    if(!slam_visualizer_->wasStopped()){
        slam_visualizer_->close();
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
        for (auto &mp : map_points){
            if(mp->isBad() || set_ref_map_points.count(mp))
                continue;
            cv::Mat pos = mp->GetWorldPos();
            PointT point;
            point.x = pos.at<float>(0);
            point.y = pos.at<float>(0);
            point.z = pos.at<float>(0);
            point.rgb = cloud_color_;
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
    while(!slam_visualizer_->wasStopped() && !is_shutdown_){
        slam_visualizer_->spinOnce(100);
        {
            std::lock_guard<std::mutex> lock(map_cloud_mutex_);
            if(cloud_updated_){
                slam_visualizer_->updatePointCloud(map_cloud_, "map_cloud");
                cloud_updated_ = false;
            }
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