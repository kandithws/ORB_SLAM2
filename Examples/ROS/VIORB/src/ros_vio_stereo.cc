/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <System.h>
#include <utils/vector_utils.h>
#include "MsgSync/StereoMsgSynchronizer.h"
#include <imu/IMUData.h>
#include <utils/Config.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

using namespace std;
/**
 * @ A Node to Run VIORB with Stereo/RGB-D settings, only support 1 IMU for now
 * */

int main(int argc, char **argv) {
    ros::init(argc, argv, "VI_ORBSLAM");
    //ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    /*
    if (argc != 3) {
      cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;
      ros::shutdown();
      return 1;
    }
     */
    std::string vocab_path, config_path, bagfile_path, sensor_type_str;
    bool realtime_mode = true;

    pnh.param<std::string>("vocab_path", vocab_path, "Vocabulary/ORBvoc.bin");
    pnh.param<std::string>("config_path", config_path, "config/euroc.yaml");
    pnh.param<std::string>("sensor_type", sensor_type_str, "STEREO");

    ORB_SLAM2::System::eSensor sensor_type;
    if ( sensor_type_str == "STEREO" ){
        sensor_type = ORB_SLAM2::System::STEREO;
    }
    else if (sensor_type_str == "RGBD"){
        sensor_type = ORB_SLAM2::System::RGBD;
    }
    else {
        ROS_FATAL("Unexpected Sensor type %s", sensor_type_str.c_str());
        exit(-1);
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], sensor_type, true);



    // Add this section to directly config from launch files instead of config
    if (!pnh.getParam("realtime_mode", realtime_mode)) {
        ROS_WARN("Realtime mode config is not given by ROS param, reading through VI-ORB Config");
        realtime_mode = ORB_SLAM2::Config::getInstance().SystemParams().real_time;

    } else {
        // Overiding VI-ORB file config with ROS Config (to use with internal SLAM classes)
        ORB_SLAM2::Config::getInstance().SetRealTimeFlag(realtime_mode);
    }

    if (!pnh.getParam("bagfile_path", bagfile_path) && !realtime_mode) {
        bagfile_path = ORB_SLAM2::Config::getInstance().RuntimeParams().bagfile;
    }

    /**
     * @brief added data sync
     */
    ROS_INFO("INITIALIZE MSG SYNC");
    double imageMsgDelaySec = ORB_SLAM2::Config::getInstance().RuntimeParams().image_delay_to_imu;
    ORBVIO::StereoMsgSynchronizer msgsync(imageMsgDelaySec, imageMsgDelaySec, realtime_mode);
    ros::Subscriber imagesub;
    ros::Subscriber imusub;
    ROS_INFO("INITIALIZED !");
    if (realtime_mode) {
        /*
        imagesub = nh.subscribe(ORB_SLAM2::Config::getInstance().RuntimeParams().image_topic, 2,
                                &ORBVIO::MsgSynchronizer::imageCallback, &msgsync);
        imusub = nh.subscribe(ORB_SLAM2::Config::getInstance().RuntimeParams().imu_topic, 200,
                              &ORBVIO::MsgSynchronizer::imuCallback, &msgsync);
        */

        ROS_FATAL("Not Implemented !");
        exit(-1);
    }
    sensor_msgs::ImageConstPtr imageMsg;
    sensor_msgs::ImageConstPtr image2Msg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = ORB_SLAM2::Config::getInstance().RuntimeParams().multiply_g;
    const bool bRGB = ORB_SLAM2::Config::getInstance().CameraParams().rgb;
    ros::Rate r(1000);
    double discard_time = ORB_SLAM2::Config::getInstance().RuntimeParams().discard_time;
    if (!realtime_mode) {
        std::string bagfile = bagfile_path;
        ROS_WARN("Run Offline Mode:: Bagfile=%s", bagfile.c_str());
        rosbag::Bag bag;
        bag.open(bagfile, rosbag::bagmode::Read);
        ROS_WARN("---START----");
        std::vector<std::string> topics;
        std::string imutopic = ORB_SLAM2::Config::getInstance().RuntimeParams().imu_topic;
        std::string imagetopic = ORB_SLAM2::Config::getInstance().RuntimeParams().image_topic;
        std::string image2topic = ORB_SLAM2::Config::getInstance().RuntimeParams().image2_topic;
        topics.push_back(imagetopic);
        topics.push_back(image2topic);
        topics.push_back(imutopic);

        rosbag::View view(bag, rosbag::TopicQuery(topics));


        for (rosbag::MessageInstance const m: view){
            sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
            if (simu != NULL)
                msgsync.addImuMsg(simu);

            sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
            if (simage != NULL){
                if ( m.getTopic() == imagetopic ){
                    msgsync.addImage1Msg(simage);
                }
                else{
                    msgsync.addImage2Msg(simage);
                }
            }

            bool bdata = msgsync.getRecentMsgs(imageMsg, image2Msg, vimuMsg);
            if (bdata) {
                ROS_DEBUG("Num IMUs between images: %d", (int)vimuMsg.size());
                ORB_SLAM2::utils::eigen_aligned_vector<ORB_SLAM2::IMUData> vimuData;

                for (unsigned int i = 0; i < vimuMsg.size(); i++) {
                    sensor_msgs::ImuConstPtr imuMsg = vimuMsg[i];
                    double ax = imuMsg->linear_acceleration.x;
                    double ay = imuMsg->linear_acceleration.y;
                    double az = imuMsg->linear_acceleration.z;
                    if (bAccMultiply98) {
                        ax *= g3dm;
                        ay *= g3dm;
                        az *= g3dm;
                    }
                    ORB_SLAM2::IMUData
                            imudata(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y,
                                    imuMsg->angular_velocity.z,
                                    ax, ay, az, imuMsg->header.stamp.toSec());
                    vimuData.push_back(imudata);
                    //ROS_INFO("imu time: %.3f",vimuMsg[i]->header.stamp.toSec());
                }

                // Copy the ros image message to cv::Mat.
                cv_bridge::CvImageConstPtr cv_ptr;
                cv_bridge::CvImageConstPtr cv_ptr2;
                try {
                    cv_ptr = cv_bridge::toCvShare(imageMsg);
                    cv_ptr2 = cv_bridge::toCvShare(image2Msg);
                }
                catch (cv_bridge::Exception &e) {
                    ROS_ERROR("cv_bridge exception: %s", e.what());
                    return -1;
                }


                cv::Mat im = cv_ptr->image.clone();
                cv::Mat im2 = cv_ptr2->image.clone();

                if (im.channels() == 1){
                    if (bRGB){
                        cv::cvtColor(im, im, CV_GRAY2RGB);
                    }
                    else{
                        cv::cvtColor(im, im, CV_GRAY2BGR);
                    }
                }

                if (sensor_type == ORB_SLAM2::System::STEREO){
                    if (im2.channels() == 1){
                        if (bRGB){
                            cv::cvtColor(im2, im2, CV_GRAY2RGB);
                        }
                        else{
                            cv::cvtColor(im2, im2, CV_GRAY2BGR);
                        }
                    }
                }

                {
                    // To test relocalization
                    static double startT = -1;
                    if (startT < 0)
                        startT = imageMsg->header.stamp.toSec();
                    // Below to test relocalizaiton
                    //if(imageMsg->header.stamp.toSec() > startT+25 && imageMsg->header.stamp.toSec() < startT+25.3)
                    if (imageMsg->header.stamp.toSec() < startT + discard_time){
                        im = cv::Mat::zeros(im.size(), im.type());
                        im2 = cv::Mat::zeros(im2.size(), im2.type());
                    }

                }
                if (sensor_type == ORB_SLAM2::System::STEREO){
                    SLAM.TrackStereoVI(im, im2, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                }
                else {
                    SLAM.TrackRGBDVI(im, im2, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                }

                // Wait local mapping end.
                bool bstop = false;
                while (!SLAM.bLocalMapAcceptKF()) {
                    if (!ros::ok()) {
                        bstop = true;
                    }
                };
                if (bstop)
                    break;

            }

            ros::spinOnce();
            r.sleep();
            if (!ros::ok())
                break;
        }
        ROS_INFO("------Bag DONE-----");
        ros::spin();
    } else {
        ROS_WARN("Run realtime");
        exit(-1);
    }



    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");


    SLAM.SaveKeyFrameTrajectoryNavState(
            ORB_SLAM2::Config::getInstance().RuntimeParams().log_file_path + "KeyFrameNavStateTrajectory.txt");

    cout << endl << endl << "Trajectory Saved!" << endl;

    // Stop all threads

    ros::shutdown();
    SLAM.Shutdown();
    return 0;
}


