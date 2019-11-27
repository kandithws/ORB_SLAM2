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
#include <sensor_msgs/CompressedImage.h>

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
        ROS_INFO("Use real-time mode!, configured from ROS param %d", realtime_mode);
        ORB_SLAM2::Config::getInstance().SetRealTimeFlag(realtime_mode);
    }

    if (!pnh.getParam("bagfile_path", bagfile_path) && !realtime_mode) {
        bagfile_path = ORB_SLAM2::Config::getInstance().RuntimeParams().bagfile;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    bool pre_rectify_images = ORB_SLAM2::Config::getInstance().RuntimeParams().pre_rectify_images;

    if (pre_rectify_images) {
        if (sensor_type == ORB_SLAM2::System::STEREO){
            // Read rectification parameters
            cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
            if(!fsSettings.isOpened())
            {
                cerr << "ERROR: Wrong path to settings" << endl;
                return -1;
            }

            cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r, T;
            fsSettings["LEFT.K"] >> K_l;
            fsSettings["RIGHT.K"] >> K_r;



            fsSettings["LEFT.D"] >> D_l;
            fsSettings["RIGHT.D"] >> D_r;

            int rows_l = fsSettings["LEFT.height"];
            int rows_l_org = fsSettings["LEFT.original_height"];
            int cols_l = fsSettings["LEFT.width"];
            int cols_l_org = fsSettings["LEFT.original_width"];
            int rows_r = fsSettings["RIGHT.height"];
            int cols_r = fsSettings["RIGHT.width"];



            fsSettings["T_LEFT_TO_RIGHT"] >> T;



            if(K_l.empty() || K_r.empty() || D_l.empty() || D_r.empty() ||
               rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
            {
                ROS_FATAL("[PRERECT Stereo] Calibration parameters to rectify stereo are missing!");
                return -1;
            }

            if (!T.empty()) {
                ROS_INFO("[PRERECT Stereo]T_LEFT_TO_RIGHT given, automatically identify P and R");
                if ( (rows_l_org == 0) || (cols_l_org == 0) ){
                    rows_l_org = rows_l;
                    cols_l_org = cols_l;
                }

                cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(cols_l_org, rows_l_org),
                        T.rowRange(0,3).colRange(0,3), T.rowRange(0,3).col(3), R_l, R_r, P_l, P_r, cv::Mat());
            }
            else {
                fsSettings["LEFT.P"] >> P_l;
                fsSettings["RIGHT.P"] >> P_r;

                fsSettings["LEFT.R"] >> R_l;
                fsSettings["RIGHT.R"] >> R_r;

                if (P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty()) {
                    ROS_FATAL("[PRERECT Stereo] LEFT/RIGHT P, R are missing!");
                    return -1;
                }
            }

            std::cout << "P_l: " << P_l << std::endl;
            std::cout << "P_r: " << P_r << std::endl;
            cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
            cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
        }
        else {
            ROS_FATAL("Not Implemented pre rectified for RGB-D images");
            exit(0);
        }
    }


    std::string imutopic = ORB_SLAM2::Config::getInstance().RuntimeParams().imu_topic;
    std::string imagetopic = ORB_SLAM2::Config::getInstance().RuntimeParams().image_topic;
    std::string image2topic = ORB_SLAM2::Config::getInstance().RuntimeParams().image2_topic;


    /**
     * @brief added data sync
     */
    ROS_INFO("INITIALIZE MSG SYNC");
    double imageMsgDelaySec = ORB_SLAM2::Config::getInstance().RuntimeParams().image_delay_to_imu;
    std::shared_ptr<ORBVIO::StereoMsgSynchronizer> msgsync;
    if(realtime_mode){
        msgsync = std::make_shared<ORBVIO::StereoMsgSynchronizer>(imagetopic, image2topic, imutopic, imageMsgDelaySec);
    }
    else{
        msgsync = std::make_shared<ORBVIO::StereoMsgSynchronizer>(imageMsgDelaySec, imageMsgDelaySec);
    }

    ROS_INFO("INITIALIZED !");

    sensor_msgs::ImageConstPtr imageMsg;
    sensor_msgs::ImageConstPtr image2Msg;
    std::vector<sensor_msgs::ImuConstPtr> vimuMsg;

    // 3dm imu output per g. 1g=9.80665 according to datasheet
    const double g3dm = 9.80665;
    const bool bAccMultiply98 = ORB_SLAM2::Config::getInstance().RuntimeParams().multiply_g;
    const bool bRGB = ORB_SLAM2::Config::getInstance().CameraParams().rgb;
    const std::string encoding = bRGB ? "rgb8" : "bgr8";
    int rate = realtime_mode ? 300 : 1000;

    ros::Rate r(rate);

    double discard_time = ORB_SLAM2::Config::getInstance().RuntimeParams().discard_time;
    if (!realtime_mode) {
        std::string bagfile = bagfile_path;
        ROS_WARN("Run Offline Mode:: Bagfile=%s", bagfile.c_str());
        rosbag::Bag bag;
        bag.open(bagfile, rosbag::bagmode::Read);
        ROS_WARN("---START----");
        std::vector<std::string> topics;

        topics.push_back(imagetopic);
        topics.push_back(image2topic);
        topics.push_back(imutopic);
        rosbag::View view(bag, rosbag::TopicQuery(topics));


        for (rosbag::MessageInstance const m: view){
            sensor_msgs::ImuConstPtr simu = m.instantiate<sensor_msgs::Imu>();
            if ( (simu != NULL) && (m.getTopic() == imutopic) ) {
                msgsync->addImuMsg(simu);
            }


            sensor_msgs::ImageConstPtr simage = m.instantiate<sensor_msgs::Image>();
            if (simage != NULL){
                if ( m.getTopic() == imagetopic ){
                    msgsync->addImage1Msg(simage);
                }
                else{
                    msgsync->addImage2Msg(simage);
                }
            }


            sensor_msgs::CompressedImageConstPtr simage_compressed = m.instantiate<sensor_msgs::CompressedImage>();
            if (simage_compressed != NULL) {


                auto img_msg_ptr = cv_bridge::toCvCopy(simage_compressed, encoding)->toImageMsg();

                if ( m.getTopic() == imagetopic ){
                    msgsync->addImage1Msg(img_msg_ptr);
                }
                else{
                    msgsync->addImage2Msg(img_msg_ptr);
                }
            }


            bool bdata = msgsync->getRecentMsgs(imageMsg, image2Msg, vimuMsg);
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
                    if (pre_rectify_images){
                        cv::Mat imLeftRect, imRightRect;
                        cv::remap(im,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
                        cv::remap(im2,imRightRect,M1r,M2r,cv::INTER_LINEAR);
                        SLAM.TrackStereoVI(imLeftRect, imRightRect, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                    }
                    else{
                        SLAM.TrackStereoVI(im, im2, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                    }

                }
                else {
                    assert(image2Msg.encoding == "mono16");
                    ROS_INFO("TRACKING RGB-D");
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
        ROS_INFO("------Bag DONE, press CTRL+C to save output-----");
        ros::spin();
    } else {
        ROS_WARN("Run realtime");

        while(ros::ok()){

            bool bdata = msgsync->getRecentMsgs(imageMsg, image2Msg, vimuMsg);
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
                    if (pre_rectify_images){
                        cv::Mat imLeftRect, imRightRect;
                        cv::remap(im,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
                        cv::remap(im2,imRightRect,M1r,M2r,cv::INTER_LINEAR);
                        SLAM.TrackStereoVI(imLeftRect, imRightRect, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                    }
                    else{
                        SLAM.TrackStereoVI(im, im2, vimuData, imageMsg->header.stamp.toSec() - imageMsgDelaySec);
                    }

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
        }
    }

    // Stop all threads
    ROS_INFO("Shutting down ROS");
    ros::shutdown();
    ROS_INFO("Shutting down SLAM");
    SLAM.Shutdown();
    ROS_INFO("Saving Trajectory Output");
    SLAM.SaveKeyFrameTrajectoryTUMWithObjects(ORB_SLAM2::Config::getInstance().RuntimeParams().log_file_path);
    ROS_INFO("Trajectory Saved at %s", ORB_SLAM2::Config::getInstance().RuntimeParams().log_file_path.c_str());
    return 0;
}


/*
 *     def set_alpha(self, a):
        """
        Set the alpha value for the calibrated camera solution. The
        alpha value is a zoom, and ranges from 0 (zoomed in, all pixels
        in calibrated image are valid) to 1 (zoomed out, all pixels in
        original image are in calibrated image).
        """

        cv2.stereoRectify(self.l.intrinsics,
                         self.l.distortion,
                         self.r.intrinsics,
                         self.r.distortion,
                         self.size,
                         self.R,
                         self.T,
                         self.l.R, self.r.R, self.l.P, self.r.P,
                         alpha = a)

        cv2.initUndistortRectifyMap(self.l.intrinsics, self.l.distortion, self.l.R, self.l.P, self.size, cv2.CV_32FC1,
                                   self.l.mapx, self.l.mapy)
        cv2.initUndistortRectifyMap(self.r.intrinsics, self.r.distortion, self.r.R, self.r.P, self.size, cv2.CV_32FC1,
                                   self.r.mapx, self.r.mapy)
 * */