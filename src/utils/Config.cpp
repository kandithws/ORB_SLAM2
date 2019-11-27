//
// Created by kandithws on 22/1/2562.
//

#include <fstream>
#include "utils/Config.h"
#include <algorithm>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/case_conv.hpp>

namespace ORB_SLAM2 {

// 1, true, True, TRUE => true, else false
bool parseBool(const std::string& s){
    std::string data = boost::trim_copy(s);
    if ( (std::atoi(data.c_str()) == 1) || (boost::to_lower_copy(data) == "true") ){
        return true;
    }
    return false;
}

Config::Config() : cam_mat_(3,3, CV_64F) {}

void Config::readConfig(std::string cfg_file) {
    std::lock_guard<std::mutex> lock(fs_mutex_);
    if(is_read_){
        SPDLOG_WARN("Config file is already read, overwriting with {}", cfg_file);
    }
    cfg_file_ = cfg_file;
    fs_.open(cfg_file, cv::FileStorage::READ);
    is_read_ = true;
    parseConfig();
    fs_.release();
}

/*
 *  Utility Macro to use with parseConfig()
 *
 * */

#define ORB_SLAM2_PARSE_CONFIG_SCOPE(scope_name) \
node = fs_[scope_name]; \
if (node.begin() != node.end())

#define ORB_SLAM2_PARSE_CONFIG(PARAM_TYPE, ATTR_TYPE, ATTR_NAME) \
if (!node[#ATTR_NAME].empty()) \
m##PARAM_TYPE##Param.ATTR_NAME = (ATTR_TYPE) node[#ATTR_NAME];

#define ORB_SLAM2_PARSE_BOOL_CONFIG(PARAM_TYPE, ATTR_NAME) \
if (!node[#ATTR_NAME].empty()) \
m##PARAM_TYPE##Param.ATTR_NAME = static_cast<bool>((int) node[#ATTR_NAME]);

void Config::parseConfig() {
    if(fs_.isOpened()) {
        // TODO -- refactor to proper YAML node
        cv::FileNode node;
        // Camera (legacy format), no yaml hierachy

        mCameraParam.fx = (double) fs_["Camera.fx"];
        mCameraParam.fy = (double) fs_["Camera.fy"];
        mCameraParam.cx = (double) fs_["Camera.cx"];
        mCameraParam.cy = (double) fs_["Camera.cy"];
        mCameraParam.k1 = (double) fs_["Camera.k1"];
        mCameraParam.k2 = (double) fs_["Camera.k2"];
        mCameraParam.p1 = (double) fs_["Camera.p1"];
        mCameraParam.p1 = (double) fs_["Camera.p1"];
        mCameraParam.fps = (double) fs_["Camera.fps"];
        mCameraParam.rgb = static_cast<bool>( (int) fs_["Camera.rgb"] );

        cam_mat_.at<double>(0,0) = mCameraParam.fx;
        cam_mat_.at<double>(0,2) = mCameraParam.cx;
        cam_mat_.at<double>(1,1) = mCameraParam.fy;
        cam_mat_.at<double>(1,2) = mCameraParam.cy;
        cam_mat_.at<double>(2,2) = 1.0;

        // Object Detection
        ORB_SLAM2_PARSE_CONFIG_SCOPE("object_detection") {
            std::string label_map_path = (std::string) node["label_map_path"];
            if(!label_map_path.empty()){
                //parse label map
                std::ifstream label_map_file(label_map_path);
                std::string line;
                while (std::getline(label_map_file, line))
                    mObjectDetectionParam.label_map.push_back(line);
            }
            else{
                cv::FileNode label_map_node = node["label_map"];
                if(label_map_node.type() == cv::FileNode::SEQ){
                    cv::FileNodeIterator it = label_map_node.begin(), it_end = label_map_node.end();
                    for(;it != it_end; it++)
                        mObjectDetectionParam.label_map.push_back(*it);
                }
                else {
                    SPDLOG_WARN("Label map should be stored as string array");
                }
            }

            ORB_SLAM2_PARSE_BOOL_CONFIG(ObjectDetection, apply_nms)
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, std::string, model_path);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, std::string, config_path);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, double, nms_threshold);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, double, min_confidence);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, int, input_size);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, std::string, grpc_url);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, std::string, type);
            ORB_SLAM2_PARSE_BOOL_CONFIG(ObjectDetection, allow_skip);
        }

        // Object Initialization
        ORB_SLAM2_PARSE_CONFIG_SCOPE("object_initialization") {
            ORB_SLAM2_PARSE_CONFIG(ObjectInitializer, int, mean_k)
            ORB_SLAM2_PARSE_CONFIG(ObjectInitializer, double, std_dev_mul_th)
            ORB_SLAM2_PARSE_BOOL_CONFIG(ObjectInitializer, project_2d_outlier)
            ORB_SLAM2_PARSE_BOOL_CONFIG(ObjectInitializer, use_mask)
            ORB_SLAM2_PARSE_BOOL_CONFIG(ObjectInitializer, use_stat_rm_outlier)
            ORB_SLAM2_PARSE_CONFIG(ObjectInitializer, int, outlier_filter_type)
            ORB_SLAM2_PARSE_CONFIG(ObjectInitializer, double, outlier_threshold)
            ORB_SLAM2_PARSE_BOOL_CONFIG(ObjectInitializer, associate_centroid_only)
        }

        ORB_SLAM2_PARSE_CONFIG_SCOPE("system") {
            ORB_SLAM2_PARSE_BOOL_CONFIG(System, use_object)
            ORB_SLAM2_PARSE_BOOL_CONFIG(System, use_imu)
            ORB_SLAM2_PARSE_BOOL_CONFIG(System, real_time)
        }

        ORB_SLAM2_PARSE_CONFIG_SCOPE("local_mapping") {
            ORB_SLAM2_PARSE_CONFIG(LocalMapping, int, window_size)
            ORB_SLAM2_PARSE_CONFIG(LocalMapping, double, object_detect_timeout)
        }


        ORB_SLAM2_PARSE_CONFIG_SCOPE("imu") {
            ORB_SLAM2_PARSE_CONFIG(IMU, double, vins_init_time)
            ORB_SLAM2_PARSE_BOOL_CONFIG(IMU, fast_init)
            ORB_SLAM2_PARSE_CONFIG(IMU, int, fast_init_num_msgs)
            ORB_SLAM2_PARSE_BOOL_CONFIG(IMU, fast_init_inverse_g)

            // Legacy Matrix
            auto tbc_node = node["Tbc"];

            if (!node["Tbc"].empty()){
                for (int i=0; i < 12; i++){
                    mIMUParam.Tbc.at<float>(i) = tbc_node[i];
                }

                auto Rt = mIMUParam.Tbc.rowRange(0,3).colRange(0,3).t();
                mIMUParam.Tcb.rowRange(0,3).colRange(0,3) =  Rt;
                mIMUParam.Tcb.rowRange(0,3).col(3) = - Rt * mIMUParam.Tbc.rowRange(0,3).col(3);
                SPDLOG_INFO("Tbc= ");
                std::cout << mIMUParam.GetMatTbc() << std::endl;
                SPDLOG_INFO("Tcb= ");
                std::cout << mIMUParam.GetMatTcb() << std::endl;
                SPDLOG_INFO("Tbc * Tcb= (Should be eye(4,4) )");
                std::cout << mIMUParam.GetEigTbc() * mIMUParam.GetEigTcb() << std::endl;
                mIMUParam.mbTbcRead = true;
            }
        }


        ORB_SLAM2_PARSE_CONFIG_SCOPE("runtime") {
            ORB_SLAM2_PARSE_CONFIG(Runtime, std::string, bagfile)
            ORB_SLAM2_PARSE_CONFIG(Runtime, std::string, imu_topic)
            ORB_SLAM2_PARSE_CONFIG(Runtime, std::string, image_topic)
            ORB_SLAM2_PARSE_CONFIG(Runtime, std::string, image2_topic)
            ORB_SLAM2_PARSE_CONFIG(Runtime, std::string, log_file_path)
            ORB_SLAM2_PARSE_CONFIG(Runtime, double, image_delay_to_imu)
            ORB_SLAM2_PARSE_CONFIG(Runtime, double, discard_time)
            ORB_SLAM2_PARSE_BOOL_CONFIG(Runtime, multiply_g)
            ORB_SLAM2_PARSE_BOOL_CONFIG(Runtime, pre_rectify_images)
        }

        ORB_SLAM2_PARSE_CONFIG_SCOPE("eval") {
            ORB_SLAM2_PARSE_CONFIG(Eval, int, num_save_kf_images)
            ORB_SLAM2_PARSE_BOOL_CONFIG(Eval, enable)
        }

        SPDLOG_INFO("\n-------Config Summary------\n"
                    "System:\n"
                    "\tuse_object: {}\n"
                    "\tuse_imu: {}\n"
                    "\treal_time: {}\n"
                    "Runtime:\n"
                    "\tbagfile/dataset: {}\n"
                    "\tlogfilepath: {}",
                    mSystemParam.use_object,
                    mSystemParam.use_imu,
                    mSystemParam.real_time,
                    mRuntimeParam.bagfile,
                    mRuntimeParam.log_file_path
                    );

        if (mSystemParam.use_imu){
            SPDLOG_INFO("\n-------IMU Config Summary------\n"
                        "IMU:\n"
                        "\tmultiply_g: {}\n"
                        "\tg: {}\n"
                        "\tvins_init_time: {}\n"
                        "\tfast_init: {}"
                        "\t\tnum_msgs: {}"
                        "\t\tinverse_g: {}",
                        mRuntimeParam.multiply_g,
                        mIMUParam.g,
                        mIMUParam.vins_init_time,
                        mIMUParam.fast_init,
                        mIMUParam.fast_init_num_msgs,
                        mIMUParam.fast_init_inverse_g);
        }


        // TODO-- Config Summary
    }
    else{
        SPDLOG_CRITICAL("Config is not open");
    }
}

cv::Mat Config::getCamMatrix() {
    return cam_mat_.clone();
}

std::string Config::getLabelName(const int &label_id) {
    try{
        return mObjectDetectionParam.label_map[label_id];
    }
    catch (std::exception &e) {
        return "Unknown";
    }
}

}