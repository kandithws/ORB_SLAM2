//
// Created by kandithws on 22/1/2562.
//

#include <fstream>
#include "utils/Config.h"

namespace ORB_SLAM2 {


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

//#define ORB_SLAM2_PARSE_CONFIG(PARAM_OBJ, PARAM_TYPE, PARAM_NAME) PARAM_OBJ.PARAM_NAME = (PARAM_TYPE) node[#PARAM_NAME]
#define ORB_SLAM2_PARSE_CONFIG(PARAM_TYPE, ATTR_TYPE, ATTR_NAME) \
m##PARAM_TYPE##Param.ATTR_NAME = (ATTR_TYPE) node[#ATTR_NAME];

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

        // Object Detection
        node = fs_["object_detection"];
        if(node.begin() != node.end()){

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

            mObjectDetectionParam.apply_nms = static_cast<bool>( (int) node["apply_nms"] );
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, std::string, model_path);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, std::string, config_path);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, double, nms_threshold);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, double, min_confidence);
            ORB_SLAM2_PARSE_CONFIG(ObjectDetection, int, input_size);
        }

        // Object Initialization
        node = fs_["object_initialization"];
        ORB_SLAM2_PARSE_CONFIG(ObjectInitializer, int, mean_k)
        ORB_SLAM2_PARSE_CONFIG(ObjectInitializer, double, std_dev_mul_th)

    }
    else{
        SPDLOG_CRITICAL("Config is not open");
    }
}

}