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

void Config::parseConfig() {
    if(fs_.isOpened()) {
        // TODO -- refactor to proper YAML node
        cv::FileNode node;
        // Camera (legacy format), no yaml hierachy

        cam_params_.fx = (double) fs_["Camera.fx"];
        cam_params_.fy = (double) fs_["Camera.fy"];
        cam_params_.cx = (double) fs_["Camera.cx"];
        cam_params_.cy = (double) fs_["Camera.cy"];
        cam_params_.k1 = (double) fs_["Camera.k1"];
        cam_params_.k2 = (double) fs_["Camera.k2"];
        cam_params_.p1 = (double) fs_["Camera.p1"];
        cam_params_.p1 = (double) fs_["Camera.p1"];
        cam_params_.fps = (double) fs_["Camera.fps"];
        cam_params_.rgb = static_cast<bool>( (int) fs_["Camera.rgb"] );

        // Object Detection
        node = fs_["object_detection"];
        if(node.begin() != node.end()){
            obj_detection_params_.apply_nms = static_cast<bool>( (int) node["apply_nms"] );
            std::string label_map_path = (std::string) node["label_map_path"];
            if(!label_map_path.empty()){
                //parse label map
                std::ifstream label_map_file(label_map_path);
                std::string line;
                while (std::getline(label_map_file, line))
                    obj_detection_params_.label_map.push_back(line);
            }
            else{
                cv::FileNode label_map_node = node["label_map"];
                if(label_map_node.type() == cv::FileNode::SEQ){
                    cv::FileNodeIterator it = label_map_node.begin(), it_end = label_map_node.end();
                    for(;it != it_end; it++)
                        obj_detection_params_.label_map.push_back(*it);
                }
                else {
                    SPDLOG_WARN("Label map should be stored as string array");
                }
            }
            obj_detection_params_.model_path = (std::string) node["model_path"];
            obj_detection_params_.config_path = (std::string) node["config_path"];
            obj_detection_params_.conf_th = (double) node["min_confidence"];
            obj_detection_params_.nms_th = (double) node["nms_threshold"];
            obj_detection_params_.input_size = (int) node["input_size"];
        }

    }
    else{
        SPDLOG_CRITICAL("Config is not open");
    }
}

}