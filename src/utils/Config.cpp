//
// Created by kandithws on 22/1/2562.
//

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
    }
    else{
        SPDLOG_CRITICAL("Config is not open");
    }
}

}