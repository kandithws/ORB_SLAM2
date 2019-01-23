#pragma once

#include <opencv2/core/core.hpp>
#include <memory>
#include <spdlog/spdlog.h>

#define ORB_SLAM2_CONFIG_RET_MACRO(PARAM) if(is_read_) {return PARAM;} \
else { SPDLOG_CRITICAL("Config is not read"); throw std::logic_error("Config is not read");}

namespace ORB_SLAM2 {

struct CameraParams {
  double fx;
  double fy;
  double cx;
  double cy;
  double k1;
  double k2;
  double p1;
  double p2;
  double fps = 10.0;
  bool rgb = false;
};

class Config {
  public:
    static Config& getInstance(){
        static Config cfg;

        return cfg;
    }

    void readConfig(std::string cfg_file);

    // TODO -- format to constant getters
    // make variable public for simplicity
    const CameraParams& cameraParams() { ORB_SLAM2_CONFIG_RET_MACRO(cam_params_) }

  protected:
    Config() = default;
    cv::FileStorage fs_;
    std::string cfg_file_;
    std::mutex fs_mutex_;
    bool is_read_ = false;
    virtual ~Config() = default;
    CameraParams cam_params_;
    void parseConfig();
};
}