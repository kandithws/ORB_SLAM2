#pragma once

#include <opencv2/core/core.hpp>
#include <memory>
#include <spdlog/spdlog.h>

#define ORB_SLAM2_CONFIG_RET_MACRO(PARAM) if(is_read_) {return PARAM;} \
else { SPDLOG_CRITICAL("Config is not read"); throw std::logic_error("Config is not read");}

#define ORB_SLAM2_DEF_PARAM_GETTER(PARAM_TYPE, PARAM_INSTANCE) const Params::PARAM_TYPE & PARAM_TYPE##Params() \
{ORB_SLAM2_CONFIG_RET_MACRO(PARAM_INSTANCE)}

#define ORB_SLAM2_DEFINE_CONFIG_PARAM(PARAM_TYPE) \
  protected: Params::PARAM_TYPE m##PARAM_TYPE##Param; \
  public: ORB_SLAM2_DEF_PARAM_GETTER(PARAM_TYPE, m##PARAM_TYPE##Param)

namespace ORB_SLAM2 {

namespace Params {

typedef struct {
    bool use_stat_rm_outlier = true;
    int outlier_filter_type = 0; // 0: PCLSOR, 1: local euclidean, 2: local std
    double outlier_threshold = 0.2; // for type 1, 2
    int mean_k = 8;
    double std_dev_mul_th = 0.8;
    bool project_2d_outlier = true;
    bool use_mask = true;
} ObjectInitializer;

typedef struct {
    double fx;
    double fy;
    double cx;
    double cy;
    double k1;
    double k2;
    double p1;
    double p2;
    double fps = 30.0;
    bool rgb = false;
} Camera;

typedef struct {
    std::vector<std::string> label_map;
    std::string type; // CV, GRPC
    std::string config_path;
    std::string model_path;
    double min_confidence = 0.5;
    bool apply_nms = true;
    double nms_threshold = 0.4;
    int input_size = 416;
    // For GRPC
    std::string grpc_url;
} ObjectDetection;


}


class Config {
  public:
    static Config &getInstance() {
        static Config cfg;

        return cfg;
    }

    void readConfig(std::string cfg_file);

    std::string getLabelName(const int& label_id);

    ORB_SLAM2_DEFINE_CONFIG_PARAM(Camera);
    ORB_SLAM2_DEFINE_CONFIG_PARAM(ObjectDetection);
    ORB_SLAM2_DEFINE_CONFIG_PARAM(ObjectInitializer);

    cv::Mat getCamMatrix();

  protected:
    Config();

    cv::FileStorage fs_;
    std::string cfg_file_;
    std::mutex fs_mutex_;
    bool is_read_ = false;

    virtual ~Config() = default;

    void parseConfig();

    cv::Mat cam_mat_;
};
}