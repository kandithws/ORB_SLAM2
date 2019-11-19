#pragma once

#include <opencv2/core/core.hpp>
#include <memory>
#include <spdlog/spdlog.h>
#include "Converter.h"

#define ORB_SLAM2_CONFIG_RET_MACRO(PARAM) if(is_read_) {return PARAM;} \
else { SPDLOG_CRITICAL("Config is not read"); throw std::logic_error("Config is not read");}

#define ORB_SLAM2_DEF_PARAM_GETTER(PARAM_TYPE, PARAM_INSTANCE) const Params::PARAM_TYPE & PARAM_TYPE##Params() \
{ORB_SLAM2_CONFIG_RET_MACRO(PARAM_INSTANCE)}

#define ORB_SLAM2_DEFINE_CONFIG_PARAM(PARAM_TYPE) \
  protected: Params::PARAM_TYPE m##PARAM_TYPE##Param; \
  public: ORB_SLAM2_DEF_PARAM_GETTER(PARAM_TYPE, m##PARAM_TYPE##Param)


namespace ORB_SLAM2 {

class Config;

namespace Params {

typedef struct {
    bool use_stat_rm_outlier = true;
    int outlier_filter_type = 0; // 0: PCLSOR, 1: local euclidean, 2: local std
    double outlier_threshold = 0.2; // for type 1, 2
    int mean_k = 8;
    double std_dev_mul_th = 0.8;
    bool project_2d_outlier = true;
    bool use_mask = true;
    bool associate_centroid_only = true; // On Object association, full BB check or centroid only?
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
    bool use_object = true;
    bool use_imu = false;
    bool real_time = false;
} System;

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
    bool allow_skip=true; // TODO -- can use SystemParams.real_time instead
} ObjectDetection;


typedef struct {
    int window_size = 10;
    double object_detect_timeout = 1.0;
} LocalMapping;

// Made compatible with LearnVIORB
typedef struct imu {
  public:

    double g = 9.810;
    double vins_init_time = 15.0;
    bool fast_init = false; // only consider measurement @ first KF, assuming no movements
    int fast_init_num_msgs = 8;
    bool fast_init_inverse_g = false;


    imu() {
        Tbc = cv::Mat::eye(4,4, CV_32F);
        Tcb = cv::Mat::eye(4,4, CV_32F);
    }

    cv::Mat GetMatTbc() const { return Tbc.clone(); }
    cv::Mat GetMatTcb() const { return Tcb.clone(); }
    // TODO -- make class attribute with safe allocation (EIGEN_MAKE_ALIGN_OPERATOR_NEW)
    inline Eigen::Matrix4d GetEigTbc() const { return Converter::toHomogeneous4d(Tbc); }
    inline Eigen::Matrix4d GetEigTcb() const { return Converter::toHomogeneous4d(Tcb); }

  protected:
    cv::Mat Tbc;
    cv::Mat Tcb;
    bool mbTbcRead = false;
    friend ORB_SLAM2::Config;
} IMU;

// For "main"
typedef struct {
    double image_delay_to_imu = 0.0;
    std::string log_file_path;
    // For ROSbag dataset config
    std::string bagfile;
    std::string imu_topic;
    std::string image_topic;
    std::string image2_topic;
    bool multiply_g =  false; // _bAccMultiply9p8
    double discard_time = 0.0;
    bool pre_rectify_images = false;
} Runtime;

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
    ORB_SLAM2_DEFINE_CONFIG_PARAM(System);
    ORB_SLAM2_DEFINE_CONFIG_PARAM(LocalMapping);
    ORB_SLAM2_DEFINE_CONFIG_PARAM(IMU);
    ORB_SLAM2_DEFINE_CONFIG_PARAM(Runtime);

    cv::Mat getCamMatrix();

    void SetRealTimeFlag(bool st) {
        mSystemParam.real_time = st;
    }

    void SetUseIMU(bool st){
        mSystemParam.use_imu = st;
        if(mSystemParam.use_imu){
            if(!mIMUParam.mbTbcRead){
                SPDLOG_ERROR("SystemParam.use_imu was set but Tbc is not provided, abort !");
                exit(1);
            }
        }
    }

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