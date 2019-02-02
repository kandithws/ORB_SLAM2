//
// Created by kandithws on 6/1/2562.
//

#ifndef ORB_SLAM2_DNNCVOBJECTDETECTOR_H
#define ORB_SLAM2_DNNCVOBJECTDETECTOR_H


#include "BaseObjectDetector.h"
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/opencv.hpp>

#define CV_DNN_FRAMEWORK_CAFFE "Caffe"
#define CV_DNN_FRAMEWORK_TENSORFLOW "Tensorflow"
#define CV_DNN_FRAMEWORK_DARKNET "Darknet"

namespace ORB_SLAM2{

class CVObjectDetector : public BaseObjectDetector {
  public:

    CVObjectDetector(std::string weights, std::string config="", std::string framework="");
    /*
     * @param rgb: true if input channel if rgb else, bgr
     * */
    void detectObject(const cv::Mat& img, std::vector<PredictedObject>& preds, bool rgb=false);
    void setApplyNMS(bool st = true) { _apply_nms = st; }
    void setConfidenceThreshold(double val) { _conf_th = val; }
    void setNMSThreshold(double val) { _nms_th = val; }
    void setInputSize(int input_width, int input_height=-1);
    int _input_width;
    int _input_height;

  protected:
    /*
     * @return Model's output layer name
     *
     * */
    virtual std::string setupModel(std::shared_ptr<cv::dnn::Net>& model) { return std::string(); }
    //virtual void postProcess(cv::Mat& img, std::vector<cv::Mat>& outs, std::vector<PredictedObject>& preds);
    virtual void postProcess(const cv::Mat& img, std::vector<cv::Mat>& raw_outs, std::vector<int>& class_ids,
                             std::vector<float>& confidences,
                             std::vector<cv::Rect>& boxes);
    void applyNMSBoxes(std::vector<PredictedObject>& preds, std::vector<int>& class_ids,
                       std::vector<float>& confidences,
                       std::vector<cv::Rect>& boxes); // Non-maxima supression
    std::shared_ptr<cv::dnn::Net> _model;
    double _nms_th = 0.4;
    double _conf_th = 0.5;
    bool _apply_nms= false;

  private:
    void postProcessRegionOutLayer(const cv::Mat& img, std::vector<cv::Mat>& raw_outs, std::vector<int>& class_ids,
                                    std::vector<float>& confidences,
                                    std::vector<cv::Rect>& boxes);

    void postProcessDetectionOutLayer(const cv::Mat& img, std::vector<cv::Mat>& raw_outs, std::vector<int>& class_ids,
                                   std::vector<float>& confidences,
                                   std::vector<cv::Rect>& boxes);

    void postProcessDetectionOutRCNN(const cv::Mat& img, std::vector<cv::Mat>& raw_outs, std::vector<int>& class_ids,
                                      std::vector<float>& confidences,
                                      std::vector<cv::Rect>& boxes);

};

}
#endif //ORB_SLAM2_DNNCVOBJECTDETECTOR_H
