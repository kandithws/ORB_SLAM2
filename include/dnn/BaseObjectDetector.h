//
// Created by kandithws on 22/12/2561.
//

#ifndef ORB_SLAM2_BASEOBJECTDETECTOR_H
#define ORB_SLAM2_BASEOBJECTDETECTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>

namespace ORB_SLAM2 {

class PredictedObject {
  public:
    PredictedObject();
    PredictedObject(int label, float conf);
    PredictedObject(int label, float conf, cv::Rect& box);
    PredictedObject(int label, float conf, std::vector<cv::Point2f>& poly);
    int _label;
    float _confidence;
    void setPolyfromRect(cv::Rect& rect);
    std::vector<cv::Point2f> _poly;
    cv::Rect2f& box();

  private:
    void generateBoundingBox();
    std::shared_ptr<cv::Rect2f> _bbox;
};

class BaseObjectDetector {
  public:

    void setLabelMap(std::vector<std::string>& label_map) { _label_map = label_map; };
    std::vector<std::string> const & getLabelMap() const { return _label_map;}
    virtual void detectObject(cv::Mat& img, std::vector<PredictedObject>& preds, bool rgb=false) = 0;
    void drawPredictionBoxes(cv::Mat& img, std::vector<PredictedObject>& preds);// box
    void parseLabelMap(std::string path, char delim='\n');
    //void drawPredictions(cv::Mat& img, std::vector<PredictedObject>& preds, bool draw_boxes=false);

  protected:
    BaseObjectDetector() = default;
    std::vector<std::string> _label_map; // class_id -> class_name
};

}


#endif //ORB_SLAM2_BASEOBJECTDETECTOR_H
