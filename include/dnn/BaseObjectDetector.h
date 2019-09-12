//
// Created by kandithws on 22/12/2561.
//

#ifndef ORB_SLAM2_BASEOBJECTDETECTOR_H
#define ORB_SLAM2_BASEOBJECTDETECTOR_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <memory>
#include "Converter.h"

namespace ORB_SLAM2 {

class PredictedObject {
  public:
    enum class MASK_TYPE : int {
        NO_MASK=-1,
        CROPPED=0,
        FULL=1
    };
    PredictedObject(const int& label, const float& conf, const cv::Rect& box);
    PredictedObject(const int& label, const float& conf, const cv::Rect& box,
                    const cv::Mat &mask, const MASK_TYPE& mask_type=MASK_TYPE::CROPPED);

    const int _label;
    const float _confidence;
    const cv::Rect2f _bbox;
    const cv::Mat _mask;
    const MASK_TYPE _mask_type;

    // box getters to alias the legacy version


    const cv::Rect2f& box() const;

    inline cv::Point2f GetCentroid2D() const {
        return (_bbox.tl() + _bbox.br()) * 0.5;
    }

    friend inline std::ostream& operator<<(std::ostream& os, const PredictedObject& pred){
        os << "PredObj{  label: " << pred._label << " , conf: "
           << pred._confidence << "\n   box: ["
           << pred._bbox << "], mask: " << static_cast<int>(pred._mask_type)
           << " }";
        return os;
    }
};

class BaseObjectDetector {
  public:

    void setLabelMap(std::vector<std::string>& label_map) { _label_map = label_map; };
    std::vector<std::string> const & getLabelMap() const { return _label_map;}
    virtual void detectObject(const cv::Mat& img, std::vector<std::shared_ptr<PredictedObject> >& preds, bool rgb=false) = 0;
    void drawPredictionBoxes(cv::Mat& img, std::vector<std::shared_ptr<PredictedObject> >& preds);// box
    static void drawPredictionBoxes(const std::vector<std::string>& label_map,
            cv::Mat& img,
            std::vector<std::shared_ptr<PredictedObject> >& preds);
    static std::vector<std::string> parseLabelMap(std::string path, char delim='\n'); // TODO--move to static method
    //void drawPredictions(cv::Mat& img, std::vector<PredictedObject>& preds, bool draw_boxes=false);
    //static buildDetector();

  protected:
    BaseObjectDetector() = default;
    std::vector<std::string> _label_map; // class_id -> class_name
};

}


#endif //ORB_SLAM2_BASEOBJECTDETECTOR_H
