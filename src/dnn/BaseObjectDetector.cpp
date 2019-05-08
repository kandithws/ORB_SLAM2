//
// Created by kandithws on 22/12/2561.
//

#include "dnn/BaseObjectDetector.h"

namespace ORB_SLAM2 {

PredictedObject::PredictedObject(const int &label, const float &conf, const cv::Rect &box)
        : _label(label), _confidence(conf), _bbox(box), _mask_type(MASK_TYPE::NO_MASK)
{}


PredictedObject::PredictedObject(const int &label, const float& conf, const cv::Rect& box,
                                 const cv::Mat &mask, const MASK_TYPE& mask_type) :
        _label(label), _confidence(conf), _bbox(box),
        _mask(mask),
        _mask_type(mask_type)
{}

const cv::Rect2f& PredictedObject::box() const {
    return _bbox;
}
std::vector<std::string> BaseObjectDetector::parseLabelMap(std::string path, char delim) {
    std::vector<std::string> label_map;
    std::ifstream file(path);

    auto rstrip = [](std::string& str) {
      str.erase(str.find_last_not_of("\t\n\v\f\r ") + 1);
    };

    if (file.is_open()) {
        std::string line;
        while (std::getline(file, line, delim)) {
            rstrip(line);
            label_map.push_back(line);
        }
    }
    else{
        // Warning!
        std::cout << "Couldn't open file " << path << std::endl;
    }

    return label_map;
}

void BaseObjectDetector::drawPredictionBoxes(cv::Mat &img, std::vector<std::shared_ptr<PredictedObject> > &preds) {
    for (auto &pred : preds) {
        int top = (int) pred->box().tl().y;
        int left = (int) pred->box().tl().x;
        cv::rectangle(img, pred->box().tl(), pred->box().br(), cv::Scalar(0, 255, 0));

        char conf_str[10];
        sprintf(conf_str, "%.2f", pred->_confidence);
        std::string label(conf_str);
        if (_label_map.size() > 0) {
            CV_Assert(pred->_label < (int) _label_map.size());
            label = _label_map[pred->_label] + ": " + label;
        } else {
            label = "class" + std::to_string(pred->_label) + ": " + label;
        }

        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        cv::putText(img, label, cv::Point(left, top), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    }
}

void BaseObjectDetector::drawPredictionBoxes(const std::vector<std::string>& label_map,
        cv::Mat &img, std::vector<std::shared_ptr<PredictedObject> > &preds) {
    for (auto &pred : preds) {
        int top = (int) pred->box().tl().y;
        int left = (int) pred->box().tl().x;
        cv::rectangle(img, pred->box().tl(), pred->box().br(), cv::Scalar(0, 255, 0));

        char conf_str[10];
        sprintf(conf_str, "%.2f", pred->_confidence);
        std::string label(conf_str);
        if (label_map.size() > 0) {
            CV_Assert(pred->_label < (int) label_map.size());
            label = label_map[pred->_label] + ": " + label;
        } else {
            label = "class" + std::to_string(pred->_label) + ": " + label;
        }

        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        top = std::max(top, labelSize.height);
        cv::putText(img, label, cv::Point(left, top), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    }
}

}