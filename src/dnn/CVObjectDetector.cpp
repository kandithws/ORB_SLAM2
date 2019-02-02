//
// Created by kandithws on 6/1/2562.
//

#include <dnn/CVObjectDetector.h>

namespace ORB_SLAM2 {

CVObjectDetector::CVObjectDetector(std::string weights, std::string config, std::string framework)
    : BaseObjectDetector() {
    _model = std::make_shared<cv::dnn::Net>();
    *_model = cv::dnn::readNet(weights, config, framework);
    _model->setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    _model->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
}

void CVObjectDetector::setInputSize(int input_width, int input_height) {
    _input_width = input_width;
    _input_height = input_height < 0 ? input_width : input_height;
}
void CVObjectDetector::detectObject(const cv::Mat &img, std::vector<PredictedObject> &preds, bool rgb) {
    // required RGB input
    cv::Size inp_size(_input_width > 0 ? _input_width : img.cols,
                      _input_height > 0 ? _input_height : img.rows);
    cv::Mat blob = cv::dnn::blobFromImage(img, 1.0f / 255.0f,
                                          inp_size,
                                          cv::Scalar(0, 0, 0), !rgb); // require rgb

    _model->setInput(blob);
    static std::vector<cv::String> outnames = _model->getUnconnectedOutLayersNames();
// TODO
//  if (_model->getLayer(0)->outputNameToIndex("im_info") != -1) { // Faster-RCNN or R-FCN
//    cv::Mat resized_img;
//    cv::resize(img, resized_img, inp_size);
//    cv::Mat imInfo = (cv::Mat_< float>(1, 3) << inp_size.height, inp_size.width, 1.6f);
//    _model->setInput(imInfo, "im_info");
//  }
    std::vector<cv::Mat> outs;
    _model->forward(outs, outnames);
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    postProcess(img, outs, class_ids, confidences, boxes);
    if (_apply_nms)
        applyNMSBoxes(preds, class_ids, confidences, boxes);
    else {
        preds.resize(class_ids.size());
        for (size_t i = 0; i < class_ids.size(); i++) {
            preds[i] = PredictedObject(class_ids[i], confidences[i], boxes[i]);
        }
    }

}

void CVObjectDetector::applyNMSBoxes(std::vector<PredictedObject> &preds, std::vector<int> &class_ids,
                                     std::vector<float> &confidences,
                                     std::vector<cv::Rect> &boxes) {
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, _conf_th, _nms_th, indices);
    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];

        // Build results on the fly for simplicity
        preds.push_back(PredictedObject(class_ids[idx], confidences[idx], boxes[idx]));
    }

}

void CVObjectDetector::postProcess(const cv::Mat &img, std::vector<cv::Mat> &raw_outs, std::vector<int> &class_ids,
                                   std::vector<float> &confidences,
                                   std::vector<cv::Rect> &boxes) {

    static std::vector<int> out_layers = _model->getUnconnectedOutLayers();
    static std::string out_layer_type = _model->getLayer(out_layers[0])->type;

    if (out_layer_type == "Region") // Darknet !
        postProcessRegionOutLayer(img, raw_outs, class_ids, confidences, boxes);
    else if (out_layer_type == "DetectionOutput")
        postProcessDetectionOutLayer(img, raw_outs, class_ids, confidences, boxes);
    else if (_model->getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
        postProcessDetectionOutRCNN(img, raw_outs, class_ids, confidences, boxes);
    else
        CV_Error(cv::Error::StsNotImplemented, "Unknown output layer type: " + out_layer_type);

}

void CVObjectDetector::postProcessRegionOutLayer(const cv::Mat &img,
                                                 std::vector<cv::Mat> &raw_outs,
                                                 std::vector<int> &class_ids,
                                                 std::vector<float> &confidences,
                                                 std::vector<cv::Rect> &boxes) {
    // Network produces output blob with a shape NxC where N is a number of
    // detected objects and C is a number of classes + 4 where the first 4
    // numbers are [center_x, center_y, width, height]

    for (size_t i = 0; i < raw_outs.size(); i++) {
        float *data = (float *) raw_outs[i].data;

        for (size_t j = 0; j < raw_outs[i].rows; j++, data += raw_outs[i].cols) {
            cv::Mat scores = raw_outs[i].row(j).colRange(5, raw_outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > _conf_th) {
                int centerX = (int) (data[0] * img.cols);
                int centerY = (int) (data[1] * img.rows);
                int width = (int) (data[2] * img.cols);
                int height = (int) (data[3] * img.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;
                cv::Rect rect(left, top, width, height);
                class_ids.push_back(classIdPoint.x);
                confidences.push_back(confidence);
                boxes.push_back(rect);
            }
        }
    }
}

void CVObjectDetector::postProcessDetectionOutLayer(const cv::Mat &img,
                                                    std::vector<cv::Mat> &raw_outs,
                                                    std::vector<int> &class_ids,
                                                    std::vector<float> &confidences,
                                                    std::vector<cv::Rect> &boxes) {
    // Network produces output blob with a shape 1x1xNx7 where N is a number of
    // detections and an every detection is a vector of values
    // [batchId, classId, confidence, left, top, right, bottom]
    CV_Assert(raw_outs.size() == 1);
    float *data = (float *) raw_outs[0].data;
    for (size_t i = 0; i < raw_outs[0].total(); i += 7) {
        float confidence = data[i + 2];
        if (confidence > _conf_th) {
            int left = (int) (data[i + 3] * img.cols);
            int top = (int) (data[i + 4] * img.rows);
            int right = (int) (data[i + 5] * img.cols);
            int bottom = (int) (data[i + 6] * img.rows);
            int width = right - left + 1;
            int height = bottom - top + 1;
            class_ids.push_back((int) (data[i + 1]) - 1);  // Skip 0th background class id.
            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(confidence);
        }
    }
}

void CVObjectDetector::postProcessDetectionOutRCNN(const cv::Mat &img,
                                                   std::vector<cv::Mat> &raw_outs,
                                                   std::vector<int> &class_ids,
                                                   std::vector<float> &confidences,
                                                   std::vector<cv::Rect> &boxes) {
    // Network produces output blob with a shape 1x1xNx7 where N is a number of
    // detections and an every detection is a vector of values
    // [batchId, classId, confidence, left, top, right, bottom]
    CV_Assert(raw_outs.size() == 1);
    float *data = (float *) raw_outs[0].data;
    for (size_t i = 0; i < raw_outs[0].total(); i += 7) {
        float confidence = data[i + 2];
        if (confidence > _conf_th) {
            int left = (int) data[i + 3];
            int top = (int) data[i + 4];
            int right = (int) data[i + 5];
            int bottom = (int) data[i + 6];
            int width = right - left + 1;
            int height = bottom - top + 1;
            class_ids.push_back((int) (data[i + 1]) - 1);  // Skip 0th background class id.
            boxes.push_back(cv::Rect(left, top, width, height));
            confidences.push_back(confidence);
        }
    }
}

}
