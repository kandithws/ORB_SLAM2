//
// Created by kandithws on 3/4/2562.
//

#include "dnn/GrpcObjectDetector.h"

namespace ORB_SLAM2 {

GrpcObjectDetector::GrpcObjectDetector(std::string url, bool verbose)
: _channel(grpc::CreateChannel(url, grpc::InsecureChannelCredentials())), _verbose(verbose)
{
    _stub = DetectionService::NewStub(_channel);
    std::chrono::system_clock::time_point deadline =
            std::chrono::system_clock::now() + std::chrono::seconds(100);
    SPDLOG_INFO("Waiting for connection @ {}", url);
    _channel->WaitForConnected(deadline);
    SPDLOG_INFO("Connected to", url);
}

void GrpcObjectDetector::detectObject(const cv::Mat &img, std::vector<std::shared_ptr<ORB_SLAM2::PredictedObject> > &preds, bool rgb) {

    Image req; Detections res;
    toGrpcImage(img, req, rgb ? "rgb8" : "bgr8");
    if(requestDetectObject(req, res)){
        toPredictions(res, preds);
    }
    else {
        SPDLOG_WARN("Fail tos request an image for prediction");
    }
}

void GrpcObjectDetector::toGrpcImage(const cv::Mat &img, Image &grpc_img, std::string encoding) {
    grpc_img.set_encoding(encoding);
    size_t size = img.total() * img.elemSize();
    grpc_img.set_data(img.data, size * sizeof(char));
    grpc_img.set_height(img.rows);
    grpc_img.set_width(img.cols);
}

bool GrpcObjectDetector::requestDetectObject(const Image &req, Detections &res) {
    grpc::ClientContext context;
    auto st = _stub->ObjectDetection(&context, req, &res);
    if(!st.ok() && _verbose){
        SPDLOG_WARN("{}", st.error_message());
    }
    return st.ok();
}

void GrpcObjectDetector::toPredictions(const Detections &detections, std::vector<std::shared_ptr<PredictedObject> > &preds) {
    auto detect_vect = detections.detections();
    auto end = detect_vect.end();

    for (auto it = detect_vect.begin(); it != end; it++){
        cv::Rect box(cv::Point(it->box().tl().x(), it->box().tl().y()),
                cv::Point(it->box().br().x(), it->box().br().y()));

        preds.push_back(std::make_shared<PredictedObject>(it->label_id(), it->confidence(), box));
    }
}

}