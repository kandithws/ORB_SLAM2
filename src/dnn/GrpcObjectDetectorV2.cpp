//
// Created by kandithws on 8/5/2562.
//

#include "dnn/GrpcObjectDetectorV2.h"

namespace ORB_SLAM2 {

GrpcObjectDetectorV2::GrpcObjectDetectorV2(std::string url, bool verbose)
        : _channel(grpc::CreateChannel(url, grpc::InsecureChannelCredentials())), _verbose(verbose)
{
    _stub = detection_service_v2::InstanceDetectionService::NewStub(_channel);
    std::chrono::system_clock::time_point deadline =
            std::chrono::system_clock::now() + std::chrono::seconds(100);
    SPDLOG_INFO("Waiting for connection @ {}", url);
    _channel->WaitForConnected(deadline);
    SPDLOG_INFO("Connected to", url);
}

void GrpcObjectDetectorV2::detectObject(const cv::Mat &img, std::vector<std::shared_ptr<ORB_SLAM2::PredictedObject> > &preds, bool rgb) {

    detection_service_v2::Image req; detection_service_v2::InstanceDetections res;
    toGrpcImage(img, req, rgb);
    if(requestDetectObject(req, res)){
        toPredictions(res, preds);
    }
    else {
        SPDLOG_WARN("Fail to request an image for prediction");
    }
}

bool GrpcObjectDetectorV2::requestDetectObject(const detection_service_v2::Image &req,
                                               detection_service_v2::InstanceDetections &res) {
    grpc::ClientContext context;
    auto st = _stub->DetectInstances(&context, req, &res);
    if(!st.ok() && _verbose){
        SPDLOG_WARN("{}", st.error_message());
    }
    return st.ok();
}


void GrpcObjectDetectorV2::toGrpcImage(const cv::Mat &img, detection_service_v2::Image &grpc_img, bool rgb) {
    grpc_img.set_channel(3);
    size_t size = img.total() * img.elemSize();
    grpc_img.set_data(img.data, size * sizeof(char));
    grpc_img.set_height(img.rows);
    grpc_img.set_width(img.cols);
    grpc_img.set_type("uint8"); // TODO -- check cvtype
    grpc_img.set_rgb(rgb);
}

void GrpcObjectDetectorV2::toPredictions(const detection_service_v2::InstanceDetections &res,
                                         std::vector<std::shared_ptr<ORB_SLAM2::PredictedObject>> &preds) {
    preds.reserve(res.predictions_size());

    for (const auto& out : res.predictions()){
        int label_id = out.label_id();
        float conf = out.confidence();
        cv::Rect2f box(cv::Point2f(out.box().tlx(), out.box().tly()),
                       cv::Point2f(out.box().brx(), out.box().bry()));

//        if (out.mask_type() == 1){
//            SPDLOG_ERROR("Fullmask type still not implemented");
//            throw std::logic_error("Not Implemented!");
//        }
        PredictedObject::MASK_TYPE mt(static_cast<PredictedObject::MASK_TYPE>(out.mask_type()));

        auto buffer = out.mask().data().c_str();
        cv::Mat mask(out.mask().height(), out.mask().width(), CV_8UC1, &buffer);
        preds.push_back(std::make_shared<PredictedObject>(
                label_id, conf, box, mask, mt
        ));

    }
}




}