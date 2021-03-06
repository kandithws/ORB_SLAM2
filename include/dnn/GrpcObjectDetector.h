//
// Created by kandithws on 3/4/2562.
//

#ifndef ORB_SLAM2_GRPCOBJECTDETECTOR_H
#define ORB_SLAM2_GRPCOBJECTDETECTOR_H

#include "BaseObjectDetector.h"
#include <grpcpp/grpcpp.h>
#include "grpc/detection.grpc.pb.h"
#include "spdlog/spdlog.h"


namespace ORB_SLAM2 {
class GrpcObjectDetector : public BaseObjectDetector {
  public:
    GrpcObjectDetector(std::string url, bool verbose=true);

    void detectObject(const cv::Mat& img, std::vector<std::shared_ptr<PredictedObject> >& preds, bool rgb=false);

  private:
    void toGrpcImage(const cv::Mat& img, detection_service::Image &grpc_img, std::string encoding="bgr8");

    bool requestDetectObject(const detection_service::Image &req, detection_service::Detections& res);

    void toPredictions(const detection_service::Detections &detections, std::vector<std::shared_ptr<PredictedObject> >& preds);
    std::shared_ptr<grpc::Channel> _channel;
    std::unique_ptr<detection_service::DetectionService::Stub> _stub;
    bool _verbose;

};
}

#endif //ORB_SLAM2_GRPCOBJECTDETECTOR_H
