//
// Created by kandithws on 8/5/2562.
//

#ifndef ORB_SLAM2_GRPCOBJECTDETECTORV2_H
#define ORB_SLAM2_GRPCOBJECTDETECTORV2_H
#include "BaseObjectDetector.h"
#include <grpcpp/grpcpp.h>
#include "grpc/detection_v2.grpc.pb.h"
#include "spdlog/spdlog.h"


namespace ORB_SLAM2 {
class GrpcObjectDetectorV2 : public BaseObjectDetector {
  public:
    GrpcObjectDetectorV2(std::string url, bool verbose=true);

    void detectObject(const cv::Mat& img, std::vector<std::shared_ptr<PredictedObject> >& preds, bool rgb=false);

  private:
    // void toImage(const cv::Mat& img, detection_service_v2::Image &grpc_img, std::string encoding="bgr8");
    void toGrpcImage(const cv::Mat& img, detection_service_v2::Image &grpc_img, bool rgb);
    bool requestDetectObject(const detection_service_v2::Image &req,
            detection_service_v2::InstanceDetections& res);

    void toPredictions(const detection_service_v2::InstanceDetections &res,
            std::vector<std::shared_ptr<PredictedObject> >& preds);

    std::shared_ptr<grpc::Channel> _channel;
    std::unique_ptr<detection_service_v2::InstanceDetectionService::Stub> _stub;
    bool _verbose;

};
}
#endif //ORB_SLAM2_GRPCOBJECTDETECTORV2_H
