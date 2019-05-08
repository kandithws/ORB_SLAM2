// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: detection.proto
#ifndef GRPC_detection_2eproto__INCLUDED
#define GRPC_detection_2eproto__INCLUDED

#include "detection.pb.h"

#include <functional>
#include <grpcpp/impl/codegen/async_generic_service.h>
#include <grpcpp/impl/codegen/async_stream.h>
#include <grpcpp/impl/codegen/async_unary_call.h>
#include <grpcpp/impl/codegen/client_callback.h>
#include <grpcpp/impl/codegen/method_handler_impl.h>
#include <grpcpp/impl/codegen/proto_utils.h>
#include <grpcpp/impl/codegen/rpc_method.h>
#include <grpcpp/impl/codegen/server_callback.h>
#include <grpcpp/impl/codegen/service_type.h>
#include <grpcpp/impl/codegen/status.h>
#include <grpcpp/impl/codegen/stub_options.h>
#include <grpcpp/impl/codegen/sync_stream.h>

namespace grpc {
class CompletionQueue;
class Channel;
class ServerCompletionQueue;
class ServerContext;
}  // namespace grpc

namespace detection_service {

class DetectionService final {
 public:
  static constexpr char const* service_full_name() {
    return "detection_service.DetectionService";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    virtual ::grpc::Status ObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image& request, ::detection_service::Detections* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::detection_service::Detections>> AsyncObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::detection_service::Detections>>(AsyncObjectDetectionRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::detection_service::Detections>> PrepareAsyncObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::detection_service::Detections>>(PrepareAsyncObjectDetectionRaw(context, request, cq));
    }
    class experimental_async_interface {
     public:
      virtual ~experimental_async_interface() {}
      virtual void ObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response, std::function<void(::grpc::Status)>) = 0;
    };
    virtual class experimental_async_interface* experimental_async() { return nullptr; }
  private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::detection_service::Detections>* AsyncObjectDetectionRaw(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) = 0;
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::detection_service::Detections>* PrepareAsyncObjectDetectionRaw(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    ::grpc::Status ObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image& request, ::detection_service::Detections* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::detection_service::Detections>> AsyncObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::detection_service::Detections>>(AsyncObjectDetectionRaw(context, request, cq));
    }
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::detection_service::Detections>> PrepareAsyncObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::detection_service::Detections>>(PrepareAsyncObjectDetectionRaw(context, request, cq));
    }
    class experimental_async final :
      public StubInterface::experimental_async_interface {
     public:
      void ObjectDetection(::grpc::ClientContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response, std::function<void(::grpc::Status)>) override;
     private:
      friend class Stub;
      explicit experimental_async(Stub* stub): stub_(stub) { }
      Stub* stub() { return stub_; }
      Stub* stub_;
    };
    class experimental_async_interface* experimental_async() override { return &async_stub_; }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    class experimental_async async_stub_{this};
    ::grpc::ClientAsyncResponseReader< ::detection_service::Detections>* AsyncObjectDetectionRaw(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) override;
    ::grpc::ClientAsyncResponseReader< ::detection_service::Detections>* PrepareAsyncObjectDetectionRaw(::grpc::ClientContext* context, const ::detection_service::Image& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::internal::RpcMethod rpcmethod_ObjectDetection_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    virtual ::grpc::Status ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_ObjectDetection : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_ObjectDetection() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_ObjectDetection() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestObjectDetection(::grpc::ServerContext* context, ::detection_service::Image* request, ::grpc::ServerAsyncResponseWriter< ::detection_service::Detections>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_ObjectDetection<Service > AsyncService;
  template <class BaseClass>
  class ExperimentalWithCallbackMethod_ObjectDetection : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    ExperimentalWithCallbackMethod_ObjectDetection() {
      ::grpc::Service::experimental().MarkMethodCallback(0,
        new ::grpc::internal::CallbackUnaryHandler< ::detection_service::Image, ::detection_service::Detections>(
          [this](::grpc::ServerContext* context,
                 const ::detection_service::Image* request,
                 ::detection_service::Detections* response,
                 ::grpc::experimental::ServerCallbackRpcController* controller) {
                   return this->ObjectDetection(context, request, response, controller);
                 }));
    }
    ~ExperimentalWithCallbackMethod_ObjectDetection() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual void ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response, ::grpc::experimental::ServerCallbackRpcController* controller) { controller->Finish(::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "")); }
  };
  typedef ExperimentalWithCallbackMethod_ObjectDetection<Service > ExperimentalCallbackService;
  template <class BaseClass>
  class WithGenericMethod_ObjectDetection : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_ObjectDetection() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_ObjectDetection() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithRawMethod_ObjectDetection : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithRawMethod_ObjectDetection() {
      ::grpc::Service::MarkMethodRaw(0);
    }
    ~WithRawMethod_ObjectDetection() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void RequestObjectDetection(::grpc::ServerContext* context, ::grpc::ByteBuffer* request, ::grpc::ServerAsyncResponseWriter< ::grpc::ByteBuffer>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  template <class BaseClass>
  class ExperimentalWithRawCallbackMethod_ObjectDetection : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    ExperimentalWithRawCallbackMethod_ObjectDetection() {
      ::grpc::Service::experimental().MarkMethodRawCallback(0,
        new ::grpc::internal::CallbackUnaryHandler< ::grpc::ByteBuffer, ::grpc::ByteBuffer>(
          [this](::grpc::ServerContext* context,
                 const ::grpc::ByteBuffer* request,
                 ::grpc::ByteBuffer* response,
                 ::grpc::experimental::ServerCallbackRpcController* controller) {
                   this->ObjectDetection(context, request, response, controller);
                 }));
    }
    ~ExperimentalWithRawCallbackMethod_ObjectDetection() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    virtual void ObjectDetection(::grpc::ServerContext* context, const ::grpc::ByteBuffer* request, ::grpc::ByteBuffer* response, ::grpc::experimental::ServerCallbackRpcController* controller) { controller->Finish(::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "")); }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_ObjectDetection : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithStreamedUnaryMethod_ObjectDetection() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::internal::StreamedUnaryHandler< ::detection_service::Image, ::detection_service::Detections>(std::bind(&WithStreamedUnaryMethod_ObjectDetection<BaseClass>::StreamedObjectDetection, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_ObjectDetection() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status ObjectDetection(::grpc::ServerContext* context, const ::detection_service::Image* request, ::detection_service::Detections* response) override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status StreamedObjectDetection(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::detection_service::Image,::detection_service::Detections>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_ObjectDetection<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_ObjectDetection<Service > StreamedService;
};

}  // namespace detection_service


#endif  // GRPC_detection_2eproto__INCLUDED
