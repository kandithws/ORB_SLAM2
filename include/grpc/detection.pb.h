// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: detection.proto

#ifndef PROTOBUF_INCLUDED_detection_2eproto
#define PROTOBUF_INCLUDED_detection_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_detection_2eproto 

namespace protobuf_detection_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[5];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_detection_2eproto
namespace detection_service {
class Detection;
class DetectionDefaultTypeInternal;
extern DetectionDefaultTypeInternal _Detection_default_instance_;
class Detections;
class DetectionsDefaultTypeInternal;
extern DetectionsDefaultTypeInternal _Detections_default_instance_;
class Image;
class ImageDefaultTypeInternal;
extern ImageDefaultTypeInternal _Image_default_instance_;
class Point2d;
class Point2dDefaultTypeInternal;
extern Point2dDefaultTypeInternal _Point2d_default_instance_;
class Rect;
class RectDefaultTypeInternal;
extern RectDefaultTypeInternal _Rect_default_instance_;
}  // namespace detection_service
namespace google {
namespace protobuf {
template<> ::detection_service::Detection* Arena::CreateMaybeMessage<::detection_service::Detection>(Arena*);
template<> ::detection_service::Detections* Arena::CreateMaybeMessage<::detection_service::Detections>(Arena*);
template<> ::detection_service::Image* Arena::CreateMaybeMessage<::detection_service::Image>(Arena*);
template<> ::detection_service::Point2d* Arena::CreateMaybeMessage<::detection_service::Point2d>(Arena*);
template<> ::detection_service::Rect* Arena::CreateMaybeMessage<::detection_service::Rect>(Arena*);
}  // namespace protobuf
}  // namespace google
namespace detection_service {

// ===================================================================

class Image : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:detection_service.Image) */ {
 public:
  Image();
  virtual ~Image();

  Image(const Image& from);

  inline Image& operator=(const Image& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Image(Image&& from) noexcept
    : Image() {
    *this = ::std::move(from);
  }

  inline Image& operator=(Image&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Image& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Image* internal_default_instance() {
    return reinterpret_cast<const Image*>(
               &_Image_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Image* other);
  friend void swap(Image& a, Image& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Image* New() const final {
    return CreateMaybeMessage<Image>(NULL);
  }

  Image* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Image>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Image& from);
  void MergeFrom(const Image& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Image* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // string encoding = 4;
  void clear_encoding();
  static const int kEncodingFieldNumber = 4;
  const ::std::string& encoding() const;
  void set_encoding(const ::std::string& value);
  #if LANG_CXX11
  void set_encoding(::std::string&& value);
  #endif
  void set_encoding(const char* value);
  void set_encoding(const char* value, size_t size);
  ::std::string* mutable_encoding();
  ::std::string* release_encoding();
  void set_allocated_encoding(::std::string* encoding);

  // bytes data = 5;
  void clear_data();
  static const int kDataFieldNumber = 5;
  const ::std::string& data() const;
  void set_data(const ::std::string& value);
  #if LANG_CXX11
  void set_data(::std::string&& value);
  #endif
  void set_data(const char* value);
  void set_data(const void* value, size_t size);
  ::std::string* mutable_data();
  ::std::string* release_data();
  void set_allocated_data(::std::string* data);

  // uint32 width = 1;
  void clear_width();
  static const int kWidthFieldNumber = 1;
  ::google::protobuf::uint32 width() const;
  void set_width(::google::protobuf::uint32 value);

  // uint32 height = 2;
  void clear_height();
  static const int kHeightFieldNumber = 2;
  ::google::protobuf::uint32 height() const;
  void set_height(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:detection_service.Image)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr encoding_;
  ::google::protobuf::internal::ArenaStringPtr data_;
  ::google::protobuf::uint32 width_;
  ::google::protobuf::uint32 height_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_detection_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Point2d : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:detection_service.Point2d) */ {
 public:
  Point2d();
  virtual ~Point2d();

  Point2d(const Point2d& from);

  inline Point2d& operator=(const Point2d& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Point2d(Point2d&& from) noexcept
    : Point2d() {
    *this = ::std::move(from);
  }

  inline Point2d& operator=(Point2d&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Point2d& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Point2d* internal_default_instance() {
    return reinterpret_cast<const Point2d*>(
               &_Point2d_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(Point2d* other);
  friend void swap(Point2d& a, Point2d& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Point2d* New() const final {
    return CreateMaybeMessage<Point2d>(NULL);
  }

  Point2d* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Point2d>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Point2d& from);
  void MergeFrom(const Point2d& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Point2d* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // int32 x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  ::google::protobuf::int32 x() const;
  void set_x(::google::protobuf::int32 value);

  // int32 y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  ::google::protobuf::int32 y() const;
  void set_y(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:detection_service.Point2d)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 x_;
  ::google::protobuf::int32 y_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_detection_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Rect : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:detection_service.Rect) */ {
 public:
  Rect();
  virtual ~Rect();

  Rect(const Rect& from);

  inline Rect& operator=(const Rect& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Rect(Rect&& from) noexcept
    : Rect() {
    *this = ::std::move(from);
  }

  inline Rect& operator=(Rect&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Rect& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Rect* internal_default_instance() {
    return reinterpret_cast<const Rect*>(
               &_Rect_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    2;

  void Swap(Rect* other);
  friend void swap(Rect& a, Rect& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Rect* New() const final {
    return CreateMaybeMessage<Rect>(NULL);
  }

  Rect* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Rect>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Rect& from);
  void MergeFrom(const Rect& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Rect* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .detection_service.Point2d tl = 1;
  bool has_tl() const;
  void clear_tl();
  static const int kTlFieldNumber = 1;
  private:
  const ::detection_service::Point2d& _internal_tl() const;
  public:
  const ::detection_service::Point2d& tl() const;
  ::detection_service::Point2d* release_tl();
  ::detection_service::Point2d* mutable_tl();
  void set_allocated_tl(::detection_service::Point2d* tl);

  // .detection_service.Point2d br = 2;
  bool has_br() const;
  void clear_br();
  static const int kBrFieldNumber = 2;
  private:
  const ::detection_service::Point2d& _internal_br() const;
  public:
  const ::detection_service::Point2d& br() const;
  ::detection_service::Point2d* release_br();
  ::detection_service::Point2d* mutable_br();
  void set_allocated_br(::detection_service::Point2d* br);

  // @@protoc_insertion_point(class_scope:detection_service.Rect)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::detection_service::Point2d* tl_;
  ::detection_service::Point2d* br_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_detection_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Detection : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:detection_service.Detection) */ {
 public:
  Detection();
  virtual ~Detection();

  Detection(const Detection& from);

  inline Detection& operator=(const Detection& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Detection(Detection&& from) noexcept
    : Detection() {
    *this = ::std::move(from);
  }

  inline Detection& operator=(Detection&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Detection& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Detection* internal_default_instance() {
    return reinterpret_cast<const Detection*>(
               &_Detection_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    3;

  void Swap(Detection* other);
  friend void swap(Detection& a, Detection& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Detection* New() const final {
    return CreateMaybeMessage<Detection>(NULL);
  }

  Detection* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Detection>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Detection& from);
  void MergeFrom(const Detection& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Detection* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // string label = 3;
  void clear_label();
  static const int kLabelFieldNumber = 3;
  const ::std::string& label() const;
  void set_label(const ::std::string& value);
  #if LANG_CXX11
  void set_label(::std::string&& value);
  #endif
  void set_label(const char* value);
  void set_label(const char* value, size_t size);
  ::std::string* mutable_label();
  ::std::string* release_label();
  void set_allocated_label(::std::string* label);

  // .detection_service.Rect box = 4;
  bool has_box() const;
  void clear_box();
  static const int kBoxFieldNumber = 4;
  private:
  const ::detection_service::Rect& _internal_box() const;
  public:
  const ::detection_service::Rect& box() const;
  ::detection_service::Rect* release_box();
  ::detection_service::Rect* mutable_box();
  void set_allocated_box(::detection_service::Rect* box);

  // float confidence = 1;
  void clear_confidence();
  static const int kConfidenceFieldNumber = 1;
  float confidence() const;
  void set_confidence(float value);

  // uint32 label_id = 2;
  void clear_label_id();
  static const int kLabelIdFieldNumber = 2;
  ::google::protobuf::uint32 label_id() const;
  void set_label_id(::google::protobuf::uint32 value);

  // @@protoc_insertion_point(class_scope:detection_service.Detection)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::ArenaStringPtr label_;
  ::detection_service::Rect* box_;
  float confidence_;
  ::google::protobuf::uint32 label_id_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_detection_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class Detections : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:detection_service.Detections) */ {
 public:
  Detections();
  virtual ~Detections();

  Detections(const Detections& from);

  inline Detections& operator=(const Detections& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Detections(Detections&& from) noexcept
    : Detections() {
    *this = ::std::move(from);
  }

  inline Detections& operator=(Detections&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Detections& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Detections* internal_default_instance() {
    return reinterpret_cast<const Detections*>(
               &_Detections_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    4;

  void Swap(Detections* other);
  friend void swap(Detections& a, Detections& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Detections* New() const final {
    return CreateMaybeMessage<Detections>(NULL);
  }

  Detections* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Detections>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Detections& from);
  void MergeFrom(const Detections& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Detections* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .detection_service.Detection detections = 1;
  int detections_size() const;
  void clear_detections();
  static const int kDetectionsFieldNumber = 1;
  ::detection_service::Detection* mutable_detections(int index);
  ::google::protobuf::RepeatedPtrField< ::detection_service::Detection >*
      mutable_detections();
  const ::detection_service::Detection& detections(int index) const;
  ::detection_service::Detection* add_detections();
  const ::google::protobuf::RepeatedPtrField< ::detection_service::Detection >&
      detections() const;

  // @@protoc_insertion_point(class_scope:detection_service.Detections)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::detection_service::Detection > detections_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_detection_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Image

// uint32 width = 1;
inline void Image::clear_width() {
  width_ = 0u;
}
inline ::google::protobuf::uint32 Image::width() const {
  // @@protoc_insertion_point(field_get:detection_service.Image.width)
  return width_;
}
inline void Image::set_width(::google::protobuf::uint32 value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:detection_service.Image.width)
}

// uint32 height = 2;
inline void Image::clear_height() {
  height_ = 0u;
}
inline ::google::protobuf::uint32 Image::height() const {
  // @@protoc_insertion_point(field_get:detection_service.Image.height)
  return height_;
}
inline void Image::set_height(::google::protobuf::uint32 value) {
  
  height_ = value;
  // @@protoc_insertion_point(field_set:detection_service.Image.height)
}

// string encoding = 4;
inline void Image::clear_encoding() {
  encoding_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Image::encoding() const {
  // @@protoc_insertion_point(field_get:detection_service.Image.encoding)
  return encoding_.GetNoArena();
}
inline void Image::set_encoding(const ::std::string& value) {
  
  encoding_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:detection_service.Image.encoding)
}
#if LANG_CXX11
inline void Image::set_encoding(::std::string&& value) {
  
  encoding_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:detection_service.Image.encoding)
}
#endif
inline void Image::set_encoding(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  encoding_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:detection_service.Image.encoding)
}
inline void Image::set_encoding(const char* value, size_t size) {
  
  encoding_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:detection_service.Image.encoding)
}
inline ::std::string* Image::mutable_encoding() {
  
  // @@protoc_insertion_point(field_mutable:detection_service.Image.encoding)
  return encoding_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Image::release_encoding() {
  // @@protoc_insertion_point(field_release:detection_service.Image.encoding)
  
  return encoding_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Image::set_allocated_encoding(::std::string* encoding) {
  if (encoding != NULL) {
    
  } else {
    
  }
  encoding_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), encoding);
  // @@protoc_insertion_point(field_set_allocated:detection_service.Image.encoding)
}

// bytes data = 5;
inline void Image::clear_data() {
  data_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Image::data() const {
  // @@protoc_insertion_point(field_get:detection_service.Image.data)
  return data_.GetNoArena();
}
inline void Image::set_data(const ::std::string& value) {
  
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:detection_service.Image.data)
}
#if LANG_CXX11
inline void Image::set_data(::std::string&& value) {
  
  data_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:detection_service.Image.data)
}
#endif
inline void Image::set_data(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:detection_service.Image.data)
}
inline void Image::set_data(const void* value, size_t size) {
  
  data_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:detection_service.Image.data)
}
inline ::std::string* Image::mutable_data() {
  
  // @@protoc_insertion_point(field_mutable:detection_service.Image.data)
  return data_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Image::release_data() {
  // @@protoc_insertion_point(field_release:detection_service.Image.data)
  
  return data_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Image::set_allocated_data(::std::string* data) {
  if (data != NULL) {
    
  } else {
    
  }
  data_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), data);
  // @@protoc_insertion_point(field_set_allocated:detection_service.Image.data)
}

// -------------------------------------------------------------------

// Point2d

// int32 x = 1;
inline void Point2d::clear_x() {
  x_ = 0;
}
inline ::google::protobuf::int32 Point2d::x() const {
  // @@protoc_insertion_point(field_get:detection_service.Point2d.x)
  return x_;
}
inline void Point2d::set_x(::google::protobuf::int32 value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:detection_service.Point2d.x)
}

// int32 y = 2;
inline void Point2d::clear_y() {
  y_ = 0;
}
inline ::google::protobuf::int32 Point2d::y() const {
  // @@protoc_insertion_point(field_get:detection_service.Point2d.y)
  return y_;
}
inline void Point2d::set_y(::google::protobuf::int32 value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:detection_service.Point2d.y)
}

// -------------------------------------------------------------------

// Rect

// .detection_service.Point2d tl = 1;
inline bool Rect::has_tl() const {
  return this != internal_default_instance() && tl_ != NULL;
}
inline void Rect::clear_tl() {
  if (GetArenaNoVirtual() == NULL && tl_ != NULL) {
    delete tl_;
  }
  tl_ = NULL;
}
inline const ::detection_service::Point2d& Rect::_internal_tl() const {
  return *tl_;
}
inline const ::detection_service::Point2d& Rect::tl() const {
  const ::detection_service::Point2d* p = tl_;
  // @@protoc_insertion_point(field_get:detection_service.Rect.tl)
  return p != NULL ? *p : *reinterpret_cast<const ::detection_service::Point2d*>(
      &::detection_service::_Point2d_default_instance_);
}
inline ::detection_service::Point2d* Rect::release_tl() {
  // @@protoc_insertion_point(field_release:detection_service.Rect.tl)
  
  ::detection_service::Point2d* temp = tl_;
  tl_ = NULL;
  return temp;
}
inline ::detection_service::Point2d* Rect::mutable_tl() {
  
  if (tl_ == NULL) {
    auto* p = CreateMaybeMessage<::detection_service::Point2d>(GetArenaNoVirtual());
    tl_ = p;
  }
  // @@protoc_insertion_point(field_mutable:detection_service.Rect.tl)
  return tl_;
}
inline void Rect::set_allocated_tl(::detection_service::Point2d* tl) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete tl_;
  }
  if (tl) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      tl = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, tl, submessage_arena);
    }
    
  } else {
    
  }
  tl_ = tl;
  // @@protoc_insertion_point(field_set_allocated:detection_service.Rect.tl)
}

// .detection_service.Point2d br = 2;
inline bool Rect::has_br() const {
  return this != internal_default_instance() && br_ != NULL;
}
inline void Rect::clear_br() {
  if (GetArenaNoVirtual() == NULL && br_ != NULL) {
    delete br_;
  }
  br_ = NULL;
}
inline const ::detection_service::Point2d& Rect::_internal_br() const {
  return *br_;
}
inline const ::detection_service::Point2d& Rect::br() const {
  const ::detection_service::Point2d* p = br_;
  // @@protoc_insertion_point(field_get:detection_service.Rect.br)
  return p != NULL ? *p : *reinterpret_cast<const ::detection_service::Point2d*>(
      &::detection_service::_Point2d_default_instance_);
}
inline ::detection_service::Point2d* Rect::release_br() {
  // @@protoc_insertion_point(field_release:detection_service.Rect.br)
  
  ::detection_service::Point2d* temp = br_;
  br_ = NULL;
  return temp;
}
inline ::detection_service::Point2d* Rect::mutable_br() {
  
  if (br_ == NULL) {
    auto* p = CreateMaybeMessage<::detection_service::Point2d>(GetArenaNoVirtual());
    br_ = p;
  }
  // @@protoc_insertion_point(field_mutable:detection_service.Rect.br)
  return br_;
}
inline void Rect::set_allocated_br(::detection_service::Point2d* br) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete br_;
  }
  if (br) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      br = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, br, submessage_arena);
    }
    
  } else {
    
  }
  br_ = br;
  // @@protoc_insertion_point(field_set_allocated:detection_service.Rect.br)
}

// -------------------------------------------------------------------

// Detection

// float confidence = 1;
inline void Detection::clear_confidence() {
  confidence_ = 0;
}
inline float Detection::confidence() const {
  // @@protoc_insertion_point(field_get:detection_service.Detection.confidence)
  return confidence_;
}
inline void Detection::set_confidence(float value) {
  
  confidence_ = value;
  // @@protoc_insertion_point(field_set:detection_service.Detection.confidence)
}

// uint32 label_id = 2;
inline void Detection::clear_label_id() {
  label_id_ = 0u;
}
inline ::google::protobuf::uint32 Detection::label_id() const {
  // @@protoc_insertion_point(field_get:detection_service.Detection.label_id)
  return label_id_;
}
inline void Detection::set_label_id(::google::protobuf::uint32 value) {
  
  label_id_ = value;
  // @@protoc_insertion_point(field_set:detection_service.Detection.label_id)
}

// string label = 3;
inline void Detection::clear_label() {
  label_.ClearToEmptyNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline const ::std::string& Detection::label() const {
  // @@protoc_insertion_point(field_get:detection_service.Detection.label)
  return label_.GetNoArena();
}
inline void Detection::set_label(const ::std::string& value) {
  
  label_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:detection_service.Detection.label)
}
#if LANG_CXX11
inline void Detection::set_label(::std::string&& value) {
  
  label_.SetNoArena(
    &::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:detection_service.Detection.label)
}
#endif
inline void Detection::set_label(const char* value) {
  GOOGLE_DCHECK(value != NULL);
  
  label_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:detection_service.Detection.label)
}
inline void Detection::set_label(const char* value, size_t size) {
  
  label_.SetNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:detection_service.Detection.label)
}
inline ::std::string* Detection::mutable_label() {
  
  // @@protoc_insertion_point(field_mutable:detection_service.Detection.label)
  return label_.MutableNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline ::std::string* Detection::release_label() {
  // @@protoc_insertion_point(field_release:detection_service.Detection.label)
  
  return label_.ReleaseNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited());
}
inline void Detection::set_allocated_label(::std::string* label) {
  if (label != NULL) {
    
  } else {
    
  }
  label_.SetAllocatedNoArena(&::google::protobuf::internal::GetEmptyStringAlreadyInited(), label);
  // @@protoc_insertion_point(field_set_allocated:detection_service.Detection.label)
}

// .detection_service.Rect box = 4;
inline bool Detection::has_box() const {
  return this != internal_default_instance() && box_ != NULL;
}
inline void Detection::clear_box() {
  if (GetArenaNoVirtual() == NULL && box_ != NULL) {
    delete box_;
  }
  box_ = NULL;
}
inline const ::detection_service::Rect& Detection::_internal_box() const {
  return *box_;
}
inline const ::detection_service::Rect& Detection::box() const {
  const ::detection_service::Rect* p = box_;
  // @@protoc_insertion_point(field_get:detection_service.Detection.box)
  return p != NULL ? *p : *reinterpret_cast<const ::detection_service::Rect*>(
      &::detection_service::_Rect_default_instance_);
}
inline ::detection_service::Rect* Detection::release_box() {
  // @@protoc_insertion_point(field_release:detection_service.Detection.box)
  
  ::detection_service::Rect* temp = box_;
  box_ = NULL;
  return temp;
}
inline ::detection_service::Rect* Detection::mutable_box() {
  
  if (box_ == NULL) {
    auto* p = CreateMaybeMessage<::detection_service::Rect>(GetArenaNoVirtual());
    box_ = p;
  }
  // @@protoc_insertion_point(field_mutable:detection_service.Detection.box)
  return box_;
}
inline void Detection::set_allocated_box(::detection_service::Rect* box) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete box_;
  }
  if (box) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      box = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, box, submessage_arena);
    }
    
  } else {
    
  }
  box_ = box;
  // @@protoc_insertion_point(field_set_allocated:detection_service.Detection.box)
}

// -------------------------------------------------------------------

// Detections

// repeated .detection_service.Detection detections = 1;
inline int Detections::detections_size() const {
  return detections_.size();
}
inline void Detections::clear_detections() {
  detections_.Clear();
}
inline ::detection_service::Detection* Detections::mutable_detections(int index) {
  // @@protoc_insertion_point(field_mutable:detection_service.Detections.detections)
  return detections_.Mutable(index);
}
inline ::google::protobuf::RepeatedPtrField< ::detection_service::Detection >*
Detections::mutable_detections() {
  // @@protoc_insertion_point(field_mutable_list:detection_service.Detections.detections)
  return &detections_;
}
inline const ::detection_service::Detection& Detections::detections(int index) const {
  // @@protoc_insertion_point(field_get:detection_service.Detections.detections)
  return detections_.Get(index);
}
inline ::detection_service::Detection* Detections::add_detections() {
  // @@protoc_insertion_point(field_add:detection_service.Detections.detections)
  return detections_.Add();
}
inline const ::google::protobuf::RepeatedPtrField< ::detection_service::Detection >&
Detections::detections() const {
  // @@protoc_insertion_point(field_list:detection_service.Detections.detections)
  return detections_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace detection_service

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_detection_2eproto
