#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace lingtu::drivers::camera {

struct Topics {
  std::string camera_namespace;

  std::string color_image;
  std::string color_camera_info;
  std::string color_metadata;
  std::string color_undistorted_image;
  std::string left_color_image;
  std::string left_color_camera_info;
  std::string right_color_image;
  std::string right_color_camera_info;

  std::string depth_image;
  std::string depth_camera_info;
  std::string depth_metadata;
  std::string depth_unaligned_image;
  std::string depth_points;
  std::string depth_registered_points;
  std::string depth_to_color_image;

  std::string ir_image;
  std::string ir_camera_info;
  std::string left_ir_image;
  std::string left_ir_camera_info;
  std::string right_ir_image;
  std::string right_ir_camera_info;

  std::string gyro_sample;
  std::string gyro_imu_info;
  std::string accel_sample;
  std::string accel_imu_info;
  std::string gyro_accel_sample;

  std::string depth_to_ir_extrinsics;
  std::string depth_to_color_extrinsics;
  std::string depth_to_left_ir_extrinsics;
  std::string depth_to_right_ir_extrinsics;
  std::string depth_to_accel_extrinsics;
  std::string depth_to_gyro_extrinsics;
  std::string left_color_to_right_color_extrinsics;

  std::string device_status;
  std::string depth_filter_status;
  std::string depth_filters_status;
};

Topics default_topics(const std::string &camera_namespace = "/camera");

enum class PixelFormat : uint32_t {
  kUnknown = 0,
  kRgb8 = 1,
  kBgr8 = 2,
  kDepthU16 = 3,
};

struct VideoMode {
  uint32_t width = 640;
  uint32_t height = 480;
  uint32_t fps = 30;
};

struct Config {
  std::string sdk_config_path;
  std::string serial_number;
  std::string uid;
  int product_id = 0;
  uint32_t device_index = 0;
  bool use_device_index = false;
  uint32_t connect_timeout_ms = 10000;
  VideoMode color;
  VideoMode depth;
  bool enable_frame_sync = false;
};

struct DeviceInfo {
  std::string name;
  std::string serial_number;
  std::string firmware_version;
  std::string connection_type;
  std::string uid;
  int pid = 0;
  int vid = 0;
};

struct Intrinsics {
  uint32_t width = 0;
  uint32_t height = 0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double depth_scale_m = 0.001;
  double dist_k1 = 0.0;
  double dist_k2 = 0.0;
  double dist_p1 = 0.0;
  double dist_p2 = 0.0;
  double dist_k3 = 0.0;
};

struct Frame {
  uint32_t width = 0;
  uint32_t height = 0;
  uint32_t channels = 0;
  PixelFormat format = PixelFormat::kUnknown;
  const void *data = nullptr;
  uint32_t data_size = 0;
  std::shared_ptr<const void> owner;

  bool valid() const noexcept {
    return data != nullptr && data_size > 0 && format != PixelFormat::kUnknown;
  }
};

struct Frames {
  Frame color;
  Frame depth;
  double depth_scale_m = 0.001;

  bool valid() const noexcept {
    return color.valid() || depth.valid();
  }
};

class Service {
 public:
  virtual ~Service() = default;

  virtual void connect(const Config &config) = 0;
  virtual void disconnect() noexcept = 0;
  virtual bool is_connected() const noexcept = 0;

  virtual DeviceInfo info() const = 0;
  virtual Intrinsics intrinsics(double depth_scale_m = 0.001) const = 0;
  virtual Frames read(uint32_t timeout_ms) = 0;
};

using RosTopics = Topics;
using CameraConfig = Config;

}  // namespace lingtu::drivers::camera
