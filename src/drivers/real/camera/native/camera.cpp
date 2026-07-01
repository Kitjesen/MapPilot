#include "camera.hpp"

#include <libobsensor/ObSensor.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace lingtu::drivers::orbbec {
namespace {

std::string make_topic(const std::string &camera_namespace, const std::string &name) {
  std::string ns = camera_namespace.empty() ? "camera" : camera_namespace;
  if (ns.front() != '/') {
    ns.insert(ns.begin(), '/');
  }
  while (ns.size() > 1 && ns.back() == '/') {
    ns.pop_back();
  }
  return ns + "/" + name;
}

std::string safe_cstr(const char *value) {
  return value == nullptr ? std::string{} : std::string(value);
}

std::shared_ptr<ob::Config> make_streams(const CameraConfig &options) {
  auto config = std::make_shared<ob::Config>();
  config->enableVideoStream(
      OB_STREAM_COLOR,
      options.color.width,
      options.color.height,
      options.color.fps,
      OB_FORMAT_RGB);
  config->enableVideoStream(
      OB_STREAM_DEPTH,
      options.depth.width,
      options.depth.height,
      options.depth.fps,
      OB_FORMAT_Y16);
  return config;
}

PixelFormat color_format(OBFormat format) {
  if (format == OB_FORMAT_RGB) {
    return PixelFormat::kRgb8;
  }
  if (format == OB_FORMAT_BGR) {
    return PixelFormat::kBgr8;
  }
  return PixelFormat::kUnknown;
}

bool is_depth_format(OBFormat format) {
  return format == OB_FORMAT_Y16 || format == OB_FORMAT_Z16;
}

Frame color_frame(const std::shared_ptr<ob::ColorFrame> &color) {
  if (!color) {
    return {};
  }
  const auto format = color_format(color->getFormat());
  if (format == PixelFormat::kUnknown) {
    return {};
  }

  Frame view;
  view.width = color->getWidth();
  view.height = color->getHeight();
  view.channels = 3;
  view.format = format;
  view.data = color->getData();
  view.data_size = color->getDataSize();
  view.owner = std::shared_ptr<const void>(color, static_cast<const void *>(color.get()));
  return view;
}

Frame depth_frame(const std::shared_ptr<ob::DepthFrame> &depth) {
  if (!depth || !is_depth_format(depth->getFormat())) {
    return {};
  }

  Frame view;
  view.width = depth->getWidth();
  view.height = depth->getHeight();
  view.channels = 1;
  view.format = PixelFormat::kDepthU16;
  view.data = depth->getData();
  view.data_size = depth->getDataSize();
  view.owner = std::shared_ptr<const void>(depth, static_cast<const void *>(depth.get()));
  return view;
}

double depth_scale(const std::shared_ptr<ob::DepthFrame> &depth) {
  if (!depth) {
    return 0.001;
  }
  return static_cast<double>(depth->getValueScale()) * 0.001;
}

DeviceInfo device_info(const std::shared_ptr<ob::Device> &device) {
  if (!device) {
    return {};
  }
  const auto info = device->getDeviceInfo();
  if (!info) {
    return {};
  }

  DeviceInfo out;
  out.name = safe_cstr(info->getName());
  out.serial_number = safe_cstr(info->getSerialNumber());
  out.firmware_version = safe_cstr(info->getFirmwareVersion());
  out.connection_type = safe_cstr(info->getConnectionType());
  out.uid = safe_cstr(info->getUid());
  out.pid = info->getPid();
  out.vid = info->getVid();
  return out;
}

void check_product_id(const DeviceInfo &info, int product_id) {
  if (product_id != 0 && info.pid != product_id) {
    throw std::runtime_error("selected Orbbec device product_id does not match");
  }
}

std::shared_ptr<ob::Device> open_device(
    ob::Context &context,
    const CameraConfig &config) {
  const auto start = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::milliseconds(config.connect_timeout_ms);
  std::string last_error = "no Orbbec device found";

  while (true) {
    auto list = context.queryDeviceList();
    const uint32_t count = list ? list->getCount() : 0;
    if (count > 0) {
      try {
        if (!config.serial_number.empty()) {
          auto device = list->getDeviceBySN(config.serial_number.c_str());
          check_product_id(device_info(device), config.product_id);
          return device;
        }
        if (!config.uid.empty()) {
          auto device = list->getDeviceByUid(config.uid.c_str());
          check_product_id(device_info(device), config.product_id);
          return device;
        }
        if (config.product_id != 0) {
          for (uint32_t i = 0; i < count; ++i) {
            if (list->getPid(i) == config.product_id) {
              return list->getDevice(i);
            }
          }
          last_error = "no Orbbec device matched product_id";
        } else {
          const uint32_t index = config.use_device_index ? config.device_index : 0;
          if (index < count) {
            return list->getDevice(index);
          }
          last_error = "Orbbec device_index out of range";
        }
      } catch (const std::exception &exc) {
        last_error = exc.what();
      }
    }

    if (std::chrono::steady_clock::now() - start >= timeout) {
      throw std::runtime_error(last_error);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

}  // namespace

RosTopics default_topics(const std::string &camera_namespace) {
  RosTopics topics;
  topics.camera_namespace = camera_namespace.empty() ? "/camera" : camera_namespace;
  if (topics.camera_namespace.front() != '/') {
    topics.camera_namespace.insert(topics.camera_namespace.begin(), '/');
  }
  while (topics.camera_namespace.size() > 1 && topics.camera_namespace.back() == '/') {
    topics.camera_namespace.pop_back();
  }

  topics.color_image = make_topic(topics.camera_namespace, "color/image_raw");
  topics.color_camera_info = make_topic(topics.camera_namespace, "color/camera_info");
  topics.color_metadata = make_topic(topics.camera_namespace, "color/metadata");
  topics.color_undistorted_image = make_topic(topics.camera_namespace, "color/image_undistorted");
  topics.left_color_image = make_topic(topics.camera_namespace, "left_color/image_raw");
  topics.left_color_camera_info = make_topic(topics.camera_namespace, "left_color/camera_info");
  topics.right_color_image = make_topic(topics.camera_namespace, "right_color/image_raw");
  topics.right_color_camera_info = make_topic(topics.camera_namespace, "right_color/camera_info");

  topics.depth_image = make_topic(topics.camera_namespace, "depth/image_raw");
  topics.depth_camera_info = make_topic(topics.camera_namespace, "depth/camera_info");
  topics.depth_metadata = make_topic(topics.camera_namespace, "depth/metadata");
  topics.depth_unaligned_image = make_topic(topics.camera_namespace, "depth/image_unaligned");
  topics.depth_points = make_topic(topics.camera_namespace, "depth/points");
  topics.depth_registered_points = make_topic(topics.camera_namespace, "depth_registered/points");
  topics.depth_to_color_image = make_topic(topics.camera_namespace, "depth_to_color/image_raw");

  topics.ir_image = make_topic(topics.camera_namespace, "ir/image_raw");
  topics.ir_camera_info = make_topic(topics.camera_namespace, "ir/camera_info");
  topics.left_ir_image = make_topic(topics.camera_namespace, "left_ir/image_raw");
  topics.left_ir_camera_info = make_topic(topics.camera_namespace, "left_ir/camera_info");
  topics.right_ir_image = make_topic(topics.camera_namespace, "right_ir/image_raw");
  topics.right_ir_camera_info = make_topic(topics.camera_namespace, "right_ir/camera_info");

  topics.gyro_sample = make_topic(topics.camera_namespace, "gyro/sample");
  topics.gyro_imu_info = make_topic(topics.camera_namespace, "gyro/imu_info");
  topics.accel_sample = make_topic(topics.camera_namespace, "accel/sample");
  topics.accel_imu_info = make_topic(topics.camera_namespace, "accel/imu_info");
  topics.gyro_accel_sample = make_topic(topics.camera_namespace, "gyro_accel/sample");

  topics.depth_to_ir_extrinsics = make_topic(topics.camera_namespace, "depth_to_ir");
  topics.depth_to_color_extrinsics = make_topic(topics.camera_namespace, "depth_to_color");
  topics.depth_to_left_ir_extrinsics = make_topic(topics.camera_namespace, "depth_to_left_ir");
  topics.depth_to_right_ir_extrinsics = make_topic(topics.camera_namespace, "depth_to_right_ir");
  topics.depth_to_accel_extrinsics = make_topic(topics.camera_namespace, "depth_to_accel");
  topics.depth_to_gyro_extrinsics = make_topic(topics.camera_namespace, "depth_to_gyro");
  topics.left_color_to_right_color_extrinsics =
      make_topic(topics.camera_namespace, "left_color_to_right_color");

  topics.device_status = make_topic(topics.camera_namespace, "device_status");
  topics.depth_filter_status = make_topic(topics.camera_namespace, "depth_filter_status");
  topics.depth_filters_status = make_topic(topics.camera_namespace, "depth_filters/status");
  return topics;
}

struct Camera::Impl {
  std::unique_ptr<ob::Context> context;
  std::shared_ptr<ob::Device> device;
  std::unique_ptr<ob::Pipeline> pipeline;
  OBCameraParam camera_param{};
  CameraConfig config;
  DeviceInfo device_info;
  bool connected = false;
};

Camera::Camera() : impl_(std::make_unique<Impl>()) {}

Camera::~Camera() {
  disconnect();
}

Camera::Camera(Camera &&) noexcept = default;

Camera &Camera::operator=(Camera &&) noexcept = default;

void Camera::connect(const CameraConfig &config) {
  disconnect();
  impl_->config = config;
  impl_->context = std::make_unique<ob::Context>(config.sdk_config_path.c_str());
  impl_->device = open_device(*impl_->context, config);
  impl_->pipeline = std::make_unique<ob::Pipeline>(impl_->device);
  if (config.enable_frame_sync) {
    impl_->pipeline->enableFrameSync();
  }
  impl_->pipeline->start(make_streams(config));
  impl_->camera_param = impl_->pipeline->getCameraParam();
  impl_->device_info = device_info(impl_->pipeline->getDevice());
  impl_->connected = true;
}

void Camera::disconnect() noexcept {
  if (!impl_) {
    return;
  }
  if (impl_->pipeline && impl_->connected) {
    try {
      impl_->pipeline->stop();
    } catch (...) {
    }
  }
  impl_->pipeline.reset();
  impl_->device.reset();
  impl_->context.reset();
  impl_->connected = false;
}

bool Camera::is_connected() const noexcept {
  return impl_ != nullptr && impl_->connected;
}

DeviceInfo Camera::info() const {
  if (!is_connected()) {
    throw std::runtime_error("Orbbec device is not connected");
  }
  return impl_->device_info;
}

Intrinsics Camera::intrinsics(double depth_scale_m) const {
  if (!is_connected()) {
    throw std::runtime_error("Orbbec device is not connected");
  }

  const auto intr = impl_->camera_param.rgbIntrinsic;
  Intrinsics out;
  out.width = intr.width ? intr.width : impl_->config.color.width;
  out.height = intr.height ? intr.height : impl_->config.color.height;
  out.fx = intr.fx;
  out.fy = intr.fy;
  out.cx = intr.cx;
  out.cy = intr.cy;
  out.depth_scale_m = depth_scale_m;
  return out;
}

Frames Camera::read(uint32_t timeout_ms) {
  if (!is_connected()) {
    throw std::runtime_error("Orbbec device is not connected");
  }

  auto frameset = impl_->pipeline->waitForFrames(timeout_ms);
  if (!frameset) {
    return {};
  }

  auto color = frameset->colorFrame();
  auto depth = frameset->depthFrame();

  Frames out;
  out.color = color_frame(color);
  out.depth = depth_frame(depth);
  out.depth_scale_m = depth_scale(depth);
  return out;
}

}  // namespace lingtu::drivers::orbbec
