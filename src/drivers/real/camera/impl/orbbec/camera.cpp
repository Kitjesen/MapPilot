#include "camera.hpp"

#include <libobsensor/ObSensor.hpp>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace lingtu::drivers::orbbec {
namespace {

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
  const auto dist = impl_->camera_param.rgbDistortion;
  out.dist_k1 = dist.k1;
  out.dist_k2 = dist.k2;
  out.dist_p1 = dist.p1;
  out.dist_p2 = dist.p2;
  out.dist_k3 = dist.k3;
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
