#pragma once

#include <cstdint>
#include <memory>

#include "../../native/sdk.hpp"

namespace lingtu::drivers::orbbec {

using lingtu::drivers::camera::CameraConfig;
using lingtu::drivers::camera::DeviceInfo;
using lingtu::drivers::camera::Frame;
using lingtu::drivers::camera::Frames;
using lingtu::drivers::camera::Intrinsics;
using lingtu::drivers::camera::PixelFormat;
using lingtu::drivers::camera::RosTopics;
using lingtu::drivers::camera::VideoMode;
using lingtu::drivers::camera::default_topics;

class Camera final : public lingtu::drivers::camera::Service {
 public:
  Camera();
  ~Camera() override;

  Camera(const Camera &) = delete;
  Camera &operator=(const Camera &) = delete;
  Camera(Camera &&) noexcept;
  Camera &operator=(Camera &&) noexcept;

  void connect(const CameraConfig &config) override;
  void disconnect() noexcept override;
  bool is_connected() const noexcept override;

  DeviceInfo info() const override;
  Intrinsics intrinsics(double depth_scale_m = 0.001) const override;
  Frames read(uint32_t timeout_ms) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::drivers::orbbec
