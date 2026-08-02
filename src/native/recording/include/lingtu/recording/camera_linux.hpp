#pragma once

#if !defined(__linux__)
#error "lingtu camera Linux adapters require Linux"
#endif

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "lingtu/recording/camera_recording.hpp"

namespace lingtu::recording {

struct PosixShmCameraSourceOptions {
  std::string name{"/lingtu_camera_color"};
  std::chrono::milliseconds poll_interval{2};
  std::uint64_t max_frames{0};
  std::function<bool()> stop_requested;
};

class PosixShmCameraFrameSource final : public CameraFrameSource {
 public:
  explicit PosixShmCameraFrameSource(PosixShmCameraSourceOptions options = {});
  ~PosixShmCameraFrameSource() override;

  PosixShmCameraFrameSource(const PosixShmCameraFrameSource &) = delete;
  PosixShmCameraFrameSource &operator=(const PosixShmCameraFrameSource &) = delete;

  bool next(CameraFrame &frame) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct FfmpegSegmentEncoderOptions {
  // libx264 is the supported generic software baseline. It remains an external
  // FFmpeg runtime choice rather than a linked or accelerator-specific dependency.
  std::string executable{"ffmpeg"};
  std::string codec{"libx264"};
};

class FfmpegSegmentEncoder final : public CameraSegmentEncoder {
 public:
  explicit FfmpegSegmentEncoder(FfmpegSegmentEncoderOptions options = {});
  ~FfmpegSegmentEncoder() override;

  FfmpegSegmentEncoder(const FfmpegSegmentEncoder &) = delete;
  FfmpegSegmentEncoder &operator=(const FfmpegSegmentEncoder &) = delete;

  std::string codec() const override;
  std::string container_extension() const override;
  // Checks the configured executable/codec/pixel-format/container combination
  // before a camera SHM ring is opened for a recording session.
  void verify();
  void begin(const std::filesystem::path &temporary_path,
             const CameraStreamDescription &stream) override;
  void write(const CameraFrame &frame) override;
  void finish() override;
  void abort() noexcept override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::recording
