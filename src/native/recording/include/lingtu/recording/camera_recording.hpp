#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::recording {

inline constexpr const char *kCameraMcapProfile = "lingtu.camera.v1";

struct CameraFrame {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t stride{0};
  std::string encoding;
  std::string frame_id;
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double depth_scale{0.001};
  double dist_k1{0.0};
  double dist_k2{0.0};
  double dist_p1{0.0};
  double dist_p2{0.0};
  double dist_k3{0.0};
  std::vector<std::byte> payload;
};

struct CameraStreamDescription {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t stride{0};
  std::string encoding;
  std::string frame_id;
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double depth_scale{0.001};
  double dist_k1{0.0};
  double dist_k2{0.0};
  double dist_p1{0.0};
  double dist_p2{0.0};
  double dist_k3{0.0};
  double nominal_fps{30.0};
};

struct CameraSegment {
  std::uint32_t index{0};
  std::string camera_id;
  std::string relative_path;
  std::string codec;
  std::string source_encoding;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint64_t start_time_ns{0};
  std::uint64_t end_time_ns{0};
  std::uint64_t first_sequence{0};
  std::uint64_t last_sequence{0};
  std::uint64_t frame_count{0};
  std::uint64_t dropped_frames{0};
  std::uint64_t byte_size{0};
  std::uint32_t crc32{0};
  std::string frame_id;
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  double depth_scale{0.001};
  double dist_k1{0.0};
  double dist_k2{0.0};
  double dist_p1{0.0};
  double dist_p2{0.0};
  double dist_k3{0.0};

  bool operator==(const CameraSegment &other) const noexcept;
};

struct CameraRecordingOptions {
  std::filesystem::path session_directory;
  std::string camera_id{"color"};
  std::uint64_t segment_duration_ns{5'000'000'000ULL};
  double nominal_fps{30.0};
};

class CameraFrameSource {
 public:
  virtual ~CameraFrameSource() = default;
  virtual bool next(CameraFrame &frame) = 0;
};

class CameraSegmentEncoder {
 public:
  virtual ~CameraSegmentEncoder() = default;
  virtual std::string codec() const = 0;
  virtual std::string container_extension() const = 0;
  virtual void begin(const std::filesystem::path &temporary_path,
                     const CameraStreamDescription &stream) = 0;
  virtual void write(const CameraFrame &frame) = 0;
  virtual void finish() = 0;
  virtual void abort() noexcept = 0;
};

class CameraSegmentRecorder {
 public:
  CameraSegmentRecorder(CameraRecordingOptions options, CameraFrameSource &source,
                        CameraSegmentEncoder &encoder);

  std::vector<CameraSegment> record();

 private:
  CameraRecordingOptions options_;
  CameraFrameSource &source_;
  CameraSegmentEncoder &encoder_;
};

struct CameraVerificationIssue {
  std::filesystem::path path;
  std::string message;
};

std::filesystem::path camera_index_path(const std::filesystem::path &session_directory,
                                        const std::string &camera_id);

std::filesystem::path resolve_camera_segment_path(const std::filesystem::path &index_path,
                                                  const std::string &relative_path);

std::vector<CameraSegment> read_camera_segment_index(const std::filesystem::path &index_path);

std::vector<CameraVerificationIssue>
verify_camera_segments(const std::filesystem::path &index_path);

std::vector<std::filesystem::path>
extract_camera_window(const std::filesystem::path &index_path,
                      const std::filesystem::path &output_directory, std::uint64_t start_time_ns,
                      std::uint64_t end_time_ns);

}  // namespace lingtu::recording
