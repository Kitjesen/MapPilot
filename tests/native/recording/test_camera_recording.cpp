#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "lingtu/recording/camera_recording.hpp"
#include "mcap/reader.hpp"

namespace {

using lingtu::recording::CameraFrame;
using lingtu::recording::CameraFrameSource;
using lingtu::recording::CameraRecordingOptions;
using lingtu::recording::CameraSegmentEncoder;
using lingtu::recording::CameraSegmentRecorder;
using lingtu::recording::CameraStreamDescription;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class VectorFrameSource final : public CameraFrameSource {
 public:
  explicit VectorFrameSource(std::vector<CameraFrame> frames) : frames_(std::move(frames)) {}

  bool next(CameraFrame &frame) override {
    if (next_ == frames_.size()) {
      return false;
    }
    frame = frames_[next_++];
    return true;
  }

 private:
  std::vector<CameraFrame> frames_;
  std::size_t next_{0};
};

class FileEncoder final : public CameraSegmentEncoder {
 public:
  std::string codec() const override { return "fake-h264"; }
  std::string container_extension() const override { return ".mkv"; }

  void begin(const std::filesystem::path &temporary_path,
             const CameraStreamDescription &) override {
    require(temporary_path.extension() == ".tmp", "encoder did not receive a temporary path");
    output_.open(temporary_path, std::ios::binary);
    if (!output_) {
      throw std::runtime_error("fake encoder failed to open output");
    }
  }

  void write(const CameraFrame &frame) override {
    output_.write(reinterpret_cast<const char *>(frame.payload.data()),
                  static_cast<std::streamsize>(frame.payload.size()));
  }

  void finish() override {
    output_.close();
    if (!output_) {
      throw std::runtime_error("fake encoder failed to finish");
    }
  }

  void abort() noexcept override { output_.close(); }

 private:
  std::ofstream output_;
};

CameraFrame frame(std::uint64_t sequence, std::uint64_t timestamp_ns, std::uint8_t value) {
  CameraFrame result;
  result.sequence = sequence;
  result.timestamp_ns = timestamp_ns;
  result.width = 2;
  result.height = 1;
  result.stride = 6;
  result.encoding = "rgb8";
  result.frame_id = "camera_color_optical_frame";
  result.fx = 401.5;
  result.fy = 402.5;
  result.cx = 320.5;
  result.cy = 240.5;
  result.depth_scale = 0.001;
  result.dist_k1 = 0.1;
  result.dist_k2 = -0.2;
  result.dist_p1 = 0.003;
  result.dist_p2 = -0.004;
  result.dist_k3 = 0.05;
  result.payload.assign(6, static_cast<std::byte>(value));
  return result;
}

std::filesystem::path unique_directory(const std::string &name) {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("lingtu-camera-" + name + "-" + std::to_string(nonce));
}

void test_segment_rotation_mcap_index_verify_and_extract() {
  const auto session = unique_directory("roundtrip");
  VectorFrameSource source({frame(1, 100, 1), frame(2, 150, 2), frame(4, 300, 3)});
  FileEncoder encoder;
  CameraRecordingOptions options;
  options.session_directory = session;
  options.camera_id = "color";
  options.segment_duration_ns = 100;
  options.nominal_fps = 30.0;

  CameraSegmentRecorder recorder(options, source, encoder);
  const auto written = recorder.record();
  require(written.size() == 2, "recorder did not rotate the segment");
  require(written[0].frame_count == 2, "first segment frame count is wrong");
  require(written[0].first_sequence == 1 && written[0].last_sequence == 2,
          "first segment sequence range is wrong");
  require(written[0].dropped_frames == 0, "first segment drop count is wrong");
  require(written[1].frame_count == 1, "second segment frame count is wrong");
  require(written[1].first_sequence == 4 && written[1].last_sequence == 4,
          "second segment sequence range is wrong");
  require(written[1].dropped_frames == 1, "sequence gap was not recorded");
  require(written[0].frame_id == "camera_color_optical_frame" && written[0].fx == 401.5 &&
              written[0].dist_k3 == 0.05,
          "CameraInfo snapshot was not retained");
  require(std::filesystem::exists(session / written[0].relative_path),
          "first final segment is missing");
  require(!std::filesystem::exists((session / written[0].relative_path).string() + ".tmp"),
          "first temporary segment survived commit");

  const auto index_path = lingtu::recording::camera_index_path(session, "color");
  require(std::filesystem::exists(index_path), "camera MCAP index is missing");
  require(!std::filesystem::exists(index_path.string() + ".tmp"),
          "camera MCAP temporary index survived commit");

  mcap::McapReader raw_reader;
  require(raw_reader.open(index_path.string()).ok(), "failed to open camera MCAP index");
  require(raw_reader.header().has_value(), "camera MCAP index has no header");
  require(raw_reader.header()->profile == lingtu::recording::kCameraMcapProfile,
          "camera MCAP profile is wrong");
  std::size_t raw_count = 0;
  for (const auto &view : raw_reader.readMessages()) {
    ++raw_count;
    require(view.channel->topic == "/camera/color/segments", "camera index topic is wrong");
    require(view.channel->messageEncoding == "json", "camera index is not JSON");
    require(view.schema->encoding == "jsonschema", "camera index schema is not JSON Schema");
  }
  require(raw_count == 2, "camera MCAP index message count is wrong");
  raw_reader.close();

  const auto loaded = lingtu::recording::read_camera_segment_index(index_path);
  require(loaded == written, "camera index did not round-trip segment metadata");
  require(lingtu::recording::verify_camera_segments(index_path).empty(),
          "valid camera session did not verify");

  const auto extract_dir = unique_directory("extract");
  const auto extracted =
      lingtu::recording::extract_camera_window(index_path, extract_dir, 250, 350);
  require(extracted.size() == 1, "window extraction selected the wrong number of segments");
  require(extracted[0].filename() == std::filesystem::path(written[1].relative_path).filename(),
          "window extraction selected the wrong segment");
  require(std::filesystem::exists(extracted[0]), "window extraction did not copy the segment");

  {
    std::ofstream corrupt(session / written[1].relative_path, std::ios::binary | std::ios::app);
    corrupt.put('x');
  }
  require(!lingtu::recording::verify_camera_segments(index_path).empty(),
          "corrupted camera segment passed verification");

  std::filesystem::remove_all(session);
  std::filesystem::remove_all(extract_dir);
}

void test_segment_path_cannot_escape_session() {
  const auto session = unique_directory("safe-path");
  const auto outside = unique_directory("outside");
  std::filesystem::create_directories(session);
  std::filesystem::create_directories(outside);
  const auto index = session / "camera_color.mcap";
  bool rejected = false;
  try {
    (void)lingtu::recording::resolve_camera_segment_path(index, "../outside.mkv");
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "parent traversal escaped the camera session");
#if !defined(_WIN32)
  std::filesystem::create_directory_symlink(outside, session / "linked-outside");
  rejected = false;
  try {
    (void)lingtu::recording::resolve_camera_segment_path(index, "linked-outside/video.mkv");
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "symlink escaped the camera session");
#endif
  std::filesystem::remove_all(session);
  std::filesystem::remove_all(outside);
}

void test_timestamp_regression_is_rejected_without_final_index() {
  const auto session = unique_directory("regression");
  VectorFrameSource source({frame(1, 200, 1), frame(2, 100, 2)});
  FileEncoder encoder;
  CameraRecordingOptions options;
  options.session_directory = session;
  options.segment_duration_ns = 1000;

  bool rejected = false;
  try {
    CameraSegmentRecorder recorder(options, source, encoder);
    recorder.record();
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "timestamp regression was accepted");
  require(!std::filesystem::exists(lingtu::recording::camera_index_path(session, "color")),
          "failed recording promoted a final index");
  std::filesystem::remove_all(session);
}

#if !defined(_WIN32)
void test_recorder_rejects_segment_directory_symlink_escape() {
  const auto session = unique_directory("recorder-symlink");
  const auto outside = unique_directory("recorder-outside");
  std::filesystem::create_directories(session);
  std::filesystem::create_directories(outside);
  std::filesystem::create_directory_symlink(outside, session / "camera_color");

  VectorFrameSource source({frame(1, 100, 1)});
  FileEncoder encoder;
  CameraRecordingOptions options;
  options.session_directory = session;

  bool rejected = false;
  try {
    CameraSegmentRecorder recorder(options, source, encoder);
    static_cast<void>(recorder.record());
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "recorder followed a segment directory symlink outside the session");
  require(std::filesystem::is_empty(outside),
          "recorder wrote bytes outside the session before rejecting the symlink");

  std::filesystem::remove_all(session);
  std::filesystem::remove_all(outside);
}
#endif

}  // namespace

int main() {
  test_segment_rotation_mcap_index_verify_and_extract();
  test_timestamp_regression_is_rejected_without_final_index();
  test_segment_path_cannot_escape_session();
#if !defined(_WIN32)
  test_recorder_rejects_segment_directory_symlink_escape();
#endif
  return 0;
}
