#include <algorithm>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <locale>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "drivers/real/camera/native/shm_frame_ring.hpp"
#include "lingtu/recording/camera_linux.hpp"

namespace {

namespace camera_shm = lingtu::drivers::camera::shm;
namespace recording = lingtu::recording;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

int fake_ffmpeg(int argc, char **argv) {
  std::string codec;
  std::string framerate;
  bool has_ultrafast_preset = false;
  bool has_zerolatency_tune = false;
  for (int index = 1; index + 1 < argc; ++index) {
    if (std::string_view(argv[index]) == "-c:v") {
      codec = argv[index + 1];
    } else if (std::string_view(argv[index]) == "-framerate") {
      framerate = argv[index + 1];
    } else if (std::string_view(argv[index]) == "-preset" &&
               std::string_view(argv[index + 1]) == "ultrafast") {
      has_ultrafast_preset = true;
    } else if (std::string_view(argv[index]) == "-tune" &&
               std::string_view(argv[index + 1]) == "zerolatency") {
      has_zerolatency_tune = true;
    }
  }
  if (codec == "intentional-failure") {
    return 17;
  }
  if (codec == "expect-dot-decimal" && framerate != "29.5") {
    return 21;
  }
  if (codec == "libx264" && (!has_ultrafast_preset || !has_zerolatency_tune)) {
    return 23;
  }
  if (argc < 2) {
    return 18;
  }
  std::ofstream output(argv[argc - 1], std::ios::binary);
  if (!output) {
    return 19;
  }
  std::vector<char> buffer(4096);
  while (std::cin) {
    std::cin.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    output.write(buffer.data(), std::cin.gcount());
  }
  return output ? 0 : 20;
}

std::string unique_shm_name(const std::string &suffix) {
  return "/lingtu_camera_test_" + std::to_string(::getpid()) + "_" + suffix;
}

camera_shm::FrameMetadata color_metadata() {
  camera_shm::FrameMetadata metadata;
  metadata.stream_kind = camera_shm::StreamKind::kColor;
  metadata.timestamp_ns = 123'456'789;
  metadata.width = 2;
  metadata.height = 2;
  metadata.stride = 6;
  metadata.encoding = "rgb8";
  metadata.frame_id = "camera_color_optical_frame";
  metadata.fx = 100.0;
  metadata.fy = 101.0;
  metadata.cx = 1.0;
  metadata.cy = 1.5;
  return metadata;
}

void test_shm_source_reads_and_deduplicates() {
  const auto name = unique_shm_name("valid");
  camera_shm::WriterConfig config;
  config.name = name;
  config.slot_count = 2;
  config.slot_capacity = 1024;
  config.unlink_on_destroy = true;
  camera_shm::FrameWriter writer(config);
  const std::vector<std::uint8_t> payload{
      1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
  };
  writer.publish(color_metadata(), payload.data(), payload.size());

  recording::PosixShmCameraSourceOptions options;
  options.name = name;
  options.max_frames = 1;
  recording::PosixShmCameraFrameSource source(options);
  recording::CameraFrame frame;
  require(source.next(frame), "SHM source did not return its first frame");
  require(frame.sequence == 1, "SHM frame sequence differs");
  require(frame.timestamp_ns == 123'456'789, "SHM frame timestamp differs");
  require(frame.width == 2 && frame.height == 2 && frame.stride == 6, "SHM frame geometry differs");
  require(frame.encoding == "rgb8", "SHM frame encoding differs");
  require(frame.frame_id == "camera_color_optical_frame", "SHM frame id differs");
  require(frame.fx == 100.0 && frame.fy == 101.0, "SHM intrinsics differ");
  require(frame.payload.size() == payload.size(), "SHM payload size differs");
  require(std::equal(frame.payload.begin(), frame.payload.end(),
                     reinterpret_cast<const std::byte *>(payload.data())),
          "SHM payload differs");
  require(!source.next(frame), "SHM max-frames limit did not stop the source");
}

void test_shm_source_drains_available_backlog_in_sequence_order() {
  const auto name = unique_shm_name("backlog");
  camera_shm::WriterConfig config;
  config.name = name;
  config.slot_count = 8;
  config.slot_capacity = 1024;
  config.unlink_on_destroy = true;
  camera_shm::FrameWriter writer(config);
  const std::vector<std::uint8_t> payload(12, 0x2a);
  auto metadata = color_metadata();
  writer.publish(metadata, payload.data(), payload.size());

  recording::PosixShmCameraSourceOptions options;
  options.name = name;
  options.max_frames = 8;
  recording::PosixShmCameraFrameSource source(options);
  recording::CameraFrame frame;
  require(source.next(frame) && frame.sequence == 1, "SHM source did not read the initial frame");

  for (std::uint64_t sequence = 2; sequence <= 8; ++sequence) {
    metadata.timestamp_ns += 1;
    writer.publish(metadata, payload.data(), payload.size());
  }
  for (std::uint64_t expected = 2; expected <= 8; ++expected) {
    require(source.next(frame), "SHM source stopped before draining the available backlog");
    require(frame.sequence == expected, "SHM source skipped an available backlog frame");
  }
  require(!source.next(frame), "SHM source ignored its max-frames limit after draining backlog");
}

void test_shm_source_rejects_crc_corruption() {
  const auto name = unique_shm_name("crc");
  camera_shm::WriterConfig config;
  config.name = name;
  config.slot_count = 2;
  config.slot_capacity = 1024;
  config.unlink_on_destroy = true;
  camera_shm::FrameWriter writer(config);
  const std::vector<std::uint8_t> payload(12, 0x5a);
  writer.publish(color_metadata(), payload.data(), payload.size());

  const int fd = ::shm_open(name.c_str(), O_RDWR | O_CLOEXEC, 0);
  require(fd >= 0, "failed to reopen test SHM");
  struct stat info{};
  require(::fstat(fd, &info) == 0, "failed to stat test SHM");
  void *mapping = ::mmap(nullptr, static_cast<std::size_t>(info.st_size), PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, 0);
  require(mapping != MAP_FAILED, "failed to map test SHM");
  auto *bytes = static_cast<std::byte *>(mapping);
  bytes[sizeof(camera_shm::SharedHeader) + sizeof(camera_shm::SlotHeader)] ^= std::byte{0xff};
  require(::munmap(mapping, static_cast<std::size_t>(info.st_size)) == 0,
          "failed to unmap test SHM");
  require(::close(fd) == 0, "failed to close test SHM");

  recording::PosixShmCameraSourceOptions options;
  options.name = name;
  options.max_frames = 1;
  recording::PosixShmCameraFrameSource source(options);
  recording::CameraFrame frame;
  bool rejected = false;
  try {
    static_cast<void>(source.next(frame));
  } catch (const std::runtime_error &error) {
    rejected = std::string(error.what()).find("CRC32") != std::string::npos;
  }
  require(rejected, "SHM source accepted a payload with a bad CRC32");
}

void test_ffmpeg_encoder_uses_argv_pipe_and_strips_padding(const char *self) {
  const auto directory = std::filesystem::temp_directory_path() /
                         ("lingtu_camera_linux_" + std::to_string(::getpid()));
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  const auto output_path = directory / "segment.mkv.tmp";

  recording::FfmpegSegmentEncoder encoder({self, "fake-codec"});
  recording::CameraStreamDescription stream;
  stream.width = 2;
  stream.height = 2;
  stream.stride = 8;
  stream.encoding = "bgr8";
  stream.nominal_fps = 30.0;
  encoder.begin(output_path, stream);

  recording::CameraFrame frame;
  frame.sequence = 1;
  frame.timestamp_ns = 1;
  frame.width = 2;
  frame.height = 2;
  frame.stride = 8;
  frame.encoding = "bgr8";
  frame.payload = {
      std::byte{1},  std::byte{2},  std::byte{3},  std::byte{4},  std::byte{5}, std::byte{6},
      std::byte{99}, std::byte{99}, std::byte{7},  std::byte{8},  std::byte{9}, std::byte{10},
      std::byte{11}, std::byte{12}, std::byte{99}, std::byte{99},
  };
  encoder.write(frame);
  encoder.finish();

  std::ifstream input(output_path, std::ios::binary);
  const std::vector<char> actual((std::istreambuf_iterator<char>(input)),
                                 std::istreambuf_iterator<char>());
  const std::vector<char> expected{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  require(actual == expected, "FFmpeg pipe did not strip padded row bytes");
  std::filesystem::remove_all(directory);
}

void test_ffmpeg_exec_failure_is_reported() {
  recording::FfmpegSegmentEncoder encoder({"/definitely/missing/lingtu-ffmpeg", "libx264"});
  recording::CameraStreamDescription stream;
  stream.width = 2;
  stream.height = 2;
  stream.stride = 6;
  stream.encoding = "rgb8";
  stream.nominal_fps = 30.0;
  bool rejected = false;
  try {
    encoder.begin("/tmp/lingtu-never-created.mkv.tmp", stream);
  } catch (const std::runtime_error &error) {
    rejected = std::string(error.what()).find("executing FFmpeg") != std::string::npos;
  }
  require(rejected, "missing FFmpeg executable was not reported");
}

void test_ffmpeg_runtime_failure_is_reported(const char *self) {
  const auto directory = std::filesystem::temp_directory_path() /
                         ("lingtu_camera_linux_failure_" + std::to_string(::getpid()));
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  const auto output_path = directory / "segment.mkv.tmp";

  recording::FfmpegSegmentEncoder encoder({self, "intentional-failure"});
  recording::CameraStreamDescription stream;
  stream.width = 2;
  stream.height = 2;
  stream.stride = 6;
  stream.encoding = "rgb8";
  stream.nominal_fps = 30.0;
  encoder.begin(output_path, stream);
  bool rejected = false;
  try {
    encoder.finish();
  } catch (const std::runtime_error &error) {
    rejected = std::string(error.what()).find("exit code 17") != std::string::npos;
  }
  require(rejected, "FFmpeg runtime failure was not reported");
  std::filesystem::remove_all(directory);
}

void test_ffmpeg_encoder_rejects_odd_yuv420_geometry() {
  recording::FfmpegSegmentEncoder encoder({"/not-called-for-invalid-stream", "libx264"});
  recording::CameraStreamDescription stream;
  stream.width = 3;
  stream.height = 2;
  stream.stride = 9;
  stream.encoding = "rgb8";
  stream.nominal_fps = 30.0;
  bool rejected = false;
  try {
    encoder.begin("/tmp/lingtu-odd-geometry.mkv.tmp", stream);
  } catch (const std::invalid_argument &error) {
    rejected = std::string(error.what()).find("even frame dimensions") != std::string::npos;
  }
  require(rejected, "FFmpeg encoder accepted odd dimensions for yuv420p");
}

class CommaDecimalPoint final : public std::numpunct<char> {
 protected:
  char do_decimal_point() const override { return ','; }
};

void test_ffmpeg_framerate_is_locale_invariant(const char *self) {
  const auto directory = std::filesystem::temp_directory_path() /
                         ("lingtu_camera_linux_locale_" + std::to_string(::getpid()));
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  const auto output_path = directory / "segment.mkv.tmp";

  const std::locale previous = std::locale();
  std::locale::global(std::locale(previous, new CommaDecimalPoint));
  try {
    recording::FfmpegSegmentEncoder encoder({self, "expect-dot-decimal"});
    recording::CameraStreamDescription stream;
    stream.width = 2;
    stream.height = 2;
    stream.stride = 6;
    stream.encoding = "rgb8";
    stream.nominal_fps = 29.5;
    encoder.begin(output_path, stream);
    encoder.finish();
  } catch (...) {
    std::locale::global(previous);
    std::filesystem::remove_all(directory);
    throw;
  }
  std::locale::global(previous);
  std::filesystem::remove_all(directory);
}

void test_ffmpeg_profile_preflight_runs(const char *self) {
  recording::FfmpegSegmentEncoder supported({self, "fake-codec"});
  supported.verify();

  recording::FfmpegSegmentEncoder unsupported({self, "intentional-failure"});
  bool rejected = false;
  try {
    unsupported.verify();
  } catch (const std::runtime_error &error) {
    rejected = std::string(error.what()).find("preflight failed") != std::string::npos;
  }
  require(rejected, "FFmpeg preflight accepted an unusable encoder profile");
}

void test_libx264_uses_realtime_preset_for_field_recording(const char *self) {
  const auto directory = std::filesystem::temp_directory_path() /
                         ("lingtu_camera_linux_x264_" + std::to_string(::getpid()));
  std::filesystem::remove_all(directory);
  std::filesystem::create_directories(directory);
  const auto output_path = directory / "segment.mkv.tmp";

  recording::FfmpegSegmentEncoder encoder({self, "libx264"});
  recording::CameraStreamDescription stream;
  stream.width = 2;
  stream.height = 2;
  stream.stride = 6;
  stream.encoding = "rgb8";
  stream.nominal_fps = 30.0;
  encoder.begin(output_path, stream);
  recording::CameraFrame frame;
  frame.sequence = 1;
  frame.timestamp_ns = 1;
  frame.width = stream.width;
  frame.height = stream.height;
  frame.stride = stream.stride;
  frame.encoding = stream.encoding;
  frame.payload.assign(static_cast<std::size_t>(frame.stride) * frame.height, std::byte{1});
  encoder.write(frame);
  encoder.finish();
  std::filesystem::remove_all(directory);
}

[[noreturn]] void child_exit(bool success) {
  _exit(success ? 0 : 1);
}

void test_ffmpeg_segment_survives_recorder_process_group_stop(const char *self) {
  const pid_t child = ::fork();
  require(child >= 0, "failed to fork process-group stop test");
  if (child == 0) {
    if (::setpgid(0, 0) != 0) {
      child_exit(false);
    }

    const auto directory = std::filesystem::temp_directory_path() /
                           ("lingtu_camera_linux_stop_" + std::to_string(::getpid()));
    std::filesystem::remove_all(directory);
    std::filesystem::create_directories(directory);
    const auto output_path = directory / "segment.mkv.tmp";

    try {
      recording::FfmpegSegmentEncoder encoder({self, "fake-codec"});
      recording::CameraStreamDescription stream;
      stream.width = 2;
      stream.height = 2;
      stream.stride = 6;
      stream.encoding = "rgb8";
      stream.nominal_fps = 30.0;
      encoder.begin(output_path, stream);

      struct sigaction action{};
      action.sa_handler = SIG_IGN;
      sigemptyset(&action.sa_mask);
      if (::sigaction(SIGTERM, &action, nullptr) != 0) {
        child_exit(false);
      }

      recording::CameraFrame frame;
      frame.sequence = 1;
      frame.timestamp_ns = 1;
      frame.width = stream.width;
      frame.height = stream.height;
      frame.stride = stream.stride;
      frame.encoding = stream.encoding;
      frame.payload.assign(static_cast<std::size_t>(frame.stride) * frame.height, std::byte{7});
      encoder.write(frame);

      if (::kill(0, SIGTERM) != 0) {
        child_exit(false);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      encoder.finish();
      child_exit(std::filesystem::is_regular_file(output_path) &&
                 std::filesystem::file_size(output_path) == frame.payload.size());
    } catch (...) {
      child_exit(false);
    }
  }

  int status = 0;
  while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
  require(WIFEXITED(status) && WEXITSTATUS(status) == 0,
          "FFmpeg segment did not survive recorder process-group SIGTERM");
}

}  // namespace

int main(int argc, char **argv) {
  if (argc > 1 && std::string_view(argv[1]) == "-hide_banner") {
    return fake_ffmpeg(argc, argv);
  }
  test_shm_source_reads_and_deduplicates();
  test_shm_source_drains_available_backlog_in_sequence_order();
  test_shm_source_rejects_crc_corruption();
  test_ffmpeg_encoder_uses_argv_pipe_and_strips_padding(argv[0]);
  test_ffmpeg_exec_failure_is_reported();
  test_ffmpeg_runtime_failure_is_reported(argv[0]);
  test_ffmpeg_encoder_rejects_odd_yuv420_geometry();
  test_ffmpeg_framerate_is_locale_invariant(argv[0]);
  test_ffmpeg_profile_preflight_runs(argv[0]);
  test_libx264_uses_realtime_preset_for_field_recording(argv[0]);
  test_ffmpeg_segment_survives_recorder_process_group_stop(argv[0]);
  return 0;
}
