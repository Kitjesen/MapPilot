#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <limits>
#include <signal.h>
#include <stdexcept>
#include <string>
#include <string_view>

#include "lingtu/recording/camera_linux.hpp"

namespace {

volatile sig_atomic_t stop_requested = 0;

extern "C" void request_stop(int) {
  stop_requested = 1;
}

struct Options {
  std::filesystem::path output_directory;
  std::string color_shm{"/lingtu_camera_color"};
  double segment_seconds{5.0};
  double fps{30.0};
  std::string ffmpeg{"ffmpeg"};
  std::string codec{"libx264"};
  std::uint64_t max_frames{0};
};

void print_usage(std::ostream &output, const char *program) {
  output << "Usage: " << program << " --output-dir DIR [options]\n"
         << "\n"
         << "Record the native color-camera POSIX SHM ring as durable MKV segments\n"
         << "with an MCAP timeline index. No middleware graph is involved.\n"
         << "\n"
         << "Options:\n"
         << "  --output-dir DIR       New or existing session directory (required)\n"
         << "  --color-shm NAME       POSIX SHM name (default: /lingtu_camera_color)\n"
         << "  --segment-seconds SEC  Segment duration (default: 5)\n"
         << "  --fps FPS              Raw input frame rate (default: 30)\n"
         << "  --ffmpeg PATH          FFmpeg executable (default: ffmpeg)\n"
         << "  --codec NAME           FFmpeg video codec (default: libx264, generic software)\n"
         << "  --max-frames N         Stop after N frames; 0 means unlimited\n"
         << "  -h, --help             Show this help\n"
         << "\n"
         << "The default codec is the supported generic software path. Select another\n"
         << "FFmpeg codec only after validating it on the target image.\n";
}

std::string require_value(int &index, int argc, char **argv, std::string_view option) {
  if (++index >= argc) {
    throw std::invalid_argument(std::string(option) + " requires a value");
  }
  return argv[index];
}

double parse_positive_double(const std::string &input, std::string_view option) {
  char *end = nullptr;
  errno = 0;
  const double value = std::strtod(input.c_str(), &end);
  if (errno != 0 || end != input.c_str() + input.size() || !std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(option) + " requires a positive number");
  }
  return value;
}

std::uint64_t parse_uint64(const std::string &input, std::string_view option) {
  std::uint64_t value = 0;
  const auto result = std::from_chars(input.data(), input.data() + input.size(), value);
  if (result.ec != std::errc{} || result.ptr != input.data() + input.size()) {
    throw std::invalid_argument(std::string(option) + " requires an unsigned integer");
  }
  return value;
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "-h" || argument == "--help") {
      print_usage(std::cout, argv[0]);
      std::exit(0);
    }
    if (argument == "--output-dir") {
      options.output_directory = require_value(index, argc, argv, argument);
    } else if (argument == "--color-shm") {
      options.color_shm = require_value(index, argc, argv, argument);
    } else if (argument == "--segment-seconds") {
      options.segment_seconds =
          parse_positive_double(require_value(index, argc, argv, argument), argument);
    } else if (argument == "--fps") {
      options.fps = parse_positive_double(require_value(index, argc, argv, argument), argument);
    } else if (argument == "--ffmpeg") {
      options.ffmpeg = require_value(index, argc, argv, argument);
    } else if (argument == "--codec") {
      options.codec = require_value(index, argc, argv, argument);
    } else if (argument == "--max-frames") {
      options.max_frames = parse_uint64(require_value(index, argc, argv, argument), argument);
    } else {
      throw std::invalid_argument("unknown option: " + std::string(argument));
    }
  }
  if (options.output_directory.empty()) {
    throw std::invalid_argument("--output-dir is required");
  }
  constexpr long double kNanosecondsPerSecond = 1'000'000'000.0L;
  if (static_cast<long double>(options.segment_seconds) >
      static_cast<long double>(std::numeric_limits<std::uint64_t>::max()) / kNanosecondsPerSecond) {
    throw std::invalid_argument("--segment-seconds is too large");
  }
  return options;
}

void install_signal_handlers() {
  struct sigaction action{};
  action.sa_handler = request_stop;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (::sigaction(SIGINT, &action, nullptr) != 0 || ::sigaction(SIGTERM, &action, nullptr) != 0) {
    throw std::runtime_error("failed to install signal handlers");
  }
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const auto options = parse_options(argc, argv);
    install_signal_handlers();

    lingtu::recording::PosixShmCameraSourceOptions source_options;
    source_options.name = options.color_shm;
    source_options.max_frames = options.max_frames;
    source_options.stop_requested = [] { return stop_requested != 0; };
    lingtu::recording::FfmpegSegmentEncoder encoder({options.ffmpeg, options.codec});
    encoder.verify();
    lingtu::recording::PosixShmCameraFrameSource source(std::move(source_options));
    lingtu::recording::CameraRecordingOptions recording_options;
    recording_options.session_directory = options.output_directory;
    recording_options.segment_duration_ns = static_cast<std::uint64_t>(
        static_cast<long double>(options.segment_seconds) * 1'000'000'000.0L);
    recording_options.nominal_fps = options.fps;
    lingtu::recording::CameraSegmentRecorder recorder(std::move(recording_options), source,
                                                      encoder);
    const auto segments = recorder.record();

    std::uint64_t frames = 0;
    std::uint64_t dropped_frames = 0;
    for (const auto &segment : segments) {
      frames += segment.frame_count;
      dropped_frames += segment.dropped_frames;
    }
    std::cout << "Recorded " << frames << " color frames in " << segments.size()
              << " MKV segment(s); index: "
              << lingtu::recording::camera_index_path(options.output_directory, "color") << '\n';
    if (dropped_frames != 0) {
      std::cerr << "lingtu_camera_recorder: recorded " << dropped_frames
                << " dropped source frame(s); refusing a clean session result\n";
      return 4;
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "lingtu_camera_recorder: " << error.what() << '\n';
    return 1;
  }
}
