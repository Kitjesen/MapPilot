#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "lingtu/recording/camera_recording.hpp"

namespace {

enum class Action { kList, kVerify, kExtract };

struct Options {
  std::filesystem::path index_path;
  std::filesystem::path output_directory;
  Action action{Action::kList};
  bool action_selected{false};
  std::uint64_t start_time_ns{0};
  std::uint64_t end_time_ns{0};
};

void print_help() {
  std::cout << "usage: lingtu_camera_player INDEX [action]\n\n"
            << "Offline camera segment inspection/export only; never publishes DDS or SHM.\n\n"
            << "  --list                         List indexed segments (default)\n"
            << "  --verify                       Verify every segment size and CRC32\n"
            << "  --extract-window START END     Copy overlapping segments by nanosecond time\n"
            << "  --output DIRECTORY             Required with --extract-window\n";
}

std::uint64_t parse_time(const std::string &value, const char *option) {
  std::size_t consumed = 0;
  const auto result = std::stoull(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument(std::string("invalid timestamp for ") + option + ": " + value);
  }
  return result;
}

Options parse_options(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto value = [&]() -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument("missing value for " + argument);
      }
      return argv[index];
    };
    const auto select = [&](Action action) {
      if (options.action_selected) {
        throw std::invalid_argument("select exactly one camera player action");
      }
      options.action = action;
      options.action_selected = true;
    };
    if (argument == "--list") {
      select(Action::kList);
    } else if (argument == "--verify") {
      select(Action::kVerify);
    } else if (argument == "--extract-window") {
      select(Action::kExtract);
      options.start_time_ns = parse_time(value(), "--extract-window");
      options.end_time_ns = parse_time(value(), "--extract-window");
    } else if (argument == "--output") {
      options.output_directory = value();
    } else if (argument == "-h" || argument == "--help") {
      print_help();
      std::exit(0);
    } else if (!argument.empty() && argument.front() == '-') {
      throw std::invalid_argument("unknown option: " + argument);
    } else if (options.index_path.empty()) {
      options.index_path = argument;
    } else {
      throw std::invalid_argument("unexpected positional argument: " + argument);
    }
  }
  if (options.index_path.empty()) {
    throw std::invalid_argument("camera MCAP index path is required");
  }
  if (options.action == Action::kExtract && options.output_directory.empty()) {
    throw std::invalid_argument("--output is required with --extract-window");
  }
  if (options.action != Action::kExtract && !options.output_directory.empty()) {
    throw std::invalid_argument("--output is valid only with --extract-window");
  }
  return options;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    const auto options = parse_options(argc, argv);
    if (options.action == Action::kList) {
      const auto segments = lingtu::recording::read_camera_segment_index(options.index_path);
      for (const auto &segment : segments) {
        std::cout << "segment=" << segment.index << " camera=" << segment.camera_id
                  << " file=" << segment.relative_path << " codec=" << segment.codec
                  << " size=" << segment.width << "x" << segment.height
                  << " time_ns=" << segment.start_time_ns << ".." << segment.end_time_ns
                  << " sequence=" << segment.first_sequence << ".." << segment.last_sequence
                  << " frames=" << segment.frame_count << " dropped=" << segment.dropped_frames
                  << " bytes=" << segment.byte_size << " crc32=" << segment.crc32 << "\n";
      }
      return 0;
    }
    if (options.action == Action::kVerify) {
      const auto issues = lingtu::recording::verify_camera_segments(options.index_path);
      if (!issues.empty()) {
        for (const auto &issue : issues) {
          std::cerr << issue.path.string() << ": " << issue.message << "\n";
        }
        return 3;
      }
      const auto segments = lingtu::recording::read_camera_segment_index(options.index_path);
      std::cout << "verified=" << segments.size() << "\n";
      return 0;
    }
    const auto extracted = lingtu::recording::extract_camera_window(
        options.index_path, options.output_directory, options.start_time_ns, options.end_time_ns);
    for (const auto &path : extracted) {
      std::cout << path.string() << "\n";
    }
    std::cout << "extracted=" << extracted.size() << "\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "lingtu_camera_player: " << error.what() << "\n";
    return 2;
  }
}
