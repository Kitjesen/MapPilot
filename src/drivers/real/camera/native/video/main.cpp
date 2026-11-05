#include <lingtu/settings.hpp>

#include "encoder.hpp"
#include "../shm_reader.hpp"

#include <fcntl.h>
#include <signal.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
namespace shm = lingtu::drivers::camera::shm;
namespace video = lingtu::drivers::camera::video;
using Clock = std::chrono::steady_clock;
volatile sig_atomic_t stopping = 0;
void request_stop(int) { stopping = 1; }

struct Options {
  std::string source{lingtu::settings::camera::camera::shm::color};
  std::string output{"-"};
  std::string metadata;
  video::EncoderOptions encoder;
  unsigned max_age_ms{lingtu::settings::camera::camera::video::max_age_ms};
  unsigned idle_timeout_ms{lingtu::settings::camera::camera::video::idle_timeout_ms};
  unsigned frames{0};
};

unsigned number(const std::string& text, const std::string& key, bool allow_zero = false) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument(key + " requires an integer");
  }
  const auto value = std::stoull(text);
  if ((!allow_zero && value == 0) || value > std::numeric_limits<unsigned>::max()) {
    throw std::invalid_argument(key + " is out of range");
  }
  return static_cast<unsigned>(value);
}

void usage() {
  std::cout <<
      "lingtu_camera_video: existing color SHM -> FFmpeg -> Annex-B H.264\n"
      "  --color-shm NAME         color shared memory name\n"
      "  --output FILE|-          new H.264 file or stdout (default -)\n"
      "  --metadata FILE          new JSONL file (otherwise mixed stderr diagnostics)\n"
      "  --ffmpeg EXECUTABLE      default ffmpeg\n"
      "  --codec NAME             default libx264 (software; no automatic fallback)\n"
      "  --fps N                  maximum input rate\n"
      "  --bitrate-kbps N          target bitrate\n"
      "  --gop N                  keyframe interval, default one second\n"
      "  --max-age-ms N           reject older camera frames\n"
      "  --write-timeout-ms N     per-frame encoder input deadline, default 500\n"
      "  --idle-timeout-ms N      fail without fresh camera frames\n"
      "  --frames N              stop after N submitted frames; 0 means unlimited\n"
      "Only consumes an already-running camera. Does not manage a Product or WebRTC.\n";
}

Options parse(int argc, char** argv) {
  Options options;
  bool explicit_gop = false;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 == argc) throw std::invalid_argument("missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--color-shm") options.source = value;
    else if (key == "--output") options.output = value;
    else if (key == "--metadata") options.metadata = value;
    else if (key == "--ffmpeg") options.encoder.executable = value;
    else if (key == "--codec") options.encoder.codec = value;
    else if (key == "--fps") options.encoder.fps = number(value, key);
    else if (key == "--bitrate-kbps") options.encoder.bitrate_kbps = number(value, key);
    else if (key == "--gop") { options.encoder.gop = number(value, key); explicit_gop = true; }
    else if (key == "--max-age-ms") options.max_age_ms = number(value, key);
    else if (key == "--write-timeout-ms") options.encoder.write_timeout = std::chrono::milliseconds(number(value, key));
    else if (key == "--idle-timeout-ms") options.idle_timeout_ms = number(value, key);
    else if (key == "--frames") options.frames = number(value, key, true);
    else throw std::invalid_argument("unknown option: " + key);
  }
  if (options.encoder.fps > 120) throw std::invalid_argument("--fps must be at most 120");
  if (!explicit_gop) options.encoder.gop = options.encoder.fps;
  options.encoder.stop_requested = [] { return stopping != 0; };
  return options;
}

class Output {
 public:
  explicit Output(const std::string& path) {
    if (path == "-") {
      if (::isatty(STDOUT_FILENO)) throw std::runtime_error("redirect H.264 stdout or use --output FILE");
      fd = STDOUT_FILENO;
    } else {
      fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
      if (fd < 0) throw std::runtime_error("cannot create new H.264 output: " + path + ": " + std::strerror(errno));
    }
  }
  ~Output() { if (fd != STDOUT_FILENO) ::close(fd); }
  Output(const Output&) = delete;
  Output& operator=(const Output&) = delete;
  int fd{-1};
};

std::int64_t microseconds(Clock::duration elapsed) {
  return std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
}

int run(const Options& options) {
  shm::ColorReader reader(options.source);
  video::Encoder encoder(options.encoder);
  Output output(options.output);
  std::ofstream metadata_file;
  if (!options.metadata.empty()) {
    if (std::filesystem::exists(options.metadata)) throw std::runtime_error("metadata output already exists");
    metadata_file.open(options.metadata);
    if (!metadata_file) throw std::runtime_error("cannot create metadata output");
  }
  std::ostream& metadata = options.metadata.empty() ? std::cerr : metadata_file;
  auto emit = [&](const std::string& line) {
    metadata << line << '\n';
    metadata.flush();
    if (!metadata) throw std::runtime_error("writing video metadata failed");
  };
  std::cerr << "camera video: source=" << options.source << " codec=" << options.encoder.codec
            << " fps=" << options.encoder.fps << " bitrate_kbps=" << options.encoder.bitrate_kbps << '\n';

  const auto started = Clock::now();
  auto last_fresh = started;
  auto next_frame = started;
  const auto interval = std::chrono::nanoseconds(1000000000ULL / options.encoder.fps);
  const auto max_age_ns = static_cast<std::uint64_t>(options.max_age_ms) * 1000000ULL;
  std::uint64_t submitted = 0, skipped = 0, stream = 0, stream_index = 0;
  std::uint64_t generation = 0, sequence = 0;
  std::uint32_t width = 0, height = 0, stride = 0;
  std::string encoding;
  bool open = false;
  bool aborted = false;
  shm::ColorFrame frame;

  try {
    while (!stopping && (options.frames == 0 || submitted < options.frames)) {
      const auto now = Clock::now();
      if (now - last_fresh >= std::chrono::milliseconds(options.idle_timeout_ms)) {
        throw std::runtime_error("camera video timed out waiting for fresh color frames");
      }
      if (now < next_frame) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      const auto read_start = Clock::now();
      if (!reader.readLatest(frame, shm::unixTimeNs(), max_age_ns)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      const auto read_us = microseconds(Clock::now() - read_start);
      last_fresh = Clock::now();
      if (!open || frame.source_generation != generation || frame.width != width ||
          frame.height != height || frame.stride != stride || frame.encoding != encoding) {
        if (open) encoder.finish();
        encoder.begin(frame, output.fd);
        open = true;
        ++stream;
        stream_index = 0;
        sequence = 0;
        generation = frame.source_generation;
        width = frame.width;
        height = frame.height;
        stride = frame.stride;
        encoding = frame.encoding;
        std::ostringstream event;
        event << "{\"event\":\"stream\",\"stream\":" << stream
              << ",\"source_generation\":" << generation << ",\"width\":" << width
              << ",\"height\":" << height << ",\"fps\":" << options.encoder.fps << '}';
        emit(event.str());
      }
      // Startup or a previous encoder drain can age the copied frame.
      const auto before_submit = shm::unixTimeNs();
      if (before_submit < frame.timestamp_ns || before_submit - frame.timestamp_ns > max_age_ns) continue;
      const auto skipped_here = sequence != 0 && frame.sequence > sequence ? frame.sequence - sequence - 1 : 0;
      const auto submit_start = Clock::now();
      encoder.write(frame);
      const auto submit_us = microseconds(Clock::now() - submit_start);
      ++submitted;
      skipped += skipped_here;
      sequence = frame.sequence;
      std::ostringstream event;
      event << "{\"event\":\"submitted\",\"stream\":" << stream << ",\"index\":" << stream_index++
            << ",\"source_generation\":" << generation << ",\"sequence\":" << sequence
            << ",\"capture_timestamp_ns\":" << frame.timestamp_ns << ",\"read_us\":" << read_us
            << ",\"submit_us\":" << submit_us << ",\"age_us\":" << (before_submit - frame.timestamp_ns) / 1000
            << ",\"skipped\":" << skipped_here << '}';
      emit(event.str());
      // No catch-up bursts: the shared ring holds the next latest raw frame.
      next_frame = submit_start + interval;
    }
  } catch (const video::Cancelled&) {
    encoder.abort();
    open = false;
    aborted = true;
  }
  if (open) encoder.finish();
  std::ostringstream summary;
  summary << "{\"event\":\"" << (aborted ? "stopped" : "finished")
          << "\",\"encoder_aborted\":" << (aborted ? "true" : "false")
          << ",\"submitted\":" << submitted << ",\"streams\":" << stream
          << ",\"skipped\":" << skipped << ",\"elapsed_us\":" << microseconds(Clock::now() - started) << '}';
  emit(summary.str());
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") { usage(); return 0; }
  struct sigaction action{};
  action.sa_handler = request_stop;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  try {
    return run(parse(argc, argv));
  } catch (const std::exception& error) {
    std::cerr << "camera video stopped: " << error.what() << '\n';
    return 1;
  }
}
