#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <signal.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include "../shm_reader.hpp"
#include "gst.hpp"

namespace {
namespace shm = lingtu::drivers::camera::shm;
namespace video = lingtu::drivers::camera::video;
using Clock = std::chrono::steady_clock;

volatile sig_atomic_t stopping = 0;
void requestStop(int) {
  stopping = 1;
}

struct Options {
  std::string source{"/lingtu_camera_color"};
  std::string metrics_path;
  std::string status_path;
  std::string h264_output;
  video::GstOptions gst;
  unsigned max_age_ms{250};
  unsigned idle_timeout_ms{5000};
  unsigned metrics_interval_ms{1000};
};

unsigned number(const std::string &text, const std::string &key) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument(key + " requires an integer");
  }
  const auto value = std::stoull(text);
  if (value == 0 || value > std::numeric_limits<unsigned>::max()) {
    throw std::invalid_argument(key + " is out of range");
  }
  return static_cast<unsigned>(value);
}

unsigned nonNegativeNumber(const std::string &text, const std::string &key) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument(key + " requires an integer");
  }
  const auto value = std::stoull(text);
  if (value > std::numeric_limits<unsigned>::max())
    throw std::invalid_argument(key + " is out of range");
  return static_cast<unsigned>(value);
}

Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (i + 1 == argc)
      throw std::invalid_argument("missing value for " + key);
    const std::string value = argv[++i];
    if (key == "--color-shm")
      options.source = value;
    else if (key == "--output") {
      options.h264_output = value;
      options.gst.output = video::GstOutput::H264Fd;
    } else if (key == "--listen")
      options.gst.listen_uri = value;
    else if (key == "--stun-server")
      options.gst.stun_server = value;
    else if (key == "--encoder")
      options.gst.encoder_factory = value;
    else if (key == "--fps")
      options.gst.fps = number(value, key);
    else if (key == "--min-bitrate")
      options.gst.min_bitrate = number(value, key);
    else if (key == "--start-bitrate")
      options.gst.start_bitrate = number(value, key);
    else if (key == "--max-bitrate")
      options.gst.max_bitrate = number(value, key);
    else if (key == "--max-age-ms")
      options.max_age_ms = number(value, key);
    else if (key == "--idle-timeout-ms")
      options.idle_timeout_ms = nonNegativeNumber(value, key);
    else if (key == "--metrics-interval-ms")
      options.metrics_interval_ms = number(value, key);
    else if (key == "--metrics")
      options.metrics_path = value;
    else if (key == "--status-file")
      options.status_path = value;
    else
      throw std::invalid_argument("unknown option: " + key);
  }
  if (options.gst.fps > 120)
    throw std::invalid_argument("--fps must be at most 120");
  if (options.gst.min_bitrate > options.gst.start_bitrate ||
      options.gst.start_bitrate > options.gst.max_bitrate) {
    throw std::invalid_argument("bitrate must satisfy min <= start <= max");
  }
  if (options.gst.output == video::GstOutput::H264Fd && options.idle_timeout_ms == 0)
    throw std::invalid_argument("--idle-timeout-ms must be positive for H.264 output");
  return options;
}

void usage() {
  std::cout << "lingtu_camera_webrtc: existing color SHM -> exact H.264 encoder -> WHEP or H.264\n"
               "  --color-shm NAME             default /lingtu_camera_color\n"
               "  --listen URI                 default http://127.0.0.1:9090\n"
               "  --stun-server URI            stun://host:port; default empty (LAN direct)\n"
               "  --output FILE|-              fixed-rate Annex-B H.264 instead of WHEP\n"
               "  --encoder FACTORY            default nvv4l2h264enc; exact, no fallback\n"
               "  --fps N                      maximum raw input rate, default 20\n"
               "  --min-bitrate BPS            default 200000\n"
               "  --start-bitrate BPS          default 1500000\n"
               "  --max-bitrate BPS            default 4000000\n"
               "  --max-age-ms N               reject stale camera frames, default 250\n"
               "  --idle-timeout-ms N          H.264 input timeout; WHEP accepts 0, default 5000\n"
               "  --metrics FILE               append runtime JSONL (default stderr)\n"
               "  --status-file FILE           atomic service status JSON\n"
               "  --metrics-interval-ms N      default 1000\n";
}

class H264Output {
 public:
  explicit H264Output(const std::string &path) {
    if (path.empty())
      return;
    if (path == "-") {
      if (::isatty(STDOUT_FILENO))
        throw std::runtime_error("redirect H.264 stdout or use --output FILE");
      fd_ = STDOUT_FILENO;
    } else {
      fd_ = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
      if (fd_ < 0)
        throw std::runtime_error("cannot create H.264 output: " + path + ": " +
                                 std::strerror(errno));
    }
  }
  ~H264Output() {
    if (fd_ >= 0 && fd_ != STDOUT_FILENO)
      ::close(fd_);
  }
  H264Output(const H264Output &) = delete;
  H264Output &operator=(const H264Output &) = delete;
  int fd() const { return fd_; }

 private:
  int fd_{-1};
};

std::string escape(const std::string &value) {
  std::string out;
  out.reserve(value.size());
  for (const char ch : value) {
    if (ch == '\\' || ch == '"')
      out.push_back('\\');
    if (ch == '\n')
      out += "\\n";
    else if (ch != '\r')
      out.push_back(ch);
  }
  return out;
}

double unixTimeSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

class StatusFile {
 public:
  explicit StatusFile(const Options &options)
      : path_(options.status_path),
        endpoint_(video::GstSession::endpoint(options.gst)),
        encoder_(options.gst.encoder_factory),
        whep_(options.gst.output == video::GstOutput::Whep) {
    publish("starting", false, false, std::nullopt, "");
  }

  void waiting(std::optional<double> frame_age_ms = std::nullopt) {
    publish("waiting_for_source", false, false, frame_age_ms, "");
  }

  void running(const shm::ColorFrame &frame, double frame_age_ms) {
    observe(frame);
    publish("running", whep_, true, frame_age_ms, "");
  }

  void observe(const shm::ColorFrame &frame) {
    generation_ = frame.source_generation;
    sequence_ = frame.sequence;
  }

  std::uint64_t generation() const { return generation_; }

  void heartbeat(const std::string &status, bool ready, bool source_fresh,
                 std::optional<double> frame_age_ms) {
    publish(status, ready, source_fresh, frame_age_ms, "");
  }

  void failed(const std::string &error) {
    publish("error", false, false, last_frame_age_ms_, error);
  }

  void stopped() {
    publish("stopped", false, false, last_frame_age_ms_, "");
  }

 private:
  void publish(const std::string &status, bool ready, bool source_fresh,
               std::optional<double> frame_age_ms, const std::string &error) {
    last_frame_age_ms_ = frame_age_ms;
    if (path_.empty())
      return;
    const std::filesystem::path path(path_);
    if (!path.parent_path().empty()) {
      std::error_code mkdir_error;
      std::filesystem::create_directories(path.parent_path(), mkdir_error);
      if (mkdir_error)
        throw std::runtime_error("cannot create video status directory: " +
                                 mkdir_error.message());
    }
    const std::filesystem::path temporary =
        path.string() + ".tmp." + std::to_string(static_cast<long long>(::getpid()));
    {
      std::ofstream out(temporary, std::ios::trunc);
      if (!out)
        throw std::runtime_error("cannot open video status file: " + temporary.string());
      out << std::fixed << std::setprecision(6)
          << "{\n"
          << "  \"schema_version\": \"lingtu.camera_video.status.v1\",\n"
          << "  \"status\": \"" << status << "\",\n"
          << "  \"ready\": " << (ready ? "true" : "false") << ",\n"
          << "  \"source_fresh\": " << (source_fresh ? "true" : "false") << ",\n"
          << "  \"heartbeat_timestamp_s\": " << unixTimeSeconds() << ",\n"
          << "  \"source_generation\": " << generation_ << ",\n"
          << "  \"source_sequence\": " << sequence_ << ",\n"
          << "  \"frame_age_ms\": ";
      if (frame_age_ms)
        out << *frame_age_ms;
      else
        out << "null";
      out << ",\n"
          << "  \"endpoint\": \"" << escape(endpoint_) << "\",\n"
          << "  \"encoder\": \"" << escape(encoder_) << "\",\n"
          << "  \"last_error\": \"" << escape(error) << "\"\n"
          << "}\n";
      out.flush();
      if (!out)
        throw std::runtime_error("writing video status failed: " + temporary.string());
    }
    std::error_code rename_error;
    std::filesystem::rename(temporary, path, rename_error);
    if (rename_error) {
      std::error_code cleanup_error;
      std::filesystem::remove(temporary, cleanup_error);
      throw std::runtime_error("cannot publish video status: " + rename_error.message());
    }
  }

  std::string path_;
  std::string endpoint_;
  std::string encoder_;
  bool whep_{false};
  std::uint64_t generation_{0};
  std::uint64_t sequence_{0};
  std::optional<double> last_frame_age_ms_;
};

bool streamChanged(const shm::ColorFrame &frame, std::uint64_t generation, std::uint32_t width,
                   std::uint32_t height, std::uint32_t stride, const std::string &encoding) {
  return frame.source_generation != generation || frame.width != width || frame.height != height ||
         frame.stride != stride || frame.encoding != encoding;
}

Clock::time_point frameFreshTime(Clock::time_point accepted_at, std::uint64_t accepted_age_ns) {
  return accepted_at - std::chrono::nanoseconds(accepted_age_ns);
}

std::uint64_t frameAgeNs(Clock::time_point now, Clock::time_point last_fresh) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_fresh).count();
}

int run(const Options &options, StatusFile &status) {
  shm::ColorReader reader(options.source);
  H264Output h264_output(options.h264_output);
  video::GstOptions gst_options = options.gst;
  if (gst_options.output == video::GstOutput::H264Fd)
    gst_options.output_fd = h264_output.fd();
  std::ofstream metrics_file;
  if (!options.metrics_path.empty()) {
    metrics_file.open(options.metrics_path, std::ios::app);
    if (!metrics_file)
      throw std::runtime_error("cannot open metrics output");
  }
  std::ostream &output = options.metrics_path.empty() ? std::cerr : metrics_file;
  auto emit = [&](const std::string &line) {
    output << line << '\n';
    output.flush();
    if (!output)
      throw std::runtime_error("writing video metrics failed");
  };

  std::unique_ptr<video::GstSession> session;
  auto last_fresh = Clock::now();
  auto next_push = Clock::now();
  auto next_metrics = Clock::now();
  auto next_status = Clock::now();
  const auto interval = std::chrono::nanoseconds(1000000000ULL / gst_options.fps);
  const auto max_age_ns = static_cast<std::uint64_t>(options.max_age_ms) * 1000000ULL;
  std::uint64_t generation = 0;
  std::uint32_t width = 0, height = 0, stride = 0;
  std::string encoding;
  shm::ColorFrame frame;
  std::uint64_t active_capture_timestamp_ns = 0;
  std::uint64_t last_capture_timestamp_ns = 0;
  std::string service_status = "waiting_for_source";
  status.waiting();

  while (!stopping) {
    const auto now = Clock::now();
    if (session)
      session->poll();
    const auto wall_now_ns = shm::unixTimeNs();
    const auto frame_age_ns =
        gst_options.output == video::GstOutput::Whep
            ? frameAgeNs(now, last_fresh)
            : (last_capture_timestamp_ns != 0 && wall_now_ns > last_capture_timestamp_ns
                   ? wall_now_ns - last_capture_timestamp_ns
                   : 0);
    const bool source_fresh =
        active_capture_timestamp_ns != 0 && frame_age_ns <= max_age_ns;
    if (gst_options.output == video::GstOutput::Whep && session &&
        active_capture_timestamp_ns != 0 && !source_fresh) {
      session->stop();
      session.reset();
      active_capture_timestamp_ns = 0;
      service_status = "waiting_for_source";
      status.waiting(last_capture_timestamp_ns == 0
                         ? std::nullopt
                         : std::optional<double>(static_cast<double>(frame_age_ns) / 1000000.0));
      next_status = now + std::chrono::seconds(1);
    }
    if (gst_options.output == video::GstOutput::H264Fd &&
        now - last_fresh >= std::chrono::milliseconds(options.idle_timeout_ms)) {
      throw std::runtime_error("camera WebRTC timed out waiting for fresh color frames");
    }
    if (now >= next_status) {
      const auto age = last_capture_timestamp_ns == 0
                           ? std::nullopt
                           : std::optional<double>(static_cast<double>(frame_age_ns) / 1000000.0);
      status.heartbeat(service_status,
                       gst_options.output == video::GstOutput::Whep &&
                           service_status == "running" && source_fresh,
                       source_fresh, age);
      next_status = now + std::chrono::seconds(1);
    }
    if (now >= next_metrics && session) {
      const auto metrics = session->metrics();
      std::ostringstream line;
      line << "{\"event\":\"metrics\",\"source_generation\":" << generation
           << ",\"accepted_frames\":" << metrics.accepted_frames
           << ",\"rejected_timestamps\":" << metrics.rejected_timestamps
           << ",\"last_pts_ns\":" << metrics.last_pts_ns
           << ",\"last_running_time_ns\":" << metrics.last_running_time_ns
           << ",\"active_consumers\":" << metrics.active_consumers
           << ",\"queued_raw_frames\":" << metrics.queued_raw_frames << ",\"selected_encoder\":\""
           << escape(metrics.selected_encoder) << "\",\"rtc_stats\":\"" << escape(metrics.rtc_stats)
           << "\",\"transport\":\"" << metrics.transport << "\"}";
      emit(line.str());
      next_metrics = now + std::chrono::milliseconds(options.metrics_interval_ms);
    }
    if (now < next_push) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    const auto read_start = Clock::now();
    if (!reader.readLatest(frame, wall_now_ns, max_age_ns)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    const bool clock_regressed = gst_options.output == video::GstOutput::Whep &&
                                 frame.timestamp_ns < active_capture_timestamp_ns;
    if (!session || streamChanged(frame, generation, width, height, stride, encoding) ||
        clock_regressed) {
      if (session) {
        session->stop();
        session.reset();
      }
      session = std::make_unique<video::GstSession>(gst_options, frame);
      generation = frame.source_generation;
      width = frame.width;
      height = frame.height;
      stride = frame.stride;
      encoding = frame.encoding;
      std::ostringstream line;
      line << "{\"event\":\"stream\",\"source_generation\":" << generation << ",\"width\":" << width
           << ",\"height\":" << height << ",\"encoding\":\"" << encoding << "\",\"endpoint\":\""
           << escape(video::GstSession::endpoint(gst_options)) << "\",\"encoder\":\""
           << escape(gst_options.encoder_factory) << "\"}";
      emit(line.str());
    }

    const auto before_push_ns = shm::unixTimeNs();
    const auto push_age_ns =
        before_push_ns > frame.timestamp_ns ? before_push_ns - frame.timestamp_ns : 0;
    if (gst_options.output == video::GstOutput::Whep && push_age_ns > max_age_ns) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      continue;
    }
    if (session->push(frame)) {
      const bool publish_ready =
          service_status != "running" || status.generation() != frame.source_generation;
      last_fresh = Clock::now();
      active_capture_timestamp_ns = frame.timestamp_ns;
      last_capture_timestamp_ns = frame.timestamp_ns;
      service_status = "running";
      const auto accepted_wall_ns = shm::unixTimeNs();
      const auto accepted_age_ns =
          accepted_wall_ns > frame.timestamp_ns ? accepted_wall_ns - frame.timestamp_ns : 0;
      if (gst_options.output == video::GstOutput::Whep) {
        // Preserve the accepted frame's age, then expire it using only steady time.
        last_fresh = frameFreshTime(last_fresh, accepted_age_ns);
      }
      if (publish_ready) {
        status.running(frame, static_cast<double>(accepted_age_ns) / 1000000.0);
        next_status = Clock::now() + std::chrono::seconds(1);
      } else
        status.observe(frame);
    }
    // Keep the configured cadence between SHM reads. If processing
    // overruns, the next read takes only the latest SHM frame; it never replays
    // missed frames to catch up.
    next_push = read_start + interval;
  }
  if (session)
    session->stop();
  status.stopped();
  emit("{\"event\":\"stopped\"}");
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    usage();
    return 0;
  }
  struct sigaction action{};
  action.sa_handler = requestStop;
  sigemptyset(&action.sa_mask);
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
  try {
    const Options options = parse(argc, argv);
    StatusFile status(options);
    try {
      return run(options, status);
    } catch (const std::exception &error) {
      try {
        status.failed(error.what());
      } catch (const std::exception &status_error) {
        std::cerr << "camera video status failed: " << status_error.what() << '\n';
      }
      throw;
    }
  } catch (const std::exception &error) {
    std::cerr << "camera video stopped: " << error.what() << '\n';
    return 1;
  }
}
