#include "camera_record.hpp"
#include "message/cpp/topics.hpp"
#include "message/cpp/qos.hpp"
#include "shm_frame_ring.hpp"

#include "dds/dds.h"
#include "messages.h"

#include <sys/types.h>
#include <sys/wait.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic_bool g_running{true};

namespace camera_record = lingtu::drivers::camera::record;
using camera_record::RecordHeader;
using camera_record::kKindColor;
using camera_record::kKindDepth;
using camera_record::kKindIntrinsics;

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (!camera_record::isValidTimestampSeconds(stamp_s)) {
    throw std::runtime_error("camera_record_timestamp_invalid");
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void logDdsError(dds_return_t value, const char* what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

enum class RecordSource {
  kChild,
  kStdin,
};

const char* recordSourceName(RecordSource source) noexcept {
  return source == RecordSource::kStdin ? "stdin" : "child";
}

struct CliConfig {
  std::string capture_bin{"build/orbbec_native/orbbec_capture"};
  std::string frame_id{"camera_link"};
  std::string color_topic{std::string(lingtu::message::kCameraColor.dds_topic)};
  std::string depth_topic{std::string(lingtu::message::kCameraDepth.dds_topic)};
  std::string info_topic{std::string(lingtu::message::kCameraInfo.dds_topic)};
  std::string status_file{"/dev/shm/lingtu/camera_status.json"};
  std::string color_shm{"/lingtu_camera_color"};
  std::string depth_shm{"/lingtu_camera_depth"};
  std::string info_shm{"/lingtu_camera_info"};
  std::vector<std::string> capture_args;
  RecordSource record_source{RecordSource::kChild};
  bool capture_bin_explicit{false};
  int domain_id{0};
  int capture_stale_timeout_ms{15000};
  std::uint32_t shm_slot_count{lingtu::drivers::camera::shm::kDefaultSlotCount};
  std::uint32_t shm_slot_capacity{
      lingtu::drivers::camera::shm::kDefaultSlotCapacity};
  bool publish_image_dds{false};
  std::uint64_t max_frames{0};
};

CliConfig parseArgs(int argc, char** argv) {
  CliConfig cfg;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--domain-id" || arg == "--domain") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--record-source") {
      const std::string source = next();
      if (source == "child") {
        cfg.record_source = RecordSource::kChild;
      } else if (source == "stdin") {
        cfg.record_source = RecordSource::kStdin;
      } else {
        throw std::runtime_error(
            "unsupported camera record source '" + source + "'; expected child or stdin");
      }
    } else if (arg == "--capture-bin") {
      cfg.capture_bin = next();
      cfg.capture_bin_explicit = true;
    } else if (arg == "--frame-id") {
      cfg.frame_id = next();
    } else if (arg == "--color-topic") {
      cfg.color_topic = next();
    } else if (arg == "--depth-topic") {
      cfg.depth_topic = next();
    } else if (arg == "--info-topic") {
      cfg.info_topic = next();
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--color-shm") {
      cfg.color_shm = next();
    } else if (arg == "--depth-shm") {
      cfg.depth_shm = next();
    } else if (arg == "--info-shm") {
      cfg.info_shm = next();
    } else if (arg == "--shm-slot-count") {
      cfg.shm_slot_count = static_cast<std::uint32_t>(std::stoul(next()));
    } else if (arg == "--shm-slot-capacity-bytes") {
      cfg.shm_slot_capacity = static_cast<std::uint32_t>(std::stoul(next()));
    } else if (arg == "--publish-image-dds") {
      cfg.publish_image_dds = true;
    } else if (arg == "--max-frames") {
      cfg.max_frames = static_cast<std::uint64_t>(std::stoull(next()));
    } else if (arg == "--capture-stale-timeout-ms") {
      cfg.capture_stale_timeout_ms = std::stoi(next());
    } else if (arg == "--") {
      while (i + 1 < argc) {
        cfg.capture_args.emplace_back(argv[++i]);
      }
      break;
    } else if (arg == "--help" || arg == "-h") {
      std::fprintf(stderr,
                   "usage: lingtu_camera_dds [--domain-id N] [--record-source child|stdin]\n"
                   "                         [--capture-bin PATH]\n"
                   "                         [--frame-id FRAME] [--color-topic TOPIC]\n"
                   "                         [--depth-topic TOPIC] [--info-topic TOPIC]\n"
                   "                         [--status-file PATH] [--max-frames N]\n"
                   "                         [--color-shm NAME] [--depth-shm NAME]\n"
                   "                         [--info-shm NAME] [--shm-slot-count N]\n"
                   "                         [--shm-slot-capacity-bytes N]\n"
                   "                         [--publish-image-dds]\n"
                   "                         [--capture-stale-timeout-ms N]\n"
                   "                         [-- CAPTURE_ARGS...]\n");
      std::exit(0);
    } else {
      cfg.capture_args.push_back(arg);
    }
  }
  if (!camera_record::isValidRecordTimeoutMs(cfg.capture_stale_timeout_ms)) {
    throw std::runtime_error(
        "--capture-stale-timeout-ms must be between " +
        std::to_string(camera_record::kMinRecordTimeoutMs) + " and " +
        std::to_string(camera_record::kMaxRecordTimeoutMs));
  }
  if (cfg.record_source == RecordSource::kStdin &&
      (cfg.capture_bin_explicit || !cfg.capture_args.empty())) {
    throw std::runtime_error(
        "stdin camera record source rejects capture command and arguments");
  }
  return cfg;
}

struct CaptureProcess {
  pid_t pid{-1};
  int fd{-1};
  bool owns_fd{false};

  ~CaptureProcess() {
    close();
  }

  void start(const CliConfig& cfg) {
    if (cfg.record_source == RecordSource::kStdin) {
      fd = STDIN_FILENO;
      owns_fd = false;
      return;
    }

    int pipefd[2] = {-1, -1};
    if (pipe(pipefd) != 0) {
      throw std::runtime_error(std::string("pipe: ") + std::strerror(errno));
    }
    pid = fork();
    if (pid < 0) {
      ::close(pipefd[0]);
      ::close(pipefd[1]);
      throw std::runtime_error(std::string("fork: ") + std::strerror(errno));
    }
    if (pid == 0) {
      ::close(pipefd[0]);
      dup2(pipefd[1], STDOUT_FILENO);
      ::close(pipefd[1]);
      std::vector<std::string> args;
      args.push_back(cfg.capture_bin);
      args.insert(args.end(), cfg.capture_args.begin(), cfg.capture_args.end());
      std::vector<char*> cargs;
      cargs.reserve(args.size() + 1);
      for (auto& item : args) {
        cargs.push_back(item.data());
      }
      cargs.push_back(nullptr);
      execv(cfg.capture_bin.c_str(), cargs.data());
      std::fprintf(stderr, "execv(%s): %s\n", cfg.capture_bin.c_str(), std::strerror(errno));
      _exit(127);
    }
    ::close(pipefd[1]);
    fd = pipefd[0];
    owns_fd = true;
  }

  bool readExact(
      void* out,
      std::size_t size,
      camera_record::RecordDeadline deadline) {
    const auto wait_readable = [&](int timeout_ms) {
      if (!g_running) {
        return camera_record::RecordWaitResult::kEndOfStream;
      }
      pollfd pfd{};
      pfd.fd = fd;
      pfd.events = POLLIN;
      const int wait_rc = ::poll(&pfd, 1, timeout_ms);
      if (wait_rc == 0) {
        return camera_record::RecordWaitResult::kTimeout;
      }
      if (wait_rc < 0) {
        if (errno == EINTR) {
          return camera_record::RecordWaitResult::kRetry;
        }
        throw std::runtime_error(
            std::string("poll camera capture: ") + std::strerror(errno));
      }
      if ((pfd.revents & (POLLERR | POLLNVAL)) != 0) {
        throw std::runtime_error("camera capture pipe error");
      }
      if ((pfd.revents & POLLHUP) != 0 && (pfd.revents & POLLIN) == 0) {
        return camera_record::RecordWaitResult::kEndOfStream;
      }
      return camera_record::RecordWaitResult::kReady;
    };
    const auto read_some = [&](void* destination, std::size_t remaining) {
      const ssize_t count = ::read(fd, destination, remaining);
      if (count >= 0) {
        return static_cast<std::ptrdiff_t>(count);
      }
      if (errno == EINTR) {
        return static_cast<std::ptrdiff_t>(-1);
      }
      throw std::runtime_error(
          std::string("read camera capture: ") + std::strerror(errno));
    };
    const auto result = camera_record::readExactUntil(
        out, size, deadline, wait_readable, read_some);
    if (result == camera_record::RecordReadResult::kTimeout) {
      throw std::runtime_error(
          "camera record exceeded absolute read deadline");
    }
    return result == camera_record::RecordReadResult::kComplete;
  }

  bool readHeader(
      RecordHeader& header,
      camera_record::RecordDeadline deadline) {
    std::size_t matched = 0;
    std::uint64_t skipped = 0;
    while (g_running) {
      char ch = 0;
      if (!readExact(&ch, 1, deadline)) {
        return false;
      }
      if (ch == camera_record::kMagic[matched]) {
        ++matched;
        if (matched == camera_record::kMagic.size()) {
          break;
        }
        continue;
      }
      if (matched > 0) {
        skipped += matched;
      }
      if (ch == camera_record::kMagic[0]) {
        matched = 1;
        continue;
      }
      matched = 0;
      skipped += 1;
    }
    if (matched != camera_record::kMagic.size()) {
      return false;
    }
    std::copy(
        camera_record::kMagic.begin(), camera_record::kMagic.end(), header.magic);
    auto* rest = reinterpret_cast<std::uint8_t*>(&header) + camera_record::kMagic.size();
    if (!readExact(rest, sizeof(header) - camera_record::kMagic.size(), deadline)) {
      return false;
    }
    if (skipped > 0) {
      std::fprintf(stderr,
                   "lingtu_camera_dds: skipped %llu non-record camera capture bytes\n",
                   static_cast<unsigned long long>(skipped));
    }
    return true;
  }

  void close() {
    if (fd >= 0) {
      if (owns_fd) {
        ::close(fd);
      }
      fd = -1;
      owns_fd = false;
    }
    if (pid > 0) {
      int status = 0;
      if (waitpid(pid, &status, WNOHANG) == 0) {
        kill(pid, SIGINT);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (waitpid(pid, &status, WNOHANG) == 0) {
          kill(pid, SIGKILL);
          waitpid(pid, &status, 0);
        }
      }
      pid = -1;
    }
  }
};

struct DdsRuntime {
  explicit DdsRuntime(const CliConfig& cfg)
      : participant_(checked(dds_create_participant(
            static_cast<dds_domainid_t>(cfg.domain_id), nullptr, nullptr),
            "dds_create_participant")),
        publisher_(checked(dds_create_publisher(participant_, nullptr, nullptr),
            "dds_create_publisher")) {
    if (cfg.publish_image_dds) {
      color_writer_ = createWriter(
          cfg.color_topic.c_str(), &lingtu_dds_Image_desc, "camera_color",
          lingtu::dds::QosProfile::CameraStream);
      depth_writer_ = createWriter(
          cfg.depth_topic.c_str(), &lingtu_dds_Image_desc, "camera_depth",
          lingtu::dds::QosProfile::CameraStream);
    }
    info_writer_ = createWriter(
        cfg.info_topic.c_str(), &lingtu_dds_CameraInfo_desc, "camera_info",
        lingtu::dds::QosProfile::CameraInfo);
  }

  ~DdsRuntime() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  dds_entity_t createWriter(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label,
      lingtu::dds::QosProfile profile = lingtu::dds::QosProfile::Default) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(profile);
    return checked(
        dds_create_writer(publisher_, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  void writeColor(lingtu_dds_Image& msg) {
    if (color_writer_ > 0) {
      logDdsError(dds_write(color_writer_, &msg), "dds_write(camera_color)");
    }
  }

  void writeDepth(lingtu_dds_Image& msg) {
    if (depth_writer_ > 0) {
      logDdsError(dds_write(depth_writer_, &msg), "dds_write(camera_depth)");
    }
  }

  bool imageDdsEnabled() const noexcept {
    return color_writer_ > 0 && depth_writer_ > 0;
  }

  void writeInfo(lingtu_dds_CameraInfo& msg) {
    logDdsError(dds_write(info_writer_, &msg), "dds_write(camera_info)");
  }

  dds_entity_t participant_{0};
  dds_entity_t publisher_{0};
  dds_entity_t color_writer_{0};
  dds_entity_t depth_writer_{0};
  dds_entity_t info_writer_{0};
};

struct Status {
  std::uint64_t color_frames{0};
  std::uint64_t depth_frames{0};
  std::uint64_t info_frames{0};
  double last_ts{0.0};
  std::uint64_t color_shm_sequence{0};
  std::uint64_t depth_shm_sequence{0};
  std::uint64_t info_shm_sequence{0};
  bool image_dds_enabled{false};
  std::string last_error;
};

void writeStatus(const std::string& path, const Status& status) {
  if (path.empty()) {
    return;
  }
  try {
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    std::ofstream out(path, std::ios::trunc);
    out << "{"
        << "\"schema\":\"lingtu.camera_dds_status.v1\","
        << "\"status\":\"" << (status.last_error.empty() ? "running" : "error") << "\","
        << "\"color_frames\":" << status.color_frames << ","
        << "\"depth_frames\":" << status.depth_frames << ","
        << "\"info_frames\":" << status.info_frames << ","
        << "\"last_ts\":" << status.last_ts << ","
        << "\"data_plane\":\"posix_shm\","
        << "\"image_dds_enabled\":"
        << (status.image_dds_enabled ? "true" : "false") << ","
        << "\"color_shm_sequence\":" << status.color_shm_sequence << ","
        << "\"depth_shm_sequence\":" << status.depth_shm_sequence << ","
        << "\"info_shm_sequence\":" << status.info_shm_sequence << ","
        << "\"last_error\":\"" << status.last_error << "\""
        << "}\n";
  } catch (...) {
  }
}

std::string encodingFor(std::uint32_t format) {
  if (format == camera_record::kFormatRgb8) {
    return "rgb8";
  }
  if (format == camera_record::kFormatBgr8) {
    return "bgr8";
  }
  if (format == camera_record::kFormatDepthU16) {
    return "16UC1";
  }
  return "";
}

std::uint32_t stepFor(const RecordHeader& header) {
  if (header.format == camera_record::kFormatDepthU16) {
    return header.width * 2;
  }
  return header.width * std::max<std::uint32_t>(1, header.channels);
}

std::uint64_t timestampNs(double timestamp_s) {
  if (!camera_record::isValidTimestampSeconds(timestamp_s)) {
    throw std::runtime_error("camera_record_timestamp_invalid");
  }
  return static_cast<std::uint64_t>(timestamp_s * 1e9);
}

lingtu::drivers::camera::shm::FrameMetadata toShmMetadata(
    const RecordHeader& header,
    lingtu::drivers::camera::shm::StreamKind stream_kind,
    const std::string& encoding,
    const std::string& frame_id) {
  lingtu::drivers::camera::shm::FrameMetadata metadata;
  metadata.stream_kind = stream_kind;
  metadata.timestamp_ns = timestampNs(header.timestamp_s);
  metadata.width = header.width;
  metadata.height = header.height;
  metadata.stride = stream_kind == lingtu::drivers::camera::shm::StreamKind::kInfo
      ? 0u
      : stepFor(header);
  metadata.encoding = encoding;
  metadata.frame_id = frame_id;
  metadata.fx = header.fx;
  metadata.fy = header.fy;
  metadata.cx = header.cx;
  metadata.cy = header.cy;
  metadata.depth_scale = header.depth_scale_m;
  metadata.dist_k1 = header.dist_k1;
  metadata.dist_k2 = header.dist_k2;
  metadata.dist_p1 = header.dist_p1;
  metadata.dist_p2 = header.dist_p2;
  metadata.dist_k3 = header.dist_k3;
  return metadata;
}

RecordHeader withCalibrationFrom(
    const RecordHeader& frame,
    const RecordHeader& calibration) {
  RecordHeader merged = frame;
  merged.fx = calibration.fx;
  merged.fy = calibration.fy;
  merged.cx = calibration.cx;
  merged.cy = calibration.cy;
  merged.depth_scale_m = calibration.depth_scale_m;
  merged.dist_k1 = calibration.dist_k1;
  merged.dist_k2 = calibration.dist_k2;
  merged.dist_p1 = calibration.dist_p1;
  merged.dist_p2 = calibration.dist_p2;
  merged.dist_k3 = calibration.dist_k3;
  return merged;
}

lingtu_dds_Image toImageMsg(
    const RecordHeader& header,
    std::vector<std::uint8_t>& payload,
    std::string& encoding,
    const std::string& frame_id) {
  lingtu_dds_Image msg{};
  fillHeader(msg.header, header.timestamp_s, frame_id.c_str());
  msg.height = header.height;
  msg.width = header.width;
  encoding = encodingFor(header.format);
  msg.encoding = const_cast<char*>(encoding.c_str());
  msg.is_bigendian = false;
  msg.step = stepFor(header);
  msg.data._maximum = static_cast<std::uint32_t>(payload.size());
  msg.data._length = static_cast<std::uint32_t>(payload.size());
  msg.data._buffer = payload.data();
  msg.data._release = false;
  return msg;
}

lingtu_dds_CameraInfo toCameraInfoMsg(
    const RecordHeader& header,
    std::vector<double>& d,
    const std::string& frame_id) {
  lingtu_dds_CameraInfo msg{};
  fillHeader(msg.header, header.timestamp_s, frame_id.c_str());
  msg.height = header.height;
  msg.width = header.width;
  msg.depth_scale = header.depth_scale_m;
  msg.distortion_model = const_cast<char*>("plumb_bob");
  d = {
      header.dist_k1,
      header.dist_k2,
      header.dist_p1,
      header.dist_p2,
      header.dist_k3,
  };
  msg.d._maximum = static_cast<std::uint32_t>(d.size());
  msg.d._length = static_cast<std::uint32_t>(d.size());
  msg.d._buffer = d.data();
  msg.d._release = false;
  msg.k[0] = header.fx;
  msg.k[1] = 0.0;
  msg.k[2] = header.cx;
  msg.k[3] = 0.0;
  msg.k[4] = header.fy;
  msg.k[5] = header.cy;
  msg.k[6] = 0.0;
  msg.k[7] = 0.0;
  msg.k[8] = 1.0;
  msg.r[0] = 1.0;
  msg.r[1] = 0.0;
  msg.r[2] = 0.0;
  msg.r[3] = 0.0;
  msg.r[4] = 1.0;
  msg.r[5] = 0.0;
  msg.r[6] = 0.0;
  msg.r[7] = 0.0;
  msg.r[8] = 1.0;
  msg.p[0] = header.fx;
  msg.p[1] = 0.0;
  msg.p[2] = header.cx;
  msg.p[3] = 0.0;
  msg.p[4] = 0.0;
  msg.p[5] = header.fy;
  msg.p[6] = header.cy;
  msg.p[7] = 0.0;
  msg.p[8] = 0.0;
  msg.p[9] = 0.0;
  msg.p[10] = 1.0;
  msg.p[11] = 0.0;
  return msg;
}

void validateHeader(const RecordHeader& header) {
  const auto validation = camera_record::validateRecordHeader(header);
  if (validation != camera_record::RecordValidation::kValid) {
    throw std::runtime_error(
        camera_record::recordValidationReason(validation));
  }
}

lingtu::drivers::camera::shm::WriterConfig shmWriterConfig(
    const CliConfig& cfg,
    const std::string& name) {
  lingtu::drivers::camera::shm::WriterConfig writer;
  writer.name = name;
  writer.slot_count = cfg.shm_slot_count;
  writer.slot_capacity = cfg.shm_slot_capacity;
  return writer;
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, stopSignal);
  std::signal(SIGTERM, stopSignal);

  try {
    const CliConfig cfg = parseArgs(argc, argv);
    DdsRuntime dds(cfg);
    lingtu::drivers::camera::shm::FrameWriter color_shm(
        shmWriterConfig(cfg, cfg.color_shm));
    lingtu::drivers::camera::shm::FrameWriter depth_shm(
        shmWriterConfig(cfg, cfg.depth_shm));
    lingtu::drivers::camera::shm::FrameWriter info_shm(
        shmWriterConfig(cfg, cfg.info_shm));
    CaptureProcess capture;
    capture.start(cfg);

    Status status;
    status.image_dds_enabled = dds.imageDdsEnabled();
    std::uint64_t records = 0;
    RecordHeader last_info_header{};
    bool has_info = false;
    double next_info_publish_s = 0.0;
    std::fprintf(stderr,
                 "lingtu_camera_dds: domain=%d color_shm=%s depth_shm=%s info_shm=%s "
                 "image_dds=%s record_source=%s capture=%s\n",
                 cfg.domain_id,
                 cfg.color_shm.c_str(),
                 cfg.depth_shm.c_str(),
                 cfg.info_shm.c_str(),
                 cfg.publish_image_dds ? "enabled" : "disabled",
                 recordSourceName(cfg.record_source),
                 cfg.record_source == RecordSource::kStdin ? "stdin" : cfg.capture_bin.c_str());

    while (g_running) {
      RecordHeader header{};
      std::vector<std::uint8_t> payload;
      try {
        const auto record_deadline =
            camera_record::makeRecordDeadline(cfg.capture_stale_timeout_ms);
        if (!capture.readHeader(header, record_deadline)) {
          if (g_running && (cfg.max_frames == 0 || records < cfg.max_frames)) {
            status.last_error = "camera capture ended before publishing the next record";
            writeStatus(cfg.status_file, status);
            throw std::runtime_error(status.last_error);
          }
          break;
        }
        validateHeader(header);
        const auto sequence_validation =
            camera_record::validateRecordSequence(header, has_info);
        if (sequence_validation != camera_record::RecordValidation::kValid) {
          throw std::runtime_error(
              camera_record::recordValidationReason(sequence_validation));
        }
        payload.resize(header.payload_size);
        if (!payload.empty() &&
            !capture.readExact(payload.data(), payload.size(), record_deadline)) {
          status.last_error = "camera capture ended before publishing full payload";
          writeStatus(cfg.status_file, status);
          throw std::runtime_error(status.last_error);
        }
      } catch (const std::exception& exc) {
        status.last_error = exc.what();
        writeStatus(cfg.status_file, status);
        throw;
      }

      if (header.kind == kKindIntrinsics) {
        last_info_header = header;
        has_info = true;
        const auto metadata = toShmMetadata(
            header,
            lingtu::drivers::camera::shm::StreamKind::kInfo,
            "camera_info",
            cfg.frame_id);
        status.info_shm_sequence = info_shm.publish(metadata, nullptr, 0);
        std::vector<double> d;
        auto msg = toCameraInfoMsg(header, d, cfg.frame_id);
        dds.writeInfo(msg);
        status.info_frames += 1;
        next_info_publish_s = nowSeconds() + 1.0;
      } else if (header.kind == kKindColor) {
        const std::string encoding = encodingFor(header.format);
        const RecordHeader shm_header = has_info
            ? withCalibrationFrom(header, last_info_header)
            : header;
        const auto metadata = toShmMetadata(
            shm_header,
            lingtu::drivers::camera::shm::StreamKind::kColor,
            encoding,
            cfg.frame_id);
        status.color_shm_sequence =
            color_shm.publish(metadata, payload.data(), payload.size());
        if (dds.imageDdsEnabled()) {
          std::string dds_encoding;
          auto msg = toImageMsg(header, payload, dds_encoding, cfg.frame_id);
          dds.writeColor(msg);
        }
        status.color_frames += 1;
      } else if (header.kind == kKindDepth) {
        const std::string encoding = encodingFor(header.format);
        const RecordHeader shm_header = has_info
            ? withCalibrationFrom(header, last_info_header)
            : header;
        const auto metadata = toShmMetadata(
            shm_header,
            lingtu::drivers::camera::shm::StreamKind::kDepth,
            encoding,
            cfg.frame_id);
        status.depth_shm_sequence =
            depth_shm.publish(metadata, payload.data(), payload.size());
        if (dds.imageDdsEnabled()) {
          std::string dds_encoding;
          auto msg = toImageMsg(header, payload, dds_encoding, cfg.frame_id);
          dds.writeDepth(msg);
        }
        status.depth_frames += 1;
      }
      const double now_s = nowSeconds();
      if (has_info && now_s >= next_info_publish_s) {
        RecordHeader info_header = last_info_header;
        info_header.timestamp_s = now_s;
        const auto metadata = toShmMetadata(
            info_header,
            lingtu::drivers::camera::shm::StreamKind::kInfo,
            "camera_info",
            cfg.frame_id);
        status.info_shm_sequence = info_shm.publish(metadata, nullptr, 0);
        std::vector<double> d;
        auto msg = toCameraInfoMsg(info_header, d, cfg.frame_id);
        dds.writeInfo(msg);
        status.info_frames += 1;
        next_info_publish_s = now_s + 1.0;
      }
      status.last_ts = now_s;
      records += 1;
      if (records % 30 == 0) {
        writeStatus(cfg.status_file, status);
      }
      if (cfg.max_frames > 0 && records >= cfg.max_frames) {
        break;
      }
    }
    writeStatus(cfg.status_file, status);
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_camera_dds failed: %s\n", exc.what());
    return 1;
  }
}
