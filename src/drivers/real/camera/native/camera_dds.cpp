#include "message/cpp/dds_topics.hpp"
#include "message/cpp/dds_qos_profiles.hpp"
#include "shm_frame_ring.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

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

constexpr char kMagic[4] = {'L', 'T', 'O', 'B'};
constexpr std::uint16_t kVersion = 2;
constexpr std::uint16_t kKindIntrinsics = 1;
constexpr std::uint16_t kKindColor = 2;
constexpr std::uint16_t kKindDepth = 3;
constexpr std::uint32_t kFmtRgb8 = 1;
constexpr std::uint32_t kFmtBgr8 = 2;
constexpr std::uint32_t kFmtDepthU16 = 3;

#pragma pack(push, 1)
struct RecordHeader {
  char magic[4];
  std::uint16_t version;
  std::uint16_t kind;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t channels;
  std::uint32_t format;
  double timestamp_s;
  double fx;
  double fy;
  double cx;
  double cy;
  double depth_scale_m;
  std::uint32_t payload_size;
  double dist_k1;
  double dist_k2;
  double dist_p1;
  double dist_p2;
  double dist_k3;
};
#pragma pack(pop)

static_assert(sizeof(RecordHeader) == 116, "unexpected camera record header size");

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (stamp_s <= 0.0) {
    stamp_s = nowSeconds();
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
    } else if (arg == "--capture-bin") {
      cfg.capture_bin = next();
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
                   "usage: lingtu_camera_dds [--domain-id N] [--capture-bin PATH]\n"
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
  return cfg;
}

struct CaptureProcess {
  pid_t pid{-1};
  int fd{-1};

  ~CaptureProcess() {
    close();
  }

  void start(const CliConfig& cfg) {
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
  }

  bool readExact(void* out, std::size_t size, int stale_timeout_ms) {
    auto* dst = static_cast<std::uint8_t*>(out);
    std::size_t got = 0;
    while (got < size && g_running) {
      pollfd pfd{};
      pfd.fd = fd;
      pfd.events = POLLIN;
      const int wait_rc = ::poll(&pfd, 1, stale_timeout_ms);
      if (wait_rc == 0) {
        throw std::runtime_error(
            "camera capture produced no bytes before stale timeout");
      }
      if (wait_rc < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error(std::string("poll camera capture: ") + std::strerror(errno));
      }
      if ((pfd.revents & (POLLERR | POLLNVAL)) != 0) {
        throw std::runtime_error("camera capture pipe error");
      }
      if ((pfd.revents & POLLHUP) != 0 && (pfd.revents & POLLIN) == 0) {
        return false;
      }
      const ssize_t n = ::read(fd, dst + got, size - got);
      if (n > 0) {
        got += static_cast<std::size_t>(n);
        continue;
      }
      if (n == 0) {
        return false;
      }
      if (errno == EINTR) {
        continue;
      }
      throw std::runtime_error(std::string("read camera capture: ") + std::strerror(errno));
    }
    return got == size;
  }

  bool readHeader(RecordHeader& header, int stale_timeout_ms) {
    std::size_t matched = 0;
    std::uint64_t skipped = 0;
    while (g_running) {
      char ch = 0;
      if (!readExact(&ch, 1, stale_timeout_ms)) {
        return false;
      }
      if (ch == kMagic[matched]) {
        ++matched;
        if (matched == sizeof(kMagic)) {
          break;
        }
        continue;
      }
      if (matched > 0) {
        skipped += matched;
      }
      if (ch == kMagic[0]) {
        matched = 1;
        continue;
      }
      matched = 0;
      skipped += 1;
    }
    if (matched != sizeof(kMagic)) {
      return false;
    }
    std::memcpy(header.magic, kMagic, sizeof(kMagic));
    auto* rest = reinterpret_cast<std::uint8_t*>(&header) + sizeof(kMagic);
    if (!readExact(rest, sizeof(header) - sizeof(kMagic), stale_timeout_ms)) {
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
      ::close(fd);
      fd = -1;
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
  if (format == kFmtRgb8) {
    return "rgb8";
  }
  if (format == kFmtBgr8) {
    return "bgr8";
  }
  if (format == kFmtDepthU16) {
    return "16UC1";
  }
  return "";
}

std::uint32_t stepFor(const RecordHeader& header) {
  if (header.format == kFmtDepthU16) {
    return header.width * 2;
  }
  return header.width * std::max<std::uint32_t>(1, header.channels);
}

std::uint64_t timestampNs(double timestamp_s) {
  const double value = timestamp_s > 0.0 ? timestamp_s : nowSeconds();
  return static_cast<std::uint64_t>(value * 1e9);
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
  if (std::memcmp(header.magic, kMagic, 4) != 0) {
    throw std::runtime_error("invalid camera record magic");
  }
  if (header.version != kVersion) {
    throw std::runtime_error("unsupported camera record version");
  }
  if (header.payload_size > 128u * 1024u * 1024u) {
    throw std::runtime_error("camera record payload too large");
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
                 "image_dds=%s capture=%s\n",
                 cfg.domain_id,
                 cfg.color_shm.c_str(),
                 cfg.depth_shm.c_str(),
                 cfg.info_shm.c_str(),
                 cfg.publish_image_dds ? "enabled" : "disabled",
                 cfg.capture_bin.c_str());

    while (g_running) {
      RecordHeader header{};
      std::vector<std::uint8_t> payload;
      try {
        if (!capture.readHeader(header, cfg.capture_stale_timeout_ms)) {
          if (g_running && (cfg.max_frames == 0 || records < cfg.max_frames)) {
            status.last_error = "camera capture ended before publishing the next record";
            writeStatus(cfg.status_file, status);
            throw std::runtime_error(status.last_error);
          }
          break;
        }
        validateHeader(header);
        payload.resize(header.payload_size);
        if (!payload.empty() &&
            !capture.readExact(payload.data(), payload.size(), cfg.capture_stale_timeout_ms)) {
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
        const auto metadata = toShmMetadata(
            header,
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
        const auto metadata = toShmMetadata(
            header,
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
