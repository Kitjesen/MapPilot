#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <streambuf>
#include <thread>
#include <utility>
#include <vector>

#include "dds_domain.hpp"
#include "dds/dds.h"
#include "messages.h"
#include "local_endpoint_server.hpp"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "native/dds_module.hpp"
#include "replay_deadline_restamper.hpp"
#include "run_plan_process_environment.hpp"
#include "sensor_record_protocol.hpp"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

namespace adapter = lingtu::sim::dds_adapter;
namespace lidar = lingtu::drivers::lidar;
namespace local_endpoint = lingtu::sim::local_endpoint;
namespace run_plan_process = lingtu::sim::run_plan_process;

constexpr char kSensorEndpointProtocol[] = "ltu1-v1";

enum class Stream { All, Lidar, Imu, Camera };

struct EndpointContract final {
  const char *role;
  const char *readiness;
  const char *auth;
};

EndpointContract endpoint_contract(const Stream stream) {
  switch (stream) {
    case Stream::Lidar:
      return {"lidar_publisher", "lidar.ready.json", "lidar.auth"};
    case Stream::Imu:
      return {"imu_publisher", "imu.ready.json", "imu.auth"};
    case Stream::Camera:
      return {"camera_publisher", "camera.ready.json", "camera.auth"};
    case Stream::All:
      break;
  }
  throw std::runtime_error("local endpoint requires --stream lidar|imu|camera");
}

dds_entity_t checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

struct CliConfig {
  bool show_help{false};
  bool dds{false};
  bool stdin_records{false};
  bool validate_records{false};
  bool restamp_stdin_records{false};
  bool navigation_fixture{false};
  bool sim_lidar_raw_frame{false};
  bool local_endpoint_records{false};
  bool stream_explicit{false};
  bool replay_rate_explicit{false};
  Stream stream{Stream::All};
  int domain_id{0};
  double replay_rate{1.0};
  double scan_window_s{0.1};
  double imu_publish_hz{0.0};
  std::string lidar_frame{"lidar_link"};
  std::string imu_frame{"imu_link"};
  std::string camera_frame{"camera_link"};
  std::filesystem::path ready_file;
};

constexpr const char *kUsage =
    "usage: lingtu_mujoco_sensor_publisher --stdin-records "
    "[--dds|--validate-records] [--domain-id N] [--lidar-frame FRAME] "
    "[--imu-frame FRAME] [--camera-frame FRAME] [--navigation-fixture] "
    "[--sim-lidar-raw-frame] "
    "[--restamp-stdin-records] [--replay-rate RATE] "
    "[--scan-window SEC|--publish-freq HZ] [--imu-publish-freq HZ] "
    "[--ready-file PATH] [--stream lidar|imu|camera]\n"
    "       lingtu_mujoco_sensor_publisher --local-endpoint --dds "
    "--stream lidar|imu|camera "
    "[--domain-id N] [--lidar-frame FRAME] [--imu-frame FRAME] "
    "[--camera-frame FRAME] "
    "[--navigation-fixture] "
    "[--scan-window SEC|--publish-freq HZ] [--imu-publish-freq HZ]\n";

volatile std::sig_atomic_t g_stop_requested = 0;

void signal_stop(int /*signal*/) {
  g_stop_requested = 1;
}

std::string next_arg(int &index, int argc, const char *const argv[]) {
  if (index + 1 >= argc) {
    throw std::runtime_error(std::string("missing value for ") + argv[index]);
  }
  return argv[++index];
}

CliConfig parse_args(int argc, const char *const argv[]) {
  CliConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--dds") {
      config.dds = true;
    } else if (arg == "--stdin-records") {
      config.stdin_records = true;
    } else if (arg == "--local-endpoint") {
      config.local_endpoint_records = true;
    } else if (arg == "--stream") {
      if (config.stream_explicit) {
        throw std::runtime_error("--stream may be specified only once");
      }
      config.stream_explicit = true;
      const std::string value = next_arg(i, argc, argv);
      if (value == "lidar") {
        config.stream = Stream::Lidar;
      } else if (value == "imu") {
        config.stream = Stream::Imu;
      } else if (value == "camera") {
        config.stream = Stream::Camera;
      } else {
        throw std::runtime_error("--stream must be lidar, imu, or camera");
      }
    } else if (arg == "--validate-records") {
      config.stdin_records = true;
      config.validate_records = true;
    } else if (arg == "--restamp-stdin-records") {
      config.restamp_stdin_records = true;
    } else if (arg == "--navigation-fixture") {
      config.navigation_fixture = true;
    } else if (arg == "--sim-lidar-raw-frame") {
      config.sim_lidar_raw_frame = true;
    } else if (arg == "--domain-id") {
      config.domain_id = adapter::parse_supported_dds_domain_id(next_arg(i, argc, argv));
    } else if (arg == "--lidar-frame") {
      config.lidar_frame = next_arg(i, argc, argv);
    } else if (arg == "--imu-frame") {
      config.imu_frame = next_arg(i, argc, argv);
    } else if (arg == "--camera-frame") {
      config.camera_frame = next_arg(i, argc, argv);
    } else if (arg == "--ready-file") {
      config.ready_file = std::filesystem::path(next_arg(i, argc, argv));
    } else if (arg == "--replay-rate") {
      config.replay_rate = std::stod(next_arg(i, argc, argv));
      config.replay_rate_explicit = true;
    } else if (arg == "--scan-window") {
      config.scan_window_s = std::stod(next_arg(i, argc, argv));
    } else if (arg == "--publish-freq") {
      const double hz = std::stod(next_arg(i, argc, argv));
      if (!std::isfinite(hz) || hz <= 0.0) {
        throw std::runtime_error("--publish-freq must be finite and positive");
      }
      config.scan_window_s = 1.0 / hz;
    } else if (arg == "--imu-publish-freq") {
      const double hz = std::stod(next_arg(i, argc, argv));
      if (!std::isfinite(hz) || hz < 0.0) {
        throw std::runtime_error("--imu-publish-freq must be finite and non-negative");
      }
      config.imu_publish_hz = hz;
    } else if (arg == "--help" || arg == "-h") {
      config.show_help = true;
    } else {
      throw std::runtime_error("unexpected argument: " + arg);
    }
  }
  if (config.show_help) {
    return config;
  }
  if (config.local_endpoint_records) {
    if (config.stdin_records || config.validate_records) {
      throw std::runtime_error("--local-endpoint cannot be combined with stdin record mode");
    }
    if (!config.dds) {
      throw std::runtime_error("--local-endpoint requires --dds");
    }
    if (!config.ready_file.empty()) {
      throw std::runtime_error("--ready-file is not a Product readiness contract");
    }
    if (config.restamp_stdin_records) {
      throw std::runtime_error("stdin replay restamping is invalid in --local-endpoint mode");
    }
    if (!config.stream_explicit) {
      throw std::runtime_error("--local-endpoint requires --stream lidar|imu|camera");
    }
    if (config.replay_rate_explicit && config.replay_rate != 0.0) {
      throw std::runtime_error("--local-endpoint is live input and requires --replay-rate 0");
    }
    config.replay_rate = 0.0;
  } else if (!config.stdin_records) {
    throw std::runtime_error("--stdin-records, --validate-records, or --local-endpoint is required");
  }
  if (config.stdin_records && !config.dds && !config.validate_records) {
    throw std::runtime_error("--stdin-records requires --dds");
  }
  if (config.dds && config.validate_records) {
    throw std::runtime_error("--validate-records runs without DDS");
  }
  if (config.navigation_fixture && !config.stdin_records && !config.local_endpoint_records) {
    throw std::runtime_error("--navigation-fixture requires a sensor record input mode");
  }
  if (config.navigation_fixture && config.stream != Stream::All &&
      config.stream != Stream::Lidar) {
    throw std::runtime_error("--navigation-fixture requires --stream lidar");
  }
  if (config.sim_lidar_raw_frame && !config.dds) {
    throw std::runtime_error("--sim-lidar-raw-frame requires --dds");
  }
  if (config.sim_lidar_raw_frame && config.stream != Stream::All &&
      config.stream != Stream::Lidar) {
    throw std::runtime_error("--sim-lidar-raw-frame requires --stream lidar");
  }
  if (!std::isfinite(config.replay_rate) || config.replay_rate < 0.0) {
    throw std::runtime_error("--replay-rate must be finite and non-negative");
  }
  if (config.scan_window_s < 0.0 || !std::isfinite(config.scan_window_s)) {
    throw std::runtime_error("--scan-window must be finite and non-negative");
  }
  return config;
}

class EndpointStreamBuf final : public std::streambuf {
 public:
  explicit EndpointStreamBuf(local_endpoint::ClientStream stream) : stream_(std::move(stream)) {
    setg(nullptr, nullptr, nullptr);
  }

 protected:
  int_type underflow() override {
    if (gptr() != nullptr && gptr() < egptr()) {
      return traits_type::to_int_type(*gptr());
    }
    while (g_stop_requested == 0) {
      try {
        const auto chunk = stream_.readSome(8192U, std::chrono::milliseconds(250));
        if (!chunk.has_value()) {
          return traits_type::eof();
        }
        buffer_ = std::move(*chunk);
        if (buffer_.empty()) {
          continue;
        }
        auto *begin = reinterpret_cast<char *>(buffer_.data());
        setg(begin, begin, begin + static_cast<std::ptrdiff_t>(buffer_.size()));
        return traits_type::to_int_type(*gptr());
      } catch (const local_endpoint::EndpointTimeout &) {
        continue;
      }
    }
    return traits_type::eof();
  }

 private:
  local_endpoint::ClientStream stream_;
  std::vector<std::uint8_t> buffer_;
};

class ReadyFile final {
 public:
  explicit ReadyFile(std::filesystem::path path) : path_(std::move(path)) {
    if (!path_.empty()) {
      std::error_code ec;
      std::filesystem::remove(path_, ec);
      std::filesystem::remove(tmp_path(), ec);
    }
  }

  void mark_ready(const std::string &topic = {}) const {
    if (path_.empty()) {
      return;
    }
    if (path_.has_parent_path()) {
      std::filesystem::create_directories(path_.parent_path());
    }
    const auto tmp = tmp_path();
    {
      std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
      if (!out) {
        throw std::runtime_error("failed to open ready temp file: " + tmp.string());
      }
      out << "{\"ready\":true,\"schema\":\"lingtu.mujoco_sensor_publisher.ready.v1\"";
      if (!topic.empty()) {
        out << ",\"topic\":\"" << topic << "\"";
      }
      out << "}\n";
      if (!out) {
        throw std::runtime_error("failed to write ready temp file: " + tmp.string());
      }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path_, ec);
    if (!ec) {
      return;
    }
    std::filesystem::remove(path_, ec);
    ec.clear();
    std::filesystem::rename(tmp, path_, ec);
    if (ec) {
      throw std::runtime_error("failed to publish ready file " + path_.string() + ": " +
                               ec.message());
    }
  }

 private:
  std::filesystem::path tmp_path() const {
    auto tmp = path_;
    tmp += ".tmp";
    return tmp;
  }

  std::filesystem::path path_;
};

class SimLidarRawFramePublisher final {
 public:
  SimLidarRawFramePublisher(int domain_id, std::string lidar_frame)
      : lidar_frame_(std::move(lidar_frame)) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant(sim_lidar)");
    publisher_ = checked(
        dds_create_publisher(participant_, nullptr, nullptr),
        "dds_create_publisher(sim_lidar)");
    topic_ = checked(
        dds_create_topic(participant_, &lingtu_dds_LivoxFrame_desc,
                         lingtu::message::kSimLidarRawFrame.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(sim_lidar_raw_frame)");
    auto qos = lingtu::dds::make_qos(
        lingtu::dds::qos_for_topic(lingtu::message::kSimLidarRawFrame.dds_topic));
    writer_ = checked(
        dds_create_writer(publisher_, topic_, qos.get(), nullptr),
        "dds_create_writer(sim_lidar_raw_frame)");
  }

  ~SimLidarRawFramePublisher() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  SimLidarRawFramePublisher(const SimLidarRawFramePublisher &) = delete;
  SimLidarRawFramePublisher &operator=(const SimLidarRawFramePublisher &) = delete;

  void publish(std::uint8_t lidar_id, std::uint64_t timestamp_ns,
               const std::vector<lidar::Point> &points) {
    if (points.empty()) {
      throw std::runtime_error("sim MID-360 publisher refuses empty frames");
    }
    std::vector<lingtu_dds_LivoxPoint> dds_points;
    dds_points.reserve(points.size());
    for (const auto &point : points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        throw std::runtime_error("sim MID-360 publisher refuses non-finite points");
      }
      lingtu_dds_LivoxPoint out{};
      out.offset_time = point.offset_time_ns;
      out.x = point.x;
      out.y = point.y;
      out.z = point.z;
      out.reflectivity = static_cast<std::uint8_t>(point.intensity);
      out.tag = point.tag;
      out.line = point.line;
      dds_points.push_back(out);
    }
    lingtu_dds_LivoxFrame msg{};
    msg.header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000ULL);
    msg.header.stamp.nanosec =
        static_cast<std::uint32_t>(timestamp_ns % 1000000000ULL);
    msg.header.frame_id = const_cast<char *>(lidar_frame_.c_str());
    msg.timebase = timestamp_ns;
    msg.point_num = static_cast<std::uint32_t>(dds_points.size());
    msg.lidar_id = lidar_id;
    msg.points._maximum = msg.point_num;
    msg.points._length = msg.point_num;
    msg.points._buffer = dds_points.data();
    msg.points._release = false;
    checked(dds_write(writer_, &msg), "dds_write(sim_lidar_raw_frame)");
  }

 private:
  std::string lidar_frame_;
  dds_entity_t participant_{DDS_RETCODE_ERROR};
  dds_entity_t publisher_{DDS_RETCODE_ERROR};
  dds_entity_t topic_{DDS_RETCODE_ERROR};
  dds_entity_t writer_{DDS_RETCODE_ERROR};
};

class CameraDdsPublisher final {
 public:
  CameraDdsPublisher(int domain_id, std::string frame_id) : frame_id_(std::move(frame_id)) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(camera)");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher(camera)");
    color_writer_ = create_writer(lingtu::message::kCameraColor, &lingtu_dds_Image_desc);
    depth_writer_ = create_writer(lingtu::message::kCameraDepth, &lingtu_dds_Image_desc);
    info_writer_ = create_writer(lingtu::message::kCameraInfo, &lingtu_dds_CameraInfo_desc);
  }

  ~CameraDdsPublisher() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  CameraDdsPublisher(const CameraDdsPublisher &) = delete;
  CameraDdsPublisher &operator=(const CameraDdsPublisher &) = delete;

  void publish(std::uint64_t timestamp_ns, adapter::CameraRecord camera) {
    camera.header.timestamp_s = static_cast<double>(timestamp_ns) / 1000000000.0;
    namespace record = lingtu::drivers::camera::record;
    if (camera.header.kind == record::kKindIntrinsics) {
      lingtu_dds_CameraInfo msg{};
      fill_header(msg.header, timestamp_ns);
      msg.height = camera.header.height;
      msg.width = camera.header.width;
      msg.depth_scale = camera.header.depth_scale_m;
      msg.distortion_model = const_cast<char *>("plumb_bob");
      std::array<double, 5> distortion{{camera.header.dist_k1, camera.header.dist_k2,
                                        camera.header.dist_p1, camera.header.dist_p2,
                                        camera.header.dist_k3}};
      msg.d._maximum = static_cast<std::uint32_t>(distortion.size());
      msg.d._length = static_cast<std::uint32_t>(distortion.size());
      msg.d._buffer = distortion.data();
      msg.d._release = false;
      msg.k[0] = camera.header.fx;
      msg.k[2] = camera.header.cx;
      msg.k[4] = camera.header.fy;
      msg.k[5] = camera.header.cy;
      msg.k[8] = 1.0;
      msg.r[0] = msg.r[4] = msg.r[8] = 1.0;
      msg.p[0] = camera.header.fx;
      msg.p[2] = camera.header.cx;
      msg.p[5] = camera.header.fy;
      msg.p[6] = camera.header.cy;
      msg.p[10] = 1.0;
      checked(dds_write(info_writer_, &msg), "dds_write(camera_info)");
      return;
    }

    lingtu_dds_Image msg{};
    fill_header(msg.header, timestamp_ns);
    msg.height = camera.header.height;
    msg.width = camera.header.width;
    std::string encoding;
    if (camera.header.kind == record::kKindColor) {
      encoding = camera.header.format == record::kFormatRgb8 ? "rgb8" : "bgr8";
      msg.step = camera.header.width * 3U;
    } else {
      encoding = "16UC1";
      msg.step = camera.header.width * 2U;
    }
    msg.encoding = const_cast<char *>(encoding.c_str());
    msg.is_bigendian = false;
    msg.data._maximum = static_cast<std::uint32_t>(camera.payload.size());
    msg.data._length = static_cast<std::uint32_t>(camera.payload.size());
    msg.data._buffer = camera.payload.data();
    msg.data._release = false;
    checked(
        dds_write(camera.header.kind == record::kKindColor ? color_writer_ : depth_writer_, &msg),
        camera.header.kind == record::kKindColor ? "dds_write(camera_color)"
                                                 : "dds_write(camera_depth)");
  }

 private:
  dds_entity_t create_writer(const lingtu::message::TopicContract &contract,
                             const dds_topic_descriptor_t *descriptor) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, descriptor, contract.dds_topic.data(), nullptr, nullptr),
        "dds_create_topic(camera)");
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(dds_create_writer(publisher_, topic, qos.get(), nullptr),
                   "dds_create_writer(camera)");
  }

  void fill_header(lingtu_dds_Header &header, std::uint64_t timestamp_ns) {
    header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000ULL);
    header.stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1000000000ULL);
    header.frame_id = const_cast<char *>(frame_id_.c_str());
  }

  std::string frame_id_;
  dds_entity_t participant_{DDS_RETCODE_ERROR};
  dds_entity_t publisher_{DDS_RETCODE_ERROR};
  dds_entity_t color_writer_{DDS_RETCODE_ERROR};
  dds_entity_t depth_writer_{DDS_RETCODE_ERROR};
  dds_entity_t info_writer_{DDS_RETCODE_ERROR};
};

class ScanAccumulator final {
 public:
  explicit ScanAccumulator(double window_s)
      : window_ns_(static_cast<std::uint64_t>(std::max(0.0, window_s) * 1000000000.0)) {}

  std::vector<std::pair<std::uint64_t, std::vector<lidar::Point>>>
  submit(std::uint64_t timestamp_ns, const std::vector<lidar::Point> &points) {
    std::vector<std::pair<std::uint64_t, std::vector<lidar::Point>>> ready;
    if (points.empty()) {
      return ready;
    }
    const std::uint64_t packet_end_ns = packet_end_time_ns(timestamp_ns, points);
    if (has_scan_ && timestamp_ns < scan_start_ns_) {
      ready.push_back(take());
    }
    if (has_scan_ && window_ns_ > 0 && packet_end_ns > scan_start_ns_ &&
        packet_end_ns - scan_start_ns_ > window_ns_ && !points_.empty()) {
      ready.push_back(take());
    }
    if (!has_scan_) {
      has_scan_ = true;
      scan_start_ns_ = timestamp_ns;
    }
    append(timestamp_ns, points);
    if (window_ns_ == 0) {
      ready.push_back(take());
    }
    return ready;
  }

  std::optional<std::pair<std::uint64_t, std::vector<lidar::Point>>> flush() {
    if (!has_scan_ || points_.empty()) {
      return std::nullopt;
    }
    return take();
  }

 private:
  static std::uint64_t packet_end_time_ns(std::uint64_t timestamp_ns,
                                          const std::vector<lidar::Point> &points) {
    std::uint64_t max_offset_ns = 0;
    for (const auto &point : points) {
      max_offset_ns = std::max<std::uint64_t>(max_offset_ns, point.offset_time_ns);
    }
    return max_offset_ns > std::numeric_limits<std::uint64_t>::max() - timestamp_ns
               ? std::numeric_limits<std::uint64_t>::max()
               : timestamp_ns + max_offset_ns;
  }

  void append(std::uint64_t timestamp_ns, const std::vector<lidar::Point> &points) {
    const std::uint64_t base_offset =
        timestamp_ns > scan_start_ns_ ? timestamp_ns - scan_start_ns_ : 0ULL;
    points_.reserve(points_.size() + points.size());
    for (const auto &point : points) {
      lidar::Point merged = point;
      const std::uint64_t offset =
          point.offset_time_ns > std::numeric_limits<std::uint64_t>::max() - base_offset
              ? std::numeric_limits<std::uint64_t>::max()
              : base_offset + point.offset_time_ns;
      merged.offset_time_ns = static_cast<std::uint32_t>(
          std::min<std::uint64_t>(offset, std::numeric_limits<std::uint32_t>::max()));
      points_.push_back(merged);
    }
  }

  std::pair<std::uint64_t, std::vector<lidar::Point>> take() {
    std::vector<lidar::Point> points;
    points.swap(points_);
    const std::uint64_t timestamp_ns = scan_start_ns_;
    has_scan_ = false;
    scan_start_ns_ = 0;
    return {timestamp_ns, std::move(points)};
  }

  std::uint64_t window_ns_{0};
  bool has_scan_{false};
  std::uint64_t scan_start_ns_{0};
  std::vector<lidar::Point> points_;
};

bool should_publish_imu(std::uint64_t timestamp_ns, std::uint64_t min_interval_ns,
                        std::uint64_t &last_timestamp_ns) {
  if (min_interval_ns == 0) {
    return true;
  }
  if (last_timestamp_ns != 0) {
    if (timestamp_ns <= last_timestamp_ns) {
      return false;
    }
    if (timestamp_ns - last_timestamp_ns < min_interval_ns) {
      return false;
    }
  }
  last_timestamp_ns = timestamp_ns;
  return true;
}

bool accepts(const Stream stream, const adapter::SensorRecordType type) {
  switch (stream) {
    case Stream::All:
      return true;
    case Stream::Lidar:
      return type == adapter::SensorRecordType::Cloud ||
             type == adapter::SensorRecordType::OdomPrior ||
             type == adapter::SensorRecordType::RegisteredCloud;
    case Stream::Imu:
      return type == adapter::SensorRecordType::Imu;
    case Stream::Camera:
      return type == adapter::SensorRecordType::Camera;
  }
  return false;
}

int run_records(const CliConfig &config, lidar::DdsModule *dds,
                SimLidarRawFramePublisher *sim_lidar, CameraDdsPublisher *camera_dds,
                std::istream &input,
                const bool require_stop_for_eof = false) {
  adapter::SensorRecordStats stats;
  std::optional<std::uint64_t> first_timestamp_ns;
  std::chrono::steady_clock::time_point replay_start;
  lidar::ReplayDeadlineRestamper<> replay_restamper;
  ScanAccumulator scans(config.scan_window_s);
  const std::uint64_t imu_min_interval_ns =
      config.imu_publish_hz > 0.0 ? static_cast<std::uint64_t>(1000000000.0 / config.imu_publish_hz)
                                  : 0ULL;
  std::uint64_t last_imu_publish_ns = 0;
  bool has_camera_intrinsics = false;
  std::unique_ptr<CameraDdsPublisher> all_camera_dds;

  auto finish = [&](int code) {
    if (dds != nullptr) {
      if (auto ready = scans.flush()) {
        dds->publish_cloud(0, ready->first, ready->second);
      }
    }
    std::fprintf(
        stderr,
        "sensor records: clouds=%llu imu=%llu odom=%llu registered_clouds=%llu camera=%llu "
        "bytes=%llu mode=%s\n",
        static_cast<unsigned long long>(stats.clouds), static_cast<unsigned long long>(stats.imu),
        static_cast<unsigned long long>(stats.odom_priors),
        static_cast<unsigned long long>(stats.registered_clouds),
        static_cast<unsigned long long>(stats.camera), static_cast<unsigned long long>(stats.bytes),
        config.validate_records ? "validate" : "replay");
    return code;
  };

  auto output_timestamp = [&](std::uint64_t source_timestamp_ns,
                              std::chrono::steady_clock::time_point target_deadline) {
    if (!config.restamp_stdin_records || config.navigation_fixture) {
      return source_timestamp_ns;
    }
    return replay_restamper.stamp_ns(source_timestamp_ns, target_deadline);
  };

  while (g_stop_requested == 0) {
    adapter::SensorRecord record;
    std::string error;
    const auto status = adapter::read_sensor_record(input, record, error);
    if (status == adapter::SensorRecordReadStatus::Eof) {
      if (require_stop_for_eof && g_stop_requested == 0) {
        std::fputs("sensor endpoint closed before stop was requested\n", stderr);
        return finish(2);
      }
      return finish(0);
    }
    if (status == adapter::SensorRecordReadStatus::Error) {
      if (g_stop_requested != 0) {
        return finish(0);
      }
      std::fprintf(stderr, "stdin record error: %s\n", error.c_str());
      return finish(2);
    }
    adapter::accumulate_sensor_record_stats(record, stats);
    if (!accepts(config.stream, record.header.type)) {
      std::fputs("sensor record does not belong to selected --stream\n", stderr);
      return finish(2);
    }
    std::optional<adapter::CameraRecord> camera;
    if (record.header.type == adapter::SensorRecordType::Camera) {
      camera = adapter::decode_camera_payload(record);
      namespace camera_record = lingtu::drivers::camera::record;
      const auto sequence_validation =
          camera_record::validateRecordSequence(camera->header, has_camera_intrinsics);
      if (sequence_validation != camera_record::RecordValidation::kValid) {
        std::fprintf(stderr, "stdin record error: %s\n",
                     camera_record::recordValidationReason(sequence_validation));
        return finish(2);
      }
      has_camera_intrinsics =
          has_camera_intrinsics || camera->header.kind == camera_record::kKindIntrinsics;
    }

    if (!first_timestamp_ns.has_value()) {
      first_timestamp_ns = record.header.timestamp_ns;
      replay_start = std::chrono::steady_clock::now();
    }
    const std::uint64_t relative_ns = record.header.timestamp_ns >= *first_timestamp_ns
                                          ? record.header.timestamp_ns - *first_timestamp_ns
                                          : 0U;
    const double timeline_rate = config.replay_rate > 0.0 ? config.replay_rate : 1.0;
    const long double scaled_relative_ns = static_cast<long double>(relative_ns) / timeline_rate;
    if (scaled_relative_ns > static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      std::fprintf(stderr, "stdin record replay timeline overflows steady clock\n");
      return finish(2);
    }
    const auto target_deadline =
        replay_start + std::chrono::nanoseconds(static_cast<std::int64_t>(scaled_relative_ns));
    if (config.replay_rate > 0.0 && !config.navigation_fixture && !config.validate_records) {
      std::this_thread::sleep_until(target_deadline);
    }

    if (config.validate_records) {
      continue;
    }
    const std::uint64_t timestamp_ns =
        output_timestamp(record.header.timestamp_ns, target_deadline);
    switch (record.header.type) {
      case adapter::SensorRecordType::Cloud: {
        const auto points = adapter::decode_point_payload(record);
        if (sim_lidar != nullptr) {
          sim_lidar->publish(0, timestamp_ns, points);
          break;
        }
        dds->publish_raw_packet(0, timestamp_ns, points);
        for (const auto &ready : scans.submit(timestamp_ns, points)) {
          dds->publish_cloud(0, ready.first, ready.second);
        }
        break;
      }
      case adapter::SensorRecordType::Imu: {
        if (sim_lidar != nullptr) {
          std::fprintf(stderr, "sim MID-360 publisher accepts cloud records only\n");
          return finish(2);
        }
        if (should_publish_imu(timestamp_ns, imu_min_interval_ns, last_imu_publish_ns)) {
          dds->publish_imu(timestamp_ns, adapter::decode_imu_payload(record));
        }
        break;
      }
      case adapter::SensorRecordType::OdomPrior:
        if (sim_lidar != nullptr) {
          std::fprintf(stderr, "sim MID-360 publisher accepts cloud records only\n");
          return finish(2);
        }
        dds->publish_odom_prior(timestamp_ns, adapter::decode_odom_prior_payload(record));
        break;
      case adapter::SensorRecordType::RegisteredCloud:
        if (sim_lidar != nullptr) {
          std::fprintf(stderr, "sim MID-360 publisher accepts cloud records only\n");
          return finish(2);
        }
        if (!config.navigation_fixture) {
          std::fprintf(stderr, "stdin registered cloud requires --navigation-fixture\n");
          return finish(2);
        }
        dds->publish_registered_cloud(timestamp_ns, adapter::decode_point_payload(record));
        break;
      case adapter::SensorRecordType::Camera:
        if (sim_lidar != nullptr || (dds == nullptr && camera_dds == nullptr)) {
          std::fprintf(stderr, "camera records require the canonical DDS sensor publisher\n");
          return finish(2);
        }
        if (camera_dds == nullptr && all_camera_dds == nullptr) {
          all_camera_dds =
              std::make_unique<CameraDdsPublisher>(config.domain_id, config.camera_frame);
        }
        (camera_dds != nullptr ? camera_dds : all_camera_dds.get())
            ->publish(timestamp_ns, std::move(*camera));
        break;
    }
  }
  return finish(0);
}

}  // namespace

int main(int argc, const char *const argv[]) {
  CliConfig config;
  try {
    config = parse_args(argc, argv);
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "%s\n", exc.what());
    return 2;
  }
  if (config.show_help) {
    std::fputs(kUsage, stdout);
    return 0;
  }

#ifdef _WIN32
  _setmode(_fileno(stdin), _O_BINARY);
#endif

  std::signal(SIGINT, signal_stop);
  std::signal(SIGTERM, signal_stop);
#ifdef _WIN32
  std::signal(SIGBREAK, signal_stop);
#endif

  try {
    std::optional<run_plan_process::RunPlanProcessEnvironment> process_environment;
    std::optional<EndpointContract> endpoint;
    if (config.local_endpoint_records) {
      endpoint = endpoint_contract(config.stream);
      process_environment = run_plan_process::loadRunPlanProcessEnvironment(
          run_plan_process::EndpointFiles{endpoint->readiness, endpoint->auth});
    }
    ReadyFile ready_file(config.ready_file);
    std::unique_ptr<lidar::DdsModule> dds;
    std::unique_ptr<SimLidarRawFramePublisher> sim_lidar;
    std::unique_ptr<CameraDdsPublisher> camera_dds;
    if (config.dds && config.sim_lidar_raw_frame) {
      sim_lidar = std::make_unique<SimLidarRawFramePublisher>(config.domain_id, config.lidar_frame);
    } else if (config.dds && config.stream != Stream::Camera) {
      const auto output = config.stream == Stream::Lidar
                              ? lidar::DdsOutput::Lidar
                              : config.stream == Stream::Imu ? lidar::DdsOutput::Imu
                                                              : lidar::DdsOutput::All;
      dds = std::make_unique<lidar::DdsModule>(config.domain_id, config.lidar_frame,
                                                config.imu_frame, config.navigation_fixture,
                                                output, !config.local_endpoint_records);
    } else if (config.dds) {
      camera_dds =
          std::make_unique<CameraDdsPublisher>(config.domain_id, config.camera_frame);
    }
    ready_file.mark_ready(config.sim_lidar_raw_frame
                              ? std::string(lingtu::message::kSimLidarRawFrame.dds_topic)
                              : std::string{});
    if (config.local_endpoint_records) {
      local_endpoint::LocalEndpointServer server(local_endpoint::ServerConfig{
          endpoint->role,
          kSensorEndpointProtocol,
          process_environment->product_session_id,
          process_environment->readiness_path,
          process_environment->auth_file_name,
      });
      server.start();
      std::optional<local_endpoint::ClientStream> stream;
      while (g_stop_requested == 0 && !stream.has_value()) {
        try {
          stream.emplace(server.acceptAuthenticated(std::chrono::milliseconds(250)));
        } catch (const local_endpoint::EndpointTimeout &) {
          continue;
        }
      }
      if (!stream.has_value()) {
        return 0;
      }
      EndpointStreamBuf buffer(std::move(*stream));
      std::istream input(&buffer);
      return run_records(config, dds.get(), sim_lidar.get(), camera_dds.get(), input, true);
    }
    return run_records(config, dds.get(), sim_lidar.get(), camera_dds.get(), std::cin);
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "sensor_publisher failed: %s\n", exc.what());
    return 1;
  }
}
