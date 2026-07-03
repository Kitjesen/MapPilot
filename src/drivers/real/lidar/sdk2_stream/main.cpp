#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
#include "dds/dds.h"
#include "lingtu_slam.h"
#include "message/cpp/dds_topics.hpp"
#endif

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

constexpr char kMagic[4] = {'L', 'T', 'U', '1'};
constexpr uint8_t kRecordCloud = 1;
constexpr uint8_t kRecordImu = 2;
constexpr uint8_t kLineNumberDefault = 1;
constexpr uint8_t kLineNumberMid360 = 4;
constexpr uint8_t kLineNumberHap = 6;
constexpr uint8_t kTimestampTypeNoSync = 0;
constexpr uint8_t kTimestampTypeGptpOrPtp = 1;
constexpr uint8_t kTimestampTypeGps = 2;

#pragma pack(push, 1)
struct RecordHeader {
  char magic[4];
  uint8_t record_type;
  uint8_t reserved[3];
  uint64_t timestamp_ns;
  uint32_t sequence;
  uint32_t count;
  uint32_t payload_bytes;
};

struct WirePoint {
  float x;
  float y;
  float z;
  float intensity;
  uint32_t offset_time_ns;
  uint8_t tag;
  uint8_t line;
  uint16_t flags;
};

struct WireImu {
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
};
#pragma pack(pop)

static_assert(sizeof(RecordHeader) == 28, "unexpected record header size");
static_assert(sizeof(WirePoint) == 24, "unexpected point record size");
static_assert(sizeof(WireImu) == 24, "unexpected imu record size");

std::mutex g_stdout_mutex;
std::condition_variable g_quit_cv;
std::mutex g_quit_mutex;
std::atomic<bool> g_quit{false};
std::atomic<uint32_t> g_sequence{0};
char g_work_mode_command[] = "work_mode";
char g_enable_imu_command[] = "enable_imu";

struct CliConfig {
  bool dds = false;
  int domain_id = 0;
  double scan_window_s = 0.1;
  std::string lidar_frame = "livox_frame";
  std::string imu_frame = "imu_link";
  std::string config_path;
};

uint64_t now_ns() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

uint64_t read_le_u64(const uint8_t bytes[8]) {
  uint64_t out = 0;
  for (int i = 7; i >= 0; --i) {
    out <<= 8;
    out |= static_cast<uint64_t>(bytes[i]);
  }
  return out;
}

uint64_t packet_timestamp_ns(const LivoxLidarEthernetPacket* data) {
  if (data == nullptr) {
    return now_ns();
  }
  if (data->time_type == kTimestampTypeGptpOrPtp ||
      data->time_type == kTimestampTypeGps) {
    return read_le_u64(data->timestamp);
  }
  return now_ns();
}

uint8_t line_count_for_device(uint8_t dev_type) {
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP ||
      dev_type == LivoxLidarDeviceType::kLivoxLidarTypeHAP) {
    return kLineNumberHap;
  }
  if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) {
    return kLineNumberMid360;
  }
  return kLineNumberDefault;
}

bool write_record(uint8_t record_type,
                  uint64_t timestamp_ns,
                  uint32_t count,
                  const void* payload,
                  uint32_t payload_bytes) {
  RecordHeader header{};
  std::memcpy(header.magic, kMagic, sizeof(header.magic));
  header.record_type = record_type;
  header.timestamp_ns = timestamp_ns;
  header.sequence = g_sequence.fetch_add(1, std::memory_order_relaxed);
  header.count = count;
  header.payload_bytes = payload_bytes;

  std::lock_guard<std::mutex> lock(g_stdout_mutex);
  if (std::fwrite(&header, sizeof(header), 1, stdout) != 1) {
    return false;
  }
  if (payload_bytes > 0 &&
      std::fwrite(payload, payload_bytes, 1, stdout) != 1) {
    return false;
  }
  std::fflush(stdout);
  return true;
}

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
class DdsPublisher {
 public:
  DdsPublisher(int domain_id, std::string lidar_frame, std::string imu_frame)
      : lidar_frame_(std::move(lidar_frame)),
        imu_frame_(std::move(imu_frame)) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher");
    lidar_topic_ = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_LivoxFrame_desc,
            lingtu::message::kLidarRawFrame.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(lidar)");
    raw_packet_topic_ = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_LivoxFrame_desc,
            lingtu::message::kLidarRawPacket.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(raw_packet)");
    imu_topic_ = checked(
        dds_create_topic(
            participant_,
            &lingtu_dds_Imu_desc,
            lingtu::message::kImuRaw.dds_topic.data(),
            nullptr,
            nullptr),
        "dds_create_topic(imu)");
    auto sensor_qos = sensor_qos_profile();
    lidar_writer_ = checked(
        dds_create_writer(publisher_, lidar_topic_, sensor_qos.get(), nullptr),
        "dds_create_writer(lidar)");
    raw_packet_writer_ = checked(
        dds_create_writer(publisher_, raw_packet_topic_, sensor_qos.get(), nullptr),
        "dds_create_writer(raw_packet)");
    imu_writer_ = checked(
        dds_create_writer(publisher_, imu_topic_, sensor_qos.get(), nullptr),
        "dds_create_writer(imu)");
  }

  ~DdsPublisher() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  DdsPublisher(const DdsPublisher&) = delete;
  DdsPublisher& operator=(const DdsPublisher&) = delete;

  void publish_cloud(uint8_t lidar_id,
                     uint64_t timestamp_ns,
                     const std::vector<WirePoint>& points) {
    publish_cloud_to(lidar_writer_, lidar_id, timestamp_ns, points);
  }

  void publish_raw_packet(uint8_t lidar_id,
                          uint64_t timestamp_ns,
                          const std::vector<WirePoint>& points) {
    publish_cloud_to(raw_packet_writer_, lidar_id, timestamp_ns, points);
  }

  void publish_imu(uint64_t timestamp_ns, const WireImu& imu) {
    std::lock_guard<std::mutex> lock(write_mutex_);
    lingtu_dds_Imu msg{};
    fill_header(msg.header, timestamp_ns, imu_frame_);
    msg.orientation.w = 1.0;
    msg.angular_velocity.x = imu.gyro_x;
    msg.angular_velocity.y = imu.gyro_y;
    msg.angular_velocity.z = imu.gyro_z;
    msg.linear_acceleration.x = imu.acc_x;
    msg.linear_acceleration.y = imu.acc_y;
    msg.linear_acceleration.z = imu.acc_z;
    checked(dds_write(imu_writer_, &msg), "dds_write(imu)");
  }

 private:
  void publish_cloud_to(dds_entity_t writer,
                        uint8_t lidar_id,
                        uint64_t timestamp_ns,
                        const std::vector<WirePoint>& points) {
    std::vector<lingtu_dds_LivoxPoint> dds_points;
    dds_points.reserve(points.size());
    for (const auto& point : points) {
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
    std::lock_guard<std::mutex> lock(write_mutex_);
    lingtu_dds_LivoxFrame msg{};
    fill_header(msg.header, timestamp_ns, lidar_frame_);
    msg.timebase = timestamp_ns;
    msg.point_num = static_cast<std::uint32_t>(dds_points.size());
    msg.lidar_id = lidar_id;
    msg.points._maximum = msg.point_num;
    msg.points._length = msg.point_num;
    msg.points._buffer = dds_points.data();
    msg.points._release = false;
    checked(dds_write(writer, &msg), "dds_write(livox)");
  }

  static dds_entity_t checked(dds_return_t value, const char* what) {
    if (value < 0) {
      throw std::runtime_error(
          std::string(what) + ": " + dds_strretcode(-value));
    }
    return static_cast<dds_entity_t>(value);
  }

  static void fill_header(
      lingtu_dds_Header& header,
      uint64_t timestamp_ns,
      const std::string& frame_id) {
    header.stamp.sec = static_cast<std::int32_t>(timestamp_ns / 1000000000ULL);
    header.stamp.nanosec = static_cast<std::uint32_t>(timestamp_ns % 1000000000ULL);
    header.frame_id = const_cast<char*>(frame_id.c_str());
  }

  static std::unique_ptr<dds_qos_t, decltype(&dds_delete_qos)> sensor_qos_profile() {
    std::unique_ptr<dds_qos_t, decltype(&dds_delete_qos)> qos(
        dds_create_qos(), dds_delete_qos);
    if (!qos) {
      throw std::runtime_error("dds_create_qos failed");
    }
    dds_qset_reliability(qos.get(), DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
    dds_qset_history(qos.get(), DDS_HISTORY_KEEP_LAST, 256);
    return qos;
  }

  std::string lidar_frame_;
  std::string imu_frame_;
  dds_entity_t participant_ = DDS_RETCODE_ERROR;
  dds_entity_t publisher_ = DDS_RETCODE_ERROR;
  dds_entity_t lidar_topic_ = DDS_RETCODE_ERROR;
  dds_entity_t raw_packet_topic_ = DDS_RETCODE_ERROR;
  dds_entity_t imu_topic_ = DDS_RETCODE_ERROR;
  dds_entity_t lidar_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t raw_packet_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t imu_writer_ = DDS_RETCODE_ERROR;
  std::mutex write_mutex_;
};

std::unique_ptr<DdsPublisher> g_dds_publisher;
#endif

bool emit_cloud(uint8_t lidar_id,
                uint64_t timestamp_ns,
                const std::vector<WirePoint>& points) {
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (g_dds_publisher) {
    g_dds_publisher->publish_cloud(lidar_id, timestamp_ns, points);
    return true;
  }
#else
  (void)lidar_id;
#endif
  return write_record(
      kRecordCloud,
      timestamp_ns,
      static_cast<std::uint32_t>(points.size()),
      points.data(),
      static_cast<std::uint32_t>(points.size() * sizeof(WirePoint)));
}

void emit_raw_packet(uint8_t lidar_id,
                     uint64_t timestamp_ns,
                     const std::vector<WirePoint>& points) {
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (g_dds_publisher) {
    g_dds_publisher->publish_raw_packet(lidar_id, timestamp_ns, points);
  }
#else
  (void)lidar_id;
  (void)timestamp_ns;
  (void)points;
#endif
}

bool emit_imu(uint64_t timestamp_ns, const WireImu& imu) {
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (g_dds_publisher) {
    g_dds_publisher->publish_imu(timestamp_ns, imu);
    return true;
  }
#endif
  return write_record(kRecordImu, timestamp_ns, 1, &imu, sizeof(imu));
}

struct CloudBatch {
  uint8_t lidar_id = 0;
  uint64_t timestamp_ns = 0;
  std::vector<WirePoint> points;
};

class ScanAccumulator {
 public:
  explicit ScanAccumulator(double window_s)
      : window_ns_(static_cast<uint64_t>(
            std::max(0.0, window_s) * 1000000000.0)) {}

  std::vector<CloudBatch> submit(uint8_t lidar_id,
                                 uint64_t timestamp_ns,
                                 const std::vector<WirePoint>& points) {
    std::vector<CloudBatch> ready;
    if (points.empty()) {
      return ready;
    }
    const uint64_t packet_end_ns = packet_end_time_ns(timestamp_ns, points);

    std::lock_guard<std::mutex> lock(mutex_);
    if (has_scan_ && timestamp_ns < scan_start_ns_) {
      ready.push_back(take_locked());
    }
    if (has_scan_ && window_ns_ > 0 &&
        packet_end_ns > scan_start_ns_ &&
        packet_end_ns - scan_start_ns_ > window_ns_ &&
        !points_.empty()) {
      ready.push_back(take_locked());
    }
    if (!has_scan_) {
      has_scan_ = true;
      scan_start_ns_ = timestamp_ns;
      lidar_id_ = lidar_id;
    }
    append_locked(timestamp_ns, points);
    if (window_ns_ == 0) {
      ready.push_back(take_locked());
    }
    return ready;
  }

  std::optional<CloudBatch> flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_scan_ || points_.empty()) {
      return std::nullopt;
    }
    return take_locked();
  }

 private:
  static uint64_t packet_end_time_ns(
      uint64_t timestamp_ns,
      const std::vector<WirePoint>& points) {
    uint64_t max_offset_ns = 0;
    for (const auto& point : points) {
      max_offset_ns = std::max<uint64_t>(max_offset_ns, point.offset_time_ns);
    }
    return timestamp_ns + max_offset_ns;
  }

  void append_locked(uint64_t timestamp_ns, const std::vector<WirePoint>& points) {
    const uint64_t base_offset =
        timestamp_ns > scan_start_ns_ ? timestamp_ns - scan_start_ns_ : 0ULL;
    points_.reserve(points_.size() + points.size());
    for (const auto& point : points) {
      WirePoint merged = point;
      const uint64_t offset = base_offset + point.offset_time_ns;
      merged.offset_time_ns = static_cast<uint32_t>(
          std::min<uint64_t>(offset, std::numeric_limits<uint32_t>::max()));
      points_.push_back(merged);
    }
  }

  CloudBatch take_locked() {
    CloudBatch batch;
    batch.lidar_id = lidar_id_;
    batch.timestamp_ns = scan_start_ns_;
    batch.points.swap(points_);
    has_scan_ = false;
    scan_start_ns_ = 0;
    lidar_id_ = 0;
    return batch;
  }

  const uint64_t window_ns_;
  std::mutex mutex_;
  bool has_scan_ = false;
  uint64_t scan_start_ns_ = 0;
  uint8_t lidar_id_ = 0;
  std::vector<WirePoint> points_;
};

std::unique_ptr<ScanAccumulator> g_scan_accumulator;

void emit_lidar_packet(uint8_t lidar_id,
                       uint64_t timestamp_ns,
                       const std::vector<WirePoint>& points) {
  emit_raw_packet(lidar_id, timestamp_ns, points);
  if (!g_scan_accumulator) {
    emit_cloud(lidar_id, timestamp_ns, points);
    return;
  }
  for (const auto& batch : g_scan_accumulator->submit(lidar_id, timestamp_ns, points)) {
    emit_cloud(batch.lidar_id, batch.timestamp_ns, batch.points);
  }
}

void handle_high_points(uint8_t dev_type,
                        const LivoxLidarEthernetPacket* data,
                        uint64_t base_time_ns) {
  const auto* raw =
      reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
  const uint32_t point_count = static_cast<uint32_t>(data->dot_num);
  if (point_count == 0) {
    return;
  }
  const uint8_t line_count = line_count_for_device(dev_type);
  const uint32_t interval_ns =
      point_count > 0 ? static_cast<uint32_t>(data->time_interval) * 100U / point_count : 0U;
  std::vector<WirePoint> points(point_count);
  for (uint32_t i = 0; i < point_count; ++i) {
    WirePoint point{};
    point.x = static_cast<float>(raw[i].x) / 1000.0f;
    point.y = static_cast<float>(raw[i].y) / 1000.0f;
    point.z = static_cast<float>(raw[i].z) / 1000.0f;
    point.intensity = static_cast<float>(raw[i].reflectivity);
    point.offset_time_ns = i * interval_ns;
    point.tag = raw[i].tag;
    point.line = static_cast<uint8_t>(i % line_count);
    points[i] = point;
  }
  emit_lidar_packet(dev_type, base_time_ns, points);
}

void handle_low_points(uint8_t dev_type,
                       const LivoxLidarEthernetPacket* data,
                       uint64_t base_time_ns) {
  const auto* raw =
      reinterpret_cast<const LivoxLidarCartesianLowRawPoint*>(data->data);
  const uint32_t point_count = static_cast<uint32_t>(data->dot_num);
  if (point_count == 0) {
    return;
  }
  const uint8_t line_count = line_count_for_device(dev_type);
  const uint32_t interval_ns =
      point_count > 0 ? static_cast<uint32_t>(data->time_interval) * 100U / point_count : 0U;
  std::vector<WirePoint> points(point_count);
  for (uint32_t i = 0; i < point_count; ++i) {
    WirePoint point{};
    point.x = static_cast<float>(raw[i].x) / 100.0f;
    point.y = static_cast<float>(raw[i].y) / 100.0f;
    point.z = static_cast<float>(raw[i].z) / 100.0f;
    point.intensity = static_cast<float>(raw[i].reflectivity);
    point.offset_time_ns = i * interval_ns;
    point.tag = raw[i].tag;
    point.line = static_cast<uint8_t>(i % line_count);
    points[i] = point;
  }
  emit_lidar_packet(dev_type, base_time_ns, points);
}

void handle_spherical_points(uint8_t dev_type,
                             const LivoxLidarEthernetPacket* data,
                             uint64_t base_time_ns) {
  const auto* raw = reinterpret_cast<const LivoxLidarSpherPoint*>(data->data);
  const uint32_t point_count = static_cast<uint32_t>(data->dot_num);
  if (point_count == 0) {
    return;
  }
  const uint8_t line_count = line_count_for_device(dev_type);
  const uint32_t interval_ns =
      point_count > 0 ? static_cast<uint32_t>(data->time_interval) * 100U / point_count : 0U;
  std::vector<WirePoint> points(point_count);
  constexpr float kPi = 3.14159265358979323846f;
  for (uint32_t i = 0; i < point_count; ++i) {
    const float radius = static_cast<float>(raw[i].depth) / 1000.0f;
    const float theta = static_cast<float>(raw[i].theta) / 100.0f / 180.0f * kPi;
    const float phi = static_cast<float>(raw[i].phi) / 100.0f / 180.0f * kPi;
    WirePoint point{};
    point.x = radius * std::sin(theta) * std::cos(phi);
    point.y = radius * std::sin(theta) * std::sin(phi);
    point.z = radius * std::cos(theta);
    point.intensity = static_cast<float>(raw[i].reflectivity);
    point.offset_time_ns = i * interval_ns;
    point.tag = raw[i].tag;
    point.line = static_cast<uint8_t>(i % line_count);
    points[i] = point;
  }
  emit_lidar_packet(dev_type, base_time_ns, points);
}

void point_cloud_callback(uint32_t /*handle*/,
                          const uint8_t dev_type,
                          LivoxLidarEthernetPacket* data,
                          void* /*client_data*/) {
  if (data == nullptr) {
    return;
  }
  const uint64_t base_time_ns = packet_timestamp_ns(data);
  switch (data->data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      handle_high_points(dev_type, data, base_time_ns);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      handle_low_points(dev_type, data, base_time_ns);
      break;
    case kLivoxLidarSphericalCoordinateData:
      handle_spherical_points(dev_type, data, base_time_ns);
      break;
    default:
      break;
  }
}

void imu_callback(uint32_t /*handle*/,
                  const uint8_t /*dev_type*/,
                  LivoxLidarEthernetPacket* data,
                  void* /*client_data*/) {
  if (data == nullptr || data->data_type != kLivoxLidarImuData) {
    return;
  }
  const auto* raw = reinterpret_cast<const LivoxLidarImuRawPoint*>(data->data);
  WireImu imu{};
  imu.gyro_x = raw->gyro_x;
  imu.gyro_y = raw->gyro_y;
  imu.gyro_z = raw->gyro_z;
  imu.acc_x = raw->acc_x;
  imu.acc_y = raw->acc_y;
  imu.acc_z = raw->acc_z;
  emit_imu(packet_timestamp_ns(data), imu);
}

void command_callback(livox_status status,
                      uint32_t handle,
                      LivoxLidarAsyncControlResponse* response,
                      void* client_data) {
  const uint8_t ret_code = response ? response->ret_code : 255;
  const char* command = static_cast<const char*>(client_data);
  std::fprintf(stderr,
               "livox_sdk2_stream %s status=%u handle=%u ret=%u\n",
               command ? command : "command",
               static_cast<unsigned>(status),
               handle,
               static_cast<unsigned>(ret_code));
}

void info_change_callback(const uint32_t handle,
                          const LivoxLidarInfo* info,
                          void* /*client_data*/) {
  if (info == nullptr) {
    return;
  }
  std::fprintf(stderr,
               "livox_sdk2_stream lidar handle=%u sn=%s ip=%s type=%u\n",
               handle,
               info->sn,
               info->lidar_ip,
               static_cast<unsigned>(info->dev_type));
  SetLivoxLidarWorkMode(
      handle, kLivoxLidarNormal, command_callback, g_work_mode_command);
  EnableLivoxLidarImuData(handle, command_callback, g_enable_imu_command);
}

void signal_stop(int /*signal*/) {
  g_quit.store(true, std::memory_order_relaxed);
  g_quit_cv.notify_all();
}

CliConfig parse_args(int argc, const char* argv[]) {
  CliConfig config;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--dds") {
      config.dds = true;
    } else if (arg == "--domain-id") {
      config.domain_id = std::stoi(next());
    } else if (arg == "--scan-window") {
      config.scan_window_s = std::max(0.0, std::stod(next()));
    } else if (arg == "--publish-freq") {
      const double hz = std::stod(next());
      if (hz <= 0.0) {
        throw std::runtime_error("--publish-freq must be positive");
      }
      config.scan_window_s = 1.0 / hz;
    } else if (arg == "--lidar-frame") {
      config.lidar_frame = next();
    } else if (arg == "--imu-frame") {
      config.imu_frame = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: livox_sdk2_stream [--dds] [--domain-id N] "
          "[--publish-freq HZ|--scan-window SEC] "
          "[--lidar-frame FRAME] [--imu-frame FRAME] <MID360_config.json>");
    } else if (config.config_path.empty()) {
      config.config_path = arg;
    } else {
      throw std::runtime_error("unexpected argument: " + arg);
    }
  }
  if (config.config_path.empty()) {
    throw std::runtime_error(
        "usage: livox_sdk2_stream [--dds] [--domain-id N] "
        "[--publish-freq HZ|--scan-window SEC] "
        "[--lidar-frame FRAME] [--imu-frame FRAME] <MID360_config.json>");
  }
  return config;
}

}  // namespace

int main(int argc, const char* argv[]) {
  CliConfig cli;
  try {
    cli = parse_args(argc, argv);
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "%s\n", exc.what());
    return 2;
  }

#ifdef _WIN32
  _setmode(_fileno(stdout), _O_BINARY);
#endif

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (cli.dds) {
    try {
      g_dds_publisher = std::make_unique<DdsPublisher>(
          cli.domain_id, cli.lidar_frame, cli.imu_frame);
    } catch (const std::exception& exc) {
      std::fprintf(stderr, "livox_sdk2_stream DDS init failed: %s\n", exc.what());
      return 1;
    }
  }
#else
  if (cli.dds) {
    std::fprintf(stderr, "livox_sdk2_stream was built without DDS support\n");
    return 2;
  }
#endif
  g_scan_accumulator = std::make_unique<ScanAccumulator>(cli.scan_window_s);

  std::signal(SIGINT, signal_stop);
  std::signal(SIGTERM, signal_stop);

  DisableLivoxSdkConsoleLogger();
  if (!LivoxLidarSdkInit(cli.config_path.c_str())) {
    std::fprintf(stderr, "livox_sdk2_stream sdk init failed: %s\n", cli.config_path.c_str());
    LivoxLidarSdkUninit();
    return 1;
  }

  SetLivoxLidarPointCloudCallBack(point_cloud_callback, nullptr);
  SetLivoxLidarImuDataCallback(imu_callback, nullptr);
  SetLivoxLidarInfoChangeCallback(info_change_callback, nullptr);

  if (!LivoxLidarSdkStart()) {
    std::fprintf(stderr, "livox_sdk2_stream sdk start failed\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  std::fprintf(stderr, "livox_sdk2_stream started\n");
  {
    std::unique_lock<std::mutex> lock(g_quit_mutex);
    g_quit_cv.wait(lock, [] { return g_quit.load(std::memory_order_relaxed); });
  }
  if (g_scan_accumulator) {
    if (auto batch = g_scan_accumulator->flush()) {
      emit_cloud(batch->lidar_id, batch->timestamp_ns, batch->points);
    }
  }

  LivoxLidarSdkUninit();
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  g_dds_publisher.reset();
#endif
  std::fprintf(stderr, "livox_sdk2_stream stopped\n");
  return 0;
}
