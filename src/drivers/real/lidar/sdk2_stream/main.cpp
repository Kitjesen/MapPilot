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
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
#include "dds/dds.hpp"
#include "lingtu_slam.hpp"
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

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
namespace lt = lingtu::dds;
#endif

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
lt::Header make_dds_header(uint64_t timestamp_ns, const std::string& frame_id) {
  const auto sec = static_cast<std::int32_t>(timestamp_ns / 1000000000ULL);
  const auto nsec = static_cast<std::uint32_t>(timestamp_ns % 1000000000ULL);
  return lt::Header(lt::Time(sec, nsec), frame_id);
}

class DdsPublisher {
 public:
  DdsPublisher(int domain_id, std::string lidar_frame, std::string imu_frame)
      : lidar_frame_(std::move(lidar_frame)),
        imu_frame_(std::move(imu_frame)),
        participant_(domain_id),
        publisher_(participant_),
        lidar_topic_(participant_, std::string(lingtu::message::kLidarRawFrame.dds_topic)),
        imu_topic_(participant_, std::string(lingtu::message::kImuRaw.dds_topic)),
        lidar_writer_(publisher_, lidar_topic_),
        imu_writer_(publisher_, imu_topic_) {}

  void publish_cloud(uint8_t lidar_id,
                     uint64_t timestamp_ns,
                     const std::vector<WirePoint>& points) {
    std::vector<lt::LivoxPoint> dds_points;
    dds_points.reserve(points.size());
    for (const auto& point : points) {
      dds_points.emplace_back(
          point.offset_time_ns,
          point.x,
          point.y,
          point.z,
          static_cast<std::uint8_t>(point.intensity),
          point.tag,
          point.line);
    }
    std::array<std::uint8_t, 3> reserved{};
    std::lock_guard<std::mutex> lock(write_mutex_);
    lidar_writer_.write(lt::LivoxFrame(
        make_dds_header(timestamp_ns, lidar_frame_),
        timestamp_ns,
        static_cast<std::uint32_t>(dds_points.size()),
        lidar_id,
        reserved,
        dds_points));
  }

  void publish_imu(uint64_t timestamp_ns, const WireImu& imu) {
    std::array<double, 9> covariance{};
    covariance.fill(0.0);
    std::lock_guard<std::mutex> lock(write_mutex_);
    imu_writer_.write(lt::Imu(
        make_dds_header(timestamp_ns, imu_frame_),
        lt::Quaternion(0.0, 0.0, 0.0, 1.0),
        covariance,
        lt::Vector3(imu.gyro_x, imu.gyro_y, imu.gyro_z),
        covariance,
        lt::Vector3(imu.acc_x, imu.acc_y, imu.acc_z),
        covariance));
  }

 private:
  std::string lidar_frame_;
  std::string imu_frame_;
  dds::domain::DomainParticipant participant_;
  dds::pub::Publisher publisher_;
  dds::topic::Topic<lt::LivoxFrame> lidar_topic_;
  dds::topic::Topic<lt::Imu> imu_topic_;
  dds::pub::DataWriter<lt::LivoxFrame> lidar_writer_;
  dds::pub::DataWriter<lt::Imu> imu_writer_;
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

bool emit_imu(uint64_t timestamp_ns, const WireImu& imu) {
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (g_dds_publisher) {
    g_dds_publisher->publish_imu(timestamp_ns, imu);
    return true;
  }
#endif
  return write_record(kRecordImu, timestamp_ns, 1, &imu, sizeof(imu));
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
  emit_cloud(dev_type, base_time_ns, points);
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
  emit_cloud(dev_type, base_time_ns, points);
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
  emit_cloud(dev_type, base_time_ns, points);
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
    } else if (arg == "--lidar-frame") {
      config.lidar_frame = next();
    } else if (arg == "--imu-frame") {
      config.imu_frame = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: livox_sdk2_stream [--dds] [--domain-id N] "
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

  LivoxLidarSdkUninit();
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  g_dds_publisher.reset();
#endif
  std::fprintf(stderr, "livox_sdk2_stream stopped\n");
  return 0;
}
