#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
#include "fallback_timestamp_clock.hpp"
#include "native/module.hpp"
#include "packet_timestamp_clock.hpp"
#include "replay_deadline_restamper.hpp"
#include "single_lidar_handle_guard.hpp"
#include "stream_exit_state.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <cmath>
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
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
#include "native/dds_module.hpp"
#endif

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

namespace lidar = lingtu::drivers::lidar;
using lidar::CliConfig;
using lidar::ImuSample;
using lidar::OdomPrior;
using lidar::Point;
using WireImu = ImuSample;
using WireOdomPrior = OdomPrior;
using WirePoint = Point;

constexpr char kMagic[4] = {'L', 'T', 'U', '1'};
constexpr uint8_t kRecordCloud = 1;
constexpr uint8_t kRecordImu = 2;
constexpr uint8_t kRecordOdomPrior = 3;
constexpr uint8_t kRecordRegisteredCloud = 4;
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
#pragma pack(pop)

static_assert(sizeof(RecordHeader) == 28, "unexpected record header size");

std::mutex g_stdout_mutex;
std::atomic<uint32_t> g_sequence{0};
std::atomic<uint64_t> g_imu_min_interval_ns{0};
std::atomic<uint64_t> g_last_imu_emit_ns{0};
lidar::FallbackTimestampClock<> g_fallback_timestamp_clock;
lidar::PacketTimestampClock g_packet_timestamp_clock;
lidar::SingleLidarHandleGuard g_lidar_handle_guard;
lidar::StreamExitState g_exit_state;
char g_work_mode_command[] = "work_mode";
char g_enable_imu_command[] = "enable_imu";

uint64_t read_le_u64(const uint8_t bytes[8]) {
  uint64_t out = 0;
  for (int i = 7; i >= 0; --i) {
    out <<= 8;
    out |= static_cast<uint64_t>(bytes[i]);
  }
  return out;
}

void request_timestamp_fault(std::string_view reason) {
  const bool first = g_exit_state.request_timestamp_fault();
  if (first) {
    std::fprintf(
        stderr,
        "livox_sdk2_stream fatal timestamp fault: %.*s\n",
        static_cast<int>(reason.size()),
        reason.data());
  }
}

bool accept_lidar_handle(uint32_t handle) {
  if (g_lidar_handle_guard.accept(handle)) {
    return true;
  }
  request_timestamp_fault("multiple_lidar_devices_unsupported");
  return false;
}

std::optional<uint64_t> packet_timestamp_ns(
    lidar::PacketTimestampStream stream,
    const LivoxLidarEthernetPacket* data) {
  const uint64_t fallback_ns = g_fallback_timestamp_clock.now_ns();
  lidar::PacketTimestampSource source = lidar::PacketTimestampSource::Invalid;
  uint64_t device_ns = 0U;
  if (data == nullptr || data->time_type == kTimestampTypeNoSync) {
    source = lidar::PacketTimestampSource::Fallback;
  } else if (data->time_type == kTimestampTypeGptpOrPtp) {
    source = lidar::PacketTimestampSource::Ptp;
    device_ns = read_le_u64(data->timestamp);
  } else if (data->time_type == kTimestampTypeGps) {
    source = lidar::PacketTimestampSource::Gps;
    device_ns = read_le_u64(data->timestamp);
  }

  const auto mapped =
      g_packet_timestamp_clock.map(stream, source, device_ns, fallback_ns);
  if (mapped.publish()) {
    return mapped.stamp_ns;
  }
  if (mapped.action == lidar::PacketTimestampAction::Fatal) {
    request_timestamp_fault(mapped.reason);
  }
  return std::nullopt;
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

bool read_exact(void* dst, std::size_t bytes) {
  auto* out = static_cast<std::uint8_t*>(dst);
  std::size_t total = 0;
  while (total < bytes) {
    const std::size_t got = std::fread(out + total, 1, bytes - total, stdin);
    if (got == 0) {
      return false;
    }
    total += got;
  }
  return true;
}

bool should_emit_imu(uint64_t timestamp_ns) {
  const uint64_t min_interval = g_imu_min_interval_ns.load(std::memory_order_relaxed);
  if (min_interval == 0) {
    return true;
  }
  uint64_t previous = g_last_imu_emit_ns.load(std::memory_order_relaxed);
  while (true) {
    if (previous != 0) {
      if (timestamp_ns <= previous) {
        return false;
      }
      if (timestamp_ns - previous < min_interval) {
        return false;
      }
    }
    if (g_last_imu_emit_ns.compare_exchange_weak(
            previous, timestamp_ns, std::memory_order_relaxed)) {
      return true;
    }
  }
}

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
std::unique_ptr<lidar::DdsModule> g_dds_publisher;
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

bool emit_odom_prior(uint64_t timestamp_ns, const WireOdomPrior& prior) {
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (g_dds_publisher) {
    g_dds_publisher->publish_odom_prior(timestamp_ns, prior);
    return true;
  }
#endif
  return write_record(kRecordOdomPrior, timestamp_ns, 1, &prior, sizeof(prior));
}

bool emit_registered_cloud(
    uint64_t timestamp_ns,
    const std::vector<WirePoint>& points) {
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (g_dds_publisher) {
    g_dds_publisher->publish_registered_cloud(timestamp_ns, points);
    return true;
  }
#else
  (void)timestamp_ns;
  (void)points;
#endif
  return false;
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
    return max_offset_ns > std::numeric_limits<uint64_t>::max() - timestamp_ns
        ? std::numeric_limits<uint64_t>::max()
        : timestamp_ns + max_offset_ns;
  }

  void append_locked(uint64_t timestamp_ns, const std::vector<WirePoint>& points) {
    const uint64_t base_offset =
        timestamp_ns > scan_start_ns_ ? timestamp_ns - scan_start_ns_ : 0ULL;
    points_.reserve(points_.size() + points.size());
    for (const auto& point : points) {
      WirePoint merged = point;
      const uint64_t offset =
          point.offset_time_ns >
                  std::numeric_limits<uint64_t>::max() - base_offset
              ? std::numeric_limits<uint64_t>::max()
              : base_offset + point.offset_time_ns;
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

int run_stdin_records(
    double replay_rate,
    bool validate_only,
    bool restamp_records,
    bool navigation_fixture) {
  constexpr std::uint32_t kMaxReplayPayloadBytes = 256U * 1024U * 1024U;
  std::optional<std::uint64_t> first_timestamp_ns;
  lidar::ReplayDeadlineRestamper<> replay_restamper;
  std::chrono::steady_clock::time_point replay_start;
  std::uint64_t cloud_records = 0U;
  std::uint64_t imu_records = 0U;
  std::uint64_t odom_records = 0U;
  std::uint64_t registered_cloud_records = 0U;
  auto finish = [&](int code) {
    std::fprintf(
        stderr,
        "stdin records: clouds=%llu imu=%llu odom=%llu registered_clouds=%llu mode=%s\n",
        static_cast<unsigned long long>(cloud_records),
        static_cast<unsigned long long>(imu_records),
        static_cast<unsigned long long>(odom_records),
        static_cast<unsigned long long>(registered_cloud_records),
        validate_only ? "validate" : "replay");
    return code;
  };
  auto output_timestamp = [&](std::uint64_t source_timestamp_ns,
                              std::chrono::steady_clock::time_point target_deadline) {
    if (!restamp_records) {
      return source_timestamp_ns;
    }
    if (navigation_fixture) {
      // Navigation fixtures already provide one monotonic simulated-hardware
      // clock for IMU, odometry, TF and clouds. Preserve it so interpolation
      // and source-order checks cannot inherit WSL CLOCK_REALTIME steps.
      // Receiver freshness is measured independently from DDS receipt time.
      return source_timestamp_ns;
    }
    // Re-evaluate the realtime/steady offset for every source timestamp. WSL
    // can slew or step CLOCK_REALTIME while the replay is running; retaining a
    // single startup offset makes correctly paced samples appear to come from
    // the future. Steady-clock lateness is subtracted so a genuinely queued or
    // delayed sample remains stale and the navigation fail-safe still closes.
    return replay_restamper.stamp_ns(source_timestamp_ns, target_deadline);
  };
  while (!g_exit_state.quit_requested()) {
    RecordHeader header{};
    if (!read_exact(&header, sizeof(header))) {
      return finish(std::feof(stdin) ? 0 : 1);
    }
    if (std::memcmp(header.magic, kMagic, sizeof(header.magic)) != 0) {
      std::fprintf(stderr, "stdin record has bad magic\n");
      return finish(2);
    }
    if (header.payload_bytes > kMaxReplayPayloadBytes) {
      std::fprintf(stderr, "stdin record payload exceeds safety limit\n");
      return finish(2);
    }
    if (!first_timestamp_ns.has_value()) {
      first_timestamp_ns = header.timestamp_ns;
      replay_start = std::chrono::steady_clock::now();
    }
    const std::uint64_t relative_ns = header.timestamp_ns >= *first_timestamp_ns
        ? header.timestamp_ns - *first_timestamp_ns
        : 0U;
    const double timeline_rate = replay_rate > 0.0 ? replay_rate : 1.0;
    const long double scaled_relative_ns =
        static_cast<long double>(relative_ns) / timeline_rate;
    if (scaled_relative_ns >
        static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
      std::fprintf(stderr, "stdin record replay timeline overflows steady clock\n");
      return finish(2);
    }
    const auto target_offset = std::chrono::nanoseconds(
        static_cast<std::int64_t>(scaled_relative_ns));
    const auto target_deadline = replay_start + target_offset;
    if (replay_rate > 0.0 && !navigation_fixture) {
      std::this_thread::sleep_until(target_deadline);
    }
    if (header.record_type == kRecordCloud) {
      if (header.payload_bytes != header.count * sizeof(WirePoint)) {
        std::fprintf(stderr, "stdin cloud payload size mismatch\n");
        return finish(2);
      }
      std::vector<WirePoint> points(header.count);
      if (!read_exact(points.data(), header.payload_bytes)) {
        std::fprintf(stderr, "stdin cloud payload truncated\n");
        return finish(1);
      }
      ++cloud_records;
      const std::uint64_t output_timestamp_ns =
          output_timestamp(header.timestamp_ns, target_deadline);
      if (!validate_only && !emit_cloud(0, output_timestamp_ns, points)) {
        return finish(1);
      }
    } else if (header.record_type == kRecordImu) {
      if (header.count != 1 || header.payload_bytes != sizeof(WireImu)) {
        std::fprintf(stderr, "stdin imu payload size mismatch\n");
        return finish(2);
      }
      WireImu imu{};
      if (!read_exact(&imu, sizeof(imu))) {
        std::fprintf(stderr, "stdin imu payload truncated\n");
        return finish(1);
      }
      ++imu_records;
      const std::uint64_t output_timestamp_ns =
          output_timestamp(header.timestamp_ns, target_deadline);
      if (!validate_only && !emit_imu(output_timestamp_ns, imu)) {
        return finish(1);
      }
    } else if (header.record_type == kRecordOdomPrior) {
      if (header.count != 1 || header.payload_bytes != sizeof(WireOdomPrior)) {
        std::fprintf(stderr, "stdin odom prior payload size mismatch\n");
        return finish(2);
      }
      WireOdomPrior prior{};
      if (!read_exact(&prior, sizeof(prior))) {
        std::fprintf(stderr, "stdin odom prior payload truncated\n");
        return finish(1);
      }
      ++odom_records;
      const std::uint64_t output_timestamp_ns =
          output_timestamp(header.timestamp_ns, target_deadline);
      if (!validate_only && !emit_odom_prior(output_timestamp_ns, prior)) {
        return finish(1);
      }
    } else if (header.record_type == kRecordRegisteredCloud) {
      if (header.payload_bytes != header.count * sizeof(WirePoint)) {
        std::fprintf(stderr, "stdin registered cloud payload size mismatch\n");
        return finish(2);
      }
      std::vector<WirePoint> points(header.count);
      if (!read_exact(points.data(), header.payload_bytes)) {
        std::fprintf(stderr, "stdin registered cloud payload truncated\n");
        return finish(1);
      }
      ++registered_cloud_records;
      if (!validate_only && !navigation_fixture) {
        std::fprintf(
            stderr,
            "stdin registered cloud requires --navigation-fixture\n");
        return finish(2);
      }
      const std::uint64_t output_timestamp_ns =
          output_timestamp(header.timestamp_ns, target_deadline);
      if (!validate_only &&
          !emit_registered_cloud(output_timestamp_ns, points)) {
        return finish(1);
      }
    } else {
      std::fprintf(stderr, "stdin record has unknown type: %u\n", header.record_type);
      return finish(2);
    }
  }
  return finish(0);
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

void point_cloud_callback(uint32_t handle,
                           const uint8_t dev_type,
                           LivoxLidarEthernetPacket* data,
                           void* /*client_data*/) {
  if (data == nullptr || !accept_lidar_handle(handle)) {
    return;
  }
  const auto base_time_ns = packet_timestamp_ns(
      lidar::PacketTimestampStream::Lidar, data);
  if (!base_time_ns.has_value()) {
    return;
  }
  switch (data->data_type) {
    case kLivoxLidarCartesianCoordinateHighData:
      handle_high_points(dev_type, data, *base_time_ns);
      break;
    case kLivoxLidarCartesianCoordinateLowData:
      handle_low_points(dev_type, data, *base_time_ns);
      break;
    case kLivoxLidarSphericalCoordinateData:
      handle_spherical_points(dev_type, data, *base_time_ns);
      break;
    default:
      break;
  }
}

void imu_callback(uint32_t handle,
                   const uint8_t /*dev_type*/,
                   LivoxLidarEthernetPacket* data,
                   void* /*client_data*/) {
  if (data == nullptr || data->data_type != kLivoxLidarImuData ||
      !accept_lidar_handle(handle)) {
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
  const auto timestamp_ns = packet_timestamp_ns(
      lidar::PacketTimestampStream::Imu, data);
  if (timestamp_ns.has_value() && should_emit_imu(*timestamp_ns)) {
    emit_imu(*timestamp_ns, imu);
  }
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
  if (info == nullptr || !accept_lidar_handle(handle)) {
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
  g_exit_state.request_signal_stop();
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
    } else if (arg == "--stdin-records") {
      config.stdin_records = true;
    } else if (arg == "--validate-records") {
      config.stdin_records = true;
      config.validate_records = true;
    } else if (arg == "--restamp-stdin-records") {
      config.restamp_stdin_records = true;
    } else if (arg == "--navigation-fixture") {
      config.navigation_fixture = true;
    } else if (arg == "--replay-rate") {
      config.replay_rate = std::stod(next());
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
    } else if (arg == "--imu-publish-freq") {
      const double hz = std::stod(next());
      if (hz < 0.0) {
        throw std::runtime_error("--imu-publish-freq must be non-negative");
      }
      config.imu_publish_hz = hz;
    } else if (arg == "--lidar-frame") {
      config.lidar_frame = next();
    } else if (arg == "--imu-frame") {
      config.imu_frame = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: livox_sdk2_stream [--dds] [--stdin-records|--validate-records] "
          "[--restamp-stdin-records] [--navigation-fixture] "
          "[--replay-rate RATE] [--domain-id N] "
          "[--publish-freq HZ|--scan-window SEC] "
          "[--imu-publish-freq HZ] "
          "[--lidar-frame FRAME] [--imu-frame FRAME] [<MID360_config.json>]");
    } else if (config.config_path.empty()) {
      config.config_path = arg;
    } else {
      throw std::runtime_error("unexpected argument: " + arg);
    }
  }
  if (!std::isfinite(config.replay_rate) || config.replay_rate < 0.0) {
    throw std::runtime_error("--replay-rate must be finite and non-negative");
  }
  if (config.stdin_records && !config.dds && !config.validate_records) {
    throw std::runtime_error("--stdin-records requires --dds");
  }
  if (config.navigation_fixture && !config.stdin_records) {
    throw std::runtime_error("--navigation-fixture requires --stdin-records");
  }
  if (!config.stdin_records && config.config_path.empty()) {
    throw std::runtime_error(
        "usage: livox_sdk2_stream [--dds] [--stdin-records|--validate-records] "
        "[--restamp-stdin-records] [--navigation-fixture] "
        "[--replay-rate RATE] [--domain-id N] "
        "[--publish-freq HZ|--scan-window SEC] "
        "[--imu-publish-freq HZ] "
        "[--lidar-frame FRAME] [--imu-frame FRAME] [<MID360_config.json>]");
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
  _setmode(_fileno(stdin), _O_BINARY);
#endif

#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  if (cli.dds) {
    try {
      g_dds_publisher = std::make_unique<lidar::DdsModule>(
          cli.domain_id,
          cli.lidar_frame,
          cli.imu_frame,
          cli.navigation_fixture);
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
  g_imu_min_interval_ns.store(
      cli.imu_publish_hz > 0.0
          ? static_cast<uint64_t>(1000000000.0 / cli.imu_publish_hz)
          : 0ULL,
      std::memory_order_relaxed);

  std::signal(SIGINT, signal_stop);
  std::signal(SIGTERM, signal_stop);

  if (cli.stdin_records) {
    return run_stdin_records(
        cli.validate_records ? 0.0 : cli.replay_rate,
        cli.validate_records,
        cli.restamp_stdin_records,
        cli.navigation_fixture);
  }

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
  g_exit_state.wait_for_stop();
  const auto exit_decision = g_exit_state.snapshot_exit_decision();
  if (exit_decision.flush_final_batch && g_scan_accumulator) {
    if (auto batch = g_scan_accumulator->flush()) {
      emit_cloud(batch->lidar_id, batch->timestamp_ns, batch->points);
    }
  }

  LivoxLidarSdkUninit();
#if defined(LINGTU_LIVOX_SDK2_STREAM_HAS_DDS) && LINGTU_LIVOX_SDK2_STREAM_HAS_DDS
  g_dds_publisher.reset();
#endif
  std::fprintf(stderr, "livox_sdk2_stream stopped\n");
  return exit_decision.return_code;
}
