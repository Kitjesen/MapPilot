#include "slam.hpp"
#include "message/cpp/dds_topics.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace {

using lingtu::slam::Cloud;
using lingtu::slam::ISlamBackend;
using lingtu::slam::ImuSample;
using lingtu::slam::LidarFrame;
using lingtu::slam::PointXYZIT;
using lingtu::slam::Pose3d;
using lingtu::slam::SlamConfig;
using lingtu::slam::SlamMode;
using lingtu::slam::SlamOutputs;
using lingtu::slam::Status;
using lingtu::slam::Transform3d;
using lingtu::slam::makeContractBackend;
using lingtu::slam::makeFastLioBackend;
using lingtu::slam::makePointLioBackend;
using lingtu::slam::modeFromString;
using lingtu::slam::toString;

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

double stampSeconds(const lingtu_dds_Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = nowSeconds();
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

std::string normalizedBackend(std::string backend) {
  std::transform(backend.begin(), backend.end(), backend.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (backend.empty() || backend == "fastlio" || backend == "fastlio2" || backend == "localizer") {
    return "fastlio2";
  }
  return backend;
}

std::unique_ptr<ISlamBackend> createBackend(const std::string& backend) {
  const std::string normalized = normalizedBackend(backend);
  if (normalized == "fastlio2") {
    return makeFastLioBackend();
  }
  if (normalized == "pointlio") {
    return makePointLioBackend();
  }
  return makeContractBackend(normalized);
}

LidarFrame toLidarFrame(const lingtu_dds_LivoxFrame& msg) {
  LidarFrame frame;
  frame.stamp_s = msg.timebase > 0
      ? static_cast<double>(msg.timebase) * 1e-9
      : stampSeconds(msg.header.stamp);
  frame.frame_id = msg.header.frame_id && msg.header.frame_id[0] != '\0'
      ? msg.header.frame_id
      : "livox_frame";
  frame.points.reserve(msg.points._length);
  for (uint32_t i = 0; i < msg.points._length; ++i) {
    const auto& src = msg.points._buffer[i];
    PointXYZIT point;
    point.x = src.x;
    point.y = src.y;
    point.z = src.z;
    point.intensity = static_cast<float>(src.reflectivity);
    point.offset_time_ns = static_cast<std::int64_t>(src.offset_time);
    point.line = src.line;
    point.tag = src.tag;
    frame.points.push_back(point);
  }
  return frame;
}

ImuSample toImuSample(const lingtu_dds_Imu& msg) {
  ImuSample sample;
  sample.stamp_s = stampSeconds(msg.header.stamp);
  sample.qx = msg.orientation.x;
  sample.qy = msg.orientation.y;
  sample.qz = msg.orientation.z;
  sample.qw = msg.orientation.w;
  sample.gx = msg.angular_velocity.x;
  sample.gy = msg.angular_velocity.y;
  sample.gz = msg.angular_velocity.z;
  sample.ax = msg.linear_acceleration.x;
  sample.ay = msg.linear_acceleration.y;
  sample.az = msg.linear_acceleration.z;
  return sample;
}

lingtu_dds_Pose toDdsPose(const Pose3d& pose) {
  lingtu_dds_Pose out{};
  out.position.x = pose.x;
  out.position.y = pose.y;
  out.position.z = pose.z;
  out.orientation.x = pose.qx;
  out.orientation.y = pose.qy;
  out.orientation.z = pose.qz;
  out.orientation.w = pose.qw;
  return out;
}

lingtu_dds_Odometry toDdsOdom(
    const Pose3d& pose,
    double stamp_s,
    const char* frame_id,
    const char* child_frame_id) {
  lingtu_dds_Odometry out{};
  fillHeader(out.header, stamp_s, frame_id);
  out.child_frame_id = const_cast<char*>(child_frame_id);
  out.pose.pose = toDdsPose(pose);
  return out;
}

struct TfMessage {
  lingtu_dds_TFMessage msg{};
  std::array<lingtu_dds_TransformStamped, 1> transforms{};
};

TfMessage toDdsTfMessage(const Transform3d& transform, double stamp_s) {
  TfMessage out;
  auto& stamped = out.transforms[0];
  fillHeader(stamped.header, stamp_s, transform.frame_id.c_str());
  stamped.child_frame_id = const_cast<char*>(transform.child_frame_id.c_str());
  stamped.transform.translation.x = transform.pose.x;
  stamped.transform.translation.y = transform.pose.y;
  stamped.transform.translation.z = transform.pose.z;
  stamped.transform.rotation.x = transform.pose.qx;
  stamped.transform.rotation.y = transform.pose.qy;
  stamped.transform.rotation.z = transform.pose.qz;
  stamped.transform.rotation.w = transform.pose.qw;
  out.msg.transforms._maximum = static_cast<std::uint32_t>(out.transforms.size());
  out.msg.transforms._length = static_cast<std::uint32_t>(out.transforms.size());
  out.msg.transforms._buffer = out.transforms.data();
  out.msg.transforms._release = false;
  return out;
}

void writeFloat(std::vector<std::uint8_t>& data, std::size_t offset, float value) {
  std::memcpy(data.data() + offset, &value, sizeof(float));
}

void fieldsFill(std::array<lingtu_dds_PointField, 4>& fields) {
  fields[0].name = const_cast<char*>("x");
  fields[0].offset = 0;
  fields[0].datatype = 7;
  fields[0].count = 1;
  fields[1].name = const_cast<char*>("y");
  fields[1].offset = 4;
  fields[1].datatype = 7;
  fields[1].count = 1;
  fields[2].name = const_cast<char*>("z");
  fields[2].offset = 8;
  fields[2].datatype = 7;
  fields[2].count = 1;
  fields[3].name = const_cast<char*>("intensity");
  fields[3].offset = 12;
  fields[3].datatype = 7;
  fields[3].count = 1;
}

std::string jsonEscape(const std::string& value);

struct CloudMessage {
  lingtu_dds_PointCloud2 msg{};
  std::array<lingtu_dds_PointField, 4> fields{};
  std::vector<std::uint8_t> data;
};

CloudMessage toDdsCloud(const Cloud& cloud) {
  CloudMessage out;
  fieldsFill(out.fields);
  constexpr std::uint32_t point_step = 16;
  const auto width = static_cast<std::uint32_t>(cloud.points.size());
  const auto row_step = point_step * width;
  out.data.resize(static_cast<std::size_t>(row_step));
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const std::size_t base = i * point_step;
    writeFloat(out.data, base + 0, cloud.points[i].x);
    writeFloat(out.data, base + 4, cloud.points[i].y);
    writeFloat(out.data, base + 8, cloud.points[i].z);
    writeFloat(out.data, base + 12, cloud.points[i].intensity);
  }
  fillHeader(out.msg.header, cloud.stamp_s, cloud.frame_id.c_str());
  out.msg.height = 1;
  out.msg.width = width;
  out.msg.fields._maximum = static_cast<std::uint32_t>(out.fields.size());
  out.msg.fields._length = static_cast<std::uint32_t>(out.fields.size());
  out.msg.fields._buffer = out.fields.data();
  out.msg.fields._release = false;
  out.msg.is_bigendian = false;
  out.msg.point_step = point_step;
  out.msg.row_step = row_step;
  out.msg.data._maximum = static_cast<std::uint32_t>(out.data.size());
  out.msg.data._length = static_cast<std::uint32_t>(out.data.size());
  out.msg.data._buffer = out.data.data();
  out.msg.data._release = false;
  out.msg.is_dense = false;
  return out;
}

std::string healthJson(const SlamOutputs& out) {
  const std::string tf_json = out.map_odom_tf.has_value()
      ? std::string("{\"valid\":true,\"frame_id\":\"") + jsonEscape(out.map_odom_tf->frame_id) +
          "\",\"child_frame_id\":\"" + jsonEscape(out.map_odom_tf->child_frame_id) +
          "\",\"tx\":" + std::to_string(out.map_odom_tf->pose.x) +
          ",\"ty\":" + std::to_string(out.map_odom_tf->pose.y) +
          ",\"tz\":" + std::to_string(out.map_odom_tf->pose.z) +
          ",\"qx\":" + std::to_string(out.map_odom_tf->pose.qx) +
          ",\"qy\":" + std::to_string(out.map_odom_tf->pose.qy) +
          ",\"qz\":" + std::to_string(out.map_odom_tf->pose.qz) +
          ",\"qw\":" + std::to_string(out.map_odom_tf->pose.qw) +
          ",\"ts\":" + std::to_string(out.stamp_s) + "}"
      : "null";
  return std::string("{\"state\":\"") + toString(out.state) +
      "\",\"confidence\":" + std::to_string(out.confidence) +
      ",\"backend\":\"cpp_cyclone_slam\",\"reason\":\"" + jsonEscape(out.reason) +
      "\",\"map_odom_tf\":" + tf_json +
      ",\"relocalization_supported\":" +
      (out.relocalization_supported ? "true" : "false") +
      ",\"saved_map_relocalization_supported\":" +
      (out.saved_map_relocalization_supported ? "true" : "false") +
      ",\"relocalization_state\":\"" + jsonEscape(out.relocalization_state) + "\"" +
      ",\"last_relocalization_message\":\"" +
      jsonEscape(out.last_relocalization_message) + "\"" +
      ",\"relocalization_quality\":" +
      std::to_string(out.relocalization_quality) + "}";
}

struct RuntimeRates {
  double status_target_hz = 0.0;
  double imu_input_hz = 0.0;
  double lidar_input_hz = 0.0;
  double slam_tick_hz = 0.0;
  double processed_scan_hz = 0.0;
};

class RateCounter {
 public:
  void mark(double t) {
    if (!std::isfinite(t) || t <= 0.0) {
      return;
    }
    if (window_start_s_ <= 0.0) {
      window_start_s_ = t;
    }
    ++count_;
    const double span_s = t - window_start_s_;
    if (span_s >= 1.0) {
      hz_ = static_cast<double>(count_) / span_s;
      count_ = 0;
      window_start_s_ = t;
    }
  }

  double hz() const {
    return hz_;
  }

 private:
  double window_start_s_ = 0.0;
  int count_ = 0;
  double hz_ = 0.0;
};

std::string gnssFusionHealthJson(const lingtu::slam::GnssFusionHealth& health) {
  return std::string("{\"enabled\":") + (health.enabled ? "true" : "false") +
      ",\"alignment_locked\":" + (health.alignment_locked ? "true" : "false") +
      ",\"last_fix_type\":\"" + jsonEscape(health.last_fix_type) + "\"" +
      ",\"last_gnss_age_s\":" + std::to_string(health.last_gnss_age_s) +
      ",\"last_residual_m\":" + std::to_string(health.last_residual_m) +
      ",\"relock_count\":" + std::to_string(health.relock_count) + "}";
}

std::string statusSnapshotJson(
    const SlamOutputs& out,
    const std::string& backend,
    const std::string& mode,
    const RuntimeRates& rates) {
  const int registered_points = out.registered_cloud_body.has_value()
      ? static_cast<int>(out.registered_cloud_body->points.size())
      : 0;
  const int map_points = out.map_cloud_map.has_value()
      ? static_cast<int>(out.map_cloud_map->points.size())
      : 0;
  const int saved_map_points = out.saved_map_cloud_map.has_value()
      ? static_cast<int>(out.saved_map_cloud_map->points.size())
      : out.saved_map_points;
  const std::string registered_cloud_frame_id = out.registered_cloud_body.has_value()
      ? out.registered_cloud_body->frame_id
      : "";
  const std::string map_cloud_frame_id = out.map_cloud_map.has_value()
      ? out.map_cloud_map->frame_id
      : "";
  const std::string saved_map_cloud_frame_id = out.saved_map_cloud_map.has_value()
      ? out.saved_map_cloud_map->frame_id
      : (out.saved_map_points > 0 && out.map_odom_tf.has_value()
          ? out.map_odom_tf->frame_id
          : "");
  const Pose3d pose = out.odometry_odom_body.value_or(Pose3d{});
  const std::string tf_json = out.map_odom_tf.has_value()
      ? std::string("{\"valid\":true,\"frame_id\":\"") + jsonEscape(out.map_odom_tf->frame_id) +
          "\",\"child_frame_id\":\"" + jsonEscape(out.map_odom_tf->child_frame_id) +
          "\",\"tx\":" + std::to_string(out.map_odom_tf->pose.x) +
          ",\"ty\":" + std::to_string(out.map_odom_tf->pose.y) +
          ",\"tz\":" + std::to_string(out.map_odom_tf->pose.z) +
          ",\"qx\":" + std::to_string(out.map_odom_tf->pose.qx) +
          ",\"qy\":" + std::to_string(out.map_odom_tf->pose.qy) +
          ",\"qz\":" + std::to_string(out.map_odom_tf->pose.qz) +
          ",\"qw\":" + std::to_string(out.map_odom_tf->pose.qw) +
          ",\"ts\":" + std::to_string(out.stamp_s) + "}"
      : "null";
  return std::string("{") +
      "\"schema_version\":\"lingtu.slam.status_snapshot.v1\"," +
      "\"source\":\"cpp_cyclone_slam\"," +
      "\"backend\":\"" + jsonEscape(backend) + "\"," +
      "\"mode\":\"" + jsonEscape(mode) + "\"," +
      "\"state\":\"" + toString(out.state) + "\"," +
      "\"reason\":\"" + jsonEscape(out.reason) + "\"," +
      "\"alive\":" + (out.alive ? "true" : "false") + "," +
      "\"has_odom\":" + (out.odometry_odom_body.has_value() ? "true" : "false") + "," +
      "\"stamp_s\":" + std::to_string(out.stamp_s) + "," +
      "\"confidence\":" + std::to_string(out.confidence) + "," +
      "\"localization_quality\":" + std::to_string(out.localization_quality) + "," +
      "\"status_target_hz\":" + std::to_string(rates.status_target_hz) + "," +
      "\"imu_input_hz\":" + std::to_string(rates.imu_input_hz) + "," +
      "\"lidar_input_hz\":" + std::to_string(rates.lidar_input_hz) + "," +
      "\"slam_tick_hz\":" + std::to_string(rates.slam_tick_hz) + "," +
      "\"processed_scan_hz\":" + std::to_string(rates.processed_scan_hz) + "," +
      "\"registered_points\":" + std::to_string(registered_points) + "," +
      "\"map_points\":" + std::to_string(map_points) + "," +
      "\"saved_map_points\":" + std::to_string(saved_map_points) + "," +
      "\"registered_cloud_frame_id\":\"" + jsonEscape(registered_cloud_frame_id) + "\"," +
      "\"map_cloud_frame_id\":\"" + jsonEscape(map_cloud_frame_id) + "\"," +
      "\"saved_map_cloud_frame_id\":\"" + jsonEscape(saved_map_cloud_frame_id) + "\"," +
      "\"imu_buffer\":" + std::to_string(out.imu_buffer) + "," +
      "\"lidar_buffer\":" + std::to_string(out.lidar_buffer) + "," +
      "\"imu_batch\":" + std::to_string(out.imu_batch) + "," +
      "\"scan_start_s\":" + std::to_string(out.scan_start_s) + "," +
      "\"scan_end_s\":" + std::to_string(out.scan_end_s) + "," +
      "\"last_imu_s\":" + std::to_string(out.last_imu_s) + "," +
      "\"sync_wait_count\":" + std::to_string(out.sync_wait_count) + "," +
      "\"imu_rollback_count\":" + std::to_string(out.imu_rollback_count) + "," +
      "\"lidar_rollback_count\":" + std::to_string(out.lidar_rollback_count) + "," +
      "\"dropped_lidar_frames\":" + std::to_string(out.dropped_lidar_frames) + "," +
      "\"dropped_imu_frames\":" + std::to_string(out.dropped_imu_frames) + "," +
      "\"map_loaded\":" + (out.map_loaded ? "true" : "false") + "," +
      "\"map_frame_jump\":" + (out.map_frame_jump ? "true" : "false") + "," +
      "\"relocalization_supported\":" +
      (out.relocalization_supported ? "true" : "false") + "," +
      "\"saved_map_relocalization_supported\":" +
      (out.saved_map_relocalization_supported ? "true" : "false") + "," +
      "\"relocalization_state\":\"" + jsonEscape(out.relocalization_state) + "\"," +
      "\"last_relocalization_message\":\"" +
      jsonEscape(out.last_relocalization_message) + "\"," +
      "\"relocalization_quality\":" +
      std::to_string(out.relocalization_quality) + "," +
      "\"scene_mode\":\"" + jsonEscape(out.scene_mode) + "\"," +
      "\"gnss_fusion_health\":" + gnssFusionHealthJson(out.gnss_fusion_health) + "," +
      "\"map_odom_tf\":" + tf_json + "," +
      "\"odometry\":{\"frame_id\":\"odom\",\"child_frame_id\":\"body\"," +
      "\"pose\":{\"x\":" + std::to_string(pose.x) +
      ",\"y\":" + std::to_string(pose.y) +
      ",\"z\":" + std::to_string(pose.z) +
      ",\"qx\":" + std::to_string(pose.qx) +
      ",\"qy\":" + std::to_string(pose.qy) +
      ",\"qz\":" + std::to_string(pose.qz) +
      ",\"qw\":" + std::to_string(pose.qw) + "}}}";
}

void writeTextAtomic(const std::string& path, const std::string& text) {
  if (path.empty()) {
    return;
  }
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::out | std::ios::trunc);
    if (!out) {
      std::fprintf(stderr, "status_json open failed: %s\n", tmp.c_str());
      return;
    }
    out << text << "\n";
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    std::fprintf(stderr, "status_json rename failed: %s -> %s\n", tmp.c_str(), path.c_str());
  }
}

void writeBytesAtomic(const std::string& path, const std::string& bytes, const char* label) {
  if (path.empty()) {
    return;
  }
  const std::filesystem::path target(path);
  const std::filesystem::path parent = target.parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      std::fprintf(stderr, "%s mkdir failed: %s\n", label, parent.string().c_str());
      return;
    }
  }
  const std::string tmp = path + ".tmp";
  {
    std::ofstream out(tmp, std::ios::out | std::ios::binary | std::ios::trunc);
    if (!out) {
      std::fprintf(stderr, "%s open failed: %s\n", label, tmp.c_str());
      return;
    }
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
  }
  if (std::rename(tmp.c_str(), path.c_str()) != 0) {
    std::fprintf(stderr, "%s rename failed: %s -> %s\n", label, tmp.c_str(), path.c_str());
  }
}

void appendU32(std::string& out, std::uint32_t value) {
  out.append(reinterpret_cast<const char*>(&value), sizeof(value));
}

void appendDouble(std::string& out, double value) {
  out.append(reinterpret_cast<const char*>(&value), sizeof(value));
}

void appendFloat(std::string& out, float value) {
  out.append(reinterpret_cast<const char*>(&value), sizeof(value));
}

void writeCloudSnapshotAtomic(const std::string& path, const Cloud& cloud) {
  constexpr std::uint32_t kCols = 4;
  const auto n = static_cast<std::uint32_t>(cloud.points.size());
  std::string payload;
  payload.reserve(
      sizeof(std::uint32_t) * 3 + sizeof(double) + cloud.frame_id.size() +
      static_cast<std::size_t>(n) * kCols * sizeof(float));
  appendU32(payload, n);
  appendU32(payload, kCols);
  appendDouble(payload, cloud.stamp_s);
  appendU32(payload, static_cast<std::uint32_t>(cloud.frame_id.size()));
  payload.append(cloud.frame_id);
  for (const auto& point : cloud.points) {
    appendFloat(payload, point.x);
    appendFloat(payload, point.y);
    appendFloat(payload, point.z);
    appendFloat(payload, point.intensity);
  }
  writeBytesAtomic(path, payload, "cloud_snapshot");
}

std::string jsonEscape(const std::string& value) {
  std::string out;
  out.reserve(value.size() + 8);
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += ch;
        break;
    }
  }
  return out;
}

std::optional<std::string> jsonStringValue(const std::string& json, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = json.find(needle);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  pos = json.find(':', pos + needle.size());
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  pos = json.find('"', pos + 1);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  std::string value;
  bool escaped = false;
  for (std::size_t i = pos + 1; i < json.size(); ++i) {
    const char ch = json[i];
    if (escaped) {
      switch (ch) {
        case 'n':
          value += '\n';
          break;
        case 'r':
          value += '\r';
          break;
        case 't':
          value += '\t';
          break;
        default:
          value += ch;
          break;
      }
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return value;
    }
    value += ch;
  }
  return std::nullopt;
}

std::optional<double> jsonDoubleValue(const std::string& json, const std::string& key) {
  const std::string needle = "\"" + key + "\"";
  std::size_t pos = json.find(needle);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  pos = json.find(':', pos + needle.size());
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  ++pos;
  while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos]))) {
    ++pos;
  }
  const std::size_t start = pos;
  while (pos < json.size()) {
    const char ch = json[pos];
    if (!(std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+' ||
          ch == '.' || ch == 'e' || ch == 'E')) {
      break;
    }
    ++pos;
  }
  if (pos == start) {
    return std::nullopt;
  }
  try {
    return std::stod(json.substr(start, pos - start));
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

struct MapCommand {
  std::string request_id;
  std::string action;
  std::string path;
  std::optional<double> x;
  std::optional<double> y;
  std::optional<double> z;
  std::optional<double> yaw;
  std::optional<double> qx;
  std::optional<double> qy;
  std::optional<double> qz;
  std::optional<double> qw;
};

MapCommand parseMapCommand(const lingtu_dds_Text& msg) {
  const std::string payload = msg.data ? msg.data : "";
  MapCommand command;
  command.request_id = jsonStringValue(payload, "request_id").value_or("");
  command.action = jsonStringValue(payload, "action").value_or(payload);
  command.path = jsonStringValue(payload, "path").value_or("");
  command.x = jsonDoubleValue(payload, "x");
  command.y = jsonDoubleValue(payload, "y");
  command.z = jsonDoubleValue(payload, "z");
  command.yaw = jsonDoubleValue(payload, "yaw");
  command.qx = jsonDoubleValue(payload, "qx");
  command.qy = jsonDoubleValue(payload, "qy");
  command.qz = jsonDoubleValue(payload, "qz");
  command.qw = jsonDoubleValue(payload, "qw");
  if (command.action == "save" || command.action == "save-map") {
    command.action = "save_map";
  } else if (command.action == "load" || command.action == "load-map") {
    command.action = "load_map";
  } else if (
      command.action == "relocalize_saved_map" ||
      command.action == "relocalize-saved-map") {
    command.action = "relocalize";
  } else if (command.action == "global-relocalize") {
    command.action = "global_relocalize";
  }
  return command;
}

std::optional<Pose3d> poseFromCommand(const MapCommand& command) {
  const bool has_position =
      command.x.has_value() || command.y.has_value() || command.z.has_value();
  const bool has_orientation =
      command.yaw.has_value() || command.qx.has_value() || command.qy.has_value() ||
      command.qz.has_value() || command.qw.has_value();
  if (!has_position && !has_orientation) {
    return std::nullopt;
  }
  Pose3d pose;
  pose.x = command.x.value_or(0.0);
  pose.y = command.y.value_or(0.0);
  pose.z = command.z.value_or(0.0);
  if (command.yaw.has_value()) {
    const double half = *command.yaw * 0.5;
    pose.qz = std::sin(half);
    pose.qw = std::cos(half);
  }
  pose.qx = command.qx.value_or(pose.qx);
  pose.qy = command.qy.value_or(pose.qy);
  pose.qz = command.qz.value_or(pose.qz);
  pose.qw = command.qw.value_or(pose.qw);
  return pose;
}

std::string mapEventJson(
    const MapCommand& command,
    bool success,
    const std::string& message,
    const std::string& path,
    const SlamOutputs* out = nullptr) {
  std::string extras;
  if (out != nullptr) {
    extras += std::string(",\"relocalization_supported\":") +
        (out->relocalization_supported ? "true" : "false");
    extras += std::string(",\"saved_map_relocalization_supported\":") +
        (out->saved_map_relocalization_supported ? "true" : "false");
    extras += ",\"relocalization_state\":\"" +
        jsonEscape(out->relocalization_state) + "\"";
    extras += ",\"last_relocalization_message\":\"" +
        jsonEscape(out->last_relocalization_message) + "\"";
    extras += ",\"relocalization_quality\":" +
        std::to_string(out->relocalization_quality);
    if (out->map_odom_tf.has_value()) {
      const auto& tf = *out->map_odom_tf;
      extras += ",\"map_odom_tf\":{\"valid\":true,\"frame_id\":\"" +
          jsonEscape(tf.frame_id) + "\",\"child_frame_id\":\"" +
          jsonEscape(tf.child_frame_id) + "\",\"tx\":" +
          std::to_string(tf.pose.x) + ",\"ty\":" + std::to_string(tf.pose.y) +
          ",\"tz\":" + std::to_string(tf.pose.z) + ",\"qx\":" +
          std::to_string(tf.pose.qx) + ",\"qy\":" +
          std::to_string(tf.pose.qy) + ",\"qz\":" +
          std::to_string(tf.pose.qz) + ",\"qw\":" +
          std::to_string(tf.pose.qw) + "}";
    }
  }
  return std::string("{\"schema_version\":\"lingtu.slam.map_event.v1\",") +
      "\"request_id\":\"" + jsonEscape(command.request_id) + "\"," +
      "\"action\":\"" + jsonEscape(command.action) + "\"," +
      "\"success\":" + (success ? "true" : "false") + "," +
      "\"path\":\"" + jsonEscape(path) + "\"," +
      "\"message\":\"" + jsonEscape(message) + "\"," +
      "\"source\":\"cpp_cyclone_slam\"" + extras + "}";
}

struct CliConfig {
  std::string backend = "fastlio2";
  std::string mode = "mapping";
  std::string map_path;
  std::string config_path;
  std::string status_json_path;
  std::string cloud_snapshot_dir;
  int domain_id = 0;
  double tick_hz = 50.0;
  double log_status_s = 0.0;
  double status_json_hz = 10.0;
  double cloud_snapshot_hz = 5.0;
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
    if (arg == "--backend") {
      cfg.backend = next();
    } else if (arg == "--mode") {
      cfg.mode = next();
    } else if (arg == "--map") {
      cfg.map_path = next();
    } else if (arg == "--config") {
      cfg.config_path = next();
    } else if (arg == "--status-json") {
      cfg.status_json_path = next();
    } else if (arg == "--cloud-snapshot-dir") {
      cfg.cloud_snapshot_dir = next();
    } else if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
    } else if (arg == "--log-status-s") {
      cfg.log_status_s = std::stod(next());
    } else if (arg == "--status-json-hz") {
      cfg.status_json_hz = std::stod(next());
    } else if (arg == "--cloud-snapshot-hz") {
      cfg.cloud_snapshot_hz = std::stod(next());
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_slam_cyclone_runtime [--backend fastlio2] "
          "[--mode mapping|localization] [--map PATH] [--config PATH] "
          "[--domain-id N] [--tick-hz HZ] [--log-status-s SECONDS] "
          "[--status-json PATH] [--status-json-hz HZ] "
          "[--cloud-snapshot-dir DIR] [--cloud-snapshot-hz HZ]");
    }
  }
  return cfg;
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

template <typename T, typename Handler>
void drainReader(
    dds_entity_t reader,
    const dds_topic_descriptor_t& descriptor,
    Handler&& handler) {
  constexpr std::size_t kMaxSamples = 16;
  void* samples[kMaxSamples];
  dds_sample_info_t infos[kMaxSamples];
  for (auto& sample : samples) {
    sample = dds_alloc(sizeof(T));
    std::memset(sample, 0, sizeof(T));
  }
  const dds_return_t count = dds_take(reader, samples, infos, kMaxSamples, kMaxSamples);
  if (count >= 0) {
    for (dds_return_t i = 0; i < count; ++i) {
      if (infos[i].valid_data) {
        handler(*static_cast<T*>(samples[i]));
      }
    }
  } else {
    logDdsError(count, "dds_take");
  }
  for (auto& sample : samples) {
    dds_sample_free(sample, &descriptor, DDS_FREE_ALL);
  }
}

enum class WriterQos {
  Default,
  TfDynamic,
  TfStatic,
};

enum class ReaderQos {
  Default,
  SensorStream,
};

void applySensorQos(dds_qos_t* qos) {
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
  dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 256);
}

void applyWriterQos(dds_qos_t* qos, WriterQos kind) {
  if (kind == WriterQos::TfDynamic) {
    dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, DDS_SECS(1));
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 100);
    return;
  }
  if (kind == WriterQos::TfStatic) {
    dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    dds_qset_durability(qos, DDS_DURABILITY_TRANSIENT_LOCAL);
    dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);
  }
}

class DdsRuntime {
 public:
  explicit DdsRuntime(int domain_id) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    subscriber_ = checked(dds_create_subscriber(participant_, nullptr, nullptr),
                          "dds_create_subscriber");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher");

    lidar_reader_ = reader<lingtu_dds_LivoxFrame>(
        lingtu::message::kLidarRawFrame.dds_topic.data(),
        &lingtu_dds_LivoxFrame_desc,
        "lidar",
        ReaderQos::SensorStream);
    imu_reader_ = reader<lingtu_dds_Imu>(
        lingtu::message::kImuRaw.dds_topic.data(),
        &lingtu_dds_Imu_desc,
        "imu",
        ReaderQos::SensorStream);
    map_command_reader_ = reader<lingtu_dds_Text>(
        lingtu::message::kSlamMapCommand.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "map_command");
    tf_writer_ = writer<lingtu_dds_TFMessage>(
        lingtu::message::kTf.dds_topic.data(),
        &lingtu_dds_TFMessage_desc,
        "tf",
        WriterQos::TfDynamic);
    tf_static_writer_ = writer<lingtu_dds_TFMessage>(
        lingtu::message::kTfStatic.dds_topic.data(),
        &lingtu_dds_TFMessage_desc,
        "tf_static",
        WriterQos::TfStatic);
    odom_writer_ = writer<lingtu_dds_Odometry>(
        lingtu::message::kSlamOdometry.dds_topic.data(), &lingtu_dds_Odometry_desc, "odom");
    state_writer_ = writer<lingtu_dds_Odometry>(
        lingtu::message::kSlamStateAtScan.dds_topic.data(), &lingtu_dds_Odometry_desc, "state");
    registered_writer_ = writer<lingtu_dds_PointCloud2>(
        lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "registered_cloud");
    map_writer_ = writer<lingtu_dds_PointCloud2>(
        lingtu::message::kSlamMapCloud.dds_topic.data(), &lingtu_dds_PointCloud2_desc, "map_cloud");
    saved_map_writer_ = writer<lingtu_dds_PointCloud2>(
        lingtu::message::kSlamSavedMapCloud.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "saved_map_cloud");
    map_event_writer_ = writer<lingtu_dds_Text>(
        lingtu::message::kSlamMapEvent.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "map_event");
    quality_writer_ = writer<lingtu_dds_Float32>(
        lingtu::message::kSlamLocalizationQuality.dds_topic.data(),
        &lingtu_dds_Float32_desc,
        "quality");
    health_writer_ = writer<lingtu_dds_Text>(
        lingtu::message::kSlamLocalizationHealth.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "health");
  }

  ~DdsRuntime() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  DdsRuntime(const DdsRuntime&) = delete;
  DdsRuntime& operator=(const DdsRuntime&) = delete;

  template <typename Handler>
  void drainImu(Handler&& handler) {
    drainReader<lingtu_dds_Imu>(
        imu_reader_, lingtu_dds_Imu_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainLidar(Handler&& handler) {
    drainReader<lingtu_dds_LivoxFrame>(
        lidar_reader_, lingtu_dds_LivoxFrame_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainMapCommands(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        map_command_reader_, lingtu_dds_Text_desc, std::forward<Handler>(handler));
  }

  void writeOdom(const lingtu_dds_Odometry& msg) {
    logDdsError(dds_write(odom_writer_, &msg), "dds_write(odom)");
  }

  void writeState(const lingtu_dds_Odometry& msg) {
    logDdsError(dds_write(state_writer_, &msg), "dds_write(state)");
  }

  void writeRegistered(const lingtu_dds_PointCloud2& msg) {
    logDdsError(dds_write(registered_writer_, &msg), "dds_write(registered_cloud)");
  }

  void writeMap(const lingtu_dds_PointCloud2& msg) {
    logDdsError(dds_write(map_writer_, &msg), "dds_write(map_cloud)");
  }

  void writeSavedMap(const lingtu_dds_PointCloud2& msg) {
    logDdsError(dds_write(saved_map_writer_, &msg), "dds_write(saved_map_cloud)");
  }

  void writeMapEvent(const std::string& event) {
    lingtu_dds_Text msg{};
    msg.data = const_cast<char*>(event.c_str());
    logDdsError(dds_write(map_event_writer_, &msg), "dds_write(map_event)");
  }

  void writeQuality(float quality) {
    lingtu_dds_Float32 msg{};
    msg.data = quality;
    logDdsError(dds_write(quality_writer_, &msg), "dds_write(quality)");
  }

  void writeHealth(const std::string& health) {
    lingtu_dds_Text msg{};
    msg.data = const_cast<char*>(health.c_str());
    logDdsError(dds_write(health_writer_, &msg), "dds_write(health)");
  }

  void writeTf(const lingtu_dds_TFMessage& msg) {
    logDdsError(dds_write(tf_writer_, &msg), "dds_write(tf)");
  }

  void writeStaticTf(const lingtu_dds_TFMessage& msg) {
    logDdsError(dds_write(tf_static_writer_, &msg), "dds_write(tf_static)");
  }

 private:
  template <typename T>
  dds_entity_t reader(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label,
      ReaderQos qos_kind = ReaderQos::Default) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    topics_.push_back(topic);
    std::unique_ptr<dds_qos_t, decltype(&dds_delete_qos)> qos(nullptr, dds_delete_qos);
    if (qos_kind == ReaderQos::SensorStream) {
      qos.reset(dds_create_qos());
      if (!qos) {
        throw std::runtime_error("dds_create_qos failed");
      }
      applySensorQos(qos.get());
    }
    return checked(
        dds_create_reader(subscriber_, topic, qos.get(), nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
  }

  template <typename T>
  dds_entity_t writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label,
      WriterQos qos_kind = WriterQos::Default) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    topics_.push_back(topic);
    std::unique_ptr<dds_qos_t, decltype(&dds_delete_qos)> qos(nullptr, dds_delete_qos);
    if (qos_kind != WriterQos::Default) {
      qos.reset(dds_create_qos());
      if (!qos) {
        throw std::runtime_error("dds_create_qos failed");
      }
      applyWriterQos(qos.get(), qos_kind);
    }
    return checked(
        dds_create_writer(publisher_, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t participant_ = DDS_RETCODE_ERROR;
  dds_entity_t subscriber_ = DDS_RETCODE_ERROR;
  dds_entity_t publisher_ = DDS_RETCODE_ERROR;
  dds_entity_t lidar_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t imu_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t map_command_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t tf_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t tf_static_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t odom_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t state_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t registered_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t map_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t saved_map_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t map_event_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t quality_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t health_writer_ = DDS_RETCODE_ERROR;
  std::vector<dds_entity_t> topics_;
};

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    const CliConfig cli = parseArgs(argc, argv);
    auto backend = createBackend(cli.backend);
    SlamConfig config;
    config.backend = normalizedBackend(cli.backend);
    config.config_path = cli.config_path;
    Status status = backend->configure(config);
    if (!status.ok) {
      throw std::runtime_error("SLAM configure failed: " + status.message);
    }
    status = backend->setMode(modeFromString(cli.mode), cli.map_path);
    if (!status.ok) {
      std::fprintf(stderr, "SLAM mode set returned: %s\n", status.message.c_str());
    }

    DdsRuntime dds(cli.domain_id);
    const double hz = std::max(1.0, cli.tick_hz);
    const auto period = std::chrono::duration<double>(1.0 / hz);
    double last_log_s = 0.0;
    double last_status_json_s = 0.0;
    double last_cloud_snapshot_s = 0.0;
    double last_registered_cloud_stamp_s = -1.0;
    double last_map_cloud_stamp_s = -1.0;
    const double status_json_period_s = cli.status_json_hz > 0.0
        ? 1.0 / cli.status_json_hz
        : 0.0;
    const double cloud_snapshot_period_s = cli.cloud_snapshot_hz > 0.0
        ? 1.0 / cli.cloud_snapshot_hz
        : 0.0;
    RateCounter imu_input_rate;
    RateCounter lidar_input_rate;
    RateCounter slam_tick_rate;
    RateCounter processed_scan_rate;
    auto next_tick = std::chrono::steady_clock::now();
    while (g_running) {
      dds.drainImu([&](const lingtu_dds_Imu& msg) {
        imu_input_rate.mark(nowSeconds());
        const Status s = backend->feedImu(toImuSample(msg));
        if (!s.ok) {
          std::fprintf(stderr, "feedImu: %s\n", s.message.c_str());
        }
      });
      dds.drainLidar([&](const lingtu_dds_LivoxFrame& msg) {
        lidar_input_rate.mark(nowSeconds());
        const Status s = backend->feedLidar(toLidarFrame(msg));
        if (!s.ok) {
          std::fprintf(stderr, "feedLidar: %s\n", s.message.c_str());
        }
      });
      dds.drainMapCommands([&](const lingtu_dds_Text& msg) {
        const MapCommand command = parseMapCommand(msg);
        if (command.action != "save_map" && command.action != "load_map" &&
            command.action != "relocalize" && command.action != "global_relocalize") {
          dds.writeMapEvent(mapEventJson(
              command,
              false,
              "unsupported_map_command",
              command.path));
          return;
        }
        if ((command.action == "save_map" || command.action == "load_map") &&
            command.path.empty()) {
          dds.writeMapEvent(mapEventJson(command, false, "missing_path", command.path));
          return;
        }
        if ((command.action == "relocalize" || command.action == "global_relocalize") &&
            modeFromString(cli.mode) != SlamMode::Localization) {
          const SlamOutputs out = backend->outputs();
          dds.writeMapEvent(mapEventJson(
              command,
              false,
              "localization_mode_required",
              command.path,
              &out));
          return;
        }

        Status command_status;
        if (command.action == "save_map") {
          command_status = backend->saveMap(command.path);
        } else if (command.action == "load_map") {
          command_status = backend->loadMap(command.path);
        } else {
          if (!command.path.empty()) {
            const Status load_status = backend->loadMap(command.path);
            if (!load_status.ok) {
              const SlamOutputs out = backend->outputs();
              dds.writeMapEvent(mapEventJson(
                  command,
                  false,
                  load_status.message,
                  command.path,
                  &out));
              return;
            }
          }
          command_status = backend->relocalize(
              command.action == "global_relocalize" ? std::optional<Pose3d>{}
                                                     : poseFromCommand(command));
        }

        const SlamOutputs out = backend->outputs();
        if (command_status.ok && out.saved_map_cloud_map.has_value()) {
          auto saved_msg = toDdsCloud(*out.saved_map_cloud_map);
          dds.writeSavedMap(saved_msg.msg);
        }
        dds.writeMapEvent(mapEventJson(
            command,
            command_status.ok,
            command_status.message,
            command.path,
            &out));
      });

      status = backend->tick();
      if (!status.ok) {
        std::fprintf(stderr, "tick: %s\n", status.message.c_str());
      }
      slam_tick_rate.mark(nowSeconds());
      const SlamOutputs out = backend->outputs();
      if (out.map_odom_tf.has_value()) {
        const auto msg = toDdsTfMessage(*out.map_odom_tf, out.stamp_s);
        dds.writeTf(msg.msg);
      }
      if (out.odometry_odom_body.has_value()) {
        const auto msg = toDdsOdom(*out.odometry_odom_body, out.stamp_s, "odom", "body");
        dds.writeOdom(msg);
      }
      if (out.state_estimation_at_scan.has_value()) {
        const auto msg = toDdsOdom(*out.state_estimation_at_scan, out.stamp_s, "odom", "body");
        dds.writeState(msg);
      }
      if (out.registered_cloud_body.has_value() &&
          std::abs(
              out.registered_cloud_body->stamp_s - last_registered_cloud_stamp_s) > 1e-6) {
        last_registered_cloud_stamp_s = out.registered_cloud_body->stamp_s;
        auto msg = toDdsCloud(*out.registered_cloud_body);
        dds.writeRegistered(msg.msg);
      }
      if (out.map_cloud_map.has_value() &&
          std::abs(out.map_cloud_map->stamp_s - last_map_cloud_stamp_s) >
              1e-6) {
        last_map_cloud_stamp_s = out.map_cloud_map->stamp_s;
        auto msg = toDdsCloud(*out.map_cloud_map);
        dds.writeMap(msg.msg);
        processed_scan_rate.mark(nowSeconds());
      }
      if (!cli.cloud_snapshot_dir.empty() && cloud_snapshot_period_s > 0.0) {
        const double t = nowSeconds();
        if (t - last_cloud_snapshot_s >= cloud_snapshot_period_s) {
          last_cloud_snapshot_s = t;
          const std::string base = cli.cloud_snapshot_dir;
          if (out.registered_cloud_body.has_value()) {
            writeCloudSnapshotAtomic(base + "/registered_cloud.bin", *out.registered_cloud_body);
          }
          if (out.map_cloud_map.has_value()) {
            writeCloudSnapshotAtomic(base + "/map_cloud.bin", *out.map_cloud_map);
          }
          if (out.saved_map_cloud_map.has_value()) {
            writeCloudSnapshotAtomic(base + "/saved_map_cloud.bin", *out.saved_map_cloud_map);
          }
        }
      }
      dds.writeQuality(static_cast<float>(out.localization_quality));
      dds.writeHealth(healthJson(out));
      if (!cli.status_json_path.empty() && status_json_period_s > 0.0) {
        const double t = nowSeconds();
        if (t - last_status_json_s >= status_json_period_s) {
          last_status_json_s = t;
          const RuntimeRates rates{
              cli.status_json_hz,
              imu_input_rate.hz(),
              lidar_input_rate.hz(),
              slam_tick_rate.hz(),
              processed_scan_rate.hz(),
          };
          writeTextAtomic(
              cli.status_json_path,
              statusSnapshotJson(out, config.backend, cli.mode, rates));
        }
      }
      if (cli.log_status_s > 0.0) {
        const double t = nowSeconds();
        if (t - last_log_s >= cli.log_status_s) {
          last_log_s = t;
          const int registered_points = out.registered_cloud_body.has_value()
              ? static_cast<int>(out.registered_cloud_body->points.size())
              : 0;
          const int map_points = out.map_cloud_map.has_value()
              ? static_cast<int>(out.map_cloud_map->points.size())
              : 0;
          const Pose3d pose = out.odometry_odom_body.value_or(Pose3d{});
          std::fprintf(
              stderr,
              "slam_status state=%s reason=%s quality=%.3f imu_buffer=%d lidar_buffer=%d "
              "imu_batch=%d drops_lidar=%d drops_imu=%d registered_points=%d "
              "map_points=%d odom=%d x=%.3f y=%.3f z=%.3f stamp=%.3f\n",
              toString(out.state).c_str(),
              out.reason.c_str(),
              out.localization_quality,
              out.imu_buffer,
              out.lidar_buffer,
              out.imu_batch,
              out.dropped_lidar_frames,
              out.dropped_imu_frames,
              registered_points,
              map_points,
              out.odometry_odom_body.has_value() ? 1 : 0,
              pose.x,
              pose.y,
              pose.z,
              out.stamp_s);
        }
      }
      next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      const auto steady_now = std::chrono::steady_clock::now();
      if (next_tick > steady_now) {
        std::this_thread::sleep_until(next_tick);
      } else {
        next_tick = steady_now;
      }
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "%s\n", exc.what());
    return 2;
  }
}
