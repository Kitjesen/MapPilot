#include "slam.hpp"
#include "map_tracking_health.hpp"
#include "message/cpp/dds_topics.hpp"
#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/snapshot_file.hpp"

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
#include <iterator>
#include <limits>
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
using lingtu::slam::MapTrackingHealthInput;
using lingtu::slam::MapTrackingHealthProjection;
using lingtu::slam::OdomSample;
using lingtu::slam::PointXYZIT;
using lingtu::slam::Pose3d;
using lingtu::slam::ProjectMapTrackingHealth;
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

const std::string& runtimeInstanceId() {
  static const std::string value = [] {
    const auto system_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const auto steady_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    return std::string("slam-") + std::to_string(system_ns) + "-" +
        std::to_string(steady_ns);
  }();
  return value;
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

OdomSample toOdomSample(const lingtu_dds_Odometry& msg) {
  OdomSample sample;
  sample.stamp_s = stampSeconds(msg.header.stamp);
  sample.odom_body = Pose3d{
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z,
      msg.pose.pose.orientation.x,
      msg.pose.pose.orientation.y,
      msg.pose.pose.orientation.z,
      msg.pose.pose.orientation.w};
  sample.vx = msg.twist.twist.linear.x;
  sample.vy = msg.twist.twist.linear.y;
  sample.vz = msg.twist.twist.linear.z;
  sample.has_velocity =
      std::isfinite(sample.vx) && std::isfinite(sample.vy) && std::isfinite(sample.vz);
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

Pose3d composePoses(const Pose3d& lhs, const Pose3d& rhs) {
  auto normalized = [](double& x, double& y, double& z, double& w) {
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    if (!std::isfinite(norm) || norm <= 1e-12) {
      x = 0.0;
      y = 0.0;
      z = 0.0;
      w = 1.0;
      return;
    }
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  };

  double lx = lhs.qx;
  double ly = lhs.qy;
  double lz = lhs.qz;
  double lw = lhs.qw;
  double rx = rhs.qx;
  double ry = rhs.qy;
  double rz = rhs.qz;
  double rw = rhs.qw;
  normalized(lx, ly, lz, lw);
  normalized(rx, ry, rz, rw);

  const double tx = 2.0 * (ly * rhs.z - lz * rhs.y);
  const double ty = 2.0 * (lz * rhs.x - lx * rhs.z);
  const double tz = 2.0 * (lx * rhs.y - ly * rhs.x);

  Pose3d out;
  out.x = lhs.x + rhs.x + lw * tx + (ly * tz - lz * ty);
  out.y = lhs.y + rhs.y + lw * ty + (lz * tx - lx * tz);
  out.z = lhs.z + rhs.z + lw * tz + (lx * ty - ly * tx);
  out.qx = lw * rx + lx * rw + ly * rz - lz * ry;
  out.qy = lw * ry - lx * rz + ly * rw + lz * rx;
  out.qz = lw * rz + lx * ry - ly * rx + lz * rw;
  out.qw = lw * rw - lx * rx - ly * ry - lz * rz;
  normalized(out.qx, out.qy, out.qz, out.qw);
  return out;
}

lingtu_dds_Odometry toDdsOdom(
    const Pose3d& pose,
    double stamp_s,
    double vx,
    double vy,
    double vz,
    const char* frame_id,
    const char* child_frame_id) {
  lingtu_dds_Odometry out{};
  fillHeader(out.header, stamp_s, frame_id);
  out.child_frame_id = const_cast<char*>(child_frame_id);
  out.pose.pose = toDdsPose(pose);
  out.twist.twist.linear.x = vx;
  out.twist.twist.linear.y = vy;
  out.twist.twist.linear.z = vz;
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

  void bindStorage() {
    msg.fields._maximum = static_cast<std::uint32_t>(fields.size());
    msg.fields._length = static_cast<std::uint32_t>(fields.size());
    msg.fields._buffer = fields.data();
    msg.fields._release = false;
    msg.data._maximum = static_cast<std::uint32_t>(data.size());
    msg.data._length = static_cast<std::uint32_t>(data.size());
    msg.data._buffer = data.data();
    msg.data._release = false;
  }
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
  out.msg.is_bigendian = false;
  out.msg.point_step = point_step;
  out.msg.row_step = row_step;
  out.msg.is_dense = false;
  out.bindStorage();
  return out;
}

struct MapObservationMessage {
  lingtu_dds_MapObservation msg{};
  CloudMessage scan;
  std::string map_frame;
  std::string sensor_frame;
  std::string pose_state;
  std::string pose_reason;

  void bindStorage() {
    scan.bindStorage();
    scan.msg.header.frame_id = const_cast<char*>(sensor_frame.c_str());
    msg.header.frame_id = const_cast<char*>(map_frame.c_str());
    msg.sensor_frame = const_cast<char*>(sensor_frame.c_str());
    msg.scan = scan.msg;
    msg.pose_state = const_cast<char*>(pose_state.c_str());
    msg.pose_reason = const_cast<char*>(pose_reason.c_str());
  }
};

std::optional<MapObservationMessage> toDdsMapObservation(const SlamOutputs& out) {
  if (out.source_epoch == 0U || out.observation_sequence == 0U ||
      !out.registered_cloud_body.has_value() ||
      !out.state_estimation_at_scan.has_value() ||
      !out.map_odom_tf.has_value()) {
    return std::nullopt;
  }

  MapObservationMessage message;
  const Cloud& scan = *out.registered_cloud_body;
  const Pose3d map_sensor =
      composePoses(out.map_odom_tf->pose, *out.state_estimation_at_scan);
  message.map_frame =
      out.map_odom_tf->frame_id.empty() ? "map" : out.map_odom_tf->frame_id;
  message.sensor_frame = scan.frame_id.empty() ? "body" : scan.frame_id;
  message.pose_state = toString(out.state);
  message.pose_reason = out.reason;
  message.scan = toDdsCloud(scan);

  fillHeader(message.msg.header, scan.stamp_s, message.map_frame.c_str());
  message.msg.observation_sequence = out.observation_sequence;
  message.msg.reset_epoch = out.source_epoch;
  message.msg.map_sensor.translation.x = map_sensor.x;
  message.msg.map_sensor.translation.y = map_sensor.y;
  message.msg.map_sensor.translation.z = map_sensor.z;
  message.msg.map_sensor.rotation.x = map_sensor.qx;
  message.msg.map_sensor.rotation.y = map_sensor.qy;
  message.msg.map_sensor.rotation.z = map_sensor.qz;
  message.msg.map_sensor.rotation.w = map_sensor.qw;
  message.msg.sensor_origin.x = map_sensor.x;
  message.msg.sensor_origin.y = map_sensor.y;
  message.msg.sensor_origin.z = map_sensor.z;
  message.msg.pose_confidence = static_cast<float>(out.confidence);
  message.msg.localization_quality = static_cast<float>(out.localization_quality);
  message.bindStorage();
  return message;
}

std::string mapOptimizationJson(const SlamOutputs& out);

struct RuntimeMapTrackingStatus {
  bool enabled = false;
  double period_s = 0.0;
  int consecutive_failures = 0;
  std::uint64_t attempts = 0;
  std::uint64_t successes = 0;
  std::uint64_t rejections = 0;
  std::uint64_t waits = 0;
  double last_success_age_s = -1.0;
  bool async_in_flight = false;
};

MapTrackingHealthProjection projectedHealth(
    const SlamOutputs& out,
    const RuntimeMapTrackingStatus& tracking) {
  return ProjectMapTrackingHealth(
      out.state,
      out.confidence,
      out.reason,
      MapTrackingHealthInput{
          tracking.enabled,
          tracking.period_s,
          tracking.consecutive_failures,
          tracking.successes,
          tracking.last_success_age_s,
          3});
}

std::string healthJson(
    const SlamOutputs& out,
    const RuntimeMapTrackingStatus& tracking) {
  const MapTrackingHealthProjection health = projectedHealth(out, tracking);
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
  return std::string("{\"runtime_instance_id\":\"") +
      jsonEscape(runtimeInstanceId()) + "\",\"state\":\"" + toString(health.state) +
      "\",\"ts\":" + std::to_string(out.stamp_s) +
      ",\"confidence\":" + std::to_string(health.confidence) +
      ",\"map_frame_jump_sequence\":" +
      std::to_string(out.map_frame_jump_sequence) +
      ",\"source_epoch\":" + std::to_string(out.source_epoch) +
      ",\"backend\":\"cpp_cyclone_slam\",\"reason\":\"" + jsonEscape(health.reason) +
      "\",\"map_odom_tf\":" + tf_json +
      ",\"track_against_map\":{\"enabled\":" +
      (tracking.enabled ? "true" : "false") +
      ",\"consecutive_failures\":" +
      std::to_string(tracking.consecutive_failures) +
      ",\"successes\":" + std::to_string(tracking.successes) +
      ",\"last_success_age_s\":" +
      std::to_string(tracking.last_success_age_s) +
      ",\"stale_after_s\":" + std::to_string(health.stale_after_s) +
      ",\"degraded\":" + (health.degraded ? "true" : "false") + "}" +
      ",\"relocalization_supported\":" +
      (out.relocalization_supported ? "true" : "false") +
      ",\"saved_map_relocalization_supported\":" +
      (out.saved_map_relocalization_supported ? "true" : "false") +
      ",\"relocalization_state\":\"" + jsonEscape(out.relocalization_state) + "\"" +
      ",\"last_relocalization_message\":\"" +
      jsonEscape(out.last_relocalization_message) + "\"" +
      ",\"relocalization_quality\":" +
      std::to_string(out.relocalization_quality) +
      ",\"relocalization_refine_input_points\":" +
      std::to_string(out.relocalization_refine_input_points) +
      ",\"relocalization_refine_evaluated_points\":" +
      std::to_string(out.relocalization_refine_evaluated_points) +
      ",\"relocalization_min_inliers\":" +
      std::to_string(out.relocalization_min_inliers) +
      ",\"relocalization_min_evaluated_points\":" +
      std::to_string(out.relocalization_min_evaluated_points) +
      ",\"relocalization_refine_support_ratio\":" +
      std::to_string(out.relocalization_refine_support_ratio) +
      ",\"relocalization_refine_overlap_inlier_ratio\":" +
      std::to_string(out.relocalization_refine_overlap_inlier_ratio) +
      ",\"odom_prior_enabled\":" + (out.odom_prior_enabled ? "true" : "false") +
      ",\"odom_prior_active\":" + (out.odom_prior_active ? "true" : "false") +
      ",\"odom_prior_age_s\":" + std::to_string(out.odom_prior_age_s) +
      ",\"odom_prior_error_xy_m\":" + std::to_string(out.odom_prior_error_xy_m) +
      ",\"odom_prior_map_points\":" + std::to_string(out.odom_prior_map_points) +
      ",\"map_optimization\":" + mapOptimizationJson(out) + "}";
}

std::string poseJson(const Pose3d& pose) {
  return std::string("{\"x\":") + std::to_string(pose.x) +
      ",\"y\":" + std::to_string(pose.y) +
      ",\"z\":" + std::to_string(pose.z) +
      ",\"qx\":" + std::to_string(pose.qx) +
      ",\"qy\":" + std::to_string(pose.qy) +
      ",\"qz\":" + std::to_string(pose.qz) +
      ",\"qw\":" + std::to_string(pose.qw) + "}";
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

std::string mapOptimizationJson(const SlamOutputs& out) {
  return std::string("{\"status\":\"") + jsonEscape(out.map_optimization_status) +
      "\",\"backend\":\"" + jsonEscape(out.map_optimization_backend) +
      "\",\"refine_backend\":\"" + jsonEscape(out.map_optimization_refine_backend) +
      "\",\"enabled\":" + (out.map_optimization_enabled ? "true" : "false") +
      ",\"loop_closure_enabled\":" +
      (out.map_optimization_loop_closure_enabled ? "true" : "false") +
      ",\"loop_closure_applied\":" +
      (out.map_optimization_loop_closure_applied ? "true" : "false") +
      ",\"refine_enabled\":" +
      (out.map_optimization_refine_enabled ? "true" : "false") +
      ",\"refine_applied\":" +
      (out.map_optimization_refine_applied ? "true" : "false") +
      ",\"hba_refine_enabled\":" +
      (out.map_optimization_hba_refine_enabled ? "true" : "false") +
      ",\"hba_refine_applied\":" +
      (out.map_optimization_hba_refine_applied ? "true" : "false") +
      ",\"patch_count\":" + std::to_string(out.map_optimization_patch_count) +
      ",\"pose_count\":" + std::to_string(out.map_optimization_pose_count) +
      ",\"optimized_pose_count\":" +
      std::to_string(out.map_optimization_optimized_pose_count) +
      ",\"loop_count\":" + std::to_string(out.map_optimization_loop_count) +
      ",\"raw_map_points\":" +
      std::to_string(out.map_optimization_raw_map_points) +
      ",\"optimized_map_points\":" +
      std::to_string(out.map_optimization_optimized_map_points) +
      ",\"loop_closure_error_m\":" +
      std::to_string(out.map_optimization_loop_error_m) + "}";
}

std::string finiteJsonNumber(double value) {
  return std::isfinite(value) ? std::to_string(value) : "null";
}

std::string fastLioLidarUpdateJson(
    const lingtu::slam::FastLioLidarUpdateDiagnostics& diagnostics) {
  return std::string("{") +
      "\"attempted\":" + (diagnostics.attempted ? "true" : "false") +
      ",\"accepted\":" + (diagnostics.accepted ? "true" : "false") +
      ",\"attempt_sequence\":" +
      std::to_string(diagnostics.attempt_sequence) +
      ",\"rejection_reason\":\"" +
      jsonEscape(diagnostics.rejection_reason) + "\"" +
      ",\"previous_rejection_reason\":\"" +
      jsonEscape(diagnostics.previous_rejection_reason) + "\"" +
      ",\"consecutive_rejections\":" +
      std::to_string(diagnostics.consecutive_rejections) +
      ",\"downsampled_points\":" +
      std::to_string(diagnostics.downsampled_points) +
      ",\"effective_points\":" +
      std::to_string(diagnostics.effective_points) +
      ",\"candidate\":{\"translation_m\":" +
      finiteJsonNumber(diagnostics.candidate_translation_m) +
      ",\"rotation_rad\":" +
      finiteJsonNumber(diagnostics.candidate_rotation_rad) +
      ",\"velocity_mps\":" +
      finiteJsonNumber(diagnostics.candidate_velocity_mps) +
      ",\"velocity_delta_mps\":" +
      finiteJsonNumber(diagnostics.candidate_velocity_delta_mps) + "}" +
      ",\"thresholds\":{\"max_translation_m\":" +
      finiteJsonNumber(diagnostics.max_update_translation_m) +
      ",\"max_rotation_rad\":" +
      finiteJsonNumber(diagnostics.max_update_rotation_rad) +
      ",\"max_velocity_mps\":" +
      finiteJsonNumber(diagnostics.max_update_velocity_mps) +
      ",\"max_velocity_delta_mps\":" +
      finiteJsonNumber(diagnostics.max_update_velocity_delta_mps) + "}" +
      ",\"information_ldlt\":{\"evaluated\":" +
      (diagnostics.information_ldlt_evaluated ? "true" : "false") +
      ",\"decomposition_success\":" +
      (diagnostics.information_ldlt_decomposition_success ? "true" : "false") +
      ",\"positive\":" +
      (diagnostics.information_ldlt_positive ? "true" : "false") + "}" +
      ",\"candidate_covariance\":{\"evaluated\":" +
      (diagnostics.candidate_covariance_evaluated ? "true" : "false") +
      ",\"finite\":" +
      (diagnostics.candidate_covariance_finite ? "true" : "false") +
      ",\"positive_diagonal\":" +
      (diagnostics.candidate_covariance_positive_diagonal ? "true" : "false") +
      "}" +
      ",\"posterior_covariance\":{\"evaluated\":" +
      (diagnostics.posterior_covariance_evaluated ? "true" : "false") +
      ",\"finite\":" +
      (diagnostics.posterior_covariance_finite ? "true" : "false") +
      ",\"positive_diagonal\":" +
      (diagnostics.posterior_covariance_positive_diagonal ? "true" : "false") +
      "}}";
}

std::string statusSnapshotJson(
    const SlamOutputs& out,
    const std::string& backend,
    const std::string& mode,
    const RuntimeRates& rates,
    const RuntimeMapTrackingStatus& tracking) {
  const MapTrackingHealthProjection health = projectedHealth(out, tracking);
  const double localization_quality = health.degraded ||
          health.state == lingtu::slam::SlamState::Localizing
      ? 0.0
      : out.localization_quality;
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
  const std::string state_estimation_at_scan_json = out.state_estimation_at_scan.has_value()
      ? std::string("{\"stamp_s\":") +
          std::to_string(out.registered_cloud_body.has_value()
              ? out.registered_cloud_body->stamp_s
              : out.stamp_s) +
          ",\"frame_id\":\"odom\",\"child_frame_id\":\"body\",\"pose\":" +
          poseJson(*out.state_estimation_at_scan) + "}"
      : "null";
  const std::string relocalization_map_body_json = out.relocalization_map_body.has_value()
      ? poseJson(*out.relocalization_map_body)
      : "null";
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
      "\"runtime_instance_id\":\"" + jsonEscape(runtimeInstanceId()) + "\"," +
      "\"source\":\"cpp_cyclone_slam\"," +
      "\"backend\":\"" + jsonEscape(backend) + "\"," +
      "\"mode\":\"" + jsonEscape(mode) + "\"," +
      "\"state\":\"" + toString(health.state) + "\"," +
      "\"reason\":\"" + jsonEscape(health.reason) + "\"," +
      "\"alive\":" + (out.alive ? "true" : "false") + "," +
      "\"has_odom\":" + (out.odometry_odom_body.has_value() ? "true" : "false") + "," +
      "\"snapshot_written_at_s\":" + std::to_string(nowSeconds()) + "," +
      "\"stamp_s\":" + std::to_string(out.stamp_s) + "," +
      "\"confidence\":" + std::to_string(health.confidence) + "," +
      "\"localization_quality\":" + std::to_string(localization_quality) + "," +
      "\"odom_prior_enabled\":" + (out.odom_prior_enabled ? "true" : "false") + "," +
      "\"odom_prior_active\":" + (out.odom_prior_active ? "true" : "false") + "," +
      "\"odom_prior_age_s\":" + std::to_string(out.odom_prior_age_s) + "," +
      "\"odom_prior_error_xy_m\":" + std::to_string(out.odom_prior_error_xy_m) + "," +
      "\"odom_prior_map_points\":" + std::to_string(out.odom_prior_map_points) + "," +
      "\"fastlio_velocity\":{\"x\":" + std::to_string(out.fastlio_velocity_x) +
      ",\"y\":" + std::to_string(out.fastlio_velocity_y) +
      ",\"z\":" + std::to_string(out.fastlio_velocity_z) + "}," +
      "\"fastlio_degeneracy\":{\"detected\":" +
      (out.fastlio_degeneracy_detected ? "true" : "false") +
      ",\"degenerate_dof_count\":" + std::to_string(out.fastlio_degenerate_dof_count) +
      ",\"condition_number\":" + std::to_string(out.fastlio_condition_number) +
      ",\"min_eigenvalue\":" + std::to_string(out.fastlio_min_eigenvalue) +
      ",\"max_eigenvalue\":" + std::to_string(out.fastlio_max_eigenvalue) +
      ",\"effective_ratio\":" + std::to_string(out.fastlio_effective_ratio) +
      ",\"pos_cov_trace\":" + std::to_string(out.fastlio_pos_cov_trace) +
      ",\"iter_num\":" + std::to_string(out.fastlio_iter_num) +
      ",\"converged\":" + (out.fastlio_converged ? "true" : "false") + "}," +
      "\"fastlio_lidar_update\":" +
      fastLioLidarUpdateJson(out.fastlio_lidar_update) + "," +
      "\"status_target_hz\":" + std::to_string(rates.status_target_hz) + "," +
      "\"imu_input_hz\":" + std::to_string(rates.imu_input_hz) + "," +
      "\"lidar_input_hz\":" + std::to_string(rates.lidar_input_hz) + "," +
      "\"slam_tick_hz\":" + std::to_string(rates.slam_tick_hz) + "," +
      "\"processed_scan_hz\":" + std::to_string(rates.processed_scan_hz) + "," +
      "\"registered_points\":" + std::to_string(registered_points) + "," +
      "\"observation_sequence\":" + std::to_string(out.observation_sequence) + "," +
      "\"source_epoch\":" + std::to_string(out.source_epoch) + "," +
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
      "\"map_frame_jump_sequence\":" +
      std::to_string(out.map_frame_jump_sequence) + "," +
      "\"relocalization_supported\":" +
      (out.relocalization_supported ? "true" : "false") + "," +
      "\"saved_map_relocalization_supported\":" +
      (out.saved_map_relocalization_supported ? "true" : "false") + "," +
      "\"track_against_map\":{\"enabled\":" +
      (tracking.enabled ? "true" : "false") +
      ",\"period_s\":" + std::to_string(tracking.period_s) +
      ",\"consecutive_failures\":" +
      std::to_string(tracking.consecutive_failures) +
      ",\"attempts\":" + std::to_string(tracking.attempts) +
      ",\"successes\":" + std::to_string(tracking.successes) +
      ",\"rejections\":" + std::to_string(tracking.rejections) +
      ",\"waits\":" + std::to_string(tracking.waits) +
      ",\"last_success_age_s\":" +
      std::to_string(tracking.last_success_age_s) +
      ",\"async_in_flight\":" +
      (tracking.async_in_flight ? "true" : "false") +
      ",\"stale_after_s\":" + std::to_string(health.stale_after_s) +
      ",\"degraded\":" + (health.degraded ? "true" : "false") + "}," +
      "\"relocalization_state\":\"" + jsonEscape(out.relocalization_state) + "\"," +
      "\"last_relocalization_message\":\"" +
      jsonEscape(out.last_relocalization_message) + "\"," +
      "\"relocalization_quality\":" +
      std::to_string(out.relocalization_quality) + "," +
      "\"relocalization_map_body\":" + relocalization_map_body_json + "," +
      "\"relocalization_refine_backend\":\"" +
      jsonEscape(out.relocalization_refine_backend) + "\"," +
      "\"relocalization_refine_iterations\":" +
      std::to_string(out.relocalization_refine_iterations) + "," +
      "\"relocalization_refine_inliers\":" +
      std::to_string(out.relocalization_refine_inliers) + "," +
      "\"relocalization_refine_input_points\":" +
      std::to_string(out.relocalization_refine_input_points) + "," +
      "\"relocalization_refine_evaluated_points\":" +
      std::to_string(out.relocalization_refine_evaluated_points) + "," +
      "\"relocalization_min_inliers\":" +
      std::to_string(out.relocalization_min_inliers) + "," +
      "\"relocalization_min_evaluated_points\":" +
      std::to_string(out.relocalization_min_evaluated_points) + "," +
      "\"relocalization_refine_support_ratio\":" +
      std::to_string(out.relocalization_refine_support_ratio) + "," +
      "\"relocalization_refine_overlap_inlier_ratio\":" +
      std::to_string(out.relocalization_refine_overlap_inlier_ratio) + "," +
      "\"relocalization_refine_converged\":" +
      (out.relocalization_refine_converged ? "true" : "false") + "," +
      "\"relocalization_refine_pos_cov_trace\":" +
      std::to_string(out.relocalization_refine_pos_cov_trace) + "," +
      "\"scene_mode\":\"" + jsonEscape(out.scene_mode) + "\"," +
      "\"map_optimization\":" + mapOptimizationJson(out) + "," +
      "\"gnss_fusion_health\":" + gnssFusionHealthJson(out.gnss_fusion_health) + "," +
      "\"map_odom_tf\":" + tf_json + "," +
      "\"state_estimation_at_scan\":" + state_estimation_at_scan_json + "," +
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
  std::error_code ec;
  if (!lingtu::message::replaceSnapshotFile(tmp, path, &ec)) {
    std::fprintf(
        stderr,
        "status_json replace failed: %s -> %s: %s\n",
        tmp.c_str(),
        path.c_str(),
        ec.message().c_str());
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
  std::error_code ec;
  if (!lingtu::message::replaceSnapshotFile(tmp, path, &ec)) {
    std::fprintf(
        stderr,
        "%s replace failed: %s -> %s: %s\n",
        label,
        tmp.c_str(),
        path.c_str(),
        ec.message().c_str());
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
  const auto& points = cloud.points;
  constexpr std::uint32_t kCols = 4;
  const auto n = static_cast<std::uint32_t>(points.size());
  std::string payload;
  payload.reserve(
      sizeof(std::uint32_t) * 3 + sizeof(double) + cloud.frame_id.size() +
      static_cast<std::size_t>(n) * kCols * sizeof(float));
  appendU32(payload, n);
  appendU32(payload, kCols);
  appendDouble(payload, cloud.stamp_s);
  appendU32(payload, static_cast<std::uint32_t>(cloud.frame_id.size()));
  payload.append(cloud.frame_id);
  for (const auto& point : points) {
    appendFloat(payload, point.x);
    appendFloat(payload, point.y);
    appendFloat(payload, point.z);
    appendFloat(payload, point.intensity);
  }
  writeBytesAtomic(path, payload, "cloud_snapshot");
}

void writeLidarScanSnapshotAtomic(const std::string& path, const LidarFrame& frame) {
  Cloud cloud;
  cloud.stamp_s = frame.stamp_s;
  cloud.frame_id = frame.frame_id;
  cloud.points = frame.points;
  writeCloudSnapshotAtomic(path, cloud);
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

std::optional<Pose3d> loadTrackSeed(const std::string& path, const std::string& map_path) {
  if (path.empty()) {
    return std::nullopt;
  }
  std::ifstream in(path);
  if (!in) {
    return std::nullopt;
  }
  const std::string json(
      (std::istreambuf_iterator<char>(in)),
      std::istreambuf_iterator<char>());
  const auto saved_map = jsonStringValue(json, "map_path");
  if (saved_map.has_value() && !map_path.empty() && *saved_map != map_path) {
    return std::nullopt;
  }
  Pose3d pose;
  pose.x = jsonDoubleValue(json, "x").value_or(std::numeric_limits<double>::quiet_NaN());
  pose.y = jsonDoubleValue(json, "y").value_or(std::numeric_limits<double>::quiet_NaN());
  pose.z = jsonDoubleValue(json, "z").value_or(std::numeric_limits<double>::quiet_NaN());
  pose.qx = jsonDoubleValue(json, "qx").value_or(0.0);
  pose.qy = jsonDoubleValue(json, "qy").value_or(0.0);
  pose.qz = jsonDoubleValue(json, "qz").value_or(0.0);
  pose.qw = jsonDoubleValue(json, "qw").value_or(1.0);
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.z)) {
    return std::nullopt;
  }
  return pose;
}

void saveTrackSeed(
    const std::string& path,
    const std::string& map_path,
    const std::optional<Pose3d>& pose) {
  if (path.empty() || !pose.has_value()) {
    return;
  }
  writeTextAtomic(
      path,
      std::string("{\"schema_version\":\"lingtu.slam.track_seed.v1\",") +
          "\"map_path\":\"" + jsonEscape(map_path) + "\"," +
          "\"pose\":" + poseJson(*pose) + "}");
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

Pose3d poseFromDds(const lingtu_dds_Pose& pose) {
  Pose3d out;
  out.x = pose.position.x;
  out.y = pose.position.y;
  out.z = pose.position.z;
  out.qx = pose.orientation.x;
  out.qy = pose.orientation.y;
  out.qz = pose.orientation.z;
  out.qw = pose.orientation.w;
  return out;
}

std::string normalizedRelocalizationAction(std::string action) {
  if (action == "load-map") {
    return "load_map";
  }
  if (action == "relocalize" || action == "relocalize-saved-map" ||
      action == "relocalize_saved_map") {
    return "seeded_relocalize";
  }
  if (action == "global-relocalize") {
    return "global_relocalize";
  }
  if (action == "status") {
    return "query_status";
  }
  if (action == "track-against-map") {
    return "track_against_map";
  }
  return action;
}

bool isTrackAgainstMapInputWait(const std::string& message) {
  return message == "registered_cloud_unavailable" ||
      message == "registered_cloud_stale" ||
      message == "waiting_for_scan" ||
      message == "waiting_for_odometry" ||
      message == "odometry_unavailable";
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
    extras += ",\"relocalization_map_body\":";
    extras += out->relocalization_map_body.has_value()
        ? poseJson(*out->relocalization_map_body)
        : "null";
    extras += ",\"relocalization_refine_backend\":\"" +
        jsonEscape(out->relocalization_refine_backend) + "\"";
    extras += ",\"relocalization_refine_iterations\":" +
        std::to_string(out->relocalization_refine_iterations);
    extras += ",\"relocalization_refine_inliers\":" +
        std::to_string(out->relocalization_refine_inliers);
    extras += ",\"relocalization_refine_input_points\":" +
        std::to_string(out->relocalization_refine_input_points);
    extras += ",\"relocalization_refine_evaluated_points\":" +
        std::to_string(out->relocalization_refine_evaluated_points);
    extras += ",\"relocalization_min_inliers\":" +
        std::to_string(out->relocalization_min_inliers);
    extras += ",\"relocalization_min_evaluated_points\":" +
        std::to_string(out->relocalization_min_evaluated_points);
    extras += ",\"relocalization_refine_support_ratio\":" +
        std::to_string(out->relocalization_refine_support_ratio);
    extras += ",\"relocalization_refine_overlap_inlier_ratio\":" +
        std::to_string(out->relocalization_refine_overlap_inlier_ratio);
    extras += std::string(",\"relocalization_refine_converged\":") +
        (out->relocalization_refine_converged ? "true" : "false");
    extras += ",\"relocalization_refine_pos_cov_trace\":" +
        std::to_string(out->relocalization_refine_pos_cov_trace);
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

struct RelocalizationResponseMessage {
  lingtu_dds_RelocalizationResponse msg{};
  std::string request_id;
  std::string action;
  std::string engine;
  std::string message;
  std::string state;
  std::string refine_backend;
};

void fillDdsPose(lingtu_dds_Pose& out, const Pose3d& pose) {
  out.position.x = pose.x;
  out.position.y = pose.y;
  out.position.z = pose.z;
  out.orientation.x = pose.qx;
  out.orientation.y = pose.qy;
  out.orientation.z = pose.qz;
  out.orientation.w = pose.qw;
}

RelocalizationResponseMessage relocalizationResponse(
    const lingtu_dds_RelocalizationRequest& request,
    bool success,
    const std::string& message,
    const SlamOutputs& out,
    bool track_against_map_supported = false,
    bool track_against_map_enabled = false,
    int track_against_map_failures = 0) {
  RelocalizationResponseMessage response;
  response.request_id = request.request_id ? request.request_id : "";
  response.action = normalizedRelocalizationAction(request.action ? request.action : "");
  response.engine = request.engine && request.engine[0] != '\0'
      ? request.engine
      : (response.action == "global_relocalize" ? "bbs3d_gicp" :
         response.action == "seeded_relocalize" ? "seeded_gicp" : "auto");
  response.message = message;
  response.state = toString(out.state);
  response.refine_backend = out.relocalization_refine_backend;

  response.msg.request_id = const_cast<char*>(response.request_id.c_str());
  response.msg.action = const_cast<char*>(response.action.c_str());
  response.msg.engine = const_cast<char*>(response.engine.c_str());
  response.msg.success = success;
  response.msg.message = const_cast<char*>(response.message.c_str());
  response.msg.quality = out.relocalization_quality;
  response.msg.map_loaded = out.map_loaded;
  response.msg.has_map_body = out.relocalization_map_body.has_value();
  if (out.relocalization_map_body.has_value()) {
    fillDdsPose(response.msg.map_body, *out.relocalization_map_body);
  }
  response.msg.has_map_odom = out.map_odom_tf.has_value();
  if (out.map_odom_tf.has_value()) {
    fillDdsPose(response.msg.map_odom, out.map_odom_tf->pose);
  }
  response.msg.state = const_cast<char*>(response.state.c_str());
  response.msg.refine_backend = const_cast<char*>(response.refine_backend.c_str());
  response.msg.refine_iterations = out.relocalization_refine_iterations;
  response.msg.refine_inliers = out.relocalization_refine_inliers;
  response.msg.refine_input_points = out.relocalization_refine_input_points;
  response.msg.refine_evaluated_points =
      out.relocalization_refine_evaluated_points;
  response.msg.refine_support_ratio =
      out.relocalization_refine_support_ratio;
  response.msg.refine_overlap_inlier_ratio =
      out.relocalization_refine_overlap_inlier_ratio;
  response.msg.refine_converged = out.relocalization_refine_converged;
  response.msg.refine_pos_cov_trace = out.relocalization_refine_pos_cov_trace;
  response.msg.track_against_map_supported = track_against_map_supported;
  response.msg.track_against_map_enabled = track_against_map_enabled;
  response.msg.track_against_map_failures = track_against_map_failures;
  return response;
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
  double lidar_scan_snapshot_hz = 10.0;
  double track_against_map_period_s = 5.0;
  std::string track_against_map_seed_file;
  std::optional<Pose3d> track_against_map_initial_pose;
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
    } else if (arg == "--lidar-scan-snapshot-hz") {
      cfg.lidar_scan_snapshot_hz = std::stod(next());
    } else if (arg == "--track-against-map-period-s") {
      cfg.track_against_map_period_s = std::stod(next());
    } else if (arg == "--track-against-map-seed-file") {
      cfg.track_against_map_seed_file = next();
    } else if (arg == "--track-against-map-initial-pose") {
      Pose3d pose;
      pose.x = std::stod(next());
      pose.y = std::stod(next());
      pose.z = std::stod(next());
      const double yaw = std::stod(next());
      const double half = yaw * 0.5;
      pose.qz = std::sin(half);
      pose.qw = std::cos(half);
      cfg.track_against_map_initial_pose = pose;
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: slamd [--backend fastlio2] "
          "[--mode mapping|localization] [--map PATH] [--config PATH] "
          "[--domain-id N] [--tick-hz HZ] [--log-status-s SECONDS] "
          "[--status-json PATH] [--status-json-hz HZ] "
          "[--cloud-snapshot-dir DIR] [--cloud-snapshot-hz HZ] "
          "[--lidar-scan-snapshot-hz HZ] "
          "[--track-against-map-period-s SECONDS] "
          "[--track-against-map-seed-file PATH] "
          "[--track-against-map-initial-pose X Y Z YAW]");
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
    Handler&& handler,
    std::size_t max_batches = 1) {
  constexpr std::size_t kMaxSamples = 16;
  for (std::size_t batch = 0; batch < std::max<std::size_t>(1, max_batches); ++batch) {
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
    if (count < 0 || count < static_cast<dds_return_t>(kMaxSamples)) {
      break;
    }
  }
}

constexpr std::size_t kSensorStreamCatchupBatches = 16;
constexpr std::size_t kSensorStreamBatchSamples = 16;

template <typename T>
class DdsSampleBatch {
 public:
  explicit DdsSampleBatch(const dds_topic_descriptor_t& descriptor)
      : descriptor_(descriptor) {
    for (auto& sample : samples_) {
      sample = dds_alloc(sizeof(T));
      std::memset(sample, 0, sizeof(T));
    }
  }

  ~DdsSampleBatch() {
    for (auto& sample : samples_) {
      dds_sample_free(sample, &descriptor_, DDS_FREE_ALL);
    }
  }

  DdsSampleBatch(const DdsSampleBatch&) = delete;
  DdsSampleBatch& operator=(const DdsSampleBatch&) = delete;

  void** samples() {
    return samples_.data();
  }

  dds_sample_info_t* infos() {
    return infos_.data();
  }

 private:
  const dds_topic_descriptor_t& descriptor_;
  std::array<void*, kSensorStreamBatchSamples> samples_{};
  std::array<dds_sample_info_t, kSensorStreamBatchSamples> infos_{};
};

template <typename T, typename Handler>
void drainLatestReader(
    dds_entity_t reader,
    const dds_topic_descriptor_t& descriptor,
    Handler&& handler,
    std::size_t max_batches) {
  std::unique_ptr<DdsSampleBatch<T>> latest_batch;
  dds_return_t latest_index = -1;
  for (std::size_t batch = 0; batch < std::max<std::size_t>(1, max_batches); ++batch) {
    auto current_batch = std::make_unique<DdsSampleBatch<T>>(descriptor);
    const dds_return_t count = dds_take(
        reader,
        current_batch->samples(),
        current_batch->infos(),
        kSensorStreamBatchSamples,
        kSensorStreamBatchSamples);
    if (count >= 0) {
      dds_return_t current_latest_index = -1;
      for (dds_return_t i = 0; i < count; ++i) {
        if (current_batch->infos()[i].valid_data) {
          current_latest_index = i;
        }
      }
      if (current_latest_index >= 0) {
        latest_index = current_latest_index;
        latest_batch = std::move(current_batch);
      }
    } else {
      logDdsError(count, "dds_take");
    }
    if (count < 0 || count < static_cast<dds_return_t>(kSensorStreamBatchSamples)) {
      break;
    }
  }
  if (latest_batch && latest_index >= 0) {
    handler(*static_cast<T*>(latest_batch->samples()[latest_index]));
  }
}

using lingtu::dds::QosProfile;
using lingtu::dds::make_qos;

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
        QosProfile::RawLidarStream);
    imu_reader_ = reader<lingtu_dds_Imu>(
        lingtu::message::kImuRaw.dds_topic.data(),
        &lingtu_dds_Imu_desc,
        "imu",
        QosProfile::SensorStream);
    odom_prior_reader_ = reader<lingtu_dds_Odometry>(
        lingtu::message::kSlamOdomPrior.dds_topic.data(),
        &lingtu_dds_Odometry_desc,
        "odom_prior",
        QosProfile::SensorStream);
    map_command_reader_ = reader<lingtu_dds_Text>(
        lingtu::message::kSlamMapCommand.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "map_command");
    relocalization_request_reader_ = reader<lingtu_dds_RelocalizationRequest>(
        lingtu::message::kSlamRelocalizationRequest.dds_topic.data(),
        &lingtu_dds_RelocalizationRequest_desc,
        "relocalization_request");
    tf_writer_ = writer<lingtu_dds_TFMessage>(
        lingtu::message::kTf.dds_topic.data(),
        &lingtu_dds_TFMessage_desc,
        "tf",
        QosProfile::TfDynamic);
    tf_static_writer_ = writer<lingtu_dds_TFMessage>(
        lingtu::message::kTfStatic.dds_topic.data(),
        &lingtu_dds_TFMessage_desc,
        "tf_static",
        QosProfile::TfStatic);
    odom_writer_ = writer<lingtu_dds_Odometry>(
        lingtu::message::kSlamOdometry.dds_topic.data(), &lingtu_dds_Odometry_desc, "odom",
        QosProfile::HighFreqState);
    state_writer_ = writer<lingtu_dds_Odometry>(
        lingtu::message::kSlamStateAtScan.dds_topic.data(), &lingtu_dds_Odometry_desc, "state",
        QosProfile::HighFreqState);
    registered_writer_ = writer<lingtu_dds_PointCloud2>(
        lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "registered_cloud",
        QosProfile::LidarPointcloud);
    map_observation_writer_ = writer<lingtu_dds_MapObservation>(
        lingtu::message::kSlamMapObservation.dds_topic.data(),
        &lingtu_dds_MapObservation_desc,
        "map_observation",
        QosProfile::LidarPointcloud);
    map_writer_ = writer<lingtu_dds_PointCloud2>(
        lingtu::message::kSlamMapCloud.dds_topic.data(), &lingtu_dds_PointCloud2_desc, "map_cloud",
        QosProfile::LidarPointcloud);
    saved_map_writer_ = writer<lingtu_dds_PointCloud2>(
        lingtu::message::kSlamSavedMapCloud.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "saved_map_cloud");
    map_event_writer_ = writer<lingtu_dds_Text>(
        lingtu::message::kSlamMapEvent.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "map_event");
    relocalization_response_writer_ = writer<lingtu_dds_RelocalizationResponse>(
        lingtu::message::kSlamRelocalizationResponse.dds_topic.data(),
        &lingtu_dds_RelocalizationResponse_desc,
        "relocalization_response");
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
        imu_reader_,
        lingtu_dds_Imu_desc,
        std::forward<Handler>(handler),
        kSensorStreamCatchupBatches);
  }

  template <typename Handler>
  void drainLidar(Handler&& handler) {
    drainLatestReader<lingtu_dds_LivoxFrame>(
        lidar_reader_,
        lingtu_dds_LivoxFrame_desc,
        std::forward<Handler>(handler),
        kSensorStreamCatchupBatches);
  }

  template <typename Handler>
  void drainOdomPrior(Handler&& handler) {
    drainReader<lingtu_dds_Odometry>(
        odom_prior_reader_,
        lingtu_dds_Odometry_desc,
        std::forward<Handler>(handler),
        16);
  }

  template <typename Handler>
  void drainMapCommands(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        map_command_reader_, lingtu_dds_Text_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainRelocalizationRequests(Handler&& handler) {
    drainReader<lingtu_dds_RelocalizationRequest>(
        relocalization_request_reader_,
        lingtu_dds_RelocalizationRequest_desc,
        std::forward<Handler>(handler));
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

  void writeMapObservation(MapObservationMessage& message) {
    message.bindStorage();
    logDdsError(
        dds_write(map_observation_writer_, &message.msg),
        "dds_write(map_observation)");
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

  void writeRelocalizationResponse(const lingtu_dds_RelocalizationResponse& response) {
    logDdsError(
        dds_write(relocalization_response_writer_, &response),
        "dds_write(relocalization_response)");
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
      QosProfile qos_kind = QosProfile::Default) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    topics_.push_back(topic);
    auto qos = make_qos(qos_kind);
    return checked(
        dds_create_reader(subscriber_, topic, qos.get(), nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
  }

  template <typename T>
  dds_entity_t writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label,
      QosProfile qos_kind = QosProfile::Default) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    topics_.push_back(topic);
    auto qos = make_qos(qos_kind);
    return checked(
        dds_create_writer(publisher_, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t participant_ = DDS_RETCODE_ERROR;
  dds_entity_t subscriber_ = DDS_RETCODE_ERROR;
  dds_entity_t publisher_ = DDS_RETCODE_ERROR;
  dds_entity_t lidar_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t imu_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t odom_prior_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t map_command_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t relocalization_request_reader_ = DDS_RETCODE_ERROR;
  dds_entity_t tf_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t tf_static_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t odom_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t state_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t registered_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t map_observation_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t map_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t saved_map_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t map_event_writer_ = DDS_RETCODE_ERROR;
  dds_entity_t relocalization_response_writer_ = DDS_RETCODE_ERROR;
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
    const SlamMode runtime_mode = modeFromString(cli.mode);
    status = backend->setMode(runtime_mode, cli.map_path);
    if (!status.ok) {
      std::fprintf(stderr, "SLAM mode set returned: %s\n", status.message.c_str());
    }

    DdsRuntime dds(cli.domain_id);
    const double hz = std::max(1.0, cli.tick_hz);
    const auto period = std::chrono::duration<double>(1.0 / hz);
    double last_log_s = 0.0;
    double last_status_json_s = 0.0;
    double last_cloud_snapshot_s = 0.0;
    double last_lidar_scan_snapshot_s = 0.0;
    double last_registered_cloud_stamp_s = -1.0;
    double last_map_cloud_stamp_s = -1.0;
    double last_odometry_stamp_s = -1.0;
    double last_state_estimation_stamp_s = -1.0;
    std::uint64_t last_map_observation_epoch = 0U;
    std::uint64_t last_map_observation_sequence = 0U;
    double last_lidar_scan_snapshot_stamp_s = -1.0;
    const double status_json_period_s = cli.status_json_hz > 0.0
        ? 1.0 / cli.status_json_hz
        : 0.0;
    const double cloud_snapshot_period_s = cli.cloud_snapshot_hz > 0.0
        ? 1.0 / cli.cloud_snapshot_hz
        : 0.0;
    const double lidar_scan_snapshot_period_s = cli.lidar_scan_snapshot_hz > 0.0
        ? 1.0 / cli.lidar_scan_snapshot_hz
        : 0.0;
    RateCounter imu_input_rate;
    RateCounter lidar_input_rate;
    RateCounter slam_tick_rate;
    RateCounter processed_scan_rate;
    std::optional<Pose3d> startup_track_seed =
        loadTrackSeed(cli.track_against_map_seed_file, cli.map_path);
    if (!startup_track_seed.has_value()) {
      startup_track_seed = cli.track_against_map_initial_pose;
    }
    bool track_against_map_enabled =
        runtime_mode == SlamMode::Localization &&
        !cli.map_path.empty();
    std::optional<Pose3d> track_against_map_seed = startup_track_seed;
    int track_against_map_failures = 0;
    std::uint64_t track_against_map_attempts = 0;
    std::uint64_t track_against_map_successes = 0;
    std::uint64_t track_against_map_rejections = 0;
    std::uint64_t track_against_map_waits = 0;
    double last_track_against_map_success_s = -1.0;
    double last_track_against_map_s = 0.0;
    double last_track_against_map_scan_s = -1.0;
    const double track_against_map_period_s =
        std::max(1.0, cli.track_against_map_period_s);
    constexpr int kTrackAgainstMapDegradedFailureCount = 3;
    auto note_track_failure = [&](const std::string& message) {
      ++track_against_map_failures;
      ++track_against_map_rejections;
      std::fprintf(stderr, "track_against_map: %s\n", message.c_str());
      if (track_against_map_failures == kTrackAgainstMapDegradedFailureCount) {
        std::fprintf(
            stderr,
            "track_against_map degraded after repeated failures; retaining the seed and retrying\n");
      } else if (
          track_against_map_failures > kTrackAgainstMapDegradedFailureCount &&
          track_against_map_failures % 10 == 0) {
        std::fprintf(
            stderr,
            "track_against_map still degraded after %d consecutive failures; retrying\n",
            track_against_map_failures);
      }
    };
    auto note_track_wait = [&](const std::string& message) {
      ++track_against_map_waits;
      std::fprintf(stderr, "track_against_map waiting: %s\n", message.c_str());
    };
    auto restart_track_against_map = [&]() {
      track_against_map_enabled = true;
      track_against_map_failures = 0;
      ++track_against_map_successes;
      last_track_against_map_success_s = nowSeconds();
      last_track_against_map_s = last_track_against_map_success_s;
      last_track_against_map_scan_s = -1.0;
      track_against_map_seed.reset();
    };
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
        const LidarFrame frame = toLidarFrame(msg);
        if (!cli.cloud_snapshot_dir.empty() && lidar_scan_snapshot_period_s > 0.0) {
          const bool first_snapshot = last_lidar_scan_snapshot_stamp_s < 0.0;
          const bool new_frame =
              std::abs(frame.stamp_s - last_lidar_scan_snapshot_stamp_s) > 1e-6;
          const double frame_dt_s =
              first_snapshot ? lidar_scan_snapshot_period_s
                             : frame.stamp_s - last_lidar_scan_snapshot_stamp_s;
          if (new_frame &&
              (first_snapshot || frame_dt_s >= lidar_scan_snapshot_period_s * 0.9)) {
            last_lidar_scan_snapshot_s = nowSeconds();
            last_lidar_scan_snapshot_stamp_s = frame.stamp_s;
            writeLidarScanSnapshotAtomic(
                cli.cloud_snapshot_dir + "/lidar_scan.bin",
                frame);
          }
        }
        const Status s = backend->feedLidar(frame);
        if (!s.ok) {
          std::fprintf(stderr, "feedLidar: %s\n", s.message.c_str());
        }
      });
      dds.drainOdomPrior([&](const lingtu_dds_Odometry& msg) {
        const Status s = backend->feedVisualOdom(toOdomSample(msg));
        if (!s.ok) {
          std::fprintf(stderr, "feedOdomPrior: %s\n", s.message.c_str());
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
            runtime_mode != SlamMode::Localization) {
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
        if (command_status.ok &&
            (command.action == "relocalize" || command.action == "global_relocalize")) {
          restart_track_against_map();
        }

        const SlamOutputs out = backend->outputs();
        if (command_status.ok &&
            (command.action == "relocalize" || command.action == "global_relocalize")) {
          saveTrackSeed(cli.track_against_map_seed_file, cli.map_path, out.relocalization_map_body);
        }
        if (command_status.ok && out.saved_map_cloud_map.has_value()) {
          auto saved_msg = toDdsCloud(*out.saved_map_cloud_map);
          saved_msg.bindStorage();
          dds.writeSavedMap(saved_msg.msg);
        }
        dds.writeMapEvent(mapEventJson(
            command,
            command_status.ok,
            command_status.message,
            command.path,
            &out));
      });
      dds.drainRelocalizationRequests([&](const lingtu_dds_RelocalizationRequest& request) {
        const std::string action =
            normalizedRelocalizationAction(request.action ? request.action : "");
        Status command_status;
        if (action == "query_status") {
          const SlamOutputs out = backend->outputs();
          auto response = relocalizationResponse(
              request,
              true,
              "status",
              out,
              runtime_mode == SlamMode::Localization,
              track_against_map_enabled,
              track_against_map_failures);
          dds.writeRelocalizationResponse(response.msg);
          return;
        }
        if (action == "track_against_map") {
          const std::string map_path = request.map_path ? request.map_path : "";
          if (runtime_mode != SlamMode::Localization) {
            const SlamOutputs out = backend->outputs();
            auto response = relocalizationResponse(
                request,
                false,
                "localization_mode_required",
                out,
                false);
            dds.writeRelocalizationResponse(response.msg);
            return;
          }
          if (!map_path.empty()) {
            const Status load_status = backend->loadMap(map_path);
            if (!load_status.ok) {
              const SlamOutputs out = backend->outputs();
              auto response = relocalizationResponse(
                  request,
                  false,
                  load_status.message,
                  out,
                  false);
              dds.writeRelocalizationResponse(response.msg);
              return;
            }
          }
          track_against_map_enabled = true;
          track_against_map_failures = 0;
          track_against_map_successes = 0;
          last_track_against_map_success_s = -1.0;
          last_track_against_map_s = 0.0;
          last_track_against_map_scan_s = -1.0;
          track_against_map_seed = request.has_initial_pose
              ? std::optional<Pose3d>{poseFromDds(request.initial_pose)}
              : std::optional<Pose3d>{};
          const SlamOutputs out = backend->outputs();
          auto response = relocalizationResponse(
              request,
              true,
              "track_against_map_started",
              out,
              true,
              true,
              track_against_map_failures);
          dds.writeRelocalizationResponse(response.msg);
          return;
        }
        if (action != "load_map" && action != "seeded_relocalize" &&
            action != "global_relocalize") {
          const SlamOutputs out = backend->outputs();
          auto response = relocalizationResponse(
              request,
              false,
              "unsupported_relocalization_action",
              out);
          dds.writeRelocalizationResponse(response.msg);
          return;
        }

        const std::string map_path = request.map_path ? request.map_path : "";
        if (action == "load_map" && map_path.empty()) {
          const SlamOutputs out = backend->outputs();
          auto response = relocalizationResponse(request, false, "missing_map_path", out);
          dds.writeRelocalizationResponse(response.msg);
          return;
        }
        if ((action == "seeded_relocalize" || action == "global_relocalize") &&
            runtime_mode != SlamMode::Localization) {
          const SlamOutputs out = backend->outputs();
          auto response = relocalizationResponse(
              request,
              false,
              "localization_mode_required",
              out);
          dds.writeRelocalizationResponse(response.msg);
          return;
        }
        if (action == "seeded_relocalize" && !request.has_initial_pose) {
          const SlamOutputs out = backend->outputs();
          auto response = relocalizationResponse(
              request,
              false,
              "missing_initial_pose",
              out);
          dds.writeRelocalizationResponse(response.msg);
          return;
        }

        if (action == "load_map") {
          command_status = backend->loadMap(map_path);
        } else {
          if (!map_path.empty()) {
            const Status load_status = backend->loadMap(map_path);
            if (!load_status.ok) {
              const SlamOutputs out = backend->outputs();
              auto response = relocalizationResponse(request, false, load_status.message, out);
              dds.writeRelocalizationResponse(response.msg);
              return;
            }
          }
          command_status = backend->relocalize(
              action == "global_relocalize"
                  ? std::optional<Pose3d>{}
                  : std::optional<Pose3d>{poseFromDds(request.initial_pose)});
        }
        if (command_status.ok &&
            (action == "seeded_relocalize" || action == "global_relocalize")) {
          restart_track_against_map();
        }

        const SlamOutputs out = backend->outputs();
        if (command_status.ok &&
            (action == "seeded_relocalize" || action == "global_relocalize")) {
          saveTrackSeed(cli.track_against_map_seed_file, cli.map_path, out.relocalization_map_body);
        }
        if (command_status.ok && out.saved_map_cloud_map.has_value()) {
          auto saved_msg = toDdsCloud(*out.saved_map_cloud_map);
          saved_msg.bindStorage();
          dds.writeSavedMap(saved_msg.msg);
        }
        auto response =
            relocalizationResponse(
                request,
                command_status.ok,
                command_status.message,
                out,
                runtime_mode == SlamMode::Localization,
                track_against_map_enabled,
                track_against_map_failures);
        dds.writeRelocalizationResponse(response.msg);
      });

      status = backend->tick();
      if (!status.ok) {
        std::fprintf(stderr, "tick: %s\n", status.message.c_str());
      }
      slam_tick_rate.mark(nowSeconds());
      SlamOutputs out = backend->outputs();
      if (track_against_map_enabled) {
        const double t = nowSeconds();
        if (const auto completed = backend->pollRelocalizeAsync(); completed.has_value()) {
          if (completed->ok) {
            track_against_map_seed.reset();
            track_against_map_failures = 0;
            ++track_against_map_successes;
            last_track_against_map_success_s = t;
          } else if (isTrackAgainstMapInputWait(completed->message)) {
            note_track_wait(completed->message);
          } else {
            note_track_failure(completed->message);
          }
          out = backend->outputs();
          if (completed->ok) {
            saveTrackSeed(
                cli.track_against_map_seed_file,
                cli.map_path,
                out.relocalization_map_body);
          }
        }
        if (t - last_track_against_map_s >= track_against_map_period_s) {
          last_track_against_map_s = t;
          if (backend->relocalizeAsyncInFlight()) {
            note_track_wait("async_relocalization_in_progress");
          } else if (!out.registered_cloud_body.has_value()) {
            note_track_wait("registered_cloud_unavailable");
          } else if (std::abs(
                         out.registered_cloud_body->stamp_s -
                         last_track_against_map_scan_s) <= 1e-6) {
            note_track_wait("registered_cloud_stale");
          } else {
            last_track_against_map_scan_s = out.registered_cloud_body->stamp_s;
            ++track_against_map_attempts;
            const Status start_status =
                backend->startRelocalizeAsync(track_against_map_seed);
            if (!start_status.ok && isTrackAgainstMapInputWait(start_status.message)) {
              note_track_wait(start_status.message);
            } else if (!start_status.ok) {
              note_track_failure(start_status.message);
            } else {
              std::fprintf(
                  stderr,
                  "track_against_map async start scan_sequence=%llu stamp=%.6f\n",
                  static_cast<unsigned long long>(out.observation_sequence),
                  out.registered_cloud_body->stamp_s);
            }
          }
        }
      }
      const double tracking_status_t = nowSeconds();
      const RuntimeMapTrackingStatus tracking_status{
          track_against_map_enabled,
          track_against_map_period_s,
          track_against_map_failures,
          track_against_map_attempts,
          track_against_map_successes,
          track_against_map_rejections,
          track_against_map_waits,
          last_track_against_map_success_s > 0.0
              ? std::max(0.0, tracking_status_t - last_track_against_map_success_s)
              : -1.0,
          backend->relocalizeAsyncInFlight()};
      const bool map_tf_publish_allowed =
          runtime_mode != SlamMode::Localization || track_against_map_enabled;
      if (map_tf_publish_allowed && out.map_odom_tf.has_value()) {
        const auto msg = toDdsTfMessage(*out.map_odom_tf, out.stamp_s);
        dds.writeTf(msg.msg);
      }
      if (out.odometry_odom_body.has_value() &&
          std::abs(out.stamp_s - last_odometry_stamp_s) > 1e-6) {
        last_odometry_stamp_s = out.stamp_s;
        const auto msg = toDdsOdom(
            *out.odometry_odom_body,
            out.stamp_s,
            out.fastlio_velocity_x,
            out.fastlio_velocity_y,
            out.fastlio_velocity_z,
            "odom",
            "body");
        dds.writeOdom(msg);
      }
      if (out.state_estimation_at_scan.has_value() &&
          std::abs(out.stamp_s - last_state_estimation_stamp_s) > 1e-6) {
        last_state_estimation_stamp_s = out.stamp_s;
        const auto msg = toDdsOdom(
            *out.state_estimation_at_scan,
            out.stamp_s,
            out.fastlio_velocity_x,
            out.fastlio_velocity_y,
            out.fastlio_velocity_z,
            "odom",
            "body");
        dds.writeState(msg);
      }
      if (out.registered_cloud_body.has_value() &&
          std::abs(
              out.registered_cloud_body->stamp_s - last_registered_cloud_stamp_s) > 1e-6) {
        last_registered_cloud_stamp_s = out.registered_cloud_body->stamp_s;
        auto msg = toDdsCloud(*out.registered_cloud_body);
        msg.bindStorage();
        dds.writeRegistered(msg.msg);
      }
      const bool observation_is_new =
          out.source_epoch > last_map_observation_epoch ||
          (out.source_epoch == last_map_observation_epoch &&
           out.observation_sequence > last_map_observation_sequence);
      if (out.source_epoch > 0U && out.observation_sequence > 0U &&
          observation_is_new) {
        if (auto observation = toDdsMapObservation(out); observation.has_value()) {
          dds.writeMapObservation(*observation);
          last_map_observation_epoch = out.source_epoch;
          last_map_observation_sequence = out.observation_sequence;
        }
      }
      if (out.map_cloud_map.has_value() &&
          std::abs(out.map_cloud_map->stamp_s - last_map_cloud_stamp_s) >
              1e-6) {
        last_map_cloud_stamp_s = out.map_cloud_map->stamp_s;
        auto msg = toDdsCloud(*out.map_cloud_map);
        msg.bindStorage();
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
      dds.writeHealth(healthJson(out, tracking_status));
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
              statusSnapshotJson(
                  out,
                  config.backend,
                  cli.mode,
                  rates,
                  tracking_status));
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
              "map_points=%d odom=%d odom_prior=%d prior_age=%.3f prior_error_xy=%.3f "
              "prior_map_points=%d x=%.3f y=%.3f z=%.3f stamp=%.3f\n",
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
              out.odom_prior_active ? 1 : 0,
              out.odom_prior_age_s,
              out.odom_prior_error_xy_m,
              out.odom_prior_map_points,
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
