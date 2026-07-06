#include "message/cpp/dds_topics.hpp"
#include "nav_loop.hpp"
#include "nav_status_writer.hpp"
#include "octoplanner3d_core.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::atomic_bool g_running{true};
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::PlanDiagnostics;
using lingtu::nav::endpoint::StatusWriterConfig;
using lingtu::nav::endpoint::writeStatusSnapshot;

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

double yawFromQuaternion(const lingtu_dds_Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

lingtu_dds_Quaternion quaternionFromYaw(double yaw) {
  lingtu_dds_Quaternion q{};
  const double half = yaw * 0.5;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

std::string headerFrameId(const lingtu_dds_Header& header) {
  return header.frame_id == nullptr ? std::string{} : std::string(header.frame_id);
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

struct FieldOffsets {
  int x{-1};
  int y{-1};
  int z{-1};
  int height{-1};
  int intensity{-1};
};

struct RigidTransform {
  nav_kernel::Vec3 translation{};
  lingtu_dds_Quaternion rotation{};
  double yaw{0.0};
  double stamp_s{0.0};
  bool valid{false};
};

lingtu_dds_Quaternion normalized(lingtu_dds_Quaternion q) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm <= 1e-12) {
    q = {};
    q.w = 1.0;
    return q;
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return q;
}

nav_kernel::Vec3 rotateVector(const lingtu_dds_Quaternion& raw_q, const nav_kernel::Vec3& v) {
  const auto q = normalized(raw_q);
  const double tx = 2.0 * (q.y * v.z - q.z * v.y);
  const double ty = 2.0 * (q.z * v.x - q.x * v.z);
  const double tz = 2.0 * (q.x * v.y - q.y * v.x);
  return {
      v.x + q.w * tx + (q.y * tz - q.z * ty),
      v.y + q.w * ty + (q.z * tx - q.x * tz),
      v.z + q.w * tz + (q.x * ty - q.y * tx),
  };
}

nav_kernel::Vec3 transformPoint(const RigidTransform& tf, const nav_kernel::Vec3& p) {
  const auto rotated = rotateVector(tf.rotation, p);
  return {
      tf.translation.x + rotated.x,
      tf.translation.y + rotated.y,
      tf.translation.z + rotated.z,
  };
}

nav_kernel::Pose transformPose(const RigidTransform& tf, const nav_kernel::Pose& pose) {
  nav_kernel::Pose out;
  out.position = transformPoint(tf, pose.position);
  out.yaw = tf.yaw + pose.yaw;
  return out;
}

RigidTransform inverseTransform(const RigidTransform& tf) {
  RigidTransform inv;
  inv.rotation = normalized(tf.rotation);
  inv.rotation.x = -inv.rotation.x;
  inv.rotation.y = -inv.rotation.y;
  inv.rotation.z = -inv.rotation.z;
  inv.translation = rotateVector(
      inv.rotation,
      {-tf.translation.x, -tf.translation.y, -tf.translation.z});
  inv.yaw = -tf.yaw;
  inv.stamp_s = tf.stamp_s;
  inv.valid = tf.valid;
  return inv;
}

std::optional<RigidTransform> mapOdomTransformFromTf(const lingtu_dds_TFMessage& msg) {
  for (std::uint32_t i = 0; i < msg.transforms._length; ++i) {
    const auto& item = msg.transforms._buffer[i];
    const std::string parent = headerFrameId(item.header);
    const std::string child = item.child_frame_id == nullptr
        ? std::string{}
        : std::string(item.child_frame_id);
    RigidTransform tf;
    tf.translation = {
        item.transform.translation.x,
        item.transform.translation.y,
        item.transform.translation.z,
    };
    tf.rotation = normalized(item.transform.rotation);
    tf.yaw = yawFromQuaternion(tf.rotation);
    tf.stamp_s = stampSeconds(item.header.stamp);
    tf.valid = true;
    if (parent == "map" && child == "odom") {
      return tf;
    }
    if (parent == "odom" && child == "map") {
      return inverseTransform(tf);
    }
  }
  return std::nullopt;
}

FieldOffsets fieldOffsets(const lingtu_dds_PointCloud2& msg) {
  FieldOffsets offsets;
  for (std::uint32_t i = 0; i < msg.fields._length; ++i) {
    const auto& field = msg.fields._buffer[i];
    const std::string name = field.name ? field.name : "";
    if (name == "x") {
      offsets.x = static_cast<int>(field.offset);
    } else if (name == "y") {
      offsets.y = static_cast<int>(field.offset);
    } else if (name == "z") {
      offsets.z = static_cast<int>(field.offset);
    } else if (name == "height" || name == "terrain_height") {
      offsets.height = static_cast<int>(field.offset);
    } else if (name == "intensity") {
      offsets.intensity = static_cast<int>(field.offset);
    }
  }
  return offsets;
}

float readFloat(const std::uint8_t* data) {
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

std::vector<float> cloudToXyzh(
    const lingtu_dds_PointCloud2& msg,
    const std::size_t max_points,
    const std::optional<nav_kernel::Pose>& map_body,
    const std::optional<RigidTransform>& map_odom) {
  std::vector<float> out;
  const FieldOffsets offsets = fieldOffsets(msg);
  if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0 ||
      msg.point_step < 12 || msg.data._buffer == nullptr) {
    return out;
  }
  const std::size_t count =
      std::min<std::size_t>(msg.width * std::max<std::uint32_t>(1, msg.height),
                            msg.data._length / std::max<std::uint32_t>(1, msg.point_step));
  if (count == 0) {
    return out;
  }
  const std::size_t stride = max_points > 0 && count > max_points
      ? static_cast<std::size_t>(std::ceil(static_cast<double>(count) / max_points))
      : 1;
  const std::string frame_id = headerFrameId(msg.header);
  const bool map_frame = frame_id == "map";
  const bool odom_frame = frame_id == "odom";
  if (!map_frame && !odom_frame && !map_body) {
    return out;
  }
  if (odom_frame && !map_odom) {
    return out;
  }
  const double c = map_body ? std::cos(map_body->yaw) : 1.0;
  const double s = map_body ? std::sin(map_body->yaw) : 0.0;
  out.reserve((count / stride + 1) * 4);
  for (std::size_t i = 0; i < count; i += stride) {
    const auto* base = msg.data._buffer + i * msg.point_step;
    const float x = readFloat(base + offsets.x);
    const float y = readFloat(base + offsets.y);
    const float z = readFloat(base + offsets.z);
    const bool has_height = offsets.height >= 0;
    const float raw_height = has_height ? readFloat(base + offsets.height) : 0.0f;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    double world_z = z;
    float height = z;
    if (map_frame) {
      out.push_back(x);
      out.push_back(y);
      out.push_back(z);
      world_z = z;
    } else if (odom_frame && map_odom) {
      const auto p = transformPoint(*map_odom, {x, y, z});
      out.push_back(static_cast<float>(p.x));
      out.push_back(static_cast<float>(p.y));
      out.push_back(static_cast<float>(p.z));
      world_z = p.z;
    } else {
      out.push_back(static_cast<float>(
          map_body->position.x + c * x - s * y));
      out.push_back(static_cast<float>(
          map_body->position.y + s * x + c * y));
      out.push_back(static_cast<float>(map_body->position.z + z));
      world_z = map_body->position.z + z;
    }
    if (has_height) {
      height = raw_height;
    } else if (map_body) {
      height = static_cast<float>(world_z - map_body->position.z);
    }
    out.push_back(std::isfinite(height) ? height : z);
  }
  return out;
}

void appendXyzhCloud(
    std::vector<float>& out,
    const std::vector<float>& in,
    std::size_t max_points) {
  const std::size_t current_points = out.size() / 4;
  if (max_points > 0 && current_points >= max_points) {
    return;
  }
  const std::size_t input_points = in.size() / 4;
  if (input_points == 0) {
    return;
  }
  const std::size_t remaining = max_points == 0 ? input_points : max_points - current_points;
  const std::size_t stride = remaining > 0 && input_points > remaining
      ? static_cast<std::size_t>(std::ceil(static_cast<double>(input_points) / remaining))
      : 1;
  out.reserve(out.size() + std::min(input_points, remaining) * 4);
  std::size_t added = 0;
  for (std::size_t i = 0; i < input_points && (max_points == 0 || added < remaining);
       i += stride) {
    const std::size_t base = i * 4;
    out.push_back(in[base + 0]);
    out.push_back(in[base + 1]);
    out.push_back(in[base + 2]);
    out.push_back(in[base + 3]);
    ++added;
  }
}

nav_kernel::Pose toPose(const lingtu_dds_Odometry& msg) {
  nav_kernel::Pose pose;
  pose.position = {
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z,
  };
  pose.yaw = yawFromQuaternion(msg.pose.pose.orientation);
  return pose;
}

std::vector<nav_kernel::Vec3> toPath(const lingtu_dds_Path& msg) {
  std::vector<nav_kernel::Vec3> path;
  path.reserve(msg.poses._length);
  for (std::uint32_t i = 0; i < msg.poses._length; ++i) {
    const auto& pose = msg.poses._buffer[i].pose.position;
    path.push_back({pose.x, pose.y, pose.z});
  }
  return path;
}

nav_kernel::Vec3 toGoalPoint(const lingtu_dds_PoseStamped& msg) {
  return {
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z,
  };
}

std::vector<nav_kernel::Vec3> toNavPath(
    const std::vector<octoplanner3d::runtime::Point>& path) {
  std::vector<nav_kernel::Vec3> out;
  out.reserve(path.size());
  for (const auto& point : path) {
    out.push_back({point.x, point.y, point.z});
  }
  return out;
}

struct PathMessage {
  lingtu_dds_Path msg{};
  std::vector<lingtu_dds_PoseStamped> poses;
};

PathMessage toDdsPath(
    const std::vector<nav_kernel::Vec3>& path,
    const char* frame_id) {
  PathMessage out;
  const double stamp = nowSeconds();
  fillHeader(out.msg.header, stamp, frame_id);
  out.poses.resize(path.size());
  for (std::size_t i = 0; i < path.size(); ++i) {
    fillHeader(out.poses[i].header, stamp, frame_id);
    out.poses[i].pose.position.x = path[i].x;
    out.poses[i].pose.position.y = path[i].y;
    out.poses[i].pose.position.z = path[i].z;
    out.poses[i].pose.orientation = quaternionFromYaw(0.0);
  }
  out.msg.poses._maximum = static_cast<std::uint32_t>(out.poses.size());
  out.msg.poses._length = static_cast<std::uint32_t>(out.poses.size());
  out.msg.poses._buffer = out.poses.data();
  out.msg.poses._release = false;
  return out;
}

lingtu_dds_TwistStamped toDdsTwist(const nav_kernel::Twist& cmd) {
  lingtu_dds_TwistStamped out{};
  fillHeader(out.header, nowSeconds(), "base_link");
  out.twist.linear.x = cmd.vx;
  out.twist.linear.y = cmd.vy;
  out.twist.angular.z = cmd.wz;
  return out;
}

lingtu_dds_PoseStamped toDdsPoseStamped(const nav_kernel::Vec3& point, const char* frame_id) {
  lingtu_dds_PoseStamped out{};
  fillHeader(out.header, nowSeconds(), frame_id);
  out.pose.position.x = point.x;
  out.pose.position.y = point.y;
  out.pose.position.z = point.z;
  out.pose.orientation = quaternionFromYaw(0.0);
  return out;
}

struct CliConfig {
  int domain_id{0};
  double tick_hz{20.0};
  double status_s{5.0};
  double traversability_max_age_s{1.5};
  double terrain_map_max_age_s{0.5};
  double traversability_hard_cost{101.0};
  double traversability_soft_cost{40.0};
  double traversability_weight{0.01};
  std::size_t max_obstacle_points{20000};
  bool publish_cmd_vel{true};
  bool check_obstacle{true};
  bool use_traversability_cost{false};
  double vehicle_length_m{1.0};
  double vehicle_width_m{0.6};
  double sensor_offset_x_m{0.0};
  double sensor_offset_y_m{0.0};
  std::string path_library_dir;
  std::string map_path;
  std::string status_file;
};

struct TraversabilityGrid {
  std::vector<float> values;
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};

  lingtu::nav::plan::TraversabilityGridView view() const {
    if (values.empty() || rows <= 0 || cols <= 0 || resolution <= 0.0) {
      return {};
    }
    return {
        values.data(),
        rows,
        cols,
        resolution,
        origin_x,
        origin_y,
    };
  }
};

std::string envOrEmpty(const char* name) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

bool parseBool(const std::string& raw, const char* name) {
  std::string value = raw;
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::runtime_error(std::string(name) + " expects true/false or 1/0");
}

std::string textData(const lingtu_dds_Text& msg) {
  return msg.data == nullptr ? std::string{} : std::string(msg.data);
}

double vecDistance(const nav_kernel::Vec3& a, const nav_kernel::Vec3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

CliConfig parseArgs(int argc, char** argv) {
  CliConfig cfg;
  cfg.path_library_dir = envOrEmpty("LINGTU_LOCAL_PLANNER_PATHS");
  cfg.map_path = envOrEmpty("LINGTU_ACTIVE_OCTOMAP");
  cfg.status_file = envOrEmpty("LINGTU_NAV_STATUS_FILE");
  const std::string publish_cmd_vel = envOrEmpty("LINGTU_NAV_PUBLISH_CMD_VEL");
  if (!publish_cmd_vel.empty()) {
    cfg.publish_cmd_vel = parseBool(publish_cmd_vel, "LINGTU_NAV_PUBLISH_CMD_VEL");
  }
  const std::string check_obstacle = envOrEmpty("LINGTU_NAV_CHECK_OBSTACLE");
  if (!check_obstacle.empty()) {
    cfg.check_obstacle = parseBool(check_obstacle, "LINGTU_NAV_CHECK_OBSTACLE");
  }
  const std::string use_traversability =
      envOrEmpty("LINGTU_NAV_USE_TRAVERSABILITY_COST");
  if (!use_traversability.empty()) {
    cfg.use_traversability_cost =
        parseBool(use_traversability, "LINGTU_NAV_USE_TRAVERSABILITY_COST");
  }
  const std::string traversability_max_age =
      envOrEmpty("LINGTU_NAV_TRAVERSABILITY_MAX_AGE_S");
  if (!traversability_max_age.empty()) {
    cfg.traversability_max_age_s = std::stod(traversability_max_age);
  }
  const std::string terrain_map_max_age =
      envOrEmpty("LINGTU_NAV_TERRAIN_MAP_MAX_AGE_S");
  if (!terrain_map_max_age.empty()) {
    cfg.terrain_map_max_age_s = std::stod(terrain_map_max_age);
  }
  const std::string traversability_hard_cost =
      envOrEmpty("LINGTU_NAV_TRAVERSABILITY_HARD_COST");
  if (!traversability_hard_cost.empty()) {
    cfg.traversability_hard_cost = std::stod(traversability_hard_cost);
  }
  const std::string traversability_soft_cost =
      envOrEmpty("LINGTU_NAV_TRAVERSABILITY_SOFT_COST");
  if (!traversability_soft_cost.empty()) {
    cfg.traversability_soft_cost = std::stod(traversability_soft_cost);
  }
  const std::string traversability_weight =
      envOrEmpty("LINGTU_NAV_TRAVERSABILITY_WEIGHT");
  if (!traversability_weight.empty()) {
    cfg.traversability_weight = std::stod(traversability_weight);
  }
  const std::string vehicle_length = envOrEmpty("LINGTU_NAV_VEHICLE_LENGTH_M");
  if (!vehicle_length.empty()) {
    cfg.vehicle_length_m = std::stod(vehicle_length);
  }
  const std::string vehicle_width = envOrEmpty("LINGTU_NAV_VEHICLE_WIDTH_M");
  if (!vehicle_width.empty()) {
    cfg.vehicle_width_m = std::stod(vehicle_width);
  }
  const std::string sensor_offset_x = envOrEmpty("LINGTU_NAV_SENSOR_OFFSET_X_M");
  if (!sensor_offset_x.empty()) {
    cfg.sensor_offset_x_m = std::stod(sensor_offset_x);
  }
  const std::string sensor_offset_y = envOrEmpty("LINGTU_NAV_SENSOR_OFFSET_Y_M");
  if (!sensor_offset_y.empty()) {
    cfg.sensor_offset_y_m = std::stod(sensor_offset_y);
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
    } else if (arg == "--status-s") {
      cfg.status_s = std::stod(next());
    } else if (arg == "--max-obstacle-points") {
      cfg.max_obstacle_points = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--path-library") {
      cfg.path_library_dir = next();
    } else if (arg == "--map") {
      cfg.map_path = next();
    } else if (arg == "--publish-cmd-vel") {
      cfg.publish_cmd_vel = parseBool(next(), "--publish-cmd-vel");
    } else if (arg == "--check-obstacle") {
      cfg.check_obstacle = parseBool(next(), "--check-obstacle");
    } else if (arg == "--use-traversability-cost") {
      cfg.use_traversability_cost = parseBool(next(), "--use-traversability-cost");
    } else if (arg == "--traversability-max-age-s") {
      cfg.traversability_max_age_s = std::stod(next());
    } else if (arg == "--terrain-map-max-age-s") {
      cfg.terrain_map_max_age_s = std::stod(next());
    } else if (arg == "--traversability-hard-cost") {
      cfg.traversability_hard_cost = std::stod(next());
    } else if (arg == "--traversability-soft-cost") {
      cfg.traversability_soft_cost = std::stod(next());
    } else if (arg == "--traversability-weight") {
      cfg.traversability_weight = std::stod(next());
    } else if (arg == "--vehicle-length-m") {
      cfg.vehicle_length_m = std::stod(next());
    } else if (arg == "--vehicle-width-m") {
      cfg.vehicle_width_m = std::stod(next());
    } else if (arg == "--sensor-offset-x-m") {
      cfg.sensor_offset_x_m = std::stod(next());
    } else if (arg == "--sensor-offset-y-m") {
      cfg.sensor_offset_y_m = std::stod(next());
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_nav_native_endpoint --path-library DIR "
          "[--map octomap.ot] [--domain-id N] [--tick-hz HZ] "
          "[--max-obstacle-points N] [--publish-cmd-vel true|false] "
          "[--check-obstacle true|false] "
          "[--use-traversability-cost true|false] [--traversability-max-age-s S] "
          "[--terrain-map-max-age-s S] [--traversability-hard-cost C] "
          "[--traversability-soft-cost C] [--traversability-weight W] "
          "[--vehicle-length-m M] [--vehicle-width-m M] [--sensor-offset-x-m M] "
          "[--sensor-offset-y-m M] [--status-file PATH]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  if (cfg.path_library_dir.empty()) {
    throw std::runtime_error(
        "path library is required; pass --path-library or set LINGTU_LOCAL_PLANNER_PATHS");
  }
  cfg.tick_hz = std::max(1.0, cfg.tick_hz);
  cfg.traversability_max_age_s = std::max(0.0, cfg.traversability_max_age_s);
  cfg.terrain_map_max_age_s = std::max(0.0, cfg.terrain_map_max_age_s);
  cfg.traversability_hard_cost = std::max(0.0, cfg.traversability_hard_cost);
  cfg.traversability_soft_cost = std::max(0.0, cfg.traversability_soft_cost);
  cfg.traversability_weight = std::max(0.0, cfg.traversability_weight);
  cfg.vehicle_length_m = std::max(0.1, cfg.vehicle_length_m);
  cfg.vehicle_width_m = std::max(0.1, cfg.vehicle_width_m);
  return cfg;
}

TraversabilityGrid toTraversabilityGrid(const lingtu_dds_OccupancyGrid& msg) {
  TraversabilityGrid grid;
  grid.rows = static_cast<int>(msg.info.height);
  grid.cols = static_cast<int>(msg.info.width);
  grid.resolution = static_cast<double>(msg.info.resolution);
  grid.origin_x = msg.info.origin.position.x;
  grid.origin_y = msg.info.origin.position.y;
  const std::size_t count = static_cast<std::size_t>(grid.rows) *
                            static_cast<std::size_t>(grid.cols);
  if (grid.rows <= 0 || grid.cols <= 0 || grid.resolution <= 0.0 ||
      msg.data._buffer == nullptr || msg.data._length < count) {
    return {};
  }
  grid.values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto raw = static_cast<unsigned int>(msg.data._buffer[i]);
    grid.values.push_back(static_cast<float>(std::min(raw, 100u)));
  }
  return grid;
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

    odom_reader_ = reader(
        lingtu::message::kSlamOdometry.dds_topic.data(), &lingtu_dds_Odometry_desc, "odom");
    tf_reader_ = reader(
        lingtu::message::kTf.dds_topic.data(), &lingtu_dds_TFMessage_desc, "tf");
    goal_reader_ = reader(
        lingtu::message::kNavGoalPose.dds_topic.data(), &lingtu_dds_PoseStamped_desc, "goal_pose");
    cloud_reader_ = reader(
        lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "registered_cloud");
    terrain_map_reader_ = reader(
        lingtu::message::kNavTerrainMap.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "terrain_map");
    terrain_map_ext_reader_ = reader(
        lingtu::message::kNavTerrainMapExt.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "terrain_map_ext");
    global_path_reader_ = reader(
        lingtu::message::kNavGlobalPath.dds_topic.data(), &lingtu_dds_Path_desc, "global_path");
    traversability_reader_ = reader(
        lingtu::message::kNavTraversability.dds_topic.data(),
        &lingtu_dds_OccupancyGrid_desc,
        "traversability");
    map_clearing_reader_ = reader(
        lingtu::message::kNavMapClearing.dds_topic.data(),
        &lingtu_dds_Bool_desc,
        "map_clearing",
        true);
    cloud_clearing_reader_ = reader(
        lingtu::message::kNavCloudClearing.dds_topic.data(),
        &lingtu_dds_Bool_desc,
        "cloud_clearing",
        true);
    cancel_reader_ = reader(
        lingtu::message::kNavCancel.dds_topic.data(), &lingtu_dds_Text_desc, "cancel");
    instruction_reader_ = reader(
        lingtu::message::kNavSemanticInstruction.dds_topic.data(),
        &lingtu_dds_Text_desc,
        "semantic_instruction");
    local_path_writer_ = writer(
        lingtu::message::kNavLocalPath.dds_topic.data(), &lingtu_dds_Path_desc, "local_path");
    global_path_writer_ = writer(
        lingtu::message::kNavGlobalPath.dds_topic.data(), &lingtu_dds_Path_desc, "global_path");
    way_point_writer_ = writer(
        lingtu::message::kNavWayPoint.dds_topic.data(), &lingtu_dds_PoseStamped_desc, "way_point");
    cmd_vel_writer_ = writer(
        lingtu::message::kNavCmdVel.dds_topic.data(), &lingtu_dds_TwistStamped_desc, "cmd_vel");
  }

  ~DdsRuntime() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  template <typename Handler>
  void drainOdometry(Handler&& handler) {
    drainReader<lingtu_dds_Odometry>(
        odom_reader_, lingtu_dds_Odometry_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTf(Handler&& handler) {
    drainReader<lingtu_dds_TFMessage>(
        tf_reader_, lingtu_dds_TFMessage_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainGoals(Handler&& handler) {
    drainReader<lingtu_dds_PoseStamped>(
        goal_reader_, lingtu_dds_PoseStamped_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCloud(Handler&& handler) {
    drainReader<lingtu_dds_PointCloud2>(
        cloud_reader_, lingtu_dds_PointCloud2_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTerrainMap(Handler&& handler) {
    drainReader<lingtu_dds_PointCloud2>(
        terrain_map_reader_, lingtu_dds_PointCloud2_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTerrainMapExt(Handler&& handler) {
    drainReader<lingtu_dds_PointCloud2>(
        terrain_map_ext_reader_, lingtu_dds_PointCloud2_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainGlobalPath(Handler&& handler) {
    drainReader<lingtu_dds_Path>(
        global_path_reader_, lingtu_dds_Path_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTraversability(Handler&& handler) {
    drainReader<lingtu_dds_OccupancyGrid>(
        traversability_reader_, lingtu_dds_OccupancyGrid_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainMapClearing(Handler&& handler) {
    drainReader<lingtu_dds_Bool>(
        map_clearing_reader_, lingtu_dds_Bool_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCloudClearing(Handler&& handler) {
    drainReader<lingtu_dds_Bool>(
        cloud_clearing_reader_, lingtu_dds_Bool_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCancel(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        cancel_reader_, lingtu_dds_Text_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainInstructions(Handler&& handler) {
    drainReader<lingtu_dds_Text>(
        instruction_reader_, lingtu_dds_Text_desc, std::forward<Handler>(handler));
  }

  void writeLocalPath(const std::vector<nav_kernel::Vec3>& path) {
    PathMessage msg = toDdsPath(path, "map");
    logDdsError(dds_write(local_path_writer_, &msg.msg), "dds_write(local_path)");
  }

  void writeGlobalPath(const std::vector<nav_kernel::Vec3>& path) {
    PathMessage msg = toDdsPath(path, "map");
    logDdsError(dds_write(global_path_writer_, &msg.msg), "dds_write(global_path)");
  }

  void writeWayPoint(const nav_kernel::Vec3& point) {
    lingtu_dds_PoseStamped msg = toDdsPoseStamped(point, "map");
    logDdsError(dds_write(way_point_writer_, &msg), "dds_write(way_point)");
  }

  void writeCmdVel(const nav_kernel::Twist& cmd) {
    lingtu_dds_TwistStamped msg = toDdsTwist(cmd);
    logDdsError(dds_write(cmd_vel_writer_, &msg), "dds_write(cmd_vel)");
  }

 private:
  dds_entity_t reader(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label,
      bool reliable = false) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    dds_qos_t* qos = nullptr;
    if (reliable) {
      qos = dds_create_qos();
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    }
    const dds_entity_t entity = checked(
        dds_create_reader(subscriber_, topic, qos, nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
    if (qos != nullptr) {
      dds_delete_qos(qos);
    }
    return entity;
  }

  dds_entity_t writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    return checked(
        dds_create_writer(publisher_, topic, nullptr, nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t odom_reader_{0};
  dds_entity_t tf_reader_{0};
  dds_entity_t goal_reader_{0};
  dds_entity_t cloud_reader_{0};
  dds_entity_t terrain_map_reader_{0};
  dds_entity_t terrain_map_ext_reader_{0};
  dds_entity_t global_path_reader_{0};
  dds_entity_t traversability_reader_{0};
  dds_entity_t map_clearing_reader_{0};
  dds_entity_t cloud_clearing_reader_{0};
  dds_entity_t cancel_reader_{0};
  dds_entity_t instruction_reader_{0};
  dds_entity_t global_path_writer_{0};
  dds_entity_t local_path_writer_{0};
  dds_entity_t way_point_writer_{0};
  dds_entity_t cmd_vel_writer_{0};
};

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    const CliConfig cfg = parseArgs(argc, argv);
    const StatusWriterConfig status_cfg{
        cfg.domain_id,
        cfg.tick_hz,
        cfg.publish_cmd_vel,
        cfg.check_obstacle,
        cfg.use_traversability_cost,
        cfg.path_library_dir,
        cfg.map_path,
        cfg.status_file,
    };
    DdsRuntime dds(cfg.domain_id);

    lingtu::nav::plan::NavLoopConfig nav_config;
    nav_config.path_library_dir = cfg.path_library_dir;
    nav_config.max_speed = 0.4;
    nav_config.local_planner.autonomySpeed = 0.4;
    nav_config.local_planner.maxSpeed = 1.0;
    nav_config.local_planner.vehicleLength = cfg.vehicle_length_m;
    nav_config.local_planner.vehicleWidth = cfg.vehicle_width_m;
    nav_config.local_planner.sensorOffsetX = cfg.sensor_offset_x_m;
    nav_config.local_planner.sensorOffsetY = cfg.sensor_offset_y_m;
    nav_config.local_planner.useTraversabilityCost = cfg.use_traversability_cost;
    nav_config.local_planner.traversabilityHardCost = cfg.traversability_hard_cost;
    nav_config.local_planner.traversabilitySoftCost = cfg.traversability_soft_cost;
    nav_config.local_planner.traversabilityWeight = cfg.traversability_weight;
    nav_config.path_follower.maxSpeed = 0.4;
    nav_config.path_follower.maxAccel = 1.0;

    lingtu::nav::plan::NavLoop nav(nav_config);
    if (!nav.configure()) {
      throw std::runtime_error("failed to load local planner path library: " + cfg.path_library_dir);
    }

    std::optional<nav_kernel::Pose> odom_body;
    std::optional<nav_kernel::Pose> map_body;
    std::optional<RigidTransform> map_odom_tf;
    std::vector<float> obstacle_xyzh;
    std::vector<float> terrain_xyzh;
    std::vector<float> terrain_ext_xyzh;
    std::vector<float> planner_terrain_xyzh;
    TraversabilityGrid traversability_grid;
    double last_terrain_map_s = 0.0;
    double last_terrain_ext_s = 0.0;
    double last_traversability_s = 0.0;
    PlanDiagnostics last_plan;
    LocalDiagnostics last_local;
    std::vector<nav_kernel::Vec3> last_global_path;
    std::vector<nav_kernel::Vec3> last_local_path;
    std::string last_instruction;
    bool has_path = false;
    bool suppress_next_global_path_echo = false;
    std::uint64_t odom_count = 0;
    std::uint64_t tf_count = 0;
    std::uint64_t goal_count = 0;
    std::uint64_t cancel_count = 0;
    std::uint64_t map_clearing_count = 0;
    std::uint64_t cloud_clearing_count = 0;
    std::uint64_t instruction_count = 0;
    std::uint64_t cloud_count = 0;
    std::uint64_t terrain_map_count = 0;
    std::uint64_t terrain_map_ext_count = 0;
    std::uint64_t traversability_count = 0;
    std::uint64_t path_count = 0;
    std::uint64_t plan_fail_count = 0;
    std::uint64_t output_count = 0;
    std::uint64_t cmd_vel_count = 0;
    double next_status = nowSeconds() + cfg.status_s;
    auto clear_planner_terrain_inputs = [&]() {
      terrain_xyzh.clear();
      terrain_ext_xyzh.clear();
      planner_terrain_xyzh.clear();
      traversability_grid = TraversabilityGrid{};
      last_terrain_map_s = 0.0;
      last_terrain_ext_s = 0.0;
      last_traversability_s = 0.0;
    };

    std::fprintf(
        stderr,
        "lingtu_nav_native_endpoint: domain=%d tick_hz=%.1f path_library=%s\n",
        cfg.domain_id,
        cfg.tick_hz,
        cfg.path_library_dir.c_str());
    std::fprintf(
        stderr,
        "nav_native: publish_cmd_vel=%d check_obstacle=%d use_traversability_cost=%d traversability_max_age_s=%.2f terrain_map_max_age_s=%.2f status_file=%s\n",
        cfg.publish_cmd_vel ? 1 : 0,
        cfg.check_obstacle ? 1 : 0,
        cfg.use_traversability_cost ? 1 : 0,
        cfg.traversability_max_age_s,
        cfg.terrain_map_max_age_s,
        cfg.status_file.empty() ? "(disabled)" : cfg.status_file.c_str());
    if (cfg.map_path.empty()) {
      std::fprintf(
          stderr,
          "nav_native: no --map/LINGTU_ACTIVE_OCTOMAP configured; "
          "goal_pose will be rejected instead of direct fallback\n");
    } else {
      std::fprintf(stderr, "nav_native: active_octomap=%s\n", cfg.map_path.c_str());
    }

    while (g_running) {
      dds.drainTf([&](const lingtu_dds_TFMessage& msg) {
        const auto tf = mapOdomTransformFromTf(msg);
        if (tf) {
          map_odom_tf = *tf;
          if (odom_body) {
            map_body = transformPose(*map_odom_tf, *odom_body);
          }
        }
        ++tf_count;
      });
      dds.drainOdometry([&](const lingtu_dds_Odometry& msg) {
        odom_body = toPose(msg);
        if (headerFrameId(msg.header) == "map") {
          map_body = *odom_body;
        } else if (map_odom_tf) {
          map_body = transformPose(*map_odom_tf, *odom_body);
        } else {
          map_body.reset();
        }
        ++odom_count;
      });
      dds.drainGoals([&](const lingtu_dds_PoseStamped& msg) {
        ++goal_count;
        last_plan = PlanDiagnostics{};
        last_plan.seen = true;
        last_plan.goal = toGoalPoint(msg);
        if (!map_body) {
          last_plan.reason = odom_body ? "map_odom_tf_not_ready" : "odometry_not_ready";
          ++plan_fail_count;
          std::fprintf(
              stderr,
              "nav_native: reject goal, %s\n",
              odom_body ? "map->odom tf is not ready" : "odometry is not ready");
          return;
        }
        last_plan.start = map_body->position;
        if (cfg.map_path.empty()) {
          last_plan.reason = "active_octomap_not_configured";
          ++plan_fail_count;
          std::fprintf(stderr, "nav_native: reject goal, active OctoMap is not configured\n");
          return;
        }
        octoplanner3d::runtime::PlanRequest request;
        request.map_path = cfg.map_path;
        request.start = {
            map_body->position.x,
            map_body->position.y,
            map_body->position.z,
        };
        request.goal = {last_plan.goal.x, last_plan.goal.y, last_plan.goal.z};
        const auto result = octoplanner3d::runtime::runPlan(request);
        last_plan.reached_goal = result.reached_goal;
        last_plan.goal_error_m = result.goal_error_m;
        last_plan.elapsed_ms = result.elapsed_ms;
        if (!result.ok || !result.reached_goal) {
          if (!result.ok) {
            last_plan.reason = "octoplanner3d_failed";
          } else {
            last_plan.reason = "goal_not_reached";
          }
          last_plan.waypoints = result.path.size();
          ++plan_fail_count;
          std::fprintf(
              stderr,
              "nav_native: OctoPlanner3D rejected reason=%s ok=%d reached=%d waypoints=%zu goal_error=%.3f elapsed_ms=%.1f\n",
              last_plan.reason.c_str(),
              result.ok ? 1 : 0,
              result.reached_goal ? 1 : 0,
              result.path.size(),
              result.goal_error_m,
              result.elapsed_ms);
          return;
        }
        auto global_path = toNavPath(result.path);
        if (global_path.empty()) {
          last_plan.reason = "empty_path";
          last_plan.waypoints = 0;
          ++plan_fail_count;
          return;
        }
        if (global_path.size() == 1) {
          if (vecDistance(map_body->position, global_path.front()) > 0.02) {
            global_path.insert(global_path.begin(), map_body->position);
          } else {
            global_path.push_back(last_plan.goal);
          }
        }
        last_plan.waypoints = global_path.size();
        nav.setGlobalPath(global_path);
        dds.writeGlobalPath(global_path);
        last_global_path = global_path;
        last_local_path.clear();
        has_path = true;
        suppress_next_global_path_echo = true;
        last_plan.accepted = true;
        last_plan.reason = "accepted";
        last_local = LocalDiagnostics{};
        ++path_count;
        std::fprintf(
            stderr,
            "nav_native: OctoPlanner3D path accepted waypoints=%zu elapsed_ms=%.1f reached=%d\n",
            last_plan.waypoints,
            result.elapsed_ms,
            result.reached_goal ? 1 : 0);
      });
      dds.drainCloud([&](const lingtu_dds_PointCloud2& msg) {
        obstacle_xyzh = cloudToXyzh(msg, cfg.max_obstacle_points, map_body, map_odom_tf);
        ++cloud_count;
      });
      dds.drainTerrainMap([&](const lingtu_dds_PointCloud2& msg) {
        const auto xyzh = cloudToXyzh(msg, cfg.max_obstacle_points, map_body, map_odom_tf);
        if (!xyzh.empty()) {
          terrain_xyzh = std::move(xyzh);
          last_terrain_map_s = nowSeconds();
          ++terrain_map_count;
        }
      });
      dds.drainTerrainMapExt([&](const lingtu_dds_PointCloud2& msg) {
        const auto xyzh = cloudToXyzh(msg, cfg.max_obstacle_points, map_body, map_odom_tf);
        if (!xyzh.empty()) {
          terrain_ext_xyzh = std::move(xyzh);
          last_terrain_ext_s = nowSeconds();
          ++terrain_map_ext_count;
        }
      });
      dds.drainMapClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        clear_planner_terrain_inputs();
        ++map_clearing_count;
      });
      dds.drainCloudClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        clear_planner_terrain_inputs();
        obstacle_xyzh.clear();
        ++cloud_clearing_count;
      });
      dds.drainGlobalPath([&](const lingtu_dds_Path& msg) {
        const auto path = toPath(msg);
        if (path.size() >= 2) {
          if (suppress_next_global_path_echo && last_plan.accepted &&
              path.size() == last_plan.waypoints) {
            suppress_next_global_path_echo = false;
            return;
          }
          nav.setGlobalPath(path);
          last_global_path = path;
          last_local_path.clear();
          has_path = true;
          last_plan = PlanDiagnostics{};
          last_plan.seen = true;
          last_plan.accepted = true;
          last_plan.reached_goal = true;
          last_plan.reason = "external_global_path";
          last_plan.waypoints = path.size();
          if (map_body) {
            last_plan.start = map_body->position;
          }
          last_plan.goal = path.back();
          last_local = LocalDiagnostics{};
          ++path_count;
        }
      });
      dds.drainTraversability([&](const lingtu_dds_OccupancyGrid& msg) {
        TraversabilityGrid grid = toTraversabilityGrid(msg);
        if (!grid.values.empty()) {
          traversability_grid = std::move(grid);
          last_traversability_s = nowSeconds();
          ++traversability_count;
        }
      });
      dds.drainCancel([&](const lingtu_dds_Text&) {
        ++cancel_count;
        nav.clearGlobalPath();
        last_global_path.clear();
        last_local_path.clear();
        has_path = false;
        last_local = LocalDiagnostics{};
        last_plan.reason = "cancelled";
        if (cfg.publish_cmd_vel) {
          dds.writeCmdVel({});
          ++cmd_vel_count;
        }
      });
      dds.drainInstructions([&](const lingtu_dds_Text& msg) {
        ++instruction_count;
        last_instruction = textData(msg);
        last_plan.seen = true;
        last_plan.accepted = false;
        last_plan.reason = "semantic_instruction_not_supported";
        ++plan_fail_count;
        std::fprintf(
            stderr,
            "nav_native: reject semantic instruction, convert instruction to goal_pose before native nav: %s\n",
            last_instruction.c_str());
      });

      if (map_body && has_path) {
        const double tick_now = nowSeconds();
        const bool traversability_available = !traversability_grid.values.empty();
        const bool traversability_fresh =
            traversability_available &&
            (last_traversability_s > 0.0) &&
            (cfg.traversability_max_age_s <= 0.0 ||
             tick_now - last_traversability_s <= cfg.traversability_max_age_s);
        const auto traversability_view =
            (cfg.check_obstacle && cfg.use_traversability_cost && traversability_fresh)
                ? traversability_grid.view()
                : lingtu::nav::plan::TraversabilityGridView{};
        const bool terrain_map_fresh =
            !terrain_xyzh.empty() &&
            (last_terrain_map_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             tick_now - last_terrain_map_s <= cfg.terrain_map_max_age_s);
        const bool terrain_ext_fresh =
            !terrain_ext_xyzh.empty() &&
            (last_terrain_ext_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             tick_now - last_terrain_ext_s <= cfg.terrain_map_max_age_s);
        const std::vector<float>* planner_obstacles_ptr =
            cfg.check_obstacle ? &obstacle_xyzh : nullptr;
        if (cfg.check_obstacle && (terrain_map_fresh || terrain_ext_fresh)) {
          planner_terrain_xyzh.clear();
          if (terrain_map_fresh) {
            appendXyzhCloud(planner_terrain_xyzh, terrain_xyzh, cfg.max_obstacle_points);
          }
          if (terrain_ext_fresh) {
            appendXyzhCloud(planner_terrain_xyzh, terrain_ext_xyzh, cfg.max_obstacle_points);
          }
          if (!planner_terrain_xyzh.empty()) {
            planner_obstacles_ptr = &planner_terrain_xyzh;
          }
        }
        const bool has_planner_obstacles =
            planner_obstacles_ptr != nullptr && !planner_obstacles_ptr->empty();
        const auto out = nav.tick(
            *map_body,
            has_planner_obstacles ? planner_obstacles_ptr->data() : nullptr,
            has_planner_obstacles ? static_cast<int>(planner_obstacles_ptr->size() / 4) : 0,
            tick_now,
            traversability_view);
        dds.writeLocalPath(out.local_path_map);
        last_local_path = out.local_path_map;
        last_local.seen = true;
        last_local.active = out.active;
        last_local.goal_reached = out.goal_reached;
        last_local.path_found = out.path_found;
        last_local.near_field_stop = out.near_field_stop;
        last_local.reason = out.reason;
        last_local.slow_down = out.slow_down;
        last_local.recovery_state = out.recovery_state;
        last_local.target_index = out.target_index;
        last_local.target_distance_m = out.target_distance_m;
        last_local.target = out.target;
        last_local.local_path_points = out.local_path_map.size();
        last_local.cmd_vel = out.cmd_vel;
        dds.writeWayPoint(out.target);
        if (cfg.publish_cmd_vel) {
          dds.writeCmdVel(out.cmd_vel);
          ++cmd_vel_count;
        }
        ++output_count;
        if (out.goal_reached) {
          has_path = false;
        }
      }

      const double now = nowSeconds();
      if (cfg.status_s > 0.0 && now >= next_status) {
        next_status = now + cfg.status_s;
        const bool traversability_available = !traversability_grid.values.empty();
        const bool traversability_fresh =
            traversability_available &&
            (last_traversability_s > 0.0) &&
            (cfg.traversability_max_age_s <= 0.0 ||
             now - last_traversability_s <= cfg.traversability_max_age_s);
        const bool terrain_map_fresh =
            !terrain_xyzh.empty() &&
            (last_terrain_map_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             now - last_terrain_map_s <= cfg.terrain_map_max_age_s);
        const bool terrain_ext_fresh =
            !terrain_ext_xyzh.empty() &&
            (last_terrain_ext_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             now - last_terrain_ext_s <= cfg.terrain_map_max_age_s);
        if (cfg.check_obstacle && (terrain_map_fresh || terrain_ext_fresh)) {
          planner_terrain_xyzh.clear();
          if (terrain_map_fresh) {
            appendXyzhCloud(planner_terrain_xyzh, terrain_xyzh, cfg.max_obstacle_points);
          }
          if (terrain_ext_fresh) {
            appendXyzhCloud(planner_terrain_xyzh, terrain_ext_xyzh, cfg.max_obstacle_points);
          }
        }
        const std::size_t planner_obstacle_points =
            !cfg.check_obstacle
                ? 0
                : (terrain_map_fresh || terrain_ext_fresh)
                ? planner_terrain_xyzh.size() / 4
                : obstacle_xyzh.size() / 4;
        std::fprintf(
            stderr,
            "nav_native: odom=%llu goals=%llu cancels=%llu instructions=%llu registered_clouds=%llu terrain_maps=%llu terrain_map_exts=%llu traversability=%llu paths=%llu plan_fail=%llu outputs=%llu cmd_vel=%llu obstacle_points=%zu active=%d\n",
            static_cast<unsigned long long>(odom_count),
            static_cast<unsigned long long>(goal_count),
            static_cast<unsigned long long>(cancel_count),
            static_cast<unsigned long long>(instruction_count),
            static_cast<unsigned long long>(cloud_count),
            static_cast<unsigned long long>(terrain_map_count),
            static_cast<unsigned long long>(terrain_map_ext_count),
            static_cast<unsigned long long>(traversability_count),
            static_cast<unsigned long long>(path_count),
            static_cast<unsigned long long>(plan_fail_count),
            static_cast<unsigned long long>(output_count),
            static_cast<unsigned long long>(cmd_vel_count),
            planner_obstacle_points,
            has_path ? 1 : 0);
        writeStatusSnapshot(
            status_cfg,
            now,
            map_body.has_value(),
            map_odom_tf.has_value(),
            has_path,
            cfg.check_obstacle && cfg.use_traversability_cost && traversability_fresh,
            cfg.check_obstacle && terrain_map_fresh,
            cfg.check_obstacle && terrain_ext_fresh,
            odom_count,
            tf_count,
            goal_count,
            cancel_count,
            map_clearing_count,
            cloud_clearing_count,
            instruction_count,
            cloud_count,
            terrain_map_count,
            terrain_map_ext_count,
            traversability_count,
            path_count,
            plan_fail_count,
            output_count,
            cmd_vel_count,
            planner_obstacle_points,
            last_plan,
            last_local,
            last_global_path,
            last_local_path,
            last_instruction);
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(1000.0 / cfg.tick_hz)));
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_nav_native_endpoint failed: %s\n", exc.what());
    return 1;
  }
}
