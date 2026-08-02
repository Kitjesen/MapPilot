#include "nav_endpoint_messages.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <unordered_map>

#include "traversability/point_cloud_layout.hpp"

namespace lingtu::nav::endpoint {
namespace {

double stampSeconds(const lingtu_dds_Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

struct FieldOffsets {
  int x{-1};
  int y{-1};
  int z{-1};
  int height{-1};
  int intensity{-1};
  bool valid{true};
};

FieldOffsets fieldOffsets(const lingtu_dds_PointCloud2 &msg) {
  FieldOffsets offsets;
  if (msg.fields._length > 0 && msg.fields._buffer == nullptr) {
    offsets.valid = false;
    return offsets;
  }
  auto assign = [&](int &target, const lingtu_dds_PointField &field) {
    if (target >= 0 ||
        !pointFieldIsScalarFloat32(field.datatype, field.count, field.offset, msg.point_step)) {
      offsets.valid = false;
      return;
    }
    target = static_cast<int>(field.offset);
  };
  for (std::uint32_t i = 0; i < msg.fields._length; ++i) {
    const auto &field = msg.fields._buffer[i];
    const std::string name = field.name ? field.name : "";
    if (name == "x") {
      assign(offsets.x, field);
    } else if (name == "y") {
      assign(offsets.y, field);
    } else if (name == "z") {
      assign(offsets.z, field);
    } else if (name == "height" || name == "terrain_height") {
      assign(offsets.height, field);
    } else if (name == "intensity") {
      assign(offsets.intensity, field);
    }
  }
  return offsets;
}

float readFloat(const std::uint8_t *data) {
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

struct VoxelKey {
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t h = 1469598103934665603ull;
    auto mix = [&](int value) {
      h ^= static_cast<std::size_t>(value) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
    };
    mix(key.x);
    mix(key.y);
    mix(key.z);
    return h;
  }
};

using VoxelIndex = std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash>;

struct XyzhPoint {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float height{0.0f};
};

double positiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

ObstacleMergeConfig normalizedMergeConfig(ObstacleMergeConfig config) {
  config.voxel_size_m = positiveOr(config.voxel_size_m, 0.08);
  config.registered_share = std::max(0.0, config.registered_share);
  config.terrain_share = std::max(0.0, config.terrain_share);
  config.terrain_ext_share = std::max(0.0, config.terrain_ext_share);
  if (config.registered_share <= 0.0 && config.terrain_share <= 0.0 &&
      config.terrain_ext_share <= 0.0) {
    config.registered_share = 1.0;
  }
  return config;
}

VoxelKey makeVoxelKey(float x, float y, float z, double voxel_size_m) {
  return {
      static_cast<int>(std::floor(static_cast<double>(x) / voxel_size_m)),
      static_cast<int>(std::floor(static_cast<double>(y) / voxel_size_m)),
      static_cast<int>(std::floor(static_cast<double>(z) / voxel_size_m)),
  };
}

using XyzhVoxelMap = std::unordered_map<VoxelKey, XyzhPoint, VoxelKeyHash>;

void keepMaxHeight(XyzhVoxelMap &cells, const VoxelKey &key, const XyzhPoint &point) {
  auto [it, inserted] = cells.emplace(key, point);
  if (!inserted && point.height > it->second.height) {
    it->second = point;
  }
}

XyzhVoxelMap voxelizeXyzh(const std::vector<float> &in, double voxel_size_m,
                          bool keep_negative_height) {
  XyzhVoxelMap cells;
  const std::size_t input_points = in.size() / 4;
  cells.reserve(input_points);
  for (std::size_t i = 0; i < input_points; ++i) {
    const std::size_t base = i * 4;
    const XyzhPoint point{
        in[base + 0],
        in[base + 1],
        in[base + 2],
        in[base + 3],
    };
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !std::isfinite(point.height)) {
      continue;
    }
    if (!keep_negative_height && point.height < 0.0f) {
      continue;
    }
    keepMaxHeight(cells, makeVoxelKey(point.x, point.y, point.z, voxel_size_m), point);
  }
  return cells;
}

XyzhVoxelMap reduceVoxelBudget(const XyzhVoxelMap &cells, std::size_t budget, double voxel_size_m) {
  if (budget == 0 || cells.size() <= budget) {
    return cells;
  }
  const double initial_ratio =
      std::cbrt(static_cast<double>(cells.size()) / static_cast<double>(budget));
  double coarse_size = voxel_size_m * std::max(1.0, initial_ratio);
  XyzhVoxelMap reduced;
  for (int attempt = 0; attempt < 8; ++attempt) {
    reduced.clear();
    reduced.reserve(std::min(cells.size(), budget * 2));
    for (const auto &entry : cells) {
      const auto &point = entry.second;
      keepMaxHeight(reduced, makeVoxelKey(point.x, point.y, point.z, coarse_size), point);
    }
    if (reduced.size() <= budget) {
      return reduced;
    }
    coarse_size *= 1.25;
  }
  return reduced;
}

std::size_t sourceBudget(std::size_t max_points, double share, double active_share_sum,
                         bool active) {
  if (!active) {
    return 0;
  }
  if (max_points == 0) {
    return 0;
  }
  if (active_share_sum <= 0.0 || share <= 0.0) {
    return 1;
  }
  return std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(
                                      static_cast<double>(max_points) * share / active_share_sum)));
}

void appendXyzhCloudDedupe(std::vector<float> &out, VoxelIndex &seen, const std::vector<float> &in,
                           std::size_t max_points, std::size_t source_budget, double voxel_size_m,
                           bool keep_negative_height = true) {
  const std::size_t current_points = out.size() / 4;
  if (max_points > 0 && current_points >= max_points) {
    return;
  }
  const std::size_t input_points = in.size() / 4;
  if (input_points == 0) {
    return;
  }
  const std::size_t remaining_total = max_points == 0 ? input_points : max_points - current_points;
  const std::size_t source_limit =
      max_points == 0 ? input_points : std::min(source_budget, remaining_total);
  if (source_limit == 0) {
    return;
  }
  auto cells = voxelizeXyzh(in, voxel_size_m, keep_negative_height);
  cells = reduceVoxelBudget(cells, source_limit, voxel_size_m);
  out.reserve(out.size() + std::min(cells.size(), source_limit) * 4);
  seen.reserve(seen.size() + std::min(cells.size(), source_limit));
  std::size_t added = 0;
  for (const auto &entry : cells) {
    if (added >= source_limit || (max_points > 0 && out.size() / 4 >= max_points)) {
      break;
    }
    const auto &point = entry.second;
    const auto key = makeVoxelKey(point.x, point.y, point.z, voxel_size_m);
    const auto existing = seen.find(key);
    if (existing != seen.end()) {
      const std::size_t out_base = existing->second * 4;
      if (point.height > out[out_base + 3]) {
        out[out_base + 0] = point.x;
        out[out_base + 1] = point.y;
        out[out_base + 2] = point.z;
        out[out_base + 3] = point.height;
      }
      continue;
    }
    const std::size_t out_index = out.size() / 4;
    seen.emplace(key, out_index);
    out.push_back(point.x);
    out.push_back(point.y);
    out.push_back(point.z);
    out.push_back(point.height);
    ++added;
  }
}

}  // namespace

std::string headerFrameId(const lingtu_dds_Header &header) {
  return header.frame_id == nullptr ? std::string{} : std::string(header.frame_id);
}

double headerStampSeconds(const lingtu_dds_Header &header) {
  return stampSeconds(header.stamp);
}

std::string sourceStampError(const std::string &prefix, double source_stamp_s, double receive_s,
                             double max_age_s, double future_tolerance_s) {
  if (!std::isfinite(source_stamp_s) || source_stamp_s <= 0.0) {
    return prefix + "_source_stamp_invalid";
  }
  const double source_age_s = receive_s - source_stamp_s;
  if (source_age_s < -std::max(0.0, future_tolerance_s)) {
    return prefix + "_source_stamp_future";
  }
  if (max_age_s > 0.0 && source_age_s > max_age_s) {
    return prefix + "_source_stamp_stale";
  }
  return {};
}

bool sourceStampPredates(double source_stamp_s, double not_before_s) {
  if (!(not_before_s > 0.0)) {
    return false;
  }
  if (!std::isfinite(source_stamp_s) || source_stamp_s <= 0.0) {
    return true;
  }
  return source_stamp_s <= not_before_s;
}

std::string textData(const lingtu_dds_Text &msg) {
  return msg.data == nullptr ? std::string{} : std::string(msg.data);
}

std::vector<float> cloudToXyzh(const lingtu_dds_PointCloud2 &msg, const std::size_t max_points,
                               const std::optional<RigidTransform> &map_body,
                               const std::optional<RigidTransform> &map_odom) {
  std::vector<float> out;
  const FieldOffsets offsets = fieldOffsets(msg);
  if (!offsets.valid || offsets.x < 0 || offsets.y < 0 || offsets.z < 0 || msg.is_bigendian ||
      msg.width == 0 || msg.height == 0 || msg.point_step < 12 || msg.data._buffer == nullptr) {
    return out;
  }
  const std::size_t rows = static_cast<std::size_t>(msg.height);
  const std::size_t cols = static_cast<std::size_t>(msg.width);
  const std::size_t point_step = static_cast<std::size_t>(msg.point_step);
  const std::size_t row_step = static_cast<std::size_t>(msg.row_step);
  if (cols > std::numeric_limits<std::size_t>::max() / point_step) {
    return out;
  }
  const std::size_t packed_row = cols * point_step;
  if (row_step < packed_row || rows > std::numeric_limits<std::size_t>::max() / cols) {
    return out;
  }
  if (rows - 1 > (std::numeric_limits<std::size_t>::max() - packed_row) / row_step) {
    return out;
  }
  const std::size_t required_bytes = (rows - 1) * row_step + packed_row;
  if (required_bytes > msg.data._length) {
    return out;
  }
  const std::size_t count = rows * cols;
  if (count == 0) {
    return out;
  }
  const std::size_t stride =
      max_points > 0 && count > max_points
          ? static_cast<std::size_t>(std::ceil(static_cast<double>(count) / max_points))
          : 1;
  const std::string frame_id = headerFrameId(msg.header);
  const bool map_frame = frame_id == "map";
  const bool odom_frame = frame_id == "odom";
  const bool body_frame = frame_id == "body" || frame_id == "base" || frame_id == "base_link";
  if (!map_frame && !odom_frame && !body_frame) {
    return out;
  }
  if (body_frame && !map_body) {
    return out;
  }
  if (odom_frame && !map_odom) {
    return out;
  }
  out.reserve((count / stride + 1) * 4);
  for (std::size_t i = 0; i < count; i += stride) {
    const std::size_t row = i / cols;
    const std::size_t col = i % cols;
    const auto *base = msg.data._buffer + row * row_step + col * point_step;
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
      const auto p = transformPoint(*map_body, {x, y, z});
      out.push_back(static_cast<float>(p.x));
      out.push_back(static_cast<float>(p.y));
      out.push_back(static_cast<float>(p.z));
      world_z = p.z;
    }
    if (has_height) {
      height = raw_height;
    } else if (map_body) {
      height = static_cast<float>(world_z - map_body->translation.z);
    }
    out.push_back(std::isfinite(height) ? height : z);
  }
  return out;
}

void buildPlannerObstacleCloud(std::vector<float> &out, const std::vector<float> &registered_xyzh,
                               const std::vector<float> &terrain_xyzh, bool terrain_map_fresh,
                               const std::vector<float> &terrain_ext_xyzh, bool terrain_ext_fresh,
                               std::size_t max_points, const ObstacleMergeConfig &raw_config) {
  out.clear();
  const auto config = normalizedMergeConfig(raw_config);
  VoxelIndex seen;
  const bool registered_active = config.registered_share > 0.0 && !registered_xyzh.empty();
  const bool terrain_active =
      config.terrain_share > 0.0 && terrain_map_fresh && !terrain_xyzh.empty();
  const bool terrain_ext_active =
      config.terrain_ext_share > 0.0 && terrain_ext_fresh && !terrain_ext_xyzh.empty();
  const double active_share_sum = (registered_active ? config.registered_share : 0.0) +
                                  (terrain_active ? config.terrain_share : 0.0) +
                                  (terrain_ext_active ? config.terrain_ext_share : 0.0);
  const std::size_t registered_budget =
      sourceBudget(max_points, config.registered_share, active_share_sum, registered_active);
  const std::size_t terrain_budget =
      sourceBudget(max_points, config.terrain_share, active_share_sum, terrain_active);
  const std::size_t terrain_ext_budget =
      sourceBudget(max_points, config.terrain_ext_share, active_share_sum, terrain_ext_active);
  if (registered_active) {
    appendXyzhCloudDedupe(out, seen, registered_xyzh, max_points, registered_budget,
                          config.voxel_size_m);
  }
  if (terrain_active) {
    appendXyzhCloudDedupe(out, seen, terrain_xyzh, max_points, terrain_budget, config.voxel_size_m);
  }
  if (terrain_ext_active) {
    appendXyzhCloudDedupe(out, seen, terrain_ext_xyzh, max_points, terrain_ext_budget,
                          config.voxel_size_m);
  }
}

nav_kernel::Pose toPose(const lingtu_dds_Odometry &msg) {
  nav_kernel::Pose pose;
  pose.position = {
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z,
  };
  pose.yaw = quaternionYaw(msg.pose.pose.orientation);
  return pose;
}

std::vector<nav_kernel::Vec3> toPath(const lingtu_dds_Path &msg) {
  std::vector<nav_kernel::Vec3> path;
  if (msg.poses._length > 0 && msg.poses._buffer == nullptr) {
    return path;
  }
  path.reserve(msg.poses._length);
  for (std::uint32_t i = 0; i < msg.poses._length; ++i) {
    const auto &pose = msg.poses._buffer[i].pose.position;
    path.push_back({pose.x, pose.y, pose.z});
  }
  return path;
}

nav_kernel::Vec3 toGoalPoint(const lingtu_dds_PoseStamped &msg) {
  return {
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z,
  };
}

std::vector<nav_kernel::Vec3>
toNavPath(const std::vector<lingtu::nav::plan::GlobalPlanPoint> &path) {
  std::vector<nav_kernel::Vec3> out;
  out.reserve(path.size());
  for (const auto &point : path) {
    out.push_back({point.x, point.y, point.z});
  }
  return out;
}

nav_kernel::Twist toTwist(const lingtu_dds_TwistStamped &msg) {
  nav_kernel::Twist out;
  out.vx = msg.twist.linear.x;
  out.vy = msg.twist.linear.y;
  out.wz = msg.twist.angular.z;
  return out;
}

TraversabilityGrid toTraversabilityGrid(const lingtu_dds_OccupancyGrid &msg) {
  TraversabilityGrid grid;
  grid.rows = static_cast<int>(msg.info.height);
  grid.cols = static_cast<int>(msg.info.width);
  grid.resolution = static_cast<double>(msg.info.resolution);
  grid.origin_x = msg.info.origin.position.x;
  grid.origin_y = msg.info.origin.position.y;
  if (grid.rows <= 0 || grid.cols <= 0 || grid.resolution <= 0.0 || !std::isfinite(grid.origin_x) ||
      !std::isfinite(grid.origin_y) ||
      static_cast<std::size_t>(grid.rows) >
          std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(grid.cols)) {
    return {};
  }
  const std::size_t count =
      static_cast<std::size_t>(grid.rows) * static_cast<std::size_t>(grid.cols);
  if (msg.data._buffer == nullptr || msg.data._length < count) {
    return {};
  }
  grid.values.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto raw = static_cast<unsigned int>(msg.data._buffer[i]);
    grid.values.push_back(static_cast<float>(std::min(raw, 100u)));
  }
  return grid;
}

void PathEcho::arm(const std::vector<nav_kernel::Vec3> &path, double stamp_s) {
  path_ = path;
  stamp_s_ = stamp_s;
  armed_ = true;
}

bool PathEcho::take(const std::vector<nav_kernel::Vec3> &path, double stamp_s, double now_s) {
  constexpr double kStampToleranceS = 1e-5;
  constexpr double kMaxAgeS = 2.0;
  constexpr double kPointToleranceM = 1e-6;
  if (!armed_) {
    return false;
  }
  if (!std::isfinite(now_s) || now_s - stamp_s_ > kMaxAgeS) {
    reset();
    return false;
  }
  if (!std::isfinite(stamp_s) || std::abs(stamp_s - stamp_s_) > kStampToleranceS ||
      path.size() != path_.size()) {
    return false;
  }
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (std::abs(path[i].x - path_[i].x) > kPointToleranceM ||
        std::abs(path[i].y - path_[i].y) > kPointToleranceM ||
        std::abs(path[i].z - path_[i].z) > kPointToleranceM) {
      return false;
    }
  }
  reset();
  return true;
}

void PathEcho::reset() {
  path_.clear();
  stamp_s_ = -1.0;
  armed_ = false;
}

Decoded<std::vector<nav_kernel::Vec3>> decodePath(const lingtu_dds_Path &msg,
                                                  const std::optional<RigidTransform> &map_odom) {
  Decoded<std::vector<nav_kernel::Vec3>> out;
  const std::string frame = headerFrameId(msg.header);
  if (frame.empty()) {
    out.error = "path_frame_empty";
    return out;
  }
  for (std::uint32_t i = 0; i < msg.poses._length; ++i) {
    if (msg.poses._buffer == nullptr) {
      out.error = "path_payload_invalid";
      return out;
    }
    const std::string pose_frame = headerFrameId(msg.poses._buffer[i].header);
    if (!pose_frame.empty() && pose_frame != frame) {
      out.error = "path_pose_frame_mismatch";
      return out;
    }
  }
  out.value = toPath(msg);
  for (const auto &point : out.value) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      out.value.clear();
      out.error = "path_nonfinite";
      return out;
    }
  }
  if (frame == "map") {
    return out;
  }
  if (frame != "odom") {
    out.value.clear();
    out.error = "path_frame_unsupported";
    return out;
  }
  if (!map_odom || !map_odom->valid) {
    out.value.clear();
    out.error = "path_tf_missing";
    return out;
  }
  for (auto &point : out.value) {
    point = transformPoint(*map_odom, point);
  }
  return out;
}

Decoded<GoalTarget> decodeGoal(const lingtu_dds_PoseStamped &msg,
                               const std::optional<RigidTransform> &map_odom) {
  Decoded<GoalTarget> out;
  out.value.position = toGoalPoint(msg);
  if (!std::isfinite(out.value.position.x) || !std::isfinite(out.value.position.y) ||
      !std::isfinite(out.value.position.z)) {
    out.error = "goal_nonfinite";
    return out;
  }
  const auto &orientation = msg.pose.orientation;
  const double orientation_norm =
      std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(orientation_norm) || orientation_norm <= 1e-12) {
    out.error = "goal_orientation_invalid";
    return out;
  }
  out.value.yaw = quaternionYaw(normalizeQuaternion(orientation));
  const std::string frame = headerFrameId(msg.header);
  if (frame == "map") {
    return out;
  }
  if (frame != "odom") {
    out.error = frame.empty() ? "goal_frame_empty" : "goal_frame_unsupported";
    return out;
  }
  if (!map_odom || !map_odom->valid) {
    out.error = "goal_tf_missing";
    return out;
  }
  out.value.position = transformPoint(*map_odom, out.value.position);
  out.value.yaw =
      std::atan2(std::sin(map_odom->yaw + out.value.yaw), std::cos(map_odom->yaw + out.value.yaw));
  return out;
}

Decoded<nav_kernel::Twist> decodeTwist(const lingtu_dds_TwistStamped &msg) {
  Decoded<nav_kernel::Twist> out;
  const std::string frame = headerFrameId(msg.header);
  if (frame != "base_link" && frame != "body") {
    out.error = frame.empty() ? "twist_frame_empty" : "twist_frame_unsupported";
    return out;
  }
  out.value = toTwist(msg);
  if (!std::isfinite(out.value.vx) || !std::isfinite(out.value.vy) ||
      !std::isfinite(out.value.wz)) {
    out.value = {};
    out.error = "twist_nonfinite";
  }
  return out;
}

Decoded<TraversabilityGrid> decodeGrid(const lingtu_dds_OccupancyGrid &msg) {
  Decoded<TraversabilityGrid> out;
  const std::string frame = headerFrameId(msg.header);
  if (frame != "map") {
    out.error = frame.empty() ? "grid_frame_empty" : "grid_frame_unsupported";
    return out;
  }
  out.value = toTraversabilityGrid(msg);
  if (out.value.values.empty()) {
    out.error = "grid_payload_invalid";
  }
  return out;
}

double vecDistance(const nav_kernel::Vec3 &a, const nav_kernel::Vec3 &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace lingtu::nav::endpoint
