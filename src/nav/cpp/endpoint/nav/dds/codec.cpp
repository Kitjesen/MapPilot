#include "dds/codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>
#include <utility>

#include "command/ingress.hpp"
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

}  // namespace

std::string stringValue(const char *value) {
  return value == nullptr ? std::string{} : std::string(value);
}

CommandIngressRequest
commandIngressRequestFromDds(const lingtu_dds_NavigationCommandRequest &message) {
  CommandIngressRequest request;
  request.client_id = stringValue(message.client_id);
  request.task_id = stringValue(message.task_id);
  request.request_id = stringValue(message.request_id);
  request.raw_kind = message.kind;

  auto &payload = request.payload;
  payload.frame_id = stringValue(message.header.frame_id);
  payload.goal = {
      message.goal.position.x,    message.goal.position.y,    message.goal.position.z,
      message.goal.orientation.x, message.goal.orientation.y, message.goal.orientation.z,
      message.goal.orientation.w,
  };
  payload.reason = stringValue(message.reason);
  return request;
}

std::string headerFrameId(const lingtu_dds_Header &header) {
  return header.frame_id == nullptr ? std::string{} : std::string(header.frame_id);
}

double headerStampSeconds(const lingtu_dds_Header &header) {
  return stampSeconds(header.stamp);
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

InputSample<TransformSample> copyTransformSample(const lingtu_dds_TFMessage &message) {
  InputSample<TransformSample> sample;
  const auto transform = mapOdomTransformFromTf(message);
  if (!transform) {
    sample.error = "map_odom_tf_invalid";
    return sample;
  }
  sample.value.map_odom = *transform;
  return sample;
}

InputSample<OdometrySample> copyOdometrySample(const lingtu_dds_Odometry &message) {
  InputSample<OdometrySample> sample;
  if (const char *error = odometryFramePairError(message)) {
    sample.error = error;
    return sample;
  }
  sample.value.header = {headerStampSeconds(message.header), headerFrameId(message.header)};
  sample.value.child_frame_id = stringValue(message.child_frame_id);
  sample.value.body = rigidTransformFromOdometry(message);
  sample.value.pose.position = sample.value.body.translation;
  sample.value.pose.yaw = sample.value.body.yaw;
  sample.value.linear_velocity = {
      message.twist.twist.linear.x,
      message.twist.twist.linear.y,
      message.twist.twist.linear.z,
  };
  sample.value.angular_velocity = {
      message.twist.twist.angular.x,
      message.twist.twist.angular.y,
      message.twist.twist.angular.z,
  };
  return sample;
}

InputSample<PointCloudSample> copyPointCloudSample(const lingtu_dds_PointCloud2 &msg,
                                                   const bool terrain_height) {
  InputSample<PointCloudSample> sample;
  sample.value.header = {headerStampSeconds(msg.header), headerFrameId(msg.header)};
  const FieldOffsets offsets = fieldOffsets(msg);
  if (!offsets.valid || offsets.x < 0 || offsets.y < 0 || offsets.z < 0 || msg.is_bigendian ||
      msg.width == 0 || msg.height == 0 || msg.point_step < 12 || msg.data._buffer == nullptr) {
    sample.error = "point_cloud_layout_invalid";
    return sample;
  }
  const std::size_t rows = static_cast<std::size_t>(msg.height);
  const std::size_t cols = static_cast<std::size_t>(msg.width);
  const std::size_t point_step = static_cast<std::size_t>(msg.point_step);
  const std::size_t row_step = static_cast<std::size_t>(msg.row_step);
  if (cols > std::numeric_limits<std::size_t>::max() / point_step) {
    sample.error = "point_cloud_layout_invalid";
    return sample;
  }
  const std::size_t packed_row = cols * point_step;
  if (row_step < packed_row || rows > std::numeric_limits<std::size_t>::max() / cols ||
      rows - 1 > (std::numeric_limits<std::size_t>::max() - packed_row) / row_step) {
    sample.error = "point_cloud_layout_invalid";
    return sample;
  }
  const std::size_t required_bytes = (rows - 1) * row_step + packed_row;
  if (required_bytes > msg.data._length) {
    sample.error = "point_cloud_payload_invalid";
    return sample;
  }

  const bool explicit_height = offsets.height >= 0 || (terrain_height && offsets.intensity >= 0);
  sample.value.xyzh.reserve(rows * cols * 4U);
  for (std::size_t row = 0U; row < rows; ++row) {
    for (std::size_t col = 0U; col < cols; ++col) {
      const auto *base = msg.data._buffer + row * row_step + col * point_step;
      const float x = readFloat(base + offsets.x);
      const float y = readFloat(base + offsets.y);
      const float z = readFloat(base + offsets.z);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      const float height = offsets.height >= 0
                               ? readFloat(base + offsets.height)
                               : terrain_height && offsets.intensity >= 0
                                     ? readFloat(base + offsets.intensity)
                                     : std::numeric_limits<float>::quiet_NaN();
      sample.value.xyzh.insert(sample.value.xyzh.end(),
                               {x, y, z, explicit_height && std::isfinite(height)
                                             ? height
                                             : std::numeric_limits<float>::quiet_NaN()});
    }
  }
  if (sample.value.xyzh.empty()) {
    sample.error = "point_cloud_payload_invalid";
  }
  return sample;
}

DriverControlSample copyDriverControlSample(const lingtu_dds_DriverControlState &message) {
  DriverControlSample sample;
  sample.stamp_s = headerStampSeconds(message.header);
  if (message.header.stamp.sec >= 0) {
    sample.stamp_ns = static_cast<std::uint64_t>(message.header.stamp.sec) * 1'000'000'000ULL +
                      static_cast<std::uint64_t>(message.header.stamp.nanosec);
  }
  sample.connected = message.connected;
  sample.ready = message.ready;
  sample.motors_enabled = message.motors_enabled;
  sample.critical_fault = message.critical_fault;
  sample.control_assured = message.control_assured;
  sample.reason = stringValue(message.reason);
  sample.accepted_producer_boot_id = stringValue(message.accepted_producer_boot_id);
  sample.accepted_output_sequence = message.accepted_output_sequence;
  sample.last_command_accepted = message.last_command_accepted;
  return sample;
}

namespace {

std::vector<float> decodeCloudToXyzh(const lingtu_dds_PointCloud2 &msg,
                                     const std::size_t max_points,
                                     const std::optional<RigidTransform> &map_body,
                                     const std::optional<RigidTransform> &map_odom,
                                     const bool terrain_intensity_is_height) {
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
    const bool has_height = offsets.height >= 0 ||
                            (terrain_intensity_is_height && offsets.intensity >= 0);
    const float raw_height = offsets.height >= 0
                                 ? readFloat(base + offsets.height)
                                 : terrain_intensity_is_height && offsets.intensity >= 0
                                       ? readFloat(base + offsets.intensity)
                                       : 0.0f;
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

}  // namespace

std::vector<float> cloudToXyzh(const lingtu_dds_PointCloud2 &msg,
                               const std::size_t max_points,
                               const std::optional<RigidTransform> &map_body,
                               const std::optional<RigidTransform> &map_odom) {
  return decodeCloudToXyzh(msg, max_points, map_body, map_odom, false);
}

std::vector<float> terrainCloudToXyzh(const lingtu_dds_PointCloud2 &msg,
                                      const std::size_t max_points,
                                      const std::optional<RigidTransform> &map_body,
                                      const std::optional<RigidTransform> &map_odom) {
  return decodeCloudToXyzh(msg, max_points, map_body, map_odom, true);
}

nav_kernel::LocalCollisionMapView LocalCollisionMap::view() const noexcept {
  return {
      occupied_xyz.empty() ? nullptr : occupied_xyz.data(),
      static_cast<int>(occupied_xyz.size() / 3U),
      resolution,
      aabb_min,
      aabb_max,
      reset_epoch,
      observation_sequence,
      generation,
      stamp_s,
      receive_stamp_s,
      complete,
      live,
  };
}

Decoded<LocalCollisionMap> decodeLocalCollisionMap(
    const lingtu_dds_MapCollisionLayer &msg) {
  Decoded<LocalCollisionMap> decoded;
  const std::string frame = headerFrameId(msg.header);
  const std::string occupied_frame = headerFrameId(msg.occupied.header);
  decoded.value.stamp_s = headerStampSeconds(msg.header);
  decoded.value.resolution = msg.resolution;
  decoded.value.aabb_min = {msg.aabb_min.x, msg.aabb_min.y, msg.aabb_min.z};
  decoded.value.aabb_max = {msg.aabb_max.x, msg.aabb_max.y, msg.aabb_max.z};
  decoded.value.reset_epoch = msg.reset_epoch;
  decoded.value.observation_sequence = msg.observation_sequence;
  decoded.value.generation = msg.generation;
  decoded.value.complete = msg.complete;
  decoded.value.live = msg.live;

  const auto finite_point = [](const nav_kernel::Vec3 &point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z);
  };
  if (frame != "map" || occupied_frame != "map") {
    decoded.error = "local_collision_frame_invalid";
    return decoded;
  }
  if (!std::isfinite(decoded.value.stamp_s) || decoded.value.stamp_s <= 0.0 ||
      !std::isfinite(decoded.value.resolution) || decoded.value.resolution <= 0.0 ||
      !finite_point(decoded.value.aabb_min) || !finite_point(decoded.value.aabb_max) ||
      decoded.value.aabb_min.x >= decoded.value.aabb_max.x ||
      decoded.value.aabb_min.y >= decoded.value.aabb_max.y ||
      decoded.value.aabb_min.z >= decoded.value.aabb_max.z ||
      decoded.value.reset_epoch == 0U || decoded.value.observation_sequence == 0U ||
      decoded.value.generation == 0U) {
    decoded.error = "local_collision_metadata_invalid";
    return decoded;
  }

  const std::size_t expected_points =
      static_cast<std::size_t>(msg.occupied.width) *
      static_cast<std::size_t>(msg.occupied.height);
  if (expected_points == 0U) {
    if (msg.occupied.data._length != 0U) {
      decoded.error = "local_collision_empty_cloud_has_payload";
    }
    return decoded;
  }
  std::vector<float> xyz = cloudToXyzh(msg.occupied, 0U, std::nullopt, std::nullopt);
  if (xyz.size() != expected_points * 4U) {
    decoded.error = "local_collision_cloud_invalid";
    return decoded;
  }
  for (std::size_t index = 0U; index < expected_points; ++index) {
    xyz[index * 3U] = xyz[index * 4U];
    xyz[index * 3U + 1U] = xyz[index * 4U + 1U];
    xyz[index * 3U + 2U] = xyz[index * 4U + 2U];
  }
  xyz.resize(expected_points * 3U);
  decoded.value.occupied_xyz = std::move(xyz);
  return decoded;
}

nav_kernel::Pose toPose(const lingtu_dds_Odometry &msg) {
  nav_kernel::Pose pose;
  pose.position = {
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z,
  };
  pose.yaw = quaternionYaw(quaternionFromDds(msg.pose.pose.orientation));
  return pose;
}

static nav_kernel::Vec3 toGoalPoint(const lingtu_dds_PoseStamped &msg) {
  return {
      msg.pose.position.x,
      msg.pose.position.y,
      msg.pose.position.z,
  };
}

static nav_kernel::Twist toTwist(const lingtu_dds_TwistStamped &msg) {
  nav_kernel::Twist out;
  out.vx = msg.twist.linear.x;
  out.vy = msg.twist.linear.y;
  out.wz = msg.twist.angular.z;
  return out;
}

static TraversabilityGrid toTraversabilityGrid(const lingtu_dds_OccupancyGrid &msg) {
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
  if (!std::isfinite(orientation_norm)) {
    out.error = "goal_orientation_invalid";
    return out;
  }
  if (orientation_norm > 1e-12) {
    out.value.yaw = quaternionYaw(normalizeQuaternion(quaternionFromDds(orientation)));
  }
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
  if (out.value.yaw) {
    out.value.yaw = std::atan2(std::sin(map_odom->yaw + *out.value.yaw),
                               std::cos(map_odom->yaw + *out.value.yaw));
  }
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

Decoded<TraversabilityGrid> decodeLocalRiskGrid(const lingtu_dds_OccupancyGrid &msg) {
  Decoded<TraversabilityGrid> out;
  const std::string frame = headerFrameId(msg.header);
  if (frame != "odom") {
    out.error = frame.empty() ? "local_grid_frame_empty" : "local_grid_frame_unsupported";
    return out;
  }
  out.value = toTraversabilityGrid(msg);
  if (out.value.values.empty()) {
    out.error = "local_grid_payload_invalid";
  }
  return out;
}

InputSample<GridSample> copyGridSample(const lingtu_dds_OccupancyGrid &message,
                                      const char *required_frame) {
  InputSample<GridSample> sample;
  sample.value.header = {headerStampSeconds(message.header), headerFrameId(message.header)};
  const bool local = required_frame != nullptr && std::string_view{required_frame} == "odom";
  auto decoded = local ? decodeLocalRiskGrid(message) : decodeGrid(message);
  if (!decoded.ok()) {
    sample.error = std::move(decoded.error);
    return sample;
  }
  sample.value.grid = std::move(decoded.value);
  return sample;
}

InputSample<LocalCollisionMap> copyLocalCollisionSample(
    const lingtu_dds_MapCollisionLayer &message) {
  InputSample<LocalCollisionMap> sample;
  auto decoded = decodeLocalCollisionMap(message);
  sample.value = std::move(decoded.value);
  sample.error = std::move(decoded.error);
  return sample;
}

}  // namespace lingtu::nav::endpoint
