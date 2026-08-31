#include "command/goal.hpp"

#include <algorithm>
#include <cmath>

namespace lingtu::nav::endpoint {

ParsedCommand<GoalTarget> parseGoal(const GoalSample &sample,
                                    const std::optional<RigidTransform> &map_odom) {
  ParsedCommand<GoalTarget> out;
  out.value.position = sample.position;
  if (!std::isfinite(out.value.position.x) || !std::isfinite(out.value.position.y) ||
      !std::isfinite(out.value.position.z)) {
    out.error = "goal_nonfinite";
    return out;
  }

  const auto &orientation = sample.orientation;
  const double orientation_norm =
      std::sqrt(orientation.x * orientation.x + orientation.y * orientation.y +
                orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(orientation_norm)) {
    out.error = "goal_orientation_invalid";
    return out;
  }
  if (sample.has_orientation && orientation_norm > 1e-12) {
    out.value.yaw = quaternionYaw(normalizeQuaternion(orientation));
  }

  if (sample.frame_id == "map") {
    return out;
  }
  if (sample.frame_id != "odom") {
    out.error = sample.frame_id.empty() ? "goal_frame_empty" : "goal_frame_unsupported";
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

ParsedCommand<nav_kernel::Twist> parseMotion(const OperatorMotionInputSample &sample) {
  ParsedCommand<nav_kernel::Twist> out;
  if (sample.frame_id != "base_link" && sample.frame_id != "body") {
    out.error = sample.frame_id.empty() ? "twist_frame_empty" : "twist_frame_unsupported";
    return out;
  }
  out.value = sample.velocity;
  if (!std::isfinite(out.value.vx) || !std::isfinite(out.value.vy) ||
      !std::isfinite(out.value.wz)) {
    out.value = {};
    out.error = "twist_nonfinite";
  }
  return out;
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

}  // namespace lingtu::nav::endpoint
