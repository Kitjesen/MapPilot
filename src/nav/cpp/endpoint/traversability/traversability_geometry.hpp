#pragma once

#include <cmath>
#include <optional>

#include "frame_transform.hpp"

namespace lingtu::nav::endpoint {

struct TraversabilityPose {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  double stamp_s{0.0};
  lingtu_dds_Quaternion rotation{};
};

inline std::optional<TraversabilityPose> traversabilityPose(const RigidTransform &transform) {
  if (!transform.valid || !std::isfinite(transform.translation.x) ||
      !std::isfinite(transform.translation.y) || !std::isfinite(transform.translation.z) ||
      !std::isfinite(transform.stamp_s) || transform.stamp_s <= 0.0 ||
      !quaternionIsFiniteAndNonzero(transform.rotation)) {
    return std::nullopt;
  }
  const auto rotation = normalizeQuaternion(transform.rotation);
  TraversabilityPose pose;
  pose.x = transform.translation.x;
  pose.y = transform.translation.y;
  pose.z = transform.translation.z;
  pose.roll = quaternionRoll(rotation);
  pose.pitch = quaternionPitch(rotation);
  pose.yaw = quaternionYaw(rotation);
  pose.stamp_s = transform.stamp_s;
  pose.rotation = rotation;
  if (!std::isfinite(pose.roll) || !std::isfinite(pose.pitch) || !std::isfinite(pose.yaw)) {
    return std::nullopt;
  }
  return pose;
}

inline std::optional<nav_kernel::Vec3>
traversabilitySensorOrigin(const TraversabilityPose &pose, const nav_kernel::Vec3 &sensor_offset) {
  if (!std::isfinite(sensor_offset.x) || !std::isfinite(sensor_offset.y) ||
      !std::isfinite(sensor_offset.z) || !quaternionIsFiniteAndNonzero(pose.rotation)) {
    return std::nullopt;
  }
  const auto offset = rotatePoint(pose.rotation, sensor_offset);
  nav_kernel::Vec3 origin{
      pose.x + offset.x,
      pose.y + offset.y,
      pose.z + offset.z,
  };
  if (!std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z)) {
    return std::nullopt;
  }
  return origin;
}

inline std::optional<nav_kernel::Vec3>
traversabilitySensorOrigin(const RigidTransform &transform, const nav_kernel::Vec3 &sensor_offset) {
  const auto pose = traversabilityPose(transform);
  return pose ? traversabilitySensorOrigin(*pose, sensor_offset) : std::nullopt;
}

}  // namespace lingtu::nav::endpoint
