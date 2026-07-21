#pragma once

#include "lingtu_slam.h"
#include "nav_kernel/types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>

namespace lingtu::nav::endpoint {

struct RigidTransform {
  nav_kernel::Vec3 translation{};
  lingtu_dds_Quaternion rotation{};
  double yaw{0.0};
  double stamp_s{0.0};
  bool valid{false};
};

inline double ddsStampSeconds(const lingtu_dds_Time& stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline double quaternionYaw(const lingtu_dds_Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline bool quaternionIsFiniteAndNonzero(const lingtu_dds_Quaternion& q) {
  const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
      std::isfinite(q.w) && std::isfinite(norm_squared) && norm_squared > 1e-24;
}

inline lingtu_dds_Quaternion normalizeQuaternion(lingtu_dds_Quaternion q) {
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

inline double quaternionRoll(const lingtu_dds_Quaternion& raw_q) {
  const auto q = normalizeQuaternion(raw_q);
  return std::atan2(
      2.0 * (q.w * q.x + q.y * q.z),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

inline double quaternionPitch(const lingtu_dds_Quaternion& raw_q) {
  const auto q = normalizeQuaternion(raw_q);
  return std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}

inline RigidTransform rigidTransformFromOdometry(
    const lingtu_dds_Odometry& message) {
  RigidTransform transform;
  transform.translation = {
      message.pose.pose.position.x,
      message.pose.pose.position.y,
      message.pose.pose.position.z,
  };
  const bool rotation_valid =
      quaternionIsFiniteAndNonzero(message.pose.pose.orientation);
  transform.rotation = normalizeQuaternion(message.pose.pose.orientation);
  transform.yaw = quaternionYaw(transform.rotation);
  transform.stamp_s = ddsStampSeconds(message.header.stamp);
  transform.valid =
      std::isfinite(transform.translation.x) &&
      std::isfinite(transform.translation.y) &&
      std::isfinite(transform.translation.z) &&
      rotation_valid &&
      std::isfinite(transform.yaw);
  return transform;
}

inline lingtu_dds_Quaternion inverseQuaternion(const lingtu_dds_Quaternion& q) {
  lingtu_dds_Quaternion out = normalizeQuaternion(q);
  out.x = -out.x;
  out.y = -out.y;
  out.z = -out.z;
  return out;
}

inline lingtu_dds_Quaternion multiplyQuaternions(
    const lingtu_dds_Quaternion& raw_lhs,
    const lingtu_dds_Quaternion& raw_rhs) {
  const auto lhs = normalizeQuaternion(raw_lhs);
  const auto rhs = normalizeQuaternion(raw_rhs);
  lingtu_dds_Quaternion out{};
  out.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  out.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  out.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  out.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  return normalizeQuaternion(out);
}

inline nav_kernel::Vec3 rotatePoint(
    const lingtu_dds_Quaternion& raw_q,
    const nav_kernel::Vec3& point) {
  const auto q = normalizeQuaternion(raw_q);
  const nav_kernel::Vec3 u{q.x, q.y, q.z};
  const double s = q.w;
  const double dot = u.x * point.x + u.y * point.y + u.z * point.z;
  const nav_kernel::Vec3 cross{
      u.y * point.z - u.z * point.y,
      u.z * point.x - u.x * point.z,
      u.x * point.y - u.y * point.x,
  };
  return {
      2.0 * dot * u.x +
          (s * s - (u.x * u.x + u.y * u.y + u.z * u.z)) * point.x +
          2.0 * s * cross.x,
      2.0 * dot * u.y +
          (s * s - (u.x * u.x + u.y * u.y + u.z * u.z)) * point.y +
          2.0 * s * cross.y,
      2.0 * dot * u.z +
          (s * s - (u.x * u.x + u.y * u.y + u.z * u.z)) * point.z +
          2.0 * s * cross.z,
  };
}

inline nav_kernel::Vec3 transformPoint(
    const RigidTransform& transform,
    const nav_kernel::Vec3& point) {
  const auto rotated = rotatePoint(transform.rotation, point);
  return {
      transform.translation.x + rotated.x,
      transform.translation.y + rotated.y,
      transform.translation.z + rotated.z,
  };
}

inline RigidTransform inverseTransform(const RigidTransform& transform) {
  RigidTransform out;
  out.rotation = inverseQuaternion(transform.rotation);
  const nav_kernel::Vec3 negative_translation{
      -transform.translation.x,
      -transform.translation.y,
      -transform.translation.z,
  };
  out.translation = rotatePoint(out.rotation, negative_translation);
  out.yaw = quaternionYaw(out.rotation);
  out.stamp_s = transform.stamp_s;
  out.valid = transform.valid;
  return out;
}

// Compose parent<-middle with middle<-child to produce parent<-child.
inline RigidTransform composeTransforms(
    const RigidTransform& parent_middle,
    const RigidTransform& middle_child) {
  RigidTransform out;
  out.translation = transformPoint(parent_middle, middle_child.translation);
  out.rotation = multiplyQuaternions(parent_middle.rotation, middle_child.rotation);
  out.yaw = quaternionYaw(out.rotation);
  out.stamp_s = std::max(parent_middle.stamp_s, middle_child.stamp_s);
  out.valid = parent_middle.valid && middle_child.valid;
  return out;
}

inline bool transformJumpExceeds(
    const RigidTransform& previous,
    const RigidTransform& next,
    double translation_threshold_m,
    double yaw_threshold_rad) {
  if (!previous.valid || !next.valid) {
    return false;
  }
  const auto correction = composeTransforms(next, inverseTransform(previous));
  const double translation_jump_m = std::sqrt(
      correction.translation.x * correction.translation.x +
      correction.translation.y * correction.translation.y +
      correction.translation.z * correction.translation.z);
  return translation_jump_m > std::max(0.0, translation_threshold_m) ||
      std::abs(correction.yaw) > std::max(0.0, yaw_threshold_rad);
}

inline nav_kernel::Pose transformPose(
    const RigidTransform& transform,
    const nav_kernel::Pose& pose) {
  nav_kernel::Pose out;
  out.position = transformPoint(transform, pose.position);
  out.yaw = transform.yaw + pose.yaw;
  return out;
}

inline std::optional<RigidTransform> mapOdomTransformFromTf(
    const lingtu_dds_TFMessage& message) {
  if (message.transforms._length > 0 && message.transforms._buffer == nullptr) {
    return std::nullopt;
  }
  for (std::uint32_t index = 0; index < message.transforms._length; ++index) {
    const auto& stamped = message.transforms._buffer[index];
    const std::string parent =
        stamped.header.frame_id == nullptr ? std::string{} : std::string(stamped.header.frame_id);
    const std::string child = stamped.child_frame_id == nullptr
        ? std::string{}
        : std::string(stamped.child_frame_id);
    if (!((parent == "map" && child == "odom") ||
          (parent == "odom" && child == "map"))) {
      continue;
    }
    RigidTransform transform;
    transform.translation = {
        stamped.transform.translation.x,
        stamped.transform.translation.y,
        stamped.transform.translation.z,
    };
    if (!std::isfinite(transform.translation.x) ||
        !std::isfinite(transform.translation.y) ||
        !std::isfinite(transform.translation.z) ||
        !quaternionIsFiniteAndNonzero(stamped.transform.rotation)) {
      return std::nullopt;
    }
    transform.rotation = normalizeQuaternion(stamped.transform.rotation);
    transform.yaw = quaternionYaw(transform.rotation);
    transform.stamp_s = ddsStampSeconds(stamped.header.stamp);
    transform.valid = true;
    return parent == "map" ? std::optional<RigidTransform>{transform}
                           : std::optional<RigidTransform>{inverseTransform(transform)};
  }
  return std::nullopt;
}

}  // namespace lingtu::nav::endpoint
