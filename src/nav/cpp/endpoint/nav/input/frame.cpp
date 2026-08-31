#include "input/frame.hpp"

#include <algorithm>
#include <cmath>

namespace lingtu::nav::endpoint {

double quaternionYaw(const Quaternion &q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

bool quaternionIsFiniteAndNonzero(const Quaternion &q) {
  const double norm_squared = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
         std::isfinite(q.w) && std::isfinite(norm_squared) && norm_squared > 1e-24;
}

Quaternion normalizeQuaternion(Quaternion q) {
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!std::isfinite(norm) || norm <= 1e-12) {
    return {};
  }
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  q.w /= norm;
  return q;
}

double quaternionRoll(const Quaternion &raw_q) {
  const auto q = normalizeQuaternion(raw_q);
  return std::atan2(2.0 * (q.w * q.x + q.y * q.z),
                    1.0 - 2.0 * (q.x * q.x + q.y * q.y));
}

double quaternionPitch(const Quaternion &raw_q) {
  const auto q = normalizeQuaternion(raw_q);
  return std::asin(std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0));
}

Quaternion inverseQuaternion(const Quaternion &q) {
  Quaternion out = normalizeQuaternion(q);
  out.x = -out.x;
  out.y = -out.y;
  out.z = -out.z;
  return out;
}

Quaternion multiplyQuaternions(const Quaternion &raw_lhs, const Quaternion &raw_rhs) {
  const auto lhs = normalizeQuaternion(raw_lhs);
  const auto rhs = normalizeQuaternion(raw_rhs);
  Quaternion out;
  out.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  out.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  out.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  out.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  return normalizeQuaternion(out);
}

nav_kernel::Vec3 rotatePoint(const Quaternion &raw_q, const nav_kernel::Vec3 &point) {
  const auto q = normalizeQuaternion(raw_q);
  const nav_kernel::Vec3 u{q.x, q.y, q.z};
  const double s = q.w;
  const double dot = u.x * point.x + u.y * point.y + u.z * point.z;
  const nav_kernel::Vec3 cross{
      u.y * point.z - u.z * point.y,
      u.z * point.x - u.x * point.z,
      u.x * point.y - u.y * point.x,
  };
  const double uu = u.x * u.x + u.y * u.y + u.z * u.z;
  return {
      2.0 * dot * u.x + (s * s - uu) * point.x + 2.0 * s * cross.x,
      2.0 * dot * u.y + (s * s - uu) * point.y + 2.0 * s * cross.y,
      2.0 * dot * u.z + (s * s - uu) * point.z + 2.0 * s * cross.z,
  };
}

nav_kernel::Vec3 transformPoint(const RigidTransform &transform,
                                const nav_kernel::Vec3 &point) {
  const auto rotated = rotatePoint(transform.rotation, point);
  return {
      transform.translation.x + rotated.x,
      transform.translation.y + rotated.y,
      transform.translation.z + rotated.z,
  };
}

RigidTransform inverseTransform(const RigidTransform &transform) {
  RigidTransform out;
  out.rotation = inverseQuaternion(transform.rotation);
  out.translation = rotatePoint(out.rotation, {-transform.translation.x,
                                                -transform.translation.y,
                                                -transform.translation.z});
  out.yaw = quaternionYaw(out.rotation);
  out.stamp_s = transform.stamp_s;
  out.valid = transform.valid;
  return out;
}

RigidTransform composeTransforms(const RigidTransform &parent_middle,
                                 const RigidTransform &middle_child) {
  RigidTransform out;
  out.translation = transformPoint(parent_middle, middle_child.translation);
  out.rotation = multiplyQuaternions(parent_middle.rotation, middle_child.rotation);
  out.yaw = quaternionYaw(out.rotation);
  out.stamp_s = std::max(parent_middle.stamp_s, middle_child.stamp_s);
  out.valid = parent_middle.valid && middle_child.valid;
  return out;
}

bool transformJumpExceeds(const RigidTransform &previous, const RigidTransform &next,
                          double translation_threshold_m, double yaw_threshold_rad) {
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

nav_kernel::Pose transformPose(const RigidTransform &transform, const nav_kernel::Pose &pose) {
  nav_kernel::Pose out;
  out.position = transformPoint(transform, pose.position);
  out.yaw = transform.yaw + pose.yaw;
  return out;
}

}  // namespace lingtu::nav::endpoint
