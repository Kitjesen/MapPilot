#pragma once

#include <array>
#include <cmath>
#include <limits>

#include "localization/opt/graph.hpp"

namespace lingtu::localization::opt {

inline double square(double value) {
  return value * value;
}

inline double wrap_angle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline Pose normalize_pose(Pose pose) {
  const double norm =
      std::sqrt(square(pose.qw) + square(pose.qx) + square(pose.qy) + square(pose.qz));
  if (!std::isfinite(norm) || norm <= std::numeric_limits<double>::epsilon()) {
    pose.qw = 1.0;
    pose.qx = 0.0;
    pose.qy = 0.0;
    pose.qz = 0.0;
    return pose;
  }
  pose.qw /= norm;
  pose.qx /= norm;
  pose.qy /= norm;
  pose.qz /= norm;
  return pose;
}

inline std::array<double, 3> rotate_vector(const Pose &pose, double x, double y, double z) {
  return {(1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz)) * x +
              2.0 * (pose.qx * pose.qy - pose.qz * pose.qw) * y +
              2.0 * (pose.qx * pose.qz + pose.qy * pose.qw) * z,
          2.0 * (pose.qx * pose.qy + pose.qz * pose.qw) * x +
              (1.0 - 2.0 * (pose.qx * pose.qx + pose.qz * pose.qz)) * y +
              2.0 * (pose.qy * pose.qz - pose.qx * pose.qw) * z,
          2.0 * (pose.qx * pose.qz - pose.qy * pose.qw) * x +
              2.0 * (pose.qy * pose.qz + pose.qx * pose.qw) * y +
              (1.0 - 2.0 * (pose.qx * pose.qx + pose.qy * pose.qy)) * z};
}

inline Pose inverse_pose(const Pose &input) {
  const Pose pose = normalize_pose(input);
  Pose out;
  out.qw = pose.qw;
  out.qx = -pose.qx;
  out.qy = -pose.qy;
  out.qz = -pose.qz;
  const auto translated = rotate_vector(out, -pose.x, -pose.y, -pose.z);
  out.x = translated[0];
  out.y = translated[1];
  out.z = translated[2];
  return normalize_pose(out);
}

inline Pose compose_pose(const Pose &lhs_input, const Pose &rhs_input) {
  const Pose lhs = normalize_pose(lhs_input);
  const Pose rhs = normalize_pose(rhs_input);
  Pose out;
  out.qw = lhs.qw * rhs.qw - lhs.qx * rhs.qx - lhs.qy * rhs.qy - lhs.qz * rhs.qz;
  out.qx = lhs.qw * rhs.qx + lhs.qx * rhs.qw + lhs.qy * rhs.qz - lhs.qz * rhs.qy;
  out.qy = lhs.qw * rhs.qy - lhs.qx * rhs.qz + lhs.qy * rhs.qw + lhs.qz * rhs.qx;
  out.qz = lhs.qw * rhs.qz + lhs.qx * rhs.qy - lhs.qy * rhs.qx + lhs.qz * rhs.qw;
  const auto translated = rotate_vector(lhs, rhs.x, rhs.y, rhs.z);
  out.x = lhs.x + translated[0];
  out.y = lhs.y + translated[1];
  out.z = lhs.z + translated[2];
  return normalize_pose(out);
}

inline Pose between_poses(const Pose &from, const Pose &to) {
  return compose_pose(inverse_pose(from), to);
}

inline std::array<double, 3> pose_rpy(const Pose &input) {
  const Pose pose = normalize_pose(input);
  const double sinr = 2.0 * (pose.qw * pose.qx + pose.qy * pose.qz);
  const double cosr = 1.0 - 2.0 * (pose.qx * pose.qx + pose.qy * pose.qy);
  const double roll = std::atan2(sinr, cosr);
  const double sinp = 2.0 * (pose.qw * pose.qy - pose.qz * pose.qx);
  const double pitch =
      std::abs(sinp) >= 1.0 ? std::copysign(1.57079632679489661923, sinp) : std::asin(sinp);
  const double siny = 2.0 * (pose.qw * pose.qz + pose.qx * pose.qy);
  const double cosy = 1.0 - 2.0 * (pose.qy * pose.qy + pose.qz * pose.qz);
  const double yaw = std::atan2(siny, cosy);
  return {roll, pitch, yaw};
}

inline Pose pose_with_rpy(Pose pose, double roll, double pitch, double yaw) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  pose.qw = cr * cp * cy + sr * sp * sy;
  pose.qx = sr * cp * cy - cr * sp * sy;
  pose.qy = cr * sp * cy + sr * cp * sy;
  pose.qz = cr * cp * sy - sr * sp * cy;
  return normalize_pose(pose);
}

inline double pose_xy_distance(const Pose &lhs, const Pose &rhs) {
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline double pose_translation_distance(const Pose &lhs, const Pose &rhs) {
  return std::sqrt(square(lhs.x - rhs.x) + square(lhs.y - rhs.y) + square(lhs.z - rhs.z));
}

inline double pose_yaw_difference(const Pose &lhs, const Pose &rhs) {
  return std::abs(wrap_angle(pose_rpy(lhs)[2] - pose_rpy(rhs)[2]));
}

}  // namespace lingtu::localization::opt
