#pragma once

#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

struct Quaternion {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double w{1.0};
};

struct RigidTransform {
  nav_kernel::Vec3 translation{};
  Quaternion rotation{};
  double yaw{0.0};
  double stamp_s{0.0};
  bool valid{false};
};

double quaternionYaw(const Quaternion &q);
bool quaternionIsFiniteAndNonzero(const Quaternion &q);
Quaternion normalizeQuaternion(Quaternion q);
double quaternionRoll(const Quaternion &q);
double quaternionPitch(const Quaternion &q);
Quaternion inverseQuaternion(const Quaternion &q);
Quaternion multiplyQuaternions(const Quaternion &lhs, const Quaternion &rhs);
nav_kernel::Vec3 rotatePoint(const Quaternion &q, const nav_kernel::Vec3 &point);
nav_kernel::Vec3 transformPoint(const RigidTransform &transform,
                                const nav_kernel::Vec3 &point);
RigidTransform inverseTransform(const RigidTransform &transform);
RigidTransform composeTransforms(const RigidTransform &parent_middle,
                                 const RigidTransform &middle_child);
bool transformJumpExceeds(const RigidTransform &previous, const RigidTransform &next,
                          double translation_threshold_m, double yaw_threshold_rad);
nav_kernel::Pose transformPose(const RigidTransform &transform, const nav_kernel::Pose &pose);

}  // namespace lingtu::nav::endpoint
