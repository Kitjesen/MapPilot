#include "transform_buffer.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "test_transform_buffer failed: %s\n", message);
    std::exit(1);
  }
}

lingtu::nav::endpoint::RigidTransform transform(
    double x,
    double roll_rad,
    double stamp_s) {
  lingtu::nav::endpoint::RigidTransform value;
  value.translation.x = x;
  value.rotation.x = std::sin(roll_rad * 0.5);
  value.rotation.w = std::cos(roll_rad * 0.5);
  value.stamp_s = stamp_s;
  value.valid = true;
  return value;
}

lingtu::nav::endpoint::RigidTransform yawTransform(
    double x,
    double yaw_rad,
    double stamp_s) {
  auto value = transform(x, 0.0, stamp_s);
  value.rotation = {};
  value.rotation.z = std::sin(yaw_rad * 0.5);
  value.rotation.w = std::cos(yaw_rad * 0.5);
  return value;
}

}  // namespace

int main() {
  lingtu_dds_Odometry invalid_odom{};
  invalid_odom.pose.pose.position.x = 1.0;
  require(
      !lingtu::nav::endpoint::rigidTransformFromOdometry(invalid_odom).valid,
      "zero quaternion odometry must fail closed instead of becoming identity");
  invalid_odom.pose.pose.orientation.w = 1.0;
  require(
      lingtu::nav::endpoint::rigidTransformFromOdometry(invalid_odom).valid,
      "finite unit quaternion odometry must remain valid");

  lingtu::nav::endpoint::TransformBuffer buffer;
  buffer.push(10.0, transform(0.0, 0.0, 10.0));
  buffer.push(10.2, transform(0.2, 0.4, 10.2));

  const auto sample = buffer.sample(10.1, 0.2);
  require(sample.has_value(), "bracketed source timestamp must interpolate");
  require(std::abs(sample->translation.x - 0.1) < 1e-9, "translation must interpolate");
  const auto rotated = lingtu::nav::endpoint::rotatePoint(sample->rotation, {0.0, 1.0, 0.0});
  require(std::abs(rotated.z) > 0.1, "roll/pitch information must survive interpolation");
  require(!buffer.sample(9.0, 0.05).has_value(), "large source-time gap must be rejected");

  const auto source_time_tf = buffer.sample(10.0, 0.2);
  require(source_time_tf.has_value(), "source-time transform must be available");
  const auto source_time_point =
      lingtu::nav::endpoint::transformPoint(*source_time_tf, {1.0, 0.0, 0.0});
  require(
      std::abs(source_time_point.x - 1.0) < 1e-9,
      "a source-time point transform must not use the latest buffered translation");

  const auto small_delta = transform(0.49, 0.0, 10.3);
  require(
      !lingtu::nav::endpoint::transformJumpExceeds(
          transform(0.0, 0.0, 10.0), small_delta, 0.50, 0.25),
      "sub-threshold map-frame correction must preserve the current epoch");
  const auto large_translation = transform(0.51, 0.0, 10.3);
  require(
      lingtu::nav::endpoint::transformJumpExceeds(
          transform(0.0, 0.0, 10.0), large_translation, 0.50, 0.25),
      "large map-frame translation must start a new epoch");
  const auto large_yaw = yawTransform(0.0, 0.26, 10.3);
  require(
      lingtu::nav::endpoint::transformJumpExceeds(
          transform(0.0, 0.0, 10.0), large_yaw, 0.50, 0.25),
      "large map-frame rotation must start a new epoch");

  std::puts("test_transform_buffer passed");
  return 0;
}
