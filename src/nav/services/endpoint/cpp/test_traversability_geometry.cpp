#include "observed_safety_grid.hpp"
#include "traversability_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "test_traversability_geometry failed: %s\n", message);
    std::exit(1);
  }
}

lingtu_dds_Quaternion quaternionFromRpy(
    double roll,
    double pitch,
    double yaw) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  lingtu_dds_Quaternion q{};
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  return q;
}

float costAt(
    const lingtu::maps::layers::Grid2D& grid,
    double x,
    double y) {
  const int col = static_cast<int>(std::floor((x - grid.originX) / grid.resolution));
  const int row = static_cast<int>(std::floor((y - grid.originY) / grid.resolution));
  require(row >= 0 && row < grid.rows && col >= 0 && col < grid.cols,
          "test coordinate must be inside grid");
  return grid.data[static_cast<std::size_t>(grid.index(row, col))];
}

void testSixDofPoseAndSensorOrigin() {
  constexpr double kRoll = 0.40;
  constexpr double kPitch = -0.25;
  constexpr double kYaw = 0.30;
  lingtu::nav::endpoint::RigidTransform transform;
  transform.translation = {1.0, 2.0, 3.0};
  transform.rotation = quaternionFromRpy(kRoll, kPitch, kYaw);
  transform.yaw = kYaw;
  transform.stamp_s = 42.0;
  transform.valid = true;

  const auto pose = lingtu::nav::endpoint::traversabilityPose(transform);
  require(pose.has_value(), "finite 6DoF source-time pose must be accepted");
  require(std::abs(pose->roll - kRoll) < 1e-9, "roll must survive pose extraction");
  require(std::abs(pose->pitch - kPitch) < 1e-9, "pitch must survive pose extraction");
  require(std::abs(pose->yaw - kYaw) < 1e-9, "yaw must survive pose extraction");

  const nav_kernel::Vec3 offset{0.0, 0.0, 1.0};
  const auto origin = lingtu::nav::endpoint::traversabilitySensorOrigin(
      transform,
      offset);
  require(origin.has_value(), "finite sensor extrinsics must produce an origin");
  require(std::abs(origin->x - transform.translation.x) > 0.05,
          "roll/pitch must rotate sensor offset into map x");
  require(std::abs(origin->y - transform.translation.y) > 0.05,
          "roll/pitch must rotate sensor offset into map y");
  require(std::abs(origin->z - 4.0) > 0.01,
          "sensor origin must not use yaw-only z translation");
}

void testInvalidPoseFailsClosed() {
  lingtu::nav::endpoint::RigidTransform invalid;
  invalid.valid = true;
  invalid.stamp_s = 1.0;
  require(!lingtu::nav::endpoint::traversabilityPose(invalid).has_value(),
          "zero quaternion must not degrade to identity");
  require(
      !lingtu::nav::endpoint::traversabilitySensorOrigin(invalid, {0.0, 0.0, 0.0})
           .has_value(),
      "invalid pose must not publish a sensor origin");
}

void testObservedUnknownSafetyGrid() {
  auto grid = lingtu::nav::endpoint::makeUnknownSafetyGrid(
      20,
      20,
      0.5,
      -5.0,
      -5.0,
      100.0F);
  require(costAt(grid, 1.0, 2.0) == 100.0F,
          "unobserved forward cell must remain hard/unknown");

  require(
      lingtu::nav::endpoint::markObservedRayFree(grid, 0.0, 0.0, 3.0, 0.0),
      "finite in-grid ray must be accepted");
  require(costAt(grid, 1.0, 0.0) == 0.0F,
          "cells crossed by a sensor ray must become observed-free");
  require(costAt(grid, 1.0, 2.0) == 100.0F,
          "cells outside observed rays must remain hard/unknown");
  require(costAt(grid, 3.0, 0.0) == 0.0F,
          "ray hit cell is observed before obstacle classification");

  require(
      lingtu::nav::endpoint::raiseSafetyCostAt(grid, 3.0, 0.0, 100.0F),
      "in-grid obstacle hit must be accepted");
  require(costAt(grid, 3.0, 0.0) == 100.0F,
          "obstacle overlay must restore hard cost at hit cell");
}

}  // namespace

int main() {
  testSixDofPoseAndSensorOrigin();
  testInvalidPoseFailsClosed();
  testObservedUnknownSafetyGrid();
  std::puts("test_traversability_geometry passed");
  return 0;
}
