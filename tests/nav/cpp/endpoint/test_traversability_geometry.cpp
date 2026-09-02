#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "traversability/local_risk_projection.hpp"
#include "traversability/observed_safety_grid.hpp"
#include "traversability/traversability_geometry.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_traversability_geometry failed: %s\n", message);
    std::exit(1);
  }
}

lingtu::nav::endpoint::Quaternion quaternionFromRpy(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  lingtu::nav::endpoint::Quaternion q{};
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;
  q.w = cr * cp * cy + sr * sp * sy;
  return q;
}

float costAt(const lingtu::maps::layers::Grid2D &grid, double x, double y) {
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
  const auto origin = lingtu::nav::endpoint::traversabilitySensorOrigin(transform, offset);
  require(origin.has_value(), "finite sensor extrinsics must produce an origin");
  require(std::abs(origin->x - transform.translation.x) > 0.05,
          "roll/pitch must rotate sensor offset into map x");
  require(std::abs(origin->y - transform.translation.y) > 0.05,
          "roll/pitch must rotate sensor offset into map y");
  require(std::abs(origin->z - 4.0) > 0.01, "sensor origin must not use yaw-only z translation");
}

void testInvalidPoseFailsClosed() {
  lingtu::nav::endpoint::RigidTransform invalid;
  invalid.rotation = {0.0, 0.0, 0.0, 0.0};
  invalid.valid = true;
  invalid.stamp_s = 1.0;
  require(!lingtu::nav::endpoint::traversabilityPose(invalid).has_value(),
          "zero quaternion must not degrade to identity");
  require(!lingtu::nav::endpoint::traversabilitySensorOrigin(invalid, {0.0, 0.0, 0.0}).has_value(),
          "invalid pose must not publish a sensor origin");
}

void testRobotSelfFilterUsesTheCurrentBodyFrame() {
  lingtu::nav::endpoint::RigidTransform map_body;
  map_body.translation = {3.0, -2.0, 0.4};
  constexpr double kQuarterTurnRad = 1.57079632679489661923;
  map_body.rotation = quaternionFromRpy(0.0, 0.0, kQuarterTurnRad);
  map_body.yaw = kQuarterTurnRad;
  map_body.stamp_s = 5.0;
  map_body.valid = true;

  const auto inside = lingtu::nav::endpoint::transformPoint(map_body, {0.39, 0.17, 0.0});
  const auto outside_front =
      lingtu::nav::endpoint::transformPoint(map_body, {0.50, 0.0, 0.0});
  const auto outside_side =
      lingtu::nav::endpoint::transformPoint(map_body, {0.0, 0.25, 0.0});

  require(lingtu::nav::endpoint::pointInsideRobotSelfFilter(
              map_body, inside, 0.76, 0.31, 0.03),
          "Go2 self points must be filtered in the rotated body frame");
  require(!lingtu::nav::endpoint::pointInsideRobotSelfFilter(
              map_body, outside_front, 0.76, 0.31, 0.03),
          "points beyond the Go2 front must remain obstacles");
  require(!lingtu::nav::endpoint::pointInsideRobotSelfFilter(
              map_body, outside_side, 0.76, 0.31, 0.03),
          "points beyond the Go2 side must remain obstacles");
}

void testObservedUnknownSafetyGrid() {
  auto grid = lingtu::nav::endpoint::makeUnknownSafetyGrid(20, 20, 0.5, -5.0, -5.0, 100.0F);
  require(costAt(grid, 1.0, 2.0) == 100.0F, "unobserved forward cell must remain hard/unknown");

  require(lingtu::nav::endpoint::markObservedRayFree(grid, 0.0, 0.0, 3.0, 0.0),
          "finite in-grid ray must be accepted");
  require(costAt(grid, 1.0, 0.0) == 0.0F,
          "cells crossed by a sensor ray must become observed-free");
  require(costAt(grid, 1.0, 2.0) == 100.0F, "cells outside observed rays must remain hard/unknown");
  require(costAt(grid, 3.0, 0.0) == 0.0F,
          "ray hit cell is observed before obstacle classification");

  require(lingtu::nav::endpoint::raiseSafetyCostAt(grid, 3.0, 0.0, 100.0F),
          "in-grid obstacle hit must be accepted");
  require(costAt(grid, 3.0, 0.0) == 100.0F, "obstacle overlay must restore hard cost at hit cell");
}

void testLimitedForwardRaysLeaveRearAndSideRearUnknown() {
  auto grid = lingtu::nav::endpoint::makeUnknownSafetyGrid(80, 80, 0.1, -4.0, -4.0, 100.0F);

  constexpr double kSensorX = 0.402876074867229;
  constexpr double kSensorY = 0.0;
  require(lingtu::nav::endpoint::markObservedRayFree(grid, kSensorX, kSensorY, 3.0, 0.0),
          "forward center ray must be accepted");
  require(lingtu::nav::endpoint::markObservedRayFree(grid, kSensorX, kSensorY, 2.8, 0.45),
          "forward-left ray must be accepted");
  require(lingtu::nav::endpoint::markObservedRayFree(grid, kSensorX, kSensorY, 2.8, -0.45),
          "forward-right ray must be accepted");

  require(costAt(grid, 1.0, 0.0) == 0.0F,
          "cells crossed by forward rays must become observed-free");
  require(costAt(grid, -0.6, 0.0) == 100.0F,
          "rear cells not crossed by forward rays must remain unknown");
  require(costAt(grid, -0.2, 0.8) == 100.0F,
          "side-rear cells not crossed by forward rays must remain unknown");
  require(costAt(grid, -0.2, -0.8) == 100.0F,
          "opposite side-rear cells not crossed by forward rays must remain unknown");
}

void testMapRiskProjectsIntoRobotLocalOdomWindow() {
  auto map_grid = lingtu::maps::layers::makeGrid2D(6, 6, 1.0, 7.0, -3.0, 0.0F);
  // map <- odom is a 45-degree rotation about (10, 0). The risk must land
  // at its transformed stable odom coordinate, not retain map coordinates.
  constexpr int kRiskRow = 3;
  constexpr int kRiskCol = 4;
  map_grid.data[static_cast<std::size_t>(map_grid.index(kRiskRow, kRiskCol))] = 83.0F;

  lingtu::nav::endpoint::RigidTransform map_from_odom;
  map_from_odom.translation = {10.0, 0.0, 0.0};
  constexpr double kEighthTurnRad = 0.78539816339744830962;
  map_from_odom.rotation = quaternionFromRpy(0.0, 0.0, kEighthTurnRad);
  map_from_odom.yaw = kEighthTurnRad;
  map_from_odom.stamp_s = 42.0;
  map_from_odom.valid = true;

  const auto local = lingtu::nav::endpoint::projectRollingRiskGridToOdom(
      map_grid, map_from_odom, nav_kernel::Vec3{0.0, 0.0, 0.0}, 0.0F);
  require(local.ok(), "finite map<-odom transform must project a local risk grid");
  require(std::abs(local.grid->originX + 3.0) < 1e-9 && std::abs(local.grid->originY + 3.0) < 1e-9,
          "odom local grid must stay centered on the odom robot position");
  const auto expected_odom_point = lingtu::nav::endpoint::transformPoint(
      lingtu::nav::endpoint::inverseTransform(map_from_odom),
      {map_grid.originX + (static_cast<double>(kRiskCol) + 0.5) * map_grid.resolution,
       map_grid.originY + (static_cast<double>(kRiskRow) + 0.5) * map_grid.resolution, 0.0});
  require(costAt(*local.grid, expected_odom_point.x, expected_odom_point.y) >= 83.0F,
          "hard map risk must remain hard at its transformed odom coordinate");
  require(costAt(*local.grid, -2.5, -2.5) == 0.0F,
          "uncovered odom cells must stay open when no-data obstacles are disabled");

  const auto fail_closed = lingtu::nav::endpoint::projectRollingRiskGridToOdom(
      map_grid, map_from_odom, nav_kernel::Vec3{0.0, 0.0, 0.0}, 100.0F);
  require(fail_closed.ok(), "explicit no-data obstacle policy must remain supported");
  require(costAt(*fail_closed.grid, -2.5, -2.5) == 100.0F,
          "explicit no-data obstacle policy must close uncovered projection cells");
}

void testMapRiskProjectionRejectsInvalidTransform() {
  const auto grid = lingtu::maps::layers::makeGrid2D(2, 2, 0.5, -0.5, -0.5, 0.0F);
  lingtu::nav::endpoint::RigidTransform invalid;
  invalid.rotation = {0.0, 0.0, 0.0, 0.0};
  invalid.valid = true;
  invalid.stamp_s = 1.0;
  const auto local = lingtu::nav::endpoint::projectRollingRiskGridToOdom(
      grid, invalid, nav_kernel::Vec3{}, 0.0F);
  require(!local.ok() && local.reason == "map_odom_transform_invalid",
          "invalid map<-odom transform must not create a local risk grid");
}

void testControlRiskKeepsTerrainSeparateFromOccupancy() {
  auto geometry = lingtu::maps::layers::makeGrid2D(6, 6, 0.2, -0.6, -0.6, 100.0F);
  auto terrain = lingtu::maps::layers::makeGrid2D(6, 6, 0.2, -0.4, -0.6, 0.0F);
  terrain.data[static_cast<std::size_t>(terrain.index(3, 3))] = 64.0F;

  const auto selected = lingtu::nav::endpoint::makeControlRiskGrid(geometry, &terrain);

  require(selected.has_value(), "valid rolling geometry and terrain must produce control risk");
  require(selected->data.front() == 0.0F,
          "unknown occupancy geometry must not become a traversability obstacle");
  require(selected->data[static_cast<std::size_t>(selected->index(3, 3))] == 0.0F,
          "LiDAR occupancy must remain owned by the obstacle and collision inputs");
  require(selected->data[static_cast<std::size_t>(selected->index(3, 4))] == 64.0F,
          "terrain risk must be retained after alignment to the current rolling window");
}

void testSurfaceRiskUsesConnectedGroundInsteadOfObstacleTops() {
  std::vector<float> terrain;
  for (int row = -2; row <= 2; ++row) {
    for (int col = -2; col <= 2; ++col) {
      terrain.insert(terrain.end(), {col * 0.2F, row * 0.2F, 0.0F, 0.0F});
    }
  }
  terrain.insert(terrain.end(), {0.0F, 0.0F, 0.3F, 0.3F});
  terrain.insert(terrain.end(), {0.0F, 0.0F, 0.8F, 0.8F});

  const auto flat = lingtu::nav::endpoint::supportElevation(
      terrain, 0.0, 0.0, 0.2, 0.5, -1.0, 1.0, 0.2F);
  require(flat.has_value(), "connected terrain support must form an elevation grid");
  const auto flat_risk = lingtu::maps::layers::computeTerrainRisk(*flat);
  require(*std::max_element(flat_risk.stepHeight.data.begin(), flat_risk.stepHeight.data.end()) ==
              0.0F,
          "a vertical obstacle point must not become a ground-surface step");

  terrain.clear();
  for (int row = -2; row <= 2; ++row) {
    for (int col = -2; col <= 2; ++col) {
      const float z = col >= 0 ? 0.3F : 0.0F;
      terrain.insert(terrain.end(), {col * 0.2F, row * 0.2F, z, 0.0F});
    }
  }
  const auto stepped = lingtu::nav::endpoint::supportElevation(
      terrain, 0.0, 0.0, 0.2, 0.5, -1.0, 1.0, 0.2F);
  require(stepped.has_value(), "a connected support step must remain representable");
  const auto stepped_risk = lingtu::maps::layers::computeTerrainRisk(*stepped);
  require(*std::max_element(stepped_risk.stepHeight.data.begin(), stepped_risk.stepHeight.data.end()) >=
              0.29F,
          "real support-surface height changes must remain visible to terrain risk");
}
}  // namespace

int main() {
  testSixDofPoseAndSensorOrigin();
  testInvalidPoseFailsClosed();
  testRobotSelfFilterUsesTheCurrentBodyFrame();
  testObservedUnknownSafetyGrid();
  testLimitedForwardRaysLeaveRearAndSideRearUnknown();
  testMapRiskProjectsIntoRobotLocalOdomWindow();
  testMapRiskProjectionRejectsInvalidTransform();
  testControlRiskKeepsTerrainSeparateFromOccupancy();
  testSurfaceRiskUsesConnectedGroundInsteadOfObstacleTops();
  std::puts("test_traversability_geometry passed");
  return 0;
}
