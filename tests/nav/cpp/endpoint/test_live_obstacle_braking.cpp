#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "lingtu/maps/layers/rolling_occupancy.hpp"
#include "planning/local/scan/grid.hpp"

namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void observe(lingtu::maps::layers::RollingOccupancyGrid& map,
             const std::vector<float>& points, std::int64_t stamp) {
  lingtu::maps::MapCloudFrame frame;
  frame.cloud.frame_id = "map";
  frame.cloud.stamp_ns = stamp;
  frame.cloud.point_count = points.size() / 3U;
  frame.cloud.interleaved = {points.data(), points.size()};
  frame.sensor_origin_x_m = 0.025;
  frame.sensor_origin_y_m = 0.025;
  frame.sensor_origin_z_m = 0.525;
  map.Update(frame);
}

nav_kernel::LocalCollisionMapView collisionView(
    const lingtu::maps::layers::RollingInflatedSnapshot& snapshot) {
  nav_kernel::LocalCollisionMapView view;
  view.inflatedBits = snapshot.occupied_bits.data();
  view.inflatedBytes = snapshot.occupied_bits.size();
  view.sizeX = snapshot.size_x;
  view.sizeY = snapshot.size_y;
  view.sizeZ = snapshot.size_z;
  view.resolution = snapshot.resolution_m;
  view.aabbMin = {snapshot.origin_x_m, snapshot.origin_y_m, snapshot.origin_z_m};
  view.aabbMax = {
      snapshot.origin_x_m + snapshot.size_x * snapshot.resolution_m,
      snapshot.origin_y_m + snapshot.size_y * snapshot.resolution_m,
      snapshot.origin_z_m + snapshot.size_z * snapshot.resolution_m};
  view.resetEpoch = 1;
  view.observationSequence = snapshot.generation;
  view.generation = snapshot.generation;
  view.stampS = static_cast<double>(snapshot.stamp_ns) / 1e9;
  view.receiveStampS = view.stampS;
  view.live = true;
  view.complete = true;
  return view;
}

void testNewObstacleRemainsInBrakingSweepUntilObservedClear(double dx, double dy) {
  lingtu::maps::layers::RollingOccupancyConfig config;
  config.auto_roll = false;
  lingtu::maps::layers::RollingOccupancyGrid map(config);
  map.Reset("map", 0.0, 0.0, 0.5, 1);

  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.checkObstacle = true;
  params.scan.voxelResolution = config.resolution_m;
  params.scan.cylinderOffset = 0.18;
  const nav_kernel::Pose body{{0.025, 0.025, 0.525}, 0.0};
  const nav_kernel::Twist motion{0.5 * dx, 0.5 * dy, 0.0};
  const double distance = dx != 0.0 ? 0.6 : 0.45;
  const std::vector<float> farReturn{
      static_cast<float>(0.025 + 3.0 * dx),
      static_cast<float>(0.025 + 3.0 * dy), 0.525F};
  const std::vector<float> newObstacle{
      static_cast<float>(0.025 + distance * dx),
      static_cast<float>(0.025 + distance * dy), 0.525F};
  const std::vector<float> differentDirection{
      static_cast<float>(0.025 - 3.0 * dy),
      static_cast<float>(0.025 + 3.0 * dx), 0.525F};
  std::int64_t stamp = 1000000000;
  const auto scan = [&](const std::vector<float>& points) {
    stamp += 100000000;
    observe(map, points, stamp);
  };
  const auto brakingState = [&]() {
    const auto snapshot = map.InflatedSnapshot();
    nav_kernel::LocalPlanRequest request;
    request.environment.collision = collisionView(snapshot);
    const nav_kernel::local::scan::Grid grid(params, request);
    require(grid.valid(), "mapd collision bitmap was rejected by SCAN");
    require(grid.inflatedOccupancy(body.position, body.yaw) == 0,
            "fixture starts in an occupied footprint instead of testing braking");
    return grid.brakingOccupancy(body, motion, 0.35, 0.5, 1.0);
  };

  // Repeated unobstructed rays establish the old free-space evidence.
  for (int i = 0; i < 4; ++i) scan(farReturn);
  require(brakingState() == 0, "unobstructed motion should have a clear braking sweep");
  scan(newObstacle);
  require(brakingState() == 1, "the first new obstacle return must block measured motion");

  // A later scan looking elsewhere is not evidence that the obstacle left.
  for (int i = 0; i < 3; ++i) {
    scan(differentDirection);
    require(brakingState() == 1,
            "unobserved new obstacle disappeared from the SCAN braking sweep");
  }

  // Actual free rays through the old endpoint may release the motion again.
  for (int i = 0; i < 4; ++i) scan(farReturn);
  require(brakingState() == 0, "observed-clear obstacle must not block motion permanently");
}

}  // namespace

int main() {
  try {
    testNewObstacleRemainsInBrakingSweepUntilObservedClear(1.0, 0.0);
    testNewObstacleRemainsInBrakingSweepUntilObservedClear(-1.0, 0.0);
    testNewObstacleRemainsInBrakingSweepUntilObservedClear(0.0, 1.0);
    testNewObstacleRemainsInBrakingSweepUntilObservedClear(0.0, -1.0);
    std::cout << "live obstacle braking passed in four translation directions\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
