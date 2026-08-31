#include "planning/local/planner.hpp"
#include "planning/local/scan/backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

double percentile(std::vector<double> values, double q) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
      std::clamp(q, 0.0, 1.0) * static_cast<double>(values.size() - 1));
  return values[index];
}

std::vector<float> obstacleCloud(std::size_t count) {
  constexpr double kResolution = 0.05;
  const auto voxel_center = [=](double value, double lattice_origin) {
    return lattice_origin +
           (std::floor((value - lattice_origin) / kResolution) + 0.5) *
               kResolution;
  };
  std::vector<float> out;
  out.reserve(count * 4U);
  std::uint32_t state = 0x6d2b79f5U;
  const auto unit = [&state]() {
    state = state * 1664525U + 1013904223U;
    return static_cast<double>(state) /
           static_cast<double>(0xffffffffU);
  };
  for (std::size_t i = 0; i < count; ++i) {
    const double x = -3.5 + 7.0 * unit();
    const double y = std::copysign(
        2.0 + 1.5 * unit(), unit() < 0.5 ? -1.0 : 1.0);
    out.push_back(static_cast<float>(voxel_center(x, -5.0)));
    out.push_back(static_cast<float>(voxel_center(y, -5.0)));
    out.push_back(static_cast<float>(
        voxel_center(-0.1 + 0.8 * unit(), -3.0)));
    out.push_back(0.8F);
  }
  // A vertical band closes the global route itself but leaves a lateral gap.
  // Every measured iteration must therefore execute projected A* before spline
  // construction; this is not a route-clear microbenchmark.
  for (int yi = -7; yi <= 7; ++yi) {
    for (int zi = -2; zi <= 6; ++zi) {
      out.push_back(static_cast<float>(voxel_center(1.5, -5.0)));
      out.push_back(static_cast<float>(
          voxel_center(static_cast<double>(yi) * 0.10, -5.0)));
      out.push_back(static_cast<float>(
          voxel_center(static_cast<double>(zi) * 0.10, -3.0)));
      out.push_back(0.8F);
    }
  }
  return out;
}

std::vector<float> collisionCenters(const std::vector<float>& xyzh) {
  std::vector<float> xyz;
  xyz.reserve((xyzh.size() / 4U) * 3U);
  for (std::size_t offset = 0; offset + 3 < xyzh.size(); offset += 4U) {
    xyz.insert(xyz.end(), {xyzh[offset], xyzh[offset + 1], xyzh[offset + 2]});
  }
  return xyz;
}

}  // namespace

int main(int argc, char** argv) {
  const int iterations = argc > 1 ? std::max(1, std::atoi(argv[1])) : 200;
  const std::size_t obstacle_count =
      argc > 2 ? static_cast<std::size_t>(std::max(0, std::atoi(argv[2])))
               : 8000U;

  nav_kernel::LocalPlannerParams params;
  params.backend = nav_kernel::LocalPlannerBackend::Scan;
  params.vehicleLength = 0.60;
  params.vehicleWidth = 0.45;
  params.footprintPadding = 0.10;
  params.autonomySpeed = 0.40;
  params.maxSpeed = 1.0;
  params.useTraversabilityCost = false;
  params.scan.horizontalRange = 4.0;
  params.scan.maxSearchNodes = 12000;

  nav_kernel::local::scan::Backend planner(params);
  const std::vector<nav_kernel::Vec3> route{
      {0.0, 0.0, 0.0},
      {0.8, 0.2, 0.08},
      {1.6, 0.35, 0.18},
      {2.4, 0.2, 0.30},
      {3.0, 0.0, 0.38},
  };
  const std::vector<float> obstacles = obstacleCloud(obstacle_count);
  const std::vector<float> collision_xyz = collisionCenters(obstacles);

  std::vector<double> elapsed_ms;
  std::vector<double> grid_ms;
  std::vector<double> search_ms;
  std::vector<double> spline_ms;
  elapsed_ms.reserve(static_cast<std::size_t>(iterations));
  grid_ms.reserve(static_cast<std::size_t>(iterations));
  search_ms.reserve(static_cast<std::size_t>(iterations));
  spline_ms.reserve(static_cast<std::size_t>(iterations));
  int successes = 0;
  int astar_runs = 0;
  int max_expanded = 0;
  for (int iteration = -5; iteration < iterations; ++iteration) {
    nav_kernel::LocalPlanRequest input;
    input.robot.pose = {route.front(), 0.0};
    input.objective = nav_kernel::RouteTarget{{
        route.data(), static_cast<int>(route.size()),
        static_cast<std::uint64_t>(iteration + 6), false}};
    input.identity = {1, static_cast<std::uint64_t>(iteration + 6), 0};
    input.clock.timestampS = 1.0 + 0.05 * static_cast<double>(iteration + 5);
    input.environment.collision.occupiedXyz = collision_xyz.data();
    input.environment.collision.occupiedCount =
        static_cast<int>(collision_xyz.size() / 3U);
    input.environment.collision.resolution = params.scan.voxelResolution;
    // The official SCAN profile advertises a complete 5 m-high rolling map.
    // Keep this fixture's collision AABB large enough to cover that exact ROI.
    input.environment.collision.aabbMin = {-5.0, -5.0, -3.0};
    input.environment.collision.aabbMax = {5.0, 5.0, 3.0};
    input.environment.collision.resetEpoch = 1;
    input.environment.collision.observationSequence =
        static_cast<std::uint64_t>(iteration + 6);
    input.environment.collision.generation =
        static_cast<std::uint64_t>(iteration + 6);
    input.environment.collision.stampS = input.clock.timestampS;
    input.environment.collision.receiveStampS = input.clock.timestampS;
    input.environment.collision.complete = true;
    input.environment.collision.live = true;

    const auto started = std::chrono::steady_clock::now();
    const auto result = planner.plan(input);
    const double elapsed = std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - started)
                               .count();
    if (iteration >= 0) {
      elapsed_ms.push_back(elapsed);
      const auto debug = planner.debugSnapshot();
      grid_ms.push_back(debug.gridTimeMs);
      search_ms.push_back(debug.searchTimeMs);
      spline_ms.push_back(debug.splineTimeMs);
      if (result.status() == nav_kernel::LocalPlanStatus::Ready) ++successes;
      const int expanded = debug.expandedNodes;
      if (expanded > 0) ++astar_runs;
      max_expanded = std::max(max_expanded, expanded);
    }
  }

  const double p95_ms = percentile(elapsed_ms, 0.95);
  const double max_ms = elapsed_ms.empty()
                            ? 0.0
                            : *std::max_element(elapsed_ms.begin(), elapsed_ms.end());
  const bool meets_20hz = p95_ms <= 50.0 && max_ms <= 100.0;
  std::cout << std::fixed << std::setprecision(3)
            << "{\"scenario\":\"mapd_collision_3d_detour\",\"iterations\":" << iterations
            << ",\"obstacles\":" << collision_xyz.size() / 3U
            << ",\"successes\":" << successes
            << ",\"astar_runs\":" << astar_runs
            << ",\"p50_ms\":" << percentile(elapsed_ms, 0.50)
            << ",\"p95_ms\":" << p95_ms
            << ",\"grid_p50_ms\":" << percentile(grid_ms, 0.50)
            << ",\"grid_p95_ms\":" << percentile(grid_ms, 0.95)
            << ",\"search_p50_ms\":" << percentile(search_ms, 0.50)
            << ",\"search_p95_ms\":" << percentile(search_ms, 0.95)
            << ",\"spline_p50_ms\":" << percentile(spline_ms, 0.50)
            << ",\"spline_p95_ms\":" << percentile(spline_ms, 0.95)
            << ",\"max_ms\":" << max_ms
            << ",\"max_expanded_nodes\":" << max_expanded
            << ",\"budget_20hz_ms\":50.0"
            << ",\"meets_20hz\":"
            << (meets_20hz ? "true" : "false")
            << "}\n";
  return successes == iterations && astar_runs == iterations && meets_20hz ? 0 : 1;
}
