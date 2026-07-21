#include "octoplanner3d_core.hpp"

#include <octomap/OcTree.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

constexpr double kResolution = 0.2;

double center(int index)
{
  return (static_cast<double>(index) + 0.5) * kResolution;
}

void occupyCell(octomap::OcTree & tree, int ix, int iy, int iz)
{
  tree.updateNode(
    octomap::point3d(center(ix), center(iy), center(iz)),
    true);
}

void addFloor(octomap::OcTree & tree)
{
  for (int ix = -22; ix <= 22; ++ix) {
    for (int iy = -8; iy <= 8; ++iy) {
      occupyCell(tree, ix, iy, 0);
      occupyCell(tree, ix, iy, 0);
    }
  }
}

void addRemoteBoundsPillar(octomap::OcTree & tree)
{
  for (int iz = 1; iz <= 14; ++iz) {
    occupyCell(tree, 28, 28, iz);
  }
}

void addNarrowRail(octomap::OcTree & tree)
{
  for (int ix = -18; ix <= 18; ++ix) {
    occupyCell(tree, ix, 30, 5);
  }
}

void addLowBarrier(octomap::OcTree & tree)
{
  for (int iy = -8; iy <= 8; ++iy) {
    occupyCell(tree, 0, iy, 2);
  }
}

std::filesystem::path writeTestMap()
{
  octomap::OcTree tree(kResolution);
  tree.setProbHit(0.7);
  tree.setProbMiss(0.4);
  tree.setClampingThresMin(0.12);
  tree.setClampingThresMax(0.97);
  addFloor(tree);
  addNarrowRail(tree);
  addLowBarrier(tree);
  addRemoteBoundsPillar(tree);
  tree.updateInnerOccupancy();

  auto path = std::filesystem::temp_directory_path() / "octoplanner3d_no_air_climb.bt";
  if (!tree.writeBinary(path.string())) {
    throw std::runtime_error("failed to write test OctoMap: " + path.string());
  }
  return path;
}

std::filesystem::path writeHumpMap()
{
  octomap::OcTree tree(kResolution);
  tree.setProbHit(0.7);
  for (int ix = -10; ix <= 10; ++ix) {
    const int height = std::max(0, 3 - std::abs(ix));
    for (int iy = -1; iy <= 1; ++iy) {
      occupyCell(tree, ix, iy, height);
    }
  }
  occupyCell(tree, 15, -3, 0);
  occupyCell(tree, 15, 3, 12);
  tree.updateInnerOccupancy();

  auto path = std::filesystem::temp_directory_path() / "octoplanner3d_same_floor_hump.bt";
  if (!tree.writeBinary(path.string())) {
    throw std::runtime_error("failed to write same-floor hump OctoMap: " + path.string());
  }
  return path;
}

octoplanner3d::runtime::PlannerOptions testOptions()
{
  octoplanner3d::runtime::PlannerOptions options;
  options.robot_radius = 0.1;
  options.snap_search_radius_cells = 2;
  options.strict_direct_ground_support = true;
  options.ground_support_xy_radius_cells = 1;
  options.ground_support_depth_cells = 2;
  options.max_step_height = 0.45;
  options.max_slope = 0.0;
  options.max_iterations = 20000;
  return options;
}

octoplanner3d::runtime::PlanResult plan(
  const std::string & map_path,
  const octoplanner3d::runtime::Point & start,
  const octoplanner3d::runtime::Point & goal,
  const octoplanner3d::runtime::PlannerOptions & options = testOptions())
{
  octoplanner3d::runtime::PlanRequest request;
  request.start = start;
  request.goal = goal;
  request.options = options;
  return octoplanner3d::runtime::runPlan(map_path, request);
}

}  // namespace

int main()
{
  try {
    const auto map_path = writeTestMap();
    const octoplanner3d::runtime::Point start{center(-18), center(0), center(1)};

    const auto ground_result =
      plan(map_path.string(), start, {center(18), center(0), center(1)});
    if (!ground_result.ok || !ground_result.reached_goal) {
      std::cerr << "expected same-floor supported route to succeed; points="
                << ground_result.path.size()
                << " goal_error=" << ground_result.goal_error_m << "\n";
      return 1;
    }

    octoplanner3d::runtime::PlanRequest cached_request;
    cached_request.start = start;
    cached_request.goal = {center(18), center(0), center(1)};
    cached_request.options = testOptions();
    lingtu::nav::plan::MapIdentity map_identity{
      "smoke-map", 1, "fixture-sha256", "map"};
    octoplanner3d::runtime::PlannerSession session;
    const auto cached_first = session.run(map_path, map_identity, cached_request);
    const auto cached_second = session.run(map_path, map_identity, cached_request);
    if (!cached_first.ok || !cached_second.ok || session.mapLoadCount() != 1) {
      std::cerr << "same map identity did not reuse the in-memory OctoMap\n";
      return 11;
    }
    ++map_identity.version;
    const auto reloaded = session.run(map_path, map_identity, cached_request);
    if (!reloaded.ok || session.mapLoadCount() != 2) {
      std::cerr << "changed map identity did not reload the in-memory OctoMap\n";
      return 12;
    }

    octoplanner3d::runtime::PlanRequest cancelled_request;
    cancelled_request.start = start;
    cancelled_request.goal = {center(18), center(0), center(1)};
    cancelled_request.options = testOptions();
    const auto cancelled_result = octoplanner3d::runtime::runPlan(
      map_path,
      cancelled_request,
      []() { return true; });
    if (!cancelled_result.cancelled || cancelled_result.ok || !cancelled_result.path.empty()) {
      std::cerr << "cancelled planning request produced a usable path\n";
      return 8;
    }

    const auto air_result =
      plan(map_path.string(), start, {center(18), center(0), center(10)});
    if (air_result.ok) {
      std::cerr << "unsupported air goal unexpectedly planned; points="
                << air_result.path.size()
                << " goal_error=" << air_result.goal_error_m << "\n";
      return 2;
    }

    auto supported_layer_options = testOptions();
    supported_layer_options.snap_search_radius_cells = 6;
    supported_layer_options.ground_support_depth_cells = 5;
    supported_layer_options.support_height_m = 0.6;
    supported_layer_options.support_height_tolerance_m = 0.05;
    supported_layer_options.lowest_traversable_only = true;
    const auto supported_layer_result = plan(
      map_path.string(),
      {center(-18), center(0), center(5)},
      {center(18), center(0), center(5)},
      supported_layer_options);
    if (!supported_layer_result.ok || !supported_layer_result.reached_goal) {
      std::cerr << "expected support-height-constrained route to succeed\n";
      return 3;
    }
    for (const auto & point : supported_layer_result.path) {
      if (std::abs(point.z - center(3)) > 1e-6) {
        std::cerr << "lowest supported layer drifted to z=" << point.z << "\n";
        return 4;
      }
    }

    auto body_envelope_options = supported_layer_options;
    body_envelope_options.body_clearance_below_m = 0.25;
    body_envelope_options.body_clearance_above_m = 0.10;
    const auto body_envelope_result = plan(
      map_path.string(),
      {center(-18), center(0), center(5)},
      {center(18), center(0), center(5)},
      body_envelope_options);
    if (body_envelope_result.ok) {
      std::cerr << "below-body barrier was ignored by the cylinder envelope\n";
      return 5;
    }

    auto support_patch_options = supported_layer_options;
    support_patch_options.snap_search_radius_cells = 2;
    support_patch_options.support_patch_radius_cells = 2;
    support_patch_options.support_patch_min_samples = 3;
    const auto narrow_rail_result = plan(
      map_path.string(),
      {center(-18), center(30), center(8)},
      {center(18), center(30), center(8)},
      support_patch_options);
    if (narrow_rail_result.ok) {
      std::cerr << "narrow rail was incorrectly accepted as a support surface\n";
      return 6;
    }

    const auto hump_map_path = writeHumpMap();
    auto hump_options = testOptions();
    hump_options.robot_radius = 0.05;
    hump_options.snap_search_radius_cells = 1;
    hump_options.ground_support_xy_radius_cells = 0;
    hump_options.enable_preblocked_costmap = false;
    hump_options.obstacle_clearance_radius_cells = 0;
    hump_options.max_same_floor_z_excursion = 1.0;
    const auto hump_result = plan(
      hump_map_path.string(),
      {center(-8), center(0), center(1)},
      {center(8), center(0), center(1)},
      hump_options);
    if (!hump_result.ok || !hump_result.reached_goal) {
      std::cerr << "expected same-floor hump route to succeed before excursion gating\n";
      return 9;
    }
    hump_options.max_same_floor_z_excursion = 0.2;
    const auto excessive_excursion_result = plan(
      hump_map_path.string(),
      {center(-8), center(0), center(1)},
      {center(8), center(0), center(1)},
      hump_options);
    if (excessive_excursion_result.ok ||
        excessive_excursion_result.failure_reason != "same_floor_z_excursion") {
      std::cerr << "same-floor route with excessive z excursion was not rejected\n";
      return 10;
    }

    std::cout
      << "{\"ok\":true,"
      << "\"ground_points\":" << ground_result.path.size() << ","
      << "\"supported_layer_points\":" << supported_layer_result.path.size() << ","
      << "\"below_body_barrier_rejected\":true,"
      << "\"narrow_rail_rejected\":true,"
      << "\"air_goal_rejected\":true,"
      << "\"cancelled_request_rejected\":true,"
      << "\"same_floor_z_excursion_rejected\":true,"
      << "\"map_path\":\"" << map_path.string() << "\"}\n";
    return 0;
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << "\n";
    return 7;
  }
}
