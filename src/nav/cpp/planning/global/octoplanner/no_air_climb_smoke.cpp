#include "octoplanner3d_core.hpp"

#include <octomap/OcTree.h>

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

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

std::filesystem::path writeWallClimbMap()
{
  octomap::OcTree tree(kResolution);
  tree.setProbHit(0.7);
  for (int ix = -8; ix <= -2; ++ix) {
    for (int iy = -4; iy <= 4; ++iy) {
      occupyCell(tree, ix, iy, 0);
    }
  }
  for (int iy = -4; iy <= 4; ++iy) {
    for (int iz = 0; iz <= 10; ++iz) {
      occupyCell(tree, 1, iy, iz);
    }
  }
  occupyCell(tree, 8, 8, 12);
  tree.updateInnerOccupancy();

  auto path = std::filesystem::temp_directory_path() /
    "octoplanner3d_wall_climb.bt";
  if (!tree.writeBinary(path.string())) {
    throw std::runtime_error("failed to write wall-climb OctoMap: " + path.string());
  }
  return path;
}

std::filesystem::path writeOverlayMap()
{
  octomap::OcTree tree(kResolution);
  tree.setProbHit(0.7);
  addFloor(tree);
  addRemoteBoundsPillar(tree);
  tree.updateInnerOccupancy();

  auto path = std::filesystem::temp_directory_path() / "octoplanner3d_temporary_overlay.bt";
  if (!tree.writeBinary(path.string())) {
    throw std::runtime_error("failed to write temporary-overlay OctoMap: " + path.string());
  }
  return path;
}

std::filesystem::path writeOverlayBudgetMap()
{
  constexpr double resolution = 0.01;
  octomap::OcTree tree(resolution);
  tree.setProbHit(0.7);
  for (const double x : {-2.0, 2.0}) {
    for (const double y : {-2.0, 2.0}) {
      for (const double z : {0.0, 3.0}) {
        tree.updateNode(octomap::point3d(x, y, z), true);
      }
    }
  }
  tree.updateInnerOccupancy();

  auto path = std::filesystem::temp_directory_path() /
    "octoplanner3d_temporary_overlay_budget.bt";
  if (!tree.writeBinary(path.string())) {
    throw std::runtime_error("failed to write temporary-overlay budget OctoMap: " + path.string());
  }
  return path;
}

bool pathIntersects(
  const std::vector<octoplanner3d::runtime::Point> & path,
  const lingtu::nav::plan::GlobalPlanBlockedRegion & region)
{
  if (path.empty()) {
    return false;
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    const auto & from = path[index - 1U];
    const auto & to = path[index];
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double dz = to.z - from.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.05)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double t = static_cast<double>(sample) / static_cast<double>(samples);
      const double z = from.z + dz * t;
      if (z < region.min_z || z > region.max_z) {
        continue;
      }
      const double x = from.x + dx * t - region.center.x;
      const double y = from.y + dy * t - region.center.y;
      if (x * x + y * y <= region.radius_xy_m * region.radius_xy_m) {
        return true;
      }
    }
  }
  return false;
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
    if (ground_result.path.size() > 6U) {
      std::cerr << "safe route retained raw voxel zigzags; points="
                << ground_result.path.size() << "\n";
      return 26;
    }

    auto support_footprint_options = testOptions();
    support_footprint_options.snap_search_radius_cells = 4;
    support_footprint_options.robot_radius = 0.1;
    support_footprint_options.ground_support_xy_radius_cells = 2;
    const auto support_footprint_result = plan(
      map_path.string(),
      {center(-18), center(7), center(1)},
      {center(18), center(7), center(1)},
      support_footprint_options);
    if (!support_footprint_result.ok || !support_footprint_result.reached_goal) {
      std::cerr << "support-footprint route unexpectedly failed\n";
      return 27;
    }
    for (const auto & point : support_footprint_result.path) {
      if (point.y > center(6) + 1e-6) {
        std::cerr << "support footprint did not keep the route off the floor edge; y="
                  << point.y << "\n";
        return 28;
      }
    }

    octoplanner3d::runtime::PlanRequest cached_request;
    cached_request.start = start;
    cached_request.goal = {center(18), center(0), center(1)};
    cached_request.options = testOptions();
    lingtu::nav::plan::MapIdentity map_identity{
      "smoke-map", 1, "map"};
    octoplanner3d::runtime::PlannerSession session;
    const auto cached_first = session.run(map_path, map_identity, cached_request);
    const auto cached_second = session.run(map_path, map_identity, cached_request);
    if (!cached_first.ok || !cached_second.ok || session.mapLoadCount() != 1 ||
        session.prepareCount() != 1) {
      std::cerr << "same map identity did not reuse prepared planner state\n";
      return 11;
    }
    ++map_identity.content_epoch;
    const auto reloaded = session.run(map_path, map_identity, cached_request);
    if (!reloaded.ok || session.mapLoadCount() != 2 || session.prepareCount() != 2) {
      std::cerr << "changed map identity did not rebuild prepared planner state\n";
      return 12;
    }

    const auto overlay_map_path = writeOverlayMap();
    lingtu::nav::plan::MapIdentity overlay_map_identity{
      "overlay-smoke-map", 1, "map"};
    octoplanner3d::runtime::PlannerSession overlay_session;
    octoplanner3d::runtime::PlanRequest overlay_request;
    overlay_request.start = start;
    overlay_request.goal = {center(18), center(0), center(1)};
    overlay_request.options = testOptions();
    const auto unobstructed =
      overlay_session.run(overlay_map_path, overlay_map_identity, overlay_request);
    lingtu::nav::plan::GlobalPlanBlockedRegion blocked_region;
    blocked_region.center = {center(0), center(0), center(1)};
    blocked_region.radius_xy_m = 0.8;
    blocked_region.min_z = center(0);
    blocked_region.max_z = center(3);
    if (!unobstructed.ok || !unobstructed.reached_goal ||
        !pathIntersects(unobstructed.path, blocked_region)) {
      std::cerr << "temporary-overlay fixture did not produce the expected direct baseline\n";
      return 17;
    }
    overlay_request.temporary_overlay.revision = 1U;
    overlay_request.temporary_overlay.frame_epoch = 7U;
    overlay_request.temporary_overlay.obstacle_generation = 11U;
    overlay_request.temporary_overlay.traversability_generation = 13U;
    overlay_request.temporary_overlay.blocked_regions.push_back(blocked_region);
    const auto detoured =
      overlay_session.run(overlay_map_path, overlay_map_identity, overlay_request);
    if (!detoured.ok || !detoured.reached_goal ||
        pathIntersects(detoured.path, blocked_region)) {
      std::cerr << "request-scoped temporary overlay did not force a safe detour\n";
      return 18;
    }
    if (detoured.overlay_revision != overlay_request.temporary_overlay.revision ||
        detoured.overlay_frame_epoch != overlay_request.temporary_overlay.frame_epoch ||
        detoured.overlay_obstacle_generation !=
          overlay_request.temporary_overlay.obstacle_generation ||
        detoured.overlay_traversability_generation !=
          overlay_request.temporary_overlay.traversability_generation ||
        overlay_session.mapLoadCount() != 1U) {
      std::cerr << "temporary overlay identity was not echoed or reloaded the immutable map\n";
      return 19;
    }
    auto invalid_overlay_request = overlay_request;
    invalid_overlay_request.temporary_overlay.revision = 0U;
    const auto invalid_overlay =
      overlay_session.run(overlay_map_path, overlay_map_identity, invalid_overlay_request);
    if (invalid_overlay.ok || !invalid_overlay.path.empty() ||
        invalid_overlay.failure_reason != "temporary_overlay_revision_missing") {
      std::cerr << "temporary overlay without an identity was not rejected\n";
      return 20;
    }
    octoplanner3d::runtime::PlannerSession invalid_overlay_session;
    const auto invalid_before_load = invalid_overlay_session.run(
      overlay_map_path, overlay_map_identity, invalid_overlay_request);
    if (invalid_before_load.ok || invalid_overlay_session.mapLoadCount() != 0U) {
      std::cerr << "invalid temporary overlay caused an immutable map load\n";
      return 22;
    }
    const auto rejectedBeforeLoad = [&](const auto & overlay, const std::string & reason) {
      octoplanner3d::runtime::PlannerSession rejected_session;
      auto rejected_request = overlay_request;
      rejected_request.temporary_overlay = overlay;
      const auto rejected = rejected_session.run(
        overlay_map_path, overlay_map_identity, rejected_request);
      return !rejected.ok && rejected.path.empty() &&
             rejected.failure_reason == reason && rejected_session.mapLoadCount() == 0U;
    };
    auto missing_frame = overlay_request.temporary_overlay;
    missing_frame.frame_epoch = 0U;
    auto missing_obstacles = overlay_request.temporary_overlay;
    missing_obstacles.obstacle_generation = 0U;
    auto missing_traversability = overlay_request.temporary_overlay;
    missing_traversability.traversability_generation = 0U;
    auto invalid_geometry = overlay_request.temporary_overlay;
    invalid_geometry.blocked_regions.front().radius_xy_m = 0.0;
    auto overflowing_geometry = overlay_request.temporary_overlay;
    overflowing_geometry.blocked_regions.front().center.x =
      std::numeric_limits<double>::max();
    overflowing_geometry.blocked_regions.front().radius_xy_m =
      std::numeric_limits<double>::max();
    auto too_many_regions = overlay_request.temporary_overlay;
    too_many_regions.blocked_regions.assign(65U, blocked_region);
    if (!rejectedBeforeLoad(missing_frame, "temporary_overlay_frame_epoch_missing") ||
        !rejectedBeforeLoad(
          missing_obstacles, "temporary_overlay_obstacle_generation_missing") ||
        !rejectedBeforeLoad(
          missing_traversability, "temporary_overlay_traversability_generation_missing") ||
        !rejectedBeforeLoad(invalid_geometry, "temporary_overlay_region_invalid") ||
        !rejectedBeforeLoad(overflowing_geometry, "temporary_overlay_region_invalid") ||
        !rejectedBeforeLoad(
          too_many_regions, "temporary_overlay_region_limit_exceeded")) {
      std::cerr << "temporary overlay validation was not fail-closed before map loading\n";
      return 23;
    }

    const auto overlay_budget_map_path = writeOverlayBudgetMap();
    lingtu::nav::plan::MapIdentity overlay_budget_map_identity{
      "overlay-budget-smoke-map", 1, "map"};
    octoplanner3d::runtime::PlannerSession overlay_budget_session;
    const auto over_budget = overlay_budget_session.run(
      overlay_budget_map_path, overlay_budget_map_identity, overlay_request);
    if (over_budget.ok || !over_budget.path.empty() ||
        over_budget.failure_reason != "temporary_overlay_raster_budget_exceeded" ||
        overlay_budget_session.mapLoadCount() != 1U) {
      std::cerr << "temporary overlay raster budget was not enforced\n";
      return 24;
    }
    auto cleared_overlay_request = overlay_request;
    cleared_overlay_request.temporary_overlay = {};
    const auto cleared_overlay =
      overlay_session.run(overlay_map_path, overlay_map_identity, cleared_overlay_request);
    if (!cleared_overlay.ok || !cleared_overlay.reached_goal ||
        !pathIntersects(cleared_overlay.path, blocked_region) ||
        overlay_session.mapLoadCount() != 1U) {
      std::cerr << "temporary overlay leaked into a later request or reloaded the saved map\n";
      return 21;
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
    if (air_result.ok || air_result.failure_reason != "goal_snap_exhausted") {
      std::cerr << "unsupported air goal unexpectedly planned; points="
                << air_result.path.size()
                << " goal_error=" << air_result.goal_error_m << "\n";
      return 2;
    }

    const auto wall_map_path = writeWallClimbMap();
    octoplanner3d::runtime::PlannerOptions wall_options;
    wall_options.robot_radius = 0.1;
    wall_options.snap_search_radius_cells = 2;
    wall_options.ground_support_depth_cells = 2;
    wall_options.enable_preblocked_costmap = false;
    wall_options.obstacle_clearance_radius_cells = 0;
    wall_options.max_iterations = 20000;
    const auto wall_climb_result = plan(
      wall_map_path.string(),
      {center(-6), center(0), center(1)},
      {center(-1), center(0), center(8)},
      wall_options);
    if (wall_climb_result.ok) {
      std::cerr << "sideways wall cells were incorrectly accepted as ground support\n";
      return 25;
    }

    const auto outside_map_result =
      plan(map_path.string(), start, {center(80), center(0), center(1)});
    if (outside_map_result.ok ||
        outside_map_result.failure_reason != "goal_outside_static_map") {
      std::cerr << "far outside-map goal did not report static-map boundary failure; reason="
                << outside_map_result.failure_reason << "\n";
      return 13;
    }

    auto outside_map_snap_options = testOptions();
    outside_map_snap_options.snap_search_radius_cells = 5;
    const auto outside_map_snap_result =
      plan(
        map_path.string(),
        start,
        {center(-26), center(0), center(1)},
        outside_map_snap_options);
    if (!outside_map_snap_result.ok ||
        outside_map_snap_result.reached_goal ||
        !outside_map_snap_result.failure_reason.empty()) {
      std::cerr << "near outside-map goal did not preserve snapped terminal semantics; ok="
                << outside_map_snap_result.ok
                << " reached=" << outside_map_snap_result.reached_goal
                << " reason=" << outside_map_snap_result.failure_reason << "\n";
      return 14;
    }

    const auto start_outside_map_result =
      plan(
        map_path.string(),
        {center(-80), center(0), center(1)},
        {center(18), center(0), center(1)});
    if (start_outside_map_result.ok ||
        start_outside_map_result.failure_reason != "start_outside_static_map") {
      std::cerr << "far outside-map start did not report static-map boundary failure; reason="
                << start_outside_map_result.failure_reason << "\n";
      return 15;
    }

    const auto start_snap_result =
      plan(
        map_path.string(),
        {center(18), center(0), center(10)},
        {center(18), center(0), center(1)});
    if (start_snap_result.ok ||
        start_snap_result.failure_reason != "start_snap_exhausted") {
      std::cerr << "in-map unsupported start did not report snap exhaustion; reason="
                << start_snap_result.failure_reason << "\n";
      return 16;
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
      << "\"outside_static_map_rejected\":true,"
      << "\"start_static_map_boundary_rejected\":true,"
      << "\"same_floor_z_excursion_rejected\":true,"
      << "\"temporary_overlay_detour\":true,"
      << "\"temporary_overlay_request_scoped\":true,"
      << "\"map_path\":\"" << map_path.string() << "\"}\n";
    return 0;
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << "\n";
    return 7;
  }
}
