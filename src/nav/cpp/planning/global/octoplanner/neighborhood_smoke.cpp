#include "global_planner.h"
#include "octoplanner3d_core.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using Planner = global_planner::OctoPlanner3D;
using Outcome = Planner::SearchInfo::Outcome;

double center(int cell) { return (cell + 0.5) * 0.1; }

global_planner::PointPose point(int x, int z)
{
  return {center(x), center(0), center(z)};
}

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

std::shared_ptr<octomap::OcTree> makeMap(bool step, bool wall = false)
{
  auto map = std::make_shared<octomap::OcTree>(0.1);
  const auto occupy = [&](int x, int y, int z) {
    map->updateNode(octomap::point3d(center(x), center(y), center(z)), true);
  };
  // Model the traversable volume as measured free ray evidence.  Unknown
  // cells are reserved for the dedicated gap/absence cases below.
  for (int x = -8; x <= 8; ++x)
    for (int y = -4; y <= 4; ++y)
      for (int z = 0; z <= 10; ++z)
        map->updateNode(octomap::point3d(center(x), center(y), center(z)), false);
  for (int x = -8; x <= 8; ++x) {
    for (int y = -4; y <= 4; ++y) {
      // A 0.30 m riser needs an extended edge with two support layers. A
      // 0.20 m riser can use basic neighbors after correcting footprint support.
      for (int z = 0; z <= (step && x >= 0 ? 3 : 0); ++z) occupy(x, y, z);
    }
  }
  // A remote pillar establishes vertical bounds without supporting the route.
  for (int z = 0; z <= 10; ++z) occupy(15, 7, z);
  if (wall) {
    for (int y = -5; y <= 5; ++y) {
      for (int z = 1; z <= 8; ++z) occupy(0, y, z);
    }
  }
  map->updateInnerOccupancy();
  return map;
}

global_planner::PlannerConfig config()
{
  global_planner::PlannerConfig result;
  result.robot_radius = 0.04;
  result.snap_search_radius_cells = 0;
  result.support_depth_cells = 2;
  result.max_iterations = 20000;
  result.enable_preblocked_costmap = false;
  result.obstacle_clearance_radius_cells = 0;
  return result;
}

std::vector<global_planner::PointPose> run(
  Planner & planner, global_planner::PointPose start, global_planner::PointPose goal)
{
  planner.makePlan(start, goal);
  std::vector<global_planner::PointPose> path;
  planner.getPlannerResults(path);
  return path;
}

std::shared_ptr<octomap::OcTree> makeFullSizeStairs(bool wall = false, bool gap = false)
{
  auto map = std::make_shared<octomap::OcTree>(0.1);
  const auto occupy = [&](int x, int y, int z) {
    map->updateNode(octomap::point3d(center(x), center(y), center(z)), true);
  };
  // Saved-ray maps contain explicit free cells above each tread.  Leave the
  // requested gap completely absent so it remains unknown and impassable.
  for (int x = -25; x <= 60; ++x) {
    if (gap && x >= 9 && x <= 21) continue;
    for (int y = -11; y <= 11; ++y)
      for (int z = 0; z <= 40; ++z)
        map->updateNode(octomap::point3d(center(x), center(y), center(z)), false);
  }
  // Rasterized 0.15 m rise / 0.30 m run, 2.2 m wide; the current factory flight.
  for (int x = -25; x <= 60; ++x) {
    if (gap && x >= 9 && x <= 21) continue;
    const int steps = x < 0 ? 0 : std::min(8, x / 3 + 1);
    const int height = (3 * steps + 1) / 2;
    for (int y = -11; y <= 11; ++y) {
      for (int z = 0; z <= height; ++z) occupy(x,y,z);
    }
  }
  if (wall) {
    for (int y = -12; y <= 12; ++y)
      for (int z = 0; z <= 38; ++z) occupy(12,y,z);
  }
  occupy(70,15,40);
  map->updateInnerOccupancy();
  return map;
}

void checkFullSizeStairs()
{
  Planner planner;
  auto options = config();
  options.robot_radius = 0.640512;
  options.max_iterations = 30000;
  // The synthetic staircase has complete direct support. Keep this large
  // geometry regression on the strict diagnostic predicate; the production
  // default remains the relaxed one-voxel fallback exercised below.
  options.direct_support_only = true;
  planner.setConfig(options);
  planner.setOctomap(makeFullSizeStairs());
  const auto bottom = point(-15,2);
  const auto top = point(45,14);
  require(!run(planner,bottom,top).empty(), "full-size factory stair ascent failed");
  require(!run(planner,top,bottom).empty(), "full-size factory stair descent failed");
  planner.setOctomap(makeFullSizeStairs(true));
  require(run(planner,bottom,top).empty(), "full-size stairs crossed a wall");
  planner.setOctomap(makeFullSizeStairs(false,true));
  require(run(planner,bottom,top).empty(), "full-size stairs crossed a missing flight");

  auto flat = std::make_shared<octomap::OcTree>(0.1);
  for (int x = -20; x <= 20; ++x)
    for (int y = -20; y <= 20; ++y)
      for (int z = 0; z <= 10; ++z)
        flat->updateNode(octomap::point3d(center(x),center(y),center(z)), false);
  for (int x = -20; x <= 20; ++x)
    for (int y = -20; y <= 20; ++y)
      flat->updateNode(octomap::point3d(center(x),center(y),center(0)), true);
  // A low shelf beam inside the body envelope is not a tread. All support
  // samples remain on the flat floor, so lateral body clearance must hold.
  for (int y = -10; y <= 10; ++y)
    flat->updateNode(octomap::point3d(center(5),center(y),center(5)), true);
  flat->updateNode(octomap::point3d(center(25),center(25),center(15)), true);
  flat->updateInnerOccupancy();
  planner.setOctomap(flat);
  require(run(planner,point(0,2),point(0,2)).empty(), "flat-ground shelf beam lost body clearance");
}

void checkQuantizedFloor()
{
  // A five-centimetre change remains connected at a configured 30-degree
  // slope. Voxel-centre angles must not remove upstream's adjacent edges.
  auto map = std::make_shared<octomap::OcTree>(0.05);
  const auto c = [](int i) { return (i + 0.5) * 0.05; };
  for (int x = -10; x <= 10; ++x)
    for (int y = -3; y <= 3; ++y) {
      for (int z = 0; z <= 8; ++z)
        map->updateNode(octomap::point3d(c(x), c(y), c(z)), false);
      for (int z = 0; z <= (x >= 0 ? 1 : 0); ++z)
        map->updateNode(octomap::point3d(c(x), c(y), c(z)), true);
    }
  map->updateInnerOccupancy();
  Planner planner;
  auto options = config();
  options.robot_radius = 0.01;
  options.max_slope = 0.57735;
  planner.setConfig(options);
  planner.setOctomap(map);
  const global_planner::PointPose bottom{c(-5), c(0), c(1)};
  const global_planner::PointPose top{c(5), c(0), c(2)};
  for (bool reverse : {false, true}) {
    const auto path = run(planner, reverse ? top : bottom, reverse ? bottom : top);
    require(!path.empty(), "5 cm floor change lost its adjacent connection");
    for (std::size_t i = 1; i < path.size(); ++i) {
      require(std::abs(path[i].x-path[i-1].x) <= 0.050001 &&
        std::abs(path[i].y-path[i-1].y) <= 0.050001 &&
        std::abs(path[i].z-path[i-1].z) <= 0.050001,
        "returned graph route contains a non-adjacent shortcut");
    }
  }
}

void checkBodyHeightSlopeLattice()
{
  const auto make_ramp = [](bool wall, bool gap) {
    auto map = std::make_shared<octomap::OcTree>(0.1);
    for (int x = -8; x <= 16; ++x) {
      if (gap && x >= 4 && x <= 7) continue;
      const int floor = x < 0 ? 0 : std::min(6, x / 2 + 1);
      // Only one observed column across Y: a route cannot evade the slope
      // bound by winding sideways or cross the dedicated unknown gap.
      for (int z = 0; z <= 16; ++z)
        map->updateNode(octomap::point3d(center(x), center(0), center(z)),
          z <= floor || (wall && x == 5));
    }
    map->updateInnerOccupancy();
    return map;
  };
  Planner planner;
  auto options = config();
  options.robot_radius = .01;
  options.body_clearance_below_m = options.body_clearance_above_m = .02;
  options.support_height_m = .35;
  options.support_height_tolerance_m = .05;
  options.max_slope = .57735;
  options.max_step_height = .15;
  planner.setConfig(options);
  planner.setOctomap(make_ramp(false, false));
  const global_planner::PointPose bottom{center(-6), center(0), .4};
  const global_planner::PointPose top{center(14), center(0), 1.0};
  for (bool reverse : {false, true}) {
    const auto path = run(planner, reverse ? top : bottom, reverse ? bottom : top);
    require(path.size() > 2U, "body-height lattice disconnected a supported shallow ramp");
    bool rises = false;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const double horizontal = std::hypot(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
      const double rise = std::abs(path[i].z-path[i-1].z);
      rises = rises || rise > 1e-6;
      require(rise <= .050001 && horizontal > 1e-6 &&
        rise <= options.max_slope * horizontal + 1e-6,
        "body-height route violates its configured metric slope");
    }
    require(rises, "ramp route omitted the requested height change");
  }
  options.max_slope = .20;
  planner.setConfig(options);
  const auto limited = run(planner, bottom, top);
  for (std::size_t i = 1; i < limited.size(); ++i) {
    const double horizontal = std::hypot(limited[i].x-limited[i-1].x, limited[i].y-limited[i-1].y);
    require(std::abs(limited[i].z-limited[i-1].z) <= options.max_slope * horizontal + 1e-6,
      "body-height route bypassed the tighter slope limit");
  }
  options.max_slope = .57735;
  options.max_step_height = .01;
  planner.setConfig(options);
  require(run(planner, bottom, top).empty(), "body-height route bypassed the step-height limit");
  options.max_step_height = .15;
  planner.setConfig(options);
  planner.setOctomap(make_ramp(true, false));
  require(run(planner, bottom, top).empty(), "body-height route crossed an occupied wall");
  planner.setOctomap(make_ramp(false, true));
  require(run(planner, bottom, top).empty(), "body-height route crossed unknown space");
}

void checkTerminalToleranceBoundary()
{
  auto flat = makeMap(false);
  const auto start = point(-4, 1);
  const auto snapped_goal = point(4, 1);
  // Occupy every nearer endpoint candidate. The remaining cell at x=0.45 m
  // is exactly 0.2 m from the requested x=0.65 m endpoint.
  for (int x = 5; x <= 8; ++x)
    for (int y = -2; y <= 2; ++y)
      for (int z = 1; z <= 3; ++z)
        flat->updateNode(octomap::point3d(center(x), center(y), center(z)), true);
  flat->updateInnerOccupancy();
  const auto request_goal = global_planner::PointPose{
    snapped_goal.x + 0.2, snapped_goal.y, snapped_goal.z};

  auto options = config();
  options.ground_support = false;
  options.snap_search_radius_cells = 2;
  options.terminal_goal_tolerance_m = 0.2;
  options.terminal_goal_xy_tolerance_m = 0.2;
  options.terminal_goal_z_tolerance_m = 0.001;

  Planner planner;
  planner.setConfig(options);
  planner.setOctomap(flat);
  require(!run(planner, start, request_goal).empty(),
    "exact terminal tolerance rejected by endpoint snapping");

  auto outside_goal = request_goal;
  outside_goal.x += 0.0001;
  require(run(planner, start, outside_goal).empty() &&
    planner.endpointResolution().failure ==
      Planner::EndpointResolutionInfo::Failure::GoalSnapExhausted,
    "endpoint snapping accepted a goal meaningfully outside terminal tolerance");

  const auto map_path = std::filesystem::temp_directory_path() /
    "lingtu_terminal_tolerance_boundary.bt";
  require(flat->writeBinary(map_path.string()),
    "failed to write terminal tolerance fixture");

  octoplanner3d::runtime::PlanRequest request;
  request.start = {start.x, start.y, start.z};
  request.goal = {request_goal.x, request_goal.y, request_goal.z};
  request.options.robot_radius = options.robot_radius;
  request.options.snap_search_radius_cells = options.snap_search_radius_cells;
  request.options.support_depth_cells = options.support_depth_cells;
  request.options.ground_support = options.ground_support;
  request.options.max_iterations = options.max_iterations;
  request.options.enable_preblocked_costmap = false;
  request.options.obstacle_clearance_radius_cells = 0;
  request.options.terminal_goal_tolerance_m = options.terminal_goal_tolerance_m;
  request.options.terminal_goal_xy_tolerance_m = options.terminal_goal_xy_tolerance_m;
  request.options.terminal_goal_z_tolerance_m = options.terminal_goal_z_tolerance_m;
  const auto boundary = octoplanner3d::runtime::runPlan(map_path, request);
  require(boundary.ok && boundary.reached_goal && !boundary.path.empty() &&
    boundary.goal_error_m > options.terminal_goal_tolerance_m &&
    boundary.goal_error_m <= options.terminal_goal_tolerance_m + 1e-6,
    "runtime rejected float-rounded endpoint at exact terminal tolerance");

  request.goal.x = outside_goal.x;
  const auto outside = octoplanner3d::runtime::runPlan(map_path, request);
  std::filesystem::remove(map_path);
  require(!outside.ok && !outside.reached_goal && outside.path.empty() &&
    outside.failure_reason == "goal_snap_exhausted",
    "runtime accepted a goal meaningfully outside terminal tolerance");
}

}  // namespace

int main()
{
  try {
    checkFullSizeStairs();
    checkQuantizedFloor();
    checkBodyHeightSlopeLattice();
    checkTerminalToleranceBoundary();
    Planner planner;
    const auto flat = makeMap(false);
    const auto step = makeMap(true);
    const auto start = point(-4, 1);
    const auto flat_goal = point(4, 1);
    const auto step_goal = point(4, 4);
    auto options = config();
    planner.setConfig(options);
    planner.setOctomap(flat);
    require(!run(planner, start, flat_goal).empty(), "flat route failed");
    auto info = planner.searchInfo();
    require(info.outcome == Outcome::Found && info.iterations > 0,
      "flat route did not use the adjacent graph");

    // A disconnected tall riser must not be bridged by invented long edges.
    planner.setOctomap(step);
    require(run(planner, start, step_goal).empty(), "adjacent graph jumped a tall riser");
    require(planner.searchInfo().outcome == Outcome::Exhausted,
      "disconnected riser did not exhaust its reachable graph");
    planner.setOctomap(makeMap(false, true));
    require(run(planner, start, flat_goal).empty(), "adjacent graph crossed a solid wall");

    planner.setOctomap(flat);
    require(!run(planner, start, flat_goal).empty(), "failure leaked into next request");
    require(run(planner, start, point(4, 6)).empty(), "unsupported goal produced a path");
    info = planner.searchInfo();
    require(planner.endpointResolution().failure ==
      Planner::EndpointResolutionInfo::Failure::GoalSnapExhausted &&
      info.outcome == Outcome::NotStarted && info.iterations == 0,
      "invalid endpoint started a search");

    options.max_iterations = 1;
    planner.setConfig(options);
    require(run(planner, start, flat_goal).empty(), "one-iteration budget produced a path");
    info = planner.searchInfo();
    require(info.outcome == Outcome::IterationLimit && info.iterations == 1,
      "search exceeded its iteration budget");

    planner.setConfig(config());
    planner.setCancelCheck([&]() { return planner.searchInfo().iterations >= 1; });
    require(run(planner, start, flat_goal).empty(), "cancelled search produced a path");
    require(planner.searchInfo().outcome == Outcome::Cancelled &&
      planner.searchInfo().iterations == 1, "search ignored cancellation");
    planner.setCancelCheck({});

    const auto map_path = std::filesystem::temp_directory_path() /
      "lingtu_neighborhood_smoke.bt";
    require(flat->writeBinary(map_path.string()), "failed to write runtime fixture");
    octoplanner3d::runtime::PlanRequest request;
    request.start = {start.x, start.y, start.z};
    request.goal = {flat_goal.x, flat_goal.y, flat_goal.z};
    request.options.robot_radius = 0.04;
    request.options.snap_search_radius_cells = 0;
    request.options.max_iterations = 1;
    const auto limited = octoplanner3d::runtime::runPlan(map_path, request);
    std::filesystem::remove(map_path);
    require(!limited.ok && !limited.cancelled && limited.path.empty() &&
      limited.failure_reason == "search_iteration_limit",
      "runtime reported search budget exhaustion as no path");

    std::cout << "26-neighbor search passed: flat, quantized floor, stairs, wall, reset, "
      "endpoint/tolerance, budget, cancellation, runtime reason\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
