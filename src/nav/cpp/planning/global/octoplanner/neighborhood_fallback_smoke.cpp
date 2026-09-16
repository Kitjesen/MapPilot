#include "global_planner.h"
#include "octoplanner3d_core.hpp"

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
  result.ground_support_depth_cells = 2;
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

}  // namespace

int main()
{
  try {
    checkFullSizeStairs();
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
    require(info.outcome == Outcome::Found && !info.used_stair_connections &&
      info.basic_iterations > 0 && info.extended_iterations == 0,
      "flat route did not stay in the basic neighborhood");

    planner.setOctomap(step);
    require(!run(planner, start, step_goal).empty(), "stair ascent fallback failed");
    info = planner.searchInfo();
    require(info.outcome == Outcome::Found && info.used_stair_connections &&
      info.basic_iterations > 0 && info.extended_iterations > 0,
      "stair fixture did not require fallback after basic exhaustion");
    const int exhausted_basic_iterations = info.basic_iterations;
    require(!run(planner, step_goal, start).empty(), "stair descent fallback failed");
    require(planner.searchInfo().used_stair_connections, "stair descent skipped fallback");

    options.max_iterations = 2 * exhausted_basic_iterations;
    planner.setConfig(options);
    require(!run(planner, start, step_goal).empty(), "reserved fallback budget failed to reach stair goal");
    info = planner.searchInfo();
    require(info.used_stair_connections &&
      info.basic_iterations == options.max_iterations / 4 &&
      info.basic_iterations + info.extended_iterations <= options.max_iterations,
      "basic graph consumed the work reserved for fallback");
    options = config();
    planner.setConfig(options);

    planner.setOctomap(makeMap(true, true));
    require(run(planner, start, step_goal).empty(), "fallback crossed a solid wall");
    info = planner.searchInfo();
    require(info.outcome == Outcome::Exhausted && info.used_stair_connections &&
      info.extended_iterations > 0, "unreachable goal did not exhaust both stages");

    // The same planner must reset the previous failure and fallback state.
    planner.setOctomap(flat);
    require(!run(planner, start, flat_goal).empty() &&
      !planner.searchInfo().used_stair_connections, "fallback leaked into the next request");
    require(run(planner, start, point(4, 6)).empty(), "unsupported goal produced a path");
    info = planner.searchInfo();
    require(planner.endpointResolution().failure ==
      Planner::EndpointResolutionInfo::Failure::GoalSnapExhausted &&
      info.outcome == Outcome::NotStarted && !info.used_stair_connections &&
      info.basic_iterations == 0 && info.extended_iterations == 0,
      "invalid endpoint started the fallback search");

    options.max_iterations = 1;
    planner.setConfig(options);
    require(run(planner, start, flat_goal).empty(), "one-iteration budget produced a path");
    info = planner.searchInfo();
    require(info.outcome == Outcome::IterationLimit && !info.used_stair_connections &&
      info.basic_iterations == 1 && info.extended_iterations == 0,
      "basic iteration limit incorrectly triggered fallback");

    planner.setOctomap(step);
    options.max_iterations = 4;
    planner.setConfig(options);
    require(run(planner, start, step_goal).empty(), "shared budget was exceeded");
    info = planner.searchInfo();
    require(info.outcome == Outcome::IterationLimit && info.used_stair_connections &&
      info.basic_iterations == 1 && info.extended_iterations == 3,
      "fallback reset the total iteration budget");

    planner.setConfig(config());
    planner.setCancelCheck([&]() {
      return planner.searchInfo().basic_iterations >= exhausted_basic_iterations;
    });
    require(run(planner, start, step_goal).empty(), "cancelled basic search produced a path");
    info = planner.searchInfo();
    require(info.outcome == Outcome::Cancelled && !info.used_stair_connections &&
      info.extended_iterations == 0, "cancellation at basic exhaustion triggered fallback");
    planner.setCancelCheck([&]() {
      return planner.searchInfo().extended_iterations >= 1;
    });
    require(run(planner, start, step_goal).empty(), "cancelled fallback produced a path");
    info = planner.searchInfo();
    require(info.outcome == Outcome::Cancelled && info.used_stair_connections &&
      info.extended_iterations == 1, "fallback did not honor cancellation");
    planner.setCancelCheck({});

    const auto map_path = std::filesystem::temp_directory_path() /
      "lingtu_neighborhood_fallback_smoke.bt";
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

    std::cout << "neighborhood fallback passed: flat, stairs up/down, wall, reset, "
      "endpoint, basic/shared budget, basic/fallback cancellation, runtime reason\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
