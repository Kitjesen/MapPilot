#include "global_planner.h"
#include "mls_backend.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace global_planner;

namespace {

void require(bool value, const char *message) {
  if (!value) throw std::runtime_error(message);
}

std::shared_ptr<octomap::OcTree> fixture(bool closed_wall) {
  auto tree = std::make_shared<octomap::OcTree>(.05);
  for (int x = -24; x < 24; ++x) {
    for (int y = -14; y < 14; ++y) {
      for (int z = -1; z < 55; ++z) {
        const bool floor = z == -1 || z == 39;
        const bool wall = x == 0 && (closed_wall || y < 0);
        tree->updateNode((x + .5) * .05, (y + .5) * .05,
                         (z + .5) * .05, floor || wall, true);
      }
    }
  }
  tree->updateInnerOccupancy();
  return tree;
}

}  // namespace

namespace global_planner {

struct OctoPlannerGridQueryTest {
  static void checkBodyHeight(OctoPlanner3D &planner) {
    const PointPose from{-.5, 0, .325};
    const PointPose to{-.43, 0, .375};
    require(planner.metricConnectionWithinLimits(from, to) &&
                planner.routeConnectionFree(from, to),
            "body-height change over one floor was rejected");
    require(!planner.metricConnectionWithinLimits(from, {-.4, 0, .465}),
            "unsupported steep climb was accepted");
    require(!planner.metricConnectionWithinLimits(from, {.5, 0, .525}),
            "body-height exception bypassed the step limit");
    require(!planner.metricConnectionWithinLimits(from, {-.5, 0, .375}),
            "body-height exception accepted a vertical-only climb");
    auto raised = fixture(false);
    raised->updateNode(-.375, .025, .075, true);
    raised->updateInnerOccupancy();
    OctoPlanner3D steps;
    steps.setConfig(planner.graph_config_);
    steps.setOctomap(raised);
    require(!steps.metricConnectionWithinLimits({-.475, .025, .325},
                                               {-.375, .025, .425}),
            "body-height exception ignored a real change in observed floor height");
  }

  static void checkPreparedArtifact(OctoPlanner3D &planner) {
    require(planner.artifact() && planner.artifact()->rust_graph,
            "prepared Rust MLS graph is missing");
    const auto surfaces = planner.artifact()->rust_graph->surfaces();
    require(!surfaces.observed.empty() || !surfaces.inferred.empty(),
            "Rust MLS artifact has no prepared surface");
  }

  static void checkLatticeCache(OctoPlanner3D &planner, const PointPose &start) {
    planner.anchorPlanningLattice(start, .1);
    const auto cell = planner.worldToPlanningGrid(.225, -.075, .325);
    require(planner.isPlanningCellTraversableDetailed(
                cell, planner.robot_radius_, true, true, 2, nullptr),
            "lattice cache fixture must start with a free supported cell");
    planner.anchorPlanningLattice(start, .05);
    const bool expected = planner.queryWorld(planner.planningPoint(cell),
        planner.robot_radius_, true) == OctoPlanner3D::TraversabilityFailure::None;
    require(!expected, "lattice cache fixture must change the physical cell");
    require(planner.isPlanningCellTraversableDetailed(
                cell, planner.robot_radius_, true, true, 2, nullptr) == expected,
            "fine lattice reused a body result from a different physical position");
  }

  static void checkQueryReuse(OctoPlanner3D &planner, const PlannerConfig &config) {
    const auto prepared = planner.artifact_->rust_graph;
    auto query_config = config;
    query_config.max_iterations = 12000;
    query_config.snap_search_radius_cells = 10;
    query_config.terminal_goal_tolerance_m = .12;
    query_config.same_floor_preference = false;
    planner.setConfig(query_config);
    require(planner.artifact_->rust_graph == prepared,
            "query-only changes rebuilt the prepared Rust artifact");
  }

  static void checkGoalPolicy(OctoPlanner3D &planner, const PointPose &start,
                              const PointPose &goal) {
    planner.makePlan(start, goal);
    std::vector<PointPose> rust_path;
    planner.getPlannerResults(rust_path);
    require(!rust_path.empty() &&
                (planner.searchInfo().used_rust_graph || planner.searchInfo().rust_fallback),
            "Rust preparation rejected an in-tolerance terminal");
    auto prepared = std::move(planner.artifact_->rust_graph);
    planner.makePlan(start, goal);
    std::vector<PointPose> grid_path;
    planner.getPlannerResults(grid_path);
    planner.artifact_->rust_graph = std::move(prepared);
    require(!grid_path.empty(), "resolved terminal could not attach to grid search");
    const auto &a = rust_path.back(), &b = grid_path.back();
    require(std::hypot(a.x - b.x, a.y - b.y, a.z - b.z) < 1e-6,
            "Rust and grid search selected different terminal positions");
    require(planner.queryWorld({float(b.x), float(b.y), float(b.z)},
                planner.robot_radius_, true) == OctoPlanner3D::TraversabilityFailure::None,
            "resolved terminal violates support or body geometry");
    require(std::hypot(b.x - goal.x, b.y - goal.y, b.z - goal.z) <=
                planner.terminal_goal_tolerance_m_ + 1e-6 &&
            std::hypot(b.x - goal.x, b.y - goal.y) <=
                planner.terminal_goal_xy_tolerance_m_ + 1e-6 &&
            std::abs(b.z - goal.z) <= planner.terminal_goal_z_tolerance_m_ + 1e-6,
            "resolved terminal exceeds a configured goal tolerance");
  }
};

}  // namespace global_planner

int main() try {
  PlannerConfig config;
  config.robot_radius = .1;
  config.body_clearance_below_m = .1;
  config.body_clearance_above_m = .2;
  config.support_height_m = .35;
  config.support_height_tolerance_m = .05;
  config.ground_support = true;
  config.direct_support_only = true;
  config.search_resolution_m = .1;
  config.snap_search_radius_cells = 12;
  config.max_step_height = .15;
  config.max_slope = .6;
  config.max_iterations = 10000;
  config.terminal_goal_tolerance_m = .1;
  config.terminal_goal_xy_tolerance_m = .1;
  config.terminal_goal_z_tolerance_m = .05;
  config.enable_preblocked_costmap = false;

  const PointPose start{-.975, .025, .325};
  const PointPose goal{.975, .025, .325};
  OctoPlanner3D planner;
  planner.setConfig(config);
  planner.setOctomap(fixture(false));
  OctoPlannerGridQueryTest::checkPreparedArtifact(planner);
  OctoPlannerGridQueryTest::checkBodyHeight(planner);
  OctoPlannerGridQueryTest::checkLatticeCache(planner, start);

  const auto run = [&](const PointPose &target) {
    planner.makePlan(start, target);
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    return path;
  };
  const auto route = run(goal);
  require(!route.empty(), "open map did not produce a route");
  require(planner.searchInfo().used_rust_graph || planner.searchInfo().rust_fallback,
          "body-centre request bypassed the prepared Rust artifact");
  require(std::hypot(route.front().x - start.x, route.front().y - start.y,
                     route.front().z - start.z) < 1e-6,
          "prepared route moved the actual start");
  require(std::hypot(route.back().x - goal.x, route.back().y - goal.y,
                     route.back().z - goal.z) < 1e-6,
          "valid raw goal was lost");
  for (const auto &point : route)
    require(std::abs(point.z - start.z) < .051, "route switched floors");
  require(planner.validateRoute(route).valid,
          "Rust route failed the final OctoMap geometry check");
  OctoPlannerGridQueryTest::checkQueryReuse(planner, config);
  OctoPlannerGridQueryTest::checkGoalPolicy(planner, start, {.125, -.075, .325});
  OctoPlannerGridQueryTest::checkGoalPolicy(planner, start, {.975, .025, .41});
  require(run({goal.x, goal.y, 2.325}).empty(), "separate map floors were connected");

  planner.setOctomap(fixture(true));
  OctoPlannerGridQueryTest::checkPreparedArtifact(planner);
  require(run(goal).empty(), "closed wall was crossed by the global route");

  std::cout << "Prepared Rust surface integration passed: raw endpoints, terminal policy, "
               "separate floors, static wall and query cache\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}
