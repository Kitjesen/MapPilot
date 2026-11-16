#include "global_planner.h"
#include "mls_backend.hpp"
#include "mls_backend.hpp"

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
    require(planner.metricConnectionWithinLimits(from, {-.43, 0, .375}),
            "body-height change over one floor was rejected");
    require(!planner.metricConnectionWithinLimits(from, {-.4, 0, .465}),
            "unsupported steep climb was accepted");
  }

  static void checkPreparedArtifact(OctoPlanner3D &planner) {
    require(planner.artifact() && planner.artifact()->rust_graph,
            "prepared Rust MLS graph is missing");
    const auto surfaces = planner.artifact()->rust_graph->surfaces();
    require(!surfaces.observed.empty() || !surfaces.inferred.empty(),
            "Rust MLS artifact has no prepared surface");
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

  planner.makePlan(start, goal);
  std::vector<PointPose> route;
  planner.getPlannerResults(route);
  require(!route.empty(), "open map did not produce a route");
  require(planner.validateRoute(route).valid,
          "Rust route failed the final OctoMap geometry check");

  planner.setOctomap(fixture(true));
  OctoPlannerGridQueryTest::checkPreparedArtifact(planner);
  planner.makePlan(start, goal);
  planner.getPlannerResults(route);
  require(route.empty(), "closed wall was crossed by the global route");

  std::cout << "Prepared Rust surface integration passed\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}
