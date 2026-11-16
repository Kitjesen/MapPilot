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
  static void checkTwinBody() {
    using Failure=OctoPlanner3D::TraversabilityFailure;
    constexpr double pi=3.14159265358979323846;
    PlannerConfig config;
    config.robot_radius=.32;
    config.body_offset_m=.23;
    config.body_clearance_below_m=.05;
    config.body_clearance_above_m=.35;
    config.ground_support=false;
    const auto map=[](double x,double y) {
      auto tree=std::make_shared<octomap::OcTree>(.005);
      tree->updateNode(octomap::point3d(-2,-2,-1),false);
      tree->updateNode(octomap::point3d(2,2,2),false);
      tree->updateNode(octomap::point3d(x,y,.5025),true);
      tree->updateInnerOccupancy();
      return tree;
    };
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(map(.5025,.0025));
    const octomap::point3d center(0,0,.5);
    require(planner.queryWorld(center,.32,false,true,0)==Failure::OccupiedBody,
            "twin body missed occupancy ahead of the central cylinder");
    require(planner.queryWorld(center,.32,false,true,pi/2)==Failure::None,
            "twin body ignored its yaw or substituted a larger circle");
    planner.setOctomap(map(.5125,.2075));
    for(double yaw:{0.0,pi/4,pi/2})
      require(planner.queryWorld(center,.32,false,true,yaw)==Failure::None,
              "turn fixture must clear endpoints and midpoint");
    Failure failure;
    require(!planner.turnFree({0,0,.5},0,pi/2,&failure) && failure==Failure::OccupiedBody,
            "turn missed a submillimetre arc intersection between sampled headings");
    require(planner.turnFree({0,0,1.0},0,pi/2),
            "turn merged occupied and clear height layers");
    planner.start_point_={-.8,0,.5,0.0};
    planner.has_start_=true;
    require(planner.sweptRouteFree({-.8,0,.5},center) &&
                planner.sweptRouteFree(center,{0,.8,.5}),
            "corner fixture must admit both straight segments");
    require(!planner.validateRoute({{-.8,0,.5},{0,0,.5},{0,.8,.5}}).valid,
            "route validation omitted the turn between individually clear segments");
    planner.setNoGoRegions(std::make_shared<const std::vector<lingtu::maps::NoGoRegion>>(
        std::vector<lingtu::maps::NoGoRegion>{{"turn",{.51,.205,.5},{.515,.21,.505}}}));
    require(!planner.turnFree({0,0,.5},0,pi/2,&failure) && failure==Failure::ExternalPreblockedBody,
            "turn did not apply the same no-go envelope");
  }

  static void checkWallCost() {
    PlannerConfig config;
    config.robot_radius=.32;
    config.ground_support=false;
    config.body_clearance_below_m=.05;
    config.body_clearance_above_m=.35;
    config.obstacle_clearance_radius_cells=6;
    auto tree=std::make_shared<octomap::OcTree>(.1);
    for(int x=-12;x<=12;++x)for(int y=-12;y<=12;++y)
      tree->updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,-.05),true);
    tree->updateNode(octomap::point3d(1.25,1.25,1.05),false);
    tree->updateInnerOccupancy();
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto cell=planner.worldToGrid(.05,.05,.435);
    require(planner.getObstacleClearanceCost(cell)==0,
            "support floor hid the wall's soft preference");
    tree->updateNode(octomap::point3d(.05,.45,.55),true);
    tree->updateInnerOccupancy();
    planner.setOctomap(tree);
    require(planner.getObstacleClearanceCost(cell)>0,
            "body-height wall did not contribute soft cost");
  }

  static void checkSweep() {
    using TraversabilityFailure = OctoPlanner3D::TraversabilityFailure;
    PlannerConfig config;
    config.robot_radius = .32;
    config.body_clearance_below_m = .05;
    config.body_clearance_above_m = .35;
    config.ground_support = false;
    const octomap::point3d from(5.723865762700855, 2.4249119484856907, 1.585);
    const octomap::point3d to(5.703264895298435, 2.5036701332266036, 1.585);
    for (bool coarse : {false, true}) {
      auto tree = std::make_shared<octomap::OcTree>(.1);
      tree->updateNode(octomap::point3d(5.05, 1.95, .55), false);
      tree->updateNode(octomap::point3d(6.05, 3.05, 2.55), false);
      for (int x = coarse ? 52 : 53; x <= 53; ++x)
        for (int y = coarse ? 22 : 23; y <= 23; ++y)
          for (int z = coarse ? 14 : 15; z <= 15; ++z)
            tree->updateNode(octomap::point3d((x+.5)*.1, (y+.5)*.1, (z+.5)*.1), true);
      tree->updateInnerOccupancy();
      tree->prune();
      if (coarse) {
        bool pruned = false;
        for (auto leaf = tree->begin_leafs(); leaf != tree->end_leafs(); ++leaf)
          pruned = pruned || (tree->isNodeOccupied(*leaf) && leaf.getSize() > .15);
        require(pruned, "sweep fixture lost its coarse occupied leaf");
      }
      OctoPlanner3D planner;
      planner.setConfig(config);
      planner.setOctomap(tree);
      for (const auto &point : {from, (from+to)*.5f, to})
        require(planner.queryWorld(point, .32, false) == TraversabilityFailure::None,
                "corner regression must clear both endpoints and midpoint");
      TraversabilityFailure failure;
      octomap::point3d failed_point;
      require(!planner.sweptRouteFree(from, to, &failure, &failed_point) &&
                  failure == TraversabilityFailure::OccupiedBody,
              "continuous sweep missed a submillimetre corner intersection");
      const octomap::point3d outward(.01, 0, 0), upward(0, 0, .5);
      require(planner.sweptRouteFree(from+outward, to+outward),
              "continuous sweep blocked the clear side of a corner");
      require(planner.sweptRouteFree(from+upward, to+upward),
              "continuous sweep merged separated height layers");
      auto touch_from=from, touch_to=to;
      touch_from.z()=touch_to.z()=1.65f;
      require(planner.sweptRouteFree(touch_from, touch_to),
              "continuous sweep rejected contact at the body bottom");
      touch_from.z()=touch_to.z()=coarse ? 1.05f : 1.15f;
      require(planner.sweptRouteFree(touch_from, touch_to),
              "continuous sweep rejected contact at the body top");
      planner.setStartExemption([](const PointPose &cell, double size) {
        return std::abs(size-.1)<1e-8 && std::abs(cell.x-5.35)<1e-8 &&
            std::abs(cell.y-2.35)<1e-8 && std::abs(cell.z-1.55)<1e-8;
      });
      require(planner.sweptRouteFree(from, to),
              "continuous sweep lost a fine-cell start exemption");
    }
  }

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
    if (std::hypot(a.x - b.x, a.y - b.y, a.z - b.z) >= 1e-6)
      std::cerr << "Terminal Rust=" << a.x << ',' << a.y << ',' << a.z
                << " grid=" << b.x << ',' << b.y << ',' << b.z << '\n';
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
  OctoPlannerGridQueryTest::checkTwinBody();
  OctoPlannerGridQueryTest::checkWallCost();
  OctoPlannerGridQueryTest::checkSweep();
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
  auto route = run(goal);
  require(!route.empty(), "open map did not produce a route");
  require(planner.searchInfo().used_rust_graph && !planner.searchInfo().rust_fallback,
          "open map did not use the checked Rust route");
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
  int cancel_checks = 0;
  planner.setCancelCheck([&] { return ++cancel_checks > 5; });
  require(run(goal).empty() &&
          planner.searchInfo().outcome == OctoPlanner3D::SearchInfo::Outcome::Cancelled,
          "cancelled Rust query retained an executable route");
  planner.setCancelCheck({});
  require(!run(goal).empty(), "cancelled query left the planner busy");
  OctoPlannerGridQueryTest::checkQueryReuse(planner, config);
  OctoPlannerGridQueryTest::checkGoalPolicy(planner, start, {.125, -.075, .325});
  OctoPlannerGridQueryTest::checkGoalPolicy(planner, start, {.975, .025, .41});
  require(run({goal.x, goal.y, 2.325}).empty(), "separate map floors were connected");

  planner.setCancelCheck([] { return true; });
  planner.makePlan(start, goal);
  planner.getPlannerResults(route);
  require(route.empty() && planner.searchInfo().outcome ==
              OctoPlanner3D::SearchInfo::Outcome::Cancelled,
          "cancelled Rust query returned a route or lost cancellation");
  planner.setCancelCheck({});

  planner.setOctomap(fixture(true));
  OctoPlannerGridQueryTest::checkPreparedArtifact(planner);
  require(run(goal).empty(), "closed wall was crossed by the global route");
  require(!planner.searchInfo().rust_fallback && planner.searchInfo().outcome ==
              OctoPlanner3D::SearchInfo::Outcome::Exhausted,
          "Rust exhaustion triggered a second planning backend");

  std::cout << "Prepared Rust surface integration passed: raw endpoints, terminal policy, "
               "separate floors, static wall and query cache\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n';
  return 1;
}
