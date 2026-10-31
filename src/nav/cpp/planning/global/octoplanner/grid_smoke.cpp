#include "global_planner.h"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace global_planner {

struct OctoPlannerGridQueryTest
{
  static void neighboringSupportInterpolatesHolesButNotFloorEdges() {
    const auto supported = [](const std::vector<octomap::point3d> &floor,
                              PointPose pose = {.05,.05,.45}) {
      auto tree = std::make_shared<octomap::OcTree>(.1);
      tree->updateNode(octomap::point3d(-1,-1,-1),false);
      tree->updateNode(octomap::point3d(1,1,1),false);
      for (const auto &point : floor) tree->updateNode(point,true);
      PlannerConfig config;
      config.support_height_m=.4;
      config.support_height_tolerance_m=.05;
      config.direct_support_only=false;
      config.support_radius_cells=1;
      OctoPlanner3D planner;
      planner.setConfig(config);
      planner.setOctomap(tree);
      return planner.poseSupportFree(pose);
    };
    if (supported({{-.05,.05,.05},{-.05,-.05,.05},{-.05,.15,.05}}))
      throw std::runtime_error("one-sided floor evidence expanded past its boundary");
    if (supported({{-.05,.05,.05},{.05,-.05,.05}}))
      throw std::runtime_error("corner support extrapolated beyond its convex boundary");
    if (supported({{-.05,.15,.05},{.15,-.05,.05}}))
      throw std::runtime_error("collinear floor edges were mistaken for an interior sampling hole");
    if (!supported({{-.05,.05,.05},{.15,-.05,.05},{.15,.15,.05}}))
      throw std::runtime_error("surrounding floor observations did not repair a sampling hole");
    if (!supported({{.05,.05,.05}}))
      throw std::runtime_error("direct observed surface unnecessarily required surrounding evidence");
    if (supported({{-.05,.05,-.55},{.15,.05,-.55}}))
      throw std::runtime_error("another floor was borrowed outside the configured height band");
    const std::vector<octomap::point3d> edge{{-.05,-.05,.05},
      {-.05,.15,.05},{.15,.05,.05}};
    if (supported(edge,{.08,.09,.45}))
      throw std::runtime_error("actual off-centre pose inherited the voxel centre's support");
    if (!supported(edge,{.08,.07,.45}))
      throw std::runtime_error("actual off-centre pose inside the observed surface was rejected");
  }

  static void supportedRouteTurnsInsideAnLShapedFloor() {
    auto tree=std::make_shared<octomap::OcTree>(.1);
    tree->updateNode(octomap::point3d(-1.2,-1.2,-1),false);
    tree->updateNode(octomap::point3d(1.2,1.2,1),false);
    for(int x=-10;x<=10;++x) for(int y=-10;y<=10;++y)
      if(y<=0 || x<=-3)
        tree->updateNode(octomap::point3d((x+.5)*.1,(y+.5)*.1,.05),true);
    PlannerConfig config;
    config.robot_radius=.01;
    config.body_clearance_below_m=config.body_clearance_above_m=.05;
    config.support_height_m=.4;
    config.support_height_tolerance_m=.05;
    config.direct_support_only=false;
    config.support_radius_cells=1;
    config.enable_preblocked_costmap=false;
    config.obstacle_clearance_radius_cells=0;
    config.snap_search_radius_cells=0;
    OctoPlanner3D planner;
    planner.setConfig(config);planner.setOctomap(tree);
    planner.makePlan({.45,-.25,.45},{-.55,.65,.45});
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if(path.empty()) throw std::runtime_error("L-shaped supported floor lost its valid connection");
    for(const auto &point:path) {
      const auto *surface=tree->search(point.x,point.y,.05);
      if(!surface || !tree->isNodeOccupied(surface))
        throw std::runtime_error("route cut across the unsupported corner of an L-shaped floor: " +
            std::to_string(point.x) + "," + std::to_string(point.y) + "," +
            std::to_string(point.z) + " support=" + std::to_string(planner.poseSupportFree(point)));
    }
  }

  static void supportHeightUsesActualPoseWithinVoxel() {
    auto tree = std::make_shared<octomap::OcTree>(.1);
    tree->updateNode(octomap::point3d(-.5,-.5,-.5), false);
    tree->updateNode(octomap::point3d(.5,.5,.8), false);
    tree->updateNode(octomap::point3d(.05,.05,.05), true);
    PlannerConfig config;
    config.support_height_m = .435;
    config.support_height_tolerance_m = .1;
    config.direct_support_only = true;
    config.support_radius_cells = 0;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    if (!planner.poseSupportFree({.05,.05,.38861}))
      throw std::runtime_error("actual pose-to-floor height within tolerance was quantized away");
    // The surface is only known to be inside the occupied voxel [0, .1].
    if (!planner.poseSupportFree({.05,.05,.61}) ||
        planner.poseSupportFree({.05,.05,.30}) || planner.poseSupportFree({.05,.05,.66}))
      throw std::runtime_error("out-of-tolerance floor was accepted as support");
    auto drop = std::make_shared<octomap::OcTree>(.1);
    drop->updateNode(octomap::point3d(-.5,-.5,-.5), false);
    drop->updateNode(octomap::point3d(.5,.5,.8), false);
    drop->updateNode(octomap::point3d(.05,.05,.05), false);
    planner.setOctomap(drop);
    if (planner.poseSupportFree({.05,.05,.38861}))
      throw std::runtime_error("free ground voxel was accepted as support");
  }

  static void supportHeightUsesBothFacesOfTheOccupiedVoxel() {
    auto tree = std::make_shared<octomap::OcTree>(.05);
    tree->updateNode(octomap::point3d(-1,-1,-1),false);
    tree->updateNode(octomap::point3d(1,1,1),false);
    tree->updateNode(octomap::point3d(.025,.025,-.375),true);
    PlannerConfig config;
    config.support_height_m=.35;
    config.support_height_tolerance_m=.05;
    config.direct_support_only=true;
    config.support_radius_cells=0;
    OctoPlanner3D planner;
    planner.setConfig(config); planner.setOctomap(tree);
    if (!planner.poseSupportFree({.025,.025,-.0801478}))
      throw std::runtime_error("support cell overlaps the stance band but its centre rejected the real-map start");
    if (planner.poseSupportFree({.025,.025,-.12}) || planner.poseSupportFree({.025,.025,.10}))
      throw std::runtime_error("support voxel wholly outside the stance band was accepted");
  }

  static void supportedRoutesDoNotRequireObservedCentreAir() {
    const auto make_map = [](bool missing_floor, bool observed_drop, bool obstacle) {
      auto tree = std::make_shared<octomap::OcTree>(.05);
      tree->updateNode(octomap::point3d(-1, -1, -1), false);
      tree->updateNode(octomap::point3d(1, 1, 1), false);
      for (int x = -10; x <= 10; ++x) for (int y = -2; y <= 2; ++y) {
        if (!missing_floor)
          tree->updateNode(octomap::point3d((x+.5)*.05, (y+.5)*.05, -.325), true);
        if (observed_drop)
          tree->updateNode(octomap::point3d((x+.5)*.05, (y+.5)*.05, -.325), false);
      }
      if (obstacle) tree->updateNode(octomap::point3d(.025,.025,.025), true);
      return tree;
    };
    PlannerConfig config;
    config.robot_radius = .01;
    config.body_clearance_below_m = config.body_clearance_above_m = .05;
    config.support_height_m = .35;
    config.support_height_tolerance_m = .05;
    config.direct_support_only = true;
    config.support_radius_cells = 0;
    config.snap_search_radius_cells = 0;
    config.enable_preblocked_costmap = false;
    config.obstacle_clearance_radius_cells = 0;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(make_map(false, false, false));
    const PointPose start{-.225,.025,.025}, goal{.225,.025,.025};
    const octomap::point3d centre(.025,.025,.025);
    if (!planner.poseSupportFree({centre.x(),centre.y(),centre.z()}) ||
        planner.queryWorld(centre,.01,true) != OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("known floor rejected because body-centre air is unobserved");
    if (!planner.isCellTraversableDetailed(planner.worldToGrid(centre.x(),centre.y(),centre.z()),
                                          .01,true,true,0,2,nullptr))
      throw std::runtime_error("supported XYZ vertex still requires observed centre air");
    planner.makePlan(start,goal);
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if (path.empty())
      throw std::runtime_error("observed floor with unobserved centre air failed to route");
    // The older depth-based support mode must follow the same air policy.
    config.support_height_m = 0;
    config.support_depth_cells = 7;
    planner.setConfig(config);
    if (!planner.isCellTraversableDetailed(planner.worldToGrid(centre.x(),centre.y(),centre.z()),
                                          .01,true,true,0,7,nullptr))
      throw std::runtime_error("legacy supported vertex still requires observed centre air");
    config.support_height_m = .35;
    config.support_depth_cells = 2;
    planner.setConfig(config);
    for (const bool observed_drop : {false,true}) {
      planner.setOctomap(make_map(true,observed_drop,false));
      if (planner.poseSupportFree({centre.x(),centre.y(),centre.z()}) ||
          planner.queryWorld(centre,.01,true) != OctoPlanner3D::TraversabilityFailure::GroundSupport)
        throw std::runtime_error("unknown floor or observed drop became support");
      planner.makePlan(start,goal);
      planner.getPlannerResults(path);
      if (!path.empty()) throw std::runtime_error("route crossed unknown floor or observed drop");
    }
    planner.setOctomap(make_map(false,false,true));
    if (planner.queryWorld(centre,.01,true) != OctoPlanner3D::TraversabilityFailure::OccupiedBody)
      throw std::runtime_error("known body obstacle was cleared with unknown air");
  }

  static void startSnapUsesRouteHeightPreference() {
    auto tree = std::make_shared<octomap::OcTree>(.1);
    tree->updateNode(octomap::point3d(-.5,-.5,-.5), false);
    tree->updateNode(octomap::point3d(1,1,1), false);
    tree->updateNode(octomap::point3d(.05,.05,.05), true);
    tree->updateNode(octomap::point3d(.05,.05,-.05), false);
    tree->updateNode(octomap::point3d(.25,.05,.05), false);
    PlannerConfig config;
    config.robot_radius = .001;
    config.body_clearance_below_m = config.body_clearance_above_m = .01;
    config.support_height_m = .35;
    config.ground_support = false;
    config.floor_change_penalty = 6;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto seed = planner.worldToGrid(.05,.05,.05);
    GridIndex snapped;
    const auto snap = [&] {
      return planner.findNearestFreeCell(seed,.001,3,false,false,0,2,snapped);
    };
    if (!snap() || !(snapped == planner.worldToGrid(.25,.05,.05)))
      throw std::runtime_error("start snap ignored the route height-change preference");
    config.same_floor_preference = false;
    planner.setConfig(config);
    if (!snap() || !(snapped == planner.worldToGrid(.05,.05,-.05)))
      throw std::runtime_error("disabled height preference changed nearest start snapping");
    config.same_floor_preference = true;
    planner.setConfig(config);
    tree->updateNode(octomap::point3d(.25,.05,.05), true);
    planner.setOctomap(tree);
    if (!snap() || !(snapped == planner.worldToGrid(.05,.05,-.05)))
      throw std::runtime_error("height preference prevented necessary vertical snapping");
  }

  static void goalSnapUsesRequestedPosition() {
    auto tree = std::make_shared<octomap::OcTree>(.1);
    tree->updateNode(octomap::point3d(-.05, .05, .05), false);
    tree->updateNode(octomap::point3d(.05, .05, .05), true);
    tree->updateNode(octomap::point3d(.15, .05, .05), false);
    tree->updateNode(octomap::point3d(.25, .05, .05), false);
    PlannerConfig config;
    config.robot_radius = .001;
    config.ground_support = false;
    config.snap_search_radius_cells = 2;
    config.enable_preblocked_costmap = false;
    config.obstacle_clearance_radius_cells = 0;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    planner.makePlan({.25, .05, .05}, {.09, .05, .05});
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if (path.empty() || std::abs(path.back().x - .15) > 1e-6)
      throw std::runtime_error("goal snap chose the nearest cell centre instead of the requested position");
    config.terminal_goal_tolerance_m = .05;
    planner.setConfig(config);
    planner.makePlan({.25, .05, .05}, {.09, .05, .05});
    planner.getPlannerResults(path);
    if (!path.empty() || planner.searchInfo().iterations != 0)
      throw std::runtime_error("out-of-tolerance snapped goal still ran A*");
    config.terminal_goal_tolerance_m = .07;
    planner.setConfig(config);
    planner.makePlan({.25, .05, .05}, {.09, .05, .05});
    planner.getPlannerResults(path);
    if (path.empty()) throw std::runtime_error("in-tolerance goal stopped planning");
  }

  static void exactGoalConnectionDoesNotSpendArrivalToleranceOnGridQuantization() {
    auto tree=std::make_shared<octomap::OcTree>(.2);
    for(int x=-6;x<=6;++x) for(int y=-6;y<=6;++y) {
      tree->updateNode(octomap::point3d((x+.5)*.2,(y+.5)*.2,.1),true);
      for(int z=1;z<=6;++z)
        tree->updateNode(octomap::point3d((x+.5)*.2,(y+.5)*.2,(z+.5)*.2),false);
    }
    PlannerConfig cfg;
    cfg.robot_radius=.02;
    cfg.body_clearance_below_m=cfg.body_clearance_above_m=.01;
    cfg.support_height_m=.45; cfg.support_height_tolerance_m=.11;
    cfg.direct_support_only=true; cfg.support_radius_cells=0;
    cfg.lowest_traversable_only=true;
    cfg.enable_preblocked_costmap=false; cfg.obstacle_clearance_radius_cells=0;
    cfg.terminal_goal_tolerance_m=.15;
    cfg.terminal_goal_xy_tolerance_m=.15;
    cfg.terminal_goal_z_tolerance_m=.11;
    OctoPlanner3D planner;
    planner.setConfig(cfg); planner.setOctomap(tree);
    const PointPose start{-.5,-.5,.5}, goal{0,0,.435};
    if(planner.queryWorld({0,0,.435},.02,true)!=OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("exact-goal fixture must be a supported, collision-free pose");
    planner.makePlan(start,goal);
    std::vector<PointPose> path; planner.getPlannerResults(path);
    if(path.empty() || std::hypot(path.back().x-goal.x,path.back().y-goal.y,path.back().z-goal.z)>1e-6)
      throw std::runtime_error("coarse grid spent the arrival tolerance before connecting the exact goal");
    const auto &node=path[path.size()-2];
    if(std::hypot(node.x-goal.x,node.y-goal.y,node.z-goal.z)<=cfg.terminal_goal_tolerance_m)
      throw std::runtime_error("exact-goal fixture did not exercise an out-of-tolerance grid centre");

    // Both endpoint bodies clear this observed slab; the connecting sweep does not.
    planner.setObservedOccupiedCheck([](const auto &p,double radius,double below,double above,bool) {
      return std::hypot(p.x,p.y)<=.2+radius && p.z+above>.46 && p.z-below<.475;
    });
    planner.makePlan(start,goal); planner.getPlannerResults(path);
    if(!path.empty()) throw std::runtime_error("exact goal connector crossed an observed obstacle");
    planner.setObservedOccupiedCheck([](const auto &p,double radius,double below,double above,bool) {
      return std::hypot(p.x-.05,p.y-.05)<=.02+radius && p.z+above>.41 && p.z-below<.47;
    });
    planner.makePlan(start,{.05,.05,.435}); planner.getPlannerResults(path);
    if(path.empty() || std::hypot(path.back().x-.05,path.back().y-.05,path.back().z-.435)>.15 ||
        std::hypot(path.back().x-.05,path.back().y-.05,path.back().z-.435)<1e-6)
      throw std::runtime_error("invalid exact goal lost the original in-tolerance snapped fallback");
    planner.setObservedOccupiedCheck({});

    planner.makePlan(start,{0,0,.3}); planner.getPlannerResults(path);
    if(!path.empty()) throw std::runtime_error("exact goal connector bypassed unsupported goal height");
  }

  static void temporaryObstacleDoesNotCrossObservedFloor() {
    auto tree = std::make_shared<octomap::OcTree>(.05);
    for (int x = -8; x <= 8; ++x) for (int y = -8; y <= 8; ++y)
      for (int z = -7; z <= 26; ++z)
        tree->updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,(z+.5)*.05),
                         z == -7 || z == 13);
    PlannerConfig config;
    config.robot_radius = .155;
    config.support_height_m = .35;
    config.support_height_tolerance_m = .05;
    config.body_clearance_below_m = config.body_clearance_above_m = .10;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    ExternalBlockedRegion blocked;
    blocked.center = {.025,.025,.025};
    blocked.radius_xy_m = .1;
    blocked.min_z = 0.0;
    blocked.max_z = .05;
    planner.setExternalPreblockedRegions({blocked});
    if (planner.queryWorld({.025,.025,.025},.155,true) ==
          OctoPlanner3D::TraversabilityFailure::None ||
        planner.queryWorld({.025,.025,1.025},.155,true) !=
          OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("temporary obstacle missed its level or crossed an observed floor");
  }
  static void stepTransitionsKeepObservedSupportAndBodyClearance() {
    const auto map = [](bool gap = false, bool unknown_gap = false, bool wall = false,
                        int rise_cells = 4) {
      auto tree = std::make_shared<octomap::OcTree>(.05);
      for (int x = -16; x <= 16; ++x) for (int y = -8; y <= 8; ++y) {
        const bool missing = x >= -1 && x <= 1;
        if (unknown_gap && missing) continue;
        const int floor = gap && missing ? -15 : (x < 0 ? -7 : -7 + rise_cells);
        tree->updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,(floor+.5)*.05), true);
        for (int z = floor+1; z <= 12; ++z)
          tree->updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,(z+.5)*.05), false);
        if (wall && x == 0) for (int z = -2; z <= 9; ++z)
          tree->updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,(z+.5)*.05), true);
      }
      return tree;
    };
    PlannerConfig config;
    config.robot_radius = .155;
    config.support_height_m = .35;
    config.support_height_tolerance_m = .05;
    config.body_clearance_below_m = config.body_clearance_above_m = .10;
    config.direct_support_only = false;
    config.support_radius_cells = 1;
    config.max_step_height = .25;
    config.max_slope = 0.0;
    config.snap_search_radius_cells = 0;
    config.max_iterations = 5000;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(map());
    const octomap::point3d lower(-.225,.025,.025), upper(.225,.025,.225);
    const auto a = planner.worldToGrid(lower.x(),lower.y(),lower.z());
    const auto b = planner.worldToGrid(upper.x(),upper.y(),upper.z());
    if (planner.queryWorld(lower,.155,true) != OctoPlanner3D::TraversabilityFailure::None ||
        planner.queryWorld(upper,.155,true) != OctoPlanner3D::TraversabilityFailure::None ||
        !planner.isMotionAllowed(a,b) || !planner.isMotionAllowed(b,a))
      throw std::runtime_error("observed 20 cm step failed its supported transition");
    for (bool reverse : {false,true}) {
      const auto start = reverse ? upper : lower, goal = reverse ? lower : upper;
      planner.makePlan({start.x(),start.y(),start.z()}, {goal.x(),goal.y(),goal.z()});
      std::vector<PointPose> path;
      planner.getPlannerResults(path);
      // The prepared surface graph connects this observed, permitted riser;
      // the old body-height lattice could not represent that connection.
      if (path.empty() || !planner.searchInfo().used_surface_graph)
        throw std::runtime_error("prepared surface graph missed a permitted 20 cm step");
      for (std::size_t i=1;i<path.size();++i)
        if (!planner.metricConnectionWithinLimits(path[i-1],path[i]) ||
            !planner.sweptRouteFree(octomap::point3d(path[i-1].x,path[i-1].y,path[i-1].z),
                                   octomap::point3d(path[i].x,path[i].y,path[i].z)))
          throw std::runtime_error("prepared step route violated its three-dimensional query");
    }
    planner.setOctomap(map(false, false, false, 1));
    config.max_slope = 0.57735;
    planner.setConfig(config);
    const octomap::point3d small_upper(.225,.025,.075);
    for (bool reverse : {false,true}) {
      const auto start = reverse ? small_upper : lower, goal = reverse ? lower : small_upper;
      planner.makePlan({start.x(),start.y(),start.z()}, {goal.x(),goal.y(),goal.z()});
      std::vector<PointPose> path;
      planner.getPlannerResults(path);
      if (path.empty()) throw std::runtime_error("5 cm floor change failed at Go2 body height");
    }
    config.max_slope = 0.0;
    planner.setConfig(config);
    for (const auto &blocked : {map(true), map(false,true), map(false,false,true)}) {
      planner.setOctomap(blocked);
      if (planner.isMotionAllowed(a,b) || planner.isMotionAllowed(b,a))
        throw std::runtime_error("step transition crossed a drop, unknown gap or body obstacle");
      planner.makePlan({lower.x(),lower.y(),lower.z()}, {upper.x(),upper.y(),upper.z()});
      std::vector<PointPose> blocked_path;
      planner.getPlannerResults(blocked_path);
      if (!blocked_path.empty()) throw std::runtime_error("surface graph crossed a drop, unknown gap or wall");
    }
    planner.setOctomap(map());
    config.max_step_height = .15;
    planner.setConfig(config);
    if (planner.isMotionAllowed(a,b)) throw std::runtime_error("step limit was bypassed");
    config.max_step_height = .25;
    config.max_slope = .30;
    planner.setConfig(config);
    const auto slope_a = planner.worldToPlanningGrid(lower.x(), lower.y(), lower.z());
    const auto slope_b = planner.worldToPlanningGrid(upper.x(), upper.y(), upper.z());
    if (planner.isMotionAllowed(slope_a,slope_b)) throw std::runtime_error("slope limit was bypassed");
  }
  static void routeClearancePassesNarrowCorridor() {
    auto tree = std::make_shared<octomap::OcTree>(.05);
    // Observed 40 cm corridor: wider than Go2, narrower than SCAN's envelope.
    for (int x = -8; x <= 20; ++x) {
      for (int y = -4; y <= 3; ++y) {
        tree->updateNode(octomap::point3d((x+.5)*.05, (y+.5)*.05, -.325), true);
        for (int z = -6; z <= 3; ++z)
          tree->updateNode(octomap::point3d((x+.5)*.05, (y+.5)*.05, (z+.5)*.05), false);
      }
      for (int z = -6; z <= 3; ++z) {
        tree->updateNode(octomap::point3d((x+.5)*.05, -.225, (z+.5)*.05), true);
        tree->updateNode(octomap::point3d((x+.5)*.05, .225, (z+.5)*.05), true);
      }
    }
    PlannerConfig config;
    config.robot_radius = .155;
    config.support_height_m = .35;
    config.support_height_tolerance_m = .05;
    config.body_clearance_below_m = config.body_clearance_above_m = .10;
    config.snap_search_radius_cells = 0;
    config.max_iterations = 1000;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const octomap::point3d start(.025,.025,.025);
    if (planner.queryWorld(start, .25, false) !=
        OctoPlanner3D::TraversabilityFailure::OccupiedBody)
      throw std::runtime_error("corridor does not distinguish route from execution envelope");
    planner.makePlan({.025,.025,.025}, {.625,.025,.025});
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if (path.empty())
      throw std::runtime_error("global route blocked by local execution clearance");
    // A full-height barrier must still block search and path simplification.
    for (int y = -4; y <= 3; ++y) for (int z = -6; z <= 3; ++z)
      tree->updateNode(octomap::point3d(.325, (y+.5)*.05, (z+.5)*.05), true);
    planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
    planner.makePlan({.025,.025,.025}, {.625,.025,.025});
    planner.getPlannerResults(path);
    if (!path.empty())
      throw std::runtime_error("global route ignored wall across corridor");
  }
  static void formalWidthGuidanceDoesNotRequireAnAllYawCircle() {
    const auto corridor = [](int half_width_cells) {
      auto tree = std::make_shared<octomap::OcTree>(.05);
      for (int x = -12; x <= 24; ++x) {
        for (int y = -12; y <= 12; ++y) {
          tree->updateNode(octomap::point3d((x+.5)*.05, (y+.5)*.05, -.325), true);
          for (int z = -6; z <= 7; ++z)
            tree->updateNode(octomap::point3d((x+.5)*.05, (y+.5)*.05, (z+.5)*.05), false);
        }
        for (int z = -6; z <= 7; ++z) {
          tree->updateNode(octomap::point3d((x+.5)*.05,
              (-half_width_cells-.5)*.05, (z+.5)*.05), true);
          tree->updateNode(octomap::point3d((x+.5)*.05,
              (half_width_cells+.5)*.05, (z+.5)*.05), true);
        }
      }
      return tree;
    };
    PlannerConfig config;
    config.robot_radius = .25;
    config.body_clearance_below_m = .10;
    config.body_clearance_above_m = .25;
    config.support_height_m = .35;
    config.support_height_tolerance_m = .05;
    config.max_step_height = .45;
    config.max_slope = .57735;
    config.snap_search_radius_cells = 0;
    config.max_iterations = 1000;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(corridor(6));  // Observed free width: 0.60 m.
    const octomap::point3d start(.025,.025,.025);
    planner.makePlan({.025,.025,.025}, {.625,.025,.025});
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if (path.empty())
      throw std::runtime_error("formal-width global guidance rejected the observed 60 cm corridor");
    for (const auto& point : path) {
      if (std::abs(point.z - .025) > 1e-6 ||
          planner.queryWorld({static_cast<float>(point.x),
                              static_cast<float>(point.y),
                              static_cast<float>(point.z)}, .25, true) !=
              OctoPlanner3D::TraversabilityFailure::None)
        throw std::runtime_error("corridor guidance changed height or bypassed 3D clearance/support");
    }
    // The complete local body still owns its heading and swept motion. A
    // global guidance circle need not cover both offset cylinders at all yaw.
    config.robot_radius = .44;
    planner.setConfig(config);
    if (planner.queryWorld(start, .44, true) !=
        OctoPlanner3D::TraversabilityFailure::OccupiedBody)
      throw std::runtime_error("60 cm corridor no longer distinguishes the all-yaw circle");
    planner.makePlan({.025,.025,.025}, {.625,.025,.025});
    planner.getPlannerResults(path);
    if (!path.empty())
      throw std::runtime_error("legacy all-yaw circle unexpectedly fit the 60 cm corridor");

    config.robot_radius = .25;
    planner.setConfig(config);
    planner.setOctomap(corridor(4));  // Observed free width: 0.40 m.
    if (planner.queryWorld(start, 0.0, true) !=
        OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("narrow corridor fixture lost its original height/support");
    if (planner.queryWorld(start, .25, true) !=
        OctoPlanner3D::TraversabilityFailure::OccupiedBody)
      throw std::runtime_error("formal-width guidance ignored the 40 cm corridor walls");
    planner.makePlan({.025,.025,.025}, {.625,.025,.025});
    planner.getPlannerResults(path);
    if (!path.empty())
      throw std::runtime_error("formal-width guidance admitted the too-narrow 40 cm corridor");
  }

  static void fineVoxelsResolveGo2ClearanceWithoutShrinkingBody() {
    for (double resolution : {0.20, 0.10, 0.05}) {
      auto tree = std::make_shared<octomap::OcTree>(resolution);
      tree->updateNode(octomap::point3d(-1, -1, -1), false);
      tree->updateNode(octomap::point3d(1, 1, 1), false);
      // Populate an observed free scan envelope around the test body while
      // retaining the obstacle that exercises coarse-voxel expansion.
      const octomap::point3d body(.089, -.253, .026);
      for (int ix = -12; ix <= 12; ++ix) {
        for (int iy = -12; iy <= 12; ++iy) {
          for (int iz = -4; iz <= 4; ++iz) {
            tree->updateNode(octomap::point3d(
              body.x() + ix * resolution,
              body.y() + iy * resolution,
              body.z() + iz * resolution), false);
          }
        }
      }
      tree->updateNode(octomap::point3d(.589, -.253, .115), true);
      PlannerConfig config;
      config.support_height_m = .35;
      config.body_clearance_below_m = config.body_clearance_above_m = .10;
      OctoPlanner3D planner;
      planner.setConfig(config);
      planner.setOctomap(tree);
      const bool clear = planner.queryWorld(body, .43, false) ==
                         OctoPlanner3D::TraversabilityFailure::None;
      if (clear != (resolution == .05))
        throw std::runtime_error("Go2 clearance no longer distinguishes coarse voxel expansion");
      tree->updateNode(octomap::point3d(.30, -.253, .026), true);
      planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
      if (planner.queryWorld(body, .43, false) == OctoPlanner3D::TraversabilityFailure::None)
        throw std::runtime_error("fine voxels accepted an actual obstacle inside the body");
    }
  }
  static void unknownCellsDoNotAuthorizeMotion() {
    auto tree = std::make_shared<octomap::OcTree>(.1);
    const octomap::point3d body(.05, .05, .05);
    tree->updateNode(octomap::point3d(-1, -1, -1), false);
    tree->updateNode(octomap::point3d(1, 1, 1), false);
    tree->updateNode(octomap::point3d(body.x(), body.y(), -.05), true);
    PlannerConfig config;
    config.robot_radius = .01;
    config.snap_search_radius_cells = 0;
    // Isolate observed free space from ground support; support has its own tests.
    config.ground_support = false;
    config.support_radius_cells = 1;
    config.support_depth_cells = 1;
    config.enable_preblocked_costmap = false;
    config.obstacle_clearance_radius_cells = 0;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    planner.makePlan({body.x(), body.y(), body.z()}, {body.x(), body.y(), body.z()});
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if (!path.empty())
      throw std::runtime_error("unknown planning cell was authorized");

    tree->updateNode(body, false);
    planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
    planner.makePlan({body.x(), body.y(), body.z()}, {body.x(), body.y(), body.z()});
    planner.getPlannerResults(path);
    if (path.empty())
      throw std::runtime_error("observed free planning cell was rejected");
  }
  static void calibratedSupportUsesOccupiedSurface() {
    auto tree=std::make_shared<octomap::OcTree>(.05);
    for (int x=-15;x<=15;++x) for (int y=-15;y<=15;++y)
      tree->updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,-.325),true);
    tree->updateNode(octomap::point3d(1,1,1),false);
    PlannerConfig c;
    c.support_height_m=.35;c.support_height_tolerance_m=.05;
    c.body_clearance_below_m=c.body_clearance_above_m=.1;
    OctoPlanner3D planner;planner.setConfig(c);planner.setOctomap(tree);
    const octomap::point3d body(.025,.025,.025);
    for (int x = -3; x <= 3; ++x) for (int y = -3; y <= 3; ++y)
      for (int z = -3; z <= 3; ++z)
        tree->updateNode(octomap::point3d(
          body.x() + x * .05, body.y() + y * .05, body.z() + z * .05), false);
    planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
    c.direct_support_only=true;
    c.support_radius_cells=1;
    planner.setConfig(c);
    if (planner.queryWorld(body,.1,true)!=OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("strict direct support rejected an occupied exposed floor");
    // Production mode accepts a saved-map sampling hole when neighbouring
    // occupied floor cells still prove support. The strict direct check above
    // remains available as a diagnostic for maps that must contain the centre.
    c.direct_support_only=false;
    tree->deleteNode(octomap::point3d(.025,.025,-.325));
    planner.setConfig(c);
    if (planner.queryWorld(body,.1,true)!=OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("neighbouring occupied floor did not repair a sampling hole");
    for (int x=-15;x<=15;++x) for (int y=-15;y<=15;++y)
      tree->updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,-.275),false);
    planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
    if (planner.queryWorld(body,.1,true)!=OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("free ray just above neighbouring floor blocked sparse support");
    tree->updateNode(octomap::point3d(.025,.025,-.375),false);
    planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
    if (planner.queryWorld(body,.1,true)!=OctoPlanner3D::TraversabilityFailure::None)
      throw std::runtime_error("centre free ray vetoed an enclosed neighbouring floor patch");
    for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y)
      tree->deleteNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,-.325));
    planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
    if (planner.queryWorld(body,.1,true)!=OctoPlanner3D::TraversabilityFailure::GroundSupport)
      throw std::runtime_error("a gap without direct or surrounding floor evidence was accepted");
  }
  static void adjacentSupportSnapUsesNearestCell() {
    auto tree = std::make_shared<octomap::OcTree>(.05);
    for (int x=-8;x<=8;++x) for (int y=-8;y<=8;++y)
      tree->updateNode(octomap::point3d((x+.5)*.05,(y+.5)*.05,-.325),true);
    tree->updateNode(octomap::point3d(-1,-1,-1),false);
    tree->updateNode(octomap::point3d(1,1,1),false);
    // The centre column is absent from the saved occupied surface. A relaxed
    // one-voxel neighbour is valid, but the body collision check must remain.
    tree->deleteNode(octomap::point3d(.025,.025,-.325));
    PlannerConfig c;
    c.robot_radius=.10;
    c.support_height_m=.35;
    c.support_height_tolerance_m=.05;
    c.body_clearance_below_m=c.body_clearance_above_m=.10;
    c.direct_support_only=false;
    c.support_radius_cells=1;
    c.support_depth_cells=2;
    c.snap_search_radius_cells=1;
    OctoPlanner3D planner;
    planner.setConfig(c);
    for (int x = -3; x <= 9; ++x) for (int y = -3; y <= 3; ++y)
      for (int z = -3; z <= 3; ++z)
        tree->updateNode(octomap::point3d(.025 + x * .05, .025 + y * .05,
          .025 + z * .05), false);
    planner.setOctomap(tree);
    planner.makePlan({.025,.025,.025},{.325,.025,.025});
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if (path.empty() || planner.endpointResolution().failure !=
        OctoPlanner3D::EndpointResolutionInfo::Failure::None)
      throw std::runtime_error("adjacent support snap was rejected by nearest-free endpoint search");
  }
  static void leafBodyQueryMatchesFinestCellReference()
  {
    for (double r : {0.025, 0.05, 0.2}) {
      auto tree = std::make_shared<octomap::OcTree>(r);
      tree->updateNode(octomap::point3d(-2,-2,-1), false);
      tree->updateNode(octomap::point3d(2,2,1), false);
      for (int x = 0; x < 8; ++x)
        for (int y = 0; y < 8; ++y)
          for (int z = 0; z < 8; ++z)
            tree->updateNode(octomap::point3d((x+.5)*r,(y+.5)*r,(z+.5)*r), true);
      for (int i = -15; i <= 15; ++i)
        tree->updateNode(octomap::point3d((i+.5)*r,-.5, .025), true);
      PlannerConfig config;
      config.support_height_m = .35;
      config.body_clearance_below_m = config.body_clearance_above_m = .1;
      OctoPlanner3D planner;
      planner.setConfig(config);
      for (bool prune : {false, true}) {
        if (prune) tree->prune();
        planner.setOctomap(std::make_shared<octomap::OcTree>(*tree));
        for (int i = 0; i < 90; ++i) {
          const octomap::point3d p(-.9 + .021*i, -.7 + .17*(i%9), -.15 + .05*(i%8));
          const double radius = i%2 ? .43 : .25;
          const auto lo = planner.worldToGrid(p.x()-radius,p.y()-radius,p.z()-.1);
          const auto hi = planner.worldToGrid(p.x()+radius,p.y()+radius,p.z()+.1);
          bool occupied = false;
          for (int x = lo.x; x <= hi.x && !occupied; ++x)
            for (int y = lo.y; y <= hi.y && !occupied; ++y) {
              const double dx = std::max({x*r-p.x(),0.0,p.x()-(x+1)*r});
              const double dy = std::max({y*r-p.y(),0.0,p.y()-(y+1)*r});
              if (dx*dx+dy*dy > radius*radius+1e-9) continue;
              for (int z = lo.z; z <= hi.z; ++z) {
                if ((z+1)*r <= p.z()-.1+1e-7 || z*r >= p.z()+.1-1e-7) continue;
                const auto* node = tree->search((x+.5)*r,(y+.5)*r,(z+.5)*r);
                if (node && tree->isNodeOccupied(node)) { occupied = true; break; }
              }
            }
          const auto actual = planner.queryWorld(p,radius,false);
          if (occupied && actual != OctoPlanner3D::TraversabilityFailure::OccupiedBody)
            throw std::runtime_error("leaf body query missed an occupied finest cell");
          if (!occupied && actual == OctoPlanner3D::TraversabilityFailure::OccupiedBody)
            throw std::runtime_error("leaf body query invented an occupied finest cell");
        }
      }
    }
  }
  static void measuredBodyHeightSurvivesVoxelBoundaries()
  {
    for (double r : {0.2, 0.05}) {
      auto tree = std::make_shared<octomap::OcTree>(r);
      for (int x = -40; x <= 40; ++x)
      for (int y = -40; y <= 40; ++y) {
          tree->updateNode(octomap::point3d((x+.5)*r, (y+.5)*r, -.32), true);
          // The ray between the floor and the body crosses every voxel. Keep
          // each layer explicitly free so the support query sees a measured
          // gap, rather than treating one sparse sample as a drop.
          for (int z = -6; z <= 0; ++z)
            tree->updateNode(octomap::point3d(
              (x+.5)*r, (y+.5)*r, (z+.5)*r), false);
        }
      tree->updateNode(octomap::point3d(1.9,1.9,1.0), false);
      PlannerConfig config;
      config.robot_radius = .43;
      config.body_clearance_below_m = config.body_clearance_above_m = .10;
      // Keep the physical support interval wider than one coarse fixture
      // voxel; .20 m cells cannot represent a .35 m height exactly.
      config.support_height_m = .30;
      config.support_height_tolerance_m = .10;
      OctoPlanner3D planner;
      planner.setConfig(config);
      planner.setOctomap(tree);
      for (double z : {-.01, .01}) {
        planner.makePlan({.025,.025,z}, {.625,.025,z});
        std::vector<PointPose> path;
        planner.getPlannerResults(path);
        if (path.empty()) {
          throw std::runtime_error(
            "valid actual height rejected at voxel boundary r=" + std::to_string(r) +
            " z=" + std::to_string(z));
        }
        for (const auto &p : path)
          if (std::abs(p.z-z) > 1e-6)
            throw std::runtime_error("flat path changed measured body height");
      }
      tree = std::make_shared<octomap::OcTree>(*tree);
      tree->updateNode(octomap::point3d(.025,.025,.025), true);
      planner.setOctomap(tree);
      planner.makePlan({.025,.025,.01}, {.625,.025,.01});
      std::vector<PointPose> path;
      planner.getPlannerResults(path);
      const auto resolution_info = planner.endpointResolution();
      if (!path.empty() || resolution_info.failure !=
          OctoPlanner3D::EndpointResolutionInfo::Failure::StartSnapExhausted)
        throw std::runtime_error(
          "occupied raw start bypassed its required entry sweep");

      tree = std::make_shared<octomap::OcTree>(r);
      tree->updateNode(octomap::point3d(-2,-2,-1), false);
      tree->updateNode(octomap::point3d(2,2,1), false);
      planner.setOctomap(tree);
      planner.makePlan({.025,.025,.01}, {.625,.025,.01});
      planner.getPlannerResults(path);
      if (!path.empty() || planner.endpointResolution().failure !=
          OctoPlanner3D::EndpointResolutionInfo::Failure::StartSnapExhausted)
        throw std::runtime_error("unobserved start support was not handled by the official snap search");
    }
  }
  static void freeUnanchoredStartKeepsSafeQuantizationEntry()
  {
    auto tree = std::make_shared<octomap::OcTree>(.1);
    for (int x = -3; x <= 6; ++x)
      for (int y = -3; y <= 3; ++y)
        for (int z = -2; z <= 2; ++z)
          tree->updateNode(octomap::point3d((x+.5)*.1, (y+.5)*.1, (z+.5)*.1), false);
    tree->updateInnerOccupancy();
    PlannerConfig config;
    config.robot_radius = .01;
    config.body_clearance_below_m = config.body_clearance_above_m = .01;
    config.ground_support = false;
    config.max_slope = .57735;
    config.snap_search_radius_cells = 2;
    config.enable_preblocked_costmap = false;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const PointPose start{.071, .061, .05};
    planner.makePlan(start, {.35, .05, .05});
    std::vector<PointPose> path;
    planner.getPlannerResults(path);
    if (path.empty() || std::abs(path.front().x-.05)>1e-6 ||
        std::abs(path.front().y-.05)>1e-6 ||
        std::abs(path.front().z-start.z)>1e-6)
      throw std::runtime_error("safe free-space quantization entry was rejected");

    // The raw pose is clear while its containing cell centre overlaps a
    // neighbouring obstacle. Reject nearer vertical snaps, then keep looking
    // for the safe same-height quantization connection.
    config.robot_radius = .06;
    config.same_floor_preference = false;
    planner.setConfig(config);
    tree->updateNode(octomap::point3d(-.05, .05, .05), true);
    tree->updateInnerOccupancy();
    planner.setOctomap(tree);
    planner.makePlan({.099, .05, .05}, {.45, .05, .05});
    planner.getPlannerResults(path);
    if (path.empty() || std::abs(path.front().z-.05)>1e-6)
      throw std::runtime_error("entry filtering missed an available same-height candidate");
  }
  static void standingGo2CannotDuckByOneVoxel()
  {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    for (int x = -6; x <= 6; ++x)
      for (int y = -6; y <= 6; ++y) {
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, -0.3), true);
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, -0.1), false);
        // The planner queries the body centre at both candidate heights. Keep
        // that layer explicitly observed free so the height assertion tests
        // support geometry rather than missing-ray rejection.
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, 0.1), false);
      }
    tree->updateNode(octomap::point3d(2.1, 2.1, 0.9), false);
    PlannerConfig config;
    config.robot_radius = 0.43;
    config.body_clearance_below_m = config.body_clearance_above_m = 0.10;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto low = planner.worldToGrid(0.1, 0.1, -0.1);
    const auto standing = planner.worldToGrid(0.1, 0.1, 0.1);
    if (!planner.isCellTraversable(low, 0.43, true, true, 0, 2))
      throw std::runtime_error("fixture must reproduce uncalibrated low-body support");
    config.support_height_m = 0.35;
    config.support_height_tolerance_m = 0.05;
    planner.setConfig(config);
    if (planner.isCellTraversable(low, 0.43, true, true, 0, 2) ||
        !planner.isCellTraversable(standing, 0.43, true, true, 0, 2))
      throw std::runtime_error("standing Go2 height accepted a 20 cm virtual crouch");
  }
  static void footprintSamplesStayInEdgeCells()
  {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    for (int x = -2; x <= 2; ++x)
      for (int y = -2; y <= 2; ++y)
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, -0.1), true);
    tree->updateNode(octomap::point3d(2.1, 2.1, 0.9), false);
    PlannerConfig config;
    config.robot_radius = 0.43;
    config.max_step_height = 0.0;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto center = planner.worldToGrid(0.1, 0.1, 0.1);
    if (!planner.hasFootprintGroundSupport(center, 0.43, 2))
      throw std::runtime_error("support query extended outside the physical footprint edge cells");
    tree = std::make_shared<octomap::OcTree>(*tree);
    tree->deleteNode(octomap::point3d(0.1, 0.5, -0.1));
    planner.setOctomap(tree);
    if (planner.hasFootprintGroundSupport(center, 0.43, 2))
      throw std::runtime_error("missing support at a physical footprint edge was ignored");
  }
  static void supportMustBeTheFirstSurfaceBelowTheBody()
  {
    auto tree = std::make_shared<octomap::OcTree>(0.1);
    for (int x = -10; x <= 10; ++x)
      for (int y = -10; y <= 10; ++y)
        for (int z = 0; z <= 10; ++z)
          tree->updateNode(octomap::point3d((x + 0.5) * 0.1, (y + 0.5) * 0.1,
                                           (z + 0.5) * 0.1), z <= 2);
    PlannerConfig config;
    config.robot_radius = 0.2;
    config.support_height_m = 0.4;
    config.support_height_tolerance_m = 0.01;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto tooLow = planner.worldToGrid(0.05, 0.05, 0.45);
    const auto standing = planner.worldToGrid(0.05, 0.05, 0.65);
    for (bool strict : {false, true}) {
      if (planner.hasGroundSupport(tooLow, strict, 1, 5) ||
          !planner.hasGroundSupport(standing, strict, 1, 5))
        throw std::runtime_error("support height used slab interior instead of its top surface");
    }
    if (planner.hasFootprintGroundSupport(tooLow, 0.2, 5) ||
        !planner.hasFootprintGroundSupport(standing, 0.2, 5))
      throw std::runtime_error("footprint and centre disagree about the first support surface");
  }

  static void compareWithOctomap(double resolution)
  {
    auto tree = std::make_shared<octomap::OcTree>(resolution);
    for (const int base : {-16, 8}) {
      for (int x = base; x < base + 8; ++x) {
        for (int y = base; y < base + 8; ++y) {
          for (int z = base; z < base + 8; ++z) {
            tree->updateNode(octomap::point3d(
                (x + 0.5) * resolution, (y + 0.5) * resolution,
                (z + 0.5) * resolution), true);
          }
        }
      }
    }
    tree->updateNode(octomap::point3d(0.5 * resolution, 0.5 * resolution,
                                     0.5 * resolution), true);
    tree->updateNode(octomap::point3d(2.5 * resolution, 2.5 * resolution,
                                     2.5 * resolution), false);
    tree->updateInnerOccupancy();
    tree->prune();
    OctoPlanner3D planner;
    planner.setOctomap(tree);
    std::unordered_set<GridIndex, GridIndexHash> reference_centers;
    bool coarse_leaf = false;
    for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
      if (tree->isNodeOccupied(*it)) {
        coarse_leaf = coarse_leaf || it.getDepth() < tree->getTreeDepth();
        reference_centers.insert(planner.worldToGrid(it.getX(), it.getY(), it.getZ()));
      }
    }
    if (!coarse_leaf) throw std::runtime_error("mixed-depth fixture was not pruned");
    for (int x = -18; x <= 18; ++x) {
      for (int y = -18; y <= 18; ++y) {
        for (int z = -18; z <= 18; ++z) {
          const GridIndex index{x, y, z};
          const auto * node = tree->search(planner.gridToWorld(index));
          const bool occupied = node && tree->isNodeOccupied(node);
          if (planner.isOccupiedCell(index) != occupied ||
              planner.isOccupiedLeafCell(index) != (reference_centers.count(index) != 0)) {
            throw std::runtime_error("cached occupancy/leaf centre differs from OctoMap");
          }
        }
      }
    }
  }

  static void bodyCylinderDoesNotReachTheNextVerticalCell()
  {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    for (int x = -5; x <= 5; ++x)
      for (int y = -5; y <= 5; ++y) {
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, -0.3), true);
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, 0.3), true);
        tree->updateNode(octomap::point3d((x + 0.5) * 0.2, (y + 0.5) * 0.2, 0.1), false);
      }
    PlannerConfig config;
    config.robot_radius = 0.43;
    config.body_clearance_below_m = 0.10;
    config.body_clearance_above_m = 0.10;
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    const auto standing = planner.worldToGrid(0.1, 0.1, 0.1);
    if (!planner.isCellTraversable(standing, 0.43, true, true, 0, 2))
      throw std::runtime_error("0.10 m body clearance expanded into the next 0.20 m cell");
    config.body_clearance_above_m = 0.11;
    planner.setConfig(config);
    if (planner.isCellTraversable(standing, 0.43, true, true, 0, 2))
      throw std::runtime_error("body overlap with overhead voxel was ignored");
    config.body_clearance_above_m = 0.10;
    config.body_clearance_below_m = 0.31;
    planner.setConfig(config);
    if (planner.isCellTraversable(standing, 0.43, true, true, 0, 2))
      throw std::runtime_error("body overlap with lower occupied voxel was ignored");
  }
};

}  // namespace global_planner

namespace {

global_planner::PointPose point(int x, int y, int z, double resolution)
{
  return {(x + 0.5) * resolution, (y + 0.5) * resolution, (z + 0.5) * resolution};
}

std::shared_ptr<octomap::OcTree> makeMap(double resolution, bool obstacles)
{
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  const auto occupy = [&](int x, int y, int z) {
    const auto p = point(x, y, z, resolution);
    tree->updateNode(octomap::point3d(p.x, p.y, p.z), true);
  };
  // This fixture models a ray-observed free workspace rather than an
  // unexplored tree.  Unknown cells are intentionally exercised by the
  // dedicated test above; body-envelope checks here should measure only the
  // occupied-leaf geometry.
  for (int x = -20; x <= 20; ++x) {
    for (int y = -20; y <= 20; ++y) {
      for (int z = -20; z <= 20; ++z) {
        const auto p = point(x, y, z, resolution);
        tree->updateNode(octomap::point3d(p.x, p.y, p.z), false);
      }
    }
  }
  occupy(-32, -32, -32);
  occupy(32, 32, 32);
  if (obstacles) {
    occupy(-16, -16, -16);
    occupy(16, 16, 16);
  }
  tree->updateInnerOccupancy();
  return tree;
}

void expectPoint(global_planner::OctoPlanner3D & planner,
                 global_planner::PointPose p, bool free, const char * reason)
{
  planner.makePlan(p, p);
  std::vector<global_planner::PointPose> result;
  planner.getPlannerResults(result);
  if (result.empty() == free) throw std::runtime_error(reason);
}

void checkResolution(double resolution)
{
  global_planner::PlannerConfig config;
  config.robot_radius = 2.1 * resolution;
  config.ground_support = false;
  config.snap_search_radius_cells = 0;
  config.enable_preblocked_costmap = false;
  config.obstacle_clearance_radius_cells = 0;
  global_planner::OctoPlanner3D planner;
  planner.setConfig(config);
  planner.setOctomap(makeMap(resolution, true));

  // Body samples cross both positive and negative cache-block boundaries.
  for (const int edge : {-16, 16}) {
    expectPoint(planner, point(edge - 2, edge, edge, resolution), false,
                "obstacle at body radius was missed across a block boundary");
    expectPoint(planner, point(edge - 2, edge - 2, edge, resolution), true,
                "hemisphere was enlarged to its bounding box");
    expectPoint(planner, point(edge, edge, edge - 2, resolution), false,
                "obstacle above body was missed");
    expectPoint(planner, point(edge, edge, edge + 1, resolution), true,
                "support below hemisphere was treated as body collision");
  }

  config.body_clearance_below_m = 1.1 * resolution;
  config.body_clearance_above_m = 0.5 * resolution;
  planner.setConfig(config);
  const auto above = point(16, 16, 17, resolution);
  expectPoint(planner, above, false, "new cylinder did not reject a below-body obstacle");
  planner.setOctomap(makeMap(resolution, false));
  expectPoint(planner, above, true, "map replacement retained an occupied block");

  global_planner::ExternalBlockedRegion blocked;
  blocked.center = above;
  blocked.radius_xy_m = resolution * 0.2;
  blocked.min_z = above.z - resolution * 0.1;
  blocked.max_z = above.z + resolution * 0.1;
  planner.setExternalPreblockedRegions({blocked});
  expectPoint(planner, above, false, "temporary region did not invalidate free body result");
  planner.setExternalPreblockedRegions({});
  expectPoint(planner, above, true, "removed temporary region retained a blocked body result");

  config.robot_radius = 0.1 * resolution;
  config.body_clearance_below_m = config.body_clearance_above_m = 0.0;
  config.snap_search_radius_cells = 1;
  planner.setConfig(config);
  auto blocked_start = makeMap(resolution, true);
  for (int z : {15, 17}) {
    const auto p = point(16, 16, z, resolution);
    blocked_start->updateNode(octomap::point3d(p.x, p.y, p.z), true);
  }
  planner.setOctomap(blocked_start);
  expectPoint(planner, point(16, 16, 16, resolution), false,
              "occupied coincident start bypassed its required entry sweep");
  const auto endpoint_resolution = planner.endpointResolution();
  if (endpoint_resolution.failure !=
        global_planner::OctoPlanner3D::EndpointResolutionInfo::Failure::StartSnapExhausted)
    throw std::runtime_error(
      "occupied coincident start did not report an unavailable executable entry");
}

}  // namespace

int main()
{
  try {
    global_planner::OctoPlannerGridQueryTest::supportHeightUsesActualPoseWithinVoxel();
    global_planner::OctoPlannerGridQueryTest::supportHeightUsesBothFacesOfTheOccupiedVoxel();
    global_planner::OctoPlannerGridQueryTest::neighboringSupportInterpolatesHolesButNotFloorEdges();
    global_planner::OctoPlannerGridQueryTest::supportedRouteTurnsInsideAnLShapedFloor();
    global_planner::OctoPlannerGridQueryTest::supportedRoutesDoNotRequireObservedCentreAir();
    global_planner::OctoPlannerGridQueryTest::startSnapUsesRouteHeightPreference();
    global_planner::OctoPlannerGridQueryTest::goalSnapUsesRequestedPosition();
    global_planner::OctoPlannerGridQueryTest::exactGoalConnectionDoesNotSpendArrivalToleranceOnGridQuantization();
    global_planner::OctoPlannerGridQueryTest::temporaryObstacleDoesNotCrossObservedFloor();
    global_planner::OctoPlannerGridQueryTest::stepTransitionsKeepObservedSupportAndBodyClearance();
    global_planner::OctoPlannerGridQueryTest::routeClearancePassesNarrowCorridor();
    global_planner::OctoPlannerGridQueryTest::formalWidthGuidanceDoesNotRequireAnAllYawCircle();
    global_planner::OctoPlannerGridQueryTest::fineVoxelsResolveGo2ClearanceWithoutShrinkingBody();
    global_planner::OctoPlannerGridQueryTest::unknownCellsDoNotAuthorizeMotion();
    checkResolution(0.1);
    global_planner::OctoPlannerGridQueryTest::calibratedSupportUsesOccupiedSurface();
    global_planner::OctoPlannerGridQueryTest::adjacentSupportSnapUsesNearestCell();
    global_planner::OctoPlannerGridQueryTest::leafBodyQueryMatchesFinestCellReference();
    checkResolution(0.2);
    global_planner::OctoPlannerGridQueryTest::compareWithOctomap(0.1);
    global_planner::OctoPlannerGridQueryTest::compareWithOctomap(0.2);
    global_planner::OctoPlannerGridQueryTest::supportMustBeTheFirstSurfaceBelowTheBody();
    global_planner::OctoPlannerGridQueryTest::footprintSamplesStayInEdgeCells();
    global_planner::OctoPlannerGridQueryTest::bodyCylinderDoesNotReachTheNextVerticalCell();
    global_planner::OctoPlannerGridQueryTest::standingGo2CannotDuckByOneVoxel();
    global_planner::OctoPlannerGridQueryTest::measuredBodyHeightSurvivesVoxelBoundaries();
    global_planner::OctoPlannerGridQueryTest::freeUnanchoredStartKeepsSafeQuantizationEntry();
    std::cout << "grid query boundaries, body envelopes and invalidation passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
