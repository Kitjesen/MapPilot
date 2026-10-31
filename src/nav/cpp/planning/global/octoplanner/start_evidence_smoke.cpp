#include "octoplanner3d_core.hpp"
#include <octomap/OcTree.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace r = octoplanner3d::runtime;

int main() try {
  const auto require = [](bool ok, const char *message) {
    if (!ok) throw std::runtime_error(message);
  };
  octomap::OcTree tree(.05);
  for (int x=-30; x<31; ++x) for (int y=-12; y<13; ++y) for (int z=-1; z<16; ++z)
    tree.updateNode((x+.5)*.05, (y+.5)*.05, (z+.5)*.05, z==-1, true);
  tree.updateNode(-.975,.025,.325,true);
  tree.updateInnerOccupancy();
  const auto path = std::filesystem::temp_directory_path() /
    ("lingtu-start-evidence-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".ot");
  require(tree.write(path.string()), "could not write fixture");
  r::PlanRequest request;
  request.start={-.975,.025,.35,{}};
  request.goal={1.025,.025,.35,{}};
  auto &c=request.options;
  c.robot_radius=.1; c.body_clearance_below_m=.1; c.body_clearance_above_m=.2;
  c.support_height_m=.35; c.support_height_tolerance_m=.05;
  c.max_iterations=10000; c.enable_preblocked_costmap=false;
  c.obstacle_clearance_radius_cells=0; c.terminal_goal_tolerance_m=.02;
  c.terminal_goal_xy_tolerance_m=.02; c.terminal_goal_z_tolerance_m=.02;
  r::PlannerSession session;
  const lingtu::nav::plan::MapIdentity identity{"start-evidence",1,"map"};
  r::PlannerSession interrupted;
  int preparation_checks=0;
  const auto cancelled=interrupted.run(path,identity,request,[&] { return ++preparation_checks>5; });
  require(cancelled.cancelled && cancelled.path.empty() && interrupted.prepareCount()==0,
          "cancelled graph preparation published a partial cache");
  require(interrupted.run(path,identity,request).ok && interrupted.prepareCount()==1,
          "graph preparation did not recover after cancellation");
  const auto before=session.run(path,identity,request);
  require(before.ok && before.reached_goal, "saved occupancy in the initial body prevented departure");
  require(std::abs(before.path.front().x-request.start.x)<1e-6 &&
    std::abs(before.path.front().z-request.start.z)<1e-6,"departure silently moved the actual start");
  request.observed_free=[](const auto &box) {
    return std::abs(box.center.x+.975)<1e-6 && std::abs(box.center.y-.025)<1e-6 &&
      std::abs(box.center.z-.325)<1e-6 && std::abs(box.half_size.x-.025)<1e-6;
  };
  const auto after=session.run(path,identity,request);
  require(after.ok && after.reached_goal, "current measured free did not clear stale start occupancy");
  require(after.support_surface && after.support_surface({-.975,.025,0},.05).has_value(),
          "shared support reintroduced the stale body obstacle as a ground rejection");
  require(std::abs(after.path.front().x-request.start.x)<1e-6 &&
    std::abs(after.path.front().z-request.start.z)<1e-6, "actual start was silently moved");
  const auto live_hit=[](double x, double y, double z) {
    return [x,y,z](const auto &center, double radius, double below, double above, bool hemisphere) {
      if (z+.025<=center.z-below+1e-8 || z-.025>=center.z+above-1e-8) return false;
      const double dx=std::max(0.0,std::abs(x-center.x)-.025);
      const double dy=std::max(0.0,std::abs(y-center.y)-.025);
      const double dz=hemisphere ? std::max(0.0,z-.025-center.z) : 0.0;
      return dx*dx+dy*dy+dz*dz<=radius*radius;
    };
  };
  request.observed_free=[](const auto &) { return true; };
  request.observed_occupied=live_hit(-.975,.025,.325);
  require(session.run(path,identity,request).failure_reason=="start_occupied",
          "current raw hit lost to live free clearing");
  request.observed_occupied=live_hit(1.025,.025,.325);
  require(!session.run(path,identity,request).ok,
          "saved-free target ignored a current body obstacle");
  request.observed_occupied=live_hit(.025,.025,-.025);
  require(session.run(path,identity,request).ok,"live ground below the body became a body obstacle");
  request.observed_occupied={};
  require(session.run(path,identity,request).ok,"live obstacle leaked into a later request");
  request.observed_free={};
  require(session.run(path,identity,request).ok, "initial-body exemption depended on free observations");
  require(tree.isNodeOccupied(tree.search(-.975,.025,.325)), "saved map was mutated");
  request.start={1.025,.025,.35,{}}; request.goal={-.975,.025,.35,{}};
  require(!session.run(path,identity,request).ok,"initial-body exemption leaked into another request");
  request.start={-.975,.025,.35,{}}; request.goal={1.025,.025,.35,{}};
  request.temporary_overlay.revision=1;
  request.temporary_overlay.frame_epoch=1;
  request.temporary_overlay.obstacle_generation=1;
  request.temporary_overlay.blocked_regions.push_back({{-.975,.025,0},.1,.2,.5});
  require(session.run(path,identity,request).failure_reason=="start_temporary_obstacle",
          "initial-body exemption erased an explicit forbidden region");
  request.temporary_overlay={};
  request.start.x=.525;
  request.start.z=.7;
  request.observed_free=[](const auto &) { return true; };
  const auto unsupported=session.run(path,identity,request);
  require(!unsupported.ok && unsupported.failure_reason=="start_unsupported", "free air fabricated ground support");
  tree.updateNode(-.975,.025,.125,true);
  tree.updateInnerOccupancy();
  require(tree.write(path.string()), "could not update legacy fixture");
  request.start={-.975,.025,.075,{}}; request.goal={1.025,.025,.075,{}};
  c.support_height_m=0; c.body_clearance_below_m=.01; c.body_clearance_above_m=.1;
  request.observed_free={};
  require(r::runPlan(path,request).ok, "legacy initial-body occupancy prevented departure");
  request.observed_free=[](const auto &box) {
    return std::abs(box.center.x+.975)<1e-6 && std::abs(box.center.y-.025)<1e-6 &&
      std::abs(box.center.z-.125)<1e-6;
  };
  const auto legacy=r::runPlan(path,request);
  require(legacy.ok && legacy.reached_goal, "legacy candidate admission ignored current free evidence");
  request.observed_occupied=live_hit(1.025,.025,.125);
  require(!r::runPlan(path,request).ok,"legacy target ignored a current body obstacle");
  request.observed_occupied={};
  request.observed_free={};
  c.body_clearance_below_m=0;c.body_clearance_above_m=0;
  require(r::runPlan(path,request).ok,"legacy hemisphere initial occupancy prevented departure");
  request.observed_occupied=live_hit(-.975,.025,.125);
  require(r::runPlan(path,request).failure_reason=="start_occupied",
          "legacy hemisphere ignored a current start hit");
  request.observed_occupied={};
  c.body_clearance_below_m=.01;c.body_clearance_above_m=.1;
  c.lowest_traversable_only=true;
  request.start.x=-.475;
  request.start.z=request.goal.z=.025;
  r::PlannerSession lowest;
  const auto lowest_clear=lowest.run(path,identity,request);
  require(lowest_clear.ok,"lowest fixture is not initially traversable");
  request.observed_occupied=live_hit(1.025,.025,.075);
  require(!lowest.run(path,identity,request).ok,"prebuilt lowest candidates ignored current occupancy");
  request.observed_occupied={};
  require(lowest.run(path,identity,request).ok,"lowest live occupancy leaked across snapshots");
  for(int y=-12;y<=12;++y) for(int z=0;z<16;++z)
    tree.updateNode(.025,(y+.5)*.05,(z+.5)*.05,true,true);
  tree.updateInnerOccupancy();
  require(tree.write(path.string()),"could not write outside-wall fixture");
  request.start={-.975,.025,.35,{}};request.goal={1.025,.025,.35,{}};
  c.lowest_traversable_only=false;c.support_height_m=.35;
  c.body_clearance_below_m=.1;c.body_clearance_above_m=.2;
  request.observed_free={};
  r::PlannerSession wall;
  require(!wall.run(path,identity,request).ok,"initial-body exemption cleared a wall outside the start");
  std::filesystem::remove(path);
  std::cout << "Start evidence passed: initial saved occupancy admitted, current hits and outside walls retained, exact start and request isolation preserved\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n'; return 1;
}
