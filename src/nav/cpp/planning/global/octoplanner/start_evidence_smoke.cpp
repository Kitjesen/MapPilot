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
  const auto unsupported=session.run(path,identity,request);
  require(!unsupported.ok, "free air fabricated ground support");
  tree.updateNode(-.975,.025,.125,true);
  tree.updateInnerOccupancy();
  require(tree.write(path.string()), "could not update legacy fixture");
  request.start={-.975,.025,.075,{}}; request.goal={1.025,.025,.075,{}};
  c.support_height_m=0; c.body_clearance_below_m=.01; c.body_clearance_above_m=.1;
  require(r::runPlan(path,request).ok, "legacy initial-body occupancy prevented departure");
  c.body_clearance_below_m=0;c.body_clearance_above_m=0;
  require(r::runPlan(path,request).ok,"legacy hemisphere initial occupancy prevented departure");
  c.body_clearance_below_m=.01;c.body_clearance_above_m=.1;
  c.lowest_traversable_only=true;
  request.start.x=-.475;
  request.start.z=request.goal.z=.025;
  r::PlannerSession lowest;
  const auto lowest_clear=lowest.run(path,identity,request);
  require(lowest_clear.ok,"lowest fixture is not initially traversable");
  for(int y=-12;y<=12;++y) for(int z=0;z<16;++z)
    tree.updateNode(.025,(y+.5)*.05,(z+.5)*.05,true,true);
  tree.updateInnerOccupancy();
  require(tree.write(path.string()),"could not write outside-wall fixture");
  request.start={-.975,.025,.35,{}};request.goal={1.025,.025,.35,{}};
  c.lowest_traversable_only=false;c.support_height_m=.35;
  c.body_clearance_below_m=.1;c.body_clearance_above_m=.2;
  r::PlannerSession wall;
  require(!wall.run(path,identity,request).ok,"initial-body exemption cleared a wall outside the start");
  octomap::OcTree departure(.05);
  departure.updateNode(-.5,-.5,-.1,false);
  departure.updateNode(1.5,.5,.8,false);
  for(int x=0;x<26;++x)for(int y=-8;y<=8;++y)
    departure.updateNode((x+.5)*.05,(y+.5)*.05,-.025,true);
  departure.updateInnerOccupancy();
  require(departure.write(path.string()),"could not write departure fixture");
  request.start={-.02,.025,.35,{}};request.goal={.975,.025,.35,{}};
  const auto departure_result=r::runPlan(path,request);
  require(departure_result.ok && std::abs(departure_result.path.front().x+.02)<1e-6,
          "missing ground observation at the actual start prevented departure");
  octomap::OcTree noisy_departure(.05);
  noisy_departure.updateNode(-.5,-.5,-.1,false);
  noisy_departure.updateNode(1.5,.5,.8,false);
  for(int x=4;x<26;++x)for(int y=-8;y<=8;++y)
    noisy_departure.updateNode((x+.5)*.05,(y+.5)*.05,-.025,true);
  noisy_departure.updateInnerOccupancy();
  require(noisy_departure.write(path.string()),"could not write noisy departure fixture");
  require(r::runPlan(path,request).ok,"missing ground observations along departure prevented planning");
  std::filesystem::remove(path);
  std::cout << "Start evidence passed: initial saved occupancy admitted, outside walls retained, exact start and request isolation preserved\n";
  return 0;
} catch (const std::exception &error) {
  std::cerr << error.what() << '\n'; return 1;
}
