#include "octoplanner3d_core.hpp"

#include <octomap/OcTree.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace r = octoplanner3d::runtime;

int main() try {
  const auto path = std::filesystem::temp_directory_path() /
      ("lingtu-xyz-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".ot");
  const auto require = [](bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
  };
  r::PlanRequest request;
  request.start = {-1, 0, .35, {}};
  request.goal = {1, 0, .35, 1.2};
  request.options.robot_radius = .12;
  request.options.body_clearance_below_m = .1;
  request.options.body_clearance_above_m = .2;
  request.options.support_height_m = .35;
  request.options.support_height_tolerance_m = .05;
  request.options.direct_support_only = true;
  request.options.support_radius_cells = 0;
  request.options.max_step_height = .15;
  request.options.max_iterations = 20000;
  request.options.terminal_goal_tolerance_m = .01;
  request.options.terminal_goal_xy_tolerance_m = .01;
  request.options.terminal_goal_z_tolerance_m = .01;
  request.options.enable_preblocked_costmap = false;
  request.options.obstacle_clearance_radius_cells = 0;
  for (int scene = 0; scene < 3; ++scene) {
    octomap::OcTree tree(.1);
    for (int x=-15; x<15; ++x) for (int y=-10; y<10; ++y) for (int z=-1; z<9; ++z) {
      const bool ground = z == -1 && !(scene == 2 && x >= -3 && x < 3);
      const bool wall = scene == 1 && x == 0 && z >= 0;
      tree.updateNode((x+.5)*.1, (y+.5)*.1, (z+.5)*.1, ground || wall, true);
    }
    tree.updateInnerOccupancy();
    require(tree.write(path.string()), "could not save XYZ fixture");
    const auto result = r::runPlan(path, request);
    if (scene == 0) {
      require(result.ok && result.reached_goal, "supported XYZ guidance did not reach its goal");
      auto alternate = request;
      alternate.start.yaw = 2.4;
      alternate.goal.yaw = -2.4;
      const auto other = r::runPlan(path, alternate);
      require(other.ok && other.reached_goal && other.path.size() == result.path.size(),
              "endpoint heading changed XYZ reachability");
      for (std::size_t i=0; i<result.path.size(); ++i) {
        require(std::hypot(result.path[i].x-other.path[i].x,
                           result.path[i].y-other.path[i].y,
                           result.path[i].z-other.path[i].z)<1e-9,
                "endpoint heading changed the XYZ route");
      }
      const auto cancelled = r::runPlan(path, request, [] { return true; });
      require(cancelled.cancelled && cancelled.path.empty(), "cancellation returned an executable route");
    } else {
      require(!result.ok, scene == 1 ? "XYZ route crossed a solid wall" : "XYZ route crossed a ground gap");
    }
  }
  {
    octomap::OcTree coarse(.2);
    for(int x=-6;x<=6;++x) for(int y=-6;y<=6;++y) for(int z=0;z<=6;++z)
      coarse.updateNode((x+.5)*.2,(y+.5)*.2,(z+.5)*.2,z==0,true);
    coarse.updateInnerOccupancy();
    require(coarse.write(path.string()), "could not save coarse exact-goal fixture");
    auto precise=request;
    precise.start={-.5,-.5,.5,{}};
    precise.goal={0,0,.435,{}};
    precise.options.robot_radius=.02;
    precise.options.support_height_m=.45;
    precise.options.support_height_tolerance_m=.11;
    precise.options.lowest_traversable_only=true;
    precise.options.terminal_goal_tolerance_m=.15;
    precise.options.terminal_goal_xy_tolerance_m=.15;
    precise.options.terminal_goal_z_tolerance_m=.11;
    const auto result=r::runPlan(path,precise);
    require(result.ok && result.reached_goal && result.goal_error_m<1e-6,
            "runtime applied arrival tolerance to a coarse grid node instead of the exact goal");
  }
  std::filesystem::remove(path);
  std::cout << "XYZ passed: heading-independent guidance, static wall, ground gap and cancellation\n";
  return 0;
} catch (const std::exception &e) {
  std::cerr << e.what() << '\n';
  return 1;
}
