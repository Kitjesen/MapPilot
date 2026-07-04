#include "octoplanner3d_core.hpp"

#include <octomap/OcTree.h>

#include <filesystem>
#include <iostream>
#include <string>

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
    }
  }
}

void addRemoteBoundsPillar(octomap::OcTree & tree)
{
  for (int iz = 1; iz <= 14; ++iz) {
    occupyCell(tree, 28, 28, iz);
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
  addRemoteBoundsPillar(tree);
  tree.updateInnerOccupancy();

  auto path = std::filesystem::temp_directory_path() / "octoplanner3d_no_air_climb.bt";
  if (!tree.writeBinary(path.string())) {
    throw std::runtime_error("failed to write test OctoMap: " + path.string());
  }
  return path;
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
  const octoplanner3d::runtime::Point & goal)
{
  octoplanner3d::runtime::PlanRequest request;
  request.map_path = map_path;
  request.start = start;
  request.goal = goal;
  request.options = testOptions();
  return octoplanner3d::runtime::runPlan(request);
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

    const auto air_result =
      plan(map_path.string(), start, {center(18), center(0), center(10)});
    if (air_result.ok) {
      std::cerr << "unsupported air goal unexpectedly planned; points="
                << air_result.path.size()
                << " goal_error=" << air_result.goal_error_m << "\n";
      return 2;
    }

    std::cout
      << "{\"ok\":true,"
      << "\"ground_points\":" << ground_result.path.size() << ","
      << "\"air_goal_rejected\":true,"
      << "\"map_path\":\"" << map_path.string() << "\"}\n";
    return 0;
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << "\n";
    return 3;
  }
}
