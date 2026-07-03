#include "octoplanner3d_core.hpp"

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// OctoPlanner3D does not expose runtime setters for these parameters upstream.
// Keep the access shim local so product code does not patch imported source.
#define private public
#include "global_planner.h"
#undef private

#if defined(OCTOPLANNER3D_ENABLE_PCD)
#include "pcd2octomap_converter.h"
#endif

namespace octoplanner3d::runtime {
namespace {

global_planner::PointPose toPlannerPoint(const Point & point)
{
  global_planner::PointPose out;
  out.x = point.x;
  out.y = point.y;
  out.z = point.z;
  if (!std::isfinite(out.x) || !std::isfinite(out.y) || !std::isfinite(out.z)) {
    throw std::runtime_error("point contains non-finite value");
  }
  return out;
}

Point fromPlannerPoint(const global_planner::PointPose & point)
{
  return Point{point.x, point.y, point.z};
}

std::filesystem::path pcdOutputPath(const std::filesystem::path & pcd_path)
{
  auto out = pcd_path;
  out += ".octoplanner3d.ot";
  return out;
}

std::shared_ptr<octomap::OcTree> loadOctomap(const std::string & map_path)
{
  const std::filesystem::path path(map_path);
  if (!std::filesystem::exists(path)) {
    throw std::runtime_error("map_path does not exist: " + map_path);
  }

  const std::string extension = path.extension().string();
  if (extension == ".pcd" || extension == ".PCD") {
#if defined(OCTOPLANNER3D_ENABLE_PCD)
    pcd2octomap::Pcd2OctomapConverter converter;
    converter.setInputPcdFile(path.string());
    converter.setOutputBtFile(pcdOutputPath(path).string());
    converter.setFreeEnvelopeLayers(3);
    converter.setFreeEnvelopeDilationCells(1);
    if (!converter.convert()) {
      throw std::runtime_error("failed to convert PCD to OctoMap: " + map_path);
    }
    auto tree = converter.getOctomap();
    if (!tree) {
      throw std::runtime_error("PCD converter returned null OctoMap");
    }
    return tree;
#else
    throw std::runtime_error(
      "PCD input requires a headless build with PCL support; install PCL or provide a .bt OctoMap: "
      + map_path);
#endif
  }

  if (extension == ".bt" || extension == ".BT") {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    if (!tree->readBinary(path.string())) {
      throw std::runtime_error("failed to read OctoMap .bt file: " + map_path);
    }
    return tree;
  }

  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap::AbstractOcTree::read(path.string()));
  if (!abstract_tree) {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    if (tree->readBinary(path.string())) {
      return tree;
    }
    throw std::runtime_error("failed to read OctoMap file: " + map_path);
  }
  auto * raw_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.release());
  if (raw_tree == nullptr) {
    throw std::runtime_error("OctoMap file is not an OcTree: " + map_path);
  }
  return std::shared_ptr<octomap::OcTree>(raw_tree);
}

double distanceToGoal(const Point & point, const Point & goal)
{
  const double dx = point.x - goal.x;
  const double dy = point.y - goal.y;
  const double dz = point.z - goal.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void applyPlannerOptions(
  global_planner::GlobalPlanner & planner,
  const PlannerOptions & options)
{
  if (std::isfinite(options.robot_radius) && options.robot_radius > 0.0) {
    planner.robot_radius_ = options.robot_radius;
  }
  if (options.max_iterations > 0) {
    planner.max_iterations_ = options.max_iterations;
  }
  planner.snap_search_radius_cells_ =
    std::max(0, options.snap_search_radius_cells);
  planner.require_ground_support_ = options.require_ground_support;
  planner.strict_direct_ground_support_ = options.strict_direct_ground_support;
  planner.ground_support_xy_radius_cells_ =
    std::max(0, options.ground_support_xy_radius_cells);
  planner.ground_support_depth_cells_ =
    std::max(1, options.ground_support_depth_cells);
  planner.enable_preblocked_costmap_ = options.enable_preblocked_costmap;
  planner.preblocked_costmap_radius_cells_ =
    std::max(0, options.preblocked_costmap_radius_cells);
  if (std::isfinite(options.preblocked_costmap_weight) &&
      options.preblocked_costmap_weight >= 0.0) {
    planner.preblocked_costmap_weight_ = options.preblocked_costmap_weight;
  }
  planner.lowest_traversable_only_ = options.lowest_traversable_only;
  if (std::isfinite(options.floor_change_penalty) && options.floor_change_penalty >= 0.0) {
    planner.floor_change_penalty_ = options.floor_change_penalty;
  }
  if (std::isfinite(options.max_step_height) && options.max_step_height >= 0.0) {
    planner.max_step_height_ = options.max_step_height;
  }
  if (std::isfinite(options.max_slope) && options.max_slope >= 0.0) {
    planner.max_slope_ = options.max_slope;
  }
  planner.same_floor_preference_ = options.same_floor_preference;
  if (std::isfinite(options.same_floor_z_tolerance) &&
      options.same_floor_z_tolerance >= 0.0) {
    planner.same_floor_z_tolerance_ = options.same_floor_z_tolerance;
  }
  planner.obstacle_clearance_radius_cells_ =
    std::max(0, options.obstacle_clearance_radius_cells);
  if (std::isfinite(options.obstacle_clearance_weight) &&
      options.obstacle_clearance_weight >= 0.0) {
    planner.obstacle_clearance_weight_ = options.obstacle_clearance_weight;
  }
}

}  // namespace

bool pcdConversionEnabled()
{
#if defined(OCTOPLANNER3D_ENABLE_PCD)
  return true;
#else
  return false;
#endif
}

PlanResult runPlan(const PlanRequest & request)
{
  if (request.map_path.empty()) {
    throw std::runtime_error("map_path must not be empty");
  }

  const auto started = std::chrono::steady_clock::now();
  const auto start = toPlannerPoint(request.start);
  const auto goal = toPlannerPoint(request.goal);

  std::vector<global_planner::PointPose> native_path;
  auto map = loadOctomap(request.map_path);
  global_planner::GlobalPlanner planner;
  applyPlannerOptions(planner, request.options);
  planner.setOctomap(map);
  planner.makePlan(start, goal);
  planner.getPlannerResults(native_path);

  PlanResult result;
  result.options = request.options;
  result.path.reserve(native_path.size());
  for (const auto & point : native_path) {
    result.path.push_back(fromPlannerPoint(point));
  }
  result.ok = !result.path.empty();
  result.pcd_conversion = pcdConversionEnabled();
  if (!result.path.empty()) {
    result.goal_error_m = distanceToGoal(result.path.back(), request.goal);
    result.reached_goal = result.goal_error_m <= 0.5;
  }

  const auto finished = std::chrono::steady_clock::now();
  result.elapsed_ms =
    std::chrono::duration<double, std::milli>(finished - started).count();
  return result;
}

}  // namespace octoplanner3d::runtime
