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

#include "global_planner.h"

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

double xyDistanceToGoal(const Point & point, const Point & goal)
{
  const double dx = point.x - goal.x;
  const double dy = point.y - goal.y;
  return std::sqrt(dx * dx + dy * dy);
}

double zDistanceToGoal(const Point & point, const Point & goal)
{
  return std::abs(point.z - goal.z);
}

global_planner::PlannerConfig plannerConfig(const PlannerOptions & options)
{
  global_planner::PlannerConfig config;
  if (std::isfinite(options.robot_radius) && options.robot_radius > 0.0) {
    config.robot_radius = options.robot_radius;
  }
  config.body_clearance_below_m =
    std::isfinite(options.body_clearance_below_m) && options.body_clearance_below_m > 0.0
    ? options.body_clearance_below_m
    : 0.0;
  config.body_clearance_above_m =
    std::isfinite(options.body_clearance_above_m) && options.body_clearance_above_m > 0.0
    ? options.body_clearance_above_m
    : 0.0;
  if (options.max_iterations > 0) {
    config.max_iterations = options.max_iterations;
  }
  config.snap_search_radius_cells =
    std::max(0, options.snap_search_radius_cells);
  config.require_ground_support = options.require_ground_support;
  config.strict_direct_ground_support = options.strict_direct_ground_support;
  config.ground_support_xy_radius_cells =
    std::max(0, options.ground_support_xy_radius_cells);
  config.ground_support_depth_cells =
    std::max(1, options.ground_support_depth_cells);
  config.support_height_m =
    std::isfinite(options.support_height_m) && options.support_height_m > 0.0
    ? options.support_height_m
    : 0.0;
  config.support_height_tolerance_m =
    std::isfinite(options.support_height_tolerance_m) && options.support_height_tolerance_m >= 0.0
    ? options.support_height_tolerance_m
    : 0.0;
  config.support_patch_radius_cells =
    std::max(0, options.support_patch_radius_cells);
  config.support_patch_min_samples =
    std::clamp(options.support_patch_min_samples, 0, 5);
  config.enable_preblocked_costmap = options.enable_preblocked_costmap;
  config.preblocked_costmap_radius_cells =
    std::max(0, options.preblocked_costmap_radius_cells);
  if (std::isfinite(options.preblocked_costmap_weight) &&
      options.preblocked_costmap_weight >= 0.0) {
    config.preblocked_costmap_weight = options.preblocked_costmap_weight;
  }
  config.lowest_traversable_only = options.lowest_traversable_only;
  if (std::isfinite(options.floor_change_penalty) && options.floor_change_penalty >= 0.0) {
    config.floor_change_penalty = options.floor_change_penalty;
  }
  if (std::isfinite(options.max_step_height) && options.max_step_height >= 0.0) {
    config.max_step_height = options.max_step_height;
  }
  if (std::isfinite(options.max_slope) && options.max_slope >= 0.0) {
    config.max_slope = options.max_slope;
  }
  config.same_floor_preference = options.same_floor_preference;
  if (std::isfinite(options.same_floor_z_tolerance) &&
      options.same_floor_z_tolerance >= 0.0) {
    config.same_floor_z_tolerance = options.same_floor_z_tolerance;
  }
  config.obstacle_clearance_radius_cells =
    std::max(0, options.obstacle_clearance_radius_cells);
  if (std::isfinite(options.obstacle_clearance_weight) &&
      options.obstacle_clearance_weight >= 0.0) {
    config.obstacle_clearance_weight = options.obstacle_clearance_weight;
  }
  return config;
}

double positiveOrDefault(double value, double fallback)
{
  if (std::isfinite(value) && value > 0.0) {
    return value;
  }
  return fallback;
}

bool hasAcceptableSameFloorExcursion(
  const PlanRequest & request,
  const std::vector<Point> & path)
{
  const auto & options = request.options;
  if (!options.same_floor_preference || path.empty()) {
    return true;
  }

  const double start_goal_dz = std::abs(request.goal.z - request.start.z);
  const double tolerance = std::max(0.0, options.same_floor_z_tolerance);
  const double max_excursion = options.max_same_floor_z_excursion;
  if (start_goal_dz > tolerance || !std::isfinite(max_excursion) || max_excursion <= 0.0) {
    return true;
  }

  const auto [minimum, maximum] = std::minmax_element(
    path.begin(),
    path.end(),
    [](const Point & lhs, const Point & rhs) { return lhs.z < rhs.z; });
  return maximum->z - minimum->z <= max_excursion;
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

PlanResult runPlan(
  const PlanRequest & request,
  const CancelCheck & cancel_check)
{
  if (request.map_path.empty()) {
    throw std::runtime_error("map_path must not be empty");
  }

  const auto started = std::chrono::steady_clock::now();
  if (cancel_check && cancel_check()) {
    PlanResult result;
    result.options = request.options;
    result.cancelled = true;
    result.failure_reason = "cancelled";
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    return result;
  }
  const auto start = toPlannerPoint(request.start);
  const auto goal = toPlannerPoint(request.goal);

  std::vector<global_planner::PointPose> native_path;
  auto map = loadOctomap(request.map_path);
  global_planner::OctoPlanner3D planner;
  planner.setConfig(plannerConfig(request.options));
  planner.setCancelCheck(cancel_check);
  planner.setOctomap(map);
  planner.makePlan(start, goal);
  planner.getPlannerResults(native_path);

  PlanResult result;
  result.options = request.options;
  result.cancelled = cancel_check && cancel_check();
  if (result.cancelled) {
    result.failure_reason = "cancelled";
    result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    return result;
  }
  result.path.reserve(native_path.size());
  for (const auto & point : native_path) {
    result.path.push_back(fromPlannerPoint(point));
  }
  if (!hasAcceptableSameFloorExcursion(request, result.path)) {
    result.failure_reason = "same_floor_z_excursion";
    result.path.clear();
  }
  result.ok = !result.path.empty();
  if (!result.ok && result.failure_reason.empty()) {
    result.failure_reason = "empty_path";
  }
  result.pcd_conversion = pcdConversionEnabled();
  if (!result.path.empty()) {
    result.goal_error_m = distanceToGoal(result.path.back(), request.goal);
    result.goal_xy_error_m = xyDistanceToGoal(result.path.back(), request.goal);
    result.goal_z_error_m = zDistanceToGoal(result.path.back(), request.goal);
    const double terminal_tolerance =
      positiveOrDefault(request.options.terminal_goal_tolerance_m, 0.5);
    const double terminal_xy_tolerance =
      positiveOrDefault(request.options.terminal_goal_xy_tolerance_m, terminal_tolerance);
    const double terminal_z_tolerance =
      positiveOrDefault(request.options.terminal_goal_z_tolerance_m, 0.75);
    result.reached_goal =
      (result.goal_error_m <= terminal_tolerance) ||
      (result.goal_xy_error_m <= terminal_xy_tolerance &&
       result.goal_z_error_m <= terminal_z_tolerance);
  }

  const auto finished = std::chrono::steady_clock::now();
  result.elapsed_ms =
    std::chrono::duration<double, std::milli>(finished - started).count();
  return result;
}

}  // namespace octoplanner3d::runtime
