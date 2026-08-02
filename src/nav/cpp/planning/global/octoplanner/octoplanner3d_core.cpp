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
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "global_planner.h"

#if defined(OCTOPLANNER3D_ENABLE_PCD)
#include "pcd2octomap_converter.h"
#endif

namespace octoplanner3d::runtime {
namespace {

constexpr std::size_t kMaxTemporaryBlockedRegions = 64U;
constexpr double kMaxTemporaryOverlayRasterCells = 500000.0;

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

void copyOverlayIdentity(const PlanRequest & request, PlanResult & result)
{
  result.overlay_revision = request.temporary_overlay.revision;
  result.overlay_frame_epoch = request.temporary_overlay.frame_epoch;
  result.overlay_obstacle_generation = request.temporary_overlay.obstacle_generation;
  result.overlay_traversability_generation =
    request.temporary_overlay.traversability_generation;
}

std::string validateTemporaryOverlay(const PlanRequest & request)
{
  const auto & overlay = request.temporary_overlay;
  if (overlay.empty()) {
    return {};
  }
  if (overlay.revision == 0U) {
    return "temporary_overlay_revision_missing";
  }
  if (overlay.frame_epoch == 0U) {
    return "temporary_overlay_frame_epoch_missing";
  }
  if (overlay.obstacle_generation == 0U) {
    return "temporary_overlay_obstacle_generation_missing";
  }
  if (overlay.traversability_generation == 0U) {
    return "temporary_overlay_traversability_generation_missing";
  }
  if (overlay.blocked_regions.size() > kMaxTemporaryBlockedRegions) {
    return "temporary_overlay_region_limit_exceeded";
  }
  for (const auto & region : overlay.blocked_regions) {
    const double min_x = region.center.x - region.radius_xy_m;
    const double max_x = region.center.x + region.radius_xy_m;
    const double min_y = region.center.y - region.radius_xy_m;
    const double max_y = region.center.y + region.radius_xy_m;
    if (!std::isfinite(region.center.x) || !std::isfinite(region.center.y) ||
        !std::isfinite(region.center.z) || !std::isfinite(region.radius_xy_m) ||
        !std::isfinite(region.min_z) || !std::isfinite(region.max_z) ||
        !std::isfinite(min_x) || !std::isfinite(max_x) ||
        !std::isfinite(min_y) || !std::isfinite(max_y) ||
        region.radius_xy_m <= 0.0 || region.min_z > region.max_z) {
      return "temporary_overlay_region_invalid";
    }
  }
  return {};
}

std::vector<global_planner::ExternalBlockedRegion> toPlannerBlockedRegions(
  const lingtu::nav::plan::GlobalPlanTemporaryOverlay & overlay)
{
  std::vector<global_planner::ExternalBlockedRegion> regions;
  regions.reserve(overlay.blocked_regions.size());
  for (const auto & region : overlay.blocked_regions) {
    global_planner::ExternalBlockedRegion converted;
    converted.center = toPlannerPoint(region.center);
    converted.radius_xy_m = region.radius_xy_m;
    converted.min_z = region.min_z;
    converted.max_z = region.max_z;
    regions.push_back(converted);
  }
  return regions;
}

std::string validateTemporaryOverlayForMap(
  const std::shared_ptr<octomap::OcTree> & map,
  const PlanRequest & request)
{
  if (request.temporary_overlay.empty()) {
    return {};
  }

  double map_min_x = 0.0;
  double map_min_y = 0.0;
  double map_min_z = 0.0;
  double map_max_x = 0.0;
  double map_max_y = 0.0;
  double map_max_z = 0.0;
  map->getMetricMin(map_min_x, map_min_y, map_min_z);
  map->getMetricMax(map_max_x, map_max_y, map_max_z);
  const double resolution = map->getResolution();
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return "temporary_overlay_map_resolution_invalid";
  }

  double total_cells = 0.0;
  for (const auto & region : request.temporary_overlay.blocked_regions) {
    const double min_x = std::max(map_min_x, region.center.x - region.radius_xy_m);
    const double max_x = std::min(map_max_x, region.center.x + region.radius_xy_m);
    const double min_y = std::max(map_min_y, region.center.y - region.radius_xy_m);
    const double max_y = std::min(map_max_y, region.center.y + region.radius_xy_m);
    const double min_z = std::max(map_min_z, region.min_z);
    const double max_z = std::min(map_max_z, region.max_z);
    if (min_x > max_x || min_y > max_y || min_z > max_z) {
      continue;
    }

    const double x_cells = std::ceil((max_x - min_x) / resolution) + 1.0;
    const double y_cells = std::ceil((max_y - min_y) / resolution) + 1.0;
    const double z_cells = std::ceil((max_z - min_z) / resolution) + 1.0;
    const double region_cells = x_cells * y_cells * z_cells;
    if (!std::isfinite(region_cells) || region_cells <= 0.0 ||
        region_cells > kMaxTemporaryOverlayRasterCells - total_cells) {
      return "temporary_overlay_raster_budget_exceeded";
    }
    total_cells += region_cells;
  }
  return {};
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

namespace {

PlanResult cancelledResult(
  const PlanRequest & request,
  const std::chrono::steady_clock::time_point & started)
{
  PlanResult result;
  result.options = request.options;
  copyOverlayIdentity(request, result);
  result.cancelled = true;
  result.failure_reason = "cancelled";
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return result;
}

PlanResult invalidTemporaryOverlayResult(
  const PlanRequest & request,
  std::string failure_reason,
  const std::chrono::steady_clock::time_point & started)
{
  PlanResult result;
  result.options = request.options;
  result.failure_reason = std::move(failure_reason);
  copyOverlayIdentity(request, result);
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return result;
}

std::string endpointResolutionFailureReason(
  const global_planner::OctoPlanner3D::EndpointResolutionInfo & resolution)
{
  using Failure = global_planner::OctoPlanner3D::EndpointResolutionInfo::Failure;
  switch (resolution.failure) {
    case Failure::StartSnapExhausted:
      return resolution.start_raw_outside_bounds
        ? "start_outside_static_map"
        : "start_snap_exhausted";
    case Failure::GoalSnapExhausted:
      return resolution.goal_raw_outside_bounds
        ? "goal_outside_static_map"
        : "goal_snap_exhausted";
    case Failure::None:
      return {};
  }
  return {};
}

PlanResult runWithLoadedMap(
  const std::shared_ptr<octomap::OcTree> & map,
  const PlanRequest & request,
  const CancelCheck & cancel_check,
  const std::chrono::steady_clock::time_point & started)
{
  if (cancel_check && cancel_check()) {
    return cancelledResult(request, started);
  }
  const std::string overlay_map_error = validateTemporaryOverlayForMap(map, request);
  if (!overlay_map_error.empty()) {
    return invalidTemporaryOverlayResult(request, overlay_map_error, started);
  }
  const auto start = toPlannerPoint(request.start);
  const auto goal = toPlannerPoint(request.goal);

  std::vector<global_planner::PointPose> native_path;
  global_planner::OctoPlanner3D planner;
  planner.setConfig(plannerConfig(request.options));
  planner.setCancelCheck(cancel_check);
  planner.setExternalPreblockedRegions(toPlannerBlockedRegions(request.temporary_overlay));
  planner.setOctomap(map);
  planner.makePlan(start, goal);
  planner.getPlannerResults(native_path);
  const auto endpoint_resolution = planner.endpointResolution();

  PlanResult result;
  result.options = request.options;
  copyOverlayIdentity(request, result);
  result.cancelled = cancel_check && cancel_check();
  if (result.cancelled) {
    return cancelledResult(request, started);
  }
  result.path.reserve(native_path.size());
  for (const auto & point : native_path) {
    result.path.push_back(fromPlannerPoint(point));
  }
  if (native_path.empty()) {
    result.failure_reason = endpointResolutionFailureReason(endpoint_resolution);
  }
  if (!hasAcceptableSameFloorExcursion(request, result.path)) {
    result.failure_reason = "same_floor_z_excursion";
    result.path.clear();
  }
  result.ok = !result.path.empty();
  if (!result.ok && result.failure_reason.empty()) {
    result.failure_reason = "empty_path";
  }
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

}  // namespace

PlanResult runPlan(
  const std::filesystem::path & map_path,
  const PlanRequest & request,
  const CancelCheck & cancel_check)
{
  if (map_path.empty()) {
    throw std::runtime_error("map_path must not be empty");
  }
  const auto started = std::chrono::steady_clock::now();
  if (cancel_check && cancel_check()) {
    return cancelledResult(request, started);
  }
  const std::string overlay_error = validateTemporaryOverlay(request);
  if (!overlay_error.empty()) {
    return invalidTemporaryOverlayResult(request, overlay_error, started);
  }
  return runWithLoadedMap(
    loadOctomap(map_path.string()), request, cancel_check, started);
}

struct PlannerSession::Impl {
  mutable std::mutex mutex;
  lingtu::nav::plan::MapIdentity map_identity;
  std::shared_ptr<octomap::OcTree> map;
  std::size_t map_load_count{0};
};

PlannerSession::PlannerSession()
: impl_(std::make_unique<Impl>()) {}

PlannerSession::~PlannerSession() = default;

PlanResult PlannerSession::run(
  const std::filesystem::path & map_path,
  const lingtu::nav::plan::MapIdentity & map_identity,
  const PlanRequest & request,
  const CancelCheck & cancel_check)
{
  if (map_path.empty()) {
    throw std::runtime_error("map_path must not be empty");
  }
  if (!map_identity.valid()) {
    throw std::runtime_error("map_identity must be complete");
  }

  const auto started = std::chrono::steady_clock::now();
  if (cancel_check && cancel_check()) {
    return cancelledResult(request, started);
  }
  const std::string overlay_error = validateTemporaryOverlay(request);
  if (!overlay_error.empty()) {
    auto result = invalidTemporaryOverlayResult(request, overlay_error, started);
    result.map_identity = map_identity;
    return result;
  }

  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->map || !lingtu::nav::plan::sameMapIdentity(
          impl_->map_identity, map_identity)) {
    impl_->map = loadOctomap(map_path.string());
    impl_->map_identity = map_identity;
    ++impl_->map_load_count;
  }
  auto result = runWithLoadedMap(impl_->map, request, cancel_check, started);
  result.map_identity = map_identity;
  return result;
}

std::size_t PlannerSession::mapLoadCount() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->map_load_count;
}

}  // namespace octoplanner3d::runtime
