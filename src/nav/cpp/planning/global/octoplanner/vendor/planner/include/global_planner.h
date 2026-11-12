/**
 * @file      octo_planner/include/global_planner.h
 * @brief     3D A star Planner
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 12:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 */
#pragma once

#include <lingtu/settings.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "octomap/OcTree.h"
#include "lingtu/maps/no_go.hpp"
#include "lingtu/maps/layers/surface.hpp"

namespace global_planner
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    // Nearby integer coordinates must not collapse into the same few buckets.
    std::size_t seed = h1;
    seed ^= h2 + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    seed ^= h3 + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    // Equal-f nodes are equally optimal. Prefer the node with the larger
    // travelled cost, which has the smaller remaining heuristic distance.
    // This keeps A* optimal while avoiding broad plateaus near the goal.
    return a.g < b.g;
  }
};

struct PointPose
{
    double x;
    double y;
    double z;
    std::optional<double> yaw{};
};

struct ExternalBlockedRegion
{
  PointPose center{};
  double radius_xy_m{0.0};
  double min_z{0.0};
  double max_z{0.0};
};

struct PlannerConfig
{
  double surface_closing_radius_m{::lingtu::settings::global::native_nav::global::surface_closing_radius_m};
  double surface_max_hole_area_m2{::lingtu::settings::global::native_nav::global::surface_max_hole_area_m2};
  double surface_max_slope{::lingtu::settings::global::native_nav::global::surface_max_slope};
  double search_resolution_m{::lingtu::settings::global::native_nav::global::search_resolution_m};
  double robot_radius{0.20};
  double body_clearance_below_m{0.0};
  double body_clearance_above_m{0.0};
  int max_iterations{::lingtu::settings::global::native_nav::global::max_iterations};
  int snap_search_radius_cells{::lingtu::settings::global::native_nav::global::snap_search_radius_cells};
  double terminal_goal_tolerance_m{::lingtu::settings::global::native_nav::global::terminal_tolerance_m};
  double terminal_goal_xy_tolerance_m{::lingtu::settings::global::native_nav::global::terminal_xy_tolerance_m};
  double terminal_goal_z_tolerance_m{::lingtu::settings::global::native_nav::global::terminal_z_tolerance_m};
  bool ground_support{::lingtu::settings::global::native_nav::global::ground_support};
  // Normal navigation consumes prepared inferred support; direct-only is diagnostic.
  bool direct_support_only{::lingtu::settings::global::native_nav::global::direct_support_only};
  int support_depth_cells{::lingtu::settings::global::native_nav::global::support_depth_cells};
  double support_height_m{::lingtu::settings::global::standalone::native_nav::global::support_height_m};
  double support_height_tolerance_m{::lingtu::settings::global::standalone::native_nav::global::support_height_tolerance_m};
  int support_patch_radius_cells{::lingtu::settings::global::native_nav::global::support_patch_radius_cells};
  int support_patch_min_samples{::lingtu::settings::global::native_nav::global::support_patch_min_samples};
  bool enable_preblocked_costmap{::lingtu::settings::global::native_nav::global::preblocked};
  int preblocked_costmap_radius_cells{::lingtu::settings::global::native_nav::global::preblocked_radius_cells};
  double preblocked_costmap_weight{::lingtu::settings::global::native_nav::global::preblocked_weight};
  bool lowest_traversable_only{::lingtu::settings::global::native_nav::global::lowest_traversable_only};
  double floor_change_penalty{::lingtu::settings::global::native_nav::global::floor_change_penalty};
  double max_step_height{::lingtu::settings::global::native_nav::global::max_step_height_m};
  double max_slope{::lingtu::settings::global::standalone::native_nav::global::max_slope};
  bool same_floor_preference{::lingtu::settings::global::native_nav::global::same_floor_preference};
  double same_floor_z_tolerance{::lingtu::settings::global::native_nav::global::same_floor_z_tolerance_m};
  int obstacle_clearance_radius_cells{::lingtu::settings::global::native_nav::global::obstacle_clearance_radius_cells};
  double obstacle_clearance_weight{::lingtu::settings::global::native_nav::global::obstacle_clearance_weight};

  std::array<double,23> preparation() const {
    return {surface_closing_radius_m,surface_max_hole_area_m2,surface_max_slope,
      search_resolution_m,robot_radius,body_clearance_below_m,body_clearance_above_m,
      double(ground_support),double(direct_support_only),double(support_depth_cells),
      support_height_m,support_height_tolerance_m,double(support_patch_radius_cells),
      double(support_patch_min_samples),double(enable_preblocked_costmap),
      double(preblocked_costmap_radius_cells),preblocked_costmap_weight,
      double(lowest_traversable_only),floor_change_penalty,max_step_height,max_slope,
      double(obstacle_clearance_radius_cells),obstacle_clearance_weight};
  }
};

using SurfaceIndex = lingtu::maps::layers::SurfaceIndex;
class SurfaceGraph;

class OctoPlanner3D
{
public:
  enum class TraversabilityFailure
  {
    None,
    OutsideBounds,
    UnknownSpace,
    GroundSupport,
    ExternalPreblockedBelow,
    ExternalPreblockedBody,
    OccupiedBody,
  };

  struct EndpointResolutionInfo
  {
    enum class Failure
    {
      None,
      StartSnapExhausted,
      GoalSnapExhausted,
    };

    Failure failure{Failure::None};
    bool start_raw_outside_bounds{false};
    bool goal_raw_outside_bounds{false};
    bool start_snapped{false};
    bool goal_snapped{false};
    TraversabilityFailure start_failure{TraversabilityFailure::None};
  };

  struct SearchInfo
  {
    enum class Outcome { NotStarted, Found, Exhausted, IterationLimit, Cancelled };
    Outcome outcome{Outcome::NotStarted};
    int iterations{0};
    bool used_surface_graph{false};
  };

  OctoPlanner3D();

  ~OctoPlanner3D();

  void setConfig(const PlannerConfig & config);
  void setQuery(const PlannerConfig & config);

  void setCancelCheck(std::function<bool()> cancel_check);
  void setStartExemption(std::function<bool(const PointPose &, double)> check) {
    start_exemption_ = std::move(check);
  }
  const octomap::OcTree &map() const { return *octree_; }

  void setExternalPreblockedRegions(std::vector<ExternalBlockedRegion> regions);
  void setNoGoRegions(std::shared_ptr<const std::vector<lingtu::maps::NoGoRegion>> regions);

  void setOctomap(std::shared_ptr<octomap::OcTree> map,
                 std::shared_ptr<const SurfaceIndex> surface = {},
                 std::shared_ptr<SurfaceGraph> graph = {});
  void writePrepared(lingtu::maps::BinaryWriter& out) const;

  void makePlan(const PointPose start,const PointPose goal);

  void getPlannerResults(std::vector<PointPose>& plannerResults);

  EndpointResolutionInfo endpointResolution() const noexcept;
  SearchInfo searchInfo() const noexcept;
  std::shared_ptr<const SurfaceIndex> supportSurface() const { return surface_index_; }

  struct PlanningSlice {
    bool available{false};
    std::string reason;
    double resolution{0.0};
    int rows{0};
    int cols{0};
    PointPose origin{};
    double reference_z{0.0};
    // X-fastest: 0 = unsupported/unclassified, 1 = allowed, 2 = body blocked.
    std::vector<std::uint8_t> cells;
  };
  PlanningSlice projectPlanningSlice(double reference_z, std::size_t max_cells = 250000) const;

  struct PoseQueryInfo {
    bool ready{false};
    double xy_resolution{0.0};
    double z_resolution{0.0};
    PointPose minimum{};
    PointPose maximum{};
    int max_iterations{0};
    int snap_search_radius_cells{0};
    double goal_tolerance_m{0.0};
    double goal_xy_tolerance_m{0.0};
    double goal_z_tolerance_m{0.0};
  };

  // Pose search supplies its own body shape, while keeping these map/support
  // and metric-motion rules identical to the XYZ planner.
  PoseQueryInfo preparePoseQueries(const PointPose & anchor);
  bool poseSupportFree(const PointPose & point) const;
  bool supportConnectionFree(const PointPose & from, const PointPose & to) const;
  // Metric translation limits only; callers establish bounds, support and collision.
  bool metricConnectionWithinLimits(const PointPose & from, const PointPose & to) const;

private:
  friend struct OctoPlannerGridQueryTest;

  enum class CellState : std::uint8_t
  {
    Unqueried,
    Unknown,
    Free,
    Occupied,
    OccupiedLeafCenter,
  };

  void tryPlan();

  GridIndex worldToGrid(double x, double y, double z) const;

  octomap::point3d gridToWorld(const GridIndex & idx) const;
  double planningXYResolution() const;
  double planningZResolution() const;
  GridIndex worldToPlanningGrid(double x, double y, double z) const;
  void anchorPlanningLattice(const PointPose & point, double resolution = 0.0);
  octomap::point3d planningPoint(const GridIndex & idx) const;
  TraversabilityFailure queryWorld(const octomap::point3d &point, double radius,
                                   bool require_support, bool check_body = true) const;
  bool sweptRouteFree(
    const octomap::point3d &from, const octomap::point3d &to,
    TraversabilityFailure *failure = nullptr,
    octomap::point3d *failed_point = nullptr, bool check_body = true,
    bool endpoints_validated = false, bool require_support = true) const;
  PointPose planning_offset_{};
  double planning_resolution_m_{0.0};

  bool isInsideMetricBounds(const GridIndex & idx) const;

  bool hasGroundSupport(
    const GridIndex & idx,
    bool direct_support_only,
    int support_depth_cells,
    double centre_offset = 0.0) const;
  std::pair<int, int> supportDepthRange(int support_depth_cells, double centre_offset = 0.0) const;
  bool isSupportPatch(const GridIndex & support) const;
  bool isSupportCell(const GridIndex & support) const;
  bool hasOccupiedNearZ(const GridIndex & index, int z_tolerance_cells) const;
  bool hasFootprintGroundSupport(
    const GridIndex & idx,
    double robot_radius,
    int support_depth_cells,
    bool * uneven = nullptr) const;
  bool isTraversableTerrainBelow(const GridIndex & point, const GridIndex & body) const;

  bool isOccupiedCell(const GridIndex & idx) const;
  bool isOccupiedLeafCell(const GridIndex & idx) const;
  CellState cellState(const GridIndex & idx) const;

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;

  void clearMapQueryCaches();
  void rebuildQueryOffsets();
  bool isPreblockedCell(const GridIndex & idx) const;

  void rebuildExternalPreblockedCells();
  bool noGoIntersects(const octomap::point3d &from, const octomap::point3d &to,
                      double radius) const;

  double getPreblockedCost(const GridIndex & idx) const;

  double getObstacleClearanceCost(const GridIndex & idx) const;

  bool isStepWithinLimits(const GridIndex & step) const;
  bool isStepWithinLimits(double dx, double dy, double dz) const;
  bool isMotionAllowed(const GridIndex & from, const GridIndex & to) const;
  bool isStartConnectionAllowed(
    const octomap::point3d & from, const octomap::point3d & to,
    TraversabilityFailure *failure = nullptr) const;

  void rebuildSurfaceIndex();
  void rebuildSurfaceGraph();
  void rebuildDerivedLayers();

  bool isCellTraversable(
    const GridIndex & idx,
    double robot_radius,
    bool ground_support,
    bool direct_support_only,
    int support_depth_cells) const;

  bool isCellTraversableDetailed(
    const GridIndex & idx,
    double robot_radius,
    bool ground_support,
    bool direct_support_only,
    int support_depth_cells,
    TraversabilityFailure * failure) const;

  bool isPlanningCellTraversableDetailed(
    const GridIndex & idx,
    double robot_radius,
    bool ground_support,
    bool direct_support_only,
    int support_depth_cells,
    TraversabilityFailure * failure) const;

  bool findNearestFreeCell(
    const GridIndex & seed,
    double robot_radius,
    int radius_cells,
    bool ground_support,
    bool direct_support_only,
    int support_depth_cells,
    GridIndex & out,
    const PointPose * reference = nullptr,
    const PointPose * connection_start = nullptr,
    TraversabilityFailure *start_failure = nullptr) const;

  bool resolveGoal();
  bool resolvePlanEndpoints(GridIndex & start, GridIndex & goal);

  std::vector<GridIndex> make26Directions() const;

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;

  bool startPlan();
  bool searchGrid(int max_iterations);
  bool routeConnectionFree(const PointPose &from, const PointPose &to,
                           bool endpoints_validated = false) const;

  double euclidean(const GridIndex & a, const GridIndex & b)
  {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z) *
      planningZResolution() / planningXYResolution();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double surface_closing_radius_m_ = PlannerConfig{}.surface_closing_radius_m;
  double search_resolution_m_ = PlannerConfig{}.search_resolution_m;
  double robot_radius_ = 0.20;
  double body_clearance_below_m_ = 0.0;
  double body_clearance_above_m_ = 0.0;
  int max_iterations_ = PlannerConfig{}.max_iterations;
  int snap_search_radius_cells_ = PlannerConfig{}.snap_search_radius_cells;
  double terminal_goal_tolerance_m_ = PlannerConfig{}.terminal_goal_tolerance_m;
  double terminal_goal_xy_tolerance_m_ = PlannerConfig{}.terminal_goal_xy_tolerance_m;
  double terminal_goal_z_tolerance_m_ = PlannerConfig{}.terminal_goal_z_tolerance_m;
  bool ground_support_ = PlannerConfig{}.ground_support;
  bool direct_support_only_ = PlannerConfig{}.direct_support_only;
  int support_depth_cells_ = PlannerConfig{}.support_depth_cells;
  double support_height_m_ = PlannerConfig{}.support_height_m;
  double support_height_tolerance_m_ = PlannerConfig{}.support_height_tolerance_m;
  int support_patch_radius_cells_ = PlannerConfig{}.support_patch_radius_cells;
  int support_patch_min_samples_ = PlannerConfig{}.support_patch_min_samples;
  bool enable_preblocked_costmap_ = PlannerConfig{}.enable_preblocked_costmap;
  int preblocked_costmap_radius_cells_ = PlannerConfig{}.preblocked_costmap_radius_cells;
  double preblocked_costmap_weight_ = PlannerConfig{}.preblocked_costmap_weight;
  bool lowest_traversable_only_ = PlannerConfig{}.lowest_traversable_only;
  double floor_change_penalty_ = PlannerConfig{}.floor_change_penalty;
  double max_step_height_ = PlannerConfig{}.max_step_height;
  double max_slope_ = PlannerConfig{}.max_slope;
  bool same_floor_preference_ = PlannerConfig{}.same_floor_preference;
  double same_floor_z_tolerance_ = PlannerConfig{}.same_floor_z_tolerance;
  int obstacle_clearance_radius_cells_ = PlannerConfig{}.obstacle_clearance_radius_cells;
  double obstacle_clearance_weight_ = PlannerConfig{}.obstacle_clearance_weight;

  bool map_ready_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
  bool planning_in_progress_ = false;

  PointPose start_point_;
  PointPose goal_point_;

  EndpointResolutionInfo endpoint_resolution_{};
  SearchInfo search_info_{};
  std::vector<PointPose> planner_results_;

  std::function<bool()> cancel_check_;
  std::function<bool(const PointPose &, double)> start_exemption_;

  std::shared_ptr<octomap::OcTree> octree_;
  std::shared_ptr<const SurfaceIndex> surface_index_;
  std::shared_ptr<SurfaceGraph> surface_graph_;
  PlannerConfig graph_config_;

  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::vector<GridIndex> body_offsets_;
  mutable std::vector<GridIndex> snap_offsets_;
  mutable std::unordered_map<GridIndex, TraversabilityFailure, GridIndexHash> body_cache_;
  // Allocate only queried 16^3 regions. Adjacent footprint samples share one
  // block lookup instead of allocating/hashing an entry for every voxel.
  using OccupancyBlock = std::array<CellState, 16 * 16 * 16>;
  mutable std::unordered_map<GridIndex, OccupancyBlock, GridIndexHash> occupied_blocks_;
  mutable GridIndex last_occupied_block_index_{};
  mutable OccupancyBlock * last_occupied_block_{nullptr};
  mutable std::unordered_map<GridIndex, bool, GridIndexHash> preblocked_cache_;
  std::vector<ExternalBlockedRegion> external_preblocked_regions_;
  std::shared_ptr<const std::vector<lingtu::maps::NoGoRegion>> no_go_regions_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  mutable std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;
  mutable std::unordered_map<GridIndex, double, GridIndexHash> obstacle_clearance_costmap_;
};

// Compatibility name for the original standalone demo. Product code uses the
// algorithm-specific name so the generic LingTu planner contract stays clear.
using GlobalPlanner = OctoPlanner3D;

}  // namespace global_planner
