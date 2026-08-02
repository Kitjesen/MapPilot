/**
 * @file      octo_planner/include/global_planner.h
 * @brief     3D A star Planner
 * @author    juchunyu <juchunyu@qq.com>
 * @date      2026-05-31 12:00:01
 * @copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
 *            All rights reserved.
 */
#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "octomap/OcTree.h"

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
    return h1 ^ (h2 << 1) ^ (h3 << 2);
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
  double robot_radius{0.20};
  double body_clearance_below_m{0.0};
  double body_clearance_above_m{0.0};
  int max_iterations{250000};
  int snap_search_radius_cells{8};
  bool require_ground_support{true};
  bool strict_direct_ground_support{true};
  int ground_support_xy_radius_cells{2};
  int ground_support_depth_cells{2};
  double support_height_m{0.0};
  double support_height_tolerance_m{0.0};
  int support_patch_radius_cells{0};
  int support_patch_min_samples{0};
  bool enable_preblocked_costmap{true};
  int preblocked_costmap_radius_cells{3};
  double preblocked_costmap_weight{1.5};
  bool lowest_traversable_only{false};
  double floor_change_penalty{4.0};
  double max_step_height{0.45};
  double max_slope{0.0};
  bool same_floor_preference{true};
  double same_floor_z_tolerance{0.75};
  int obstacle_clearance_radius_cells{4};
  double obstacle_clearance_weight{2.0};
};

class OctoPlanner3D
{
public:
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
  };

  OctoPlanner3D();

  ~OctoPlanner3D();

  void setConfig(const PlannerConfig & config);

  void setCancelCheck(std::function<bool()> cancel_check);

  void setExternalPreblockedRegions(std::vector<ExternalBlockedRegion> regions);

  void setOctomap(std::shared_ptr<octomap::OcTree> map);

  void makePlan(const PointPose start,const PointPose goal);

  void getPlannerResults(std::vector<PointPose>& plannerResults);

  EndpointResolutionInfo endpointResolution() const noexcept;

private:
  enum class TraversabilityFailure
  {
    None,
    OutsideBounds,
    GroundSupport,
    ExternalPreblockedBelow,
    ExternalPreblockedBody,
    OccupiedBody,
  };

  void tryPlan();

  GridIndex worldToGrid(double x, double y, double z) const;

  octomap::point3d gridToWorld(const GridIndex & idx) const;

  bool isInsideMetricBounds(const GridIndex & idx) const;

  bool hasGroundSupport(
    const GridIndex & idx,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;
  std::pair<int, int> supportDepthRange(int support_depth_cells) const;
  bool isSupportPatch(const GridIndex & support) const;
  bool isSupportCell(const GridIndex & support) const;
  bool hasOccupiedNearZ(const GridIndex & index, int z_tolerance_cells) const;
  bool hasFootprintGroundSupport(
    const GridIndex & idx,
    double robot_radius,
    int support_depth_cells) const;

  bool isOccupiedCell(const GridIndex & idx) const;

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;

  void rebuildPreblockedCells();

  void rebuildExternalPreblockedCells();

  void rebuildPreblockedCostmap();

  double getPreblockedCost(const GridIndex & idx) const;

  void rebuildObstacleClearanceCostmap();

  double getObstacleClearanceCost(const GridIndex & idx) const;

  bool isMotionAllowed(const GridIndex & from, const GridIndex & to) const;

  void rebuildDerivedLayers();

  bool isCellTraversable(
    const GridIndex & idx,
    double robot_radius,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells) const;

  bool isCellTraversableDetailed(
    const GridIndex & idx,
    double robot_radius,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells,
    TraversabilityFailure * failure) const;

  bool isPlanningCellTraversableDetailed(
    const GridIndex & idx,
    double robot_radius,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells,
    TraversabilityFailure * failure) const;

  bool findNearestFreeCell(
    const GridIndex & seed,
    double robot_radius,
    int radius_cells,
    bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells,
    int support_depth_cells,
    GridIndex & out) const;

  bool resolvePlanEndpoints(GridIndex & start, GridIndex & goal);

  std::vector<GridIndex> make26Directions() const;

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;

  bool startPlan();

  double euclidean(const GridIndex & a, const GridIndex & b)
  {
    const double dx = static_cast<double>(a.x - b.x);
    const double dy = static_cast<double>(a.y - b.y);
    const double dz = static_cast<double>(a.z - b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  double robot_radius_ = 0.20;
  double body_clearance_below_m_ = 0.0;
  double body_clearance_above_m_ = 0.0;
  int max_iterations_ = 250000;
  int snap_search_radius_cells_ = 8;
  bool require_ground_support_ = true;
  bool strict_direct_ground_support_ = true;
  int ground_support_xy_radius_cells_ = 2;
  int ground_support_depth_cells_ = 2;
  double support_height_m_ = 0.0;
  double support_height_tolerance_m_ = 0.0;
  int support_patch_radius_cells_ = 0;
  int support_patch_min_samples_ = 0;
  bool enable_preblocked_costmap_ = true;
  int preblocked_costmap_radius_cells_ = 3;
  double preblocked_costmap_weight_ = 1.5;
  bool lowest_traversable_only_ = false;
  double floor_change_penalty_ = 4.0;
  double max_step_height_ = 0.45;
  double max_slope_ = 0.0;
  bool same_floor_preference_ = true;
  double same_floor_z_tolerance_ = 0.75;
  int obstacle_clearance_radius_cells_ = 4;
  double obstacle_clearance_weight_ = 2.0;

  bool map_ready_ = false;
  bool has_start_ = false;
  bool has_goal_ = false;
  bool planning_in_progress_ = false;

  PointPose start_point_;
  PointPose goal_point_;

  EndpointResolutionInfo endpoint_resolution_{};
  std::vector<PointPose> planner_results_;

  std::function<bool()> cancel_check_;

  std::shared_ptr<octomap::OcTree> octree_;

  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::vector<ExternalBlockedRegion> external_preblocked_regions_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;
  std::unordered_map<GridIndex, double, GridIndexHash> obstacle_clearance_costmap_;
};

// Compatibility name for the original standalone demo. Product code uses the
// algorithm-specific name so the generic LingTu planner contract stays clear.
using GlobalPlanner = OctoPlanner3D;

}  // namespace global_planner
