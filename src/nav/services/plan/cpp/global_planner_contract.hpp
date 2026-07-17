#pragma once

#include <functional>
#include <string>
#include <vector>

namespace lingtu::nav::plan {

struct GlobalPlanPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct GlobalPlannerOptions {
  double robot_radius{0.25};
  double body_clearance_below_m{0.0};
  double body_clearance_above_m{0.0};
  int max_iterations{500000};
  int snap_search_radius_cells{12};
  bool require_ground_support{true};
  bool strict_direct_ground_support{false};
  int ground_support_xy_radius_cells{2};
  int ground_support_depth_cells{1};
  double support_height_m{0.0};
  double support_height_tolerance_m{0.0};
  int support_patch_radius_cells{0};
  int support_patch_min_samples{0};
  bool enable_preblocked_costmap{true};
  int preblocked_costmap_radius_cells{3};
  double preblocked_costmap_weight{2.5};
  bool lowest_traversable_only{false};
  double floor_change_penalty{4.0};
  double max_step_height{0.45};
  double max_slope{0.0};
  bool same_floor_preference{true};
  double same_floor_z_tolerance{0.75};
  double max_same_floor_z_excursion{2.0};
  int obstacle_clearance_radius_cells{4};
  double obstacle_clearance_weight{2.0};
  double terminal_goal_tolerance_m{0.5};
  double terminal_goal_xy_tolerance_m{0.6};
  double terminal_goal_z_tolerance_m{0.75};
};

struct GlobalPlanRequest {
  std::string map_path;
  GlobalPlanPoint start{};
  GlobalPlanPoint goal{};
  GlobalPlannerOptions options{};
};

struct GlobalPlanResult {
  bool ok{false};
  bool reached_goal{false};
  bool cancelled{false};
  std::string failure_reason{};
  bool octomap_file{true};
  bool pcd_conversion{false};
  double goal_error_m{-1.0};
  double goal_xy_error_m{-1.0};
  double goal_z_error_m{-1.0};
  double elapsed_ms{0.0};
  GlobalPlannerOptions options{};
  std::vector<GlobalPlanPoint> path{};
};

using GlobalPlanCancelCheck = std::function<bool()>;
using GlobalPlannerFunction = std::function<GlobalPlanResult(
    const GlobalPlanRequest&,
    const GlobalPlanCancelCheck&)>;

}  // namespace lingtu::nav::plan
