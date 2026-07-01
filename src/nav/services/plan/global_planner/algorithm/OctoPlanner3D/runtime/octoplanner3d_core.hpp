#pragma once

#include <string>
#include <vector>

namespace octoplanner3d::runtime {

struct Point {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct PlannerOptions {
  double robot_radius{0.25};
  int max_iterations{800000};
  int snap_search_radius_cells{12};
  bool require_ground_support{true};
  bool strict_direct_ground_support{false};
  int ground_support_xy_radius_cells{1};
  int ground_support_depth_cells{1};
  bool enable_preblocked_costmap{true};
  int preblocked_costmap_radius_cells{3};
  double preblocked_costmap_weight{2.5};
  bool lowest_traversable_only{false};
};

struct PlanRequest {
  std::string map_path;
  Point start{};
  Point goal{};
  PlannerOptions options{};
};

struct PlanResult {
  bool ok{false};
  bool reached_goal{false};
  bool octomap_file{true};
  bool pcd_conversion{false};
  double goal_error_m{-1.0};
  double elapsed_ms{0.0};
  PlannerOptions options{};
  std::vector<Point> path{};
};

bool pcdConversionEnabled();
PlanResult runPlan(const PlanRequest & request);

}  // namespace octoplanner3d::runtime
