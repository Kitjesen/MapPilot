#pragma once

#include <lingtu/settings.hpp>

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::maps { struct NoGoRegion; }

namespace lingtu::nav::plan {

struct GlobalPlanPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  // Optional heading metadata.
  std::optional<double> yaw{};
};

// Request-scoped hard no-go cylinder in the planning frame. It overlays an
// immutable saved map for one planning request and is never persisted.
struct GlobalPlanBlockedRegion {
  GlobalPlanPoint center{};
  double radius_xy_m{0.0};
  double min_z{0.0};
  double max_z{0.0};
};

struct GlobalPlanTemporaryOverlay {
  std::uint64_t revision{0U};
  std::uint64_t frame_epoch{0U};
  std::uint64_t obstacle_generation{0U};
  std::vector<GlobalPlanBlockedRegion> blocked_regions{};

  bool empty() const noexcept { return blocked_regions.empty(); }
};

struct GlobalPlannerOptions {
  double surface_closing_radius_m{::lingtu::settings::global::native_nav::global::surface_closing_radius_m};
  double surface_max_hole_area_m2{::lingtu::settings::global::native_nav::global::surface_max_hole_area_m2};
  double surface_max_slope{::lingtu::settings::global::native_nav::global::surface_max_slope};
  double search_resolution_m{::lingtu::settings::global::native_nav::global::search_resolution_m};
  double robot_radius{0.25};
  double body_clearance_below_m{0.0};
  double body_clearance_above_m{0.0};
  int max_iterations{::lingtu::settings::global::native_nav::global::max_iterations};
  int snap_search_radius_cells{::lingtu::settings::global::native_nav::global::snap_search_radius_cells};
  bool ground_support{::lingtu::settings::global::native_nav::global::ground_support};
  // The saved-map path uses OctoPlanner3D's neighbouring support fallback by
  // default. A single missing floor voxel must not block an otherwise
  // supported stance; strict direct support remains available for diagnostics.
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
  double terminal_goal_tolerance_m{::lingtu::settings::global::native_nav::global::terminal_tolerance_m};
  double terminal_goal_xy_tolerance_m{::lingtu::settings::global::native_nav::global::terminal_xy_tolerance_m};
  double terminal_goal_z_tolerance_m{::lingtu::settings::global::native_nav::global::terminal_z_tolerance_m};
};

// Stable identity of the saved map content used by a planning job.
struct MapIdentity {
  std::string map_id;
  std::int64_t content_epoch{0};
  std::string frame_id;

  bool valid() const {
    return !map_id.empty() && content_epoch > 0 && !frame_id.empty();
  }
};

inline bool sameMapIdentity(const MapIdentity& lhs, const MapIdentity& rhs) {
  return lhs.map_id == rhs.map_id && lhs.content_epoch == rhs.content_epoch &&
      lhs.frame_id == rhs.frame_id;
}

struct GlobalPlanRequest {
  GlobalPlanPoint start{};
  GlobalPlanPoint goal{};
  GlobalPlannerOptions options{};
  MapIdentity map_identity{};
  std::uint64_t map_generation{0U};
  GlobalPlanTemporaryOverlay temporary_overlay{};
  std::shared_ptr<const std::vector<lingtu::maps::NoGoRegion>> no_go_regions{};
};

struct GlobalPlanResult {
  bool ok{false};
  bool reached_goal{false};
  bool cancelled{false};
  std::string failure_reason{};
  double goal_error_m{-1.0};
  double goal_xy_error_m{-1.0};
  double goal_z_error_m{-1.0};
  double elapsed_ms{0.0};
  GlobalPlannerOptions options{};
  MapIdentity map_identity{};
  std::uint64_t map_generation{0U};
  std::uint64_t overlay_revision{0U};
  std::uint64_t overlay_frame_epoch{0U};
  std::uint64_t overlay_obstacle_generation{0U};
  std::vector<GlobalPlanPoint> path{};
};

using GlobalPlanCancelCheck = std::function<bool()>;
using GlobalPlannerFunction = std::function<GlobalPlanResult(
    const GlobalPlanRequest&,
    const GlobalPlanCancelCheck&)>;

}  // namespace lingtu::nav::plan
