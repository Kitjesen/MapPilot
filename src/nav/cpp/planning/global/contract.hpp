#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "collision/body.hpp"
#include "collision/terrain.hpp"

namespace lingtu::nav::collision { class Terrain; }

namespace lingtu::nav::plan {

struct GlobalPlanPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  // Optional input/terminal heading; global XYZ route points leave it unset.
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
  double surface_closing_radius_m{0.20};
  double search_resolution_m{0.10};
  double robot_radius{0.25};
  double body_clearance_below_m{0.0};
  double body_clearance_above_m{0.0};
  int max_iterations{500000};
  int snap_search_radius_cells{12};
  bool ground_support{true};
  // The saved-map path uses OctoPlanner3D's neighbouring support fallback by
  // default. A single missing floor voxel must not block an otherwise
  // supported stance; strict direct support remains available for diagnostics.
  bool direct_support_only{false};
  int support_radius_cells{1};
  int support_depth_cells{2};
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
  int obstacle_clearance_radius_cells{4};
  double obstacle_clearance_weight{2.0};
  double terminal_goal_tolerance_m{0.5};
  double terminal_goal_xy_tolerance_m{0.6};
  double terminal_goal_z_tolerance_m{0.75};
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
  // Immutable model forwarded to local execution. Global XYZ search does not
  // use this snapshot to search headings or reject a candidate route.
  std::shared_ptr<const lingtu::nav::collision::Body> body{};
  // Request-scoped measured free cells. The planner separately exempts saved
  // occupancy in the robot's initial envelope without erasing ground support.
  std::function<bool(const lingtu::nav::collision::Aabb &)> observed_free{};
  // Raw live hits override saved free space and live clearing. The last argument
  // preserves the legacy support-rooted upper hemisphere instead of a cylinder.
  std::function<bool(const collision::Vec3 &, double, double, double, bool)> observed_occupied{};
  // Original 3-D hits belonging to this map, used only to resolve measured
  // terrain geometry below the voxel scale. An occupancy preview is not a source.
  std::shared_ptr<const lingtu::nav::collision::Terrain> terrain{};
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
  collision::SupportQuery support_surface{};
};

using GlobalPlanCancelCheck = std::function<bool()>;
using GlobalPlannerFunction = std::function<GlobalPlanResult(
    const GlobalPlanRequest&,
    const GlobalPlanCancelCheck&)>;

}  // namespace lingtu::nav::plan
