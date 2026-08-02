#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace lingtu::nav::plan {

struct GlobalPlanPoint {
  double x{0.0};
  double y{0.0};
  double z{0.0};
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
  std::uint64_t traversability_generation{0U};
  std::vector<GlobalPlanBlockedRegion> blocked_regions{};

  bool empty() const noexcept { return blocked_regions.empty(); }
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

// Stable identity of the immutable map artifact used by a planning job.
// The planner contract deliberately carries no filesystem path or backend map type.
struct MapIdentity {
  std::string map_id;
  std::int64_t version{0};
  std::string artifact_sha256;
  std::string frame_id;

  bool valid() const {
    return !map_id.empty() && version > 0 && !artifact_sha256.empty() &&
        !frame_id.empty();
  }
};

inline bool sameMapIdentity(const MapIdentity& lhs, const MapIdentity& rhs) {
  return lhs.map_id == rhs.map_id && lhs.version == rhs.version &&
      lhs.artifact_sha256 == rhs.artifact_sha256 &&
      lhs.frame_id == rhs.frame_id;
}

struct GlobalPlanRequest {
  GlobalPlanPoint start{};
  GlobalPlanPoint goal{};
  GlobalPlannerOptions options{};
  MapIdentity map_identity{};
  std::uint64_t map_generation{0U};
  GlobalPlanTemporaryOverlay temporary_overlay{};
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
  std::uint64_t overlay_traversability_generation{0U};
  std::vector<GlobalPlanPoint> path{};
};

using GlobalPlanCancelCheck = std::function<bool()>;
using GlobalPlannerFunction = std::function<GlobalPlanResult(
    const GlobalPlanRequest&,
    const GlobalPlanCancelCheck&)>;

}  // namespace lingtu::nav::plan
