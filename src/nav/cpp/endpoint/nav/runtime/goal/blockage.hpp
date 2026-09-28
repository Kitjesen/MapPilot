#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "planning/local/planner.hpp"
#include "runtime/goal/trigger.hpp"

namespace lingtu::nav::endpoint {

struct ActivePathBlockagePolicyConfig {
  double persistence_s{1.5};
  std::size_t minimum_fresh_observations{3U};
  double lookahead_m{8.0};
  double corridor_radius_m{0.60};
  double corridor_vertical_tolerance_m{1.0};
  double obstacle_height_min_m{0.10};
  double obstacle_height_max_m{1.20};
  // Measured obstacle voxel size, without robot or obstacle inflation.
  double obstacle_voxel_size_m{0.08};
  std::size_t max_regions{16U};
  std::size_t minimum_obstacle_points{4U};
};

// One synchronous, transport-free view of the endpoint state. The pointed-to
// containers are borrowed only for the duration of observe().
struct ActivePathBlockageObservation {
  double now_s{0.0};
  bool external_active_goal{false};
  // The current autonomy tick produced a trackable local path. A successful
  // local route clears persistent obstacle evidence held by the task owner.
  bool local_path_viable{false};
  GoalReplanIdentity goal{};
  std::uint64_t frame_epoch{0U};
  nav_kernel::Vec3 robot_position{};
  // Fresh measured voxels from the exact local-collision snapshot that
  // rejected a local body pose. The pose itself is not obstacle geometry.
  const nav_kernel::LocalCollisionEvidence *local_collision_evidence{nullptr};
  const std::vector<nav_kernel::Vec3> *active_global_path{nullptr};
  // Borrowed x/y/z/height tuples from the current MotionLayer snapshot.
  const std::vector<float> *live_obstacles_xyzh{nullptr};
  std::uint64_t cloud_generation{0U};
};

struct ActivePathBlockagePolicySnapshot {
  std::optional<GoalReplanIdentity> goal;
  std::uint64_t frame_epoch{0U};
  std::uint64_t last_cloud_generation{0U};
  std::uint64_t last_collision_reset_epoch{0U};
  std::uint64_t last_collision_observation_sequence{0U};
  std::uint64_t last_collision_generation{0U};
  std::size_t fresh_blocked_observations{0U};
  std::size_t current_blocker_count{0U};
  double first_blocked_s{-1.0};
  std::string reason{"idle"};
};

// Detects persistent measured obstacles in the forward path corridor or around
// a failed local trajectory pose. After persistence is established, each fresh
// observation refreshes the candidate overlay; the task owner decides when to use it.
// This policy uses current occupancy only. Velocity prediction and TTC belong
// to NAV-DYN-01 and must not leak into global replan admission.
class ActivePathBlockagePolicy {
 public:
  static constexpr std::size_t kMaximumOverlayRegions = 64U;

  explicit ActivePathBlockagePolicy(ActivePathBlockagePolicyConfig config = {});

  [[nodiscard]] std::optional<GoalReplanTrigger>
  observe(const ActivePathBlockageObservation &observation);
  [[nodiscard]] ActivePathBlockagePolicySnapshot snapshot() const;
  void reset();

 private:
  struct CorridorBlocker {
    double along_path_m{0.0};
    bool near_local_collision{false};
    double collision_distance_m{0.0};
    double height{0.0};
    lingtu::nav::plan::GlobalPlanBlockedRegion region{};
  };

  [[nodiscard]] static bool validPoint(const nav_kernel::Vec3 &point);
  [[nodiscard]] bool sameBinding(const GoalReplanIdentity &goal, std::uint64_t frame_epoch) const;
  void bind(const GoalReplanIdentity &goal, std::uint64_t frame_epoch);
  void clearAccumulation(const char *reason);
  [[nodiscard]] std::vector<CorridorBlocker>
  corridorBlockers(const ActivePathBlockageObservation &observation,
                   bool include_live_obstacles,
                   bool include_local_collision) const;

  ActivePathBlockagePolicyConfig config_;
  std::optional<GoalReplanIdentity> goal_;
  std::uint64_t frame_epoch_{0U};
  std::uint64_t last_cloud_generation_{0U};
  std::uint64_t last_collision_reset_epoch_{0U};
  std::uint64_t last_collision_observation_sequence_{0U};
  std::uint64_t last_collision_generation_{0U};
  std::size_t fresh_blocked_observations_{0U};
  std::size_t current_blocker_count_{0U};
  double first_blocked_s_{-1.0};
  double last_now_s_{-1.0};
  std::uint64_t next_overlay_revision_{1U};
  std::string reason_{"idle"};
};

}  // namespace lingtu::nav::endpoint
