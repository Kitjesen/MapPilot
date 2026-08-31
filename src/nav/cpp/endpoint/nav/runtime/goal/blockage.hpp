#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "nav_kernel/types.hpp"
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
  double overlay_radius_m{0.75};
  double overlay_half_height_m{1.5};
  std::size_t max_regions{16U};
  std::size_t minimum_obstacle_points{4U};
};

// One synchronous, transport-free view of the endpoint state. The pointed-to
// containers are borrowed only for the duration of observe().
struct ActivePathBlockageObservation {
  double now_s{0.0};
  bool external_active_goal{false};
  // The previous autonomy tick produced a trackable local path. Command safety
  // may still stop one tick, but global replanning must not preempt the local
  // planner while it is already routing around the corridor obstacle.
  bool local_path_viable{false};
  GoalReplanIdentity goal{};
  std::uint64_t frame_epoch{0U};
  nav_kernel::Vec3 robot_position{};
  const std::vector<nav_kernel::Vec3> *active_global_path{nullptr};
  // Borrowed x/y/z/height tuples from the current MotionLayer snapshot.
  const std::vector<float> *live_obstacles_xyzh{nullptr};
  std::uint64_t cloud_generation{0U};
  std::uint64_t traversability_generation{0U};
};

struct ActivePathBlockagePolicySnapshot {
  std::optional<GoalReplanIdentity> goal;
  std::uint64_t frame_epoch{0U};
  std::uint64_t last_cloud_generation{0U};
  std::uint64_t last_traversability_generation{0U};
  std::size_t fresh_blocked_observations{0U};
  std::size_t current_blocker_count{0U};
  double first_blocked_s{-1.0};
  bool trigger_emitted{false};
  std::string reason{"idle"};
};

// Detects only a persistent blockage of the current forward path corridor.
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
    double height{0.0};
    lingtu::nav::plan::GlobalPlanBlockedRegion region{};
  };

  [[nodiscard]] static bool validPoint(const nav_kernel::Vec3 &point);
  [[nodiscard]] bool sameBinding(const GoalReplanIdentity &goal, std::uint64_t frame_epoch) const;
  void bind(const GoalReplanIdentity &goal, std::uint64_t frame_epoch);
  void clearAccumulation(const char *reason);
  void setGenerationBaseline(std::uint64_t cloud_generation,
                             std::uint64_t traversability_generation);
  [[nodiscard]] std::vector<CorridorBlocker>
  corridorBlockers(const ActivePathBlockageObservation &observation) const;

  ActivePathBlockagePolicyConfig config_;
  std::optional<GoalReplanIdentity> goal_;
  std::uint64_t frame_epoch_{0U};
  std::uint64_t last_cloud_generation_{0U};
  std::uint64_t last_traversability_generation_{0U};
  std::size_t fresh_blocked_observations_{0U};
  std::size_t current_blocker_count_{0U};
  double first_blocked_s_{-1.0};
  double last_now_s_{-1.0};
  bool trigger_emitted_{false};
  std::uint64_t next_overlay_revision_{1U};
  std::string reason_{"idle"};
};

}  // namespace lingtu::nav::endpoint
