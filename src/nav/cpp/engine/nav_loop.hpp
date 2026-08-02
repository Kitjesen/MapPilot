#pragma once

#include "local_planner.hpp"
#include "nav_kernel/path_follower_core.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::nav::plan {

struct NavLoopConfig {
  std::string path_library_dir;
  double corridor_lookahead_m{3.0};
  double waypoint_reached_m{0.6};
  double goal_reached_m{0.35};
  double goal_yaw_tolerance_rad{0.08726646259971647};
  double goal_yaw_kp{1.5};
  double goal_yaw_max_rate{0.6};
  double max_speed{0.4};
  double teleop_intent_horizon_m{2.0};
  double teleop_intent_max_deviation_deg{55.0};
  double recovery_translation_speed_mps{0.15};
  double recovery_rotation_rate_rad_s{0.25};
  double recovery_stuck_linear_progress_m{0.05};
  double recovery_stuck_yaw_progress_rad{0.05};
  double slow_rate_1{0.25};
  double slow_rate_2{0.5};
  double slow_rate_3{0.75};
  nav_kernel::LocalPlannerParams local_planner{};
  nav_kernel::PathFollowerParams path_follower{};
};

struct NavLoopOutput {
  bool active{false};
  bool goal_reached{false};
  bool path_found{false};
  bool near_field_stop{false};
  bool recovery_exhausted{false};
  std::string reason{"not_configured"};
  int slow_down{0};
  int recovery_state{0};
  int recovery_action{0};
  int recovery_attempt{0};
  int recovery_candidate_count{0};
  bool recovery_verified{false};
  bool recovery_observation_refresh_required{false};
  double recovery_progress{0.0};
  std::string recovery_reason{"inactive"};
  std::size_t target_index{0};
  double target_distance_m{0.0};
  nav_kernel::Vec3 target{};
  std::vector<nav_kernel::Vec3> local_path_body;
  std::vector<nav_kernel::Vec3> local_path_map;
  nav_kernel::LocalPlannerDebugSnapshot local_planner_debug;
  nav_kernel::Twist cmd_vel{};
};

struct PlannerObservationView {
  std::uint64_t frame_epoch{0};
  std::uint64_t cloud_generation{0};
  std::uint64_t traversability_generation{0};
  double odom_stamp_s{0.0};
  double cloud_stamp_s{0.0};
  double traversability_stamp_s{0.0};
};

struct TraversabilityGridView {
  const float* values{nullptr};
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint64_t generation{0};

  bool valid() const {
    return values != nullptr && rows > 0 && cols > 0 && resolution > 0.0;
  }
};

class NavLoop {
 public:
  explicit NavLoop(NavLoopConfig config = {});

  bool configure();
  bool configured() const;

  void setGlobalPath(const std::vector<nav_kernel::Vec3> &path,
                     std::optional<double> final_yaw = std::nullopt,
                     std::optional<double> goal_reached_m = std::nullopt,
                     std::optional<double> goal_yaw_tolerance_rad = std::nullopt);
  void clearGlobalPath();
  [[nodiscard]] bool hasRetainedGlobalPath() const;
  void stopLinearMotion();
  void suspendAutonomy();

  NavLoopOutput tick(const nav_kernel::Pose &odom_map_body, const float *obstacle_xyzh,
                     int obstacle_count, double timestamp_s,
                     TraversabilityGridView traversability = {},
                     PlannerObservationView observation = {});
  NavLoopOutput tickTeleopIntent(const nav_kernel::Pose &odom_map_body,
                                 const nav_kernel::Twist &intent, const float *obstacle_xyzh,
                                 int obstacle_count, double timestamp_s,
                                 TraversabilityGridView traversability = {});

 private:
  std::size_t selectTargetIndex(const nav_kernel::Pose &odom_map_body);
  std::vector<nav_kernel::Vec3> bodyPathToMap(const nav_kernel::Pose &odom_map_body,
                                              const std::vector<nav_kernel::Vec3> &body_path) const;
  nav_kernel::LocalPlannerDebugSnapshot
  debugSnapshotToMap(const nav_kernel::Pose &odom_map_body,
                     nav_kernel::LocalPlannerDebugSnapshot snapshot) const;
  double slowFactor(int slow_down) const;
  bool atGoal(const nav_kernel::Pose &odom_map_body) const;
  double goalYawError(const nav_kernel::Pose &odom_map_body) const;
  void syncTraversabilityGrid(TraversabilityGridView traversability);
  bool autonomyMotionStalled(const nav_kernel::Pose &odom_map_body, double timestamp_s);
  void setAutonomyMotionExpected(bool expected, const nav_kernel::Pose &odom_map_body,
                                 double timestamp_s);
  void resetAutonomyProgress();
  bool recoveryObservationAdvanced(const PlannerObservationView& observation) const;
  void clearRecoveryObservationWait();

  NavLoopConfig config_;
  nav_kernel::LocalPlannerCore local_planner_;
  nav_kernel::PathFollowerState follower_state_{};
  nav_kernel::PathFollowerState recovery_follower_state_{};
  std::vector<nav_kernel::Vec3> global_path_;
  std::optional<double> final_yaw_;
  double active_goal_reached_m_{0.35};
  double active_goal_yaw_tolerance_rad_{0.08726646259971647};
  std::size_t cursor_{0};
  bool configured_{false};
  bool traversability_loaded_{false};
  std::uint64_t traversability_generation_{0};
  bool recovery_active_{false};
  int recovery_action_{0};
  int recovery_attempt_{-1};
  bool recovery_observation_waiting_{false};
  PlannerObservationView recovery_observation_baseline_{};
  bool autonomy_motion_expected_{false};
  bool autonomy_progress_valid_{false};
  nav_kernel::Pose autonomy_progress_pose_{};
  double autonomy_progress_time_s_{0.0};
};

}  // namespace lingtu::nav::plan
