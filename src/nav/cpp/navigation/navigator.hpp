#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "navigation/recovery.hpp"
#include "planning/local/planner.hpp"
#include "planning/local/task.hpp"
#include "tracking/follower.hpp"

namespace lingtu::nav::navigation {

// Executes an admitted route or assisted-teleop intent through local planning
// and path tracking. The result is pre-safety motion intent; this class neither
// computes global routes nor publishes commands to the robot.
struct NavigatorConfig {
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
  RecoveryConfig recovery{};
  double slow_rate_1{0.25};
  double slow_rate_2{0.5};
  double slow_rate_3{0.75};
  nav_kernel::LocalPlannerParams planner{};
  nav_kernel::FollowerParams follower{};
};

struct NavigatorOutput {
  bool active{false};
  bool goal_reached{false};
  bool path_found{false};
  bool near_field_stop{false};
  bool trajectory_frozen{false};
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
  std::vector<nav_kernel::TrajectoryPoint> local_trajectory_body;
  nav_kernel::LocalPlannerDebugSnapshot local_planner_debug;
  nav_kernel::Twist cmd_vel{};
};

struct NavigatorObservation {
  std::uint64_t frame_epoch{0};
  std::uint64_t cloud_generation{0};
  std::uint64_t traversability_generation{0};
  double odom_stamp_s{0.0};
  double cloud_stamp_s{0.0};
  double traversability_stamp_s{0.0};
  nav_kernel::Vec3 body_linear_velocity{};
  double body_yaw_rate{0.0};
  bool body_velocity_valid{false};
  nav_kernel::LocalCollisionMapView collision{};
};

struct TraversabilityGridView {
  const float *values{nullptr};
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint64_t generation{0};

  [[nodiscard]] bool valid() const;
};

// Rigid 2D transform with the explicit direction map <- odom.  The local
// planner uses odom as its stable rolling frame, while global goals and
// published paths remain in map.
struct MapFromOdomTransform {
  nav_kernel::Vec3 translation{};
  double yaw{0.0};

  [[nodiscard]] bool valid() const;
  [[nodiscard]] nav_kernel::Vec3 mapPointFromOdom(const nav_kernel::Vec3 &point_odom) const;
  [[nodiscard]] nav_kernel::Vec3 odomPointFromMap(const nav_kernel::Vec3 &point_map) const;
};

class Navigator {
 public:
  explicit Navigator(NavigatorConfig config = {});

  bool configure();
  bool configured() const;

  void setRoute(const std::vector<nav_kernel::Vec3> &path,
                std::optional<double> final_yaw = std::nullopt,
                std::optional<double> goal_reached_m = std::nullopt,
                std::optional<double> goal_yaw_tolerance_rad = std::nullopt);
  void clearRoute();
  [[nodiscard]] bool hasRoute() const;
  void stopLinearMotion();
  void suspendAutonomy();

  NavigatorOutput tick(const nav_kernel::Pose &odom_map_body, const float *obstacle_xyzh,
                       int obstacle_count, double timestamp_s,
                       TraversabilityGridView traversability = {},
                       NavigatorObservation observation = {});
  // Runs local planning in odom. Route selection, terminal-goal
  // semantics, and all published targets/paths/debug candidates stay in map.
  // Obstacle points are supplied in map and transformed internally into odom.
  NavigatorOutput tickOdom(const nav_kernel::Pose &map_body, const nav_kernel::Pose &odom_body,
                           const MapFromOdomTransform &map_from_odom,
                           const float *obstacle_xyzh_map, int obstacle_count, double timestamp_s,
                           TraversabilityGridView odom_traversability = {},
                           NavigatorObservation observation = {});
  NavigatorOutput tickIntent(const nav_kernel::Pose &odom_map_body, const nav_kernel::Twist &intent,
                             const float *obstacle_xyzh, int obstacle_count, double timestamp_s,
                             TraversabilityGridView traversability = {},
                             NavigatorObservation observation = {});

 private:
  struct SegmentTarget {
    std::size_t index{0};
    nav_kernel::Vec3 point{};
    bool reachesGoal{false};
  };

  struct CollisionCache {
    bool valid{false};
    std::uint64_t reset_epoch{0};
    std::uint64_t sequence{0};
    std::uint64_t generation{0};
    int count{0};
    MapFromOdomTransform transform{};
    nav_kernel::Vec3 aabb_min{};
    nav_kernel::Vec3 aabb_max{};
  };

  NavigatorOutput tickInPlanningFrame(const nav_kernel::Pose &map_body,
                                      const nav_kernel::Pose &planning_body,
                                      const MapFromOdomTransform &map_from_odom,
                                      const float *obstacle_xyzh_planning, int obstacle_count,
                                      double timestamp_s, TraversabilityGridView traversability,
                                      NavigatorObservation observation);
  nav_kernel::LocalPlanResult planAutonomy(const nav_kernel::LocalPlanInput &input,
                                           const MapFromOdomTransform &map_from_odom,
                                           double timestamp_s,
                                           nav_kernel::LocalPlannerDebugSnapshot *debug);
  nav_kernel::LocalPlanResult planTeleop(const nav_kernel::LocalPlanInput &input,
                                         const nav_kernel::LocalMotionIntent &intent,
                                         double timestamp_s,
                                         nav_kernel::LocalPlannerDebugSnapshot *debug);
  void resetLocalPlanning();
  SegmentTarget buildSegment(const nav_kernel::Pose &map_body,
                             const nav_kernel::Pose &planning_body,
                             const MapFromOdomTransform &map_from_odom);
  void applyCommittedLocalGuide(const nav_kernel::Pose &map_body,
                                const nav_kernel::Pose &planning_body,
                                const MapFromOdomTransform &map_from_odom, double timestamp_s);
  nav_kernel::LocalKinematicState planningKinematics(const nav_kernel::Pose &planning_body,
                                                     const NavigatorObservation &observation,
                                                     double timestamp_s);
  std::vector<nav_kernel::Vec3> bodyPathToMap(const nav_kernel::Pose &odom_map_body,
                                              const std::vector<nav_kernel::Vec3> &body_path) const;
  nav_kernel::LocalPlannerDebugSnapshot
  debugSnapshotToMap(const nav_kernel::Pose &odom_map_body,
                     nav_kernel::LocalPlannerDebugSnapshot snapshot) const;
  double slowFactor(int slow_down) const;
  bool atGoal(const nav_kernel::Pose &odom_map_body) const;
  double goalYawError(const nav_kernel::Pose &odom_map_body) const;
  bool autonomyMotionStalled(const nav_kernel::Pose &odom_map_body, double timestamp_s);
  void setAutonomyMotionExpected(bool expected, const nav_kernel::Pose &odom_map_body,
                                 double timestamp_s);
  void resetAutonomyProgress();
  bool recoveryObservationAdvanced(const NavigatorObservation &observation) const;
  void clearRecoveryObservationWait();
  void advanceTrajectoryClock(double timestamp_s);

  NavigatorConfig config_;
  nav_kernel::local::Planner local_planner_;
  std::unique_ptr<nav_kernel::local::LocalPlanTask> scan_task_;
  std::optional<nav_kernel::local::LocalPlanCompletion> scan_plan_;
  nav_kernel::LocalPlanIdentity scan_submitted_identity_{};
  std::uint64_t scan_submitted_route_generation_{0};
  double scan_submitted_time_s_{-1.0};
  std::optional<nav_kernel::LocalMotionIntent> scan_submitted_intent_;
  bool scan_intent_mode_{false};
  Recovery recovery_;
  nav_kernel::Follower follower_{};
  nav_kernel::Follower recovery_follower_{};
  std::vector<nav_kernel::Vec3> route;
  std::vector<nav_kernel::Vec3> segment;
  std::vector<nav_kernel::Vec3> committed_local_path_map_;
  std::uint64_t generation{0};
  std::uint64_t committed_route_generation_{0};
  double committed_local_path_time_s_{-1.0};
  std::optional<double> final_yaw_;
  double active_goal_reached_m_{0.35};
  double active_goal_yaw_tolerance_rad_{0.08726646259971647};
  std::size_t progress{0};
  bool configured_{false};
  std::vector<float> obstacle_xyzh_odom_scratch_;
  std::vector<float> collision_xyz_odom_scratch_;
  CollisionCache collision_cache_{};
  int recovery_action_{0};
  int recovery_attempt_{-1};
  bool recovery_observation_waiting_{false};
  NavigatorObservation recovery_observation_baseline_{};
  bool autonomy_motion_expected_{false};
  bool autonomy_progress_valid_{false};
  nav_kernel::Pose autonomy_progress_pose_{};
  double autonomy_progress_time_s_{0.0};
  nav_kernel::Vec3 previous_planning_velocity_{};
  double previous_kinematics_time_s_{-1.0};
  std::uint64_t previous_kinematics_frame_epoch_{0};
  double local_blocked_since_s_{-1.0};
  double traj_delay_s_{0.0};
  double traj_clock_s_{-1.0};
  double traj_stamp_s_{-1.0};
  bool traj_frozen_{false};
};

}  // namespace lingtu::nav::navigation
