#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "navigation/recovery.hpp"
#include "navigation/route.hpp"
#include "planning/local/planner.hpp"
#include "tracking/follower.hpp"

namespace lingtu::nav::navigation {

// Executes an admitted route or assisted-teleop intent through local planning
// and path tracking. The result is pre-safety motion intent; this class neither
// computes global routes nor publishes commands to the robot.
struct ExecutorConfig {
  double corridor_lookahead_m{3.0};
  double waypoint_reached_m{0.6};
  double goal_reached_m{0.35};
  double goal_height_tolerance_m{0.35};
  double goal_yaw_tolerance_rad{0.08726646259971647};
  double goal_yaw_kp{1.5};
  double goal_yaw_max_rate{0.6};
  double max_speed{0.5};
  double teleop_intent_horizon_m{3.5};
  double teleop_intent_max_deviation_deg{90.0};
  RecoveryConfig recovery{};
  double slow_rate_1{0.25};
  double slow_rate_2{0.5};
  double slow_rate_3{0.75};
  nav_kernel::FollowerParams follower{};
};

struct ExecutionOutput {
  bool active{false};
  bool goal_reached{false};
  bool path_found{false};
  bool near_field_stop{false};
  bool hold_body_heading{false};
  bool trajectory_frozen{false};
  bool recovery_exhausted{false};
  std::string reason{"not_configured"};
  int slow_down{0};
  int recovery_state{0};
  int recovery_action{0};
  int recovery_attempt{0};
  int recovery_candidate_count{0};
  double recovery_rotation_target_rad{0.0};
  bool recovery_verified{false};
  bool recovery_observation_refresh_required{false};
  double recovery_progress{0.0};
  std::string recovery_trigger{"inactive"};
  std::string recovery_reason{"inactive"};
  std::size_t target_index{0};
  double target_distance_m{0.0};
  nav_kernel::Vec3 target{};
  std::vector<nav_kernel::Vec3> local_path_body;
  std::vector<nav_kernel::Vec3> local_path_map;
  nav_kernel::LocalPlannerDebugSnapshot local_planner_debug;
  nav_kernel::Twist cmd_vel{};
};

struct ExecutionObservation {
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

enum class ExecutionMode {
  Route,
  MotionIntent,
};

struct ExecutionInput {
  ExecutionMode mode{ExecutionMode::Route};
  nav_kernel::Pose mapBody{};
  nav_kernel::Pose odomBody{};
  MapFromOdomTransform mapFromOdom{};
  const float *obstacleXyzhMap{nullptr};
  int obstacleCount{0};
  double timestampS{0.0};
  TraversabilityGridView traversability{};
  ExecutionObservation observation{};
  nav_kernel::Twist motionIntent{};
};

class Executor {
 public:
  Executor(ExecutorConfig config, nav_kernel::local::Planner planner);

  void setRoute(Route route);
  void clear();
  ExecutionOutput tick(const ExecutionInput &input);

  [[nodiscard]] bool hasRoute() const;
  void pauseLinearMotion();
  void replanTeleop();
  void stopLinearMotion();
  void suspendAutonomy();

 private:
  struct SegmentTarget {
    std::size_t index{0};
    nav_kernel::Vec3 point{};
    bool reachesGoal{false};
  };

  struct TeleopReference {
    nav_kernel::Vec3 origin{};
    double headingMap{0.0};
    double directionBody{0.0};
  };

  void activateRoute(const std::vector<nav_kernel::Vec3> &path,
                     std::optional<double> final_yaw,
                     std::optional<double> goal_reached_m,
                     std::optional<double> goal_yaw_tolerance_rad);
  void clearRoute();
  ExecutionOutput tickRoute(const nav_kernel::Pose &map_body,
                            const nav_kernel::Pose &odom_body,
                            const MapFromOdomTransform &map_from_odom,
                            const float *obstacle_xyzh_map, int obstacle_count,
                            double timestamp_s,
                            TraversabilityGridView odom_traversability,
                            ExecutionObservation observation);
  ExecutionOutput tickIntent(const nav_kernel::Pose &map_body,
                             const nav_kernel::Twist &intent,
                             const float *obstacle_xyzh, int obstacle_count,
                             double timestamp_s, TraversabilityGridView traversability,
                             ExecutionObservation observation);

  ExecutionOutput tickInPlanningFrame(const nav_kernel::Pose &map_body,
                                      const nav_kernel::Pose &planning_body,
                                      const MapFromOdomTransform &map_from_odom,
                                      const float *obstacle_xyzh_planning, int obstacle_count,
                                      double timestamp_s, TraversabilityGridView traversability,
                                      ExecutionObservation observation);
  nav_kernel::LocalPlan planLocal(const nav_kernel::LocalPlanRequest &request,
                                  const MapFromOdomTransform &map_from_odom,
                                  nav_kernel::LocalPlannerDebugSnapshot *debug);
  void resetTeleopRotation();
  void resetTeleopReference();
  void resetLocalPlanning();
  SegmentTarget buildSegment(const nav_kernel::Pose &map_body,
                             const nav_kernel::Pose &planning_body,
                             const MapFromOdomTransform &map_from_odom);
  void applyCommittedLocalGuide(const nav_kernel::Pose &map_body,
                                const nav_kernel::Pose &planning_body,
                                const MapFromOdomTransform &map_from_odom, double timestamp_s);
  nav_kernel::LocalKinematicState planningKinematics(const nav_kernel::Pose &planning_body,
                                                     const ExecutionObservation &observation,
                                                     double timestamp_s);
  std::vector<nav_kernel::Vec3> bodyPathToMap(const nav_kernel::Pose &odom_map_body,
                                              const std::vector<nav_kernel::Vec3> &body_path) const;
  nav_kernel::LocalPlannerDebugSnapshot
  debugSnapshotToMap(const nav_kernel::Pose &odom_map_body,
                     nav_kernel::LocalPlannerDebugSnapshot snapshot) const;
  double slowFactor(int slow_down) const;
  bool atGoal(const nav_kernel::Pose &odom_map_body) const;
  double goalYawError(const nav_kernel::Pose &odom_map_body) const;
  bool autonomyMotionStalled(const nav_kernel::Pose &odom_map_body, double timestamp_s,
                             const nav_kernel::LocalKinematicState &kinematics);
  void setAutonomyMotionExpected(bool expected, const nav_kernel::Pose &odom_map_body,
                                 double timestamp_s);
  void resetAutonomyProgress();
  bool recoveryObservationAdvanced(const ExecutionObservation &observation) const;
  void clearRecoveryObservationWait();

  ExecutorConfig config_;
  nav_kernel::local::Planner local_planner_;
  Recovery recovery_;
  Recovery teleop_recovery_;
  nav_kernel::Follower follower_{};
  nav_kernel::Follower recovery_follower_{};
  std::vector<nav_kernel::Vec3> route;
  std::vector<nav_kernel::Vec3> segment;
  std::vector<nav_kernel::Vec3> committed_local_path_map_;
  std::optional<double> teleop_recovery_intent_rad_;
  std::optional<TeleopReference> teleop_reference_;
  std::uint64_t generation{0};
  std::uint64_t committed_route_generation_{0};
  double committed_local_path_time_s_{-1.0};
  std::optional<double> final_yaw_;
  std::optional<double> height_offset_;
  double active_goal_reached_m_{0.35};
  double active_goal_height_tolerance_m_{0.35};
  double active_goal_yaw_tolerance_rad_{0.08726646259971647};
  std::size_t progress{0};
  std::vector<float> obstacle_xyzh_odom_scratch_;
  int recovery_action_{0};
  int recovery_attempt_{-1};
  bool recovery_observation_waiting_{false};
  ExecutionObservation recovery_observation_baseline_{};
  bool autonomy_motion_expected_{false};
  bool autonomy_progress_valid_{false};
  nav_kernel::Pose autonomy_progress_pose_{};
  double autonomy_progress_time_s_{0.0};
  nav_kernel::Vec3 previous_planning_velocity_{};
  double previous_kinematics_time_s_{-1.0};
  std::uint64_t previous_kinematics_frame_epoch_{0};
  double local_blocked_since_s_{-1.0};
  bool traj_frozen_{false};
  bool intent_mode_{false};
};

}  // namespace lingtu::nav::navigation
