#pragma once

#include <lingtu/settings.hpp>

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

enum class PlanningFrame { Odom, Map };

// Executes an admitted route or assisted-teleop intent through local planning
// and path tracking. The result is pre-safety motion intent; this class neither
// computes global routes nor publishes commands to the robot.
struct ExecutorConfig {
  PlanningFrame planning_frame{PlanningFrame::Odom};
  double corridor_lookahead_m{::lingtu::settings::follower::native_nav::corridor_lookahead_m};
  double waypoint_reached_m{::lingtu::settings::follower::native_nav::waypoint_reached_m};
  double goal_reached_m{::lingtu::settings::follower::native_nav::goal_reached_m};
  double goal_height_tolerance_m{::lingtu::settings::follower::native_nav::goal_height_tolerance_m};
  double max_speed{::lingtu::settings::limits::native_nav::follower::max_speed_mps};
  double teleop_intent_horizon_m{::lingtu::settings::teleop::native_nav::teleop::horizon_m};
  double teleop_intent_max_deviation_deg{::lingtu::settings::teleop::native_nav::teleop::max_deviation_deg};
  RecoveryConfig recovery{};
  double slow_rate_1{0.25};
  double slow_rate_2{0.5};
  double slow_rate_3{0.75};
  nav_kernel::FollowerParams follower{};
  StairTraversalConfig stairs{};
  std::shared_ptr<const collision::Body> body{};
};

struct ExecutionOutput {
  bool active{false};
  bool goal_reached{false};
  bool path_found{false};
  bool near_field_stop{false};
  bool trajectory_frozen{false};
  bool recovery_exhausted{false};
  bool planning_failed{false};
  // Public failure names are owned by the producing boundary:
  // local_no_path/local_blocked/local_near_field_stop come from the local
  // planner; execution_stalled is emitted only after Executor observes that
  // an issued command produced no measured motion. "active" and "inactive"
  // in recovery_trigger are lifecycle markers, not failure causes.
  std::string reason{"not_configured"};
  int slow_down{0};
  int recovery_state{0};
  int recovery_action{0};
  int recovery_attempt{0};
  int recovery_candidate_count{0};
  double recovery_rotation_target_rad{0.0};
  bool recovery_verified{false};
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
  nav_kernel::FollowerTracking tracking{};
  std::size_t prediction_count{0};
  StairTraversalOutput stairs{};
  std::size_t pose_stage_index{0};
  std::size_t pose_stage_count{0};
  std::uint64_t route_generation{0};
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
  std::shared_ptr<const collision::Body> body_geometry{};
  nav_kernel::LocalCollisionMapView collision{};
  nav_kernel::PlanClockMode clock_mode{nav_kernel::PlanClockMode::Steady};
  nav_kernel::PredictionView predictions{};
  std::shared_ptr<const std::vector<lingtu::maps::NoGoRegion>> no_go_regions{};
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

// Rigid transform map <- odom. Routes and published previews are always map;
// composition selects whether local planning also uses map or uses odom.
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
  StairProfileFeedback stairProfile{};
};

class Executor {
 public:
  Executor(ExecutorConfig config, nav_kernel::LocalPlanner planner);

  void setRoute(Route route);
  void clear();
  void setSpeedLimit(double max_speed_mps);
  ExecutionOutput tick(const ExecutionInput &input);

  [[nodiscard]] bool hasRoute() const;
  [[nodiscard]] bool hasStairTransition() const { return stairs_.pending(); }
  StairTraversalOutput tickStairTransition(const nav_kernel::Pose& body,
      const ExecutionObservation& observation, double timestamp_s,
      const StairProfileFeedback& feedback);
  [[nodiscard]] double activeMaxSpeedMps() const { return active_max_speed_mps_; }
  [[nodiscard]] std::shared_ptr<const collision::Body> body() const { return config_.body; }
  [[nodiscard]] std::shared_ptr<const collision::Body> activeBody() const {
    return route_body_ ? route_body_ : config_.body;
  }
  void pauseLinearMotion();
  // Record the command actually accepted by final control.
  void commitApplied(const nav_kernel::Twist &command);
  void reportFinalMotionBlocked(bool blocked, double timestamp_s);
  void replanTeleop();
  void stopLinearMotion();
  // Reset local execution while retaining the admitted route and goal.
  void resetMotion();
  void resumeRoute();

 private:
  static constexpr double kPoseStagePositionToleranceM = 0.01;
  static std::vector<nav_kernel::Vec3> planningPathToBody(
      const nav_kernel::Pose &body, const std::vector<nav_kernel::Vec3> &path);
  static std::vector<nav_kernel::Vec3> planningPathToMap(
      const MapFromOdomTransform &transform, const std::vector<nav_kernel::Vec3> &path);
  static nav_kernel::LocalPlanRequest makeLocalPlanRequest(
      const nav_kernel::Pose &vehicle, const std::vector<nav_kernel::Vec3> &route,
      const std::vector<nav_kernel::Vec3> *reference_route, std::uint64_t route_generation,
      nav_kernel::LocalKinematicState kinematics, const ExecutionObservation &observation,
      const float *obstacle_xyzh, int obstacle_count, double timestamp_s,
      bool execution_frozen, const TraversabilityGridView &traversability,
      bool include_traversability, const nav_kernel::LocalMotionIntent *intent = nullptr);

  struct SegmentTarget {
    std::size_t index{0};
    nav_kernel::Vec3 point{};
    bool reachesGoal{false};
  };

  struct TeleopReference {
    nav_kernel::Vec3 corridorOrigin{};
    nav_kernel::Vec3 guideStart{};
    nav_kernel::Vec3 guideTarget{};
    double headingMap{0.0};
    double directionBody{0.0};
    bool operatorTurning{false};
  };

  void activateRoute(const std::vector<nav_kernel::Vec3> &path,
                     std::optional<double> stage_yaw,
                     std::optional<double> goal_reached_m,
                     std::optional<double> stage_yaw_tolerance,
                     std::optional<double> max_speed_mps);
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
  void resetTeleopBoundaryDeparture();
  void resetLocalPlanning(bool full_follower_reset = false);
  SegmentTarget buildSegment(const nav_kernel::Pose &map_body,
                             const nav_kernel::Pose &planning_body,
                             const MapFromOdomTransform &map_from_odom);
  void buildReference(const SegmentTarget &target,
                      const MapFromOdomTransform &map_from_odom);
  void applyLocalGuide(const nav_kernel::Pose &map_body,
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
  bool atGoal(const nav_kernel::Pose &body) const;
  bool autonomyMotionStalled(const nav_kernel::Pose &odom_map_body, double timestamp_s,
                            bool planning_pending);
  void setAutonomyMotionExpected(const nav_kernel::Twist &command,
                                 const nav_kernel::Pose &odom_map_body,
                                 double timestamp_s);
  void resetAutonomyProgress();

  ExecutorConfig config_;
  nav_kernel::LocalPlanner local_planner_;
  Recovery recovery_;
  Recovery teleop_recovery_;
  StairTraversal stairs_;
  StairTraversalOutput stair_step_;
  nav_kernel::Follower follower_{};
  nav_kernel::Follower recovery_follower_{};
  std::vector<nav_kernel::Vec3> route;
  std::vector<PoseRouteStage> pose_stages_;
  std::size_t pose_stage_{0};
  std::shared_ptr<const collision::Body> route_body_{};
  std::optional<double> pose_final_tolerance_;
  std::optional<double> pose_yaw_tolerance_;
  std::optional<double> pose_max_speed_;
  std::optional<std::uint64_t> pose_epoch_;
  void activatePoseStage();
  void tryBlendPoseStage(double grid_resolution_m);
  void limitPoseSpeed();
  bool pose_blending_allowed_{false};
  std::optional<double> pose_support_height_m_;
  std::optional<double> pose_support_tolerance_m_;
  bool pose_blend_checked_{false};
  std::optional<PoseRouteBlend> pose_blend_;
  std::vector<PoseRouteStage> pose_blend_original_;
  std::vector<nav_kernel::Vec3> segment;
  std::vector<nav_kernel::Vec3> reference;
  std::optional<std::uint64_t> reference_epoch;
  std::vector<nav_kernel::Vec3> guide_map_;
  std::optional<double> teleop_recovery_intent_rad_;
  std::optional<double> teleop_boundary_departure_intent_rad_;
  std::optional<TeleopReference> teleop_reference_;
  std::uint64_t generation{0};
  std::uint64_t guide_generation_{0};
  double guide_stamp_s_{-1.0};
  std::optional<double> stage_yaw_;
  std::optional<nav_kernel::Vec3> requested_goal_;
  double active_goal_reached_m_{0.35};
  double active_max_speed_mps_{nav_kernel::kDefaultLinearSpeedMps};
  double stage_yaw_tolerance_{};
  std::size_t progress{0};
  std::vector<float> obstacle_xyzh_odom_scratch_;
  std::vector<nav_kernel::PredictedObstacle> predictions_odom_scratch_;
  int recovery_action_{0};
  int recovery_attempt_{-1};
  bool autonomy_motion_expected_{false};
  bool autonomy_stall_stop_{false};
  nav_kernel::Twist autonomy_expected_command_{};  // Planning-frame translation.
  nav_kernel::Pose autonomy_progress_pose_{};
  double autonomy_progress_time_s_{0.0};
  double autonomy_sample_time_s_{0.0};
  double autonomy_observed_progress_{0.0};
  double autonomy_requested_progress_{0.0};
  nav_kernel::Vec3 previous_planning_velocity_{};
  double velocity_stamp_s_{-1.0};
  std::uint64_t previous_kinematics_frame_epoch_{0};
  double local_blocked_since_s_{-1.0};
  double final_motion_blocked_since_s_{-1.0};
  bool traj_frozen_{false};
  bool route_start_pending_{false};
  bool stair_replan_pending_{false};
  bool intent_mode_{false};
};

}  // namespace lingtu::nav::navigation
