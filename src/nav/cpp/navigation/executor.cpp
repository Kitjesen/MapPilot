#include "navigation/executor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lingtu::nav::navigation {
namespace {

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

constexpr double kTeleopIntentToleranceRad = 10.0 * M_PI / 180.0;

double bodyDistance2D(const nav_kernel::Vec3 &point) {
  return std::hypot(point.x, point.y);
}

std::vector<nav_kernel::Vec3> planningPathToBody(
    const nav_kernel::Pose &body,
    const std::vector<nav_kernel::Vec3> &planningPath) {
  std::vector<nav_kernel::Vec3> result;
  result.reserve(planningPath.size());
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  for (const auto &point : planningPath) {
    const double dx = point.x - body.position.x;
    const double dy = point.y - body.position.y;
    result.push_back({c * dx + s * dy, -s * dx + c * dy,
                      point.z - body.position.z});
  }
  return result;
}


nav_kernel::LocalPlanRequest makeLocalPlanRequest(
    const nav_kernel::Pose &vehicle, const std::vector<nav_kernel::Vec3> &route,
    std::uint64_t route_generation, bool reaches_goal,
    nav_kernel::LocalKinematicState kinematics, const ExecutionObservation &observation,
    const float *obstacle_xyzh, int obstacle_count, double timestamp_s,
    bool execution_frozen, const TraversabilityGridView &traversability,
    const nav_kernel::LocalMotionIntent *intent = nullptr) {
  nav_kernel::LocalRouteView route_view{
      route.empty() ? nullptr : route.data(),
      static_cast<int>(route.size()),
      route_generation,
      reaches_goal,
  };
  nav_kernel::LocalPlanRequest request;
  request.robot = {vehicle, kinematics};
  request.objective = intent != nullptr
                          ? nav_kernel::LocalObjective{
                                nav_kernel::MotionIntentTarget{*intent, route_view}}
                          : nav_kernel::LocalObjective{nav_kernel::RouteTarget{route_view}};
  request.identity = {
      observation.frame_epoch,
      observation.collision.present() ? observation.collision.generation
                                      : observation.cloud_generation,
      observation.traversability_generation,
  };
  request.environment.obstacles = {obstacle_xyzh, obstacle_count};
  request.environment.collision = observation.collision;
  request.clock = {timestamp_s, execution_frozen};
  if (traversability.valid()) {
    request.environment.traversability = {
        traversability.values, traversability.rows, traversability.cols,
        traversability.resolution, traversability.origin_x, traversability.origin_y,
    };
  }
  return request;
}

RecoveryConfig teleopRotationConfig(RecoveryConfig config) {
  config.behavior_order = {nav_kernel::RecoveryAction::Rotate};
  return config;
}

void copyRecoveryDiagnostics(ExecutionOutput &output, const RecoveryOutput &recovery) {
  output.recovery_state = recovery.state;
  output.recovery_action = static_cast<int>(recovery.action);
  output.recovery_attempt = recovery.attempt;
  output.recovery_candidate_count = recovery.candidate_count;
  output.recovery_rotation_target_rad = recovery.rotation_target_rad;
  output.recovery_verified = recovery.verified;
  output.recovery_observation_refresh_required = recovery.observation_refresh_required;
  output.recovery_progress = recovery.progress;
  output.recovery_reason = recovery.reason;
  output.recovery_exhausted = recovery.exhausted;
}

void applyTeleopRotation(ExecutionOutput &output, const RecoveryOutput &recovery,
                         double rotation_rate_rad_s) {
  copyRecoveryDiagnostics(output, recovery);
  output.path_found = false;
  output.near_field_stop = true;
  output.local_path_body.clear();
  output.local_path_map.clear();
  output.cmd_vel = {};
  if (recovery.active && recovery.verified && recovery.direct_command) {
    output.cmd_vel.wz =
        static_cast<double>(recovery.rotation_direction) * rotation_rate_rad_s;
  }
  output.reason = recovery.reason;
}

}  // namespace

bool TraversabilityGridView::valid() const {
  return values != nullptr && rows > 0 && cols > 0 && resolution > 0.0;
}

Executor::Executor(ExecutorConfig config, nav_kernel::local::Planner planner)
    : config_(std::move(config)),
      local_planner_(std::move(planner)),
      recovery_(local_planner_.params(), config_.recovery),
      teleop_recovery_(local_planner_.params(), teleopRotationConfig(config_.recovery)) {
  config_.goal_yaw_tolerance_rad = std::max(0.0, config_.goal_yaw_tolerance_rad);
  config_.goal_height_tolerance_m =
      std::max(config_.goal_reached_m, config_.goal_height_tolerance_m);
  config_.goal_yaw_kp = std::max(0.0, config_.goal_yaw_kp);
  config_.goal_yaw_max_rate = std::max(0.0, config_.goal_yaw_max_rate);
  config_.follower.maxSpeed = config_.max_speed;
  config_.follower.twoWayDrive = local_planner_.params().twoWayDrive;
  config_.recovery.translation_speed_mps =
      std::clamp(config_.recovery.translation_speed_mps, 0.0, config_.max_speed);
  config_.recovery.rotation_rate_rad_s = std::max(0.0, config_.recovery.rotation_rate_rad_s);
  config_.recovery.blocked_interval_s = std::max(0.0, config_.recovery.blocked_interval_s);
  config_.recovery.stuck_linear_progress_m =
      std::max(0.0, config_.recovery.stuck_linear_progress_m);
  config_.recovery.stuck_yaw_progress_rad = std::max(0.0, config_.recovery.stuck_yaw_progress_rad);
  active_goal_reached_m_ = config_.goal_reached_m;
  active_goal_height_tolerance_m_ = config_.goal_height_tolerance_m;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
}

void Executor::resetLocalPlanning() {
  local_planner_.reset();
  traj_frozen_ = false;
  intent_mode_ = false;
}


nav_kernel::LocalPlan Executor::planLocal(
    const nav_kernel::LocalPlanRequest &request,
    const MapFromOdomTransform &map_from_odom,
    nav_kernel::LocalPlannerDebugSnapshot *debug) {
  nav_kernel::LocalPlan plan = local_planner_.plan(request);
  nav_kernel::Pose map_body;
  map_body.position = map_from_odom.mapPointFromOdom(request.robot.pose.position);
  map_body.yaw = nav_kernel::normalizeAngle(map_from_odom.yaw + request.robot.pose.yaw);
  *debug = debugSnapshotToMap(map_body, local_planner_.debugSnapshot());
  return plan;
}

void Executor::setRoute(Route value) {
  activateRoute(value.points, value.finalYaw, value.goalToleranceM, value.yawToleranceRad);
}

void Executor::clear() {
  clearRoute();
}

void Executor::activateRoute(const std::vector<nav_kernel::Vec3> &path,
                             std::optional<double> final_yaw,
                             std::optional<double> goal_reached_m,
                             std::optional<double> goal_yaw_tolerance_rad) {
  route = path;
  ++generation;
  final_yaw_ = final_yaw;
  height_offset_.reset();
  active_goal_reached_m_ = std::max(0.01, goal_reached_m.value_or(config_.goal_reached_m));
  active_goal_height_tolerance_m_ = std::max(
      active_goal_reached_m_, config_.goal_height_tolerance_m);
  active_goal_yaw_tolerance_rad_ =
      std::max(0.0, goal_yaw_tolerance_rad.value_or(config_.goal_yaw_tolerance_rad));
  progress = 0;
  follower_.reset();
  recovery_follower_.reset();
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  resetLocalPlanning();
  recovery_.reset();
  segment.clear();
  committed_local_path_map_.clear();
  resetTeleopRotation();
  resetTeleopReference();
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  resetAutonomyProgress();
}

void Executor::clearRoute() {
  route.clear();
  ++generation;
  final_yaw_.reset();
  height_offset_.reset();
  active_goal_reached_m_ = config_.goal_reached_m;
  active_goal_height_tolerance_m_ = config_.goal_height_tolerance_m;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
  progress = 0;
  follower_.reset();
  recovery_follower_.reset();
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  resetLocalPlanning();
  recovery_.reset();
  segment.clear();
  committed_local_path_map_.clear();
  resetTeleopRotation();
  resetTeleopReference();
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  clearRecoveryObservationWait();
  resetAutonomyProgress();
}

bool Executor::hasRoute() const {
  return !route.empty();
}

void Executor::suspendAutonomy() {
  follower_.reset();
  recovery_follower_.reset();
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  resetLocalPlanning();
  recovery_.reset();
  committed_local_path_map_.clear();
  resetTeleopRotation();
  resetTeleopReference();
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  clearRecoveryObservationWait();
  resetAutonomyProgress();
}

void Executor::pauseLinearMotion() {
  follower_.stopLinear();
  recovery_follower_.stopLinear();
}

void Executor::replanTeleop() {
  pauseLinearMotion();
  follower_.resetTarget();
}

void Executor::stopLinearMotion() {
  pauseLinearMotion();
  follower_.resetTarget();
  resetTeleopRotation();
  resetTeleopReference();
}

void Executor::resetTeleopRotation() {
  teleop_recovery_.reset();
  teleop_recovery_intent_rad_.reset();
}

void Executor::resetTeleopReference() {
  teleop_reference_.reset();
}


ExecutionOutput Executor::tick(const ExecutionInput &input) {
  if (input.mode == ExecutionMode::MotionIntent) {
    return tickIntent(input.mapBody, input.motionIntent, input.obstacleXyzhMap,
                      input.obstacleCount, input.timestampS, input.traversability,
                      input.observation);
  }
  return tickRoute(input.mapBody, input.odomBody, input.mapFromOdom,
                   input.obstacleXyzhMap, input.obstacleCount, input.timestampS,
                   input.traversability, input.observation);
}

ExecutionOutput Executor::tickRoute(const nav_kernel::Pose &map_body,
                                    const nav_kernel::Pose &odom_body,
                                    const MapFromOdomTransform &map_from_odom,
                                    const float *obstacle_xyzh_map, int obstacle_count,
                                    double timestamp_s, TraversabilityGridView odom_traversability,
                                    ExecutionObservation observation) {
  if (!map_from_odom.valid()) {
    ExecutionOutput output;
    output.active = !route.empty();
    output.near_field_stop = output.active;
    output.reason = "invalid_map_from_odom";
    recovery_action_ = 0;
    recovery_attempt_ = -1;
    recovery_follower_.reset();
    follower_.reset();
    previous_kinematics_time_s_ = -1.0;
    resetLocalPlanning();
    recovery_.reset();
    clearRecoveryObservationWait();
    setAutonomyMotionExpected(false, odom_body, timestamp_s);
    return output;
  }

  const int safe_obstacle_count =
      obstacle_xyzh_map == nullptr ? 0 : std::max(0, obstacle_count);
  obstacle_xyzh_odom_scratch_.clear();
  obstacle_xyzh_odom_scratch_.reserve(static_cast<std::size_t>(safe_obstacle_count) * 4U);
  for (int index = 0; index < safe_obstacle_count; ++index) {
    const nav_kernel::Vec3 point_map{
        obstacle_xyzh_map[index * 4],
        obstacle_xyzh_map[index * 4 + 1],
        obstacle_xyzh_map[index * 4 + 2],
    };
    const nav_kernel::Vec3 point_odom = map_from_odom.odomPointFromMap(point_map);
    obstacle_xyzh_odom_scratch_.push_back(static_cast<float>(point_odom.x));
    obstacle_xyzh_odom_scratch_.push_back(static_cast<float>(point_odom.y));
    obstacle_xyzh_odom_scratch_.push_back(static_cast<float>(point_odom.z));
    obstacle_xyzh_odom_scratch_.push_back(obstacle_xyzh_map[index * 4 + 3]);
  }

  if (observation.collision.present()) {
    observation.collision.gridFromPlanningTranslation = map_from_odom.translation;
    observation.collision.gridFromPlanningYaw = map_from_odom.yaw;
  }

  return tickInPlanningFrame(
      map_body, odom_body, map_from_odom,
      obstacle_xyzh_odom_scratch_.empty() ? nullptr : obstacle_xyzh_odom_scratch_.data(),
      safe_obstacle_count, timestamp_s, odom_traversability, observation);
}

ExecutionOutput Executor::tickInPlanningFrame(const nav_kernel::Pose &map_body,
                                               const nav_kernel::Pose &planning_body,
                                               const MapFromOdomTransform &map_from_odom,
                                               const float *obstacle_xyzh_planning,
                                               int obstacle_count, double timestamp_s,
                                               TraversabilityGridView traversability,
                                               ExecutionObservation observation) {
  ExecutionOutput output;
  resetTeleopRotation();
  if (intent_mode_) {
    local_planner_.reset();
    intent_mode_ = false;
  }
  if (route.empty()) {
    resetLocalPlanning();
    recovery_.reset();
    clearRecoveryObservationWait();
    resetAutonomyProgress();
    output.reason = "no_global_path";
    return output;
  }

  output.active = true;
  if (atGoal(map_body)) {
    recovery_action_ = 0;
    recovery_attempt_ = -1;
    recovery_follower_.reset();
    resetLocalPlanning();
    recovery_.reset();
    clearRecoveryObservationWait();
    resetAutonomyProgress();
    const double yaw_error = goalYawError(map_body);
    if (final_yaw_ && std::abs(yaw_error) > active_goal_yaw_tolerance_rad_) {
      output.path_found = true;
      output.reason = "aligning_goal_yaw";
      output.target_index = route.size() - 1;
      output.target = route.back();
      output.target.z += height_offset_.value_or(0.0);
      output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
      output.cmd_vel.wz = std::clamp(config_.goal_yaw_kp * yaw_error, -config_.goal_yaw_max_rate,
                                     config_.goal_yaw_max_rate);
      follower_.reset();
      return output;
    }
    output.goal_reached = true;
    output.active = false;
    output.reason = "goal_reached";
    output.target_index = route.size() - 1;
    output.target = route.back();
    output.target.z += height_offset_.value_or(0.0);
    output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
    follower_.reset();
    return output;
  }

  const SegmentTarget target = buildSegment(map_body, planning_body, map_from_odom);
  applyCommittedLocalGuide(map_body, planning_body, map_from_odom, timestamp_s);
  output.target_index = target.index;
  output.target = target.point;
  output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);

  if (recovery_observation_waiting_) {
    if (observation.frame_epoch != recovery_observation_baseline_.frame_epoch) {
      recovery_observation_baseline_ = observation;
    } else if (recoveryObservationAdvanced(observation)) {
      clearRecoveryObservationWait();
    }
    if (recovery_observation_waiting_) {
      output.reason = "recovery_observation_wait";
      output.recovery_reason = output.reason;
      output.near_field_stop = true;
      follower_.stopLinear();
      recovery_follower_.stopLinear();
      setAutonomyMotionExpected(false, planning_body, timestamp_s);
      return output;
    }
  }

  const nav_kernel::LocalKinematicState kinematics =
      planningKinematics(planning_body, observation, timestamp_s);
  const nav_kernel::LocalPlanRequest plan_request = makeLocalPlanRequest(
      planning_body, segment, generation, target.reachesGoal, kinematics, observation,
      obstacle_xyzh_planning, obstacle_count, timestamp_s,
      traj_frozen_, traversability);
  nav_kernel::LocalPlan plan =
      planLocal(plan_request, map_from_odom, &output.local_planner_debug);

  const nav_kernel::LocalPlanStatus plan_status = plan.status();
  const bool plan_ready = plan.ready();
  const bool near_field_stop = plan_status == nav_kernel::LocalPlanStatus::NearFieldStop;
  const bool planner_input_valid =
      plan_status != nav_kernel::LocalPlanStatus::InvalidInput &&
      plan_status != nav_kernel::LocalPlanStatus::NotConfigured;
  const bool recovery_failure =
      plan_status == nav_kernel::LocalPlanStatus::NoPath ||
      plan_status == nav_kernel::LocalPlanStatus::Blocked || near_field_stop;
  if (recovery_failure && planner_input_valid &&
      (local_blocked_since_s_ < 0.0 || local_blocked_since_s_ > timestamp_s)) {
    local_blocked_since_s_ = timestamp_s;
  }
  const bool blocked_long_enough =
      local_blocked_since_s_ >= 0.0 &&
      timestamp_s - local_blocked_since_s_ >= std::max(0.0, config_.recovery.blocked_interval_s);
  const bool recovery_enabled = config_.recovery.max_attempts > 0;
  const bool recovery_active = recovery_.active();
  const bool stalled = !recovery_active && !blocked_long_enough &&
                       autonomyMotionStalled(planning_body, timestamp_s, kinematics);
  output.recovery_trigger = recovery_active
                                ? "active"
                                : (blocked_long_enough ? "blocked"
                                                       : (stalled ? "stalled" : "inactive"));
  const bool force_recovery = recovery_enabled && (recovery_active || blocked_long_enough || stalled);
  RecoveryOutput recovery;
  if (force_recovery && planner_input_valid) {
    recovery = recovery_.step(plan_request);
  } else if (!planner_input_valid || plan_ready) {
    recovery_.reset();
  }

  output.near_field_stop = near_field_stop;
  output.slow_down = std::clamp(plan.hints().slowdownLevel, 0, 3);
  output.recovery_state = recovery.state;
  output.recovery_action = static_cast<int>(recovery.action);
  output.recovery_attempt = recovery.attempt;
  output.recovery_candidate_count = recovery.candidate_count;
  output.recovery_rotation_target_rad = recovery.rotation_target_rad;
  output.recovery_verified = recovery.verified;
  output.recovery_observation_refresh_required = recovery.observation_refresh_required;
  output.recovery_progress = recovery.progress;
  output.recovery_reason = recovery.reason;
  output.recovery_exhausted = recovery.exhausted;
  output.local_path_body =
      recovery.active
          ? recovery.path_body
          : (std::holds_alternative<nav_kernel::SplineTarget>(plan.target())
                 ? planningPathToBody(planning_body, plan.previewPath())
                 : plan.previewPath());
  output.path_found = recovery.active
                          ? recovery.verified && recovery.path_body.size() >= 2
                          : plan_ready || output.local_path_body.size() >= 2;

  if (recovery_failure && !recovery.active && !recovery.exhausted && planner_input_valid) {
    if (local_blocked_since_s_ < 0.0 || local_blocked_since_s_ > timestamp_s) {
      local_blocked_since_s_ = timestamp_s;
    }
  } else if (plan_ready || recovery.active) {
    local_blocked_since_s_ = -1.0;
  }

  if (recovery.observation_refresh_required) {
    recovery_observation_waiting_ = true;
    recovery_observation_baseline_ = observation;
  }

  const bool recovery_action_changed =
      recovery.active &&
      (output.recovery_action != recovery_action_ || output.recovery_attempt != recovery_attempt_);
  if (recovery_action_changed) {
    recovery_follower_.reset();
  }
  recovery_action_ = recovery.active ? output.recovery_action : 0;
  recovery_attempt_ = recovery.active ? output.recovery_attempt : -1;

  if (output.path_found && !near_field_stop && output.local_path_body.size() < 2) {
    if (output.local_path_body.size() == 1 &&
        bodyDistance2D(output.local_path_body.front()) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
  }
  output.local_path_map = bodyPathToMap(map_body, output.local_path_body);
  if (!recovery.active && plan_ready && !near_field_stop &&
      plan.hints().retainRouteGuide &&
      output.local_path_map.size() >= 2) {
    committed_local_path_map_ = output.local_path_map;
    committed_route_generation_ = generation;
    committed_local_path_time_s_ = timestamp_s;
  } else if (recovery.active || (plan_ready && !plan.hints().retainRouteGuide)) {
    committed_local_path_map_.clear();
    committed_route_generation_ = 0;
    committed_local_path_time_s_ = -1.0;
  }

  if (recovery.exhausted) {
    output.reason = "local_recovery_exhausted";
    follower_.stopLinear();
    recovery_follower_.stopLinear();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }

  if (recovery.active && recovery.verified && recovery.direct_command) {
    output.cmd_vel.wz =
        static_cast<double>(recovery.rotation_direction) * config_.recovery.rotation_rate_rad_s;
    output.reason = output.recovery_reason;
    follower_.stopLinear();
    setAutonomyMotionExpected(std::abs(output.cmd_vel.wz) > 1e-6, planning_body, timestamp_s);
    return output;
  }

  if (recovery.active && recovery.verified) {
    if (output.local_path_body.size() < 2) {
      output.reason = "recovery_untrackable_path";
      recovery_follower_.stopLinear();
      follower_.stopLinear();
      setAutonomyMotionExpected(false, planning_body, timestamp_s);
      return output;
    }

    nav_kernel::FollowerParams recovery_params = config_.follower;
    recovery_params.maxSpeed = config_.recovery.translation_speed_mps;
    recovery_params.minSpeed = 0.0;
    recovery_params.yawRateGain = 0.0;
    recovery_params.stopYawRateGain = 0.0;
    recovery_params.maxYawRateRadS = 0.0;
    recovery_params.twoWayDrive = false;
    recovery_params.headingAlignEnterRad = M_PI + 0.1;
    recovery_params.headingAlignExitRad = M_PI;
    recovery_params.omniDirDiffThre = M_PI + 0.1;
    recovery_params.omniDirGoalThre =
        std::max(2.0, local_planner_.params().adjacentRange);
    recovery_params.noRotAtGoal = true;

    nav_kernel::FollowerState recovery_state;
    recovery_state.requestedSpeed = 1.0;
    recovery_state.currentTime = timestamp_s;
    recovery_state.params = recovery_params;
    recovery_state.goalDistance = bodyDistance2D(output.local_path_body.back());
    recovery_state.standardPathProfile = false;
    nav_kernel::LocalPlan recovery_plan =
        nav_kernel::LocalPlan::path(output.local_path_body);
    const nav_kernel::FollowerOutput recovery_control =
        recovery_follower_.follow(recovery_plan, recovery_state);
    output.cmd_vel = recovery_control.cmd;
    output.reason = output.recovery_reason;
    follower_.stopLinear();
    setAutonomyMotionExpected(config_.recovery.translation_speed_mps > 1e-6, planning_body,
                              timestamp_s);
    return output;
  }

  if (output.recovery_reason != "inactive") {
    output.reason = output.recovery_reason;
    follower_.stopLinear();
    recovery_follower_.stopLinear();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }
  if (near_field_stop) {
    output.reason = "near_field_stop";
    follower_.stopLinear();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }
  if (!plan_ready) {
    output.reason = plan_status == nav_kernel::LocalPlanStatus::Pending
                        ? "local_plan_pending"
                        : nav_kernel::localPlanStatusName(plan_status);
    follower_.stopLinear();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }
  const auto *spline = std::get_if<nav_kernel::SplineTarget>(&plan.target());
  const bool spline_provided = spline != nullptr;
  const bool path_provided = std::holds_alternative<nav_kernel::PathTarget>(plan.target());
  if (path_provided && output.local_path_body.size() < 2) {
    output.reason = "untrackable_local_path";
    follower_.stopLinear();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }
  nav_kernel::FollowerState follower_state;
  follower_state.measuredBodyTwist =
      observation.body_velocity_valid
          ? nav_kernel::Twist{observation.body_linear_velocity.x,
                              observation.body_linear_velocity.y, observation.body_yaw_rate}
          : nav_kernel::Twist{};
  follower_state.currentTime = timestamp_s;
  if (spline_provided) {
    follower_state.vehicleRelative = planning_body.position;
    follower_state.vehicleYawRelative = planning_body.yaw;
  }
  follower_state.slowFactor = slowFactor(output.slow_down);
  follower_state.goalDistance = std::hypot(route.back().x - map_body.position.x,
                                           route.back().y - map_body.position.y);
  follower_state.params = config_.follower;
  follower_state.params.twoWayDrive = false;
  const nav_kernel::FollowerOutput control = follower_.follow(plan, follower_state);
  output.cmd_vel = control.cmd;
  output.trajectory_frozen = control.executionFrozen;
  traj_frozen_ = control.executionFrozen;
  output.reason = spline_provided ? "spline_control_ready" : "control_ready";
  const bool command_expects_motion =
      std::hypot(output.cmd_vel.vx, output.cmd_vel.vy) > 1e-6 || std::abs(output.cmd_vel.wz) > 1e-6;
  setAutonomyMotionExpected(command_expects_motion, planning_body, timestamp_s);
  return output;
}

ExecutionOutput Executor::tickIntent(const nav_kernel::Pose &odom_map_body,
                                      const nav_kernel::Twist &intent, const float *obstacle_xyzh,
                                      int obstacle_count, double timestamp_s,
                                      TraversabilityGridView traversability,
                                      ExecutionObservation observation) {
  ExecutionOutput output;
  clearRecoveryObservationWait();

  recovery_action_ = 0;
  recovery_attempt_ = -1;
  recovery_follower_.reset();
  if (!intent_mode_) {
    local_planner_.reset();
    intent_mode_ = true;
  }
  recovery_.reset();
  resetAutonomyProgress();
  const double requested_speed = std::hypot(intent.vx, intent.vy);
  if (requested_speed <= 1e-6 || config_.max_speed <= 1e-6) {
    if (intent_mode_)
      resetLocalPlanning();
    output.reason = "teleop_intent_idle";
    follower_.stopLinear();
    resetTeleopRotation();
    resetTeleopReference();
    return output;
  }

  const double speed_norm = clamp01(requested_speed / config_.max_speed);
  const double input_direction_body = std::atan2(intent.vy, intent.vx);
  if (teleop_recovery_intent_rad_.has_value() &&
      std::abs(nav_kernel::normalizeAngle(
          input_direction_body - *teleop_recovery_intent_rad_)) >
          kTeleopIntentToleranceRad) {
    resetTeleopRotation();
  }
  const double requested_horizon = std::max(0.5, config_.teleop_intent_horizon_m);
  const double configured_horizon = requested_horizon;
  const bool teleop_direction_changed =
      teleop_reference_.has_value() &&
      std::abs(nav_kernel::normalizeAngle(
          input_direction_body - teleop_reference_->directionBody)) >
          kTeleopIntentToleranceRad;
  if (!teleop_reference_.has_value() || teleop_direction_changed) {
    if (teleop_direction_changed) {
      resetLocalPlanning();
      intent_mode_ = true;
    }
    ++generation;
    follower_.resetIntent();
    teleop_reference_ = TeleopReference{
        odom_map_body.position,
        nav_kernel::normalizeAngle(odom_map_body.yaw + input_direction_body),
        input_direction_body,
    };
    resetTeleopRotation();
  }

  const double reference_c = std::cos(teleop_reference_->headingMap);
  const double reference_s = std::sin(teleop_reference_->headingMap);
  const double from_origin_x = odom_map_body.position.x - teleop_reference_->origin.x;
  const double from_origin_y = odom_map_body.position.y - teleop_reference_->origin.y;
  const double along = from_origin_x * reference_c + from_origin_y * reference_s;

  output.active = true;
  output.target = {
      teleop_reference_->origin.x + (along + configured_horizon) * reference_c,
      teleop_reference_->origin.y + (along + configured_horizon) * reference_s,
      odom_map_body.position.z,
  };
  const double target_map_x = output.target.x - odom_map_body.position.x;
  const double target_map_y = output.target.y - odom_map_body.position.y;
  const double body_c = std::cos(odom_map_body.yaw);
  const double body_s = std::sin(odom_map_body.yaw);
  const nav_kernel::Vec3 target_body{
      body_c * target_map_x + body_s * target_map_y,
      -body_s * target_map_x + body_c * target_map_y,
      0.0,
  };
  const double planning_horizon =
      std::max(0.05, std::hypot(target_map_x, target_map_y));
  const double planning_direction_body = std::atan2(target_body.y, target_body.x);
  output.target_distance_m = planning_horizon;

  const std::vector<nav_kernel::Vec3> intent_route{odom_map_body.position, output.target};
  const nav_kernel::LocalKinematicState kinematics =
      planningKinematics(odom_map_body, observation, timestamp_s);
  const nav_kernel::LocalMotionIntent motion_intent{
      planning_direction_body * 180.0 / M_PI,
      speed_norm,
      planning_horizon,
      config_.teleop_intent_max_deviation_deg,
  };
  const nav_kernel::LocalPlanRequest plan_request = makeLocalPlanRequest(
      odom_map_body, intent_route, generation, false, kinematics, observation,
      obstacle_xyzh, obstacle_count, timestamp_s,
      traj_frozen_, traversability, &motion_intent);

  if (teleop_recovery_.active()) {
    const RecoveryOutput recovery = teleop_recovery_.step(plan_request);
    applyTeleopRotation(output, recovery, config_.recovery.rotation_rate_rad_s);
    output.recovery_trigger = "active";
    follower_.stopLinear();
    traj_frozen_ = false;
    return output;
  }

  const MapFromOdomTransform identity_transform{};
  nav_kernel::LocalPlan plan =
      planLocal(plan_request, identity_transform, &output.local_planner_debug);

  const nav_kernel::LocalPlanStatus plan_status = plan.status();
  const bool plan_ready = plan.ready();
  const bool near_field_stop = plan_status == nav_kernel::LocalPlanStatus::NearFieldStop;
  const auto *spline = std::get_if<nav_kernel::SplineTarget>(&plan.target());
  const bool spline_provided = spline != nullptr;
  const bool path_provided = std::holds_alternative<nav_kernel::PathTarget>(plan.target());
  output.path_found = plan_ready;
  output.slow_down = std::clamp(plan.hints().slowdownLevel, 0, 3);
  output.recovery_state = 0;
  output.local_path_body =
      spline_provided ? planningPathToBody(odom_map_body, plan.previewPath())
                      : plan.previewPath();
  if (plan_ready && !near_field_stop && output.local_path_body.size() < 2) {
    if (output.local_path_body.size() == 1 &&
        bodyDistance2D(output.local_path_body.front()) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
  }
  output.local_path_map = bodyPathToMap(odom_map_body, output.local_path_body);

  const bool spline_trackable = !spline_provided || plan_ready;
  const bool path_trackable = spline_provided || output.local_path_body.size() >= 2;
  const bool planner_input_valid =
      plan_status != nav_kernel::LocalPlanStatus::InvalidInput &&
      plan_status != nav_kernel::LocalPlanStatus::NotConfigured;
  const bool plan_unusable =
      !plan_ready || near_field_stop || !spline_trackable || !path_trackable;
  const bool recovery_needed =
      planner_input_valid && plan_status != nav_kernel::LocalPlanStatus::Pending && plan_unusable;
  if (recovery_needed) {
    teleop_recovery_intent_rad_ = input_direction_body;
    const RecoveryOutput recovery = teleop_recovery_.step(plan_request);
    applyTeleopRotation(output, recovery, config_.recovery.rotation_rate_rad_s);
    output.recovery_trigger = "blocked";
    follower_.stopLinear();
    traj_frozen_ = false;
    return output;
  }

  if (!plan_ready) {
    const bool pending = plan_status == nav_kernel::LocalPlanStatus::Pending;
    output.near_field_stop = near_field_stop || pending;
    output.reason = !output.local_planner_debug.searchReason.empty()
                        ? output.local_planner_debug.searchReason
                        : (pending ? "local_intent_pending" : "teleop_assist_no_path");
    follower_.stopLinear();
    return output;
  }
  if (path_provided && output.local_path_body.size() < 2) {
    output.near_field_stop = near_field_stop;
    output.reason = "teleop_assist_untrackable_path";
    follower_.stopLinear();
    return output;
  }

  resetTeleopRotation();

  nav_kernel::FollowerState follower_state;
  follower_state.requestedSpeed = speed_norm;
  follower_state.measuredBodyTwist =
      observation.body_velocity_valid
          ? nav_kernel::Twist{observation.body_linear_velocity.x,
                              observation.body_linear_velocity.y, observation.body_yaw_rate}
          : nav_kernel::Twist{};
  follower_state.currentTime = timestamp_s;
  if (spline_provided) {
    follower_state.vehicleRelative = odom_map_body.position;
    follower_state.vehicleYawRelative = odom_map_body.yaw;
  }
  follower_state.slowFactor = slowFactor(output.slow_down);
  follower_state.params = config_.follower;
  const nav_kernel::FollowerOutput control = follower_.follow(plan, follower_state);
  output.cmd_vel = control.cmd;
  output.trajectory_frozen = control.executionFrozen;
  traj_frozen_ = control.executionFrozen;
  output.near_field_stop = false;
  output.reason = control.executionFrozen
                      ? (control.directionTransition
                             ? "teleop_assist_direction_transition"
                             : "teleop_assist_heading_alignment")
                      : (spline_provided ? "teleop_assist_spline_ready"
                                         : "teleop_assist_control_ready");
  return output;
}

double Executor::slowFactor(int slow_down) const {
  switch (std::max(0, std::min(3, slow_down))) {
    case 1:
      return clamp01(config_.slow_rate_1);
    case 2:
      return clamp01(config_.slow_rate_2);
    case 3:
      return clamp01(config_.slow_rate_3);
    default:
      return 1.0;
  }
}

}  // namespace lingtu::nav::navigation
