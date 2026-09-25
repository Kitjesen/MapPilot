#include "navigation/executor.hpp"
#include "planning/local/scan/grid.hpp"
#include "planning/local/scan/upstream/plan_manage/closed_loop_controller.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
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

std::vector<nav_kernel::Vec3> planningPathToMap(
    const lingtu::nav::navigation::MapFromOdomTransform &mapFromOdom,
    const std::vector<nav_kernel::Vec3> &planningPath) {
  std::vector<nav_kernel::Vec3> result;
  result.reserve(planningPath.size());
  for (const auto &point : planningPath)
    result.push_back(mapFromOdom.mapPointFromOdom(point));
  return result;
}


nav_kernel::LocalPlanRequest makeLocalPlanRequest(
    const nav_kernel::Pose &vehicle, const std::vector<nav_kernel::Vec3> &route,
    const std::vector<nav_kernel::Vec3> *reference_route,
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
  if (reference_route != nullptr && reference_route->size() >= 2U) {
    request.reference = {
        reference_route->data(),
        static_cast<int>(reference_route->size()),
        route_generation,
        true,
    };
  }
  request.identity = {
      observation.frame_epoch,
      observation.collision.present() ? observation.collision.generation
                                      : observation.cloud_generation,
      observation.traversability_generation,
  };
  request.environment.obstacles = {obstacle_xyzh, obstacle_count};
  request.environment.collision = observation.collision;
  request.environment.predictions = observation.predictions;
  request.clock = {timestamp_s, execution_frozen, observation.clock_mode};
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
  active_max_speed_mps_ = config_.max_speed;
  active_goal_height_tolerance_m_ = config_.goal_height_tolerance_m;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
}

void Executor::resetLocalPlanning() {
  local_planner_.reset();
  follower_.resetTarget();
  last_scan_tick_s_.reset();
  traj_frozen_ = false;
  intent_mode_ = false;
}

void Executor::reportFinalMotionBlocked(bool blocked, double timestamp_s) {
  if (!blocked) {
    final_motion_blocked_since_s_ = -1.0;
  } else if (final_motion_blocked_since_s_ < 0.0 ||
             final_motion_blocked_since_s_ > timestamp_s) {
    final_motion_blocked_since_s_ = timestamp_s;
  }
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
  activateRoute(value.points, value.finalYaw, value.goalToleranceM, value.yawToleranceRad, value.maxSpeedMps);
}

void Executor::clear() {
  clearRoute();
}

void Executor::activateRoute(const std::vector<nav_kernel::Vec3> &path,
                             std::optional<double> final_yaw,
                             std::optional<double> goal_reached_m,
                             std::optional<double> goal_yaw_tolerance_rad,
                             std::optional<double> max_speed_mps) {
  resetDynamicAvoidance();
  if (max_speed_mps && (!std::isfinite(*max_speed_mps) || *max_speed_mps <= 0.0))
    throw std::invalid_argument("route speed limit must be positive and finite");
  active_max_speed_mps_ = std::min(config_.max_speed, max_speed_mps.value_or(config_.max_speed));
  route = path;
  autonomy_stall_stop_ = false;
  ++generation;
  final_yaw_ = final_yaw;
  goal_quiet_since_s_ = goal_last_odom_s_ = -1.0;
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
  reference.clear();
  committed_local_path_map_.clear();
  resetTeleopRotation();
  resetTeleopReference();
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  final_motion_blocked_since_s_ = -1.0;
  resetAutonomyProgress();
}

void Executor::clearRoute() {
  resetDynamicAvoidance();
  goal_quiet_since_s_ = goal_last_odom_s_ = -1.0;
  route.clear();
  autonomy_stall_stop_ = false;
  ++generation;
  final_yaw_.reset();
  height_offset_.reset();
  active_goal_reached_m_ = config_.goal_reached_m;
  active_max_speed_mps_ = config_.max_speed;
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
  reference.clear();
  committed_local_path_map_.clear();
  resetTeleopRotation();
  resetTeleopReference();
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  final_motion_blocked_since_s_ = -1.0;
  clearRecoveryObservationWait();
  resetAutonomyProgress();
}

void Executor::resetDynamicAvoidance() {
  dynamic_wait_since_s_ = dynamic_clear_since_s_ = dynamic_progress_s_ = -1.0;
  dynamic_resuming_ = false;
}

bool Executor::hasRoute() const {
  return !route.empty();
}

void Executor::suspendAutonomy() {
  resetDynamicAvoidance();
  goal_quiet_since_s_ = goal_last_odom_s_ = -1.0;
  follower_.reset();
  recovery_follower_.reset();
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  resetLocalPlanning();
  recovery_.reset();
  reference.clear();
  committed_local_path_map_.clear();
  resetTeleopRotation();
  resetTeleopReference();
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  final_motion_blocked_since_s_ = -1.0;
  clearRecoveryObservationWait();
  resetAutonomyProgress();
}

void Executor::pauseLinearMotion() {
  traj_frozen_ = true;
  local_planner_.pause();
  follower_.stopLinear();
  recovery_follower_.stopLinear();
}

void Executor::replanTeleop() {
  resetLocalPlanning();
  intent_mode_ = true;
  resetTeleopBoundaryDeparture();
}

void Executor::stopLinearMotion() {
  resetLocalPlanning();
  resetTeleopBoundaryDeparture();
  resetTeleopRotation();
  resetTeleopReference();
}

void Executor::resetTeleopRotation() {
  teleop_recovery_.reset();
  teleop_recovery_intent_rad_.reset();
}

void Executor::resetTeleopBoundaryDeparture() {
  recovery_follower_.reset();
  teleop_boundary_departure_intent_rad_.reset();
}

void Executor::resetTeleopReference() {
  teleop_reference_.reset();
}


ExecutionOutput Executor::tick(const ExecutionInput &input) {
  if (local_planner_.params().backend == nav_kernel::LocalPlannerBackend::Scan) {
    if (last_scan_tick_s_) {
      const double elapsed = input.timestampS - *last_scan_tick_s_;
      const double max_gap =
          nav_kernel::local::scan::upstream::ClosedLoopController::kMaxUpdateGapS;
      if (elapsed < 0.0 || (!traj_frozen_ && elapsed > max_gap)) {
        if (elapsed >= 0.0 && dynamic_wait_since_s_ >= 0.0) {
          // A delayed control tick invalidates the spline, not the budget for
          // an unresolved dynamic encounter. Otherwise repeated gaps can make
          // a stopped robot retry forever without measured progress.
          resetLocalPlanning();
          pauseLinearMotion();
          previous_kinematics_time_s_ = -1.0;
          dynamic_clear_since_s_ = -1.0;
          dynamic_resuming_ = false;
        } else {
          suspendAutonomy();
        }
        last_scan_tick_s_ = input.timestampS;
        ExecutionOutput output;
        output.active = input.mode == ExecutionMode::MotionIntent || !route.empty();
        output.near_field_stop = output.active;
        output.reason = "scan_execution_clock_discontinuity";
        if (dynamic_wait_since_s_ >= 0.0) output.dynamic_avoidance = "stale";
        return output;
      }
    }
    last_scan_tick_s_ = input.timestampS;
  }
  if (input.mode == ExecutionMode::MotionIntent) {
    return tickIntent(input.mapBody, input.motionIntent, input.obstacleXyzhMap,
                      input.obstacleCount, input.timestampS, input.traversability,
                      input.observation);
  }
  auto output = tickRoute(input.mapBody, input.odomBody, input.mapFromOdom,
                   input.obstacleXyzhMap, input.obstacleCount, input.timestampS,
                   input.traversability, input.observation);
  const double speed = std::hypot(output.cmd_vel.vx, output.cmd_vel.vy);
  if (speed > active_max_speed_mps_ && speed > 0.0) {
    output.cmd_vel.vx *= active_max_speed_mps_ / speed;
    output.cmd_vel.vy *= active_max_speed_mps_ / speed;
  }
  return output;
}

ExecutionOutput Executor::tickRoute(const nav_kernel::Pose &map_body,
                                    const nav_kernel::Pose &odom_body,
                                    const MapFromOdomTransform &map_from_odom,
                                    const float *obstacle_xyzh_map, int obstacle_count,
                                    double timestamp_s, TraversabilityGridView odom_traversability,
                                    ExecutionObservation observation) {
  if (config_.planning_frame == PlanningFrame::Odom && !map_from_odom.valid()) {
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
    resetAutonomyProgress();
    return output;
  }

  const int safe_obstacle_count =
      obstacle_xyzh_map == nullptr ? 0 : std::max(0, obstacle_count);
  if (config_.planning_frame == PlanningFrame::Map) {
    if (reference_epoch && *reference_epoch != observation.frame_epoch) {
      suspendAutonomy();
      height_offset_.reset();
    }
    reference_epoch = observation.frame_epoch;
    // The occupancy layer and complete reference are already map-frame data.
    // The optional 2D traversability view is odom-only; do not relabel it.
    observation.collision.gridFromPlanningTranslation = {};
    observation.collision.gridFromPlanningYaw = 0.0;
    return tickInPlanningFrame(map_body, map_body, {}, obstacle_xyzh_map,
                               safe_obstacle_count, timestamp_s, {}, observation);
  }
  obstacle_xyzh_odom_scratch_.clear();
  predictions_odom_scratch_.clear();
  if (observation.predictions.fresh(timestamp_s)) {
    for (std::size_t i = 0; i < observation.predictions.count; ++i) {
      auto prediction = observation.predictions.obstacles[i];
      prediction.start = map_from_odom.odomPointFromMap(prediction.start);
      prediction.end = map_from_odom.odomPointFromMap(prediction.end);
      prediction.minZ -= map_from_odom.translation.z;
      prediction.maxZ -= map_from_odom.translation.z;
      predictions_odom_scratch_.push_back(prediction);
    }
  }
  observation.predictions.obstacles = predictions_odom_scratch_.data();
  observation.predictions.count = predictions_odom_scratch_.size();
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
      goal_quiet_since_s_ = goal_last_odom_s_ = -1.0;
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
    output.target_index = route.size() - 1;
    output.target = route.back();
    output.target.z += height_offset_.value_or(0.0);
    output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
    follower_.reset();
    // Entering the radius while braking is not a settled arrival. Keep issuing
    // zero and re-evaluate position on fresh odometry before committing success.
    const double stamp = observation.odom_stamp_s;
    const double speed = std::hypot(observation.body_linear_velocity.x,
                                    observation.body_linear_velocity.y,
                                    observation.body_linear_velocity.z);
    const bool quiet = observation.body_velocity_valid && std::isfinite(speed) &&
                       speed <= 0.03 && std::isfinite(observation.body_yaw_rate) &&
                       std::abs(observation.body_yaw_rate) <= 0.08 &&
                       std::isfinite(stamp) && stamp > 0.0;
    if (!quiet) {
      goal_quiet_since_s_ = goal_last_odom_s_ = -1.0;
    } else {
      if (goal_quiet_since_s_ < 0.0 || stamp < goal_last_odom_s_ ||
          stamp - goal_last_odom_s_ > 0.25) {
        goal_quiet_since_s_ = stamp;
      }
      goal_last_odom_s_ = stamp;
    }
    output.goal_reached = quiet && stamp - goal_quiet_since_s_ >= 0.5;
    output.active = !output.goal_reached;
    output.path_found = true;
    output.reason = output.goal_reached ? "goal_reached" : "settling_at_goal";
    return output;
  }

  goal_quiet_since_s_ = goal_last_odom_s_ = -1.0;

  const SegmentTarget target = buildSegment(map_body, planning_body, map_from_odom);
  buildReference(target, map_from_odom);
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
      resetAutonomyProgress();
      return output;
    }
  }

  const nav_kernel::LocalKinematicState kinematics =
      planningKinematics(planning_body, observation, timestamp_s);
  nav_kernel::LocalPlanRequest plan_request = makeLocalPlanRequest(
      planning_body, segment, &reference, generation, target.reachesGoal, kinematics, observation,
      obstacle_xyzh_planning, obstacle_count, timestamp_s,
      traj_frozen_, traversability);
  plan_request.maxLinearSpeedMps = std::min(
      active_max_speed_mps_, std::max(config_.follower.spline.maxVx, config_.follower.spline.maxVy));
  bool dynamic_episode = false;
  if (local_planner_.params().backend == nav_kernel::LocalPlannerBackend::Scan) {
    const nav_kernel::local::scan::Grid grid(local_planner_.params(), plan_request);
    output.prediction_count = grid.predictionCount();
    bool conflict = false;
    auto from = planning_body.position;
    double remaining = std::max(0.5, active_max_speed_mps_ +
        active_max_speed_mps_ * active_max_speed_mps_ /
            (2.0 * std::max(0.05, local_planner_.params().scan.maxAcceleration)));
    for (const auto &point : segment) {
      const double length = nav_kernel::distance3D(from, point);
      if (length < 1e-6) continue;
      const double fraction = std::min(1.0, remaining / length);
      const nav_kernel::Vec3 to{from.x+(point.x-from.x)*fraction,
                               from.y+(point.y-from.y)*fraction,
                               from.z+(point.z-from.z)*fraction};
      const double heading = std::atan2(to.y-from.y, to.x-from.x);
      conflict = conflict || grid.predictionIntersects(from, heading, to, heading);
      remaining -= length;
      from = to;
      if (remaining <= 0.0) break;
    }
    if (conflict) {
      dynamic_clear_since_s_ = -1.0;
      dynamic_resuming_ = false;
      if (dynamic_wait_since_s_ < 0.0 || timestamp_s < dynamic_wait_since_s_) {
        dynamic_wait_since_s_ = dynamic_progress_s_ = timestamp_s;
        dynamic_progress_position_ = planning_body.position;
        resetLocalPlanning();
      }
    } else if (dynamic_wait_since_s_ >= 0.0 && observation.predictions.fresh(timestamp_s)) {
      if (dynamic_clear_since_s_ < 0.0) dynamic_clear_since_s_ = timestamp_s;
      if (!dynamic_resuming_ &&
          timestamp_s-dynamic_clear_since_s_ >= config_.dynamic_clear_s) {
        // Clear predictions allow replanning, but do not prove the robot has
        // escaped stale occupancy or resumed motion. Keep the original budget
        // until odometry demonstrates progress after the clear observation.
        dynamic_resuming_ = true;
        dynamic_progress_position_ = planning_body.position;
        resetLocalPlanning();
      }
    } else if (!observation.predictions.fresh(timestamp_s)) {
      dynamic_clear_since_s_ = -1.0;
      dynamic_resuming_ = false;
    }
    if (dynamic_resuming_ && observation.predictions.fresh(timestamp_s) &&
        nav_kernel::distance2D(planning_body.position, dynamic_progress_position_) >= 0.1) {
      resetDynamicAvoidance();
    }
    dynamic_episode = dynamic_wait_since_s_ >= 0.0;
    if (dynamic_episode) {
      recovery_.reset();
      recovery_follower_.stopLinear();
      local_blocked_since_s_ = final_motion_blocked_since_s_ = -1.0;
      if (nav_kernel::distance2D(planning_body.position, dynamic_progress_position_) >= 0.1) {
        dynamic_progress_position_ = planning_body.position;
        dynamic_progress_s_ = timestamp_s;
      }
      output.dynamic_blocked_s = std::max(0.0, timestamp_s-dynamic_progress_s_);
      const bool expired = output.dynamic_blocked_s >= config_.dynamic_blocked_timeout_s ||
                           timestamp_s-dynamic_wait_since_s_ >= config_.dynamic_episode_timeout_s;
      const bool stale = !observation.predictions.fresh(timestamp_s);
      const bool wait = expired || stale || (!dynamic_resuming_ &&
                       (!conflict || timestamp_s-dynamic_wait_since_s_ < config_.dynamic_wait_s));
      output.dynamic_avoidance = expired ? "timeout" : stale ? "stale" : wait ? "waiting"
                                       : dynamic_resuming_ ? "resuming" : "detour";
      if (wait) {
        output.reason = expired ? (dynamic_resuming_ ? "dynamic_resume_timeout"
                                                     : "dynamic_obstacle_timeout")
                                : stale ? "dynamic_prediction_stale" : "dynamic_obstacle_wait";
        output.recovery_reason = output.reason;
        output.recovery_exhausted = expired;
        output.near_field_stop = true;
        follower_.stopLinear();
        resetAutonomyProgress();
        return output;
      }
    }
  }
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
      (local_blocked_since_s_ >= 0.0 &&
       timestamp_s - local_blocked_since_s_ >= std::max(0.0, config_.recovery.blocked_interval_s)) ||
      (final_motion_blocked_since_s_ >= 0.0 &&
       timestamp_s - final_motion_blocked_since_s_ >=
           std::max(0.0, config_.recovery.blocked_interval_s));
  const bool recovery_enabled = config_.recovery.max_attempts > 0;
  const bool recovery_active = recovery_.active();
  const bool stalled = !dynamic_episode && !recovery_active && !blocked_long_enough &&
                       autonomyMotionStalled(planning_body, timestamp_s);
  if (stalled && !recovery_enabled) autonomy_stall_stop_ = true;
  output.recovery_trigger = recovery_active
                                ? "active"
                                : (blocked_long_enough ? "blocked"
                                                       : (stalled ? "stalled" : "inactive"));
  const bool force_recovery = !dynamic_episode && recovery_enabled &&
                              (recovery_active || blocked_long_enough || stalled);
  if (dynamic_episode && !plan_ready) {
    output.reason = dynamic_resuming_ ? "dynamic_resume_pending" : "dynamic_obstacle_replan";
    output.dynamic_avoidance = dynamic_resuming_ ? "resuming" : "waiting_for_detour";
    output.near_field_stop = true;
    follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }
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
  const bool spline_provided =
      std::holds_alternative<nav_kernel::SplineTarget>(plan.target());
  const auto &preview_path = plan.previewPath();
  output.local_path_body = recovery.active
                               ? recovery.path_body
                               : (spline_provided
                                      ? planningPathToBody(planning_body, preview_path)
                                      : preview_path);
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
  output.local_path_map = !recovery.active && spline_provided
                              ? planningPathToMap(map_from_odom, preview_path)
                              : bodyPathToMap(map_body, output.local_path_body);
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

  if (autonomy_stall_stop_) {
    output.reason = "autonomy_motion_stalled";
    output.recovery_trigger = "stalled";
    output.near_field_stop = true;
    output.trajectory_frozen = true;
    pauseLinearMotion();
    return output;
  }
  if (recovery.exhausted) {
    output.reason = "local_recovery_exhausted";
    follower_.stopLinear();
    recovery_follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }

  if (recovery.active && recovery.verified && recovery.direct_command) {
    output.cmd_vel.wz =
        static_cast<double>(recovery.rotation_direction) * config_.recovery.rotation_rate_rad_s;
    output.reason = output.recovery_reason;
    follower_.stopLinear();
    setAutonomyMotionExpected(output.cmd_vel, planning_body, timestamp_s);
    return output;
  }

  if (recovery.active && recovery.verified) {
    if (output.local_path_body.size() < 2) {
      output.reason = "recovery_untrackable_path";
      recovery_follower_.stopLinear();
      follower_.stopLinear();
      resetAutonomyProgress();
      return output;
    }

    nav_kernel::FollowerParams recovery_params = config_.follower;
    recovery_params.maxSpeed = config_.recovery.translation_speed_mps;
    recovery_params.minSpeed = 0.0;
    // Recovery completes within 0.125 m. A wider normal-goal stopping radius
    // would stop the follower before Recovery can acknowledge completion.
    recovery_params.stopDisThre = 0.08;
    recovery_params.slowDwnDisThre = 0.0;
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
    setAutonomyMotionExpected(output.cmd_vel, planning_body, timestamp_s);
    return output;
  }

  if (output.recovery_reason != "inactive") {
    output.reason = output.recovery_reason;
    follower_.stopLinear();
    recovery_follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }
  if (near_field_stop) {
    output.reason = "near_field_stop";
    follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }
  if (!plan_ready) {
    output.reason = plan_status == nav_kernel::LocalPlanStatus::Pending
                        ? "local_plan_pending"
                        : local_planner_.params().backend == nav_kernel::LocalPlannerBackend::Scan &&
                                  !output.local_planner_debug.searchReason.empty()
                              ? output.local_planner_debug.searchReason
                              : nav_kernel::localPlanStatusName(plan_status);
    follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }
  const bool path_provided = std::holds_alternative<nav_kernel::PathTarget>(plan.target());
  if (path_provided && output.local_path_body.size() < 2) {
    output.reason = "untrackable_local_path";
    follower_.stopLinear();
    resetAutonomyProgress();
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
  if (path_provided) {
    follower_state.params = nav_kernel::cmuFollowerParams(follower_state.params);
    follower_state.standardPathProfile = false;
  }
  // The follower must be able to enter a task's tighter arrival radius.
  follower_state.params.stopDisThre =
      std::min(follower_state.params.stopDisThre, active_goal_reached_m_);
  follower_state.params.spline.finishDistance =
      std::min(follower_state.params.spline.finishDistance, active_goal_reached_m_);
  follower_state.params.maxSpeed = active_max_speed_mps_;
  follower_state.params.minSpeed = std::min(follower_state.params.minSpeed, active_max_speed_mps_);
  follower_state.params.spline.maxVx = std::min(follower_state.params.spline.maxVx, active_max_speed_mps_);
  follower_state.params.spline.maxVy = std::min(follower_state.params.spline.maxVy, active_max_speed_mps_);
  follower_state.params.twoWayDrive = false;
  const nav_kernel::FollowerOutput control = follower_.follow(plan, follower_state);
  output.cmd_vel = control.cmd;
  output.tracking = control.tracking;
  output.trajectory_frozen = control.executionFrozen;
  traj_frozen_ = control.executionFrozen;
  output.reason = spline_provided
                      ? (control.awaitingTrajectory ? "spline_speed_replan"
                         : control.executionFrozen ? "spline_execution_frozen" : "spline_control_ready")
                      : "control_ready";
  setAutonomyMotionExpected(output.cmd_vel, planning_body, timestamp_s);
  return output;
}

ExecutionOutput Executor::tickIntent(const nav_kernel::Pose &odom_map_body,
                                      const nav_kernel::Twist &intent, const float *obstacle_xyzh,
                                      int obstacle_count, double timestamp_s,
                                      TraversabilityGridView traversability,
                                      ExecutionObservation observation) {
  ExecutionOutput output;
  autonomy_stall_stop_ = false;
  clearRecoveryObservationWait();

  recovery_action_ = 0;
  recovery_attempt_ = -1;
  if (!intent_mode_) {
    local_planner_.reset();
    resetTeleopBoundaryDeparture();
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
    resetTeleopBoundaryDeparture();
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
  const bool boundary_departure_direction_changed =
      teleop_direction_changed && teleop_boundary_departure_intent_rad_.has_value();
  const bool operator_turning = std::abs(intent.wz) > 1e-6;
  const double heading_map =
      nav_kernel::normalizeAngle(odom_map_body.yaw + input_direction_body);
  bool steering_changed = false;
  if (teleop_reference_.has_value()) {
    const double heading_change = std::abs(nav_kernel::normalizeAngle(
        heading_map - teleop_reference_->headingMap));
    // Re-anchor only for operator steering, not planner-induced detour yaw.
    // Keep reference segments stable while turning, then latch the release pose.
    steering_changed =
        (operator_turning && heading_change > kTeleopIntentToleranceRad) ||
        (!operator_turning && teleop_reference_->operatorTurning && heading_change > 1e-6);
  }
  if (!teleop_reference_.has_value() || teleop_direction_changed || steering_changed) {
    if (teleop_direction_changed) {
      resetLocalPlanning();
      intent_mode_ = true;
    }
    ++generation;
    if (!teleop_reference_.has_value() || teleop_direction_changed)
      follower_.resetIntent();
    const double heading_c = std::cos(heading_map);
    const double heading_s = std::sin(heading_map);
    teleop_reference_ = TeleopReference{
        odom_map_body.position,
        odom_map_body.position,
        {odom_map_body.position.x + configured_horizon * heading_c,
         odom_map_body.position.y + configured_horizon * heading_s,
         odom_map_body.position.z},
        heading_map,
        input_direction_body,
    };
    resetTeleopBoundaryDeparture();
    resetTeleopRotation();
  }
  teleop_reference_->operatorTurning = operator_turning;

  if (boundary_departure_direction_changed) {
    output.active = true;
    output.reason = "teleop_intent_direction_changed";
    follower_.stopLinear();
    return output;
  }

  const double reference_c = std::cos(teleop_reference_->headingMap);
  const double reference_s = std::sin(teleop_reference_->headingMap);
  const double from_origin_x =
      odom_map_body.position.x - teleop_reference_->corridorOrigin.x;
  const double from_origin_y =
      odom_map_body.position.y - teleop_reference_->corridorOrigin.y;
  const double along = from_origin_x * reference_c + from_origin_y * reference_s;
  const nav_kernel::Vec3 desired_target{
      teleop_reference_->corridorOrigin.x +
          (along + configured_horizon) * reference_c,
      teleop_reference_->corridorOrigin.y +
          (along + configured_horizon) * reference_s,
      odom_map_body.position.z,
  };
  const double reference_advance_m =
      std::max(0.5, 0.25 * configured_horizon);
  const double target_advance_m = std::hypot(
      desired_target.x - teleop_reference_->guideTarget.x,
      desired_target.y - teleop_reference_->guideTarget.y);
  if (target_advance_m >= reference_advance_m) {
    teleop_reference_->guideStart = odom_map_body.position;
    teleop_reference_->guideTarget = desired_target;
    ++generation;
  }

  output.active = true;
  output.target = teleop_reference_->guideTarget;
  const double target_map_x = output.target.x - odom_map_body.position.x;
  const double target_map_y = output.target.y - odom_map_body.position.y;
  const double planning_horizon =
      std::max(0.05, std::hypot(target_map_x, target_map_y));
  output.target_distance_m = planning_horizon;

  const std::vector<nav_kernel::Vec3> intent_route{
      teleop_reference_->guideStart,
      teleop_reference_->guideTarget,
  };
  const nav_kernel::LocalKinematicState kinematics =
      planningKinematics(odom_map_body, observation, timestamp_s);
  const nav_kernel::LocalMotionIntent motion_intent{
      teleop_reference_->directionBody * 180.0 / M_PI,
      speed_norm,
      configured_horizon,
      config_.teleop_intent_max_deviation_deg,
  };
  nav_kernel::LocalPlanRequest plan_request = makeLocalPlanRequest(
      odom_map_body, intent_route, nullptr, generation, false, kinematics, observation,
      obstacle_xyzh, obstacle_count, timestamp_s,
      traj_frozen_, traversability, &motion_intent);
  plan_request.maxLinearSpeedMps = std::min(
      std::min(requested_speed, config_.max_speed),
      std::max(config_.follower.spline.maxVx, config_.follower.spline.maxVy));

  if (teleop_recovery_.active()) {
    resetTeleopBoundaryDeparture();
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
  const auto &preview_path = plan.previewPath();
  output.local_path_body = spline_provided
                               ? planningPathToBody(odom_map_body, preview_path)
                               : preview_path;
  if (plan_ready && !near_field_stop && output.local_path_body.size() < 2) {
    if (output.local_path_body.size() == 1 &&
        bodyDistance2D(output.local_path_body.front()) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
  }
  output.local_path_map = spline_provided
                              ? preview_path
                              : bodyPathToMap(odom_map_body, output.local_path_body);

  const bool spline_trackable = !spline_provided || plan_ready;
  const bool path_trackable = spline_provided || output.local_path_body.size() >= 2;
  const bool planner_input_valid =
      plan_status != nav_kernel::LocalPlanStatus::InvalidInput &&
      plan_status != nav_kernel::LocalPlanStatus::NotConfigured;
  const bool plan_unusable =
      !plan_ready || near_field_stop || !spline_trackable || !path_trackable;
  const bool scan_boundary_departure_candidate =
      local_planner_.params().backend == nav_kernel::LocalPlannerBackend::Scan &&
      planner_input_valid && plan_status != nav_kernel::LocalPlanStatus::Pending &&
      plan_unusable && config_.recovery.translation_speed_mps > 1e-6 &&
      std::abs(intent.wz) <= 1e-6 && observation.collision.valid();
  if (scan_boundary_departure_candidate) {
    nav_kernel::local::scan::Grid grid(local_planner_.params(), plan_request);
    constexpr double departure_distance_m = 0.35;
    const std::vector<nav_kernel::Vec3> departure_body{
        {},
        {departure_distance_m * std::cos(input_direction_body),
         departure_distance_m * std::sin(input_direction_body), 0.0},
    };
    auto departure_map = bodyPathToMap(odom_map_body, departure_body);
    const bool occupied_start =
        grid.valid() && grid.inflatedOccupancy(odom_map_body.position, odom_map_body.yaw) == 1;
    if (occupied_start &&
        grid.boundaryDepartureFree(odom_map_body, departure_map.back())) {
      if (!teleop_boundary_departure_intent_rad_.has_value()) {
        recovery_follower_.reset();
      }
      teleop_boundary_departure_intent_rad_ = input_direction_body;
      output.path_found = true;
      output.near_field_stop = false;
      output.local_path_body = departure_body;
      output.local_path_map = std::move(departure_map);
      output.target = output.local_path_map.back();
      output.target_distance_m = departure_distance_m;
      output.recovery_state = 2;
      output.recovery_action = static_cast<int>(nav_kernel::RecoveryAction::Translate);
      output.recovery_candidate_count = 1;
      output.recovery_verified = true;
      output.recovery_trigger = "blocked";
      output.recovery_reason = "scan_boundary_departure";

      nav_kernel::FollowerParams recovery_params = config_.follower;
      recovery_params.maxSpeed =
          std::min(config_.recovery.translation_speed_mps, plan_request.maxLinearSpeedMps);
      recovery_params.minSpeed = 0.0;
      recovery_params.stopDisThre = 0.08;
      recovery_params.slowDwnDisThre = 0.0;
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
      recovery_state.goalDistance = departure_distance_m;
      recovery_state.standardPathProfile = false;
      const nav_kernel::FollowerOutput recovery_control = recovery_follower_.follow(
          nav_kernel::LocalPlan::path(output.local_path_body), recovery_state);
      output.cmd_vel = recovery_control.cmd;
      output.tracking = recovery_control.tracking;
      output.trajectory_frozen = recovery_control.executionFrozen;
      output.reason = output.recovery_reason;
      follower_.stopLinear();
      traj_frozen_ = recovery_control.executionFrozen;
      return output;
    }
  }
  resetTeleopBoundaryDeparture();
  const bool recovery_needed =
      local_planner_.params().backend != nav_kernel::LocalPlannerBackend::Scan &&
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
  resetTeleopBoundaryDeparture();

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
    follower_state.desiredHeading = nav_kernel::normalizeAngle(
        teleop_reference_->headingMap - teleop_reference_->directionBody);
    if (std::abs(intent.wz) > 1e-6 && config_.follower.spline.yawGain > 0.0) {
      follower_state.desiredHeading = nav_kernel::normalizeAngle(
          odom_map_body.yaw + intent.wz / config_.follower.spline.yawGain);
    }
  }
  follower_state.slowFactor = slowFactor(output.slow_down);
  follower_state.params = config_.follower;
  const nav_kernel::FollowerOutput control = follower_.follow(plan, follower_state);
  output.cmd_vel = control.cmd;
  output.tracking = control.tracking;
  output.trajectory_frozen = control.executionFrozen;
  traj_frozen_ = control.executionFrozen;
  output.near_field_stop = false;
  output.reason = control.executionFrozen
                      ? (control.awaitingTrajectory
                             ? "teleop_assist_speed_replan"
                             : control.directionTransition
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
