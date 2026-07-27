#include "nav_loop.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lingtu::nav::plan {
namespace {

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

nav_kernel::Vec3 mapPointToBody(
    const nav_kernel::Pose& odom_map_body,
    const nav_kernel::Vec3& point_map) {
  const double dx = point_map.x - odom_map_body.position.x;
  const double dy = point_map.y - odom_map_body.position.y;
  const double c = std::cos(odom_map_body.yaw);
  const double s = std::sin(odom_map_body.yaw);
  return {
      c * dx + s * dy,
      -s * dx + c * dy,
      point_map.z - odom_map_body.position.z,
  };
}

double bodyDistance2D(const nav_kernel::Vec3& point) {
  return std::hypot(point.x, point.y);
}

double wrappedAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

NavLoop::NavLoop(NavLoopConfig config)
    : config_(std::move(config)), local_planner_(config_.local_planner) {
  config_.goal_yaw_tolerance_rad =
      std::max(0.0, config_.goal_yaw_tolerance_rad);
  config_.goal_yaw_kp = std::max(0.0, config_.goal_yaw_kp);
  config_.goal_yaw_max_rate = std::max(0.0, config_.goal_yaw_max_rate);
  config_.path_follower.maxSpeed = config_.max_speed;
  config_.recovery_translation_speed_mps =
      std::clamp(config_.recovery_translation_speed_mps, 0.0, config_.max_speed);
  config_.recovery_rotation_rate_rad_s = std::max(0.0, config_.recovery_rotation_rate_rad_s);
  active_goal_reached_m_ = config_.goal_reached_m;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
}

bool NavLoop::configure() {
  configured_ = local_planner_.loadPaths(config_.path_library_dir);
  return configured_;
}

bool NavLoop::configured() const {
  return configured_;
}

void NavLoop::setGlobalPath(const std::vector<nav_kernel::Vec3> &path,
                            std::optional<double> final_yaw, std::optional<double> goal_reached_m,
                            std::optional<double> goal_yaw_tolerance_rad) {
  global_path_ = path;
  final_yaw_ = final_yaw;
  active_goal_reached_m_ = std::max(0.01, goal_reached_m.value_or(config_.goal_reached_m));
  active_goal_yaw_tolerance_rad_ =
      std::max(0.0, goal_yaw_tolerance_rad.value_or(config_.goal_yaw_tolerance_rad));
  cursor_ = 0;
  follower_state_ = nav_kernel::PathFollowerState{};
  recovery_follower_state_ = nav_kernel::PathFollowerState{};
  recovery_active_ = false;
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  local_planner_.resetRecovery();
  resetAutonomyProgress();
}

void NavLoop::clearGlobalPath() {
  global_path_.clear();
  final_yaw_.reset();
  active_goal_reached_m_ = config_.goal_reached_m;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
  cursor_ = 0;
  follower_state_ = nav_kernel::PathFollowerState{};
  recovery_follower_state_ = nav_kernel::PathFollowerState{};
  recovery_active_ = false;
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  local_planner_.resetRecovery();
  clearRecoveryObservationWait();
  resetAutonomyProgress();
}

void NavLoop::stopLinearMotion() {
  follower_state_.vehicleSpeed = 0.0;
  recovery_follower_state_.vehicleSpeed = 0.0;
}

NavLoopOutput NavLoop::tick(const nav_kernel::Pose &odom_map_body, const float *obstacle_xyzh,
                            int obstacle_count, double timestamp_s,
                            TraversabilityGridView traversability,
                            PlannerObservationView observation) {
  NavLoopOutput output;
  if (!configured_) {
    output.reason = "not_configured";
    return output;
  }
  if (global_path_.empty()) {
    recovery_active_ = false;
    local_planner_.resetRecovery();
    clearRecoveryObservationWait();
    resetAutonomyProgress();
    output.reason = "no_global_path";
    return output;
  }

  output.active = true;
  if (atGoal(odom_map_body)) {
    recovery_active_ = false;
    recovery_action_ = 0;
    recovery_attempt_ = -1;
    recovery_follower_state_ = nav_kernel::PathFollowerState{};
    local_planner_.resetRecovery();
    clearRecoveryObservationWait();
    resetAutonomyProgress();
    const double yaw_error = goalYawError(odom_map_body);
    if (final_yaw_ && std::abs(yaw_error) > active_goal_yaw_tolerance_rad_) {
      output.path_found = true;
      output.reason = "aligning_goal_yaw";
      output.target_index = global_path_.size() - 1;
      output.target = global_path_.back();
      output.target_distance_m =
          nav_kernel::distance2D(output.target, odom_map_body.position);
      output.cmd_vel.wz = std::clamp(
          config_.goal_yaw_kp * yaw_error,
          -config_.goal_yaw_max_rate,
          config_.goal_yaw_max_rate);
      follower_state_ = nav_kernel::PathFollowerState{};
      return output;
    }
    output.goal_reached = true;
    output.active = false;
    output.reason = "goal_reached";
    output.target_index = global_path_.size() - 1;
    output.target = global_path_.back();
    output.target_distance_m = nav_kernel::distance2D(output.target, odom_map_body.position);
    follower_state_ = nav_kernel::PathFollowerState{};
    return output;
  }

  const std::size_t target_index = selectTargetIndex(odom_map_body);
  cursor_ = target_index;
  output.target_index = target_index;
  output.target = global_path_[target_index];
  output.target_distance_m = nav_kernel::distance2D(output.target, odom_map_body.position);

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
      follower_state_.vehicleSpeed = 0.0;
      recovery_follower_state_.vehicleSpeed = 0.0;
      setAutonomyMotionExpected(false, odom_map_body, timestamp_s);
      return output;
    }
  }

  syncTraversabilityGrid(traversability);
  local_planner_.setVehicle(odom_map_body.position.x, odom_map_body.position.y,
                            odom_map_body.position.z, odom_map_body.yaw);
  local_planner_.setGoal(output.target.x, output.target.y);
  const bool force_recovery =
      recovery_active_ || autonomyMotionStalled(odom_map_body, timestamp_s);
  nav_kernel::LocalPlanResult plan =
      force_recovery
          ? local_planner_.planRecovery(obstacle_xyzh, obstacle_count, timestamp_s)
          : local_planner_.plan(obstacle_xyzh, obstacle_count, timestamp_s);
  output.local_planner_debug =
      debugSnapshotToMap(odom_map_body, local_planner_.debugSnapshot());

  output.path_found = plan.pathFound;
  output.near_field_stop = plan.nearFieldStop;
  output.slow_down = std::max(0, std::min(3, plan.slowDown));
  output.recovery_state = plan.recoveryState;
  output.recovery_action = static_cast<int>(plan.recoveryAction);
  output.recovery_attempt = plan.recoveryAttempt;
  output.recovery_candidate_count = plan.recoveryCandidateCount;
  output.recovery_verified = plan.recoveryVerified;
  output.recovery_observation_refresh_required =
      plan.recoveryObservationRefreshRequired;
  output.recovery_progress = plan.recoveryProgress;
  output.recovery_reason =
      plan.recoveryReason.empty() ? "inactive" : plan.recoveryReason;
  output.recovery_exhausted = plan.recoveryExhausted;
  output.local_path_body = plan.path;

  if (plan.recoveryObservationRefreshRequired) {
    recovery_observation_waiting_ = true;
    recovery_observation_baseline_ = observation;
  }

  const bool recovery_action_changed =
      plan.recoveryActive &&
      (output.recovery_action != recovery_action_ ||
       output.recovery_attempt != recovery_attempt_);
  if (recovery_action_changed) {
    recovery_follower_state_ = nav_kernel::PathFollowerState{};
  }
  recovery_active_ = plan.recoveryActive;
  recovery_action_ = recovery_active_ ? output.recovery_action : 0;
  recovery_attempt_ = recovery_active_ ? output.recovery_attempt : -1;

  if (plan.pathFound && !plan.nearFieldStop &&
      output.local_path_body.size() < 2) {
    const nav_kernel::Vec3 target_body =
        mapPointToBody(odom_map_body, output.target);
    if (bodyDistance2D(target_body) > 0.05) {
      if (output.local_path_body.empty() ||
          bodyDistance2D(output.local_path_body.front()) > 0.05) {
        output.local_path_body.insert(
            output.local_path_body.begin(), {0.0, 0.0, 0.0});
      }
      output.local_path_body.push_back(target_body);
    }
  }
  output.local_path_map =
      bodyPathToMap(odom_map_body, output.local_path_body);

  if (plan.recoveryExhausted) {
    output.reason = "local_recovery_exhausted";
    follower_state_.vehicleSpeed = 0.0;
    recovery_follower_state_.vehicleSpeed = 0.0;
    setAutonomyMotionExpected(false, odom_map_body, timestamp_s);
    return output;
  }

  if (plan.recoveryActive && plan.recoveryVerified &&
      plan.recoveryDirectCommand) {
    output.cmd_vel.wz =
        static_cast<double>(plan.recoveryRotationDirection) *
        config_.recovery_rotation_rate_rad_s;
    output.reason = output.recovery_reason;
    follower_state_.vehicleSpeed = 0.0;
    setAutonomyMotionExpected(
        std::abs(output.cmd_vel.wz) > 1e-6,
        odom_map_body,
        timestamp_s);
    return output;
  }

  if (plan.recoveryActive && plan.recoveryVerified) {
    if (output.local_path_body.size() < 2) {
      output.reason = "recovery_untrackable_path";
      recovery_follower_state_.vehicleSpeed = 0.0;
      setAutonomyMotionExpected(false, odom_map_body, timestamp_s);
      return output;
    }

    nav_kernel::PathFollowerParams recovery_params = config_.path_follower;
    recovery_params.maxSpeed = config_.recovery_translation_speed_mps;
    recovery_params.minSpeed = 0.0;
    recovery_params.yawRateGain = 0.0;
    recovery_params.stopYawRateGain = 0.0;
    recovery_params.maxYawRate = 0.0;
    recovery_params.twoWayDrive = false;
    recovery_params.dirDiffThre = M_PI + 0.1;
    recovery_params.omniDirDiffThre = M_PI + 0.1;
    recovery_params.omniDirGoalThre =
        std::max(2.0, config_.local_planner.adjacentRange);
    recovery_params.noRotAtGoal = true;

    const nav_kernel::PathFollowerOutput recovery_control =
        nav_kernel::computeControl(
            {0.0, 0.0, 0.0},
            0.0,
            output.local_path_body,
            1.0,
            timestamp_s,
            1.0,
            0,
            recovery_params,
            recovery_follower_state_,
            bodyDistance2D(output.local_path_body.back()));
    output.cmd_vel = recovery_control.cmd;
    output.reason = output.recovery_reason;
    follower_state_.vehicleSpeed = 0.0;
    setAutonomyMotionExpected(
        config_.recovery_translation_speed_mps > 1e-6,
        odom_map_body,
        timestamp_s);
    return output;
  }

  if (output.recovery_reason != "inactive") {
    output.reason = output.recovery_reason;
    follower_state_.vehicleSpeed = 0.0;
    recovery_follower_state_.vehicleSpeed = 0.0;
    setAutonomyMotionExpected(false, odom_map_body, timestamp_s);
    return output;
  }
  if (plan.nearFieldStop) {
    output.reason = "near_field_stop";
    follower_state_.vehicleSpeed = 0.0;
    setAutonomyMotionExpected(false, odom_map_body, timestamp_s);
    return output;
  }
  if (output.local_path_body.size() < 2) {
    output.reason =
        plan.pathFound ? "untrackable_local_path" : "no_local_path";
    follower_state_.vehicleSpeed = 0.0;
    setAutonomyMotionExpected(false, odom_map_body, timestamp_s);
    return output;
  }

  const nav_kernel::Vec3 vehicle_rel{0.0, 0.0, 0.0};
  const nav_kernel::PathFollowerOutput control = nav_kernel::computeControl(
      vehicle_rel,
      0.0,
      output.local_path_body,
      1.0,
      timestamp_s,
      slowFactor(output.slow_down),
      0,
      config_.path_follower,
      follower_state_,
      nav_kernel::distance2D(
          global_path_.back(), odom_map_body.position));
  output.cmd_vel = control.cmd;
  output.reason = "control_ready";
  const bool command_expects_motion =
      std::hypot(output.cmd_vel.vx, output.cmd_vel.vy) > 1e-6 || std::abs(output.cmd_vel.wz) > 1e-6;
  setAutonomyMotionExpected(command_expects_motion, odom_map_body, timestamp_s);
  return output;
}

NavLoopOutput NavLoop::tickTeleopIntent(const nav_kernel::Pose &odom_map_body,
                                        const nav_kernel::Twist &intent, const float *obstacle_xyzh,
                                        int obstacle_count, double timestamp_s,
                                        TraversabilityGridView traversability) {
  NavLoopOutput output;
  clearRecoveryObservationWait();
  if (!configured_) {
    output.reason = "not_configured";
    return output;
  }

  recovery_active_ = false;
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  recovery_follower_state_ = nav_kernel::PathFollowerState{};
  local_planner_.resetRecovery();
  resetAutonomyProgress();
  const double requested_speed = std::hypot(intent.vx, intent.vy);
  if (requested_speed <= 1e-6 || config_.max_speed <= 1e-6) {
    output.reason = "teleop_intent_idle";
    follower_state_.vehicleSpeed = 0.0;
    return output;
  }

  const double speed_norm = clamp01(requested_speed / config_.max_speed);
  const double direction_body = std::atan2(intent.vy, intent.vx);
  const double horizon = std::max(0.5, config_.teleop_intent_horizon_m);
  const nav_kernel::Vec3 target_body{
      horizon * std::cos(direction_body),
      horizon * std::sin(direction_body),
      0.0,
  };
  const double c = std::cos(odom_map_body.yaw);
  const double s = std::sin(odom_map_body.yaw);

  output.active = true;
  output.target = {
      odom_map_body.position.x + c * target_body.x - s * target_body.y,
      odom_map_body.position.y + s * target_body.x + c * target_body.y,
      odom_map_body.position.z,
  };
  output.target_distance_m = horizon;

  syncTraversabilityGrid(traversability);
  local_planner_.setVehicle(odom_map_body.position.x, odom_map_body.position.y,
                            odom_map_body.position.z, odom_map_body.yaw);
  nav_kernel::LocalPlanResult plan = local_planner_.planIntent(
      obstacle_xyzh, obstacle_count, timestamp_s, direction_body * 180.0 / M_PI, speed_norm,
      horizon, config_.teleop_intent_max_deviation_deg);
  output.local_planner_debug = debugSnapshotToMap(odom_map_body, local_planner_.debugSnapshot());

  output.path_found = plan.pathFound;
  output.slow_down = std::max(0, std::min(3, plan.slowDown));
  output.recovery_state = 0;
  output.local_path_body = plan.path;
  if (plan.pathFound && !plan.nearFieldStop && output.local_path_body.size() < 2) {
    if (output.local_path_body.empty() || bodyDistance2D(output.local_path_body.front()) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
    output.local_path_body.push_back(target_body);
  }
  output.local_path_map = bodyPathToMap(odom_map_body, output.local_path_body);

  if (output.local_path_body.size() < 2) {
    output.near_field_stop = plan.nearFieldStop;
    output.reason = plan.pathFound
        ? "teleop_assist_untrackable_path"
        : "teleop_assist_no_path";
    follower_state_.vehicleSpeed = 0.0;
    return output;
  }

  const nav_kernel::PathFollowerOutput control = nav_kernel::computeControl(
      {0.0, 0.0, 0.0},
      0.0,
      output.local_path_body,
      speed_norm,
      timestamp_s,
      slowFactor(output.slow_down),
      0,
      config_.path_follower,
      follower_state_,
      horizon);
  output.cmd_vel = control.cmd;
  output.near_field_stop = false;
  output.reason = plan.nearFieldStop
      ? "teleop_assist_detour"
      : "teleop_assist_control_ready";
  return output;
}

std::size_t NavLoop::selectTargetIndex(const nav_kernel::Pose& odom_map_body) {
  if (global_path_.empty()) {
    return 0;
  }

  const std::size_t start = std::min(cursor_, global_path_.size() - 1);
  std::size_t nearest = start;
  double nearest_distance = nav_kernel::distance2D(global_path_[start], odom_map_body.position);
  for (std::size_t i = start + 1; i < global_path_.size(); ++i) {
    const double distance = nav_kernel::distance2D(global_path_[i], odom_map_body.position);
    if (distance < nearest_distance) {
      nearest = i;
      nearest_distance = distance;
    }
  }

  while (nearest + 1 < global_path_.size() &&
         nav_kernel::distance2D(global_path_[nearest], odom_map_body.position) <
             config_.waypoint_reached_m) {
    ++nearest;
  }

  for (std::size_t i = nearest; i < global_path_.size(); ++i) {
    if (nav_kernel::distance2D(global_path_[i], odom_map_body.position) >=
        config_.corridor_lookahead_m) {
      return i;
    }
  }
  return global_path_.size() - 1;
}

std::vector<nav_kernel::Vec3> NavLoop::bodyPathToMap(
    const nav_kernel::Pose& odom_map_body,
    const std::vector<nav_kernel::Vec3>& body_path) const {
  std::vector<nav_kernel::Vec3> out;
  out.reserve(body_path.size());
  const double c = std::cos(odom_map_body.yaw);
  const double s = std::sin(odom_map_body.yaw);
  for (const auto& point : body_path) {
    out.push_back({
        odom_map_body.position.x + c * point.x - s * point.y,
        odom_map_body.position.y + s * point.x + c * point.y,
        odom_map_body.position.z + point.z,
    });
  }
  return out;
}

nav_kernel::LocalPlannerDebugSnapshot NavLoop::debugSnapshotToMap(
    const nav_kernel::Pose& odom_map_body,
    nav_kernel::LocalPlannerDebugSnapshot snapshot) const {
  for (auto& candidate : snapshot.candidates) {
    candidate.path = bodyPathToMap(odom_map_body, candidate.path);
  }
  return snapshot;
}

double NavLoop::slowFactor(int slow_down) const {
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

bool NavLoop::atGoal(const nav_kernel::Pose& odom_map_body) const {
  if (global_path_.empty()) {
    return false;
  }
  return nav_kernel::distance2D(global_path_.back(), odom_map_body.position) <=
         active_goal_reached_m_;
}

double NavLoop::goalYawError(const nav_kernel::Pose& odom_map_body) const {
  if (!final_yaw_) {
    return 0.0;
  }
  return wrappedAngle(*final_yaw_ - odom_map_body.yaw);
}
bool NavLoop::autonomyMotionStalled(
    const nav_kernel::Pose& odom_map_body, double timestamp_s) {
  if (!autonomy_motion_expected_) {
    return false;
  }
  if (!autonomy_progress_valid_ ||
      !std::isfinite(timestamp_s) ||
      timestamp_s < autonomy_progress_time_s_) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    autonomy_progress_valid_ = true;
    return false;
  }

  const double linear_progress = nav_kernel::distance2D(
      autonomy_progress_pose_.position, odom_map_body.position);
  const double yaw_progress = std::abs(
      wrappedAngle(odom_map_body.yaw - autonomy_progress_pose_.yaw));
  if (linear_progress >= config_.recovery_stuck_linear_progress_m ||
      yaw_progress >= config_.recovery_stuck_yaw_progress_rad) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    return false;
  }

  return timestamp_s - autonomy_progress_time_s_ >=
         std::max(0.0, config_.local_planner.recoveryBlockedThre);
}

void NavLoop::setAutonomyMotionExpected(
    bool expected,
    const nav_kernel::Pose& odom_map_body,
    double timestamp_s) {
  if (!expected) {
    resetAutonomyProgress();
    return;
  }
  if (!autonomy_motion_expected_ || !autonomy_progress_valid_) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    autonomy_progress_valid_ = true;
  }
  autonomy_motion_expected_ = true;
}

void NavLoop::resetAutonomyProgress() {
  autonomy_motion_expected_ = false;
  autonomy_progress_valid_ = false;
  autonomy_progress_pose_ = {};
  autonomy_progress_time_s_ = 0.0;
}

bool NavLoop::recoveryObservationAdvanced(
    const PlannerObservationView& observation) const {
  if (!recovery_observation_waiting_ ||
      observation.frame_epoch != recovery_observation_baseline_.frame_epoch) {
    return false;
  }
  const double barrier_stamp = recovery_observation_baseline_.odom_stamp_s;
  const auto source_after_rotation = [barrier_stamp](double stamp_s) {
    return std::isfinite(stamp_s) && std::isfinite(barrier_stamp) &&
           stamp_s > barrier_stamp + 1e-9;
  };
  if (config_.local_planner.checkObstacle &&
      (observation.cloud_generation <=
           recovery_observation_baseline_.cloud_generation ||
       !source_after_rotation(observation.cloud_stamp_s))) {
    return false;
  }
  if (config_.local_planner.useTraversabilityCost &&
      (observation.traversability_generation <=
           recovery_observation_baseline_.traversability_generation ||
       !source_after_rotation(observation.traversability_stamp_s))) {
    return false;
  }
  return true;
}

void NavLoop::clearRecoveryObservationWait() {
  recovery_observation_waiting_ = false;
  recovery_observation_baseline_ = {};
}


void NavLoop::syncTraversabilityGrid(TraversabilityGridView traversability) {
  if (!traversability.valid()) {
    if (traversability_loaded_) {
      local_planner_.clearTraversabilityGrid();
      traversability_loaded_ = false;
      traversability_generation_ = 0;
    }
    return;
  }
  if (traversability.generation != 0 &&
      traversability_loaded_ &&
      traversability_generation_ == traversability.generation) {
    return;
  }
  local_planner_.setTraversabilityGrid(
      traversability.values,
      traversability.rows,
      traversability.cols,
      traversability.resolution,
      traversability.origin_x,
      traversability.origin_y);
  traversability_loaded_ = true;
  traversability_generation_ = traversability.generation;
}

}  // namespace lingtu::nav::plan
