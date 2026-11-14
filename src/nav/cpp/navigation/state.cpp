#include "navigation/executor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::navigation {
namespace {

RecoveryConfig teleopRotationConfig(RecoveryConfig config) {
  config.behavior_order = {nav_kernel::RecoveryAction::Rotate};
  return config;
}


double wrappedAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

nav_kernel::LocalKinematicState Executor::planningKinematics(
    const nav_kernel::Pose& planning_body,
    const ExecutionObservation& observation,
    double timestamp_s) {
  nav_kernel::LocalKinematicState state;
  if (!observation.body_velocity_valid ||
      !std::isfinite(observation.body_linear_velocity.x) ||
      !std::isfinite(observation.body_linear_velocity.y) ||
      !std::isfinite(observation.body_linear_velocity.z) ||
      !std::isfinite(observation.body_yaw_rate)) {
    velocity_stamp_s_ = -1.0;
    return state;
  }
  const double c = std::cos(planning_body.yaw);
  const double s = std::sin(planning_body.yaw);
  state.linearVelocity = {
      c * observation.body_linear_velocity.x -
          s * observation.body_linear_velocity.y,
      s * observation.body_linear_velocity.x +
          c * observation.body_linear_velocity.y,
      observation.body_linear_velocity.z,
  };
  state.yawRate = observation.body_yaw_rate;
  if (velocity_stamp_s_ >= 0.0 &&
      previous_kinematics_frame_epoch_ == observation.frame_epoch &&
      timestamp_s > velocity_stamp_s_) {
    const double dt = timestamp_s - velocity_stamp_s_;
    state.linearAcceleration = {
        (state.linearVelocity.x - previous_planning_velocity_.x) / dt,
        (state.linearVelocity.y - previous_planning_velocity_.y) / dt,
        (state.linearVelocity.z - previous_planning_velocity_.z) / dt,
    };
  }
  previous_planning_velocity_ = state.linearVelocity;
  velocity_stamp_s_ = timestamp_s;
  previous_kinematics_frame_epoch_ = observation.frame_epoch;
  state.valid = true;
  return state;
}

bool Executor::autonomyMotionStalled(
    const nav_kernel::Pose& odom_map_body, double timestamp_s, bool planning_pending) {
  if (!autonomy_motion_expected_) return false;
  if (!std::isfinite(timestamp_s) ||
      timestamp_s < autonomy_sample_time_s_) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    autonomy_sample_time_s_ = timestamp_s;
    autonomy_observed_progress_ = 0.0;
    autonomy_requested_progress_ = 0.0;
    return false;
  }

  const double dt = timestamp_s - autonomy_sample_time_s_;
  if (planning_pending) {
    // No executable trajectory means no requested travel during this wait.
    autonomy_progress_time_s_ += dt;
    autonomy_sample_time_s_ = timestamp_s;
    autonomy_progress_pose_ = odom_map_body;
    return false;
  }
  const auto& command = autonomy_expected_command_;
  const double speed = std::hypot(command.vx, command.vy);
  const bool translating = speed > 1e-6;
  // Measure signed displacement along the previous requested motion. Heave,
  // lateral slip and unwanted yaw cannot keep a translation attempt alive.
  if (translating) {
    const double dx = odom_map_body.position.x - autonomy_progress_pose_.position.x;
    const double dy = odom_map_body.position.y - autonomy_progress_pose_.position.y;
    autonomy_observed_progress_ += (dx * command.vx + dy * command.vy) / speed;
    autonomy_requested_progress_ += speed * dt;
  } else {
    autonomy_observed_progress_ +=
        wrappedAngle(odom_map_body.yaw - autonomy_progress_pose_.yaw) *
        std::copysign(1.0, command.wz);
    autonomy_requested_progress_ += std::abs(command.wz) * dt;
  }
  autonomy_progress_pose_ = odom_map_body;
  autonomy_sample_time_s_ = timestamp_s;
  // Keep the existing absolute progress bound, but do not require a slowly
  // commanded robot to cover more than half its requested travel to stay live.
  const double threshold = std::min(
      translating ? config_.recovery.stuck_linear_progress_m
                  : config_.recovery.stuck_yaw_progress_rad,
      0.5 * autonomy_requested_progress_);
  if (autonomy_observed_progress_ > 1e-6 && autonomy_observed_progress_ >= threshold) {
    autonomy_progress_time_s_ = timestamp_s;
    autonomy_observed_progress_ = 0.0;
    autonomy_requested_progress_ = 0.0;
    return false;
  }

  return timestamp_s - autonomy_progress_time_s_ >=
         std::max(0.0, config_.recovery.blocked_interval_s);
}

void Executor::setAutonomyMotionExpected(
    const nav_kernel::Twist& command,
    const nav_kernel::Pose& odom_map_body,
    double timestamp_s) {
  const bool translating = std::hypot(command.vx, command.vy) > 1e-6;
  if (!translating && std::abs(command.wz) <= 1e-6) {
    resetAutonomyProgress();
    return;
  }
  const bool was_translating =
      std::hypot(autonomy_expected_command_.vx, autonomy_expected_command_.vy) > 1e-6;
  if (!autonomy_motion_expected_ ||
      translating != was_translating ||
      (!translating && command.wz * autonomy_expected_command_.wz < 0.0)) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    autonomy_sample_time_s_ = timestamp_s;
    autonomy_observed_progress_ = 0.0;
    autonomy_requested_progress_ = 0.0;
  }
  const double c = std::cos(odom_map_body.yaw);
  const double s = std::sin(odom_map_body.yaw);
  autonomy_expected_command_ = {
      c * command.vx - s * command.vy,
      s * command.vx + c * command.vy,
      command.wz,
  };
  autonomy_motion_expected_ = true;
}

void Executor::resetAutonomyProgress() {
  autonomy_motion_expected_ = false;
  autonomy_expected_command_ = {};
  autonomy_progress_pose_ = {};
  autonomy_progress_time_s_ = 0.0;
  autonomy_sample_time_s_ = 0.0;
  autonomy_observed_progress_ = 0.0;
  autonomy_requested_progress_ = 0.0;
}

Executor::Executor(ExecutorConfig config, nav_kernel::LocalPlanner planner)
    : config_(std::move(config)),
      local_planner_(std::move(planner)),
      recovery_(local_planner_.params(), config_.recovery),
      teleop_recovery_(local_planner_.params(), teleopRotationConfig(config_.recovery)),
      stairs_(config_.stairs) {
  config_.goal_height_tolerance_m = std::max(0.0, config_.goal_height_tolerance_m);
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
  stage_yaw_tolerance_ = kPoseYawToleranceRad;
}


void Executor::resetLocalPlanning(bool full_follower_reset) {
  local_planner_.reset();
  if (full_follower_reset)
    follower_.reset();
  else
    follower_.resetTarget();
  traj_frozen_ = false;
  intent_mode_ = false;
}


void Executor::reportFinalMotionBlocked(bool blocked, double timestamp_s) {
  if (blocked && local_planner_.capabilities().ownsReferencePath)
    resetLocalPlanning();
  if (!blocked) {
    final_motion_blocked_since_s_ = -1.0;
  } else if (final_motion_blocked_since_s_ < 0.0 ||
             final_motion_blocked_since_s_ > timestamp_s) {
    final_motion_blocked_since_s_ = timestamp_s;
  }
}


void Executor::setRoute(Route value) {
  if (!value.body) value.body = config_.body;
  auto stages = compilePoseRoute(value.posePoints,
      value.yawToleranceRad.value_or(kPoseYawToleranceRad));
  // Terminal yaw-only stages are not part of position navigation.
  while (stages.size() > 1 && stages.back().rotation) stages.pop_back();
  if (!stages.empty() && (!value.body || value.body->parts.empty() ||
                         !local_planner_.capabilities().ownsReferencePath))
    throw std::invalid_argument("pose route requires a body and spline planner");
  if (!stages.empty() && (!std::isfinite(value.body->margin_m) || value.body->margin_m < 0.0 ||
                         !std::isfinite(value.body->horizontal_margin_m) ||
                         value.body->horizontal_margin_m < 0.0))
    throw std::invalid_argument("pose route body margin must be finite and nonnegative");
  pose_stages_.clear();
  pose_stage_ = 0;
  pose_support_height_m_=value.supportHeightM;
  pose_support_tolerance_m_=value.supportHeightToleranceM;
  pose_blending_allowed_ = value.stairs.empty() && pose_support_height_m_ &&
      pose_support_tolerance_m_ && std::isfinite(*pose_support_height_m_) &&
      *pose_support_height_m_>0 && std::isfinite(*pose_support_tolerance_m_) &&
      *pose_support_tolerance_m_>=0;
  pose_blend_.reset();
  pose_blend_original_.clear();
  route_body_ = std::move(value.body);
  pose_epoch_.reset();
  reference_epoch.reset();
  requested_goal_ = value.requestedGoal;
  activateRoute(value.points, std::nullopt, value.goalToleranceM,
                std::nullopt, value.maxSpeedMps);
  if (!stages.empty()) {
    pose_stages_ = stages;
    pose_final_tolerance_ = value.goalToleranceM;
    pose_yaw_tolerance_ = value.yawToleranceRad;
    pose_max_speed_ = value.maxSpeedMps;
    activatePoseStage();
  }
  stairs_.setSegments(std::move(value.stairs));
}


void Executor::activatePoseStage() {
  pose_blend_checked_ = false;
  pose_blend_.reset();
  pose_blend_original_.clear();
  const auto& stage = pose_stages_.at(pose_stage_);
  const bool last = pose_stage_ + 1 == pose_stages_.size();
  const double position_tolerance = kPoseStagePositionToleranceM;
  const double rotation_radius = collision::sweepBound(
      *route_body_, {{0,0,0},0}, {{0,0,0},1});
  const double yaw_tolerance = std::min(0.04,
      position_tolerance / std::max(1e-6, rotation_radius));
  activateRoute(stage.points, std::optional<double>{stage.yaw},
                last ? pose_final_tolerance_ : std::optional<double>{position_tolerance},
                last ? pose_yaw_tolerance_ : std::optional<double>{yaw_tolerance},
                pose_max_speed_);
}


void Executor::tryBlendPoseStage(double grid_resolution_m) {
  if (pose_blend_checked_ || pose_stages_.empty()) return;
  pose_blend_checked_ = true;
  const double tolerance=pose_yaw_tolerance_.value_or(kPoseYawToleranceRad);
  auto candidate=pose_blending_allowed_ ?
      poseRouteBlend(pose_stages_,pose_stage_,tolerance,grid_resolution_m) :
      std::optional<PoseRouteBlend>{};
  const bool rounded=bool(candidate);
  if(!candidate && !stairs_.pending())
    candidate=poseRouteContinuous(pose_stages_,pose_stage_,tolerance,grid_resolution_m);
  if (!candidate || !route_body_) return;
  const auto original=pose_stages_;
  pose_stages_[pose_stage_]=candidate->stage;
  pose_stages_.erase(pose_stages_.begin()+pose_stage_+1,
                     pose_stages_.begin()+candidate->lastStageIndex+1);
  activatePoseStage();
  pose_blend_=candidate;
  if(rounded) pose_blend_original_=original;
  pose_blend_checked_=true;
  limitPoseSpeed();
}


void Executor::limitPoseSpeed() {
  if (!pose_blend_) return;
  // Use moving-body limits, not terminal yaw-alignment settings.
  const auto& follower=config_.follower.spline;
  const double entry=std::max(0.,follower.headingErrorThreshold);
  const double lateral=entry<=1e-9 ? follower.maxVx :
      (entry<M_PI/2 ? follower.maxVy/std::sin(entry) : follower.maxVy);
  active_max_speed_mps_=std::min({active_max_speed_mps_,follower.maxVx,lateral,
      pose_blend_->radiusM*follower.maxYawRateRadS});
}


void Executor::setSpeedLimit(double max_speed_mps) {
  active_max_speed_mps_ = std::min(config_.max_speed, max_speed_mps);
  if (!pose_stages_.empty()) pose_max_speed_ = max_speed_mps;
  limitPoseSpeed();
}


void Executor::clear() {
  clearRoute();
  stairs_.cancel();
}


void Executor::activateRoute(const std::vector<nav_kernel::Vec3> &path,
                             std::optional<double> stage_yaw,
                             std::optional<double> goal_reached_m,
                             std::optional<double> stage_yaw_tolerance,
                             std::optional<double> max_speed_mps) {
  if (max_speed_mps && (!std::isfinite(*max_speed_mps) || *max_speed_mps <= 0.0))
    throw std::invalid_argument("route speed limit must be positive and finite");
  active_max_speed_mps_ = std::min(config_.max_speed, max_speed_mps.value_or(config_.max_speed));
  route = path;
  autonomy_stall_stop_ = false;
  ++generation;
  stage_yaw_ = stage_yaw;
  active_goal_reached_m_ = std::max(0.01, goal_reached_m.value_or(config_.goal_reached_m));
  stage_yaw_tolerance_ =
      std::max(0.0, stage_yaw_tolerance.value_or(kPoseYawToleranceRad));
  progress = 0;
  route_start_pending_ = true;
  stair_replan_pending_ = false;
  resetMotion();
}


void Executor::clearRoute() {
  pose_stages_.clear();
  pose_stage_ = 0;
  pose_blending_allowed_ = false;
  pose_support_height_m_.reset();
  pose_support_tolerance_m_.reset();
  pose_blend_checked_ = false;
  pose_blend_.reset();
  pose_blend_original_.clear();
  route_body_.reset();
  pose_epoch_.reset();
  pose_final_tolerance_.reset();
  pose_yaw_tolerance_.reset();
  pose_max_speed_.reset();
  reference_epoch.reset();
  route.clear();
  route_start_pending_ = false;
  stair_replan_pending_ = false;
  autonomy_stall_stop_ = false;
  ++generation;
  stage_yaw_.reset();
  requested_goal_.reset();
  active_goal_reached_m_ = config_.goal_reached_m;
  active_max_speed_mps_ = config_.max_speed;
  stage_yaw_tolerance_ = kPoseYawToleranceRad;
  progress = 0;
  resetMotion();
}


bool Executor::hasRoute() const {
  return !route.empty();
}


void Executor::resetMotion() {
  recovery_follower_.reset();
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  resetLocalPlanning(true);
  recovery_.reset();
  reference.clear();
  guide_map_.clear();
  resetTeleopRotation();
  teleop_reference_.reset();
  velocity_stamp_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  final_motion_blocked_since_s_ = -1.0;
  resetAutonomyProgress();
}


void Executor::resumeRoute() {
  resetMotion();
  // Manual movement can return to an earlier route segment. Reconnect from
  // the next measured pose while retaining the planned elevation and goal.
  progress = 0;
  autonomy_stall_stop_ = false;
  ++generation;
}


void Executor::pauseLinearMotion() {
  traj_frozen_ = true;
  local_planner_.pause();
  follower_.stopLinear();
  recovery_follower_.stopLinear();
  resetAutonomyProgress();
}


void Executor::commitApplied(const nav_kernel::Twist &command) {
  follower_.commitApplied(command);
  recovery_follower_.commitApplied(command);
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
  teleop_reference_.reset();
}


double Executor::slowFactor(int slow_down) const {
  switch (std::max(0, std::min(3, slow_down))) {
    case 1:
      return std::max(0.0, std::min(1.0, config_.slow_rate_1));
    case 2:
      return std::max(0.0, std::min(1.0, config_.slow_rate_2));
    case 3:
      return std::max(0.0, std::min(1.0, config_.slow_rate_3));
    default:
      return 1.0;
  }
}


}  // namespace lingtu::nav::navigation
