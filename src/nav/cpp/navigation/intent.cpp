#include "navigation/executor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace lingtu::nav::navigation {
namespace {

constexpr double kTeleopIntentToleranceRad = 10.0 * M_PI / 180.0;

void applyTeleopRotation(ExecutionOutput &output, const RecoveryOutput &recovery,
                         double rotation_rate_rad_s) {
  output.recovery_state = recovery.state;
  output.recovery_action = static_cast<int>(recovery.action);
  output.recovery_attempt = recovery.attempt;
  output.recovery_candidate_count = recovery.candidate_count;
  output.recovery_rotation_target_rad = recovery.rotation_target_rad;
  output.recovery_verified = recovery.verified;
  output.recovery_progress = recovery.progress;
  output.recovery_reason = recovery.reason;
  output.recovery_exhausted = recovery.exhausted;
  output.path_found = false;
  output.near_field_stop = true;
  output.local_path_body.clear();
  output.local_path_map.clear();
  output.cmd_vel = {};
  if (recovery.active && recovery.verified && recovery.direct_command) {
    output.cmd_vel = nav_kernel::Follower::rotationCommand(
        recovery.rotation_direction, rotation_rate_rad_s);
  }
  output.reason = recovery.reason;
}


}  // namespace

void Executor::resetTeleopRotation() {
  teleop_recovery_.reset();
  teleop_recovery_intent_rad_.reset();
}


void Executor::resetTeleopBoundaryDeparture() {
  recovery_follower_.reset();
  teleop_boundary_departure_intent_rad_.reset();
}


ExecutionOutput Executor::tickIntent(const nav_kernel::Pose &odom_map_body,
                                      const nav_kernel::Twist &intent, const float *obstacle_xyzh,
                                      int obstacle_count, double timestamp_s,
                                      TraversabilityGridView traversability,
                                      ExecutionObservation observation) {
  ExecutionOutput output;
  autonomy_stall_stop_ = false;

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
    stopLinearMotion();
    output.reason = "teleop_intent_idle";
    follower_.stopLinear();
    return output;
  }

  const double speed_norm = std::max(0.0, std::min(1.0, requested_speed / config_.max_speed));
  const double input_direction_body = std::atan2(intent.vy, intent.vx);
  if (teleop_recovery_intent_rad_.has_value() &&
      std::abs(nav_kernel::normalizeAngle(
          input_direction_body - *teleop_recovery_intent_rad_)) >
          kTeleopIntentToleranceRad) {
    resetTeleopRotation();
  }
  const double configured_horizon = std::max(0.5, config_.teleop_intent_horizon_m);
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
      odom_map_body, intent_route, nullptr, generation, kinematics, observation,
      obstacle_xyzh, obstacle_count, timestamp_s,
      traj_frozen_, traversability,
      local_planner_.capabilities().consumesTraversability,
      &motion_intent);
  const auto tracking = follower_.diagnostics().tracking;
  if (tracking.active)
    plan_request.execution = {tracking.trajectoryId, tracking.sampleTimeS,
                              tracking.executionTimeS};
  plan_request.body = route_body_ ? route_body_ : config_.body;
  plan_request.observedBody = plan_request.body ? observation.body_geometry : nullptr;
  plan_request.maxLinearSpeedMps = std::min(
      std::min(requested_speed, config_.max_speed),
      std::hypot(config_.follower.spline.maxVx, config_.follower.spline.maxVy));

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
        std::hypot(output.local_path_body.front().x, output.local_path_body.front().y) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
  }
  output.local_path_map = spline_provided
                              ? preview_path
                              : bodyPathToMap(odom_map_body, output.local_path_body);
  if (output.path_found && plan_request.body && output.local_path_map.size() >= 2) {
    std::vector<nav_kernel::Pose> body_sweep;
    body_sweep.reserve(output.local_path_map.size());
    double yaw = odom_map_body.yaw;
    for (std::size_t index = 0; index < output.local_path_map.size(); ++index) {
      const auto &point = output.local_path_map[index];
      if (index > 0) {
        const auto &previous = output.local_path_map[index - 1];
        if (std::hypot(point.x - previous.x, point.y - previous.y) > 1e-9)
          yaw = std::atan2(point.y - previous.y, point.x - previous.x);
      }
      body_sweep.push_back({point, yaw});
    }
    if (!local_planner_.sweepClear(plan_request, body_sweep)) {
      output.path_found = false;
      output.near_field_stop = true;
      output.reason = "scan_body_collision";
      output.cmd_vel = {};
      follower_.stopLinear();
      return output;
    }
  }

  const bool spline_trackable = !spline_provided || plan_ready;
  const bool path_trackable = spline_provided || output.local_path_body.size() >= 2;
  const bool planner_input_valid =
      plan_status != nav_kernel::LocalPlanStatus::InvalidInput &&
      plan_status != nav_kernel::LocalPlanStatus::NotConfigured;
  const bool plan_unusable =
      !plan_ready || near_field_stop || !spline_trackable || !path_trackable;
  const bool scan_boundary_departure_candidate =
      local_planner_.capabilities().supportsBoundaryDeparture && planner_input_valid &&
      plan_status != nav_kernel::LocalPlanStatus::Pending &&
      plan_unusable && config_.recovery.translation_speed_mps > 1e-6 &&
      std::abs(intent.wz) <= 1e-6 && observation.collision.valid();
  if (scan_boundary_departure_candidate) {
    constexpr double departure_distance_m = 0.35;
    const std::vector<nav_kernel::Vec3> departure_body{
        {},
        {departure_distance_m * std::cos(input_direction_body),
         departure_distance_m * std::sin(input_direction_body), 0.0},
    };
    auto departure_map = bodyPathToMap(odom_map_body, departure_body);
    if (local_planner_.boundaryDepartureClear(plan_request, departure_map.back())) {
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
      output.recovery_trigger = "local_blocked";
      output.recovery_reason = "scan_boundary_departure";

      const auto recovery_params = nav_kernel::recoveryFollowerParams(
          config_.follower,
          std::min(config_.recovery.translation_speed_mps, plan_request.maxLinearSpeedMps),
          local_planner_.params().adjacentRange);

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
      !local_planner_.capabilities().supportsBoundaryDeparture && planner_input_valid &&
      plan_status != nav_kernel::LocalPlanStatus::Pending && plan_unusable;
  if (recovery_needed) {
    teleop_recovery_intent_rad_ = input_direction_body;
    const RecoveryOutput recovery = teleop_recovery_.step(plan_request);
    applyTeleopRotation(output, recovery, config_.recovery.rotation_rate_rad_s);
    output.recovery_trigger = "local_blocked";
    follower_.stopLinear();
    traj_frozen_ = false;
    return output;
  }

  if (!plan_ready) {
    const bool pending = plan_status == nav_kernel::LocalPlanStatus::Pending;
    output.near_field_stop = near_field_stop || pending;
    output.reason = pending ? "local_intent_pending"
                            : nav_kernel::localPlanStatusName(plan_status);
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
  follower_state.measuredBodyTwistValid = observation.body_velocity_valid;
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
    if (std::abs(intent.wz) > 1e-6)
      follower_state.desiredYawRate = intent.wz;
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


}  // namespace lingtu::nav::navigation
