#include "navigation/executor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lingtu::nav::navigation {

StairTraversalOutput Executor::tickStairTransition(
    const nav_kernel::Pose& body, const ExecutionObservation& observation,
    double timestamp_s, const StairProfileFeedback& feedback) {
  const StairPhase previous_stair_phase = stair_step_.phase;
  stair_step_ = stairs_.step(body,
      {observation.body_linear_velocity.x,
       observation.body_linear_velocity.y,
       observation.body_yaw_rate}, observation.body_velocity_valid,
      timestamp_s, feedback);
  if (stair_step_.phase != previous_stair_phase) {
    resetLocalPlanning();
    reference.clear();
    guide_map_.clear();
    recovery_.reset();
    resetAutonomyProgress();
  }
  return stair_step_;
}

ExecutionOutput Executor::tick(const ExecutionInput &input) {
  if (!pose_stages_.empty()) {
    if (pose_epoch_ && *pose_epoch_ != input.observation.frame_epoch) {
      clear();
      ExecutionOutput output;
      output.near_field_stop = true;
      output.reason = "pose_route_frame_changed";
      return output;
    }
    pose_epoch_ = input.observation.frame_epoch;
  }
  tickStairTransition(input.mapBody, input.observation, input.timestampS, input.stairProfile);
  if (stair_step_.hold && input.mode == ExecutionMode::Route) {
    ExecutionOutput output;
    output.active = !route.empty();
    output.near_field_stop = true;
    output.reason = stair_step_.reason;
    output.stairs = stair_step_;
    pauseLinearMotion();
    return output;
  }
  if (input.mode == ExecutionMode::MotionIntent) {
    if (!pose_stages_.empty() && pose_stages_[pose_stage_].tangentHeading && !intent_mode_)
      resetMotion();
    return tickIntent(input.mapBody, input.motionIntent, input.obstacleXyzhMap,
                      input.obstacleCount, input.timestampS, input.traversability,
                      input.observation);
  }
  auto output = tickRoute(input.mapBody, input.odomBody, input.mapFromOdom,
                   input.obstacleXyzhMap, input.obstacleCount, input.timestampS,
                   input.traversability, input.observation);
  output.pose_stage_index = pose_stage_;
  output.pose_stage_count = pose_stages_.size();
  output.route_generation = generation;
  output.stairs = stair_step_;
  if (!stairs_.localPathAllowed(output.local_path_map, input.mapBody.yaw)) {
    output.cmd_vel = {};
    output.near_field_stop = true;
    output.reason = "stair_route_wait";
    // A rejected trajectory cannot become valid by freezing its clock.
    // Keep the flight/profile, but regenerate locally from measured motion.
    resetLocalPlanning();
    follower_.stopLinear();
    recovery_follower_.stopLinear();
    resetAutonomyProgress();
    output.tracking = {};
  }
  const double speed = std::hypot(output.cmd_vel.vx, output.cmd_vel.vy);
  const double max_speed = stair_step_.speedLimit > 0.0
      ? std::min(active_max_speed_mps_, stair_step_.speedLimit) : active_max_speed_mps_;
  // The spline follower applies the requested cap, including bounded deceleration
  // from measured speed. Clipping its transition again would create a jump.
  if (!(output.tracking.active && !output.tracking.finished) && speed > max_speed && speed > 0.0) {
    output.cmd_vel.vx *= max_speed / speed;
    output.cmd_vel.vy *= max_speed / speed;
  }
  return output;
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
  if (intent_mode_) resetMotion();
  if (route.empty()) {
    resetLocalPlanning();
    recovery_.reset();
    resetAutonomyProgress();
    output.reason = "no_global_path";
    return output;
  }

  output.active = true;
  // The observed flight owns its traversal heading. Global pose headings stay
  // as the guide and resume after profile restoration; local body queries and
  // the follower approve the same flight heading while traversing.
  const auto execution_goal_yaw = [&]() {
    return !pose_stages_.empty() && stair_step_.phase == StairPhase::Traversing &&
        stair_step_.bodyYaw ? stair_step_.bodyYaw : stage_yaw_;
  };
  const auto execution_yaw_error = [&]() {
    const auto yaw = execution_goal_yaw();
    return yaw ? nav_kernel::normalizeAngle(*yaw-map_body.yaw) : 0.0;
  };
  const auto initial_turn_stage = [&]() {
    const auto translation = std::find_if(pose_stages_.begin(), pose_stages_.end(),
        [](const auto &stage) { return !stage.rotation; });
    const auto index = static_cast<std::size_t>(translation - pose_stages_.begin());
    if (translation == pose_stages_.end() || index <= 1 || pose_stage_ >= index)
      return false;
    const auto &origin = pose_stages_.front().points.front();
    // Only absorb stance drift inside the existing task arrival radius.
    // A manual relocation must retain the ordinary route reconnect behavior.
    return std::hypot(map_body.position.x-origin.x, map_body.position.y-origin.y) <=
               std::max(kPoseStagePositionToleranceM, config_.goal_reached_m) &&
           std::abs(map_body.position.z-origin.z) <=
               config_.goal_height_tolerance_m;
  };
  const auto measured_pose_stage_arrival = [&]() {
    if (pose_stages_.empty() || pose_stage_ == 0 ||
        pose_stage_ + 1 >= pose_stages_.size() || !route_body_)
      return false;
    const auto &goal = route.back();
    const double tolerance = std::max(kPoseStagePositionToleranceM,
        std::min(config_.goal_reached_m, config_.follower.spline.finishDistance));
    const double height_tolerance = std::min(config_.goal_height_tolerance_m,
        pose_support_tolerance_m_.value_or(config_.goal_height_tolerance_m));
    if (std::hypot(goal.x-map_body.position.x, goal.y-map_body.position.y) > tolerance ||
        std::abs(goal.z-map_body.position.z) > height_tolerance)
      return false;
    auto request = makeLocalPlanRequest(
        planning_body, route, nullptr, generation, {}, observation,
        obstacle_xyzh_planning, obstacle_count, timestamp_s, true, traversability, false);
    request.body = route_body_;
    request.observedBody = observation.body_geometry;
    std::vector<nav_kernel::Pose> sweep{planning_body};
    // A legged base need not hit an exact turn centre: validate the actual
    // rotation and reconnect instead of inheriting the nominal sweep clearance.
    for (std::size_t index = pose_stage_; index < pose_stages_.size(); ++index) {
      const auto &stage = pose_stages_[index];
      const double next_yaw = nav_kernel::normalizeAngle(stage.yaw-map_from_odom.yaw);
      sweep.push_back({planning_body.position, next_yaw});
      if (index > pose_stage_ && !stage.rotation) {
        sweep.push_back({map_from_odom.odomPointFromMap(stage.points.front()), next_yaw});
        break;
      }
    }
    return local_planner_.sweepClear(request, sweep);
  };
  const auto following = follower_.diagnostics();
  // Let the active terminal spline brake before the goal radius resets it.
  const bool moving_spline = following.active &&
      following.algorithm == nav_kernel::FollowerAlgorithm::Spline && following.linearSpeed > 0.03;
  bool measured_stage_arrival = measured_pose_stage_arrival();
  if (!pose_stages_.empty() && pose_stage_ + 1 < pose_stages_.size() &&
      (!moving_spline || !pose_stages_[pose_stage_ + 1].rotation) &&
      (atGoal(map_body) || initial_turn_stage() || measured_stage_arrival) &&
      std::abs(execution_yaw_error()) <= (measured_stage_arrival
          ? std::max(stage_yaw_tolerance_, kPoseYawToleranceRad)
          : stage_yaw_tolerance_)) {
    const double speed = std::hypot(observation.body_linear_velocity.x,
                                    observation.body_linear_velocity.y);
    // An aligned turn hands measured motion to the next translation without a second stop.
    const bool moving_handoff = !pose_stages_[pose_stage_ + 1].rotation;
    if (observation.body_velocity_valid && std::isfinite(speed) &&
        std::isfinite(observation.body_linear_velocity.z) &&
        (moving_handoff || speed <= 0.03) &&
        std::isfinite(observation.body_yaw_rate) &&
        (moving_handoff || std::abs(observation.body_yaw_rate) <= 0.08)) {
      ++pose_stage_;
      activatePoseStage();
    } else if (measured_stage_arrival) {
      output.reason = "settling_before_pose_turn";
      output.path_found = true;
      pauseLinearMotion();
      return output;
    }
  }
  tryBlendPoseStage(observation.collision.resolution);
  measured_stage_arrival = measured_pose_stage_arrival();
  auto goal_body = route.back();
  const auto stair_goal = stairs_.guideGoal(goal_body, active_goal_reached_m_, map_body.yaw);
  if (stair_goal.hold) {
    output.reason = "stair_exit_target";
    output.near_field_stop = true;
    pauseLinearMotion();
    return output;
  }
  const auto stair_target = stair_step_.approachTarget
      ? stair_step_.approachTarget : stair_goal.target;

  const bool at_stage = atGoal(map_body) || initial_turn_stage() || measured_stage_arrival;
  const bool rotating = !pose_stages_.empty() && pose_stages_[pose_stage_].rotation;
  if (rotating && at_stage && std::abs(execution_yaw_error()) > stage_yaw_tolerance_) {
    auto request = makeLocalPlanRequest(
        planning_body, route, nullptr, generation, {}, observation,
        obstacle_xyzh_planning, obstacle_count, timestamp_s, true, traversability, false);
    request.body = route_body_;
    request.observedBody = observation.body_geometry;
    const double heading = nav_kernel::normalizeAngle(*execution_goal_yaw() - map_from_odom.yaw);
    output.path_found = local_planner_.turnClear(request, heading);
    output.near_field_stop = !output.path_found;
    output.reason = output.path_found ? "pose_turn" : "pose_turn_blocked";
    if (output.path_found)
      output.cmd_vel.wz = nav_kernel::Follower::turnRate(execution_yaw_error(),
          config_.follower.spline.yawGain, config_.follower.spline.maxYawRateRadS);
    follower_.stopLinear();
    return output;
  }
  if (!stair_target && stair_goal.arrivalAllowed && atGoal(map_body) &&
      (pose_stages_.empty() || pose_stage_ + 1 == pose_stages_.size()) &&
      (pose_stages_.empty() || !following.tracking.active ||
       following.tracking.executionTimeS >= following.tracking.durationS)) {
    output.active = false;
    output.goal_reached = true;
    output.path_found = true;
    output.target_index = route.size() - 1;
    output.target = requested_goal_.value_or(route.back());
    output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
    output.reason = "goal_reached";
    follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }

  const auto planner_capabilities = local_planner_.capabilities();
  SegmentTarget target;
  const std::vector<nav_kernel::Vec3> *plan_route = nullptr;
  if (planner_capabilities.ownsReferencePath) {
    if (reference.empty() || config_.planning_frame == PlanningFrame::Odom) {
      target = buildSegment(map_body, planning_body, map_from_odom);
      buildReference(target, map_from_odom);
    }
    plan_route = &reference;
    output.target_index = route.size() - 1;
    output.target = route.back();
  } else {
    target = buildSegment(map_body, planning_body, map_from_odom);
    buildReference(target, map_from_odom);
    applyLocalGuide(map_body, planning_body, map_from_odom, timestamp_s);
    plan_route = &segment;
    output.target_index = target.index;
    output.target = target.point;
  }
  output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);

  if (stair_step_.approachTarget) {
    const auto landing_target = map_from_odom.odomPointFromMap(*stair_step_.approachTarget);
    const auto shorten = [&](std::vector<nav_kernel::Vec3>& points) {
      if (points.empty()) return;
      std::size_t nearest = 0;
      double closest = std::numeric_limits<double>::infinity();
      for (std::size_t i = 1; i < points.size(); ++i) {
        const auto& a = points[i - 1];
        const auto& b = points[i];
        const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
        const double length = dx * dx + dy * dy + dz * dz;
        const double t = length <= 1e-12 ? 0.0 : std::clamp(
            ((landing_target.x - a.x) * dx + (landing_target.y - a.y) * dy + (landing_target.z - a.z) * dz) / length,
            0.0, 1.0);
        const double distance = nav_kernel::distance3D(landing_target,
            {a.x + t * dx, a.y + t * dy, a.z + t * dz});
        if (distance < closest) { closest = distance; nearest = i - 1; }
      }
      points.resize(nearest + 1);
      points.push_back(landing_target);
    };
    shorten(segment);
    shorten(reference);
    output.target = *stair_step_.approachTarget;
    output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
    if (output.target_distance_m <= 0.08 && stair_step_.bodyYaw) {
      output.path_found = true;
      output.reason = "stair_aligning";
      output.cmd_vel.wz = nav_kernel::Follower::turnRate(
          nav_kernel::normalizeAngle(*stair_step_.bodyYaw - map_body.yaw),
          config_.follower.spline.yawGain, config_.follower.spline.maxYawRateRadS);
      const auto body = route_body_ ? route_body_ : config_.body;
      if (body) {
        auto turn_request = makeLocalPlanRequest(
            planning_body, route, nullptr, generation, {}, observation,
            obstacle_xyzh_planning, obstacle_count, timestamp_s, true, traversability, false);
        turn_request.body = body;
        turn_request.observedBody = observation.body_geometry;
        const double target_yaw = nav_kernel::normalizeAngle(*stair_step_.bodyYaw-map_from_odom.yaw);
        if (!local_planner_.turnClear(turn_request, target_yaw)) {
          output.cmd_vel = {};
          output.path_found = false;
          output.near_field_stop = true;
          output.reason = "stair_turn_blocked";
        }
      }
      follower_.stopLinear();
      return output;
    }
  }
  if (stair_goal.target) {
    const auto landing_target = map_from_odom.odomPointFromMap(*stair_goal.target);
    for (auto* points : {&segment, &reference}) {
      if (points->empty() || nav_kernel::distance3D(points->back(), landing_target) > 1e-6)
        points->push_back(landing_target);
    }
    output.target = *stair_goal.target;
    output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
  }

  const nav_kernel::LocalKinematicState kinematics =
      planningKinematics(planning_body, observation, timestamp_s);
  nav_kernel::LocalPlanRequest plan_request = makeLocalPlanRequest(
      planning_body, *plan_route, &reference, generation, kinematics, observation,
      obstacle_xyzh_planning, obstacle_count, timestamp_s,
      traj_frozen_, traversability,
      planner_capabilities.consumesTraversability);
  const auto tracking = follower_.diagnostics().tracking;
  if (tracking.active)
    plan_request.execution = {tracking.trajectoryId, tracking.sampleTimeS,
                              tracking.executionTimeS};
  plan_request.body = route_body_ ? route_body_ : config_.body;
  plan_request.observedBody = plan_request.body ? observation.body_geometry : nullptr;
  plan_request.supportHeightM = pose_support_height_m_;
  plan_request.supportHeightToleranceM = pose_support_tolerance_m_;
  if (!pose_stages_.empty() && !pose_stages_[pose_stage_].tangentHeading)
    plan_request.bodyHeading = nav_kernel::normalizeAngle(
        pose_stages_[pose_stage_].yaw - map_from_odom.yaw);
  plan_request.maxLinearSpeedMps = std::min(
      active_max_speed_mps_, std::max(config_.follower.spline.maxVx, config_.follower.spline.maxVy));
  if (stair_step_.speedLimit > 0.0)
    plan_request.maxLinearSpeedMps = std::min(plan_request.maxLinearSpeedMps, stair_step_.speedLimit);
  if (pose_blend_) plan_request.bodyHeading.reset();
  if (stair_step_.phase == StairPhase::Traversing && stair_step_.bodyYaw)
    plan_request.bodyHeading = nav_kernel::normalizeAngle(*stair_step_.bodyYaw - map_from_odom.yaw);
  if (planner_capabilities.consumesPredictions && observation.predictions.fresh(timestamp_s)) {
    output.prediction_count = observation.predictions.count;
  }
  const nav_kernel::LocalPlan plan =
      planLocal(plan_request, map_from_odom, &output.local_planner_debug);
  if (planner_capabilities.ownsReferencePath && output.local_planner_debug.localTargetValid) {
    output.target = map_from_odom.mapPointFromOdom(
        output.local_planner_debug.localTarget);
    output.target_distance_m =
        nav_kernel::distance3D(output.target, map_body.position);
  }

  const nav_kernel::LocalPlanStatus plan_status = plan.status();
  const bool plan_ready = plan.ready();
  const bool near_field_stop = plan_status == nav_kernel::LocalPlanStatus::NearFieldStop;
  const bool planner_input_valid =
      plan_status != nav_kernel::LocalPlanStatus::InvalidInput &&
      plan_status != nav_kernel::LocalPlanStatus::NotConfigured;
  const bool recovery_failure =
      plan_status == nav_kernel::LocalPlanStatus::NoPath ||
      plan_status == nav_kernel::LocalPlanStatus::Blocked || near_field_stop;
  if(pose_blend_ && !pose_blend_original_.empty() && recovery_failure && observation.body_velocity_valid &&
     std::hypot(observation.body_linear_velocity.x,observation.body_linear_velocity.y)<=0.03 &&
     std::abs(observation.body_yaw_rate)<=0.08 &&
     nav_kernel::distance3D(map_body.position,
         pose_blend_original_[pose_stage_].points.front())<=kPoseStagePositionToleranceM) {
    pose_stages_=std::move(pose_blend_original_);
    activatePoseStage();
    pose_blending_allowed_=false;
    output.reason="pose_blend_replanning";
    output.trajectory_frozen=true;
    return output;
  }
  RecoveryOutput recovery;
  if (!planner_capabilities.ownsReferencePath) {
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
    const bool recovery_enabled = config_.recovery.max_attempts > 0 && !stair_step_.constrainRecovery;
    const bool recovery_active = recovery_.active();
    const bool stalled = !recovery_active && !blocked_long_enough &&
                          autonomyMotionStalled(planning_body, timestamp_s,
                              plan_status == nav_kernel::LocalPlanStatus::Pending);
    if (stalled && stair_step_.constrainRecovery) {
      // A stair profile forbids ordinary recovery translations/rotations, not
      // replanning within the same observed flight. Discard the stalled local
      // trajectory and stop this tick; the next plan uses current collision data.
      resetLocalPlanning();
      follower_.stopLinear();
      recovery_follower_.stopLinear();
      resetAutonomyProgress();
      output.reason = "stair_replanning";
      output.recovery_trigger = "execution_stalled";
      output.near_field_stop = near_field_stop;
      output.trajectory_frozen = true;
      return output;
    }
    if (stalled && !recovery_enabled) autonomy_stall_stop_ = true;
    output.recovery_trigger = recovery_active
                                  ? "active"
                                  : (blocked_long_enough ? "local_blocked"
                                                         : (stalled ? "execution_stalled" : "inactive"));
    const bool force_recovery = recovery_enabled &&
                                (recovery_active || blocked_long_enough || stalled);
    if (force_recovery && planner_input_valid) {
      recovery = recovery_.step(plan_request);
    } else if (!planner_input_valid || plan_ready) {
      recovery_.reset();
    }
  }

  output.near_field_stop = near_field_stop;
  output.slow_down = std::clamp(plan.hints().slowdownLevel, 0, 3);
  output.recovery_state = recovery.state;
  output.recovery_action = static_cast<int>(recovery.action);
  output.recovery_attempt = recovery.attempt;
  output.recovery_candidate_count = recovery.candidate_count;
  output.recovery_rotation_target_rad = recovery.rotation_target_rad;
  output.recovery_verified = recovery.verified;
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
        std::hypot(output.local_path_body.front().x, output.local_path_body.front().y) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
  }
  output.local_path_map = !recovery.active && spline_provided
                              ? planningPathToMap(map_from_odom, preview_path)
                              : bodyPathToMap(map_body, output.local_path_body);
  if (!recovery.active && plan_ready && !near_field_stop &&
      plan.hints().retainRouteGuide &&
      output.local_path_map.size() >= 2) {
    guide_map_ = output.local_path_map;
    guide_generation_ = generation;
    guide_stamp_s_ = timestamp_s;
  } else if (recovery.active || (plan_ready && !plan.hints().retainRouteGuide)) {
    guide_map_.clear();
  }

  if (autonomy_stall_stop_) {
    output.reason = "execution_stalled";
    output.recovery_trigger = "execution_stalled";
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
    output.cmd_vel = nav_kernel::Follower::rotationCommand(
        recovery.rotation_direction, config_.recovery.rotation_rate_rad_s);
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

    const auto recovery_params = nav_kernel::recoveryFollowerParams(
        config_.follower, config_.recovery.translation_speed_mps,
        local_planner_.params().adjacentRange);

    nav_kernel::FollowerState recovery_state;
    recovery_state.requestedSpeed = 1.0;
    recovery_state.currentTime = timestamp_s;
    recovery_state.params = recovery_params;
    recovery_state.goalDistance = std::hypot(output.local_path_body.back().x, output.local_path_body.back().y);
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
  const bool path_provided = std::holds_alternative<nav_kernel::PathTarget>(plan.target());
  nav_kernel::FollowerState follower_state;
  follower_state.measuredBodyTwistValid = observation.body_velocity_valid;
  follower_state.measuredBodyTwist =
      observation.body_velocity_valid
          ? nav_kernel::Twist{observation.body_linear_velocity.x,
                              observation.body_linear_velocity.y, observation.body_yaw_rate}
          : nav_kernel::Twist{};
  follower_state.currentTime = timestamp_s;
  if (spline_provided || !plan_ready) {
    follower_state.vehicleRelative = planning_body.position;
    follower_state.vehicleYawRelative = planning_body.yaw;
    follower_state.desiredHeading = plan_request.bodyHeading;
  }
  follower_state.slowFactor = slowFactor(output.slow_down);
  follower_state.goalDistance = std::hypot(route.back().x - map_body.position.x,
                                           route.back().y - map_body.position.y);
  follower_state.params = config_.follower;
  if(!pose_stages_.empty() && pose_stages_[pose_stage_].tangentHeading) {
    follower_state.params.spline.timeForward=0;
  }
  if (path_provided) {
    follower_state.params = nav_kernel::cmuFollowerParams(follower_state.params);
    follower_state.standardPathProfile = false;
  }
  // A shifted endpoint must not add another stopping radius outside the task's goal region.
  double endpoint_tolerance = active_goal_reached_m_;
  if (requested_goal_ && (pose_stages_.empty() || pose_stage_ + 1 == pose_stages_.size())) {
    endpoint_tolerance = std::max(0.0, active_goal_reached_m_ -
        std::hypot(route.back().x - requested_goal_->x, route.back().y - requested_goal_->y));
    follower_state.goalDistance = std::hypot(requested_goal_->x - map_body.position.x,
                                            requested_goal_->y - map_body.position.y);
  }
  follower_state.params.stopDisThre =
      std::min(follower_state.params.stopDisThre, endpoint_tolerance);
  follower_state.params.spline.finishDistance =
      std::min(follower_state.params.spline.finishDistance, endpoint_tolerance);
  follower_state.params.maxSpeed = active_max_speed_mps_;
  follower_state.params.minSpeed = std::min(follower_state.params.minSpeed, active_max_speed_mps_);
  if (stair_step_.speedLimit > 0.0) {
    follower_state.params.maxSpeed = std::min(follower_state.params.maxSpeed, stair_step_.speedLimit);
    follower_state.params.minSpeed = std::min(follower_state.params.minSpeed, stair_step_.speedLimit);
  }
  if (stair_target) {
    follower_state.goalDistance = nav_kernel::distance3D(*stair_target, map_body.position);
    follower_state.params.stopDisThre = std::min(follower_state.params.stopDisThre, 0.04);
    follower_state.params.spline.finishDistance = std::min(follower_state.params.spline.finishDistance, 0.04);
  }
  follower_state.params.twoWayDrive = false;
  const nav_kernel::FollowerOutput control = follower_.follow(plan, follower_state);
  if (near_field_stop) {
    output.reason = "local_near_field_stop";
    follower_.stopLinear();
    return output;
  }
  if (!plan_ready) {
    output.planning_failed = planner_capabilities.ownsReferencePath &&
                             plan_status == nav_kernel::LocalPlanStatus::NoPath;
    output.reason = plan_status == nav_kernel::LocalPlanStatus::Pending
                        ? "local_plan_pending"
                        : nav_kernel::localPlanStatusName(plan_status);
    follower_.stopLinear();
    // Preserve earlier motion evidence across transient holds. Pending pauses
    // its timer above; blocked holds still expose a stalled motion attempt.
    if (!planner_input_valid || plan_status == nav_kernel::LocalPlanStatus::Cancelled) {
      resetAutonomyProgress();
    }
    return output;
  }
  if (path_provided && output.local_path_body.size() < 2) {
    output.reason = "untrackable_local_path";
    follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }
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

}  // namespace lingtu::nav::navigation
