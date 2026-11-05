#include "navigation/executor.hpp"
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
constexpr double kPoseStagePositionToleranceM = 0.01;

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
    std::uint64_t route_generation,
    nav_kernel::LocalKinematicState kinematics, const ExecutionObservation &observation,
    const float *obstacle_xyzh, int obstacle_count, double timestamp_s,
    bool execution_frozen, const TraversabilityGridView &traversability,
    bool include_traversability,
    const nav_kernel::LocalMotionIntent *intent = nullptr) {
  nav_kernel::LocalRouteView route_view{
      route.empty() ? nullptr : route.data(),
      static_cast<int>(route.size()),
      route_generation,
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
    };
  }
  request.identity = {
      observation.frame_epoch,
      observation.collision.present() ? observation.collision.generation
                                      : observation.cloud_generation,
      include_traversability ? observation.traversability_generation : 0,
  };
  request.environment.obstacles = {obstacle_xyzh, obstacle_count};
  request.environment.collision = observation.collision;
  request.environment.predictions = observation.predictions;
  request.clock = {timestamp_s, execution_frozen, observation.clock_mode};
  if (include_traversability && traversability.valid()) {
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
    output.cmd_vel = nav_kernel::Follower::rotationCommand(
        recovery.rotation_direction, rotation_rate_rad_s);
  }
  output.reason = recovery.reason;
}

}  // namespace

bool TraversabilityGridView::valid() const {
  return values != nullptr && rows > 0 && cols > 0 && resolution > 0.0;
}

Executor::Executor(ExecutorConfig config, nav_kernel::LocalPlanner planner)
    : config_(std::move(config)),
      local_planner_(std::move(planner)),
      recovery_(local_planner_.params(), config_.recovery),
      teleop_recovery_(local_planner_.params(), teleopRotationConfig(config_.recovery)),
      stairs_(config_.stairs) {
  config_.goal_yaw_tolerance_rad = std::max(0.0, config_.goal_yaw_tolerance_rad);
  config_.goal_height_tolerance_m = std::max(0.0, config_.goal_height_tolerance_m);
  config_.goal_yaw_kp = std::max(0.0, config_.goal_yaw_kp);
  config_.goal_yaw_max_rate = std::max(0.0, config_.goal_yaw_max_rate);
  config_.goal_yaw_min_rate = std::clamp(config_.goal_yaw_min_rate, 0.0,
                                        config_.goal_yaw_max_rate);
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
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
}

void Executor::resetLocalPlanning(bool full_follower_reset) {
  local_planner_.reset();
  if (full_follower_reset)
    follower_.reset();
  else
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
  if (debug->collisionEvidence && config_.planning_frame == PlanningFrame::Odom) {
    auto &evidence = *debug->collisionEvidence;
    evidence.rejectedPosition = map_from_odom.mapPointFromOdom(evidence.rejectedPosition);
    if (evidence.measuredPoints) {
      auto points = std::make_shared<std::vector<nav_kernel::Vec3>>(*evidence.measuredPoints);
      for (auto &point : *points) point = map_from_odom.mapPointFromOdom(point);
      evidence.measuredPoints = std::move(points);
    }
  }
  return plan;
}

void Executor::setRoute(Route value) {
  if (!value.body) value.body = config_.body;
  const auto stages = compilePoseRoute(value.posePoints,
      value.yawToleranceRad.value_or(config_.goal_yaw_tolerance_rad));
  if (!stages.empty() && (!value.body || value.body->parts.empty() ||
                         !local_planner_.capabilities().ownsReferencePath))
    throw std::invalid_argument("pose route requires a body and spline planner");
  if (!stages.empty() && (!std::isfinite(value.body->margin_m) || value.body->margin_m < 0.0 ||
                         !std::isfinite(value.body->horizontal_margin_m) ||
                         value.body->horizontal_margin_m < 0.0))
    throw std::invalid_argument("pose route body margin must be finite and nonnegative");
  if (!stages.empty() && value.finalYaw && std::abs(nav_kernel::normalizeAngle(
          *value.finalYaw - stages.back().yaw)) > 1e-8)
    throw std::invalid_argument("pose route final yaw must match its terminal pose");
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
  activateRoute(value.points, value.finalYaw, value.goalToleranceM, value.yawToleranceRad, value.maxSpeedMps);
  if (requested_goal_) height_offset_ = 0.0;
  if (!stages.empty()) {
    pose_stages_ = stages;
    pose_final_tolerance_ = value.goalToleranceM;
    pose_final_yaw_tolerance_ = value.yawToleranceRad;
    pose_requested_final_yaw_ = value.finalYaw;
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
  activateRoute(stage.points, last ? pose_requested_final_yaw_ : std::optional<double>{stage.yaw},
                last ? pose_final_tolerance_ : std::optional<double>{position_tolerance},
                last ? pose_final_yaw_tolerance_ : std::optional<double>{yaw_tolerance},
                pose_max_speed_);
  // Pose routes already reference the body origin; do not recalibrate their
  // elevation from the first observed pose and erase a vertical discrepancy.
  height_offset_ = 0.0;
}

void Executor::tryBlendPoseStage(double grid_resolution_m) {
  if (pose_blend_checked_ || pose_stages_.empty()) return;
  pose_blend_checked_ = true;
  const double tolerance=pose_final_yaw_tolerance_.value_or(config_.goal_yaw_tolerance_rad);
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
                             std::optional<double> final_yaw,
                             std::optional<double> goal_reached_m,
                             std::optional<double> goal_yaw_tolerance_rad,
                             std::optional<double> max_speed_mps) {
  if (max_speed_mps && (!std::isfinite(*max_speed_mps) || *max_speed_mps <= 0.0))
    throw std::invalid_argument("route speed limit must be positive and finite");
  active_max_speed_mps_ = std::min(config_.max_speed, max_speed_mps.value_or(config_.max_speed));
  route = path;
  autonomy_stall_stop_ = false;
  ++generation;
  final_yaw_ = final_yaw;
  height_offset_.reset();
  active_goal_reached_m_ = std::max(0.01, goal_reached_m.value_or(config_.goal_reached_m));
  active_goal_yaw_tolerance_rad_ =
      std::max(0.0, goal_yaw_tolerance_rad.value_or(config_.goal_yaw_tolerance_rad));
  progress = 0;
  segment.clear();
  suspendAutonomy();
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
  pose_final_yaw_tolerance_.reset();
  pose_requested_final_yaw_.reset();
  pose_max_speed_.reset();
  reference_epoch.reset();
  route.clear();
  autonomy_stall_stop_ = false;
  ++generation;
  final_yaw_.reset();
  requested_goal_.reset();
  height_offset_.reset();
  active_goal_reached_m_ = config_.goal_reached_m;
  active_max_speed_mps_ = config_.max_speed;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
  progress = 0;
  segment.clear();
  suspendAutonomy();
}

bool Executor::hasRoute() const {
  return !route.empty();
}

void Executor::suspendAutonomy() {
  goal_heading_active_ = false;
  recovery_follower_.reset();
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  resetLocalPlanning(true);
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
  resetAutonomyProgress();
}

void Executor::resumeRoute() {
  suspendAutonomy();
  // Manual movement can return to an earlier route segment. Reconnect from
  // the next measured pose while retaining the planned elevation and goal.
  progress = 0;
  segment.clear();
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

void Executor::commitAppliedCommand(const nav_kernel::Twist &command) {
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
    committed_local_path_map_.clear();
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
  const auto planner_capabilities = local_planner_.capabilities();
  if (planner_capabilities.ownsReferencePath) {
    if (last_scan_tick_s_) {
      const double elapsed = input.timestampS - *last_scan_tick_s_;
      const double max_gap =
          nav_kernel::local::scan::upstream::ClosedLoopController::kMaxUpdateGapS;
      if (elapsed < 0.0 || (!traj_frozen_ && elapsed > max_gap)) {
        suspendAutonomy();
        last_scan_tick_s_ = input.timestampS;
        ExecutionOutput output;
        output.active = input.mode == ExecutionMode::MotionIntent || !route.empty();
        output.near_field_stop = output.active;
        output.reason = "scan_execution_clock_discontinuity";
        return output;
      }
    }
    last_scan_tick_s_ = input.timestampS;
  }
  if (input.mode == ExecutionMode::MotionIntent) {
    if (!pose_stages_.empty() && pose_stages_[pose_stage_].tangentHeading && !intent_mode_)
      suspendAutonomy();
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
    previous_kinematics_time_s_ = -1.0;
    resetLocalPlanning(true);
    recovery_.reset();
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
  if (intent_mode_) suspendAutonomy();
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
        stair_step_.bodyYaw ? stair_step_.bodyYaw : final_yaw_;
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
  if (!moving_spline && !stairs_.pending() && pose_stage_ + 2 == pose_stages_.size() &&
      !pose_stages_[pose_stage_].rotation && pose_stages_.back().rotation &&
      nav_kernel::distance3D(pose_stages_[pose_stage_].points.back(),
                            pose_stages_.back().points.back()) <= 1e-8) {
    const auto &goal = pose_stages_.back().points.back();
    const double tolerance = std::max(0.01, pose_final_tolerance_.value_or(config_.goal_reached_m));
    if (std::hypot(goal.x - map_body.position.x, goal.y - map_body.position.y) <= tolerance &&
        std::abs(goal.z - map_body.position.z) <= config_.goal_height_tolerance_m) {
      const double yaw_error = std::abs(nav_kernel::normalizeAngle(
          pose_stages_.back().yaw - map_body.yaw));
      const double yaw_tolerance = pose_final_yaw_tolerance_.value_or(config_.goal_yaw_tolerance_rad);
      const double speed = std::hypot(observation.body_linear_velocity.x,
                                      observation.body_linear_velocity.y);
      if (yaw_error > yaw_tolerance &&
          (!observation.body_velocity_valid || !std::isfinite(speed) || speed > 0.03)) {
        output.reason = "settling_before_goal_yaw";
        output.path_found = true;
        pauseLinearMotion();
        return output;
      }
      ++pose_stage_;
      activatePoseStage();
    }
  }
  bool measured_stage_arrival = measured_pose_stage_arrival();
  if (!pose_stages_.empty() && pose_stage_ + 1 < pose_stages_.size() &&
      (!moving_spline || !pose_stages_[pose_stage_ + 1].rotation) &&
      (atGoal(map_body) || initial_turn_stage() || measured_stage_arrival) &&
      std::abs(execution_yaw_error()) <= (measured_stage_arrival
          ? std::max(active_goal_yaw_tolerance_rad_, config_.goal_yaw_tolerance_rad)
          : active_goal_yaw_tolerance_rad_)) {
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
  goal_body.z += height_offset_.value_or(0.0);
  const auto stair_goal = stairs_.guideGoal(goal_body, active_goal_reached_m_, map_body.yaw);
  if (!pose_stages_.empty() && pose_stage_+1 == pose_stages_.size() &&
      pose_requested_final_yaw_ && stair_step_.phase == StairPhase::Traversing &&
      stair_goal.arrivalAllowed && stair_step_.bodyYaw && atGoal(map_body) &&
      std::abs(nav_kernel::normalizeAngle(*pose_requested_final_yaw_-*stair_step_.bodyYaw)) >
          active_goal_yaw_tolerance_rad_) {
    output.reason = "stair_goal_heading_wait";
    output.near_field_stop = true;
    pauseLinearMotion();
    return output;
  }
  if (stair_goal.hold) {
    output.reason = "stair_exit_target";
    output.near_field_stop = true;
    pauseLinearMotion();
    return output;
  }
  const auto stair_target = stair_step_.approachTarget
      ? stair_step_.approachTarget : stair_goal.target;
  if (!moving_spline && !stair_target && stair_goal.arrivalAllowed &&
      (atGoal(map_body) || initial_turn_stage() || measured_stage_arrival)) {
    recovery_action_ = 0;
    recovery_attempt_ = -1;
    recovery_follower_.reset();
    resetLocalPlanning(true);
    recovery_.reset();
    resetAutonomyProgress();
    const double yaw_error = execution_yaw_error();
    if (final_yaw_) goal_heading_active_ = true;
    if (final_yaw_ && std::abs(yaw_error) > active_goal_yaw_tolerance_rad_) {
      output.path_found = true;
      output.reason = "aligning_goal_yaw";
      output.target_index = route.size() - 1;
      // Turning can move a legged base slightly; the initial turn stays at the
      // measured pose, then SCAN connects it to the unchanged global reference.
      output.target = initial_turn_stage() ? map_body.position : route.back();
      if (!initial_turn_stage()) output.target.z += height_offset_.value_or(0.0);
      output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
      output.cmd_vel.wz = nav_kernel::Follower::turnRate(
          yaw_error, config_.goal_yaw_kp, config_.goal_yaw_min_rate, config_.goal_yaw_max_rate);
      if (route_body_) {
        auto turn_request = makeLocalPlanRequest(
            planning_body, route, nullptr, generation, {}, observation,
            obstacle_xyzh_planning, obstacle_count, timestamp_s, true, traversability, false);
        turn_request.body = route_body_;
        turn_request.observedBody = observation.body_geometry;
        const double target_yaw = nav_kernel::normalizeAngle(
            *execution_goal_yaw() - map_from_odom.yaw);
        if (!local_planner_.turnClear(turn_request, target_yaw)) {
          output.cmd_vel = {};
          output.path_found = false;
          output.near_field_stop = true;
          output.reason = "pose_turn_blocked";
        }
      }
      return output;
    }
    output.target_index = route.size() - 1;
    output.target = route.back();
    output.target.z += height_offset_.value_or(0.0);
    output.target_distance_m = nav_kernel::distance3D(output.target, map_body.position);
    // Entering the radius while braking is not an arrival.
    const double stamp = observation.odom_stamp_s;
    const double speed = std::hypot(observation.body_linear_velocity.x,
                                    observation.body_linear_velocity.y,
                                    observation.body_linear_velocity.z);
    const bool valid_motion = observation.body_velocity_valid && std::isfinite(speed) &&
                              std::isfinite(observation.body_yaw_rate) &&
                              std::isfinite(stamp) && stamp > 0.0;
    const bool quiet = valid_motion && speed <= 0.03 &&
                       std::abs(observation.body_yaw_rate) <= 0.08;
    output.goal_reached = quiet &&
        (pose_stages_.empty() || pose_stage_ + 1 == pose_stages_.size());
    output.active = !output.goal_reached;
    output.path_found = true;
    output.reason = output.goal_reached ? "goal_reached" : "settling_at_goal";
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
    output.target.z += height_offset_.value_or(0.0);
  } else {
    target = buildSegment(map_body, planning_body, map_from_odom);
    buildReference(target, map_from_odom);
    applyCommittedLocalGuide(map_body, planning_body, map_from_odom, timestamp_s);
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
          config_.goal_yaw_kp, config_.goal_yaw_min_rate, config_.goal_yaw_max_rate);
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
  // Turning can drift outside the arrival radius. Position correction must
  // keep the final heading rather than turn back toward the cruise tangent.
  if (goal_heading_active_)
    plan_request.bodyHeading = nav_kernel::normalizeAngle(
        *execution_goal_yaw() - map_from_odom.yaw);
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
  const bool path_provided = std::holds_alternative<nav_kernel::PathTarget>(plan.target());
  if (path_provided && output.local_path_body.size() < 2) {
    output.reason = "untrackable_local_path";
    follower_.stopLinear();
    resetAutonomyProgress();
    return output;
  }
  nav_kernel::FollowerState follower_state;
  follower_state.measuredBodyTwistValid = observation.body_velocity_valid;
  follower_state.measuredBodyTwist =
      observation.body_velocity_valid
          ? nav_kernel::Twist{observation.body_linear_velocity.x,
                              observation.body_linear_velocity.y, observation.body_yaw_rate}
          : nav_kernel::Twist{};
  follower_state.currentTime = timestamp_s;
  if (spline_provided) {
    follower_state.vehicleRelative = planning_body.position;
    follower_state.vehicleYawRelative = planning_body.yaw;
    follower_state.desiredHeading = plan_request.bodyHeading;
    if (!pose_stages_.empty() && pose_stages_[pose_stage_].tangentHeading &&
        nav_kernel::distance3D(map_body.position,route.back())<=
                           config_.follower.spline.finishDistance) {
      // A millimetre-scale arrival correction may point behind the robot.
      // Keep the admitted terminal heading while correcting position, rather
      // than rotating through 180 degrees to follow that short return tangent.
      // Arrival itself still uses the task's unchanged position/yaw tolerances.
      follower_state.desiredHeading=nav_kernel::normalizeAngle(
          pose_stages_[pose_stage_].yaw-map_from_odom.yaw);
    }
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
      odom_map_body, intent_route, nullptr, generation, kinematics, observation,
      obstacle_xyzh, obstacle_count, timestamp_s,
      traj_frozen_, traversability,
      local_planner_.capabilities().consumesTraversability,
      &motion_intent);
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
  resetTeleopBoundaryDeparture();

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
