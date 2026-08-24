#include "navigation/navigator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace lingtu::nav::navigation {
namespace {

double clamp01(double value) {
  return std::max(0.0, std::min(1.0, value));
}

double bodyDistance2D(const nav_kernel::Vec3 &point) {
  return std::hypot(point.x, point.y);
}

bool sameIdentity(const nav_kernel::LocalPlanIdentity &left,
                  const nav_kernel::LocalPlanIdentity &right) {
  return left.frameEpoch == right.frameEpoch &&
         left.obstacleGeneration == right.obstacleGeneration &&
         left.traversabilityGeneration == right.traversabilityGeneration;
}

nav_kernel::TrajectoryPoint
interpolateTrajectory(const std::vector<nav_kernel::TrajectoryPoint> &trajectory, double time_s) {
  if (trajectory.empty())
    return {};
  if (time_s <= trajectory.front().timeFromStartS)
    return trajectory.front();
  const auto upper = std::lower_bound(trajectory.begin() + 1, trajectory.end(), time_s,
                                      [](const nav_kernel::TrajectoryPoint &point, double time) {
                                        return point.timeFromStartS < time;
                                      });
  if (upper == trajectory.end())
    return trajectory.back();

  const auto &before = *std::prev(upper);
  const auto &after = *upper;
  const double duration = after.timeFromStartS - before.timeFromStartS;
  const double ratio =
      duration > 1e-9 ? std::clamp((time_s - before.timeFromStartS) / duration, 0.0, 1.0) : 0.0;
  const auto blend = [ratio](const nav_kernel::Vec3 &left, const nav_kernel::Vec3 &right) {
    return nav_kernel::Vec3{
        left.x + ratio * (right.x - left.x),
        left.y + ratio * (right.y - left.y),
        left.z + ratio * (right.z - left.z),
    };
  };
  nav_kernel::TrajectoryPoint point;
  point.position = blend(before.position, after.position);
  point.velocity = blend(before.velocity, after.velocity);
  point.acceleration = blend(before.acceleration, after.acceleration);
  point.yaw = nav_kernel::normalizeAngle(
      before.yaw + ratio * nav_kernel::normalizeAngle(after.yaw - before.yaw));
  point.yawRate = before.yawRate + ratio * (after.yawRate - before.yawRate);
  point.timeFromStartS = time_s;
  return point;
}

nav_kernel::Vec3 bodyPointToPlanning(const nav_kernel::Pose &body, const nav_kernel::Vec3 &point) {
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  return {
      body.position.x + c * point.x - s * point.y,
      body.position.y + s * point.x + c * point.y,
      body.position.z + point.z,
  };
}

nav_kernel::Vec3 planningPointToBody(const nav_kernel::Pose &body, const nav_kernel::Vec3 &point) {
  const double dx = point.x - body.position.x;
  const double dy = point.y - body.position.y;
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  return {
      c * dx + s * dy,
      -s * dx + c * dy,
      point.z - body.position.z,
  };
}

nav_kernel::Vec3 bodyVectorToPlanning(const nav_kernel::Pose &body,
                                      const nav_kernel::Vec3 &vector) {
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  return {
      c * vector.x - s * vector.y,
      s * vector.x + c * vector.y,
      vector.z,
  };
}

nav_kernel::Vec3 planningVectorToBody(const nav_kernel::Pose &body,
                                      const nav_kernel::Vec3 &vector) {
  const double c = std::cos(body.yaw);
  const double s = std::sin(body.yaw);
  return {
      c * vector.x + s * vector.y,
      -s * vector.x + c * vector.y,
      vector.z,
  };
}

nav_kernel::LocalPlanResult rebasePlan(const nav_kernel::local::LocalPlanCompletion &completion,
                                       const nav_kernel::Pose &current_body, double timestamp_s,
                                       double execution_delay_s) {
  nav_kernel::LocalPlanResult result = completion.result;
  if (!result.pathFound)
    return result;

  const auto to_current_body = [&](const nav_kernel::Vec3 &point) {
    return planningPointToBody(current_body, bodyPointToPlanning(completion.vehicle, point));
  };
  const auto vector_to_current_body = [&](const nav_kernel::Vec3 &vector) {
    return planningVectorToBody(current_body, bodyVectorToPlanning(completion.vehicle, vector));
  };

  if (!completion.result.trajectory.empty()) {
    const double execution_time_s = timestamp_s - std::max(0.0, execution_delay_s);
    const double local_execution_time_s = execution_time_s - completion.timestampS;
    if (local_execution_time_s >= completion.result.trajectory.back().timeFromStartS - 1e-6) {
      result.status = nav_kernel::LocalPlanStatus::NoPath;
      result.reason = "scan_trajectory_expired";
      result.pathFound = false;
      result.nearFieldStop = true;
      result.path.clear();
      result.trajectory.clear();
      return result;
    }
    result.trajectory.clear();
    nav_kernel::TrajectoryPoint current =
        interpolateTrajectory(completion.result.trajectory, std::max(0.0, local_execution_time_s));
    current.position = to_current_body(current.position);
    current.velocity = vector_to_current_body(current.velocity);
    current.acceleration = vector_to_current_body(current.acceleration);
    current.yaw =
        nav_kernel::normalizeAngle(completion.vehicle.yaw + current.yaw - current_body.yaw);
    current.timeFromStartS = 0.0;
    result.trajectory.push_back(current);
    for (const auto &sample : completion.result.trajectory) {
      const double absolute_time = completion.timestampS + sample.timeFromStartS;
      if (absolute_time <= execution_time_s + 1e-6)
        continue;
      nav_kernel::TrajectoryPoint rebased;
      rebased.position = to_current_body(sample.position);
      rebased.velocity = vector_to_current_body(sample.velocity);
      rebased.acceleration = vector_to_current_body(sample.acceleration);
      rebased.yaw =
          nav_kernel::normalizeAngle(completion.vehicle.yaw + sample.yaw - current_body.yaw);
      rebased.yawRate = sample.yawRate;
      rebased.timeFromStartS = absolute_time - execution_time_s;
      result.trajectory.push_back(rebased);
    }
    if (result.trajectory.size() < 2U) {
      result.status = nav_kernel::LocalPlanStatus::NoPath;
      result.reason = "scan_trajectory_expired";
      result.pathFound = false;
      result.nearFieldStop = true;
      result.path.clear();
      result.trajectory.clear();
      return result;
    }
    result.path.clear();
    result.path.reserve(result.trajectory.size());
    for (const auto &sample : result.trajectory) {
      result.path.push_back(sample.position);
    }
    return result;
  }

  std::vector<nav_kernel::Vec3> rebased_path;
  rebased_path.reserve(completion.result.path.size());
  for (const auto &point : completion.result.path) {
    rebased_path.push_back(to_current_body(point));
  }
  const auto closest = std::min_element(rebased_path.begin(), rebased_path.end(),
                                        [](const auto &left, const auto &right) {
                                          return bodyDistance2D(left) < bodyDistance2D(right);
                                        });
  result.path.clear();
  result.path.push_back({});
  if (closest != rebased_path.end()) {
    for (auto point = closest; point != rebased_path.end(); ++point) {
      if (nav_kernel::distance3D(result.path.back(), *point) > 1e-4) {
        result.path.push_back(*point);
      }
    }
  }
  if (result.path.size() < 2U) {
    result.status = nav_kernel::LocalPlanStatus::NoPath;
    result.reason = "scan_path_expired";
    result.pathFound = false;
    result.nearFieldStop = true;
    result.path.clear();
  }
  return result;
}

nav_kernel::LocalPlanInput makeLocalPlanInput(
    const nav_kernel::Pose &vehicle, const std::vector<nav_kernel::Vec3> &route,
    std::uint64_t route_generation, bool reaches_goal, nav_kernel::LocalKinematicState kinematics,
    const NavigatorObservation &observation, const float *obstacle_xyzh, int obstacle_count,
    double timestamp_s, const TraversabilityGridView &traversability) {
  nav_kernel::LocalPlanInput input;
  input.vehicle = vehicle;
  input.route = {
      route.empty() ? nullptr : route.data(),
      static_cast<int>(route.size()),
      route_generation,
      reaches_goal,
  };
  input.kinematics = kinematics;
  input.identity = {
      observation.frame_epoch,
      observation.collision.present() ? observation.collision.generation
                                      : observation.cloud_generation,
      observation.traversability_generation,
  };
  input.obstacles = {obstacle_xyzh, obstacle_count};
  input.collision = observation.collision;
  input.timestampS = timestamp_s;
  if (traversability.valid()) {
    input.traversability = {
        traversability.values,     traversability.rows,     traversability.cols,
        traversability.resolution, traversability.origin_x, traversability.origin_y,
    };
  }
  return input;
}

bool sameIntent(const nav_kernel::LocalMotionIntent &left,
                const nav_kernel::LocalMotionIntent &right) {
  const double direction_error = std::abs(
      nav_kernel::normalizeAngle((left.directionBodyDeg - right.directionBodyDeg) * M_PI / 180.0));
  return direction_error <= 2.0 * M_PI / 180.0 &&
         std::abs(left.speedNormalized - right.speedNormalized) <= 0.05 &&
         std::abs(left.horizonM - right.horizonM) <= 0.05 &&
         std::abs(left.maxDirectionDeviationDeg - right.maxDirectionDeviationDeg) <= 1e-6;
}

}  // namespace

bool TraversabilityGridView::valid() const {
  return values != nullptr && rows > 0 && cols > 0 && resolution > 0.0;
}

Navigator::Navigator(NavigatorConfig config)
    : config_(std::move(config)),
      local_planner_(config_.planner),
      scan_task_(config_.planner.backend == nav_kernel::LocalPlannerBackend::Scan
                     ? std::make_unique<nav_kernel::local::LocalPlanTask>(config_.planner)
                     : nullptr),
      recovery_(config_.planner, config_.recovery) {
  config_.goal_yaw_tolerance_rad = std::max(0.0, config_.goal_yaw_tolerance_rad);
  config_.goal_yaw_kp = std::max(0.0, config_.goal_yaw_kp);
  config_.goal_yaw_max_rate = std::max(0.0, config_.goal_yaw_max_rate);
  config_.follower.maxSpeed = config_.max_speed;
  config_.recovery.translation_speed_mps =
      std::clamp(config_.recovery.translation_speed_mps, 0.0, config_.max_speed);
  config_.recovery.rotation_rate_rad_s = std::max(0.0, config_.recovery.rotation_rate_rad_s);
  config_.recovery.blocked_interval_s = std::max(0.0, config_.recovery.blocked_interval_s);
  config_.recovery.stuck_linear_progress_m =
      std::max(0.0, config_.recovery.stuck_linear_progress_m);
  config_.recovery.stuck_yaw_progress_rad = std::max(0.0, config_.recovery.stuck_yaw_progress_rad);
  active_goal_reached_m_ = config_.goal_reached_m;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
}

bool Navigator::configure() {
  configured_ = local_planner_.configure(config_.path_library_dir);
  if (configured_ && scan_task_) {
    configured_ = scan_task_->configure(config_.path_library_dir);
  }
  return configured_;
}

bool Navigator::configured() const {
  return configured_;
}

void Navigator::resetLocalPlanning() {
  local_planner_.reset();
  if (scan_task_)
    scan_task_->reset();
  scan_plan_.reset();
  scan_submitted_identity_ = {};
  scan_submitted_route_generation_ = 0;
  scan_submitted_time_s_ = -1.0;
  scan_submitted_intent_.reset();
  scan_intent_mode_ = false;
  traj_delay_s_ = 0.0;
  traj_clock_s_ = -1.0;
  traj_stamp_s_ = -1.0;
  traj_frozen_ = false;
}

void Navigator::advanceTrajectoryClock(double timestamp_s) {
  if (!std::isfinite(timestamp_s))
    return;
  if (traj_clock_s_ >= 0.0 && timestamp_s >= traj_clock_s_) {
    const double elapsed = timestamp_s - traj_clock_s_;
    if (traj_frozen_ || elapsed > 0.20)
      traj_delay_s_ += elapsed;
  }
  traj_clock_s_ = timestamp_s;
}

nav_kernel::LocalPlanResult Navigator::planAutonomy(const nav_kernel::LocalPlanInput &input,
                                                    const MapFromOdomTransform &map_from_odom,
                                                    double timestamp_s,
                                                    nav_kernel::LocalPlannerDebugSnapshot *debug) {
  const auto debug_anchor = [&](const nav_kernel::Pose &planning_body) {
    nav_kernel::Pose map_body;
    map_body.position = map_from_odom.mapPointFromOdom(planning_body.position);
    map_body.yaw = nav_kernel::normalizeAngle(map_from_odom.yaw + planning_body.yaw);
    return map_body;
  };

  if (!scan_task_) {
    nav_kernel::LocalPlanResult result = local_planner_.plan(input);
    *debug = debugSnapshotToMap(debug_anchor(input.vehicle), local_planner_.debugSnapshot());
    return result;
  }

  if (scan_intent_mode_) {
    scan_task_->reset();
    scan_plan_.reset();
    scan_submitted_identity_ = {};
    scan_submitted_route_generation_ = 0;
    scan_submitted_time_s_ = -1.0;
    scan_submitted_intent_.reset();
    scan_intent_mode_ = false;
  }

  if (auto completion = scan_task_->poll()) {
    const bool same_route = completion->routeGeneration == input.route.generation;
    if (same_route && sameIdentity(completion->identity, input.identity)) {
      const bool new_trajectory =
          !scan_plan_ || std::abs(completion->timestampS - traj_stamp_s_) > 1e-9;
      const double accepted_stamp = completion->timestampS;
      scan_plan_ = std::move(completion);
      if (new_trajectory) {
        traj_delay_s_ = 0.0;
        traj_frozen_ = false;
        traj_stamp_s_ = accepted_stamp;
        traj_clock_s_ = timestamp_s;
      }
    }
  }

  const bool identity_changed =
      input.identity.frameEpoch != scan_submitted_identity_.frameEpoch ||
      input.identity.obstacleGeneration != scan_submitted_identity_.obstacleGeneration ||
      input.identity.traversabilityGeneration != scan_submitted_identity_.traversabilityGeneration;
  const double refresh_interval = 0.5 * std::max(0.05, config_.planner.scan.continuityHorizon);
  const bool refresh_due = scan_submitted_time_s_ < 0.0 || !std::isfinite(timestamp_s) ||
                           timestamp_s < scan_submitted_time_s_ ||
                           timestamp_s - scan_submitted_time_s_ >= refresh_interval;
  if (identity_changed || input.route.generation != scan_submitted_route_generation_ ||
      refresh_due) {
    if (scan_task_->submit(input) != 0U) {
      scan_submitted_identity_ = input.identity;
      scan_submitted_route_generation_ = input.route.generation;
      scan_submitted_time_s_ = timestamp_s;
    }
  }

  const bool current_plan = scan_plan_ && scan_plan_->routeGeneration == input.route.generation &&
                            sameIdentity(scan_plan_->identity, input.identity);
  if (!current_plan) {
    nav_kernel::LocalPlanResult pending;
    pending.backend = nav_kernel::LocalPlannerBackend::Scan;
    pending.status = nav_kernel::LocalPlanStatus::NoPath;
    pending.reason = "local_plan_pending";
    pending.nearFieldStop = true;
    *debug = {};
    debug->backend = nav_kernel::LocalPlannerBackend::Scan;
    return pending;
  }

  const double age = timestamp_s - scan_plan_->timestampS;
  if (!std::isfinite(age) || age < -0.10 ||
      age > std::max(0.10, config_.planner.scan.collisionMaxAge)) {
    nav_kernel::LocalPlanResult stale;
    stale.backend = nav_kernel::LocalPlannerBackend::Scan;
    stale.status = nav_kernel::LocalPlanStatus::NoPath;
    stale.reason = "local_plan_stale";
    stale.nearFieldStop = true;
    *debug = debugSnapshotToMap(debug_anchor(scan_plan_->vehicle), scan_plan_->debug);
    return stale;
  }

  *debug = debugSnapshotToMap(debug_anchor(scan_plan_->vehicle), scan_plan_->debug);
  return rebasePlan(*scan_plan_, input.vehicle, timestamp_s, traj_delay_s_);
}

nav_kernel::LocalPlanResult Navigator::planTeleop(const nav_kernel::LocalPlanInput &input,
                                                  const nav_kernel::LocalMotionIntent &intent,
                                                  double timestamp_s,
                                                  nav_kernel::LocalPlannerDebugSnapshot *debug) {
  if (!scan_task_) {
    nav_kernel::LocalPlanResult result = local_planner_.planIntent(input, intent);
    *debug = debugSnapshotToMap(input.vehicle, local_planner_.debugSnapshot());
    return result;
  }

  if (!scan_intent_mode_) {
    scan_task_->reset();
    scan_plan_.reset();
    scan_submitted_identity_ = {};
    scan_submitted_route_generation_ = 0;
    scan_submitted_time_s_ = -1.0;
    scan_submitted_intent_.reset();
    scan_intent_mode_ = true;
  }

  if (auto completion = scan_task_->poll()) {
    if (completion->intent && sameIntent(*completion->intent, intent) &&
        completion->routeGeneration == input.route.generation &&
        sameIdentity(completion->identity, input.identity)) {
      const double accepted_stamp = completion->timestampS;
      scan_plan_ = std::move(completion);
      traj_delay_s_ = 0.0;
      traj_frozen_ = false;
      traj_stamp_s_ = accepted_stamp;
      traj_clock_s_ = timestamp_s;
    }
  }

  const bool identity_changed =
      input.identity.frameEpoch != scan_submitted_identity_.frameEpoch ||
      input.identity.obstacleGeneration != scan_submitted_identity_.obstacleGeneration ||
      input.identity.traversabilityGeneration != scan_submitted_identity_.traversabilityGeneration;
  const bool intent_changed =
      !scan_submitted_intent_ || !sameIntent(*scan_submitted_intent_, intent);
  const double refresh_interval = 0.5 * std::max(0.05, config_.planner.scan.continuityHorizon);
  const bool refresh_due = scan_submitted_time_s_ < 0.0 || !std::isfinite(timestamp_s) ||
                           timestamp_s < scan_submitted_time_s_ ||
                           timestamp_s - scan_submitted_time_s_ >= refresh_interval;
  if (identity_changed || intent_changed || refresh_due) {
    if (scan_task_->submitIntent(input, intent) != 0U) {
      scan_submitted_identity_ = input.identity;
      scan_submitted_route_generation_ = input.route.generation;
      scan_submitted_time_s_ = timestamp_s;
      scan_submitted_intent_ = intent;
    }
  }

  const bool current_plan = scan_plan_ && scan_plan_->intent &&
                            sameIntent(*scan_plan_->intent, intent) &&
                            scan_plan_->routeGeneration == input.route.generation &&
                            sameIdentity(scan_plan_->identity, input.identity);
  if (!current_plan) {
    nav_kernel::LocalPlanResult pending;
    pending.backend = nav_kernel::LocalPlannerBackend::Scan;
    pending.status = nav_kernel::LocalPlanStatus::NoPath;
    pending.reason = "local_intent_pending";
    pending.nearFieldStop = true;
    *debug = {};
    debug->backend = nav_kernel::LocalPlannerBackend::Scan;
    return pending;
  }

  const double age = timestamp_s - scan_plan_->timestampS;
  if (!std::isfinite(age) || age < -0.10 ||
      age > std::max(0.10, config_.planner.scan.collisionMaxAge)) {
    nav_kernel::LocalPlanResult stale;
    stale.backend = nav_kernel::LocalPlannerBackend::Scan;
    stale.status = nav_kernel::LocalPlanStatus::NoPath;
    stale.reason = "local_intent_stale";
    stale.nearFieldStop = true;
    *debug = debugSnapshotToMap(scan_plan_->vehicle, scan_plan_->debug);
    return stale;
  }

  *debug = debugSnapshotToMap(scan_plan_->vehicle, scan_plan_->debug);
  return rebasePlan(*scan_plan_, input.vehicle, timestamp_s, traj_delay_s_);
}

void Navigator::setRoute(const std::vector<nav_kernel::Vec3> &path, std::optional<double> final_yaw,
                         std::optional<double> goal_reached_m,
                         std::optional<double> goal_yaw_tolerance_rad) {
  route = path;
  ++generation;
  final_yaw_ = final_yaw;
  active_goal_reached_m_ = std::max(0.01, goal_reached_m.value_or(config_.goal_reached_m));
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
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  resetAutonomyProgress();
}

void Navigator::clearRoute() {
  route.clear();
  ++generation;
  final_yaw_.reset();
  active_goal_reached_m_ = config_.goal_reached_m;
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
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  clearRecoveryObservationWait();
  resetAutonomyProgress();
}

bool Navigator::hasRoute() const {
  return !route.empty();
}

void Navigator::suspendAutonomy() {
  follower_.reset();
  recovery_follower_.reset();
  recovery_action_ = 0;
  recovery_attempt_ = -1;
  resetLocalPlanning();
  recovery_.reset();
  committed_local_path_map_.clear();
  committed_route_generation_ = 0;
  committed_local_path_time_s_ = -1.0;
  previous_kinematics_time_s_ = -1.0;
  local_blocked_since_s_ = -1.0;
  clearRecoveryObservationWait();
  resetAutonomyProgress();
}

void Navigator::stopLinearMotion() {
  follower_.stopLinear();
  recovery_follower_.stopLinear();
}

NavigatorOutput Navigator::tick(const nav_kernel::Pose &odom_map_body, const float *obstacle_xyzh,
                                int obstacle_count, double timestamp_s,
                                TraversabilityGridView traversability,
                                NavigatorObservation observation) {
  return tickInPlanningFrame(odom_map_body, odom_map_body, {}, obstacle_xyzh, obstacle_count,
                             timestamp_s, traversability, observation);
}

NavigatorOutput Navigator::tickOdom(const nav_kernel::Pose &map_body,
                                    const nav_kernel::Pose &odom_body,
                                    const MapFromOdomTransform &map_from_odom,
                                    const float *obstacle_xyzh_map, int obstacle_count,
                                    double timestamp_s, TraversabilityGridView odom_traversability,
                                    NavigatorObservation observation) {
  if (!map_from_odom.valid()) {
    NavigatorOutput output;
    output.active = configured_ && !route.empty();
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

  const bool collision_authoritative =
      config_.planner.backend == nav_kernel::LocalPlannerBackend::Scan &&
      observation.collision.present();
  const int safe_obstacle_count =
      collision_authoritative || obstacle_xyzh_map == nullptr ? 0 : std::max(0, obstacle_count);
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

  if (observation.collision.present() && observation.collision.occupiedCount >= 0 &&
      (observation.collision.occupiedCount == 0 || observation.collision.occupiedXyz != nullptr)) {
    const auto &source = observation.collision;
    const bool transform_unchanged =
        collision_cache_.valid && collision_cache_.transform.yaw == map_from_odom.yaw &&
        collision_cache_.transform.translation.x == map_from_odom.translation.x &&
        collision_cache_.transform.translation.y == map_from_odom.translation.y &&
        collision_cache_.transform.translation.z == map_from_odom.translation.z;
    const bool collision_unchanged = collision_cache_.valid &&
                                     collision_cache_.reset_epoch == source.resetEpoch &&
                                     collision_cache_.sequence == source.observationSequence &&
                                     collision_cache_.generation == source.generation &&
                                     collision_cache_.count == source.occupiedCount;
    if (!transform_unchanged || !collision_unchanged) {
      collision_xyz_odom_scratch_.clear();
      collision_xyz_odom_scratch_.reserve(static_cast<std::size_t>(source.occupiedCount) * 3U);
      const double c = std::cos(map_from_odom.yaw);
      const double s = std::sin(map_from_odom.yaw);
      const auto point_to_odom = [&](const nav_kernel::Vec3 &point_map) {
        const double dx = point_map.x - map_from_odom.translation.x;
        const double dy = point_map.y - map_from_odom.translation.y;
        return nav_kernel::Vec3{
            c * dx + s * dy,
            -s * dx + c * dy,
            point_map.z - map_from_odom.translation.z,
        };
      };
      for (int index = 0; index < source.occupiedCount; ++index) {
        const float *point_xyz = source.occupiedXyz + index * 3;
        const nav_kernel::Vec3 point = point_to_odom({point_xyz[0], point_xyz[1], point_xyz[2]});
        collision_xyz_odom_scratch_.push_back(static_cast<float>(point.x));
        collision_xyz_odom_scratch_.push_back(static_cast<float>(point.y));
        collision_xyz_odom_scratch_.push_back(static_cast<float>(point.z));
      }

      nav_kernel::Vec3 aabb_min{
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
          std::numeric_limits<double>::infinity(),
      };
      nav_kernel::Vec3 aabb_max{
          -std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(),
      };
      for (const double x : {source.aabbMin.x, source.aabbMax.x}) {
        for (const double y : {source.aabbMin.y, source.aabbMax.y}) {
          for (const double z : {source.aabbMin.z, source.aabbMax.z}) {
            const nav_kernel::Vec3 point = point_to_odom({x, y, z});
            aabb_min.x = std::min(aabb_min.x, point.x);
            aabb_min.y = std::min(aabb_min.y, point.y);
            aabb_min.z = std::min(aabb_min.z, point.z);
            aabb_max.x = std::max(aabb_max.x, point.x);
            aabb_max.y = std::max(aabb_max.y, point.y);
            aabb_max.z = std::max(aabb_max.z, point.z);
          }
        }
      }
      collision_cache_ = {
          true,
          source.resetEpoch,
          source.observationSequence,
          source.generation,
          source.occupiedCount,
          map_from_odom,
          aabb_min,
          aabb_max,
      };
    }
    observation.collision.occupiedXyz =
        collision_xyz_odom_scratch_.empty() ? nullptr : collision_xyz_odom_scratch_.data();
    observation.collision.aabbMin = collision_cache_.aabb_min;
    observation.collision.aabbMax = collision_cache_.aabb_max;
  } else {
    collision_cache_.valid = false;
    collision_xyz_odom_scratch_.clear();
  }

  return tickInPlanningFrame(
      map_body, odom_body, map_from_odom,
      obstacle_xyzh_odom_scratch_.empty() ? nullptr : obstacle_xyzh_odom_scratch_.data(),
      safe_obstacle_count, timestamp_s, odom_traversability, observation);
}

NavigatorOutput Navigator::tickInPlanningFrame(const nav_kernel::Pose &map_body,
                                               const nav_kernel::Pose &planning_body,
                                               const MapFromOdomTransform &map_from_odom,
                                               const float *obstacle_xyzh_planning,
                                               int obstacle_count, double timestamp_s,
                                               TraversabilityGridView traversability,
                                               NavigatorObservation observation) {
  advanceTrajectoryClock(timestamp_s);
  NavigatorOutput output;
  if (!configured_) {
    output.reason = "not_configured";
    return output;
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
  const nav_kernel::LocalPlanInput plan_input = makeLocalPlanInput(
      planning_body, segment, generation, target.reachesGoal, kinematics, observation,
      obstacle_xyzh_planning, obstacle_count, timestamp_s, traversability);
  nav_kernel::LocalPlanResult plan =
      planAutonomy(plan_input, map_from_odom, timestamp_s, &output.local_planner_debug);

  const bool planner_input_valid = plan.status != nav_kernel::LocalPlanStatus::InvalidInput &&
                                   plan.status != nav_kernel::LocalPlanStatus::NotConfigured;
  if (!plan.pathFound && planner_input_valid &&
      (local_blocked_since_s_ < 0.0 || local_blocked_since_s_ > timestamp_s)) {
    local_blocked_since_s_ = timestamp_s;
  }
  const bool blocked_long_enough =
      local_blocked_since_s_ >= 0.0 &&
      timestamp_s - local_blocked_since_s_ >= std::max(0.0, config_.recovery.blocked_interval_s);
  const bool force_recovery = recovery_.active() || blocked_long_enough ||
                              autonomyMotionStalled(planning_body, timestamp_s);
  RecoveryOutput recovery;
  if (force_recovery && planner_input_valid) {
    recovery = recovery_.step(plan_input);
  } else if (!planner_input_valid || plan.pathFound) {
    recovery_.reset();
  }

  output.path_found =
      recovery.active ? recovery.verified && recovery.path_body.size() >= 2 : plan.pathFound;
  output.near_field_stop = plan.nearFieldStop;
  output.slow_down = std::max(0, std::min(3, plan.slowDown));
  output.recovery_state = recovery.state;
  output.recovery_action = static_cast<int>(recovery.action);
  output.recovery_attempt = recovery.attempt;
  output.recovery_candidate_count = recovery.candidate_count;
  output.recovery_verified = recovery.verified;
  output.recovery_observation_refresh_required = recovery.observation_refresh_required;
  output.recovery_progress = recovery.progress;
  output.recovery_reason = recovery.reason;
  output.recovery_exhausted = recovery.exhausted;
  output.local_path_body = recovery.active ? recovery.path_body : plan.path;
  output.local_trajectory_body =
      recovery.active ? std::vector<nav_kernel::TrajectoryPoint>{} : plan.trajectory;

  if (!plan.pathFound && !recovery.active && !recovery.exhausted && planner_input_valid) {
    if (local_blocked_since_s_ < 0.0 || local_blocked_since_s_ > timestamp_s) {
      local_blocked_since_s_ = timestamp_s;
    }
  } else if (plan.pathFound || recovery.active) {
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

  if (output.path_found && !plan.nearFieldStop && output.local_path_body.size() < 2) {
    if (output.local_path_body.size() == 1 &&
        bodyDistance2D(output.local_path_body.front()) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
  }
  output.local_path_map = bodyPathToMap(map_body, output.local_path_body);
  if (!recovery.active && plan.pathFound && !plan.nearFieldStop &&
      output.local_path_map.size() >= 2) {
    committed_local_path_map_ = output.local_path_map;
    committed_route_generation_ = generation;
    committed_local_path_time_s_ = timestamp_s;
  } else if (recovery.active) {
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
    recovery_params.maxYawRate = 0.0;
    recovery_params.twoWayDrive = false;
    recovery_params.dirDiffThre = M_PI + 0.1;
    recovery_params.omniDirDiffThre = M_PI + 0.1;
    recovery_params.omniDirGoalThre = std::max(2.0, config_.planner.adjacentRange);
    recovery_params.noRotAtGoal = true;

    nav_kernel::FollowerInput recovery_input(output.local_path_body);
    recovery_input.requestedSpeed = 1.0;
    recovery_input.currentTime = timestamp_s;
    recovery_input.params = recovery_params;
    recovery_input.goalDistance = bodyDistance2D(output.local_path_body.back());
    const nav_kernel::FollowerOutput recovery_control = recovery_follower_.follow(recovery_input);
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
  if (plan.nearFieldStop) {
    output.reason = plan.reason.empty() ? "near_field_stop" : plan.reason;
    follower_.stopLinear();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }
  const bool trajectory_provided = !output.local_trajectory_body.empty();
  if (trajectory_provided && output.local_trajectory_body.size() < 2) {
    output.reason = plan.reason.empty() ? "untrackable_local_trajectory" : plan.reason;
    follower_.reset();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }
  if (!trajectory_provided && output.local_path_body.size() < 2) {
    output.reason = plan.pathFound ? "untrackable_local_path"
                                   : (plan.reason.empty() ? "no_local_path" : plan.reason);
    follower_.stopLinear();
    setAutonomyMotionExpected(false, planning_body, timestamp_s);
    return output;
  }
  nav_kernel::FollowerInput follower_input =
      trajectory_provided ? nav_kernel::FollowerInput(output.local_trajectory_body)
                          : nav_kernel::FollowerInput(output.local_path_body);
  follower_input.measuredBodyTwist =
      observation.body_velocity_valid
          ? nav_kernel::Twist{observation.body_linear_velocity.x,
                              observation.body_linear_velocity.y, observation.body_yaw_rate}
          : nav_kernel::Twist{};
  follower_input.currentTime = timestamp_s;
  follower_input.slowFactor = slowFactor(output.slow_down);
  follower_input.params = config_.follower;
  follower_input.goalDistance = nav_kernel::distance3D(route.back(), map_body.position);
  const nav_kernel::FollowerOutput control = follower_.follow(follower_input);
  output.cmd_vel = control.cmd;
  output.trajectory_frozen = control.executionFrozen;
  traj_frozen_ = control.executionFrozen;
  output.reason = trajectory_provided ? "trajectory_control_ready" : "control_ready";
  const bool command_expects_motion =
      std::hypot(output.cmd_vel.vx, output.cmd_vel.vy) > 1e-6 || std::abs(output.cmd_vel.wz) > 1e-6;
  setAutonomyMotionExpected(command_expects_motion, planning_body, timestamp_s);
  return output;
}

NavigatorOutput Navigator::tickIntent(const nav_kernel::Pose &odom_map_body,
                                      const nav_kernel::Twist &intent, const float *obstacle_xyzh,
                                      int obstacle_count, double timestamp_s,
                                      TraversabilityGridView traversability,
                                      NavigatorObservation observation) {
  advanceTrajectoryClock(timestamp_s);
  NavigatorOutput output;
  clearRecoveryObservationWait();
  if (!configured_) {
    output.reason = "not_configured";
    return output;
  }

  recovery_action_ = 0;
  recovery_attempt_ = -1;
  recovery_follower_.reset();
  if (!scan_task_)
    local_planner_.reset();
  recovery_.reset();
  resetAutonomyProgress();
  const double requested_speed = std::hypot(intent.vx, intent.vy);
  if (requested_speed <= 1e-6 || config_.max_speed <= 1e-6) {
    if (scan_intent_mode_)
      resetLocalPlanning();
    output.reason = "teleop_intent_idle";
    follower_.stopLinear();
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

  const std::vector<nav_kernel::Vec3> intent_route{odom_map_body.position, output.target};
  const nav_kernel::LocalKinematicState kinematics =
      planningKinematics(odom_map_body, observation, timestamp_s);
  const nav_kernel::LocalPlanInput plan_input =
      makeLocalPlanInput(odom_map_body, intent_route, 0, false, kinematics, observation,
                         obstacle_xyzh, obstacle_count, timestamp_s, traversability);
  const nav_kernel::LocalMotionIntent motion_intent{
      direction_body * 180.0 / M_PI,
      speed_norm,
      horizon,
      config_.teleop_intent_max_deviation_deg,
  };
  nav_kernel::LocalPlanResult plan =
      planTeleop(plan_input, motion_intent, timestamp_s, &output.local_planner_debug);

  output.path_found = plan.pathFound;
  output.slow_down = std::max(0, std::min(3, plan.slowDown));
  output.recovery_state = 0;
  output.local_path_body = plan.path;
  output.local_trajectory_body = plan.trajectory;
  if (plan.pathFound && !plan.nearFieldStop && output.local_path_body.size() < 2) {
    if (output.local_path_body.size() == 1 &&
        bodyDistance2D(output.local_path_body.front()) > 0.05) {
      output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
    }
  }
  output.local_path_map = bodyPathToMap(odom_map_body, output.local_path_body);

  const bool trajectory_provided = !output.local_trajectory_body.empty();
  if (trajectory_provided && output.local_trajectory_body.size() < 2) {
    output.near_field_stop = plan.nearFieldStop;
    output.reason = plan.reason.empty() ? "teleop_assist_untrackable_trajectory" : plan.reason;
    follower_.stopLinear();
    return output;
  }
  if (!trajectory_provided && output.local_path_body.size() < 2) {
    output.near_field_stop = plan.nearFieldStop;
    output.reason = plan.pathFound
                        ? "teleop_assist_untrackable_path"
                        : (plan.reason.empty() || plan.reason == "no_path" ? "teleop_assist_no_path"
                                                                           : plan.reason);
    follower_.stopLinear();
    return output;
  }

  nav_kernel::FollowerInput follower_input =
      trajectory_provided ? nav_kernel::FollowerInput(output.local_trajectory_body)
                          : nav_kernel::FollowerInput(output.local_path_body);
  follower_input.requestedSpeed = speed_norm;
  follower_input.measuredBodyTwist =
      observation.body_velocity_valid
          ? nav_kernel::Twist{observation.body_linear_velocity.x,
                              observation.body_linear_velocity.y, observation.body_yaw_rate}
          : nav_kernel::Twist{};
  follower_input.currentTime = timestamp_s;
  follower_input.slowFactor = slowFactor(output.slow_down);
  follower_input.params = config_.follower;
  follower_input.goalDistance = horizon;
  const nav_kernel::FollowerOutput control = follower_.follow(follower_input);
  output.cmd_vel = control.cmd;
  output.trajectory_frozen = control.executionFrozen;
  traj_frozen_ = control.executionFrozen;
  output.near_field_stop = false;
  output.reason = trajectory_provided ? "teleop_assist_trajectory_ready"
                                      : (plan.nearFieldStop ? "teleop_assist_detour"
                                                            : "teleop_assist_control_ready");
  return output;
}

double Navigator::slowFactor(int slow_down) const {
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
