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

void NavLoop::setGlobalPath(
    const std::vector<nav_kernel::Vec3>& path,
    std::optional<double> final_yaw,
    std::optional<double> goal_reached_m,
    std::optional<double> goal_yaw_tolerance_rad) {
  global_path_ = path;
  final_yaw_ = final_yaw;
  active_goal_reached_m_ = std::max(
      0.01, goal_reached_m.value_or(config_.goal_reached_m));
  active_goal_yaw_tolerance_rad_ = std::max(
      0.0,
      goal_yaw_tolerance_rad.value_or(config_.goal_yaw_tolerance_rad));
  cursor_ = 0;
  follower_state_ = nav_kernel::PathFollowerState{};
}

void NavLoop::clearGlobalPath() {
  global_path_.clear();
  final_yaw_.reset();
  active_goal_reached_m_ = config_.goal_reached_m;
  active_goal_yaw_tolerance_rad_ = config_.goal_yaw_tolerance_rad;
  cursor_ = 0;
  follower_state_ = nav_kernel::PathFollowerState{};
}

void NavLoop::stopLinearMotion() {
  follower_state_.vehicleSpeed = 0.0;
}

NavLoopOutput NavLoop::tick(
    const nav_kernel::Pose& odom_map_body,
    const float* obstacle_xyzh,
    int obstacle_count,
    double timestamp_s,
    TraversabilityGridView traversability) {
  NavLoopOutput output;
  if (!configured_) {
    output.reason = "not_configured";
    return output;
  }
  if (global_path_.empty()) {
    output.reason = "no_global_path";
    return output;
  }

  output.active = true;
  if (atGoal(odom_map_body)) {
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

  syncTraversabilityGrid(traversability);
  local_planner_.setVehicle(
      odom_map_body.position.x,
      odom_map_body.position.y,
      odom_map_body.position.z,
      odom_map_body.yaw);
  local_planner_.setGoal(output.target.x, output.target.y);
  nav_kernel::LocalPlanResult plan =
      local_planner_.plan(obstacle_xyzh, obstacle_count, timestamp_s);
  output.local_planner_debug = debugSnapshotToMap(
      odom_map_body,
      local_planner_.debugSnapshot());

  output.path_found = plan.pathFound;
  output.near_field_stop = plan.nearFieldStop;
  output.slow_down = std::max(0, std::min(3, plan.slowDown));
  output.recovery_state = plan.recoveryState;
  output.recovery_exhausted = plan.recoveryExhausted;
  output.local_path_body = plan.path;
  if (plan.pathFound && !plan.nearFieldStop && output.local_path_body.size() < 2) {
    const nav_kernel::Vec3 target_body = mapPointToBody(odom_map_body, output.target);
    if (bodyDistance2D(target_body) > 0.05) {
      if (output.local_path_body.empty() ||
          bodyDistance2D(output.local_path_body.front()) > 0.05) {
        output.local_path_body.insert(output.local_path_body.begin(), {0.0, 0.0, 0.0});
      }
      output.local_path_body.push_back(target_body);
    }
  }
  output.local_path_map = bodyPathToMap(odom_map_body, output.local_path_body);

  if (plan.recoveryExhausted) {
    output.reason = "local_recovery_exhausted";
    follower_state_.vehicleSpeed = 0.0;
    return output;
  }
  if (plan.nearFieldStop) {
    output.reason = "near_field_stop";
    follower_state_.vehicleSpeed = 0.0;
    return output;
  }
  if (output.local_path_body.size() < 2) {
    output.reason = plan.pathFound ? "untrackable_local_path" : "no_local_path";
    follower_state_.vehicleSpeed = 0.0;
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
      nav_kernel::distance2D(global_path_.back(), odom_map_body.position));
  output.cmd_vel = control.cmd;
  output.reason = plan.pathFound ? "control_ready" : "recovery_path";
  return output;
}

NavLoopOutput NavLoop::tickTeleopIntent(
    const nav_kernel::Pose& odom_map_body,
    const nav_kernel::Twist& intent,
    const float* obstacle_xyzh,
    int obstacle_count,
    double timestamp_s,
    TraversabilityGridView traversability) {
  NavLoopOutput output;
  if (!configured_) {
    output.reason = "not_configured";
    return output;
  }

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
  local_planner_.setVehicle(
      odom_map_body.position.x,
      odom_map_body.position.y,
      odom_map_body.position.z,
      odom_map_body.yaw);
  nav_kernel::LocalPlanResult plan = local_planner_.planIntent(
      obstacle_xyzh,
      obstacle_count,
      timestamp_s,
      direction_body * 180.0 / M_PI,
      speed_norm,
      horizon,
      config_.teleop_intent_max_deviation_deg);
  output.local_planner_debug = debugSnapshotToMap(
      odom_map_body,
      local_planner_.debugSnapshot());

  output.path_found = plan.pathFound;
  output.slow_down = std::max(0, std::min(3, plan.slowDown));
  output.recovery_state = 0;
  output.local_path_body = plan.path;
  if (plan.pathFound && output.local_path_body.size() < 2) {
    if (output.local_path_body.empty() ||
        bodyDistance2D(output.local_path_body.front()) > 0.05) {
      output.local_path_body.insert(
          output.local_path_body.begin(), {0.0, 0.0, 0.0});
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
