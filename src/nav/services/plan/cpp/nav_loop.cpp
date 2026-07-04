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

}  // namespace

NavLoop::NavLoop(NavLoopConfig config)
    : config_(std::move(config)), local_planner_(config_.local_planner) {
  config_.path_follower.maxSpeed = config_.max_speed;
}

bool NavLoop::configure() {
  configured_ = local_planner_.loadPaths(config_.path_library_dir);
  return configured_;
}

bool NavLoop::configured() const {
  return configured_;
}

void NavLoop::setGlobalPath(const std::vector<nav_kernel::Vec3>& path) {
  global_path_ = path;
  cursor_ = 0;
  follower_state_ = nav_kernel::PathFollowerState{};
}

void NavLoop::clearGlobalPath() {
  global_path_.clear();
  cursor_ = 0;
  follower_state_ = nav_kernel::PathFollowerState{};
}

NavLoopOutput NavLoop::tick(
    const nav_kernel::Pose& odom_map_body,
    const float* obstacle_xyzh,
    int obstacle_count,
    double timestamp_s,
    TraversabilityGridView traversability) {
  NavLoopOutput output;
  if (!configured_ || global_path_.empty()) {
    return output;
  }

  output.active = true;
  if (atGoal(odom_map_body)) {
    output.goal_reached = true;
    output.active = false;
    follower_state_ = nav_kernel::PathFollowerState{};
    return output;
  }

  const std::size_t target_index = selectTargetIndex(odom_map_body);
  cursor_ = target_index;
  output.target_index = target_index;
  output.target = global_path_[target_index];

  nav_kernel::LocalPlanResult plan = local_planner_.planFrame(
      odom_map_body.position.x,
      odom_map_body.position.y,
      odom_map_body.position.z,
      odom_map_body.yaw,
      output.target.x,
      output.target.y,
      traversability.valid() ? traversability.values : nullptr,
      traversability.valid() ? traversability.rows : 0,
      traversability.valid() ? traversability.cols : 0,
      traversability.valid() ? traversability.resolution : 0.0,
      traversability.valid() ? traversability.origin_x : 0.0,
      traversability.valid() ? traversability.origin_y : 0.0,
      obstacle_xyzh,
      obstacle_count,
      timestamp_s);

  output.path_found = plan.pathFound;
  output.near_field_stop = plan.nearFieldStop;
  output.slow_down = std::max(0, std::min(3, plan.slowDown));
  output.recovery_state = plan.recoveryState;
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

  if (plan.nearFieldStop || output.local_path_body.size() < 2) {
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
      follower_state_);
  output.cmd_vel = control.cmd;
  return output;
}

std::size_t NavLoop::selectTargetIndex(const nav_kernel::Pose& odom_map_body) {
  if (global_path_.empty()) {
    return 0;
  }

  const std::size_t start = std::min(cursor_, global_path_.size() - 1);
  std::size_t nearest = start;
  double nearest_distance = nav_kernel::distance2D(global_path_[start], odom_map_body.position);
  const std::size_t search_end =
      std::min(global_path_.size(), start + static_cast<std::size_t>(12));
  for (std::size_t i = start + 1; i < search_end; ++i) {
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
         config_.goal_reached_m;
}

}  // namespace lingtu::nav::plan
