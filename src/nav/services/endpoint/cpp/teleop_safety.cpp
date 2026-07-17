#include "teleop_safety.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace lingtu::nav::endpoint {

double linearSpeed(const nav_kernel::Twist& cmd) {
  return std::sqrt(cmd.vx * cmd.vx + cmd.vy * cmd.vy);
}

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kSweepLinearStepM = 0.05;
constexpr double kSweepAngularStepRad = 0.05;

struct SweptPose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double travel_m{0.0};
};

nav_kernel::Vec3 mapPointToBody(
    const nav_kernel::Pose& map_body,
    const nav_kernel::Vec3& p_map) {
  const double dx = p_map.x - map_body.position.x;
  const double dy = p_map.y - map_body.position.y;
  const double c = std::cos(map_body.yaw);
  const double s = std::sin(map_body.yaw);
  return {
      c * dx + s * dy,
      -s * dx + c * dy,
      p_map.z - map_body.position.z,
  };
}

nav_kernel::Twist limitedTeleopCommand(
    const CommandSafetyConfig& cfg,
    const nav_kernel::Twist& raw,
    bool& limited) {
  nav_kernel::Twist out = raw;
  const double speed = linearSpeed(out);
  if (speed > cfg.max_speed_mps && speed > 1e-6) {
    const double scale = cfg.max_speed_mps / speed;
    out.vx *= scale;
    out.vy *= scale;
    limited = true;
  }
  if (std::abs(out.wz) > cfg.max_yaw_rate) {
    out.wz = std::copysign(cfg.max_yaw_rate, out.wz);
    limited = true;
  }
  return out;
}

std::vector<SweptPose2D> sweptPoses(
    const CommandSafetyConfig& cfg,
    const nav_kernel::Twist& cmd) {
  const double speed = linearSpeed(cmd);
  const double horizon_m = std::max(cfg.stop_distance_m, cfg.slow_distance_m);
  std::vector<SweptPose2D> poses;
  if (speed >= cfg.min_motion_speed_mps) {
    double duration_s = horizon_m / speed;
    if (std::abs(cmd.wz) > 1e-6) {
      duration_s = std::min(duration_s, 2.0 * kPi / std::abs(cmd.wz));
    }
    const double travel_m = speed * duration_s;
    const double yaw_span = std::abs(cmd.wz) * duration_s;
    const int sample_count = std::max(
        1,
        std::max(
            static_cast<int>(std::ceil(travel_m / kSweepLinearStepM)),
            static_cast<int>(std::ceil(yaw_span / kSweepAngularStepRad))));
    poses.reserve(static_cast<std::size_t>(sample_count + 1));
    for (int i = 0; i <= sample_count; ++i) {
      const double t = duration_s * static_cast<double>(i) /
                       static_cast<double>(sample_count);
      SweptPose2D pose;
      pose.yaw = cmd.wz * t;
      pose.travel_m = speed * t;
      if (std::abs(cmd.wz) <= 1e-6) {
        pose.x = cmd.vx * t;
        pose.y = cmd.vy * t;
      } else {
        const double s = std::sin(pose.yaw);
        const double c = std::cos(pose.yaw);
        pose.x = (cmd.vx * s + cmd.vy * (c - 1.0)) / cmd.wz;
        pose.y = (cmd.vx * (1.0 - c) + cmd.vy * s) / cmd.wz;
      }
      poses.push_back(pose);
    }
    return poses;
  }

  const double half_length = cfg.vehicle_length_m * 0.5 + cfg.obstacle_margin_m;
  const double half_width = cfg.vehicle_width_m * 0.5 + cfg.obstacle_margin_m;
  const double corner_radius = std::max(0.1, std::hypot(half_length, half_width));
  const double yaw_span = std::min(0.5 * kPi, horizon_m / corner_radius);
  const int sample_count = std::max(
      1,
      static_cast<int>(std::ceil(yaw_span / kSweepAngularStepRad)));
  poses.reserve(static_cast<std::size_t>(sample_count + 1));
  for (int i = 0; i <= sample_count; ++i) {
    const double fraction = static_cast<double>(i) / static_cast<double>(sample_count);
    const double yaw = std::copysign(yaw_span * fraction, cmd.wz);
    poses.push_back({0.0, 0.0, yaw, corner_radius * std::abs(yaw)});
  }
  return poses;
}

std::vector<SweptPose2D> sweptPathPoses(
    const CommandSafetyConfig& cfg,
    const nav_kernel::Pose& map_body,
    const std::vector<nav_kernel::Vec3>& local_path_map) {
  std::vector<nav_kernel::Vec3> body_path;
  body_path.reserve(local_path_map.size() + 1);
  body_path.push_back({0.0, 0.0, 0.0});
  for (const auto& point_map : local_path_map) {
    const auto point_body = mapPointToBody(map_body, point_map);
    if (std::hypot(
            point_body.x - body_path.back().x,
            point_body.y - body_path.back().y) > 1e-4) {
      body_path.push_back(point_body);
    }
  }
  if (body_path.size() < 2) {
    return {};
  }

  const double horizon_m = std::max(cfg.stop_distance_m, cfg.slow_distance_m);
  std::vector<SweptPose2D> poses;
  poses.push_back({0.0, 0.0, 0.0, 0.0});
  double cumulative_m = 0.0;
  double next_sample_m = kSweepLinearStepM;
  double final_x = 0.0;
  double final_y = 0.0;
  double final_yaw = 0.0;
  double final_travel_m = 0.0;
  for (std::size_t i = 1; i < body_path.size() && cumulative_m < horizon_m; ++i) {
    const auto& start = body_path[i - 1];
    const auto& end = body_path[i];
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    const double segment_m = std::hypot(dx, dy);
    if (segment_m <= 1e-6) {
      continue;
    }
    const double yaw = std::atan2(dy, dx);
    const double segment_end_m = std::min(horizon_m, cumulative_m + segment_m);
    while (next_sample_m <= segment_end_m + 1e-6) {
      const double ratio = std::clamp(
          (next_sample_m - cumulative_m) / segment_m,
          0.0,
          1.0);
      poses.push_back({
          start.x + ratio * dx,
          start.y + ratio * dy,
          yaw,
          next_sample_m,
      });
      next_sample_m += kSweepLinearStepM;
    }
    const double final_ratio =
        std::clamp((segment_end_m - cumulative_m) / segment_m, 0.0, 1.0);
    final_x = start.x + final_ratio * dx;
    final_y = start.y + final_ratio * dy;
    final_yaw = yaw;
    final_travel_m = segment_end_m;
    cumulative_m += segment_m;
  }
  if (final_travel_m > poses.back().travel_m + 1e-4) {
    poses.push_back({final_x, final_y, final_yaw, final_travel_m});
  }
  return poses;
}

bool pointInsideFootprint(
    const SweptPose2D& pose,
    const nav_kernel::Vec3& point,
    double half_length,
    double half_width,
    double& local_x,
    double& local_y) {
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  local_x = c * dx + s * dy;
  local_y = -s * dx + c * dy;
  return std::abs(local_x) <= half_length && std::abs(local_y) <= half_width;
}

bool traversabilityCostAt(
    const TraversabilityGrid& grid,
    double map_x,
    double map_y,
    double& cost) {
  if (grid.values.empty() || grid.rows <= 0 || grid.cols <= 0 || grid.resolution <= 0.0) {
    return false;
  }
  const int col = static_cast<int>(std::floor((map_x - grid.origin_x) / grid.resolution));
  const int row = static_cast<int>(std::floor((map_y - grid.origin_y) / grid.resolution));
  if (row < 0 || col < 0 || row >= grid.rows || col >= grid.cols) {
    return false;
  }
  cost = grid.values[static_cast<std::size_t>(row) * static_cast<std::size_t>(grid.cols) +
                     static_cast<std::size_t>(col)];
  return true;
}

}  // namespace

namespace {

CommandSafetyDecision evaluateSafetyWithSweep(
    const CommandSafetyConfig& cfg,
    const nav_kernel::Twist& raw_request,
    double request_age_s,
    const std::optional<nav_kernel::Pose>& map_body,
    const std::vector<float>& obstacle_xyzh,
    const TraversabilityGrid& traversability_grid,
    bool traversability_fresh,
    const std::vector<SweptPose2D>* prescribed_sweep) {
  CommandSafetyDecision decision;
  decision.should_publish = true;
  if (cfg.cmd_max_age_s > 0.0 && request_age_s > cfg.cmd_max_age_s) {
    decision.stopped = true;
    decision.reason = "stale";
    return decision;
  }
  decision.cmd = limitedTeleopCommand(cfg, raw_request, decision.limited);
  double speed = linearSpeed(decision.cmd);
  const bool requested_translation = linearSpeed(raw_request) > 1e-9;
  if (speed > 1e-9 && speed < cfg.min_motion_speed_mps) {
    decision.cmd.vx = 0.0;
    decision.cmd.vy = 0.0;
    decision.limited = true;
    speed = 0.0;
  }
  if (speed < cfg.min_motion_speed_mps && std::abs(decision.cmd.wz) < 1e-4) {
    decision.cmd = {};
    if (requested_translation) {
      decision.stopped = true;
      decision.limited = true;
      decision.reason = "below_min_motion";
    } else {
      decision.reason = decision.limited ? "zero_limited" : "zero";
    }
    return decision;
  }

  if ((cfg.check_obstacle || cfg.use_traversability_cost) && !map_body) {
    decision.cmd = {};
    decision.stopped = true;
    decision.reason = "no_pose";
    return decision;
  }

  if (cfg.use_traversability_cost && !traversability_fresh) {
    decision.cmd = {};
    decision.stopped = true;
    decision.reason = "terrain_stale";
    return decision;
  }

  const nav_kernel::Twist safety_cmd = decision.cmd;
  std::vector<SweptPose2D> command_sweep;
  if (prescribed_sweep == nullptr) {
    command_sweep = sweptPoses(cfg, safety_cmd);
  }
  const auto& sweep = prescribed_sweep != nullptr ? *prescribed_sweep : command_sweep;
  const bool path_sweep = prescribed_sweep != nullptr;
  if (path_sweep && speed >= cfg.min_motion_speed_mps && sweep.size() < 2) {
    decision.cmd = {};
    decision.stopped = true;
    decision.reason = "no_safety_path";
    return decision;
  }

  if (cfg.check_obstacle) {
    const double half_length =
        std::max(0.1, cfg.vehicle_length_m * 0.5 + cfg.obstacle_margin_m);
    const double half_width =
        std::max(0.1, cfg.vehicle_width_m * 0.5 + cfg.obstacle_margin_m);
    const bool translation_motion = speed >= cfg.min_motion_speed_mps;
    const double dir_x = translation_motion ? safety_cmd.vx / speed : 0.0;
    const double dir_y = translation_motion ? safety_cmd.vy / speed : 0.0;
    double min_path_distance = std::numeric_limits<double>::infinity();
    double min_radial = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 3 < obstacle_xyzh.size(); i += 4) {
      const double height = obstacle_xyzh[i + 3];
      if (!std::isfinite(height) || height < cfg.obstacle_height_min_m ||
          height > cfg.obstacle_height_max_m) {
        continue;
      }
      const auto p_body = mapPointToBody(
          *map_body,
          {obstacle_xyzh[i + 0], obstacle_xyzh[i + 1], obstacle_xyzh[i + 2]});
      for (const auto& swept_pose : sweep) {
        double local_x = 0.0;
        double local_y = 0.0;
        if (!pointInsideFootprint(
                swept_pose, p_body, half_length, half_width, local_x, local_y)) {
          continue;
        }
        if (translation_motion) {
          const double local_ahead = path_sweep
              ? local_x
              : local_x * dir_x + local_y * dir_y;
          const double path_distance = swept_pose.travel_m + std::max(0.0, local_ahead);
          if (path_distance <= cfg.slow_distance_m + 1e-6) {
            min_path_distance = std::min(min_path_distance, path_distance);
          }
        } else {
          const double radial = std::hypot(p_body.x, p_body.y);
          min_radial = std::min(min_radial, radial);
        }
        break;
      }
    }
    if (translation_motion && std::isfinite(min_path_distance)) {
      decision.obstacle_distance_m = min_path_distance;
      if (min_path_distance <= cfg.stop_distance_m) {
        decision.cmd = {};
        decision.stopped = true;
        decision.reason = "obstacle_stop";
        return decision;
      }
      decision.cmd.vx *= cfg.linear_slow_scale;
      decision.cmd.vy *= cfg.linear_slow_scale;
      decision.slowed = true;
      decision.reason = "obstacle_slow";
    } else if (!translation_motion && std::isfinite(min_radial)) {
      decision.obstacle_distance_m = min_radial;
      decision.cmd = {};
      decision.stopped = true;
      decision.reason = "yaw_obstacle";
      return decision;
    }
  }

  if (cfg.use_traversability_cost && traversability_fresh &&
      speed >= cfg.min_motion_speed_mps) {
    const double c = std::cos(map_body->yaw);
    const double s = std::sin(map_body->yaw);
    const double step = std::max(0.20, traversability_grid.resolution);
    double next_sample_distance = step;
    double max_cost = -1.0;
    double nearest_hard_distance = std::numeric_limits<double>::infinity();
    for (const auto& swept_pose : sweep) {
      if (swept_pose.travel_m + 1e-6 < next_sample_distance) {
        continue;
      }
      const double mx = map_body->position.x + c * swept_pose.x - s * swept_pose.y;
      const double my = map_body->position.y + s * swept_pose.x + c * swept_pose.y;
      double cost = -1.0;
      if (traversabilityCostAt(traversability_grid, mx, my, cost)) {
        max_cost = std::max(max_cost, cost);
        if (cost >= cfg.traversability_hard_cost) {
          nearest_hard_distance =
              std::min(nearest_hard_distance, swept_pose.travel_m);
        }
      }
      next_sample_distance += step;
    }
    decision.traversability_cost = max_cost;
    if (nearest_hard_distance <= cfg.stop_distance_m + 1e-6) {
      decision.cmd = {};
      decision.stopped = true;
      decision.reason = "terrain_stop";
      return decision;
    }
    if (max_cost >= cfg.traversability_soft_cost) {
      decision.cmd.vx *= cfg.linear_slow_scale;
      decision.cmd.vy *= cfg.linear_slow_scale;
      decision.slowed = true;
      if (decision.reason == "obstacle_slow") {
        decision.reason = "obstacle_terrain_slow";
      } else {
        decision.reason = "terrain_slow";
      }
    }
  }

  if (decision.reason == "idle") {
    decision.reason = decision.limited ? "limited" : "accepted";
  }
  const double final_speed = linearSpeed(decision.cmd);
  if (final_speed > 1e-9 && final_speed < cfg.min_motion_speed_mps) {
    decision.cmd.vx = 0.0;
    decision.cmd.vy = 0.0;
    decision.limited = true;
    if (std::abs(decision.cmd.wz) < 1e-4) {
      decision.stopped = true;
      decision.slowed = false;
      decision.reason += "_below_min_motion";
    }
  }
  return decision;
}

}  // namespace

CommandSafetyDecision evaluateCommandSafety(
    const CommandSafetyConfig& cfg,
    const nav_kernel::Twist& raw_request,
    double request_age_s,
    const std::optional<nav_kernel::Pose>& map_body,
    const std::vector<float>& obstacle_xyzh,
    const TraversabilityGrid& traversability_grid,
    bool traversability_fresh) {
  return evaluateSafetyWithSweep(
      cfg,
      raw_request,
      request_age_s,
      map_body,
      obstacle_xyzh,
      traversability_grid,
      traversability_fresh,
      nullptr);
}

CommandSafetyDecision evaluateAutonomyPathSafety(
    const CommandSafetyConfig& cfg,
    const nav_kernel::Twist& raw_request,
    const std::optional<nav_kernel::Pose>& map_body,
    const std::vector<nav_kernel::Vec3>& local_path_map,
    const std::vector<float>& obstacle_xyzh,
    const TraversabilityGrid& traversability_grid,
    bool traversability_fresh) {
  if (!map_body || linearSpeed(raw_request) < cfg.min_motion_speed_mps) {
    return evaluateCommandSafety(
        cfg,
        raw_request,
        0.0,
        map_body,
        obstacle_xyzh,
        traversability_grid,
        traversability_fresh);
  }
  const auto path_sweep = sweptPathPoses(cfg, *map_body, local_path_map);
  return evaluateSafetyWithSweep(
      cfg,
      raw_request,
      0.0,
      map_body,
      obstacle_xyzh,
      traversability_grid,
      traversability_fresh,
      &path_sweep);
}

}  // namespace lingtu::nav::endpoint
