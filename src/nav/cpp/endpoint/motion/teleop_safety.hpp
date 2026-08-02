#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "nav_kernel/types.hpp"
#include "nav_loop.hpp"

namespace lingtu::nav::endpoint {

struct TraversabilityGrid {
  std::vector<float> values;
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint64_t generation{0};

  lingtu::nav::plan::TraversabilityGridView view() const {
    if (values.empty() || rows <= 0 || cols <= 0 || resolution <= 0.0) {
      return {};
    }
    return {
        values.data(), rows, cols, resolution, origin_x, origin_y, generation,
    };
  }
};

struct CommandSafetyConfig {
  double cmd_max_age_s{0.35};
  double max_speed_mps{0.4};
  double max_yaw_rate{1.0};
  double slow_distance_m{1.2};
  double stop_distance_m{0.55};
  double linear_slow_scale{0.35};
  double min_motion_speed_mps{0.03};
  double obstacle_height_min_m{0.10};
  double obstacle_height_max_m{1.20};
  double obstacle_margin_m{0.15};
  double traversability_hard_cost{80.0};
  double traversability_soft_cost{40.0};
  double vehicle_length_m{1.0};
  double vehicle_width_m{0.6};
  bool check_obstacle{true};
  bool use_traversability_cost{false};
};

struct CommandSafetyDecision {
  bool should_publish{false};
  bool stopped{false};
  bool slowed{false};
  bool limited{false};
  std::string reason{"idle"};
  nav_kernel::Twist cmd{};
  double obstacle_distance_m{-1.0};
  double traversability_cost{-1.0};
};

using TeleopSafetyConfig = CommandSafetyConfig;
using TeleopDecision = CommandSafetyDecision;

double linearSpeed(const nav_kernel::Twist &cmd);

CommandSafetyDecision
evaluateCommandSafety(const CommandSafetyConfig &cfg, const nav_kernel::Twist &raw_request,
                      double request_age_s, const std::optional<nav_kernel::Pose> &map_body,
                      const std::vector<float> &obstacle_xyzh,
                      const TraversabilityGrid &traversability_grid, bool traversability_fresh);

CommandSafetyDecision evaluateAutonomyPathSafety(
    const CommandSafetyConfig &cfg, const nav_kernel::Twist &raw_request,
    const std::optional<nav_kernel::Pose> &map_body,
    const std::vector<nav_kernel::Vec3> &local_path_map, const std::vector<float> &obstacle_xyzh,
    const TraversabilityGrid &traversability_grid, bool traversability_fresh);

inline TeleopDecision
arbitrateTeleopCommand(const TeleopSafetyConfig &cfg, const nav_kernel::Twist &raw_request,
                       double request_age_s, const std::optional<nav_kernel::Pose> &map_body,
                       const std::vector<float> &obstacle_xyzh,
                       const TraversabilityGrid &traversability_grid, bool traversability_fresh) {
  return evaluateCommandSafety(cfg, raw_request, request_age_s, map_body, obstacle_xyzh,
                               traversability_grid, traversability_fresh);
}

}  // namespace lingtu::nav::endpoint
