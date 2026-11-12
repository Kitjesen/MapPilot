#pragma once

#include <lingtu/settings.hpp>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "nav_kernel/types.hpp"
#include "navigation/executor.hpp"

namespace lingtu::nav::endpoint {

struct TraversabilityGrid {
  std::vector<float> values;
  int rows{0};
  int cols{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::uint64_t generation{0};

  lingtu::nav::navigation::TraversabilityGridView view() const {
    if (values.empty() || rows <= 0 || cols <= 0 || resolution <= 0.0) {
      return {};
    }
    return {
        values.data(), rows, cols, resolution, origin_x, origin_y, generation,
    };
  }
};

struct CommandSafetyConfig {
  double max_speed_mps{::lingtu::settings::teleop::native_nav::teleop::max_speed_mps};
  double max_yaw_rate{::lingtu::settings::teleop::native_nav::teleop::max_yaw_rate_rad_s};
  double min_motion_speed_mps{::lingtu::settings::teleop::native_nav::teleop::min_motion_speed_mps};
  bool verified_recovery_translation{false};
  // Body-axis bound, independent of the total planar speed above.
  double max_vx_mps{std::numeric_limits<double>::infinity()};
  double max_vy_mps{std::numeric_limits<double>::infinity()};
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

double linearSpeed(const nav_kernel::Twist &cmd);

CommandSafetyDecision
evaluateCommandSafety(const CommandSafetyConfig &cfg, const nav_kernel::Twist &raw_request);



}  // namespace lingtu::nav::endpoint
