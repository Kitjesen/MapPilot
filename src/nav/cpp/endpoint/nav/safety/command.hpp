#pragma once

#include <cstdint>
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
  double cmd_max_age_s{0.35};
  double max_speed_mps{0.4};
  double max_yaw_rate{1.0};
  double min_motion_speed_mps{0.03};
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
                      double request_age_s);

inline TeleopDecision
arbitrateTeleopCommand(const TeleopSafetyConfig &cfg, const nav_kernel::Twist &raw_request,
                       double request_age_s) {
  return evaluateCommandSafety(cfg, raw_request, request_age_s);
}

}  // namespace lingtu::nav::endpoint
