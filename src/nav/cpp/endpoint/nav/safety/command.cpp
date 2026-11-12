#include "safety/command.hpp"

#include <algorithm>
#include <cmath>

namespace lingtu::nav::endpoint {

double linearSpeed(const nav_kernel::Twist &cmd) {
  return std::hypot(cmd.vx, cmd.vy);
}

CommandSafetyDecision
evaluateCommandSafety(const CommandSafetyConfig &cfg,
                      const nav_kernel::Twist &raw_request) {
  CommandSafetyDecision decision;
  decision.should_publish = true;
  decision.cmd = raw_request;
  decision.cmd.vx = std::clamp(decision.cmd.vx, -cfg.max_vx_mps, cfg.max_vx_mps);
  decision.cmd.vy = std::clamp(decision.cmd.vy, -cfg.max_vy_mps, cfg.max_vy_mps);
  decision.limited = decision.cmd.vx != raw_request.vx || decision.cmd.vy != raw_request.vy;
  const double speed = linearSpeed(decision.cmd);
  if (speed > cfg.max_speed_mps && speed > 1e-6) {
    const double scale = cfg.max_speed_mps / speed;
    decision.cmd.vx *= scale;
    decision.cmd.vy *= scale;
    decision.limited = true;
  }
  if (std::abs(decision.cmd.wz) > cfg.max_yaw_rate) {
    decision.cmd.wz = std::copysign(cfg.max_yaw_rate, decision.cmd.wz);
    decision.limited = true;
  }

  const double limited_speed = linearSpeed(decision.cmd);
  // A deadband above the single-axis budget would suppress every legal axis input.
  const double single_axis_budget = std::min({cfg.max_speed_mps, cfg.max_vx_mps, cfg.max_vy_mps});
  if (cfg.min_motion_speed_mps <= single_axis_budget &&
      limited_speed > 1e-9 && limited_speed < cfg.min_motion_speed_mps) {
    decision.cmd.vx = 0.0;
    decision.cmd.vy = 0.0;
    decision.stopped = std::abs(decision.cmd.wz) < 1e-4;
    decision.limited = true;
    decision.reason = decision.stopped ? "below_min_motion" : "limited";
    return decision;
  }

  decision.reason = decision.limited ? "limited" : "accepted";
  return decision;
}

}  // namespace lingtu::nav::endpoint
