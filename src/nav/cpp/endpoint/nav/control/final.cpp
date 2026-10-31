#include "control/final.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

bool isZeroCommand(const nav_kernel::Twist &command) {
  return linearSpeed(command) <= 1e-6 && std::abs(command.wz) <= 1e-6;
}

bool isTeleop(FinalMode mode) {
  return mode == FinalMode::kTeleopCommand || mode == FinalMode::kTeleopPath;
}

CommandSafetyDecision stoppedDecision(const std::string &reason) {
  CommandSafetyDecision decision;
  decision.should_publish = true;
  decision.stopped = true;
  decision.limited = true;
  decision.reason = reason;
  return decision;
}

}  // namespace

FinalControl::FinalControl(FinalActions actions) : actions_(std::move(actions)) {
  if (!actions_.command_safety || !actions_.shape || !actions_.commit || !actions_.stop) {
    throw std::invalid_argument("final control actions must all be configured");
  }
}

FinalResult FinalControl::finalize(const FinalInput &input) {
  FinalResult result;
  const auto shaped = actions_.shape(input.candidate, input.now_s);
  if (!shaped.valid || shaped.timed_out) {
    result.reason = shaped.reason.empty() ? "velocity_smoother_invalid" : shaped.reason;
    result.decision = stoppedDecision(result.reason);
    stop(input.now_s, result.reason);
    result.smoother_stopped = true;
    return result;
  }

  if (isZeroCommand(shaped.command)) {
    applied_yaw_rate_ = 0.0;
    applied_yaw_stamp_s_ = input.now_s;
    result.decision.should_publish = true;
    result.decision.reason = "zero";
    return result;
  }

  auto candidate = shaped.command;
  bool yaw_limited = false;
  if (input.mode == FinalMode::kTeleopCommand &&
      input.direct_yaw_accel_rad_s2 > 0.0 &&
      linearSpeed(input.candidate) <= 1e-6 &&
      std::abs(input.candidate.wz) > 1e-6) {
    constexpr double kMaxYawUpdateGapS = 0.2;
    double dt = applied_yaw_stamp_s_ ? input.now_s - *applied_yaw_stamp_s_
                                    : input.nominal_dt_s;
    // A late tick re-anchors time without spending the missed acceleration budget.
    if (!std::isfinite(dt) || dt < 0.0 || dt > kMaxYawUpdateGapS) dt = 0.0;
    const double step = input.direct_yaw_accel_rad_s2 * dt;
    candidate.wz = std::clamp(candidate.wz, applied_yaw_rate_ - step,
                              applied_yaw_rate_ + step);
    yaw_limited = candidate.wz != shaped.command.wz;
  }

  CommandSafetyConfig final_safety = input.safety;
  if (isTeleop(input.mode) && input.max_request_age_s > 0.0) {
    final_safety.cmd_max_age_s = final_safety.cmd_max_age_s > 0.0
                                     ? std::min(final_safety.cmd_max_age_s,
                                                input.max_request_age_s)
                                     : input.max_request_age_s;
  }
  result.decision =
      actions_.command_safety(final_safety, candidate, input.request_age_s);
  result.safety_applied = true;

  if (isTeleop(input.mode)) {
    result.decision.limited =
        result.decision.limited || input.limited_before_final || shaped.limited || yaw_limited;
    if (result.decision.stopped && result.decision.reason == "stale") {
      result.decision.limited = true;
    }
    if (input.mode == FinalMode::kTeleopCommand && !result.decision.stopped &&
        result.decision.reason == "accepted" && input.limited_before_final) {
      result.decision.reason = "limited";
    }
  }
  if (result.decision.stopped) {
    result.reason = result.decision.reason;
    stop(input.now_s, result.reason);
    result.smoother_stopped = true;
  }

  if (input.publish && !isZeroCommand(result.decision.cmd) &&
      !actions_.commit(result.decision.cmd, input.now_s)) {
    result.decision.cmd = {};
    result.decision.stopped = true;
    result.decision.reason = "velocity_smoother_commit_failed";
    if (isTeleop(input.mode)) {
      result.decision.slowed = false;
      result.decision.limited = true;
    }
    result.reason = result.decision.reason;
    stop(input.now_s, result.reason);
    result.smoother_stopped = true;
  }
  if (input.publish && result.decision.should_publish && !result.decision.stopped) {
    applied_yaw_rate_ = result.decision.cmd.wz;
    applied_yaw_stamp_s_ = applied_yaw_stamp_s_
                               ? std::max(*applied_yaw_stamp_s_, input.now_s)
                               : input.now_s;
  }
  return result;
}

void FinalControl::stop(double now_s, const std::string &reason) {
  applied_yaw_rate_ = 0.0;
  // Retain the clock anchor so a repeated tick cannot spend another first step.
  applied_yaw_stamp_s_ = now_s;
  actions_.stop(now_s, reason);
}

}  // namespace lingtu::nav::endpoint
