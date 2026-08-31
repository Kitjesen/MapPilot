#include "control/final.hpp"

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
    result.decision.should_publish = true;
    result.decision.reason = "zero";
    return result;
  }

  result.decision =
      actions_.command_safety(input.safety, shaped.command, input.request_age_s);
  result.safety_applied = true;

  if (isTeleop(input.mode)) {
    result.decision.limited =
        result.decision.limited || input.limited_before_final || shaped.limited;
    if (input.mode == FinalMode::kTeleopCommand && !result.decision.stopped &&
        result.decision.reason == "accepted" && input.limited_before_final) {
      result.decision.reason = "limited";
    }
  }
  if (isTeleop(input.mode) && input.max_request_age_s > 0.0 &&
      input.request_age_s > input.max_request_age_s) {
    result.decision = stoppedDecision("stale");
    result.reason = result.decision.reason;
    stop(input.now_s, result.reason);
    result.smoother_stopped = true;
    return result;
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
  return result;
}

void FinalControl::stop(double now_s, const std::string &reason) {
  actions_.stop(now_s, reason);
}

}  // namespace lingtu::nav::endpoint
