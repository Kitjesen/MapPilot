#include "motion/control_loop_runtime_guard.hpp"

#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

ControlLoopRuntimeGuard::ControlLoopRuntimeGuard(ControlLoopRuntimeGuardConfig config)
    : config_(std::move(config)) {
  if (config_.unhealthy_confirmation_samples == 0) {
    throw std::invalid_argument("unhealthy confirmation count must be positive");
  }
  if (config_.recovery_confirmation_samples == 0) {
    throw std::invalid_argument("recovery confirmation count must be positive");
  }
}

ControlLoopRuntimeGuardDecision ControlLoopRuntimeGuard::decision(bool clear_motion,
                                                                  bool resume_completed) const {
  const bool motion_held = state_ == ControlLoopRuntimeGuardState::kLatched ||
                           state_ == ControlLoopRuntimeGuardState::kRecovered;
  return {
      state_,           clear_motion,
      motion_held,      state_ == ControlLoopRuntimeGuardState::kRecovered,
      resume_completed, reason_,
  };
}

ControlLoopRuntimeGuardDecision ControlLoopRuntimeGuard::latch(const std::string &reason,
                                                               bool clear_motion) {
  state_ = ControlLoopRuntimeGuardState::kLatched;
  unhealthy_samples_ = 0;
  recovery_samples_ = 0;
  reason_ = reason;
  return decision(clear_motion);
}

ControlLoopRuntimeGuardDecision
ControlLoopRuntimeGuard::observe(const ControlLoopHealthSnapshot &health) {
  if (!health.ready) {
    unhealthy_samples_ = 0;
    recovery_samples_ = 0;
    reason_ = health.reason.empty() ? "warming_up" : health.reason;
    if (state_ == ControlLoopRuntimeGuardState::kSuspect) {
      state_ = ControlLoopRuntimeGuardState::kMonitoring;
    } else if (state_ == ControlLoopRuntimeGuardState::kRecovered) {
      state_ = ControlLoopRuntimeGuardState::kLatched;
    }
    return decision();
  }

  if (health.healthy) {
    unhealthy_samples_ = 0;
    if (state_ == ControlLoopRuntimeGuardState::kMonitoring ||
        state_ == ControlLoopRuntimeGuardState::kSuspect) {
      state_ = ControlLoopRuntimeGuardState::kMonitoring;
      recovery_samples_ = 0;
      reason_ = health.reason.empty() ? "healthy" : health.reason;
      return decision();
    }

    if (state_ == ControlLoopRuntimeGuardState::kLatched) {
      ++recovery_samples_;
      if (recovery_samples_ >= config_.recovery_confirmation_samples) {
        state_ = ControlLoopRuntimeGuardState::kRecovered;
      }
    }
    return decision();
  }

  const std::string reason = health.reason.empty() ? "control_loop_unhealthy" : health.reason;
  recovery_samples_ = 0;
  if (state_ == ControlLoopRuntimeGuardState::kLatched ||
      state_ == ControlLoopRuntimeGuardState::kRecovered) {
    return latch(reason, false);
  }

  ++unhealthy_samples_;
  if (reason == "consecutive_deadline_misses" ||
      unhealthy_samples_ >= config_.unhealthy_confirmation_samples) {
    return latch(reason, true);
  }

  state_ = ControlLoopRuntimeGuardState::kSuspect;
  reason_ = reason;
  return decision();
}

ControlLoopRuntimeGuardDecision ControlLoopRuntimeGuard::requestResume() const {
  return decision();
}

ControlLoopRuntimeGuardDecision ControlLoopRuntimeGuard::snapshot() const {
  return decision();
}

ControlLoopRuntimeGuardDecision ControlLoopRuntimeGuard::completeResume(bool success) {
  if (success && state_ == ControlLoopRuntimeGuardState::kRecovered) {
    state_ = ControlLoopRuntimeGuardState::kMonitoring;
    unhealthy_samples_ = 0;
    recovery_samples_ = 0;
    reason_ = "healthy";
    return decision(false, true);
  }

  if (!success && state_ == ControlLoopRuntimeGuardState::kRecovered) {
    state_ = ControlLoopRuntimeGuardState::kLatched;
    unhealthy_samples_ = 0;
    recovery_samples_ = 0;
    reason_ = "resume_failed";
  }
  return decision();
}

}  // namespace lingtu::nav::endpoint
