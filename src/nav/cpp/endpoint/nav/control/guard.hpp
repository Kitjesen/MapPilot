#pragma once

#include <cstddef>
#include <string>

#include "status/control_loop_health.hpp"

namespace lingtu::nav::endpoint {

enum class ControlLoopRuntimeGuardState {
  kMonitoring,
  kSuspect,
  kLatched,
  kRecovered,
};

struct ControlLoopRuntimeGuardConfig {
  std::size_t unhealthy_confirmation_samples{2};
  std::size_t recovery_confirmation_samples{20};
  std::size_t latch_miss_streak{20};
  bool auto_resume{false};
};

struct ControlLoopRuntimeGuardDecision {
  ControlLoopRuntimeGuardState state{ControlLoopRuntimeGuardState::kMonitoring};
  bool clear_motion{false};
  bool hold_motion{false};
  bool resume_allowed{false};
  bool resume_completed{false};
  std::string reason{"monitoring"};
};

// Converts sampled control-loop health into an explicitly recoverable motion
// hold. It owns no transport and performs no motion side effects itself.
class ControlLoopRuntimeGuard {
 public:
  explicit ControlLoopRuntimeGuard(ControlLoopRuntimeGuardConfig config = {});

  ControlLoopRuntimeGuardDecision observe(const ControlLoopHealthSnapshot &health);

  // Both queries are side-effect-free. Recovery still requires completeResume().
  ControlLoopRuntimeGuardDecision requestResume() const;
  ControlLoopRuntimeGuardDecision snapshot() const;

  // A failed completion conservatively re-latches and requires a fresh healthy
  // confirmation window. Success is accepted only from the recovered state.
  ControlLoopRuntimeGuardDecision completeResume(bool success);

 private:
  ControlLoopRuntimeGuardDecision decision(bool clear_motion = false,
                                           bool resume_completed = false) const;
  ControlLoopRuntimeGuardDecision latch(const std::string &reason, bool clear_motion);

  ControlLoopRuntimeGuardConfig config_;
  ControlLoopRuntimeGuardState state_{ControlLoopRuntimeGuardState::kMonitoring};
  std::size_t unhealthy_samples_{0};
  std::size_t recovery_samples_{0};
  std::string reason_{"monitoring"};
};

}  // namespace lingtu::nav::endpoint
