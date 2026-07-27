#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "motion/stop_confirmation.hpp"

namespace lingtu::nav::endpoint {

struct MotionStopResult {
  bool accepted{false};
  std::string reason;
};

struct ResumeAutonomyRequest {
  std::string precondition_error;
  bool operator_takeover_latched{false};
  double source_stamp_s{0.0};
};

struct FinalShutdownResult {
  bool success{false};
  std::optional<StopConfirmationState> confirmation_state;
};

using MotionStopTerminalCommit = std::function<void()>;

struct MotionStopActions {
  std::function<MotionStopTerminalCommit(const std::string &)> defer_goal_abort;
  std::function<void(const std::string &)> record_stop_evidence_failure;
  std::function<void()> sync_goal_diagnostics;
  std::function<bool()> rolling_segment_active;
  std::function<bool(const std::string &)> preempt_rolling_segment;
  std::function<bool(const std::string &)> clear_motion_outputs;

  std::function<void()> cancel_control;
  std::function<void()> stop_control;
  std::function<void(const std::string &)> latch_estop;
  std::function<bool()> clear_control_estop;
  std::function<bool()> resume_autonomy;

  std::function<void(const std::string &)> cancel_inspection;
  std::function<void()> clear_operator_resume_required;
  std::function<void(double)> set_autonomy_request_not_before;

  std::function<bool(const std::string &)> persist_estop_latch;
  std::function<bool()> clear_persisted_estop_latch;

  std::function<bool()> publish_zero;
  std::function<std::uint64_t()> last_output_sequence;
  std::function<std::optional<std::uint64_t>()> publish_sequenced_zero;
  std::function<StopConfirmationState(std::uint64_t)> confirm_zero;
  std::function<void()> clear_global_path;
};

class MotionStopCoordinator {
 public:
  MotionStopCoordinator(bool publish_cmd_vel, MotionStopActions actions);

  bool clearEndpointMotion(const std::string &reason) const;

  MotionStopResult cancel() const;
  MotionStopResult commitGoalTerminalAfterStop(const std::string &reason,
                                               MotionStopTerminalCommit commit_terminal) const;
  MotionStopResult stop() const;
  MotionStopResult estop(const std::string &reason) const;
  MotionStopResult clearEstop(const std::string &precondition_error) const;
  MotionStopResult resumeAutonomy(const ResumeAutonomyRequest &request) const;

  bool driverAuthorityLost(const std::string &blocker) const;
  bool keepZeroFresh() const;
  FinalShutdownResult finalShutdown() const;

 private:
  bool clearMotionOutputs(const std::string &reason) const;
  MotionStopTerminalCommit deferGoalAbort(const std::string &reason) const;
  MotionStopResult failClosed(MotionStopResult result) const;
  MotionStopResult confirmAndCommitTerminal(MotionStopTerminalCommit commit_terminal,
                                            const std::string &confirmed_reason,
                                            const std::string &failure_suffix,
                                            bool fail_closed) const;
  MotionStopResult confirmLastZero(const std::string &confirmed_reason,
                                   const std::string &failure_suffix) const;

  bool publish_cmd_vel_{false};
  MotionStopActions actions_;
};

}  // namespace lingtu::nav::endpoint
