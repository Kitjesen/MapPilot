#include "motion/motion_stop_coordinator.hpp"

#include <utility>

namespace lingtu::nav::endpoint {

MotionStopCoordinator::MotionStopCoordinator(bool publish_cmd_vel, MotionStopActions actions)
    : publish_cmd_vel_(publish_cmd_vel), actions_(std::move(actions)) {}

bool MotionStopCoordinator::clearEndpointMotion(const std::string &reason) const {
  return clearMotionOutputs(reason);
}

bool MotionStopCoordinator::clearMotionOutputs(const std::string &reason) const {
  if (actions_.rolling_segment_active()) {
    return actions_.preempt_rolling_segment(reason);
  }
  return actions_.clear_motion_outputs(reason);
}

MotionStopTerminalCommit MotionStopCoordinator::deferGoalAbort(const std::string &reason) const {
  MotionStopTerminalCommit commit_terminal = actions_.defer_goal_abort(reason);
  actions_.sync_goal_diagnostics();
  return commit_terminal;
}

MotionStopResult MotionStopCoordinator::failClosed(MotionStopResult result) const {
  actions_.record_stop_evidence_failure(result.reason);
  return result;
}

MotionStopResult MotionStopCoordinator::confirmAndCommitTerminal(
    MotionStopTerminalCommit commit_terminal, const std::string &confirmed_reason,
    const std::string &failure_suffix, bool fail_closed) const {
  const MotionStopResult confirmation = confirmLastZero(confirmed_reason, failure_suffix);
  if (!confirmation.accepted) {
    return fail_closed ? failClosed(confirmation) : confirmation;
  }
  commit_terminal();
  return confirmation;
}

MotionStopResult MotionStopCoordinator::confirmLastZero(const std::string &confirmed_reason,
                                                        const std::string &failure_suffix) const {
  if (!publish_cmd_vel_) {
    return {
        false,
        "zero_publish_unavailable_" + failure_suffix,
    };
  }
  const std::uint64_t output_sequence = actions_.last_output_sequence();
  if (output_sequence == 0U) {
    return {
        false,
        "zero_publish_unavailable_" + failure_suffix,
    };
  }
  const StopConfirmationState state = actions_.confirm_zero(output_sequence);
  if (state == StopConfirmationState::Confirmed) {
    return {true, confirmed_reason};
  }
  if (state == StopConfirmationState::DriverRejected) {
    return {
        false,
        "driver_rejected_zero_" + failure_suffix,
    };
  }
  return {
      false,
      "stop_confirmation_timeout_" + failure_suffix,
  };
}

MotionStopResult MotionStopCoordinator::cancel() const {
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_cancelled");
  MotionStopTerminalCommit commit_cancelled = deferGoalAbort("cancelled");
  if (!clearMotionOutputs("cancelled")) {
    return failClosed({false, "zero_publish_failed"});
  }
  return confirmAndCommitTerminal(std::move(commit_cancelled), "cancelled",
                                  "cancel_remains_stopped", true);
}

MotionStopResult
MotionStopCoordinator::commitGoalTerminalAfterStop(const std::string &reason,
                                                   MotionStopTerminalCommit commit_terminal) const {
  actions_.stop_control();
  if (!clearMotionOutputs(reason)) {
    return failClosed({false, "zero_publish_failed_goal_terminal_pending"});
  }
  return confirmAndCommitTerminal(std::move(commit_terminal), reason, "goal_terminal_pending",
                                  true);
}

MotionStopResult MotionStopCoordinator::stop() const {
  actions_.stop_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_stopped");
  MotionStopTerminalCommit commit_stopped = deferGoalAbort("stopped");
  if (!clearMotionOutputs("stopped")) {
    return failClosed({false, "zero_publish_failed"});
  }
  return confirmAndCommitTerminal(std::move(commit_stopped), "stopped", "stop_remains_latched",
                                  true);
}

MotionStopResult MotionStopCoordinator::estop(const std::string &reason) const {
  actions_.latch_estop(reason);
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("estop_latched");
  const bool persisted = actions_.persist_estop_latch(reason);
  MotionStopTerminalCommit commit_estop = deferGoalAbort("estop_latched");
  if (!clearMotionOutputs("estop_latched")) {
    return {
        false,
        "zero_publish_failed_estop_remains_latched",
    };
  }
  const MotionStopResult confirmation = confirmAndCommitTerminal(
      std::move(commit_estop), "estop_latched", "estop_remains_latched", false);
  if (!confirmation.accepted) {
    return confirmation;
  }
  if (!persisted) {
    return {
        false,
        "estop_latch_persist_failed_estop_remains_latched",
    };
  }
  return confirmation;
}

MotionStopResult MotionStopCoordinator::clearEstop(const std::string &precondition_error) const {
  if (!precondition_error.empty()) {
    return {false, precondition_error};
  }
  MotionStopTerminalCommit commit_cleared = deferGoalAbort("estop_cleared");
  if (!clearMotionOutputs("estop_cleared")) {
    return {
        false,
        "zero_publish_failed_estop_remains_latched",
    };
  }
  const MotionStopResult confirmation = confirmAndCommitTerminal(
      std::move(commit_cleared), "zero_confirmed", "estop_remains_latched", false);
  if (!confirmation.accepted) {
    return confirmation;
  }
  if (!actions_.clear_persisted_estop_latch()) {
    return {
        false,
        "estop_latch_clear_failed_estop_remains_latched",
    };
  }
  const bool cleared = actions_.clear_control_estop();
  actions_.clear_operator_resume_required();
  return {
      cleared,
      cleared ? "estop_cleared" : "estop_remains_latched",
  };
}

MotionStopResult MotionStopCoordinator::resumeAutonomy(const ResumeAutonomyRequest &request) const {
  if (!request.precondition_error.empty()) {
    return {false, request.precondition_error};
  }
  if (!request.operator_takeover_latched) {
    return {true, "autonomy_already_ready"};
  }
  MotionStopTerminalCommit commit_ready = deferGoalAbort("autonomy_resume_ready");
  if (!clearMotionOutputs("autonomy_resume_ready")) {
    return {
        false,
        "zero_publish_failed_takeover_remains_latched",
    };
  }
  const MotionStopResult confirmation = confirmAndCommitTerminal(
      std::move(commit_ready), "zero_confirmed", "takeover_remains_latched", false);
  if (!confirmation.accepted) {
    return confirmation;
  }
  if (!actions_.resume_autonomy()) {
    return {false, "estop_latched"};
  }
  actions_.set_autonomy_request_not_before(request.source_stamp_s);
  actions_.clear_operator_resume_required();
  return {true, "autonomy_resume_ready_reissue_goal"};
}

bool MotionStopCoordinator::driverAuthorityLost(const std::string &blocker) const {
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("driver_control_lost:" + blocker);
  return clearEndpointMotion(blocker);
}

bool MotionStopCoordinator::keepZeroFresh() const {
  return actions_.publish_zero();
}

FinalShutdownResult MotionStopCoordinator::finalShutdown() const {
  actions_.stop_control();
  MotionStopTerminalCommit commit_shutdown = deferGoalAbort("navd_shutdown");
  actions_.clear_global_path();
  if (!publish_cmd_vel_) {
    commit_shutdown();
    return {true, std::nullopt};
  }
  const std::optional<std::uint64_t> output_sequence = actions_.publish_sequenced_zero();
  if (!output_sequence) {
    return {false, std::nullopt};
  }
  const StopConfirmationState state = actions_.confirm_zero(*output_sequence);
  if (state == StopConfirmationState::Confirmed) {
    commit_shutdown();
  }
  return {
      state == StopConfirmationState::Confirmed,
      state,
  };
}

}  // namespace lingtu::nav::endpoint
