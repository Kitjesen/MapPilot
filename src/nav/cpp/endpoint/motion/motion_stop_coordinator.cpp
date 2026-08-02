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

MotionStopTerminalBarrierResult
MotionStopCoordinator::cancelPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const {
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_cancelled");
  if (!clearMotionOutputs("cancelled")) {
    const MotionStopResult failure = failClosed({false, "zero_publish_failed"});
    return {failure.accepted, failure.reason, false};
  }
  const MotionStopResult confirmation = confirmLastZero("cancelled", "cancel_remains_stopped");
  if (!confirmation.accepted) {
    const MotionStopResult failure = failClosed(confirmation);
    return {failure.accepted, failure.reason, false};
  }
  commit_terminal();
  return {true, confirmation.reason, true};
}

MotionStopResult MotionStopCoordinator::pauseTask(
    MotionStopTerminalCommit commit_paused) const {
  actions_.stop_control();
  if (!actions_.suspend_motion_outputs("task_paused")) {
    return failClosed({false, "zero_publish_failed_pause_remains_stopped"});
  }
  return confirmAndCommitTerminal(std::move(commit_paused), "pause_requested",
                                  "pause_remains_stopped", true);
}

MotionStopResult
MotionStopCoordinator::confirmGoalReplanStop(const std::string &reason) const {
  actions_.stop_control();
  if (!clearMotionOutputs(reason)) {
    return failClosed({false, "zero_publish_failed_goal_replan_pending"});
  }
  const MotionStopResult confirmation =
      confirmLastZero("replan_stop_confirmed", "goal_replan_pending");
  if (!confirmation.accepted) {
    return failClosed(confirmation);
  }
  return confirmation;
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

MotionStopResult MotionStopCoordinator::stopWithoutTerminalCommit() const {
  const MotionStopTerminalBarrierResult stopped = runStopPipeline(std::nullopt);
  return {stopped.accepted, stopped.reason};
}

MotionStopTerminalBarrierResult
MotionStopCoordinator::stopPreservingGoalTerminal(MotionStopTerminalCommit commit_terminal) const {
  return runStopPipeline(std::move(commit_terminal));
}

MotionStopTerminalBarrierResult MotionStopCoordinator::runStopPipeline(
    std::optional<MotionStopTerminalCommit> commit_terminal) const {
  actions_.stop_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("navigation_stopped");
  if (!clearMotionOutputs("stopped")) {
    const MotionStopResult failure = failClosed({false, "zero_publish_failed"});
    return {failure.accepted, failure.reason, false};
  }
  const MotionStopResult confirmation = confirmLastZero("stopped", "stop_remains_latched");
  if (!confirmation.accepted) {
    const MotionStopResult failure = failClosed(confirmation);
    return {failure.accepted, failure.reason, false};
  }
  if (commit_terminal) {
    (*commit_terminal)();
  }
  return {true, confirmation.reason, commit_terminal.has_value()};
}

MotionStopResult MotionStopCoordinator::estop(const std::string &reason) const {
  const MotionStopTerminalBarrierResult estopped =
      runEstopPipeline(reason, std::nullopt, true);
  return {estopped.accepted, estopped.reason};
}

MotionStopResult
MotionStopCoordinator::estopWithoutTerminalCommit(const std::string &reason) const {
  const MotionStopTerminalBarrierResult estopped =
      runEstopPipeline(reason, std::nullopt, false);
  return {estopped.accepted, estopped.reason};
}

MotionStopTerminalBarrierResult
MotionStopCoordinator::estopPreservingGoalTerminal(const std::string &estop_reason,
                                                   MotionStopTerminalCommit commit_terminal) const {
  return runEstopPipeline(estop_reason, std::move(commit_terminal), false);
}

MotionStopTerminalBarrierResult MotionStopCoordinator::runEstopPipeline(
    const std::string &estop_reason,
    std::optional<MotionStopTerminalCommit> commit_terminal,
    bool defer_goal_terminal) const {
  actions_.latch_estop(estop_reason);
  const bool persisted = actions_.persist_estop_latch(estop_reason);
  actions_.cancel_control();
  actions_.clear_operator_resume_required();
  actions_.cancel_inspection("estop_latched");
  if (!clearMotionOutputs("estop_latched")) {
    return {
        false,
        "zero_publish_failed_estop_remains_latched",
        false,
    };
  }
  const MotionStopResult confirmation = confirmLastZero("estop_latched", "estop_remains_latched");
  if (!confirmation.accepted) {
    return {confirmation.accepted, confirmation.reason, false};
  }
  if (!persisted) {
    return {
        false,
        "estop_latch_persist_failed_estop_remains_latched",
        false,
    };
  }
  if (defer_goal_terminal) {
    commit_terminal = deferGoalAbort("estop_latched");
  }
  if (commit_terminal) {
    (*commit_terminal)();
  }
  return {true, "estop_latched", commit_terminal.has_value()};
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
  return finalShutdownWithoutTerminalCommit();
}

FinalShutdownResult MotionStopCoordinator::finalShutdownWithoutTerminalCommit() const {
  return runFinalShutdownPipeline(std::nullopt);
}

FinalShutdownResult MotionStopCoordinator::finalShutdownPreservingGoalTerminal(
    MotionStopTerminalCommit commit_terminal) const {
  return runFinalShutdownPipeline(std::move(commit_terminal));
}

FinalShutdownResult MotionStopCoordinator::runFinalShutdownPipeline(
    std::optional<MotionStopTerminalCommit> commit_terminal) const {
  actions_.stop_control();
  actions_.cancel_inspection("navd_shutdown");
  if (!clearMotionOutputs("navd_shutdown")) {
    return {false, std::nullopt};
  }
  if (!publish_cmd_vel_) {
    return {false, std::nullopt};
  }
  const std::optional<std::uint64_t> output_sequence = actions_.publish_sequenced_zero();
  if (!output_sequence) {
    return {false, std::nullopt};
  }
  const StopConfirmationState state = actions_.confirm_zero(*output_sequence);
  if (state == StopConfirmationState::Confirmed) {
    if (commit_terminal) {
      (*commit_terminal)();
    }
  }
  return {
      state == StopConfirmationState::Confirmed,
      state,
  };
}

}  // namespace lingtu::nav::endpoint
