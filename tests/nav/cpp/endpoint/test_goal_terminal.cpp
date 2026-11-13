#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "command/cancel.hpp"
#include "safety/stop.hpp"
#include "app/goal/runtime.hpp"
#include "status/goal_terminal_status_delivery.hpp"
#include "app/goal/terminal.hpp"
#include "status/navigation_goal_status_outbox.hpp"

namespace {

using lingtu::nav::endpoint::GoalPlanActions;
using lingtu::nav::endpoint::GoalPlanAdmissionContext;
using lingtu::nav::endpoint::GoalPlanAdvanceContext;
using lingtu::nav::endpoint::GoalPlanController;
using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
using lingtu::nav::endpoint::GoalPlanOrigin;
using lingtu::nav::endpoint::GoalPlanPathActivation;
using lingtu::nav::endpoint::GoalPlanRequest;
using lingtu::nav::endpoint::GoalPlanStatus;
using lingtu::nav::endpoint::GoalPlanTarget;
using lingtu::nav::endpoint::GoalReplanRuntimeCoordinator;
using lingtu::nav::endpoint::GoalReplanRuntimeInterruption;
using lingtu::nav::endpoint::GoalReplanRuntimeResult;
using lingtu::nav::endpoint::GoalTerminalStatusDelivery;
using lingtu::nav::endpoint::GoalTerminal;
using lingtu::nav::endpoint::GoalTerminalActions;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopBarrier;
using lingtu::nav::endpoint::MotionStopTerminalBarrierResult;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::StopConfirmationEvidencePolicy;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  lingtu::nav::plan::MapIdentity map_identity{"field", 7, "map"};
  bool writes_allowed{true};
  StopConfirmationState confirmation{StopConfirmationState::Confirmed};
  std::vector<std::optional<StopConfirmationEvidencePolicy>> evidence_policies;
  std::vector<GoalPlanStatus> observed;
  std::vector<GoalPlanStatus> write_attempts;
  std::vector<std::string> reported_errors;
  std::vector<std::string> inspection_pauses;
  std::vector<std::string> latched_estop_reasons;
  std::size_t stop_control_calls{0U};
  std::size_t diagnostic_syncs{0U};
  NavigationGoalStatusOutbox outbox;
  GoalPlanController goal_plan;
  MotionStopBarrier motion_stop;
  GoalReplanRuntimeCoordinator coordinator;
  GoalTerminalStatusDelivery delivery;
  GoalTerminal transaction;

  Fixture()
      : outbox([this](const GoalPlanStatus &status) { observed.push_back(status); },
               [this](const GoalPlanStatus &status) {
                 write_attempts.push_back(status);
                 return writes_allowed;
               }),
        goal_plan(
            [this](const lingtu::nav::plan::GlobalPlanRequest &request,
                   const lingtu::nav::plan::GlobalPlanCancelCheck &) {
              lingtu::nav::plan::GlobalPlanResult result;
              result.ok = true;
              result.reached_goal = true;
              result.map_identity = map_identity;
              result.path = {request.start, request.goal};
              return result;
            },
            goalActions()),
        motion_stop(true, stopActions()),
        coordinator(goal_plan, motion_stop),
        delivery(outbox),
        transaction(coordinator, motion_stop, delivery, transactionActions()) {}

  GoalPlanActions goalActions() {
    GoalPlanActions actions;
    actions.preempt_rolling = [](const std::string &) { return true; };
    actions.clear_external_inspection = [] {};
    actions.current_map_identity = [this] { return GoalPlanMapIdentityResult{map_identity, {}}; };
    actions.publish_status = [this](const GoalPlanStatus &status) {
      if (!outbox.record(status)) {
        throw std::runtime_error("fixture status was rejected by the outbox");
      }
    };
    actions.inspection_active = [] { return false; };
    actions.inspection_leg_failed = [](const std::string &, double) {};
    actions.inspection_pause = [](const std::string &) {};
    actions.inspection_plan_ready = [](double) {
      return lingtu::nav::endpoint::GoalPlanInspectionDecision{};
    };
    actions.activate_path = [](const GoalPlanPathActivation &) {};
    return actions;
  }

  MotionStopActions stopActions() {
    MotionStopActions actions;
    actions.defer_goal_abort = [this](const std::string &reason) {
      return goal_plan.deferAbort(reason);
    };
    actions.hold = [](const std::string &) {};
    actions.sync_goal_diagnostics = [] {};
    actions.rolling_segment_active = [] { return false; };
    actions.preempt_rolling_segment = [](const std::string &) { return true; };
    actions.clear_motion_outputs = [](const std::string &) { return true; };
    actions.suspend_motion_outputs = [](const std::string &) { return true; };
    actions.cancel_control = [] {};
    actions.stop_control = [this] { ++stop_control_calls; };
    actions.latch_estop = [this](const std::string &reason) {
      latched_estop_reasons.push_back(reason);
    };
    actions.clear_control_estop = [] { return true; };
    actions.resume_control = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};

    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [] { return true; };
    actions.last_output_sequence = [] { return 1U; };
    actions.publish_sequenced_zero = [] { return std::optional<std::uint64_t>{1U}; };
    actions.confirm_zero = [this](std::uint64_t, auto policy) {
      evidence_policies.push_back(policy);
      return confirmation;
    };
    actions.clear_global_path = [] {};
    return actions;
  }

  GoalTerminalActions transactionActions() {
    GoalTerminalActions actions;
    actions.report_error = [this](const std::string &reason) { reported_errors.push_back(reason); };
    actions.pause_inspection = [this](const std::string &reason) {
      inspection_pauses.push_back(reason);
    };
    actions.sync_goal_diagnostics = [this] { ++diagnostic_syncs; };
    return actions;
  }

  GoalPlanAdmissionContext admission() const {
    GoalPlanAdmissionContext context;
    context.motion_allowed = true;
    context.autonomy_mode = true;
    context.map_position = nav_kernel::Vec3{0.0, 0.0, 0.0};
    context.odometry_ready = true;
    context.input_ready = true;
    context.planner_map_configured = true;
    context.frame_epoch = 7U;
    return context;
  }

  void activateGoal() {
    GoalPlanRequest request;
    request.task_id = "task-a";
    request.request_id = "request-a";
    request.origin = GoalPlanOrigin::kExternal;
    request.source_stamp_s = 1.0;
    request.target = GoalPlanTarget{nav_kernel::Vec3{2.0, 1.0, 0.0}, 0.0};
    require(goal_plan.submit(request, admission()).accepted, "fixture goal submission failed");

    for (int index = 0; index < 1000; ++index) {
      if (goal_plan.advance(GoalPlanAdvanceContext{7U, {}, 2.0 + index * 0.001})
              .path_activated) {
        require(outbox.flush() > 0U, "fixture initial goal statuses were not delivered");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "fixture goal planning did not complete");
  }

  GoalReplanRuntimeResult terminalIntent(
      GoalReplanRuntimeInterruption interruption = GoalReplanRuntimeInterruption::kMapDrift) {
    const auto result = coordinator.interrupt(interruption, 10.0);
    require(result.terminal_intent_id != 0U && result.terminal_after_stop.has_value(),
            "fixture did not create a terminal intent");
    require(coordinator.terminalPending(), "fixture terminal intent was not retained");
    return result;
  }

  GoalReplanRuntimeResult reachedIntent() {
    lingtu::nav::endpoint::GoalReplanRuntimeFrameInput frame;
    frame.steady_now_s = frame.wall_now_s = 10.0;
    frame.fresh_admission = admission();
    lingtu::nav::endpoint::GoalReplanRuntimeAutonomyEvent event;
    event.outcome.kind = lingtu::nav::endpoint::AutonomyTickOutcomeKind::kGoalReached;
    event.goal_snapshot = goal_plan.snapshot();
    const auto result = coordinator.handleAutonomyOutcome(frame, event);
    require(result.terminal_after_stop.has_value(), "fixture did not create a reached intent");
    return result;
  }
};

void testReachedCommitsWithoutStopEvidence() {
  Fixture reached;
  reached.activateGoal();
  const auto intent = reached.reachedIntent();
  const auto reached_result = reached.transaction.advance(intent);
  require(reached_result.action_committed && reached_result.delivery_acknowledged,
          "reached did not commit directly after FinalControl zero output");
  require(reached.evidence_policies.empty(),
          "normal reached still entered stop evidence tracking");

  Fixture failed;
  failed.activateGoal();
  require(failed.transaction.advance(failed.terminalIntent()).delivery_acknowledged,
          "failed terminal did not complete");
  require(!failed.evidence_policies.back(), "failed terminal weakened its stop evidence");
}

void testSafetyStopAndEstopKeepExplicitTerminalEvidence() {
  for (const bool estop : {false, true}) {
    Fixture fixture;
    fixture.activateGoal();
    fixture.confirmation = StopConfirmationState::Pending;
    const auto pending_terminal = fixture.transaction.advance(fixture.terminalIntent());
    require(pending_terminal.reason == "stop_confirmation_pending",
            "explicit terminal did not enter stop confirmation");
    const auto stopped = estop ? fixture.transaction.estopPending("operator_request")
                               : fixture.transaction.stopPending();
    require(stopped.pending && !stopped.terminal_committed,
            "safety replacement committed before complete stop evidence");
    require(!fixture.evidence_policies.back(),
            "safety replacement reused the reached ACK-only policy");
    (void)fixture.motion_stop.poll();
    require(!fixture.evidence_policies.back(),
            "pending safety replacement recovered the weaker reached policy");
    const auto replay = fixture.transaction.advance(fixture.coordinator.replayPendingTerminal());
    require(!replay.action_committed && !fixture.evidence_policies.back(),
            "ordinary terminal replay weakened the safety replacement policy");
    if (estop) {
      require(fixture.latched_estop_reasons == std::vector<std::string>{"operator_request"},
              "ordinary replay overwrote the latched estop reason");
      (void)fixture.transaction.stopPending();
      (void)fixture.transaction.advance(fixture.coordinator.replayPendingTerminal());
      require(!fixture.evidence_policies.back() &&
                  fixture.latched_estop_reasons == std::vector<std::string>{"operator_request"},
              "a later stop weakened the estop evidence or changed its latch reason");
    }
  }
}

void testSuccessfulTerminalTransactionCommitsDeliversAndAcknowledges() {
  Fixture fixture;
  fixture.activateGoal();
  const GoalReplanRuntimeResult terminal = fixture.terminalIntent();

  const auto result = fixture.transaction.advance(terminal);

  require(result.action_committed && result.delivery_acknowledged &&
              result.reason == "goal_terminal_acknowledged",
          "successful transaction result mismatch");
  require(!fixture.coordinator.terminalPending(),
          "successful transaction did not acknowledge the terminal intent");
  require(fixture.diagnostic_syncs == 1U,
          "successful transaction did not synchronize goal diagnostics exactly once");
  require(fixture.reported_errors.empty(), "successful transaction reported an error");
  require(fixture.inspection_pauses.empty(), "successful transaction paused inspection");
}

void testPendingDeliveryReplayAcknowledgesWithoutRepeatingCommit() {
  Fixture fixture;
  fixture.activateGoal();
  const std::size_t writes_before_terminal = fixture.write_attempts.size();
  fixture.writes_allowed = false;
  const GoalReplanRuntimeResult terminal = fixture.terminalIntent();

  const auto pending = fixture.transaction.advance(terminal);

  require(pending.action_committed && !pending.delivery_acknowledged &&
              pending.reason == "goal_terminal_delivery_pending",
          "failed first delivery did not retain the committed transaction");
  require(fixture.coordinator.terminalPending(),
          "failed first delivery acknowledged the pending terminal");
  require(fixture.diagnostic_syncs == 1U,
          "first transaction did not commit and synchronize exactly once");
  require(fixture.write_attempts.size() == writes_before_terminal + 1U,
          "first terminal delivery did not make exactly one additional write attempt");

  fixture.writes_allowed = true;
  const GoalReplanRuntimeResult replay = fixture.coordinator.replayPendingTerminal();
  const auto acknowledged = fixture.transaction.advance(replay);

  require(acknowledged.action_committed && acknowledged.delivery_acknowledged &&
              acknowledged.reason == "goal_terminal_acknowledged",
          "terminal replay did not finish delivery and acknowledgement");
  require(!fixture.coordinator.terminalPending(),
          "successful replay did not acknowledge the exact terminal intent");
  require(fixture.diagnostic_syncs == 1U,
          "terminal replay repeated the stop-confirmed terminal commit");
  require(fixture.reported_errors.empty(), "delivery retry reported a terminal error");
}

void testStopConfirmationFailureKeepsTerminalPendingAndFailsClosed() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.confirmation = StopConfirmationState::TimedOut;
  const GoalReplanRuntimeResult terminal = fixture.terminalIntent();

  const auto result = fixture.transaction.advance(terminal);
  const std::string expected_reason = "stop_confirmation_timeout_goal_terminal_pending";
  const std::string expected_error = "goal_terminal_stop_unconfirmed:" + expected_reason;

  require(!result.action_committed && !result.delivery_acknowledged &&
              result.reason == expected_reason,
          "stop-confirmation failure result mismatch");
  require(fixture.coordinator.terminalPending(),
          "stop-confirmation failure discarded the pending terminal");
  require(fixture.diagnostic_syncs == 0U,
          "stop-confirmation failure marked the terminal committed");
  require(fixture.inspection_pauses.size() == 1U &&
              fixture.inspection_pauses.front() == expected_error,
          "stop-confirmation failure did not pause inspection with the exact reason");
  require(fixture.reported_errors.size() == 1U && fixture.reported_errors.front() == expected_error,
          "stop-confirmation failure did not report the exact fail-closed reason");
}

void testPendingStopConfirmationHasNoTerminalOrInspectionSideEffects() {
  Fixture fixture;
  fixture.activateGoal();
  const std::size_t observed_before_terminal = fixture.observed.size();
  fixture.confirmation = StopConfirmationState::Pending;
  const GoalReplanRuntimeResult terminal = fixture.terminalIntent();

  const auto pending = fixture.transaction.advance(terminal);
  require(!pending.action_committed && !pending.delivery_acknowledged &&
              pending.reason == "stop_confirmation_pending",
          "pending terminal stop was reported as committed or failed");
  require(fixture.coordinator.terminalPending() && fixture.diagnostic_syncs == 0U &&
              fixture.inspection_pauses.empty() && fixture.reported_errors.empty() &&
              fixture.observed.size() == observed_before_terminal &&
              !fixture.delivery.isCommitted(terminal.terminal_intent_id),
          "pending terminal stop published state, paused inspection, or marked delivery committed");

  fixture.confirmation = StopConfirmationState::Confirmed;
  const auto stop_confirmed = fixture.motion_stop.poll();
  require(stop_confirmed && stop_confirmed->accepted && !stop_confirmed->pending,
          "pending terminal stop did not complete after ACK");
  require(!fixture.delivery.isCommitted(terminal.terminal_intent_id) &&
              fixture.diagnostic_syncs == 0U,
          "physical stop completion bypassed the terminal transaction owner");

  const auto committed = fixture.transaction.advance(fixture.coordinator.replayPendingTerminal());
  require(committed.action_committed && committed.delivery_acknowledged &&
              committed.reason == "goal_terminal_acknowledged" &&
              fixture.diagnostic_syncs == 1U && fixture.reported_errors.empty() &&
              fixture.inspection_pauses.empty(),
          "terminal replay did not commit and acknowledge exactly once after stop ACK");
}

void testMissingAndShutdownTerminalsAreRejectedByOrdinaryTransaction() {
  Fixture fixture;
  const auto missing = fixture.transaction.advance(GoalReplanRuntimeResult{});
  require(!missing.action_committed && !missing.delivery_acknowledged &&
              missing.reason == "goal_terminal_missing",
          "missing terminal was not rejected");
  require(fixture.reported_errors.empty() && fixture.inspection_pauses.empty() &&
              fixture.diagnostic_syncs == 0U,
          "missing terminal performed transaction actions");

  fixture.activateGoal();
  const GoalReplanRuntimeResult shutdown =
      fixture.terminalIntent(GoalReplanRuntimeInterruption::kShutdown);
  const auto rejected = fixture.transaction.advance(shutdown);
  require(!rejected.action_committed && !rejected.delivery_acknowledged &&
              rejected.reason == "shutdown_terminal_requires_transaction",
          "ordinary terminal transaction accepted a shutdown policy");
  require(fixture.coordinator.terminalPending(),
          "ordinary transaction discarded the shutdown terminal");
  require(fixture.diagnostic_syncs == 0U, "ordinary transaction committed the shutdown terminal");
}

void testSafetyStopCommitsExactPendingTerminalWithoutAcknowledgingIt() {
  Fixture fixture;
  fixture.activateGoal();
  (void)fixture.terminalIntent();

  const MotionStopTerminalBarrierResult stopped = fixture.transaction.stopPending();

  require(stopped.accepted && stopped.terminal_committed && stopped.reason == "stopped",
          "safety stop did not preserve the pending terminal commit");
  require(fixture.coordinator.terminalPending(),
          "safety stop acknowledged the terminal before status delivery");
  require(fixture.diagnostic_syncs == 1U,
          "safety stop did not synchronize the committed terminal exactly once");
  require(fixture.stop_control_calls == 1U, "safety stop did not execute the physical stop path");
  require(fixture.reported_errors.empty(), "successful safety stop reported an error");

  const auto delivered = fixture.transaction.advance(fixture.coordinator.replayPendingTerminal());
  require(delivered.action_committed && delivered.delivery_acknowledged,
          "the preserved terminal could not finish delivery and acknowledgement");
  require(fixture.diagnostic_syncs == 1U,
          "terminal delivery replay repeated the safety-stop commit");
}

void testSafetyEstopPreservesTerminalAndUsesExactReason() {
  Fixture fixture;
  fixture.activateGoal();
  (void)fixture.terminalIntent();

  const MotionStopTerminalBarrierResult estopped =
      fixture.transaction.estopPending("operator_estop");

  require(estopped.accepted && estopped.terminal_committed && estopped.reason == "estop_latched",
          "safety estop did not preserve the pending terminal commit");
  require(fixture.coordinator.terminalPending(),
          "safety estop acknowledged the terminal before status delivery");
  require(fixture.latched_estop_reasons.size() == 1U &&
              fixture.latched_estop_reasons.front() == "operator_estop",
          "safety estop did not latch the caller's exact reason");
  require(fixture.diagnostic_syncs == 1U,
          "safety estop did not synchronize the committed terminal exactly once");
}

void testMissingPendingTerminalStillPerformsPhysicalStop() {
  Fixture fixture;

  const MotionStopTerminalBarrierResult stopped =
      fixture.transaction.estopPending("operator_estop");

  require(!stopped.accepted && !stopped.terminal_committed &&
              stopped.reason == "goal_terminal_surface_missing",
          "missing terminal safety action did not fail with the exact reason");
  require(fixture.stop_control_calls == 1U,
          "missing terminal safety action did not execute the physical stop fallback");
  require(fixture.latched_estop_reasons.empty(),
          "missing terminal safety action changed the physical-stop fallback to estop");
  require(fixture.reported_errors.size() == 1U &&
              fixture.reported_errors.front() == "goal_terminal_surface_missing",
          "missing terminal safety action did not report the surface failure");
  require(fixture.diagnostic_syncs == 0U,
          "missing terminal safety action synchronized a nonexistent commit");
}

void testStageConflictStillCommitsExactPendingTerminalButFailsAdmission() {
  Fixture fixture;
  fixture.activateGoal();
  const GoalReplanRuntimeResult terminal = fixture.terminalIntent();
  require(fixture.delivery.stage(terminal.terminal_intent_id + 1U,
                                 terminal.terminal_after_stop->delivery_ticket) ==
              GoalTerminalStatusDelivery::StageResult::kStaged,
          "fixture could not stage the conflicting delivery identity");

  const MotionStopTerminalBarrierResult stopped = fixture.transaction.stopPending();

  require(!stopped.accepted && stopped.terminal_committed &&
              stopped.reason == "goal_terminal_delivery_ticket_conflict",
          "ticket conflict did not preserve the exact terminal while failing admission");
  require(fixture.coordinator.terminalPending(),
          "ticket conflict acknowledged the pending terminal");
  require(fixture.diagnostic_syncs == 1U,
          "ticket conflict did not synchronize the preserved terminal commit");
  require(fixture.reported_errors.size() == 1U &&
              fixture.reported_errors.front() == "goal_terminal_delivery_ticket_conflict",
          "ticket conflict did not report the exact delivery error");
}

void testConstructorRejectsIncompleteActions() {
  Fixture fixture;
  bool rejected = false;
  try {
    GoalTerminal invalid(fixture.coordinator, fixture.motion_stop, fixture.delivery,
                                    GoalTerminalActions{});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "GoalTerminal accepted incomplete actions");
}

void testTaskCancelAcceptsPendingStopWithoutEarlyTerminal() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.confirmation = StopConfirmationState::Pending;
  lingtu::nav::endpoint::GoalTaskCancelRouter router(
      fixture.goal_plan, fixture.coordinator,
      [&](const GoalReplanRuntimeResult &result) {
        const auto serviced = fixture.transaction.advance(result);
        return lingtu::nav::endpoint::GoalTaskCancelTerminalServiceResult{
            serviced.action_committed, serviced.reason, fixture.motion_stop.confirmationPending()};
      }, [] {});
  const auto receipt = router.handle({"task-a", "cancel-request", "operator_cancel", 2.0});
  require(receipt.accepted && receipt.reason == "cancel_requested",
          "accepted cancellation was rejected while the zero ACK was pending");
  require(fixture.coordinator.terminalPending() &&
              fixture.goal_plan.snapshot().active_task_id == "task-a",
          "pending cancellation committed the task early");
  fixture.confirmation = StopConfirmationState::Confirmed;
  require(fixture.motion_stop.poll()->accepted, "cancellation zero did not confirm");
  const auto done = fixture.transaction.advance(fixture.coordinator.replayPendingTerminal());
  require(done.action_committed && done.delivery_acknowledged &&
              !fixture.coordinator.terminalPending(), "confirmed cancellation did not deliver its ticket");
  std::size_t cancelled = 0U;
  for (const auto &status : fixture.observed) {
    if (status.task_id == "task-a" && status.state == lingtu::message::NavigationGoalState::Cancelled) ++cancelled;
  }
  require(cancelled == 1U, "cancellation delivered its terminal more than once");
}

}  // namespace

int main() {
  try {
    testReachedCommitsWithoutStopEvidence();
    testSafetyStopAndEstopKeepExplicitTerminalEvidence();
    testTaskCancelAcceptsPendingStopWithoutEarlyTerminal();
    testSuccessfulTerminalTransactionCommitsDeliversAndAcknowledges();
    testPendingDeliveryReplayAcknowledgesWithoutRepeatingCommit();
    testStopConfirmationFailureKeepsTerminalPendingAndFailsClosed();
    testPendingStopConfirmationHasNoTerminalOrInspectionSideEffects();
    testMissingAndShutdownTerminalsAreRejectedByOrdinaryTransaction();
    testSafetyStopCommitsExactPendingTerminalWithoutAcknowledgingIt();
    testSafetyEstopPreservesTerminalAndUsesExactReason();
    testMissingPendingTerminalStillPerformsPhysicalStop();
    testStageConflictStillCommitsExactPendingTerminalButFailsAdmission();
    testConstructorRejectsIncompleteActions();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_goal_terminal: FAIL: %s\n", exc.what());
    return 1;
  }
}
