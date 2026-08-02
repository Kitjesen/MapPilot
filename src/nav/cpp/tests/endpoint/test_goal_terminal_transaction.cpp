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

#include "motion/motion_stop_coordinator.hpp"
#include "plan/goal_replan_runtime_coordinator.hpp"
#include "status/goal_terminal_status_delivery.hpp"
#include "status/goal_terminal_transaction.hpp"
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
using lingtu::nav::endpoint::GoalTerminalTransaction;
using lingtu::nav::endpoint::GoalTerminalTransactionActions;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopCoordinator;
using lingtu::nav::endpoint::MotionStopTerminalBarrierResult;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::StopConfirmationState;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  lingtu::nav::plan::MapIdentity map_identity{"field", 7, "sha256-a", "map"};
  bool writes_allowed{true};
  StopConfirmationState confirmation{StopConfirmationState::Confirmed};
  std::vector<GoalPlanStatus> observed;
  std::vector<GoalPlanStatus> write_attempts;
  std::vector<std::string> reported_errors;
  std::vector<std::string> inspection_pauses;
  std::vector<std::string> latched_estop_reasons;
  std::size_t stop_control_calls{0U};
  std::size_t diagnostic_syncs{0U};
  NavigationGoalStatusOutbox outbox;
  GoalPlanController goal_plan;
  MotionStopCoordinator motion_stop;
  GoalReplanRuntimeCoordinator coordinator;
  GoalTerminalStatusDelivery delivery;
  GoalTerminalTransaction transaction;

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
    actions.record_stop_evidence_failure = [](const std::string &) {};
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
    actions.resume_autonomy = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};
    actions.clear_operator_resume_required = [] {};
    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [] { return true; };
    actions.last_output_sequence = [] { return 1U; };
    actions.publish_sequenced_zero = [] { return std::optional<std::uint64_t>{1U}; };
    actions.confirm_zero = [this](std::uint64_t) { return confirmation; };
    actions.clear_global_path = [] {};
    return actions;
  }

  GoalTerminalTransactionActions transactionActions() {
    GoalTerminalTransactionActions actions;
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
      if (goal_plan.advance(GoalPlanAdvanceContext{7U, false, 2.0 + index * 0.001})
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
};

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

  const MotionStopTerminalBarrierResult stopped = fixture.transaction.stopWhileTerminalPending();

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
      fixture.transaction.estopWhileTerminalPending("operator_estop");

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
      fixture.transaction.estopWhileTerminalPending("operator_estop");

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

  const MotionStopTerminalBarrierResult stopped = fixture.transaction.stopWhileTerminalPending();

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
    GoalTerminalTransaction invalid(fixture.coordinator, fixture.motion_stop, fixture.delivery,
                                    GoalTerminalTransactionActions{});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "GoalTerminalTransaction accepted incomplete actions");
}

}  // namespace

int main() {
  try {
    testSuccessfulTerminalTransactionCommitsDeliversAndAcknowledges();
    testPendingDeliveryReplayAcknowledgesWithoutRepeatingCommit();
    testStopConfirmationFailureKeepsTerminalPendingAndFailsClosed();
    testMissingAndShutdownTerminalsAreRejectedByOrdinaryTransaction();
    testSafetyStopCommitsExactPendingTerminalWithoutAcknowledgingIt();
    testSafetyEstopPreservesTerminalAndUsesExactReason();
    testMissingPendingTerminalStillPerformsPhysicalStop();
    testStageConflictStillCommitsExactPendingTerminalButFailsAdmission();
    testConstructorRejectsIncompleteActions();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_goal_terminal_transaction: FAIL: %s\n", exc.what());
    return 1;
  }
}
