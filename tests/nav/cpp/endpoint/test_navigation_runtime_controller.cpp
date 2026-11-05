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

#include "safety/stop.hpp"
#include "app/navigation.hpp"
#include "app/goal/runtime.hpp"
#include "status/goal_terminal_status_delivery.hpp"
#include "status/goal_terminal_transaction.hpp"
#include "status/navigation_goal_status_outbox.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::AutonomyTickOutcome;
using lingtu::nav::endpoint::AutonomyTickOutcomeKind;
using lingtu::nav::endpoint::GoalPlanActions;
using lingtu::nav::endpoint::GoalPlanAdmissionContext;
using lingtu::nav::endpoint::GoalPlanAdvanceContext;
using lingtu::nav::endpoint::GoalPlanController;
using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
using lingtu::nav::endpoint::GoalPlanOrigin;
using lingtu::nav::endpoint::GoalPlanPathActivation;
using lingtu::nav::endpoint::GoalPlanRequest;
using lingtu::nav::endpoint::GoalPlanSnapshot;
using lingtu::nav::endpoint::GoalPlanStatus;
using lingtu::nav::endpoint::GoalPlanTarget;
using lingtu::nav::endpoint::GoalReplanRuntimeCoordinator;
using lingtu::nav::endpoint::GoalReplanRuntimeFrameInput;
using lingtu::nav::endpoint::GoalReplanRuntimeInterruption;
using lingtu::nav::endpoint::GoalReplanRuntimeResult;
using lingtu::nav::endpoint::GoalTerminalStatusDelivery;
using lingtu::nav::endpoint::GoalTerminalTransaction;
using lingtu::nav::endpoint::GoalTerminalTransactionActions;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopBarrier;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::NavigationRuntimeAutonomyObservation;
using lingtu::nav::endpoint::NavigationRuntimeController;
using lingtu::nav::endpoint::NavigationRuntimeFrameActions;
using lingtu::nav::endpoint::StopConfirmationState;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  lingtu::nav::plan::MapIdentity map_identity{"field", 7, "map"};
  bool writes_allowed{true};
  StopConfirmationState confirmation{StopConfirmationState::Confirmed};
  std::vector<GoalPlanStatus> observed_statuses;
  std::vector<GoalPlanStatus> write_attempts;
  std::vector<std::string> reported_errors;
  std::vector<std::string> inspection_pauses;
  std::vector<std::string> estop_reasons;
  std::vector<std::string> motion_outputs;
  std::size_t stop_control_calls{0U};
  std::size_t diagnostic_syncs{0U};
  std::size_t path_activations{0U};
  NavigationGoalStatusOutbox outbox;
  GoalPlanController goal_plan;
  MotionStopBarrier motion_stop;
  GoalReplanRuntimeCoordinator coordinator;
  GoalTerminalStatusDelivery delivery;
  GoalTerminalTransaction terminal_transaction;
  NavigationRuntimeController runtime;

  Fixture()
      : outbox([this](const GoalPlanStatus &status) { observed_statuses.push_back(status); },
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
        terminal_transaction(coordinator, motion_stop, delivery, terminalActions()),
        runtime(goal_plan, coordinator, terminal_transaction) {}

  GoalPlanActions goalActions() {
    GoalPlanActions actions;
    actions.preempt_rolling = [](const std::string &) { return true; };
    actions.clear_external_inspection = [] {};
    actions.current_map_identity = [this] { return GoalPlanMapIdentityResult{map_identity, {}}; };
    actions.publish_status = [this](const GoalPlanStatus &status) {
      require(outbox.record(status), "fixture status was rejected by the outbox");
    };
    actions.inspection_active = [] { return false; };
    actions.inspection_leg_failed = [](const std::string &, double) {};
    actions.inspection_pause = [](const std::string &) {};
    actions.inspection_plan_ready = [](double) {
      return lingtu::nav::endpoint::GoalPlanInspectionDecision{};
    };
    actions.activate_path = [this](const GoalPlanPathActivation &) { ++path_activations; };
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
    actions.latch_estop = [this](const std::string &reason) { estop_reasons.push_back(reason); };
    actions.clear_control_estop = [] { return true; };
    actions.resume_control = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};
    actions.clear_operator_resume_required = [] {};
    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [this] {
      motion_outputs.emplace_back("zero");
      return true;
    };
    actions.last_output_sequence = [] { return 1U; };
    actions.publish_sequenced_zero = [this] {
      motion_outputs.emplace_back("sequenced_zero");
      return std::optional<std::uint64_t>{2U};
    };
    actions.confirm_zero = [this](std::uint64_t, auto) { return confirmation; };
    actions.clear_global_path = [] {};
    return actions;
  }

  GoalTerminalTransactionActions terminalActions() {
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

  GoalReplanRuntimeFrameInput frame(double now_s) const {
    GoalReplanRuntimeFrameInput input;
    input.steady_now_s = now_s;
    input.wall_now_s = now_s;
    input.fresh_admission = admission();
    return input;
  }

  GoalPlanRequest request(std::string task_id = "task-a",
                          std::string request_id = "request-a") const {
    GoalPlanRequest result;
    result.task_id = std::move(task_id);
    result.request_id = std::move(request_id);
    result.origin = GoalPlanOrigin::kExternal;
    result.source_stamp_s = 1.0;
    result.target = GoalPlanTarget{nav_kernel::Vec3{2.0, 1.0, 0.0}, 0.0};
    return result;
  }

  void activateGoal(GoalPlanOrigin origin = GoalPlanOrigin::kExternal) {
    auto goal = request();
    goal.origin = origin;
    require(goal_plan.submit(goal, admission()).accepted, "fixture goal submission failed");
    for (int index = 0; index < 1000; ++index) {
      if (goal_plan.advance(GoalPlanAdvanceContext{7U, false, 2.0 + index * 0.001})
              .path_activated) {
        const auto delivered = outbox.flush();
        require(origin == GoalPlanOrigin::kInspection || delivered > 0U,
                "fixture initial goal statuses were not delivered");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "fixture goal planning did not complete");
  }

  NavigationRuntimeFrameActions idleActions(std::size_t *run_calls = nullptr,
                                            std::size_t *apply_calls = nullptr) {
    NavigationRuntimeFrameActions actions;
    actions.complete_endpoint_work_before_autonomy = [](const GoalReplanRuntimeResult &) {};
    actions.run_autonomy = [this, run_calls](const GoalPlanSnapshot &) {
      if (run_calls) {
        ++*run_calls;
      }
      return NavigationRuntimeAutonomyObservation{frame(10.0), AutonomyTickOutcome{}, false, false};
    };
    actions.apply_autonomy_outputs = [apply_calls](const GoalReplanRuntimeResult &) {
      if (apply_calls) {
        ++*apply_calls;
      }
    };
    return actions;
  }

  std::size_t countStatus(NavigationGoalState state) const {
    std::size_t count = 0U;
    for (const GoalPlanStatus &status : observed_statuses) {
      if (status.task_id == "task-a" && status.state == state) {
        ++count;
      }
    }
    return count;
  }
};

void testNormalFrameUsesStrictTypedContinuationOrder() {
  Fixture fixture;
  std::vector<std::string> order;
  NavigationRuntimeFrameActions actions;
  actions.complete_endpoint_work_before_autonomy = [&](const GoalReplanRuntimeResult &planning) {
    order.push_back("endpoint_work");
    require(planning.reason == "idle", "normal planning result changed");
    require(fixture.goal_plan.submit(fixture.request(), fixture.admission()).accepted,
            "endpoint work could not submit the goal used for snapshot ordering");
  };
  actions.run_autonomy = [&](const GoalPlanSnapshot &snapshot) {
    order.push_back("autonomy");
    require(snapshot.planning_task_id == "task-a" && snapshot.planning_request_id == "request-a",
            "controller captured the goal snapshot before endpoint work");
    return NavigationRuntimeAutonomyObservation{fixture.frame(10.1), AutonomyTickOutcome{}, false, false};
  };
  actions.apply_autonomy_outputs = [&](const GoalReplanRuntimeResult &runtime_outcome) {
    order.push_back("apply_outputs");
    require(!runtime_outcome.handled, "empty autonomy outcome was unexpectedly handled");
  };

  const auto result = fixture.runtime.advanceFrame(fixture.frame(10.0), actions);

  require(order == std::vector<std::string>({"endpoint_work", "autonomy", "apply_outputs"}),
          "normal frame callback order changed");
  require(result.autonomy_result.has_value() && result.pending_cycle_advanced,
          "normal frame omitted autonomy handling or pending drain");
  require(!result.terminal_delivery_acknowledged && !result.inspection_completion,
          "normal frame fabricated a terminal acknowledgement or inspection completion");
}

void testPlanningTerminalBlocksAutonomyAndDrainUntilReplayAcknowledges() {
  Fixture fixture;
  fixture.activateGoal();
  const GoalReplanRuntimeResult pending =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 20.0);
  require(pending.terminal_after_stop.has_value(), "fixture did not create a pending terminal");
  fixture.writes_allowed = false;
  std::size_t run_calls = 0U;
  std::size_t apply_calls = 0U;
  auto actions = fixture.idleActions(&run_calls, &apply_calls);
  std::size_t endpoint_work_calls = 0U;
  actions.complete_endpoint_work_before_autonomy = [&](const GoalReplanRuntimeResult &planning) {
    ++endpoint_work_calls;
    require(planning.terminal_intent_id == pending.terminal_intent_id,
            "planning cycle did not replay the exact terminal intent");
  };

  const auto blocked = fixture.runtime.advanceFrame(fixture.frame(20.1), actions);

  require(endpoint_work_calls == 1U && run_calls == 0U && apply_calls == 0U,
          "terminal-pending frame ran autonomy callbacks or skipped endpoint work");
  require(!blocked.autonomy_result && !blocked.pending_cycle_advanced &&
              !blocked.terminal_delivery_acknowledged && fixture.runtime.terminalPending(),
          "undelivered terminal did not block autonomy and pending drain");

  fixture.writes_allowed = true;
  const auto replayed = fixture.runtime.advanceFrame(fixture.frame(20.2), actions);

  require(endpoint_work_calls == 2U && run_calls == 0U && apply_calls == 0U,
          "acknowledged replay ran autonomy in the surfaced-terminal frame");
  require(replayed.terminal_delivery_acknowledged && replayed.pending_cycle_advanced &&
              !fixture.runtime.terminalPending(),
          "exact terminal replay did not acknowledge and permit pending drain");
  require(fixture.diagnostic_syncs == 1U,
          "terminal replay repeated the stop-confirmed terminal commit");
}

void testInspectionCompletionAfterStop() {
  for (const auto kind : {AutonomyTickOutcomeKind::kGoalReached,
                          AutonomyTickOutcomeKind::kGoalFailed}) {
    Fixture fixture;
    fixture.activateGoal(GoalPlanOrigin::kInspection);
    fixture.confirmation = StopConfirmationState::Pending;
    std::size_t run_calls = 0U;
    auto actions = fixture.idleActions();
    const std::string reason = kind == AutonomyTickOutcomeKind::kGoalReached
                                   ? "goal_reached" : "local_recovery_exhausted";
    actions.run_autonomy = [&](const GoalPlanSnapshot &) {
      ++run_calls;
      return NavigationRuntimeAutonomyObservation{
          fixture.frame(30.1), AutonomyTickOutcome{kind, reason}, true, false};
    };
    const auto pending = fixture.runtime.advanceFrame(fixture.frame(30.0), actions);
    require(pending.autonomy_result && pending.autonomy_result->handled,
            "inspection outcome did not reach its native owner");
    require(!pending.inspection_completion && fixture.runtime.terminalPending(),
            "inspection advanced before stop confirmation");

    fixture.confirmation = StopConfirmationState::Confirmed;
    const auto completed = fixture.runtime.advanceFrame(fixture.frame(30.2), actions);
    require(run_calls == 1U, "terminal replay reran navigation");
    require(completed.inspection_completion && completed.inspection_completion->kind == kind &&
                completed.inspection_completion->reason == reason,
            "normal inspection completion was lost or changed");
    require(completed.terminal_delivery_acknowledged && completed.pending_cycle_advanced,
            "confirmed inspection leg did not release the next leg");
    require(fixture.write_attempts.empty(), "internal waypoint result leaked as a public task end");
    const auto next = fixture.runtime.advanceFrame(fixture.frame(30.3), fixture.idleActions());
    require(!next.inspection_completion, "inspection completion was returned twice");
  }
}

void testInspectionCompletionUsesCommittedResult() {
  Fixture fixture;
  fixture.activateGoal(GoalPlanOrigin::kInspection);
  auto actions = fixture.idleActions();
  actions.run_autonomy = [&](const GoalPlanSnapshot &) {
    auto frame = fixture.frame(30.1);
    frame.map_drift = true;
    return NavigationRuntimeAutonomyObservation{
        frame, AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalReached, "goal_reached"},
        true, false};
  };
  const auto completed = fixture.runtime.advanceFrame(fixture.frame(30.0), actions);
  require(completed.inspection_completion &&
              completed.inspection_completion->kind == AutonomyTickOutcomeKind::kGoalFailed &&
              completed.inspection_completion->reason == "active_goal_map_identity_changed",
          "raw arrival bypassed the committed map-change failure");
}

void testInspectionCancelDoesNotFinishWaypoint() {
  Fixture fixture;
  fixture.activateGoal(GoalPlanOrigin::kInspection);
  (void)fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kCancel, 30.0);
  const auto completed = fixture.runtime.advanceFrame(fixture.frame(30.1), fixture.idleActions());
  require(completed.terminal_delivery_acknowledged && !completed.inspection_completion,
          "cancelling a route was returned as arrival or a retryable failure");
}

void testConsecutiveInspectionLegs() {
  Fixture fixture;
  for (int leg = 0; leg < 3; ++leg) {
    fixture.activateGoal(GoalPlanOrigin::kInspection);
    auto actions = fixture.idleActions();
    actions.run_autonomy = [&](const GoalPlanSnapshot &) {
      return NavigationRuntimeAutonomyObservation{
          fixture.frame(30.0 + leg),
          AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalReached, "goal_reached"},
          true, false};
    };
    const auto completed = fixture.runtime.advanceFrame(fixture.frame(30.0 + leg), actions);
    require(completed.inspection_completion &&
                completed.inspection_completion->kind == AutonomyTickOutcomeKind::kGoalReached &&
                !fixture.runtime.terminalPending(),
            "inspection leg did not finish and release the next goal");
  }
  require(fixture.write_attempts.empty(), "waypoint progress published multiple public tasks");
}

void testExternalGoalCannotCompleteInspection() {
  Fixture fixture;
  fixture.activateGoal();
  NavigationRuntimeFrameActions actions;
  actions.complete_endpoint_work_before_autonomy = [](const GoalReplanRuntimeResult &) {};
  actions.run_autonomy = [&](const GoalPlanSnapshot &) {
    return NavigationRuntimeAutonomyObservation{
        fixture.frame(35.1),
        AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalFailed, "ignored_tick_failure", false,
                            std::nullopt},
        true,
        false,
    };
  };
  actions.apply_autonomy_outputs = [](const GoalReplanRuntimeResult &runtime_outcome) {
    require(!runtime_outcome.handled,
            "unhandled autonomy fixture was unexpectedly consumed by the replan runtime");
  };

  const auto result = fixture.runtime.advanceFrame(fixture.frame(35.0), actions);

  require(result.autonomy_result && !result.autonomy_result->handled,
          "unhandled autonomy observation was not preserved");
  require(!result.terminal_delivery_acknowledged && !result.inspection_completion &&
              result.pending_cycle_advanced && !fixture.runtime.terminalPending(),
          "external goal manufactured an inspection terminal");
}

void testCapturedSnapshotCannotCommitReachedAfterGoalStateChangesDuringTick() {
  Fixture fixture;
  fixture.activateGoal();
  NavigationRuntimeFrameActions actions;
  actions.complete_endpoint_work_before_autonomy = [](const GoalReplanRuntimeResult &) {};
  actions.run_autonomy = [&](const GoalPlanSnapshot &snapshot) {
    require(snapshot.active_task_id == "task-a" && !snapshot.busy,
            "stale-snapshot fixture did not capture stable goal A");
    require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
                .accepted,
            "stale-snapshot fixture could not begin replacement goal B");
    return NavigationRuntimeAutonomyObservation{
        fixture.frame(40.1),
        AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalReached, "goal_reached", true,
                            std::nullopt},
        false,
        false,
    };
  };
  actions.apply_autonomy_outputs = [](const GoalReplanRuntimeResult &) {
  };

  const auto result = fixture.runtime.advanceFrame(fixture.frame(40.0), actions);

  require(result.autonomy_result && result.autonomy_result->handled &&
              result.autonomy_result->reason == "goal_reached_plan_unstable",
          "goal change during tick was not rejected against the controller snapshot");
  require(!result.terminal_delivery_acknowledged && !fixture.runtime.terminalPending() &&
              fixture.countStatus(NavigationGoalState::Reached) == 0U,
          "stale captured goal identity committed a Reached terminal");
}

void testTerminalCompanionInterfacesForwardExactTransactions() {
  {
    Fixture fixture;
    fixture.activateGoal();
    const auto terminal = fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kStop, 50.0);
    const auto completed = fixture.runtime.completeTerminal(terminal);
    require(completed.delivery_acknowledged && !fixture.runtime.terminalPending(),
            "completeTerminal did not forward the exact terminal transaction");
  }

  {
    Fixture fixture;
    fixture.activateGoal();
    (void)fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 51.0);
    const auto stopped = fixture.runtime.stopWhileTerminalPending();
    require(stopped.accepted && stopped.terminal_committed && fixture.runtime.terminalPending() &&
                fixture.stop_control_calls == 1U,
            "terminal-pending Stop did not forward through the controller");
  }

  {
    Fixture fixture;
    fixture.activateGoal();
    (void)fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 52.0);
    const auto estopped = fixture.runtime.estopWhileTerminalPending("operator_estop");
    require(estopped.accepted && estopped.terminal_committed && fixture.runtime.terminalPending() &&
                fixture.estop_reasons == std::vector<std::string>({"operator_estop"}),
            "terminal-pending Estop did not preserve the caller's exact reason");
  }
}

void testLifecycleInterruptionsAreCompletedByController() {
  Fixture fixture;
  fixture.activateGoal();

  const auto stopped =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kStop, 55.0);
  require(stopped.runtime_result.terminal_after_stop.has_value(),
          "controller interruption did not surface the Stop terminal");
  require(stopped.terminal_transaction.has_value() &&
              stopped.terminal_transaction->action_committed &&
              stopped.terminal_transaction->delivery_acknowledged,
          "controller interruption did not complete the Stop terminal transaction");
  require(!fixture.runtime.terminalPending(),
          "completed interruption left an acknowledged terminal pending");
}

void testGoalSubmissionOwnsExternalPreemptionAndTerminalBarrier() {
  {
    Fixture fixture;
    fixture.activateGoal();
    const auto submitted =
        fixture.runtime.submitGoal(fixture.request("task-b", "request-b"), fixture.admission(), 56.0);
    require(submitted.plan_result.accepted,
            "controller did not admit an external replacement goal");
    require(!submitted.preemption_terminal,
            "ordinary external replacement unexpectedly created a terminal transaction");
  }

  {
    Fixture fixture;
    fixture.activateGoal();
    const auto pending =
        fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 57.0);
    require(pending.terminal_after_stop.has_value(),
            "fixture did not create a terminal barrier for goal admission");
    const auto blocked =
        fixture.runtime.submitGoal(fixture.request("task-b", "request-b"), fixture.admission(), 58.0);
    require(!blocked.plan_result.accepted && blocked.plan_result.reason == "goal_terminal_pending",
            "new goal bypassed the pending terminal barrier");
    require(fixture.runtime.terminalPending(),
            "blocked new goal unexpectedly cleared the pending terminal");
  }
}

void testRejectedReplacementPreservesTheActiveGoalRetry() {
  Fixture fixture;
  fixture.activateGoal();
  const auto original = fixture.goal_plan.snapshot();
  lingtu::nav::endpoint::GoalReplanTrigger trigger;
  trigger.kind = lingtu::nav::endpoint::GoalReplanTriggerKind::kLocalRecoveryExhausted;
  trigger.reason = "local_recovery_exhausted";
  trigger.goal = {original.active_task_id, original.active_request_id,
                  original.active_goal_epoch, *original.active_map_identity};
  const auto armed = fixture.coordinator.handleAutonomyOutcome(
      fixture.frame(58.1),
      {AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalFailed,
                           "local_recovery_exhausted", false, trigger},
       original, false, false});
  require(armed.reason == "backoff_pending" && fixture.coordinator.replanPending(),
          "rejected replacement fixture did not arm the active goal retry");
  const auto retry_before = fixture.coordinator.snapshot();

  auto blocked_admission = fixture.admission();
  blocked_admission.driver_control_blocker = "driver_heartbeat_stale";
  const auto rejected = fixture.runtime.submitGoal(
      fixture.request("task-b", "request-b"), blocked_admission, 58.2);
  require(!rejected.plan_result.accepted &&
              rejected.plan_result.reason == "driver_heartbeat_stale" &&
              !rejected.preemption_terminal && !fixture.runtime.terminalPending(),
          "blocked replacement goal did not retain its admission rejection");
  const auto retained = fixture.goal_plan.snapshot();
  const auto retry_after = fixture.coordinator.snapshot();
  require(retained.active_task_id == original.active_task_id &&
              retained.active_request_id == original.active_request_id &&
              retained.active_goal_epoch == original.active_goal_epoch &&
              fixture.coordinator.replanPending() && retry_after.state == retry_before.state &&
              retry_after.deadline_s == retry_before.deadline_s,
          "rejected replacement cancelled the active goal retry");

  const auto restarted =
      fixture.coordinator.advancePlanningCycle(fixture.frame(retry_before.deadline_s));
  require(restarted.replan_started && !restarted.terminal_after_stop,
          "healthy frame could not resume the original goal after replacement rejection");
  for (int index = 0; index < 1000; ++index) {
    const auto completed = fixture.coordinator.advancePlanningCycle(
        fixture.frame(retry_before.deadline_s + .001 + index * .001));
    if (completed.plan_advance.path_activated) {
      require(fixture.goal_plan.snapshot().active_task_id == original.active_task_id &&
                  fixture.countStatus(NavigationGoalState::Failed) == 0U,
              "replacement rejection changed the goal completed by replan");
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(false, "original goal did not activate its replanned path");
}

void testAcceptedGoalWaitsForSupersededDeferredReplacementTerminal() {
  Fixture fixture;
  fixture.activateGoal();
  auto advance = [&](double now) {
    const auto frame = fixture.frame(now);
    auto actions = fixture.idleActions();
    actions.run_autonomy = [frame](const GoalPlanSnapshot &) {
      return NavigationRuntimeAutonomyObservation{frame, AutonomyTickOutcome{}, false, false};
    };
    return fixture.runtime.advanceFrame(frame, actions);
  };
  require(fixture.runtime.submitGoal(fixture.request("task-b", "request-b"),
                                      fixture.admission(), 58.3)
              .plan_result.accepted,
          "deferred replacement fixture could not submit B");
  GoalReplanRuntimeResult replacement;
  for (int index = 0; index < 1000; ++index) {
    replacement = fixture.coordinator.advancePlanningCycle(fixture.frame(58.4 + index * .001));
    if (replacement.terminal_after_stop) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(replacement.terminal_after_stop && replacement.terminal_task_id == "task-a",
          "replacement B did not wait for A terminal");
  require(fixture.terminal_transaction.advance(replacement).delivery_acknowledged &&
              !fixture.runtime.terminalPending() &&
              fixture.goal_plan.snapshot().deferred_replacement_task_id == "task-b",
          "replacement fixture did not retain B after A acknowledgement");

  fixture.writes_allowed = false;
  const auto submitted = fixture.runtime.submitGoal(
      fixture.request("task-c", "request-c"), fixture.admission(), 59.5);
  require(submitted.plan_result.accepted && submitted.preemption_terminal &&
              !submitted.preemption_terminal->delivery_acknowledged &&
              fixture.runtime.terminalPending() &&
              fixture.goal_plan.snapshot().planning_task_id == "task-c",
          "accepted C was lost while superseding deferred B");
  const auto held = advance(59.6);
  require(!held.planning_result.plan_advance.path_activated &&
              fixture.path_activations == 1U && fixture.runtime.terminalPending(),
          "new C path activated before superseded B terminal acknowledgement");

  fixture.writes_allowed = true;
  const auto acknowledged = advance(59.7);
  require(acknowledged.terminal_delivery_acknowledged && !fixture.runtime.terminalPending(),
          "superseded B terminal could not be acknowledged");
  for (int index = 0; index < 1000; ++index) {
    const auto completed = advance(59.8 + index * .001);
    if (completed.planning_result.plan_advance.path_activated) {
      require(fixture.goal_plan.snapshot().active_task_id == "task-c" &&
                  fixture.path_activations == 2U,
              "terminal acknowledgement activated the superseded B path instead of C");
      for (const auto &status : fixture.observed_statuses) {
        require(status.task_id != "task-c" ||
                    (status.state != NavigationGoalState::Cancelled &&
                     status.state != NavigationGoalState::Failed),
                "superseded B terminal was attributed to accepted C");
      }
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(false, "accepted C did not activate after the superseded B terminal");
}

void testControllerInterruptionPreservesEstopReason() {
  Fixture fixture;
  fixture.activateGoal();
  const auto estopped =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kEstop, 59.0, "operator_estop");
  require(estopped.terminal_transaction.has_value() &&
              estopped.terminal_transaction->action_committed,
          "controller did not commit the estop terminal transaction");
  require(fixture.estop_reasons == std::vector<std::string>{"operator_estop"},
          "controller did not preserve the caller's estop reason");
}

void testIncompleteFrameActionsAreRejectedBeforeRuntimeMutation() {
  Fixture fixture;
  const auto before = fixture.goal_plan.snapshot();
  bool rejected = false;
  try {
    (void)fixture.runtime.advanceFrame(fixture.frame(60.0), NavigationRuntimeFrameActions{});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  const auto after = fixture.goal_plan.snapshot();
  require(rejected, "controller accepted incomplete frame actions");
  require(before.goal_epoch == after.goal_epoch && before.busy == after.busy &&
              !fixture.runtime.terminalPending(),
          "invalid frame actions mutated navigation runtime state");
}

void testManualTakeoverIsTheOnlyFrameOutputForPausedActiveGoal() {
  Fixture fixture;
  fixture.activateGoal();
  const auto paused = fixture.runtime.interrupt(
      GoalReplanRuntimeInterruption::kOperatorTakeover, 61.0);
  require(paused.runtime_result.reason == "task_paused" &&
              fixture.goal_plan.snapshot().active_paused,
          "active-goal takeover fixture did not pause its task");
  fixture.motion_outputs.clear();

  auto actions = fixture.idleActions();
  actions.complete_endpoint_work_before_autonomy = [&](const GoalReplanRuntimeResult &) {
    fixture.motion_outputs.emplace_back("manual_nonzero");
  };
  (void)fixture.runtime.advanceFrame(fixture.frame(61.1), actions);

  require(fixture.motion_outputs == std::vector<std::string>{"manual_nonzero"},
          "paused active goal overwrote the frame's manual nonzero command with a later zero");
}

void testManualTakeoverIsTheOnlyFrameOutputForPausedInitialPlanning() {
  Fixture fixture;
  require(fixture.goal_plan.submit(fixture.request(), fixture.admission()).accepted,
          "initial-planning takeover fixture could not submit its goal");
  const auto original = fixture.goal_plan.snapshot();
  require(!original.planning_task_id.empty() && original.active_task_id.empty(),
          "initial-planning takeover fixture unexpectedly activated its goal");
  const auto paused = fixture.runtime.interrupt(
      GoalReplanRuntimeInterruption::kOperatorTakeover, 62.0);
  const auto retained = fixture.goal_plan.snapshot();
  require(paused.runtime_result.reason == "task_paused" && retained.active_paused &&
              retained.pending_plan_queued &&
              retained.pending_goal_epoch == original.planning_goal_epoch,
          "planning takeover did not retain the same pending goal epoch");
  fixture.motion_outputs.clear();

  auto actions = fixture.idleActions();
  actions.complete_endpoint_work_before_autonomy = [&](const GoalReplanRuntimeResult &) {
    fixture.motion_outputs.emplace_back("manual_nonzero");
  };
  (void)fixture.runtime.advanceFrame(fixture.frame(62.1), actions);

  require(fixture.motion_outputs == std::vector<std::string>{"manual_nonzero"},
          "paused pending drain overwrote the frame's manual nonzero command with zero");
}

void testPausedCoordinatorIsSilentAndEndpointOwnsZeroHeartbeat() {
  Fixture fixture;
  fixture.activateGoal();
  (void)fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kOperatorTakeover, 63.0);
  fixture.motion_outputs.clear();

  const auto planning = fixture.coordinator.advancePlanningCycle(fixture.frame(63.1));
  const auto draining = fixture.coordinator.drainPendingCycle(fixture.frame(63.1));
  require(planning.reason == "task_paused" && !planning.zero_kept_fresh &&
              !draining.zero_kept_fresh && fixture.motion_outputs.empty(),
          "paused coordinator wrote a fallback zero without owning a frame command");

  require(fixture.motion_stop.keepZeroFresh() &&
              fixture.motion_outputs == std::vector<std::string>{"zero"},
          "Endpoint MotionStop no longer owns the explicit idle zero heartbeat");
}

}  // namespace

int main() {
  try {
    testNormalFrameUsesStrictTypedContinuationOrder();
    testPlanningTerminalBlocksAutonomyAndDrainUntilReplayAcknowledges();
    testInspectionCompletionAfterStop();
    testInspectionCompletionUsesCommittedResult();
    testInspectionCancelDoesNotFinishWaypoint();
    testConsecutiveInspectionLegs();
    testExternalGoalCannotCompleteInspection();
    testCapturedSnapshotCannotCommitReachedAfterGoalStateChangesDuringTick();
    testTerminalCompanionInterfacesForwardExactTransactions();
    testLifecycleInterruptionsAreCompletedByController();
    testGoalSubmissionOwnsExternalPreemptionAndTerminalBarrier();
    testRejectedReplacementPreservesTheActiveGoalRetry();
    testAcceptedGoalWaitsForSupersededDeferredReplacementTerminal();
    testControllerInterruptionPreservesEstopReason();
    testIncompleteFrameActionsAreRejectedBeforeRuntimeMutation();
    testManualTakeoverIsTheOnlyFrameOutputForPausedActiveGoal();
    testManualTakeoverIsTheOnlyFrameOutputForPausedInitialPlanning();
    testPausedCoordinatorIsSilentAndEndpointOwnsZeroHeartbeat();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_navigation_runtime_controller: FAIL: %s\n", exc.what());
    return 1;
  }
}
