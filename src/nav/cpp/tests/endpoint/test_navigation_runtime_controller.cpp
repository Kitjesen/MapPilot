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
#include "navigation_runtime_controller.hpp"
#include "plan/goal_replan_runtime_coordinator.hpp"
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
using lingtu::nav::endpoint::MotionStopCoordinator;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::NavigationRuntimeAutonomyObservation;
using lingtu::nav::endpoint::NavigationRuntimeController;
using lingtu::nav::endpoint::NavigationRuntimeFrameActions;
using lingtu::nav::endpoint::NavigationRuntimePostAutonomyState;
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
  std::vector<GoalPlanStatus> observed_statuses;
  std::vector<GoalPlanStatus> write_attempts;
  std::vector<std::string> reported_errors;
  std::vector<std::string> inspection_pauses;
  std::vector<std::string> estop_reasons;
  std::size_t stop_control_calls{0U};
  std::size_t diagnostic_syncs{0U};
  NavigationGoalStatusOutbox outbox;
  GoalPlanController goal_plan;
  MotionStopCoordinator motion_stop;
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
    actions.latch_estop = [this](const std::string &reason) { estop_reasons.push_back(reason); };
    actions.clear_control_estop = [] { return true; };
    actions.resume_autonomy = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};
    actions.clear_operator_resume_required = [] {};
    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [] { return true; };
    actions.last_output_sequence = [] { return 1U; };
    actions.publish_sequenced_zero = [] { return std::optional<std::uint64_t>{2U}; };
    actions.confirm_zero = [this](std::uint64_t) { return confirmation; };
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

  void activateGoal() {
    require(goal_plan.submit(request(), admission()).accepted, "fixture goal submission failed");
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

  NavigationRuntimeFrameActions idleActions(std::size_t *run_calls = nullptr,
                                            std::size_t *apply_calls = nullptr) {
    NavigationRuntimeFrameActions actions;
    actions.complete_endpoint_work_before_autonomy = [](const GoalReplanRuntimeResult &) {};
    actions.run_autonomy = [this, run_calls](const GoalPlanSnapshot &) {
      if (run_calls) {
        ++*run_calls;
      }
      return NavigationRuntimeAutonomyObservation{frame(10.0), AutonomyTickOutcome{}, false, false,
                                                  false};
    };
    actions.apply_autonomy_outputs = [apply_calls](const GoalReplanRuntimeResult &) {
      if (apply_calls) {
        ++*apply_calls;
      }
      return NavigationRuntimePostAutonomyState{};
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
    return NavigationRuntimeAutonomyObservation{fixture.frame(10.1), AutonomyTickOutcome{}, false,
                                                false, false};
  };
  actions.apply_autonomy_outputs = [&](const GoalReplanRuntimeResult &runtime_outcome) {
    order.push_back("apply_outputs");
    require(!runtime_outcome.handled, "empty autonomy outcome was unexpectedly handled");
    return NavigationRuntimePostAutonomyState{};
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

void testInspectionFallbackReturnsCompletionOnlyAfterTerminalAck() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = false;
  std::size_t run_calls = 0U;
  std::size_t apply_calls = 0U;
  NavigationRuntimeFrameActions actions;
  actions.complete_endpoint_work_before_autonomy = [](const GoalReplanRuntimeResult &) {};
  actions.run_autonomy = [&](const GoalPlanSnapshot &snapshot) {
    ++run_calls;
    require(snapshot.active_task_id == "task-a", "fallback tick lost the captured active goal");
    return NavigationRuntimeAutonomyObservation{
        fixture.frame(30.1),
        AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalFailed, "inspection_nav_failed", false,
                            std::nullopt},
        true,
        true,
        false,
    };
  };
  actions.apply_autonomy_outputs = [&](const GoalReplanRuntimeResult &runtime_outcome) {
    ++apply_calls;
    require(!runtime_outcome.handled,
            "external goal failure with active inspection bypassed fallback ownership");
    return NavigationRuntimePostAutonomyState{true, true};
  };

  const auto pending = fixture.runtime.advanceFrame(fixture.frame(30.0), actions);

  require(run_calls == 1U && apply_calls == 1U && pending.autonomy_result.has_value(),
          "fallback frame did not run and apply one autonomy observation");
  require(!pending.terminal_delivery_acknowledged && !pending.inspection_completion &&
              !pending.pending_cycle_advanced && fixture.runtime.terminalPending(),
          "inspection completion escaped before durable terminal acknowledgement");

  fixture.writes_allowed = true;
  const auto completed = fixture.runtime.advanceFrame(fixture.frame(30.2), actions);

  require(run_calls == 1U && apply_calls == 1U,
          "terminal replay reran the autonomy observation or output application");
  require(completed.terminal_delivery_acknowledged && completed.inspection_completion &&
              completed.inspection_completion->kind == AutonomyTickOutcomeKind::kGoalFailed &&
              completed.inspection_completion->reason == "inspection_nav_failed" &&
              completed.pending_cycle_advanced,
          "terminal acknowledgement did not return the exact deferred inspection completion");

  const auto next = fixture.runtime.advanceFrame(fixture.frame(30.3), fixture.idleActions());
  require(!next.inspection_completion,
          "acknowledged inspection completion was returned more than once");
}

void testUnhandledAutonomyTickCannotCreateInspectionFallbackTerminal() {
  Fixture fixture;
  fixture.activateGoal();
  NavigationRuntimeFrameActions actions;
  actions.complete_endpoint_work_before_autonomy = [](const GoalReplanRuntimeResult &) {};
  actions.run_autonomy = [&](const GoalPlanSnapshot &) {
    return NavigationRuntimeAutonomyObservation{
        fixture.frame(35.1),
        AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalFailed, "ignored_tick_failure", false,
                            std::nullopt},
        false,
        true,
        false,
    };
  };
  actions.apply_autonomy_outputs = [](const GoalReplanRuntimeResult &runtime_outcome) {
    require(!runtime_outcome.handled,
            "unhandled autonomy fixture was unexpectedly consumed by the replan runtime");
    return NavigationRuntimePostAutonomyState{true, true};
  };

  const auto result = fixture.runtime.advanceFrame(fixture.frame(35.0), actions);

  require(result.autonomy_result && !result.autonomy_result->handled,
          "unhandled autonomy observation was not preserved");
  require(!result.terminal_delivery_acknowledged && !result.inspection_completion &&
              result.pending_cycle_advanced && !fixture.runtime.terminalPending(),
          "unhandled autonomy tick created an inspection fallback terminal");
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
        true,
        false,
        false,
    };
  };
  actions.apply_autonomy_outputs = [](const GoalReplanRuntimeResult &) {
    return NavigationRuntimePostAutonomyState{};
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

}  // namespace

int main() {
  try {
    testNormalFrameUsesStrictTypedContinuationOrder();
    testPlanningTerminalBlocksAutonomyAndDrainUntilReplayAcknowledges();
    testInspectionFallbackReturnsCompletionOnlyAfterTerminalAck();
    testUnhandledAutonomyTickCannotCreateInspectionFallbackTerminal();
    testCapturedSnapshotCannotCommitReachedAfterGoalStateChangesDuringTick();
    testTerminalCompanionInterfacesForwardExactTransactions();
    testIncompleteFrameActionsAreRejectedBeforeRuntimeMutation();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_navigation_runtime_controller: FAIL: %s\n", exc.what());
    return 1;
  }
}
