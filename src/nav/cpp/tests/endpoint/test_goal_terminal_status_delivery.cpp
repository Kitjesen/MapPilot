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
#include "runtime/goal/runtime.hpp"
#include "status/goal_terminal_status_delivery.hpp"
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
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopBarrier;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::StopConfirmationState;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  lingtu::nav::plan::MapIdentity map_identity{"field", 7, "map"};
  bool writes_allowed{false};
  std::vector<GoalPlanStatus> observed;
  std::vector<GoalPlanStatus> write_attempts;
  NavigationGoalStatusOutbox outbox;
  GoalPlanController goal_plan;
  MotionStopBarrier motion_stop;
  GoalReplanRuntimeCoordinator coordinator;
  int terminal_commit_count{0};

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
        coordinator(goal_plan, motion_stop) {}

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
    actions.stop_control = [] {};
    actions.latch_estop = [](const std::string &) {};
    actions.clear_control_estop = [] { return true; };
    actions.resume_control = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};
    actions.clear_operator_resume_required = [] {};
    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [] { return true; };
    actions.last_output_sequence = [] { return 1U; };
    actions.publish_sequenced_zero = [] { return std::optional<std::uint64_t>{1U}; };
    actions.confirm_zero = [](std::uint64_t) { return StopConfirmationState::Confirmed; };
    actions.clear_global_path = [] {};
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

    for (int i = 0; i < 1000; ++i) {
      if (goal_plan.advance(GoalPlanAdvanceContext{7U, false, 2.0 + i * 0.001}).path_activated) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "fixture goal planning did not complete");
  }

  GoalReplanRuntimeResult terminalIntent() {
    const auto result = coordinator.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 10.0);
    require(result.terminal_intent_id != 0U && result.terminal_after_stop.has_value(),
            "fixture did not create a terminal intent");
    require(coordinator.terminalPending(), "fixture terminal intent was not retained");
    return result;
  }

  void commitAfterConfirmedStop(const GoalReplanRuntimeResult &result) {
    require(result.terminal_after_stop.has_value(), "terminal commit was missing");
    const auto &terminal = *result.terminal_after_stop;
    const auto committed =
        motion_stop.commitGoalTerminalAfterStop(terminal.reason, terminal.commit);
    require(committed.accepted, "terminal did not commit after confirmed stop");
    ++terminal_commit_count;
  }
};

void testStageReplayWaitsForCommitAndRetriesBeforeExactAck() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() > 0U, "fixture did not deliver initial goal statuses");
  fixture.writes_allowed = false;
  const auto pending = fixture.terminalIntent();
  require(!pending.terminal_after_stop->delivery_ticket.statuses.empty(),
          "fixture terminal ticket was empty");

  GoalTerminalStatusDelivery delivery(fixture.outbox);
  auto mutable_ticket = pending.terminal_after_stop->delivery_ticket;
  const auto semantic_replay = mutable_ticket;
  const std::size_t observed_before_stage = fixture.observed.size();
  const std::size_t writes_before_commit = fixture.write_attempts.size();
  require(delivery.stage(0U, semantic_replay) ==
              GoalTerminalStatusDelivery::StageResult::kInvalidIntent,
          "zero terminal intent was accepted");
  require(delivery.stage(pending.terminal_intent_id, mutable_ticket) ==
              GoalTerminalStatusDelivery::StageResult::kStaged,
          "first terminal ticket was not staged");
  require(!delivery.isCommitted(pending.terminal_intent_id),
          "staged terminal was marked committed before its stop-confirmed commit");
  require(fixture.observed.size() == observed_before_stage,
          "delivery bridge recorded a status before terminal commit");

  mutable_ticket.statuses.front().reason = "caller_mutated_after_stage";
  require(delivery.stage(pending.terminal_intent_id, semantic_replay) ==
              GoalTerminalStatusDelivery::StageResult::kReplay,
          "same semantic terminal replay was not idempotent");
  auto conflicting_ticket = semantic_replay;
  conflicting_ticket.statuses.front().reason = "conflicting_terminal";
  require(delivery.stage(pending.terminal_intent_id, conflicting_ticket) ==
              GoalTerminalStatusDelivery::StageResult::kConflict,
          "same intent with a different ticket was accepted");
  require(delivery.stage(pending.terminal_intent_id + 1U, semantic_replay) ==
              GoalTerminalStatusDelivery::StageResult::kConflict,
          "different intent replaced the staged transaction");

  require(!delivery.markCommitted(pending.terminal_intent_id + 1U),
          "wrong intent marked the terminal committed");
  require(delivery.flushAndAcknowledge(fixture.coordinator) ==
              GoalTerminalStatusDelivery::FlushResult::kCommitPending,
          "terminal delivery flushed before a stop-confirmed commit");
  require(fixture.write_attempts.size() == writes_before_commit,
          "terminal delivery wrote before commit");
  require(fixture.coordinator.terminalPending(), "terminal was acknowledged before commit");

  fixture.commitAfterConfirmedStop(pending);
  require(delivery.markCommitted(pending.terminal_intent_id),
          "committed terminal was not marked for delivery");
  require(delivery.isCommitted(pending.terminal_intent_id),
          "delivery bridge did not retain the committed terminal identity");
  require(delivery.flushAndAcknowledge(fixture.coordinator) ==
              GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not retain the terminal acknowledgement");
  require(fixture.write_attempts.size() == writes_before_commit + 1U,
          "terminal writer failure did not attempt exactly one ticket status");
  require(!fixture.outbox.delivered(semantic_replay.statuses.front()),
          "failed writer marked the terminal ticket delivered");
  require(fixture.coordinator.terminalPending(), "writer failure acknowledged the terminal");
  require(fixture.terminal_commit_count == 1,
          "delivery retry repeated the stop-confirmed terminal commit");

  fixture.writes_allowed = true;
  require(delivery.flushAndAcknowledge(fixture.coordinator) ==
              GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "successful retry did not acknowledge the exact terminal intent");
  require(fixture.outbox.delivered(semantic_replay.statuses.front()),
          "successful retry did not deliver the terminal ticket");
  require(!fixture.coordinator.terminalPending(), "exact acknowledgement did not clear terminal");
  require(!delivery.isCommitted(pending.terminal_intent_id),
          "acknowledged terminal remained marked committed");
  require(fixture.terminal_commit_count == 1,
          "successful delivery retry repeated the terminal commit");
}

void testWrongIntentCannotClearPendingTerminal() {
  Fixture fixture;
  fixture.activateGoal();
  const auto pending = fixture.terminalIntent();
  GoalTerminalStatusDelivery delivery(fixture.outbox);
  const auto ticket = pending.terminal_after_stop->delivery_ticket;

  require(delivery.stage(pending.terminal_intent_id + 1U, ticket) ==
              GoalTerminalStatusDelivery::StageResult::kStaged,
          "fixture could not stage the deliberately wrong intent");
  fixture.commitAfterConfirmedStop(pending);
  require(delivery.markCommitted(pending.terminal_intent_id + 1U),
          "wrong staged intent did not record commit evidence");
  fixture.writes_allowed = true;
  require(delivery.flushAndAcknowledge(fixture.coordinator) ==
              GoalTerminalStatusDelivery::FlushResult::kAcknowledgementRejected,
          "wrong terminal intent was acknowledged");
  require(fixture.outbox.delivered(ticket.statuses.front()),
          "wrong-intent test never delivered the terminal ticket");
  require(fixture.coordinator.terminalPending(),
          "wrong terminal intent cleared the coordinator transaction");
  require(delivery.stage(pending.terminal_intent_id, ticket) ==
              GoalTerminalStatusDelivery::StageResult::kConflict,
          "failed acknowledgement discarded the staged wrong intent");
}

}  // namespace

int main() {
  try {
    testStageReplayWaitsForCommitAndRetriesBeforeExactAck();
    testWrongIntentCannotClearPendingTerminal();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_goal_terminal_status_delivery: FAIL: %s\n", exc.what());
    return 1;
  }
}
