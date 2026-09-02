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

using lingtu::message::NavigationGoalState;
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
using lingtu::nav::endpoint::GoalPlanTerminalDeliveryTicket;
using lingtu::nav::endpoint::GoalReplanRuntimeCoordinator;
using lingtu::nav::endpoint::GoalReplanRuntimeFrameInput;
using lingtu::nav::endpoint::GoalReplanRuntimeInterruption;
using lingtu::nav::endpoint::GoalReplanRuntimeResult;
using lingtu::nav::endpoint::GoalTerminalStatusDelivery;
using lingtu::nav::endpoint::advanceShutdownTransaction;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopBarrier;
using lingtu::nav::endpoint::MotionStopTerminalBarrierResult;
using lingtu::nav::endpoint::MotionStopTerminalCommit;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::TerminalStopPolicy;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool sameStatus(const GoalPlanStatus &left, const GoalPlanStatus &right) {
  return left.task_id == right.task_id && left.request_id == right.request_id &&
      left.goal_epoch == right.goal_epoch && left.state == right.state &&
      left.reason == right.reason &&
      left.project_to_navigation_state == right.project_to_navigation_state;
}

bool sameTicket(const GoalPlanTerminalDeliveryTicket &left,
                const GoalPlanTerminalDeliveryTicket &right) {
  if (left.statuses.size() != right.statuses.size()) {
    return false;
  }
  for (std::size_t index = 0U; index < left.statuses.size(); ++index) {
    if (!sameStatus(left.statuses[index], right.statuses[index])) {
      return false;
    }
  }
  return true;
}

std::size_t statusCount(const std::vector<GoalPlanStatus> &statuses,
                        NavigationGoalState state, const std::string &reason) {
  std::size_t count = 0U;
  for (const auto &status : statuses) {
    if (status.state == state && status.reason == reason) {
      ++count;
    }
  }
  return count;
}

std::size_t actionCount(const std::vector<std::string> &actions,
                        const std::string &expected) {
  std::size_t count = 0U;
  for (const auto &action : actions) {
    if (action == expected) {
      ++count;
    }
  }
  return count;
}

struct EstopEndpointResult {
  GoalReplanRuntimeResult runtime_result;
  std::optional<GoalTerminalStatusDelivery::FlushResult> terminal_flush;
  std::optional<lingtu::nav::endpoint::MotionStopResult> physical_only;
};

struct Fixture {
  lingtu::nav::plan::MapIdentity map_identity{"field", 7, "map"};
  bool writes_allowed{true};
  std::vector<std::string> order;
  std::vector<GoalPlanStatus> observed;
  std::vector<GoalPlanStatus> write_attempts;
  int terminal_commit_count{0};
  bool persist_estop_ok{true};
  bool estop_latched{false};
  bool persisted_estop_latched{false};
  int clear_estop_calls{0};
  int clear_persisted_estop_calls{0};
  bool clear_outputs_ok{true};
  std::uint64_t last_output_sequence{19U};
  std::optional<std::uint64_t> sequenced_zero{20U};
  std::vector<StopConfirmationState> confirmation_results{StopConfirmationState::Confirmed};
  NavigationGoalStatusOutbox outbox;
  GoalPlanController goal_plan;
  MotionStopBarrier motion_stop;
  GoalReplanRuntimeCoordinator runtime;
  GoalTerminalStatusDelivery delivery;

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
        runtime(goal_plan, motion_stop),
        delivery(outbox) {}

  GoalPlanActions goalActions() {
    GoalPlanActions actions;
    actions.preempt_rolling = [](const std::string &) { return true; };
    actions.clear_external_inspection = [] {};
    actions.current_map_identity = [this] { return GoalPlanMapIdentityResult{map_identity, {}}; };
    actions.publish_status = [this](const GoalPlanStatus &status) {
      if (status.state == NavigationGoalState::Cancelled) {
        order.emplace_back("status_cancelled:" + status.reason);
      } else if (status.state == NavigationGoalState::Failed) {
        order.emplace_back("status_failed:" + status.reason);
      }
      if (!outbox.record(status)) {
        throw std::runtime_error("status outbox rejected fixture status");
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
    actions.clear_motion_outputs = [this](const std::string &) {
      order.emplace_back("publish_zero_and_clear_motion_outputs");
      return clear_outputs_ok;
    };
    actions.suspend_motion_outputs = [](const std::string &) { return true; };
    actions.cancel_control = [this] { order.emplace_back("cancel_control"); };
    actions.stop_control = [this] { order.emplace_back("stop_control"); };
    actions.latch_estop = [this](const std::string &reason) {
      order.emplace_back("latch_estop:" + reason);
      estop_latched = true;
    };
    actions.clear_control_estop = [this] {
      ++clear_estop_calls;
      estop_latched = false;
      return true;
    };
    actions.resume_control = [] { return true; };
    actions.cancel_inspection = [this](const std::string &reason) {
      order.emplace_back("cancel_inspection:" + reason);
    };
    actions.clear_operator_resume_required = [this] {
      order.emplace_back("clear_operator_resume_required");
    };
    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [this](const std::string &reason) {
      order.emplace_back("persist_estop:" + reason);
      if (persist_estop_ok) {
        persisted_estop_latched = true;
      }
      return persist_estop_ok;
    };
    actions.clear_persisted_estop_latch = [this] {
      ++clear_persisted_estop_calls;
      persisted_estop_latched = false;
      return true;
    };
    actions.publish_zero = [this] {
      order.emplace_back("publish_zero");
      return true;
    };
    actions.last_output_sequence = [this] {
      order.emplace_back("last_output_sequence");
      return last_output_sequence;
    };
    actions.publish_sequenced_zero = [this] {
      order.emplace_back("publish_sequenced_zero");
      return sequenced_zero;
    };
    actions.confirm_zero = [this](std::uint64_t sequence) {
      order.emplace_back("confirm_zero:" + std::to_string(sequence));
      if (confirmation_results.empty()) {
        return StopConfirmationState::Confirmed;
      }
      const StopConfirmationState result = confirmation_results.front();
      confirmation_results.erase(confirmation_results.begin());
      return result;
    };
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
    submitGoalOnly();
    for (int i = 0; i < 1000; ++i) {
      if (goal_plan.advance(GoalPlanAdvanceContext{7U, false, 2.0 + i * 0.001}).path_activated) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "goal planning did not complete");
  }

  void submitGoalOnly() {
    GoalPlanRequest request;
    request.task_id = "task-active";
    request.request_id = "goal-request";
    request.origin = GoalPlanOrigin::kExternal;
    request.source_stamp_s = 1.0;
    request.target = GoalPlanTarget{nav_kernel::Vec3{2.0, 1.0, 0.0}, 0.0};
    require(goal_plan.submit(request, admission()).accepted, "goal submission failed");
  }

  GoalTerminalStatusDelivery::FlushResult serviceTerminal(
      const GoalReplanRuntimeResult &result) {
    require(result.terminal_intent_id != 0U && result.terminal_after_stop.has_value(),
            "cancel did not create a terminal intent");
    const auto &terminal = *result.terminal_after_stop;
    const auto stage = delivery.stage(result.terminal_intent_id, terminal.delivery_ticket);
    require(stage == GoalTerminalStatusDelivery::StageResult::kStaged ||
                stage == GoalTerminalStatusDelivery::StageResult::kReplay,
            "terminal intent did not stage or replay");
    if (!delivery.isCommitted(result.terminal_intent_id)) {
      require(delivery.flushAndAcknowledge(runtime) ==
                  GoalTerminalStatusDelivery::FlushResult::kCommitPending,
              "terminal delivery flushed before active cancel stop confirmation");
      const auto committed = motion_stop.cancelPreservingGoalTerminal(terminal.commit);
      require(committed.accepted && committed.terminal_committed,
              "cancel terminal did not commit after zero confirmation");
      ++terminal_commit_count;
      require(delivery.markCommitted(result.terminal_intent_id),
              "committed cancel terminal was not marked");
    }
    return delivery.flushAndAcknowledge(runtime);
  }

  GoalTerminalStatusDelivery::FlushResult serviceTerminalUsingPolicy(
      const GoalReplanRuntimeResult &result) {
    require(result.terminal_intent_id != 0U && result.terminal_after_stop.has_value(),
            "cancel did not create a terminal intent");
    const auto &terminal = *result.terminal_after_stop;
    const auto stage = delivery.stage(result.terminal_intent_id, terminal.delivery_ticket);
    require(stage == GoalTerminalStatusDelivery::StageResult::kStaged ||
                stage == GoalTerminalStatusDelivery::StageResult::kReplay,
            "terminal intent did not stage or replay");
    if (!delivery.isCommitted(result.terminal_intent_id)) {
      require(delivery.flushAndAcknowledge(runtime) ==
                  GoalTerminalStatusDelivery::FlushResult::kCommitPending,
              "terminal delivery flushed before active cancel stop confirmation");
      MotionStopTerminalBarrierResult committed;
      switch (result.terminal_stop_policy) {
        case TerminalStopPolicy::kStop: {
          const auto stopped = motion_stop.stopPreservingGoalTerminal(terminal.commit);
          committed = MotionStopTerminalBarrierResult{stopped.accepted, stopped.reason,
                                                      stopped.accepted};
          break;
        }
        case TerminalStopPolicy::kCancel:
          committed = motion_stop.cancelPreservingGoalTerminal(terminal.commit);
          break;
        case TerminalStopPolicy::kEstop:
          committed =
              motion_stop.estopPreservingGoalTerminal("operator_request", terminal.commit);
          break;
        case TerminalStopPolicy::kShutdown:
          throw std::runtime_error(
              "shutdown terminal must use the shared shutdown transaction");
        case TerminalStopPolicy::kGenericStop: {
          const auto stopped =
              motion_stop.commitGoalTerminalAfterStop(terminal.reason, terminal.commit);
          committed = MotionStopTerminalBarrierResult{stopped.accepted, stopped.reason,
                                                      stopped.accepted};
          break;
        }
      }
      if (!committed.accepted) {
        return GoalTerminalStatusDelivery::FlushResult::kCommitPending;
      }
      require(committed.terminal_committed,
              "confirmed policy terminal did not commit after zero confirmation");
      ++terminal_commit_count;
      require(delivery.markCommitted(result.terminal_intent_id),
              "committed policy terminal was not marked");
    }
    return delivery.flushAndAcknowledge(runtime);
  }

  GoalTerminalStatusDelivery::FlushResult
  serviceReplayedTerminalAfterPhysicalStop() {
    const auto replay = runtime.replayPendingTerminal();
    require(replay.terminal_intent_id != 0U && replay.terminal_after_stop.has_value(),
            "runtime did not replay the pending terminal");
    const auto &terminal = *replay.terminal_after_stop;
    const auto stage = delivery.stage(replay.terminal_intent_id, terminal.delivery_ticket);
    require(stage == GoalTerminalStatusDelivery::StageResult::kStaged ||
                stage == GoalTerminalStatusDelivery::StageResult::kReplay,
            "replayed terminal intent did not stage or replay");
    if (!delivery.isCommitted(replay.terminal_intent_id)) {
      require(delivery.flushAndAcknowledge(runtime) ==
                  GoalTerminalStatusDelivery::FlushResult::kCommitPending,
              "replayed terminal delivery flushed before physical stop confirmation");
      const auto stopped = motion_stop.stopPreservingGoalTerminal(terminal.commit);
      if (!stopped.accepted) {
        return GoalTerminalStatusDelivery::FlushResult::kCommitPending;
      }
      require(stopped.terminal_committed,
              "physical stop did not commit the replayed terminal");
      ++terminal_commit_count;
      require(delivery.markCommitted(replay.terminal_intent_id),
              "committed replayed terminal was not marked");
    }
    return delivery.flushAndAcknowledge(runtime);
  }

  GoalTerminalStatusDelivery::FlushResult
  serviceReplayedTerminalAfterPhysicalEstop() {
    const auto replay = runtime.replayPendingTerminal();
    require(replay.terminal_intent_id != 0U && replay.terminal_after_stop.has_value(),
            "runtime did not replay the pending terminal for estop");
    const auto &terminal = *replay.terminal_after_stop;
    const auto stage = delivery.stage(replay.terminal_intent_id, terminal.delivery_ticket);
    require(stage == GoalTerminalStatusDelivery::StageResult::kStaged ||
                stage == GoalTerminalStatusDelivery::StageResult::kReplay,
            "replayed estop terminal intent did not stage or replay");
    if (!delivery.isCommitted(replay.terminal_intent_id)) {
      require(delivery.flushAndAcknowledge(runtime) ==
                  GoalTerminalStatusDelivery::FlushResult::kCommitPending,
              "replayed estop delivery flushed before physical estop confirmation");
      const auto estopped =
          motion_stop.estopPreservingGoalTerminal("operator_request", terminal.commit);
      if (!estopped.accepted) {
        return GoalTerminalStatusDelivery::FlushResult::kCommitPending;
      }
      require(estopped.terminal_committed,
              "physical estop did not commit the replayed terminal");
      ++terminal_commit_count;
      require(delivery.markCommitted(replay.terminal_intent_id),
              "committed estop-replayed terminal was not marked");
    }
    return delivery.flushAndAcknowledge(runtime);
  }

  EstopEndpointResult handleEstop(double steady_now_s,
                                  const std::string &audit_reason = "operator_request") {
    EstopEndpointResult result;
    result.runtime_result =
        runtime.interrupt(GoalReplanRuntimeInterruption::kEstop, steady_now_s);
    if (result.runtime_result.terminal_after_stop.has_value()) {
      result.terminal_flush = serviceTerminalUsingPolicy(result.runtime_result);
    } else {
      result.physical_only = motion_stop.estopWithoutTerminalCommit(audit_reason);
    }
    return result;
  }

  GoalReplanRuntimeFrameInput frame(double steady_now_s) const {
    GoalReplanRuntimeFrameInput input;
    input.steady_now_s = steady_now_s;
    input.wall_now_s = steady_now_s;
    input.fresh_admission = admission();
    return input;
  }
};

void testActiveCancelAckIsNotTerminalWriterCompletion() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");

  fixture.writes_allowed = false;
  fixture.order.clear();
  const auto cancel_result =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kCancel, 10.0);
  require(cancel_result.handled && cancel_result.interrupted && cancel_result.reason == "cancelled",
          "active cancel did not produce the runtime cancel interruption");
  require(cancel_result.terminal_after_stop.has_value(),
          "active cancel did not create a terminal intent");
  require(cancel_result.terminal_intent_id != 0U,
          "active cancel did not create a non-zero terminal intent");
  const auto &ticket = cancel_result.terminal_after_stop->delivery_ticket;
  require(ticket.statuses.size() == 1U &&
              ticket.statuses.front().task_id == "task-active" &&
              ticket.statuses.front().request_id == "goal-request" &&
              ticket.statuses.front().state == NavigationGoalState::Cancelled &&
              ticket.statuses.front().reason == "cancelled",
          "active cancel terminal ticket lost identity or Cancelled reason");
  require(fixture.runtime.terminalPending(), "cancel terminal was not retained before delivery");

  const auto first_flush = fixture.serviceTerminal(cancel_result);
  require(first_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not leave active cancel delivery pending");
  require(fixture.order == std::vector<std::string>{"cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_cancelled",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_cancelled:cancelled"},
          "active cancel did not zero/confirm before terminal commit");
  require(fixture.terminal_commit_count == 1,
          "active cancel terminal was not committed exactly once");
  require(fixture.runtime.terminalPending(),
          "writer failure acknowledged GoalRuntime terminal");

  fixture.writes_allowed = true;
  const auto second_flush = fixture.serviceTerminal(cancel_result);
  require(second_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "writer retry did not acknowledge active cancel terminal");
  require(fixture.terminal_commit_count == 1,
          "writer retry repeated the stop-confirmed terminal commit");
  require(!fixture.runtime.terminalPending(),
          "successful terminal delivery did not ACK GoalRuntime");
}

void testActiveCancelRetryKeepsCancelStopPolicyAfterUnconfirmedZero() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  fixture.confirmation_results = {StopConfirmationState::TimedOut,
                                  StopConfirmationState::Confirmed};

  fixture.order.clear();
  const auto cancel_result =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kCancel, 10.0);
  require(cancel_result.terminal_after_stop.has_value(),
          "active cancel did not create a terminal intent");
  require(cancel_result.terminal_stop_policy == TerminalStopPolicy::kCancel,
          "active cancel terminal did not retain the cancel stop policy");

  const auto failed_commit = fixture.serviceTerminalUsingPolicy(cancel_result);
  require(failed_commit == GoalTerminalStatusDelivery::FlushResult::kCommitPending,
          "unconfirmed zero should leave cancel terminal commit pending");
  require(fixture.terminal_commit_count == 0,
          "unconfirmed zero committed the cancel terminal");
  require(fixture.runtime.terminalPending(),
          "unconfirmed zero cleared the cancel terminal intent");
  require(fixture.order == std::vector<std::string>{"cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_cancelled",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19"},
          "first active cancel attempt did not stop through the cancel path");

  fixture.order.clear();
  const auto retry_result = fixture.runtime.advancePlanningCycle(fixture.frame(10.2));
  require(retry_result.terminal_intent_id == cancel_result.terminal_intent_id,
          "retry surfaced a different terminal intent");
  require(retry_result.terminal_stop_policy == TerminalStopPolicy::kCancel,
          "retry of active cancel terminal lost the cancel stop policy");
  const auto retry_flush = fixture.serviceTerminalUsingPolicy(retry_result);
  require(retry_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "confirmed retry did not deliver and acknowledge cancel terminal");
  require(fixture.terminal_commit_count == 1,
          "confirmed retry did not commit cancel terminal exactly once");
  require(fixture.order == std::vector<std::string>{"cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_cancelled",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_cancelled:cancelled"},
          "retry did not use the active cancel stop policy");
  require(!fixture.runtime.terminalPending(),
          "acknowledged cancel terminal remained pending");
}

void testActiveStopAckIsNotTerminalWriterCompletion() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");

  fixture.writes_allowed = false;
  fixture.order.clear();
  const auto stop_result =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kStop, 10.0);
  require(stop_result.handled && stop_result.interrupted && stop_result.reason == "stopped",
          "typed stop did not acknowledge the synchronous stop fact");
  require(stop_result.terminal_after_stop.has_value(),
          "active stop did not create a persistent terminal intent");
  require(stop_result.terminal_intent_id != 0U,
          "active stop did not create a non-zero terminal intent");
  require(stop_result.terminal_stop_policy == TerminalStopPolicy::kStop,
          "active stop terminal did not retain the stop policy");
  const auto &ticket = stop_result.terminal_after_stop->delivery_ticket;
  require(ticket.statuses.size() == 1U &&
              ticket.statuses.front().task_id == "task-active" &&
              ticket.statuses.front().request_id == "goal-request" &&
              ticket.statuses.front().state == NavigationGoalState::Cancelled &&
              ticket.statuses.front().reason == "stopped",
          "active stop terminal ticket lost identity or Cancelled stopped reason");
  require(fixture.runtime.terminalPending(), "stop terminal was not retained before delivery");

  const auto first_flush = fixture.serviceTerminalUsingPolicy(stop_result);
  require(first_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not leave active stop delivery pending");
  require(fixture.order == std::vector<std::string>{"stop_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_stopped",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_cancelled:stopped"},
          "active stop did not zero/confirm before terminal commit");
  require(fixture.terminal_commit_count == 1,
          "active stop terminal was not committed exactly once");
  require(fixture.runtime.terminalPending(),
          "writer failure acknowledged GoalRuntime stop terminal");

  fixture.writes_allowed = true;
  const auto second_flush = fixture.serviceTerminalUsingPolicy(stop_result);
  require(second_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "writer retry did not acknowledge active stop terminal");
  require(fixture.terminal_commit_count == 1,
          "writer retry repeated the stop-confirmed terminal commit");
  require(!fixture.runtime.terminalPending(),
          "successful stop terminal delivery did not ACK GoalRuntime");
}

void testActiveStopRetryKeepsStopPolicyAfterUnconfirmedZero() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  fixture.confirmation_results = {StopConfirmationState::TimedOut,
                                  StopConfirmationState::Confirmed};

  fixture.order.clear();
  const auto stop_result =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kStop, 10.0);
  require(stop_result.handled && stop_result.interrupted && stop_result.reason == "stopped",
          "typed stop did not acknowledge the synchronous stop fact");
  require(stop_result.terminal_after_stop.has_value(),
          "active stop did not create a terminal intent for retry");
  require(stop_result.terminal_stop_policy == TerminalStopPolicy::kStop,
          "active stop terminal did not retain the stop policy");

  const auto failed_commit = fixture.serviceTerminalUsingPolicy(stop_result);
  require(failed_commit == GoalTerminalStatusDelivery::FlushResult::kCommitPending,
          "unconfirmed zero should leave stop terminal commit pending");
  require(fixture.terminal_commit_count == 0,
          "unconfirmed zero committed the stop terminal");
  require(fixture.runtime.terminalPending(),
          "unconfirmed zero cleared the stop terminal intent");
  require(fixture.order == std::vector<std::string>{"stop_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_stopped",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19"},
          "first active stop attempt did not stop through the stop path");

  fixture.order.clear();
  const auto retry_result = fixture.runtime.advancePlanningCycle(fixture.frame(10.2));
  require(retry_result.terminal_intent_id == stop_result.terminal_intent_id,
          "retry surfaced a different stop terminal intent");
  require(retry_result.terminal_stop_policy == TerminalStopPolicy::kStop,
          "retry of active stop terminal lost the stop policy");
  const auto retry_flush = fixture.serviceTerminalUsingPolicy(retry_result);
  require(retry_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "confirmed retry did not deliver and acknowledge stop terminal");
  require(fixture.terminal_commit_count == 1,
          "confirmed retry did not commit stop terminal exactly once");
  require(fixture.order == std::vector<std::string>{"stop_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_stopped",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_cancelled:stopped"},
          "retry did not use the active stop path");
  require(!fixture.runtime.terminalPending(),
          "acknowledged stop terminal remained pending");
}

void testActiveEstopAckIsNotTerminalWriterCompletion() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");

  fixture.writes_allowed = false;
  fixture.order.clear();
  const auto estop = fixture.handleEstop(10.0);
  const auto &estop_result = estop.runtime_result;
  require(estop_result.handled && estop_result.interrupted &&
              estop_result.reason == "estop_latched",
          "typed estop did not acknowledge the synchronous latched-estop fact");
  require(estop_result.terminal_after_stop.has_value(),
          "active estop did not create a persistent terminal intent");
  require(estop_result.terminal_intent_id != 0U,
          "active estop did not create a non-zero terminal intent");
  require(estop_result.terminal_stop_policy == TerminalStopPolicy::kEstop,
          "active estop terminal did not retain the estop policy");
  const auto &ticket = estop_result.terminal_after_stop->delivery_ticket;
  require(ticket.statuses.size() == 1U &&
              ticket.statuses.front().task_id == "task-active" &&
              ticket.statuses.front().request_id == "goal-request" &&
              ticket.statuses.front().state == NavigationGoalState::Cancelled &&
              ticket.statuses.front().reason == "estop_latched",
          "active estop terminal ticket lost identity or Cancelled/estop_latched reason");
  require(fixture.runtime.terminalPending(), "estop terminal was not retained before delivery");

  require(estop.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not leave active estop delivery pending");
  require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                    "persist_estop:operator_request",
                                                    "cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:estop_latched",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_cancelled:estop_latched"},
          "active estop did not latch/persist/zero/confirm before terminal commit");
  require(fixture.terminal_commit_count == 1,
          "active estop terminal was not committed exactly once");
  require(fixture.runtime.terminalPending(),
          "writer failure acknowledged GoalRuntime estop terminal");

  fixture.writes_allowed = true;
  const auto second_flush = fixture.serviceTerminalUsingPolicy(estop_result);
  require(second_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "writer retry did not acknowledge active estop terminal");
  require(fixture.terminal_commit_count == 1,
          "writer retry repeated the stop-confirmed estop terminal commit");
  require(!fixture.runtime.terminalPending(),
          "successful estop terminal delivery did not ACK GoalRuntime");
}

void testActiveEstopRetryKeepsEstopPolicyAfterPersistFailure() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  fixture.persist_estop_ok = false;

  fixture.order.clear();
  const auto estop = fixture.handleEstop(10.0);
  const auto &estop_result = estop.runtime_result;
  require(estop_result.terminal_after_stop.has_value(),
          "active estop did not create a terminal intent for retry");
  require(estop_result.terminal_stop_policy == TerminalStopPolicy::kEstop,
          "active estop terminal did not retain the estop policy");

  require(estop.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kCommitPending,
          "persist failure should leave estop terminal commit pending");
  require(fixture.terminal_commit_count == 0,
          "persist-failed estop committed the terminal");
  require(fixture.runtime.terminalPending(),
          "persist-failed estop cleared the terminal intent");
  require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                    "persist_estop:operator_request",
                                                    "cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:estop_latched",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19"},
          "persist-failed estop did not still zero and confirm");

  fixture.persist_estop_ok = true;
  fixture.order.clear();
  const auto retry_result = fixture.runtime.advancePlanningCycle(fixture.frame(10.2));
  require(retry_result.terminal_intent_id == estop_result.terminal_intent_id,
          "retry surfaced a different estop terminal intent");
  require(retry_result.terminal_stop_policy == TerminalStopPolicy::kEstop,
          "retry of active estop terminal lost the estop policy");
  const auto retry_flush = fixture.serviceTerminalUsingPolicy(retry_result);
  require(retry_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "confirmed retry did not deliver and acknowledge estop terminal");
  require(fixture.terminal_commit_count == 1,
          "confirmed retry did not commit estop terminal exactly once");
  require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                    "persist_estop:operator_request",
                                                    "cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:estop_latched",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_cancelled:estop_latched"},
          "retry did not use the active estop path");
  require(!fixture.runtime.terminalPending(),
          "acknowledged estop terminal remained pending");
}

void testDuplicateAlreadyLatchedEstopCompletesPersistWithoutNewTerminal() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  fixture.persist_estop_ok = false;

  fixture.order.clear();
  const auto first_estop = fixture.handleEstop(10.0);
  const auto &first = first_estop.runtime_result;
  require(first.terminal_after_stop.has_value() &&
              first.terminal_stop_policy == TerminalStopPolicy::kEstop,
          "first estop did not create the estop terminal intent");
  require(first_estop.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kCommitPending,
          "persist-failed estop should remain commit-pending");
  require(fixture.terminal_commit_count == 0,
          "persist-failed estop committed terminal before persist succeeded");
  require(fixture.runtime.terminalPending(),
          "persist-failed estop did not retain the pending terminal");

  fixture.persist_estop_ok = true;
  fixture.order.clear();
  const auto duplicate_estop = fixture.handleEstop(10.1);
  const auto &duplicate = duplicate_estop.runtime_result;
  require(duplicate.terminal_intent_id == first.terminal_intent_id,
          "duplicate already-latched estop created a new terminal intent");
  require(duplicate.terminal_stop_policy == first.terminal_stop_policy,
          "duplicate already-latched estop changed the terminal policy");
  require(duplicate.terminal_after_stop.has_value() &&
              duplicate.terminal_after_stop->delivery_ticket.statuses.size() ==
                  first.terminal_after_stop->delivery_ticket.statuses.size() &&
              duplicate.terminal_after_stop->delivery_ticket.statuses.front().reason ==
                  first.terminal_after_stop->delivery_ticket.statuses.front().reason,
          "duplicate already-latched estop changed the terminal ticket");

  require(duplicate_estop.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "duplicate already-latched estop did not complete the missing persist/terminal delivery");
  require(fixture.terminal_commit_count == 1,
          "duplicate already-latched estop did not commit the terminal exactly once");
  require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                    "persist_estop:operator_request",
                                                    "cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:estop_latched",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_cancelled:estop_latched"},
          "duplicate already-latched estop did not fill missing physical steps before commit");
  require(!fixture.runtime.terminalPending(),
          "duplicate already-latched estop left the terminal pending after successful retry");
}

void testDuplicateAlreadyLatchedEstopReplaysWriterPendingIntentWithoutRepeatCommit() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");

  fixture.writes_allowed = false;
  fixture.order.clear();
  const auto first_estop = fixture.handleEstop(10.0);
  const auto &first = first_estop.runtime_result;
  require(first.terminal_after_stop.has_value(),
          "first estop did not create a terminal intent");
  require(first_estop.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not leave estop terminal pending");
  require(fixture.terminal_commit_count == 1,
          "first estop did not commit once before writer retry");
  require(fixture.runtime.terminalPending(), "writer-pending estop terminal was cleared early");

  fixture.order.clear();
  const auto duplicate_estop = fixture.handleEstop(10.1);
  const auto &duplicate = duplicate_estop.runtime_result;
  require(duplicate.terminal_intent_id == first.terminal_intent_id,
          "duplicate writer-pending estop created a new terminal intent");
  require(duplicate.terminal_stop_policy == first.terminal_stop_policy,
          "duplicate writer-pending estop changed the terminal policy");
  require(duplicate_estop.terminal_flush ==
              GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "duplicate writer-pending estop should replay the same pending writer intent");
  require(fixture.terminal_commit_count == 1,
          "duplicate writer-pending estop repeated the terminal commit");
  require(fixture.order == std::vector<std::string>{},
          "duplicate writer-pending estop repeated physical stop side effects");
  require(fixture.runtime.terminalPending(), "duplicate writer-pending estop cleared intent early");

  fixture.writes_allowed = true;
  const auto final_flush = fixture.serviceTerminalUsingPolicy(duplicate);
  require(final_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "writer retry did not acknowledge duplicate estop terminal");
  require(fixture.terminal_commit_count == 1,
          "writer retry after duplicate estop repeated terminal commit");
  require(!fixture.runtime.terminalPending(),
          "acknowledged duplicate estop terminal remained pending");
}

void testActiveEstopRetryKeepsEstopPolicyAfterZeroUnavailableAndTimeout() {
  {
    Fixture fixture;
    fixture.activateGoal();
    fixture.writes_allowed = true;
    require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
    fixture.last_output_sequence = 0U;

    fixture.order.clear();
    const auto estop = fixture.handleEstop(10.0);
    const auto &estop_result = estop.runtime_result;
    require(estop.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kCommitPending,
            "zero-unavailable estop should leave terminal commit pending");
    require(fixture.terminal_commit_count == 0,
            "zero-unavailable estop committed the terminal");
    require(fixture.runtime.terminalPending(),
            "zero-unavailable estop cleared the terminal intent");
    require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                      "persist_estop:operator_request",
                                                      "cancel_control",
                                                      "clear_operator_resume_required",
                                                      "cancel_inspection:estop_latched",
                                                      "publish_zero_and_clear_motion_outputs",
                                                      "last_output_sequence"},
            "zero-unavailable estop did not preserve the expected physical attempt");

    fixture.last_output_sequence = 19U;
    fixture.order.clear();
    const auto retry = fixture.runtime.advancePlanningCycle(fixture.frame(10.2));
    require(retry.terminal_intent_id == estop_result.terminal_intent_id,
            "zero-unavailable retry changed estop terminal intent");
    require(retry.terminal_stop_policy == TerminalStopPolicy::kEstop,
            "zero-unavailable retry lost estop policy");
    const auto retry_flush = fixture.serviceTerminalUsingPolicy(retry);
    require(retry_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
            "zero-unavailable retry did not acknowledge after confirmed zero");
    require(fixture.terminal_commit_count == 1,
            "zero-unavailable retry did not commit exactly once");
  }

  {
    Fixture fixture;
    fixture.activateGoal();
    fixture.writes_allowed = true;
    require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
    fixture.confirmation_results = {StopConfirmationState::TimedOut,
                                    StopConfirmationState::Confirmed};

    fixture.order.clear();
    const auto estop = fixture.handleEstop(20.0);
    const auto &estop_result = estop.runtime_result;
    require(estop.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kCommitPending,
            "timed-out estop should leave terminal commit pending");
    require(fixture.terminal_commit_count == 0, "timed-out estop committed the terminal");
    require(fixture.runtime.terminalPending(), "timed-out estop cleared the terminal intent");
    require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                      "persist_estop:operator_request",
                                                      "cancel_control",
                                                      "clear_operator_resume_required",
                                                      "cancel_inspection:estop_latched",
                                                      "publish_zero_and_clear_motion_outputs",
                                                      "last_output_sequence",
                                                      "confirm_zero:19"},
            "timed-out estop did not attempt zero confirmation before failing");

    fixture.order.clear();
    const auto retry = fixture.runtime.advancePlanningCycle(fixture.frame(20.2));
    require(retry.terminal_intent_id == estop_result.terminal_intent_id,
            "timed-out retry changed estop terminal intent");
    require(retry.terminal_stop_policy == TerminalStopPolicy::kEstop,
            "timed-out retry lost estop policy");
    const auto retry_flush = fixture.serviceTerminalUsingPolicy(retry);
    require(retry_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
            "timed-out retry did not acknowledge after confirmed zero");
    require(fixture.terminal_commit_count == 1,
            "timed-out retry did not commit exactly once");
  }
}

void testStopWithoutActiveGoalDoesNotCreateExecutionTerminal() {
  Fixture fixture;
  fixture.writes_allowed = true;

  const auto stop_result =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kStop, 10.0);
  const auto physical_stop = fixture.motion_stop.stopWithoutTerminalCommit();
  require(stop_result.handled && stop_result.interrupted && stop_result.reason == "stopped",
          "stop without active goal did not acknowledge the synchronous stop fact");
  require(physical_stop.accepted && physical_stop.reason == "stopped",
          "stop without active goal did not execute physical-only stop");
  require(stop_result.terminal_intent_id == 0U,
          "stop without active goal created a terminal intent");
  require(!stop_result.terminal_after_stop.has_value(),
          "stop without active goal created a fake execution terminal");
  require(!fixture.runtime.terminalPending(),
          "stop without active goal left a runtime terminal pending");
  require(fixture.outbox.flush() == 0U,
          "stop without active goal published a fake execution status");
  require(fixture.order == std::vector<std::string>{"stop_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_stopped",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19"},
          "stop without active goal did not use the physical-only stop path");
}

void testEstopWithoutActiveGoalDoesNotCreateExecutionTerminal() {
  Fixture fixture;
  fixture.writes_allowed = true;

  const auto estop = fixture.handleEstop(10.0);
  const auto &estop_result = estop.runtime_result;
  require(estop_result.handled && estop_result.interrupted &&
              estop_result.reason == "estop_latched",
          "estop without active goal did not acknowledge the synchronous estop fact");
  require(estop.physical_only.has_value() && estop.physical_only->accepted &&
              estop.physical_only->reason == "estop_latched",
          "estop without active goal did not execute physical-only estop");
  require(estop_result.terminal_intent_id == 0U,
          "estop without active goal created a terminal intent");
  require(!estop_result.terminal_after_stop.has_value(),
          "estop without active goal created a fake execution terminal");
  require(!fixture.runtime.terminalPending(),
          "estop without active goal left a runtime terminal pending");
  require(fixture.outbox.flush() == 0U,
          "estop without active goal published a fake execution status");
  require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                    "persist_estop:operator_request",
                                                    "cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:estop_latched",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19"},
          "estop without active goal did not use the physical-only estop path");
}

void testSafetyStopServicesReplayedNonStopTerminalFromRuntimeOwner() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");

  std::uint64_t intent_id = 0U;
  GoalPlanTerminalDeliveryTicket ticket;
  {
    const auto map_drift =
        fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 10.0);
    require(map_drift.terminal_after_stop.has_value(),
            "map drift did not create a pending terminal");
    intent_id = map_drift.terminal_intent_id;
    ticket = map_drift.terminal_after_stop->delivery_ticket;
    require(intent_id != 0U && ticket.statuses.size() == 1U &&
                ticket.statuses.front().task_id == "task-active" &&
                ticket.statuses.front().request_id == "goal-request" &&
                ticket.statuses.front().state == NavigationGoalState::Failed &&
                ticket.statuses.front().reason == "map_drift",
            "non-stop pending terminal did not preserve original state or reason");
  }

  fixture.writes_allowed = false;
  fixture.order.clear();
  const auto first_flush = fixture.serviceReplayedTerminalAfterPhysicalStop();
  require(first_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not leave replayed terminal pending");
  require(fixture.terminal_commit_count == 1,
          "replayed terminal was not committed exactly once after physical stop");
  require(fixture.runtime.terminalPending(),
          "writer failure acknowledged replayed terminal");
  require(fixture.order == std::vector<std::string>{"stop_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:navigation_stopped",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_failed:map_drift"},
          "safety stop did not use physical stop-preserving replay path");

  const auto replay = fixture.runtime.replayPendingTerminal();
  require(replay.terminal_intent_id == intent_id && replay.terminal_after_stop.has_value(),
          "retry changed the replayed terminal intent");
  require(replay.terminal_after_stop->delivery_ticket.statuses.size() == ticket.statuses.size() &&
              replay.terminal_after_stop->delivery_ticket.statuses.front().state ==
                  ticket.statuses.front().state &&
              replay.terminal_after_stop->delivery_ticket.statuses.front().reason ==
                  ticket.statuses.front().reason,
          "retry changed the replayed terminal state or reason");

  fixture.writes_allowed = true;
  const auto second_flush = fixture.serviceReplayedTerminalAfterPhysicalStop();
  require(second_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "writer retry did not acknowledge replayed terminal");
  require(fixture.terminal_commit_count == 1,
          "writer retry repeated replayed terminal commit");
  require(!fixture.runtime.terminalPending(),
          "acknowledged replayed terminal remained pending");
}

void testEstopPreservesExistingPendingTerminalFromRuntimeOwner() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");

  const auto map_drift =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 10.0);
  require(map_drift.terminal_after_stop.has_value(),
          "map drift did not create a pending terminal before estop");
  const std::uint64_t intent_id = map_drift.terminal_intent_id;
  const auto ticket = map_drift.terminal_after_stop->delivery_ticket;

  fixture.writes_allowed = false;
  fixture.order.clear();
  const auto first_flush = fixture.serviceReplayedTerminalAfterPhysicalEstop();
  require(first_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not leave estop-replayed terminal pending");
  require(fixture.terminal_commit_count == 1,
          "estop-replayed terminal was not committed exactly once");
  require(fixture.runtime.terminalPending(),
          "writer failure acknowledged estop-replayed terminal");
  require(fixture.order == std::vector<std::string>{"latch_estop:operator_request",
                                                    "persist_estop:operator_request",
                                                    "cancel_control",
                                                    "clear_operator_resume_required",
                                                    "cancel_inspection:estop_latched",
                                                    "publish_zero_and_clear_motion_outputs",
                                                    "last_output_sequence", "confirm_zero:19",
                                                    "status_failed:map_drift"},
          "estop did not preserve the existing pending terminal state and reason");

  const auto replay = fixture.runtime.replayPendingTerminal();
  require(replay.terminal_intent_id == intent_id && replay.terminal_after_stop.has_value(),
          "retry changed the estop-preserved terminal intent");
  require(replay.terminal_after_stop->delivery_ticket.statuses.size() == ticket.statuses.size() &&
              replay.terminal_after_stop->delivery_ticket.statuses.front().state ==
                  ticket.statuses.front().state &&
              replay.terminal_after_stop->delivery_ticket.statuses.front().reason ==
                  ticket.statuses.front().reason,
          "retry changed the estop-preserved terminal state or reason");

  fixture.writes_allowed = true;
  const auto second_flush = fixture.serviceReplayedTerminalAfterPhysicalEstop();
  require(second_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "writer retry did not acknowledge estop-preserved terminal");
  require(fixture.terminal_commit_count == 1,
          "writer retry repeated estop-preserved terminal commit");
  require(!fixture.runtime.terminalPending(),
          "acknowledged estop-preserved terminal remained pending");
}

void testStopDuringInitialPlanningOnlyDoesNotCreateExecutionTerminal() {
  Fixture fixture;
  fixture.writes_allowed = true;
  fixture.submitGoalOnly();
  require(fixture.goal_plan.snapshot().busy, "fixture did not remain in initial planning");
  require(fixture.goal_plan.snapshot().active_task_id.empty(),
          "fixture unexpectedly activated the goal before stop");
  require(fixture.outbox.flush() == 1U, "initial planning status was not delivered");

  fixture.writes_allowed = false;
  const auto stop_result =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kStop, 10.0);
  require(stop_result.handled && stop_result.interrupted && stop_result.reason == "stopped",
          "initial-planning stop did not acknowledge the synchronous stop fact");
  require(stop_result.terminal_intent_id != 0U,
          "initial-planning stop did not create a planning terminal intent");
  require(stop_result.terminal_after_stop.has_value(),
          "initial-planning stop did not create a planning terminal ticket");
  require(stop_result.terminal_stop_policy == TerminalStopPolicy::kStop,
          "initial-planning stop terminal did not retain the stop policy");
  const auto &ticket = stop_result.terminal_after_stop->delivery_ticket;
  require(ticket.statuses.size() == 1U &&
              ticket.statuses.front().task_id == "task-active" &&
              ticket.statuses.front().request_id == "goal-request" &&
              ticket.statuses.front().state == NavigationGoalState::Cancelled &&
              ticket.statuses.front().reason == "stopped" &&
              !ticket.statuses.front().project_to_navigation_state,
          "initial-planning stop did not create a planning-scoped Cancelled/stopped status");
  require(fixture.runtime.terminalPending(),
          "initial-planning stop terminal was not retained before delivery");

  const auto first_flush = fixture.serviceTerminalUsingPolicy(stop_result);
  require(first_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending,
          "writer failure did not leave initial-planning stop delivery pending");
  require(fixture.terminal_commit_count == 1,
          "initial-planning stop terminal was not committed exactly once");
  require(fixture.runtime.terminalPending(),
          "writer failure acknowledged initial-planning stop terminal");

  const auto retry_result = fixture.runtime.advancePlanningCycle(fixture.frame(10.2));
  require(retry_result.terminal_intent_id == stop_result.terminal_intent_id,
          "retry surfaced a different initial-planning stop terminal intent");
  require(retry_result.terminal_after_stop.has_value(),
          "retry did not resurface the initial-planning stop ticket");
  const auto &retry_ticket = retry_result.terminal_after_stop->delivery_ticket;
  require(retry_ticket.statuses.size() == ticket.statuses.size() &&
              retry_ticket.statuses.front().task_id == ticket.statuses.front().task_id &&
              retry_ticket.statuses.front().request_id == ticket.statuses.front().request_id &&
              retry_ticket.statuses.front().state == ticket.statuses.front().state &&
              retry_ticket.statuses.front().reason == ticket.statuses.front().reason &&
              retry_ticket.statuses.front().project_to_navigation_state ==
                  ticket.statuses.front().project_to_navigation_state,
          "retry changed the initial-planning stop delivery ticket");
  fixture.writes_allowed = true;
  const auto second_flush = fixture.serviceTerminalUsingPolicy(retry_result);
  require(second_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged,
          "writer success did not acknowledge initial-planning stop terminal");
  require(fixture.terminal_commit_count == 1,
          "initial-planning stop retry repeated the terminal commit");
  require(!fixture.runtime.terminalPending(),
          "acknowledged initial-planning stop terminal remained pending");
}

void testEstopDuringInitialPlanningOnlyDoesNotProjectExecutionTerminal() {
  Fixture fixture;
  fixture.writes_allowed = true;
  fixture.submitGoalOnly();
  require(fixture.goal_plan.snapshot().busy, "fixture did not remain in initial planning");
  require(fixture.goal_plan.snapshot().active_task_id.empty(),
          "fixture unexpectedly activated the goal before estop");
  require(fixture.outbox.flush() == 1U, "initial planning status was not delivered");

  fixture.writes_allowed = false;
  const auto estop = fixture.handleEstop(10.0);
  const auto &estop_result = estop.runtime_result;
  require(estop_result.handled && estop_result.interrupted &&
              estop_result.reason == "estop_latched",
          "initial-planning estop did not acknowledge the synchronous estop fact");
  require(estop_result.terminal_intent_id != 0U,
          "initial-planning estop did not create a planning terminal intent");
  require(estop_result.terminal_after_stop.has_value(),
          "initial-planning estop did not create a planning terminal ticket");
  require(estop_result.terminal_stop_policy == TerminalStopPolicy::kEstop,
          "initial-planning estop terminal did not retain the estop policy");
  const auto &ticket = estop_result.terminal_after_stop->delivery_ticket;
  require(ticket.statuses.size() == 1U &&
              ticket.statuses.front().task_id == "task-active" &&
              ticket.statuses.front().request_id == "goal-request" &&
              ticket.statuses.front().state == NavigationGoalState::Cancelled &&
              ticket.statuses.front().reason == "estop_latched" &&
              !ticket.statuses.front().project_to_navigation_state,
          "initial-planning estop did not create a planning-scoped Cancelled/estop_latched status");
  require(fixture.runtime.terminalPending(),
          "initial-planning estop terminal was not retained before delivery");
}

void testActiveShutdownWaitsForWriterAndReplaysExactTicketOnce() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");

  fixture.writes_allowed = false;
  fixture.order.clear();
  const auto first =
      advanceShutdownTransaction(fixture.runtime, fixture.motion_stop, fixture.delivery, 30.0);
  require(first.runtime_result.terminal_intent_id != 0U &&
              first.runtime_result.terminal_after_stop.has_value() &&
              first.runtime_result.terminal_stop_policy == TerminalStopPolicy::kShutdown,
          "active shutdown did not create a durable shutdown terminal intent");
  const auto first_ticket = first.runtime_result.terminal_after_stop->delivery_ticket;
  require(first_ticket.statuses.size() == 1U &&
              first_ticket.statuses.front().state == NavigationGoalState::Cancelled &&
              first_ticket.statuses.front().reason == "navd_shutdown" &&
              first_ticket.statuses.front().project_to_navigation_state,
          "active shutdown lost Cancelled/navd_shutdown execution identity");
  require(first.stop_confirmed && first.terminal_required &&
              first.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kDeliveryPending &&
              !first.delivery_acknowledged && !first.decision.allow_exit,
          "writer-pending shutdown was allowed to exit");
  require(statusCount(fixture.observed, NavigationGoalState::Cancelled, "navd_shutdown") == 1U &&
              fixture.runtime.terminalPending(),
          "writer-pending shutdown did not retain the committed terminal exactly once");
  require(fixture.order ==
              std::vector<std::string>{"stop_control", "cancel_inspection:navd_shutdown",
                                       "publish_zero_and_clear_motion_outputs",
                                       "publish_sequenced_zero", "confirm_zero:20",
                                       "status_cancelled:navd_shutdown"},
          "active shutdown did not stop/confirm before terminal commit");

  fixture.order.clear();
  const auto duplicate =
      advanceShutdownTransaction(fixture.runtime, fixture.motion_stop, fixture.delivery, 30.1);
  require(duplicate.runtime_result.terminal_intent_id ==
              first.runtime_result.terminal_intent_id &&
              duplicate.runtime_result.terminal_stop_policy == TerminalStopPolicy::kShutdown &&
              duplicate.runtime_result.terminal_after_stop.has_value() &&
              sameTicket(duplicate.runtime_result.terminal_after_stop->delivery_ticket,
                         first_ticket),
          "duplicate shutdown changed the pending terminal intent, policy, or ticket");
  require(duplicate.terminal_flush ==
              GoalTerminalStatusDelivery::FlushResult::kDeliveryPending &&
              !duplicate.decision.allow_exit &&
              statusCount(fixture.observed, NavigationGoalState::Cancelled, "navd_shutdown") ==
                  1U &&
              fixture.order.empty(),
          "duplicate writer-pending shutdown repeated physical stop or terminal commit");

  fixture.writes_allowed = true;
  const auto delivered =
      advanceShutdownTransaction(fixture.runtime, fixture.motion_stop, fixture.delivery, 30.2);
  require(delivered.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged &&
              delivered.delivery_acknowledged && delivered.decision.allow_exit &&
              delivered.reason == "shutdown_complete" &&
              statusCount(fixture.observed, NavigationGoalState::Cancelled, "navd_shutdown") ==
                  1U &&
              !fixture.runtime.terminalPending(),
          "shutdown did not wait for exact terminal delivery and runtime acknowledgement");
}

void testPermanentWriterFailureRemainsFailClosedAcrossThreeShutdownRounds() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  fixture.write_attempts.clear();
  fixture.writes_allowed = false;
  fixture.order.clear();

  std::uint64_t intent_id = 0U;
  std::optional<GoalPlanTerminalDeliveryTicket> ticket;
  for (int round = 0; round < 3; ++round) {
    const auto pending = advanceShutdownTransaction(
        fixture.runtime, fixture.motion_stop, fixture.delivery, 31.0 + round * 0.1);
    require(pending.runtime_result.terminal_intent_id != 0U &&
                pending.runtime_result.terminal_after_stop.has_value() &&
                pending.runtime_result.terminal_stop_policy == TerminalStopPolicy::kShutdown,
            "permanent writer failure lost the shutdown terminal identity");
    if (!ticket.has_value()) {
      intent_id = pending.runtime_result.terminal_intent_id;
      ticket = pending.runtime_result.terminal_after_stop->delivery_ticket;
    } else {
      require(pending.runtime_result.terminal_intent_id == intent_id &&
                  sameTicket(pending.runtime_result.terminal_after_stop->delivery_ticket, *ticket),
              "permanent writer failure changed the exact shutdown intent or ticket");
    }
    require(pending.stop_confirmed && pending.terminal_required &&
                pending.terminal_flush ==
                    GoalTerminalStatusDelivery::FlushResult::kDeliveryPending &&
                !pending.delivery_acknowledged && !pending.decision.allow_exit &&
                pending.reason == "shutdown_terminal_delivery_pending" &&
                fixture.runtime.terminalPending(),
            "permanent writer failure did not remain observably fail-closed");
    require(fixture.motion_stop.keepZeroFresh(),
            "shutdown-pending writer retry did not refresh the zero command");
  }

  require(statusCount(fixture.observed, NavigationGoalState::Cancelled, "navd_shutdown") == 1U,
          "permanent writer failure repeated the terminal commit");
  require(actionCount(fixture.order, "stop_control") == 1U &&
              actionCount(fixture.order, "publish_sequenced_zero") == 1U &&
              actionCount(fixture.order, "confirm_zero:20") == 1U,
          "permanent writer failure repeated the confirmed physical shutdown");
  require(actionCount(fixture.order, "publish_zero") == 3U,
          "permanent writer failure did not keep zero fresh on every pending round");
  require(fixture.write_attempts.size() == 3U,
          "permanent writer failure did not retry exact delivery on every round");
  for (const auto &attempt : fixture.write_attempts) {
    require(ticket.has_value() && ticket->statuses.size() == 1U &&
                sameStatus(attempt, ticket->statuses.front()),
            "permanent writer failure retried a different terminal status");
  }
}

void testPermanentZeroConfirmationFailureRemainsPendingAcrossThreeRounds() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  fixture.write_attempts.clear();
  fixture.confirmation_results = {StopConfirmationState::TimedOut,
                                  StopConfirmationState::TimedOut,
                                  StopConfirmationState::TimedOut};
  fixture.order.clear();

  std::uint64_t intent_id = 0U;
  std::optional<GoalPlanTerminalDeliveryTicket> ticket;
  for (int round = 0; round < 3; ++round) {
    const auto pending = advanceShutdownTransaction(
        fixture.runtime, fixture.motion_stop, fixture.delivery, 32.0 + round * 0.1);
    require(pending.runtime_result.terminal_intent_id != 0U &&
                pending.runtime_result.terminal_after_stop.has_value() &&
                pending.runtime_result.terminal_stop_policy == TerminalStopPolicy::kShutdown,
            "permanent zero failure lost the shutdown terminal identity");
    if (!ticket.has_value()) {
      intent_id = pending.runtime_result.terminal_intent_id;
      ticket = pending.runtime_result.terminal_after_stop->delivery_ticket;
    } else {
      require(pending.runtime_result.terminal_intent_id == intent_id &&
                  sameTicket(pending.runtime_result.terminal_after_stop->delivery_ticket, *ticket),
              "permanent zero failure changed the exact shutdown intent or ticket");
    }
    require(!pending.stop_confirmed && pending.terminal_required &&
                pending.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kCommitPending &&
                !pending.delivery_acknowledged && !pending.decision.allow_exit &&
                pending.reason == "shutdown_zero_confirm_pending" &&
                fixture.runtime.terminalPending(),
            "permanent zero failure did not remain observably fail-closed");
    require(fixture.motion_stop.keepZeroFresh(),
            "shutdown-pending zero retry did not refresh the zero command");
  }

  require(statusCount(fixture.observed, NavigationGoalState::Cancelled, "navd_shutdown") == 0U,
          "permanent zero failure committed a terminal without stop evidence");
  require(fixture.write_attempts.empty(),
          "permanent zero failure attempted terminal delivery before commit");
  require(actionCount(fixture.order, "stop_control") == 3U &&
              actionCount(fixture.order, "publish_sequenced_zero") == 3U &&
              actionCount(fixture.order, "confirm_zero:20") == 3U,
          "permanent zero failure did not retry zero publication and confirmation every round");
  require(actionCount(fixture.order, "publish_zero") == 3U,
          "permanent zero failure did not keep zero fresh on every pending round");
}

void testShutdownZeroFailureDoesNotExitAndRetriesSameTicket() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  fixture.sequenced_zero.reset();

  const auto failed =
      advanceShutdownTransaction(fixture.runtime, fixture.motion_stop, fixture.delivery, 40.0);
  require(failed.runtime_result.terminal_after_stop.has_value() &&
              failed.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kCommitPending &&
              !failed.stop_confirmed && !failed.decision.allow_exit &&
              statusCount(fixture.observed, NavigationGoalState::Cancelled, "navd_shutdown") ==
                  0U &&
              fixture.runtime.terminalPending(),
          "zero-publish failure committed shutdown or allowed exit");
  const std::uint64_t intent_id = failed.runtime_result.terminal_intent_id;
  const auto ticket = failed.runtime_result.terminal_after_stop->delivery_ticket;

  fixture.sequenced_zero = 20U;
  fixture.order.clear();
  const auto retried =
      advanceShutdownTransaction(fixture.runtime, fixture.motion_stop, fixture.delivery, 40.1);
  require(retried.runtime_result.terminal_intent_id == intent_id &&
              retried.runtime_result.terminal_after_stop.has_value() &&
              sameTicket(retried.runtime_result.terminal_after_stop->delivery_ticket, ticket),
          "shutdown retry changed the zero-failed terminal ticket");
  require(retried.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged &&
              retried.decision.allow_exit &&
              statusCount(fixture.observed, NavigationGoalState::Cancelled, "navd_shutdown") ==
                  1U,
          "confirmed shutdown retry did not commit, deliver, and allow exit exactly once");
}

void testShutdownPreservesExistingPendingTerminalDuringFinalStop() {
  Fixture fixture;
  fixture.activateGoal();
  fixture.writes_allowed = true;
  require(fixture.outbox.flush() == 2U, "initial planning statuses were not delivered");
  const auto existing =
      fixture.runtime.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 50.0);
  require(existing.terminal_intent_id != 0U && existing.terminal_after_stop.has_value(),
          "existing-pending fixture did not create the original terminal");

  const auto shutdown =
      advanceShutdownTransaction(fixture.runtime, fixture.motion_stop, fixture.delivery, 50.1);
  require(shutdown.runtime_result.terminal_intent_id == existing.terminal_intent_id &&
              shutdown.runtime_result.terminal_stop_policy == existing.terminal_stop_policy &&
              shutdown.runtime_result.terminal_after_stop.has_value() &&
              sameTicket(shutdown.runtime_result.terminal_after_stop->delivery_ticket,
                         existing.terminal_after_stop->delivery_ticket),
          "shutdown replaced the existing pending terminal fact");
  require(shutdown.terminal_flush == GoalTerminalStatusDelivery::FlushResult::kAcknowledged &&
              shutdown.decision.allow_exit &&
              statusCount(fixture.observed, NavigationGoalState::Failed, "map_drift") == 1U,
          "shutdown did not final-stop and deliver the preserved pending terminal");
}

void testShutdownWhileEstopLatchedDoesNotClearLatch() {
  Fixture fixture;
  const auto estop = fixture.handleEstop(60.0);
  require(estop.physical_only.has_value() && estop.physical_only->accepted &&
              fixture.estop_latched && fixture.persisted_estop_latched,
          "estop-latched shutdown fixture did not latch and persist estop");

  fixture.order.clear();
  const auto shutdown =
      advanceShutdownTransaction(fixture.runtime, fixture.motion_stop, fixture.delivery, 60.1);
  require(!shutdown.terminal_required && shutdown.stop_confirmed &&
              shutdown.decision.allow_exit && shutdown.reason == "shutdown_complete",
          "goal-free estop-latched shutdown did not complete physical final stop");
  require(fixture.estop_latched && fixture.persisted_estop_latched &&
              fixture.clear_estop_calls == 0 && fixture.clear_persisted_estop_calls == 0,
          "shutdown cleared the live or persisted estop latch");
  require(fixture.outbox.flush() == 0U,
          "goal-free estop-latched shutdown published a fake terminal status");
}

}  // namespace

int main() {
  try {
    testActiveCancelAckIsNotTerminalWriterCompletion();
    testActiveCancelRetryKeepsCancelStopPolicyAfterUnconfirmedZero();
    testStopWithoutActiveGoalDoesNotCreateExecutionTerminal();
    testEstopWithoutActiveGoalDoesNotCreateExecutionTerminal();
    testSafetyStopServicesReplayedNonStopTerminalFromRuntimeOwner();
    testEstopPreservesExistingPendingTerminalFromRuntimeOwner();
    testStopDuringInitialPlanningOnlyDoesNotCreateExecutionTerminal();
    testEstopDuringInitialPlanningOnlyDoesNotProjectExecutionTerminal();
    testActiveShutdownWaitsForWriterAndReplaysExactTicketOnce();
    testPermanentWriterFailureRemainsFailClosedAcrossThreeShutdownRounds();
    testPermanentZeroConfirmationFailureRemainsPendingAcrossThreeRounds();
    testShutdownZeroFailureDoesNotExitAndRetriesSameTicket();
    testShutdownPreservesExistingPendingTerminalDuringFinalStop();
    testShutdownWhileEstopLatchedDoesNotClearLatch();
    testActiveStopRetryKeepsStopPolicyAfterUnconfirmedZero();
    testActiveStopAckIsNotTerminalWriterCompletion();
    testActiveEstopRetryKeepsEstopPolicyAfterZeroUnavailableAndTimeout();
    testDuplicateAlreadyLatchedEstopReplaysWriterPendingIntentWithoutRepeatCommit();
    testDuplicateAlreadyLatchedEstopCompletesPersistWithoutNewTerminal();
    testActiveEstopRetryKeepsEstopPolicyAfterPersistFailure();
    testActiveEstopAckIsNotTerminalWriterCompletion();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_goal_terminal_endpoint_acceptance: FAIL: %s\n", exc.what());
    return 1;
  }
}
