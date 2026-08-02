#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "motion/command_ingress_controller.hpp"
#include "motion/goal_task_cancel_router.hpp"
#include "motion/motion_stop_coordinator.hpp"
#include "plan/goal_replan_runtime_coordinator.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::AutonomyTickOutcome;
using lingtu::nav::endpoint::AutonomyTickOutcomeKind;
using lingtu::nav::endpoint::CommandAck;
using lingtu::nav::endpoint::CommandIngressController;
using lingtu::nav::endpoint::CommandIngressRequest;
using lingtu::nav::endpoint::decideGoalTerminalScheduling;
using lingtu::nav::endpoint::decideShutdownExit;
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
using lingtu::nav::endpoint::GoalPlanTerminalAfterStop;
using lingtu::nav::endpoint::GoalPlanTerminalDeliveryTicket;
using lingtu::nav::endpoint::GoalReplanIdentity;
using lingtu::nav::endpoint::GoalReplanRuntimeAutonomyEvent;
using lingtu::nav::endpoint::GoalReplanRuntimeCoordinator;
using lingtu::nav::endpoint::GoalReplanRuntimeFrameInput;
using lingtu::nav::endpoint::GoalReplanRuntimeInterruption;
using lingtu::nav::endpoint::GoalReplanRuntimeResult;
using lingtu::nav::endpoint::GoalReplanTrigger;
using lingtu::nav::endpoint::GoalReplanTriggerKind;
using lingtu::nav::endpoint::GoalTaskCancelRequest;
using lingtu::nav::endpoint::GoalTaskCancelRouter;
using lingtu::nav::endpoint::GoalTaskCancelTerminalServiceResult;
using lingtu::nav::endpoint::GoalTerminalSchedulingDecision;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopCoordinator;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::TerminalStopPolicy;

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_goal_replan_runtime_coordinator: FAIL: %s\n", message);
    std::exit(1);
  }
}

bool sameStatus(const GoalPlanStatus &lhs, const GoalPlanStatus &rhs) {
  return lhs.task_id == rhs.task_id && lhs.request_id == rhs.request_id &&
         lhs.goal_epoch == rhs.goal_epoch && lhs.state == rhs.state && lhs.reason == rhs.reason &&
         lhs.project_to_navigation_state == rhs.project_to_navigation_state;
}

bool sameStatuses(const std::vector<GoalPlanStatus> &lhs, const std::vector<GoalPlanStatus> &rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (!sameStatus(lhs[i], rhs[i])) {
      return false;
    }
  }
  return true;
}

bool sameBlockedRegion(const lingtu::nav::plan::GlobalPlanBlockedRegion &lhs,
                       const lingtu::nav::plan::GlobalPlanBlockedRegion &rhs) {
  return lhs.center.x == rhs.center.x && lhs.center.y == rhs.center.y &&
         lhs.center.z == rhs.center.z && lhs.radius_xy_m == rhs.radius_xy_m &&
         lhs.min_z == rhs.min_z && lhs.max_z == rhs.max_z;
}

bool sameTemporaryOverlay(const lingtu::nav::plan::GlobalPlanTemporaryOverlay &lhs,
                          const lingtu::nav::plan::GlobalPlanTemporaryOverlay &rhs) {
  if (lhs.revision != rhs.revision || lhs.frame_epoch != rhs.frame_epoch ||
      lhs.obstacle_generation != rhs.obstacle_generation ||
      lhs.traversability_generation != rhs.traversability_generation ||
      lhs.blocked_regions.size() != rhs.blocked_regions.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.blocked_regions.size(); ++i) {
    if (!sameBlockedRegion(lhs.blocked_regions[i], rhs.blocked_regions[i])) {
      return false;
    }
  }
  return true;
}

struct Fixture {
  lingtu::nav::plan::MapIdentity map_identity{"field", 7, "sha256-a", "map"};
  std::vector<GoalPlanStatus> statuses;
  std::vector<GoalPlanPathActivation> activations;
  mutable std::mutex planner_requests_mutex;
  std::vector<lingtu::nav::plan::GlobalPlanRequest> planner_requests;
  std::atomic<int> planner_calls{0};
  std::atomic<int> fail_on_call{0};
  std::atomic<int> block_on_call{0};
  std::atomic<bool> release_blocked{false};
  std::atomic<int> cancelled_plans{0};
  int stop_control_calls{0};
  int clear_motion_calls{0};
  int keep_zero_calls{0};
  bool clear_motion_ok{true};
  bool keep_zero_ok{true};
  std::uint64_t last_output_sequence{17U};
  StopConfirmationState confirmation{StopConfirmationState::Confirmed};
  bool inspection_active{false};
  GoalPlanController goal_plan;
  MotionStopCoordinator motion_stop;
  GoalReplanRuntimeCoordinator coordinator;

  Fixture()
      : goal_plan(
            [this](const lingtu::nav::plan::GlobalPlanRequest &request,
                   const lingtu::nav::plan::GlobalPlanCancelCheck &cancelled) {
              const int call = ++planner_calls;
              lingtu::nav::plan::GlobalPlanResult result;
              {
                std::lock_guard<std::mutex> lock(planner_requests_mutex);
                planner_requests.push_back(request);
              }
              result.overlay_revision = request.temporary_overlay.revision;
              result.overlay_frame_epoch = request.temporary_overlay.frame_epoch;
              result.overlay_obstacle_generation = request.temporary_overlay.obstacle_generation;
              result.overlay_traversability_generation =
                  request.temporary_overlay.traversability_generation;
              if (block_on_call.load() == call) {
                while (!release_blocked.load() && !cancelled()) {
                  std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
              }
              if (cancelled()) {
                ++cancelled_plans;
                result.cancelled = true;
                return result;
              }
              if (fail_on_call.load() == call) {
                result.failure_reason = "planned_failure";
                result.map_identity = map_identity;
                return result;
              }
              result.ok = true;
              result.reached_goal = true;
              result.map_identity = map_identity;
              result.path = {request.start, request.goal};
              return result;
            },
            goalActions()),
        motion_stop(true, stopActions()),
        coordinator(goal_plan, motion_stop) {}

  ~Fixture() { release_blocked.store(true); }

  GoalPlanActions goalActions() {
    GoalPlanActions actions;
    actions.preempt_rolling = [](const std::string &) { return true; };
    actions.clear_external_inspection = [] {};
    actions.current_map_identity = [this] { return GoalPlanMapIdentityResult{map_identity, {}}; };
    actions.publish_status = [this](const GoalPlanStatus &status) { statuses.push_back(status); };
    actions.inspection_active = [this] { return inspection_active; };
    actions.inspection_leg_failed = [](const std::string &, double) {};
    actions.inspection_pause = [](const std::string &) {};
    actions.inspection_plan_ready = [](double) {
      return lingtu::nav::endpoint::GoalPlanInspectionDecision{};
    };
    actions.activate_path = [this](const GoalPlanPathActivation &activation) {
      activations.push_back(activation);
    };
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
      ++clear_motion_calls;
      return clear_motion_ok;
    };
    actions.suspend_motion_outputs = [](const std::string &) { return true; };
    actions.cancel_control = [] {};
    actions.stop_control = [this] { ++stop_control_calls; };
    actions.latch_estop = [](const std::string &) {};
    actions.clear_control_estop = [] { return true; };
    actions.resume_autonomy = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};
    actions.clear_operator_resume_required = [] {};
    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [this] {
      ++keep_zero_calls;
      return keep_zero_ok;
    };
    actions.last_output_sequence = [this] { return last_output_sequence; };
    actions.publish_sequenced_zero = [] { return std::optional<std::uint64_t>{18U}; };
    actions.confirm_zero = [this](std::uint64_t) { return confirmation; };
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
    context.frame_epoch = 3U;
    return context;
  }

  GoalPlanRequest request(std::string task_id = "task-a",
                          std::string request_id = "request-a") const {
    GoalPlanRequest result;
    result.task_id = std::move(task_id);
    result.request_id = std::move(request_id);
    result.origin = GoalPlanOrigin::kExternal;
    result.source_stamp_s = 1.0;
    result.target = GoalPlanTarget{nav_kernel::Vec3{4.0, 1.0, 0.0}, 0.25};
    return result;
  }

  void activate(const GoalPlanRequest &goal) {
    require(goal_plan.submit(goal, admission()).accepted, "fixture goal submission failed");
    for (int i = 0; i < 1000; ++i) {
      const auto result = goal_plan.advance(GoalPlanAdvanceContext{3U, false, 2.0});
      if (result.path_activated) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "fixture goal planning did not complete");
  }

  GoalReplanRuntimeFrameInput frameInput(double steady_now_s, double wall_now_s) const {
    GoalReplanRuntimeFrameInput input;
    input.steady_now_s = steady_now_s;
    input.wall_now_s = wall_now_s;
    input.fresh_admission = admission();
    return input;
  }

  GoalReplanRuntimeFrameInput frameInput(double time_s) const { return frameInput(time_s, time_s); }

  GoalReplanIdentity activeIdentity() const {
    const auto snapshot = goal_plan.snapshot();
    require(snapshot.active_map_identity.has_value(), "fixture active map identity missing");
    return GoalReplanIdentity{snapshot.active_task_id, snapshot.active_request_id,
                              snapshot.active_goal_epoch, *snapshot.active_map_identity};
  }

  std::vector<lingtu::nav::plan::GlobalPlanRequest> plannerRequests() const {
    std::lock_guard<std::mutex> lock(planner_requests_mutex);
    return planner_requests;
  }

  GoalReplanRuntimeAutonomyEvent recoveryEvent() const {
    GoalReplanTrigger trigger;
    trigger.kind = GoalReplanTriggerKind::kLocalRecoveryExhausted;
    trigger.reason = "local_recovery_exhausted";
    trigger.goal = activeIdentity();
    return {
        AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalFailed, "local_recovery_exhausted", false,
                            trigger},
        goal_plan.snapshot(),
        false,
        false,
    };
  }

  GoalReplanRuntimeAutonomyEvent persistentObstructionEvent() const {
    GoalReplanTrigger trigger;
    trigger.kind = GoalReplanTriggerKind::kPersistentPathObstruction;
    trigger.reason = "persistent_path_obstruction";
    trigger.goal = activeIdentity();
    trigger.temporary_overlay.revision = 91U;
    trigger.temporary_overlay.frame_epoch = 3U;
    trigger.temporary_overlay.obstacle_generation = 101U;
    trigger.temporary_overlay.traversability_generation = 103U;
    trigger.temporary_overlay.blocked_regions = {
        {{1.0, 0.25, 0.2}, 0.65, -0.4, 1.6},
        {{2.5, -0.5, 0.3}, 0.55, -0.3, 1.7},
    };
    return {
        AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalFailed, "persistent_path_obstruction",
                            false, trigger},
        goal_plan.snapshot(),
        false,
        false,
    };
  }
  GoalReplanRuntimeAutonomyEvent reachedEvent() const {
    return {
        AutonomyTickOutcome{AutonomyTickOutcomeKind::kGoalReached,
                            "planner_specific_reached_reason", true},
        goal_plan.snapshot(),
        false,
        false,
    };
  }

  GoalReplanRuntimeResult arm(double time_s) {
    const auto frame = frameInput(time_s);
    const auto event = recoveryEvent();
    return coordinator.handleAutonomyOutcome(frame, event);
  }

  void waitForPlannerCalls(int expected, const char *message, int limit = 2000) {
    for (int i = 0; i < limit; ++i) {
      if (planner_calls.load() >= expected) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::fprintf(
        stderr,
        "test_goal_replan_runtime_coordinator: planner call wait timeout expected=%d actual=%d\n",
        expected, planner_calls.load());
    require(false, message);
  }
  GoalReplanRuntimeResult
  waitForPlanning(const std::function<bool(const GoalReplanRuntimeResult &)> &predicate,
                  double start_s, int limit = 2000) {
    GoalReplanRuntimeResult last_result;
    for (int i = 0; i < limit; ++i) {
      auto result = coordinator.advancePlanningCycle(frameInput(start_s + i * 0.001));
      if (predicate(result)) {
        return result;
      }
      last_result = result;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::fprintf(stderr,
                 "test_goal_replan_runtime_coordinator: wait timeout start_s=%.3f "
                 "last_reason=%s last_terminal=%d last_path_activated=%d\n",
                 start_s, last_result.reason.c_str(), last_result.terminal_after_stop.has_value(),
                 last_result.plan_advance.path_activated);
    require(false, "timed out waiting for coordinator result");
    return {};
  }

  GoalReplanRuntimeResult startReplan(double time_s) {
    const auto armed = arm(time_s);
    require(armed.handled && armed.reason == "backoff_pending", "fixture failed to arm replan");
    auto started = coordinator.advancePlanningCycle(frameInput(time_s + 0.5));
    require(started.replan_started, "fixture failed to start replan");
    return started;
  }

  void commitTerminal(const GoalReplanRuntimeResult &result) {
    require(result.terminal_after_stop.has_value(), "terminal intent missing");
    const GoalPlanTerminalAfterStop terminal = *result.terminal_after_stop;
    const std::size_t status_count_before = statuses.size();
    const auto committed =
        motion_stop.commitGoalTerminalAfterStop(terminal.reason, terminal.commit);
    require(committed.accepted, "terminal intent did not commit after confirmed stop");
    std::vector<GoalPlanStatus> published(statuses.begin() + status_count_before, statuses.end());
    require(sameStatuses(terminal.delivery_ticket.statuses, published),
            "terminal delivery ticket did not match committed status vector");
    require(coordinator.acknowledgeTerminal(result.terminal_intent_id),
            "committed terminal intent was not acknowledged");
  }

  auto tryCommitTerminal(const GoalReplanRuntimeResult &result) {
    require(result.terminal_after_stop.has_value(), "terminal intent missing");
    return motion_stop.commitGoalTerminalAfterStop(result.terminal_after_stop->reason,
                                                   result.terminal_after_stop->commit);
  }

  std::size_t countStatus(const std::string &task_id, NavigationGoalState state) const {
    std::size_t count = 0U;
    for (const auto &status : statuses) {
      if (status.task_id == task_id && status.state == state) {
        ++count;
      }
    }
    return count;
  }
};

void testNormalReplanChainHasNoIntermediateTerminal() {
  Fixture fixture;
  fixture.activate(fixture.request());
  require(fixture.statuses.size() == 2U, "fixture did not activate exactly one goal");

  const auto failed_frame = fixture.frameInput(10.0);
  const auto failure_event = fixture.recoveryEvent();
  const auto armed = fixture.coordinator.handleAutonomyOutcome(failed_frame, failure_event);
  require(armed.handled && armed.reason == "backoff_pending",
          "eligible local recovery failure did not arm backoff");
  require(!armed.terminal_after_stop.has_value(), "normal replan arm returned a terminal intent");
  require(fixture.stop_control_calls == 1 && fixture.clear_motion_calls == 1,
          "normal replan did not confirm stop exactly once");
  require(fixture.statuses.size() == 2U,
          "normal replan published an intermediate lifecycle state while arming");

  const auto waiting = fixture.coordinator.advancePlanningCycle(fixture.frameInput(10.499));
  require(waiting.handled && waiting.reason == "backoff_pending" && waiting.zero_kept_fresh &&
              !waiting.replan_started,
          "backoff tick did not hold with a fresh zero");

  const auto started = fixture.coordinator.advancePlanningCycle(fixture.frameInput(10.5));
  require(started.handled && started.replan_started && started.reason == "replan_started",
          "deadline tick did not start one replan");
  require(fixture.statuses.size() == 3U &&
              fixture.statuses.back().state == NavigationGoalState::Planning,
          "replan start did not publish Planning");

  bool completed = false;
  for (int i = 0; i < 1000; ++i) {
    const auto result =
        fixture.coordinator.advancePlanningCycle(fixture.frameInput(10.501 + i * 0.001));
    if (result.plan_advance.path_activated) {
      require(result.reason == "replan_completed", "successful replan completion reason changed");
      require(!result.terminal_after_stop.has_value(),
              "successful replan requested a terminal intent");
      completed = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(completed, "replan did not complete");
  require(fixture.planner_calls == 2, "normal chain did not execute exactly one replan");
  require(fixture.statuses.size() == 4U &&
              fixture.statuses.back().state == NavigationGoalState::PathActive,
          "successful replan did not restore PathActive");
  for (const auto &status : fixture.statuses) {
    require(!lingtu::message::isTerminalNavigationGoalState(status.state),
            "successful replan published an intermediate terminal");
  }
}

void testLegacyRecoveryReasonWithoutTypedTriggerDoesNotArm() {
  Fixture fixture;
  fixture.activate(fixture.request());
  auto legacy_event = fixture.recoveryEvent();
  legacy_event.outcome.replan_trigger.reset();

  const auto ignored =
      fixture.coordinator.handleAutonomyOutcome(fixture.frameInput(10.0), legacy_event);

  require(!ignored.handled && !ignored.replan_started && !ignored.terminal_after_stop.has_value(),
          "legacy kGoalFailed plus reason text must not request a global replan");
  require(fixture.stop_control_calls == 0 && fixture.clear_motion_calls == 0 &&
              fixture.planner_calls.load() == 1 &&
              fixture.coordinator.snapshot().state ==
                  lingtu::nav::endpoint::BoundedGoalReplanState::kIdle,
          "untyped recovery text changed motion or retry ownership");
}

void testPersistentObstructionReplaysExactOverlayAfterStopAndBackoff() {
  Fixture fixture;
  fixture.activate(fixture.request());
  const auto event = fixture.persistentObstructionEvent();
  require(event.outcome.replan_trigger.has_value(),
          "persistent obstruction fixture did not create a typed trigger");
  const auto expected_trigger = *event.outcome.replan_trigger;

  const auto armed = fixture.coordinator.handleAutonomyOutcome(fixture.frameInput(300.0), event);
  require(armed.handled && armed.reason == "backoff_pending" && !armed.replan_started &&
              fixture.stop_control_calls == 1 && fixture.clear_motion_calls == 1 &&
              fixture.planner_calls.load() == 1,
          "persistent obstruction must stop-confirm and only arm the bounded backoff");

  const auto early = fixture.coordinator.advancePlanningCycle(fixture.frameInput(300.499));
  require(early.handled && early.reason == "backoff_pending" && early.zero_kept_fresh &&
              !early.replan_started && fixture.planner_calls.load() == 1,
          "persistent obstruction started planning before the bounded deadline");

  const auto started = fixture.coordinator.advancePlanningCycle(fixture.frameInput(300.5));
  require(started.handled && started.reason == "replan_started" && started.replan_started,
          "persistent obstruction did not start its one replacement plan at the deadline");
  fixture.waitForPlannerCalls(2, "persistent obstruction planner call did not start");

  const auto requests = fixture.plannerRequests();
  require(requests.size() == 2U && requests.front().temporary_overlay.empty(),
          "initial planning unexpectedly received the persistent obstruction overlay");
  require(
      sameTemporaryOverlay(requests.back().temporary_overlay, expected_trigger.temporary_overlay),
      "persistent obstruction overlay identity or regions changed before planning");

  const auto completed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.plan_advance.path_activated; },
      300.501);
  require(completed.reason == "replan_completed" && !completed.terminal_after_stop.has_value(),
          "planner overlay echo was not accepted as the successful replan completion");
}
void testHandleAutonomyOutcomeDoesNotAdvanceOrResume() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.block_on_call.store(2);
  (void)fixture.startReplan(260.0);
  fixture.waitForPlannerCalls(2, "planner call wait timed out");
  const auto queued =
      fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission());
  require(queued.accepted && queued.reason == "planning_queued",
          "outcome-only fixture did not queue B");
  for (int i = 0; i < 2000 && fixture.cancelled_plans.load() < 1; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(fixture.cancelled_plans.load() == 1,
          "outcome-only fixture did not make the cancelled completion ready");

  const auto before = fixture.goal_plan.snapshot();
  const auto activation_count = fixture.activations.size();
  auto event = fixture.recoveryEvent();
  event.outcome = AutonomyTickOutcome{};
  const auto result = fixture.coordinator.handleAutonomyOutcome(fixture.frameInput(260.6), event);
  const auto after = fixture.goal_plan.snapshot();

  require(!result.plan_advance.completion_consumed && !result.plan_advance.path_activated &&
              !result.pending_resumed && !result.replan_started,
          "outcome handling advanced planner state");
  require(before.busy && before.pending_plan_queued && after.busy && after.pending_plan_queued &&
              after.pending_task_id == "task-b" && fixture.activations.size() == activation_count,
          "outcome handling drained or resumed queued planning");
}

void testAdvancePlanningCycleProgressesWithoutAutonomyOutcome() {
  Fixture fixture;
  require(fixture.goal_plan.submit(fixture.request(), fixture.admission()).accepted,
          "cycle-only fixture goal submission failed");

  bool activated = false;
  for (int i = 0; i < 2000; ++i) {
    const auto result =
        fixture.coordinator.advancePlanningCycle(fixture.frameInput(270.0 + i * 0.001));
    if (result.plan_advance.path_activated) {
      activated = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  require(activated && fixture.planner_calls.load() == 1 &&
              fixture.goal_plan.snapshot().active_task_id == "task-a",
          "advancePlanningCycle did not progress a planner completion without an outcome");
}

void testRecoveryFailureOnlyArmsUntilAdvanceDeadline() {
  Fixture fixture;
  fixture.activate(fixture.request());
  const auto frame = fixture.frameInput(280.0);
  const auto event = fixture.recoveryEvent();

  const auto armed = fixture.coordinator.handleAutonomyOutcome(frame, event);
  require(armed.handled && armed.reason == "backoff_pending" && !armed.replan_started &&
              !armed.plan_advance.completion_consumed && fixture.planner_calls.load() == 1,
          "recovery outcome did more than arm bounded retry");

  const auto early = fixture.coordinator.advancePlanningCycle(fixture.frameInput(280.499));
  require(early.handled && early.reason == "backoff_pending" && !early.replan_started &&
              fixture.planner_calls.load() == 1,
          "advancePlanningCycle started retry before the 0.5 second deadline");

  const auto due = fixture.coordinator.advancePlanningCycle(fixture.frameInput(280.5));
  require(due.handled && due.reason == "replan_started" && due.replan_started,
          "advancePlanningCycle did not exclusively start retry at the deadline");
}

void testPendingTerminalBlocksAllRuntimePhases() {
  Fixture fixture;
  fixture.activate(fixture.request());
  const auto pending =
      fixture.coordinator.handleAutonomyOutcome(fixture.frameInput(290.0), fixture.reachedEvent());
  require(pending.terminal_after_stop.has_value() && pending.terminal_intent_id != 0U,
          "terminal-barrier fixture did not create a pending intent");

  const auto before = fixture.goal_plan.snapshot();
  const int planner_calls = fixture.planner_calls.load();
  const std::size_t activation_count = fixture.activations.size();
  const auto cycle_replay = fixture.coordinator.advancePlanningCycle(fixture.frameInput(290.1));
  auto stale_event = fixture.recoveryEvent();
  stale_event.goal_snapshot.active_task_id = "stale-task";
  const auto outcome_replay =
      fixture.coordinator.handleAutonomyOutcome(fixture.frameInput(290.2), stale_event);
  const auto drain_replay = fixture.coordinator.drainPendingCycle(fixture.frameInput(290.2));
  const auto after = fixture.goal_plan.snapshot();

  require(cycle_replay.terminal_intent_id == pending.terminal_intent_id &&
              outcome_replay.terminal_intent_id == pending.terminal_intent_id &&
              drain_replay.terminal_intent_id == pending.terminal_intent_id &&
              cycle_replay.terminal_after_stop.has_value() &&
              outcome_replay.terminal_after_stop.has_value() &&
              drain_replay.terminal_after_stop.has_value(),
          "one runtime phase did not replay the exact pending terminal");
  require(sameStatuses(pending.terminal_after_stop->delivery_ticket.statuses,
                       cycle_replay.terminal_after_stop->delivery_ticket.statuses) &&
              sameStatuses(pending.terminal_after_stop->delivery_ticket.statuses,
                           outcome_replay.terminal_after_stop->delivery_ticket.statuses) &&
              sameStatuses(pending.terminal_after_stop->delivery_ticket.statuses,
                           drain_replay.terminal_after_stop->delivery_ticket.statuses),
          "pending terminal replay changed its delivery ticket");
  require(!cycle_replay.plan_advance.completion_consumed && !cycle_replay.pending_resumed &&
              !cycle_replay.replan_started && !outcome_replay.plan_advance.completion_consumed &&
              !outcome_replay.pending_resumed && !outcome_replay.replan_started &&
              !drain_replay.plan_advance.completion_consumed && !drain_replay.pending_resumed &&
              !drain_replay.replan_started && before.active_task_id == after.active_task_id &&
              before.busy == after.busy &&
              before.pending_plan_queued == after.pending_plan_queued &&
              fixture.planner_calls.load() == planner_calls &&
              fixture.activations.size() == activation_count,
          "a runtime phase progressed through the terminal barrier");
}
void testDrainPendingCycleDoesNotAdvanceOrStartRetry() {
  Fixture completion;
  completion.block_on_call.store(1);
  require(completion.goal_plan.submit(completion.request(), completion.admission()).accepted,
          "drain-isolation planner submission failed");
  completion.waitForPlannerCalls(1, "planner call wait timed out");
  completion.release_blocked.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  const auto before = completion.goal_plan.snapshot();
  const std::size_t activation_count = completion.activations.size();
  const auto drained = completion.coordinator.drainPendingCycle(completion.frameInput(300.0));
  const auto after = completion.goal_plan.snapshot();
  require(!drained.plan_advance.completion_consumed && !drained.plan_advance.path_activated &&
              !drained.replan_started && !drained.pending_resumed && before.busy == after.busy &&
              completion.activations.size() == activation_count,
          "pending drain consumed a ready planner completion");
  const auto activated = completion.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.plan_advance.path_activated; },
      300.0);
  require(activated.reason == "plan_advanced",
          "planning phase did not consume completion left by pending drain");

  Fixture retry;
  retry.activate(retry.request());
  require(retry.arm(310.0).reason == "backoff_pending", "drain-isolation retry did not arm");
  const auto no_start = retry.coordinator.drainPendingCycle(retry.frameInput(310.5));
  require(!no_start.replan_started && retry.planner_calls.load() == 1 &&
              retry.coordinator.snapshot().state ==
                  lingtu::nav::endpoint::BoundedGoalReplanState::kBackoffPending,
          "pending drain started a due bounded retry");
  const auto due = retry.coordinator.advancePlanningCycle(retry.frameInput(310.5));
  require(due.replan_started && due.reason == "replan_started",
          "planning phase did not retain exclusive bounded-start ownership");
}

void testSteadyClockOwnsRetryDeadlineAcrossWallRegression() {
  Fixture fixture;
  fixture.activate(fixture.request());

  const auto observed = fixture.coordinator.advancePlanningCycle(fixture.frameInput(320.0, 1000.0));
  require(!observed.terminal_after_stop.has_value() &&
              !fixture.coordinator.snapshot().budget_consumed,
          "initial dual-clock observation failed");
  const auto wall_regressed =
      fixture.coordinator.advancePlanningCycle(fixture.frameInput(320.1, 900.0));
  require(!wall_regressed.terminal_after_stop.has_value() &&
              !fixture.coordinator.snapshot().budget_consumed,
          "wall-clock regression consumed the bounded steady-clock budget");

  const auto event = fixture.recoveryEvent();
  const auto armed =
      fixture.coordinator.handleAutonomyOutcome(fixture.frameInput(320.2, 800.0), event);
  require(armed.reason == "backoff_pending" &&
              (fixture.coordinator.snapshot().deadline_s > 320.699 &&
               fixture.coordinator.snapshot().deadline_s < 320.701),
          "retry deadline was not based on steady time");
  const auto early = fixture.coordinator.advancePlanningCycle(fixture.frameInput(320.699, 5000.0));
  require(early.reason == "backoff_pending" && !early.replan_started,
          "wall-clock jump started retry before its steady deadline");
  const auto due = fixture.coordinator.advancePlanningCycle(fixture.frameInput(320.7, 100.0));
  require(due.replan_started && due.reason == "replan_started",
          "wall-clock regression moved the steady retry deadline");
}

void testInvalidWallTimeFailsClosedBeforePlannerAdvance() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.block_on_call.store(2);
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "invalid-wall fixture replacement submission failed");
  fixture.waitForPlannerCalls(2, "planner call wait timed out");
  fixture.release_blocked.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  const std::size_t activation_count = fixture.activations.size();

  const auto failed = fixture.coordinator.advancePlanningCycle(
      fixture.frameInput(330.0, std::numeric_limits<double>::quiet_NaN()));
  require(failed.reason == "invalid_wall_time" && failed.terminal_after_stop.has_value() &&
              !failed.plan_advance.completion_consumed && !failed.plan_advance.path_activated &&
              fixture.activations.size() == activation_count &&
              !fixture.coordinator.snapshot().budget_consumed,
          "invalid wall time reached GoalPlan or consumed the steady retry budget");
  fixture.commitTerminal(failed);
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 1U &&
              fixture.goal_plan.snapshot().active_task_id.empty(),
          "invalid wall time did not commit one durable Failed terminal");
}
void testExternalGoalReachedDefersStableTerminalUntilStop() {
  Fixture fixture;
  fixture.activate(fixture.request());
  const std::size_t statuses_before_reached = fixture.statuses.size();

  const auto frame = fixture.frameInput(15.0);
  const auto event = fixture.reachedEvent();
  const auto reached = fixture.coordinator.handleAutonomyOutcome(frame, event);

  require(reached.handled && reached.reason == "goal_reached" &&
              reached.terminal_after_stop.has_value() && reached.terminal_intent_id != 0U &&
              reached.terminal_task_id == "task-a",
          "external goal reach did not surface one stable terminal intent");
  require(fixture.statuses.size() == statuses_before_reached &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "external goal reach published terminal before the stop barrier");
  require(fixture.goal_plan.snapshot().active_task_id == "task-a" &&
              fixture.stop_control_calls == 0,
          "deferring external reach mutated the active goal or stopped motion directly");
}
void testExternalGoalReachedIntentReplaysUntilExactAck() {
  Fixture fixture;
  fixture.activate(fixture.request());
  const auto frame = fixture.frameInput(16.0);
  const auto event = fixture.reachedEvent();
  const auto pending = fixture.coordinator.handleAutonomyOutcome(frame, event);
  const std::uint64_t intent_id = pending.terminal_intent_id;
  require(intent_id != 0U && fixture.coordinator.terminalPending(),
          "external reach did not retain a durable terminal intent");

  fixture.confirmation = StopConfirmationState::TimedOut;
  require(!fixture.tryCommitTerminal(pending).accepted &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "timed-out reached stop published a terminal");
  const auto timeout_retry = fixture.coordinator.advancePlanningCycle(fixture.frameInput(16.1));
  require(timeout_retry.terminal_after_stop.has_value() &&
              timeout_retry.terminal_intent_id == intent_id,
          "timed-out reached intent did not replay its exact ID");

  fixture.confirmation = StopConfirmationState::DriverRejected;
  require(!fixture.tryCommitTerminal(timeout_retry).accepted &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "driver-rejected reached stop published a terminal");
  const auto rejected_retry = fixture.coordinator.advancePlanningCycle(fixture.frameInput(16.2));
  require(rejected_retry.terminal_after_stop.has_value() &&
              rejected_retry.terminal_intent_id == intent_id,
          "driver-rejected reached intent did not replay its exact ID");
  require(!fixture.coordinator.acknowledgeTerminal(intent_id + 1U) &&
              fixture.coordinator.terminalPending(),
          "wrong acknowledgement cleared a reached terminal intent");

  fixture.confirmation = StopConfirmationState::Confirmed;
  require(fixture.tryCommitTerminal(rejected_retry).accepted &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 1U &&
              fixture.coordinator.terminalPending(),
          "confirmed reached stop did not commit exactly one terminal before ack");
  const auto awaiting_ack = fixture.coordinator.advancePlanningCycle(fixture.frameInput(16.3));
  require(awaiting_ack.terminal_after_stop.has_value() &&
              awaiting_ack.terminal_intent_id == intent_id,
          "committed reached intent did not await its exact acknowledgement");
  require(fixture.coordinator.acknowledgeTerminal(intent_id) &&
              !fixture.coordinator.terminalPending(),
          "exact reached acknowledgement did not close the transaction");
  require(!fixture.coordinator.advancePlanningCycle(fixture.frameInput(16.4)).terminal_after_stop &&
              !fixture.coordinator.acknowledgeTerminal(intent_id),
          "acknowledged reached intent resurfaced or accepted a stale ack");
  require(fixture.tryCommitTerminal(rejected_retry).accepted &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 1U,
          "replayed reached commit published more than one terminal");
}
void testStaleGoalReachedIdentityIsIgnored() {
  auto run = [](const std::function<void(GoalReplanRuntimeAutonomyEvent &)> &mutate) {
    Fixture fixture;
    fixture.activate(fixture.request());
    const std::size_t statuses_before_reached = fixture.statuses.size();
    auto event = fixture.reachedEvent();
    mutate(event);
    const auto frame = fixture.frameInput(17.0);
    const auto ignored = fixture.coordinator.handleAutonomyOutcome(frame, event);
    require(ignored.handled && ignored.reason == "stale_autonomy_goal_reached_ignored" &&
                !ignored.terminal_after_stop.has_value() && !fixture.coordinator.terminalPending(),
            "stale goal reach identity was not ignored");
    require(fixture.statuses.size() == statuses_before_reached &&
                fixture.goal_plan.snapshot().active_task_id == "task-a" &&
                fixture.stop_control_calls == 0,
            "stale goal reach changed the active lifecycle");
  };

  run([](GoalReplanRuntimeAutonomyEvent &event) {
    event.goal_snapshot.active_task_id = "stale-task";
  });
  run([](GoalReplanRuntimeAutonomyEvent &event) {
    event.goal_snapshot.active_request_id = "stale-request";
  });
  run([](GoalReplanRuntimeAutonomyEvent &event) { ++event.goal_snapshot.active_goal_epoch; });
}
void testGoalReachedMapDriftFailsClosed() {
  Fixture fixture;
  fixture.activate(fixture.request());
  const auto frame = fixture.frameInput(18.0);
  auto event = fixture.reachedEvent();
  event.goal_snapshot.active_map_identity->artifact_sha256 = "sha256-stale";

  const auto failed = fixture.coordinator.handleAutonomyOutcome(frame, event);

  require(failed.handled && failed.reason == "active_goal_map_identity_changed" &&
              failed.terminal_after_stop.has_value() && failed.terminal_intent_id != 0U,
          "same-ID reached map drift did not fail closed");
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 0U &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "reached map drift published terminal before the stop barrier");
  fixture.commitTerminal(failed);
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 1U &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "reached map drift did not commit exactly one Failed terminal");
}
void testGoalReachedHonorsCurrentMapDriftAndControlHold() {
  Fixture drift;
  drift.activate(drift.request());
  auto drift_input = drift.frameInput(19.0);
  const auto drift_event = drift.reachedEvent();
  drift_input.map_drift = true;
  const auto failed = drift.coordinator.handleAutonomyOutcome(drift_input, drift_event);

  require(failed.reason == "active_goal_map_identity_changed" &&
              failed.terminal_after_stop.has_value() &&
              drift.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "current map drift was consumed as Reached");
  drift.commitTerminal(failed);
  require(drift.countStatus("task-a", NavigationGoalState::Failed) == 1U &&
              drift.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "current map drift did not commit exactly one Failed terminal");

  Fixture held;
  held.activate(held.request());
  auto held_input = held.frameInput(19.1);
  const auto held_event = held.reachedEvent();
  held_input.control_hold = true;
  const auto ignored = held.coordinator.handleAutonomyOutcome(held_input, held_event);

  require(ignored.handled && ignored.reason == "goal_reached_control_hold_ignored" &&
              !ignored.terminal_after_stop.has_value() && !held.coordinator.terminalPending(),
          "control-held reach consumed the active goal");
  require(held.goal_plan.snapshot().active_task_id == "task-a" &&
              held.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "control-held reach changed the active lifecycle");
}
void testGoalReachedRequiresStableCapturedAndCurrentPlan() {
  auto require_unstable = [](Fixture &fixture, const GoalReplanRuntimeResult &result) {
    require(result.handled && result.reason == "goal_reached_plan_unstable" &&
                !result.terminal_after_stop.has_value() && !fixture.coordinator.terminalPending(),
            "unstable plan state consumed goal reach");
    require(fixture.goal_plan.snapshot().active_task_id == "task-a" &&
                fixture.countStatus("task-a", NavigationGoalState::Reached) == 0U,
            "unstable plan state changed the active lifecycle");
  };

  auto run_captured =
      [&require_unstable](const std::function<void(GoalReplanRuntimeAutonomyEvent &)> &mutate) {
        Fixture fixture;
        fixture.activate(fixture.request());
        const auto frame = fixture.frameInput(20.0);
        auto event = fixture.reachedEvent();
        mutate(event);
        require_unstable(fixture, fixture.coordinator.handleAutonomyOutcome(frame, event));
      };
  run_captured([](GoalReplanRuntimeAutonomyEvent &event) { event.goal_snapshot.busy = true; });
  run_captured(
      [](GoalReplanRuntimeAutonomyEvent &event) { event.goal_snapshot.replan_in_progress = true; });

  Fixture current_busy;
  current_busy.activate(current_busy.request());
  auto before_busy = current_busy.reachedEvent();
  current_busy.block_on_call.store(2);
  require(current_busy.goal_plan
              .submit(current_busy.request("task-b", "request-b"), current_busy.admission())
              .accepted,
          "current-busy fixture did not start replacement planning");
  current_busy.waitForPlannerCalls(2, "planner call wait timed out");
  const auto busy_frame = current_busy.frameInput(20.1);
  require_unstable(current_busy,
                   current_busy.coordinator.handleAutonomyOutcome(busy_frame, before_busy));

  Fixture current_replan;
  current_replan.activate(current_replan.request());
  auto before_replan = current_replan.reachedEvent();
  current_replan.block_on_call.store(2);
  (void)current_replan.startReplan(21.0);
  current_replan.waitForPlannerCalls(2, "planner call wait timed out");
  const auto replan_frame = current_replan.frameInput(21.6);
  require_unstable(current_replan,
                   current_replan.coordinator.handleAutonomyOutcome(replan_frame, before_replan));

  Fixture replacement;
  replacement.activate(replacement.request());
  require(replacement.arm(22.0).reason == "backoff_pending",
          "replacement fixture did not arm bounded state");
  auto before_replacement = replacement.reachedEvent();
  replacement.block_on_call.store(2);
  require(replacement.goal_plan
              .submit(replacement.request("task-b", "request-b"), replacement.admission())
              .accepted,
          "replacement fixture did not start replacement planning");
  replacement.waitForPlannerCalls(2, "planner call wait timed out");
  const auto replacement_frame = replacement.frameInput(22.1);
  require_unstable(replacement, replacement.coordinator.handleAutonomyOutcome(replacement_frame,
                                                                              before_replacement));
}
void testReachedClosesAThenResumesPendingB() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.block_on_call.store(2);
  (void)fixture.startReplan(23.0);
  fixture.waitForPlannerCalls(2, "planner call wait timed out");
  const auto queued =
      fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission());
  require(queued.accepted && queued.reason == "planning_queued",
          "pending-B reached fixture did not queue B");

  GoalReplanRuntimeResult planning;
  for (int i = 0; i < 2000; ++i) {
    planning = fixture.coordinator.advancePlanningCycle(fixture.frameInput(23.501 + i * 0.001));
    if (!fixture.goal_plan.snapshot().busy) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto stable_a = fixture.goal_plan.snapshot();
  require(planning.reason == "pending_plan_ready" && !planning.pending_resumed && !stable_a.busy &&
              !stable_a.replan_in_progress && stable_a.active_task_id == "task-a" &&
              stable_a.pending_plan_queued && stable_a.pending_task_id == "task-b",
          "planning phase resumed B before the autonomy outcome");

  const auto reached_frame = fixture.frameInput(24.0);
  const auto reached_event = fixture.reachedEvent();
  const auto reached = fixture.coordinator.handleAutonomyOutcome(reached_frame, reached_event);
  require(reached.terminal_after_stop.has_value() && reached.reason == "goal_reached" &&
              fixture.goal_plan.snapshot().pending_plan_queued &&
              fixture.goal_plan.snapshot().pending_task_id == "task-b",
          "A reach resumed or discarded pending B before stop confirmation");
  const auto blocked_drain = fixture.coordinator.drainPendingCycle(reached_frame);
  require(blocked_drain.terminal_intent_id == reached.terminal_intent_id &&
              blocked_drain.terminal_after_stop.has_value() && !blocked_drain.pending_resumed,
          "pending drain progressed before A terminal commit and exact acknowledgement");

  fixture.commitTerminal(reached);
  const auto stopped_a = fixture.goal_plan.snapshot();
  require(stopped_a.active_task_id.empty() && stopped_a.pending_plan_queued &&
              stopped_a.pending_task_id == "task-b" &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 1U,
          "A stop-confirmed reach did not preserve queued B");

  const auto resumed = fixture.coordinator.drainPendingCycle(reached_frame);
  require(resumed.pending_resumed && resumed.reason == "pending_plan_resumed" &&
              !fixture.goal_plan.snapshot().pending_plan_queued,
          "queued B did not resume after exact A acknowledgement");
  const auto activated = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.plan_advance.path_activated; },
      24.001);
  require(activated.reason == "replacement_plan_completed" &&
              fixture.goal_plan.snapshot().active_task_id == "task-b" &&
              fixture.countStatus("task-a", NavigationGoalState::Reached) == 1U,
          "resumed B did not activate after exactly one A Reached terminal");
}

void testReplacementCompletionDefersBActivationUntilOldActiveTerminalAck() {
  Fixture fixture;
  fixture.activate(fixture.request());
  require(fixture.activations.size() == 1U &&
              fixture.goal_plan.snapshot().active_task_id == "task-a",
          "replacement-barrier fixture did not start with active A");

  const auto replacement =
      fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission());
  require(replacement.accepted, "replacement-barrier fixture could not submit replacement B");

  const auto completed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) {
        return result.terminal_after_stop.has_value() || result.plan_advance.path_activated;
      },
      30.0);
  require(completed.terminal_after_stop.has_value() && completed.terminal_intent_id != 0U &&
              completed.terminal_task_id == "task-a",
          "replacement B completion did not surface A exact terminal before activation");
  require(!completed.plan_advance.path_activated && fixture.activations.size() == 1U &&
              fixture.goal_plan.snapshot().active_task_id == "task-a" &&
              fixture.countStatus("task-a", NavigationGoalState::Cancelled) == 0U &&
              fixture.countStatus("task-b", NavigationGoalState::PathActive) == 0U,
          "replacement B activated or published A terminal before stop/commit/ACK");

  const auto blocked_drain = fixture.coordinator.drainPendingCycle(fixture.frameInput(30.25));
  require(blocked_drain.terminal_after_stop.has_value() &&
              blocked_drain.terminal_intent_id == completed.terminal_intent_id &&
              !blocked_drain.pending_resumed && fixture.activations.size() == 1U,
          "pending drain progressed replacement B before exact A terminal ACK");

  fixture.commitTerminal(completed);
  require(!fixture.coordinator.terminalPending() &&
              fixture.countStatus("task-a", NavigationGoalState::Cancelled) == 1U,
          "exact A terminal ACK did not close the old active transaction");

  auto stopped_path_frame = fixture.frameInput(30.5);
  stopped_path_frame.fresh_admission.retained_path_ready = false;
  stopped_path_frame.fresh_admission.retained_path_reason = "retained_global_path_missing";
  const auto activated = fixture.coordinator.advancePlanningCycle(stopped_path_frame);
  require(activated.reason == "replacement_plan_completed" &&
              activated.plan_advance.path_activated &&
              fixture.goal_plan.snapshot().active_task_id == "task-b" &&
              fixture.countStatus("task-b", NavigationGoalState::PathActive) == 1U,
          "replacement B reused A's cleared retained-path gate after exact terminal ACK");
}

void testReplacementActivationAfterAckRechecksFreshEstopAdmission() {
  Fixture fixture;
  fixture.activate(fixture.request());
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "estop replacement fixture could not submit B");
  const auto completed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      31.0);
  fixture.commitTerminal(completed);

  auto blocked_input = fixture.frameInput(31.5);
  blocked_input.fresh_admission.motion_allowed = false;
  const auto blocked = fixture.coordinator.advancePlanningCycle(blocked_input);
  require(blocked.terminal_after_stop.has_value() && blocked.terminal_task_id == "task-b" &&
              blocked.reason == "estop_latched" && fixture.activations.size() == 1U &&
              fixture.countStatus("task-b", NavigationGoalState::PathActive) == 0U,
          "deferred B activated or missed terminal when Estop latched after A ACK");
  const auto &ticket = blocked.terminal_after_stop->delivery_ticket.statuses;
  require(ticket.size() == 1U && ticket.front().task_id == "task-b" &&
              ticket.front().state == NavigationGoalState::Cancelled &&
              ticket.front().reason == "estop_latched" &&
              !ticket.front().project_to_navigation_state,
          "deferred B Estop rejection did not create a non-projecting Cancelled ticket");
  fixture.commitTerminal(blocked);
  require(fixture.countStatus("task-b", NavigationGoalState::Cancelled) == 1U &&
              fixture.goal_plan.snapshot().active_task_id.empty(),
          "deferred B Estop rejection did not close exactly once without activation");
  const auto retry = fixture.coordinator.advancePlanningCycle(blocked_input);
  require(!retry.terminal_after_stop.has_value() && fixture.activations.size() == 1U,
          "deferred B Estop rejection replayed after exact ACK or activated later");
}

void testReplacementActivationAfterAckRechecksStoredMapIdentity() {
  Fixture fixture;
  fixture.activate(fixture.request());
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "map replacement fixture could not submit B");
  const auto completed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      32.0);
  fixture.commitTerminal(completed);

  fixture.map_identity.version = 8;
  const auto blocked = fixture.coordinator.advancePlanningCycle(fixture.frameInput(32.5));
  require(blocked.terminal_after_stop.has_value() && blocked.terminal_task_id == "task-b" &&
              blocked.reason == "active_map_changed_before_replacement_activation" &&
              fixture.activations.size() == 1U &&
              fixture.countStatus("task-b", NavigationGoalState::PathActive) == 0U,
          "deferred B activated after map identity changed post-ACK");
  const auto &ticket = blocked.terminal_after_stop->delivery_ticket.statuses;
  require(ticket.size() == 1U && ticket.front().task_id == "task-b" &&
              ticket.front().state == NavigationGoalState::Failed &&
              ticket.front().reason == "active_map_changed_before_replacement_activation" &&
              !ticket.front().project_to_navigation_state,
          "deferred B map mismatch did not create a non-projecting Failed ticket");
  fixture.commitTerminal(blocked);
  require(fixture.countStatus("task-b", NavigationGoalState::Failed) == 1U &&
              fixture.goal_plan.snapshot().active_task_id.empty(),
          "deferred B map mismatch did not close exactly once without activation");
}
void testDeferredReplacementInterruptionsCloseBWithExactNonProjectingTerminals() {
  struct InterruptionCase {
    GoalReplanRuntimeInterruption interruption;
    NavigationGoalState state;
    const char *reason;
    TerminalStopPolicy stop_policy;
  };
  const std::vector<InterruptionCase> cases = {
      {GoalReplanRuntimeInterruption::kNewGoal, NavigationGoalState::Cancelled,
       "superseded_by_new_goal", TerminalStopPolicy::kGenericStop},
      {GoalReplanRuntimeInterruption::kCancel, NavigationGoalState::Cancelled, "cancelled",
       TerminalStopPolicy::kCancel},
      {GoalReplanRuntimeInterruption::kStop, NavigationGoalState::Cancelled, "stopped",
       TerminalStopPolicy::kStop},
      {GoalReplanRuntimeInterruption::kEstop, NavigationGoalState::Cancelled, "estop_latched",
       TerminalStopPolicy::kEstop},
      {GoalReplanRuntimeInterruption::kShutdown, NavigationGoalState::Cancelled, "navd_shutdown",
       TerminalStopPolicy::kShutdown},
      {GoalReplanRuntimeInterruption::kOperatorTakeover, NavigationGoalState::Cancelled,
       "operator_takeover_resume_required", TerminalStopPolicy::kGenericStop},
      {GoalReplanRuntimeInterruption::kDriverAuthorityLost, NavigationGoalState::Cancelled,
       "driver_authority_lost", TerminalStopPolicy::kGenericStop},
      {GoalReplanRuntimeInterruption::kControlHold, NavigationGoalState::Cancelled, "control_hold",
       TerminalStopPolicy::kGenericStop},
      {GoalReplanRuntimeInterruption::kInspectionPause, NavigationGoalState::Cancelled,
       "inspection_pause_requested", TerminalStopPolicy::kGenericStop},
      {GoalReplanRuntimeInterruption::kInspectionCancel, NavigationGoalState::Cancelled,
       "inspection_cancel_requested", TerminalStopPolicy::kGenericStop},
      {GoalReplanRuntimeInterruption::kMapDrift, NavigationGoalState::Failed, "map_drift",
       TerminalStopPolicy::kGenericStop},
  };

  for (std::size_t i = 0; i < cases.size(); ++i) {
    Fixture fixture;
    fixture.activate(fixture.request());
    require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
                .accepted,
            "deferred-interrupt fixture could not submit replacement B");
    const auto completed = fixture.waitForPlanning(
        [](const GoalReplanRuntimeResult &result) {
          return result.terminal_after_stop.has_value();
        },
        33.0 + static_cast<double>(i));
    fixture.commitTerminal(completed);
    const auto hidden_replacement = fixture.goal_plan.snapshot();
    require(hidden_replacement.active_task_id.empty() &&
                hidden_replacement.planning_task_id.empty() &&
                hidden_replacement.deferred_replacement_task_id == "task-b" &&
                hidden_replacement.deferred_replacement_request_id == "request-b" &&
                hidden_replacement.deferred_replacement_goal_epoch != 0U,
            "deferred B identity was not visible to targeted endpoint routing after A ACK");

    const auto interrupted =
        fixture.coordinator.interrupt(cases[i].interruption, 40.0 + static_cast<double>(i));
    require(interrupted.handled && interrupted.interrupted &&
                interrupted.terminal_after_stop.has_value(),
            "deferred B interruption did not surface a terminal");
    require(interrupted.terminal_task_id == "task-b" && interrupted.reason == cases[i].reason &&
                interrupted.terminal_stop_policy == cases[i].stop_policy,
            "deferred B interruption lost task/reason/stop-policy identity");
    require(fixture.goal_plan.snapshot().active_task_id.empty() &&
                fixture.activations.size() == 1U &&
                fixture.countStatus("task-b", NavigationGoalState::PathActive) == 0U,
            "interrupted deferred B activated or became active");

    const auto &ticket = interrupted.terminal_after_stop->delivery_ticket.statuses;
    require(
        ticket.size() == 1U && ticket.front().task_id == "task-b" &&
            ticket.front().request_id == "request-b" && ticket.front().state == cases[i].state &&
            ticket.front().reason == cases[i].reason && !ticket.front().project_to_navigation_state,
        "deferred B interruption ticket was not exact and non-projecting");

    const auto replay = fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kControlHold,
                                                      50.0 + static_cast<double>(i));
    require(replay.terminal_intent_id == interrupted.terminal_intent_id &&
                replay.terminal_task_id == interrupted.terminal_task_id &&
                replay.terminal_stop_policy == interrupted.terminal_stop_policy &&
                replay.terminal_after_stop.has_value() &&
                sameStatuses(replay.terminal_after_stop->delivery_ticket.statuses,
                             interrupted.terminal_after_stop->delivery_ticket.statuses),
            "deferred B interruption did not replay the exact same terminal intent");

    fixture.commitTerminal(interrupted);
    const auto after_ack = fixture.coordinator.advancePlanningCycle(fixture.frameInput(60.0 + i));
    require(!after_ack.terminal_after_stop.has_value() && fixture.activations.size() == 1U,
            "acknowledged deferred B interruption replayed or activated later");
  }
}

void testTypedTaskCancelRoutesHiddenReplacementWithoutCommandIdentityLeak() {
  Fixture fixture;
  fixture.activate(fixture.request());
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "typed-cancel fixture could not submit replacement B");
  const auto completed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      60.0);
  fixture.commitTerminal(completed);
  require(fixture.goal_plan.snapshot().deferred_replacement_task_id == "task-b",
          "typed-cancel fixture did not expose hidden replacement B");

  int terminal_service_calls = 0;
  int status_requests = 0;
  std::optional<GoalReplanRuntimeResult> serviced_terminal;
  GoalTaskCancelRouter router(
      fixture.goal_plan, fixture.coordinator,
      [&](const GoalReplanRuntimeResult &result) {
        ++terminal_service_calls;
        serviced_terminal = result;
        fixture.commitTerminal(result);
        return GoalTaskCancelTerminalServiceResult{true, "goal_terminal_acknowledged"};
      },
      [&] { ++status_requests; });

  CommandIngressRequest request;
  request.client_id = "operator-a";
  request.request_id = "cancel-command-1";
  request.task_id = "task-b";
  request.raw_kind = static_cast<std::int32_t>(lingtu::message::NavigationCommandKind::TaskCancel);
  request.payload.reason = "operator_cancel";

  CommandIngressController ingress;
  const auto handled = ingress.handle(
      request, [&](lingtu::message::NavigationCommandKind kind, const auto &payload) {
        require(kind == lingtu::message::NavigationCommandKind::TaskCancel,
                "typed ingress changed TaskCancel kind before routing");
        return router.handle(
            GoalTaskCancelRequest{request.task_id, request.request_id, payload.reason, 61.0});
      });

  require(handled.dispatched && !handled.replayed && handled.ack.accepted &&
              handled.ack.reason == "cancel_requested" && handled.request_id == "cancel-command-1",
          "typed hidden-B cancel did not return the command-scoped accepted ACK");
  require(terminal_service_calls == 1 && status_requests == 1 && serviced_terminal.has_value(),
          "typed hidden-B cancel did not service exactly one terminal and status refresh");
  const auto &statuses = serviced_terminal->terminal_after_stop->delivery_ticket.statuses;
  require(statuses.size() == 1U && statuses.front().task_id == "task-b" &&
              statuses.front().request_id == "request-b" &&
              statuses.front().request_id != request.request_id &&
              statuses.front().state == NavigationGoalState::Cancelled &&
              statuses.front().reason == "cancelled" &&
              !statuses.front().project_to_navigation_state,
          "typed hidden-B cancel leaked command identity or changed the exact terminal");
  const auto after_ack = fixture.coordinator.advancePlanningCycle(fixture.frameInput(61.1));
  require(!after_ack.terminal_after_stop.has_value() && fixture.activations.size() == 1U &&
              fixture.goal_plan.snapshot().active_task_id.empty(),
          "acknowledged typed hidden-B cancel later activated replacement B");
}

void testTypedTaskCancelWaitsForForeignTerminalThenRetriesExactHiddenB() {
  Fixture fixture;
  fixture.activate(fixture.request());
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "barrier-retry fixture could not submit replacement B");
  const auto active_a_terminal = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      61.5);
  const auto blocked_snapshot = fixture.goal_plan.snapshot();
  require(fixture.coordinator.terminalPending() && blocked_snapshot.active_task_id == "task-a" &&
              blocked_snapshot.deferred_replacement_task_id == "task-b",
          "barrier-retry fixture did not retain A terminal over hidden B");

  int terminal_service_calls = 0;
  std::optional<GoalReplanRuntimeResult> serviced_terminal;
  GoalTaskCancelRouter router(
      fixture.goal_plan, fixture.coordinator,
      [&](const GoalReplanRuntimeResult &result) {
        ++terminal_service_calls;
        serviced_terminal = result;
        fixture.commitTerminal(result);
        return GoalTaskCancelTerminalServiceResult{true, "goal_terminal_acknowledged"};
      },
      [] {});

  CommandIngressRequest request;
  request.client_id = "operator-a";
  request.request_id = "cancel-command-retry";
  request.task_id = "task-b";
  request.raw_kind = static_cast<std::int32_t>(lingtu::message::NavigationCommandKind::TaskCancel);
  request.payload.reason = "operator_cancel";
  CommandIngressController ingress;
  const auto dispatch = [&](lingtu::message::NavigationCommandKind kind, const auto &payload) {
    require(kind == lingtu::message::NavigationCommandKind::TaskCancel,
            "barrier retry changed TaskCancel kind");
    return router.handle(
        GoalTaskCancelRequest{request.task_id, request.request_id, payload.reason, 61.6});
  };

  const auto blocked = ingress.handle(request, dispatch);
  require(blocked.dispatched && !blocked.replayed && !blocked.ack.accepted &&
              blocked.ack.reason == "goal_terminal_pending" && terminal_service_calls == 0 &&
              fixture.coordinator.terminalPending(),
          "hidden-B cancel acknowledged or serviced A's foreign pending terminal");

  fixture.commitTerminal(active_a_terminal);
  const auto retried = ingress.handle(request, dispatch);
  require(retried.dispatched && !retried.replayed && retried.ack.accepted &&
              retried.ack.reason == "cancel_requested" && terminal_service_calls == 1 &&
              serviced_terminal.has_value(),
          "same hidden-B cancel was cached negative or did not retry after A ACK");
  const auto &statuses = serviced_terminal->terminal_after_stop->delivery_ticket.statuses;
  require(statuses.size() == 1U && statuses.front().task_id == "task-b" &&
              statuses.front().request_id == "request-b" &&
              statuses.front().state == NavigationGoalState::Cancelled &&
              !statuses.front().project_to_navigation_state,
          "retried hidden-B cancel did not service B's exact terminal");

  const auto replayed = ingress.handle(request, dispatch);
  require(replayed.replayed && !replayed.dispatched && replayed.ack.accepted &&
              replayed.ack.reason == "cancel_requested" && terminal_service_calls == 1,
          "accepted hidden-B cancel did not replay ACK without repeating terminal service");
  const auto after_ack = fixture.coordinator.advancePlanningCycle(fixture.frameInput(61.7));
  require(!after_ack.terminal_after_stop.has_value() && fixture.activations.size() == 1U,
          "retried and acknowledged hidden-B cancel later activated B");
}

void testTypedTaskCancelDuringInitialPlanningUsesTicketedRuntimeTerminal() {
  Fixture fixture;
  fixture.block_on_call.store(1);
  const GoalPlanRequest goal = fixture.request();
  require(fixture.goal_plan.submit(goal, fixture.admission()).accepted,
          "initial-planning cancel fixture could not submit goal");
  fixture.waitForPlannerCalls(1, "initial-planning cancel fixture did not enter planner");
  const auto planning = fixture.goal_plan.snapshot();
  require(planning.active_task_id.empty() && planning.planning_task_id == goal.task_id,
          "initial-planning cancel fixture unexpectedly activated the goal");

  int terminal_service_calls = 0;
  std::optional<GoalReplanRuntimeResult> serviced_terminal;
  GoalTaskCancelRouter router(
      fixture.goal_plan, fixture.coordinator,
      [&](const GoalReplanRuntimeResult &result) {
        ++terminal_service_calls;
        serviced_terminal = result;
        fixture.commitTerminal(result);
        return GoalTaskCancelTerminalServiceResult{true, "goal_terminal_acknowledged"};
      },
      [] {});

  CommandIngressRequest request;
  request.client_id = "operator-a";
  request.request_id = "cancel-initial-planning";
  request.task_id = goal.task_id;
  request.raw_kind = static_cast<std::int32_t>(lingtu::message::NavigationCommandKind::TaskCancel);
  request.payload.reason = "operator_cancel";
  CommandIngressController ingress;
  const auto handled =
      ingress.handle(request, [&](lingtu::message::NavigationCommandKind, const auto &payload) {
        return router.handle(
            GoalTaskCancelRequest{request.task_id, request.request_id, payload.reason, 64.0});
      });

  require(handled.dispatched && handled.ack.accepted && handled.ack.reason == "cancel_requested" &&
              terminal_service_calls == 1 && serviced_terminal.has_value() &&
              !fixture.coordinator.terminalPending(),
          "initial-planning TaskCancel bypassed ticketed runtime terminal service");
  require(serviced_terminal->terminal_stop_policy == TerminalStopPolicy::kCancel &&
              serviced_terminal->terminal_task_id == goal.task_id &&
              serviced_terminal->terminal_after_stop.has_value(),
          "initial-planning TaskCancel lost cancel stop policy or terminal identity");
  const auto &statuses = serviced_terminal->terminal_after_stop->delivery_ticket.statuses;
  require(statuses.size() == 1U && statuses.front().task_id == goal.task_id &&
              statuses.front().request_id == goal.request_id &&
              statuses.front().state == NavigationGoalState::Cancelled &&
              statuses.front().reason == "cancelled" &&
              statuses.front().project_to_navigation_state,
          "initial-planning TaskCancel did not preserve its exact ticketed terminal");
  for (int i = 0; i < 1000 && fixture.cancelled_plans.load() == 0; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(fixture.cancelled_plans.load() == 1 &&
              fixture.goal_plan.snapshot().planning_task_id.empty(),
          "initial-planning TaskCancel did not cancel the in-flight planner exactly once");
}

void testTypedTaskCancelRoutesActiveTaskThroughTicketedRuntimeTerminal() {
  Fixture fixture;
  const GoalPlanRequest goal = fixture.request();
  fixture.activate(goal);

  int terminal_service_calls = 0;
  std::optional<GoalReplanRuntimeResult> serviced_terminal;
  GoalTaskCancelRouter router(
      fixture.goal_plan, fixture.coordinator,
      [&](const GoalReplanRuntimeResult &result) {
        ++terminal_service_calls;
        serviced_terminal = result;
        fixture.commitTerminal(result);
        return GoalTaskCancelTerminalServiceResult{true, "goal_terminal_acknowledged"};
      },
      [] {});

  CommandIngressRequest request;
  request.client_id = "operator-a";
  request.request_id = "cancel-active-command";
  request.task_id = goal.task_id;
  request.raw_kind = static_cast<std::int32_t>(lingtu::message::NavigationCommandKind::TaskCancel);
  request.payload.reason = "operator_cancel";
  CommandIngressController ingress;
  const auto handled =
      ingress.handle(request, [&](lingtu::message::NavigationCommandKind, const auto &payload) {
        return router.handle(
            GoalTaskCancelRequest{request.task_id, request.request_id, payload.reason, 64.5});
      });

  require(handled.dispatched && handled.ack.accepted && handled.ack.reason == "cancel_requested" &&
              terminal_service_calls == 1 && serviced_terminal.has_value() &&
              !fixture.coordinator.terminalPending(),
          "active TaskCancel did not complete one ticketed runtime terminal service");
  const auto &terminal = *serviced_terminal;
  const auto &statuses = terminal.terminal_after_stop->delivery_ticket.statuses;
  require(terminal.terminal_stop_policy == TerminalStopPolicy::kCancel &&
              terminal.terminal_task_id == goal.task_id && statuses.size() == 1U &&
              statuses.front().task_id == goal.task_id &&
              statuses.front().request_id == goal.request_id &&
              statuses.front().request_id != request.request_id &&
              statuses.front().state == NavigationGoalState::Cancelled &&
              statuses.front().project_to_navigation_state &&
              fixture.goal_plan.snapshot().active_task_id.empty(),
          "active TaskCancel lost goal identity, cancel policy, or terminal projection");
}

void testTypedTaskCancelRoutesQueuedAndReplacementPlanningWithoutStoppingActiveA() {
  {
    Fixture fixture;
    fixture.activate(fixture.request());
    fixture.block_on_call.store(2);
    require(fixture.goal_plan.replanActive(fixture.admission()).accepted,
            "queued-cancel fixture could not start active-A replan");
    fixture.waitForPlannerCalls(2, "queued-cancel fixture did not enter active-A replan");
    require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
                    .accepted &&
                fixture.goal_plan.snapshot().pending_task_id == "task-b",
            "queued-cancel fixture did not expose pending B");
    const std::size_t status_count_before = fixture.statuses.size();
    int terminal_service_calls = 0;
    int status_requests = 0;
    GoalTaskCancelRouter router(
        fixture.goal_plan, fixture.coordinator,
        [&](const GoalReplanRuntimeResult &) {
          ++terminal_service_calls;
          return GoalTaskCancelTerminalServiceResult{true, "unexpected_terminal_service"};
        },
        [&] { ++status_requests; });

    const CommandAck cancelled = router.handle(
        GoalTaskCancelRequest{"task-b", "cancel-queued-command", "operator_cancel", 65.0});
    const auto after = fixture.goal_plan.snapshot();
    require(cancelled.accepted && cancelled.reason == "cancel_requested" &&
                terminal_service_calls == 0 && status_requests == 1 && !after.pending_plan_queued &&
                after.active_task_id == "task-a" && fixture.activations.size() == 1U &&
                fixture.statuses.size() == status_count_before + 1U,
            "queued B TaskCancel routed through stop/terminal service or changed active A");
    const auto &status = fixture.statuses.back();
    require(status.task_id == "task-b" && status.request_id == "cancel-queued-command" &&
                status.state == NavigationGoalState::Cancelled &&
                !status.project_to_navigation_state,
            "queued B TaskCancel changed command identity or projected over active A");
  }

  {
    Fixture fixture;
    fixture.activate(fixture.request());
    fixture.block_on_call.store(2);
    require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
                .accepted,
            "replacement-planning cancel fixture could not submit B");
    fixture.waitForPlannerCalls(2, "replacement-planning cancel fixture did not enter planner");
    const auto before = fixture.goal_plan.snapshot();
    require(before.replacement_plan_in_progress && before.planning_task_id == "task-b" &&
                before.active_task_id == "task-a",
            "replacement-planning cancel fixture did not expose planning B over active A");
    const std::size_t status_count_before = fixture.statuses.size();
    int terminal_service_calls = 0;
    int status_requests = 0;
    GoalTaskCancelRouter router(
        fixture.goal_plan, fixture.coordinator,
        [&](const GoalReplanRuntimeResult &) {
          ++terminal_service_calls;
          return GoalTaskCancelTerminalServiceResult{true, "unexpected_terminal_service"};
        },
        [&] { ++status_requests; });

    const CommandAck cancelled = router.handle(
        GoalTaskCancelRequest{"task-b", "cancel-replacement-command", "operator_cancel", 65.5});
    const auto after = fixture.goal_plan.snapshot();
    require(cancelled.accepted && cancelled.reason == "cancel_requested" &&
                terminal_service_calls == 0 && status_requests == 1 &&
                after.planning_task_id.empty() && after.active_task_id == "task-a" &&
                fixture.activations.size() == 1U &&
                fixture.statuses.size() == status_count_before + 1U,
            "replacement-planning B cancel used stop service or changed active A");
    const auto &status = fixture.statuses.back();
    require(status.task_id == "task-b" && status.request_id == "cancel-replacement-command" &&
                status.state == NavigationGoalState::Cancelled &&
                !status.project_to_navigation_state,
            "replacement-planning B cancel changed command identity or projected over active A");
    for (int i = 0; i < 1000 && fixture.cancelled_plans.load() == 0; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(fixture.cancelled_plans.load() == 1,
            "replacement-planning B cancel did not stop its planner exactly once");
  }
}

void testTypedTaskCancelRejectsForeignTaskWithoutLifecycleEffects() {
  Fixture fixture;
  fixture.activate(fixture.request());
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "foreign-cancel fixture could not submit replacement B");
  const auto completed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      62.0);
  fixture.commitTerminal(completed);
  const auto before = fixture.goal_plan.snapshot();
  const std::size_t status_count_before = fixture.statuses.size();
  const int clear_motion_before = fixture.clear_motion_calls;

  int terminal_service_calls = 0;
  int status_requests = 0;
  GoalTaskCancelRouter router(
      fixture.goal_plan, fixture.coordinator,
      [&](const GoalReplanRuntimeResult &) {
        ++terminal_service_calls;
        return GoalTaskCancelTerminalServiceResult{true, "unexpected_terminal_service"};
      },
      [&] { ++status_requests; });

  const CommandAck rejected = router.handle(
      GoalTaskCancelRequest{"task-foreign", "cancel-command-foreign", "operator_cancel", 63.0});
  const auto after = fixture.goal_plan.snapshot();
  require(!rejected.accepted && rejected.reason == "task_not_active",
          "foreign typed TaskCancel was not rejected by task identity");
  require(terminal_service_calls == 0 && status_requests == 0 &&
              fixture.clear_motion_calls == clear_motion_before &&
              fixture.statuses.size() == status_count_before &&
              !fixture.coordinator.terminalPending(),
          "foreign typed TaskCancel triggered terminal, stop, status, or publication effects");
  require(after.deferred_replacement_task_id == before.deferred_replacement_task_id &&
              after.deferred_replacement_request_id == before.deferred_replacement_request_id &&
              after.deferred_replacement_goal_epoch == before.deferred_replacement_goal_epoch,
          "foreign typed TaskCancel mutated hidden replacement identity");
  const auto activated = fixture.coordinator.advancePlanningCycle(fixture.frameInput(63.1));
  require(activated.plan_advance.path_activated &&
              fixture.goal_plan.snapshot().active_task_id == "task-b",
          "foreign typed TaskCancel prevented the untouched replacement from activating");
}

void testInspectionOutcomesSurfaceTicketedTerminalsAndRollingStaysExcluded() {
  auto require_excluded = [](Fixture &fixture, const GoalReplanRuntimeFrameInput &frame,
                             const GoalReplanRuntimeAutonomyEvent &event) {
    const std::size_t statuses_before = fixture.statuses.size();
    const auto excluded = fixture.coordinator.handleAutonomyOutcome(frame, event);
    require(!excluded.terminal_after_stop.has_value() && !fixture.coordinator.terminalPending() &&
                fixture.statuses.size() == statuses_before,
            "inspection/rolling reach produced a terminal intent or status");
    require(!fixture.goal_plan.snapshot().active_task_id.empty() && fixture.stop_control_calls == 0,
            "inspection/rolling reach changed the active lifecycle");
  };

  Fixture inspection_origin;
  auto inspection_goal = inspection_origin.request();
  inspection_goal.origin = GoalPlanOrigin::kInspection;
  inspection_origin.activate(inspection_goal);
  auto inspection_frame = inspection_origin.frameInput(25.0);
  inspection_frame.inspection_active = true;
  auto inspection_event = inspection_origin.reachedEvent();
  inspection_event.inspection_active = true;
  const auto inspection_reached =
      inspection_origin.coordinator.handleAutonomyOutcome(inspection_frame, inspection_event);
  require(inspection_reached.terminal_after_stop.has_value() &&
              inspection_reached.reason == "goal_reached" &&
              inspection_reached.terminal_task_id == "task-a" &&
              inspection_reached.terminal_intent_id != 0U,
          "inspection-origin reach did not surface a ticketed terminal intent");
  require(inspection_reached.terminal_after_stop->delivery_ticket.statuses.size() == 1U &&
              inspection_reached.terminal_after_stop->delivery_ticket.statuses.front().task_id ==
                  "task-a" &&
              inspection_reached.terminal_after_stop->delivery_ticket.statuses.front().request_id ==
                  "request-a" &&
              inspection_reached.terminal_after_stop->delivery_ticket.statuses.front().state ==
                  NavigationGoalState::Reached,
          "inspection-origin reach terminal ticket lost active identity");
  require(inspection_origin.countStatus("task-a", NavigationGoalState::Reached) == 0U,
          "inspection-origin reach published before stop confirmation");
  inspection_origin.commitTerminal(inspection_reached);
  require(inspection_origin.countStatus("task-a", NavigationGoalState::Reached) == 1U,
          "inspection-origin reach did not commit exactly one terminal");

  Fixture inspection_failed;
  auto failed_goal = inspection_failed.request();
  failed_goal.origin = GoalPlanOrigin::kInspection;
  inspection_failed.activate(failed_goal);
  auto failed_frame = inspection_failed.frameInput(25.05);
  failed_frame.inspection_active = true;
  auto failed_event = inspection_failed.recoveryEvent();
  failed_event.inspection_active = true;
  failed_event.outcome.reason = "inspection_action_failed";
  const auto failed_result =
      inspection_failed.coordinator.handleAutonomyOutcome(failed_frame, failed_event);
  require(failed_result.terminal_after_stop.has_value() &&
              failed_result.reason == "inspection_action_failed" &&
              failed_result.terminal_task_id == "task-a",
          "inspection-origin failure did not surface a ticketed terminal intent");
  require(failed_result.terminal_after_stop->delivery_ticket.statuses.size() == 1U &&
              failed_result.terminal_after_stop->delivery_ticket.statuses.front().state ==
                  NavigationGoalState::Failed &&
              failed_result.terminal_after_stop->delivery_ticket.statuses.front().reason ==
                  "inspection_action_failed",
          "inspection-origin failure terminal ticket lost status values");
  inspection_failed.commitTerminal(failed_result);
  require(inspection_failed.countStatus("task-a", NavigationGoalState::Failed) == 1U,
          "inspection-origin failure did not commit exactly one terminal");

  auto run_external =
      [&require_excluded](
          const std::function<void(GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &)>
              &mutate) {
        Fixture fixture;
        fixture.activate(fixture.request());
        auto frame = fixture.frameInput(25.1);
        auto event = fixture.reachedEvent();
        mutate(frame, event);
        require_excluded(fixture, frame, event);
      };
  run_external([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.inspection_active = true;
  });
  run_external([](GoalReplanRuntimeFrameInput &frame, GoalReplanRuntimeAutonomyEvent &) {
    frame.inspection_active = true;
  });
  run_external([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.rolling_segment_active = true;
  });
  run_external([](GoalReplanRuntimeFrameInput &frame, GoalReplanRuntimeAutonomyEvent &) {
    frame.rolling_segment_active = true;
  });
  run_external([](GoalReplanRuntimeFrameInput &frame, GoalReplanRuntimeAutonomyEvent &) {
    frame.fresh_admission.rolling_segment_active = true;
  });
  run_external([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.outcome.kind = AutonomyTickOutcomeKind::kRollingReached;
  });
}

void testStopAndZeroFailuresOnlyReturnTerminalIntents() {
  Fixture stop_failed;
  stop_failed.activate(stop_failed.request());
  stop_failed.clear_motion_ok = false;
  auto stopped = stop_failed.arm(20.0);
  require(stopped.terminal_after_stop.has_value(), "stop failure did not return terminal intent");
  require(stop_failed.coordinator.snapshot().budget_consumed,
          "stop failure did not consume retry budget");
  require(stop_failed.countStatus("task-a", NavigationGoalState::Failed) == 0U,
          "stop failure published terminal before a confirmed stop");
  stop_failed.clear_motion_ok = true;
  stop_failed.commitTerminal(stopped);
  require(stop_failed.countStatus("task-a", NavigationGoalState::Failed) == 1U,
          "stop failure terminal did not commit exactly once");

  Fixture zero_failed;
  zero_failed.activate(zero_failed.request());
  require(zero_failed.arm(30.0).reason == "backoff_pending", "zero failure fixture did not arm");
  zero_failed.keep_zero_ok = false;
  auto refreshed = zero_failed.coordinator.advancePlanningCycle(zero_failed.frameInput(30.1));
  require(refreshed.reason == "zero_refresh_failed" && refreshed.terminal_after_stop.has_value(),
          "zero refresh failure did not return terminal intent");
  require(zero_failed.countStatus("task-a", NavigationGoalState::Failed) == 0U,
          "zero refresh failure published terminal directly");
  zero_failed.keep_zero_ok = true;
  zero_failed.commitTerminal(refreshed);
  require(zero_failed.countStatus("task-a", NavigationGoalState::Failed) == 1U,
          "zero refresh terminal did not commit once");
}

void testPlannerFailureConsumesBudgetAndDefersTerminal() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.fail_on_call.store(2);
  (void)fixture.startReplan(40.0);
  auto failed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      40.501);
  require(failed.reason == "planned_failure", "planner failure reason changed");
  require(fixture.coordinator.snapshot().budget_consumed, "planner failure did not consume budget");
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 0U,
          "planner failure published terminal before stop barrier");
  fixture.commitTerminal(failed);
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 1U,
          "planner failure terminal did not commit once");
}

void testTerminalIntentPersistsUntilExactSuccessfulAck() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.fail_on_call.store(2);
  (void)fixture.startReplan(45.0);
  auto pending = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      45.501);
  const std::uint64_t first_intent_id = pending.terminal_intent_id;
  require(first_intent_id != 0U && pending.terminal_task_id == "task-a" &&
              fixture.coordinator.terminalPending(),
          "terminal intent lost its exact identity");
  require(!fixture.coordinator.acknowledgeTerminal(first_intent_id + 1U),
          "wrong terminal intent ID cleared the pending transaction");
  require(!fixture.coordinator.acknowledgeTerminal(0U),
          "zero terminal intent ID cleared the pending transaction");

  fixture.confirmation = StopConfirmationState::TimedOut;
  require(!fixture.tryCommitTerminal(pending).accepted,
          "timed-out stop unexpectedly committed terminal");
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 0U,
          "timed-out stop published terminal");
  auto retry = fixture.coordinator.advancePlanningCycle(fixture.frameInput(46.0));
  require(retry.terminal_after_stop.has_value() && retry.terminal_intent_id == first_intent_id,
          "timed-out terminal intent was not resurfaced");

  fixture.confirmation = StopConfirmationState::DriverRejected;
  require(!fixture.tryCommitTerminal(retry).accepted,
          "driver-rejected stop unexpectedly committed terminal");
  auto final_retry = fixture.coordinator.advancePlanningCycle(fixture.frameInput(46.1));
  require(final_retry.terminal_after_stop.has_value() &&
              final_retry.terminal_intent_id == first_intent_id,
          "driver-rejected terminal intent was not resurfaced");
  const auto blocked_new_goal =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kNewGoal, 46.15);
  require(blocked_new_goal.terminal_after_stop.has_value() &&
              blocked_new_goal.terminal_intent_id == first_intent_id &&
              !blocked_new_goal.interrupted,
          "pending terminal did not block and resurface at command ingress");

  fixture.confirmation = StopConfirmationState::Confirmed;
  fixture.commitTerminal(final_retry);
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 1U &&
              !fixture.coordinator.terminalPending(),
          "retried terminal intent did not commit exactly once");
  require(!fixture.coordinator.acknowledgeTerminal(first_intent_id),
          "stale terminal acknowledgement succeeded after completion");
  require(!fixture.coordinator.advancePlanningCycle(fixture.frameInput(46.2)).terminal_after_stop,
          "acknowledged terminal intent resurfaced");

  fixture.fail_on_call.store(0);
  fixture.activate(fixture.request("task-a", "request-a-reused"));
  fixture.clear_motion_ok = false;
  auto second = fixture.arm(47.0);
  require(second.terminal_after_stop.has_value() && second.terminal_intent_id > first_intent_id &&
              second.terminal_task_id == "task-a",
          "same-task reuse did not allocate a fresh terminal intent ID");
  require(!fixture.coordinator.acknowledgeTerminal(first_intent_id) &&
              fixture.coordinator.terminalPending(),
          "stale acknowledgement cleared a newer terminal transaction");
  fixture.clear_motion_ok = true;
  fixture.commitTerminal(second);
}

void testReplayPendingTerminalIsReadOnlyExactSingleOwnerView() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.fail_on_call.store(2);
  (void)fixture.startReplan(48.0);
  auto pending = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      48.501);
  require(pending.terminal_intent_id != 0U && pending.terminal_after_stop.has_value() &&
              fixture.coordinator.terminalPending(),
          "fixture did not create a pending terminal intent");

  const auto replay_a = fixture.coordinator.replayPendingTerminal();
  const auto replay_b = fixture.coordinator.replayPendingTerminal();
  require(replay_a.terminal_after_stop.has_value() && replay_b.terminal_after_stop.has_value(),
          "pending terminal was not replayed");
  require(replay_a.terminal_intent_id == pending.terminal_intent_id &&
              replay_b.terminal_intent_id == pending.terminal_intent_id,
          "replay allocated or changed the terminal intent");
  require(replay_a.terminal_task_id == pending.terminal_task_id &&
              replay_b.terminal_task_id == pending.terminal_task_id,
          "replay changed the terminal task");
  require(replay_a.terminal_stop_policy == pending.terminal_stop_policy &&
              replay_b.terminal_stop_policy == pending.terminal_stop_policy,
          "replay changed the terminal stop policy");
  require(sameStatuses(replay_a.terminal_after_stop->delivery_ticket.statuses,
                       pending.terminal_after_stop->delivery_ticket.statuses) &&
              sameStatuses(replay_b.terminal_after_stop->delivery_ticket.statuses,
                           pending.terminal_after_stop->delivery_ticket.statuses),
          "replay changed the terminal delivery ticket");
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 0U &&
              fixture.coordinator.terminalPending(),
          "read-only replay committed, acknowledged, or cleared the terminal");

  require(fixture.coordinator.acknowledgeTerminal(pending.terminal_intent_id),
          "exact acknowledgement did not clear the pending terminal");
  const auto empty = fixture.coordinator.replayPendingTerminal();
  require(!empty.terminal_after_stop.has_value() && empty.terminal_intent_id == 0U &&
              !fixture.coordinator.terminalPending(),
          "acknowledged terminal replay did not become empty");
}

void testOnlyNewTaskRestoresRetryBudget() {
  Fixture fixture;
  fixture.activate(fixture.request());
  (void)fixture.startReplan(50.0);
  const auto replanned = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.plan_advance.path_activated; },
      50.501);
  require(replanned.reason == "replan_completed", "successful replan did not complete");

  auto pause = fixture.goal_plan.deferPause("task-a", "pause-request", "operator_pause");
  require(pause.accepted, "same-task pause was rejected");
  pause.commit();
  auto resume = fixture.goal_plan.deferResume("task-a", "resume-request", fixture.admission());
  require(resume.accepted, "same-task resume was rejected");
  resume.commit();

  auto exhausted = fixture.arm(51.0);
  require(exhausted.terminal_after_stop.has_value(),
          "same task/request transition reset retry budget");
  fixture.commitTerminal(exhausted);

  fixture.activate(fixture.request("task-b", "request-b"));
  auto fresh = fixture.arm(60.0);
  require(fresh.reason == "backoff_pending" && !fresh.terminal_after_stop.has_value(),
          "new task did not receive a fresh retry budget");
}

void testPendingGoalDrainsAndResumesWithFreshBudget() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.block_on_call.store(2);
  (void)fixture.startReplan(70.0);
  fixture.waitForPlannerCalls(2, "planner call wait timed out");

  const auto queued =
      fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission());
  require(queued.accepted && queued.reason == "planning_queued",
          "new task was not retained by GoalPlan while replan was busy");

  GoalReplanRuntimeResult planning;
  double drain_time_s = 70.501;
  for (int i = 0; i < 2000; ++i) {
    drain_time_s = 70.501 + i * 0.001;
    planning = fixture.coordinator.advancePlanningCycle(fixture.frameInput(drain_time_s));
    if (planning.reason == "pending_plan_ready") {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(planning.reason == "pending_plan_ready" && !planning.pending_resumed,
          "planning phase did not drain stale A without resuming B");
  const auto resumed = fixture.coordinator.drainPendingCycle(fixture.frameInput(drain_time_s));
  require(resumed.pending_resumed && resumed.reason == "pending_plan_resumed",
          "drain phase did not resume pending task without an outcome");
  const auto deferred_terminal = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      72.0);
  require(deferred_terminal.reason == "superseded_by_new_goal" &&
              deferred_terminal.terminal_task_id == "task-a" &&
              !deferred_terminal.plan_advance.path_activated && fixture.activations.size() == 1U &&
              fixture.countStatus("task-b", NavigationGoalState::PathActive) == 0U,
          "replacement completion did not defer B behind A terminal barrier");
  fixture.commitTerminal(deferred_terminal);
  const auto activated = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.plan_advance.path_activated; },
      72.5);
  require(activated.reason == "replacement_plan_completed", "replacement task did not activate");
  require(fixture.cancelled_plans.load() == 1, "stale replan was not cancelled exactly once");
  require(fixture.goal_plan.snapshot().active_task_id == "task-b",
          "stale A completion displaced task B");

  auto fresh = fixture.arm(73.0);
  require(fresh.reason == "backoff_pending" && !fresh.terminal_after_stop.has_value(),
          "task B did not get a fresh retry budget");
}

void testPendingRejectionClosesBThenDefersA() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.block_on_call.store(2);
  (void)fixture.startReplan(80.0);
  fixture.waitForPlannerCalls(2, "planner call wait timed out");
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "pending rejection fixture did not queue B");

  auto rejected_input = fixture.frameInput(80.501);
  rejected_input.fresh_admission.planner_map_configured = false;
  rejected_input.fresh_admission.planner_map_missing_reason = "fresh_map_missing";
  GoalReplanRuntimeResult rejected;
  for (int i = 0; i < 2000; ++i) {
    rejected_input.steady_now_s = 80.501 + i * 0.001;
    rejected_input.wall_now_s = 80.501 + i * 0.001;
    const auto planning = fixture.coordinator.advancePlanningCycle(rejected_input);
    if (planning.reason == "pending_plan_ready") {
      require(!planning.pending_resumed, "planning phase resumed B before pending rejection");
      rejected = fixture.coordinator.drainPendingCycle(rejected_input);
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(rejected.terminal_after_stop.has_value() && rejected.reason == "fresh_map_missing",
          "pending B rejection did not return A terminal intent");
  require(fixture.countStatus("task-b", NavigationGoalState::Failed) == 1U,
          "pending B was not terminalled exactly once");
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 0U,
          "active A terminal bypassed stop-before-terminal intent");
  require(rejected.terminal_task_id == "task-a", "active A terminal task changed");
  require(fixture.coordinator.terminalPending(), "active A terminal was not retained");
  const auto pending_intent_id = rejected.terminal_intent_id;
  require(pending_intent_id != 0U, "active A terminal intent ID was not allocated");
  require(!fixture.coordinator.acknowledgeTerminal(pending_intent_id + 1U) &&
              fixture.coordinator.terminalPending(),
          "wrong pending-rejection acknowledgement cleared active A terminal");
  const auto blocked_retry = fixture.coordinator.drainPendingCycle(rejected_input);
  require(blocked_retry.terminal_after_stop.has_value() &&
              blocked_retry.terminal_intent_id == pending_intent_id &&
              blocked_retry.terminal_task_id == "task-a" &&
              blocked_retry.reason == "fresh_map_missing",
          "pending terminal did not block drain with the exact A intent");
  require(sameStatuses(rejected.terminal_after_stop->delivery_ticket.statuses,
                       blocked_retry.terminal_after_stop->delivery_ticket.statuses),
          "pending A/B replay changed active A terminal ticket");
  require(!blocked_retry.pending_resumed && fixture.activations.size() == 1U,
          "pending terminal allowed B execution or path activation");
  fixture.commitTerminal(rejected);
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 1U,
          "active A terminal did not commit once");
  require(!fixture.coordinator.terminalPending(), "exact acknowledgement did not clear terminal");
  const auto after_ack = fixture.coordinator.drainPendingCycle(rejected_input);
  require(!after_ack.pending_resumed && !after_ack.terminal_after_stop.has_value() &&
              fixture.activations.size() == 1U,
          "acknowledged pending rejection resumed B or replayed A terminal");
}

void testReplacementPlannerFailureDefersOldActiveTerminal() {
  Fixture fixture;
  fixture.activate(fixture.request());
  fixture.block_on_call.store(2);
  fixture.fail_on_call.store(3);
  (void)fixture.startReplan(90.0);
  fixture.waitForPlannerCalls(2, "planner call wait timed out");
  require(fixture.goal_plan.submit(fixture.request("task-b", "request-b"), fixture.admission())
              .accepted,
          "replacement failure fixture did not queue B");

  GoalReplanRuntimeResult planning;
  double drain_time_s = 90.501;
  for (int i = 0; i < 2000; ++i) {
    drain_time_s = 90.501 + i * 0.001;
    planning = fixture.coordinator.advancePlanningCycle(fixture.frameInput(drain_time_s));
    if (planning.reason == "pending_plan_ready") {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(planning.reason == "pending_plan_ready" && !planning.pending_resumed,
          "planning phase resumed failed replacement B");
  const auto resumed = fixture.coordinator.drainPendingCycle(fixture.frameInput(drain_time_s));
  require(resumed.pending_resumed, "drain phase did not start replacement B");
  auto failed = fixture.waitForPlanning(
      [](const GoalReplanRuntimeResult &result) { return result.terminal_after_stop.has_value(); },
      drain_time_s + 0.001);
  require(failed.reason == "planned_failure", "replacement failure reason changed");
  require(fixture.countStatus("task-b", NavigationGoalState::Failed) == 1U,
          "failed replacement B did not publish exactly one terminal");
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 0U,
          "old active A terminal published before stop barrier");
  fixture.commitTerminal(failed);
  require(fixture.countStatus("task-a", NavigationGoalState::Failed) == 1U,
          "old active A terminal was not committed exactly once");
}

void testAllInterruptionsConsumeActiveAttempt() {
  const std::vector<GoalReplanRuntimeInterruption> interruptions = {
      GoalReplanRuntimeInterruption::kNewGoal,
      GoalReplanRuntimeInterruption::kCancel,
      GoalReplanRuntimeInterruption::kStop,
      GoalReplanRuntimeInterruption::kEstop,
      GoalReplanRuntimeInterruption::kOperatorTakeover,
      GoalReplanRuntimeInterruption::kDriverAuthorityLost,
      GoalReplanRuntimeInterruption::kControlHold,
      GoalReplanRuntimeInterruption::kInspectionPause,
      GoalReplanRuntimeInterruption::kInspectionCancel,
      GoalReplanRuntimeInterruption::kMapDrift,
  };
  for (std::size_t i = 0; i < interruptions.size(); ++i) {
    Fixture fixture;
    fixture.activate(fixture.request());
    require(fixture.arm(100.0 + i * 10.0).reason == "backoff_pending",
            "interrupt fixture did not arm");
    const auto interrupted = fixture.coordinator.interrupt(interruptions[i], 100.1 + i * 10.0);
    require(interrupted.handled && interrupted.interrupted, "interrupt was not acknowledged");
    const auto state = fixture.coordinator.snapshot().state;
    require(state != lingtu::nav::endpoint::BoundedGoalReplanState::kBackoffPending &&
                state != lingtu::nav::endpoint::BoundedGoalReplanState::kReplanInFlight,
            "interrupt left retry attempt active");
    const bool owns_terminal =
        interruptions[i] == GoalReplanRuntimeInterruption::kCancel ||
        interruptions[i] == GoalReplanRuntimeInterruption::kStop ||
        interruptions[i] == GoalReplanRuntimeInterruption::kEstop ||
        interruptions[i] == GoalReplanRuntimeInterruption::kOperatorTakeover ||
        interruptions[i] == GoalReplanRuntimeInterruption::kDriverAuthorityLost ||
        interruptions[i] == GoalReplanRuntimeInterruption::kControlHold ||
        interruptions[i] == GoalReplanRuntimeInterruption::kInspectionPause ||
        interruptions[i] == GoalReplanRuntimeInterruption::kInspectionCancel ||
        interruptions[i] == GoalReplanRuntimeInterruption::kMapDrift;
    require(interrupted.terminal_after_stop.has_value() == owns_terminal,
            "hold/map terminal classification changed");
    if (interruptions[i] == GoalReplanRuntimeInterruption::kEstop) {
      require(interrupted.reason == "estop_latched",
              "active estop must acknowledge the latched estop fact");
      require(interrupted.terminal_stop_policy == TerminalStopPolicy::kEstop,
              "active estop terminal must retain the estop stop policy");
      const auto &ticket = interrupted.terminal_after_stop->delivery_ticket;
      require(ticket.statuses.size() == 1U &&
                  ticket.statuses.front().state == NavigationGoalState::Cancelled &&
                  ticket.statuses.front().reason == "estop_latched" &&
                  ticket.statuses.front().project_to_navigation_state,
              "active estop must create a durable Cancelled/estop_latched terminal");
    }
  }

  Fixture stale;
  stale.activate(stale.request());
  stale.block_on_call.store(2);
  (void)stale.startReplan(190.0);
  stale.waitForPlannerCalls(2, "planner call wait timed out");
  const auto interrupted =
      stale.coordinator.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 190.6);
  require(interrupted.terminal_after_stop.has_value(),
          "in-flight map drift did not return terminal intent");
  stale.commitTerminal(interrupted);
  for (int i = 0; i < 2000 && stale.goal_plan.snapshot().busy; ++i) {
    (void)stale.coordinator.advancePlanningCycle(stale.frameInput(190.601 + i * 0.001));
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(!stale.goal_plan.snapshot().busy, "interrupted planner completion did not drain");
  require(stale.activations.size() == 1U, "stale interrupted completion activated a path");
}

void testInspectionInterruptionsProduceCancelledTicketsWithDiagnosticReasons() {
  struct InspectionInterruptionCase {
    GoalReplanRuntimeInterruption interruption;
    const char *reason;
  };
  const std::vector<InspectionInterruptionCase> cases = {
      {GoalReplanRuntimeInterruption::kInspectionPause, "inspection_pause_requested"},
      {GoalReplanRuntimeInterruption::kInspectionCancel, "inspection_cancel_requested"},
  };

  for (const auto &test_case : cases) {
    Fixture fixture;
    fixture.activate(fixture.request());
    const auto terminal = fixture.coordinator.interrupt(test_case.interruption, 300.0);
    require(terminal.terminal_after_stop.has_value() && terminal.terminal_intent_id != 0U &&
                terminal.reason == test_case.reason,
            "inspection interruption did not surface a diagnostic terminal ticket");
    const auto &ticket = terminal.terminal_after_stop->delivery_ticket;
    require(ticket.statuses.size() == 1U &&
                ticket.statuses.front().state == NavigationGoalState::Cancelled &&
                ticket.statuses.front().reason == test_case.reason,
            "inspection interruption terminal ticket lost Cancelled status or diagnostic reason");
    require(fixture.countStatus("task-a", NavigationGoalState::Cancelled) == 0U,
            "inspection interruption published before stop confirmation");
    fixture.commitTerminal(terminal);
    require(fixture.countStatus("task-a", NavigationGoalState::Cancelled) == 1U,
            "inspection interruption did not commit exactly one Cancelled terminal");
  }
}

void testIneligibleOutcomesNeverTakeOver() {
  auto run =
      [](const std::function<void(GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &)>
             &mutate) {
        Fixture fixture;
        fixture.activate(fixture.request());
        auto frame = fixture.frameInput(210.0);
        auto event = fixture.recoveryEvent();
        mutate(frame, event);
        const auto result = fixture.coordinator.handleAutonomyOutcome(frame, event);
        require(!result.handled && fixture.stop_control_calls == 0 &&
                    fixture.coordinator.snapshot().state ==
                        lingtu::nav::endpoint::BoundedGoalReplanState::kIdle,
                "ineligible autonomy outcome took over goal lifecycle");
      };

  run([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.outcome.reason = "other_failure";
    event.outcome.replan_trigger.reset();
  });
  run([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.goal_snapshot.active_origin = GoalPlanOrigin::kInspection;
  });
  run([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.inspection_active = true;
  });
  run([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.rolling_segment_active = true;
  });
  run([](GoalReplanRuntimeFrameInput &frame, GoalReplanRuntimeAutonomyEvent &) {
    frame.inspection_active = true;
  });
  run([](GoalReplanRuntimeFrameInput &frame, GoalReplanRuntimeAutonomyEvent &) {
    frame.rolling_segment_active = true;
  });
  run([](GoalReplanRuntimeFrameInput &, GoalReplanRuntimeAutonomyEvent &event) {
    event.outcome.kind = AutonomyTickOutcomeKind::kRollingRecoveryExhausted;
  });
}

void testInvalidTimeAdmissionAndIdentityFailClosed() {
  for (double invalid :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    Fixture fixture;
    fixture.activate(fixture.request());
    const auto frame = fixture.frameInput(invalid, 1000.0);
    const auto event = fixture.recoveryEvent();
    auto failed = fixture.coordinator.handleAutonomyOutcome(frame, event);
    require(failed.terminal_after_stop.has_value() &&
                fixture.coordinator.snapshot().budget_consumed,
            "invalid time did not consume the current task budget");
  }

  Fixture rollback;
  rollback.activate(rollback.request());
  (void)rollback.coordinator.advancePlanningCycle(rollback.frameInput(220.0, 1000.0));
  auto regressed = rollback.coordinator.advancePlanningCycle(rollback.frameInput(219.0, 1001.0));
  require(regressed.terminal_after_stop.has_value() &&
              rollback.coordinator.snapshot().budget_consumed,
          "clock rollback did not fail closed");

  Fixture missing;
  missing.activate(missing.request());
  const auto missing_frame = missing.frameInput(230.0);
  auto missing_event = missing.recoveryEvent();
  missing_event.goal_snapshot.active_map_identity.reset();
  auto missing_result = missing.coordinator.handleAutonomyOutcome(missing_frame, missing_event);
  require(missing_result.terminal_after_stop.has_value() &&
              missing.coordinator.snapshot().task_id == "task-a" &&
              missing.coordinator.snapshot().budget_consumed,
          "missing captured identity did not observe then consume current task");

  Fixture invalid_position;
  invalid_position.activate(invalid_position.request());
  require(invalid_position.arm(240.0).reason == "backoff_pending",
          "invalid admission fixture did not arm");
  auto invalid_input = invalid_position.frameInput(240.5);
  invalid_input.fresh_admission.map_position->x = std::numeric_limits<double>::quiet_NaN();
  auto invalid_result = invalid_position.coordinator.advancePlanningCycle(invalid_input);
  require(invalid_result.terminal_after_stop.has_value() &&
              invalid_result.reason == "invalid_replan_admission",
          "invalid fresh position did not fail closed");

  Fixture map_changed;
  map_changed.activate(map_changed.request());
  require(map_changed.arm(250.0).reason == "backoff_pending", "map-change fixture did not arm");
  map_changed.map_identity.artifact_sha256 = "sha256-b";
  auto changed = map_changed.coordinator.advancePlanningCycle(map_changed.frameInput(250.5));
  require(changed.terminal_after_stop.has_value() &&
              changed.reason == "active_map_changed_before_replan",
          "map identity change did not fail closed");
}

void testActiveShutdownCreatesProjectedCancelledIntentAndExactDuplicateReplay() {
  Fixture fixture;
  fixture.activate(fixture.request());

  const auto shutdown =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kShutdown, 260.0);
  require(shutdown.handled && shutdown.interrupted && shutdown.reason == "navd_shutdown",
          "active shutdown did not produce the canonical interruption reason");
  require(shutdown.terminal_intent_id != 0U && shutdown.terminal_after_stop.has_value(),
          "active shutdown did not create a terminal intent");
  require(shutdown.terminal_stop_policy == TerminalStopPolicy::kShutdown,
          "active shutdown did not retain the shutdown stop policy");
  const auto &ticket = shutdown.terminal_after_stop->delivery_ticket;
  require(ticket.statuses.size() == 1U && ticket.statuses.front().task_id == "task-a" &&
              ticket.statuses.front().request_id == "request-a" &&
              ticket.statuses.front().state == NavigationGoalState::Cancelled &&
              ticket.statuses.front().reason == "navd_shutdown" &&
              ticket.statuses.front().project_to_navigation_state,
          "active shutdown terminal did not preserve projected Cancelled/navd_shutdown identity");

  const auto duplicate =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kShutdown, 260.1);
  require(
      duplicate.terminal_intent_id == shutdown.terminal_intent_id &&
          duplicate.terminal_stop_policy == TerminalStopPolicy::kShutdown &&
          duplicate.terminal_after_stop.has_value() &&
          sameStatuses(duplicate.terminal_after_stop->delivery_ticket.statuses, ticket.statuses),
      "duplicate shutdown did not replay the exact intent, policy, and ticket");
}

void testPlanningOnlyShutdownCreatesNonProjectedCancelledIntent() {
  Fixture fixture;
  require(fixture.goal_plan.submit(fixture.request(), fixture.admission()).accepted,
          "planning-only shutdown fixture did not accept the goal");
  const auto before = fixture.goal_plan.snapshot();
  require(!before.planning_task_id.empty() && before.active_task_id.empty(),
          "planning-only shutdown fixture unexpectedly activated the goal");

  const auto shutdown =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kShutdown, 270.0);
  require(shutdown.terminal_intent_id != 0U && shutdown.terminal_after_stop.has_value() &&
              shutdown.terminal_stop_policy == TerminalStopPolicy::kShutdown,
          "planning-only shutdown did not create a shutdown terminal intent");
  const auto &statuses = shutdown.terminal_after_stop->delivery_ticket.statuses;
  require(statuses.size() == 1U && statuses.front().task_id == "task-a" &&
              statuses.front().request_id == "request-a" &&
              statuses.front().state == NavigationGoalState::Cancelled &&
              statuses.front().reason == "navd_shutdown" &&
              !statuses.front().project_to_navigation_state,
          "planning-only shutdown fabricated an execution-projected terminal");
}

void testShutdownWithoutGoalCreatesNoTerminalIntent() {
  Fixture fixture;
  const auto shutdown =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kShutdown, 280.0);
  require(shutdown.handled && shutdown.interrupted && shutdown.reason == "navd_shutdown",
          "goal-free shutdown did not record the synchronous shutdown fact");
  require(shutdown.terminal_intent_id == 0U && !shutdown.terminal_after_stop.has_value() &&
              !fixture.coordinator.terminalPending(),
          "goal-free shutdown created a fake terminal intent");
}

void testShutdownPreservesExistingPendingTerminalAndPolicy() {
  Fixture fixture;
  fixture.activate(fixture.request());
  const auto existing =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kMapDrift, 290.0);
  require(existing.terminal_intent_id != 0U && existing.terminal_after_stop.has_value(),
          "existing-pending shutdown fixture did not create the original terminal");

  const auto shutdown =
      fixture.coordinator.interrupt(GoalReplanRuntimeInterruption::kShutdown, 290.1);
  require(shutdown.terminal_intent_id == existing.terminal_intent_id &&
              shutdown.terminal_stop_policy == existing.terminal_stop_policy &&
              shutdown.terminal_after_stop.has_value() &&
              sameStatuses(shutdown.terminal_after_stop->delivery_ticket.statuses,
                           existing.terminal_after_stop->delivery_ticket.statuses),
          "shutdown replaced an existing pending terminal fact or its policy");
}

void testShutdownExitDecisionRequiresConfirmedStopAndExactDelivery() {
  struct Case {
    const char *name;
    bool stop_confirmed;
    bool terminal_required;
    bool terminal_pending;
    bool delivery_acknowledged;
    bool allow_exit;
  };
  const Case cases[] = {
      {"unconfirmed_physical_stop_stays_pending", false, false, false, false, false},
      {"unconfirmed_stop_cannot_be_overridden_by_delivery_ack", false, true, false, true, false},
      {"confirmed_physical_only_shutdown_may_exit", true, false, false, false, true},
      {"physical_only_with_impossible_pending_terminal_must_not_exit", true, false, true, false,
       false},
      {"physical_only_pending_terminal_ignores_delivery_ack", true, false, true, true, false},
      {"committed_writer_pending_terminal_must_not_exit", true, true, true, false, false},
      {"terminal_not_pending_but_not_acknowledged_must_not_exit", true, true, false, false, false},
      {"confirmed_and_acknowledged_terminal_may_exit", true, true, false, true, true},
  };

  for (const Case &entry : cases) {
    const auto decision = decideShutdownExit(entry.stop_confirmed, entry.terminal_required,
                                             entry.terminal_pending, entry.delivery_acknowledged);
    require(decision.allow_exit == entry.allow_exit, entry.name);
  }
}

GoalReplanRuntimeResult terminalResult() {
  GoalReplanRuntimeResult result;
  result.terminal_intent_id = 42U;
  result.terminal_after_stop =
      GoalPlanTerminalAfterStop{"stopped", GoalPlanTerminalDeliveryTicket{}, [] {}};
  return result;
}

void testGoalTerminalSchedulingDecisionTable() {
  struct Case {
    const char *name;
    bool surfaced_terminal;
    bool terminal_pending;
    bool expect_service_terminal;
    bool expect_autonomy_tick;
  };

  const Case cases[] = {
      {"no_terminal_and_not_pending_allows_autonomy", false, false, false, true},
      {"surfaced_terminal_services_and_freezes_autonomy_this_tick", true, false, true, false},
      {"surfaced_terminal_while_pending_services_and_freezes_autonomy", true, true, true, false},
      {"pending_without_surface_skips_service_and_autonomy", false, true, false, false},
  };

  for (const Case &entry : cases) {
    GoalReplanRuntimeResult result;
    if (entry.surfaced_terminal) {
      result = terminalResult();
    }

    const GoalTerminalSchedulingDecision decision =
        decideGoalTerminalScheduling(result, entry.terminal_pending);

    require(decision.service_terminal == entry.expect_service_terminal, entry.name);
    require(decision.run_autonomy_tick == entry.expect_autonomy_tick, entry.name);
    if (entry.surfaced_terminal) {
      require(!decision.run_autonomy_tick,
              "surfaced terminal must freeze autonomy even if service succeeds this tick");
    }
  }
}

}  // namespace

int main() {
  testNormalReplanChainHasNoIntermediateTerminal();
  testLegacyRecoveryReasonWithoutTypedTriggerDoesNotArm();
  testPersistentObstructionReplaysExactOverlayAfterStopAndBackoff();
  testHandleAutonomyOutcomeDoesNotAdvanceOrResume();
  testAdvancePlanningCycleProgressesWithoutAutonomyOutcome();
  testRecoveryFailureOnlyArmsUntilAdvanceDeadline();
  testPendingTerminalBlocksAllRuntimePhases();
  testDrainPendingCycleDoesNotAdvanceOrStartRetry();
  testSteadyClockOwnsRetryDeadlineAcrossWallRegression();
  testInvalidWallTimeFailsClosedBeforePlannerAdvance();
  testExternalGoalReachedDefersStableTerminalUntilStop();
  testExternalGoalReachedIntentReplaysUntilExactAck();
  testStaleGoalReachedIdentityIsIgnored();
  testGoalReachedMapDriftFailsClosed();
  testGoalReachedHonorsCurrentMapDriftAndControlHold();
  testGoalReachedRequiresStableCapturedAndCurrentPlan();
  testReachedClosesAThenResumesPendingB();
  testReplacementCompletionDefersBActivationUntilOldActiveTerminalAck();
  testReplacementActivationAfterAckRechecksFreshEstopAdmission();
  testReplacementActivationAfterAckRechecksStoredMapIdentity();
  testDeferredReplacementInterruptionsCloseBWithExactNonProjectingTerminals();
  testTypedTaskCancelRoutesHiddenReplacementWithoutCommandIdentityLeak();
  testTypedTaskCancelWaitsForForeignTerminalThenRetriesExactHiddenB();
  testTypedTaskCancelDuringInitialPlanningUsesTicketedRuntimeTerminal();
  testTypedTaskCancelRoutesActiveTaskThroughTicketedRuntimeTerminal();
  testTypedTaskCancelRoutesQueuedAndReplacementPlanningWithoutStoppingActiveA();
  testTypedTaskCancelRejectsForeignTaskWithoutLifecycleEffects();
  testInspectionOutcomesSurfaceTicketedTerminalsAndRollingStaysExcluded();
  testStopAndZeroFailuresOnlyReturnTerminalIntents();
  testPlannerFailureConsumesBudgetAndDefersTerminal();
  testTerminalIntentPersistsUntilExactSuccessfulAck();
  testReplayPendingTerminalIsReadOnlyExactSingleOwnerView();
  testOnlyNewTaskRestoresRetryBudget();
  testPendingGoalDrainsAndResumesWithFreshBudget();
  testPendingRejectionClosesBThenDefersA();
  testReplacementPlannerFailureDefersOldActiveTerminal();
  testAllInterruptionsConsumeActiveAttempt();
  testInspectionInterruptionsProduceCancelledTicketsWithDiagnosticReasons();
  testIneligibleOutcomesNeverTakeOver();
  testInvalidTimeAdmissionAndIdentityFailClosed();
  testActiveShutdownCreatesProjectedCancelledIntentAndExactDuplicateReplay();
  testPlanningOnlyShutdownCreatesNonProjectedCancelledIntent();
  testShutdownWithoutGoalCreatesNoTerminalIntent();
  testShutdownPreservesExistingPendingTerminalAndPolicy();
  testShutdownExitDecisionRequiresConfirmedStopAndExactDelivery();
  testGoalTerminalSchedulingDecisionTable();
  return 0;
}
