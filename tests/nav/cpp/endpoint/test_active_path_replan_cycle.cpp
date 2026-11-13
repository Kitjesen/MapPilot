#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "control/autonomy.hpp"
#include "safety/stop.hpp"
#include "app/goal/runtime.hpp"
#include "status/goal_terminal_status_delivery.hpp"
#include "status/navigation_goal_status_outbox.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::AutonomyTickActions;
using lingtu::nav::endpoint::AutonomyTickController;
using lingtu::nav::endpoint::AutonomyTickInput;
using lingtu::nav::endpoint::AutonomyTickOutcomeKind;
using lingtu::nav::endpoint::PlanView;
using lingtu::nav::endpoint::AutonomyTickResult;
using lingtu::nav::endpoint::FinalControl;
using lingtu::nav::endpoint::FinalActions;
using lingtu::nav::endpoint::BoundedGoalReplanConfig;
using lingtu::nav::endpoint::BoundedGoalReplanState;
using lingtu::nav::endpoint::CommandSafetyConfig;
using lingtu::nav::endpoint::CommandSafetyDecision;
using lingtu::nav::endpoint::decideGoalTerminalScheduling;
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
using lingtu::nav::endpoint::GoalReplanIdentity;
using lingtu::nav::endpoint::GoalReplanRuntimeAutonomyEvent;
using lingtu::nav::endpoint::GoalReplanRuntimeCoordinator;
using lingtu::nav::endpoint::GoalReplanRuntimeFrameInput;
using lingtu::nav::endpoint::GoalReplanRuntimeResult;
using lingtu::nav::endpoint::GoalReplanTrigger;
using lingtu::nav::endpoint::GoalReplanTriggerKind;
using lingtu::nav::endpoint::GoalTerminalStatusDelivery;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopBarrier;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;
using lingtu::nav::plan::GlobalPlanRequest;
using lingtu::nav::plan::GlobalPlanResult;
using lingtu::nav::plan::MapIdentity;

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_active_path_replan_cycle: FAIL: %s\n", message);
    std::exit(1);
  }
}

struct Fixture {
  static constexpr std::uint64_t kFrameEpoch = 3U;

  MapIdentity map_identity{"field", 7, "map"};
  std::vector<GoalPlanStatus> statuses;
  std::vector<GoalPlanPathActivation> activations;
  mutable std::mutex planner_requests_mutex;
  std::vector<GlobalPlanRequest> planner_requests;
  std::atomic<int> planner_calls{0};
  StopConfirmationState confirmation{StopConfirmationState::Confirmed};
  int stop_control_calls{0};
  int clear_motion_calls{0};
  int keep_zero_calls{0};
  int stop_evidence_failure_calls{0};
  std::string last_stop_evidence_failure;
  int planner_input_calls{0};
  int motion_calls{0};
  int command_safety_calls{0};
  CommandSafetyConfig safety;
  std::optional<nav_kernel::Pose> map_body{nav_kernel::Pose{}};
  InputGateState input_gate;
  TraversabilityGrid traversability;
  LocalDiagnostics local;
  TimingDiagnostics timing;
  std::vector<float> unused_planner_obstacles;
  GoalPlanController goal_plan;
  MotionStopBarrier motion_stop;
  NavigationGoalStatusOutbox outbox;
  GoalTerminalStatusDelivery delivery;
  GoalReplanRuntimeCoordinator coordinator;
  FinalControl final_control;
  AutonomyTickController autonomy_tick;

  Fixture()
      : goal_plan(
            [this](const GlobalPlanRequest &request,
                   const lingtu::nav::plan::GlobalPlanCancelCheck &) {
              ++planner_calls;
              {
                std::lock_guard<std::mutex> lock(planner_requests_mutex);
                planner_requests.push_back(request);
              }
              GlobalPlanResult result;
              result.ok = true;
              result.reached_goal = true;
              result.map_identity = map_identity;
              result.overlay_revision = request.temporary_overlay.revision;
              result.overlay_frame_epoch = request.temporary_overlay.frame_epoch;
              result.overlay_obstacle_generation = request.temporary_overlay.obstacle_generation;
              result.path = {request.start, request.goal};
              return result;
            },
            goalActions()),
        motion_stop(true, stopActions()),
        outbox([](const GoalPlanStatus &) {}, [](const GoalPlanStatus &) { return true; }),
        delivery(outbox),
        coordinator(goal_plan, motion_stop, delivery,
                    GoalTerminalActions{[](const std::string &) {},
                                        [](const std::string &) {}, [] {}},
                    BoundedGoalReplanConfig{0.5}),
        final_control(finalControlActions()),
        autonomy_tick(autonomyActions(), final_control) {
    input_gate.ready = true;
    traversability.values = {0.0F};
    traversability.rows = 1;
    traversability.cols = 1;
    traversability.resolution = 0.2;
    traversability.generation = 201U;
  }

  GoalPlanActions goalActions() {
    GoalPlanActions actions;
    actions.preempt_rolling = [](const std::string &) { return true; };
    actions.clear_external_inspection = [] {};
    actions.current_map_identity = [this] { return GoalPlanMapIdentityResult{map_identity, {}}; };
    actions.publish_status = [this](const GoalPlanStatus &status) { statuses.push_back(status); };
    actions.inspection_active = [] { return false; };
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
    actions.hold = [this](const std::string &reason) {
      ++stop_evidence_failure_calls;
      last_stop_evidence_failure = reason;
    };
    actions.sync_goal_diagnostics = [] {};
    actions.rolling_segment_active = [] { return false; };
    actions.preempt_rolling_segment = [](const std::string &) { return true; };
    actions.clear_motion_outputs = [this](const std::string &) {
      ++clear_motion_calls;
      return true;
    };
    actions.suspend_motion_outputs = [](const std::string &) { return true; };
    actions.cancel_control = [] {};
    actions.stop_control = [this] { ++stop_control_calls; };
    actions.latch_estop = [](const std::string &) {};
    actions.clear_control_estop = [] { return true; };
    actions.resume_control = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};

    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [this] {
      ++keep_zero_calls;
      return true;
    };
    actions.last_output_sequence = [] { return 17U; };
    actions.publish_sequenced_zero = [] { return std::optional<std::uint64_t>{18U}; };
    actions.confirm_zero = [this](std::uint64_t, auto) { return confirmation; };
    actions.clear_global_path = [] {};
    return actions;
  }

  AutonomyTickActions autonomyActions() {
    AutonomyTickActions actions;
    actions.steady_now_s = [] { return 11.0; };
    actions.current_map_identity = [this] { return GoalPlanMapIdentityResult{map_identity, {}}; };
    actions.read_plan = [this](double, TimingDiagnostics &) {
      ++planner_input_calls;
      return PlanView{false, {}, &unused_planner_obstacles};
    };
    actions.tick_autonomy = [this](const nav_kernel::Pose &, const float *, int, double,
                                   lingtu::nav::navigation::TraversabilityGridView) {
      ++motion_calls;
      lingtu::nav::navigation::ExecutionOutput output;
      output.recovery_exhausted = true;
      output.reason = "local_recovery_exhausted";
      return output;
    };
    actions.pause_linear_motion = [] {};
    return actions;
  }

  FinalActions finalControlActions() {
    FinalActions actions;
    actions.command_safety = [this](const CommandSafetyConfig &,
                                    const nav_kernel::Twist &command) {
      ++command_safety_calls;
      CommandSafetyDecision decision;
      decision.should_publish = true;
      decision.cmd = command;
      decision.reason = "accepted";
      return decision;
    };
    actions.shape = [](const nav_kernel::Twist &command, double) {
      nav_kernel::VelocitySmootherOutput output;
      output.command = command;
      output.valid = true;
      return output;
    };
    actions.commit = [](const nav_kernel::Twist &, double) { return true; };
    actions.stop = [](double, const std::string &) {};
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
    context.frame_epoch = kFrameEpoch;
    return context;
  }

  GoalPlanRequest request() const {
    GoalPlanRequest result;
    result.task_id = "task-a";
    result.request_id = "request-a";
    result.origin = GoalPlanOrigin::kExternal;
    result.source_stamp_s = 1.0;
    result.target = GoalPlanTarget{nav_kernel::Vec3{4.0, 0.0, 0.0}, 0.0};
    return result;
  }

  GoalReplanRuntimeFrameInput frame(double steady_now_s) const {
    GoalReplanRuntimeFrameInput result;
    result.steady_now_s = steady_now_s;
    result.wall_now_s = steady_now_s;
    result.fresh_admission = admission();
    return result;
  }

  void activateInitialPath() {
    require(goal_plan.submit(request(), admission()).accepted, "initial goal was rejected");
    for (int attempt = 0; attempt < 2000; ++attempt) {
      const auto result = goal_plan.advance(GoalPlanAdvanceContext{kFrameEpoch, {}, 2.0});
      if (result.path_activated) {
        require(activations.size() == 1U, "initial path activated more than once");
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "initial plan did not activate");
  }

  GoalReplanIdentity activeIdentity() const {
    const auto snapshot = goal_plan.snapshot();
    require(snapshot.active_map_identity.has_value(), "active map identity is missing");
    return {snapshot.active_task_id, snapshot.active_request_id, snapshot.active_goal_epoch,
            *snapshot.active_map_identity};
  }

  GoalReplanTrigger localRecoveryTrigger() const {
    GoalReplanTrigger trigger;
    trigger.kind = GoalReplanTriggerKind::kLocalRecoveryExhausted;
    trigger.reason = "local_recovery_exhausted";
    trigger.goal = activeIdentity();
    return trigger;
  }

  AutonomyTickInput autonomyInput() {
    return {
        safety, map_body,       input_gate, true,   map_identity,     true,    false,
        true,   traversability, local,      timing, activeIdentity(),
    };
  }

  AutonomyTickResult runExhaustedAutonomy() {
    return autonomy_tick.tick(autonomyInput());
  }

  std::vector<GlobalPlanRequest> plannerRequests() const {
    std::lock_guard<std::mutex> lock(planner_requests_mutex);
    return planner_requests;
  }

  void waitForPlannerCalls(int expected) const {
    for (int attempt = 0; attempt < 2000; ++attempt) {
      if (planner_calls.load() >= expected) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "replacement planner call did not start");
  }

  GoalReplanRuntimeResult waitForReplacementActivation(double start_s) {
    for (int attempt = 0; attempt < 2000; ++attempt) {
      const auto result = coordinator.advancePlanningCycle(frame(start_s + attempt * 0.001));
      if (result.plan_advance.path_activated) {
        return result;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(false, "replacement path did not activate");
    return {};
  }
};

void requireRecoveryCompletedBeforeReplanning(const Fixture &fixture,
                                               const AutonomyTickResult &result,
                                               const GoalReplanTrigger &expected) {
  require(result.handled && result.output && result.output->recovery_exhausted,
          "local recovery did not run to exhaustion");
  require(fixture.planner_input_calls == 1 && fixture.motion_calls == 1 &&
              fixture.command_safety_calls == 0,
          "local execution was bypassed or exhaustion produced a motion command");
  require(result.publish.cmd_vel && result.publish.command.vx == 0.0 &&
              result.publish.command.vy == 0.0 && result.publish.command.wz == 0.0,
          "exhausted recovery did not hold zero velocity");
  require(result.outcome.kind == AutonomyTickOutcomeKind::kGoalFailed &&
              result.outcome.replan_trigger &&
              result.outcome.replan_trigger->kind == GoalReplanTriggerKind::kLocalRecoveryExhausted &&
              lingtu::nav::endpoint::sameGoalReplanIdentity(result.outcome.replan_trigger->goal,
                                                          expected.goal),
          "local execution should report exhaustion without deciding a global overlay");
}

void testRecoveryExhaustionRunsOneAtomicReplacementCycle() {
  Fixture fixture;
  fixture.activateInitialPath();
  require(fixture.planner_calls.load() == 1, "initial planning call count changed");

  const GoalReplanTrigger trigger = fixture.localRecoveryTrigger();
  const auto captured_snapshot = fixture.goal_plan.snapshot();
  const auto tick = fixture.runExhaustedAutonomy();
  requireRecoveryCompletedBeforeReplanning(fixture, tick, trigger);

  const GoalReplanRuntimeAutonomyEvent event{
      tick.outcome, captured_snapshot, false, false};
  const auto armed = fixture.coordinator.handleAutonomyOutcome(fixture.frame(30.0), event);
  require(armed.handled && armed.reason == "backoff_pending" && !armed.replan_started &&
              !armed.terminal_after_stop.has_value() && fixture.stop_control_calls == 0 &&
              fixture.clear_motion_calls == 0 && fixture.planner_calls.load() == 1,
          "local recovery did not arm exactly one bounded replacement cycle");

  const auto early = fixture.coordinator.advancePlanningCycle(fixture.frame(30.499));
  require(early.handled && early.reason == "backoff_pending" && early.zero_kept_fresh &&
              !early.replan_started && fixture.keep_zero_calls == 0 &&
              fixture.planner_calls.load() == 1,
          "replacement planning started before the 0.5 second backoff elapsed");

  const auto started = fixture.coordinator.advancePlanningCycle(fixture.frame(30.5));
  require(started.handled && started.reason == "replan_started" && started.replan_started,
          "replacement planning did not start at the bounded deadline");
  fixture.waitForPlannerCalls(2);

  const auto completed = fixture.waitForReplacementActivation(30.501);
  require(completed.reason == "replan_completed" && !completed.terminal_after_stop.has_value(),
          "successful replacement did not complete without an intermediate terminal");

  const auto requests = fixture.plannerRequests();
  require(requests.size() == 2U,
          "local recovery replan must submit exactly one replacement global request");
  require(fixture.activations.size() == 2U && fixture.statuses.size() == 4U &&
              fixture.statuses.back().state == NavigationGoalState::PathActive,
          "successful replacement path was not atomically activated");
  const auto final_snapshot = fixture.goal_plan.snapshot();
  require(final_snapshot.active_task_id == trigger.goal.task_id &&
              final_snapshot.active_request_id == trigger.goal.request_id &&
              final_snapshot.active_goal_epoch == trigger.goal.goal_epoch + 1U &&
              final_snapshot.goal_epoch == final_snapshot.active_goal_epoch && !final_snapshot.busy,
          "successful replacement changed goal ownership or did not advance its generation");
}

void testReplanningDoesNotWaitForStopConfirmation() {
  Fixture fixture;
  fixture.activateInitialPath();

  const GoalReplanTrigger trigger = fixture.localRecoveryTrigger();
  const auto captured_snapshot = fixture.goal_plan.snapshot();
  const auto tick = fixture.runExhaustedAutonomy();
  requireRecoveryCompletedBeforeReplanning(fixture, tick, trigger);

  const GoalReplanRuntimeAutonomyEvent event{
      tick.outcome, captured_snapshot, false, false};
  const auto armed = fixture.coordinator.handleAutonomyOutcome(fixture.frame(40.0), event);
  require(armed.handled && armed.reason == "backoff_pending" && !armed.replan_started &&
              !armed.terminal_after_stop.has_value() && fixture.stop_control_calls == 0 &&
              fixture.clear_motion_calls == 0 && fixture.stop_evidence_failure_calls == 0 &&
              fixture.planner_calls.load() == 1 && fixture.activations.size() == 1U,
          "ordinary replan still entered stop confirmation");
  const auto started = fixture.coordinator.advancePlanningCycle(fixture.frame(40.5));
  require(started.replan_started && !started.terminal_after_stop.has_value(),
          "ordinary replan did not start after the usual backoff");
  const auto completed = fixture.waitForReplacementActivation(40.701);
  require(completed.reason == "replan_completed" && fixture.activations.size() == 2U &&
              fixture.statuses.back().state == NavigationGoalState::PathActive,
          "retained destination did not recover after stop confirmation arrived");
}

void testSubsequentRecoveryUsesFreshGoalPlan() {
  Fixture fixture;
  fixture.activateInitialPath();
  const auto tick = fixture.runExhaustedAutonomy();
  const auto armed = fixture.coordinator.handleAutonomyOutcome(
      fixture.frame(13.0), {tick.outcome, fixture.goal_plan.snapshot(), false, false});
  require(armed.reason == "backoff_pending", "later exhaustion did not reach the task owner");
  const auto started = fixture.coordinator.advancePlanningCycle(fixture.frame(13.5));
  require(started.replan_started, "later exhaustion did not start a replacement");
  fixture.waitForReplacementActivation(13.501);
  const auto requests = fixture.plannerRequests();
  require(requests.size() == 2U,
          "subsequent local recovery must submit a fresh replacement request");
}

}  // namespace

int main() {
  testRecoveryExhaustionRunsOneAtomicReplacementCycle();
  testReplanningDoesNotWaitForStopConfirmation();
  testSubsequentRecoveryUsesFreshGoalPlan();
  return 0;
}
