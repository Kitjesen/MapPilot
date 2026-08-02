#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "motion/autonomy_tick_controller.hpp"
#include "motion/motion_stop_coordinator.hpp"
#include "plan/active_path_blockage_policy.hpp"
#include "plan/goal_replan_runtime_coordinator.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::ActivePathBlockageObservation;
using lingtu::nav::endpoint::ActivePathBlockagePolicy;
using lingtu::nav::endpoint::ActivePathBlockagePolicyConfig;
using lingtu::nav::endpoint::AutonomyTickActions;
using lingtu::nav::endpoint::AutonomyTickController;
using lingtu::nav::endpoint::AutonomyTickInput;
using lingtu::nav::endpoint::AutonomyTickOutcomeKind;
using lingtu::nav::endpoint::AutonomyTickPlannerInputs;
using lingtu::nav::endpoint::AutonomyTickResult;
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
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopCoordinator;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;
using lingtu::nav::plan::GlobalPlanBlockedRegion;
using lingtu::nav::plan::GlobalPlanRequest;
using lingtu::nav::plan::GlobalPlanResult;
using lingtu::nav::plan::GlobalPlanTemporaryOverlay;
using lingtu::nav::plan::MapIdentity;

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_active_path_replan_cycle: FAIL: %s\n", message);
    std::exit(1);
  }
}

bool sameRegion(const GlobalPlanBlockedRegion &lhs, const GlobalPlanBlockedRegion &rhs) {
  return lhs.center.x == rhs.center.x && lhs.center.y == rhs.center.y &&
         lhs.center.z == rhs.center.z && lhs.radius_xy_m == rhs.radius_xy_m &&
         lhs.min_z == rhs.min_z && lhs.max_z == rhs.max_z;
}

bool sameOverlay(const GlobalPlanTemporaryOverlay &lhs, const GlobalPlanTemporaryOverlay &rhs) {
  if (lhs.revision != rhs.revision || lhs.frame_epoch != rhs.frame_epoch ||
      lhs.obstacle_generation != rhs.obstacle_generation ||
      lhs.traversability_generation != rhs.traversability_generation ||
      lhs.blocked_regions.size() != rhs.blocked_regions.size()) {
    return false;
  }
  for (std::size_t index = 0; index < lhs.blocked_regions.size(); ++index) {
    if (!sameRegion(lhs.blocked_regions[index], rhs.blocked_regions[index])) {
      return false;
    }
  }
  return true;
}

struct Fixture {
  static constexpr std::uint64_t kFrameEpoch = 3U;

  MapIdentity map_identity{"field", 7, "sha256-a", "map"};
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
  int nav_loop_calls{0};
  int path_safety_calls{0};
  int command_safety_calls{0};
  int stop_linear_motion_calls{0};
  CommandSafetyConfig safety;
  std::optional<nav_kernel::Pose> map_body{nav_kernel::Pose{}};
  InputGateState input_gate;
  TraversabilityGrid traversability;
  LocalDiagnostics local;
  TimingDiagnostics timing;
  std::vector<float> unused_planner_obstacles;
  GoalPlanController goal_plan;
  MotionStopCoordinator motion_stop;
  GoalReplanRuntimeCoordinator coordinator;
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
              result.overlay_traversability_generation =
                  request.temporary_overlay.traversability_generation;
              result.path = {request.start, request.goal};
              return result;
            },
            goalActions()),
        motion_stop(true, stopActions()),
        coordinator(goal_plan, motion_stop, BoundedGoalReplanConfig{0.5}),
        autonomy_tick(autonomyActions()) {
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
    actions.record_stop_evidence_failure = [this](const std::string &reason) {
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
    actions.resume_autonomy = [] { return true; };
    actions.cancel_inspection = [](const std::string &) {};
    actions.clear_operator_resume_required = [] {};
    actions.set_autonomy_request_not_before = [](double) {};
    actions.persist_estop_latch = [](const std::string &) { return true; };
    actions.clear_persisted_estop_latch = [] { return true; };
    actions.publish_zero = [this] {
      ++keep_zero_calls;
      return true;
    };
    actions.last_output_sequence = [] { return 17U; };
    actions.publish_sequenced_zero = [] { return std::optional<std::uint64_t>{18U}; };
    actions.confirm_zero = [this](std::uint64_t) { return confirmation; };
    actions.clear_global_path = [] {};
    return actions;
  }

  AutonomyTickActions autonomyActions() {
    AutonomyTickActions actions;
    actions.steady_now_s = [] { return 11.0; };
    actions.current_map_identity = [this] { return GoalPlanMapIdentityResult{map_identity, {}}; };
    actions.compute_planner_inputs = [this](TimingDiagnostics &) {
      ++planner_input_calls;
      return AutonomyTickPlannerInputs{false, {}, &unused_planner_obstacles};
    };
    actions.tick_autonomy = [this](const nav_kernel::Pose &, const float *, int, double,
                                   lingtu::nav::plan::TraversabilityGridView) {
      ++nav_loop_calls;
      lingtu::nav::plan::NavLoopOutput output;
      output.cmd_vel = {0.4, 0.0, 0.2};
      return output;
    };
    actions.evaluate_path_safety =
        [this](const CommandSafetyConfig &, const nav_kernel::Twist &,
               const std::optional<nav_kernel::Pose> &, const std::vector<nav_kernel::Vec3> &,
               const std::vector<float> &, const TraversabilityGrid &, bool) {
          ++path_safety_calls;
          return CommandSafetyDecision{};
        };
    actions.evaluate_command_safety = [this](const CommandSafetyConfig &, const nav_kernel::Twist &,
                                             double, const std::optional<nav_kernel::Pose> &,
                                             const std::vector<float> &, const TraversabilityGrid &,
                                             bool) {
      ++command_safety_calls;
      return CommandSafetyDecision{};
    };
    actions.stop_linear_motion = [this] { ++stop_linear_motion_calls; };
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
      const auto result = goal_plan.advance(GoalPlanAdvanceContext{kFrameEpoch, false, 2.0});
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

  GoalReplanTrigger observePersistentBlockage() const {
    require(!activations.empty(), "blockage observation requires an active path");
    ActivePathBlockagePolicyConfig config;
    config.persistence_s = 1.0;
    config.minimum_fresh_observations = 3U;
    config.lookahead_m = 5.0;
    config.corridor_radius_m = 0.5;
    config.corridor_vertical_tolerance_m = 0.75;
    config.overlay_radius_m = 0.65;
    config.overlay_half_height_m = 1.25;
    config.max_regions = 8U;
    config.minimum_obstacle_points = 4U;
    ActivePathBlockagePolicy policy(config);
    const std::vector<float> blocked{
        1.92F, -0.08F, 0.2F, 0.4F, 1.92F, 0.08F, 0.2F, 0.4F,
        2.08F, -0.08F, 0.2F, 0.4F, 2.08F, 0.08F, 0.2F, 0.4F,
    };
    const auto identity = activeIdentity();
    auto observe = [&](double now_s, std::uint64_t cloud_generation,
                       std::uint64_t traversability_generation) {
      ActivePathBlockageObservation observation;
      observation.now_s = now_s;
      observation.external_active_goal = true;
      observation.goal = identity;
      observation.frame_epoch = kFrameEpoch;
      observation.robot_position = {0.0, 0.0, 0.0};
      observation.active_global_path = &activations.back().path;
      observation.live_obstacles_xyzh = &blocked;
      observation.cloud_generation = cloud_generation;
      observation.traversability_generation = traversability_generation;
      return policy.observe(observation);
    };

    require(!observe(10.0, 101U, 201U), "first fresh blockage triggered early");
    require(!observe(10.5, 102U, 202U), "second fresh blockage triggered early");
    const auto trigger = observe(11.0, 103U, 203U);
    require(trigger.has_value(), "persistent fresh blockage did not emit a trigger");
    require(trigger->kind == GoalReplanTriggerKind::kPersistentPathObstruction &&
                trigger->reason == "persistent_path_obstruction" &&
                lingtu::nav::endpoint::sameGoalReplanIdentity(trigger->goal, identity) &&
                !trigger->temporary_overlay.empty(),
            "persistent blockage did not produce a valid typed trigger");
    return *trigger;
  }

  AutonomyTickInput autonomyInput(const GoalReplanTrigger &trigger) {
    return {
        safety, map_body,       input_gate, true,   map_identity,     true,    false,
        true,   traversability, local,      timing, activeIdentity(), trigger,
    };
  }

  AutonomyTickResult runTriggeredAutonomy(const GoalReplanTrigger &trigger) {
    return autonomy_tick.tick(autonomyInput(trigger));
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

void requireTriggeredAutonomyStoppedBeforePlanning(const Fixture &fixture,
                                                   const AutonomyTickResult &result,
                                                   const GoalReplanTrigger &expected) {
  require(result.handled && result.clear_local_path && result.clear_local_planner_debug,
          "typed blockage did not take over the autonomy tick");
  require(fixture.planner_input_calls == 0 && fixture.nav_loop_calls == 0 &&
              fixture.path_safety_calls == 0 && fixture.command_safety_calls == 0,
          "typed blockage entered NavLoop or final safety");
  require(fixture.stop_linear_motion_calls == 1 && result.publish.cmd_vel &&
              result.publish.command.vx == 0.0 && result.publish.command.vy == 0.0 &&
              result.publish.command.wz == 0.0 && result.delta.cmd_vel_count == 1U,
          "typed blockage did not produce exactly one immediate zero command intent");
  require(
      result.outcome.kind == AutonomyTickOutcomeKind::kGoalFailed &&
          result.outcome.replan_trigger.has_value() &&
          lingtu::nav::endpoint::sameGoalReplanIdentity(result.outcome.replan_trigger->goal,
                                                        expected.goal) &&
          sameOverlay(result.outcome.replan_trigger->temporary_overlay, expected.temporary_overlay),
      "autonomy tick changed the typed trigger or its frozen overlay");
}

void testPersistentBlockageRunsOneAtomicReplacementCycle() {
  Fixture fixture;
  fixture.activateInitialPath();
  require(fixture.planner_calls.load() == 1, "initial planning call count changed");

  const GoalReplanTrigger trigger = fixture.observePersistentBlockage();
  const auto captured_snapshot = fixture.goal_plan.snapshot();
  const auto tick = fixture.runTriggeredAutonomy(trigger);
  requireTriggeredAutonomyStoppedBeforePlanning(fixture, tick, trigger);

  const GoalReplanRuntimeAutonomyEvent event{tick.outcome, captured_snapshot, false, false};
  const auto armed = fixture.coordinator.handleAutonomyOutcome(fixture.frame(30.0), event);
  require(armed.handled && armed.reason == "backoff_pending" && !armed.replan_started &&
              !armed.terminal_after_stop.has_value() && fixture.stop_control_calls == 1 &&
              fixture.clear_motion_calls == 1 && fixture.planner_calls.load() == 1,
          "confirmed stop did not arm exactly one bounded replacement cycle");

  const auto early = fixture.coordinator.advancePlanningCycle(fixture.frame(30.499));
  require(early.handled && early.reason == "backoff_pending" && early.zero_kept_fresh &&
              !early.replan_started && fixture.keep_zero_calls == 1 &&
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
  require(requests.size() == 2U && requests.front().temporary_overlay.empty() &&
              sameOverlay(requests.back().temporary_overlay, trigger.temporary_overlay),
          "the exact frozen overlay was not injected into only the replacement request");
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

void testStopConfirmationFailureRemainsFailClosed() {
  Fixture fixture;
  fixture.confirmation = StopConfirmationState::TimedOut;
  fixture.activateInitialPath();

  const GoalReplanTrigger trigger = fixture.observePersistentBlockage();
  const auto captured_snapshot = fixture.goal_plan.snapshot();
  const auto tick = fixture.runTriggeredAutonomy(trigger);
  requireTriggeredAutonomyStoppedBeforePlanning(fixture, tick, trigger);

  const GoalReplanRuntimeAutonomyEvent event{tick.outcome, captured_snapshot, false, false};
  const auto failed = fixture.coordinator.handleAutonomyOutcome(fixture.frame(40.0), event);
  require(failed.handled && failed.reason == "stop_confirmation_timeout_goal_replan_pending" &&
              !failed.replan_started && failed.terminal_after_stop.has_value() &&
              failed.terminal_intent_id != 0U && fixture.coordinator.terminalPending(),
          "failed stop confirmation did not leave an exact terminal barrier pending");
  require(fixture.stop_control_calls == 1 && fixture.clear_motion_calls == 1 &&
              fixture.stop_evidence_failure_calls == 1 &&
              fixture.last_stop_evidence_failure ==
                  "stop_confirmation_timeout_goal_replan_pending" &&
              fixture.planner_calls.load() == 1 && fixture.activations.size() == 1U,
          "failed stop confirmation was not held fail-closed before replanning");
  const auto retry = fixture.coordinator.snapshot();
  require(retry.state == BoundedGoalReplanState::kAttemptConsumed && retry.budget_consumed,
          "failed stop confirmation left the replan budget reusable");

  const auto scheduling =
      decideGoalTerminalScheduling(failed, fixture.coordinator.terminalPending());
  require(scheduling.service_terminal && !scheduling.run_autonomy_tick,
          "pending stop-failure terminal did not suppress the next autonomy tick");
  const auto replay = fixture.coordinator.advancePlanningCycle(fixture.frame(40.5));
  require(replay.terminal_intent_id == failed.terminal_intent_id &&
              replay.terminal_after_stop.has_value() && !replay.replan_started &&
              fixture.planner_calls.load() == 1 && fixture.coordinator.terminalPending(),
          "stop-failure terminal was not replayed exactly without starting a planner");
}

}  // namespace

int main() {
  testPersistentBlockageRunsOneAtomicReplacementCycle();
  testStopConfirmationFailureRemainsFailClosed();
  return 0;
}
