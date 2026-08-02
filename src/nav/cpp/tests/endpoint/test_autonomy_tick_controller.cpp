#include <cmath>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "motion/autonomy_tick_controller.hpp"

namespace {

using lingtu::nav::endpoint::AutonomyTickActions;
using lingtu::nav::endpoint::AutonomyTickController;
using lingtu::nav::endpoint::AutonomyTickInput;
using lingtu::nav::endpoint::AutonomyTickOutcomeKind;
using lingtu::nav::endpoint::AutonomyTickPlannerInputs;
using lingtu::nav::endpoint::CommandSafetyConfig;
using lingtu::nav::endpoint::CommandSafetyDecision;
using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
using lingtu::nav::endpoint::GoalReplanIdentity;
using lingtu::nav::endpoint::GoalReplanTrigger;
using lingtu::nav::endpoint::GoalReplanTriggerKind;
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool near(double lhs, double rhs) {
  return std::abs(lhs - rhs) < 1e-9;
}

struct Fixture {
  CommandSafetyConfig safety;
  std::optional<nav_kernel::Pose> map_body{nav_kernel::Pose{}};
  InputGateState gate;
  TraversabilityGrid traversability;
  LocalDiagnostics previous;
  lingtu::nav::plan::MapIdentity active_identity{"field", 7, "sha256-a", "map"};
  GoalPlanMapIdentityResult current_map{active_identity, {}};
  TimingDiagnostics timing;
  std::vector<float> obstacles{1.0F, 2.0F, 0.3F, 1.0F, 3.0F, 4.0F, 0.4F, 1.0F};
  lingtu::nav::plan::NavLoopOutput next_output;
  CommandSafetyDecision path_safety;
  CommandSafetyDecision rotation_safety;
  int current_map_calls{0};
  int compute_calls{0};
  int tick_calls{0};
  int path_safety_calls{0};
  int command_safety_calls{0};
  int stop_calls{0};
  const float *tick_obstacles{nullptr};
  int tick_obstacle_count{-1};
  double tick_stamp{-1.0};
  std::uint64_t tick_traversability_generation{0};
  AutonomyTickActions actions;

  Fixture() {
    gate.ready = true;
    traversability.values = {10.0F};
    traversability.rows = 1;
    traversability.cols = 1;
    traversability.resolution = 0.2;
    traversability.generation = 7;
    previous.goal_reached = true;
    previous.target = {9.0, 8.0, 7.0};
    previous.slow_down = 3;
    path_safety.cmd = {0.2, 0.0, 0.0};
    path_safety.reason = "safe";
    rotation_safety.cmd = {0.0, 0.0, 0.25};
    rotation_safety.reason = "rotation_safe";
    actions.steady_now_s = [] { return 42.0; };
    actions.current_map_identity = [&] {
      ++current_map_calls;
      return current_map;
    };
    actions.compute_planner_inputs = [&](TimingDiagnostics &observed_timing) {
      require(&observed_timing == &timing,
              "planner input callback must receive the endpoint timing object");
      ++compute_calls;
      return AutonomyTickPlannerInputs{true, traversability.view(), &obstacles};
    };
    actions.tick_autonomy = [&](const nav_kernel::Pose &, const float *obstacle_data,
                                int obstacle_count, double stamp,
                                lingtu::nav::plan::TraversabilityGridView view) {
      ++tick_calls;
      tick_obstacles = obstacle_data;
      tick_obstacle_count = obstacle_count;
      tick_stamp = stamp;
      tick_traversability_generation = view.generation;
      return next_output;
    };
    actions.evaluate_path_safety = [&](const CommandSafetyConfig &, const nav_kernel::Twist &,
                                       const std::optional<nav_kernel::Pose> &,
                                       const std::vector<nav_kernel::Vec3> &,
                                       const std::vector<float> &observed_obstacles,
                                       const TraversabilityGrid &, bool traversability_fresh) {
      ++path_safety_calls;
      require(&observed_obstacles == &obstacles, "safety must borrow merged obstacle storage");
      require(traversability_fresh, "fresh traversability must reach final safety");
      return path_safety;
    };
    actions.evaluate_command_safety = [&](const CommandSafetyConfig &, const nav_kernel::Twist &,
                                          double, const std::optional<nav_kernel::Pose> &,
                                          const std::vector<float> &, const TraversabilityGrid &,
                                          bool) {
      ++command_safety_calls;
      return rotation_safety;
    };
    actions.stop_linear_motion = [&] { ++stop_calls; };
  }

  AutonomyTickInput input(bool path_active = true, bool motion_allowed = true, bool rolling = false,
                          bool publish = true,
                          std::optional<GoalReplanTrigger> precomputed_trigger = std::nullopt) {
    return {
        safety,
        map_body,
        gate,
        path_active,
        active_identity,
        motion_allowed,
        rolling,
        publish,
        traversability,
        previous,
        timing,
        GoalReplanIdentity{"task-a", "request-a", 11U, active_identity},
        std::move(precomputed_trigger),
    };
  }
};

void testIdleAndAuthorityDeniedDoNothing() {
  Fixture fixture;
  AutonomyTickController controller(fixture.actions);

  const auto idle = controller.tick(fixture.input(false));
  const auto denied = controller.tick(fixture.input(true, false));

  require(!idle.handled && !denied.handled,
          "inactive or authority-denied paths must stay untouched");
  require(fixture.current_map_calls == 0, "inactive branches must not read active map identity");
  require(fixture.compute_calls == 0 && fixture.tick_calls == 0,
          "inactive branches must not compute planner inputs");
}

void testBlockedInputGateFailsClosedWithoutPlanning() {
  Fixture fixture;
  fixture.gate.ready = false;
  fixture.gate.reason = "input_cloud_stale";
  AutonomyTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(result.handled, "active blocked path must be handled");
  require(result.clear_local_path && result.clear_local_planner_debug,
          "blocked input must clear stale path products");
  require(result.local.has_value(), "blocked input must update diagnostics");
  require(result.local->reason == "input_cloud_stale" &&
              result.local->final_safety_reason == "input_gate_input_cloud_stale",
          "blocked diagnostics must retain the gate reason");
  require(result.local->goal_reached && result.local->slow_down == 3 &&
              near(result.local->target.x, 9.0),
          "fields untouched by the legacy gate branch must be preserved");
  require(result.publish.cmd_vel && result.delta.cmd_vel_count == 1 &&
              result.delta.output_count == 0,
          "blocked input must request exactly one zero command publish");
  require(near(result.publish.command.vx, 0.0) && near(result.publish.command.wz, 0.0),
          "blocked input must carry an explicit zero command intent");
  require(fixture.compute_calls == 0 && fixture.tick_calls == 0,
          "blocked input must never run the planner");
}

void testActiveMapIdentityGuardFailsClosedBeforePlanning() {
  auto expect_blocked = [](Fixture &fixture, const char *expected_reason) {
    fixture.next_output.cmd_vel = {0.3, 0.0, 0.0};
    AutonomyTickController controller(fixture.actions);
    const auto result = controller.tick(fixture.input());

    require(result.handled, "map identity blocker must be handled");
    require(result.clear_local_path && result.clear_local_planner_debug,
            "map identity blocker must clear stale local products");
    require(fixture.current_map_calls == 1,
            "map identity blocker must read the current map directly");
    require(fixture.compute_calls == 0 && fixture.tick_calls == 0 && fixture.path_safety_calls == 0,
            "map identity blocker must not enter NavLoop or final safety");
    require(result.publish.cmd_vel && near(result.publish.command.vx, 0.0) &&
                near(result.publish.command.wz, 0.0),
            "map identity blocker must publish only zero");
    require(result.outcome.kind == AutonomyTickOutcomeKind::kGoalFailed &&
                result.outcome.reason == expected_reason,
            "map identity blocker must fail the active goal with a stable reason");
    require(result.local.has_value() && result.local->reason == expected_reason &&
                result.local->final_safety_reason == expected_reason &&
                result.local->final_safety_stopped,
            "map identity blocker diagnostics must retain stop evidence");
  };

  Fixture missing_active;
  missing_active.active_identity = {};
  expect_blocked(missing_active, "active_path_map_identity_missing");

  Fixture unavailable_current;
  unavailable_current.current_map.identity.reset();
  unavailable_current.current_map.reason = "active_map_lookup_failed";
  expect_blocked(unavailable_current, "active_map_unavailable_during_navigation");

  Fixture changed_map_id;
  changed_map_id.current_map.identity->map_id = "field-b";
  expect_blocked(changed_map_id, "active_map_changed_during_navigation");

  Fixture changed_version;
  changed_version.current_map.identity->version = 8;
  expect_blocked(changed_version, "active_map_changed_during_navigation");

  Fixture changed_hash;
  changed_hash.current_map.identity->artifact_sha256 = "sha256-b";
  expect_blocked(changed_hash, "active_map_changed_during_navigation");

  Fixture changed_frame;
  changed_frame.current_map.identity->frame_id = "odom";
  expect_blocked(changed_frame, "active_map_changed_during_navigation");
}

void testNormalTickProducesBorrowedInputIntentsAndDiagnostics() {
  Fixture fixture;
  fixture.next_output.active = true;
  fixture.next_output.path_found = true;
  fixture.next_output.reason = "tracking";
  fixture.next_output.slow_down = 1;
  fixture.next_output.recovery_state = 2;
  fixture.next_output.recovery_action = 1;
  fixture.next_output.recovery_attempt = 2;
  fixture.next_output.recovery_candidate_count = 7;
  fixture.next_output.recovery_verified = true;
  fixture.next_output.recovery_progress = 0.625;
  fixture.next_output.recovery_reason = "recovery_translation_active";
  fixture.next_output.recovery_exhausted = false;
  fixture.next_output.target_index = 4;
  fixture.next_output.target_distance_m = 1.25;
  fixture.next_output.target = {5.0, 6.0, 0.0};
  fixture.next_output.local_path_map = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  fixture.next_output.cmd_vel = {0.3, 0.0, 0.0};
  fixture.path_safety.cmd = {0.15, 0.0, 0.0};
  fixture.path_safety.slowed = true;
  fixture.path_safety.reason = "obstacle_slow";
  fixture.path_safety.obstacle_distance_m = 0.9;
  AutonomyTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(result.handled && result.output.has_value() && result.local.has_value(),
          "normal autonomy tick must expose its output");
  require(fixture.current_map_calls == 1 && fixture.compute_calls == 1 && fixture.tick_calls == 1 &&
              fixture.path_safety_calls == 1,
          "normal tick must compute, plan, and apply final safety once");
  require(fixture.tick_obstacles == fixture.obstacles.data() && fixture.tick_obstacle_count == 2,
          "planner must borrow the XYZH cloud without copying it");
  require(near(fixture.tick_stamp, 42.0) && fixture.tick_traversability_generation == 7,
          "planner must receive the injected clock and grid view");
  require(near(result.local->path_follower_cmd_vel.vx, 0.3) && near(result.local->cmd_vel.vx, 0.15),
          "diagnostics must distinguish follower and safety commands");
  require(result.local->final_safety_applied && result.local->final_safety_slowed &&
              result.local->final_safety_reason == "obstacle_slow",
          "final-safety diagnostics must be projected");
  require(result.local->recovery_state == 2 && result.local->recovery_action == 1 &&
              result.local->recovery_attempt == 2 && result.local->recovery_candidate_count == 7 &&
              result.local->recovery_verified && near(result.local->recovery_progress, 0.625) &&
              result.local->recovery_reason == "recovery_translation_active" &&
              !result.local->recovery_exhausted,
          "recovery diagnostics must be projected without collapsing the legacy state");
  require(result.publish.local_path && result.publish.waypoint && result.publish.cmd_vel,
          "normal tick must emit all transport intents");
  require(near(result.publish.command.vx, 0.15),
          "published command must carry the final-safety output");
  require(result.delta.cmd_vel_count == 1 && result.delta.output_count == 1 &&
              result.timing.nav_tick_measured,
          "normal tick must report counters and nav timing");
}

void testZeroCommandSkipsFinalSafety() {
  Fixture fixture;
  fixture.next_output.reason = "holding";
  fixture.next_output.cmd_vel = {};
  AutonomyTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(fixture.path_safety_calls == 0 && fixture.command_safety_calls == 0 &&
              fixture.stop_calls == 0,
          "zero command must bypass final safety");
  require(result.local.has_value() && !result.local->final_safety_applied &&
              result.local->final_safety_reason == "zero_command",
          "zero command must retain the legacy diagnostic reason");
}

void testGenericStopMayRetainSafeRotationOnly() {
  Fixture fixture;
  fixture.next_output.reason = "tracking";
  fixture.next_output.cmd_vel = {0.25, 0.0, 0.5};
  fixture.path_safety.stopped = true;
  fixture.path_safety.cmd = {};
  fixture.path_safety.reason = "obstacle_stop";
  fixture.rotation_safety.stopped = false;
  fixture.rotation_safety.cmd = {0.0, 0.0, 0.2};
  AutonomyTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(fixture.stop_calls == 1 && fixture.command_safety_calls == 1,
          "stopped command with yaw must evaluate rotation-only escape");
  require(result.output.has_value() && near(result.output->cmd_vel.wz, 0.2),
          "generic navigation may retain a safe rotation-only command");
  require(result.output->reason == "final_safety_obstacle_stop_rotation_only" &&
              result.local->final_safety_stopped && result.local->final_safety_limited,
          "rotation-only fallback must retain stopped and limited evidence");
  require(result.outcome.kind == AutonomyTickOutcomeKind::kNone,
          "generic final safety alone must not complete the goal");
}

void testRecoveryCommandIsVetoedByFinalSafety() {
  Fixture fixture;
  fixture.next_output.active = true;
  fixture.next_output.path_found = true;
  fixture.next_output.reason = "recovery_translation_active";
  fixture.next_output.recovery_state = 2;
  fixture.next_output.recovery_action = 1;
  fixture.next_output.recovery_verified = true;
  fixture.next_output.recovery_reason = "recovery_translation_active";
  fixture.next_output.local_path_map = {{0.0, 0.0, 0.0}, {0.0, 0.5, 0.0}};
  fixture.next_output.cmd_vel = {0.0, 0.2, 0.0};
  fixture.path_safety.stopped = true;
  fixture.path_safety.cmd = {};
  fixture.path_safety.reason = "obstacle_stop";
  AutonomyTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(fixture.path_safety_calls == 1 && fixture.command_safety_calls == 0,
          "recovery translation must pass through final path safety exactly once");
  require(result.output.has_value() && near(result.output->cmd_vel.vx, 0.0) &&
              near(result.output->cmd_vel.vy, 0.0) && near(result.output->cmd_vel.wz, 0.0),
          "final safety must be able to veto a verified recovery command");
  require(result.publish.cmd_vel && near(result.publish.command.vx, 0.0) &&
              near(result.publish.command.vy, 0.0) && near(result.publish.command.wz, 0.0),
          "only the vetoed zero command may be published");
  require(result.local.has_value() && result.local->recovery_verified &&
              result.local->recovery_state == 2 && result.local->final_safety_stopped &&
              result.local->final_safety_reason == "obstacle_stop",
          "status must retain both verified recovery and final-safety veto evidence");
}

void testRollingFinalSafetyHasHighestPriorityAndZerosRotation() {
  Fixture fixture;
  fixture.next_output.goal_reached = true;
  fixture.next_output.recovery_exhausted = true;
  fixture.next_output.cmd_vel = {0.25, 0.0, 0.5};
  fixture.path_safety.stopped = true;
  fixture.path_safety.cmd = {};
  fixture.path_safety.reason = "grid_stop";
  fixture.rotation_safety.stopped = false;
  fixture.rotation_safety.cmd = {0.0, 0.0, 0.2};
  AutonomyTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input(true, true, true));

  require(result.outcome.kind == AutonomyTickOutcomeKind::kRollingFinalSafetyStopped,
          "rolling final safety must outrank recovery and reached outcomes");
  require(result.outcome.reason == "segment_final_safety_grid_stop_rotation_only",
          "rolling outcome must carry the exact final-safety reason");
  require(result.output.has_value() && near(result.output->cmd_vel.vx, 0.0) &&
              near(result.output->cmd_vel.wz, 0.0),
          "rolling segment must discard rotation-only escape motion");
}

void testRecoveryOutcomesDistinguishRollingAndGenericGoals() {
  Fixture generic;
  generic.next_output.recovery_exhausted = true;
  generic.next_output.recovery_reason = "recovery_exhausted";
  generic.next_output.reason.clear();
  AutonomyTickController generic_controller(generic.actions);
  const auto generic_result = generic_controller.tick(generic.input());

  Fixture rolling;
  rolling.next_output.recovery_exhausted = true;
  rolling.next_output.reason = "planner_stuck";
  AutonomyTickController rolling_controller(rolling.actions);
  const auto rolling_result = rolling_controller.tick(rolling.input(true, true, true));

  require(generic_result.outcome.kind == AutonomyTickOutcomeKind::kGoalFailed &&
              generic_result.outcome.reason == "local_recovery_exhausted",
          "generic recovery exhaustion must fail the active goal");
  require(generic_result.outcome.replan_trigger.has_value() &&
              generic_result.outcome.replan_trigger->kind ==
                  GoalReplanTriggerKind::kLocalRecoveryExhausted &&
              generic_result.outcome.replan_trigger->reason == "local_recovery_exhausted" &&
              lingtu::nav::endpoint::sameGoalReplanIdentity(
                  generic_result.outcome.replan_trigger->goal,
                  GoalReplanIdentity{"task-a", "request-a", 11U, generic.active_identity}) &&
              generic_result.outcome.replan_trigger->temporary_overlay.empty(),
          "generic recovery exhaustion must carry one typed, identity-bound replan trigger");
  require(generic_result.local.has_value() && generic_result.local->recovery_exhausted &&
              generic_result.local->recovery_reason == "recovery_exhausted",
          "recovery exhaustion evidence must survive the endpoint projection");
  require(rolling_result.outcome.kind == AutonomyTickOutcomeKind::kRollingRecoveryExhausted &&
              rolling_result.outcome.reason == "planner_stuck" &&
              !rolling_result.outcome.replan_trigger.has_value(),
          "rolling recovery exhaustion must remain a segment outcome");
}

void testPrecomputedPersistentReplanBypassesPlannerAndFinalSafetyWithZeroCommand() {
  Fixture fixture;
  fixture.next_output.cmd_vel = {0.8, 0.0, 0.4};

  GoalReplanTrigger trigger;
  trigger.kind = GoalReplanTriggerKind::kPersistentPathObstruction;
  trigger.reason = "persistent_path_obstruction";
  trigger.goal = GoalReplanIdentity{"task-a", "request-a", 11U, fixture.active_identity};
  trigger.temporary_overlay.revision = 23U;
  trigger.temporary_overlay.frame_epoch = 3U;
  trigger.temporary_overlay.obstacle_generation = 41U;
  trigger.temporary_overlay.traversability_generation = 43U;
  trigger.temporary_overlay.blocked_regions = {
      {{1.25, -0.5, 0.2}, 0.65, -0.4, 1.6},
      {{2.75, 0.25, 0.3}, 0.55, -0.3, 1.7},
  };
  const GoalReplanTrigger expected = trigger;
  AutonomyTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input(true, true, false, true, trigger));

  require(result.handled && result.clear_local_path && result.clear_local_planner_debug,
          "persistent obstruction must synchronously take over the active tick");
  require(fixture.current_map_calls == 1 && fixture.compute_calls == 0 && fixture.tick_calls == 0 &&
              fixture.path_safety_calls == 0 && fixture.command_safety_calls == 0,
          "persistent obstruction must bypass NavLoop and every final-safety evaluator");
  require(fixture.stop_calls == 1 && !result.output.has_value() && result.publish.cmd_vel &&
              near(result.publish.command.vx, 0.0) && near(result.publish.command.vy, 0.0) &&
              near(result.publish.command.wz, 0.0) && result.delta.cmd_vel_count == 1U &&
              result.delta.output_count == 0U,
          "persistent obstruction must expose only one zero-command publish intent");
  require(result.local.has_value() && result.local->near_field_stop &&
              result.local->final_safety_stopped &&
              result.local->final_safety_reason == "persistent_path_obstruction",
          "persistent obstruction diagnostics must retain explicit stopped evidence");
  require(result.outcome.kind == AutonomyTickOutcomeKind::kGoalFailed &&
              result.outcome.replan_trigger.has_value(),
          "persistent obstruction must surface one typed replan outcome");

  const auto &actual = *result.outcome.replan_trigger;
  require(actual.kind == expected.kind && actual.reason == expected.reason &&
              lingtu::nav::endpoint::sameGoalReplanIdentity(actual.goal, expected.goal),
          "precomputed replan trigger kind, reason, or goal identity changed in the tick");
  require(actual.temporary_overlay.revision == expected.temporary_overlay.revision &&
              actual.temporary_overlay.frame_epoch == expected.temporary_overlay.frame_epoch &&
              actual.temporary_overlay.obstacle_generation ==
                  expected.temporary_overlay.obstacle_generation &&
              actual.temporary_overlay.traversability_generation ==
                  expected.temporary_overlay.traversability_generation &&
              actual.temporary_overlay.blocked_regions.size() ==
                  expected.temporary_overlay.blocked_regions.size(),
          "precomputed replan overlay identity changed in the tick");
  for (std::size_t i = 0; i < actual.temporary_overlay.blocked_regions.size(); ++i) {
    const auto &lhs = actual.temporary_overlay.blocked_regions[i];
    const auto &rhs = expected.temporary_overlay.blocked_regions[i];
    require(near(lhs.center.x, rhs.center.x) && near(lhs.center.y, rhs.center.y) &&
                near(lhs.center.z, rhs.center.z) && near(lhs.radius_xy_m, rhs.radius_xy_m) &&
                near(lhs.min_z, rhs.min_z) && near(lhs.max_z, rhs.max_z),
            "precomputed blocked region changed in the tick");
  }
}

void testReachedOutcomesDistinguishInspectionArrival() {
  Fixture generic;
  generic.next_output.goal_reached = true;
  AutonomyTickController generic_controller(generic.actions);
  const auto generic_result = generic_controller.tick(generic.input());

  Fixture rolling;
  rolling.next_output.goal_reached = true;
  AutonomyTickController rolling_controller(rolling.actions);
  const auto rolling_result = rolling_controller.tick(rolling.input(true, true, true));

  require(generic_result.outcome.kind == AutonomyTickOutcomeKind::kGoalReached &&
              generic_result.outcome.reason == "goal_reached" &&
              generic_result.outcome.inspection_arrival_intent,
          "generic reach must expose the inspection arrival intent");
  require(rolling_result.outcome.kind == AutonomyTickOutcomeKind::kRollingReached &&
              rolling_result.outcome.reason == "segment_reached" &&
              !rolling_result.outcome.inspection_arrival_intent,
          "rolling reach must not notify inspection as a generic arrival");
}

}  // namespace

int main() {
  try {
    testIdleAndAuthorityDeniedDoNothing();
    testBlockedInputGateFailsClosedWithoutPlanning();
    testActiveMapIdentityGuardFailsClosedBeforePlanning();
    testNormalTickProducesBorrowedInputIntentsAndDiagnostics();
    testZeroCommandSkipsFinalSafety();
    testGenericStopMayRetainSafeRotationOnly();
    testRecoveryCommandIsVetoedByFinalSafety();
    testRollingFinalSafetyHasHighestPriorityAndZerosRotation();
    testRecoveryOutcomesDistinguishRollingAndGenericGoals();
    testPrecomputedPersistentReplanBypassesPlannerAndFinalSafetyWithZeroCommand();
    testReachedOutcomesDistinguishInspectionArrival();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_autonomy_tick_controller: FAIL: %s\n", error.what());
    return 1;
  }
  return 0;
}
