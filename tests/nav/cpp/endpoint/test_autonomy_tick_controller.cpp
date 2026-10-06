#include <cmath>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "control/autonomy.hpp"
#include "runtime/loop.hpp"

namespace {

using lingtu::nav::endpoint::AutonomyTickActions;
using lingtu::nav::endpoint::AutonomyTickController;
using lingtu::nav::endpoint::AutonomyTickInput;
using lingtu::nav::endpoint::AutonomyTickOutcomeKind;
using lingtu::nav::endpoint::PlanView;
using lingtu::nav::endpoint::CommandSafetyConfig;
using lingtu::nav::endpoint::CommandSafetyDecision;
using lingtu::nav::endpoint::FinalControl;
using lingtu::nav::endpoint::FinalActions;
using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
using lingtu::nav::endpoint::GoalReplanIdentity;
using lingtu::nav::endpoint::GoalReplanTrigger;
using lingtu::nav::endpoint::GoalReplanTriggerKind;
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;
using lingtu::nav::endpoint::enforcePostPlanningInputReadiness;

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
  lingtu::nav::plan::MapIdentity active_identity{"field", 7, "map"};
  GoalPlanMapIdentityResult current_map{active_identity, {}};
  TimingDiagnostics timing;
  std::vector<float> obstacles{1.0F, 2.0F, 0.3F, 1.0F, 3.0F, 4.0F, 0.4F, 1.0F};
  lingtu::nav::navigation::ExecutionOutput next_output;
  nav_kernel::VelocitySmootherOutput shaped_velocity;
  int current_map_calls{0};
  int now_calls{0};
  int compute_calls{0};
  int tick_calls{0};
  int command_safety_calls{0};
  int shape_calls{0};
  int commit_calls{0};
  int velocity_stop_calls{0};
  int stop_calls{0};
  int pause_calls{0};
  double now_s{42.0};
  const float *tick_obstacles{nullptr};
  int tick_obstacle_count{-1};
  double tick_stamp{-1.0};
  double shape_stamp{-1.0};
  double commit_stamp{-1.0};
  double stop_stamp{-1.0};
  nav_kernel::Twist shaped_input{};
  nav_kernel::Twist command_safety_input{};
  bool verified_translation_seen{false};
  nav_kernel::Twist committed_command{};
  std::string velocity_stop_reason;
  bool commit_succeeds{true};
  bool shape_override{false};
  std::uint64_t tick_traversability_generation{0};
  AutonomyTickActions actions;
  FinalActions final_actions;
  std::optional<FinalControl> final_control;

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
    shaped_velocity.valid = true;
    actions.steady_now_s = [&] {
      ++now_calls;
      return now_s;
    };
    actions.current_map_identity = [&] {
      ++current_map_calls;
      return current_map;
    };
    actions.read_plan = [&](double now_s, TimingDiagnostics &observed_timing) {
      require(near(now_s, this->now_s), "planner input callback must receive the tick timestamp");
      require(&observed_timing == &timing,
              "planner input callback must receive the endpoint timing object");
      ++compute_calls;
      return PlanView{true, traversability.view(), &obstacles};
    };
    actions.tick_autonomy = [&](const nav_kernel::Pose &, const float *obstacle_data,
                                int obstacle_count, double stamp,
                                lingtu::nav::navigation::TraversabilityGridView view) {
      ++tick_calls;
      tick_obstacles = obstacle_data;
      tick_obstacle_count = obstacle_count;
      tick_stamp = stamp;
      tick_traversability_generation = view.generation;
      return next_output;
    };
    final_actions.command_safety =
        [&](const CommandSafetyConfig &config, const nav_kernel::Twist &command, double) {
      ++command_safety_calls;
      verified_translation_seen = config.verified_recovery_translation;
      command_safety_input = command;
      CommandSafetyDecision decision;
      decision.should_publish = true;
      decision.cmd = command;
      decision.reason = "accepted";
      return decision;
    };
    final_actions.shape = [&](const nav_kernel::Twist &command, double stamp) {
      ++shape_calls;
      shaped_input = command;
      shape_stamp = stamp;
      auto output = shaped_velocity;
      if (!shape_override) {
        output.command = command;
      }
      return output;
    };
    final_actions.commit = [&](const nav_kernel::Twist &command, double stamp) {
      ++commit_calls;
      committed_command = command;
      commit_stamp = stamp;
      return commit_succeeds;
    };
    final_actions.stop = [&](double stamp, const std::string &reason) {
      ++velocity_stop_calls;
      stop_stamp = stamp;
      velocity_stop_reason = reason;
    };
    actions.stop_linear_motion = [&] { ++stop_calls; };
    actions.pause_linear_motion = [&] { ++pause_calls; };
  }

  FinalControl &control() {
    final_control.emplace(final_actions);
    return *final_control;
  }

  AutonomyTickInput input(bool path_active = true, bool motion_allowed = true, bool rolling = false,
                          bool publish = true) {
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
    };
  }
};

void testIdleAndAuthorityDeniedDoNothing() {
  Fixture fixture;
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto idle = controller.tick(fixture.input(false));
  const auto denied = controller.tick(fixture.input(true, false));

  require(!idle.handled && !denied.handled,
          "inactive or authority-denied paths must stay untouched");
  require(fixture.current_map_calls == 0, "inactive branches must not read active map identity");
  require(fixture.pause_calls == 1, "authority denial pauses the retained trajectory");
  require(fixture.compute_calls == 0 && fixture.tick_calls == 0,
          "inactive branches must not compute planner inputs");
}

void testBlockedInputGateFailsClosedWithoutPlanning() {
  Fixture fixture;
  fixture.gate.ready = false;
  fixture.gate.reason = "input_cloud_stale";
  fixture.previous.tracking.active = true;
  fixture.previous.tracking.trajectoryId = 17;
  fixture.previous.tracking.executionTimeS = 1.25;
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.handled, "active blocked path must be handled");
  require(fixture.pause_calls == 1 && fixture.stop_calls == 0,
          "input hold must freeze planning and tracking without resetting the route");
  require(result.clear_local_path && result.clear_local_planner_debug,
          "blocked input must clear stale path products");
  require(result.local.has_value(), "blocked input must update diagnostics");
  require(result.local->reason == "input_cloud_stale" &&
              result.local->final_safety_reason == "input_gate_input_cloud_stale",
          "blocked diagnostics must retain the gate reason");
  require(result.local->goal_reached && result.local->slow_down == 3 &&
              near(result.local->target.x, 9.0),
          "fields untouched by the legacy gate branch must be preserved");
  require(result.local->tracking.active && result.local->tracking.trajectoryId == 17 &&
              near(result.local->tracking.executionTimeS, 1.25) &&
              result.local->tracking.executionFrozen,
          "input hold must report frozen progress for the retained trajectory");
  require(result.publish.cmd_vel && result.delta.cmd_vel_count == 1 &&
              result.delta.output_count == 0,
          "blocked input must request exactly one zero command publish");
  require(near(result.publish.command.vx, 0.0) && near(result.publish.command.wz, 0.0),
          "blocked input must carry an explicit zero command intent");
  require(fixture.compute_calls == 0 && fixture.tick_calls == 0,
          "blocked input must never run the planner");
  require(fixture.velocity_stop_calls == 1 &&
              fixture.velocity_stop_reason == "input_gate_input_cloud_stale",
          "blocked input must reset smoother state with the gate reason");
}

void testLocalizationHoldRecoversWithoutTerminalState() {
  Fixture fixture;
  fixture.gate.ready = false;
  fixture.gate.reason = "localization_not_tracking";
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto held = controller.tick(fixture.input());
  require(held.handled && held.publish.cmd_vel && near(held.publish.command.vx, 0.0) &&
              held.outcome.kind == AutonomyTickOutcomeKind::kNone && fixture.tick_calls == 0,
          "localization loss must hold motion without immediately terminating the goal");
  fixture.now_s += 20.0;
  fixture.gate.reason = "recovering";
  require(controller.tick(fixture.input()).outcome.kind == AutonomyTickOutcomeKind::kNone,
          "a temporary input hold must not become a terminal state");
  fixture.now_s += 9.0;
  fixture.gate.ready = true;
  fixture.gate.reason.clear();
  fixture.next_output.cmd_vel = {0.2, 0.0, 0.0};
  const auto resumed = controller.tick(fixture.input());
  require(resumed.outcome.kind == AutonomyTickOutcomeKind::kNone &&
              fixture.tick_calls == 1 && near(resumed.publish.command.vx, 0.2),
          "a healthy gate must resume through normal checked autonomy");

  fixture.now_s += 2.0;
  fixture.gate.ready = false;
  fixture.gate.reason = "localization_not_tracking";
  require(controller.tick(fixture.input()).outcome.kind == AutonomyTickOutcomeKind::kNone,
          "a goal may hold again after a later localization loss");
}

void testLocalizationHoldDoesNotExpireTheGoal() {
  for (const std::string gate_state : {"localization_not_tracking", "recovering",
                                       "cloud_stale", "ready"}) {
    Fixture fixture;
    fixture.gate.ready = false;
    fixture.gate.reason = "localization_not_tracking";
    AutonomyTickController controller(fixture.actions, fixture.control());
    controller.tick(fixture.input());
    fixture.now_s += 20.0;
    fixture.gate.reason = "cloud_stale";
    require(controller.tick(fixture.input()).outcome.kind == AutonomyTickOutcomeKind::kNone,
            "an intervening input gap must retain the localization recovery window");
    fixture.now_s += 10.0;
    fixture.gate.ready = gate_state == "ready";
    fixture.gate.reason = gate_state;
    fixture.next_output.cmd_vel = {0.2, 0.0, 0.0};
    const auto held_or_resumed = controller.tick(fixture.input());
    if (gate_state == "ready") {
      require(held_or_resumed.outcome.kind == AutonomyTickOutcomeKind::kNone &&
                  fixture.tick_calls == 1 && near(held_or_resumed.publish.command.vx, 0.2),
              "a recovered gate must resume the same goal after a long hold");
    } else {
      require(held_or_resumed.outcome.kind == AutonomyTickOutcomeKind::kNone &&
                  held_or_resumed.publish.cmd_vel && near(held_or_resumed.publish.command.vx, 0.0) &&
                  fixture.tick_calls == 0 && fixture.commit_calls == 0 &&
                  held_or_resumed.clear_local_path && held_or_resumed.local &&
                  held_or_resumed.local->reason == gate_state,
              "a long temporary input gap must hold zero without terminating the goal");
    }
  }
}

void testLocalizationHoldDoesNotCrossGoalOrControlChanges() {
  for (const std::string transition : {"new_request", "new_epoch", "new_map", "inactive",
                                       "authority_hold"}) {
    Fixture fixture;
    fixture.gate.ready = false;
    fixture.gate.reason = "localization_not_tracking";
    AutonomyTickController controller(fixture.actions, fixture.control());
    controller.tick(fixture.input());
    fixture.now_s += 20.0;
    auto changed = fixture.input();
    if (transition == "new_request") changed.active_goal_identity->request_id = "request-b";
    if (transition == "new_epoch") ++changed.active_goal_identity->goal_epoch;
    if (transition == "new_map") ++changed.active_goal_identity->map_identity.content_epoch;
    if (transition == "inactive") changed.path_active = false;
    if (transition == "authority_hold") changed.motion_allowed = false;
    controller.tick(changed);
    fixture.now_s += 11.0;
    changed.path_active = true;
    changed.motion_allowed = true;
    fixture.gate.reason = "localization_not_tracking";
    require(controller.tick(changed).outcome.kind == AutonomyTickOutcomeKind::kNone,
            "a new goal or control interruption must not inherit stale hold state");
  }
}

void testLocalizationHoldDoesNotTerminalizeRollingOrUnownedPaths() {
  for (const bool rolling : {false, true}) {
    Fixture fixture;
    fixture.gate.ready = false;
    fixture.gate.reason = "localization_not_tracking";
    AutonomyTickController controller(fixture.actions, fixture.control());
    auto input = fixture.input(true, true, rolling);
    if (!rolling) input.active_goal_identity.reset();
    controller.tick(input);
    fixture.now_s += 31.0;
    require(controller.tick(input).outcome.kind == AutonomyTickOutcomeKind::kNone,
            "a localization hold must not terminalize rolling or unowned paths");
  }
}

void testInputsExpiringDuringPlanningBlockPublicationWithoutCompletingGoal() {
  for (const char *reason : {"cloud_stale", "collision_stale", "odom_stale",
                             "driver_control_stale"}) {
    Fixture fixture;
    fixture.next_output.cmd_vel = {0.3, 0.1, 0.2};
    const auto planner = fixture.actions.tick_autonomy;
    fixture.actions.tick_autonomy = [&, planner](
        const nav_kernel::Pose &pose, const float *obstacles, int count, double stamp,
        lingtu::nav::navigation::TraversabilityGridView traversability) {
      auto result = planner(pose, obstacles, count, stamp, traversability);
      fixture.gate.ready = false;
      fixture.gate.reason = reason;
      return result;
    };
    AutonomyTickController controller(fixture.actions, fixture.control());
    const auto planned = controller.tick(fixture.input());
    require(planned.publish.cmd_vel && near(planned.publish.command.vx, 0.3),
            "the planner must start from ready inputs and produce a nonzero command");

    int hold_calls = 0;
    const auto readiness = enforcePostPlanningInputReadiness(
        planned.publish.command, fixture.gate, [&](const std::string &stop_reason) {
          ++hold_calls;
          require(stop_reason == (std::string(reason) == "collision_stale"
                                      ? "collision_stale"
                                      : std::string("input_gate_") + reason),
                  "the publication hold must expose the newly stale input");
          return true;
        });
    require(!readiness.allow_publish && readiness.stop_required &&
                readiness.stop_succeeded && hold_calls == 1,
            "data expiring during planning must hold motion before nonzero publication");
    require(planned.outcome.kind == AutonomyTickOutcomeKind::kNone && fixture.stop_calls == 0,
            "a temporary input hold must not complete or abort the autonomous goal");

    const auto failed_stop = enforcePostPlanningInputReadiness(
        planned.publish.command, fixture.gate, [](const std::string &) { return false; });
    require(!failed_stop.allow_publish && failed_stop.stop_required &&
                !failed_stop.stop_succeeded,
            "a failed zero publication must never allow the obsolete nonzero command");
    const auto zero = enforcePostPlanningInputReadiness(
        {}, fixture.gate, [&](const std::string &) {
          ++hold_calls;
          return false;
        });
    require(zero.allow_publish && !zero.stop_required && hold_calls == 1,
            "stale inputs must not suppress an explicit zero command");
  }
}

void testActiveMapIdentityGuardFailsClosedBeforePlanning() {
  auto expect_blocked = [](Fixture &fixture, const char *expected_reason,
                           bool terminal = true) {
    fixture.next_output.cmd_vel = {0.3, 0.0, 0.0};
    fixture.previous.tracking.active = true;
    fixture.previous.tracking.trajectoryId = 17;
    fixture.previous.tracking.executionTimeS = 1.25;
    AutonomyTickController controller(fixture.actions, fixture.control());
    const auto result = controller.tick(fixture.input());

    require(result.handled, "map identity blocker must be handled");
    require(result.clear_local_path && result.clear_local_planner_debug,
            "map identity blocker must clear stale local products");
    require(fixture.current_map_calls == 1,
            "map identity blocker must read the current map directly");
    require(fixture.compute_calls == 0 && fixture.tick_calls == 0 &&
                fixture.command_safety_calls == 0,
            "map identity blocker must not enter Executor or the command boundary");
    require(result.publish.cmd_vel && near(result.publish.command.vx, 0.0) &&
                near(result.publish.command.wz, 0.0),
            "map identity blocker must publish only zero");
    if (terminal) {
      require(result.outcome.kind == AutonomyTickOutcomeKind::kGoalFailed &&
                  result.outcome.reason == expected_reason,
              "map identity blocker must fail the active goal with a stable reason");
    } else {
      require(result.outcome.kind == AutonomyTickOutcomeKind::kNone,
              "a temporary map lookup gap must hold without terminating the active goal");
    }
    require(result.local.has_value() && result.local->reason == expected_reason &&
                result.local->final_safety_reason == expected_reason &&
                result.local->final_safety_stopped,
            "map identity blocker diagnostics must retain stop evidence");
    require(!result.local->tracking.active && result.local->tracking.trajectoryId == 0 &&
                near(result.local->tracking.executionTimeS, 0.0),
            "map identity failure must discard tracking diagnostics for the invalid route");
    require(fixture.velocity_stop_calls == 1 && fixture.velocity_stop_reason == expected_reason,
            "map identity blocker must reset smoother state");
  };

  Fixture missing_active;
  missing_active.active_identity = {};
  expect_blocked(missing_active, "active_path_map_identity_missing");

  Fixture unavailable_current;
  unavailable_current.current_map.identity.reset();
  unavailable_current.current_map.reason = "active_map_lookup_failed";
  expect_blocked(unavailable_current, "active_map_unavailable_during_navigation", false);

  Fixture changed_map_id;
  changed_map_id.current_map.identity->map_id = "field-b";
  expect_blocked(changed_map_id, "active_map_changed_during_navigation");

  Fixture changed_version;
  changed_version.current_map.identity->content_epoch = 8;
  expect_blocked(changed_version, "active_map_changed_during_navigation");

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
  fixture.next_output.recovery_rotation_target_rad = -0.6;
  fixture.next_output.recovery_verified = true;
  fixture.next_output.recovery_progress = 0.625;
  fixture.next_output.recovery_reason = "recovery_translation_active";
  fixture.next_output.recovery_exhausted = false;
  fixture.next_output.target_index = 4;
  fixture.next_output.target_distance_m = 1.25;
  fixture.next_output.target = {5.0, 6.0, 0.0};
  fixture.next_output.local_path_map = {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}};
  fixture.next_output.cmd_vel = {0.3, 0.0, 0.0};
  fixture.shape_override = true;
  fixture.shaped_velocity.command = {0.24, 0.0, 0.0};
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(result.handled && result.output.has_value() && result.local.has_value(),
          "normal autonomy tick must expose its output");
  require(fixture.now_calls == 1, "one autonomy tick must capture monotonic time exactly once");
  require(fixture.current_map_calls == 1 && fixture.compute_calls == 1 && fixture.tick_calls == 1 &&
              fixture.command_safety_calls == 1,
          "normal tick must plan once and apply command limits once");
  require(fixture.tick_obstacles == fixture.obstacles.data() && fixture.tick_obstacle_count == 2,
          "planner must borrow the XYZH cloud without copying it");
  require(near(fixture.tick_stamp, 42.0) && fixture.tick_traversability_generation == 7,
          "planner must receive the injected clock and grid view");
  require(fixture.shape_calls == 1 && near(fixture.shaped_input.vx, 0.3) &&
              near(fixture.shape_stamp, 42.0),
          "raw autonomy command must be shaped with the tick timestamp before safety");
  require(near(fixture.command_safety_input.vx, 0.24),
          "planned motion must reach a limits-only command boundary");
  require(fixture.commit_calls == 1 && near(fixture.committed_command.vx, 0.24) &&
              near(fixture.commit_stamp, 42.0),
          "the limits-only command must be committed with the same tick timestamp");
  require(fixture.pause_calls == 0,
          "accepted nonzero motion must keep the trajectory advancing");
  require(near(result.local->path_follower_cmd_vel.vx, 0.3) && near(result.local->cmd_vel.vx, 0.24),
          "diagnostics must distinguish follower and shaped commands");
  require(result.local->final_safety_applied && !result.local->final_safety_slowed &&
              result.local->final_safety_reason == "accepted",
          "command-boundary diagnostics must be projected");
  require(result.local->recovery_state == 2 && result.local->recovery_action == 1 &&
              result.local->recovery_attempt == 2 && result.local->recovery_candidate_count == 7 &&
              near(result.local->recovery_rotation_target_rad, -0.6) &&
              result.local->recovery_verified && near(result.local->recovery_progress, 0.625) &&
              result.local->recovery_reason == "recovery_translation_active" &&
              !result.local->recovery_exhausted,
          "recovery diagnostics must be projected without collapsing the legacy state");
  require(result.publish.local_path && result.publish.waypoint && result.publish.cmd_vel,
          "normal tick must emit all transport intents");
  require(near(result.publish.command.vx, 0.24),
          "published command must carry the limits-only output");
  require(result.delta.cmd_vel_count == 1 && result.delta.output_count == 1 &&
              result.timing.nav_tick_measured,
          "normal tick must report counters and nav timing");
}

void testZeroCommandSkipsFinalSafety() {
  Fixture fixture;
  fixture.next_output.reason = "holding";
  fixture.next_output.cmd_vel = {};
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(fixture.command_safety_calls == 0 &&
              fixture.stop_calls == 0 && fixture.shape_calls == 0 &&
              fixture.velocity_stop_calls == 1 && fixture.velocity_stop_reason == "zero_command",
          "zero command must bypass shaping and the command boundary while resetting smoother state");
  require(result.local.has_value() && !result.local->final_safety_applied &&
              result.local->final_safety_reason == "zero_command",
          "zero command must retain the legacy diagnostic reason");
}

void testInvalidShapingAndCommitFailureFailClosed() {
  Fixture invalid;
  invalid.next_output.cmd_vel = {0.4, 0.0, 0.0};
  invalid.shaped_velocity.valid = false;
  invalid.shaped_velocity.timed_out = true;
  invalid.shaped_velocity.reason = "target_timeout";
  AutonomyTickController invalid_controller(invalid.actions, invalid.control());

  const auto invalid_result = invalid_controller.tick(invalid.input());

  require(invalid.shape_calls == 1 && invalid.command_safety_calls == 0 &&
              invalid.commit_calls == 0 &&
              invalid.velocity_stop_calls == 1 && invalid.velocity_stop_reason == "target_timeout",
          "invalid or timed-out shaping must stop before the command boundary and commit");
  require(invalid_result.output.has_value() && near(invalid_result.output->cmd_vel.vx, 0.0) &&
              invalid_result.publish.cmd_vel && near(invalid_result.publish.command.vx, 0.0),
          "invalid shaping must expose an immediate zero command");
  require(invalid.pause_calls == 1 && invalid.stop_calls == 0 &&
              invalid_result.output->trajectory_frozen &&
              invalid_result.local->tracking.executionFrozen,
          "invalid shaping must pause the retained trajectory without resetting it");
  require(!invalid_result.local->final_safety_applied &&
              invalid_result.local->final_safety_stopped &&
              invalid_result.local->final_safety_reason == "target_timeout",
          "pre-safety shaping failure must retain its final stop reason in diagnostics");

  Fixture commit_failure;
  commit_failure.next_output.cmd_vel = {0.4, 0.0, 0.0};
  commit_failure.commit_succeeds = false;
  AutonomyTickController commit_controller(commit_failure.actions, commit_failure.control());

  const auto commit_result = commit_controller.tick(commit_failure.input());

  require(commit_failure.commit_calls == 1 && near(commit_failure.committed_command.vx, 0.4) &&
              commit_failure.velocity_stop_calls == 1 &&
              commit_failure.velocity_stop_reason == "velocity_smoother_commit_failed",
          "commit failure must hard-stop the smoother after attempting the actual safety command");
  require(commit_result.output.has_value() && near(commit_result.output->cmd_vel.vx, 0.0) &&
              near(commit_result.publish.command.vx, 0.0) &&
              commit_result.local->final_safety_stopped &&
              commit_result.local->final_safety_reason == "velocity_smoother_commit_failed",
          "commit failure must publish zero with explicit stopped diagnostics");
  require(commit_failure.pause_calls == 1 && commit_failure.stop_calls == 0 &&
              commit_result.output->trajectory_frozen,
          "commit failure must pause planning and tracking until execution can resume");
}

void testPlannedCrawlBypassesTeleopDeadbandAndKeepsMaximumLimits() {
  Fixture fixture;
  fixture.next_output.active = true;
  fixture.next_output.path_found = true;
  fixture.next_output.cmd_vel = {0.01, 0.0, 0.0};
  fixture.safety.min_motion_speed_mps = 0.03;
  fixture.final_actions.command_safety = lingtu::nav::endpoint::evaluateCommandSafety;
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto crawl = controller.tick(fixture.input());

  require(crawl.output.has_value() && near(crawl.publish.command.vx, 0.01) &&
              !crawl.local->final_safety_stopped && fixture.pause_calls == 0,
          "planned startup and arrival speeds must not be erased by the teleop deadband");
  fixture.next_output.cmd_vel = {0.8, 0.0, 2.0};

  const auto limited = controller.tick(fixture.input());

  require(near(limited.publish.command.vx, fixture.safety.max_speed_mps) &&
              near(limited.publish.command.wz, fixture.safety.max_yaw_rate) &&
              limited.local->final_safety_limited && fixture.pause_calls == 0,
          "planned motion must retain maximum speed and yaw-rate limits");
}

void testFinalLinearStopPausesTrajectoryAndPreservesAllowedRotation() {
  for (const double yaw_rate : {0.0, 0.4}) {
    Fixture fixture;
    fixture.next_output.active = true;
    fixture.next_output.path_found = true;
    fixture.next_output.cmd_vel = {0.2, 0.0, yaw_rate};
    fixture.safety.max_speed_mps = 0.0;
    fixture.final_actions.command_safety = lingtu::nav::endpoint::evaluateCommandSafety;
    AutonomyTickController controller(fixture.actions, fixture.control());

    const auto result = controller.tick(fixture.input());

    require(fixture.pause_calls == 1 && fixture.stop_calls == 0 &&
                result.output.has_value() && result.output->trajectory_frozen &&
                result.local->tracking.executionFrozen,
            "final translation suppression must freeze the retained trajectory");
    require(near(result.publish.command.vx, 0.0) && near(result.publish.command.vy, 0.0) &&
                near(result.publish.command.wz, yaw_rate),
            "pausing trajectory progress must preserve an allowed rotation command");
  }
}

void testShapingToZeroPausesTrajectoryBeforeSafety() {
  Fixture fixture;
  fixture.next_output.cmd_vel = {0.4, 0.0, 0.0};
  fixture.shape_override = true;
  fixture.shaped_velocity.command = {};
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(fixture.command_safety_calls == 0 && fixture.commit_calls == 0 &&
              fixture.pause_calls == 1 && fixture.stop_calls == 0 &&
              result.output.has_value() && result.output->trajectory_frozen &&
              near(result.publish.command.vx, 0.0),
          "valid zero shaping must pause trajectory time even without a stopped safety decision");
}

void testUnpublishedCommandDoesNotAdvanceSmootherCommit() {
  Fixture fixture;
  fixture.next_output.cmd_vel = {0.4, 0.0, 0.0};
  fixture.commit_succeeds = false;
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input(true, true, false, false));

  require(fixture.shape_calls == 1 && fixture.command_safety_calls == 1,
          "unpublished autonomy commands must still be shaped and limited");
  require(fixture.commit_calls == 0 && fixture.velocity_stop_calls == 0,
          "an unpublished command must neither advance smoother state nor fake a commit failure");
  require(result.output.has_value() && near(result.output->cmd_vel.vx, 0.4) &&
              result.local.has_value() && near(result.local->cmd_vel.vx, 0.4) &&
              !result.publish.cmd_vel && result.delta.cmd_vel_count == 0,
          "unpublished final command must remain available for output diagnostics only");
}

void testVerifiedRecoveryCommandUsesPlannerDecision() {
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
  AutonomyTickController controller(fixture.actions, fixture.control());

  const auto result = controller.tick(fixture.input());

  require(fixture.command_safety_calls == 1,
          "verified recovery must use the planner decision and command limits only");
  require(fixture.verified_translation_seen,
          "final collision checking must know this is a verified translation");
  require(fixture.velocity_stop_calls == 0,
          "an accepted verified recovery must not reset smoother state");
  require(result.output.has_value() && near(result.output->cmd_vel.vx, 0.0) &&
              near(result.output->cmd_vel.vy, 0.2) && near(result.output->cmd_vel.wz, 0.0),
          "verified recovery translation must pass through unchanged");
  require(result.publish.cmd_vel && near(result.publish.command.vy, 0.2),
          "the planner-approved recovery command must be published");
  require(result.local.has_value() && result.local->recovery_verified &&
              result.local->recovery_state == 2 && !result.local->final_safety_stopped,
          "status must retain verified recovery without a duplicate veto");
  fixture.next_output.recovery_verified = false;
  controller.tick(fixture.input());
  require(!fixture.verified_translation_seen,
          "an unverified recovery must not receive boundary-departure admission");
  fixture.next_output.recovery_verified = true;
  fixture.next_output.recovery_action = static_cast<int>(nav_kernel::RecoveryAction::Rotate);
  controller.tick(fixture.input());
  require(!fixture.verified_translation_seen,
          "rotation must not receive boundary-departure admission");
}

void testRecoveryOutcomesDistinguishRollingAndGenericGoals() {
  Fixture generic;
  generic.next_output.recovery_exhausted = true;
  generic.next_output.recovery_reason = "recovery_exhausted";
  generic.next_output.reason.clear();
  AutonomyTickController generic_controller(generic.actions, generic.control());
  const auto generic_result = generic_controller.tick(generic.input());

  Fixture rolling;
  rolling.next_output.recovery_exhausted = true;
  rolling.next_output.reason = "planner_stuck";
  AutonomyTickController rolling_controller(rolling.actions, rolling.control());
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

void testRecoveryMotionIsNotPreemptedByTaskReplanning() {
  Fixture fixture;
  fixture.next_output.active = true;
  fixture.next_output.path_found = true;
  fixture.next_output.recovery_state = 2;
  fixture.next_output.recovery_verified = true;
  fixture.next_output.cmd_vel = {0.1, 0.0, 0.0};
  AutonomyTickController controller(fixture.actions, fixture.control());
  const auto result = controller.tick(fixture.input());
  require(result.output && result.output->recovery_state == 2,
          "active recovery must execute through the normal autonomy tick");
  require(result.outcome.kind == AutonomyTickOutcomeKind::kNone &&
              !result.outcome.replan_trigger,
          "active recovery must not request a global replacement");
}

void testActualCollisionRestartsPlannerInsteadOfHoldingUnsafeTrackingTarget() {
  Fixture fixture;
  int blockage_reports = 0;
  fixture.actions.report_final_motion_blocked = [&](bool blocked, double) {
    require(blocked, "final braking rejection was reported as accepted motion");
    ++blockage_reports;
  };
  fixture.next_output.active = true;
  fixture.next_output.path_found = true;
  fixture.next_output.cmd_vel = {0.3, 0.0, 0.0};
  fixture.final_actions.command_safety = [](const auto &, const auto &, double) {
    CommandSafetyDecision decision;
    decision.should_publish = true;
    decision.stopped = true;
    decision.reason = "scan_actual_motion_blocked";
    return decision;
  };
  AutonomyTickController controller(fixture.actions, fixture.control());
  const auto result = controller.tick(fixture.input());
  require(fixture.stop_calls == 1 && fixture.pause_calls == 0,
          "actual collision must restart planning from odometry instead of resuming the old spline");
  require(near(result.publish.command.vx, 0.0) && fixture.commit_calls == 0,
          "the blocked command must never be committed");
  require(result.local->final_safety_reason == "scan_actual_motion_blocked",
          "collision rejection must be visible in navigation status");
  require(blockage_reports == 1, "final braking rejection did not reach recovery");
}

void testBrakingSlowdownRetainsActiveTrajectory() {
  Fixture fixture;
  int accepted_reports = 0;
  fixture.actions.report_final_motion_blocked = [&](bool blocked, double) {
    require(!blocked, "safe limited motion must clear prior final blockage");
    ++accepted_reports;
  };
  fixture.next_output.active = true;
  fixture.next_output.path_found = true;
  fixture.next_output.cmd_vel = {0.75, 0.0, 0.0};
  fixture.next_output.tracking.active = true;
  fixture.next_output.tracking.trajectoryId = 17;
  fixture.final_actions.command_safety = [](const auto &, const auto &, double) {
    CommandSafetyDecision decision;
    decision.should_publish = true;
    decision.cmd = {0.25, 0.0, 0.0};
    decision.slowed = decision.limited = true;
    decision.reason = "scan_actual_motion_limited";
    return decision;
  };
  AutonomyTickController controller(fixture.actions, fixture.control());
  const auto result = controller.tick(fixture.input());
  require(accepted_reports == 1, "accepted motion did not clear prior final blockage");
  require(fixture.stop_calls == 0 && fixture.pause_calls == 0,
          "a safe lower speed must not discard or freeze the active trajectory");
  require(near(result.publish.command.vx, 0.25) && fixture.commit_calls == 1,
          "only the collision-checked lower speed may be committed");
  require(result.local->tracking.trajectoryId == 17 &&
              !result.local->tracking.executionFrozen && result.local->final_safety_slowed,
          "braking slowdown must preserve tracking identity and expose its limiting reason");
}

void testReachedOutcomesDistinguishInspectionArrival() {
  Fixture generic;
  generic.next_output.goal_reached = true;
  AutonomyTickController generic_controller(generic.actions, generic.control());
  const auto generic_result = generic_controller.tick(generic.input());

  Fixture rolling;
  rolling.next_output.goal_reached = true;
  AutonomyTickController rolling_controller(rolling.actions, rolling.control());
  const auto rolling_result = rolling_controller.tick(rolling.input(true, true, true));

  require(generic_result.outcome.kind == AutonomyTickOutcomeKind::kGoalReached &&
              generic_result.outcome.reason == "goal_reached" &&
              generic_result.outcome.inspection_arrival_intent,
          "generic reach must expose the inspection arrival intent");
  require(generic.velocity_stop_calls == 1 && generic.velocity_stop_reason == "goal_reached" &&
              generic.shape_calls == 0 && generic.command_safety_calls == 0,
          "goal reached must hard-stop without shaping or safety evaluation");
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
    testLocalizationHoldRecoversWithoutTerminalState();
    testLocalizationHoldDoesNotExpireTheGoal();
    testLocalizationHoldDoesNotCrossGoalOrControlChanges();
    testLocalizationHoldDoesNotTerminalizeRollingOrUnownedPaths();
    testInputsExpiringDuringPlanningBlockPublicationWithoutCompletingGoal();
    testActiveMapIdentityGuardFailsClosedBeforePlanning();
    testNormalTickProducesBorrowedInputIntentsAndDiagnostics();
    testZeroCommandSkipsFinalSafety();
    testInvalidShapingAndCommitFailureFailClosed();
    testPlannedCrawlBypassesTeleopDeadbandAndKeepsMaximumLimits();
    testFinalLinearStopPausesTrajectoryAndPreservesAllowedRotation();
    testShapingToZeroPausesTrajectoryBeforeSafety();
    testUnpublishedCommandDoesNotAdvanceSmootherCommit();
    testVerifiedRecoveryCommandUsesPlannerDecision();
    testRecoveryOutcomesDistinguishRollingAndGenericGoals();
    testRecoveryMotionIsNotPreemptedByTaskReplanning();
    testReachedOutcomesDistinguishInspectionArrival();
    testActualCollisionRestartsPlannerInsteadOfHoldingUnsafeTrackingTarget();
    testBrakingSlowdownRetainsActiveTrajectory();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_autonomy_tick_controller: FAIL: %s\n", error.what());
    return 1;
  }
  return 0;
}
