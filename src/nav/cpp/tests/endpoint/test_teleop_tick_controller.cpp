#include <algorithm>
#include <cstdio>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "motion/teleop_tick_controller.hpp"

namespace {

using lingtu::nav::endpoint::CliConfig;
using lingtu::nav::endpoint::CommandSafetyConfig;
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::TeleopDiagnostics;
using lingtu::nav::endpoint::TeleopTickActions;
using lingtu::nav::endpoint::TeleopTickController;
using lingtu::nav::endpoint::TeleopTickInput;
using lingtu::nav::endpoint::TeleopTickPlannerInputs;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

struct Fixture {
  CliConfig config;
  CommandSafetyConfig safety;
  std::optional<nav_kernel::Pose> map_body{nav_kernel::Pose{}};
  InputGateState gate;
  nav_kernel::Twist request{0.2, 0.0, 0.0};
  std::vector<float> obstacles;
  TraversabilityGrid traversability;
  TeleopDiagnostics previous;
  TimingDiagnostics timing;
  TeleopTickPlannerInputs planner_inputs;
  lingtu::nav::plan::NavLoopOutput planner_output;
  std::vector<double> ages{0.1, 0.1};
  std::size_t age_index{0};
  int compute_calls{0};
  int planner_calls{0};
  int stop_calls{0};
  TeleopTickActions actions;

  Fixture() {
    gate.ready = true;
    previous.request = {0.7, 0.1, 0.2};
    previous.age_s = 4.0;
    actions.teleop_receive_age_s = [&] {
      const auto index = std::min(age_index++, ages.size() - 1);
      return ages[index];
    };
    actions.steady_now_s = [] { return 10.0; };
    actions.compute_planner_inputs = [&](TimingDiagnostics &) {
      ++compute_calls;
      return planner_inputs;
    };
    actions.tick_teleop_intent = [&](const nav_kernel::Pose &, const nav_kernel::Twist &,
                                     const float *, int, double,
                                     lingtu::nav::plan::TraversabilityGridView) {
      ++planner_calls;
      return planner_output;
    };
    actions.stop_linear_motion = [&] { ++stop_calls; };
  }

  TeleopTickInput input(bool path_active = false) {
    return {
        config,    safety,         map_body, gate,     &request, path_active,
        obstacles, traversability, 9.5,      previous, timing,
    };
  }
};

void testBlockedGatePublishesOnlyAZeroIntent() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.gate.ready = false;
  fixture.gate.reason = "input_cloud_stale";
  TeleopTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(result.handled, "active teleop must be handled");
  require(result.teleop.seen && !result.teleop.fresh, "blocked input must be visible and stale");
  require(result.teleop.stopped && result.teleop.limited && !result.teleop.slowed,
          "blocked input must fail closed");
  require(result.teleop.reason == "input_cloud_stale", "gate reason must be retained");
  require(result.teleop.request.vx == fixture.previous.request.vx &&
              result.teleop.age_s == fixture.previous.age_s,
          "untouched diagnostic fields must retain their previous values");
  require(result.publish.cmd_vel, "blocked input must request a zero publish");
  require(result.publish.command.vx == 0.0 && result.publish.command.vy == 0.0 &&
              result.publish.command.wz == 0.0,
          "controller must never turn a blocked input into nonzero output");
  require(result.delta.cmd_vel_count == 1 && result.delta.teleop_output_count == 1 &&
              result.delta.teleop_stop_count == 1 && result.delta.teleop_limited_count == 1,
          "blocked counters must match endpoint behavior");
  require(fixture.stop_calls == 1 && fixture.compute_calls == 0 && fixture.planner_calls == 0,
          "blocked input must stop but never invoke planning");
}

void testDirectCommandIsAcceptedOrLimitedWithoutPlannerSideEffects() {
  for (const auto &sample : std::vector<std::pair<double, bool>>{{0.2, false}, {1.0, true}}) {
    Fixture fixture;
    fixture.config.teleop_local_planner = false;
    fixture.config.publish_cmd_vel = true;
    fixture.safety.check_obstacle = false;
    fixture.safety.use_traversability_cost = false;
    fixture.safety.max_speed_mps = 0.4;
    fixture.request.vx = sample.first;
    TeleopTickController controller(fixture.actions);

    const auto result = controller.tick(fixture.input());

    require(result.publish.cmd_vel, "direct command must return a publish intent");
    require(result.teleop.published && result.teleop.fresh,
            "fresh direct command diagnostics mismatch");
    require(!result.teleop.stopped && result.teleop.limited == sample.second,
            "direct command limit disposition mismatch");
    require(result.teleop.reason == (sample.second ? "limited" : "accepted"),
            "direct command reason mismatch");
    require(result.publish.command.vx == (sample.second ? 0.4 : 0.2),
            "direct command output mismatch");
    require(result.delta.cmd_vel_count == 1 && result.delta.teleop_output_count == 1 &&
                result.delta.teleop_limited_count == (sample.second ? 1U : 0U),
            "direct command counter mismatch");
    require(fixture.compute_calls == 0 && fixture.planner_calls == 0 && fixture.stop_calls == 0,
            "direct command must not enter or stop an unused planner");
  }
}

void testPublishTimeStaleOverridesAnEarlierAcceptedDecision() {
  Fixture fixture;
  fixture.config.publish_cmd_vel = true;
  fixture.config.teleop_cmd_max_age_s = 0.35;
  fixture.safety.check_obstacle = false;
  fixture.safety.use_traversability_cost = false;
  fixture.ages = {0.1, 0.5};
  TeleopTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(result.publish.cmd_vel, "stale transition must publish a zero intent");
  require(!result.teleop.fresh && result.teleop.stopped && result.teleop.limited,
          "publish-time stale decision must fail closed");
  require(result.teleop.reason == "stale" && result.teleop.age_s == 0.5,
          "publish-time age and reason mismatch");
  require(result.publish.command.vx == 0.0 && result.publish.command.vy == 0.0 &&
              result.publish.command.wz == 0.0,
          "stale transition must overwrite the accepted nonzero command");
  require(result.delta.teleop_stop_count == 1 && result.delta.teleop_limited_count == 1,
          "stale counters mismatch");
}

void testAssistedPathReturnsPlannerArtifactsAndFinalSafetyIntent() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.safety.check_obstacle = false;
  fixture.safety.use_traversability_cost = false;
  fixture.planner_inputs.planner_obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assisted";
  fixture.planner_output.slow_down = 1;
  fixture.planner_output.target_distance_m = 1.0;
  fixture.planner_output.target = {1.0, 0.0, 0.0};
  fixture.planner_output.cmd_vel = {0.25, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(fixture.compute_calls == 1 && fixture.planner_calls == 1 && fixture.stop_calls == 0,
          "assisted path action sequence mismatch");
  require(result.local.has_value() && result.local->path_found,
          "assisted local diagnostics missing");
  require(result.local->reason == "teleop_assisted" &&
              result.local->final_safety_reason == "teleop_assisted",
          "assisted diagnostics reason mismatch");
  require(result.local_path.size() == 2 && result.publish.local_path && result.publish.waypoint,
          "assisted planner artifacts must be returned as publish intents");
  require(result.publish.cmd_vel && result.publish.command.vx == 0.25,
          "assisted command intent mismatch");
  require(result.teleop.reason == "teleop_assisted" && result.delta.output_count == 1,
          "assisted teleop result mismatch");
}

void testAssistedNoPathFailsClosedAndStopsPlannerMotion() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.safety.check_obstacle = false;
  fixture.safety.use_traversability_cost = false;
  fixture.planner_inputs.planner_obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = false;
  fixture.planner_output.reason.clear();
  TeleopTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(fixture.compute_calls == 1 && fixture.planner_calls == 1 && fixture.stop_calls == 1,
          "no-path assist must enter then stop planner motion");
  require(result.teleop.stopped && result.teleop.limited &&
              result.teleop.reason == "teleop_assist_no_path",
          "no-path assist must fail closed with a stable reason");
  require(result.publish.command.vx == 0.0 && result.publish.command.vy == 0.0 &&
              result.publish.command.wz == 0.0,
          "no-path assist must only return a zero command intent");
  require(result.local.has_value() && !result.local->path_found &&
              result.local->final_safety_reason == "teleop_assist_no_path",
          "no-path local diagnostics mismatch");
  require(result.publish.local_path && result.publish.waypoint && result.delta.output_count == 1 &&
              result.delta.teleop_stop_count == 1 && result.delta.teleop_limited_count == 1,
          "no-path publish intents and counters mismatch");
}

void testActiveAutonomyPathSuppressesTeleopPublishing() {
  Fixture fixture;
  fixture.previous.stopped = true;
  fixture.previous.limited = true;
  fixture.previous.output = {0.1, 0.2, 0.3};
  TeleopTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input(true));

  require(result.handled && result.teleop.seen && result.teleop.fresh,
          "auto-active teleop diagnostics mismatch");
  require(!result.teleop.published && result.teleop.reason == "auto_active",
          "active autonomy path must suppress teleop publication");
  require(result.teleop.stopped && result.teleop.limited && result.teleop.output.vy == 0.2,
          "auto-active must preserve fields the endpoint did not overwrite");
  require(!result.publish.cmd_vel && !result.publish.local_path && !result.publish.waypoint,
          "auto-active branch must return no publish intents");
  require(fixture.compute_calls == 0 && fixture.planner_calls == 0 && fixture.stop_calls == 0,
          "auto-active branch must not touch teleop planner state");
}

void testAssistedPublishTimeStaleUpdatesLocalFinalSafety() {
  Fixture fixture;
  fixture.config.teleop_local_planner = true;
  fixture.config.publish_cmd_vel = true;
  fixture.config.teleop_cmd_max_age_s = 0.35;
  fixture.safety.check_obstacle = false;
  fixture.safety.use_traversability_cost = false;
  fixture.ages = {0.1, 0.5};
  fixture.planner_inputs.planner_obstacles = &fixture.obstacles;
  fixture.planner_output.active = true;
  fixture.planner_output.path_found = true;
  fixture.planner_output.reason = "teleop_assisted";
  fixture.planner_output.cmd_vel = {0.25, 0.0, 0.0};
  fixture.planner_output.local_path_map = {
      {0.0, 0.0, 0.0},
      {1.0, 0.0, 0.0},
  };
  TeleopTickController controller(fixture.actions);

  const auto result = controller.tick(fixture.input());

  require(fixture.planner_calls == 1 && fixture.stop_calls == 1,
          "publish-time stale must stop an already-ticked assisted planner");
  require(result.teleop.reason == "stale" && !result.teleop.fresh && result.teleop.stopped,
          "assisted stale teleop diagnostics mismatch");
  require(result.local.has_value() && result.local->near_field_stop &&
              result.local->cmd_vel.vx == 0.0,
          "assisted stale local command must be zeroed");
  require(result.local->final_safety_stopped && !result.local->final_safety_slowed &&
              result.local->final_safety_limited && result.local->final_safety_reason == "stale",
          "assisted stale final-safety diagnostics mismatch");
  require(result.local->reason == "teleop_assisted" && result.local_path.size() == 2,
          "stale override must retain the planner trace for diagnostics");
}

}  // namespace

int main() {
  try {
    testBlockedGatePublishesOnlyAZeroIntent();
    testDirectCommandIsAcceptedOrLimitedWithoutPlannerSideEffects();
    testPublishTimeStaleOverridesAnEarlierAcceptedDecision();
    testAssistedPathReturnsPlannerArtifactsAndFinalSafetyIntent();
    testAssistedNoPathFailsClosedAndStopsPlannerMotion();
    testActiveAutonomyPathSuppressesTeleopPublishing();
    testAssistedPublishTimeStaleUpdatesLocalFinalSafety();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_teleop_tick_controller: FAIL: %s\n", error.what());
    return 1;
  }
  return 0;
}
