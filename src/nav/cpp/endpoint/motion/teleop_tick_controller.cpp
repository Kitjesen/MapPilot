#include "motion/teleop_tick_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

using Clock = std::chrono::steady_clock;

const std::vector<float> kEmptyObstacles;

double elapsedMs(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

}  // namespace

TeleopTickController::TeleopTickController(TeleopTickActions actions)
    : actions_(std::move(actions)) {
  if (!actions_.teleop_receive_age_s || !actions_.steady_now_s ||
      !actions_.compute_planner_inputs || !actions_.tick_teleop_intent ||
      !actions_.stop_linear_motion) {
    throw std::invalid_argument("teleop tick actions must all be configured");
  }
}

TeleopTickResult TeleopTickController::tick(const TeleopTickInput &input) {
  const auto tick_start = Clock::now();
  TeleopTickResult result;
  if (input.active_request == nullptr) {
    result.timing.teleop_gate_ms = elapsedMs(tick_start);
    return result;
  }

  result.handled = true;
  if (!input.path_active && !input.input_gate.ready) {
    result.teleop = input.previous_teleop;
    if (input.config.teleop_local_planner) {
      actions_.stop_linear_motion();
    }
    result.teleop.seen = true;
    result.teleop.fresh = false;
    result.teleop.published = input.config.publish_cmd_vel;
    result.teleop.stopped = true;
    result.teleop.slowed = false;
    result.teleop.limited = true;
    result.teleop.reason = input.input_gate.reason;
    result.teleop.output = {};
    result.publish.cmd_vel = input.config.publish_cmd_vel;
    result.delta.cmd_vel_count = input.config.publish_cmd_vel ? 1U : 0U;
    result.delta.teleop_output_count = input.config.publish_cmd_vel ? 1U : 0U;
    result.delta.teleop_stop_count = 1U;
    result.delta.teleop_limited_count = 1U;
  } else if (!input.path_active) {
    const double tick_now = actions_.steady_now_s();
    const bool traversability_available = !input.traversability.values.empty();
    const bool traversability_fresh =
        traversability_available && input.last_traversability_receive_s > 0.0 &&
        (input.config.traversability_max_age_s <= 0.0 ||
         tick_now - input.last_traversability_receive_s <= input.config.traversability_max_age_s);
    const double age_s = actions_.teleop_receive_age_s();
    auto precheck_config = input.safety;
    precheck_config.check_obstacle = false;
    precheck_config.use_traversability_cost = false;
    const auto precheck =
        arbitrateTeleopCommand(precheck_config, *input.active_request, age_s, input.map_body,
                               kEmptyObstacles, TraversabilityGrid{}, false);
    auto decision = precheck;
    const bool use_assisted_planner =
        input.config.teleop_local_planner && input.map_body && precheck.should_publish &&
        !precheck.stopped && linearSpeed(precheck.cmd) >= input.config.teleop_min_motion_speed_mps;
    if (use_assisted_planner) {
      const auto planner_inputs = actions_.compute_planner_inputs(input.timing);
      const auto *planner_obstacles = planner_inputs.planner_obstacles;
      const auto nav_tick_start = Clock::now();
      auto assisted = actions_.tick_teleop_intent(
          *input.map_body, precheck.cmd,
          planner_obstacles != nullptr ? planner_obstacles->data() : nullptr,
          planner_obstacles != nullptr ? static_cast<int>(planner_obstacles->size() / 4) : 0,
          actions_.steady_now_s(), planner_inputs.traversability_view);
      result.timing.nav_tick_measured = true;
      result.timing.nav_tick_ms = elapsedMs(nav_tick_start);

      if (assisted.path_found && assisted.local_path_map.size() >= 2) {
        decision = evaluateAutonomyPathSafety(
            input.safety, assisted.cmd_vel, input.map_body, assisted.local_path_map,
            planner_obstacles != nullptr ? *planner_obstacles : kEmptyObstacles,
            input.traversability, traversability_fresh);
        if (!decision.stopped && decision.reason == "accepted") {
          decision.reason = assisted.reason;
        }
      } else {
        decision = {};
        decision.should_publish = true;
        decision.stopped = true;
        decision.limited = true;
        decision.reason = assisted.reason.empty() ? "teleop_assist_no_path" : assisted.reason;
        actions_.stop_linear_motion();
      }

      LocalDiagnostics local;
      local.seen = true;
      local.active = assisted.active;
      local.goal_reached = false;
      local.path_found = assisted.path_found;
      local.near_field_stop = decision.stopped;
      local.reason = assisted.reason;
      local.slow_down = assisted.slow_down;
      local.recovery_state = 0;
      local.target_index = 0;
      local.target_distance_m = assisted.target_distance_m;
      local.target = assisted.target;
      local.local_path_points = assisted.local_path_map.size();
      local.path_follower_cmd_vel = assisted.cmd_vel;
      local.cmd_vel = decision.cmd;
      local.final_safety_applied = true;
      local.final_safety_stopped = decision.stopped;
      local.final_safety_slowed = decision.slowed;
      local.final_safety_limited = decision.limited;
      local.final_safety_reason = decision.reason;
      local.final_safety_obstacle_distance_m = decision.obstacle_distance_m;
      local.final_safety_traversability_cost = decision.traversability_cost;
      result.local = std::move(local);
      result.local_path = std::move(assisted.local_path_map);
      result.local_planner_debug = std::move(assisted.local_planner_debug);
      result.waypoint = assisted.target;
      result.publish.local_path = true;
      result.publish.waypoint = true;
      result.delta.output_count = 1U;
    } else {
      decision =
          arbitrateTeleopCommand(input.safety, *input.active_request, age_s, input.map_body,
                                 input.obstacle_xyzh, input.traversability, traversability_fresh);
      if (input.config.teleop_local_planner) {
        actions_.stop_linear_motion();
      }
    }

    const double publish_age_s = actions_.teleop_receive_age_s();
    if (input.config.teleop_cmd_max_age_s > 0.0 &&
        publish_age_s > input.config.teleop_cmd_max_age_s) {
      decision = {};
      decision.should_publish = true;
      decision.stopped = true;
      decision.limited = true;
      decision.reason = "stale";
      if (input.config.teleop_local_planner) {
        actions_.stop_linear_motion();
      }
      if (use_assisted_planner && result.local) {
        result.local->near_field_stop = true;
        result.local->cmd_vel = {};
        result.local->final_safety_stopped = true;
        result.local->final_safety_slowed = false;
        result.local->final_safety_limited = true;
        result.local->final_safety_reason = "stale";
      }
    }

    result.teleop.seen = true;
    result.teleop.fresh = input.config.teleop_cmd_max_age_s <= 0.0 ||
                          publish_age_s <= input.config.teleop_cmd_max_age_s;
    result.teleop.age_s = publish_age_s;
    result.teleop.request = *input.active_request;
    result.teleop.output = decision.cmd;
    result.teleop.published = input.config.publish_cmd_vel && decision.should_publish;
    result.teleop.stopped = decision.stopped;
    result.teleop.slowed = decision.slowed;
    result.teleop.limited = decision.limited;
    result.teleop.reason = decision.reason;
    result.teleop.obstacle_distance_m = decision.obstacle_distance_m;
    result.teleop.traversability_cost = decision.traversability_cost;
    result.publish.cmd_vel = result.teleop.published;
    result.publish.command = decision.cmd;
    result.delta.cmd_vel_count = result.publish.cmd_vel ? 1U : 0U;
    result.delta.teleop_output_count = result.publish.cmd_vel ? 1U : 0U;
    result.delta.teleop_stop_count = decision.stopped ? 1U : 0U;
    result.delta.teleop_slow_count = decision.slowed ? 1U : 0U;
    result.delta.teleop_limited_count = decision.limited ? 1U : 0U;
  } else {
    result.teleop = input.previous_teleop;
    result.teleop.seen = true;
    result.teleop.fresh = true;
    result.teleop.published = false;
    result.teleop.reason = "auto_active";
  }

  result.timing.teleop_gate_ms = elapsedMs(tick_start);
  return result;
}

}  // namespace lingtu::nav::endpoint
