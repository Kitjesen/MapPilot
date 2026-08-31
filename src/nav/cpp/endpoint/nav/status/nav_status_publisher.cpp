#include "status/nav_status_publisher.hpp"

#include <cstdio>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "runtime/time.hpp"

namespace lingtu::nav::endpoint {
namespace {

void writeStatusLogToStderr(const std::string &message) {
  std::fputs(message.c_str(), stderr);
}

NavStatusPublisherActions normalizeActions(NavStatusPublisherActions actions) {
  if (!actions.steady_now_s) {
    actions.steady_now_s = []() { return steadySeconds(); };
  }
  if (!actions.wall_now_s) {
    actions.wall_now_s = []() { return nowSeconds(); };
  }
  if (!actions.sample_planner) {
    throw std::invalid_argument("NavStatusPublisher requires a planner sampler");
  }
  if (!actions.sample_motion_layer) {
    throw std::invalid_argument("NavStatusPublisher requires a motion-layer sampler");
  }
  if (!actions.sample_loop_health) {
    throw std::invalid_argument("NavStatusPublisher requires a control-loop health sampler");
  }
  if (!actions.sample_far_input) {
    throw std::invalid_argument("NavStatusPublisher requires a FAR-input sampler");
  }
  if (!actions.log) {
    actions.log = writeStatusLogToStderr;
  }
  return actions;
}

std::string activeCommandSource(const StatusRuntimeState &state) {
  if (state.estop_latched) {
    return "estop";
  }
  if (state.operator_takeover_latched) {
    return state.teleop_request_active && state.teleop->fresh ? "teleop" : "manual_hold";
  }
  if (state.path_active) {
    return "autonomy";
  }
  return "none";
}

nav_kernel::Twist finalCommandVelocity(const StatusWriterConfig &config,
                                       const StatusRuntimeState &state) {
  if (state.estop_latched) {
    return {};
  }
  if (state.teleop_request_active) {
    return state.teleop->output;
  }
  if (config.control_mode == "autonomy") {
    return state.local->cmd_vel;
  }
  return state.teleop->output;
}

std::string makeStatusLog(const StatusRuntimeState &state, std::size_t obstacle_points,
                          const MotionLayerStats &motion_layer) {
  std::ostringstream out;
  out << "nav_native: odom=" << state.counters.odom << " goals=" << state.counters.goals
      << " cancels=" << state.counters.cancels << " registered_clouds=" << state.counters.clouds
      << " terrain_maps=" << state.counters.terrain_maps
      << " terrain_map_exts=" << state.counters.terrain_map_exts
      << " traversability=" << state.counters.traversability
      << " teleop=" << state.counters.teleop_commands << " paths=" << state.counters.paths
      << " plan_fail=" << state.counters.plan_failures << " outputs=" << state.counters.outputs
      << " cmd_vel=" << state.counters.cmd_vel << " obstacle_points=" << obstacle_points
      << " motion_dynamic=" << motion_layer.dynamic_cells << " gate=" << state.input_gate->reason
      << " active=" << (state.path_active ? 1 : 0) << '\n';
  return out.str();
}

}  // namespace

NavStatusPublisher::NavStatusPublisher(StatusWriterConfig config, double interval_s,
                                       NavStatusPublisherActions actions)
    : config_(std::move(config)),
      interval_s_(interval_s),
      actions_(normalizeActions(std::move(actions))),
      snapshot_writer_(config_.status_file) {
  next_due_s_ = interval_s_ > 0.0 ? actions_.steady_now_s() + interval_s_ : 0.0;
}

NavStatusPublisher::NavStatusPublisher(StatusWriterConfig config, double interval_s,
                                       NavStatusPublisherActions actions,
                                       StatusSnapshotFileWriter::Sink sink)
    : config_(std::move(config)),
      interval_s_(interval_s),
      actions_(normalizeActions(std::move(actions))),
      snapshot_writer_(config_.status_file, std::move(sink)) {
  next_due_s_ = interval_s_ > 0.0 ? actions_.steady_now_s() + interval_s_ : 0.0;
}

bool NavStatusPublisher::publishIfDue(const StatusRuntimeState &state,
                                      const CommandDiagnostics &commands,
                                      const TimingDiagnostics &previous_timing,
                                      TimingDiagnostics &current_timing) {
  if (interval_s_ <= 0.0) {
    return false;
  }

  const double steady_now_s = actions_.steady_now_s();
  if (steady_now_s < next_due_s_) {
    return false;
  }

  // Schedule first so slow sampling or persistence cannot cause a burst.
  next_due_s_ = steady_now_s + interval_s_;
  validateRuntimeState(state);

  const double wall_now_s = actions_.wall_now_s();
  const StatusPlannerSample planner = actions_.sample_planner(steady_now_s, current_timing);
  if (planner.local_map_obstacle_xyzh == nullptr) {
    throw std::invalid_argument("NavStatusPublisher planner sample has no obstacle view");
  }

  config_.operator_takeover_latched = state.operator_takeover_latched;
  config_.resume_required = state.operator_resume_required;
  config_.active_cmd_source = activeCommandSource(state);

  const StatusMotionLayerSample motion = actions_.sample_motion_layer(wall_now_s);
  if (motion.dynamic_clusters == nullptr) {
    throw std::invalid_argument("NavStatusPublisher motion sample has no cluster view");
  }
  const ControlLoopHealthSnapshot loop_health = actions_.sample_loop_health();
  const StatusFarInputSample far_input = actions_.sample_far_input();

  const auto status_log_start = SteadyClock::now();
  actions_.log(makeStatusLog(state, planner.obstacle_points, motion.stats));
  current_timing.status_log_ms = elapsedMs(status_log_start);

  const auto status_snapshot_start = SteadyClock::now();
  writeStatusSnapshot(
      snapshot_writer_, config_, wall_now_s, state.has_odom, state.has_map_odom_tf,
      state.path_active, state.estop_latched, state.estop_reason,
      finalCommandVelocity(config_, state), state.final_output, state.driver_control,
      far_input,
      config_.check_obstacle && config_.use_traversability_cost && planner.traversability_fresh,
      config_.check_obstacle && planner.terrain_map_fresh,
      config_.check_obstacle && planner.terrain_ext_fresh, *state.input_gate, *state.cloud_sync,
      *state.frames, commands, *state.operator_motion_transport, state.counters.odom,
      state.counters.tf, state.counters.goals, state.counters.cancels, state.counters.map_clearing,
      state.counters.cloud_clearing, state.counters.clouds, state.counters.terrain_maps,
      state.counters.terrain_map_exts, state.counters.traversability,
      state.counters.teleop_commands, state.counters.teleop_outputs, state.counters.teleop_stops,
      state.counters.teleop_slows, state.counters.teleop_limited, state.counters.paths,
      state.counters.plan_failures, state.counters.outputs, state.counters.cmd_vel,
      motion.live_obstacle_cells, motion.stats, *motion.dynamic_clusters, *state.last_sensor_origin,
      planner.obstacle_points, *state.plan, *state.local, *state.teleop, previous_timing,
      loop_health, *state.global_path, *state.local_path, *state.local_planner_debug,
      *planner.local_map_obstacle_xyzh, *state.local_map_traversability,
      state.local_collision_map);
  current_timing.status_snapshot_ms = elapsedMs(status_snapshot_start);
  return true;
}

void NavStatusPublisher::requestImmediate() {
  next_due_s_ = 0.0;
}

void NavStatusPublisher::flush() {
  snapshot_writer_.flush();
}

void NavStatusPublisher::validateRuntimeState(const StatusRuntimeState &state) const {
  if (state.input_gate == nullptr || state.cloud_sync == nullptr || state.frames == nullptr ||
      state.operator_motion_transport == nullptr || state.last_sensor_origin == nullptr ||
      state.plan == nullptr || state.local == nullptr || state.teleop == nullptr ||
      state.global_path == nullptr || state.local_path == nullptr ||
      state.local_planner_debug == nullptr || state.local_map_traversability == nullptr) {
    throw std::invalid_argument("NavStatusPublisher received an incomplete runtime state");
  }
}

}  // namespace lingtu::nav::endpoint
