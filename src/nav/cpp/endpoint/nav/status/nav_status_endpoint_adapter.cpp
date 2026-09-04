#include "status/nav_status_endpoint_adapter.hpp"

#include "runtime/state.hpp"

namespace lingtu::nav::endpoint {

StatusRuntimeState statusRuntimeStateFromEndpoint(const EndpointState &state) {
  const ControlAuthority &authority = state.control_authority;
  StatusRuntimeState result;
  result.has_odom = state.map_body.has_value();
  result.has_map_odom_tf = state.map_odom_tf.has_value();
  result.path_active = authority.pathActive();
  result.estop_latched = authority.estopLatched();
  result.estop_reason = authority.estopReason();
  result.operator_takeover_latched = authority.operatorTakeoverLatched();
  result.teleop_request_active = authority.teleopRequest().has_value();
  result.operator_resume_required = state.operator_resume_required;
  result.driver_control.received = state.driver_control_received;
  result.driver_control.ready = state.driver_control_ready;
  result.driver_control.reason = state.driver_control_reason;
  result.driver_control.last_command_accepted = state.driver_last_command_accepted;
  result.driver_control.accepted_producer_boot_id = state.driver_accepted_producer_boot_id;
  result.driver_control.accepted_output_sequence = state.driver_accepted_output_sequence;
  result.motion_stop_evidence = state.motion_stop_evidence.snapshot();
  result.counters = {
      state.odom_count,           state.tf_count,           state.goal_count,
      state.cancel_count,         state.map_clearing_count, state.cloud_clearing_count,
      state.cloud_count,          state.terrain_map_count,  state.terrain_map_ext_count,
      state.traversability_count, state.teleop_cmd_count,   state.teleop_output_count,
      state.teleop_stop_count,    state.teleop_slow_count,  state.teleop_limited_count,
      state.path_count,           state.plan_fail_count,    state.output_count,
      state.cmd_vel_count,
  };
  result.input_gate = &state.input_gate_state;
  result.cloud_sync = &state.cloud_sync;
  result.frames = &state.frames;
  result.operator_motion_transport = &state.operator_motion_transport;
  result.last_sensor_origin = &state.last_sensor_origin;
  result.plan = &state.last_plan;
  result.local = &state.last_local;
  result.teleop = &state.last_teleop;
  result.global_path = &state.last_global_path;
  result.local_path = &state.last_local_path;
  result.local_planner_debug = &state.last_local_planner_debug;
  result.local_map_traversability = &state.local_traversability_grid;
  result.local_collision_map = state.local_collision_map.view();
  return result;
}

}  // namespace lingtu::nav::endpoint
