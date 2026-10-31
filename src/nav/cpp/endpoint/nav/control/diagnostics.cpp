#include "control/diagnostics.hpp"

namespace lingtu::nav::endpoint {

LocalDiagnostics makeLocalDiagnostics(
    const lingtu::nav::navigation::ExecutionOutput &output,
    const CommandSafetyDecision *final_safety,
    bool safety_applied) {
  LocalDiagnostics local;
  local.seen = true;
  local.active = output.active;
  local.goal_reached = output.goal_reached;
  local.path_found = output.path_found;
  local.near_field_stop = output.near_field_stop;
  local.reason = output.reason;
  local.slow_down = output.slow_down;
  local.recovery_state = output.recovery_state;
  local.recovery_action = output.recovery_action;
  local.recovery_attempt = output.recovery_attempt;
  local.recovery_candidate_count = output.recovery_candidate_count;
  local.recovery_rotation_target_rad = output.recovery_rotation_target_rad;
  local.recovery_verified = output.recovery_verified;
  local.recovery_progress = output.recovery_progress;
  local.recovery_trigger = output.recovery_trigger;
  local.recovery_reason = output.recovery_reason;
  local.prediction_count = output.prediction_count;
  local.recovery_exhausted = output.recovery_exhausted;
  local.target_index = output.target_index;
  local.target_distance_m = output.target_distance_m;
  local.target = output.target;
  local.tracking = output.tracking;
  local.local_path_points = output.local_path_map.size();
  local.path_follower_cmd_vel = output.cmd_vel;
  local.cmd_vel = output.cmd_vel;
  local.final_safety_applied = safety_applied;
  local.final_safety_stopped = final_safety != nullptr && final_safety->stopped;
  local.final_safety_slowed = final_safety != nullptr && final_safety->slowed;
  local.final_safety_limited = final_safety != nullptr && final_safety->limited;
  local.final_safety_reason = final_safety != nullptr ? final_safety->reason : "zero_command";
  local.final_safety_obstacle_distance_m =
      final_safety != nullptr ? final_safety->obstacle_distance_m : -1.0;
  local.final_safety_traversability_cost =
      final_safety != nullptr ? final_safety->traversability_cost : -1.0;
  return local;
}

LocalDiagnostics makeStoppedLocalDiagnostics(
    LocalDiagnostics local, const std::string &reason,
    const std::string &safety_reason) {
  local.seen = true;
  local.active = false;
  local.path_found = false;
  local.near_field_stop = true;
  local.reason = reason;
  local.final_safety_applied = false;
  local.final_safety_stopped = true;
  local.final_safety_slowed = false;
  local.final_safety_limited = false;
  local.final_safety_reason = safety_reason;
  local.final_safety_obstacle_distance_m = -1.0;
  local.final_safety_traversability_cost = -1.0;
  local.path_follower_cmd_vel = {};
  local.cmd_vel = {};
  return local;
}

}  // namespace lingtu::nav::endpoint
