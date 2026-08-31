#include "input/projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lingtu::nav::endpoint {

void InputProjector::projectDriverControl(const DriverControlSample &message,
                                          double receive_steady_s) {
  const double stamp_s = message.stamp_s;
  if (classifySourceOrder(state_.driver_control_stamp_s, stamp_s,
                          kSourceClockRebaseThresholdS) == SourceStampDecision::kReject) {
    state_.frames.last_error = "driver_control_stamp_invalid";
    return;
  }

  state_.driver_control_stamp_s = stamp_s;
  state_.driver_control_receive_time = SteadyClock::time_point{
      std::chrono::duration_cast<SteadyClock::duration>(
          std::chrono::duration<double>(receive_steady_s))};
  state_.driver_control_received = true;
  state_.driver_control_reason = message.reason;
  state_.driver_accepted_producer_boot_id = message.accepted_producer_boot_id;
  state_.driver_accepted_output_sequence = message.accepted_output_sequence;
  state_.driver_last_command_accepted = message.last_command_accepted;
  state_.driver_control_ready = message.connected && message.ready && message.motors_enabled &&
                                !message.critical_fault && message.control_assured;

  if (!state_.driver_control_ready && state_.driver_control_reason.empty()) {
    if (!message.connected) {
      state_.driver_control_reason = "disconnected";
    } else if (!message.motors_enabled) {
      state_.driver_control_reason = "motors_disabled";
    } else if (message.critical_fault) {
      state_.driver_control_reason = "motor_fault";
    } else if (!message.control_assured) {
      state_.driver_control_reason = "control_not_assured";
    } else {
      state_.driver_control_reason = "not_ready";
    }
  }
  ++state_.driver_control_generation;
}

double InputProjector::driverAge(SteadyClock::time_point now_time) const {
  if (!state_.driver_control_received) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now_time - state_.driver_control_receive_time).count();
}

std::string InputProjector::driverBlocker(SteadyClock::time_point now_time) const {
  if (state_.driver_control_stamp_s <= 0.0 || !state_.driver_control_received) {
    return "driver_control_missing";
  }
  const double age_s = driverAge(now_time);
  if (config_.driver_control_max_age_s > 0.0 && age_s > config_.driver_control_max_age_s) {
    return "driver_control_stale";
  }
  if (!state_.driver_control_ready) {
    return state_.driver_control_reason.empty()
               ? "driver_control_not_ready"
               : std::string{"driver_control_"} + state_.driver_control_reason;
  }
  return {};
}

InputGateState InputProjector::evaluateGate(double now_steady_s, SteadyClock::time_point now_time) {
  InputSnapshot input;
  input.now_s = now_steady_s;
  input.odom_stamp_s = state_.last_odom_s;
  input.odom_receive_s = state_.last_odom_receive_s;
  input.odom_generation = state_.odom_generation;
  input.odom_linear_speed_mps = state_.last_odom_linear_speed_mps;
  input.tf_stamp_s = state_.last_tf_s;
  input.tf_receive_s = state_.last_tf_receive_s;
  input.tf_generation = state_.tf_generation;
  input.cloud_stamp_s = state_.last_cloud_s;
  input.cloud_receive_s = state_.last_cloud_receive_s;
  input.cloud_generation = state_.cloud_generation;
  input.traversability_stamp_s = state_.last_traversability_s;
  input.traversability_receive_s = state_.last_traversability_receive_s;
  input.traversability_generation = state_.traversability_generation;
  input.localization_health_stamp_s = state_.localization_health.stamp_s;
  input.localization_health_receive_s = state_.localization_health_receive_s;
  input.localization_health_generation = state_.localization_health_generation;

  const double driver_age_s = driverAge(now_time);
  input.driver_control_stamp_s = state_.driver_control_stamp_s;
  input.driver_control_receive_s = state_.driver_control_received && std::isfinite(driver_age_s)
                                       ? now_steady_s - std::max(0.0, driver_age_s)
                                       : 0.0;
  input.driver_control_generation = state_.driver_control_generation;
  input.odom_requires_tf = state_.odom_requires_tf;
  input.localization_healthy = state_.localization_health.healthy;
  input.localization_state = state_.localization_health.state;
  input.localization_reason = state_.localization_health.reason;
  input.driver_control_ready = state_.driver_control_ready;
  input.driver_control_reason = state_.driver_control_reason;

  state_.input_gate_state = gate_.evaluate(input);
  return state_.input_gate_state;
}

void InputProjector::projectLocalizationHealth(std::string payload,
                                               double receive_steady_s) {
  auto health = decodeLocalizationHealth(payload);
  if (!health.valid) {
    state_.frames.last_error = health.error;
    return;
  }

  const auto decision = classifySourceOrder(state_.localization_health.stamp_s, health.stamp_s,
                                            kSourceClockRebaseThresholdS);
  if (decision == SourceStampDecision::kReject) {
    state_.frames.last_error = "localization_health_stamp_invalid";
    return;
  }
  if (decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }

  state_.localization_health = std::move(health);
  state_.localization_health_receive_s = receive_steady_s;
  ++state_.localization_health_generation;
}

}  // namespace lingtu::nav::endpoint
