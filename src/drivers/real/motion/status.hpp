#pragma once

#include "config.hpp"
#include "core.hpp"
#include "body.hpp"
#include "output_ack.hpp"

#include <cstdint>
#include <string>
#include <string_view>

namespace lingtu::driver {

struct MotionRunEvidence {
  bool available{false};
  bool active{false};
  std::uint64_t id{0};
  double start_s{0.0};
  double end_s{0.0};
  std::array<double, 3> start_position_m{};
  std::array<double, 3> end_position_m{};
  bool start_position_available{false};
  bool end_position_available{false};
  std::uint64_t command_samples{0};
  std::uint64_t observed_samples{0};
  double commanded_distance_m{0.0};
  double observed_velocity_distance_m{0.0};
  double last_commanded_linear_mps{0.0};
  double last_observed_linear_mps{0.0};
  bool last_observed_velocity_available{false};
  std::string end_output_kind{"none"};
};

struct RuntimeStats {
  bool connected{false};
  bool ready{false};
  bool cmd_vel_writer_ready{false};
  OutputAckState output_ack;
  std::uint32_t matched_cmd_vel_writers{0};
  Capabilities capabilities;
  ControlState control;
  BodyState body;
  HealthState health;
  std::uint64_t dds_samples{0};
  std::uint64_t backend_calls{0};
  std::uint64_t backend_errors{0};
  std::uint64_t commands_sent{0};
  std::uint64_t zeros_sent{0};
  std::uint64_t dropped_disconnected{0};
  std::uint64_t command_rejections{0};
  std::uint64_t lease_refreshes{0};
  std::uint64_t lease_rejections{0};
  std::uint64_t safety_stops{0};
  std::uint64_t cmd_vel_writer_faults{0};
  std::uint64_t freshness_accepted{0};
  std::uint64_t freshness_rejected{0};
  std::uint64_t freshness_boot_mismatch{0};
  std::uint64_t freshness_producer_boot_missing{0};
  std::uint64_t freshness_producer_rollback{0};
  std::uint64_t freshness_duplicate_sequence{0};
  std::uint64_t freshness_sequence_rollback{0};
  std::uint64_t freshness_future{0};
  std::uint64_t freshness_expired{0};
  double last_receive_s{0.0};
  double last_send_s{0.0};
  std::string last_reason;
  std::string last_output_kind{"none"};
  std::string cmd_vel_writer_reason{"missing_cmd_vel_writer"};
  std::string last_freshness_reason;
  std::string last_error;
  Velocity last_motion_command;
  Velocity last_observed_velocity;
  bool last_observed_velocity_available{false};
  std::uint64_t velocity_tracking_samples{0};
  double commanded_linear_speed_sum{0.0};
  double observed_linear_speed_sum{0.0};
  std::uint64_t motion_run_count{0};
  MotionRunEvidence current_motion_run;
  MotionRunEvidence last_completed_motion_run;
};

void recordFreshnessDecision(
    RuntimeStats& stats,
    const CommandFreshnessDecision& decision) noexcept;

void recordVelocityTracking(
    RuntimeStats& stats,
    const Velocity& command,
    const BodyState& body) noexcept;

void recordMotionOutput(
    RuntimeStats& stats,
    const Velocity& command,
    const BodyState& body,
    std::string_view output_kind,
    double stamp_s) noexcept;

void writeStatus(
    const Config& config,
    const Core& core,
    const RuntimeStats& stats,
    const AdapterDiagnostics& adapter,
    double stamp_s);

}  // namespace lingtu::driver
