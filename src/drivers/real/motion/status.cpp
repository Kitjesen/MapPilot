#include "status.hpp"

#include "message/cpp/topics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <system_error>

namespace lingtu::driver {
namespace {

std::string jsonEscape(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (static_cast<unsigned char>(ch) >= 0x20) {
          out += ch;
        }
        break;
    }
  }
  return out;
}

void writeControlStatus(
    std::ostream& out,
    const char* label,
    const RuntimeStats& stats) {
  out << "  \"" << label << "\": {\"fsm\": \"" << jsonEscape(stats.control.fsm)
      << "\", \"motors_enabled\": "
      << (stats.control.motors_enabled ? "true" : "false")
      << ", \"critical_fault\": "
      << (stats.control.critical_fault ? "true" : "false")
      << ", \"control_assured\": "
      << (stats.control.control_assured ? "true" : "false")
      << ", \"lease_valid\": "
      << (stats.control.lease_valid ? "true" : "false")
      << ", \"initial_zero_acknowledged\": "
      << (stats.control.initial_zero_acknowledged ? "true" : "false")
      << ", \"lease_remaining_ms\": " << stats.control.lease_remaining_ms
      << ", \"decision\": \"" << jsonEscape(stats.control.reason)
      << "\"},\n";
}

double planarDisplacement(const MotionRunEvidence& run) noexcept {
  if (!run.start_position_available || !run.end_position_available) {
    return 0.0;
  }
  return std::hypot(
      run.end_position_m[0] - run.start_position_m[0],
      run.end_position_m[1] - run.start_position_m[1]);
}

void writeMotionRun(
    std::ostream& out,
    const char* label,
    const MotionRunEvidence& run) {
  const double duration_s =
      run.available ? std::max(0.0, run.end_s - run.start_s) : 0.0;
  const bool odometry_available =
      run.start_position_available && run.end_position_available;
  const double odometry_displacement_m = planarDisplacement(run);
  out << "  \"" << label << "\": {\"available\": "
      << (run.available ? "true" : "false")
      << ", \"active\": " << (run.active ? "true" : "false")
      << ", \"id\": " << run.id
      << ", \"start_s\": " << run.start_s
      << ", \"end_s\": " << run.end_s
      << ", \"duration_s\": " << duration_s
      << ", \"command_samples\": " << run.command_samples
      << ", \"observed_samples\": " << run.observed_samples
      << ", \"commanded_distance_m\": " << run.commanded_distance_m
      << ", \"observed_velocity_distance_m\": "
      << run.observed_velocity_distance_m
      << ", \"odometry_available\": "
      << (odometry_available ? "true" : "false")
      << ", \"start_position_m\": {\"x_m\": " << run.start_position_m[0]
      << ", \"y_m\": " << run.start_position_m[1]
      << ", \"z_m\": " << run.start_position_m[2]
      << "}, \"end_position_m\": {\"x_m\": " << run.end_position_m[0]
      << ", \"y_m\": " << run.end_position_m[1]
      << ", \"z_m\": " << run.end_position_m[2]
      << "}, \"odometry_displacement_m\": " << odometry_displacement_m
      << ", \"odometry_to_command_ratio\": ";
  if (!odometry_available || run.commanded_distance_m <= 1e-9) {
    out << "null";
  } else {
    out << odometry_displacement_m / run.commanded_distance_m;
  }
  out << ", \"end_output_kind\": \"" << jsonEscape(run.end_output_kind)
      << "\"}";
}

void advanceMotionRun(MotionRunEvidence& run, double stamp_s) noexcept {
  if (!run.active || !std::isfinite(stamp_s) || stamp_s <= run.end_s) {
    return;
  }
  const double elapsed_s = stamp_s - run.end_s;
  run.commanded_distance_m += run.last_commanded_linear_mps * elapsed_s;
  if (run.last_observed_velocity_available) {
    run.observed_velocity_distance_m +=
        run.last_observed_linear_mps * elapsed_s;
  }
  run.end_s = stamp_s;
}

}  // namespace

void recordFreshnessDecision(
    RuntimeStats& stats,
    const CommandFreshnessDecision& decision) noexcept {
  stats.last_freshness_reason = freshnessReasonName(decision.reason);
  if (decision.accepted) {
    ++stats.freshness_accepted;
    return;
  }
  stats.output_ack.invalidate();
  ++stats.freshness_rejected;
  switch (decision.reason) {
    case CommandFreshnessReason::Accepted:
      break;
    case CommandFreshnessReason::BootMismatch:
      ++stats.freshness_boot_mismatch;
      break;
    case CommandFreshnessReason::ProducerBootMissing:
      ++stats.freshness_producer_boot_missing;
      break;
    case CommandFreshnessReason::ProducerRollback:
      ++stats.freshness_producer_rollback;
      break;
    case CommandFreshnessReason::DuplicateSequence:
      ++stats.freshness_duplicate_sequence;
      break;
    case CommandFreshnessReason::SequenceRollback:
      ++stats.freshness_sequence_rollback;
      break;
    case CommandFreshnessReason::Future:
      ++stats.freshness_future;
      break;
    case CommandFreshnessReason::Expired:
      ++stats.freshness_expired;
      break;
  }
}

void recordVelocityTracking(
    RuntimeStats& stats,
    const Velocity& command,
    const BodyState& body) noexcept {
  const double commanded_linear = std::hypot(command.vx_mps, command.vy_mps);
  if (commanded_linear <= 1e-9 && std::abs(command.yaw_rps) <= 1e-9) {
    return;
  }
  stats.last_motion_command = command;
  if (!body.velocity_available) {
    stats.last_observed_velocity_available = false;
    return;
  }
  stats.last_observed_velocity = body.velocity;
  stats.last_observed_velocity_available = true;
  ++stats.velocity_tracking_samples;
  stats.commanded_linear_speed_sum += commanded_linear;
  stats.observed_linear_speed_sum += std::hypot(body.velocity.vx_mps, body.velocity.vy_mps);
}

void recordMotionOutput(
    RuntimeStats& stats,
    const Velocity& command,
    const BodyState& body,
    std::string_view output_kind,
    double stamp_s) noexcept {
  const double commanded_linear = std::hypot(command.vx_mps, command.vy_mps);
  const bool moving = commanded_linear > 1e-9 || std::abs(command.yaw_rps) > 1e-9;
  auto& run = stats.current_motion_run;
  if (moving) {
    recordVelocityTracking(stats, command, body);
    if (!run.active) {
      run = {};
      run.available = true;
      run.active = true;
      run.id = ++stats.motion_run_count;
      run.start_s = stamp_s;
      run.end_s = stamp_s;
    } else {
      advanceMotionRun(run, stamp_s);
    }
    if (!run.start_position_available && body.odometry_position_available) {
      run.start_position_m = body.odometry_position_m;
      run.start_position_available = true;
    }
    if (body.odometry_position_available) {
      run.end_position_m = body.odometry_position_m;
      run.end_position_available = true;
    }
    ++run.command_samples;
    run.last_commanded_linear_mps = commanded_linear;
    run.last_observed_velocity_available = body.velocity_available;
    if (body.velocity_available) {
      ++run.observed_samples;
      run.last_observed_linear_mps =
          std::hypot(body.velocity.vx_mps, body.velocity.vy_mps);
    }
    return;
  }
  if (!run.active) {
    return;
  }
  advanceMotionRun(run, stamp_s);
  if (body.odometry_position_available) {
    run.end_position_m = body.odometry_position_m;
    run.end_position_available = true;
  }
  run.active = false;
  run.end_output_kind = std::string(output_kind);
  stats.last_completed_motion_run = run;
  stats.current_motion_run = {};
}

void writeStatus(
    const Config& config,
    const Core& core,
    const RuntimeStats& stats,
    const AdapterDiagnostics& adapter,
    double stamp_s) {
  if (config.status_file.empty()) {
    return;
  }
  const std::filesystem::path path(config.status_file);
  if (!path.parent_path().empty()) {
    std::error_code mkdir_error;
    std::filesystem::create_directories(path.parent_path(), mkdir_error);
    if (mkdir_error) {
      std::fprintf(
          stderr,
          "lingtu_driver: cannot create status directory %s: %s\n",
          path.parent_path().string().c_str(),
          mkdir_error.message().c_str());
      return;
    }
  }
  const std::filesystem::path temporary = path.string() + ".tmp";
  std::ofstream out(temporary, std::ios::trunc);
  if (!out) {
    std::fprintf(
        stderr,
        "lingtu_driver: cannot open status file %s\n",
        temporary.string().c_str());
    return;
  }
  out << std::fixed << std::setprecision(6);
  const auto& counts = core.counters();
  const auto& velocity = core.lastVelocity();
  const auto output_ack = stats.output_ack.current(OutputAckState::Clock::now());
  const double mean_commanded_linear =
      stats.velocity_tracking_samples == 0
          ? 0.0
          : stats.commanded_linear_speed_sum /
                static_cast<double>(stats.velocity_tracking_samples);
  const double mean_observed_linear =
      stats.velocity_tracking_samples == 0
          ? 0.0
          : stats.observed_linear_speed_sum /
                static_cast<double>(stats.velocity_tracking_samples);
  out << "{\n"
      << "  \"schema_version\": \"lingtu.driver.status.v2\",\n"
      << "  \"role\": \"driver\",\n"
      << "  \"backend\": \"" << jsonEscape(adapter.name) << "\",\n"
      << "  \"ready\": " << (stats.ready ? "true" : "false") << ",\n"
      << "  \"connected\": " << (stats.connected ? "true" : "false") << ",\n"
      << "  \"active\": " << (core.active() ? "true" : "false") << ",\n"
      << "  \"stamp_s\": " << stamp_s << ",\n"
      << "  \"dds\": {\"topic\": \"" << lingtu::message::kNavCmdVel.topic
      << "\", \"wire_topic\": \"" << lingtu::message::kNavCmdVel.dds_topic
      << "\", \"domain_id\": " << config.domain_id
      << ", \"matched_cmd_vel_writers\": "
      << stats.matched_cmd_vel_writers
      << ", \"cmd_vel_writer_ready\": "
      << (stats.cmd_vel_writer_ready ? "true" : "false")
      << ", \"cmd_vel_writer_reason\": \""
      << jsonEscape(stats.cmd_vel_writer_reason) << "\"},\n";
  out << "  \"adapter\": {\"protocol\": \"" << jsonEscape(adapter.protocol)
      << "\", \"target\": \"" << jsonEscape(adapter.target)
      << "\", \"control_owner\": \"" << jsonEscape(adapter.control_owner)
      << "\", \"control_owner_id\": \"" << jsonEscape(adapter.control_owner_id)
      << "\", \"state_code\": ";
  if (adapter.state_code_available) {
    out << adapter.state_code;
  } else {
    out << "null";
  }
  out << ", \"timeout_ms\": " << config.rpc_timeout.count() << "},\n";
  writeControlStatus(out, "control", stats);
  out
      << "  \"capabilities\": {\"stand\": "
      << (stats.capabilities.stand ? "true" : "false")
      << ", \"sit\": " << (stats.capabilities.sit ? "true" : "false")
      << ", \"recover\": " << (stats.capabilities.recover ? "true" : "false")
      << ", \"damp\": " << (stats.capabilities.damp ? "true" : "false") << "},\n"
      << "  \"body\": {\"fresh\": " << (stats.body.fresh ? "true" : "false")
      << ", \"posture\": \"" << postureName(stats.body.posture)
      << "\", \"velocity_available\": "
      << (stats.body.velocity_available ? "true" : "false")
      << ", \"velocity\": {\"vx_mps\": " << stats.body.velocity.vx_mps
      << ", \"vy_mps\": " << stats.body.velocity.vy_mps
      << ", \"yaw_rps\": " << stats.body.velocity.yaw_rps
      << "}, \"odometry_position_available\": "
      << (stats.body.odometry_position_available ? "true" : "false")
      << ", \"odometry_position_m\": {\"x_m\": "
      << stats.body.odometry_position_m[0]
      << ", \"y_m\": " << stats.body.odometry_position_m[1]
      << ", \"z_m\": " << stats.body.odometry_position_m[2]
      << "}, \"height_available\": "
      << (stats.body.height_available ? "true" : "false")
      << ", \"height_m\": " << stats.body.height_m << "},\n"
      << "  \"health\": {\"fresh\": " << (stats.health.fresh ? "true" : "false")
      << ", \"healthy\": " << (stats.health.healthy ? "true" : "false")
      << ", \"fault_code\": " << stats.health.fault_code
      << ", \"reason\": \"" << jsonEscape(stats.health.reason) << "\"},\n"
      << "  \"watchdog\": {\"timeout_ms\": "
      << config.limits.command_timeout.count() << "},\n"
      << "  \"limits\": {\"linear_mps\": " << config.limits.max_linear_mps
      << ", \"angular_rps\": " << config.limits.max_angular_rps << "},\n"
      << "  \"last_velocity\": {\"vx_mps\": " << velocity.vx_mps
      << ", \"vy_mps\": " << velocity.vy_mps
      << ", \"yaw_rps\": " << velocity.yaw_rps << "},\n"
      << "  \"velocity_tracking\": {\"last_commanded\": {\"vx_mps\": "
      << stats.last_motion_command.vx_mps
      << ", \"vy_mps\": " << stats.last_motion_command.vy_mps
      << ", \"yaw_rps\": " << stats.last_motion_command.yaw_rps
      << "}, \"last_observed_available\": "
      << (stats.last_observed_velocity_available ? "true" : "false")
      << ", \"last_observed\": {\"vx_mps\": "
      << stats.last_observed_velocity.vx_mps
      << ", \"vy_mps\": " << stats.last_observed_velocity.vy_mps
      << ", \"yaw_rps\": " << stats.last_observed_velocity.yaw_rps
      << "}, \"tracking_samples\": " << stats.velocity_tracking_samples
      << ", \"mean_commanded_linear_mps\": " << mean_commanded_linear
      << ", \"mean_observed_linear_mps\": " << mean_observed_linear
      << ", \"observed_to_command_ratio\": ";
  if (stats.velocity_tracking_samples == 0 || mean_commanded_linear <= 1e-9) {
    out << "null";
  } else {
    out << mean_observed_linear / mean_commanded_linear;
  }
  out << "},\n";
  writeMotionRun(out, "active_motion_run", stats.current_motion_run);
  out << ",\n";
  writeMotionRun(out, "last_completed_motion_run", stats.last_completed_motion_run);
  out << ",\n"
      << "  \"last_reason\": \"" << jsonEscape(stats.last_reason) << "\",\n"
      << "  \"last_output_kind\": \"" << jsonEscape(stats.last_output_kind)
      << "\",\n"
      << "  \"last_freshness_reason\": \""
      << jsonEscape(stats.last_freshness_reason) << "\",\n"
      << "  \"last_error\": \"" << jsonEscape(stats.last_error) << "\",\n"
      << "  \"last_receive_s\": " << stats.last_receive_s << ",\n"
      << "  \"last_send_s\": " << stats.last_send_s << ",\n"
      << "  \"output_ack\": {\"producer_boot_id\": \""
      << jsonEscape(output_ack.producerBootId())
      << "\", \"output_sequence\": " << output_ack.outputSequence()
      << ", \"accepted\": "
      << (output_ack.accepted() ? "true" : "false") << "},\n"
      << "  \"counters\": {\n"
      << "    \"dds_samples\": " << stats.dds_samples << ",\n"
      << "    \"received\": " << counts.received << ",\n"
      << "    \"accepted\": " << counts.accepted << ",\n"
      << "    \"rejected_frame\": " << counts.rejected_frame << ",\n"
      << "    \"rejected_nonfinite\": " << counts.rejected_nonfinite << ",\n"
      << "    \"watchdog_stops\": " << counts.watchdog_stops << ",\n"
      << "    \"invalid_stops\": " << counts.invalid_stops << ",\n"
      << "    \"backend_calls\": " << stats.backend_calls << ",\n"
      << "    \"backend_errors\": " << stats.backend_errors << ",\n"
      << "    \"commands_sent\": " << stats.commands_sent << ",\n"
      << "    \"zeros_sent\": " << stats.zeros_sent << ",\n"
      << "    \"dropped_disconnected\": " << stats.dropped_disconnected << ",\n"
      << "    \"command_rejections\": " << stats.command_rejections << ",\n"
      << "    \"lease_refreshes\": " << stats.lease_refreshes << ",\n"
      << "    \"lease_rejections\": " << stats.lease_rejections << ",\n"
      << "    \"safety_stops\": " << stats.safety_stops << ",\n"
      << "    \"cmd_vel_writer_faults\": "
      << stats.cmd_vel_writer_faults << ",\n"
      << "    \"freshness_accepted\": " << stats.freshness_accepted << ",\n"
      << "    \"freshness_rejected\": " << stats.freshness_rejected << ",\n"
      << "    \"freshness_boot_mismatch\": "
      << stats.freshness_boot_mismatch << ",\n"
      << "    \"freshness_producer_boot_missing\": "
      << stats.freshness_producer_boot_missing << ",\n"
      << "    \"freshness_producer_rollback\": "
      << stats.freshness_producer_rollback << ",\n"
      << "    \"freshness_duplicate_sequence\": "
      << stats.freshness_duplicate_sequence << ",\n"
      << "    \"freshness_sequence_rollback\": "
      << stats.freshness_sequence_rollback << ",\n"
      << "    \"freshness_future\": " << stats.freshness_future << ",\n"
      << "    \"freshness_expired\": " << stats.freshness_expired << "\n"
      << "  }\n"
      << "}\n";
  out.close();
  std::error_code rename_error;
  std::filesystem::rename(temporary, path, rename_error);
  if (rename_error) {
    std::error_code remove_error;
    std::filesystem::remove(path, remove_error);
    rename_error.clear();
    std::filesystem::rename(temporary, path, rename_error);
  }
  if (rename_error) {
    std::fprintf(
        stderr,
        "lingtu_driver: cannot publish status file %s: %s\n",
        path.string().c_str(),
        rename_error.message().c_str());
  }
}

}  // namespace lingtu::driver
