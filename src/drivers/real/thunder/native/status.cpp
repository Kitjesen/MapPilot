#include "status.hpp"

#include "message/cpp/dds_topics.hpp"

#include <filesystem>
#include <fstream>
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

}  // namespace

void recordFreshnessDecision(
    RuntimeStats& stats,
    const CommandFreshnessDecision& decision) noexcept {
  stats.last_freshness_reason = freshnessReasonName(decision.reason);
  if (decision.accepted) {
    ++stats.freshness_accepted;
    return;
  }
  invalidateCurrentOutputAck(stats);
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

void writeStatus(
    const Config& config,
    const Core& core,
    const RuntimeStats& stats,
    const std::string& target,
    double stamp_s) {
  if (config.status_file.empty()) {
    return;
  }
  const std::filesystem::path path(config.status_file);
  if (!path.parent_path().empty()) {
    std::error_code mkdir_error;
    std::filesystem::create_directories(path.parent_path(), mkdir_error);
  }
  const std::filesystem::path temporary = path.string() + ".tmp";
  std::ofstream out(temporary, std::ios::trunc);
  if (!out) {
    return;
  }
  const auto& counts = core.counters();
  const auto& walk = core.lastWalk();
  const auto output_ack = currentOutputAck(stats);
  out << "{\n"
      << "  \"schema_version\": \"lingtu.driver.status.v1\",\n"
      << "  \"role\": \"driver\",\n"
      << "  \"backend\": \"thunder\",\n"
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
      << jsonEscape(stats.cmd_vel_writer_reason) << "\"},\n"
      << "  \"brainstem\": {\"target\": \"" << jsonEscape(target)
      << "\", \"rpc_timeout_ms\": " << config.rpc_timeout.count()
      << ", \"fsm\": \"" << jsonEscape(stats.control.fsm)
      << "\", \"motors_enabled\": "
      << (stats.control.motors_enabled ? "true" : "false")
      << ", \"critical_fault\": "
      << (stats.control.critical_fault ? "true" : "false")
      << ", \"lease_valid\": "
      << (stats.control.lease_valid ? "true" : "false")
      << ", \"initial_zero_acknowledged\": "
      << (stats.control.initial_zero_acknowledged ? "true" : "false")
      << ", \"lease_remaining_ms\": " << stats.control.lease_remaining_ms
      << ", \"owner\": \"" << jsonEscape(stats.control.owner)
      << "\", \"owner_id\": \"" << jsonEscape(stats.control.owner_id)
      << "\", \"decision\": \"" << jsonEscape(stats.control.reason)
      << "\"},\n"
      << "  \"watchdog\": {\"timeout_ms\": "
      << config.limits.command_timeout.count() << "},\n"
      << "  \"limits\": {\"linear_mps\": " << config.limits.max_linear_mps
      << ", \"angular_rps\": " << config.limits.max_angular_rps << "},\n"
      << "  \"last_walk\": {\"x\": " << walk.x << ", \"y\": " << walk.y
      << ", \"z\": " << walk.z << "},\n"
      << "  \"last_reason\": \"" << jsonEscape(stats.last_reason) << "\",\n"
      << "  \"last_freshness_reason\": \""
      << jsonEscape(stats.last_freshness_reason) << "\",\n"
      << "  \"last_error\": \"" << jsonEscape(stats.last_error) << "\",\n"
      << "  \"last_receive_s\": " << stats.last_receive_s << ",\n"
      << "  \"last_send_s\": " << stats.last_send_s << ",\n"
      << "  \"output_ack\": {\"producer_boot_id\": \""
      << jsonEscape(output_ack.producer_boot_id)
      << "\", \"output_sequence\": " << output_ack.output_sequence
      << ", \"accepted\": "
      << (output_ack.accepted ? "true" : "false") << "},\n"
      << "  \"counters\": {\n"
      << "    \"dds_samples\": " << stats.dds_samples << ",\n"
      << "    \"received\": " << counts.received << ",\n"
      << "    \"accepted\": " << counts.accepted << ",\n"
      << "    \"rejected_frame\": " << counts.rejected_frame << ",\n"
      << "    \"rejected_nonfinite\": " << counts.rejected_nonfinite << ",\n"
      << "    \"watchdog_stops\": " << counts.watchdog_stops << ",\n"
      << "    \"invalid_stops\": " << counts.invalid_stops << ",\n"
      << "    \"rpc_attempts\": " << stats.rpc_attempts << ",\n"
      << "    \"rpc_errors\": " << stats.rpc_errors << ",\n"
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
}

}  // namespace lingtu::driver
