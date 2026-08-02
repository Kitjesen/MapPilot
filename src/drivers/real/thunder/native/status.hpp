#pragma once

#include "brainstem.hpp"
#include "config.hpp"
#include "core.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace lingtu::driver {

struct RuntimeStats {
  bool connected{false};
  bool ready{false};
  bool cmd_vel_writer_ready{false};
  bool last_command_accepted{false};
  bool current_output_evidence_valid{false};
  std::uint32_t matched_cmd_vel_writers{0};
  std::string accepted_producer_boot_id;
  std::uint64_t accepted_output_sequence{0};
  std::chrono::steady_clock::time_point rejected_output_evidence_expires_at{};
  ControlState control;
  std::uint64_t dds_samples{0};
  std::uint64_t rpc_attempts{0};
  std::uint64_t rpc_errors{0};
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
  std::string cmd_vel_writer_reason{"missing_cmd_vel_writer"};
  std::string last_freshness_reason;
  std::string last_error;
};

struct OutputAckEvidence {
  bool accepted{false};
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
};

inline constexpr auto kRejectedOutputEvidenceLifetime =
    std::chrono::milliseconds(250);

inline OutputAckEvidence currentOutputAck(
    const RuntimeStats& stats,
    std::chrono::steady_clock::time_point now =
        std::chrono::steady_clock::now()) {
  if (!stats.current_output_evidence_valid ||
      stats.accepted_producer_boot_id.empty() ||
      stats.accepted_output_sequence == 0 ||
      (!stats.last_command_accepted &&
       now >= stats.rejected_output_evidence_expires_at)) {
    return {};
  }
  return {
      stats.last_command_accepted,
      stats.accepted_producer_boot_id,
      stats.accepted_output_sequence,
  };
}

inline void invalidateCurrentOutputAck(RuntimeStats& stats) noexcept {
  stats.current_output_evidence_valid = false;
  stats.last_command_accepted = false;
  stats.accepted_producer_boot_id.clear();
  stats.accepted_output_sequence = 0;
  stats.rejected_output_evidence_expires_at = {};
}

inline void recordCurrentOutputResult(
    RuntimeStats& stats,
    const std::string& producer_boot_id,
    std::uint64_t output_sequence,
    bool accepted,
    std::chrono::steady_clock::time_point now =
        std::chrono::steady_clock::now()) {
  invalidateCurrentOutputAck(stats);
  if (producer_boot_id.empty() || output_sequence == 0) {
    return;
  }
  stats.current_output_evidence_valid = true;
  stats.last_command_accepted = accepted;
  stats.accepted_producer_boot_id = producer_boot_id;
  stats.accepted_output_sequence = output_sequence;
  if (!accepted) {
    stats.rejected_output_evidence_expires_at =
        now + kRejectedOutputEvidenceLifetime;
  }
}

inline void expireCurrentOutputEvidence(
    RuntimeStats& stats,
    std::chrono::steady_clock::time_point now =
        std::chrono::steady_clock::now()) noexcept {
  if (stats.current_output_evidence_valid &&
      !stats.last_command_accepted &&
      now >= stats.rejected_output_evidence_expires_at) {
    invalidateCurrentOutputAck(stats);
  }
}

void recordFreshnessDecision(
    RuntimeStats& stats,
    const CommandFreshnessDecision& decision) noexcept;

void writeStatus(
    const Config& config,
    const Core& core,
    const RuntimeStats& stats,
    const std::string& target,
    double stamp_s);

}  // namespace lingtu::driver
