#pragma once

#include "brainstem.hpp"
#include "config.hpp"
#include "core.hpp"

#include <cstdint>
#include <string>

namespace lingtu::driver {

struct RuntimeStats {
  bool connected{false};
  bool ready{false};
  bool last_command_accepted{false};
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
  std::uint64_t safety_cancels{0};
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
  std::string last_freshness_reason;
  std::string last_error;
};

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
