#include "command_freshness_gate.hpp"

#include <stdexcept>
#include <utility>

namespace lingtu::driver {

CommandFreshnessGate::CommandFreshnessGate(
    Boottime max_age,
    std::string expected_host_boot_id)
    : max_age_(max_age),
      expected_host_boot_id_(std::move(expected_host_boot_id)) {
  if (max_age_ <= Boottime::zero()) {
    throw std::invalid_argument("command freshness max age must be positive");
  }
  if (expected_host_boot_id_.empty()) {
    throw std::invalid_argument(
        "command freshness expected host boot id must not be empty");
  }
}

CommandFreshnessDecision CommandFreshnessGate::evaluate(
    const CommandFreshnessInput& input) noexcept {
  if (input.host_boot_id.empty() ||
      input.host_boot_id != expected_host_boot_id_) {
    return {false, CommandFreshnessReason::BootMismatch};
  }
  if (input.producer_boot_id.empty()) {
    return {false, CommandFreshnessReason::ProducerBootMissing};
  }
  if (input.source_boottime > input.receive_boottime) {
    return {false, CommandFreshnessReason::Future};
  }
  if (input.receive_boottime - input.source_boottime >= max_age_) {
    return {false, CommandFreshnessReason::Expired};
  }

  return {true, CommandFreshnessReason::Accepted};
}

const char* freshnessReasonName(CommandFreshnessReason reason) noexcept {
  switch (reason) {
    case CommandFreshnessReason::Accepted:
      return "accepted";
    case CommandFreshnessReason::BootMismatch:
      return "boot_mismatch";
    case CommandFreshnessReason::ProducerBootMissing:
      return "producer_boot_missing";
    case CommandFreshnessReason::Future:
      return "future";
    case CommandFreshnessReason::Expired:
      return "expired";
  }
  return "unknown";
}

}  // namespace lingtu::driver
