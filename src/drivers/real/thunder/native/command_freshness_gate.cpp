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

  const bool producer_changed =
      has_accepted_ && input.producer_boot_id != current_producer_boot_id_;
  if (producer_changed &&
      (retired_producer_boot_ids_.find(std::string(input.producer_boot_id)) !=
           retired_producer_boot_ids_.end() ||
       input.source_boottime <= last_accepted_source_boottime_)) {
    return {false, CommandFreshnessReason::ProducerRollback};
  }
  if (!producer_changed && has_accepted_ &&
      input.seq == last_accepted_seq_) {
    return {false, CommandFreshnessReason::DuplicateSequence};
  }
  if (!producer_changed && has_accepted_ &&
      input.seq < last_accepted_seq_) {
    return {false, CommandFreshnessReason::SequenceRollback};
  }

  if (producer_changed) {
    retired_producer_boot_ids_.insert(current_producer_boot_id_);
  }
  current_producer_boot_id_.assign(input.producer_boot_id);
  last_accepted_source_boottime_ = input.source_boottime;
  last_accepted_seq_ = input.seq;
  has_accepted_ = true;
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
    case CommandFreshnessReason::ProducerRollback:
      return "producer_rollback";
    case CommandFreshnessReason::DuplicateSequence:
      return "duplicate_sequence";
    case CommandFreshnessReason::SequenceRollback:
      return "sequence_rollback";
    case CommandFreshnessReason::Future:
      return "future";
    case CommandFreshnessReason::Expired:
      return "expired";
  }
  return "unknown";
}

}  // namespace lingtu::driver
