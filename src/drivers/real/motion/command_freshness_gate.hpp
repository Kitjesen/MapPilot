#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_set>

namespace lingtu::driver {

using Boottime = std::chrono::steady_clock::duration;

struct CommandFreshnessInput {
  std::string_view host_boot_id;
  std::string_view producer_boot_id;
  std::uint64_t seq{0};
  Boottime source_boottime{};
  Boottime receive_boottime{};
};

enum class CommandFreshnessReason {
  Accepted,
  BootMismatch,
  ProducerBootMissing,
  ProducerRollback,
  DuplicateSequence,
  SequenceRollback,
  Future,
  Expired,
};

struct CommandFreshnessDecision {
  bool accepted{false};
  CommandFreshnessReason reason{CommandFreshnessReason::BootMismatch};
};

class CommandFreshnessGate {
 public:
  CommandFreshnessGate(
      Boottime max_age,
      std::string expected_host_boot_id);

  CommandFreshnessDecision evaluate(
      const CommandFreshnessInput& input) noexcept;

 private:
  Boottime max_age_;
  std::string expected_host_boot_id_;
  std::string current_producer_boot_id_;
  std::unordered_set<std::string> retired_producer_boot_ids_;
  Boottime last_accepted_source_boottime_{};
  std::uint64_t last_accepted_seq_{0};
  bool has_accepted_{false};
};

const char* freshnessReasonName(CommandFreshnessReason reason) noexcept;

}  // namespace lingtu::driver
