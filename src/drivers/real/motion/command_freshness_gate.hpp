#pragma once

#include <chrono>
#include <string>
#include <string_view>

namespace lingtu::driver {

using Boottime = std::chrono::steady_clock::duration;

struct CommandFreshnessInput {
  std::string_view host_boot_id;
  std::string_view producer_boot_id;
  Boottime source_boottime{};
  Boottime receive_boottime{};
};

enum class CommandFreshnessReason {
  Accepted,
  BootMismatch,
  ProducerBootMissing,
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
};

const char* freshnessReasonName(CommandFreshnessReason reason) noexcept;

}  // namespace lingtu::driver
