#pragma once

#include "command_freshness_gate.hpp"
#include "body.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace lingtu::driver {

struct TwistSample {
  std::string host_boot_id;
  std::string producer_boot_id;
  std::uint64_t output_seq{0};
  std::uint64_t source_boottime_ns{0};
  std::uint64_t source_wall_ns{0};
  std::uint64_t receive_boottime_ns{0};
  std::string frame;
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  CommandFreshnessInput freshnessInput() const noexcept {
    return CommandFreshnessInput{
        host_boot_id,
        producer_boot_id,
        output_seq,
        std::chrono::duration_cast<Boottime>(
            std::chrono::nanoseconds(source_boottime_ns)),
        std::chrono::duration_cast<Boottime>(
            std::chrono::nanoseconds(receive_boottime_ns)),
    };
  }
};

struct ReadResult {
  std::optional<TwistSample> latest;
  std::uint64_t valid_samples{0};
};

class DdsReader {
 public:
  explicit DdsReader(int domain_id);
  ~DdsReader();

  DdsReader(const DdsReader&) = delete;
  DdsReader& operator=(const DdsReader&) = delete;

  const std::string& hostBootId() const noexcept;
  std::uint32_t matchedCommandWriters() const;
  ReadResult takeLatest();
  bool writeControlState(
      const ControlState& state,
      bool last_command_accepted,
      const std::string& accepted_producer_boot_id,
      std::uint64_t accepted_output_sequence,
      double stamp_s);
  bool writeNavigationStop(
      const std::string& reason,
      std::uint64_t sequence,
      double stamp_s);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::driver
