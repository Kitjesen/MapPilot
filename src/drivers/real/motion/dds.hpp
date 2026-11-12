#pragma once

#include "command_freshness_gate.hpp"
#include "body.hpp"
#include "profile_switch.hpp"
#include "message/protocol/posture.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace lingtu::driver {

struct TwistSample {
  std::string host_boot_id;
  std::string producer_boot_id;
  std::uint64_t source_boottime_ns{0};
  std::uint64_t receive_boottime_ns{0};
  TimePoint received_at{};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};

  CommandFreshnessInput freshnessInput() const noexcept {
    return CommandFreshnessInput{
        host_boot_id,
        producer_boot_id,
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
  std::optional<ProfileRequest> takeProfileRequest(double wall_s);
  std::optional<message::PostureRequest> takePostureRequest();
  bool writePostureAck(const message::PostureAck &ack, double stamp_s);
  struct ExternalRead {
    std::optional<message::ExternalFrame> frame;
    std::string heartbeat_session;
  };
  ExternalRead takeExternal();
  bool writeExternalState(const message::ExternalState &state, double stamp_s);
  bool writeProfileState(const ProfileStatus &state, double stamp_s);
  bool writeControlState(
      const ControlState& state,
      bool zero_output,
      double stamp_s);
  bool writeJointState(const JointState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::driver
