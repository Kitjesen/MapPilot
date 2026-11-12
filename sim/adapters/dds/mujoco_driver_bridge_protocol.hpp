#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>

namespace lingtu::sim::driver_bridge {

inline constexpr std::size_t kMaxProtocolLineBytes = 512;
inline constexpr std::size_t kMaxProducerTokenBytes = 128;

class ProtocolError final : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

enum class BridgeCommandKind {
  ActivationZero,
  Nav,
  DeactivateZero,
  WriterFaultZero,
  SafetyZero,
};

enum class BridgeFaultCode {
  None,
  ProtocolViolation,
  ControllerEof,
  HeartbeatTimeout,
  ApplyTimeout,
  WriterMissing,
  WriterAmbiguous,
  CommandSequenceOverflow,
};

struct BridgeWalk {
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct BridgeCommand {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t bridge_command_seq{0};
  BridgeCommandKind kind{BridgeCommandKind::ActivationZero};
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
  BridgeWalk walk;
};

struct ActivateMessage {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t control_seq{0};
};

struct AppliedMessage {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t bridge_command_seq{0};
  BridgeCommandKind kind{BridgeCommandKind::ActivationZero};
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
  double walk_x{0.0};
  double walk_y{0.0};
  double walk_z{0.0};
  std::uint64_t applied_step_seq{0};
};

struct HeartbeatMessage {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t control_seq{0};
  std::uint64_t step_seq{0};
};

struct DeactivateMessage {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t control_seq{0};
};

struct ReadyMessage {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t accepted_sequence{0};
};

struct HelloMessage {
  std::string bridge_boot_id;
};

struct FaultMessage {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  BridgeFaultCode fault{BridgeFaultCode::None};
};

struct StopMessage {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t bridge_command_seq{0};
  std::uint64_t applied_step_seq{0};
  BridgeCommandKind kind{BridgeCommandKind::DeactivateZero};
};

struct ProfileRequest {
  std::string task_id;
  std::string requirement;
};

struct ProfileState {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::string task_id;
  std::string requirement;
  std::string phase;
  std::string requested_profile;
  std::string active_profile;
  std::string reason;
};

using ControllerMessage =
    std::variant<ActivateMessage, AppliedMessage, HeartbeatMessage, DeactivateMessage, ProfileState>;

[[nodiscard]] bool validBootId(std::string_view value) noexcept;
[[nodiscard]] bool validProducerToken(std::string_view value) noexcept;
[[nodiscard]] const char *commandKindName(BridgeCommandKind kind) noexcept;
[[nodiscard]] const char *faultCodeName(BridgeFaultCode fault) noexcept;

[[nodiscard]] ControllerMessage parseControllerLine(std::string_view line);
[[nodiscard]] std::string serializeCommand(const BridgeCommand &command);
[[nodiscard]] std::string serializeHello(const HelloMessage &hello);
[[nodiscard]] std::string serializeReady(const ReadyMessage &ready);
[[nodiscard]] std::string serializeFault(const FaultMessage &fault);
[[nodiscard]] std::string serializeStopped(const StopMessage &stopped);
[[nodiscard]] std::string serializeProfileRequest(const ProfileRequest &request,
                                                 std::string_view bridge_boot_id,
                                                 std::string_view controller_boot_id);

}  // namespace lingtu::sim::driver_bridge
