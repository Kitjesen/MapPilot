#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "cmd_vel_writer_gate.hpp"
#include "core.hpp"
#include "mujoco_driver_bridge_protocol.hpp"
#include "output_ack.hpp"

namespace lingtu::sim::driver_bridge {

using Clock = lingtu::driver::Clock;
using TimePoint = lingtu::driver::TimePoint;

enum class BridgeLifecycle {
  AwaitWriter,
  AwaitController,
  ActivatingZero,
  Ready,
  DeactivatingZero,
  Stopped,
  FaultClosed,
};

enum class BridgeStopCause {
  None,
  Planned,
  InvalidCommand,
  RejectedCommand,
  QueuedCommandExpired,
  RequestedSafety,
  CommandTimeout,
  HeartbeatTimeout,
  WriterMissing,
  WriterAmbiguous,
};

[[nodiscard]] const char *bridgeStopCauseName(BridgeStopCause cause) noexcept;

struct BridgeConfig {
  std::string bridge_boot_id;
  std::string expected_product_session_id;
  lingtu::driver::Limits limits;
  std::chrono::milliseconds heartbeat_timeout{0};
  std::chrono::milliseconds apply_timeout{0};
};

struct NavCommand {
  std::string host_boot_id;
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
  TimePoint source_time{};
  TimePoint arrival_time{};
  std::string frame;
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct BridgeStatus {
  BridgeLifecycle lifecycle{BridgeLifecycle::AwaitWriter};
  bool ready{false};
  bool lease_valid{false};
  bool has_pending{false};
  bool has_latest{false};
  std::uint64_t accepted_sequence{0};
  std::uint32_t lease_remaining_ms{0};
  BridgeWalk applied_walk;
  lingtu::driver::OutputAckEvidence output_ack;
  BridgeStopCause stop_cause{BridgeStopCause::None};
  BridgeFaultCode fault{BridgeFaultCode::None};
  std::string controller_boot_id;
};

class MujocoDriverBridgeCore final {
 public:
  explicit MujocoDriverBridgeCore(BridgeConfig config);

  [[nodiscard]] std::optional<BridgeCommand> onWriterCount(std::uint32_t matched_writers,
                                                           TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> onActivate(const ActivateMessage &message,
                                                        TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> onApplied(const AppliedMessage &message,
                                                       TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> onHeartbeat(const HeartbeatMessage &message,
                                                         TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> onDeactivate(const DeactivateMessage &message,
                                                          TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> submitNav(const NavCommand &command);
  [[nodiscard]] std::optional<BridgeCommand> requestSafetyStop(TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> poll(TimePoint now);

  void controllerEof() noexcept;
  void protocolFault() noexcept;

  [[nodiscard]] BridgeStatus status(TimePoint now) const;
  [[nodiscard]] const std::optional<StopMessage> &stoppedEvidence() const noexcept;

 private:
  struct QueuedNav {
    std::string producer_boot_id;
    std::uint64_t output_sequence{0};
    BridgeWalk walk;
    TimePoint source_time{};
    TimePoint queued_at{};
  };

  [[nodiscard]] bool terminal() const noexcept;
  [[nodiscard]] bool controllerIdentityMatches(std::string_view bridge_boot_id,
                                               std::string_view controller_boot_id) const noexcept;
  [[nodiscard]] bool heartbeatFresh(TimePoint now) const noexcept;
  [[nodiscard]] bool heartbeatExpired(TimePoint now) const noexcept;
  [[nodiscard]] std::uint32_t leaseRemainingMs(TimePoint now) const noexcept;
  [[nodiscard]] bool checkDeadlines(TimePoint now) noexcept;
  [[nodiscard]] bool appliedMatches(const AppliedMessage &message,
                                    const BridgeCommand &command) const noexcept;
  [[nodiscard]] std::optional<BridgeCommand> issue(BridgeCommandKind kind,
                                                   std::string producer_boot_id,
                                                   std::uint64_t output_sequence, BridgeWalk walk,
                                                   TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> issueQueuedNav(QueuedNav command, TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> requestStop(BridgeStopCause cause, TimePoint now);
  [[nodiscard]] std::optional<BridgeCommand> issueStopZero(TimePoint now);
  void completeStop(const AppliedMessage &message, TimePoint now) noexcept;
  void fail(BridgeFaultCode fault) noexcept;

  BridgeConfig config_;
  lingtu::driver::Core motion_core_;
  lingtu::driver::CmdVelWriterGate writer_gate_;
  lingtu::driver::OutputAckState output_ack_;
  BridgeLifecycle lifecycle_{BridgeLifecycle::AwaitWriter};
  BridgeFaultCode fault_{BridgeFaultCode::None};
  BridgeStopCause stop_cause_{BridgeStopCause::None};
  std::string controller_boot_id_;
  std::uint64_t last_control_seq_{0};
  std::uint64_t last_heartbeat_step_seq_{0};
  std::uint64_t last_applied_step_seq_{0};
  std::uint64_t activation_applied_step_seq_{0};
  std::uint64_t last_bridge_command_seq_{0};
  std::uint64_t accepted_sequence_{0};
  BridgeWalk last_applied_walk_;
  TimePoint last_controller_arrival_{};
  bool has_controller_arrival_{false};
  TimePoint pending_issued_at_{};
  std::optional<BridgeCommand> pending_;
  std::optional<QueuedNav> latest_;
  std::optional<StopMessage> stopped_evidence_;
};

}  // namespace lingtu::sim::driver_bridge
