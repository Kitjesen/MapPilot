#include "mujoco_driver_bridge_core.hpp"

#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace lingtu::sim::driver_bridge {
namespace {

bool validConfigDuration(std::chrono::milliseconds value) noexcept {
  return value.count() > 0;
}

bool sameDouble(double left, double right) noexcept {
  if (left != right) {
    return false;
  }
  return left != 0.0 || std::signbit(left) == std::signbit(right);
}

BridgeWalk toBridgeWalk(const lingtu::driver::Velocity &velocity) noexcept {
  return {
      velocity.vx_mps == 0.0 ? 0.0 : velocity.vx_mps,
      velocity.vy_mps == 0.0 ? 0.0 : velocity.vy_mps,
      velocity.yaw_rps == 0.0 ? 0.0 : velocity.yaw_rps,
  };
}

bool isStopZero(BridgeCommandKind kind) noexcept {
  return kind == BridgeCommandKind::DeactivateZero || kind == BridgeCommandKind::WriterFaultZero ||
         kind == BridgeCommandKind::SafetyZero;
}

bool recoverableSafetyStop(BridgeStopCause cause) noexcept {
  return cause == BridgeStopCause::InvalidCommand ||
         cause == BridgeStopCause::RejectedCommand ||
         cause == BridgeStopCause::QueuedCommandExpired ||
         cause == BridgeStopCause::RequestedSafety ||
         cause == BridgeStopCause::CommandTimeout;
}

}  // namespace

const char *bridgeStopCauseName(BridgeStopCause cause) noexcept {
  switch (cause) {
    case BridgeStopCause::None:
      return "none";
    case BridgeStopCause::Planned:
      return "planned_shutdown";
    case BridgeStopCause::InvalidCommand:
      return "invalid_command";
    case BridgeStopCause::RejectedCommand:
      return "rejected_command";
    case BridgeStopCause::QueuedCommandExpired:
      return "queued_command_expired";
    case BridgeStopCause::RequestedSafety:
      return "requested_safety_stop";
    case BridgeStopCause::CommandTimeout:
      return "command_timeout";
    case BridgeStopCause::HeartbeatTimeout:
      return "heartbeat_timeout";
    case BridgeStopCause::WriterMissing:
      return "writer_missing";
    case BridgeStopCause::WriterAmbiguous:
      return "writer_ambiguous";
  }
  return "unknown";
}

MujocoDriverBridgeCore::MujocoDriverBridgeCore(BridgeConfig config)
    : config_(std::move(config)),
      motion_core_(config_.limits, config_.expected_product_session_id) {
  if (!validBootId(config_.bridge_boot_id)) {
    throw std::invalid_argument("bridge_boot_id must be lowercase 32 hexadecimal characters");
  }
  if (!validProducerToken(config_.expected_product_session_id)) {
    throw std::invalid_argument("expected_product_session_id is not a safe token");
  }
  if (!validConfigDuration(config_.heartbeat_timeout)) {
    throw std::invalid_argument("heartbeat_timeout must be positive");
  }
  if (!validConfigDuration(config_.apply_timeout)) {
    throw std::invalid_argument("apply_timeout must be positive");
  }
}

bool MujocoDriverBridgeCore::terminal() const noexcept {
  return lifecycle_ == BridgeLifecycle::Stopped || lifecycle_ == BridgeLifecycle::FaultClosed;
}

bool MujocoDriverBridgeCore::controllerIdentityMatches(
    std::string_view bridge_boot_id, std::string_view controller_boot_id) const noexcept {
  return bridge_boot_id == config_.bridge_boot_id && !controller_boot_id_.empty() &&
         controller_boot_id == controller_boot_id_;
}

bool MujocoDriverBridgeCore::heartbeatFresh(TimePoint now) const noexcept {
  return has_controller_arrival_ && now >= last_controller_arrival_ &&
         now - last_controller_arrival_ < config_.heartbeat_timeout;
}

bool MujocoDriverBridgeCore::heartbeatExpired(TimePoint now) const noexcept {
  return has_controller_arrival_ && now >= last_controller_arrival_ &&
         now - last_controller_arrival_ >= config_.heartbeat_timeout;
}

std::uint32_t MujocoDriverBridgeCore::leaseRemainingMs(TimePoint now) const noexcept {
  if (!heartbeatFresh(now)) {
    return 0;
  }
  const auto elapsed =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_controller_arrival_);
  const auto remaining = config_.heartbeat_timeout - elapsed;
  if (remaining.count() <= 0) {
    return 0;
  }
  const auto maximum = static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max());
  return static_cast<std::uint32_t>(std::min<std::int64_t>(remaining.count(), maximum));
}

bool MujocoDriverBridgeCore::checkDeadlines(TimePoint now) noexcept {
  if (terminal()) {
    return false;
  }
  if ((pending_.has_value() && now < pending_issued_at_) ||
      (has_controller_arrival_ && now < last_controller_arrival_)) {
    fail(BridgeFaultCode::ProtocolViolation);
    return false;
  }
  if (pending_.has_value() && now >= pending_issued_at_ + config_.apply_timeout) {
    fail(BridgeFaultCode::ApplyTimeout);
    return false;
  }
  return true;
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::onWriterCount(std::uint32_t matched_writers,
                                                                   TimePoint now) {
  if (terminal()) {
    return std::nullopt;
  }
  if (!controller_boot_id_.empty() && !checkDeadlines(now)) {
    return std::nullopt;
  }
  const auto decision = writer_gate_.update(matched_writers);
  if (lifecycle_ == BridgeLifecycle::AwaitWriter) {
    if (decision.ready) {
      lifecycle_ = BridgeLifecycle::AwaitController;
    } else if (matched_writers > 1) {
      fail(BridgeFaultCode::WriterAmbiguous);
    }
    return std::nullopt;
  }
  if (decision.ready) {
    if (heartbeatExpired(now) && stop_cause_ == BridgeStopCause::None) {
      return requestStop(BridgeStopCause::HeartbeatTimeout, now);
    }
    return std::nullopt;
  }
  const auto cause = matched_writers == 0 ? BridgeStopCause::WriterMissing
                                          : BridgeStopCause::WriterAmbiguous;
  if (controller_boot_id_.empty()) {
    fail(matched_writers == 0 ? BridgeFaultCode::WriterMissing : BridgeFaultCode::WriterAmbiguous);
    return std::nullopt;
  }
  return requestStop(cause, now);
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::onActivate(const ActivateMessage &message,
                                                                TimePoint now) {
  if (terminal()) {
    return std::nullopt;
  }
  if (lifecycle_ != BridgeLifecycle::AwaitController || !writer_gate_.ready() ||
      message.bridge_boot_id != config_.bridge_boot_id ||
      !validBootId(message.controller_boot_id) || message.control_seq == 0 ||
      message.control_seq <= last_control_seq_) {
    fail(BridgeFaultCode::ProtocolViolation);
    return std::nullopt;
  }
  controller_boot_id_ = message.controller_boot_id;
  last_control_seq_ = message.control_seq;
  last_controller_arrival_ = now;
  has_controller_arrival_ = true;
  lifecycle_ = BridgeLifecycle::ActivatingZero;
  const auto action = motion_core_.forceStop(lingtu::driver::ActionReason::ConnectProbe);
  return issue(BridgeCommandKind::ActivationZero, {}, 0, toBridgeWalk(action.velocity), now);
}

bool MujocoDriverBridgeCore::appliedMatches(const AppliedMessage &message,
                                            const BridgeCommand &command) const noexcept {
  return message.bridge_boot_id == command.bridge_boot_id &&
         message.controller_boot_id == command.controller_boot_id &&
         message.bridge_command_seq == command.bridge_command_seq && message.kind == command.kind &&
         message.producer_boot_id == command.producer_boot_id &&
         message.output_sequence == command.output_sequence &&
         sameDouble(message.walk_x, command.walk.x) && sameDouble(message.walk_y, command.walk.y) &&
         sameDouble(message.walk_z, command.walk.z) && message.applied_step_seq > 0 &&
         message.applied_step_seq > last_applied_step_seq_ &&
         message.applied_step_seq > last_heartbeat_step_seq_;
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::onApplied(const AppliedMessage &message,
                                                               TimePoint now) {
  if (lifecycle_ == BridgeLifecycle::FaultClosed) {
    return std::nullopt;
  }
  if (lifecycle_ == BridgeLifecycle::Stopped) {
    fail(BridgeFaultCode::ProtocolViolation);
    return std::nullopt;
  }
  if (!checkDeadlines(now)) {
    return std::nullopt;
  }
  if (heartbeatExpired(now) && stop_cause_ == BridgeStopCause::None) {
    (void)requestStop(BridgeStopCause::HeartbeatTimeout, now);
  }
  if (!pending_.has_value() ||
      !controllerIdentityMatches(message.bridge_boot_id, message.controller_boot_id) ||
      !appliedMatches(message, *pending_)) {
    fail(BridgeFaultCode::ProtocolViolation);
    return std::nullopt;
  }

  const BridgeCommand applied = *pending_;
  pending_.reset();
  accepted_sequence_ = applied.bridge_command_seq;
  last_applied_walk_ = applied.walk;
  last_applied_step_seq_ = message.applied_step_seq;

  if (applied.kind == BridgeCommandKind::Nav && stop_cause_ == BridgeStopCause::None) {
    output_ack_.record(applied.producer_boot_id, applied.output_sequence, true, now);
  } else {
    output_ack_.invalidate();
  }

  if (stop_cause_ != BridgeStopCause::None) {
    if (stop_cause_ == BridgeStopCause::Planned &&
        applied.kind != BridgeCommandKind::DeactivateZero) {
      return issueStopZero(now);
    }
    if (isStopZero(applied.kind)) {
      completeStop(message, now);
      return std::nullopt;
    }
    return issueStopZero(now);
  }

  if (applied.kind == BridgeCommandKind::ActivationZero) {
    activation_applied_step_seq_ = message.applied_step_seq;
    lifecycle_ = BridgeLifecycle::ActivatingZero;
    return std::nullopt;
  }
  if (applied.kind != BridgeCommandKind::Nav) {
    fail(BridgeFaultCode::ProtocolViolation);
    return std::nullopt;
  }

  if (latest_.has_value()) {
    QueuedNav latest = std::move(*latest_);
    latest_.reset();
    if (now < latest.source_time || now - latest.source_time >= config_.limits.command_timeout ||
        now < latest.queued_at || now - latest.queued_at >= config_.limits.command_timeout) {
      return requestStop(BridgeStopCause::QueuedCommandExpired, now);
    }
    return issueQueuedNav(std::move(latest), now);
  }
  return std::nullopt;
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::onHeartbeat(const HeartbeatMessage &message,
                                                                 TimePoint now) {
  if (terminal()) {
    return std::nullopt;
  }
  if (!checkDeadlines(now)) {
    return std::nullopt;
  }
  if (heartbeatExpired(now) && stop_cause_ == BridgeStopCause::None) {
    return requestStop(BridgeStopCause::HeartbeatTimeout, now);
  }
  if (!controllerIdentityMatches(message.bridge_boot_id, message.controller_boot_id) ||
      message.control_seq == 0 || message.control_seq <= last_control_seq_ ||
      message.step_seq == 0 || message.step_seq < last_heartbeat_step_seq_) {
    fail(BridgeFaultCode::ProtocolViolation);
    return std::nullopt;
  }
  last_control_seq_ = message.control_seq;
  last_heartbeat_step_seq_ = message.step_seq;
  last_controller_arrival_ = now;
  has_controller_arrival_ = true;
  if (lifecycle_ == BridgeLifecycle::ActivatingZero && !pending_.has_value() &&
      activation_applied_step_seq_ > 0 && message.step_seq >= activation_applied_step_seq_) {
    lifecycle_ = BridgeLifecycle::Ready;
  }
  return std::nullopt;
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::onDeactivate(const DeactivateMessage &message,
                                                                  TimePoint now) {
  if (terminal()) {
    return std::nullopt;
  }
  if (!checkDeadlines(now)) {
    return std::nullopt;
  }
  if (heartbeatExpired(now) && stop_cause_ == BridgeStopCause::None) {
    return requestStop(BridgeStopCause::HeartbeatTimeout, now);
  }
  if (!controllerIdentityMatches(message.bridge_boot_id, message.controller_boot_id) ||
      message.control_seq == 0 || message.control_seq <= last_control_seq_) {
    fail(BridgeFaultCode::ProtocolViolation);
    return std::nullopt;
  }
  last_control_seq_ = message.control_seq;
  return requestStop(BridgeStopCause::Planned, now);
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::submitNav(const NavCommand &command) {
  if (terminal() || lifecycle_ != BridgeLifecycle::Ready ||
      stop_cause_ != BridgeStopCause::None) {
    output_ack_.invalidate();
    return std::nullopt;
  }
  if (!checkDeadlines(command.arrival_time)) {
    return std::nullopt;
  }
  if (heartbeatExpired(command.arrival_time)) {
    return requestStop(BridgeStopCause::HeartbeatTimeout, command.arrival_time);
  }
  const bool producer_valid = validProducerToken(command.producer_boot_id);
  const bool sequence_valid = command.output_sequence != 0;
  const bool source_valid = command.source_time > TimePoint{};
  const bool arrival_valid = command.arrival_time > TimePoint{};
  const bool source_future = arrival_valid && source_valid &&
                             command.arrival_time < command.source_time;
  if (!producer_valid || !sequence_valid || !source_valid || !arrival_valid || source_future) {
    const auto source_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               command.source_time.time_since_epoch())
                               .count();
    const auto arrival_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                command.arrival_time.time_since_epoch())
                                .count();
    std::fprintf(stderr,
                 "mujoco_driver_bridge: invalid command producer=%d sequence=%llu "
                 "source_ns=%lld arrival_ns=%lld source_minus_arrival_ns=%lld\n",
                 producer_valid ? 1 : 0,
                 static_cast<unsigned long long>(command.output_sequence),
                 static_cast<long long>(source_ns),
                 static_cast<long long>(arrival_ns),
                 static_cast<long long>(source_ns - arrival_ns));
    return requestStop(BridgeStopCause::InvalidCommand, command.arrival_time);
  }
  const auto result = motion_core_.accept(
      lingtu::driver::CommandFreshnessInput{
          command.host_boot_id,
          command.producer_boot_id,
          command.output_sequence,
          command.source_time.time_since_epoch(),
          command.arrival_time.time_since_epoch(),
      },
      command.frame, command.vx, command.vy, command.wz);
  if (!result.freshness.accepted || !result.action.has_value() ||
      result.action->reason != lingtu::driver::ActionReason::Command) {
    return requestStop(BridgeStopCause::RejectedCommand, command.arrival_time);
  }

  QueuedNav queued{
      command.producer_boot_id, command.output_sequence, toBridgeWalk(result.action->velocity),
      command.source_time,      command.arrival_time,
  };
  if (pending_.has_value()) {
    latest_ = std::move(queued);
    return std::nullopt;
  }
  return issueQueuedNav(std::move(queued), command.arrival_time);
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::requestSafetyStop(TimePoint now) {
  if (terminal() || controller_boot_id_.empty()) {
    return std::nullopt;
  }
  if (!checkDeadlines(now)) {
    return std::nullopt;
  }
  if (heartbeatExpired(now) && stop_cause_ == BridgeStopCause::None) {
    return requestStop(BridgeStopCause::HeartbeatTimeout, now);
  }
  return requestStop(BridgeStopCause::RequestedSafety, now);
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::poll(TimePoint now) {
  if (terminal()) {
    return std::nullopt;
  }
  if (!checkDeadlines(now)) {
    return std::nullopt;
  }
  if (heartbeatExpired(now) && stop_cause_ == BridgeStopCause::None) {
    return requestStop(BridgeStopCause::HeartbeatTimeout, now);
  }
  if (lifecycle_ == BridgeLifecycle::Ready && stop_cause_ == BridgeStopCause::None &&
      motion_core_.poll(now).has_value()) {
    return requestStop(BridgeStopCause::CommandTimeout, now);
  }
  return std::nullopt;
}

void MujocoDriverBridgeCore::controllerEof() noexcept {
  if (!terminal()) {
    fail(BridgeFaultCode::ControllerEof);
  }
}

void MujocoDriverBridgeCore::protocolFault() noexcept {
  if (!terminal()) {
    fail(BridgeFaultCode::ProtocolViolation);
  }
}

BridgeStatus MujocoDriverBridgeCore::status(TimePoint now) const {
  const bool ready = lifecycle_ == BridgeLifecycle::Ready &&
                     stop_cause_ == BridgeStopCause::None &&
                     writer_gate_.ready() && heartbeatFresh(now);
  const bool lease_valid = !terminal() && writer_gate_.ready() && heartbeatFresh(now);
  return {
      lifecycle_,
      ready,
      lease_valid,
      pending_.has_value(),
      latest_.has_value(),
      accepted_sequence_,
      leaseRemainingMs(now),
      last_applied_walk_,
      output_ack_.current(now),
      stop_cause_,
      fault_,
      controller_boot_id_,
  };
}

const std::optional<StopMessage> &MujocoDriverBridgeCore::stoppedEvidence() const noexcept {
  return stopped_evidence_;
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::issue(BridgeCommandKind kind,
                                                           std::string producer_boot_id,
                                                           std::uint64_t output_sequence,
                                                           BridgeWalk walk, TimePoint now) {
  if (pending_.has_value()) {
    fail(BridgeFaultCode::ProtocolViolation);
    return std::nullopt;
  }
  if (last_bridge_command_seq_ == std::numeric_limits<std::uint64_t>::max()) {
    fail(BridgeFaultCode::CommandSequenceOverflow);
    return std::nullopt;
  }
  ++last_bridge_command_seq_;
  pending_ = BridgeCommand{
      config_.bridge_boot_id,
      controller_boot_id_,
      last_bridge_command_seq_,
      kind,
      std::move(producer_boot_id),
      output_sequence,
      walk,
  };
  pending_issued_at_ = now;
  return pending_;
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::issueQueuedNav(QueuedNav command,
                                                                     TimePoint now) {
  return issue(BridgeCommandKind::Nav, std::move(command.producer_boot_id), command.output_sequence,
               command.walk, now);
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::requestStop(BridgeStopCause cause,
                                                                 TimePoint now) {
  if (terminal()) {
    return std::nullopt;
  }
  if (stop_cause_ == BridgeStopCause::None || cause == BridgeStopCause::WriterMissing ||
      cause == BridgeStopCause::WriterAmbiguous ||
      (cause == BridgeStopCause::Planned && recoverableSafetyStop(stop_cause_))) {
    stop_cause_ = cause;
  }
  output_ack_.invalidate();
  latest_.reset();
  lifecycle_ = BridgeLifecycle::DeactivatingZero;
  (void)motion_core_.forceStop(cause == BridgeStopCause::Planned
                                  ? lingtu::driver::ActionReason::Shutdown
                                  : lingtu::driver::ActionReason::Fault);
  if (pending_.has_value()) {
    return std::nullopt;
  }
  return issueStopZero(now);
}

std::optional<BridgeCommand> MujocoDriverBridgeCore::issueStopZero(TimePoint now) {
  BridgeCommandKind kind = BridgeCommandKind::SafetyZero;
  if (stop_cause_ == BridgeStopCause::Planned) {
    kind = BridgeCommandKind::DeactivateZero;
  } else if (stop_cause_ == BridgeStopCause::WriterMissing ||
             stop_cause_ == BridgeStopCause::WriterAmbiguous) {
    kind = BridgeCommandKind::WriterFaultZero;
  }
  return issue(kind, {}, 0, {}, now);
}

void MujocoDriverBridgeCore::completeStop(const AppliedMessage &message, TimePoint now) noexcept {
  const auto cause = stop_cause_;
  stop_cause_ = BridgeStopCause::None;
  latest_.reset();
  output_ack_.invalidate();
  if (cause == BridgeStopCause::Planned) {
    lifecycle_ = BridgeLifecycle::Stopped;
    stopped_evidence_ = StopMessage{
        config_.bridge_boot_id,   controller_boot_id_, accepted_sequence_,
        message.applied_step_seq, message.kind,
    };
    return;
  }
  if (cause == BridgeStopCause::WriterMissing) {
    fail(BridgeFaultCode::WriterMissing);
    return;
  }
  if (cause == BridgeStopCause::WriterAmbiguous) {
    fail(BridgeFaultCode::WriterAmbiguous);
    return;
  }
  if (cause == BridgeStopCause::HeartbeatTimeout) {
    fail(BridgeFaultCode::HeartbeatTimeout);
    return;
  }
  if (recoverableSafetyStop(cause) && writer_gate_.ready() && heartbeatFresh(now)) {
    lifecycle_ = BridgeLifecycle::Ready;
    return;
  }
  fail(BridgeFaultCode::HeartbeatTimeout);
}

void MujocoDriverBridgeCore::fail(BridgeFaultCode fault) noexcept {
  lifecycle_ = BridgeLifecycle::FaultClosed;
  fault_ = fault;
  stop_cause_ = BridgeStopCause::None;
  pending_.reset();
  latest_.reset();
  output_ack_.invalidate();
  stopped_evidence_.reset();
  motion_core_.reset();
}

}  // namespace lingtu::sim::driver_bridge
