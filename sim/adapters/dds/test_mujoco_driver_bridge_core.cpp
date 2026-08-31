#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "mujoco_driver_bridge_core.hpp"
#include "mujoco_driver_bridge_protocol.hpp"

namespace {

using namespace std::chrono_literals;
using lingtu::sim::driver_bridge::ActivateMessage;
using lingtu::sim::driver_bridge::AppliedMessage;
using lingtu::sim::driver_bridge::BridgeCommand;
using lingtu::sim::driver_bridge::BridgeCommandKind;
using lingtu::sim::driver_bridge::BridgeConfig;
using lingtu::sim::driver_bridge::BridgeFaultCode;
using lingtu::sim::driver_bridge::BridgeLifecycle;
using lingtu::sim::driver_bridge::BridgeStopCause;
using lingtu::sim::driver_bridge::DeactivateMessage;
using lingtu::sim::driver_bridge::FaultMessage;
using lingtu::sim::driver_bridge::HeartbeatMessage;
using lingtu::sim::driver_bridge::MujocoDriverBridgeCore;
using lingtu::sim::driver_bridge::NavCommand;
using lingtu::sim::driver_bridge::ProtocolError;
using lingtu::sim::driver_bridge::ReadyMessage;
using lingtu::sim::driver_bridge::StopMessage;
using lingtu::sim::driver_bridge::TimePoint;

constexpr char kBridgeBoot[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
constexpr char kControllerBoot[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
constexpr char kWrongBoot[] = "cccccccccccccccccccccccccccccccc";
constexpr char kProductSessionId[] = "host-boot-a";

void check(bool value, const char *message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

template <typename Function>
void expectThrows(Function &&function, const char *message) {
  bool threw = false;
  try {
    function();
  } catch (const std::exception &) {
    threw = true;
  }
  check(threw, message);
}

BridgeConfig config() {
  BridgeConfig value;
  value.bridge_boot_id = kBridgeBoot;
  value.expected_product_session_id = kProductSessionId;
  value.limits.max_linear_mps = 1.0;
  value.limits.max_angular_rps = 2.0;
  value.limits.command_timeout = 100ms;
  value.heartbeat_timeout = 500ms;
  value.apply_timeout = 500ms;
  return value;
}

AppliedMessage appliedFor(const BridgeCommand &command, std::uint64_t step) {
  return {
      command.bridge_boot_id,
      command.controller_boot_id,
      command.bridge_command_seq,
      command.kind,
      command.producer_boot_id,
      command.output_sequence,
      command.walk.x,
      command.walk.y,
      command.walk.z,
      step,
  };
}

NavCommand nav(TimePoint arrival, std::uint64_t output_sequence,
               std::string producer = "producer-a") {
  NavCommand command;
  command.host_boot_id = kProductSessionId;
  command.producer_boot_id = std::move(producer);
  command.output_sequence = output_sequence;
  command.source_time = arrival - 1ms;
  command.arrival_time = arrival;
  command.frame = "body";
  command.vx = 2.0;
  command.vy = -0.5;
  command.wz = 4.0;
  return command;
}

void activateToReady(MujocoDriverBridgeCore &core, TimePoint start) {
  check(!core.onWriterCount(1, start).has_value(), "writer discovery must not emit a command");
  check(core.status(start).lifecycle == BridgeLifecycle::AwaitController,
        "one writer must advance to AwaitController");

  const auto activation =
      core.onActivate(ActivateMessage{kBridgeBoot, kControllerBoot, 1}, start + 1ms);
  check(activation.has_value(), "ACTIVATE must emit activation zero");
  check(activation->kind == BridgeCommandKind::ActivationZero, "ACTIVATE must emit ActivationZero");
  check(!core.status(start + 1ms).ready, "activation zero must not be ready");

  const auto after_applied = core.onApplied(appliedFor(*activation, 1), start + 2ms);
  check(!after_applied.has_value(), "activation APPLIED must not emit motion");
  check(!core.status(start + 2ms).ready,
        "activation APPLIED without later heartbeat must remain not ready");

  check(!core.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 2, 2}, start + 3ms)
             .has_value(),
        "HEARTBEAT must not emit a command");
  check(core.status(start + 3ms).ready, "heartbeat after applied step must make bridge ready");
}

void testConstructionIsStrictAndSideEffectFree() {
  const auto start = TimePoint{} + 1s;
  MujocoDriverBridgeCore core(config());
  const auto status = core.status(start);
  check(status.lifecycle == BridgeLifecycle::AwaitWriter, "constructor must start in AwaitWriter");
  check(!status.ready, "constructor must not claim ready");
  check(!status.has_pending, "constructor must not emit pending IO");

  auto invalid = config();
  invalid.bridge_boot_id = "ABC";
  expectThrows([&]() { MujocoDriverBridgeCore ignored(invalid); },
               "invalid bridge boot id must be rejected");
  invalid = config();
  invalid.expected_product_session_id.clear();
  expectThrows([&]() { MujocoDriverBridgeCore ignored(invalid); },
               "empty expected host boot id must be rejected");
  invalid = config();
  invalid.heartbeat_timeout = 0ms;
  expectThrows([&]() { MujocoDriverBridgeCore ignored(invalid); },
               "non-positive heartbeat timeout must be rejected");
  invalid = config();
  invalid.apply_timeout = -1ms;
  expectThrows([&]() { MujocoDriverBridgeCore ignored(invalid); },
               "non-positive apply timeout must be rejected");
}

void testActivationRequiresAppliedThenLaterHeartbeat() {
  const auto start = TimePoint{} + 2s;
  MujocoDriverBridgeCore core(config());
  (void)core.onWriterCount(1, start);
  const auto activation =
      core.onActivate(ActivateMessage{kBridgeBoot, kControllerBoot, 1}, start + 1ms);
  check(activation.has_value(), "ACTIVATE must emit a zero");

  (void)core.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 2, 1}, start + 2ms);
  check(!core.status(start + 2ms).ready, "heartbeat before APPLIED must not make bridge ready");
  (void)core.onApplied(appliedFor(*activation, 2), start + 3ms);
  check(!core.status(start + 3ms).ready, "matching APPLIED alone must not make bridge ready");
  (void)core.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 3, 3}, start + 4ms);
  auto ready = core.status(start + 4ms);
  check(ready.ready, "strictly later physics heartbeat must make bridge ready");
  check(ready.lease_remaining_ms == 500,
        "fresh heartbeat must expose the exact full controller lease");
  check(core.status(start + 503ms).lease_remaining_ms == 1,
        "fresh lease must retain a positive final millisecond");
  check(core.status(start + 504ms).lease_remaining_ms == 0,
        "expired heartbeat must expose a zero remaining lease");
}

void testPhysicalNavAppliedPublishesExactOutputAck() {
  const auto start = TimePoint{} + 3s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);

  const auto emitted = core.submitNav(nav(start + 4ms, 91));
  check(emitted.has_value(), "ready bridge must emit accepted Nav command");
  check(emitted->kind == BridgeCommandKind::Nav, "command kind must be Nav");
  check(emitted->walk.x == 1.0, "linear x must use real Core normalization");
  check(emitted->walk.y == -0.5, "linear y must use real Core normalization");
  check(emitted->walk.z == 2.0, "angular z must use the configured real Core limit");
  auto status = core.status(start + 4ms);
  check(status.output_ack.producerBootId().empty(),
        "Nav command must not ACK before physical APPLIED");
  check(status.applied_walk.x == 0.0 && status.applied_walk.y == 0.0 &&
            status.applied_walk.z == 0.0,
        "accepted DDS input must not masquerade as a physically applied walk");

  (void)core.onApplied(appliedFor(*emitted, 3), start + 5ms);
  status = core.status(start + 5ms);
  check(status.output_ack.accepted(), "matching Nav APPLIED must ACK");
  check(status.output_ack.producerBootId() == "producer-a", "ACK must preserve producer identity");
  check(status.output_ack.outputSequence() == 91, "ACK must preserve output identity");
  check(status.accepted_sequence == emitted->bridge_command_seq,
        "accepted_sequence must be physical bridge command sequence");
  check(status.accepted_sequence != status.output_ack.outputSequence(),
        "bridge accepted_sequence and Nav output sequence must remain distinct");
  check(status.applied_walk.x == 1.0 && status.applied_walk.y == -0.5 &&
            status.applied_walk.z == 2.0,
        "BridgeStatus must expose only the physically APPLIED walk");
}

void testNavZeroWalkIsCanonicalAcrossWireAndAppliedAck() {
  const auto start = TimePoint{} + 3250ms;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);

  auto zero_nav = nav(start + 4ms, 911);
  zero_nav.vx = 0.0;
  zero_nav.vy = -0.0;
  zero_nav.wz = -0.0;
  const auto emitted = core.submitNav(zero_nav);
  check(emitted.has_value(), "ready bridge must emit an accepted zero Nav command");
  check(emitted->walk.x == 0.0 && !std::signbit(emitted->walk.x) &&
            emitted->walk.y == 0.0 && !std::signbit(emitted->walk.y) &&
            emitted->walk.z == 0.0 && !std::signbit(emitted->walk.z),
        "bridge command must canonicalize every zero walk axis to positive zero");
  const auto wire = lingtu::sim::driver_bridge::serializeCommand(*emitted);
  check(wire.find("\t-0") == std::string::npos,
        "serialized bridge command must not expose negative zero");

  (void)core.onApplied(appliedFor(*emitted, 3), start + 5ms);
  const auto status = core.status(start + 5ms);
  check(status.output_ack.accepted(), "matching canonical-zero APPLIED must ACK");
  check(status.output_ack.outputSequence() == 911,
        "canonical-zero APPLIED ACK must preserve output identity");
}

void testPendingNavPreservesLastPhysicalAckUntilReplacementIsApplied() {
  const auto start = TimePoint{} + 3500ms;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);

  const auto first = core.submitNav(nav(start + 4ms, 92));
  check(first.has_value(), "first ACK fixture command must emit");
  (void)core.onApplied(appliedFor(*first, 3), start + 5ms);
  check(core.status(start + 5ms).output_ack.outputSequence() == 92,
        "first command must establish physical ACK evidence");

  const auto second = core.submitNav(nav(start + 6ms, 93));
  check(second.has_value(), "second ACK fixture command must emit");
  const auto pending = core.status(start + 6ms);
  check(pending.output_ack.accepted(),
        "pending Nav must retain the last physically applied ACK evidence");
  check(pending.output_ack.outputSequence() == 92,
        "pending Nav must not masquerade as physically applied before APPLIED arrives");

  (void)core.onApplied(appliedFor(*second, 4), start + 7ms);
  const auto applied = core.status(start + 7ms);
  check(applied.output_ack.accepted(), "second physical APPLIED must restore accepted evidence");
  check(applied.output_ack.outputSequence() == 93,
        "restored ACK must identify the newly applied output");
}

void testCoalescedNavPreservesAppliedAckWhileReleasingLatestCommand() {
  const auto start = TimePoint{} + 3750ms;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);

  const auto first = core.submitNav(nav(start + 4ms, 94));
  check(first.has_value(), "coalesced ACK fixture must emit the first command");
  check(!core.submitNav(nav(start + 5ms, 95)).has_value(),
        "newest Nav must remain coalesced while the first command is pending");

  const auto next = core.onApplied(appliedFor(*first, 3), start + 6ms);
  check(next.has_value(), "applying the first command must release the coalesced command");
  check(next->output_sequence == 95, "released command must preserve the newest output identity");
  const auto between = core.status(start + 6ms);
  check(between.has_pending, "released coalesced command must remain pending until APPLIED");
  check(between.output_ack.accepted(),
        "issuing the coalesced command must retain the first physical ACK");
  check(between.output_ack.outputSequence() == 94,
        "READY must identify the command that was physically applied, not the pending replacement");

  (void)core.onApplied(appliedFor(*next, 4), start + 7ms);
  const auto applied = core.status(start + 7ms);
  check(applied.output_ack.accepted(), "coalesced replacement APPLIED must remain accepted");
  check(applied.output_ack.outputSequence() == 95,
        "physical replacement must advance ACK to the newest output identity");
}

void testNavIngressUsesRealFreshnessAndNormalizationGates() {
  using Mutation = std::function<void(NavCommand &)>;
  const std::vector<Mutation> mutations{
      [](NavCommand &value) { value.host_boot_id = "wrong-host"; },
      [](NavCommand &value) { value.producer_boot_id = "bad producer"; },
      [](NavCommand &value) { value.output_sequence = 0; },
      [](NavCommand &value) { value.frame = "map"; },
      [](NavCommand &value) { value.vx = std::numeric_limits<double>::quiet_NaN(); },
      [](NavCommand &value) {
        value.source_time = value.arrival_time;
        value.arrival_time = value.source_time - 1ms;
      },
      [](NavCommand &value) {
        value.source_time = value.arrival_time - config().limits.command_timeout;
      },
      [](NavCommand &value) { value.source_time = TimePoint{}; },
  };

  std::uint64_t index = 0;
  for (const auto &mutate : mutations) {
    const auto start = TimePoint{} + 30s + std::chrono::seconds(index++);
    MujocoDriverBridgeCore core(config());
    activateToReady(core, start);
    auto invalid = nav(start + 4ms, 200 + index);
    mutate(invalid);
    const auto zero = core.submitNav(invalid);
    check(zero.has_value(), "invalid Nav input must issue a safety zero");
    check(zero->kind == BridgeCommandKind::SafetyZero,
          "invalid Nav input must fail closed through typed safety zero");
    check(!core.status(start + 4ms).ready, "invalid Nav input must clear readiness");
    const auto cause = core.status(start + 4ms).stop_cause;
    check(cause == BridgeStopCause::InvalidCommand ||
              cause == BridgeStopCause::RejectedCommand,
          "invalid Nav input must preserve whether validation or the real motion gate rejected it");
    check(core.status(start + 4ms).output_ack.producerBootId().empty(),
          "invalid Nav input must not publish output ACK");
  }
}

void testPendingCommandIsImmutableAndLatestIsCoalesced() {
  const auto start = TimePoint{} + 4s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto first = core.submitNav(nav(start + 4ms, 10, "producer-a"));
  check(first.has_value(), "first Nav must emit immediately");
  const auto second = core.submitNav(nav(start + 5ms, 11, "producer-a"));
  check(!second.has_value(), "second Nav must coalesce while pending");
  check(core.status(start + 5ms).has_latest, "latest slot must be observable");

  auto stale = appliedFor(*first, 3);
  stale.output_sequence = 11;
  const auto response = core.onApplied(stale, start + 6ms);
  check(!response.has_value(), "stale APPLIED must not emit another command");
  check(core.status(start + 6ms).lifecycle == BridgeLifecycle::FaultClosed,
        "stale APPLIED must not claim coalesced latest");
}

void testMatchingAppliedReleasesFreshLatest() {
  const auto start = TimePoint{} + 5s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto first = core.submitNav(nav(start + 4ms, 10));
  check(first.has_value(), "first Nav must emit");
  check(!core.submitNav(nav(start + 5ms, 11)).has_value(), "latest Nav must remain queued");

  const auto next = core.onApplied(appliedFor(*first, 3), start + 6ms);
  check(next.has_value(), "matching APPLIED must release latest Nav");
  check(next->kind == BridgeCommandKind::Nav, "released latest must be Nav");
  check(next->output_sequence == 11, "released latest identity must be exact");
  check(next->bridge_command_seq > first->bridge_command_seq,
        "bridge command sequence must increase strictly");
}

void testQueuedNavUsesOriginalSourceTimeWhenReleased() {
  const auto start = TimePoint{} + 5500ms;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto first = core.submitNav(nav(start + 4ms, 12));
  check(first.has_value(), "source-age fixture must emit first Nav");

  auto nearly_stale = nav(start + 5ms, 13);
  nearly_stale.source_time = nearly_stale.arrival_time - 99ms;
  check(!core.submitNav(nearly_stale).has_value(),
        "fresh-at-ingress second Nav must wait behind the pending command");

  const auto next = core.onApplied(appliedFor(*first, 3), start + 7ms);
  check(next.has_value(), "expired queued Nav must trigger a physical zero");
  check(next->kind == BridgeCommandKind::SafetyZero,
        "queued Nav whose original source time expired must never be released as motion");
  check(core.status(start + 7ms).stop_cause == BridgeStopCause::QueuedCommandExpired,
        "queued Nav expiry must retain its exact stop cause");
  check(core.status(start + 7ms).output_ack.producerBootId().empty(),
        "expired queued Nav transition must not retain the first command ACK");
}

void testEveryAppliedEchoFieldIsFailClosed() {
  using Mutation = std::function<void(AppliedMessage &)>;
  const std::vector<Mutation> mutations{
      [](AppliedMessage &value) { value.bridge_boot_id = kWrongBoot; },
      [](AppliedMessage &value) { value.controller_boot_id = kWrongBoot; },
      [](AppliedMessage &value) { ++value.bridge_command_seq; },
      [](AppliedMessage &value) { value.kind = BridgeCommandKind::SafetyZero; },
      [](AppliedMessage &value) { value.producer_boot_id = "producer-wrong"; },
      [](AppliedMessage &value) { ++value.output_sequence; },
      [](AppliedMessage &value) { value.walk_x += 0.01; },
      [](AppliedMessage &value) { value.walk_y -= 0.01; },
      [](AppliedMessage &value) { value.walk_z += 0.01; },
      [](AppliedMessage &value) { value.applied_step_seq = 0; },
  };

  std::uint64_t index = 0;
  for (const auto &mutate : mutations) {
    const auto start = TimePoint{} + 10s + std::chrono::milliseconds(index++);
    MujocoDriverBridgeCore core(config());
    activateToReady(core, start);
    const auto command = core.submitNav(nav(start + 4ms, 20 + index));
    check(command.has_value(), "Nav command fixture must emit");
    auto applied = appliedFor(*command, 3);
    mutate(applied);
    (void)core.onApplied(applied, start + 5ms);
    check(core.status(start + 5ms).lifecycle == BridgeLifecycle::FaultClosed,
          "any APPLIED echo mismatch must be terminal");
  }

  const auto start = TimePoint{} + 11s;
  MujocoDriverBridgeCore duplicate_core(config());
  activateToReady(duplicate_core, start);
  const auto command = duplicate_core.submitNav(nav(start + 4ms, 55));
  check(command.has_value(), "duplicate fixture must emit Nav");
  const auto applied = appliedFor(*command, 3);
  (void)duplicate_core.onApplied(applied, start + 5ms);
  (void)duplicate_core.onApplied(applied, start + 6ms);
  check(duplicate_core.status(start + 6ms).lifecycle == BridgeLifecycle::FaultClosed,
        "duplicate APPLIED must be terminal");
}

void testControlSequenceRejectsReplayAndHeartbeatStepMayRepeat() {
  const auto start = TimePoint{} + 12s;
  MujocoDriverBridgeCore control_replay(config());
  activateToReady(control_replay, start);
  (void)control_replay.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 2, 3},
                                   start + 4ms);
  check(control_replay.status(start + 4ms).lifecycle == BridgeLifecycle::FaultClosed,
        "control sequence replay must be terminal");

  MujocoDriverBridgeCore repeated_step(config());
  activateToReady(repeated_step, start + 1s);
  (void)repeated_step.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 3, 2},
                                  start + 1004ms);
  check(repeated_step.status(start + 1004ms).lifecycle == BridgeLifecycle::Ready,
        "fresh control heartbeat may repeat the latest completed physics step");

  MujocoDriverBridgeCore regressed_step(config());
  activateToReady(regressed_step, start + 2s);
  (void)regressed_step.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 3, 1},
                                   start + 2004ms);
  check(regressed_step.status(start + 2004ms).lifecycle == BridgeLifecycle::FaultClosed,
        "heartbeat physics step regression must remain terminal");
}

void testAppliedStepMustBeLaterThanObservedHeartbeat() {
  const auto start = TimePoint{} + 13s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto command = core.submitNav(nav(start + 4ms, 56));
  check(command.has_value(), "stale-physics-step fixture must emit Nav");

  (void)core.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 3, 4}, start + 5ms);
  (void)core.onApplied(appliedFor(*command, 3), start + 6ms);
  const auto status = core.status(start + 6ms);
  check(status.lifecycle == BridgeLifecycle::FaultClosed,
        "APPLIED older than an observed heartbeat must fail closed");
  check(status.fault == BridgeFaultCode::ProtocolViolation,
        "out-of-order physical APPLIED must be a protocol violation");
  check(status.output_ack.producerBootId().empty(),
        "out-of-order physical APPLIED must not publish output ACK");
}

void testApplyTimeoutIsTerminal() {
  const auto start = TimePoint{} + 14s;
  MujocoDriverBridgeCore apply_core(config());
  activateToReady(apply_core, start);
  const auto command = apply_core.submitNav(nav(start + 4ms, 61));
  check(command.has_value(), "apply-timeout fixture must emit Nav");
  (void)apply_core.poll(start + 4ms + config().apply_timeout);
  check(apply_core.status(start + 504ms).fault == BridgeFaultCode::ApplyTimeout,
        "apply timeout must be terminal and typed");
}

void testHeartbeatTimeoutRequiresAppliedPhysicalZeroBeforeFault() {
  const auto start = TimePoint{} + 14500ms;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);

  const auto nav_command = core.submitNav(nav(start + 4ms, 611));
  check(nav_command.has_value(), "heartbeat-timeout fixture must emit Nav");
  (void)core.onApplied(appliedFor(*nav_command, 3), start + 5ms);
  check(core.status(start + 5ms).output_ack.accepted(),
        "heartbeat-timeout fixture needs accepted physical output evidence");

  const auto zero = core.poll(start + 3ms + config().heartbeat_timeout);
  check(zero.has_value(), "heartbeat timeout must emit a physical zero before faulting");
  check(zero->kind == BridgeCommandKind::SafetyZero,
        "heartbeat timeout must use the typed safety zero command");
  const auto stopping = core.status(start + 503ms);
  check(stopping.lifecycle == BridgeLifecycle::DeactivatingZero,
        "bridge must await physical zero APPLIED after heartbeat timeout");
  check(!stopping.ready, "heartbeat timeout must clear readiness immediately");
  check(!stopping.lease_valid, "heartbeat timeout must invalidate the controller lease");
  check(stopping.output_ack.producerBootId().empty(),
        "heartbeat timeout must clear stale output ACK before issuing zero");

  (void)core.onApplied(appliedFor(*zero, 4), start + 504ms);
  const auto stopped = core.status(start + 504ms);
  check(stopped.lifecycle == BridgeLifecycle::FaultClosed,
        "physical zero APPLIED must complete heartbeat-timeout shutdown");
  check(stopped.accepted_sequence == zero->bridge_command_seq,
        "heartbeat-timeout fault must retain proof that the physical zero was applied");
  check(stopped.fault == BridgeFaultCode::HeartbeatTimeout,
        "heartbeat-timeout shutdown must preserve the typed root cause");
}

void testHeartbeatTimeoutDuringPendingNavSuppressesAckAndThenZeros() {
  const auto start = TimePoint{} + 14900ms;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto nav_command = core.submitNav(nav(start + 4ms, 612));
  check(nav_command.has_value(), "pending heartbeat-timeout fixture must emit Nav");

  const auto zero =
      core.onApplied(appliedFor(*nav_command, 3), start + 3ms + config().heartbeat_timeout);
  check(zero.has_value(),
        "expired heartbeat must release a physical zero after pending Nav APPLIED");
  check(zero->kind == BridgeCommandKind::SafetyZero,
        "expired heartbeat after pending Nav must emit typed safety zero");
  const auto stopping = core.status(start + 503ms);
  check(stopping.lifecycle == BridgeLifecycle::DeactivatingZero,
        "expired heartbeat must remain in the zero barrier");
  check(stopping.output_ack.producerBootId().empty(),
        "Nav APPLIED after heartbeat expiry must not publish output ACK");

  (void)core.onApplied(appliedFor(*zero, 4), start + 504ms);
  const auto stopped = core.status(start + 504ms);
  check(stopped.fault == BridgeFaultCode::HeartbeatTimeout,
        "zero APPLIED must complete the original heartbeat-timeout fault");
  check(stopped.accepted_sequence == zero->bridge_command_seq,
        "final status must retain the physically applied zero sequence");
}

void testEveryControlEntryUsesHeartbeatTimeoutZeroBarrier() {
  const auto start = TimePoint{} + 15500ms;
  const auto expired = start + 3ms + config().heartbeat_timeout;

  MujocoDriverBridgeCore writer_probe(config());
  activateToReady(writer_probe, start);
  const auto writer_zero = writer_probe.onWriterCount(1, expired);
  check(writer_zero.has_value() && writer_zero->kind == BridgeCommandKind::SafetyZero,
        "writer probe at heartbeat expiry must enter the zero barrier");

  MujocoDriverBridgeCore deactivate(config());
  activateToReady(deactivate, start + 1s);
  const auto deactivate_zero =
      deactivate.onDeactivate(DeactivateMessage{kBridgeBoot, kControllerBoot, 3}, expired + 1s);
  check(deactivate_zero.has_value() && deactivate_zero->kind == BridgeCommandKind::SafetyZero,
        "deactivate after lease expiry must preserve heartbeat-timeout zero semantics");

  MujocoDriverBridgeCore safety(config());
  activateToReady(safety, start + 2s);
  const auto safety_zero = safety.requestSafetyStop(expired + 2s);
  check(safety_zero.has_value() && safety_zero->kind == BridgeCommandKind::SafetyZero,
        "safety request at heartbeat expiry must preserve heartbeat-timeout zero semantics");
}

void testLateEventsCannotBypassDeadlines() {
  const auto start = TimePoint{} + 15s;
  MujocoDriverBridgeCore late_heartbeat(config());
  activateToReady(late_heartbeat, start);
  const auto heartbeat_zero =
      late_heartbeat.onHeartbeat(HeartbeatMessage{kBridgeBoot, kControllerBoot, 3, 3},
                                 start + 3ms + config().heartbeat_timeout);
  check(heartbeat_zero.has_value(),
        "late heartbeat must initiate physical zero instead of reviving the controller");
  check(heartbeat_zero->kind == BridgeCommandKind::SafetyZero,
        "late heartbeat must use the typed safety zero");
  check(late_heartbeat.status(start + 503ms).lifecycle == BridgeLifecycle::DeactivatingZero,
        "late heartbeat must wait for zero APPLIED");
  (void)late_heartbeat.onApplied(appliedFor(*heartbeat_zero, 3), start + 504ms);
  check(late_heartbeat.status(start + 504ms).fault == BridgeFaultCode::HeartbeatTimeout,
        "late heartbeat zero APPLIED must preserve the heartbeat-timeout root cause");

  MujocoDriverBridgeCore late_applied(config());
  activateToReady(late_applied, start + 1s);
  const auto command = late_applied.submitNav(nav(start + 1004ms, 62));
  check(command.has_value(), "late APPLIED fixture must emit Nav");
  (void)late_applied.onApplied(appliedFor(*command, 3), start + 1004ms + config().apply_timeout);
  const auto applied_status = late_applied.status(start + 1504ms);
  check(applied_status.fault == BridgeFaultCode::ApplyTimeout,
        "late APPLIED must fail at the apply deadline");
  check(applied_status.accepted_sequence != command->bridge_command_seq,
        "late APPLIED must not update accepted_sequence");
  check(applied_status.output_ack.producerBootId().empty(),
        "late APPLIED must not publish output ACK");

  MujocoDriverBridgeCore stale_submit(config());
  activateToReady(stale_submit, start + 2s);
  const auto stale = stale_submit.submitNav(nav(start + 2003ms + config().heartbeat_timeout, 63));
  check(stale.has_value(), "stale Nav submit must initiate physical zero without a prior poll");
  check(stale->kind == BridgeCommandKind::SafetyZero,
        "stale Nav submit must never emit Nav motion");
  check(stale_submit.status(start + 2503ms).lifecycle == BridgeLifecycle::DeactivatingZero,
        "stale Nav submit must await physical zero APPLIED");
  (void)stale_submit.onApplied(appliedFor(*stale, 3), start + 2504ms);
  check(stale_submit.status(start + 2504ms).fault == BridgeFaultCode::HeartbeatTimeout,
        "stale Nav zero APPLIED must preserve heartbeat-timeout root cause");
}

void testWriterLossAndAmbiguityRequireAppliedZero() {
  for (const std::uint32_t writers : {0U, 2U}) {
    const auto start = TimePoint{} + 16s + std::chrono::seconds(writers);
    MujocoDriverBridgeCore core(config());
    activateToReady(core, start);
    const auto nav_command = core.submitNav(nav(start + 4ms, 70 + writers));
    check(nav_command.has_value(), "writer fixture must emit Nav");
    (void)core.onApplied(appliedFor(*nav_command, 3), start + 5ms);
    check(core.status(start + 5ms).output_ack.accepted(), "fixture needs ACK");

    const auto zero = core.onWriterCount(writers, start + 6ms);
    check(zero.has_value(), "writer fault must emit a zero");
    check(!core.status(start + 6ms).ready, "writer fault must clear ready");
    check(core.status(start + 6ms).output_ack.producerBootId().empty(),
          "writer fault must clear output ACK");
    check(zero->kind == BridgeCommandKind::WriterFaultZero, "writer fault must use typed zero");

    (void)core.onApplied(appliedFor(*zero, 4), start + 7ms);
    check(core.status(start + 7ms).lifecycle == BridgeLifecycle::FaultClosed,
          "writer-fault zero APPLIED must end FaultClosed");
  }
}

void testPlannedDeactivateClearsAckAndStopsAfterZero() {
  const auto start = TimePoint{} + 20s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto nav_command = core.submitNav(nav(start + 4ms, 80));
  check(nav_command.has_value(), "deactivate fixture must emit Nav");
  (void)core.onApplied(appliedFor(*nav_command, 3), start + 5ms);

  const auto zero =
      core.onDeactivate(DeactivateMessage{kBridgeBoot, kControllerBoot, 3}, start + 6ms);
  check(zero.has_value(), "DEACTIVATE must emit a zero");
  check(!core.status(start + 6ms).ready, "DEACTIVATE must clear ready");
  check(core.status(start + 6ms).output_ack.producerBootId().empty(),
        "DEACTIVATE must clear ACK immediately");
  check(zero->kind == BridgeCommandKind::DeactivateZero, "DEACTIVATE must use planned zero kind");
  (void)core.onApplied(appliedFor(*zero, 4), start + 7ms);
  check(core.status(start + 7ms).lifecycle == BridgeLifecycle::Stopped,
        "planned zero APPLIED must stop bridge");

  (void)core.onApplied(appliedFor(*zero, 4), start + 7500us);
  check(core.status(start + 7500us).lifecycle == BridgeLifecycle::FaultClosed,
        "duplicate stop APPLIED must be a terminal protocol fault");

  (void)core.onWriterCount(1, start + 8ms);
  (void)core.onActivate(ActivateMessage{kBridgeBoot, kControllerBoot, 4}, start + 9ms);
  check(core.status(start + 9ms).lifecycle == BridgeLifecycle::FaultClosed,
        "terminal bridge must not recover in place");
}

void testPlannedDeactivatePublishesTypedPhysicalStopEvidence() {
  const auto start = TimePoint{} + 21s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);

  const auto zero =
      core.onDeactivate(DeactivateMessage{kBridgeBoot, kControllerBoot, 3}, start + 4ms);
  check(zero.has_value(), "planned deactivate must emit a zero");
  check(zero->kind == BridgeCommandKind::DeactivateZero,
        "planned deactivate must emit the typed deactivate zero");
  check(!core.stoppedEvidence().has_value(),
        "issuing a deactivate zero must not claim physical stop evidence");

  (void)core.onApplied(appliedFor(*zero, 7), start + 5ms);
  const auto evidence = core.stoppedEvidence();
  check(evidence.has_value(), "physical deactivate APPLIED must publish stop evidence");
  check(evidence->bridge_boot_id == kBridgeBoot &&
            evidence->controller_boot_id == kControllerBoot &&
            evidence->bridge_command_seq == zero->bridge_command_seq &&
            evidence->applied_step_seq == 7 && evidence->kind == BridgeCommandKind::DeactivateZero,
        "stop evidence must preserve the exact physical deactivate identity");
  check(lingtu::sim::driver_bridge::serializeStopped(*evidence) ==
            std::string("LT_DRIVER_STOPPED_V2\t") + kBridgeBoot + "\t" + kControllerBoot + "\t" +
                std::to_string(zero->bridge_command_seq) + "\t7\tdeactivate_zero",
        "STOPPED serializer must be deterministic and typed");
}

void testInvalidDeactivateAppliedNeverPublishesStopEvidence() {
  using Mutation = std::function<void(AppliedMessage &)>;
  const std::vector<Mutation> mutations{
      [](AppliedMessage &value) { value.bridge_boot_id = kWrongBoot; },
      [](AppliedMessage &value) { value.controller_boot_id = kWrongBoot; },
      [](AppliedMessage &value) { ++value.bridge_command_seq; },
      [](AppliedMessage &value) { value.kind = BridgeCommandKind::SafetyZero; },
      [](AppliedMessage &value) { value.applied_step_seq = 0; },
  };

  std::uint64_t index = 0;
  for (const auto &mutate : mutations) {
    const auto start = TimePoint{} + 40s + std::chrono::seconds(index++);
    MujocoDriverBridgeCore core(config());
    activateToReady(core, start);
    const auto zero =
        core.onDeactivate(DeactivateMessage{kBridgeBoot, kControllerBoot, 3}, start + 4ms);
    check(zero.has_value(), "invalid stop evidence fixture must emit deactivate zero");
    auto invalid = appliedFor(*zero, 7);
    mutate(invalid);
    (void)core.onApplied(invalid, start + 5ms);
    check(core.status(start + 5ms).lifecycle == BridgeLifecycle::FaultClosed,
          "invalid deactivate APPLIED must fault closed");
    check(!core.stoppedEvidence().has_value(),
          "invalid deactivate APPLIED must never publish stop evidence");
  }
}

void testPendingNavStopSuppressesAckThenEmitsZero() {
  const auto start = TimePoint{} + 21s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto nav_command = core.submitNav(nav(start + 4ms, 81));
  check(nav_command.has_value(), "pending-stop fixture must emit Nav");

  const auto early_zero =
      core.onDeactivate(DeactivateMessage{kBridgeBoot, kControllerBoot, 3}, start + 5ms);
  check(!early_zero.has_value(), "stop must not overwrite an immutable pending Nav command");
  check(core.status(start + 5ms).output_ack.producerBootId().empty(),
        "stop request must not ACK pending Nav");

  const auto zero = core.onApplied(appliedFor(*nav_command, 3), start + 6ms);
  check(zero.has_value(), "pending Nav APPLIED must release stop zero");
  check(zero->kind == BridgeCommandKind::DeactivateZero,
        "pending stop must preserve planned zero kind");
  check(core.status(start + 6ms).output_ack.producerBootId().empty(),
        "Nav applied after stop request must not publish output ACK");
  (void)core.onApplied(appliedFor(*zero, 4), start + 7ms);
  check(core.status(start + 7ms).lifecycle == BridgeLifecycle::Stopped,
        "stop zero must complete planned deactivation");
}

void testSafetyZeroReturnsToNonMotionReady() {
  const auto start = TimePoint{} + 22s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto command = core.submitNav(nav(start + 4ms, 90));
  check(command.has_value(), "safety fixture must emit Nav");
  (void)core.onApplied(appliedFor(*command, 3), start + 5ms);

  const auto zero = core.requestSafetyStop(start + 6ms);
  check(zero.has_value(), "safety stop must emit zero");
  check(zero->kind == BridgeCommandKind::SafetyZero, "zero must be typed");
  check(!core.status(start + 6ms).ready, "safety stop must clear ready");
  check(core.status(start + 6ms).stop_cause == BridgeStopCause::RequestedSafety,
        "explicit safety stop must retain its exact cause until zero is applied");
  (void)core.onApplied(appliedFor(*zero, 4), start + 7ms);
  const auto status = core.status(start + 7ms);
  check(status.ready, "fresh writer/controller may return non-motion Ready");
  check(status.output_ack.producerBootId().empty(), "safety zero must leave output ACK empty");
  check(status.applied_walk.x == 0.0 && status.applied_walk.y == 0.0 &&
            status.applied_walk.z == 0.0,
        "physical safety zero must clear the applied walk evidence");
  check(!core.stoppedEvidence().has_value(),
        "physical safety zero must not masquerade as planned terminal evidence");
}

void testPlannedDeactivateSupersedesInFlightSafetyStop() {
  const auto start = TimePoint{} + 23s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);

  const auto safety_zero = core.requestSafetyStop(start + 4ms);
  check(safety_zero.has_value(), "safety-stop fixture must emit zero");
  check(safety_zero->kind == BridgeCommandKind::SafetyZero,
        "safety-stop fixture must start with typed safety zero");

  const auto duplicate_zero =
      core.onDeactivate(DeactivateMessage{kBridgeBoot, kControllerBoot, 3}, start + 5ms);
  check(!duplicate_zero.has_value(),
        "planned deactivate must not overwrite the in-flight physical zero");
  const auto deactivate_zero = core.onApplied(appliedFor(*safety_zero, 3), start + 6ms);
  check(deactivate_zero.has_value() && deactivate_zero->kind == BridgeCommandKind::DeactivateZero,
        "planned deactivate must issue its own typed zero after the in-flight safety zero");
  check(core.status(start + 6ms).lifecycle == BridgeLifecycle::DeactivatingZero,
        "a safety-zero APPLIED must not masquerade as planned terminal stop");
  check(!core.stoppedEvidence().has_value(),
        "a safety-zero APPLIED must not publish planned stop evidence");

  (void)core.onApplied(appliedFor(*deactivate_zero, 4), start + 7ms);
  check(core.status(start + 7ms).lifecycle == BridgeLifecycle::Stopped,
        "planned deactivate must stop after its own zero is physically applied");
  check(core.stoppedEvidence().has_value(),
        "the exact planned deactivate zero must publish stop evidence");
}

void testRealCoreWatchdogEmitsSafetyZero() {
  const auto start = TimePoint{} + 24s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto command = core.submitNav(nav(start + 4ms, 95));
  check(command.has_value(), "watchdog fixture must emit Nav");
  (void)core.onApplied(appliedFor(*command, 3), start + 5ms);

  const auto zero = core.poll(start + 4ms + config().limits.command_timeout);
  check(zero.has_value(), "real Core watchdog must emit a zero");
  check(zero->kind == BridgeCommandKind::SafetyZero, "watchdog must use the typed safety zero");
  check(core.status(start + 104ms).stop_cause == BridgeStopCause::CommandTimeout,
        "real Core watchdog must report command_timeout instead of a generic lifecycle state");
  check(core.status(start + 104ms).output_ack.producerBootId().empty(),
        "watchdog stop must clear output ACK");
  (void)core.onApplied(appliedFor(*zero, 4), start + 105ms);
  check(core.status(start + 105ms).ready,
        "watchdog zero may return to non-motion Ready when heartbeat is fresh");
}

void testZeroApplyTimeoutAndControllerEofAreTerminal() {
  const auto start = TimePoint{} + 23s;
  MujocoDriverBridgeCore zero_timeout(config());
  activateToReady(zero_timeout, start);
  const auto zero = zero_timeout.requestSafetyStop(start + 4ms);
  check(zero.has_value(), "zero-timeout fixture must emit zero");
  (void)zero_timeout.poll(start + 4ms + config().apply_timeout);
  check(zero_timeout.status(start + 504ms).fault == BridgeFaultCode::ApplyTimeout,
        "unapplied zero must fail closed at bounded timeout");

  MujocoDriverBridgeCore eof_core(config());
  activateToReady(eof_core, start + 1s);
  eof_core.controllerEof();
  check(eof_core.status(start + 1s).fault == BridgeFaultCode::ControllerEof,
        "controller EOF must be terminal without fake APPLIED");
  check(!eof_core.status(start + 1s).has_pending, "controller EOF must not invent a pending zero");
}

void testExpiredLatestNeverEmitsOldMotion() {
  const auto start = TimePoint{} + 25s;
  MujocoDriverBridgeCore core(config());
  activateToReady(core, start);
  const auto first = core.submitNav(nav(start + 4ms, 100));
  check(first.has_value(), "latest-expiry fixture must emit first Nav");
  check(!core.submitNav(nav(start + 5ms, 101)).has_value(), "second Nav must coalesce");

  const auto next = core.onApplied(appliedFor(*first, 3), start + 105ms);
  check(next.has_value(), "expired latest must trigger a zero");
  check(next->kind == BridgeCommandKind::SafetyZero,
        "expired latest must never emit stale Nav motion");
  check(core.status(start + 105ms).stop_cause == BridgeStopCause::QueuedCommandExpired,
        "expired latest must retain the queued-command stop cause");
  check(core.status(start + 105ms).output_ack.producerBootId().empty(),
        "expired latest safety transition must clear ACK");
}

void testPrivateV2ParserAndSerializersAreStrict() {
  const std::string bridge = kBridgeBoot;
  const std::string controller = kControllerBoot;

  const auto activate = lingtu::sim::driver_bridge::parseControllerLine(
      "LT_DRIVER_ACTIVATE_V2\t" + bridge + "\t" + controller + "\t1");
  check(std::get<ActivateMessage>(activate).control_seq == 1, "ACTIVATE must parse exact uint64");
  const auto heartbeat = lingtu::sim::driver_bridge::parseControllerLine(
      "LT_DRIVER_HEARTBEAT_V2\t" + bridge + "\t" + controller + "\t2\t3");
  check(std::get<HeartbeatMessage>(heartbeat).step_seq == 3, "HEARTBEAT must parse exact step");
  const auto deactivate = lingtu::sim::driver_bridge::parseControllerLine(
      "LT_DRIVER_DEACTIVATE_V2\t" + bridge + "\t" + controller + "\t3");
  check(std::get<DeactivateMessage>(deactivate).control_seq == 3,
        "DEACTIVATE must parse exact control sequence");
  const auto applied = lingtu::sim::driver_bridge::parseControllerLine(
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
      "\t2\tnav\tproducer-a\t91\t1\t-0.5\t1\t4");
  check(std::get<AppliedMessage>(applied).output_sequence == 91,
        "APPLIED must preserve Nav output identity");

  BridgeCommand command{
      bridge, controller, 2, BridgeCommandKind::Nav, "producer-a", 91, {1.0, -0.5, 1.0},
  };
  check(lingtu::sim::driver_bridge::serializeCommand(command) ==
            "LT_DRIVER_COMMAND_V2\t" + bridge + "\t" + controller +
                "\t2\tnav\tproducer-a\t91\t1\t-0.5\t1",
        "COMMAND serializer must be deterministic");
  check(lingtu::sim::driver_bridge::serializeReady(
            ReadyMessage{bridge, controller, 2, "producer-a", 91}) ==
            "LT_DRIVER_READY_V2\t" + bridge + "\t" + controller + "\t2\tproducer-a\t91",
        "READY serializer must preserve distinct sequences");
  check(lingtu::sim::driver_bridge::serializeFault(
            FaultMessage{bridge, controller, BridgeFaultCode::ProtocolViolation}) ==
            "LT_DRIVER_FAULT_V2\t" + bridge + "\t" + controller + "\tprotocol_violation",
        "FAULT serializer must use fixed reason tokens");

  const std::vector<std::string> invalid{
      "",
      "UNKNOWN\t" + bridge,
      "ACTIVATE\t" + bridge + "\t" + controller + "\t1",
      "LT_DRIVER_ACTIVATE_V2\t" + bridge + "\t" + controller,
      "LT_DRIVER_ACTIVATE_V2\t" + bridge + "\t" + controller + "\t0",
      "LT_DRIVER_ACTIVATE_V2\t" + bridge + "\t" + controller + "\t-1",
      "LT_DRIVER_ACTIVATE_V2\t" + bridge + "\t" + controller + "\t18446744073709551616",
      "LT_DRIVER_ACTIVATE_V2\tAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\t" + controller + "\t1",
      "LT_DRIVER_ACTIVATE_V2\t" + bridge + "\tbad controller\t1",
      "LT_DRIVER_ACTIVATE_V2\t" + bridge + "\t" + controller + "\t1\textra",
      "LT_DRIVER_HEARTBEAT_V2\t" + bridge + "\t" + controller + "\t2\t0",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t2\tnav\tproducer-a\t91\tnan\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t2\tnav\tproducer-a\t91\tinf\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t2\tnav\tbad producer\t91\t0\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller + "\t2\tnav\t-\t91\t0\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller + "\t2\tnav\tproducer-a\t0\t0\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t2\tactivation_zero\tproducer-a\t0\t0\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t2\tactivation_zero\t\t0\t0\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t2\tactivation_zero\t-\t0\t0.1\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller + "\t2\tunknown\t-\t0\t0\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t0\tactivation_zero\t-\t0\t0\t0\t0\t3",
      "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
          "\t2\tactivation_zero\t-\t0\t0\t0\t0\t0",
  };
  for (const auto &line : invalid) {
    expectThrows([&]() { (void)lingtu::sim::driver_bridge::parseControllerLine(line); },
                 "invalid V2 line must fail closed");
  }
  expectThrows(
      [&]() {
        (void)lingtu::sim::driver_bridge::parseControllerLine(
            std::string(lingtu::sim::driver_bridge::kMaxProtocolLineBytes + 1, 'x'));
      },
      "oversize V2 line must fail before parsing");

  auto unknown_command = command;
  unknown_command.kind = static_cast<BridgeCommandKind>(999);
  expectThrows([&]() { (void)lingtu::sim::driver_bridge::serializeCommand(unknown_command); },
               "unknown command kind must not serialize");
  expectThrows(
      [&]() {
        (void)lingtu::sim::driver_bridge::serializeFault(
            FaultMessage{bridge, controller, static_cast<BridgeFaultCode>(999)});
      },
      "unknown fault code must not serialize");
  expectThrows(
      [&]() {
        (void)lingtu::sim::driver_bridge::serializeStopped(
            StopMessage{bridge, controller, 2, 4, BridgeCommandKind::SafetyZero});
      },
      "STOPPED must reject a safety-zero kind");

  const std::string max_producer(lingtu::sim::driver_bridge::kMaxProducerTokenBytes, 'p');
  check(lingtu::sim::driver_bridge::validProducerToken(max_producer),
        "128-byte producer token must remain valid");
  check(!lingtu::sim::driver_bridge::validProducerToken(max_producer + "p"),
        "producer token above fixed limit must fail closed");

  class CommaDecimalPoint final : public std::numpunct<char> {
   protected:
    char do_decimal_point() const override { return ','; }
  };
  const std::locale previous = std::locale();
  std::locale::global(std::locale(previous, new CommaDecimalPoint));
  try {
    const auto locale_independent = lingtu::sim::driver_bridge::parseControllerLine(
        "LT_DRIVER_APPLIED_V2\t" + bridge + "\t" + controller +
        "\t2\tnav\tproducer-a\t91\t1.5\t-0.5\t0\t4");
    check(std::get<AppliedMessage>(locale_independent).walk_x == 1.5,
          "wire doubles must ignore process C++ locale");
  } catch (...) {
    std::locale::global(previous);
    throw;
  }
  std::locale::global(previous);
}

}  // namespace

int main() {
  try {
    testConstructionIsStrictAndSideEffectFree();
    testActivationRequiresAppliedThenLaterHeartbeat();
    testPhysicalNavAppliedPublishesExactOutputAck();
    testNavZeroWalkIsCanonicalAcrossWireAndAppliedAck();
    testPendingNavPreservesLastPhysicalAckUntilReplacementIsApplied();
    testCoalescedNavPreservesAppliedAckWhileReleasingLatestCommand();
    testNavIngressUsesRealFreshnessAndNormalizationGates();
    testPendingCommandIsImmutableAndLatestIsCoalesced();
    testMatchingAppliedReleasesFreshLatest();
    testQueuedNavUsesOriginalSourceTimeWhenReleased();
    testEveryAppliedEchoFieldIsFailClosed();
    testControlSequenceRejectsReplayAndHeartbeatStepMayRepeat();
    testAppliedStepMustBeLaterThanObservedHeartbeat();
    testApplyTimeoutIsTerminal();
    testHeartbeatTimeoutRequiresAppliedPhysicalZeroBeforeFault();
    testHeartbeatTimeoutDuringPendingNavSuppressesAckAndThenZeros();
    testEveryControlEntryUsesHeartbeatTimeoutZeroBarrier();
    testLateEventsCannotBypassDeadlines();
    testWriterLossAndAmbiguityRequireAppliedZero();
    testPlannedDeactivateClearsAckAndStopsAfterZero();
    testPlannedDeactivatePublishesTypedPhysicalStopEvidence();
    testInvalidDeactivateAppliedNeverPublishesStopEvidence();
    testPendingNavStopSuppressesAckThenEmitsZero();
    testSafetyZeroReturnsToNonMotionReady();
    testPlannedDeactivateSupersedesInFlightSafetyStop();
    testRealCoreWatchdogEmitsSafetyZero();
    testZeroApplyTimeoutAndControllerEofAreTerminal();
    testExpiredLatestNeverEmitsOldMotion();
    testPrivateV2ParserAndSerializersAreStrict();
    std::cout << "test_mujoco_driver_bridge_core: PASS\n";
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "test_mujoco_driver_bridge_core: FAIL: " << exc.what() << '\n';
    return 1;
  }
}
