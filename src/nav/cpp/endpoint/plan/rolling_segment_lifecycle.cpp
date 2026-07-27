#include "plan/rolling_segment_lifecycle.hpp"

#include <cmath>
#include <iterator>
#include <limits>
#include <type_traits>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

constexpr double kPlanarTolerance = 1e-6;
constexpr double kRequestMaxAgeS = 1.0;
constexpr double kRequestFutureToleranceS = 0.25;
constexpr double kQuaternionNormTolerance = 1e-3;
constexpr std::size_t kMaximumCells = 262'144U;
constexpr std::size_t kTerminalCacheLimit = 128U;

struct DecodedGrid {
  std::optional<RollingMapSegmentSnapshot> snapshot;
  std::string reason;
};

bool requestFresh(const RollingSegmentCommand &command, double now_s) {
  return std::isfinite(command.stamp_s) && command.stamp_s > 0.0 && std::isfinite(now_s) &&
         now_s > 0.0 && command.stamp_s <= now_s + kRequestFutureToleranceS &&
         now_s - command.stamp_s <= kRequestMaxAgeS;
}

bool targetOrientationValid(const RollingSegmentTarget &target) {
  if (!std::isfinite(target.qx) || !std::isfinite(target.qy) || !std::isfinite(target.qz) ||
      !std::isfinite(target.qw)) {
    return false;
  }
  const double norm = std::sqrt(target.qx * target.qx + target.qy * target.qy +
                                target.qz * target.qz + target.qw * target.qw);
  return std::isfinite(norm) && std::abs(target.qx) <= kPlanarTolerance &&
         std::abs(target.qy) <= kPlanarTolerance &&
         std::abs(norm - 1.0) <= kQuaternionNormTolerance;
}

bool originPlanar(const RollingSegmentExecutionGrid &grid) {
  return std::isfinite(grid.origin_x) && std::isfinite(grid.origin_y) &&
         std::isfinite(grid.origin_z) && std::isfinite(grid.origin_qx) &&
         std::isfinite(grid.origin_qy) && std::isfinite(grid.origin_qz) &&
         std::isfinite(grid.origin_qw) && std::abs(grid.origin_z) <= kPlanarTolerance &&
         std::abs(grid.origin_qx) <= kPlanarTolerance &&
         std::abs(grid.origin_qy) <= kPlanarTolerance &&
         std::abs(grid.origin_qz) <= kPlanarTolerance &&
         std::abs(grid.origin_qw - 1.0) <= kPlanarTolerance;
}

DecodedGrid decodeGrid(const RollingSegmentExecutionGrid &grid) {
  DecodedGrid decoded;
  if (!grid.payload_complete) {
    decoded.reason = "execution_grid_payload_incomplete";
    return decoded;
  }
  if (grid.frame_id != "map") {
    decoded.reason = "execution_grid_frame_invalid";
    return decoded;
  }
  if (!grid.live || grid.session_id.empty() || grid.reset_epoch == 0U || grid.generation == 0U) {
    decoded.reason = "execution_grid_identity_invalid";
    return decoded;
  }
  if (!std::isfinite(grid.stamp_s) || grid.stamp_s <= 0.0 || !std::isfinite(grid.resolution) ||
      grid.resolution <= 0.0) {
    decoded.reason = "execution_grid_metadata_invalid";
    return decoded;
  }
  if (!originPlanar(grid)) {
    decoded.reason = "execution_grid_origin_non_planar";
    return decoded;
  }

  const std::size_t width = static_cast<std::size_t>(grid.width);
  const std::size_t height = static_cast<std::size_t>(grid.height);
  if (width == 0U || height == 0U || width > kMaximumCells / height ||
      width > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
      height > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    decoded.reason = "execution_grid_dimensions_invalid";
    return decoded;
  }
  const std::size_t cell_count = width * height;
  if (grid.occupancy.size() != cell_count || grid.terrain_cost.size() != cell_count) {
    decoded.reason = "execution_grid_payload_size_invalid";
    return decoded;
  }
  if (grid.terrain_risk_ready &&
      (!std::isfinite(grid.terrain_risk_stamp_s) || grid.terrain_risk_stamp_s <= 0.0)) {
    decoded.reason = "execution_grid_terrain_risk_stamp_invalid";
    return decoded;
  }

  RollingMapSegmentSnapshot snapshot;
  snapshot.occupancy.width = static_cast<int>(width);
  snapshot.occupancy.height = static_cast<int>(height);
  snapshot.occupancy.resolution = grid.resolution;
  snapshot.occupancy.origin_x = grid.origin_x;
  snapshot.occupancy.origin_y = grid.origin_y;
  snapshot.occupancy.cells.reserve(cell_count);
  snapshot.terrain_cost.width = static_cast<int>(width);
  snapshot.terrain_cost.height = static_cast<int>(height);
  snapshot.terrain_cost.resolution = grid.resolution;
  snapshot.terrain_cost.origin_x = grid.origin_x;
  snapshot.terrain_cost.origin_y = grid.origin_y;
  snapshot.terrain_cost.costs.reserve(cell_count);
  for (std::size_t index = 0U; index < cell_count; ++index) {
    const std::uint8_t occupancy = grid.occupancy[index];
    if (occupancy == 0U) {
      snapshot.occupancy.cells.push_back(lingtu::explore::kFree);
    } else if (occupancy == 100U) {
      snapshot.occupancy.cells.push_back(lingtu::explore::kOccupied);
    } else if (occupancy == 255U) {
      snapshot.occupancy.cells.push_back(lingtu::explore::kUnknown);
    } else {
      decoded.reason = "execution_grid_occupancy_encoding_invalid";
      return decoded;
    }
    const std::uint8_t terrain_cost = grid.terrain_cost[index];
    if (terrain_cost > 100U) {
      decoded.reason = "execution_grid_terrain_cost_invalid";
      return decoded;
    }
    snapshot.terrain_cost.costs.push_back(static_cast<float>(terrain_cost));
  }
  snapshot.identity.frame_id = grid.frame_id;
  snapshot.identity.session_id = grid.session_id;
  snapshot.identity.reset_epoch = grid.reset_epoch;
  snapshot.identity.generation = grid.generation;
  snapshot.identity.live = grid.live;
  snapshot.stamp_s = grid.stamp_s;
  snapshot.terrain_risk_stamp_s = grid.terrain_risk_stamp_s;
  snapshot.terrain_risk_ready = grid.terrain_risk_ready;
  if (!snapshot.identity.valid()) {
    decoded.reason = "execution_grid_identity_invalid";
    return decoded;
  }
  decoded.snapshot = std::move(snapshot);
  return decoded;
}

RollingSegmentAck rejection(const RollingSegmentCommand &command, std::string reason) {
  return {
      command.request_id,
      command.kind,
      false,
      command.session_id,
      command.reset_epoch,
      command.minimum_generation,
      false,
      std::move(reason),
  };
}

}  // namespace

RollingSegmentLifecycle::RollingSegmentLifecycle(RollingMapSegmentExecutorConfig executor_config)
    : executor_(std::move(executor_config)) {}

RollingSegmentStepResult RollingSegmentLifecycle::step(const RollingSegmentEvent &event) {
  return std::visit([this](const auto &value) { return handle(value); }, event);
}

RollingSegmentLifecycleSnapshot RollingSegmentLifecycle::snapshot() const {
  bool terminal_delivery_pending = false;
  for (const auto &terminal : terminal_cache_) {
    if (terminal.execution && !terminal.status_delivered) {
      terminal_delivery_pending = true;
      break;
    }
  }
  return {
      active_.has_value(),
      latest_input_.has_value(),
      input_invalidated_this_tick_,
      terminal_delivery_pending,
      input_error_,
  };
}

RollingSegmentStepResult RollingSegmentLifecycle::handle(const RollingSegmentBeginTick &) {
  input_invalidated_this_tick_ = false;
  RollingSegmentStepResult result;
  for (auto &terminal : terminal_cache_) {
    if (!terminal.status_delivered && terminal.execution) {
      result.effects.emplace_back(makeTerminalStatusEffect(terminal));
    }
  }
  return result;
}

RollingSegmentStepResult
RollingSegmentLifecycle::handle(const RollingSegmentObserveExecutionGrid &event) {
  auto decoded = decodeGrid(event.grid);
  if (!decoded.snapshot) {
    latest_input_.reset();
    input_error_ = decoded.reason;
    input_invalidated_this_tick_ = true;
    if (active_) {
      return terminateActive(ExplorationSegmentState::kStaleBinding, input_error_);
    }
    executor_.reset();
    return {};
  }
  if (latest_input_) {
    const auto &previous = latest_input_->snapshot;
    const auto &incoming = *decoded.snapshot;
    if (previous.identity.sameSource(incoming.identity) &&
        (incoming.identity.reset_epoch < previous.identity.reset_epoch ||
         (incoming.identity.reset_epoch == previous.identity.reset_epoch &&
          (incoming.identity.generation < previous.identity.generation ||
           (incoming.identity.generation == previous.identity.generation &&
            incoming.stamp_s + 1e-9 < previous.stamp_s))))) {
      latest_input_.reset();
      input_error_ = "execution_grid_provenance_regressed";
      input_invalidated_this_tick_ = true;
      if (active_) {
        return terminateActive(ExplorationSegmentState::kStaleBinding, input_error_);
      }
      executor_.reset();
      return {};
    }
  }
  RollingMapSegmentInput input;
  input.snapshot = std::move(*decoded.snapshot);
  latest_input_ = std::move(input);
  input_error_.clear();
  return {};
}

RollingSegmentStepResult
RollingSegmentLifecycle::handle(const RollingSegmentObserveInvalidInput &event) {
  latest_input_.reset();
  input_error_ = event.reason.empty() ? "execution_grid_invalidated" : event.reason;
  input_invalidated_this_tick_ = true;
  if (active_) {
    return terminateActive(ExplorationSegmentState::kStaleBinding,
                           event.active_terminal_reason.empty() ? input_error_
                                                                : event.active_terminal_reason);
  }
  executor_.reset();
  return {};
}

RollingSegmentStepResult RollingSegmentLifecycle::handle(const RollingSegmentCommandEvent &event) {
  const auto &command = event.command;
  RollingSegmentStepResult result;
  auto reject = [&](std::string reason, bool terminal_execute = false) {
    if (terminal_execute) {
      rememberTerminal(TerminalSegment{
          command.request_id,
          command.session_id,
          command.reset_epoch,
          command.minimum_generation,
          std::nullopt,
          ExplorationSegmentState::kFailed,
          reason,
          true,
      });
    }
    RollingSegmentAckEffect effect;
    effect.effect_id = nextEffectId();
    effect.ack = rejection(command, std::move(reason));
    result.effects.emplace_back(std::move(effect));
  };

  if (command.request_id.empty()) {
    reject("segment_request_id_empty");
    return result;
  }
  if (!isKnownExplorationSegmentCommandKind(command.kind)) {
    reject("unknown_segment_command_kind");
    return result;
  }
  if (command.frame_id != "map" || command.session_id.empty() || command.reset_epoch == 0U ||
      command.minimum_generation == 0U) {
    reject("segment_request_binding_invalid");
    return result;
  }
  const auto kind = static_cast<ExplorationSegmentCommandKind>(command.kind);
  const bool exact_active_binding = active_ && command.request_id == active_->request_id &&
                                    command.session_id == active_->identity.session_id &&
                                    command.reset_epoch == active_->identity.reset_epoch &&
                                    command.minimum_generation == active_->minimum_generation;
  TerminalSegment *terminal = findTerminal(command);
  if (!requestFresh(command, event.context.now_s) && terminal == nullptr && !exact_active_binding) {
    reject("segment_request_stale_or_invalid");
    return result;
  }
  if (terminal != nullptr) {
    RollingSegmentAckEffect effect;
    effect.effect_id = nextEffectId();
    effect.ack = {
        command.request_id,
        command.kind,
        false,
        terminal->execution ? terminal->execution->identity.session_id : command.session_id,
        terminal->execution ? terminal->execution->identity.reset_epoch : command.reset_epoch,
        terminal->execution ? terminal->execution->generation : command.minimum_generation,
        terminal->execution && terminal->execution->identity.live,
        "segment_terminal_replayed",
    };
    result.effects.emplace_back(std::move(effect));
    if (terminal->execution) {
      result.effects.emplace_back(makeTerminalStatusEffect(*terminal));
    }
    return result;
  }
  if (kind == ExplorationSegmentCommandKind::kCancel) {
    if (!active_) {
      reject("segment_not_active");
      return result;
    }
    if (!exact_active_binding) {
      reject("segment_cancel_binding_mismatch");
      return result;
    }
    RollingSegmentAckEffect ack_effect;
    ack_effect.effect_id = nextEffectId();
    ack_effect.ack = {
        command.request_id,
        command.kind,
        true,
        active_->identity.session_id,
        active_->identity.reset_epoch,
        active_->generation,
        active_->identity.live,
        "segment_cancel_accepted",
    };
    result.effects.emplace_back(std::move(ack_effect));
    auto terminal_result =
        terminateActive(ExplorationSegmentState::kCancelled, "segment_cancelled");
    result.effects.insert(result.effects.end(),
                          std::make_move_iterator(terminal_result.effects.begin()),
                          std::make_move_iterator(terminal_result.effects.end()));
    return result;
  }
  if (!std::isfinite(command.target.x) || !std::isfinite(command.target.y) ||
      !std::isfinite(command.target.z)) {
    reject("segment_target_invalid");
    return result;
  }
  if (!targetOrientationValid(command.target)) {
    reject("segment_target_orientation_invalid");
    return result;
  }
  if (active_) {
    if (!exact_active_binding) {
      reject("segment_already_active");
      return result;
    }
    RollingSegmentAckEffect ack_effect;
    ack_effect.effect_id = nextEffectId();
    ack_effect.ack = {
        command.request_id,
        command.kind,
        true,
        active_->identity.session_id,
        active_->identity.reset_epoch,
        active_->generation,
        active_->identity.live,
        "segment_execute_idempotent",
    };
    result.effects.emplace_back(std::move(ack_effect));
    RollingSegmentStatusEffect status_effect;
    status_effect.effect_id = nextEffectId();
    status_effect.status = {
        active_->request_id,          ExplorationSegmentState::kExecuting,
        active_->identity.session_id, active_->identity.reset_epoch,
        active_->generation,          active_->identity.live,
        "segment_executing",
    };
    result.effects.emplace_back(std::move(status_effect));
    return result;
  }
  if (input_invalidated_this_tick_) {
    reject("execution_grid_invalidated_this_tick", true);
    return result;
  }
  if (!latest_input_) {
    reject(input_error_.empty() ? "execution_grid_missing" : input_error_, true);
    return result;
  }
  const auto &identity = latest_input_->snapshot.identity;
  if (!identity.live || command.session_id != identity.session_id ||
      command.reset_epoch != identity.reset_epoch) {
    reject("segment_request_binding_mismatch", true);
    return result;
  }
  if (event.context.generic_navigation_active) {
    reject("generic_navigation_active", true);
    return result;
  }
  if (!event.context.input_ready || !event.context.autonomy_mode || !event.context.motion_allowed ||
      event.context.operator_takeover_latched || !event.context.driver_control_ready ||
      !event.context.robot_pose || !event.context.map_z || !std::isfinite(*event.context.map_z)) {
    reject("segment_inputs_not_ready", true);
    return result;
  }

  auto input = *latest_input_;
  input.now_s = event.context.now_s;
  input.robot_pose = *event.context.robot_pose;
  input.input_ready = true;
  const auto decision = executor_.plan(input, {
                                                  {command.target.x, command.target.y},
                                                  command.minimum_generation,
                                              });
  if (decision.action != RollingMapSegmentAction::Accepted || decision.path.size() < 2U ||
      !decision.executed_map.valid()) {
    executor_.reset();
    reject(decision.reason.empty() ? "segment_safe_prefix_rejected" : decision.reason, true);
    return result;
  }

  active_ = ActiveSegment{
      command.request_id,
      decision.executed_map,
      decision.executed_generation,
      command.minimum_generation,
  };

  const std::uint64_t activate_effect_id = nextEffectId();
  pending_critical_effects_.insert_or_assign(activate_effect_id,
                                             PendingCriticalEffectKind::kActivateAuthority);
  result.effects.emplace_back(RollingSegmentActivateAuthorityEffect{
      activate_effect_id,
      RollingSegmentEffectFailurePolicy::kAbortBatch,
  });
  result.effects.emplace_back(RollingSegmentInstallPathEffect{
      nextEffectId(),
      decision.path,
      *event.context.map_z,
      event.context.now_s,
  });
  result.effects.emplace_back(RollingSegmentPublishPathEffect{
      nextEffectId(),
      decision.path,
      *event.context.map_z,
      event.context.now_s,
  });

  RollingSegmentAckEffect ack_effect;
  ack_effect.effect_id = nextEffectId();
  ack_effect.failure_policy = RollingSegmentEffectFailurePolicy::kFailClosedActiveSegment;
  ack_effect.ack = {
      command.request_id,
      command.kind,
      true,
      decision.executed_map.session_id,
      decision.executed_map.reset_epoch,
      decision.executed_generation,
      decision.executed_map.live,
      decision.reason,
  };
  pending_critical_effects_.insert_or_assign(ack_effect.effect_id,
                                             PendingCriticalEffectKind::kAcceptedAck);
  result.effects.emplace_back(std::move(ack_effect));

  for (const auto &[state, reason] : {
           std::pair{ExplorationSegmentState::kAccepted, decision.reason},
           std::pair{ExplorationSegmentState::kExecuting, std::string{"segment_executing"}},
       }) {
    RollingSegmentStatusEffect status_effect;
    status_effect.effect_id = nextEffectId();
    status_effect.failure_policy = RollingSegmentEffectFailurePolicy::kFailClosedActiveSegment;
    status_effect.status = {
        command.request_id,
        state,
        decision.executed_map.session_id,
        decision.executed_map.reset_epoch,
        decision.executed_generation,
        decision.executed_map.live,
        reason,
    };
    pending_critical_effects_.insert_or_assign(status_effect.effect_id,
                                               PendingCriticalEffectKind::kAdmissionStatus);
    result.effects.emplace_back(std::move(status_effect));
  }
  return result;
}

RollingSegmentStepResult RollingSegmentLifecycle::handle(const RollingSegmentRevalidate &event) {
  if (!active_) {
    return {};
  }
  if (!latest_input_) {
    return terminateActive(ExplorationSegmentState::kStaleBinding,
                           input_error_.empty() ? "execution_grid_missing" : input_error_);
  }

  auto input = *latest_input_;
  input.now_s = event.context.now_s;
  input.input_ready = event.context.input_ready && event.context.autonomy_mode &&
                      event.context.motion_allowed && !event.context.operator_takeover_latched &&
                      event.context.driver_control_ready && event.context.robot_pose.has_value();
  input.robot_pose = event.context.robot_pose.value_or(lingtu::explore::Pose2D{});
  const auto decision = executor_.revalidate(input);
  if (decision.action != RollingMapSegmentAction::Cancel && executor_.active()) {
    return {};
  }
  const bool unsafe = decision.reason == "segment_path_no_longer_safe";
  return terminateActive(unsafe ? ExplorationSegmentState::kFailed
                                : ExplorationSegmentState::kStaleBinding,
                         decision.reason.empty() ? "segment_revalidation_failed" : decision.reason);
}

RollingSegmentStepResult
RollingSegmentLifecycle::handle(const RollingSegmentGenericPreempt &event) {
  return terminateActive(ExplorationSegmentState::kCancelled,
                         event.reason.empty() ? "segment_cancelled" : event.reason);
}

RollingSegmentStepResult RollingSegmentLifecycle::handle(const RollingSegmentMotionOutcome &event) {
  switch (event.kind) {
    case RollingSegmentMotionOutcomeKind::kReached:
      return terminateActive(ExplorationSegmentState::kReached,
                             event.reason.empty() ? "segment_reached" : event.reason);
    case RollingSegmentMotionOutcomeKind::kRecoveryExhausted: {
      const std::string recovery_reason =
          event.reason.empty() ? "local_recovery_exhausted" : event.reason;
      return terminateActive(ExplorationSegmentState::kFailed,
                             std::string{"segment_local_recovery_exhausted:"} + recovery_reason);
    }
    case RollingSegmentMotionOutcomeKind::kFinalSafetyStopped:
      return terminateActive(ExplorationSegmentState::kFailed,
                             event.reason.empty() ? "segment_final_safety_stopped" : event.reason);
  }
  return {};
}

RollingSegmentStepResult RollingSegmentLifecycle::handle(const RollingSegmentShutdown &) {
  return terminateActive(ExplorationSegmentState::kCancelled, "navd_shutdown");
}

RollingSegmentStepResult
RollingSegmentLifecycle::handle(const RollingSegmentEffectFeedback &event) {
  const auto terminal_pending = pending_terminal_statuses_.find(event.effect_id);
  if (terminal_pending != pending_terminal_statuses_.end()) {
    if (event.success) {
      if (auto *terminal = findTerminal(terminal_pending->second)) {
        terminal->status_delivered = true;
      }
    }
    pending_terminal_statuses_.erase(terminal_pending);
    return {};
  }

  const auto critical_pending = pending_critical_effects_.find(event.effect_id);
  if (critical_pending == pending_critical_effects_.end()) {
    return {};
  }
  const PendingCriticalEffectKind kind = critical_pending->second;
  pending_critical_effects_.erase(critical_pending);
  if (event.success || !active_) {
    return {};
  }
  if (kind == PendingCriticalEffectKind::kActivateAuthority) {
    const ActiveSegment execution = *active_;
    rememberTerminal(TerminalSegment{
        execution.request_id,
        execution.identity.session_id,
        execution.identity.reset_epoch,
        execution.minimum_generation,
        std::nullopt,
        ExplorationSegmentState::kFailed,
        "segment_control_authority_rejected",
        true,
    });
    active_.reset();
    executor_.reset();
    pending_critical_effects_.clear();
    RollingSegmentAckEffect ack_effect;
    ack_effect.effect_id = nextEffectId();
    ack_effect.ack = {
        execution.request_id,
        static_cast<std::int32_t>(ExplorationSegmentCommandKind::kExecute),
        false,
        execution.identity.session_id,
        execution.identity.reset_epoch,
        execution.minimum_generation,
        false,
        "segment_control_authority_rejected",
    };
    RollingSegmentStepResult result;
    result.effects.emplace_back(std::move(ack_effect));
    return result;
  }
  return terminateActive(ExplorationSegmentState::kFailed,
                         kind == PendingCriticalEffectKind::kAcceptedAck
                             ? "segment_ack_publish_failed"
                             : "segment_status_publish_failed");
}

std::uint64_t RollingSegmentLifecycle::nextEffectId() noexcept {
  return next_effect_id_++;
}

RollingSegmentLifecycle::TerminalSegment *
RollingSegmentLifecycle::findTerminal(const RollingSegmentCommand &command) {
  return findTerminal(TerminalKey{
      command.request_id,
      command.session_id,
      command.reset_epoch,
      command.minimum_generation,
  });
}

RollingSegmentLifecycle::TerminalSegment *
RollingSegmentLifecycle::findTerminal(const TerminalKey &key) {
  for (auto it = terminal_cache_.rbegin(); it != terminal_cache_.rend(); ++it) {
    if (it->request_id == key.request_id && it->session_id == key.session_id &&
        it->reset_epoch == key.reset_epoch && it->minimum_generation == key.minimum_generation) {
      return &*it;
    }
  }
  return nullptr;
}

RollingSegmentLifecycle::TerminalSegment &
RollingSegmentLifecycle::rememberTerminal(TerminalSegment terminal) {
  for (auto it = terminal_cache_.rbegin(); it != terminal_cache_.rend(); ++it) {
    if (it->request_id == terminal.request_id && it->session_id == terminal.session_id &&
        it->reset_epoch == terminal.reset_epoch &&
        it->minimum_generation == terminal.minimum_generation) {
      *it = std::move(terminal);
      return *it;
    }
  }
  if (terminal_cache_.size() >= kTerminalCacheLimit) {
    terminal_cache_.pop_front();
  }
  terminal_cache_.push_back(std::move(terminal));
  return terminal_cache_.back();
}

RollingSegmentStatusEffect
RollingSegmentLifecycle::makeTerminalStatusEffect(TerminalSegment &terminal) {
  RollingSegmentStatusEffect effect;
  effect.effect_id = nextEffectId();
  effect.failure_policy = RollingSegmentEffectFailurePolicy::kRetryTerminalStatus;
  const auto &execution = *terminal.execution;
  effect.status = {
      execution.request_id,
      terminal.state,
      execution.identity.session_id,
      execution.identity.reset_epoch,
      execution.generation,
      execution.identity.live,
      terminal.reason,
  };
  pending_terminal_statuses_.insert_or_assign(effect.effect_id, TerminalKey{
                                                                    terminal.request_id,
                                                                    terminal.session_id,
                                                                    terminal.reset_epoch,
                                                                    terminal.minimum_generation,
                                                                });
  return effect;
}

RollingSegmentStepResult RollingSegmentLifecycle::terminateActive(ExplorationSegmentState state,
                                                                  std::string reason) {
  RollingSegmentStepResult result;
  if (!active_) {
    return result;
  }
  const ActiveSegment execution = *active_;
  pending_critical_effects_.clear();
  TerminalSegment &terminal = rememberTerminal(TerminalSegment{
      execution.request_id,
      execution.identity.session_id,
      execution.identity.reset_epoch,
      execution.minimum_generation,
      execution,
      state,
      std::move(reason),
      false,
  });

  result.effects.emplace_back(RollingSegmentStopAuthorityEffect{nextEffectId()});
  result.effects.emplace_back(makeTerminalStatusEffect(terminal));
  result.effects.emplace_back(RollingSegmentClearMotionEffect{
      nextEffectId(),
      RollingSegmentEffectFailurePolicy::kAbortBatch,
      terminal.reason,
  });
  active_.reset();
  executor_.reset();
  return result;
}

}  // namespace lingtu::nav::endpoint
