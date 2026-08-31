#pragma once

#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include "runtime/rolling/contract.hpp"
#include "planning/rolling/segment.hpp"

namespace lingtu::nav::endpoint {

struct RollingSegmentRuntimeContext {
  double now_s{0.0};
  bool input_ready{false};
  bool autonomy_mode{false};
  bool motion_allowed{false};
  bool operator_takeover_latched{false};
  bool driver_control_ready{false};
  bool generic_navigation_active{false};
  std::optional<lingtu::explore::Pose2D> robot_pose;
  std::optional<double> map_z;
};

struct RollingSegmentBeginTick {
  double now_s{0.0};
};

struct RollingSegmentObserveExecutionGrid {
  RollingSegmentExecutionGrid grid;
};

struct RollingSegmentObserveInvalidInput {
  std::string reason;
  std::string active_terminal_reason;
};

struct RollingSegmentCommandEvent {
  RollingSegmentCommand command;
  RollingSegmentRuntimeContext context;
};

struct RollingSegmentIngressRejected {
  RollingSegmentCommand command;
  RollingSegmentRuntimeContext context;
  std::string reason;
};

struct RollingSegmentRevalidate {
  RollingSegmentRuntimeContext context;
};

struct RollingSegmentGenericPreempt {
  std::string reason;
};

enum class RollingSegmentMotionOutcomeKind {
  kReached,
  kRecoveryExhausted,
};

struct RollingSegmentMotionOutcome {
  RollingSegmentMotionOutcomeKind kind{RollingSegmentMotionOutcomeKind::kReached};
  std::string reason;
};

struct RollingSegmentShutdown {};

struct RollingSegmentEffectFeedback {
  std::uint64_t effect_id{0U};
  bool success{false};
};

using RollingSegmentEvent =
    std::variant<RollingSegmentBeginTick, RollingSegmentObserveExecutionGrid,
                 RollingSegmentObserveInvalidInput, RollingSegmentCommandEvent,
                 RollingSegmentIngressRejected, RollingSegmentRevalidate,
                 RollingSegmentGenericPreempt, RollingSegmentMotionOutcome, RollingSegmentShutdown,
                 RollingSegmentEffectFeedback>;

struct RollingSegmentAck {
  // The transport adapter stamps the wire envelope when this effect is
  // published; lifecycle time is used only for domain freshness decisions.
  std::string request_id;
  std::int32_t kind{0};
  bool accepted{false};
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{false};
  std::string reason;
};

struct RollingSegmentStatus {
  // Terminal retries retain the domain outcome while the transport adapter
  // assigns a fresh wire timestamp for each publication attempt.
  std::string request_id;
  ExplorationSegmentState state{ExplorationSegmentState::kFailed};
  std::string session_id;
  std::uint64_t reset_epoch{0U};
  std::uint64_t generation{0U};
  bool live{false};
  std::string reason;
};

enum class RollingSegmentEffectFailurePolicy {
  kIgnore,
  kAbortBatch,
  kFailClosedActiveSegment,
  kRetryTerminalStatus,
};

struct RollingSegmentActivateAuthorityEffect {
  std::uint64_t effect_id{0U};
  RollingSegmentEffectFailurePolicy failure_policy{RollingSegmentEffectFailurePolicy::kAbortBatch};
};

struct RollingSegmentInstallPathEffect {
  std::uint64_t effect_id{0U};
  std::vector<lingtu::explore::Pose2D> path;
  double map_z{0.0};
  double stamp_s{0.0};
};

struct RollingSegmentPublishPathEffect {
  std::uint64_t effect_id{0U};
  std::vector<lingtu::explore::Pose2D> path;
  double map_z{0.0};
  double stamp_s{0.0};
};

struct RollingSegmentAckEffect {
  std::uint64_t effect_id{0U};
  RollingSegmentEffectFailurePolicy failure_policy{RollingSegmentEffectFailurePolicy::kIgnore};
  RollingSegmentAck ack;
};

struct RollingSegmentStatusEffect {
  std::uint64_t effect_id{0U};
  RollingSegmentEffectFailurePolicy failure_policy{RollingSegmentEffectFailurePolicy::kIgnore};
  RollingSegmentStatus status;
};

struct RollingSegmentStopAuthorityEffect {
  std::uint64_t effect_id{0U};
};

struct RollingSegmentClearMotionEffect {
  std::uint64_t effect_id{0U};
  RollingSegmentEffectFailurePolicy failure_policy{RollingSegmentEffectFailurePolicy::kAbortBatch};
  std::string reason;
};

using RollingSegmentEffect =
    std::variant<RollingSegmentActivateAuthorityEffect, RollingSegmentInstallPathEffect,
                 RollingSegmentPublishPathEffect, RollingSegmentAckEffect,
                 RollingSegmentStatusEffect, RollingSegmentStopAuthorityEffect,
                 RollingSegmentClearMotionEffect>;

struct RollingSegmentLifecycleSnapshot {
  bool active{false};
  bool has_execution_grid{false};
  bool input_invalidated_this_tick{false};
  bool terminal_delivery_pending{false};
  std::string input_error{"execution_grid_missing"};
};

struct RollingSegmentStepResult {
  std::vector<RollingSegmentEffect> effects;
};

class RollingSegmentLifecycle {
 public:
  explicit RollingSegmentLifecycle(rolling::SegmentExecutorConfig executor_config = {});

  [[nodiscard]] RollingSegmentStepResult step(const RollingSegmentEvent &event);
  [[nodiscard]] RollingSegmentLifecycleSnapshot snapshot() const;

 private:
  struct ActiveSegment {
    std::string request_id;
    lingtu::explore::ExploreMapIdentity identity;
    std::uint64_t generation{0U};
    std::uint64_t minimum_generation{0U};
  };

  struct TerminalSegment {
    std::string request_id;
    std::string session_id;
    std::uint64_t reset_epoch{0U};
    std::uint64_t minimum_generation{0U};
    std::optional<ActiveSegment> execution;
    ExplorationSegmentState state{ExplorationSegmentState::kFailed};
    std::string reason;
    bool status_delivered{false};
    // The stored outcome is only an intent until stop-authority has run and
    // clear-motion has positively acknowledged the fail-closed barrier.
    bool motion_cleared{false};
  };

  struct TerminalKey {
    std::string request_id;
    std::string session_id;
    std::uint64_t reset_epoch{0U};
    std::uint64_t minimum_generation{0U};
  };

  struct RejectedIngressReceipt {
    std::string request_id;
    std::int32_t kind{0};
    std::string session_id;
    std::uint64_t reset_epoch{0U};
    std::uint64_t minimum_generation{0U};
    RollingSegmentAck ack;
  };

  enum class PendingCriticalEffectKind {
    kActivateAuthority,
    kAcceptedAck,
    kAdmissionStatus,
  };

  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentBeginTick &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentObserveExecutionGrid &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentObserveInvalidInput &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentCommandEvent &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentIngressRejected &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentRevalidate &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentGenericPreempt &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentMotionOutcome &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentShutdown &event);
  [[nodiscard]] RollingSegmentStepResult handle(const RollingSegmentEffectFeedback &event);
  [[nodiscard]] RollingSegmentStepResult handleCommand(const RollingSegmentCommand &command,
                                                       const RollingSegmentRuntimeContext &context,
                                                       const std::string *ingress_rejection_reason);

  [[nodiscard]] TerminalSegment *findTerminal(const RollingSegmentCommand &command);
  [[nodiscard]] TerminalSegment *findTerminal(const TerminalKey &key);
  TerminalSegment &rememberTerminal(TerminalSegment terminal);
  [[nodiscard]] const RejectedIngressReceipt *
  findRejectedIngress(const RollingSegmentCommand &command) const;
  RejectedIngressReceipt &rememberRejectedIngress(const RollingSegmentCommand &command,
                                                  RollingSegmentAck ack);
  [[nodiscard]] RollingSegmentStatusEffect makeTerminalStatusEffect(TerminalSegment &terminal);
  [[nodiscard]] RollingSegmentStepResult makeSafeStopEffects(TerminalSegment &terminal);
  [[nodiscard]] bool motionClearPending() const noexcept;
  [[nodiscard]] RollingSegmentStepResult terminateActive(ExplorationSegmentState state,
                                                         std::string reason);
  [[nodiscard]] std::uint64_t nextEffectId() noexcept;

  rolling::MapInputPolicy map_input_policy_;
  rolling::SegmentExecutor executor_;
  std::optional<rolling::SegmentInput> latest_input_;
  std::string input_error_{"execution_grid_missing"};
  bool input_invalidated_this_tick_{false};
  std::optional<ActiveSegment> active_;
  std::deque<TerminalSegment> terminal_cache_;
  std::deque<RejectedIngressReceipt> rejected_ingress_cache_;
  std::unordered_map<std::uint64_t, TerminalKey> pending_motion_clears_;
  std::unordered_map<std::uint64_t, TerminalKey> pending_terminal_statuses_;
  std::unordered_map<std::uint64_t, PendingCriticalEffectKind> pending_critical_effects_;
  std::uint64_t next_effect_id_{1U};
};

}  // namespace lingtu::nav::endpoint
