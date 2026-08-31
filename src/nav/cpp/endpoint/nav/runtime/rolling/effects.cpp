#include "runtime/rolling/effects.hpp"

#include <stdexcept>
#include <type_traits>
#include <utility>
#include <variant>

namespace lingtu::nav::endpoint {

RollingSegmentEffectCoordinator::RollingSegmentEffectCoordinator(
    RollingSegmentLifecycle &lifecycle, RollingSegmentEffectActions actions)
    : lifecycle_(lifecycle), actions_(std::move(actions)) {
  validateActions();
}

bool RollingSegmentEffectCoordinator::apply(const RollingSegmentStepResult &step_result) {
  bool motion_effects_ok = true;
  for (const auto &effect : step_result.effects) {
    const std::uint64_t effect_id =
        std::visit([](const auto &value) { return value.effect_id; }, effect);
    bool abort_batch_on_failure = false;
    const bool effect_ok = applyEffect(effect, abort_batch_on_failure);
    const bool motion_effect =
        std::holds_alternative<RollingSegmentActivateAuthorityEffect>(effect) ||
        std::holds_alternative<RollingSegmentClearMotionEffect>(effect);
    if (motion_effect) {
      motion_effects_ok = motion_effects_ok && effect_ok;
    }

    const auto feedback = lifecycle_.step(RollingSegmentEffectFeedback{effect_id, effect_ok});
    if (!feedback.effects.empty()) {
      motion_effects_ok = apply(feedback) && motion_effects_ok;
    }
    if (abort_batch_on_failure) {
      break;
    }
  }
  return motion_effects_ok;
}

bool RollingSegmentEffectCoordinator::applyEffect(const RollingSegmentEffect &effect,
                                                  bool &abort_batch_on_failure) {
  bool effect_ok = true;
  std::visit(
      [&](const auto &value) {
        using Effect = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<Effect, RollingSegmentActivateAuthorityEffect>) {
          effect_ok = actions_.activate_authority();
          abort_batch_on_failure = !effect_ok;
        } else if constexpr (std::is_same_v<Effect, RollingSegmentInstallPathEffect>) {
          actions_.install_path(value);
        } else if constexpr (std::is_same_v<Effect, RollingSegmentPublishPathEffect>) {
          actions_.publish_path(value);
        } else if constexpr (std::is_same_v<Effect, RollingSegmentAckEffect>) {
          effect_ok = actions_.publish_ack(value.ack);
          abort_batch_on_failure =
              !effect_ok && value.failure_policy != RollingSegmentEffectFailurePolicy::kIgnore &&
              value.failure_policy != RollingSegmentEffectFailurePolicy::kRetryTerminalStatus;
        } else if constexpr (std::is_same_v<Effect, RollingSegmentStatusEffect>) {
          effect_ok = actions_.publish_status(value.status);
          abort_batch_on_failure =
              !effect_ok &&
              value.failure_policy == RollingSegmentEffectFailurePolicy::kFailClosedActiveSegment;
        } else if constexpr (std::is_same_v<Effect, RollingSegmentStopAuthorityEffect>) {
          actions_.stop_authority();
        } else if constexpr (std::is_same_v<Effect, RollingSegmentClearMotionEffect>) {
          effect_ok = actions_.clear_motion(value.reason);
          abort_batch_on_failure = !effect_ok;
        }
      },
      effect);
  return effect_ok;
}

void RollingSegmentEffectCoordinator::validateActions() const {
  if (!actions_.activate_authority || !actions_.install_path || !actions_.publish_path ||
      !actions_.publish_ack || !actions_.publish_status || !actions_.stop_authority ||
      !actions_.clear_motion) {
    throw std::invalid_argument("rolling segment effect coordinator actions are incomplete");
  }
}

}  // namespace lingtu::nav::endpoint
