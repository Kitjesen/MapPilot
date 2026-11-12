#include "posture.hpp"

#include <algorithm>

#include "message/protocol/navigation.hpp"

namespace lingtu::driver {

PostureReply PostureControl::request(const message::PostureRequest &request, Body &body,
                                    const std::string &zero_producer, bool fresh_zero,
                                    bool profile_busy) {
  using Kind = message::NavigationCommandKind;
  for (const auto &ack : completed_) {
    if (message::samePostureRequest(ack.request, request)) {
      if (ack.request.action != request.action || ack.request.profile_name != request.profile_name ||
          ack.request.external_session_id != request.external_session_id) {
        return {{request, true, false, "request_id_conflict"}, std::nullopt};
      }
      if (request.phase == 2) {
        const auto reason = ack.reason == "posture_action_ack_unknown"
                                ? ack.reason
                                : (ack.accepted ? "posture_already_accepted" : "posture_cancelled");
        return {{request, true, false, reason}, std::nullopt};
      }
      auto replay = ack;
      replay.request.phase = request.phase;
      return {std::move(replay), std::nullopt};
    }
  }
  PostureReply reply{{request, true, false, {}}, std::nullopt};
  reply.new_request = request.phase == 0 &&
      (!preparing_ || !message::samePostureRequest(*preparing_, request));
  bool motion_changed = false;
  auto finish = [&](const std::string &reason, bool accepted = false) {
    reply.ack.accepted = accepted;
    reply.ack.reason = reason;
    if (motion_changed) {
      hold_ = true;
      resume_when_standing_ = accepted && (request.action == static_cast<int>(Kind::Stand) ||
          request.action == static_cast<int>(Kind::EnableMotors));
      accepted_at_ = Clock::now();
    }
    if (preparing_ && message::samePostureRequest(*preparing_, request)) preparing_.reset();
    completed_.push_back(reply.ack);
    if (completed_.size() > 32) completed_.pop_front();
    return reply;
  };
  const bool enable = request.action == static_cast<int>(Kind::EnableMotors);
  const bool disable = request.action == static_cast<int>(Kind::DisableMotors);
  const bool profile = request.action == static_cast<int>(Kind::SwitchProfile);
  const bool enter_external = request.action == static_cast<int>(Kind::EnterExternal);
  const bool enable_external = request.action == static_cast<int>(Kind::EnableExternal);
  const bool exit_external = request.action == static_cast<int>(Kind::ExitExternal);
  if (request.action != static_cast<int>(Kind::Stand) &&
      request.action != static_cast<int>(Kind::Sit) && !enable && !disable && !profile &&
      !enter_external && !enable_external && !exit_external) return finish("unknown_body_action");
  if (request.phase == 2) return finish("posture_cancelled");
  if (request.phase != 0 && request.phase != 1) return finish("invalid_posture_phase");
  if (profile_busy && !disable) return finish("profile_transition_active");
  if (enter_external || enable_external || exit_external) {
    if (!body.capabilities().external_control) return finish("external_control_unsupported");
    if (request.phase != 0) return finish("posture_not_prepared");
    if (enter_external) {
      const auto previous = body.externalState();
      const bool recovering = previous.fresh && previous.selected && !previous.active &&
          !previous.motors_enabled && previous.motor_disable_confirmed;
      if (!recovering) {
        reply.result = body.inspectControl();
        if (!reply.result->ok || !reply.result->transport_ok) return finish(reply.result->state.reason);
        const auto &state = reply.result->state;
        if (state.fsm != "grounded") return finish("external_requires_grounded");
        if (state.motors_enabled) return finish("external_requires_disabled");
        if (state.lease_valid || state.owner != "none") return finish("external_control_busy");
        if (state.critical_fault) return finish("critical_motor_fault");
      }
      motion_changed = true;
      reply.result = body.enterExternal(request.request_id);
    } else {
      if (request.external_session_id.empty() ||
          request.external_session_id != body.externalState().session_id) {
        return finish("external_session_mismatch");
      }
      motion_changed = true;
      reply.result = enable_external ? body.enableExternal() : body.exitExternal();
    }
    if (!reply.result->transport_ok) return finish("posture_action_ack_unknown");
    return finish(reply.result->state.reason, reply.result->ok && reply.result->accepted);
  }
  if (disable && !request.external_session_id.empty() &&
      request.external_session_id != body.externalState().session_id) {
    return finish("external_session_mismatch");
  }
  if (body.externalSelected() && !disable) return finish("external_control_selected");
  if ((enable || disable) && !body.capabilities().motor_power) return finish("motor_power_unsupported");
  if (disable && body.externalSelected()) {
    motion_changed = true;
    reply.result = body.motorPower(false);
    if (!reply.result->transport_ok) return finish("posture_action_ack_unknown");
    if (!reply.result->ok || !reply.result->accepted) return finish(reply.result->state.reason);
    const auto observed = body.externalState();
    if (!observed.fresh || observed.motors_enabled || !observed.motor_disable_confirmed) {
      return finish("posture_action_ack_unknown");
    }
    return finish("motors_disabled", true);
  }
  if (disable) {
    if (request.phase != 0) return finish("posture_not_prepared");
    reply.result = body.inspectControl();
    if (!reply.result->ok || !reply.result->transport_ok) return finish(reply.result->state.reason);
    if (reply.result->state.owner != "none" &&
        reply.result->state.owner_id != kDriverMotionPrincipal) return finish("control_busy");
    const auto prerequisite_calls = reply.result->calls;
    motion_changed = true;
    reply.result = body.motorPower(false);
    reply.result->calls += prerequisite_calls;
    if (!reply.result->transport_ok) return finish("posture_action_ack_unknown");
    if (!reply.result->ok || !reply.result->accepted) return finish(reply.result->state.reason);
    const auto observed = body.inspectControl();
    reply.result->calls += observed.calls;
    if (!observed.ok || !observed.transport_ok || observed.state.motors_enabled || observed.state.lease_valid) {
      return finish("posture_action_ack_unknown");
    }
    reply.result->state = observed.state;
    return finish("motors_disabled", true);
  }
  if (profile) {
    if (request.phase != 0) return finish("posture_not_prepared");
    if (request.profile_name.empty()) return finish("empty_profile");
    reply.result = body.inspectControl();
    if (!reply.result->ok || !reply.result->transport_ok) return finish(reply.result->state.reason);
    const auto &state = reply.result->state;
    if (state.fsm != "grounded") return finish("profile_requires_grounded");
    if (state.motors_enabled) return finish("profile_requires_disabled");
    if (state.lease_valid || state.owner != "none") return finish("profile_control_busy");
    if (state.critical_fault) return finish("critical_motor_fault");
    const auto observed = body.readProfile();
    if (!observed.supported) return finish("profile_unsupported");
    if (!observed.fresh) return finish("profile_readback_missing");
    if (observed.current == request.profile_name) return finish("profile_already_active", true);
    if (std::find(observed.available.begin(), observed.available.end(), request.profile_name) == observed.available.end()) {
      return finish("profile_not_installed");
    }
    const auto prerequisite_calls = reply.result->calls;
    motion_changed = true;
    reply.result = body.switchProfile(request.profile_name);
    reply.result->calls += prerequisite_calls;
    if (!reply.result->transport_ok) return finish("posture_action_ack_unknown");
    if (!reply.result->ok || !reply.result->accepted) return finish(reply.result->state.reason);
    const auto confirmed = body.readProfile();
    if (!confirmed.fresh || confirmed.current != request.profile_name) return finish("posture_action_ack_unknown");
    return finish("profile_switch_confirmed", true);
  }
  const auto action = request.action == static_cast<int>(Kind::Stand) ? BodyAction::Stand
                                                                    : BodyAction::Sit;
  if (!enable && !body.capabilities().supports(action)) return finish("body_action_unsupported");
  bool grounded = false;
  if (request.phase == 0) {
    reply.result = body.inspectControl();
    const auto &inspection = *reply.result;
    if (inspection.state.reason != "control_inspection_unsupported") {
      if (!inspection.ok || !inspection.transport_ok) return finish(inspection.state.reason);
      if (!enable && !inspection.state.motors_enabled) return finish("motors_disabled");
      if (inspection.state.critical_fault) return finish("critical_motor_fault");
      if (inspection.state.owner != "none" &&
          inspection.state.owner_id != kDriverMotionPrincipal) return finish("control_busy");
      if (inspection.state.fsm == "transitioning") return finish("body_transition_active");
      grounded = !inspection.state.lease_valid &&
          (inspection.state.fsm == "grounded" || (enable && !inspection.state.motors_enabled));
    }
    if (!grounded) {
      preparing_ = request;
      reply.ack.final_result = false;
      reply.ack.reason = "stop_required";
      return reply;
    }
  } else {
    if (!preparing_ || !message::samePostureRequest(*preparing_, request) ||
        preparing_->action != request.action || preparing_->profile_name != request.profile_name ||
        preparing_->external_session_id != request.external_session_id) return finish("posture_not_prepared");
    if (!fresh_zero || zero_producer != request.producer_boot_id) {
      return finish("posture_zero_not_confirmed");
    }
    motion_changed = true;
    reply.result = body.stop();
    if (!reply.result->confirmsStop()) return finish("posture_stop_unconfirmed");
  }
  // Grounded has no walking lease to stop. Brainstem checks the physical
  // owner and motor gate again when it accepts the posture action.
  const auto prerequisite_calls = reply.result ? reply.result->calls : 0;
  motion_changed = true;
  reply.result = enable ? body.motorPower(true) : body.act(action);
  reply.result->calls += prerequisite_calls;
  const auto &result = *reply.result;
  if (!result.transport_ok) return finish("posture_action_ack_unknown");
  if (enable && result.ok && result.accepted) {
    const auto observed = body.inspectControl();
    reply.result->calls += observed.calls;
    if (!observed.ok || !observed.transport_ok || !observed.state.motors_enabled) {
      return finish("posture_action_ack_unknown");
    }
    reply.result->state = observed.state;
    return finish("motors_enabled", true);
  }
  return finish(result.ok && result.transport_ok && result.accepted ? "body_action_accepted"
                                                                  : result.state.reason,
                result.ok && result.transport_ok && result.accepted);
}

void PostureControl::observeBody(const BodyState &state) noexcept {
  if (resume_when_standing_ && state.fresh && state.observed_at > accepted_at_ &&
      state.posture == Posture::Standing) {
    hold_ = false;
    resume_when_standing_ = false;
  }
}

}  // namespace lingtu::driver
