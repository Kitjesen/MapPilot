#include "profile_switch.hpp"

#include <algorithm>
#include <utility>

namespace lingtu::driver {
using namespace std::chrono_literals;

ProfileSwitch::ProfileSwitch(std::string up, std::string down)
    : up_(std::move(up)), down_(std::move(down)) {}

void ProfileSwitch::request(const ProfileRequest &request, Clock::time_point now) {
  if (needsCommand() && motion_changed_) return;
  if (request.requirement == "hold") {
    if (request.task_id.empty() || request.task_id != status_.task_id) return;
    paused_ = true;
    last_request_ = now;
    status_.phase = "waiting";
    status_.reason = "profile_paused";
    return;
  }
  if (request.task_id.empty() || (request.requirement != "stairs_up" &&
      request.requirement != "stairs_down" && request.requirement != "restore")) return;
  last_request_ = now;
  if (paused_) next_step_ = now;
  paused_ = false;
  if (status_.task_id == request.task_id && status_.requirement == request.requirement) return;
  status_.task_id = request.task_id;
  status_.requirement = request.requirement;
  status_.phase = "switching";
  status_.reason.clear();
  status_.requested_profile = request.requirement == "restore" ? previous_profile_
      : (request.requirement == "stairs_up" ? up_ : down_);
  stage_ = Stage::Inspect;
  next_step_ = now;
  posture_requested_ = false;
  stopped_ = false;
  motion_changed_ = false;
  initial_motors_enabled_.reset();
}

bool ProfileSwitch::holdsMotion() const noexcept {
  if (stage_ == Stage::Failed && !motion_changed_) return false;
  return stage_ != Stage::Idle && stage_ != Stage::Ready && stage_ != Stage::Unsupported;
}

void ProfileSwitch::wait(std::string reason, Clock::time_point now) {
  status_.phase = "waiting";
  status_.reason = std::move(reason);
  next_step_ = now + 1s;
}

void ProfileSwitch::fail(std::string reason) {
  stage_ = Stage::Failed;
  status_.phase = "failed";
  status_.reason = std::move(reason);
}

void ProfileSwitch::cancel(std::string reason) {
  if (stage_ != Stage::Idle && stage_ != Stage::Unsupported) fail(std::move(reason));
}

void ProfileSwitch::resetBlocked() {
  if (!needsCommand()) return;
  stage_ = Stage::Idle;
  status_.phase = "idle";
  status_.reason = "explicit_body_command";
  paused_ = false;
}

void ProfileSwitch::step(Body &body, Clock::time_point now, bool zero_command) {
  if (paused_) return;
  if (stage_ == Stage::Idle || stage_ == Stage::Unsupported || needsCommand() || now < next_step_) return;
  if (now - last_request_ > 1s) {
    wait("profile_request_missing", now);
    return;
  }
  next_step_ = now + 100ms;
  const auto observed = body.readProfile();
  if (!observed.supported) {
    if (stage_ != Stage::Inspect) {
      fail(observed.reason);
      return;
    }
    stage_ = Stage::Unsupported;
    status_.phase = "unsupported";
    status_.reason = observed.reason;
    return;
  }
  if (!observed.fresh || observed.current.empty()) {
    fail(observed.reason.empty() ? "profile_readback_missing" : observed.reason);
    return;
  }
  if (observed.critical_fault) {
    fail("critical_motor_fault");
    return;
  }
  status_.active_profile = observed.current;
  status_.reason.clear();
  if (stage_ == Stage::Inspect) {
    if (observed.requires_disabled && !observed.motors_enabled.has_value()) {
      fail("motor_output_unknown");
      return;
    }
    initial_motors_enabled_ = observed.motors_enabled;
    if (status_.requirement == "restore" && previous_profile_.empty()) {
      stage_ = Stage::Unsupported;
      status_.phase = "unsupported";
      status_.reason = "previous_profile_unknown";
      return;
    }
    if (status_.requested_profile.empty() ||
        (status_.requested_profile != observed.current &&
         std::find(observed.available.begin(), observed.available.end(),
                   status_.requested_profile) == observed.available.end())) {
      stage_ = Stage::Unsupported;
      status_.phase = "unsupported";
      status_.reason = status_.requested_profile.empty() ? "stair_profile_not_configured"
                                                       : "stair_profile_not_installed";
      return;
    }
    if (previous_profile_.empty() && status_.requirement != "restore") {
      previous_profile_ = observed.current;
    }
    stage_ = observed.current == status_.requested_profile
        ? (observed.requires_disabled && !*initial_motors_enabled_ ? Stage::Enable : Stage::Stand)
        : Stage::Stop;
  }
  if (holdsMotion() && !zero_command) {
    if (!stopped_) {
      motion_changed_ = true;
      const auto stopped = body.stop();
      stopped_ = stopped.confirmsStop();
      if (!stopped_) {
        fail(stopped.error.empty() ? "profile_stop_unconfirmed" : stopped.error);
        return;
      }
    }
    wait("waiting_for_zero_command", now);
    return;
  }
  if (stage_ == Stage::Stop) {
    if (!stopped_ && !(observed.grounded && observed.control_free)) {
      motion_changed_ = true;
      const auto stopped = body.stop();
      if (!stopped.confirmsStop()) {
        fail(stopped.error.empty() ? "profile_stop_unconfirmed" : stopped.error);
        return;
      }
      stopped_ = true;
    }
    stage_ = observed.requires_grounded ? Stage::Ground
        : (observed.requires_disabled ? Stage::Disable : Stage::Switch);
  }
  if (stage_ == Stage::Ground) {
    status_.phase = "grounding";
    if (observed.grounded) {
      stage_ = observed.requires_disabled ? Stage::Disable : Stage::Switch;
      posture_requested_ = false;
    } else {
      if (!posture_requested_ && body.state().posture != Posture::Transitioning) {
        motion_changed_ = true;
        const auto result = body.act(BodyAction::Sit);
        if (!result.ok || !result.transport_ok || !result.accepted) {
          fail(result.error.empty() ? result.state.reason : result.error);
          return;
        }
        posture_requested_ = true;
      }
      return;
    }
  }
  if (stage_ == Stage::Disable) {
    status_.phase = "disabling";
    if (!observed.motors_enabled.has_value()) {
      fail("motor_output_unknown");
      return;
    }
    if (*observed.motors_enabled) {
      motion_changed_ = true;
      const auto result = body.motorPower(false);
      if (!result.ok || !result.transport_ok || !result.accepted) {
        fail(result.error.empty() ? "profile_disable_unconfirmed" : result.error);
        return;
      }
      stage_ = Stage::ConfirmDisabled;
      return;
    }
    stage_ = Stage::ConfirmDisabled;
  }
  if (stage_ == Stage::ConfirmDisabled) {
    if (observed.motors_enabled != false || !observed.control_free) {
      fail("profile_disable_readback_unconfirmed");
      return;
    }
    stage_ = Stage::Switch;
  }
  if (stage_ == Stage::Switch) {
    status_.phase = "switching";
    if (observed.current == status_.requested_profile) {
      stage_ = observed.requires_disabled ? Stage::Enable : Stage::Stand;
      posture_requested_ = false;
    } else {
      if (observed.requires_grounded && !observed.grounded) {
        fail("profile_body_changed");
        return;
      }
      if (observed.requires_disabled && (observed.motors_enabled != false || !observed.control_free)) {
        fail("profile_disable_readback_unconfirmed");
        return;
      }
      motion_changed_ = true;
      const auto result = body.switchProfile(status_.requested_profile);
      if (!result.ok || !result.transport_ok || !result.accepted) {
        fail(result.error.empty() ? result.state.reason : result.error);
        return;
      }
      stage_ = Stage::Confirm;
      return;  // Confirmation always comes from a subsequent independent read.
    }
  }
  if (stage_ == Stage::Confirm) {
    status_.phase = "switching";
    if (observed.current != status_.requested_profile) {
      fail("profile_readback_mismatch");
      return;
    }
    stage_ = observed.requires_disabled ? Stage::Enable : Stage::Stand;
    posture_requested_ = false;
  }
  if (stage_ == Stage::Enable) {
    if (initial_motors_enabled_ != true) {
      wait("motors_initially_disabled", now);
      stage_ = Stage::AwaitEnable;
      return;
    }
    if (observed.current != status_.requested_profile || !observed.control_free ||
        (observed.requires_grounded && !observed.grounded) || observed.motors_enabled != false) {
      fail("profile_enable_precondition_changed");
      return;
    }
    motion_changed_ = true;
    const auto result = body.motorPower(true);
    if (!result.ok || !result.transport_ok || !result.accepted) {
      fail(result.error.empty() ? "profile_enable_unconfirmed" : result.error);
      return;
    }
    status_.phase = "enabling";
    stage_ = Stage::ConfirmEnabled;
    return;
  }
  if (stage_ == Stage::ConfirmEnabled) {
    if (observed.current != status_.requested_profile || observed.motors_enabled != true || !observed.control_free) {
      fail("profile_enable_readback_unconfirmed");
      return;
    }
    stage_ = Stage::Stand;
  }
  if (stage_ == Stage::Stand) {
    status_.phase = "standing";
    if (observed.current != status_.requested_profile) {
      fail("profile_readback_mismatch");
      return;
    }
    if (body.state().posture != Posture::Standing) {
      if (!posture_requested_ && body.state().posture != Posture::Transitioning) {
        motion_changed_ = true;
        const auto result = body.act(BodyAction::Stand);
        if (!result.ok || !result.transport_ok || !result.accepted) {
          fail(result.error.empty() ? result.state.reason : result.error);
          return;
        }
        posture_requested_ = true;
      }
      return;
    }
    const auto ready = body.refresh();
    if (!ready.ok || !ready.state.ready) {
      fail(ready.error.empty() ? ready.state.reason : ready.error);
      return;
    }
    stage_ = Stage::Ready;
    if (status_.requirement == "restore") previous_profile_.clear();
  }
  if (stage_ == Stage::Ready) {
    if (observed.current != status_.requested_profile) {
      fail("profile_changed");
      return;
    }
    status_.phase = "ready";
    next_step_ = now + 500ms;
  }
}

}  // namespace lingtu::driver
