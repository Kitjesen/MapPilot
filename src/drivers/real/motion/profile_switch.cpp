#include "profile_switch.hpp"

#include <algorithm>
#include <utility>

namespace lingtu::driver {
using namespace std::chrono_literals;

ProfileSwitch::ProfileSwitch(std::string up, std::string down)
    : up_(std::move(up)), down_(std::move(down)) {}

void ProfileSwitch::request(const ProfileRequest &request, Clock::time_point now) {
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
}

bool ProfileSwitch::holdsMotion() const noexcept {
  return stage_ != Stage::Idle && stage_ != Stage::Ready && stage_ != Stage::Unsupported;
}

void ProfileSwitch::wait(std::string reason, Clock::time_point now) {
  status_.phase = "waiting";
  status_.reason = std::move(reason);
  next_step_ = now + 1s;
}

void ProfileSwitch::step(Body &body, Clock::time_point now, bool zero_command) {
  if (paused_) return;
  if (stage_ == Stage::Idle || stage_ == Stage::Unsupported || now < next_step_) return;
  if (now - last_request_ > 1s) {
    wait("profile_request_missing", now);
    return;
  }
  if (holdsMotion() && !zero_command) {
    if (!stopped_) {
      const auto stopped = body.stop();
      stopped_ = stopped.confirmsStop();
    }
    wait("waiting_for_zero_command", now);
    return;
  }
  next_step_ = now + 100ms;
  const auto observed = body.readProfile();
  if (!observed.supported) {
    stage_ = Stage::Unsupported;
    status_.phase = "unsupported";
    status_.reason = observed.reason;
    return;
  }
  if (!observed.fresh || observed.current.empty()) {
    if (stage_ == Stage::Ready) {
      stage_ = Stage::Inspect;
      stopped_ = false;
    }
    wait(observed.reason.empty() ? "profile_readback_missing" : observed.reason, now);
    return;
  }
  status_.active_profile = observed.current;
  status_.reason.clear();
  if (stage_ == Stage::Inspect) {
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
    stage_ = observed.current == status_.requested_profile ? Stage::Stand : Stage::Stop;
  }
  if (stage_ == Stage::Stop) {
    if (!stopped_) {
      const auto stopped = body.stop();
      if (!stopped.confirmsStop()) {
        wait(stopped.error.empty() ? "profile_stop_unconfirmed" : stopped.error, now);
        return;
      }
      stopped_ = true;
    }
    stage_ = observed.requires_grounded ? Stage::Ground : Stage::Switch;
  }
  if (stage_ == Stage::Ground) {
    status_.phase = "grounding";
    if (observed.grounded) {
      stage_ = Stage::Switch;
      posture_requested_ = false;
    } else {
      if (!posture_requested_ && body.state().posture != Posture::Transitioning) {
        const auto result = body.act(BodyAction::Sit);
        if (!result.ok) {
          wait(result.error.empty() ? result.state.reason : result.error, now);
          return;
        }
        posture_requested_ = true;
      }
      return;
    }
  }
  if (stage_ == Stage::Switch) {
    status_.phase = "switching";
    if (observed.current == status_.requested_profile) {
      stage_ = Stage::Stand;
      posture_requested_ = false;
    } else {
      if (observed.requires_grounded && !observed.grounded) {
        stage_ = Stage::Ground;
        posture_requested_ = false;
        return;
      }
      const auto result = body.switchProfile(status_.requested_profile);
      if (!result.ok || !result.accepted) {
        wait(result.error.empty() ? result.state.reason : result.error, now);
        return;
      }
      stage_ = Stage::Confirm;
      return;  // Confirmation always comes from a subsequent independent read.
    }
  }
  if (stage_ == Stage::Confirm) {
    status_.phase = "switching";
    if (observed.current != status_.requested_profile) {
      stage_ = Stage::Switch;
      wait("profile_readback_mismatch", now);
      return;
    }
    stage_ = Stage::Stand;
    posture_requested_ = false;
  }
  if (stage_ == Stage::Stand) {
    status_.phase = "standing";
    if (observed.current != status_.requested_profile) {
      stage_ = Stage::Inspect;
      stopped_ = false;
      wait("profile_readback_mismatch", now);
      return;
    }
    if (body.state().posture != Posture::Standing) {
      if (!posture_requested_ && body.state().posture != Posture::Transitioning) {
        const auto result = body.act(BodyAction::Stand);
        if (!result.ok) {
          wait(result.error.empty() ? result.state.reason : result.error, now);
          return;
        }
        posture_requested_ = true;
      }
      return;
    }
    const auto ready = body.refresh();
    if (!ready.ok || !ready.state.ready) {
      wait(ready.error.empty() ? ready.state.reason : ready.error, now);
      return;
    }
    stage_ = Stage::Ready;
    if (status_.requirement == "restore") previous_profile_.clear();
  }
  if (stage_ == Stage::Ready) {
    if (observed.current != status_.requested_profile) {
      stage_ = Stage::Inspect;
      stopped_ = false;
      wait("profile_changed", now);
      return;
    }
    status_.phase = "ready";
    next_step_ = now + 500ms;
  }
}

}  // namespace lingtu::driver
