#include "control/authority.hpp"

#include <utility>

namespace lingtu::nav::endpoint {

bool ControlAuthority::activatePath() {
  if (estop_latched_ || operator_takeover_latched_) {
    return false;
  }
  teleop_request_.reset();
  teleop_stamp_s_ = 0.0;
  teleop_manual_mode_ = false;
  path_active_ = true;
  return true;
}

bool ControlAuthority::acceptTeleop(const nav_kernel::Twist &request, double source_stamp_s) {
  if (estop_latched_) {
    return false;
  }
  path_active_ = false;
  teleop_request_ = request;
  teleop_stamp_s_ = source_stamp_s;
  teleop_manual_mode_ = false;
  return true;
}

bool ControlAuthority::beginOperatorTakeover(const nav_kernel::Twist &request,
                                             double source_stamp_s) {
  if (estop_latched_) {
    return false;
  }
  operator_takeover_latched_ = true;
  return acceptTeleop(request, source_stamp_s);
}

void ControlAuthority::setTeleopManualMode(bool active) noexcept {
  teleop_manual_mode_ = teleop_request_.has_value() && active;
}

void ControlAuthority::holdOperatorTakeover() {
  clearMotion();
  operator_takeover_latched_ = true;
}

bool ControlAuthority::resumeMotion() {
  clearMotion();
  if (estop_latched_) {
    return false;
  }
  operator_takeover_latched_ = false;
  return true;
}

void ControlAuthority::cancel() {
  clearMotion();
  operator_takeover_latched_ = false;
}

void ControlAuthority::stop() {
  clearMotion();
  operator_takeover_latched_ = false;
}

void ControlAuthority::latchEstop(std::string reason) {
  clearMotion();
  operator_takeover_latched_ = false;
  estop_latched_ = true;
  estop_reason_ = reason.empty() ? "estop" : std::move(reason);
}

bool ControlAuthority::clearEstop(bool zero_command_published) {
  clearMotion();
  if (!zero_command_published) {
    return false;
  }
  estop_latched_ = false;
  estop_reason_ = "cleared";
  return true;
}

bool ControlAuthority::pathActive() const noexcept { return path_active_; }

bool ControlAuthority::operatorTakeoverLatched() const noexcept {
  return operator_takeover_latched_;
}

bool ControlAuthority::motionAllowed() const noexcept { return !estop_latched_; }

bool ControlAuthority::estopLatched() const noexcept { return estop_latched_; }

const std::string &ControlAuthority::estopReason() const noexcept { return estop_reason_; }

const std::optional<nav_kernel::Twist> &ControlAuthority::teleopRequest() const noexcept {
  return teleop_request_;
}

double ControlAuthority::teleopStampSeconds() const noexcept { return teleop_stamp_s_; }

bool ControlAuthority::teleopManualMode() const noexcept { return teleop_manual_mode_; }

void ControlAuthority::clearMotion() noexcept {
  path_active_ = false;
  teleop_request_.reset();
  teleop_stamp_s_ = 0.0;
  teleop_manual_mode_ = false;
}

}  // namespace lingtu::nav::endpoint
