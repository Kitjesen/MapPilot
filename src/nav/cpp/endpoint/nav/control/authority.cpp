#include "control/authority.hpp"

#include <utility>

namespace lingtu::nav::endpoint {

bool ControlAuthority::activatePath() {
  if (estop_latched_ || held()) {
    return false;
  }
  teleop_request_.reset();
  teleop_manual_mode_ = false;
  path_active_ = true;
  return true;
}

bool ControlAuthority::acceptTeleop(const nav_kernel::Twist &request) {
  if (estop_latched_ || !fault_.empty()) {
    return false;
  }
  path_active_ = false;
  teleop_request_ = request;
  teleop_manual_mode_ = false;
  return true;
}

bool ControlAuthority::takeover(const nav_kernel::Twist &request) {
  if (!acceptTeleop(request)) return false;
  operator_takeover_latched_ = true;
  return true;
}

void ControlAuthority::setTeleopManualMode(bool active) noexcept {
  teleop_manual_mode_ = teleop_request_.has_value() && active;
}

void ControlAuthority::holdOperator() {
  clearMotion();
  operator_takeover_latched_ = true;
}

void ControlAuthority::hold(std::string reason) {
  clearMotion();
  fault_ = std::move(reason);
}

bool ControlAuthority::resume() {
  clearMotion();
  if (estop_latched_) {
    return false;
  }
  operator_takeover_latched_ = false;
  fault_.clear();
  return true;
}

void ControlAuthority::cancel() {
  clearMotion();
  operator_takeover_latched_ = false;
  fault_.clear();
}

void ControlAuthority::stop() {
  cancel();
}

void ControlAuthority::latchEstop(std::string reason) {
  clearMotion();
  operator_takeover_latched_ = false;
  fault_.clear();
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
  fault_.clear();
  return true;
}

bool ControlAuthority::pathActive() const noexcept { return path_active_; }

bool ControlAuthority::operatorHeld() const noexcept {
  return operator_takeover_latched_;
}

bool ControlAuthority::held() const noexcept {
  return operator_takeover_latched_ || !fault_.empty();
}

std::string ControlAuthority::holdReason() const {
  return !fault_.empty() ? fault_ : operator_takeover_latched_ ? "operator_takeover" : "";
}

bool ControlAuthority::motionAllowed() const noexcept { return !estop_latched_; }

bool ControlAuthority::estopLatched() const noexcept { return estop_latched_; }

const std::string &ControlAuthority::estopReason() const noexcept { return estop_reason_; }

const std::optional<nav_kernel::Twist> &ControlAuthority::teleopRequest() const noexcept {
  return teleop_request_;
}

bool ControlAuthority::teleopManualMode() const noexcept { return teleop_manual_mode_; }

void ControlAuthority::clearMotion() noexcept {
  path_active_ = false;
  teleop_request_.reset();
  teleop_manual_mode_ = false;
}

}  // namespace lingtu::nav::endpoint
