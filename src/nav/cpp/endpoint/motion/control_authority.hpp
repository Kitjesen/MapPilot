#pragma once

#include <optional>
#include <string>
#include <utility>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

// Owns the mutually-exclusive resumable motion state of the native endpoint.
// Stop/cancel/estop all pass through this interface so an old path or teleop
// request can never become active again after a zero command is published.
class ControlAuthority {
 public:
  bool activatePath() {
    if (estop_latched_ || operator_takeover_latched_) {
      return false;
    }
    teleop_request_.reset();
    teleop_stamp_s_ = 0.0;
    path_active_ = true;
    return true;
  }

  bool acceptTeleop(const nav_kernel::Twist &request, double source_stamp_s) {
    if (estop_latched_) {
      return false;
    }
    path_active_ = false;
    teleop_request_ = request;
    teleop_stamp_s_ = source_stamp_s;
    return true;
  }

  bool beginOperatorTakeover(const nav_kernel::Twist &request, double source_stamp_s) {
    if (estop_latched_) {
      return false;
    }
    operator_takeover_latched_ = true;
    return acceptTeleop(request, source_stamp_s);
  }

  void holdOperatorTakeover() {
    clearMotion();
    operator_takeover_latched_ = true;
  }

  bool resumeAutonomy() {
    clearMotion();
    if (estop_latched_) {
      return false;
    }
    operator_takeover_latched_ = false;
    return true;
  }

  void cancel() {
    clearMotion();
    operator_takeover_latched_ = false;
  }

  void stop() {
    clearMotion();
    operator_takeover_latched_ = false;
  }

  void latchEstop(std::string reason) {
    clearMotion();
    operator_takeover_latched_ = false;
    estop_latched_ = true;
    estop_reason_ = reason.empty() ? "estop" : std::move(reason);
  }

  bool clearEstop(bool zero_command_published) {
    clearMotion();
    if (!zero_command_published) {
      return false;
    }
    estop_latched_ = false;
    estop_reason_ = "cleared";
    return true;
  }

  bool pathActive() const { return path_active_; }
  bool operatorTakeoverLatched() const { return operator_takeover_latched_; }
  bool motionAllowed() const { return !estop_latched_; }
  bool estopLatched() const { return estop_latched_; }
  const std::string &estopReason() const { return estop_reason_; }
  const std::optional<nav_kernel::Twist> &teleopRequest() const { return teleop_request_; }
  double teleopStampSeconds() const { return teleop_stamp_s_; }

 private:
  void clearMotion() {
    path_active_ = false;
    teleop_request_.reset();
    teleop_stamp_s_ = 0.0;
  }

  bool path_active_{false};
  bool operator_takeover_latched_{false};
  bool estop_latched_{false};
  std::string estop_reason_{"clear"};
  std::optional<nav_kernel::Twist> teleop_request_;
  double teleop_stamp_s_{0.0};
};

}  // namespace lingtu::nav::endpoint
