#pragma once

#include <optional>
#include <string>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

// Owns the mutually exclusive resumable motion mode of the native endpoint.
class ControlAuthority {
 public:
  bool activatePath();
  bool acceptTeleop(const nav_kernel::Twist &request, double source_stamp_s);
  bool beginOperatorTakeover(const nav_kernel::Twist &request, double source_stamp_s);
  void setTeleopManualMode(bool active) noexcept;
  void holdOperatorTakeover();
  bool resumeMotion();
  void cancel();
  void stop();
  void latchEstop(std::string reason);
  bool clearEstop(bool zero_command_published);

  [[nodiscard]] bool pathActive() const noexcept;
  [[nodiscard]] bool operatorTakeoverLatched() const noexcept;
  [[nodiscard]] bool motionAllowed() const noexcept;
  [[nodiscard]] bool estopLatched() const noexcept;
  [[nodiscard]] const std::string &estopReason() const noexcept;
  [[nodiscard]] const std::optional<nav_kernel::Twist> &teleopRequest() const noexcept;
  [[nodiscard]] double teleopStampSeconds() const noexcept;
  [[nodiscard]] bool teleopManualMode() const noexcept;

 private:
  void clearMotion() noexcept;

  bool path_active_{false};
  bool operator_takeover_latched_{false};
  bool estop_latched_{false};
  std::string estop_reason_{"clear"};
  std::optional<nav_kernel::Twist> teleop_request_;
  double teleop_stamp_s_{0.0};
  bool teleop_manual_mode_{false};
};

}  // namespace lingtu::nav::endpoint
