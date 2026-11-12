#pragma once

#include <optional>
#include <string>

#include "nav_kernel/types.hpp"

namespace lingtu::nav::endpoint {

// Owns the mutually exclusive resumable motion mode of the native endpoint.
class ControlAuthority {
 public:
  bool activatePath();
  bool acceptTeleop(const nav_kernel::Twist &request);
  bool takeover(const nav_kernel::Twist &request);
  void setTeleopManualMode(bool active) noexcept;
  void holdOperator();
  void hold(std::string reason);
  bool resume();
  void cancel();
  void stop();
  void latchEstop(std::string reason);
  bool clearEstop(bool zero_command_published);

  [[nodiscard]] bool pathActive() const noexcept;
  [[nodiscard]] bool operatorHeld() const noexcept;
  [[nodiscard]] bool held() const noexcept;
  [[nodiscard]] std::string holdReason() const;
  [[nodiscard]] bool motionAllowed() const noexcept;
  [[nodiscard]] bool estopLatched() const noexcept;
  [[nodiscard]] const std::string &estopReason() const noexcept;
  [[nodiscard]] const std::optional<nav_kernel::Twist> &teleopRequest() const noexcept;
  [[nodiscard]] bool teleopManualMode() const noexcept;

 private:
  void clearMotion() noexcept;

  bool path_active_{false};
  bool operator_takeover_latched_{false};
  std::string fault_;
  bool estop_latched_{false};
  std::string estop_reason_{"clear"};
  std::optional<nav_kernel::Twist> teleop_request_;
  bool teleop_manual_mode_{false};
};

}  // namespace lingtu::nav::endpoint
