#pragma once

#include <functional>
#include <optional>
#include <string>

#include "nav_kernel/velocity_smoother.hpp"
#include "safety/command.hpp"

namespace lingtu::nav::endpoint {

enum class FinalMode {
  kTeleopCommand,
  kTeleopPath,
  kAutonomyPath,
};

struct FinalInput {
  FinalMode mode;
  const CommandSafetyConfig &safety;
  nav_kernel::Twist candidate;
  double request_age_s{0.0};
  double max_request_age_s{0.0};
  bool limited_before_final{false};
  bool publish{false};
  double now_s{0.0};
  double direct_yaw_accel_rad_s2{0.0};
  double nominal_dt_s{0.01};
};

struct FinalResult {
  CommandSafetyDecision decision;
  bool safety_applied{false};
  bool smoother_stopped{false};
  std::string reason;
};

struct FinalActions {
  std::function<CommandSafetyDecision(
      const CommandSafetyConfig &, const nav_kernel::Twist &, double)>
      command_safety;
  std::function<nav_kernel::VelocitySmootherOutput(const nav_kernel::Twist &, double)>
      shape;
  std::function<bool(const nav_kernel::Twist &, double)> commit;
  std::function<void(double, const std::string &)> stop;
};

class FinalControl {
 public:
  explicit FinalControl(FinalActions actions);

  [[nodiscard]] FinalResult finalize(const FinalInput &input);
  void stop(double now_s, const std::string &reason);

 private:
  FinalActions actions_;
  double applied_yaw_rate_{0.0};
  std::optional<double> applied_yaw_stamp_s_;
};

}  // namespace lingtu::nav::endpoint
