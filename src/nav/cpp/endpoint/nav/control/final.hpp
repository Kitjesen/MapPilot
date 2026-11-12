#pragma once

#include <functional>
#include <optional>
#include <string>

#include "nav_kernel/velocity_smoother.hpp"
#include "safety/command.hpp"

namespace nav_kernel {
struct LocalPlannerParams;
struct LocalPlanRequest;
}

namespace lingtu::nav::navigation {
class Executor;
}

namespace lingtu::nav::endpoint {

struct CliConfig;
struct EndpointState;

struct FinalSafetyInput {
  const CliConfig &config;
  const nav_kernel::LocalPlannerParams &planner;
  const nav_kernel::LocalPlanRequest &collision;
  bool has_pose{false};
  nav_kernel::Twist measured{};
  bool measured_velocity_valid{false};
  bool has_route{false};
  bool operator_takeover{false};
  bool manual_mode{false};
  double active_max_speed_mps{0.0};
};

CommandSafetyDecision evaluateFinalSafety(const FinalSafetyInput &input,
                                        const CommandSafetyConfig &base_safety,
                                        const nav_kernel::Twist &command);

enum class FinalMode {
  kTeleopCommand,
  kTeleopPath,
  kAutonomyPath,
};

struct FinalInput {
  FinalMode mode;
  const CommandSafetyConfig &safety;
  nav_kernel::Twist candidate;
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
      const CommandSafetyConfig &, const nav_kernel::Twist &)>
      command_safety;
  std::function<nav_kernel::VelocitySmootherOutput(const nav_kernel::Twist &, double)>
      shape;
  std::function<bool(const nav_kernel::Twist &, double)> commit;
  std::function<void(double, const std::string &)> stop;
};

class FinalControl {
 public:
  explicit FinalControl(FinalActions actions);
  FinalControl(const CliConfig &config, const nav_kernel::LocalPlannerParams &planner,
               const EndpointState &state, navigation::Executor &executor);

  [[nodiscard]] FinalResult finalize(const FinalInput &input);
  void stop(double now_s, const std::string &reason);

 private:
  FinalActions actions_;
  double applied_yaw_rate_{0.0};
  std::optional<double> applied_yaw_stamp_s_;
};

}  // namespace lingtu::nav::endpoint
