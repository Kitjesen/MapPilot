#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "control/final.hpp"
#include "input/planner.hpp"
#include "nav_kernel/velocity_smoother.hpp"
#include "runtime/config/config.hpp"
#include "navigation/executor.hpp"
#include "status/nav_status_writer.hpp"

namespace lingtu::nav::endpoint {

struct TeleopTickActions {
  std::function<double()> teleop_receive_age_s;
  std::function<double()> steady_now_s;
  std::function<PlanView(double, TimingDiagnostics &)> read_plan;
  std::function<lingtu::nav::navigation::ExecutionOutput(const nav_kernel::Pose &,
                                                 const nav_kernel::Twist &, const float *, int,
                                                 double, lingtu::nav::navigation::TraversabilityGridView)>
      tick_teleop_intent;
  std::function<void()> pause_linear_motion;
  std::function<void()> replan_motion;
  std::function<void()> stop_linear_motion;
};

struct TeleopTickInput {
  const CliConfig &config;
  const CommandSafetyConfig &safety;
  const std::optional<nav_kernel::Pose> &map_body;
  const InputGateState &input_gate;
  const nav_kernel::Twist *active_request{nullptr};
  bool path_active{false};
  const std::vector<float> &obstacle_xyzh;
  const TraversabilityGrid &traversability;
  double last_traversability_receive_s{0.0};
  const TeleopDiagnostics &previous_teleop;
  TimingDiagnostics &timing;
  bool manual_mode{false};
};

struct TeleopTickCounterDelta {
  std::uint64_t cmd_vel_count{0};
  std::uint64_t teleop_output_count{0};
  std::uint64_t teleop_stop_count{0};
  std::uint64_t teleop_slow_count{0};
  std::uint64_t teleop_limited_count{0};
  std::uint64_t output_count{0};
};

struct TeleopTickTimingDelta {
  bool nav_tick_measured{false};
  double nav_tick_ms{0.0};
  double teleop_gate_ms{0.0};
};

struct TeleopTickPublishIntents {
  bool cmd_vel{false};
  nav_kernel::Twist command{};
  bool local_path{false};
  bool waypoint{false};
};

struct TeleopTickResult {
  bool handled{false};
  TeleopDiagnostics teleop{};
  std::optional<LocalDiagnostics> local;
  std::vector<nav_kernel::Vec3> local_path;
  nav_kernel::LocalPlannerDebugSnapshot local_planner_debug;
  nav_kernel::Vec3 waypoint{};
  TeleopTickPublishIntents publish;
  TeleopTickCounterDelta delta;
  TeleopTickTimingDelta timing;
};

class TeleopTickController {
 public:
  TeleopTickController(TeleopTickActions actions, FinalControl &final_control);

  TeleopTickResult tick(const TeleopTickInput &input);

 private:
  TeleopTickActions actions_;
  FinalControl &final_control_;
};

}  // namespace lingtu::nav::endpoint
