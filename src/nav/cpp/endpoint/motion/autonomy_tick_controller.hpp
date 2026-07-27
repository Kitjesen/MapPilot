#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "motion/teleop_safety.hpp"
#include "nav_loop.hpp"
#include "plan/input_gate.hpp"
#include "status/nav_status_writer.hpp"

namespace lingtu::nav::endpoint {

// Borrowed, transport-free view of the planner inputs used during one
// autonomy tick. The endpoint owns the pointed-to obstacle storage.
struct AutonomyTickPlannerInputs {
  bool traversability_fresh{false};
  lingtu::nav::plan::TraversabilityGridView traversability_view{};
  const std::vector<float> *planner_obstacles{nullptr};
};

struct AutonomyTickActions {
  std::function<double()> steady_now_s;
  std::function<AutonomyTickPlannerInputs(TimingDiagnostics &)> compute_planner_inputs;
  std::function<lingtu::nav::plan::NavLoopOutput(const nav_kernel::Pose &, const float *, int,
                                                 double, lingtu::nav::plan::TraversabilityGridView)>
      tick_autonomy;
  std::function<CommandSafetyDecision(const CommandSafetyConfig &, const nav_kernel::Twist &,
                                      const std::optional<nav_kernel::Pose> &,
                                      const std::vector<nav_kernel::Vec3> &,
                                      const std::vector<float> &, const TraversabilityGrid &, bool)>
      evaluate_path_safety;
  std::function<CommandSafetyDecision(const CommandSafetyConfig &, const nav_kernel::Twist &,
                                      double, const std::optional<nav_kernel::Pose> &,
                                      const std::vector<float> &, const TraversabilityGrid &, bool)>
      evaluate_command_safety;
  std::function<void()> stop_linear_motion;
};

struct AutonomyTickInput {
  const CommandSafetyConfig &safety;
  const std::optional<nav_kernel::Pose> &map_body;
  const InputGateState &input_gate;
  bool path_active{false};
  bool motion_allowed{false};
  bool rolling_segment_active{false};
  bool publish_cmd_vel{false};
  const TraversabilityGrid &traversability;
  const LocalDiagnostics &previous_local;
  TimingDiagnostics &timing;
};

struct AutonomyTickCounterDelta {
  std::uint64_t cmd_vel_count{0};
  std::uint64_t output_count{0};
};

struct AutonomyTickTimingDelta {
  bool nav_tick_measured{false};
  double nav_tick_ms{0.0};
};

struct AutonomyTickPublishIntents {
  bool local_path{false};
  bool waypoint{false};
  bool cmd_vel{false};
  nav_kernel::Twist command{};
};

enum class AutonomyTickOutcomeKind {
  kNone,
  kRollingFinalSafetyStopped,
  kRollingRecoveryExhausted,
  kRollingReached,
  kGoalFailed,
  kGoalReached,
};

struct AutonomyTickOutcome {
  AutonomyTickOutcomeKind kind{AutonomyTickOutcomeKind::kNone};
  std::string reason;
  bool inspection_arrival_intent{false};
};

struct AutonomyTickResult {
  bool handled{false};
  bool clear_local_path{false};
  bool clear_local_planner_debug{false};
  std::optional<lingtu::nav::plan::NavLoopOutput> output;
  std::optional<LocalDiagnostics> local;
  AutonomyTickPublishIntents publish;
  AutonomyTickCounterDelta delta;
  AutonomyTickTimingDelta timing;
  AutonomyTickOutcome outcome;
};

class AutonomyTickController {
 public:
  explicit AutonomyTickController(AutonomyTickActions actions);

  AutonomyTickResult tick(const AutonomyTickInput &input);

 private:
  AutonomyTickActions actions_;
};

}  // namespace lingtu::nav::endpoint
