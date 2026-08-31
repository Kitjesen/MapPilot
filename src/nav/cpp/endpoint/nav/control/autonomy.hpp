#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "control/final.hpp"
#include "input/planner.hpp"
#include "navigation/executor.hpp"
#include "nav_kernel/velocity_smoother.hpp"
#include "runtime/goal/trigger.hpp"
#include "input/gate.hpp"
#include "input/active/identity.hpp"
#include "status/nav_status_writer.hpp"

namespace lingtu::nav::endpoint {

enum class AutonomyTickOutcomeKind {
  kNone,
  kRollingRecoveryExhausted,
  kRollingReached,
  kGoalFailed,
  kGoalReached,
};

struct AutonomyTickOutcome {
  AutonomyTickOutcomeKind kind{AutonomyTickOutcomeKind::kNone};
  std::string reason;
  bool inspection_arrival_intent{false};
  std::optional<GoalReplanTrigger> replan_trigger;
};

struct AutonomyTickActions {
  std::function<double()> steady_now_s;
  std::function<PlanView(double, TimingDiagnostics &)> read_plan;
  std::function<GoalPlanMapIdentityResult()> current_map_identity;
  std::function<lingtu::nav::navigation::ExecutionOutput(const nav_kernel::Pose &, const float *, int,
                                                 double, lingtu::nav::navigation::TraversabilityGridView)>
      tick_autonomy;
  std::function<void()> stop_linear_motion;
};

struct AutonomyTickInput {
  const CommandSafetyConfig &safety;
  const std::optional<nav_kernel::Pose> &map_body;
  const InputGateState &input_gate;
  bool path_active{false};
  std::optional<lingtu::nav::plan::MapIdentity> active_path_map_identity;
  bool motion_allowed{false};
  bool rolling_segment_active{false};
  bool publish_cmd_vel{false};
  const TraversabilityGrid &traversability;
  const LocalDiagnostics &previous_local;
  TimingDiagnostics &timing;
  std::optional<GoalReplanIdentity> active_goal_identity;
  std::optional<GoalReplanTrigger> precomputed_replan_trigger;
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

struct AutonomyTickResult {
  bool handled{false};
  bool clear_local_path{false};
  bool clear_local_planner_debug{false};
  std::optional<lingtu::nav::navigation::ExecutionOutput> output;
  std::optional<LocalDiagnostics> local;
  AutonomyTickPublishIntents publish;
  AutonomyTickCounterDelta delta;
  AutonomyTickTimingDelta timing;
  AutonomyTickOutcome outcome;
};

class AutonomyTickController {
 public:
  AutonomyTickController(AutonomyTickActions actions, FinalControl &final_control);

  AutonomyTickResult tick(const AutonomyTickInput &input);

 private:
  AutonomyTickActions actions_;
  FinalControl &final_control_;
};

}  // namespace lingtu::nav::endpoint
