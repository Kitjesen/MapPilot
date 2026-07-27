#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "endpoint_time.hpp"
#include "plan/goal_plan_controller.hpp"
#include "status/nav_status_writer.hpp"
#include "status/navigation_state.hpp"

namespace lingtu::nav::plan {
class NavLoop;
}
namespace lingtu::nav::inspection {
class Executor;
}

namespace lingtu::nav::endpoint {

struct CliConfig;
struct InputGateConfig;
struct ObstacleMergeConfig;
struct CommandSafetyConfig;
class DdsRuntime;
struct EndpointState;
class NavInputStateProjector;
class GoalPlanController;
class MotionStopCoordinator;
class InspectionRuntimeController;
class InspectionCommandCoordinator;
class TeleopAdmissionController;
class TeleopTickController;
class AutonomyTickController;
class RollingSegmentLifecycle;
class RollingSegmentEffectCoordinator;
class NavStatusPublisher;
class ControlLoopHealth;
class ControlLoopRuntimeGuard;
class OperatorMotionAuthority;
class CommandIngressController;
class NavigationStateTracker;
class LiveObstacleLayer;
class InspectionStatusFileWriter;

/// All external state the endpoint main-loop needs.  Constructed once in
/// main() after the controllers are wired; the loop reads/writes through
/// these references and value members.
struct EndpointLoopContext {
  // -- Config (immutable) ---------------------------------------------------
  const CliConfig &cfg;
  const InputGateConfig &gate_cfg;
  const ObstacleMergeConfig &obstacle_merge_config;
  const CommandSafetyConfig &safety_config;
  bool operator_motion_interface_enabled;

  // -- Core objects ---------------------------------------------------------
  DdsRuntime &dds;
  EndpointState &state;
  lingtu::nav::plan::NavLoop &nav;
  LiveObstacleLayer &live_obstacles;
  InspectionStatusFileWriter &inspection_status_writer;

  // -- Controllers ----------------------------------------------------------
  NavInputStateProjector &input_projector;
  GoalPlanController &goal_plan;
  MotionStopCoordinator &motion_stop;
  InspectionRuntimeController &inspection_runtime;
  InspectionCommandCoordinator &inspection_command_coordinator;
  TeleopAdmissionController &teleop_admission;
  TeleopTickController &teleop_tick;
  AutonomyTickController &autonomy_tick;
  RollingSegmentLifecycle &rolling_segment;
  RollingSegmentEffectCoordinator &rolling_segment_effect_coordinator;
  NavStatusPublisher &nav_status;
  ControlLoopHealth &control_loop_health;
  ControlLoopRuntimeGuard &control_loop_guard;
  OperatorMotionAuthority &operator_motion_authority;
  CommandIngressController &command_ingress;
  NavigationStateTracker &navigation_state;

  // -- Callbacks wired in the setup phase -----------------------------------
  std::function<std::optional<std::pair<std::string, std::int64_t>>()> active_map_identity;
  std::function<GoalPlanMapIdentityResult()> current_map_identity;
  std::function<bool(const std::string &)> clear_motion_outputs;
  std::function<void()> sync_goal_plan_diagnostics;
  std::function<bool()> control_loop_guard_latched;

  // -- Timing (current_timing is shared with setup-phase lambdas) -----------
  TimingDiagnostics *&current_timing;

  // -- Mutable loop state (owned here, defaults are fine) -------------------
  std::uint64_t operator_motion_admitted_sequence{0U};
  std::uint64_t operator_motion_final_output_sequence{0U};
  std::optional<NavigationMapIdentity> navigation_map_identity;
  double last_navigation_map_identity_refresh_s{0.0};
};

/// Run the main while(running) loop and the post-loop shutdown sequence.
/// Returns the process exit code (0 = clean, 1 = shutdown failure).
int runEndpointLoop(EndpointLoopContext &ctx, const std::atomic_bool &running);

}  // namespace lingtu::nav::endpoint
