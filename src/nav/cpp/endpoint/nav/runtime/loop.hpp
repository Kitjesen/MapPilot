#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "input/gate.hpp"
#include "nav_kernel/types.hpp"
#include "runtime/goal/plan.hpp"
#include "runtime/time.hpp"
#include "status/nav_status_writer.hpp"
#include "status/navigation_state.hpp"

namespace lingtu::nav::navigation {
class Executor;
}
namespace lingtu::nav::inspection {
class Executor;
class Store;
class InspectionTaskEventOutbox;
}  // namespace lingtu::nav::inspection

namespace lingtu::nav::endpoint {

struct PostPlanningInputReadinessResult {
  bool allow_publish{true};
  bool stop_required{false};
  bool stop_succeeded{false};
  std::string reason;
};

inline PostPlanningInputReadinessResult enforcePostPlanningInputReadiness(
    const nav_kernel::Twist &command, const InputGateState &input_gate,
    const std::function<bool(const std::string &)> &stop_motion,
    bool manual_mode = false) {
  const bool command_is_zero = command.vx == 0.0 && command.vy == 0.0 && command.wz == 0.0;
  const bool manual_input_bypass = manual_mode && manualModeMayBypassInputGate(input_gate);
  if (manual_input_bypass || command_is_zero || input_gate.ready) {
    return {};
  }

  const std::string reason =
      std::string("input_gate_") + (input_gate.reason.empty() ? "blocked" : input_gate.reason);
  return {false, true, stop_motion(reason), reason};
}

struct CliConfig;
struct InputGateConfig;
struct CommandSafetyConfig;
class Dds;
struct DdsStatus;
class GeofenceManager;
struct EndpointState;
class InputProjector;
class GlobalPlanTask;
class GoalPlanController;
class GoalReplanRuntimeCoordinator;
class ActivePathBlockagePolicy;
class MotionStopBarrier;
class NavigationGoalStatusOutbox;
class GoalTerminalStatusDelivery;
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
class InspectionStatusFileWriter;

/// All external state the endpoint main-loop needs.  Constructed once in
/// main() after the controllers are wired; the loop reads/writes through
/// these references and value members.
struct EndpointLoopContext {
  // -- Config (immutable) ---------------------------------------------------
  const CliConfig &cfg;
  const InputGateConfig &gate_cfg;
  const CommandSafetyConfig &safety_config;
  bool operator_motion_interface_enabled;

  // -- Core objects ---------------------------------------------------------
  Dds &dds;
  DdsStatus &dds_status;
  EndpointState &state;
  lingtu::nav::navigation::Executor &executor;
  GeofenceManager &geofence;
  InspectionStatusFileWriter &inspection_status_writer;
  lingtu::nav::inspection::Store *inspection_checkpoint_store;
  lingtu::nav::inspection::InspectionTaskEventOutbox &inspection_task_event_outbox;

  // -- Controllers ----------------------------------------------------------
  InputProjector &inputs;
  GlobalPlanTask &plan_preview;
  GoalPlanController &goal_plan;
  GoalReplanRuntimeCoordinator &goal_replan_runtime;
  ActivePathBlockagePolicy &active_path_blockage_policy;
  MotionStopBarrier &motion_stop;
  NavigationGoalStatusOutbox &goal_status_outbox;
  GoalTerminalStatusDelivery &goal_terminal_delivery;
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
  std::function<GoalPlanMapIdentityResult()> current_map_identity;
  std::function<void()> sync_goal_plan_diagnostics;
  std::function<bool()> control_loop_guard_latched;

  // -- Timing (current_timing is shared with setup-phase lambdas) -----------
  TimingDiagnostics *&current_timing;
};

/// Run the main while(running) loop and the post-loop shutdown sequence.
/// Returns the process exit code (0 = clean, 1 = shutdown failure).
int runEndpointLoop(EndpointLoopContext &ctx, const std::atomic_bool &running);

}  // namespace lingtu::nav::endpoint
