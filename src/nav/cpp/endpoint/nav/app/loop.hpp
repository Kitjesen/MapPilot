#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>

#include "input/gate.hpp"
#include "nav_kernel/types.hpp"
#include "app/goal/plan.hpp"
#include "app/time.hpp"
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

struct CliConfig;
struct PreparedPlanner;
struct CommandSafetyConfig;
class Dds;
struct SensorBatch;
struct DdsStatus;
class GeofenceManager;
struct EndpointState;
class InputProjector;
class GlobalPlanTask;
class GoalPlanController;
class GoalReplanRuntimeCoordinator;
class MotionStopBarrier;
class NavigationGoalStatusOutbox;
class GoalTerminalStatusDelivery;
class InspectionRuntimeController;
class InspectionCommandCoordinator;
class TeleopTickController;
class AutonomyTickController;
class RollingSegmentLifecycle;
class RollingSegmentEffectCoordinator;
class NavStatusPublisher;
class ControlLoopHealth;
class OperatorMotionAuthority;
class CommandIngressController;
class NavigationStateTracker;
class InspectionStatusFileWriter;
class SemanticViewQuery;

/// All external state the endpoint main-loop needs.  Constructed once in
/// main() after the controllers are wired; the loop reads/writes through
/// these references and value members.
struct EndpointLoopContext {
  // Runtime planner selection updates only the local planner configuration.
  CliConfig &cfg;
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
  MotionStopBarrier &motion_stop;
  NavigationGoalStatusOutbox &goal_status_outbox;
  GoalTerminalStatusDelivery &goal_terminal_delivery;
  InspectionRuntimeController &inspection_runtime;
  InspectionCommandCoordinator &inspection_command_coordinator;
  TeleopTickController &teleop_tick;
  AutonomyTickController &autonomy_tick;
  RollingSegmentLifecycle &rolling_segment;
  RollingSegmentEffectCoordinator &rolling_segment_effect_coordinator;
  NavStatusPublisher &nav_status;
  ControlLoopHealth &control_loop_health;
  OperatorMotionAuthority &operator_motion_authority;
  CommandIngressController &command_ingress;
  NavigationStateTracker &navigation_state;

  // -- Callbacks wired in the setup phase -----------------------------------
  std::function<GoalPlanMapIdentityResult()> current_map_identity;
  std::function<void()> sync_goal_plan_diagnostics;

  // -- Timing (current_timing is shared with setup-phase lambdas) -----------
  TimingDiagnostics *&current_timing;
  SemanticViewQuery *semantic_view_query{nullptr};
  std::function<void(const SensorBatch &)> observe_stop_inputs;
  std::function<void(PreparedPlanner &)> apply_planner;
  std::function<void(const std::string &)> hold{};
};

/// Run the main while(running) loop and the post-loop shutdown sequence.
/// Returns the process exit code (0 = clean, 1 = shutdown failure).
int runEndpointLoop(EndpointLoopContext &ctx, const std::atomic_bool &running);

}  // namespace lingtu::nav::endpoint
