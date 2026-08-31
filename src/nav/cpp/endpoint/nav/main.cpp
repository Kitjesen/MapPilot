#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "command/ingress.hpp"
#include "runtime/config/config.hpp"
#include "control/admission.hpp"
#include "control/authority.hpp"
#include "control/autonomy.hpp"
#include "control/final.hpp"
#include "control/guard.hpp"
#include "control/operator.hpp"
#include "control/teleop.hpp"
#include "dds/codec.hpp"
#include "dds/runtime.hpp"
#include "input/gate.hpp"
#include "input/planner.hpp"
#include "input/projector.hpp"
#include "runtime/inspection/inspection_command_coordinator.hpp"
#include "runtime/inspection/inspection_runtime_controller.hpp"
#include "messages.h"
#include "nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav/inspection/store.hpp"
#include "navigation/executor.hpp"
#include "input/obstacle.hpp"
#include "runtime/goal/blockage.hpp"
#include "runtime/goal/plan.hpp"
#include "runtime/goal/runtime.hpp"
#include "input/active/occupancy.hpp"
#include "input/active/octomap.hpp"
#include "runtime/rolling/effects.hpp"
#include "runtime/rolling/lifecycle.hpp"
#include "runtime/loop.hpp"
#include "runtime/state.hpp"
#include "runtime/time.hpp"
#include "safety/command.hpp"
#include "safety/geofence.hpp"
#include "safety/stop.hpp"
#include "status/control_loop_health.hpp"
#include "status/goal_terminal_status_delivery.hpp"
#include "status/inspection_status_file_writer.hpp"
#include "status/nav_status_endpoint_adapter.hpp"
#include "status/nav_status_publisher.hpp"
#include "status/navigation_goal_status_outbox.hpp"
#include "status/navigation_state.hpp"
#include "traversability/transform_buffer.hpp"

namespace {

std::atomic_bool g_running{true};
// The local risk topic is volatile with a 500 ms DDS lifespan. Its in-process
// cache must never outlive that contract, even if the map-risk gate allows a
// longer age.
constexpr double kLocalTraversabilityMaxAgeS = 0.5;
using lingtu::nav::endpoint::ActiveOccupancyGate;
using lingtu::nav::endpoint::ActiveOctomapGate;
using lingtu::nav::endpoint::ActivePathBlockagePolicy;
using lingtu::nav::endpoint::ActivePathBlockagePolicyConfig;
using lingtu::nav::endpoint::AutonomyTickActions;
using lingtu::nav::endpoint::AutonomyTickController;
using lingtu::nav::endpoint::CliConfig;
using lingtu::nav::endpoint::CommandIngressController;
using lingtu::nav::endpoint::commandSafetyConfig;
using lingtu::nav::endpoint::ControlLoopHealth;
using lingtu::nav::endpoint::ControlLoopHealthConfig;
using lingtu::nav::endpoint::ControlLoopRuntimeGuard;
using lingtu::nav::endpoint::ControlLoopRuntimeGuardConfig;
using lingtu::nav::endpoint::ControlLoopRuntimeGuardState;
using lingtu::nav::endpoint::ControlMode;
using lingtu::nav::endpoint::Dds;
using lingtu::nav::endpoint::DdsStatus;
using lingtu::nav::endpoint::EstopLatchStore;
using lingtu::nav::endpoint::evaluateCommandSafety;
using lingtu::nav::endpoint::FinalActions;
using lingtu::nav::endpoint::FinalControl;
using lingtu::nav::endpoint::GeofenceManager;
using lingtu::nav::endpoint::GlobalPlannerBackend;
using lingtu::nav::endpoint::globalPlannerBackendName;
using lingtu::nav::endpoint::GlobalPlanTask;
using lingtu::nav::endpoint::GoalPlanActions;
using lingtu::nav::endpoint::GoalPlanController;
using lingtu::nav::endpoint::GoalPlanInspectionDecision;
using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
using lingtu::nav::endpoint::GoalPlanPathActivation;
using lingtu::nav::endpoint::GoalPlanPathTolerance;
using lingtu::nav::endpoint::GoalPlanStatus;
using lingtu::nav::endpoint::GoalReplanRuntimeCoordinator;
using lingtu::nav::endpoint::GoalReplanRuntimeInterruption;
using lingtu::nav::endpoint::GoalTerminalStatusDelivery;
using lingtu::nav::endpoint::headerFrameId;
using lingtu::nav::endpoint::headerStampSeconds;
using lingtu::nav::endpoint::InputActions;
using lingtu::nav::endpoint::InputConfig;
using lingtu::nav::endpoint::InputGate;
using lingtu::nav::endpoint::InputGateConfig;
using lingtu::nav::endpoint::inputGateConfig;
using lingtu::nav::endpoint::InputProjector;
using lingtu::nav::endpoint::InspectionActiveMap;
using lingtu::nav::endpoint::InspectionCommandAck;
using lingtu::nav::endpoint::InspectionCommandActions;
using lingtu::nav::endpoint::InspectionCommandCommit;
using lingtu::nav::endpoint::InspectionCommandCoordinator;
using lingtu::nav::endpoint::InspectionRuntimeController;
using lingtu::nav::endpoint::InspectionStatusFileWriter;
using lingtu::nav::endpoint::InspectionStopBarrierResult;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::makePlanView;
using lingtu::nav::endpoint::MotionLayer;
using lingtu::nav::endpoint::MotionLayerConfig;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopBarrier;
using lingtu::nav::endpoint::NavigationControlState;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;
using lingtu::nav::endpoint::NavigationStateTracker;
using lingtu::nav::endpoint::NavStatusPublisher;
using lingtu::nav::endpoint::NavStatusPublisherActions;
using lingtu::nav::endpoint::OdometrySpeedEvidence;
using lingtu::nav::endpoint::OdometrySpeedMonitor;
using lingtu::nav::endpoint::OperatorMotionAuthority;
using lingtu::nav::endpoint::parseArgs;
using lingtu::nav::endpoint::PlanConfig;
using lingtu::nav::endpoint::PlanData;
using lingtu::nav::endpoint::PlanDiagnostics;
using lingtu::nav::endpoint::runWithActiveOccupancy;
using lingtu::nav::endpoint::runWithActiveOctomap;
using lingtu::nav::endpoint::StatusMotionLayerSample;
using lingtu::nav::endpoint::StatusPlannerSample;
using lingtu::nav::endpoint::StatusWriterConfig;
using lingtu::nav::endpoint::StopConfirmation;
using lingtu::nav::endpoint::StopConfirmationConfig;
using lingtu::nav::endpoint::StopConfirmationEvidencePolicy;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::TeleopAdmissionActions;
using lingtu::nav::endpoint::TeleopAdmissionController;
using lingtu::nav::endpoint::TeleopTickActions;
using lingtu::nav::endpoint::TeleopTickController;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TransformBuffer;
using CommandKind = lingtu::message::NavigationCommandKind;
using ExplorationSegmentAck = lingtu::nav::endpoint::ExplorationSegmentAck;
using ExplorationSegmentStatus = lingtu::nav::endpoint::ExplorationSegmentStatus;
using RollingSegmentAck = lingtu::nav::endpoint::RollingSegmentAck;
using RollingSegmentBeginTick = lingtu::nav::endpoint::RollingSegmentBeginTick;
using RollingSegmentCommand = lingtu::nav::endpoint::RollingSegmentCommand;
using RollingSegmentCommandEvent = lingtu::nav::endpoint::RollingSegmentCommandEvent;
using RollingSegmentEffectActions = lingtu::nav::endpoint::RollingSegmentEffectActions;
using RollingSegmentEffectCoordinator = lingtu::nav::endpoint::RollingSegmentEffectCoordinator;
using RollingSegmentExecutionGrid = lingtu::nav::endpoint::RollingSegmentExecutionGrid;
using RollingSegmentGenericPreempt = lingtu::nav::endpoint::RollingSegmentGenericPreempt;
using RollingSegmentInstallPathEffect = lingtu::nav::endpoint::RollingSegmentInstallPathEffect;
using RollingSegmentLifecycle = lingtu::nav::endpoint::RollingSegmentLifecycle;
using RollingSegmentMotionOutcome = lingtu::nav::endpoint::RollingSegmentMotionOutcome;
using RollingSegmentMotionOutcomeKind = lingtu::nav::endpoint::RollingSegmentMotionOutcomeKind;
using RollingSegmentObserveExecutionGrid =
    lingtu::nav::endpoint::RollingSegmentObserveExecutionGrid;
using RollingSegmentObserveInvalidInput = lingtu::nav::endpoint::RollingSegmentObserveInvalidInput;
using RollingSegmentPublishPathEffect = lingtu::nav::endpoint::RollingSegmentPublishPathEffect;
using RollingSegmentRevalidate = lingtu::nav::endpoint::RollingSegmentRevalidate;
using RollingSegmentRuntimeContext = lingtu::nav::endpoint::RollingSegmentRuntimeContext;
using RollingSegmentShutdown = lingtu::nav::endpoint::RollingSegmentShutdown;
using RollingSegmentStatus = lingtu::nav::endpoint::RollingSegmentStatus;
using GoalState = lingtu::message::NavigationGoalState;
using OperatorMotionAction = lingtu::message::OperatorMotionAction;

void stopSignal(int) {
  g_running = false;
}

NavigationControlState navigationControlState(ControlMode mode) {
  switch (mode) {
    case ControlMode::Autonomy:
      return NavigationControlState::kAutonomy;
    case ControlMode::Teleop:
      return NavigationControlState::kTeleop;
    case ControlMode::TeleopAvoid:
      return NavigationControlState::kTeleopAvoid;
  }
  return NavigationControlState::kUnknown;
}

std::uint64_t headerStampNanoseconds(const lingtu_dds_Header &header) {
  if (header.stamp.sec < 0 || header.stamp.nanosec >= 1000000000U) {
    return 0U;
  }
  return static_cast<std::uint64_t>(header.stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(header.stamp.nanosec);
}

using lingtu::nav::endpoint::elapsedMs;
using lingtu::nav::endpoint::EndpointLoopContext;
using lingtu::nav::endpoint::EndpointState;
using lingtu::nav::endpoint::kLayerInflationM;
using lingtu::nav::endpoint::nowSeconds;
using lingtu::nav::endpoint::runEndpointLoop;
using lingtu::nav::endpoint::SteadyClock;
using lingtu::nav::endpoint::steadySeconds;
using lingtu::nav::endpoint::stringValue;

std::vector<nav_kernel::Vec3>
rollingSegmentPathToNav(const std::vector<lingtu::explore::Pose2D> &path, double map_z) {
  std::vector<nav_kernel::Vec3> result;
  result.reserve(path.size());
  for (const auto &waypoint : path) {
    result.push_back({waypoint.x, waypoint.y, map_z});
  }
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    const CliConfig cfg = parseArgs(argc, argv);
    std::fprintf(stderr, "navd startup: config_parsed\n");
    std::fflush(stderr);
    auto active_octomap_gate = std::make_shared<ActiveOctomapGate>(cfg.map_identity);
    auto active_occupancy_gate = std::make_shared<ActiveOccupancyGate>(cfg.map_identity);
    std::unique_ptr<lingtu::nav::inspection::Store> inspection_store;
    if (!cfg.inspection_dir.empty()) {
      inspection_store = std::make_unique<lingtu::nav::inspection::Store>(cfg.inspection_dir);
    }
    std::fprintf(stderr, "navd startup: creating_dds_runtime\n");
    std::fflush(stderr);
    DdsStatus dds_status;
    Dds dds(cfg.domain_id, &dds_status);
    std::fprintf(stderr, "navd startup: dds_runtime_ready\n");
    std::fflush(stderr);
    if (cfg.control_mode == ControlMode::Autonomy && !cfg.map_path.empty()) {
      if (cfg.global_planner == GlobalPlannerBackend::Far) {
        auto preflight = active_occupancy_gate->prepare(cfg.map_path);
        if (!preflight.ok()) {
          std::fprintf(stderr, "navd startup: FAR input not ready: %s\n", preflight.reason.c_str());
        }
      } else {
        auto preflight = active_octomap_gate->prepare(cfg.map_path);
        if (!preflight.ok()) {
          throw std::runtime_error("active OctoMap failed native Maps preflight: " +
                                   preflight.reason);
        }
      }
    }
    MotionLayerConfig motion_config;
    motion_config.voxel_size_m = cfg.obstacle_voxel_size_m;
    motion_config.decay_s = cfg.live_obstacle_decay_s;
    motion_config.inflation_radius_m = kLayerInflationM;
    motion_config.ray_clear_max_range_m = cfg.live_obstacle_ray_clear_max_range_m;
    motion_config.ray_clearing_interval_s = cfg.live_obstacle_ray_clearing_interval_s;
    motion_config.max_clearing_rays = cfg.live_obstacle_max_clearing_rays;
    motion_config.min_hits = cfg.live_obstacle_min_hits;
    motion_config.dynamic_min_cells = cfg.dynamic_min_cells;
    motion_config.dynamic_min_speed_mps = cfg.dynamic_min_speed_mps;
    motion_config.dynamic_confirm_frames =
        static_cast<std::uint32_t>(cfg.dynamic_confirm_frames);
    motion_config.ray_clearing_enabled = cfg.live_obstacle_ray_clearing;
    MotionLayer live_obstacles(motion_config);
    const InputGateConfig gate_cfg = inputGateConfig(cfg);
    StatusWriterConfig status_cfg = buildStatusWriterConfig(cfg, gate_cfg);
    InspectionStatusFileWriter inspection_status_writer(cfg.inspection_dir);
    InputGate input_gate(gate_cfg);
    TransformBuffer pose_buffer;
    TransformBuffer map_odom_buffer;
    const double source_transform_max_gap_s =
        cfg.tf_max_age_s > 0.0 ? std::min(cfg.cloud_pose_max_gap_s, cfg.tf_max_age_s)
                               : cfg.cloud_pose_max_gap_s;
    GeofenceManager geofence(cfg.geofence_file);
    NavigationStateTracker navigation_state(navigationControlState(cfg.control_mode));

    const auto safety_config = commandSafetyConfig(cfg);
    lingtu::nav::navigation::ExecutorConfig executor_config =
        buildExecutorConfig(cfg);
    const nav_kernel::LocalPlannerParams local_planner_params =
        buildLocalPlannerParams(cfg);

    nav_kernel::local::Planner local_planner(local_planner_params);
    if ((cfg.control_mode == ControlMode::Autonomy || cfg.teleop_local_planner) &&
        !local_planner.configure(cfg.path_library_dir)) {
      throw std::runtime_error(
          "failed to configure local planner backend: " +
          std::string(nav_kernel::localPlannerBackendName(cfg.local_planner_backend)));
    }
    lingtu::nav::navigation::Executor executor(executor_config,
                                                std::move(local_planner));
    auto planner_lock = std::make_shared<std::mutex>();
    auto octomap_planner = std::make_shared<octoplanner3d::runtime::PlannerSession>();
    auto far_planner =
        std::make_shared<lingtu::nav::plan::far_planner::FarPlanner>(cfg.far_options);
    auto global_planner = [active_octomap_gate, active_occupancy_gate, octomap_planner, far_planner,
                           planner_lock, planner_backend = cfg.global_planner,
                           configured_map_path = cfg.map_path](const auto &request,
                                                               const auto &cancel_check) {
      std::lock_guard<std::mutex> lock(*planner_lock);
      if (planner_backend == GlobalPlannerBackend::Far) {
        return runWithActiveOccupancy(*active_occupancy_gate, configured_map_path, *far_planner,
                                      request, cancel_check);
      }
      return runWithActiveOctomap(*active_octomap_gate, configured_map_path, request, cancel_check,
                                  [octomap_planner](const auto &map_path, const auto &map_identity,
                                                    const auto &plan_request, const auto &cancel) {
                                    return octomap_planner->run(map_path, map_identity,
                                                                plan_request, cancel);
                                  });
    };
    GlobalPlanTask plan_preview(global_planner);

    EndpointState state;
    std::optional<lingtu::nav::inspection::TaskEvent> inspection_recovery_event;
    if (inspection_store) {
      const auto checkpoint = inspection_store->LoadTaskEventCheckpoint();
      if (checkpoint.state == lingtu::nav::inspection::TaskEventCheckpointLoadState::kCorrupt ||
          checkpoint.state == lingtu::nav::inspection::TaskEventCheckpointLoadState::kIoError) {
        throw std::runtime_error("inspection task checkpoint load failed: " + checkpoint.reason);
      }
      if (checkpoint.loaded()) {
        const auto recovery = lingtu::nav::inspection::ReconcileTaskEventAfterRestart(
            *checkpoint.event, nowSeconds());
        if (!recovery.ok) {
          throw std::runtime_error("inspection task checkpoint recovery failed: " +
                                   recovery.reason);
        }
        if (recovery.synthesized_failure) {
          const auto persisted = inspection_store->PutTaskEventCheckpoint(recovery.event);
          if (!persisted.ok) {
            throw std::runtime_error("inspection restart terminal checkpoint failed: " +
                                     persisted.reason);
          }
        }
        if (recovery.event.sequence == std::numeric_limits<std::uint64_t>::max() ||
            !state.inspection_executor.SetNextTaskEventSequence(recovery.event.sequence + 1U)) {
          throw std::runtime_error("inspection task event sequence cannot advance after recovery");
        }
        inspection_recovery_event = recovery.event;
      }
    }
    lingtu::nav::inspection::InspectionTaskEventOutbox inspection_task_event_outbox(
        dds_status.producer_boot_id,
        [&dds](const lingtu::nav::inspection::TaskEventEnvelope &event) {
          const lingtu::nav::endpoint::InspectionTaskEventEnvelope dds_event{
              event.boot_id, event.sequence, event.event};
          return dds.publish(lingtu::nav::endpoint::OutputEvent{dds_event}).published;
        });
    if (inspection_recovery_event) {
      if (!inspection_task_event_outbox.InitializeNextSequence(
              inspection_recovery_event->sequence) ||
          inspection_task_event_outbox.Record(*inspection_recovery_event) !=
              lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted) {
        throw std::runtime_error("inspection recovery event outbox initialization failed");
      }
    }

    // -- Setup aliases shared by the controller wiring below ------------------
    auto &map_body = state.map_body;
    auto &latest_dynamic_clusters = state.latest_dynamic_clusters;
    auto &teleop_receive_time = state.teleop_receive_time;
    auto &teleop_received = state.teleop_received;
    auto &last_plan = state.last_plan;
    auto &last_local = state.last_local;
    auto &last_teleop = state.last_teleop;
    auto &frames = state.frames;
    auto &last_global_path = state.last_global_path;
    auto &last_local_path = state.last_local_path;
    auto &last_local_planner_debug = state.last_local_planner_debug;
    last_local_planner_debug.backend = cfg.local_planner_backend;
    auto &control_authority = state.control_authority;
    auto &inspection_executor = state.inspection_executor;
    auto &operator_resume_required = state.operator_resume_required;
    auto &odom_generation = state.odom_generation;
    auto &path_count = state.path_count;
    auto &cmd_vel_count = state.cmd_vel_count;
    auto &autonomy_request_not_before_s = state.autonomy_request_not_before_s;
    auto &teleop_request_not_before_s = state.teleop_request_not_before_s;

    std::vector<float> planner_obstacles;
    const PlanConfig plan_config{
        cfg.check_obstacle,        cfg.use_traversability_cost, cfg.traversability_max_age_s,
        cfg.terrain_map_max_age_s, cfg.max_obstacle_points,
        local_planner_params.useTerrainAnalysis,
    };
    PlanData plan_data{
        state.traversability_grid, state.last_traversability_receive_s,
        state.obstacle_xyzh,       state.predicted_obstacle_xyzh,
        state.terrain_xyzh,        state.last_terrain_map_receive_s,
        state.terrain_ext_xyzh,    state.last_terrain_ext_receive_s,
        planner_obstacles,
    };
    auto read_plan = [&](double now_s, TimingDiagnostics &timing) {
      const bool collision_authoritative =
          cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Scan &&
          state.local_collision_map.view().present();
      return makePlanView(plan_config, plan_data, now_s, timing, collision_authoritative);
    };

    EstopLatchStore estop_latch_store(cfg.estop_latch_file);
    if (const auto persisted_estop = estop_latch_store.load()) {
      control_authority.latchEstop(*persisted_estop);
      std::fprintf(stderr, "nav_native: restored persisted software estop: %s\n",
                   persisted_estop->c_str());
    }
    CommandIngressController command_ingress;
    OperatorMotionAuthority operator_motion_authority;
    const bool operator_motion_interface_enabled =
        cfg.control_mode == ControlMode::Teleop || cfg.control_mode == ControlMode::TeleopAvoid ||
        (cfg.control_mode == ControlMode::Autonomy && cfg.allow_teleop_takeover);
    RollingSegmentLifecycle rolling_segment(rollingSegmentExecutorConfig(cfg));
    InspectionRuntimeController inspection_runtime(inspection_executor);

    ControlLoopHealthConfig control_loop_health_config;
    control_loop_health_config.period_ms = 1000.0 / std::max(1.0, cfg.tick_hz);
    control_loop_health_config.deadline_miss_ratio_limit =
        cfg.control_loop_deadline_miss_ratio_limit;
    control_loop_health_config.p95_utilization_limit = cfg.control_loop_p95_utilization_limit;
    ControlLoopHealth control_loop_health(control_loop_health_config);
    ControlLoopRuntimeGuardConfig control_loop_guard_config;
    control_loop_guard_config.recovery_confirmation_samples =
        static_cast<std::size_t>(std::max(1.0, std::ceil(cfg.tick_hz)));
    control_loop_guard_config.latch_miss_streak =
        static_cast<std::size_t>(std::max(3.0, std::ceil(cfg.tick_hz)));
    control_loop_guard_config.auto_resume = cfg.control_mode != ControlMode::Autonomy;
    ControlLoopRuntimeGuard control_loop_guard(control_loop_guard_config);
    auto control_loop_guard_latched = [&]() {
      const auto state = control_loop_guard.snapshot().state;
      return state == ControlLoopRuntimeGuardState::kLatched ||
             state == ControlLoopRuntimeGuardState::kRecovered;
    };
    NavStatusPublisherActions nav_status_actions;
    nav_status_actions.sample_planner = [&](double now_s, TimingDiagnostics &status_timing) {
      const auto inputs = read_plan(now_s, status_timing);
      const std::size_t obstacle_points = inputs.obstacles->size() / 4;
      return StatusPlannerSample{
          inputs.traversability_fresh,
          inputs.terrain_fresh,
          inputs.terrain_ext_fresh,
          obstacle_points,
          inputs.obstacles,
      };
    };
    nav_status_actions.sample_motion_layer = [&](double wall_now_s) {
      latest_dynamic_clusters = live_obstacles.dynamicClusters(32, wall_now_s);
      return StatusMotionLayerSample{
          live_obstacles.size(),
          live_obstacles.stats(),
          &latest_dynamic_clusters,
      };
    };
    nav_status_actions.sample_loop_health = [&]() { return control_loop_health.snapshot(); };
    nav_status_actions.sample_far_input = [&, active_occupancy_gate]() {
      lingtu::nav::endpoint::StatusFarInputSample result;
      result.required = cfg.global_planner == GlobalPlannerBackend::Far;
      if (!result.required) {
        result.ready = true;
        result.reason = "not_required";
        return result;
      }
      const auto prepared = active_occupancy_gate->prepare(cfg.map_path);
      result.ready = prepared.ok();
      result.reason = prepared.ok() ? "ready" : prepared.reason;
      if (prepared.ok()) {
        result.map_id = prepared.artifact->identity().map_id;
        result.content_epoch = prepared.artifact->identity().content_epoch;
      }
      return result;
    };
    NavStatusPublisher nav_status(std::move(status_cfg), cfg.status_s,
                                  std::move(nav_status_actions));

    auto teleop_receive_age_s = [&]() -> double {
      if (!teleop_received) {
        return std::numeric_limits<double>::infinity();
      }
      return std::chrono::duration<double>(SteadyClock::now() - teleop_receive_time).count();
    };

    std::fprintf(stderr, "navd: domain=%d tick_hz=%.1f path_library=%s\n", cfg.domain_id,
                 cfg.tick_hz, cfg.path_library_dir.c_str());
    std::fprintf(stderr, "nav_native: velocity_smoother enabled=%d mode=%s\n",
                 cfg.velocity_smoother_enabled ? 1 : 0,
                 cfg.velocity_smoother.feedback_mode ==
                         nav_kernel::VelocityFeedbackMode::kClosedLoop
                     ? "closed"
                     : "open");
    std::fprintf(stderr,
                 "nav_native: publish_cmd_vel=%d check_obstacle=%d use_traversability_cost=%d "
                 "traversability_max_age_s=%.2f terrain_map_max_age_s=%.2f status_file=%s\n",
                 cfg.publish_cmd_vel ? 1 : 0, cfg.check_obstacle ? 1 : 0,
                 cfg.use_traversability_cost ? 1 : 0, cfg.traversability_max_age_s,
                 cfg.terrain_map_max_age_s,
                 cfg.status_file.empty() ? "(disabled)" : cfg.status_file.c_str());
    std::fprintf(stderr,
                 "nav_native: obstacle_voxel_size_m=%.3f live_obstacle_decay_s=%.2f "
                 "live_inflation=%.2f ray_clearing=%d ray_range=%.1f ray_interval=%.2f "
                  "max_rays=%zu min_hits=%d planner_obstacles=%s max_points=%zu\n",
                 cfg.obstacle_voxel_size_m, cfg.live_obstacle_decay_s,
                 cfg.live_obstacle_inflation_radius_m, cfg.live_obstacle_ray_clearing ? 1 : 0,
                  cfg.live_obstacle_ray_clear_max_range_m, cfg.live_obstacle_ray_clearing_interval_s,
                  cfg.live_obstacle_max_clearing_rays, cfg.live_obstacle_min_hits,
                  local_planner_params.useTerrainAnalysis
                      ? "fresh_terrain_map_else_registered_scan"
                      : "registered_scan",
                  cfg.max_obstacle_points);
    std::fprintf(stderr,
                 "nav_native: lidar_sensor_offset_m=(%.3f, %.3f, %.3f) "
                 "ray_origin=body_pose_plus_configured_extrinsics\n",
                 cfg.sensor_offset_x_m, cfg.sensor_offset_y_m, cfg.sensor_offset_z_m);
    if (cfg.map_path.empty()) {
      std::fprintf(stderr,
                   "nav_native: no planner map configured for %s; "
                   "goal_pose will be rejected instead of direct fallback\n",
                   globalPlannerBackendName(cfg.global_planner));
    } else {
      std::fprintf(stderr, "nav_native: global_planner=%s planner_map=%s\n",
                   globalPlannerBackendName(cfg.global_planner), cfg.map_path.c_str());
    }
    std::fprintf(stderr, "nav_native: local_planner=%s\n",
                 nav_kernel::localPlannerBackendName(cfg.local_planner_backend));

    TimingDiagnostics *current_timing = nullptr;
    std::uint64_t last_zero_output_sequence = 0U;
    std::uint64_t last_zero_source_wall_ns = 0U;
    std::optional<nav_kernel::VelocitySmoother> velocity_smoother;
    if (cfg.velocity_smoother_enabled) {
      velocity_smoother.emplace(cfg.velocity_smoother);
    }
    auto shape_velocity = [&](const nav_kernel::Twist &target, double timestamp_s) {
      if (!velocity_smoother) {
        nav_kernel::VelocitySmootherOutput output;
        output.command = target;
        output.valid = true;
        output.reason = "disabled";
        return output;
      }
      std::string error;
      if (!velocity_smoother->SetTarget(target, timestamp_s, &error)) {
        nav_kernel::VelocitySmootherOutput output;
        output.reason = error.empty() ? "velocity_smoother_set_target_failed" : error;
        return output;
      }
      return velocity_smoother->Step(timestamp_s);
    };
    auto commit_applied_velocity = [&](const nav_kernel::Twist &applied, double timestamp_s) {
      if (!velocity_smoother) {
        return true;
      }
      return velocity_smoother->CommitApplied(applied, timestamp_s);
    };
    auto stop_velocity = [&](double timestamp_s, const std::string &reason) {
      if (velocity_smoother) {
        (void)velocity_smoother->Stop(timestamp_s, reason);
      }
    };
    auto record_final_output_publish_failure = [&](const std::string &reason) {
      control_authority.holdOperatorTakeover();
      operator_resume_required = true;
      last_teleop.published = false;
      frames.last_error = reason;
      nav_status.requestImmediate();
    };

    auto publish_zero_command = [&]() -> bool {
      stop_velocity(steadySeconds(), "zero_command");
      if (!cfg.publish_cmd_vel) {
        return true;
      }
      const auto write_start = SteadyClock::now();
      const auto publish_receipt =
          dds.publish(lingtu::nav::endpoint::OutputEvent{
              lingtu::nav::endpoint::FinalVelocityOutput{}});
      const auto &receipt = publish_receipt.final_velocity;
      if (current_timing != nullptr) {
        current_timing->dds_write_ms += elapsedMs(write_start);
      }
      if (receipt) {
        ++cmd_vel_count;
        last_zero_output_sequence = receipt->output_sequence;
        last_zero_source_wall_ns = receipt->source_wall_ns;
      } else {
        record_final_output_publish_failure("cmd_vel_zero_publish_failed");
      }
      return receipt.has_value();
    };
    auto publish_sequenced_zero_command = [&]() -> std::optional<std::uint64_t> {
      stop_velocity(steadySeconds(), "sequenced_zero_command");
      const auto publish_receipt =
          dds.publish(lingtu::nav::endpoint::OutputEvent{
              lingtu::nav::endpoint::FinalVelocityOutput{}});
      const auto &receipt = publish_receipt.final_velocity;
      if (receipt) {
        ++cmd_vel_count;
        last_zero_output_sequence = receipt->output_sequence;
        last_zero_source_wall_ns = receipt->source_wall_ns;
      } else {
        record_final_output_publish_failure("cmd_vel_shutdown_zero_publish_failed");
      }
      return receipt ? std::optional<std::uint64_t>{receipt->output_sequence} : std::nullopt;
    };

    auto clear_motion_outputs = [&](const std::string &reason) -> bool {
      stop_velocity(steadySeconds(), reason);
      executor.clear();
      last_global_path.clear();
      last_local_path.clear();
      last_local_planner_debug = {};
      last_local_planner_debug.backend = cfg.local_planner_backend;
      last_local = LocalDiagnostics{};
      last_local.seen = true;
      last_local.active = false;
      last_local.near_field_stop = true;
      last_local.reason = reason;
      last_local.final_safety_stopped = true;
      last_local.final_safety_reason = reason;
      last_plan.seen = true;
      last_plan.accepted = false;
      last_plan.reached_goal = false;
      last_plan.reason = reason;
      last_teleop.fresh = false;
      last_teleop.published = false;
      last_teleop.stopped = true;
      last_teleop.slowed = false;
      last_teleop.output = {};
      last_teleop.reason = reason;
      const bool zero_published = publish_zero_command();
      last_teleop.published = cfg.publish_cmd_vel && zero_published;
      nav_status.requestImmediate();
      return zero_published;
    };
    auto suspend_motion_outputs = [&](const std::string &reason) -> bool {
      stop_velocity(steadySeconds(), reason);
      executor.suspendAutonomy();
      last_local_path.clear();
      last_local_planner_debug = {};
      last_local_planner_debug.backend = cfg.local_planner_backend;
      last_local = LocalDiagnostics{};
      last_local.seen = true;
      last_local.active = false;
      last_local.near_field_stop = true;
      last_local.reason = reason;
      last_local.final_safety_stopped = true;
      last_local.final_safety_reason = reason;
      last_teleop.fresh = false;
      last_teleop.published = false;
      last_teleop.stopped = true;
      last_teleop.slowed = false;
      last_teleop.output = {};
      last_teleop.reason = reason;
      const bool zero_published = publish_zero_command();
      last_teleop.published = cfg.publish_cmd_vel && zero_published;
      nav_status.requestImmediate();
      return zero_published;
    };

    RollingSegmentEffectActions rolling_segment_effect_actions;
    rolling_segment_effect_actions.activate_authority = [&]() {
      return control_authority.activatePath();
    };
    rolling_segment_effect_actions.install_path =
        [&](const RollingSegmentInstallPathEffect &value) {
          const auto path = rollingSegmentPathToNav(value.path, value.map_z);
          executor.setRoute(lingtu::nav::navigation::Route{path});
          last_global_path = path;
          last_local_path.clear();
          last_local_planner_debug = {};
          last_local_planner_debug.backend = cfg.local_planner_backend;
          last_plan = PlanDiagnostics{};
          last_plan.seen = true;
          last_plan.accepted = true;
          last_plan.reached_goal = false;
          if (map_body) {
            last_plan.start = map_body->position;
          }
          last_plan.goal = path.back();
          last_plan.waypoints = path.size();
          last_plan.reason = "rolling_segment_accepted";
          last_local = LocalDiagnostics{};
          ++path_count;
        };
    rolling_segment_effect_actions.publish_path =
        [&](const RollingSegmentPublishPathEffect &value) {
          const auto path = rollingSegmentPathToNav(value.path, value.map_z);
          (void)dds.publish(lingtu::nav::endpoint::OutputEvent{
              lingtu::nav::endpoint::GlobalPathOutput{path, value.stamp_s}});
        };
    rolling_segment_effect_actions.publish_ack = [&](const RollingSegmentAck &value) {
      ExplorationSegmentAck ack;
      ack.stamp_s = nowSeconds();
      ack.request_id = value.request_id;
      ack.kind = value.kind;
      ack.accepted = value.accepted;
      ack.session_id = value.session_id;
      ack.reset_epoch = value.reset_epoch;
      ack.generation = value.generation;
      ack.live = value.live;
      ack.reason = value.reason;
      return dds.publish(lingtu::nav::endpoint::OutputEvent{ack}).published;
    };
    rolling_segment_effect_actions.publish_status = [&](const RollingSegmentStatus &value) {
      ExplorationSegmentStatus status;
      status.stamp_s = nowSeconds();
      status.request_id = value.request_id;
      status.state = static_cast<std::int32_t>(value.state);
      status.session_id = value.session_id;
      status.reset_epoch = value.reset_epoch;
      status.generation = value.generation;
      status.live = value.live;
      status.reason = value.reason;
      return dds.publish(lingtu::nav::endpoint::OutputEvent{status}).published;
    };
    rolling_segment_effect_actions.stop_authority = [&]() { control_authority.stop(); };
    rolling_segment_effect_actions.clear_motion = clear_motion_outputs;
    RollingSegmentEffectCoordinator rolling_segment_effect_coordinator(
        rolling_segment, std::move(rolling_segment_effect_actions));

    auto current_map_identity = [&]() {
      GoalPlanMapIdentityResult result;
      if (cfg.global_planner == GlobalPlannerBackend::Far) {
        auto identity = active_occupancy_gate->currentIdentity(cfg.map_path);
        result.identity = std::move(identity.identity);
        result.reason = std::move(identity.reason);
      } else {
        auto identity = active_octomap_gate->currentIdentity(cfg.map_path);
        result.identity = std::move(identity.identity);
        result.reason = std::move(identity.reason);
      }
      return result;
    };

    NavigationGoalStatusOutbox goal_status_outbox(
        [&](const GoalPlanStatus &status) { navigation_state.observe(status); },
        [&](const GoalPlanStatus &status) {
          return dds
              .publish(lingtu::nav::endpoint::OutputEvent{
                  lingtu::nav::endpoint::NavigationGoalStatusOutput{
                      status.task_id, status.request_id, status.state, status.goal_epoch,
                      status.reason}})
              .published;
        });

    GoalPlanActions goal_plan_actions;
    goal_plan_actions.preempt_rolling = [&](const std::string &reason) {
      if (!rolling_segment.snapshot().active) {
        return true;
      }
      return rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentGenericPreempt{reason}));
    };
    goal_plan_actions.clear_external_inspection = [&]() { inspection_runtime.clearActivePoint(); };
    goal_plan_actions.current_map_identity = current_map_identity;
    goal_plan_actions.publish_status = [&](const GoalPlanStatus &status) {
      (void)goal_status_outbox.record(status);
    };
    goal_plan_actions.inspection_active = [&]() { return inspection_executor.active(); };
    goal_plan_actions.inspection_leg_failed = [&](const std::string &reason, double now_s) {
      inspection_executor.OnLegFailed(reason, now_s);
    };
    goal_plan_actions.inspection_pause = [&](const std::string &reason) {
      inspection_executor.Pause(reason);
    };
    goal_plan_actions.inspection_plan_ready = [&](double now_s) {
      GoalPlanInspectionDecision decision;
      decision.accepted = inspection_executor.OnPlanReady(now_s);
      decision.reason = inspection_executor.status().reason;
      const auto &active_point = inspection_runtime.activePoint();
      if (decision.accepted && active_point) {
        decision.tolerance = GoalPlanPathTolerance{
            active_point->position_tolerance_m,
            active_point->yaw_tolerance_rad,
        };
      }
      return decision;
    };
    goal_plan_actions.activate_path = [&](const GoalPlanPathActivation &activation) {
      if (activation.tolerance) {
        executor.setRoute(lingtu::nav::navigation::Route{
            activation.path, activation.goal_yaw, activation.tolerance->position_m,
            activation.tolerance->yaw_rad});
      } else {
        executor.setRoute(
            lingtu::nav::navigation::Route{activation.path, activation.goal_yaw});
      }
      const auto write_start = SteadyClock::now();
      (void)dds.publish(lingtu::nav::endpoint::OutputEvent{
          lingtu::nav::endpoint::GlobalPathOutput{activation.path, activation.stamp_s}});
      if (current_timing != nullptr) {
        current_timing->dds_write_ms += elapsedMs(write_start);
      }
      last_global_path = activation.path;
      last_local_path.clear();
      last_local_planner_debug = {};
      last_local_planner_debug.backend = cfg.local_planner_backend;
      (void)control_authority.activatePath();
      last_local = LocalDiagnostics{};
    };
    GoalPlanController goal_plan(std::move(global_planner), std::move(goal_plan_actions));
    GoalReplanRuntimeCoordinator *goal_replan_runtime_ptr = nullptr;
    auto sync_goal_plan_diagnostics = [&]() {
      const auto snapshot = goal_plan.snapshot();
      const auto &diagnostics = snapshot.diagnostics;
      last_plan = PlanDiagnostics{};
      last_plan.seen = diagnostics.seen;
      last_plan.accepted = diagnostics.accepted;
      last_plan.reached_goal = diagnostics.reached_goal;
      last_plan.reason = diagnostics.reason;
      last_plan.waypoints = diagnostics.waypoints;
      last_plan.goal_error_m = diagnostics.goal_error_m;
      last_plan.elapsed_ms = diagnostics.elapsed_ms;
      last_plan.start = diagnostics.start;
      last_plan.goal = diagnostics.goal;
    };
    auto reset_navigation_epoch = [&](double epoch_start_s, const std::string &reason,
                                      bool clear_motion) {
      (void)epoch_start_s;
      const bool rolling_segment_was_active = rolling_segment.snapshot().active;
      const std::string rolling_input_reason = std::string("execution_grid_epoch_reset:") + reason;
      const auto rolling_reset = rolling_segment.step(RollingSegmentObserveInvalidInput{
          rolling_input_reason,
          reason,
      });
      inspection_runtime.resetArrivalOdomGeneration(odom_generation);
      if (clear_motion) {
        (void)inspection_executor.Pause(reason);
        control_authority.holdOperatorTakeover();
        operator_resume_required = true;
        if (goal_replan_runtime_ptr != nullptr) {
          (void)goal_replan_runtime_ptr->interrupt(GoalReplanRuntimeInterruption::kControlHold,
                                                   steadySeconds());
        }
        sync_goal_plan_diagnostics();
      }
      if (rolling_segment_was_active) {
        (void)rolling_segment_effect_coordinator.apply(rolling_reset);
      } else if (clear_motion) {
        if (!clear_motion_outputs(reason)) {
          record_final_output_publish_failure(reason + ":zero_publish_failed");
        }
      }
    };

    InputActions inputs_actions;
    inputs_actions.on_epoch_reset = reset_navigation_epoch;
    inputs_actions.on_rolling_snapshot_invalidated = [&](const std::string &reason) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentObserveInvalidInput{reason, {}}));
      frames.last_error = rolling_segment.snapshot().input_error;
    };
    inputs_actions.on_execution_grid = [&](RollingSegmentExecutionGrid grid) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentObserveExecutionGrid{std::move(grid)}));
      const auto rolling_state = rolling_segment.snapshot();
      if (!rolling_state.has_execution_grid && !rolling_state.input_error.empty()) {
        frames.last_error = rolling_state.input_error;
      }
    };
    InputConfig inputs_config;
    inputs_config.source_transform_max_gap_s = source_transform_max_gap_s;
    inputs_config.cloud_pose_max_gap_s = cfg.cloud_pose_max_gap_s;
    inputs_config.driver_control_max_age_s = cfg.driver_control_max_age_s;
    inputs_config.sensor_offset = {
        cfg.sensor_offset_x_m,
        cfg.sensor_offset_y_m,
        cfg.sensor_offset_z_m,
    };
    inputs_config.check_obstacle = cfg.check_obstacle;
    inputs_config.max_obstacle_points = cfg.max_obstacle_points;
    InputProjector inputs(state, input_gate, pose_buffer, map_odom_buffer, live_obstacles,
                          std::move(inputs_config), std::move(inputs_actions));

    StopConfirmationConfig stop_confirmation_config;
    stop_confirmation_config.timeout = std::chrono::milliseconds(
        static_cast<std::int64_t>(std::llround(cfg.stop_confirmation_timeout_s * 1000.0)));
    stop_confirmation_config.evidence_policy =
        cfg.control_mode == ControlMode::Autonomy
            ? StopConfirmationEvidencePolicy::DriverAckAndOdometry
            : StopConfirmationEvidencePolicy::DriverAckOnly;
    auto wait_for_stop_confirmation = [&](std::uint64_t output_sequence) {
      if (output_sequence != last_zero_output_sequence || last_zero_source_wall_ns == 0U) {
        return StopConfirmationState::TimedOut;
      }
      StopConfirmation confirmation(dds_status.producer_boot_id, output_sequence,
                                    last_zero_source_wall_ns,
                                    StopConfirmation::Clock::now(), stop_confirmation_config);
      OdometrySpeedMonitor stop_speed_monitor(OdometrySpeedEvidence::PoseDerivedPlanar);
      while (true) {
        // Drain queued odometry before observing the exact driver ACK. Samples
        // counted on later iterations are therefore post-ACK by local receive
        // order without comparing the independent odometry and wall clocks.
        auto stop_inputs = dds.takeSensors(steadySeconds());
        for (const auto &sample : stop_inputs.odometry) {
          if (!sample.ok()) {
            continue;
          }
          const auto &msg = sample.value;
          const double stamp_s = msg.header.stamp_s;
          const std::string &frame_id = msg.header.frame_id;
          const auto &position = msg.body.translation;
          const auto &linear = msg.linear_velocity;
          const double linear_speed = stop_speed_monitor.observe(
              stamp_s, frame_id, position.x, position.y, position.z, linear.x, linear.y, linear.z);
          const auto &angular = msg.angular_velocity;
          const double angular_speed =
              std::sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z);
          confirmation.observeQuietOdometry(stamp_s, linear_speed, angular_speed);
        }
        if (stop_inputs.driver_control) {
          const auto &driver = *stop_inputs.driver_control;
          inputs.projectDriverControl(driver, stop_inputs.receive_steady_s);
          confirmation.observeDriverAck(driver.accepted_producer_boot_id,
                                        driver.accepted_output_sequence,
                                        driver.last_command_accepted, driver.stamp_ns);
        }
        const StopConfirmationState confirmation_state = confirmation.state();
        if (confirmation_state != StopConfirmationState::Pending) {
          if (confirmation_state != StopConfirmationState::Confirmed) {
            const auto diagnostics = confirmation.diagnostics();
            std::fprintf(
                stderr,
                "navd: stop confirmation failed state=%d output_sequence=%llu "
                "zero_stamp_ns=%llu ack_observed=%d ack_accepted=%d ack_stamp_ns=%llu "
                "odom_observed=%zu odom_post_ack=%zu odom_stale=%zu odom_moving=%zu "
                "odom_quiet=%zu/%zu last_odom_stamp_ns=%llu "
                "last_linear_speed=%.6f last_angular_speed=%.6f\n",
                static_cast<int>(confirmation_state),
                static_cast<unsigned long long>(output_sequence),
                static_cast<unsigned long long>(diagnostics.zero_published_source_wall_ns),
                diagnostics.driver_ack_observed ? 1 : 0, diagnostics.driver_accepted ? 1 : 0,
                static_cast<unsigned long long>(diagnostics.driver_ack_source_stamp_ns),
                diagnostics.odometry_samples_observed, diagnostics.post_ack_odometry_samples,
                diagnostics.stale_odometry_samples, diagnostics.moving_odometry_samples,
                diagnostics.quiet_odometry_samples, diagnostics.required_quiet_odometry_samples,
                static_cast<unsigned long long>(diagnostics.last_odometry_source_stamp_ns),
                diagnostics.last_linear_speed_mps, diagnostics.last_angular_speed_radps);
          }
          return confirmation_state;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    };

    MotionStopActions motion_stop_actions;
    // Ordinary operator safety transitions remain owned by MotionStopBarrier.
    // Inspection task pause/cancel uses the ticketed coordinator path below so
    // a GoalPlan terminal cannot bypass exact status delivery.
    motion_stop_actions.defer_goal_abort = [&](const std::string &reason) {
      return goal_plan.deferAbort(reason);
    };
    motion_stop_actions.record_stop_evidence_failure = record_final_output_publish_failure;
    motion_stop_actions.sync_goal_diagnostics = sync_goal_plan_diagnostics;
    motion_stop_actions.rolling_segment_active = [&]() {
      return rolling_segment.snapshot().active;
    };
    motion_stop_actions.preempt_rolling_segment = [&](const std::string &reason) {
      return rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentGenericPreempt{reason}));
    };
    motion_stop_actions.clear_motion_outputs = clear_motion_outputs;
    motion_stop_actions.suspend_motion_outputs = suspend_motion_outputs;
    motion_stop_actions.cancel_control = [&]() { control_authority.cancel(); };
    motion_stop_actions.stop_control = [&]() { control_authority.stop(); };
    motion_stop_actions.latch_estop = [&](const std::string &reason) {
      control_authority.latchEstop(reason);
    };
    motion_stop_actions.clear_control_estop = [&]() { return control_authority.clearEstop(true); };
    motion_stop_actions.resume_control = [&]() { return control_authority.resumeMotion(); };
    motion_stop_actions.cancel_inspection = [&](const std::string &reason) {
      (void)inspection_executor.Cancel(reason);
    };
    motion_stop_actions.clear_operator_resume_required = [&]() {
      if (!control_loop_guard_latched()) {
        operator_resume_required = false;
      }
    };
    motion_stop_actions.set_autonomy_request_not_before = [&](double stamp_s) {
      autonomy_request_not_before_s = stamp_s;
    };
    motion_stop_actions.set_teleop_request_not_before = [&](double stamp_s) {
      teleop_request_not_before_s = stamp_s;
    };
    motion_stop_actions.persist_estop_latch = [&](const std::string &reason) {
      return estop_latch_store.persist(reason);
    };
    motion_stop_actions.clear_persisted_estop_latch = [&]() { return estop_latch_store.clear(); };
    motion_stop_actions.publish_zero = publish_zero_command;
    motion_stop_actions.last_output_sequence = [&]() {
      return dds_status.final_output_sequence;
    };
    motion_stop_actions.publish_sequenced_zero = publish_sequenced_zero_command;
    motion_stop_actions.confirm_zero = [&](std::uint64_t output_sequence) {
      return wait_for_stop_confirmation(output_sequence);
    };
    motion_stop_actions.clear_global_path = [&]() { executor.clear(); };
    MotionStopBarrier motion_stop(cfg.publish_cmd_vel, std::move(motion_stop_actions));
    GoalReplanRuntimeCoordinator goal_replan_runtime(goal_plan, motion_stop);
    ActivePathBlockagePolicyConfig blockage_config;
    blockage_config.persistence_s = 0.6;
    blockage_config.minimum_fresh_observations = 2U;
    blockage_config.lookahead_m = cfg.corridor_lookahead_m;
    blockage_config.corridor_radius_m =
        std::max(0.1, cfg.vehicle_width_m * 0.5 + cfg.teleop_obstacle_margin_m);
    blockage_config.obstacle_height_min_m = cfg.teleop_obstacle_height_min_m;
    blockage_config.obstacle_height_max_m =
        std::max(cfg.local_planner_obstacle_height_max_m, cfg.teleop_obstacle_height_max_m);
    // Height filtering removes the dense ground layer; two spatial returns
    // sustained for the full persistence window are enough to represent a
    // real obstacle while still rejecting one-point lidar noise.
    blockage_config.minimum_obstacle_points = 2U;
    blockage_config.overlay_radius_m =
        std::max(0.1, cfg.obstacle_voxel_size_m + cfg.live_obstacle_inflation_radius_m);
    ActivePathBlockagePolicy active_path_blockage_policy(blockage_config);
    goal_replan_runtime_ptr = &goal_replan_runtime;
    GoalTerminalStatusDelivery goal_terminal_delivery(goal_status_outbox);
    InspectionCommandActions inspection_command_actions;
    inspection_command_actions.route_source_available = [&]() {
      return inspection_store != nullptr;
    };
    inspection_command_actions.active_map = [&]() -> std::optional<InspectionActiveMap> {
      if (!cfg.map_identity.valid()) {
        return std::nullopt;
      }
      return InspectionActiveMap{cfg.map_identity.map_id, cfg.map_identity.content_epoch};
    };
    inspection_command_actions.load_route =
        [&](const std::string &map_id,
            const std::string &route_id) -> std::optional<lingtu::nav::inspection::Route> {
      if (!inspection_store) {
        return std::nullopt;
      }
      return inspection_store->Get(map_id, route_id);
    };
    inspection_command_actions.operator_takeover_latched = [&]() {
      return control_authority.operatorTakeoverLatched() || control_loop_guard_latched();
    };
    inspection_command_actions.stop_and_commit = [&](const std::string &reason,
                                                     InspectionCommandCommit commit) {
      const auto interruption = reason == "inspection_cancel_requested"
                                    ? GoalReplanRuntimeInterruption::kInspectionCancel
                                    : GoalReplanRuntimeInterruption::kInspectionPause;
      const auto terminal = goal_replan_runtime.interrupt(interruption, steadySeconds());
      if (!terminal.terminal_after_stop || terminal.terminal_intent_id == 0U) {
        bool inspection_committed = false;
        const auto result = motion_stop.commitGoalTerminalAfterStop(
            reason, [commit = std::move(commit), &inspection_committed]() mutable {
              inspection_committed = commit();
            });
        if (result.accepted && !inspection_committed) {
          return InspectionStopBarrierResult{false, "inspection_state_commit_failed"};
        }
        return InspectionStopBarrierResult{result.accepted, result.reason};
      }

      const auto &pending = *terminal.terminal_after_stop;
      const auto stage =
          goal_terminal_delivery.stage(terminal.terminal_intent_id, pending.delivery_ticket);
      if (stage != GoalTerminalStatusDelivery::StageResult::kStaged &&
          stage != GoalTerminalStatusDelivery::StageResult::kReplay) {
        return InspectionStopBarrierResult{
            false,
            stage == GoalTerminalStatusDelivery::StageResult::kConflict
                ? "goal_terminal_delivery_ticket_conflict"
                : "goal_terminal_delivery_ticket_invalid",
        };
      }
      if (goal_terminal_delivery.isCommitted(terminal.terminal_intent_id)) {
        return InspectionStopBarrierResult{true, "inspection_goal_terminal_committed"};
      }

      bool inspection_committed = false;
      const auto result = motion_stop.commitGoalTerminalAfterStop(
          reason, [goal_terminal_commit = pending.commit, commit = std::move(commit),
                   &inspection_committed]() mutable {
            goal_terminal_commit();
            inspection_committed = commit();
          });
      if (!result.accepted) {
        return InspectionStopBarrierResult{false, result.reason};
      }
      if (!inspection_committed) {
        return InspectionStopBarrierResult{false, "inspection_state_commit_failed"};
      }
      if (!goal_terminal_delivery.markCommitted(terminal.terminal_intent_id)) {
        return InspectionStopBarrierResult{
            false,
            "goal_terminal_delivery_commit_identity_rejected",
        };
      }
      sync_goal_plan_diagnostics();
      return InspectionStopBarrierResult{true, result.reason};
    };
    inspection_command_actions.publish_ack = [&](const InspectionCommandAck &ack) {
      return dds
          .publish(lingtu::nav::endpoint::OutputEvent{
              lingtu::nav::endpoint::InspectionTaskAckOutput{
                  ack.task_id, ack.request_id, ack.kind, ack.accepted, ack.reason, ack.run_id}})
          .published;
    };
    inspection_command_actions.request_status = [&]() { inspection_runtime.requestStatus(); };
    inspection_command_actions.now_s = nowSeconds;
    InspectionCommandCoordinator inspection_command_coordinator(
        inspection_executor, std::move(inspection_command_actions));

    TeleopAdmissionActions teleop_actions;
    teleop_actions.hold_operator_takeover = [&]() { control_authority.holdOperatorTakeover(); };
    teleop_actions.stop_control = [&]() { control_authority.stop(); };
    teleop_actions.begin_operator_takeover = [&](const nav_kernel::Twist &command,
                                                 double source_stamp_s) {
      return control_authority.beginOperatorTakeover(command, source_stamp_s);
    };
    teleop_actions.mark_operator_resume_required = [&]() { operator_resume_required = true; };
    teleop_actions.clear_motion = [&](const std::string &reason) {
      return motion_stop.clearEndpointMotion(reason);
    };
    teleop_actions.pause_inspection = [&](const std::string &reason) {
      (void)inspection_executor.Pause(reason);
    };
    teleop_actions.accept_teleop = [&](const nav_kernel::Twist &command, double source_stamp_s) {
      return control_authority.acceptTeleop(command, source_stamp_s);
    };
    teleop_actions.publish_zero = [&]() { return motion_stop.keepZeroFresh(); };
    TeleopAdmissionController teleop_admission(std::move(teleop_actions));

    const auto execution_observation = [&](bool local_traversability) {
      lingtu::nav::navigation::ExecutionObservation observation{
          state.frame_epoch,
          state.cloud_generation,
          local_traversability ? state.local_traversability_generation
                               : state.traversability_generation,
          state.last_odom_s,
          state.last_cloud_s,
          local_traversability ? state.last_local_traversability_s : state.last_traversability_s,
          state.odom_linear_velocity_body,
          state.odom_yaw_rate,
          state.odom_velocity_valid,
      };
      observation.collision = state.local_collision_map.view();
      return observation;
    };

    FinalActions final_control_actions;
    final_control_actions.command_safety = evaluateCommandSafety;
    final_control_actions.shape = shape_velocity;
    final_control_actions.commit = commit_applied_velocity;
    final_control_actions.stop = stop_velocity;
    FinalControl final_control(std::move(final_control_actions));

    TeleopTickActions teleop_tick_actions;
    teleop_tick_actions.teleop_receive_age_s = teleop_receive_age_s;
    teleop_tick_actions.steady_now_s = steadySeconds;
    teleop_tick_actions.read_plan = read_plan;
    teleop_tick_actions.tick_teleop_intent =
        [&](const nav_kernel::Pose &pose, const nav_kernel::Twist &intent, const float *obstacles,
            int obstacle_count, double now_s,
            lingtu::nav::navigation::TraversabilityGridView traversability) {
          lingtu::nav::navigation::ExecutionInput input;
          input.mode = lingtu::nav::navigation::ExecutionMode::MotionIntent;
          input.mapBody = pose;
          input.odomBody = pose;
          input.obstacleXyzhMap = obstacles;
          input.obstacleCount = obstacle_count;
          input.timestampS = now_s;
          input.traversability = traversability;
          input.observation = execution_observation(false);
          input.motionIntent = intent;
          return executor.tick(input);
        };
    teleop_tick_actions.pause_linear_motion = [&]() { executor.pauseLinearMotion(); };
    teleop_tick_actions.replan_motion = [&]() { executor.replanTeleop(); };
    teleop_tick_actions.stop_linear_motion = [&]() { executor.stopLinearMotion(); };
    TeleopTickController teleop_tick(std::move(teleop_tick_actions), final_control);

    AutonomyTickActions autonomy_tick_actions;
    autonomy_tick_actions.steady_now_s = steadySeconds;
    autonomy_tick_actions.current_map_identity = current_map_identity;
    autonomy_tick_actions.read_plan = read_plan;
    autonomy_tick_actions.tick_autonomy =
        [&](const nav_kernel::Pose &pose, const float *obstacles, int obstacle_count, double now_s,
            lingtu::nav::navigation::TraversabilityGridView traversability) {
          const auto local_traversability = state.local_traversability_grid.view();
          const double local_traversability_max_age_s =
              cfg.traversability_max_age_s > 0.0
                  ? std::min(cfg.traversability_max_age_s, kLocalTraversabilityMaxAgeS)
                  : kLocalTraversabilityMaxAgeS;
          const bool local_traversability_fresh =
              local_traversability.valid() && state.last_local_traversability_receive_s > 0.0 &&
              now_s - state.last_local_traversability_receive_s <= local_traversability_max_age_s;
          const bool use_odom_local_planning =
              traversability.valid() && local_traversability_fresh && state.odom_requires_tf &&
              state.odom_body && state.map_odom_tf && state.map_odom_tf->valid;
          const auto observation = execution_observation(use_odom_local_planning);
          if (use_odom_local_planning) {
            const lingtu::nav::navigation::MapFromOdomTransform map_from_odom{
                state.map_odom_tf->translation,
                state.map_odom_tf->yaw,
            };
            return executor.tick(lingtu::nav::navigation::ExecutionInput{
                lingtu::nav::navigation::ExecutionMode::Route,
                pose,
                *state.odom_body,
                map_from_odom,
                obstacles,
                obstacle_count,
                now_s,
                local_traversability,
                observation,
                {}});
          }
          return executor.tick(lingtu::nav::navigation::ExecutionInput{
              lingtu::nav::navigation::ExecutionMode::Route,
              pose,
              pose,
              {},
              obstacles,
              obstacle_count,
              now_s,
              traversability,
              observation,
              {}});
        };
    autonomy_tick_actions.stop_linear_motion = [&]() { executor.stopLinearMotion(); };
    AutonomyTickController autonomy_tick(std::move(autonomy_tick_actions), final_control);

    EndpointLoopContext loop_ctx{
        cfg,
        gate_cfg,
        safety_config,
        operator_motion_interface_enabled,
        dds,
        dds_status,
        state,
        executor,
        geofence,
        inspection_status_writer,
        inspection_store.get(),
        inspection_task_event_outbox,
        inputs,
        plan_preview,
        goal_plan,
        goal_replan_runtime,
        active_path_blockage_policy,
        motion_stop,
        goal_status_outbox,
        goal_terminal_delivery,
        inspection_runtime,
        inspection_command_coordinator,
        teleop_admission,
        teleop_tick,
        autonomy_tick,
        rolling_segment,
        rolling_segment_effect_coordinator,
        nav_status,
        control_loop_health,
        control_loop_guard,
        operator_motion_authority,
        command_ingress,
        navigation_state,
        current_map_identity,
        sync_goal_plan_diagnostics,
        control_loop_guard_latched,
        current_timing,
    };
    return runEndpointLoop(loop_ctx, g_running);
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "navd failed: %s\n", exc.what());
    return 1;
  }
}
