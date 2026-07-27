#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "endpoint_config.hpp"
#include "endpoint_loop.hpp"
#include "endpoint_state.hpp"
#include "endpoint_time.hpp"
#include "input/nav_input_state_projector.hpp"
#include "inspection/inspection_command_coordinator.hpp"
#include "inspection/inspection_runtime_controller.hpp"
#include "lingtu/maps/store.hpp"
#include "lingtu_slam.h"
#include "motion/autonomy_tick_controller.hpp"
#include "motion/command_identity.hpp"
#include "motion/command_ingress_controller.hpp"
#include "motion/control_authority.hpp"
#include "motion/control_loop_runtime_guard.hpp"
#include "motion/estop_latch_store.hpp"
#include "motion/motion_stop_coordinator.hpp"
#include "motion/operator_motion_authority.hpp"
#include "motion/stop_confirmation.hpp"
#include "motion/teleop_admission_controller.hpp"
#include "motion/teleop_safety.hpp"
#include "motion/teleop_tick_controller.hpp"
#include "nav/cpp/planning/global/octoplanner/octoplanner3d_core.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav/inspection/store.hpp"
#include "nav_dds_runtime.hpp"
#include "nav_endpoint_config.hpp"
#include "nav_endpoint_messages.hpp"
#include "nav_loop.hpp"
#include "plan/active_occupancy_gate.hpp"
#include "plan/active_octomap_gate.hpp"
#include "plan/goal_plan_controller.hpp"
#include "plan/input_gate.hpp"
#include "plan/live_obstacle_layer.hpp"
#include "plan/planner_inputs.hpp"
#include "plan/rolling_segment_effect_coordinator.hpp"
#include "plan/rolling_segment_lifecycle.hpp"
#include "status/active_inspection_map_cache.hpp"
#include "status/control_loop_health.hpp"
#include "status/inspection_status_file_writer.hpp"
#include "status/nav_status_endpoint_adapter.hpp"
#include "status/nav_status_publisher.hpp"
#include "status/navigation_state.hpp"
#include "traversability/transform_buffer.hpp"

namespace {

std::atomic_bool g_running{true};
using lingtu::nav::endpoint::ActiveInspectionMapCache;
using lingtu::nav::endpoint::ActiveInspectionMapIdentity;
using lingtu::nav::endpoint::ActiveOccupancyGate;
using lingtu::nav::endpoint::ActiveOctomapGate;
using lingtu::nav::endpoint::AutonomyTickActions;
using lingtu::nav::endpoint::AutonomyTickController;
using lingtu::nav::endpoint::AutonomyTickPlannerInputs;
using lingtu::nav::endpoint::CliConfig;
using lingtu::nav::endpoint::CommandIngressController;
using lingtu::nav::endpoint::commandSafetyConfig;
using lingtu::nav::endpoint::ControlLoopHealth;
using lingtu::nav::endpoint::ControlLoopHealthConfig;
using lingtu::nav::endpoint::ControlLoopRuntimeGuard;
using lingtu::nav::endpoint::ControlLoopRuntimeGuardConfig;
using lingtu::nav::endpoint::ControlLoopRuntimeGuardState;
using lingtu::nav::endpoint::ControlMode;
using lingtu::nav::endpoint::DdsRuntime;
using lingtu::nav::endpoint::EstopLatchStore;
using lingtu::nav::endpoint::evaluateCommandSafety;
using lingtu::nav::endpoint::GlobalPlannerBackend;
using lingtu::nav::endpoint::globalPlannerBackendName;
using lingtu::nav::endpoint::GoalPlanActions;
using lingtu::nav::endpoint::GoalPlanController;
using lingtu::nav::endpoint::GoalPlanInspectionDecision;
using lingtu::nav::endpoint::GoalPlanMapIdentityResult;
using lingtu::nav::endpoint::GoalPlanPathActivation;
using lingtu::nav::endpoint::GoalPlanPathTolerance;
using lingtu::nav::endpoint::GoalPlanStatus;
using lingtu::nav::endpoint::headerFrameId;
using lingtu::nav::endpoint::headerStampSeconds;
using lingtu::nav::endpoint::InputGate;
using lingtu::nav::endpoint::InputGateConfig;
using lingtu::nav::endpoint::inputGateConfig;
using lingtu::nav::endpoint::InspectionActiveMap;
using lingtu::nav::endpoint::InspectionCommandAck;
using lingtu::nav::endpoint::InspectionCommandActions;
using lingtu::nav::endpoint::InspectionCommandCoordinator;
using lingtu::nav::endpoint::InspectionRuntimeController;
using lingtu::nav::endpoint::InspectionStatusFileWriter;
using lingtu::nav::endpoint::LiveObstacleLayer;
using lingtu::nav::endpoint::LiveObstacleLayerConfig;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::MotionStopActions;
using lingtu::nav::endpoint::MotionStopCoordinator;
using lingtu::nav::endpoint::NavigationControlState;
using lingtu::nav::endpoint::NavigationStateTracker;
using lingtu::nav::endpoint::NavInputStateProjector;
using lingtu::nav::endpoint::NavInputStateProjectorActions;
using lingtu::nav::endpoint::NavInputStateProjectorConfig;
using lingtu::nav::endpoint::NavStatusPublisher;
using lingtu::nav::endpoint::NavStatusPublisherActions;
using lingtu::nav::endpoint::ObstacleMergeConfig;
using lingtu::nav::endpoint::OdometrySpeedEvidence;
using lingtu::nav::endpoint::OdometrySpeedMonitor;
using lingtu::nav::endpoint::OperatorMotionAuthority;
using lingtu::nav::endpoint::parseArgs;
using lingtu::nav::endpoint::PlanDiagnostics;
using lingtu::nav::endpoint::runWithActiveOccupancy;
using lingtu::nav::endpoint::runWithActiveOctomap;
using lingtu::nav::endpoint::StatusMotionLayerSample;
using lingtu::nav::endpoint::StatusPlannerSample;
using lingtu::nav::endpoint::StatusWriterConfig;
using lingtu::nav::endpoint::StopConfirmation;
using lingtu::nav::endpoint::StopConfirmationConfig;
using lingtu::nav::endpoint::StopConfirmationState;
using lingtu::nav::endpoint::TeleopAdmissionActions;
using lingtu::nav::endpoint::TeleopAdmissionController;
using lingtu::nav::endpoint::TeleopTickActions;
using lingtu::nav::endpoint::TeleopTickController;
using lingtu::nav::endpoint::TeleopTickPlannerInputs;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TransformBuffer;
using CommandKind = lingtu::message::NavigationCommandKind;
using ExplorationExecutionGridView = lingtu::nav::endpoint::ExplorationExecutionGridView;
using ExplorationSegmentAck = lingtu::nav::endpoint::ExplorationSegmentAck;
using ExplorationSegmentStatus = lingtu::nav::endpoint::ExplorationSegmentStatus;
using ExplorationSegmentRequestView = lingtu::nav::endpoint::ExplorationSegmentRequestView;
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
    auto active_octomap_gate = std::make_shared<ActiveOctomapGate>(cfg.map_root);
    auto active_occupancy_gate = std::make_shared<ActiveOccupancyGate>(cfg.map_root);
    std::unique_ptr<lingtu::maps::MapStore> inspection_map_store;
    std::unique_ptr<lingtu::nav::inspection::Store> inspection_store;
    if (!cfg.map_root.empty()) {
      inspection_map_store =
          std::make_unique<lingtu::maps::MapStore>(lingtu::maps::MapStoreConfig{cfg.map_root});
      inspection_store = std::make_unique<lingtu::nav::inspection::Store>(cfg.map_root);
    }
    std::unique_ptr<ActiveInspectionMapCache> active_inspection_map_cache;
    if (inspection_map_store) {
      auto *map_store = inspection_map_store.get();
      active_inspection_map_cache = std::make_unique<ActiveInspectionMapCache>(
          [map_store]() -> std::optional<ActiveInspectionMapIdentity> {
            const auto record = map_store->GetActiveMap();
            if (!record) {
              return std::nullopt;
            }
            return ActiveInspectionMapIdentity{record->map_id, record->version};
          });
    }
    if (cfg.control_mode == ControlMode::Autonomy && !cfg.map_path.empty()) {
      if (cfg.global_planner == GlobalPlannerBackend::Far) {
        auto preflight = active_occupancy_gate->prepare(cfg.map_path);
        if (!preflight.ok()) {
          throw std::runtime_error("active occupancy failed native Maps preflight: " +
                                   preflight.reason);
        }
      } else {
        auto preflight = active_octomap_gate->prepare(cfg.map_path);
        if (!preflight.ok()) {
          throw std::runtime_error("active OctoMap failed native Maps preflight: " +
                                   preflight.reason);
        }
      }
    }
    const ObstacleMergeConfig obstacle_merge_config{
        cfg.obstacle_voxel_size_m,
        cfg.obstacle_registered_share,
        cfg.obstacle_terrain_share,
        cfg.obstacle_terrain_ext_share,
    };
    LiveObstacleLayer live_obstacles(LiveObstacleLayerConfig{
        cfg.obstacle_voxel_size_m,
        cfg.live_obstacle_decay_s,
        kLayerInflationM,
        cfg.live_obstacle_ray_clear_max_range_m,
        cfg.live_obstacle_ray_clearing_interval_s,
        cfg.live_obstacle_max_clearing_rays,
        cfg.live_obstacle_min_hits,
        cfg.live_obstacle_ray_clearing,
    });
    const InputGateConfig gate_cfg = inputGateConfig(cfg);
    StatusWriterConfig status_cfg = buildStatusWriterConfig(cfg, gate_cfg);
    InspectionStatusFileWriter inspection_status_writer(cfg.map_root);
    InputGate input_gate(gate_cfg);
    TransformBuffer pose_buffer;
    TransformBuffer map_odom_buffer;
    const double source_transform_max_gap_s =
        cfg.tf_max_age_s > 0.0 ? std::min(cfg.cloud_pose_max_gap_s, cfg.tf_max_age_s)
                               : cfg.cloud_pose_max_gap_s;
    DdsRuntime dds(cfg.domain_id, cfg.allow_legacy_motion_inputs);
    NavigationStateTracker navigation_state(navigationControlState(cfg.control_mode));

    const auto safety_config = commandSafetyConfig(cfg);
    lingtu::nav::plan::NavLoopConfig nav_config =
        buildNavLoopConfig(cfg, safety_config.obstacle_margin_m);

    lingtu::nav::plan::NavLoop nav(nav_config);
    if ((cfg.control_mode == ControlMode::Autonomy || cfg.teleop_local_planner) &&
        !nav.configure()) {
      throw std::runtime_error("failed to load local planner path library: " +
                               cfg.path_library_dir);
    }
    auto octomap_planner = std::make_shared<octoplanner3d::runtime::PlannerSession>();
    auto far_planner = std::make_shared<lingtu::nav::plan::far::FarPlanner>(cfg.far_options);
    auto global_planner = [active_octomap_gate, active_occupancy_gate, octomap_planner, far_planner,
                           planner_backend = cfg.global_planner,
                           configured_map_path = cfg.map_path](const auto &request,
                                                               const auto &cancel_check) {
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

    EndpointState state;

    // -- Setup aliases shared by the controller wiring below ------------------
    auto &map_body = state.map_body;
    auto &obstacle_xyzh = state.obstacle_xyzh;
    auto &terrain_xyzh = state.terrain_xyzh;
    auto &terrain_ext_xyzh = state.terrain_ext_xyzh;
    auto &planner_terrain_xyzh = state.planner_terrain_xyzh;
    auto &latest_dynamic_clusters = state.latest_dynamic_clusters;
    auto &traversability_grid = state.traversability_grid;
    auto &last_terrain_map_receive_s = state.last_terrain_map_receive_s;
    auto &last_terrain_ext_receive_s = state.last_terrain_ext_receive_s;
    auto &last_traversability_receive_s = state.last_traversability_receive_s;
    auto &last_odom_s = state.last_odom_s;
    auto &teleop_receive_time = state.teleop_receive_time;
    auto &teleop_received = state.teleop_received;
    auto &last_plan = state.last_plan;
    auto &last_local = state.last_local;
    auto &last_teleop = state.last_teleop;
    auto &frames = state.frames;
    auto &last_global_path = state.last_global_path;
    auto &last_local_path = state.last_local_path;
    auto &last_local_planner_debug = state.last_local_planner_debug;
    auto &control_authority = state.control_authority;
    auto &inspection_executor = state.inspection_executor;
    auto &operator_resume_required = state.operator_resume_required;
    auto &path_echo = state.path_echo;
    auto &odom_generation = state.odom_generation;
    auto &frame_epoch = state.frame_epoch;
    auto &path_count = state.path_count;
    auto &cmd_vel_count = state.cmd_vel_count;
    auto &autonomy_request_not_before_s = state.autonomy_request_not_before_s;

    EstopLatchStore estop_latch_store(cfg.estop_latch_file);
    if (const auto persisted_estop = estop_latch_store.load()) {
      control_authority.latchEstop(*persisted_estop);
      std::fprintf(stderr, "nav_native: restored persisted software estop: %s\n",
                   persisted_estop->c_str());
    }
    auto active_map_identity = [&]() -> std::optional<std::pair<std::string, std::int64_t>> {
      if (!active_inspection_map_cache) {
        return std::nullopt;
      }
      const auto snapshot = active_inspection_map_cache->snapshot(steadySeconds());
      if (!snapshot.fresh || !snapshot.identity) {
        return std::nullopt;
      }
      return std::pair<std::string, std::int64_t>{snapshot.identity->map_id,
                                                  snapshot.identity->version};
    };
    CommandIngressController command_ingress;
    OperatorMotionAuthority operator_motion_authority;
    const bool operator_motion_interface_enabled =
        cfg.control_mode == ControlMode::Teleop || cfg.control_mode == ControlMode::TeleopAvoid ||
        (cfg.control_mode == ControlMode::Autonomy && cfg.allow_teleop_takeover);
    RollingSegmentLifecycle rolling_segment;
    InspectionRuntimeController inspection_runtime(inspection_executor);

    ControlLoopHealthConfig control_loop_health_config;
    control_loop_health_config.period_ms = 1000.0 / std::max(1.0, cfg.tick_hz);
    ControlLoopHealth control_loop_health(control_loop_health_config);
    ControlLoopRuntimeGuardConfig control_loop_guard_config;
    control_loop_guard_config.recovery_confirmation_samples =
        static_cast<std::size_t>(std::max(1.0, std::ceil(cfg.tick_hz)));
    ControlLoopRuntimeGuard control_loop_guard(control_loop_guard_config);
    auto control_loop_guard_latched = [&]() {
      const auto state = control_loop_guard.snapshot().state;
      return state == ControlLoopRuntimeGuardState::kLatched ||
             state == ControlLoopRuntimeGuardState::kRecovered;
    };
    NavStatusPublisherActions nav_status_actions;
    nav_status_actions.sample_planner = [&](TimingDiagnostics &status_timing) {
      const auto inputs = computePlannerInputs(
          cfg, obstacle_merge_config, traversability_grid, last_traversability_receive_s,
          obstacle_xyzh, terrain_xyzh, last_terrain_map_receive_s, terrain_ext_xyzh,
          last_terrain_ext_receive_s, planner_terrain_xyzh, status_timing);
      const std::size_t obstacle_points = !cfg.check_obstacle ? 0 : planner_terrain_xyzh.size() / 4;
      return StatusPlannerSample{
          inputs.traversability_fresh, inputs.terrain_map_fresh,
          inputs.terrain_ext_fresh,    obstacle_points,
          &planner_terrain_xyzh,
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
                 "max_rays=%zu min_hits=%d obstacle_shares=%.2f/%.2f/%.2f max_points=%zu\n",
                 cfg.obstacle_voxel_size_m, cfg.live_obstacle_decay_s,
                 cfg.live_obstacle_inflation_radius_m, cfg.live_obstacle_ray_clearing ? 1 : 0,
                 cfg.live_obstacle_ray_clear_max_range_m, cfg.live_obstacle_ray_clearing_interval_s,
                 cfg.live_obstacle_max_clearing_rays, cfg.live_obstacle_min_hits,
                 cfg.obstacle_registered_share, cfg.obstacle_terrain_share,
                 cfg.obstacle_terrain_ext_share, cfg.max_obstacle_points);
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

    TimingDiagnostics *current_timing = nullptr;
    std::uint64_t last_zero_output_sequence = 0U;
    std::uint64_t last_zero_source_wall_ns = 0U;
    auto record_final_output_publish_failure = [&](const std::string &reason) {
      control_authority.holdOperatorTakeover();
      operator_resume_required = true;
      last_teleop.published = false;
      frames.last_error = reason;
      nav_status.requestImmediate();
    };

    auto publish_zero_command = [&]() -> bool {
      if (!cfg.publish_cmd_vel) {
        return true;
      }
      const auto write_start = SteadyClock::now();
      const auto receipt = dds.writeCmdVelSequenced({});
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
      const auto receipt = dds.writeCmdVelSequenced({});
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
      nav.clearGlobalPath();
      path_echo.reset();
      last_global_path.clear();
      last_local_path.clear();
      last_local_planner_debug = {};
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

    RollingSegmentEffectActions rolling_segment_effect_actions;
    rolling_segment_effect_actions.activate_authority = [&]() {
      return control_authority.activatePath();
    };
    rolling_segment_effect_actions.install_path =
        [&](const RollingSegmentInstallPathEffect &value) {
          const auto path = rollingSegmentPathToNav(value.path, value.map_z);
          nav.setGlobalPath(path);
          path_echo.arm(path, value.stamp_s);
          last_global_path = path;
          last_local_path.clear();
          last_local_planner_debug = {};
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
          dds.writeGlobalPath(path, value.stamp_s);
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
      return dds.writeExplorationSegmentAck(ack);
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
      return dds.writeExplorationSegmentStatus(status);
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
      navigation_state.observe(status);
      dds.writeNavigationGoalStatus(status.task_id.c_str(), status.request_id.c_str(), status.state,
                                    status.goal_epoch, status.reason.c_str());
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
        nav.setGlobalPath(activation.path, activation.goal_yaw, activation.tolerance->position_m,
                          activation.tolerance->yaw_rad);
      } else {
        nav.setGlobalPath(activation.path, activation.goal_yaw);
      }
      const auto write_start = SteadyClock::now();
      path_echo.arm(activation.path, activation.stamp_s);
      dds.writeGlobalPath(activation.path, activation.stamp_s);
      if (current_timing != nullptr) {
        current_timing->dds_write_ms += elapsedMs(write_start);
      }
      last_global_path = activation.path;
      last_local_path.clear();
      last_local_planner_debug = {};
      (void)control_authority.activatePath();
      last_local = LocalDiagnostics{};
    };
    GoalPlanController goal_plan(std::move(global_planner), std::move(goal_plan_actions));
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
      path_echo.reset();
      if (clear_motion) {
        (void)inspection_executor.Pause(reason);
        control_authority.holdOperatorTakeover();
        operator_resume_required = true;
        goal_plan.invalidateForHold(reason);
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

    NavInputStateProjectorActions input_projector_actions;
    input_projector_actions.on_epoch_reset = reset_navigation_epoch;
    input_projector_actions.on_rolling_snapshot_invalidated = [&](const std::string &reason) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentObserveInvalidInput{reason, {}}));
      frames.last_error = rolling_segment.snapshot().input_error;
    };
    NavInputStateProjectorConfig input_projector_config;
    input_projector_config.source_transform_max_gap_s = source_transform_max_gap_s;
    input_projector_config.cloud_pose_max_gap_s = cfg.cloud_pose_max_gap_s;
    input_projector_config.driver_control_max_age_s = cfg.driver_control_max_age_s;
    input_projector_config.sensor_offset = {
        cfg.sensor_offset_x_m,
        cfg.sensor_offset_y_m,
        cfg.sensor_offset_z_m,
    };
    input_projector_config.check_obstacle = cfg.check_obstacle;
    input_projector_config.max_obstacle_points = cfg.max_obstacle_points;
    NavInputStateProjector input_projector(state, input_gate, pose_buffer, map_odom_buffer,
                                           live_obstacles, std::move(input_projector_config),
                                           std::move(input_projector_actions));

    StopConfirmationConfig stop_confirmation_config;
    stop_confirmation_config.timeout = std::chrono::milliseconds(
        static_cast<std::int64_t>(std::llround(cfg.stop_confirmation_timeout_s * 1000.0)));
    auto wait_for_stop_confirmation = [&](std::uint64_t output_sequence) {
      if (output_sequence != last_zero_output_sequence || last_zero_source_wall_ns == 0U) {
        return StopConfirmationState::TimedOut;
      }
      StopConfirmation confirmation(dds.producerBootId(), output_sequence, last_zero_source_wall_ns,
                                    StopConfirmation::Clock::now(), stop_confirmation_config);
      OdometrySpeedMonitor stop_speed_monitor(OdometrySpeedEvidence::PoseDerivedPlanar);
      while (true) {
        dds.drainDriverControlState([&](const lingtu_dds_DriverControlState &msg) {
          input_projector.projectDriverControl(msg, SteadyClock::now());
          confirmation.observeDriverAck(stringValue(msg.accepted_producer_boot_id),
                                        msg.accepted_output_sequence, msg.last_command_accepted,
                                        headerStampNanoseconds(msg.header));
        });
        dds.drainOdometry([&](const lingtu_dds_Odometry &msg) {
          const double stamp_s = headerStampSeconds(msg.header);
          const std::string frame_id = headerFrameId(msg.header);
          const auto &position = msg.pose.pose.position;
          const auto &linear = msg.twist.twist.linear;
          const double linear_speed = stop_speed_monitor.observe(
              stamp_s, frame_id, position.x, position.y, position.z, linear.x, linear.y, linear.z);
          const auto &angular = msg.twist.twist.angular;
          const double angular_speed =
              std::sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z);
          confirmation.observeQuietOdometry(stamp_s, linear_speed, angular_speed);
        });
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
    motion_stop_actions.cancel_control = [&]() { control_authority.cancel(); };
    motion_stop_actions.stop_control = [&]() { control_authority.stop(); };
    motion_stop_actions.latch_estop = [&](const std::string &reason) {
      control_authority.latchEstop(reason);
    };
    motion_stop_actions.clear_control_estop = [&]() { return control_authority.clearEstop(true); };
    motion_stop_actions.resume_autonomy = [&]() { return control_authority.resumeAutonomy(); };
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
    motion_stop_actions.persist_estop_latch = [&](const std::string &reason) {
      return estop_latch_store.persist(reason);
    };
    motion_stop_actions.clear_persisted_estop_latch = [&]() { return estop_latch_store.clear(); };
    motion_stop_actions.publish_zero = publish_zero_command;
    motion_stop_actions.last_output_sequence = [&]() { return dds.lastOutputSequence(); };
    motion_stop_actions.publish_sequenced_zero = publish_sequenced_zero_command;
    motion_stop_actions.confirm_zero = [&](std::uint64_t output_sequence) {
      return wait_for_stop_confirmation(output_sequence);
    };
    motion_stop_actions.clear_global_path = [&]() { nav.clearGlobalPath(); };
    MotionStopCoordinator motion_stop(cfg.publish_cmd_vel, std::move(motion_stop_actions));
    InspectionCommandActions inspection_command_actions;
    inspection_command_actions.route_source_available = [&]() {
      return inspection_store != nullptr;
    };
    inspection_command_actions.active_map = [&]() -> std::optional<InspectionActiveMap> {
      const auto identity = active_map_identity();
      if (!identity) {
        return std::nullopt;
      }
      return InspectionActiveMap{identity->first, identity->second};
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
    inspection_command_actions.clear_motion = [&](const std::string &reason) {
      return motion_stop.clearEndpointMotion(reason);
    };
    inspection_command_actions.publish_ack = [&](const InspectionCommandAck &ack) {
      return dds.writeInspectionAck(ack.request_id.c_str(), ack.kind, ack.accepted,
                                    ack.reason.c_str(), ack.run_id.c_str());
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

    TeleopTickActions teleop_tick_actions;
    teleop_tick_actions.teleop_receive_age_s = teleop_receive_age_s;
    teleop_tick_actions.steady_now_s = steadySeconds;
    teleop_tick_actions.compute_planner_inputs = [&](TimingDiagnostics &tick_timing) {
      const auto inputs = computePlannerInputs(
          cfg, obstacle_merge_config, traversability_grid, last_traversability_receive_s,
          obstacle_xyzh, terrain_xyzh, last_terrain_map_receive_s, terrain_ext_xyzh,
          last_terrain_ext_receive_s, planner_terrain_xyzh, tick_timing);
      return TeleopTickPlannerInputs{
          inputs.traversability_fresh,
          inputs.traversability_view,
          inputs.planner_obstacles_ptr,
      };
    };
    teleop_tick_actions.tick_teleop_intent =
        [&](const nav_kernel::Pose &pose, const nav_kernel::Twist &intent, const float *obstacles,
            int obstacle_count, double now_s,
            lingtu::nav::plan::TraversabilityGridView traversability) {
          return nav.tickTeleopIntent(pose, intent, obstacles, obstacle_count, now_s,
                                      traversability);
        };
    teleop_tick_actions.stop_linear_motion = [&]() { nav.stopLinearMotion(); };
    TeleopTickController teleop_tick(std::move(teleop_tick_actions));

    AutonomyTickActions autonomy_tick_actions;
    autonomy_tick_actions.steady_now_s = steadySeconds;
    autonomy_tick_actions.compute_planner_inputs = [&](TimingDiagnostics &tick_timing) {
      const auto inputs = computePlannerInputs(
          cfg, obstacle_merge_config, traversability_grid, last_traversability_receive_s,
          obstacle_xyzh, terrain_xyzh, last_terrain_map_receive_s, terrain_ext_xyzh,
          last_terrain_ext_receive_s, planner_terrain_xyzh, tick_timing);
      return AutonomyTickPlannerInputs{
          inputs.traversability_fresh,
          inputs.traversability_view,
          inputs.planner_obstacles_ptr,
      };
    };
    autonomy_tick_actions.tick_autonomy =
        [&](const nav_kernel::Pose &pose, const float *obstacles, int obstacle_count, double now_s,
            lingtu::nav::plan::TraversabilityGridView traversability) {
          const lingtu::nav::plan::PlannerObservationView observation{
              state.frame_epoch, state.cloud_generation, state.traversability_generation,
              state.last_odom_s, state.last_cloud_s,     state.last_traversability_s,
          };
          return nav.tick(pose, obstacles, obstacle_count, now_s, traversability, observation);
        };
    autonomy_tick_actions.evaluate_path_safety = lingtu::nav::endpoint::evaluateAutonomyPathSafety;
    autonomy_tick_actions.evaluate_command_safety = evaluateCommandSafety;
    autonomy_tick_actions.stop_linear_motion = [&]() { nav.stopLinearMotion(); };
    AutonomyTickController autonomy_tick(std::move(autonomy_tick_actions));

    EndpointLoopContext loop_ctx{
        cfg,
        gate_cfg,
        safety_config,
        operator_motion_interface_enabled,
        dds,
        state,
        nav,
        inspection_status_writer,
        input_projector,
        goal_plan,
        motion_stop,
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
        active_map_identity,
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
