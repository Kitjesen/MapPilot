#include "active_octomap_gate.hpp"
#include "active_occupancy_gate.hpp"
#include "control_authority.hpp"
#include "estop_latch_store.hpp"
#include "input_gate.hpp"
#include "global_plan_task.hpp"
#include "live_obstacle_layer.hpp"
#include "nav_dds_runtime.hpp"
#include "nav_endpoint_config.hpp"
#include "nav_endpoint_messages.hpp"
#include "nav_loop.hpp"
#include "transform_buffer.hpp"
#include "nav_status_writer.hpp"
#include "octoplanner3d_core.hpp"
#include "teleop_safety.hpp"

#include "lingtu/maps/store.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav/inspection/store.hpp"

#include "lingtu_slam.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

std::atomic_bool g_running{true};
using lingtu::nav::endpoint::CliConfig;
using lingtu::nav::endpoint::ActiveOctomapGate;
using lingtu::nav::endpoint::ActiveOccupancyGate;
using lingtu::nav::endpoint::CloudSyncDiagnostics;
using lingtu::nav::endpoint::CommandDiagnostics;
using lingtu::nav::endpoint::ControlAuthority;
using lingtu::nav::endpoint::EstopLatchStore;
using lingtu::nav::endpoint::ControlMode;
using lingtu::nav::endpoint::DdsRuntime;
using lingtu::nav::endpoint::DynamicCluster;
using lingtu::nav::endpoint::FrameDiagnostics;
using lingtu::nav::endpoint::GlobalPlanContext;
using lingtu::nav::endpoint::GlobalPlanTask;
using lingtu::nav::endpoint::GlobalPlannerBackend;
using lingtu::nav::endpoint::globalPlanStaleReason;
using lingtu::nav::endpoint::globalPlannerBackendName;
using lingtu::nav::endpoint::InputGate;
using lingtu::nav::endpoint::InputGateConfig;
using lingtu::nav::endpoint::InputSnapshot;
using lingtu::nav::endpoint::InputGateState;
using lingtu::nav::endpoint::LocalizationHealthSample;
using lingtu::nav::endpoint::OdometrySpeedMonitor;
using lingtu::nav::endpoint::LiveObstacleLayer;
using lingtu::nav::endpoint::LiveObstacleLayerConfig;
using lingtu::nav::endpoint::LocalDiagnostics;
using lingtu::nav::endpoint::ObstacleMergeConfig;
using lingtu::nav::endpoint::PathEcho;
using lingtu::nav::endpoint::PlanDiagnostics;
using lingtu::nav::endpoint::TransformBuffer;
using lingtu::nav::endpoint::RigidTransform;
using lingtu::nav::endpoint::SensorOrigin;
using lingtu::nav::endpoint::SourceStampDecision;
using lingtu::nav::endpoint::StatusSnapshotFileWriter;
using lingtu::nav::endpoint::StatusWriterConfig;
using lingtu::nav::endpoint::TeleopDiagnostics;
using lingtu::nav::endpoint::TimingDiagnostics;
using lingtu::nav::endpoint::TraversabilityGrid;
using lingtu::nav::endpoint::arbitrateTeleopCommand;
using lingtu::nav::endpoint::buildPlannerObstacleCloud;
using lingtu::nav::endpoint::cloudToXyzh;
using lingtu::nav::endpoint::commandSafetyConfig;
using lingtu::nav::endpoint::controlModeName;
using lingtu::nav::endpoint::evaluateCommandSafety;
using lingtu::nav::endpoint::decodeGoal;
using lingtu::nav::endpoint::decodeGrid;
using lingtu::nav::endpoint::decodeLocalizationHealth;
using lingtu::nav::endpoint::decodePath;
using lingtu::nav::endpoint::decodeTwist;
using lingtu::nav::endpoint::headerFrameId;
using lingtu::nav::endpoint::headerStampSeconds;
using lingtu::nav::endpoint::sourceStampError;
using lingtu::nav::endpoint::sourceStampPredates;
using lingtu::nav::endpoint::inputGateConfig;
using lingtu::nav::endpoint::linearSpeed;
using lingtu::nav::endpoint::mapOdomTransformFromTf;
using lingtu::nav::endpoint::parseArgs;
using lingtu::nav::endpoint::runWithActiveOctomap;
using lingtu::nav::endpoint::runWithActiveOccupancy;
using lingtu::nav::endpoint::teleopSafetyConfig;
using lingtu::nav::endpoint::textData;
using lingtu::nav::endpoint::toNavPath;
using lingtu::nav::endpoint::toPose;
using lingtu::nav::endpoint::transformPose;
using lingtu::nav::endpoint::vecDistance;
using lingtu::nav::endpoint::writeStatusSnapshot;
using CommandKind = lingtu::message::NavigationCommandKind;
using lingtu::message::isKnownNavigationCommandKind;
using lingtu::message::navigationCommandKindName;

void stopSignal(int) {
  g_running = false;
}

using SteadyClock = std::chrono::steady_clock;

// Collision clearance is expanded once by LocalPlanner footprint checks.
constexpr double kLayerInflationM = 0.0;
constexpr double kMapOdomJumpTranslationM = 0.50;
constexpr double kMapOdomJumpYawRad = 0.25;
constexpr double kSourceClockRebaseThresholdS = 0.35;

bool inspectionPostArrivalState(
    lingtu::nav::inspection::RunState state) noexcept {
  using RunState = lingtu::nav::inspection::RunState;
  return state == RunState::kSettling || state == RunState::kDwelling ||
         state == RunState::kActionPending;
}

bool localizationGateBlocked(
    const InputGateState& state,
    const InputGateConfig& config) noexcept {
  if (!config.require_localization_health) return false;
  const double age_s = state.localization_health_age_s;
  return !std::isfinite(age_s) ||
         age_s < -config.future_tolerance_s ||
         (config.localization_health_max_age_s > 0.0 &&
          age_s > config.localization_health_max_age_s) ||
         !state.localization_healthy || state.localization_state.empty() ||
         !lingtu::nav::endpoint::isHealthyLocalizationState(
             state.localization_state) ||
         lingtu::nav::endpoint::isCatastrophicLocalizationReason(
             state.localization_reason);
}


double elapsedMs(
    SteadyClock::time_point start,
    SteadyClock::time_point end = SteadyClock::now()) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

double steadySeconds() {
  const auto now = SteadyClock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

SensorOrigin sensorOriginFromBody(const RigidTransform& body, const CliConfig& cfg) {
  const auto offset = lingtu::nav::endpoint::rotatePoint(
      body.rotation,
      {cfg.sensor_offset_x_m, cfg.sensor_offset_y_m, cfg.sensor_offset_z_m});
  return SensorOrigin{
      body.translation.x + offset.x,
      body.translation.y + offset.y,
      body.translation.z + offset.z,
      true,
  };
}

std::string stringValue(const char* value) {
  return value == nullptr ? std::string{} : std::string(value);
}

struct CommandAckRecord {
  CommandKind kind{CommandKind::Goal};
  bool accepted{false};
  std::string reason;
};

struct InspectionAckRecord {
  lingtu::nav::inspection::CommandKind kind{
      lingtu::nav::inspection::CommandKind::kStart};
  bool accepted{false};
  std::string reason;
  std::string run_id;
};

lingtu_dds_PoseStamped inspectionGoalMessage(
    const lingtu::nav::inspection::Point& point,
    double stamp_s) {
  lingtu_dds_PoseStamped goal{};
  goal.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  goal.header.stamp.nanosec = static_cast<std::uint32_t>(
      (stamp_s - static_cast<double>(goal.header.stamp.sec)) * 1e9);
  goal.header.frame_id = const_cast<char*>("map");
  goal.pose.position.x = point.x_m;
  goal.pose.position.y = point.y_m;
  goal.pose.position.z = point.z_m;
  const double half_yaw = point.has_yaw ? point.yaw_rad * 0.5 : 0.0;
  goal.pose.orientation.z = std::sin(half_yaw);
  goal.pose.orientation.w = std::cos(half_yaw);
  return goal;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    const CliConfig cfg = parseArgs(argc, argv);
    auto active_octomap_gate = std::make_shared<ActiveOctomapGate>(cfg.map_root);
    auto active_occupancy_gate = std::make_shared<ActiveOccupancyGate>(cfg.map_root);
    std::unique_ptr<lingtu::maps::MapStore> inspection_map_store;
    std::unique_ptr<lingtu::nav::inspection::Store> inspection_store;
    if (!cfg.map_root.empty()) {
      inspection_map_store = std::make_unique<lingtu::maps::MapStore>(
          lingtu::maps::MapStoreConfig{cfg.map_root});
      inspection_store = std::make_unique<lingtu::nav::inspection::Store>(cfg.map_root);
    }
    if (cfg.control_mode == ControlMode::Autonomy && !cfg.map_path.empty()) {
      if (cfg.global_planner == GlobalPlannerBackend::Far) {
        auto preflight = active_occupancy_gate->prepare(cfg.map_path);
        if (!preflight.ok()) {
          throw std::runtime_error(
              "active occupancy failed native Maps preflight: " + preflight.reason);
        }
      } else {
        auto preflight = active_octomap_gate->prepare(cfg.map_path);
        if (!preflight.ok()) {
          throw std::runtime_error(
              "active OctoMap failed native Maps preflight: " + preflight.reason);
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
    StatusWriterConfig status_cfg;
    status_cfg.control_mode = controlModeName(cfg.control_mode);
    status_cfg.domain_id = cfg.domain_id;
    status_cfg.tick_hz = cfg.tick_hz;
    status_cfg.max_speed_mps = cfg.nav_max_speed_mps;
    status_cfg.max_accel_mps2 = cfg.nav_max_accel_mps2;
    status_cfg.nominal_dt_s = 1.0 / cfg.tick_hz;
    status_cfg.publish_cmd_vel = cfg.publish_cmd_vel;
    status_cfg.check_obstacle = cfg.check_obstacle;
    status_cfg.use_traversability_cost = cfg.use_traversability_cost;
    status_cfg.allow_teleop_takeover = cfg.allow_teleop_takeover;
    status_cfg.teleop_local_planner = cfg.teleop_local_planner;
    status_cfg.teleop_planner_horizon_m = cfg.teleop_planner_horizon_m;
    status_cfg.teleop_planner_max_deviation_deg =
        cfg.teleop_planner_max_deviation_deg;
    status_cfg.allow_legacy_motion_inputs = cfg.allow_legacy_motion_inputs;
    status_cfg.path_library_dir = cfg.path_library_dir;
    status_cfg.map_path = cfg.map_path;
    status_cfg.global_planner = globalPlannerBackendName(cfg.global_planner);
    status_cfg.status_file = cfg.status_file;
    status_cfg.max_obstacle_points = cfg.max_obstacle_points;
    status_cfg.local_planner_debug_candidate_limit =
        cfg.local_planner_debug_candidate_limit;
    status_cfg.local_map_debug_point_limit = cfg.local_map_debug_point_limit;
    status_cfg.obstacle_voxel_size_m = cfg.obstacle_voxel_size_m;
    status_cfg.obstacle_registered_share = cfg.obstacle_registered_share;
    status_cfg.obstacle_terrain_share = cfg.obstacle_terrain_share;
    status_cfg.obstacle_terrain_ext_share = cfg.obstacle_terrain_ext_share;
    status_cfg.sensor_offset_x_m = cfg.sensor_offset_x_m;
    status_cfg.sensor_offset_y_m = cfg.sensor_offset_y_m;
    status_cfg.sensor_offset_z_m = cfg.sensor_offset_z_m;
    status_cfg.live_obstacle_decay_s = cfg.live_obstacle_decay_s;
    status_cfg.live_obstacle_inflation_radius_m = cfg.live_obstacle_inflation_radius_m;
    status_cfg.live_obstacle_layer_inflation_radius_m = kLayerInflationM;
    status_cfg.live_obstacle_ray_clear_max_range_m = cfg.live_obstacle_ray_clear_max_range_m;
    status_cfg.live_obstacle_ray_clearing_interval_s =
        cfg.live_obstacle_ray_clearing_interval_s;
    status_cfg.live_obstacle_max_clearing_rays = cfg.live_obstacle_max_clearing_rays;
    status_cfg.live_obstacle_min_hits = cfg.live_obstacle_min_hits;
    status_cfg.live_obstacle_ray_clearing = cfg.live_obstacle_ray_clearing;
    status_cfg.odom_max_age_s = cfg.odom_max_age_s;
    status_cfg.tf_max_age_s = cfg.tf_max_age_s;
    status_cfg.cloud_max_age_s = cfg.cloud_max_age_s;
    status_cfg.cloud_pose_max_gap_s = cfg.cloud_pose_max_gap_s;
    status_cfg.input_future_tolerance_s = cfg.input_future_tolerance_s;
    status_cfg.input_recovery_frames = cfg.input_recovery_frames;
    status_cfg.octoplanner_options = cfg.octoplanner_options;
    status_cfg.far_options = cfg.far_options;
    StatusSnapshotFileWriter status_snapshot_writer(cfg.status_file);
    const InputGateConfig gate_cfg = inputGateConfig(cfg);
    status_cfg.input_require_odom = gate_cfg.require_odom;
    status_cfg.input_require_cloud = gate_cfg.require_cloud;
    status_cfg.input_require_traversability = gate_cfg.require_traversability;
    status_cfg.input_require_localization_health =
        gate_cfg.require_localization_health;
    status_cfg.input_require_driver_control = gate_cfg.require_driver_control;
    status_cfg.traversability_max_age_s = gate_cfg.traversability_max_age_s;
    status_cfg.localization_health_max_age_s =
        gate_cfg.localization_health_max_age_s;
    status_cfg.driver_control_max_age_s = gate_cfg.driver_control_max_age_s;
    InputGate input_gate(gate_cfg);
    TransformBuffer pose_buffer;
    TransformBuffer map_odom_buffer;
    const double source_transform_max_gap_s = cfg.tf_max_age_s > 0.0
        ? std::min(cfg.cloud_pose_max_gap_s, cfg.tf_max_age_s)
        : cfg.cloud_pose_max_gap_s;
    DdsRuntime dds(cfg.domain_id, cfg.allow_legacy_motion_inputs);

    const auto safety_config = commandSafetyConfig(cfg);
    lingtu::nav::plan::NavLoopConfig nav_config;
    nav_config.path_library_dir = cfg.path_library_dir;
    nav_config.max_speed = cfg.nav_max_speed_mps;
    nav_config.corridor_lookahead_m = cfg.corridor_lookahead_m;
    nav_config.teleop_intent_horizon_m = cfg.teleop_planner_horizon_m;
    nav_config.teleop_intent_max_deviation_deg =
        cfg.teleop_planner_max_deviation_deg;
    nav_config.local_planner.autonomySpeed = cfg.nav_max_speed_mps;
    nav_config.local_planner.maxSpeed = 1.0;
    nav_config.local_planner.vehicleLength = cfg.vehicle_length_m;
    nav_config.local_planner.vehicleWidth = cfg.vehicle_width_m;
    // NavLoop receives the map->body pose.  The LiDAR extrinsic is already
    // applied when sensor-origin geometry is constructed and must not shift
    // the body pose a second time inside LocalPlannerCore.
    nav_config.local_planner.sensorOffsetX = 0.0;
    nav_config.local_planner.sensorOffsetY = 0.0;
    nav_config.local_planner.obstacleHeightMax =
        cfg.local_planner_obstacle_height_max_m;
    nav_config.local_planner.footprintPadding = safety_config.obstacle_margin_m;
    nav_config.local_planner.useTraversabilityCost = cfg.use_traversability_cost;
    nav_config.local_planner.traversabilityHardCost = cfg.traversability_hard_cost;
    nav_config.local_planner.traversabilitySoftCost = cfg.traversability_soft_cost;
    nav_config.local_planner.traversabilityWeight = cfg.traversability_weight;
    nav_config.local_planner.debugCandidateLimit = static_cast<int>(
        cfg.local_planner_debug_candidate_limit);
    nav_config.path_follower.maxSpeed = cfg.nav_max_speed_mps;
    nav_config.path_follower.maxAccel = cfg.nav_max_accel_mps2;
    nav_config.path_follower.nominalDt = 1.0 / cfg.tick_hz;

    lingtu::nav::plan::NavLoop nav(nav_config);
    if ((cfg.control_mode == ControlMode::Autonomy || cfg.teleop_local_planner) &&
        !nav.configure()) {
      throw std::runtime_error("failed to load local planner path library: " + cfg.path_library_dir);
    }
    auto octomap_planner =
        std::make_shared<octoplanner3d::runtime::PlannerSession>();
    auto far_planner =
        std::make_shared<lingtu::nav::plan::far::FarPlanner>(cfg.far_options);
    GlobalPlanTask global_plan_task(
        [active_octomap_gate,
         active_occupancy_gate,
         octomap_planner,
         far_planner,
         planner_backend = cfg.global_planner,
         configured_map_path = cfg.map_path](
            const auto& request,
            const auto& cancel_check) {
          if (planner_backend == GlobalPlannerBackend::Far) {
            return runWithActiveOccupancy(
                *active_occupancy_gate,
                configured_map_path,
                *far_planner,
                request,
                cancel_check);
          }
          return runWithActiveOctomap(
              *active_octomap_gate,
              configured_map_path,
              request,
              cancel_check,
              [octomap_planner](
                  const auto& map_path,
                  const auto& map_identity,
                  const auto& plan_request,
                  const auto& cancel) {
                return octomap_planner->run(
                    map_path, map_identity, plan_request, cancel);
              });
        });

    std::optional<nav_kernel::Pose> odom_body;
    std::optional<nav_kernel::Pose> map_body;
    std::optional<RigidTransform> odom_body_transform;
    std::optional<RigidTransform> map_body_transform;
    std::optional<RigidTransform> map_odom_tf;
    std::vector<float> obstacle_xyzh;
    std::vector<float> terrain_xyzh;
    std::vector<float> terrain_ext_xyzh;
    std::vector<float> planner_terrain_xyzh;
    const std::vector<float> empty_obstacles;
    std::vector<DynamicCluster> latest_dynamic_clusters;
    bool obstacle_snapshot_dirty = false;
    SensorOrigin last_sensor_origin;
    TraversabilityGrid traversability_grid;
    double last_terrain_map_s = 0.0;
    double last_terrain_map_receive_s = 0.0;
    double last_terrain_ext_s = 0.0;
    double last_terrain_ext_receive_s = 0.0;
    double last_traversability_s = 0.0;
    double last_traversability_receive_s = 0.0;
    double last_odom_s = 0.0;
    double last_odom_receive_s = 0.0;
    double last_odom_linear_speed_mps = 0.0;
    double last_odom_angular_speed_radps = 0.0;
    OdometrySpeedMonitor odom_speed_monitor;
    double last_tf_s = 0.0;
    double last_tf_receive_s = 0.0;
    double last_cloud_s = 0.0;
    double last_cloud_receive_s = 0.0;
    double map_odom_epoch_start_s = 0.0;
    LocalizationHealthSample localization_health;
    double localization_health_receive_s = 0.0;
    double driver_control_stamp_s = 0.0;
    SteadyClock::time_point driver_control_receive_time{};
    bool driver_control_received = false;
    bool driver_control_ready = false;
    std::string driver_control_reason{"not_received"};
    bool driver_authority_previous = false;
    SteadyClock::time_point teleop_receive_time{};
    bool teleop_received = false;
    bool odom_requires_tf = true;
    InputGateState input_gate_state;
    PlanDiagnostics last_plan;
    LocalDiagnostics last_local;
    TeleopDiagnostics last_teleop;
    FrameDiagnostics frames;
    CommandDiagnostics command_diagnostics;
    std::vector<nav_kernel::Vec3> last_global_path;
    std::vector<nav_kernel::Vec3> last_local_path;
    nav_kernel::LocalPlannerDebugSnapshot last_local_planner_debug;
    ControlAuthority control_authority;
    lingtu::nav::inspection::Executor inspection_executor;
    std::optional<lingtu::nav::inspection::Point> active_inspection_point;
    bool submitting_inspection_goal = false;
    double next_inspection_status_s = 0.0;
    double next_inspection_map_check_s = 0.0;
    bool operator_resume_required = false;
    EstopLatchStore estop_latch_store(cfg.estop_latch_file);
    if (const auto persisted_estop = estop_latch_store.load()) {
      control_authority.latchEstop(*persisted_estop);
      std::fprintf(
          stderr,
          "nav_native: restored persisted software estop: %s\n",
          persisted_estop->c_str());
    }
    PathEcho path_echo;
    std::uint64_t odom_count = 0;
    std::uint64_t odom_generation = 0;
    std::uint64_t inspection_arrival_odom_generation = 0;
    std::uint64_t tf_count = 0;
    std::uint64_t tf_generation = 0;
    std::uint64_t frame_epoch = 0;
    std::uint64_t goal_count = 0;
    std::uint64_t goal_epoch = 0;
    std::uint64_t cancel_count = 0;
    std::uint64_t map_clearing_count = 0;
    std::uint64_t cloud_clearing_count = 0;
    std::uint64_t cloud_count = 0;
    std::uint64_t cloud_generation = 0;
    CloudSyncDiagnostics cloud_sync;
    std::uint64_t terrain_map_count = 0;
    std::uint64_t terrain_map_ext_count = 0;
    std::uint64_t traversability_count = 0;
    std::uint64_t traversability_generation = 0;
    std::uint64_t localization_health_generation = 0;
    std::uint64_t driver_control_generation = 0;
    std::uint64_t teleop_cmd_count = 0;
    std::uint64_t teleop_output_count = 0;
    std::uint64_t teleop_stop_count = 0;
    std::uint64_t teleop_slow_count = 0;
    std::uint64_t teleop_limited_count = 0;
    std::uint64_t path_count = 0;
    std::uint64_t plan_fail_count = 0;
    std::uint64_t output_count = 0;
    std::uint64_t cmd_vel_count = 0;
    double autonomy_request_not_before_s = 0.0;
    std::unordered_map<std::string, CommandAckRecord> command_ack_cache;
    std::deque<std::string> command_ack_order;
    constexpr std::size_t kCommandAckCacheLimit = 128;
    std::unordered_map<std::string, InspectionAckRecord> inspection_ack_cache;
    std::deque<std::string> inspection_ack_order;
    constexpr std::size_t kInspectionAckCacheLimit = 128;
    auto active_map_identity = [&]()
        -> std::optional<std::pair<std::string, std::int64_t>> {
      if (!inspection_map_store) return std::nullopt;
      const auto record = inspection_map_store->GetActiveMap();
      if (!record) return std::nullopt;
      return std::pair<std::string, std::int64_t>{record->map_id, record->version};
    };
    auto remember_inspection_ack = [&](const std::string& request_id,
                                       lingtu::nav::inspection::CommandKind kind,
                                       bool accepted,
                                       const std::string& reason,
                                       const std::string& run_id) {
      if (request_id.empty()) return;
      if (inspection_ack_cache.find(request_id) == inspection_ack_cache.end()) {
        inspection_ack_order.push_back(request_id);
      }
      inspection_ack_cache[request_id] = {kind, accepted, reason, run_id};
      while (inspection_ack_order.size() > kInspectionAckCacheLimit) {
        inspection_ack_cache.erase(inspection_ack_order.front());
        inspection_ack_order.pop_front();
      }
    };
    auto remember_command_ack = [&](const std::string& request_id,
                                    CommandKind kind,
                                    bool accepted,
                                    const std::string& reason) {
      if (request_id.empty()) {
        return;
      }
      if (command_ack_cache.find(request_id) == command_ack_cache.end()) {
        command_ack_order.push_back(request_id);
      }
      command_ack_cache[request_id] = CommandAckRecord{kind, accepted, reason};
      while (command_ack_order.size() > kCommandAckCacheLimit) {
        command_ack_cache.erase(command_ack_order.front());
        command_ack_order.pop_front();
      }
    };
    auto write_command_ack = [&](const std::string& request_id,
                                 CommandKind kind,
                                 bool accepted,
                                 const std::string& reason) {
      dds.writeCommandAck(
          request_id.c_str(), kind, accepted, reason.c_str());
      ++command_diagnostics.ack_sent;
      if (!accepted) {
        ++command_diagnostics.rejected;
      }
      command_diagnostics.last_request_id = request_id;
      command_diagnostics.last_kind = navigationCommandKindName(kind);
      command_diagnostics.last_accepted = accepted;
      command_diagnostics.last_reason = reason;
      remember_command_ack(request_id, kind, accepted, reason);
    };
    double next_status = steadySeconds() + cfg.status_s;
    auto clear_planner_terrain_inputs = [&]() {
      terrain_xyzh.clear();
      terrain_ext_xyzh.clear();
      planner_terrain_xyzh.clear();
      traversability_grid = TraversabilityGrid{};
      last_terrain_map_s = 0.0;
      last_terrain_map_receive_s = 0.0;
      last_terrain_ext_s = 0.0;
      last_terrain_ext_receive_s = 0.0;
      last_traversability_s = 0.0;
      last_traversability_receive_s = 0.0;
    };
    auto driver_control_receive_age_s = [&]() -> double {
      if (!driver_control_received) {
        return std::numeric_limits<double>::infinity();
      }
      return std::chrono::duration<double>(
          SteadyClock::now() - driver_control_receive_time).count();
    };
    auto driver_control_blocker = [&]() -> std::string {
      if (driver_control_stamp_s <= 0.0 || !driver_control_received) {
        return "driver_control_missing";
      }
      const double age_s = driver_control_receive_age_s();
      if (cfg.driver_control_max_age_s > 0.0 &&
          age_s > cfg.driver_control_max_age_s) {
        return "driver_control_stale";
      }
      if (!driver_control_ready) {
        return driver_control_reason.empty()
            ? "driver_control_not_ready"
            : std::string("driver_control_") + driver_control_reason;
      }
      return {};
    };
    auto teleop_receive_age_s = [&]() -> double {
      if (!teleop_received) {
        return std::numeric_limits<double>::infinity();
      }
      return std::chrono::duration<double>(
          SteadyClock::now() - teleop_receive_time).count();
    };

    std::fprintf(
        stderr,
        "navd: domain=%d tick_hz=%.1f path_library=%s\n",
        cfg.domain_id,
        cfg.tick_hz,
        cfg.path_library_dir.c_str());
    std::fprintf(
        stderr,
        "nav_native: publish_cmd_vel=%d check_obstacle=%d use_traversability_cost=%d traversability_max_age_s=%.2f terrain_map_max_age_s=%.2f status_file=%s\n",
        cfg.publish_cmd_vel ? 1 : 0,
        cfg.check_obstacle ? 1 : 0,
        cfg.use_traversability_cost ? 1 : 0,
        cfg.traversability_max_age_s,
        cfg.terrain_map_max_age_s,
        cfg.status_file.empty() ? "(disabled)" : cfg.status_file.c_str());
    std::fprintf(
        stderr,
        "nav_native: obstacle_voxel_size_m=%.3f live_obstacle_decay_s=%.2f live_inflation=%.2f ray_clearing=%d ray_range=%.1f ray_interval=%.2f max_rays=%zu min_hits=%d obstacle_shares=%.2f/%.2f/%.2f max_points=%zu\n",
        cfg.obstacle_voxel_size_m,
        cfg.live_obstacle_decay_s,
        cfg.live_obstacle_inflation_radius_m,
        cfg.live_obstacle_ray_clearing ? 1 : 0,
        cfg.live_obstacle_ray_clear_max_range_m,
        cfg.live_obstacle_ray_clearing_interval_s,
        cfg.live_obstacle_max_clearing_rays,
        cfg.live_obstacle_min_hits,
        cfg.obstacle_registered_share,
        cfg.obstacle_terrain_share,
        cfg.obstacle_terrain_ext_share,
        cfg.max_obstacle_points);
    std::fprintf(
        stderr,
        "nav_native: lidar_sensor_offset_m=(%.3f, %.3f, %.3f) ray_origin=body_pose_plus_configured_extrinsics\n",
        cfg.sensor_offset_x_m,
        cfg.sensor_offset_y_m,
        cfg.sensor_offset_z_m);
    if (cfg.map_path.empty()) {
      std::fprintf(
          stderr,
          "nav_native: no planner map configured for %s; "
          "goal_pose will be rejected instead of direct fallback\n",
          globalPlannerBackendName(cfg.global_planner));
    } else {
      std::fprintf(
          stderr,
          "nav_native: global_planner=%s planner_map=%s\n",
          globalPlannerBackendName(cfg.global_planner),
          cfg.map_path.c_str());
    }

    const auto tick_period = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / std::max(1.0, cfg.tick_hz)));
    auto next_tick = SteadyClock::now();
    TimingDiagnostics last_timing;

    while (g_running) {
      const auto loop_start = SteadyClock::now();
      next_tick += tick_period;
      TimingDiagnostics timing;
      timing.motion_update_last_ms = last_timing.motion_update_last_ms;
      timing.obstacle_snapshot_last_ms = last_timing.obstacle_snapshot_last_ms;
      auto publish_zero_command = [&]() -> bool {
        if (!cfg.publish_cmd_vel) {
          return true;
        }
        const auto write_start = SteadyClock::now();
        const bool written = dds.writeCmdVel({});
        timing.dds_write_ms += elapsedMs(write_start);
        ++cmd_vel_count;
        return written;
      };
      auto clear_endpoint_motion = [&](const std::string& reason) -> bool {
        ++goal_epoch;
        global_plan_task.cancel();
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
        last_teleop.published = cfg.publish_cmd_vel;
        last_teleop.stopped = true;
        last_teleop.slowed = false;
        last_teleop.output = {};
        last_teleop.reason = reason;
        const bool zero_published = publish_zero_command();
        next_status = 0.0;
        return zero_published;
      };
      auto reset_navigation_epoch = [&](double epoch_start_s,
                                        const char* reason,
                                        bool clear_motion) {
        ++frame_epoch;
        InputSnapshot epoch_boundary;
        epoch_boundary.odom_generation = odom_generation;
        epoch_boundary.tf_generation = tf_generation;
        epoch_boundary.cloud_generation = cloud_generation;
        epoch_boundary.traversability_generation = traversability_generation;
        epoch_boundary.localization_health_generation = localization_health_generation;
        epoch_boundary.driver_control_generation = driver_control_generation;
        input_gate.beginRecoveryFrom(epoch_boundary);

        pose_buffer.clear();
        map_odom_buffer.clear();
        map_odom_tf.reset();
        odom_body.reset();
        odom_body_transform.reset();
        map_body.reset();
        map_body_transform.reset();
        clear_planner_terrain_inputs();
        live_obstacles.clear();
        latest_dynamic_clusters.clear();
        obstacle_xyzh.clear();
        obstacle_snapshot_dirty = false;
        last_sensor_origin = SensorOrigin{};
        last_tf_s = 0.0;
        last_tf_receive_s = 0.0;
        last_odom_s = 0.0;
        last_odom_receive_s = 0.0;
        last_odom_linear_speed_mps = 0.0;
        last_odom_angular_speed_radps = 0.0;
        inspection_arrival_odom_generation = odom_generation;
        odom_speed_monitor.reset();
        last_cloud_s = 0.0;
        last_cloud_receive_s = 0.0;
        cloud_sync = CloudSyncDiagnostics{};
        localization_health = LocalizationHealthSample{};
        localization_health_receive_s = 0.0;
        path_echo.reset();
        map_odom_epoch_start_s = epoch_start_s;
        frames.last_error = reason;
        if (clear_motion) {
          (void)inspection_executor.Pause(reason);
          (void)clear_endpoint_motion(reason);
        }
      };
      const auto input_start = SteadyClock::now();
      dds.drainTf([&](const lingtu_dds_TFMessage& msg) {
        const double receive_steady_s = steadySeconds();
        const auto tf = mapOdomTransformFromTf(msg);
        if (!tf || !std::isfinite(tf->stamp_s) || tf->stamp_s <= 0.0) {
          frames.last_error = "map_odom_tf_invalid";
          ++tf_count;
          return;
        }
        const auto tf_stamp_decision =
            lingtu::nav::endpoint::classifySourceOrder(
              last_tf_s,
              tf->stamp_s,
              kSourceClockRebaseThresholdS);
        if (tf_stamp_decision == SourceStampDecision::kReject) {
          frames.last_error =
              map_odom_epoch_start_s > 0.0 &&
                  tf->stamp_s + 1e-9 < map_odom_epoch_start_s
              ? "map_odom_tf_before_current_epoch"
              : "map_odom_tf_out_of_order";
          ++tf_count;
          return;
        }
        const bool map_frame_jump = map_odom_tf &&
            lingtu::nav::endpoint::transformJumpExceeds(
                *map_odom_tf,
                *tf,
                kMapOdomJumpTranslationM,
                kMapOdomJumpYawRad);
        if (map_frame_jump) {
          reset_navigation_epoch(tf->stamp_s, "map_frame_jump", true);
        } else if (tf_stamp_decision == SourceStampDecision::kClockRebase) {
          ++frames.clock_rebases;
          reset_navigation_epoch(tf->stamp_s, "source_clock_rebase", false);
        }

        map_odom_tf = *tf;
        map_odom_buffer.push(tf->stamp_s, *tf);
        last_tf_s = tf->stamp_s;
        last_tf_receive_s = receive_steady_s;
        ++tf_generation;
        if (!map_frame_jump && odom_body) {
          map_body = transformPose(*map_odom_tf, *odom_body);
        }
        if (!map_frame_jump && odom_body_transform) {
          map_body_transform = composeTransforms(
              *map_odom_tf,
              *odom_body_transform);
        }
        ++tf_count;
      });
      dds.drainOdometry([&](const lingtu_dds_Odometry& msg) {
        const double receive_steady_s = steadySeconds();
        const double odom_stamp_s = headerStampSeconds(msg.header);
        const std::string odom_frame = headerFrameId(msg.header);
        if (odom_frame != "map" && odom_frame != "odom") {
          ++frames.odom_rejected;
          frames.last_error = odom_frame.empty() ? "odom_frame_empty" : "odom_frame_unsupported";
          return;
        }
        const auto odom_stamp_decision =
            lingtu::nav::endpoint::classifySourceOrder(
              last_odom_s,
              odom_stamp_s,
              kSourceClockRebaseThresholdS);
        if (odom_stamp_decision == SourceStampDecision::kReject) {
          ++frames.odom_rejected;
          frames.last_error = "odom_stamp_invalid";
          return;
        }
        if (odom_stamp_decision == SourceStampDecision::kClockRebase) {
          ++frames.clock_rebases;
          reset_navigation_epoch(odom_stamp_s, "source_clock_rebase", false);
        }
        if (odom_frame == "odom" && map_odom_epoch_start_s > 0.0 &&
            odom_stamp_s + 1e-9 < map_odom_epoch_start_s) {
          ++frames.odom_rejected;
          frames.last_error = "odom_before_current_map_epoch";
          return;
        }
        const auto next_odom_body_transform =
            lingtu::nav::endpoint::rigidTransformFromOdometry(msg);
        if (!next_odom_body_transform.valid) {
          ++frames.odom_rejected;
          frames.last_error = "odom_pose_nonfinite";
          return;
        }
        const bool map_frame = odom_frame == "map";
        std::optional<RigidTransform> next_map_body_transform;
        if (map_frame) {
          next_map_body_transform = next_odom_body_transform;
        } else {
          const auto source_map_odom =
              map_odom_buffer.sample(odom_stamp_s, source_transform_max_gap_s);
          if (!source_map_odom) {
            ++frames.odom_rejected;
            frames.last_error = "odom_tf_gap_exceeded";
            return;
          }
          next_map_body_transform = composeTransforms(
              *source_map_odom,
              next_odom_body_transform);
        }
        const nav_kernel::Vec3 current_odom_position{
            next_odom_body_transform.translation.x,
            next_odom_body_transform.translation.y,
            next_odom_body_transform.translation.z,
        };
        const auto& reported_velocity = msg.twist.twist.linear;
        last_odom_linear_speed_mps = odom_speed_monitor.observe(
            odom_stamp_s,
            odom_frame,
            current_odom_position.x,
            current_odom_position.y,
            current_odom_position.z,
            reported_velocity.x,
            reported_velocity.y,
            reported_velocity.z);
        const auto& reported_angular_velocity = msg.twist.twist.angular;
        last_odom_angular_speed_radps = std::sqrt(
            reported_angular_velocity.x * reported_angular_velocity.x +
            reported_angular_velocity.y * reported_angular_velocity.y +
            reported_angular_velocity.z * reported_angular_velocity.z);
        odom_body = toPose(msg);
        odom_body_transform = next_odom_body_transform;
        odom_requires_tf = !map_frame;
        last_odom_s = odom_stamp_s;
        last_odom_receive_s = receive_steady_s;
        if (map_frame) {
          map_body = *odom_body;
          map_body_transform = next_map_body_transform;
        } else {
          map_body_transform = next_map_body_transform;
          map_body = transformPose(*map_body_transform, nav_kernel::Pose{});
        }
        if (map_body_transform) {
          pose_buffer.push(odom_stamp_s, *map_body_transform);
        }
        ++odom_generation;
        ++odom_count;
      });
      dds.drainDriverControlState(
          [&](const lingtu_dds_DriverControlState& msg) {
            const double stamp_s = headerStampSeconds(msg.header);
            const auto stamp_decision =
                lingtu::nav::endpoint::classifySourceOrder(
                    driver_control_stamp_s,
                    stamp_s,
                    kSourceClockRebaseThresholdS);
            if (stamp_decision == SourceStampDecision::kReject) {
              frames.last_error = "driver_control_stamp_invalid";
              return;
            }
            const std::string fsm = stringValue(msg.fsm);
            const std::string owner = stringValue(msg.owner);
            const std::string owner_id = stringValue(msg.owner_id);
            driver_control_stamp_s = stamp_s;
            driver_control_receive_time = SteadyClock::now();
            driver_control_received = true;
            driver_control_reason = stringValue(msg.reason);
            driver_control_ready = msg.connected && msg.ready &&
                msg.motors_enabled && !msg.critical_fault &&
                msg.lease_valid && owner == "grpc" &&
                owner_id == "lingtu-driver" &&
                (fsm == "standing" || fsm == "walking");
            if (!driver_control_ready && driver_control_reason.empty()) {
              if (!msg.connected) {
                driver_control_reason = "disconnected";
              } else if (!msg.motors_enabled) {
                driver_control_reason = "motors_disabled";
              } else if (msg.critical_fault) {
                driver_control_reason = "motor_fault";
              } else if (!msg.lease_valid || owner != "grpc" ||
                         owner_id != "lingtu-driver") {
                driver_control_reason = "lease_not_owned";
              } else if (fsm != "standing" && fsm != "walking") {
                driver_control_reason = "fsm_not_ready";
              } else {
                driver_control_reason = "not_ready";
              }
            }
            ++driver_control_generation;
          });
      auto submit_goal = [&](const lingtu_dds_PoseStamped& msg)
          -> std::pair<bool, std::string> {
        ++goal_count;
        last_plan = PlanDiagnostics{};
        last_plan.seen = true;
        if (inspection_executor.active() && !submitting_inspection_goal) {
          last_plan.reason = "inspection_run_active";
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        if (!submitting_inspection_goal) {
          active_inspection_point.reset();
        }
        if (!control_authority.motionAllowed()) {
          last_plan.reason = "estop_latched";
          ++frames.goal_rejected;
          frames.last_error = last_plan.reason;
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        if (control_authority.operatorTakeoverLatched()) {
          last_plan.reason = "operator_takeover_resume_required";
          ++frames.goal_rejected;
          frames.last_error = last_plan.reason;
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        if (cfg.control_mode != ControlMode::Autonomy) {
          last_plan.reason =
              std::string("goal_not_allowed_in_") + controlModeName(cfg.control_mode);
          ++frames.goal_rejected;
          frames.last_error = last_plan.reason;
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        if (const std::string blocker = driver_control_blocker();
            !blocker.empty()) {
          last_plan.reason = blocker;
          ++frames.goal_rejected;
          frames.last_error = last_plan.reason;
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        if (sourceStampPredates(
                headerStampSeconds(msg.header),
                autonomy_request_not_before_s)) {
          last_plan.reason = "goal_predates_autonomy_resume";
          ++frames.goal_rejected;
          frames.last_error = last_plan.reason;
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        const auto goal = decodeGoal(msg, map_odom_tf);
        if (!goal.ok()) {
          last_plan.reason = goal.error;
          ++frames.goal_rejected;
          frames.last_error = goal.error;
          ++plan_fail_count;
          std::fprintf(stderr, "nav_native: reject goal, %s\n", goal.error.c_str());
          return {false, last_plan.reason};
        }
        last_plan.goal = goal.value.position;
        if (!map_body) {
          last_plan.reason = odom_body ? "map_odom_tf_not_ready" : "odometry_not_ready";
          ++plan_fail_count;
          std::fprintf(
              stderr,
              "nav_native: reject goal, %s\n",
              odom_body ? "map->odom tf is not ready" : "odometry is not ready");
          return {false, last_plan.reason};
        }
        last_plan.start = map_body->position;
        if (cfg.map_path.empty()) {
          last_plan.reason = cfg.global_planner == GlobalPlannerBackend::Far
              ? "active_occupancy_not_configured"
              : "active_octomap_not_configured";
          ++plan_fail_count;
          std::fprintf(
              stderr,
              "nav_native: reject goal, active planner map is not configured for %s\n",
              globalPlannerBackendName(cfg.global_planner));
          return {false, last_plan.reason};
        }
        if (global_plan_task.busy()) {
          last_plan.reason = "global_planner_busy";
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        ++goal_epoch;
        GlobalPlanContext context;
        context.start = map_body->position;
        context.goal = last_plan.goal;
        context.goal_yaw = goal.value.yaw;
        context.goal_epoch = goal_epoch;
        context.frame_epoch = frame_epoch;
        context.request.start = {
            map_body->position.x,
            map_body->position.y,
            map_body->position.z,
        };
        context.request.goal = {
            last_plan.goal.x,
            last_plan.goal.y,
            last_plan.goal.z,
        };
        context.request.options = cfg.octoplanner_options;
        if (!global_plan_task.start(std::move(context))) {
          last_plan.reason = "global_planner_busy";
          ++plan_fail_count;
          return {false, last_plan.reason};
        }
        last_plan.reason = "planning";
        std::fprintf(
            stderr,
            "nav_native: %s planning started\n",
            globalPlannerBackendName(cfg.global_planner));
        return {true, "planning_started"};
      };
      dds.drainLegacyGoals([&](const lingtu_dds_PoseStamped& msg) {
        (void)submit_goal(msg);
      });
      dds.drainCloud([&](const lingtu_dds_PointCloud2& msg) {
        ++cloud_count;
        const auto convert_start = SteadyClock::now();
        const double receive_steady_s = steadySeconds();
        const double cloud_now = nowSeconds();
        const double cloud_stamp_s = headerStampSeconds(msg.header);
        cloud_sync.last_stamp_age_s = cloud_now - cloud_stamp_s;
        const auto cloud_stamp_decision =
            lingtu::nav::endpoint::classifySourceOrder(
              last_cloud_s,
              cloud_stamp_s,
              kSourceClockRebaseThresholdS);
        if (cloud_stamp_decision == SourceStampDecision::kReject) {
          ++cloud_sync.stamp_rejected;
          frames.last_error = "cloud_stamp_invalid";
          return;
        }
        if (cloud_stamp_decision == SourceStampDecision::kClockRebase) {
          ++frames.clock_rebases;
        }
        const std::string cloud_frame = headerFrameId(msg.header);
        if (cloud_frame == "odom" && map_odom_epoch_start_s > 0.0 &&
            cloud_stamp_s + 1e-9 < map_odom_epoch_start_s) {
          ++cloud_sync.pose_rejected;
          frames.last_error = "cloud_before_current_map_epoch";
          return;
        }
        cloud_sync.last_pose_gap_s = pose_buffer.nearestGap(cloud_stamp_s);
        const auto cloud_pose = pose_buffer.sample(cloud_stamp_s, cfg.cloud_pose_max_gap_s);
        if (!cloud_pose) {
          ++cloud_sync.pose_rejected;
          return;
        }
        std::optional<RigidTransform> cloud_map_odom;
        if (cloud_frame == "odom") {
          cloud_map_odom =
              map_odom_buffer.sample(cloud_stamp_s, source_transform_max_gap_s);
          if (!cloud_map_odom) {
            ++cloud_sync.pose_rejected;
            frames.last_error = "cloud_tf_gap_exceeded";
            return;
          }
        }
        const auto xyzh = cloudToXyzh(msg, 0, cloud_pose, cloud_map_odom);
        timing.cloud_convert_ms += elapsedMs(convert_start);
        if (xyzh.empty()) {
          return;
        }
        const auto motion_start = SteadyClock::now();
        last_sensor_origin = sensorOriginFromBody(*cloud_pose, cfg);
        live_obstacles.updateFromScan(last_sensor_origin, xyzh, cloud_stamp_s);
        latest_dynamic_clusters = live_obstacles.dynamicClusters(32, cloud_stamp_s);
        timing.motion_update_last_ms = elapsedMs(motion_start);
        obstacle_snapshot_dirty = true;
        last_cloud_s = cloud_stamp_s;
        last_cloud_receive_s = receive_steady_s;
        ++cloud_generation;
      });
      dds.drainTerrainMap([&](const lingtu_dds_PointCloud2& msg) {
        const double receive_steady_s = steadySeconds();
        const double stamp_s = headerStampSeconds(msg.header);
        const auto stamp_decision =
            lingtu::nav::endpoint::classifySourceOrder(
              last_terrain_map_s,
              stamp_s,
              kSourceClockRebaseThresholdS);
        if (stamp_decision == SourceStampDecision::kReject) {
          frames.last_error = "terrain_map_stamp_invalid";
          return;
        }
        if (stamp_decision == SourceStampDecision::kClockRebase) {
          ++frames.clock_rebases;
        }
        const auto convert_start = SteadyClock::now();
        const auto xyzh = cloudToXyzh(msg, 0, map_body_transform, map_odom_tf);
        timing.cloud_convert_ms += elapsedMs(convert_start);
        if (!xyzh.empty()) {
          terrain_xyzh = std::move(xyzh);
          last_terrain_map_s = stamp_s;
          last_terrain_map_receive_s = receive_steady_s;
          ++terrain_map_count;
        }
      });
      dds.drainTerrainMapExt([&](const lingtu_dds_PointCloud2& msg) {
        const double receive_steady_s = steadySeconds();
        const double stamp_s = headerStampSeconds(msg.header);
        const auto stamp_decision =
            lingtu::nav::endpoint::classifySourceOrder(
              last_terrain_ext_s,
              stamp_s,
              kSourceClockRebaseThresholdS);
        if (stamp_decision == SourceStampDecision::kReject) {
          frames.last_error = "terrain_map_ext_stamp_invalid";
          return;
        }
        if (stamp_decision == SourceStampDecision::kClockRebase) {
          ++frames.clock_rebases;
        }
        const auto convert_start = SteadyClock::now();
        const auto xyzh = cloudToXyzh(msg, 0, map_body_transform, map_odom_tf);
        timing.cloud_convert_ms += elapsedMs(convert_start);
        if (!xyzh.empty()) {
          terrain_ext_xyzh = std::move(xyzh);
          last_terrain_ext_s = stamp_s;
          last_terrain_ext_receive_s = receive_steady_s;
          ++terrain_map_ext_count;
        }
      });
      dds.drainMapClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        clear_planner_terrain_inputs();
        live_obstacles.clear();
        latest_dynamic_clusters.clear();
        obstacle_snapshot_dirty = false;
        obstacle_xyzh.clear();
        ++map_clearing_count;
      });
      dds.drainCloudClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        clear_planner_terrain_inputs();
        live_obstacles.clear();
        latest_dynamic_clusters.clear();
        obstacle_snapshot_dirty = false;
        obstacle_xyzh.clear();
        ++cloud_clearing_count;
      });
      dds.drainLegacyGlobalPath([&](const lingtu_dds_Path& msg) {
        if (!control_authority.motionAllowed()) {
          ++frames.path_rejected;
          frames.last_error = "estop_latched";
          return;
        }
        if (control_authority.operatorTakeoverLatched()) {
          ++frames.path_rejected;
          frames.last_error = "operator_takeover_resume_required";
          return;
        }
        if (cfg.control_mode != ControlMode::Autonomy) {
          ++frames.path_rejected;
          frames.last_error =
              std::string("path_not_allowed_in_") + controlModeName(cfg.control_mode);
          return;
        }
        const std::string path_driver_blocker = driver_control_blocker();
        if (!path_driver_blocker.empty()) {
          ++frames.path_rejected;
          frames.last_error = path_driver_blocker;
          return;
        }
        const double path_stamp_s = headerStampSeconds(msg.header);
        if (sourceStampPredates(
                path_stamp_s,
                autonomy_request_not_before_s)) {
          ++frames.path_rejected;
          frames.last_error = "path_predates_autonomy_resume";
          return;
        }
        const auto decoded = decodePath(msg, map_odom_tf);
        if (!decoded.ok()) {
          ++frames.path_rejected;
          frames.last_error = decoded.error;
          std::fprintf(stderr, "nav_native: reject global path, %s\n", decoded.error.c_str());
          return;
        }
        const auto& path = decoded.value;
        if (path_echo.take(path, path_stamp_s, nowSeconds())) {
          return;
        }
        if (path.size() < 2) {
          ++frames.path_rejected;
          frames.last_error = "path_too_short";
          return;
        }
        nav.setGlobalPath(path);
        last_global_path = path;
        last_local_path.clear();
        last_local_planner_debug = {};
        (void)control_authority.activatePath();
        last_plan = PlanDiagnostics{};
        last_plan.seen = true;
        last_plan.accepted = true;
        last_plan.reached_goal = false;
        last_plan.reason = "external_global_path";
        last_plan.waypoints = path.size();
        if (map_body) {
          last_plan.start = map_body->position;
        }
        last_plan.goal = path.back();
        last_local = LocalDiagnostics{};
        ++path_count;
      });
      dds.drainTraversability([&](const lingtu_dds_OccupancyGrid& msg) {
        const double receive_steady_s = steadySeconds();
        const double stamp_s = headerStampSeconds(msg.header);
        const auto stamp_decision =
            lingtu::nav::endpoint::classifySourceOrder(
              last_traversability_s,
              stamp_s,
              kSourceClockRebaseThresholdS);
        if (stamp_decision == SourceStampDecision::kReject) {
          ++frames.grid_rejected;
          frames.last_error = "traversability_stamp_invalid";
          return;
        }
        if (stamp_decision == SourceStampDecision::kClockRebase) {
          ++frames.clock_rebases;
        }
        auto decoded = decodeGrid(msg);
        if (!decoded.ok()) {
          ++frames.grid_rejected;
          frames.last_error = decoded.error;
          return;
        }
        decoded.value.generation = ++traversability_generation;
        traversability_grid = std::move(decoded.value);
        last_traversability_s = stamp_s;
        last_traversability_receive_s = receive_steady_s;
        ++traversability_count;
      });
      dds.drainLocalizationHealth([&](const lingtu_dds_Text& msg) {
        const double receive_steady_s = steadySeconds();
        auto next_health = decodeLocalizationHealth(textData(msg));
        if (!next_health.valid) {
          frames.last_error = next_health.error;
          return;
        }
        const auto stamp_decision =
            lingtu::nav::endpoint::classifySourceOrder(
              localization_health.stamp_s,
              next_health.stamp_s,
              kSourceClockRebaseThresholdS);
        if (stamp_decision == SourceStampDecision::kReject) {
          frames.last_error = "localization_health_stamp_invalid";
          return;
        }
        if (stamp_decision == SourceStampDecision::kClockRebase) {
          ++frames.clock_rebases;
        }
        localization_health = std::move(next_health);
        localization_health_receive_s = receive_steady_s;
        ++localization_health_generation;
      });
      auto handle_cancel = [&](const std::string&) -> std::pair<bool, std::string> {
        ++cancel_count;
        control_authority.cancel();
        operator_resume_required = false;
        (void)inspection_executor.Cancel("navigation_cancelled");
        const bool zero_published = clear_endpoint_motion("cancelled");
        return {
            zero_published,
            zero_published ? "cancelled" : "zero_publish_failed",
        };
      };
      auto handle_stop = [&](const std::string&) -> std::pair<bool, std::string> {
        control_authority.stop();
        operator_resume_required = false;
        (void)inspection_executor.Cancel("navigation_stopped");
        const bool zero_published = clear_endpoint_motion("stopped");
        return {
            zero_published,
            zero_published ? "stopped" : "zero_publish_failed",
        };
      };
      auto handle_estop = [&](const std::string& reason) -> std::pair<bool, std::string> {
        control_authority.latchEstop(reason);
        operator_resume_required = false;
        (void)inspection_executor.Cancel("estop_latched");
        const bool persisted = estop_latch_store.persist(reason);
        const bool zero_published = clear_endpoint_motion("estop_latched");
        return {
            zero_published && persisted,
            !persisted
                ? "estop_latch_persist_failed_estop_remains_latched"
                : (zero_published ? "estop_latched"
                                  : "zero_publish_failed_estop_latched"),
        };
      };
      auto handle_clear_estop = [&](const lingtu_dds_Header& header)
          -> std::pair<bool, std::string> {
        const std::string stamp_error = sourceStampError(
            "clear_estop",
            headerStampSeconds(header),
            nowSeconds(),
            cfg.teleop_cmd_max_age_s,
            cfg.input_future_tolerance_s);
        if (!stamp_error.empty()) {
          return {false, stamp_error};
        }
        const bool zero_published = clear_endpoint_motion("estop_cleared");
        if (!zero_published) {
          return {false, "zero_publish_failed_estop_remains_latched"};
        }
        if (!estop_latch_store.clear()) {
          return {false, "estop_latch_clear_failed_estop_remains_latched"};
        }
        const bool cleared = control_authority.clearEstop(true);
        operator_resume_required = false;
        return {
            cleared,
            cleared ? "estop_cleared" : "estop_remains_latched",
        };
      };
      auto handle_resume_autonomy = [&](const lingtu_dds_Header& header)
          -> std::pair<bool, std::string> {
        if (cfg.control_mode != ControlMode::Autonomy) {
          return {
              false,
              std::string("resume_not_allowed_in_") +
                  controlModeName(cfg.control_mode),
          };
        }
        if (!control_authority.motionAllowed()) {
          return {false, "estop_latched"};
        }
        const std::string stamp_error = sourceStampError(
            "resume_autonomy",
            headerStampSeconds(header),
            nowSeconds(),
            cfg.teleop_cmd_max_age_s,
            cfg.input_future_tolerance_s);
        if (!stamp_error.empty()) {
          return {false, stamp_error};
        }
        if (!control_authority.operatorTakeoverLatched()) {
          return {true, "autonomy_already_ready"};
        }
        const bool zero_published =
            clear_endpoint_motion("autonomy_resume_ready");
        if (!zero_published) {
          return {false, "zero_publish_failed_takeover_remains_latched"};
        }
        if (!control_authority.resumeAutonomy()) {
          return {false, "estop_latched"};
        }
        autonomy_request_not_before_s = headerStampSeconds(header);
        operator_resume_required = false;
        return {true, "autonomy_resume_ready_reissue_goal"};
      };
      dds.drainLegacyCancel([&](const lingtu_dds_Text& msg) {
        (void)handle_cancel(textData(msg));
      });
      auto handle_teleop = [&](const lingtu_dds_TwistStamped& msg)
          -> std::pair<bool, std::string> {
        ++teleop_cmd_count;
        if (!control_authority.motionAllowed()) {
          ++frames.teleop_rejected;
          frames.last_error = "estop_latched";
          last_teleop.seen = true;
          last_teleop.fresh = false;
          last_teleop.published = false;
          last_teleop.stopped = true;
          last_teleop.limited = false;
          last_teleop.reason = frames.last_error;
          last_teleop.output = {};
          return {false, last_teleop.reason};
        }
        const auto decoded = decodeTwist(msg);
        if (!decoded.ok()) {
          ++frames.teleop_rejected;
          frames.last_error = decoded.error;
          const bool teleop_owns_control =
              cfg.control_mode != ControlMode::Autonomy ||
              control_authority.operatorTakeoverLatched() ||
              control_authority.teleopRequest().has_value();
          if (teleop_owns_control) {
            if (control_authority.operatorTakeoverLatched()) {
              control_authority.holdOperatorTakeover();
            } else {
              control_authority.stop();
            }
          }
          last_teleop.seen = true;
          last_teleop.fresh = false;
          last_teleop.published = false;
          last_teleop.stopped = true;
          last_teleop.limited = true;
          last_teleop.reason = decoded.error;
          last_teleop.output = {};
          if (teleop_owns_control && cfg.publish_cmd_vel) {
            publish_zero_command();
            ++teleop_output_count;
          }
          if (teleop_owns_control) {
            ++teleop_stop_count;
            ++teleop_limited_count;
          }
          return {false, last_teleop.reason};
        }
        const double receive_s = nowSeconds();
        const double source_stamp_s = headerStampSeconds(msg.header);
        const double source_age_s = receive_s - source_stamp_s;
        last_teleop.age_s = source_age_s;
        std::string stamp_error;
        if (!std::isfinite(source_stamp_s) || source_stamp_s <= 0.0) {
          stamp_error = "teleop_source_stamp_invalid";
        } else if (source_age_s < -cfg.input_future_tolerance_s) {
          stamp_error = "teleop_source_stamp_future";
        } else if (
            cfg.teleop_cmd_max_age_s > 0.0 &&
            source_age_s > cfg.teleop_cmd_max_age_s) {
          stamp_error = "teleop_source_stamp_stale";
        }
        if (!stamp_error.empty()) {
          std::fprintf(
              stderr,
              "nav_native: reject teleop source stamp, reason=%s age_s=%.6f\n",
              stamp_error.c_str(),
              source_age_s);
          ++frames.teleop_rejected;
          frames.last_error = stamp_error;
          const bool teleop_owns_control =
              cfg.control_mode != ControlMode::Autonomy ||
              control_authority.operatorTakeoverLatched() ||
              control_authority.teleopRequest().has_value();
          if (teleop_owns_control) {
            if (control_authority.operatorTakeoverLatched()) {
              control_authority.holdOperatorTakeover();
            } else {
              control_authority.stop();
            }
          }
          last_teleop.seen = true;
          last_teleop.fresh = false;
          last_teleop.published = teleop_owns_control && cfg.publish_cmd_vel;
          last_teleop.stopped = true;
          last_teleop.limited = true;
          last_teleop.reason = stamp_error;
          last_teleop.output = {};
          if (teleop_owns_control) {
            publish_zero_command();
            ++teleop_stop_count;
            ++teleop_limited_count;
            if (cfg.publish_cmd_vel) {
              ++teleop_output_count;
            }
          }
          return {false, stamp_error};
        }
        bool takeover_started = false;
        if (cfg.control_mode == ControlMode::Autonomy) {
          if (!cfg.allow_teleop_takeover) {
            ++frames.teleop_rejected;
            frames.last_error = "teleop_takeover_disabled";
            last_teleop.seen = true;
            last_teleop.fresh = false;
            last_teleop.published = false;
            last_teleop.stopped = true;
            last_teleop.limited = false;
            last_teleop.reason = frames.last_error;
            last_teleop.output = {};
            return {false, last_teleop.reason};
          }
          const bool already_owned =
              control_authority.operatorTakeoverLatched() ||
              control_authority.teleopRequest().has_value();
          const double takeover_threshold =
              std::max(1e-6, cfg.teleop_min_motion_speed_mps);
          const bool deliberate_input =
              linearSpeed(decoded.value) >= takeover_threshold ||
              std::abs(decoded.value.wz) >= takeover_threshold;
          if (!already_owned && !deliberate_input) {
            last_teleop.seen = true;
            last_teleop.fresh = false;
            last_teleop.published = false;
            last_teleop.stopped = true;
            last_teleop.limited = false;
            last_teleop.reason = "zero_teleop_did_not_take_over";
            last_teleop.request = decoded.value;
            last_teleop.output = {};
            return {true, last_teleop.reason};
          }
          if (!already_owned) {
            if (!control_authority.beginOperatorTakeover(
                    decoded.value, source_stamp_s)) {
              return {false, "estop_latched"};
            }
            operator_resume_required = true;
            if (!clear_endpoint_motion("operator_takeover")) {
              control_authority.holdOperatorTakeover();
              return {false, "zero_publish_failed_manual_hold_latched"};
            }
            (void)inspection_executor.Pause("operator_takeover");
            takeover_started = true;
          }
        }
        if (!takeover_started &&
            !control_authority.acceptTeleop(decoded.value, source_stamp_s)) {
          return {false, "estop_latched"};
        }
        teleop_receive_time = SteadyClock::now();
        teleop_received = true;
        last_teleop.seen = true;
        last_teleop.request = decoded.value;
        return {
            true,
            takeover_started ? "takeover_active_for_safety_arbitration"
                             : "accepted_for_safety_arbitration",
        };
      };
      dds.drainLegacyTeleopCmdVel([&](const lingtu_dds_TwistStamped& msg) {
        (void)handle_teleop(msg);
      });
      dds.drainCommandRequests([&](const lingtu_dds_NavigationCommandRequest& msg) {
        ++command_diagnostics.received;
        const std::string request_id = stringValue(msg.request_id);
        if (request_id.empty()) {
          write_command_ack(
              "",
              static_cast<CommandKind>(msg.kind),
              false,
              "command_request_id_empty");
          return;
        }
        if (!isKnownNavigationCommandKind(msg.kind)) {
          write_command_ack(
              request_id,
              static_cast<CommandKind>(msg.kind),
              false,
              "unknown_command_kind");
          return;
        }
        const auto kind = static_cast<CommandKind>(msg.kind);
        const auto cached = command_ack_cache.find(request_id);
        if (cached != command_ack_cache.end()) {
          if (cached->second.kind != kind) {
            dds.writeCommandAck(
                request_id.c_str(),
                kind,
                false,
                "duplicate_request_id_kind_mismatch");
            ++command_diagnostics.ack_sent;
            ++command_diagnostics.rejected;
            command_diagnostics.last_request_id = request_id;
            command_diagnostics.last_kind = navigationCommandKindName(kind);
            command_diagnostics.last_accepted = false;
            command_diagnostics.last_reason = "duplicate_request_id_kind_mismatch";
            return;
          }
          dds.writeCommandAck(
              request_id.c_str(),
              cached->second.kind,
              cached->second.accepted,
              cached->second.reason.c_str());
          ++command_diagnostics.ack_sent;
          ++command_diagnostics.replayed;
          command_diagnostics.last_request_id = request_id;
          command_diagnostics.last_kind = navigationCommandKindName(cached->second.kind);
          command_diagnostics.last_accepted = cached->second.accepted;
          command_diagnostics.last_reason = cached->second.reason;
          return;
        }
        std::pair<bool, std::string> result;
        if (kind == CommandKind::Goal) {
          lingtu_dds_PoseStamped goal{};
          goal.header = msg.header;
          goal.pose = msg.goal;
          result = submit_goal(goal);
        } else if (kind == CommandKind::Cancel) {
          result = handle_cancel(stringValue(msg.reason));
        } else if (kind == CommandKind::Teleop) {
          lingtu_dds_TwistStamped teleop{};
          teleop.header = msg.header;
          teleop.twist = msg.velocity;
          result = handle_teleop(teleop);
        } else if (kind == CommandKind::Stop) {
          const std::string reason = stringValue(msg.reason);
          result = reason == "client_clock_sync"
              ? std::pair<bool, std::string>{true, "clock_synchronized"}
              : handle_stop(reason);
        } else if (kind == CommandKind::Estop) {
          result = handle_estop(stringValue(msg.reason));
        } else if (kind == CommandKind::ClearEstop) {
          result = handle_clear_estop(msg.header);
        } else {
          result = handle_resume_autonomy(msg.header);
        }
        write_command_ack(request_id, kind, result.first, result.second);
      });
      dds.drainInspectionCommands([&](const lingtu_dds_InspectionCommandRequest& msg) {
        using InspectionCommand = lingtu::nav::inspection::CommandKind;
        const std::string request_id = stringValue(msg.request_id);
        const auto raw_kind = msg.kind;
        if (request_id.empty() || !lingtu::nav::inspection::IsKnownCommandKind(raw_kind)) {
          dds.writeInspectionAck(
              request_id.c_str(),
              static_cast<InspectionCommand>(raw_kind),
              false,
              request_id.empty() ? "inspection_request_id_empty" : "unknown_inspection_command",
              "");
          return;
        }
        const auto kind = static_cast<InspectionCommand>(raw_kind);
        const auto cached = inspection_ack_cache.find(request_id);
        if (cached != inspection_ack_cache.end()) {
          const bool kind_matches = cached->second.kind == kind;
          dds.writeInspectionAck(
              request_id.c_str(),
              kind,
              kind_matches && cached->second.accepted,
              kind_matches ? cached->second.reason.c_str()
                           : "duplicate_request_id_kind_mismatch",
              kind_matches ? cached->second.run_id.c_str() : "");
          return;
        }

        bool accepted = false;
        std::string reason;
        std::string run_id = inspection_executor.status().run_id;
        const auto active_map = active_map_identity();
        if (kind == InspectionCommand::kStart) {
          if (!inspection_store || !active_map) {
            reason = "active_map_unavailable";
          } else if (inspection_executor.active()) {
            reason = "inspection_run_active";
          } else {
            const std::string route_id = stringValue(msg.route_id);
            const auto route = inspection_store->Get(active_map->first, route_id);
            if (!route) {
              reason = "inspection_route_not_found";
            } else if (msg.route_revision != 0U && msg.route_revision != route->revision) {
              reason = "inspection_route_revision_mismatch";
            } else {
              run_id = request_id;
              accepted = inspection_executor.Start(
                  *route,
                  run_id,
                  active_map->first,
                  active_map->second,
                  nowSeconds(),
                  &reason);
              if (accepted) reason = "inspection_route_accepted";
            }
          }
        } else if (kind == InspectionCommand::kPause) {
          accepted = inspection_executor.Pause(stringValue(msg.reason));
          reason = accepted ? "inspection_paused" : "inspection_pause_not_allowed";
          if (accepted && !clear_endpoint_motion("inspection_paused")) {
            accepted = false;
            reason = "inspection_pause_zero_publish_failed";
          }
        } else if (kind == InspectionCommand::kResume) {
          if (control_authority.operatorTakeoverLatched()) {
            accepted = false;
            reason = "inspection_resume_requires_autonomy";
          } else {
            accepted = active_map && inspection_executor.Resume(
                active_map->first, active_map->second, nowSeconds());
            reason = accepted ? "inspection_resumed" : "inspection_resume_not_allowed";
          }
        } else {
          accepted = inspection_executor.Cancel(stringValue(msg.reason));
          reason = accepted ? "inspection_cancelled" : "inspection_cancel_not_allowed";
          if (accepted && !clear_endpoint_motion("inspection_cancelled")) {
            accepted = false;
            reason = "inspection_cancel_zero_publish_failed";
          }
        }
        remember_inspection_ack(request_id, kind, accepted, reason, run_id);
        dds.writeInspectionAck(
            request_id.c_str(), kind, accepted, reason.c_str(), run_id.c_str());
        next_inspection_status_s = 0.0;
      });
      const double input_now = steadySeconds();
      InputSnapshot input_snapshot;
      input_snapshot.now_s = input_now;
      input_snapshot.odom_stamp_s = last_odom_s;
      input_snapshot.odom_receive_s = last_odom_receive_s;
      input_snapshot.odom_generation = odom_generation;
      input_snapshot.odom_linear_speed_mps = last_odom_linear_speed_mps;
      input_snapshot.tf_stamp_s = last_tf_s;
      input_snapshot.tf_receive_s = last_tf_receive_s;
      input_snapshot.tf_generation = tf_generation;
      input_snapshot.cloud_stamp_s = last_cloud_s;
      input_snapshot.cloud_receive_s = last_cloud_receive_s;
      input_snapshot.cloud_generation = cloud_generation;
      input_snapshot.traversability_stamp_s = last_traversability_s;
      input_snapshot.traversability_receive_s = last_traversability_receive_s;
      input_snapshot.traversability_generation = traversability_generation;
      input_snapshot.localization_health_stamp_s = localization_health.stamp_s;
      input_snapshot.localization_health_receive_s = localization_health_receive_s;
      input_snapshot.localization_health_generation = localization_health_generation;
      const double driver_receive_age_s = driver_control_receive_age_s();
      input_snapshot.driver_control_stamp_s = driver_control_stamp_s;
      input_snapshot.driver_control_receive_s =
          driver_control_received && std::isfinite(driver_receive_age_s)
          ? input_now - std::max(0.0, driver_receive_age_s)
          : 0.0;
      input_snapshot.driver_control_generation = driver_control_generation;
      input_snapshot.odom_requires_tf = odom_requires_tf;
      input_snapshot.localization_healthy = localization_health.healthy;
      input_snapshot.localization_state = localization_health.state;
      input_snapshot.localization_reason = localization_health.reason;
      input_snapshot.driver_control_ready = driver_control_ready;
      input_snapshot.driver_control_reason = driver_control_reason;
      input_gate_state = input_gate.evaluate(input_snapshot);
      if (inspectionPostArrivalState(inspection_executor.status().state) &&
          localizationGateBlocked(input_gate_state, gate_cfg)) {
        if (inspection_executor.Pause(
                "inspection_localization_health_blocked")) {
          (void)clear_endpoint_motion(
              "inspection_post_arrival_localization_pause");
          next_inspection_status_s = 0.0;
        }
      }
      if (auto completion = global_plan_task.poll()) {
        const double plan_completion_now = nowSeconds();
        const auto& result = completion->result;
        last_plan = PlanDiagnostics{};
        last_plan.seen = true;
        last_plan.start = completion->context.start;
        last_plan.goal = completion->context.goal;
        last_plan.reached_goal = result.reached_goal;
        last_plan.goal_error_m = result.goal_error_m;
        last_plan.elapsed_ms = result.elapsed_ms;
        timing.global_plan_ms = result.elapsed_ms;
        std::optional<lingtu::nav::plan::MapIdentity> current_plan_map;
        std::string map_identity_error;
        if (completion->context.goal_epoch == goal_epoch &&
            completion->context.frame_epoch == frame_epoch &&
            completion->error.empty() && !result.cancelled) {
          if (cfg.global_planner == GlobalPlannerBackend::Far) {
            auto identity = active_occupancy_gate->currentIdentity(cfg.map_path);
            if (identity.ok()) {
              current_plan_map = std::move(identity.identity);
            } else {
              map_identity_error = std::move(identity.reason);
            }
          } else {
            auto identity = active_octomap_gate->currentIdentity(cfg.map_path);
            if (identity.ok()) {
              current_plan_map = std::move(identity.identity);
            } else {
              map_identity_error = std::move(identity.reason);
            }
          }
        }
        const std::string stale_reason = globalPlanStaleReason(
            *completion,
            goal_epoch,
            frame_epoch,
            current_plan_map);
        if (!stale_reason.empty()) {
          last_plan.reason = stale_reason;
          last_plan.accepted = false;
          frames.last_error = stale_reason;
          ++plan_fail_count;
          std::fprintf(
              stderr,
              "nav_native: discarded stale global plan reason=%s map_check=%s\n",
              stale_reason.c_str(),
              map_identity_error.empty() ? "ok" : map_identity_error.c_str());
          if (inspection_executor.active()) {
            inspection_executor.OnLegFailed(last_plan.reason, plan_completion_now);
          }
          (void)clear_endpoint_motion(last_plan.reason);
        } else if (!completion->error.empty()) {
          last_plan.reason = "global_planner_exception";
          ++plan_fail_count;
          std::fprintf(
              stderr,
              "nav_native: %s exception: %s\n",
              globalPlannerBackendName(cfg.global_planner),
              completion->error.c_str());
          if (inspection_executor.active()) {
            inspection_executor.OnLegFailed(last_plan.reason, plan_completion_now);
          }
        } else if (!result.ok || !result.reached_goal) {
          last_plan.reason = result.ok
            ? "goal_not_reached"
            : (result.failure_reason.empty() ? "global_planner_failed" : result.failure_reason);
          last_plan.waypoints = result.path.size();
          ++plan_fail_count;
          std::fprintf(
              stderr,
              "nav_native: %s rejected reason=%s ok=%d reached=%d waypoints=%zu goal_error=%.3f elapsed_ms=%.1f\n",
              globalPlannerBackendName(cfg.global_planner),
              last_plan.reason.c_str(),
              result.ok ? 1 : 0,
              result.reached_goal ? 1 : 0,
              result.path.size(),
              result.goal_error_m,
              result.elapsed_ms);
          if (inspection_executor.active()) {
            inspection_executor.OnLegFailed(last_plan.reason, plan_completion_now);
          }
        } else if (control_authority.operatorTakeoverLatched()) {
          last_plan.reason = "operator_takeover_discarded_plan";
          last_plan.accepted = false;
          if (inspection_executor.active()) {
            inspection_executor.Pause(last_plan.reason);
          }
        } else {
          auto global_path = toNavPath(result.path);
          if (global_path.empty()) {
            last_plan.reason = "empty_path";
            ++plan_fail_count;
            if (inspection_executor.active()) {
              inspection_executor.OnLegFailed(last_plan.reason, plan_completion_now);
            }
          } else {
            if (global_path.size() == 1) {
              if (vecDistance(completion->context.start, global_path.front()) > 0.02) {
                global_path.insert(global_path.begin(), completion->context.start);
              } else {
                global_path.push_back(completion->context.goal);
              }
            }
            last_plan.waypoints = global_path.size();
            const bool inspection_plan_ready =
                !inspection_executor.active() ||
                inspection_executor.OnPlanReady(plan_completion_now);
            if (!inspection_plan_ready) {
              last_plan.accepted = false;
              last_plan.reason = inspection_executor.status().reason;
              (void)clear_endpoint_motion(last_plan.reason);
              next_inspection_status_s = 0.0;
            } else {
              if (inspection_executor.active() && active_inspection_point) {
                nav.setGlobalPath(
                    global_path,
                    completion->context.goal_yaw,
                    active_inspection_point->position_tolerance_m,
                    active_inspection_point->yaw_tolerance_rad);
              } else {
                nav.setGlobalPath(global_path, completion->context.goal_yaw);
              }
              const auto write_start = SteadyClock::now();
              const double path_stamp_s = nowSeconds();
              path_echo.arm(global_path, path_stamp_s);
              dds.writeGlobalPath(global_path, path_stamp_s);
              timing.dds_write_ms += elapsedMs(write_start);
              last_global_path = global_path;
              last_local_path.clear();
              last_local_planner_debug = {};
              (void)control_authority.activatePath();
              last_plan.accepted = true;
              last_plan.reason = "accepted";
              last_local = LocalDiagnostics{};
              ++path_count;
              std::fprintf(
                  stderr,
                  "nav_native: %s path accepted waypoints=%zu elapsed_ms=%.1f reached=%d\n",
                  globalPlannerBackendName(cfg.global_planner),
                  last_plan.waypoints,
                  result.elapsed_ms,
                  result.reached_goal ? 1 : 0);
            }
          }
        }
      }
      const double inspection_now = nowSeconds();
      const auto inspection_state_before_sample = inspection_executor.status().state;
      dds.drainInspectionEvidenceResults(
          [&](const lingtu_dds_InspectionEvidenceResult& result) {
            if (lingtu::nav::endpoint::applyInspectionEvidenceResult(
                    inspection_executor, result, inspection_now)) {
              next_inspection_status_s = 0.0;
            }
          });
      if (inspection_state_before_sample ==
              lingtu::nav::inspection::RunState::kActionPending &&
          inspection_executor.status().state != inspection_state_before_sample) {
        (void)clear_endpoint_motion(inspection_executor.status().reason);
      }
      if (inspection_state_before_sample ==
          lingtu::nav::inspection::RunState::kSettling) {
        if (odom_generation != inspection_arrival_odom_generation) {
          inspection_arrival_odom_generation = odom_generation;
          (void)inspection_executor.OnArrivalSample(
              {
                  last_odom_s,
                  last_odom_linear_speed_mps,
                  last_odom_angular_speed_radps,
              },
              inspection_now);
        }
      } else {
        // Only odometry received after OnGoalReached may count toward settling.
        inspection_arrival_odom_generation = odom_generation;
      }
      const std::string inspection_reason_before_tick =
          inspection_executor.status().reason;
      if (inspection_executor.status().state ==
              lingtu::nav::inspection::RunState::kNavigating &&
          active_inspection_point && map_body &&
          control_authority.pathActive()) {
        (void)inspection_executor.OnNavigationProgress(
            std::hypot(
                active_inspection_point->x_m - map_body->position.x,
                active_inspection_point->y_m - map_body->position.y),
            inspection_now);
      }
      inspection_executor.Tick(inspection_now);
      const std::string& inspection_reason_after_tick =
          inspection_executor.status().reason;
      if (inspection_reason_after_tick != inspection_reason_before_tick &&
          (inspection_reason_after_tick == "planning_timeout" ||
           inspection_reason_after_tick == "navigation_stalled" ||
           inspection_reason_after_tick == "settling_timeout" ||
           inspection_reason_after_tick == "action_timeout")) {
        (void)clear_endpoint_motion(inspection_reason_after_tick);
      }
      if (inspection_executor.status().state != inspection_state_before_sample) {
        next_inspection_status_s = 0.0;
      }
      const auto inspection_state_before_map_check =
          inspection_executor.status().state;
      if (inspection_now >= next_inspection_map_check_s) {
        next_inspection_map_check_s = inspection_now + 1.0;
        const auto active_map = active_map_identity();
        if (active_map) {
          inspection_executor.OnMapChanged(active_map->first, active_map->second);
        } else if (inspection_executor.active()) {
          inspection_executor.OnMapChanged("", -1);
        }
        if (inspection_state_before_map_check !=
                inspection_executor.status().state &&
            inspection_executor.status().state ==
                lingtu::nav::inspection::RunState::kFailed &&
            inspection_executor.status().reason == "active_map_changed") {
          control_authority.stop();
          (void)clear_endpoint_motion("inspection_active_map_changed");
          next_inspection_status_s = 0.0;
        }
      }
      if (const auto point = inspection_executor.PendingGoal()) {
        if (!global_plan_task.busy()) {
          active_inspection_point = *point;
          submitting_inspection_goal = true;
          const auto result = submit_goal(inspectionGoalMessage(*point, inspection_now));
          submitting_inspection_goal = false;
          if (result.first) {
            if (!inspection_executor.OnPlanningStarted(inspection_now)) {
              (void)clear_endpoint_motion(inspection_executor.status().reason);
            }
          } else {
            inspection_executor.OnLegFailed(result.second, inspection_now);
          }
          next_inspection_status_s = 0.0;
        }
      }
      timing.input_callbacks_ms = elapsedMs(input_start);
      if (const auto action = inspection_executor.PendingAction()) {
        const auto* route = inspection_executor.route();
        if (route != nullptr && dds.inspectionEvidenceWorkerMatched()) {
          if (inspection_executor.OnActionStarted(
                  action->request_id, input_now)) {
            const bool published = dds.writeInspectionEvidenceRequest(
                *action,
                route->map_id,
                route->map_version,
                inspection_executor.status().deadline_s);
            if (!published) {
              (void)inspection_executor.OnActionResult(
                  action->request_id,
                  false,
                  "",
                  "evidence_request_publish_failed",
                  input_now);
              (void)clear_endpoint_motion("evidence_request_publish_failed");
            }
            next_inspection_status_s = 0.0;
          }
        }
      }
      const std::string driver_blocker = driver_control_blocker();
      const bool driver_authority_now = driver_blocker.empty();
      if (driver_authority_previous && !driver_authority_now) {
        control_authority.cancel();
        operator_resume_required = false;
        (void)inspection_executor.Cancel(
            std::string("driver_control_lost:") + driver_blocker);
        (void)clear_endpoint_motion(driver_blocker);
      }
      if (!driver_authority_now) {
        // Keep a zero sample fresh on the only hardware command topic. If the
        // driver is disconnected it drops the sample; after recovery the
        // latest command is still guaranteed to be zero until a new task is
        // explicitly submitted.
        (void)publish_zero_command();
      }
      driver_authority_previous = driver_authority_now;
      if (cfg.check_obstacle && obstacle_snapshot_dirty) {
        const auto snapshot_start = SteadyClock::now();
        obstacle_xyzh = live_obstacles.snapshot(cfg.max_obstacle_points, last_cloud_s);
        timing.obstacle_snapshot_last_ms = elapsedMs(snapshot_start);
        obstacle_snapshot_dirty = false;
      }

      if (control_authority.estopLatched()) {
        last_local.seen = true;
        last_local.active = false;
        last_local.near_field_stop = true;
        last_local.reason = "estop_latched";
        last_local.cmd_vel = {};
        last_teleop.fresh = false;
        last_teleop.published = cfg.publish_cmd_vel;
        last_teleop.stopped = true;
        last_teleop.output = {};
        last_teleop.reason = "estop_latched";
        publish_zero_command();
      }

      const auto teleop_start = SteadyClock::now();
      const auto& active_teleop_request = control_authority.teleopRequest();
      const bool path_active = control_authority.pathActive();
      if (active_teleop_request && !path_active && !input_gate_state.ready) {
        if (cfg.teleop_local_planner) {
          nav.stopLinearMotion();
        }
        last_teleop.seen = true;
        last_teleop.fresh = false;
        last_teleop.published = cfg.publish_cmd_vel;
        last_teleop.stopped = true;
        last_teleop.slowed = false;
        last_teleop.limited = true;
        last_teleop.reason = input_gate_state.reason;
        last_teleop.output = {};
        if (cfg.publish_cmd_vel) {
          const auto write_start = SteadyClock::now();
          dds.writeCmdVel({});
          timing.dds_write_ms += elapsedMs(write_start);
          ++cmd_vel_count;
          ++teleop_output_count;
        }
        ++teleop_stop_count;
        ++teleop_limited_count;
      } else if (active_teleop_request && !path_active) {
        const double tick_now = steadySeconds();
        const bool traversability_available = !traversability_grid.values.empty();
        const bool traversability_fresh =
            traversability_available &&
            (last_traversability_receive_s > 0.0) &&
            (cfg.traversability_max_age_s <= 0.0 ||
             tick_now - last_traversability_receive_s <= cfg.traversability_max_age_s);
        const double age_s = teleop_receive_age_s();
        auto precheck_config = teleopSafetyConfig(cfg);
        precheck_config.check_obstacle = false;
        precheck_config.use_traversability_cost = false;
        const auto precheck = arbitrateTeleopCommand(
            precheck_config,
            *active_teleop_request,
            age_s,
            map_body,
            empty_obstacles,
            TraversabilityGrid{},
            false);
        auto decision = precheck;
        const bool use_assisted_planner =
            cfg.teleop_local_planner && map_body && precheck.should_publish &&
            !precheck.stopped &&
            linearSpeed(precheck.cmd) >= cfg.teleop_min_motion_speed_mps;
        if (use_assisted_planner) {
          const bool terrain_map_fresh =
              !terrain_xyzh.empty() && last_terrain_map_receive_s > 0.0 &&
              (cfg.terrain_map_max_age_s <= 0.0 ||
               tick_now - last_terrain_map_receive_s <= cfg.terrain_map_max_age_s);
          const bool terrain_ext_fresh =
              !terrain_ext_xyzh.empty() && last_terrain_ext_receive_s > 0.0 &&
              (cfg.terrain_map_max_age_s <= 0.0 ||
               tick_now - last_terrain_ext_receive_s <= cfg.terrain_map_max_age_s);
          const std::vector<float>* planner_obstacles_ptr = nullptr;
          if (cfg.check_obstacle) {
            const auto merge_start = SteadyClock::now();
            buildPlannerObstacleCloud(
                planner_terrain_xyzh,
                obstacle_xyzh,
                terrain_xyzh,
                terrain_map_fresh,
                terrain_ext_xyzh,
                terrain_ext_fresh,
                cfg.max_obstacle_points,
                obstacle_merge_config);
            timing.obstacle_merge_ms += elapsedMs(merge_start);
            planner_obstacles_ptr = &planner_terrain_xyzh;
          }
          const auto traversability_view =
              (cfg.check_obstacle && cfg.use_traversability_cost &&
               traversability_fresh)
                  ? traversability_grid.view()
                  : lingtu::nav::plan::TraversabilityGridView{};
          const auto nav_tick_start = SteadyClock::now();
          auto assisted = nav.tickTeleopIntent(
              *map_body,
              precheck.cmd,
              planner_obstacles_ptr != nullptr
                  ? planner_obstacles_ptr->data()
                  : nullptr,
              planner_obstacles_ptr != nullptr
                  ? static_cast<int>(planner_obstacles_ptr->size() / 4)
                  : 0,
              steadySeconds(),
              traversability_view);
          timing.nav_tick_ms = elapsedMs(nav_tick_start);

          if (assisted.path_found && assisted.local_path_map.size() >= 2) {
            decision = evaluateAutonomyPathSafety(
                safety_config,
                assisted.cmd_vel,
                map_body,
                assisted.local_path_map,
                planner_obstacles_ptr != nullptr
                    ? *planner_obstacles_ptr
                    : empty_obstacles,
                traversability_grid,
                traversability_fresh);
            if (!decision.stopped && decision.reason == "accepted") {
              decision.reason = assisted.reason;
            }
          } else {
            decision = {};
            decision.should_publish = true;
            decision.stopped = true;
            decision.limited = true;
            decision.reason = assisted.reason.empty()
                ? "teleop_assist_no_path"
                : assisted.reason;
            nav.stopLinearMotion();
          }

          const auto local_write_start = SteadyClock::now();
          dds.writeLocalPath(assisted.local_path_map);
          dds.writeWayPoint(assisted.target);
          timing.dds_write_ms += elapsedMs(local_write_start);
          last_local_path = assisted.local_path_map;
          last_local_planner_debug = assisted.local_planner_debug;
          last_local.seen = true;
          last_local.active = assisted.active;
          last_local.goal_reached = false;
          last_local.path_found = assisted.path_found;
          last_local.near_field_stop = decision.stopped;
          last_local.reason = assisted.reason;
          last_local.slow_down = assisted.slow_down;
          last_local.recovery_state = 0;
          last_local.target_index = 0;
          last_local.target_distance_m = assisted.target_distance_m;
          last_local.target = assisted.target;
          last_local.local_path_points = assisted.local_path_map.size();
          last_local.path_follower_cmd_vel = assisted.cmd_vel;
          last_local.cmd_vel = decision.cmd;
          last_local.final_safety_applied = true;
          last_local.final_safety_stopped = decision.stopped;
          last_local.final_safety_slowed = decision.slowed;
          last_local.final_safety_limited = decision.limited;
          last_local.final_safety_reason = decision.reason;
          last_local.final_safety_obstacle_distance_m =
              decision.obstacle_distance_m;
          last_local.final_safety_traversability_cost =
              decision.traversability_cost;
          ++output_count;
        } else {
          decision = arbitrateTeleopCommand(
              teleopSafetyConfig(cfg),
              *active_teleop_request,
              age_s,
              map_body,
              obstacle_xyzh,
              traversability_grid,
              traversability_fresh);
          if (cfg.teleop_local_planner) {
            nav.stopLinearMotion();
          }
        }
        const double publish_age_s = teleop_receive_age_s();
        if (cfg.teleop_cmd_max_age_s > 0.0 &&
            publish_age_s > cfg.teleop_cmd_max_age_s) {
          decision = {};
          decision.should_publish = true;
          decision.stopped = true;
          decision.limited = true;
          decision.reason = "stale";
          if (cfg.teleop_local_planner) {
            nav.stopLinearMotion();
          }
          if (use_assisted_planner) {
            last_local.near_field_stop = true;
            last_local.cmd_vel = {};
            last_local.final_safety_stopped = true;
            last_local.final_safety_slowed = false;
            last_local.final_safety_limited = true;
            last_local.final_safety_reason = "stale";
          }
        }
        last_teleop.seen = true;
        last_teleop.fresh =
            cfg.teleop_cmd_max_age_s <= 0.0 ||
            publish_age_s <= cfg.teleop_cmd_max_age_s;
        last_teleop.age_s = publish_age_s;
        last_teleop.request = *active_teleop_request;
        last_teleop.output = decision.cmd;
        last_teleop.published = cfg.publish_cmd_vel && decision.should_publish;
        last_teleop.stopped = decision.stopped;
        last_teleop.slowed = decision.slowed;
        last_teleop.limited = decision.limited;
        last_teleop.reason = decision.reason;
        last_teleop.obstacle_distance_m = decision.obstacle_distance_m;
        last_teleop.traversability_cost = decision.traversability_cost;
        if (cfg.publish_cmd_vel && decision.should_publish) {
          const auto write_start = SteadyClock::now();
          dds.writeCmdVel(decision.cmd);
          timing.dds_write_ms += elapsedMs(write_start);
          ++cmd_vel_count;
          ++teleop_output_count;
        }
        if (decision.stopped) {
          ++teleop_stop_count;
        }
        if (decision.slowed) {
          ++teleop_slow_count;
        }
        if (decision.limited) {
          ++teleop_limited_count;
        }
      } else if (active_teleop_request && path_active) {
        last_teleop.seen = true;
        last_teleop.fresh = true;
        last_teleop.published = false;
        last_teleop.reason = "auto_active";
      }
      timing.teleop_gate_ms = elapsedMs(teleop_start);

      if (map_body && path_active && input_gate_state.ready &&
          control_authority.motionAllowed()) {
        const double tick_now = steadySeconds();
        const bool traversability_available = !traversability_grid.values.empty();
        const bool traversability_fresh =
            traversability_available &&
            (last_traversability_receive_s > 0.0) &&
            (cfg.traversability_max_age_s <= 0.0 ||
             tick_now - last_traversability_receive_s <= cfg.traversability_max_age_s);
        const auto traversability_view =
            (cfg.check_obstacle && cfg.use_traversability_cost && traversability_fresh)
                ? traversability_grid.view()
                : lingtu::nav::plan::TraversabilityGridView{};
        const bool terrain_map_fresh =
            !terrain_xyzh.empty() &&
            (last_terrain_map_receive_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             tick_now - last_terrain_map_receive_s <= cfg.terrain_map_max_age_s);
        const bool terrain_ext_fresh =
            !terrain_ext_xyzh.empty() &&
            (last_terrain_ext_receive_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             tick_now - last_terrain_ext_receive_s <= cfg.terrain_map_max_age_s);
        const std::vector<float>* planner_obstacles_ptr = nullptr;
        if (cfg.check_obstacle) {
          const auto merge_start = SteadyClock::now();
          buildPlannerObstacleCloud(
              planner_terrain_xyzh,
              obstacle_xyzh,
              terrain_xyzh,
              terrain_map_fresh,
              terrain_ext_xyzh,
              terrain_ext_fresh,
              cfg.max_obstacle_points,
              obstacle_merge_config);
          timing.obstacle_merge_ms += elapsedMs(merge_start);
          planner_obstacles_ptr = &planner_terrain_xyzh;
        }
        const bool has_planner_obstacles =
            planner_obstacles_ptr != nullptr && !planner_obstacles_ptr->empty();
        const auto nav_tick_start = SteadyClock::now();
        auto out = nav.tick(
            *map_body,
            has_planner_obstacles ? planner_obstacles_ptr->data() : nullptr,
            has_planner_obstacles ? static_cast<int>(planner_obstacles_ptr->size() / 4) : 0,
            steadySeconds(),
            traversability_view);
        last_local_planner_debug = out.local_planner_debug;
        timing.nav_tick_ms = elapsedMs(nav_tick_start);
        last_local.path_follower_cmd_vel = out.cmd_vel;
        last_local.final_safety_applied = false;
        last_local.final_safety_stopped = false;
        last_local.final_safety_slowed = false;
        last_local.final_safety_limited = false;
        last_local.final_safety_reason = "not_applied";
        last_local.final_safety_obstacle_distance_m = -1.0;
        last_local.final_safety_traversability_cost = -1.0;
        const bool has_command =
            linearSpeed(out.cmd_vel) > 1e-6 || std::abs(out.cmd_vel.wz) > 1e-6;
        if (has_command) {
          auto final_safety = evaluateAutonomyPathSafety(
              safety_config,
              out.cmd_vel,
              map_body,
              out.local_path_map,
              planner_obstacles_ptr != nullptr ? *planner_obstacles_ptr : empty_obstacles,
              traversability_grid,
              traversability_fresh);
          if (final_safety.stopped) {
            nav.stopLinearMotion();
            if (std::abs(out.cmd_vel.wz) > 1e-6) {
              const nav_kernel::Twist rotation_only{0.0, 0.0, out.cmd_vel.wz};
              const auto rotation_safety = evaluateCommandSafety(
                  safety_config,
                  rotation_only,
                  0.0,
                  map_body,
                  planner_obstacles_ptr != nullptr
                      ? *planner_obstacles_ptr
                      : empty_obstacles,
                  traversability_grid,
                  traversability_fresh);
              if (!rotation_safety.stopped && std::abs(rotation_safety.cmd.wz) > 1e-6) {
                final_safety.cmd = rotation_safety.cmd;
                final_safety.limited = true;
                final_safety.reason += "_rotation_only";
              }
            }
          }
          out.cmd_vel = final_safety.cmd;
          last_local.final_safety_applied = true;
          last_local.final_safety_stopped = final_safety.stopped;
          last_local.final_safety_slowed = final_safety.slowed;
          last_local.final_safety_limited = final_safety.limited;
          last_local.final_safety_reason = final_safety.reason;
          last_local.final_safety_obstacle_distance_m =
              final_safety.obstacle_distance_m;
          last_local.final_safety_traversability_cost =
              final_safety.traversability_cost;
          if (final_safety.stopped) {
            out.reason = std::string("final_safety_") + final_safety.reason;
          }
        } else {
          last_local.final_safety_applied = false;
          last_local.final_safety_reason = "zero_command";
        }
        const auto local_write_start = SteadyClock::now();
        dds.writeLocalPath(out.local_path_map);
        timing.dds_write_ms += elapsedMs(local_write_start);
        last_local_path = out.local_path_map;
        last_local.seen = true;
        last_local.active = out.active;
        last_local.goal_reached = out.goal_reached;
        last_local.path_found = out.path_found;
        last_local.near_field_stop = out.near_field_stop;
        last_local.reason = out.reason;
        last_local.slow_down = out.slow_down;
        last_local.recovery_state = out.recovery_state;
        last_local.target_index = out.target_index;
        last_local.target_distance_m = out.target_distance_m;
        last_local.target = out.target;
        last_local.local_path_points = out.local_path_map.size();
        last_local.cmd_vel = out.cmd_vel;
        const auto waypoint_write_start = SteadyClock::now();
        dds.writeWayPoint(out.target);
        timing.dds_write_ms += elapsedMs(waypoint_write_start);
        if (cfg.publish_cmd_vel) {
          const auto cmd_write_start = SteadyClock::now();
          dds.writeCmdVel(out.cmd_vel);
          timing.dds_write_ms += elapsedMs(cmd_write_start);
          ++cmd_vel_count;
        }
        ++output_count;
        if (out.recovery_exhausted &&
            inspection_executor.status().state ==
                lingtu::nav::inspection::RunState::kNavigating) {
          control_authority.stop();
          if (!clear_endpoint_motion("inspection_local_recovery_exhausted")) {
            (void)inspection_executor.Pause(
                "inspection_local_recovery_zero_publish_failed");
          } else {
            (void)inspection_executor.OnNavigationFailed(
                out.reason.empty() ? "local_recovery_exhausted" : out.reason,
                nowSeconds());
          }
          next_inspection_status_s = 0.0;
        } else if (out.goal_reached) {
          if (inspection_executor.active()) {
            inspection_executor.OnGoalReached(nowSeconds());
            next_inspection_status_s = 0.0;
          }
          control_authority.stop();
        }
      } else if (path_active && !input_gate_state.ready) {
        last_local.seen = true;
        last_local.active = false;
        last_local.path_found = false;
        last_local.near_field_stop = true;
        last_local.reason = input_gate_state.reason;
        last_local.final_safety_applied = false;
        last_local.final_safety_stopped = true;
        last_local.final_safety_slowed = false;
        last_local.final_safety_limited = false;
        last_local.final_safety_reason = std::string("input_gate_") + input_gate_state.reason;
        last_local.final_safety_obstacle_distance_m = -1.0;
        last_local.final_safety_traversability_cost = -1.0;
        last_local.path_follower_cmd_vel = {};
        last_local.cmd_vel = {};
        last_local_path.clear();
        last_local_planner_debug = {};
        if (cfg.publish_cmd_vel) {
          const auto cmd_write_start = SteadyClock::now();
          dds.writeCmdVel({});
          timing.dds_write_ms += elapsedMs(cmd_write_start);
          ++cmd_vel_count;
        }
      }

      const double now = nowSeconds();
      const double steady_now = steadySeconds();
      if (inspection_executor.status().state ==
              lingtu::nav::inspection::RunState::kNavigating &&
          path_active && !input_gate_state.ready) {
        if (inspection_executor.Pause(
                std::string("input_gate_") + input_gate_state.reason)) {
          (void)clear_endpoint_motion("inspection_input_gate_pause");
          next_inspection_status_s = 0.0;
        }
      }
      if (now >= next_inspection_status_s) {
        dds.writeInspectionStatus(inspection_executor.status());
        if (inspection_store) {
          const auto status_result = inspection_store->PutStatus(
              inspection_executor.status());
          if (!status_result.ok) {
            std::fprintf(
                stderr,
                "nav_native: inspection status persistence failed: %s\n",
                status_result.reason.c_str());
          }
        }
        next_inspection_status_s = now + 0.5;
      }
      if (cfg.status_s > 0.0 && steady_now >= next_status) {
        next_status = steady_now + cfg.status_s;
        const bool traversability_available = !traversability_grid.values.empty();
        const bool traversability_fresh =
            traversability_available &&
            (last_traversability_receive_s > 0.0) &&
            (cfg.traversability_max_age_s <= 0.0 ||
             steady_now - last_traversability_receive_s <= cfg.traversability_max_age_s);
        const bool terrain_map_fresh =
            !terrain_xyzh.empty() &&
            (last_terrain_map_receive_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             steady_now - last_terrain_map_receive_s <= cfg.terrain_map_max_age_s);
        const bool terrain_ext_fresh =
            !terrain_ext_xyzh.empty() &&
            (last_terrain_ext_receive_s > 0.0) &&
            (cfg.terrain_map_max_age_s <= 0.0 ||
             steady_now - last_terrain_ext_receive_s <= cfg.terrain_map_max_age_s);
        if (cfg.check_obstacle) {
          const auto merge_start = SteadyClock::now();
          buildPlannerObstacleCloud(
              planner_terrain_xyzh,
              obstacle_xyzh,
              terrain_xyzh,
              terrain_map_fresh,
              terrain_ext_xyzh,
              terrain_ext_fresh,
              cfg.max_obstacle_points,
              obstacle_merge_config);
          timing.obstacle_merge_ms += elapsedMs(merge_start);
        }
        status_cfg.operator_takeover_latched =
            control_authority.operatorTakeoverLatched();
        status_cfg.resume_required = operator_resume_required;
        if (control_authority.estopLatched()) {
          status_cfg.active_cmd_source = "estop";
        } else if (control_authority.operatorTakeoverLatched()) {
          status_cfg.active_cmd_source =
              control_authority.teleopRequest().has_value() && last_teleop.fresh
                  ? "teleop"
                  : "manual_hold";
        } else if (control_authority.pathActive()) {
          status_cfg.active_cmd_source = "autonomy";
        } else {
          status_cfg.active_cmd_source = "none";
        }
        const std::size_t planner_obstacle_points =
            !cfg.check_obstacle
                ? 0
                : planner_terrain_xyzh.size() / 4;
        latest_dynamic_clusters = live_obstacles.dynamicClusters(32, now);
        const auto motion_layer_stats = live_obstacles.stats();
        const auto status_log_start = SteadyClock::now();
        std::fprintf(
            stderr,
            "nav_native: odom=%llu goals=%llu cancels=%llu registered_clouds=%llu terrain_maps=%llu terrain_map_exts=%llu traversability=%llu teleop=%llu paths=%llu plan_fail=%llu outputs=%llu cmd_vel=%llu obstacle_points=%zu motion_dynamic=%zu gate=%s active=%d\n",
            static_cast<unsigned long long>(odom_count),
            static_cast<unsigned long long>(goal_count),
            static_cast<unsigned long long>(cancel_count),
            static_cast<unsigned long long>(cloud_count),
            static_cast<unsigned long long>(terrain_map_count),
            static_cast<unsigned long long>(terrain_map_ext_count),
            static_cast<unsigned long long>(traversability_count),
            static_cast<unsigned long long>(teleop_cmd_count),
            static_cast<unsigned long long>(path_count),
            static_cast<unsigned long long>(plan_fail_count),
            static_cast<unsigned long long>(output_count),
            static_cast<unsigned long long>(cmd_vel_count),
            planner_obstacle_points,
            motion_layer_stats.dynamic_cells,
            input_gate_state.reason.c_str(),
            control_authority.pathActive() ? 1 : 0);
        timing.status_log_ms = elapsedMs(status_log_start);
        const auto status_snapshot_start = SteadyClock::now();
        writeStatusSnapshot(
            status_snapshot_writer,
            status_cfg,
            now,
            map_body.has_value(),
            map_odom_tf.has_value(),
            control_authority.pathActive(),
            control_authority.estopLatched(),
            control_authority.estopReason(),
            control_authority.estopLatched()
                ? nav_kernel::Twist{}
                : (control_authority.teleopRequest().has_value()
                       ? last_teleop.output
                       : (cfg.control_mode == ControlMode::Autonomy
                              ? last_local.cmd_vel
                              : last_teleop.output)),
            cfg.check_obstacle && cfg.use_traversability_cost && traversability_fresh,
            cfg.check_obstacle && terrain_map_fresh,
            cfg.check_obstacle && terrain_ext_fresh,
            input_gate_state,
            cloud_sync,
            frames,
            command_diagnostics,
            odom_count,
            tf_count,
            goal_count,
            cancel_count,
            map_clearing_count,
            cloud_clearing_count,
            cloud_count,
            terrain_map_count,
            terrain_map_ext_count,
            traversability_count,
            teleop_cmd_count,
            teleop_output_count,
            teleop_stop_count,
            teleop_slow_count,
            teleop_limited_count,
            path_count,
            plan_fail_count,
            output_count,
            cmd_vel_count,
            live_obstacles.size(),
            motion_layer_stats,
            latest_dynamic_clusters,
            last_sensor_origin,
            planner_obstacle_points,
            last_plan,
            last_local,
            last_teleop,
            last_timing,
            last_global_path,
            last_local_path,
            last_local_planner_debug,
            planner_terrain_xyzh,
            traversability_grid);
        timing.status_snapshot_ms = elapsedMs(status_snapshot_start);
      }

      const auto before_sleep = SteadyClock::now();
      if (before_sleep < next_tick) {
        timing.sleep_ms = elapsedMs(before_sleep, next_tick);
        std::this_thread::sleep_until(next_tick);
      } else {
        timing.overrun_ms = elapsedMs(next_tick, before_sleep);
        next_tick = before_sleep;
      }
      timing.loop_ms = elapsedMs(loop_start);
      last_timing = timing;
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "navd failed: %s\n", exc.what());
    return 1;
  }
}
