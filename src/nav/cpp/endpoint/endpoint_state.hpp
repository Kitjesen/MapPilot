#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "frame_transform.hpp"
#include "motion/control_authority.hpp"
#include "motion/estop_latch_store.hpp"
#include "motion/motion_layer.hpp"
#include "motion/teleop_safety.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav_endpoint_messages.hpp"
#include "nav_kernel/types.hpp"
#include "nav_loop.hpp"
#include "plan/input_gate.hpp"
#include "status/nav_status_writer.hpp"
#include "traversability/transform_buffer.hpp"

namespace lingtu::nav::endpoint {

/// Aggregated mutable state for the navd main loop.
/// Groups the ~105 previously scattered variables into named clusters so
/// functional modules can receive a single reference instead of dozens of
/// individual captures.
struct EndpointState {
  // -- Pose / transform -------------------------------------------------------
  std::optional<nav_kernel::Pose> odom_body;
  std::optional<nav_kernel::Pose> map_body;
  std::optional<RigidTransform> odom_body_transform;
  std::optional<RigidTransform> map_body_transform;
  std::optional<RigidTransform> map_odom_tf;

  // -- Obstacle / terrain ------------------------------------------------------
  std::vector<float> obstacle_xyzh;
  std::vector<float> terrain_xyzh;
  std::vector<float> terrain_ext_xyzh;
  std::vector<float> planner_terrain_xyzh;
  const std::vector<float> empty_obstacles;
  std::vector<DynamicCluster> latest_dynamic_clusters;
  bool obstacle_snapshot_dirty{false};
  SensorOrigin last_sensor_origin;
  TraversabilityGrid traversability_grid;

  // -- Timestamps --------------------------------------------------------------
  double last_terrain_map_s{0.0};
  double last_terrain_map_receive_s{0.0};
  double last_terrain_ext_s{0.0};
  double last_terrain_ext_receive_s{0.0};
  double last_traversability_s{0.0};
  double last_traversability_receive_s{0.0};
  double last_odom_s{0.0};
  double last_odom_receive_s{0.0};
  double last_odom_linear_speed_mps{0.0};
  double last_odom_angular_speed_radps{0.0};
  OdometrySpeedMonitor odom_speed_monitor;
  double last_tf_s{0.0};
  double last_tf_receive_s{0.0};
  double last_cloud_s{0.0};
  double last_cloud_receive_s{0.0};
  double map_odom_epoch_start_s{0.0};

  // -- Localization health -----------------------------------------------------
  LocalizationHealthSample localization_health;
  double localization_health_receive_s{0.0};

  // -- Driver control ----------------------------------------------------------
  double driver_control_stamp_s{0.0};
  std::chrono::steady_clock::time_point driver_control_receive_time{};
  bool driver_control_received{false};
  bool driver_control_ready{false};
  std::string driver_control_reason{"not_received"};
  std::string driver_accepted_producer_boot_id;
  std::uint64_t driver_accepted_output_sequence{0};
  bool driver_last_command_accepted{false};
  bool driver_authority_previous{false};

  // -- Teleop -------------------------------------------------------------------
  std::chrono::steady_clock::time_point teleop_receive_time{};
  bool teleop_received{false};

  // -- Input --------------------------------------------------------------------
  bool odom_requires_tf{true};
  InputGateState input_gate_state;

  // -- Diagnostics --------------------------------------------------------------
  PlanDiagnostics last_plan;
  LocalDiagnostics last_local;
  TeleopDiagnostics last_teleop;
  FrameDiagnostics frames;
  OperatorMotionTransportDiagnostics operator_motion_transport;
  std::vector<nav_kernel::Vec3> last_global_path;
  std::vector<nav_kernel::Vec3> last_local_path;
  nav_kernel::LocalPlannerDebugSnapshot last_local_planner_debug;

  // -- Control ------------------------------------------------------------------
  ControlAuthority control_authority;
  lingtu::nav::inspection::Executor inspection_executor;
  bool operator_resume_required{false};

  // -- Epoch / counters ---------------------------------------------------------
  PathEcho path_echo;
  std::uint64_t odom_count{0};
  std::uint64_t odom_generation{0};
  std::uint64_t tf_count{0};
  std::uint64_t tf_generation{0};
  // Epoch identities are externally echoed by request-scoped planner overlays;
  // zero is reserved for "missing", so the first live frame starts at one.
  std::uint64_t frame_epoch{1};
  std::uint64_t goal_count{0};
  std::uint64_t cancel_count{0};
  std::uint64_t map_clearing_count{0};
  std::uint64_t cloud_clearing_count{0};
  std::uint64_t cloud_count{0};
  std::uint64_t cloud_generation{0};
  CloudSyncDiagnostics cloud_sync;
  std::uint64_t terrain_map_count{0};
  std::uint64_t terrain_map_ext_count{0};
  std::uint64_t traversability_count{0};
  std::uint64_t traversability_generation{0};
  std::uint64_t localization_health_generation{0};
  std::uint64_t driver_control_generation{0};
  std::uint64_t teleop_cmd_count{0};
  std::uint64_t teleop_output_count{0};
  std::uint64_t teleop_stop_count{0};
  std::uint64_t teleop_slow_count{0};
  std::uint64_t teleop_limited_count{0};
  std::uint64_t path_count{0};
  std::uint64_t plan_fail_count{0};
  std::uint64_t output_count{0};
  std::uint64_t cmd_vel_count{0};
  double autonomy_request_not_before_s{0.0};
};

}  // namespace lingtu::nav::endpoint
