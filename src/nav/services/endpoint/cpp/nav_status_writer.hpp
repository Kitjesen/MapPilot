#pragma once

#include "global_planner_contract.hpp"
#include "input_gate.hpp"
#include "motion_layer.hpp"
#include "nav_kernel/types.hpp"
#include "status_snapshot_file_writer.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace nav_kernel {
struct LocalPlannerDebugSnapshot;
}

namespace lingtu::nav::endpoint {

struct TraversabilityGrid;

struct StatusWriterConfig {
  std::string control_mode{"autonomy"};
  int domain_id{0};
  double tick_hz{0.0};
  double max_speed_mps{0.0};
  double max_accel_mps2{0.0};
  double nominal_dt_s{0.0};
  bool publish_cmd_vel{false};
  bool check_obstacle{true};
  bool use_traversability_cost{false};
  bool allow_teleop_takeover{false};
  bool teleop_local_planner{false};
  double teleop_planner_horizon_m{0.0};
  double teleop_planner_max_deviation_deg{0.0};
  bool operator_takeover_latched{false};
  bool resume_required{false};
  std::string active_cmd_source{"none"};
  bool allow_legacy_motion_inputs{false};
  std::string path_library_dir;
  std::string map_path;
  std::string status_file;
  std::size_t max_obstacle_points{0};
  std::size_t local_planner_debug_candidate_limit{0};
  std::size_t local_map_debug_point_limit{0};
  double obstacle_voxel_size_m{0.0};
  double obstacle_registered_share{0.0};
  double obstacle_terrain_share{0.0};
  double obstacle_terrain_ext_share{0.0};
  double sensor_offset_x_m{0.0};
  double sensor_offset_y_m{0.0};
  double sensor_offset_z_m{0.0};
  double live_obstacle_decay_s{0.0};
  double live_obstacle_inflation_radius_m{0.0};
  double live_obstacle_layer_inflation_radius_m{0.0};
  double live_obstacle_ray_clear_max_range_m{0.0};
  double live_obstacle_ray_clearing_interval_s{0.0};
  std::size_t live_obstacle_max_clearing_rays{0};
  int live_obstacle_min_hits{1};
  bool live_obstacle_ray_clearing{false};
  double odom_max_age_s{0.0};
  double tf_max_age_s{0.0};
  double cloud_max_age_s{0.0};
  double traversability_max_age_s{0.0};
  double localization_health_max_age_s{0.0};
  double driver_control_max_age_s{0.0};
  double cloud_pose_max_gap_s{0.0};
  double input_future_tolerance_s{0.0};
  std::size_t input_recovery_frames{0};
  bool input_require_odom{true};
  bool input_require_cloud{true};
  bool input_require_traversability{false};
  bool input_require_localization_health{false};
  bool input_require_driver_control{false};
  lingtu::nav::plan::GlobalPlannerOptions octoplanner_options{};
};

struct PlanDiagnostics {
  bool seen{false};
  bool accepted{false};
  bool reached_goal{false};
  std::string reason{"no_goal_received"};
  std::size_t waypoints{0};
  double goal_error_m{-1.0};
  double elapsed_ms{0.0};
  nav_kernel::Vec3 start{};
  nav_kernel::Vec3 goal{};
};

struct LocalDiagnostics {
  bool seen{false};
  bool active{false};
  bool goal_reached{false};
  bool path_found{false};
  bool near_field_stop{false};
  std::string reason{"not_seen"};
  int slow_down{0};
  int recovery_state{0};
  std::size_t target_index{0};
  double target_distance_m{0.0};
  std::size_t local_path_points{0};
  bool final_safety_applied{false};
  bool final_safety_stopped{false};
  bool final_safety_slowed{false};
  bool final_safety_limited{false};
  std::string final_safety_reason{"not_applied"};
  double final_safety_obstacle_distance_m{-1.0};
  double final_safety_traversability_cost{-1.0};
  nav_kernel::Vec3 target{};
  nav_kernel::Twist path_follower_cmd_vel{};
  nav_kernel::Twist cmd_vel{};
};

struct TeleopDiagnostics {
  bool seen{false};
  bool fresh{false};
  bool published{false};
  bool stopped{false};
  bool slowed{false};
  bool limited{false};
  std::string reason{"idle"};
  nav_kernel::Twist request{};
  nav_kernel::Twist output{};
  double age_s{-1.0};
  double obstacle_distance_m{-1.0};
  double traversability_cost{-1.0};
};

struct TimingDiagnostics {
  double loop_ms{0.0};
  double input_callbacks_ms{0.0};
  double global_plan_ms{0.0};
  double cloud_convert_ms{0.0};
  double motion_update_last_ms{0.0};
  double obstacle_snapshot_last_ms{0.0};
  double teleop_gate_ms{0.0};
  double obstacle_merge_ms{0.0};
  double nav_tick_ms{0.0};
  double dds_write_ms{0.0};
  double status_log_ms{0.0};
  double status_snapshot_ms{0.0};
  double sleep_ms{0.0};
  double overrun_ms{0.0};
};

struct CloudSyncDiagnostics {
  std::uint64_t stamp_rejected{0};
  std::uint64_t pose_rejected{0};
  double last_stamp_age_s{-1.0};
  double last_pose_gap_s{-1.0};
};

struct FrameDiagnostics {
  std::uint64_t odom_rejected{0};
  std::uint64_t goal_rejected{0};
  std::uint64_t path_rejected{0};
  std::uint64_t grid_rejected{0};
  std::uint64_t teleop_rejected{0};
  std::string last_error{"none"};
};

struct CommandDiagnostics {
  std::uint64_t received{0};
  std::uint64_t ack_sent{0};
  std::uint64_t rejected{0};
  std::uint64_t replayed{0};
  std::string last_request_id;
  std::string last_kind{"none"};
  bool last_accepted{false};
  std::string last_reason{"none"};
};

void writeStatusSnapshot(
    StatusSnapshotFileWriter& snapshot_writer,
    const StatusWriterConfig& cfg,
    double stamp_s,
    bool has_odom,
    bool has_map_odom_tf,
    bool has_path,
    bool estop_latched,
    const std::string& estop_reason,
    const nav_kernel::Twist& final_cmd_vel,
    bool has_traversability,
    bool has_terrain_map,
    bool has_terrain_map_ext,
    const InputGateState& input_gate,
    const CloudSyncDiagnostics& cloud_sync,
    const FrameDiagnostics& frames,
    const CommandDiagnostics& commands,
    std::uint64_t odom_count,
    std::uint64_t tf_count,
    std::uint64_t goal_count,
    std::uint64_t cancel_count,
    std::uint64_t map_clearing_count,
    std::uint64_t cloud_clearing_count,
    std::uint64_t cloud_count,
    std::uint64_t terrain_map_count,
    std::uint64_t terrain_map_ext_count,
    std::uint64_t traversability_count,
    std::uint64_t teleop_cmd_count,
    std::uint64_t teleop_output_count,
    std::uint64_t teleop_stop_count,
    std::uint64_t teleop_slow_count,
    std::uint64_t teleop_limited_count,
    std::uint64_t path_count,
    std::uint64_t plan_fail_count,
    std::uint64_t output_count,
    std::uint64_t cmd_vel_count,
    std::size_t live_obstacle_cells,
    const MotionLayerStats& motion_layer,
    const std::vector<DynamicCluster>& dynamic_clusters,
    const SensorOrigin& last_sensor_origin,
    std::size_t obstacle_points,
    const PlanDiagnostics& plan,
    const LocalDiagnostics& local,
    const TeleopDiagnostics& teleop,
    const TimingDiagnostics& timing,
    const std::vector<nav_kernel::Vec3>& global_path,
    const std::vector<nav_kernel::Vec3>& local_path,
    const nav_kernel::LocalPlannerDebugSnapshot& local_planner_debug,
    const std::vector<float>& local_map_obstacle_xyzh,
    const TraversabilityGrid& local_map_traversability);

}  // namespace lingtu::nav::endpoint
