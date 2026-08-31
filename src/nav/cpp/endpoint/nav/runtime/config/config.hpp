#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "planning/global/far/planner.hpp"
#include "planning/global/contract.hpp"
#include "planning/local/planner.hpp"
#include "nav_kernel/velocity_smoother.hpp"
#include "navigation/executor.hpp"
#include "safety/command.hpp"
#include "input/gate.hpp"
#include "status/nav_status_writer.hpp"

namespace lingtu::nav::rolling {
struct SegmentExecutorConfig;
}

namespace lingtu::nav::endpoint {

enum class ControlMode {
  Autonomy,
  Teleop,
  TeleopAvoid,
};

enum class GlobalPlannerBackend {
  OctoPlanner3D,
  Far,
};

const char *controlModeName(ControlMode mode);
const char *globalPlannerBackendName(GlobalPlannerBackend backend);

struct CliConfig {
  ControlMode control_mode{ControlMode::Autonomy};
  GlobalPlannerBackend global_planner{GlobalPlannerBackend::OctoPlanner3D};
  nav_kernel::LocalPlannerBackend local_planner_backend{
      nav_kernel::LocalPlannerBackend::Cmu};
  int domain_id{0};
  double tick_hz{20.0};
  double control_loop_deadline_miss_ratio_limit{0.05};
  double control_loop_p95_utilization_limit{0.90};
  double nav_max_speed_mps{0.5};
  double nav_max_accel_mps2{1.0};
  double corridor_lookahead_m{3.0};
  double segment_max_distance_m{5.0};
  std::size_t segment_max_waypoints{32U};
  std::size_t segment_max_grid_cells{262'144U};
  double segment_risk_stop{50.0};
  double segment_risk_resume{40.0};
  double segment_map_max_age_s{0.35};
  double waypoint_reached_m{0.6};
  double goal_reached_m{0.35};
  double path_follower_goal_tolerance_m{0.2};
  double path_follower_lookahead_m{0.3};
  double path_follower_max_speed_mps{0.5};
  double path_follower_min_speed_mps{0.0};
  double path_follower_max_accel_mps2{1.0};
  double path_follower_max_yaw_rate_rad_s{0.8};
  double path_follower_max_yaw_accel_rad_s2{2.0};
  double path_follower_heading_align_enter_rad{0.7853981633974483};
  double path_follower_heading_align_exit_rad{0.35};
  nav_kernel::SplineFollowerParams scan_follower{};
  std::vector<nav_kernel::RecoveryAction> recovery_order{
      nav_kernel::RecoveryAction::Translate,
      nav_kernel::RecoveryAction::Rotate,
  };
  double recovery_blocked_interval_s{2.0};
  double recovery_rotation_timeout_s{2.5};
  double recovery_translation_timeout_s{1.5};
  int recovery_max_attempts{0};
  double recovery_translation_speed_mps{0.15};
  double recovery_rotation_rate_rad_s{0.25};
  double recovery_min_rotation_rad{0.20};
  double recovery_max_rotation_rad{1.20};
  double recovery_rotation_candidate_step_rad{0.20};
  double recovery_rotation_sample_step_rad{0.05};
  std::string product;
  std::string product_session_id;
  double status_s{5.0};
  double stop_confirmation_timeout_s{4.0};
  double teleop_cmd_max_age_s{0.35};
  double teleop_max_speed_mps{0.5};
  double teleop_max_yaw_rate{1.0};
  double teleop_min_motion_speed_mps{0.03};
  double teleop_obstacle_height_min_m{0.20};
  double teleop_obstacle_height_max_m{1.20};
  double teleop_obstacle_margin_m{0.15};
  double teleop_traversability_hard_cost{80.0};
  double teleop_traversability_soft_cost{40.0};
  double teleop_planner_horizon_m{3.5};
  double teleop_planner_max_deviation_deg{90.0};
  double traversability_max_age_s{1.5};
  double localization_health_max_age_s{0.5};
  double driver_control_max_age_s{0.35};
  double terrain_map_max_age_s{0.5};
  double odom_max_age_s{0.25};
  double tf_max_age_s{0.25};
  double cloud_max_age_s{0.35};
  double cloud_pose_max_gap_s{0.10};
  double input_future_tolerance_s{0.05};
  std::size_t input_recovery_frames{3};
  double traversability_hard_cost{80.0};
  double traversability_soft_cost{40.0};
  double traversability_weight{0.01};
  double local_planner_obstacle_height_max_m{1.20};
  std::size_t max_obstacle_points{20000};
  std::size_t local_planner_threads{2};
  std::size_t local_planner_debug_candidate_limit{0};
  std::size_t local_map_debug_point_limit{0};
  double obstacle_voxel_size_m{0.08};
  double live_obstacle_decay_s{0.45};
  double live_obstacle_inflation_radius_m{0.12};
  double live_obstacle_ray_clear_max_range_m{3.5};
  double live_obstacle_ray_clearing_interval_s{0.33};
  std::size_t live_obstacle_max_clearing_rays{160};
  int live_obstacle_min_hits{1};
  std::size_t dynamic_min_cells{8};
  double dynamic_min_speed_mps{0.25};
  std::size_t dynamic_confirm_frames{4};
  bool live_obstacle_ray_clearing{true};
  bool publish_cmd_vel{true};
  bool check_obstacle{true};
  bool use_traversability_cost{false};
  bool allow_teleop_takeover{false};
  bool teleop_local_planner{false};
  bool velocity_smoother_enabled{false};
  nav_kernel::VelocitySmootherConfig velocity_smoother{};
  double vehicle_length_m{1.0};
  double vehicle_width_m{0.6};
  double collision_cylinder_radius_m{0.40};
  double collision_cylinder_offset_m{0.25};
  double collision_clearance_below_m{0.25};
  double collision_clearance_above_m{0.35};
  double sensor_offset_x_m{0.0};
  double sensor_offset_y_m{0.0};
  double sensor_offset_z_m{0.0};
  lingtu::nav::plan::GlobalPlannerOptions octoplanner_options{};
  lingtu::nav::plan::far_planner::FarPlannerConfig far_options{};
  lingtu::nav::plan::MapIdentity map_identity{};
  std::string path_library_dir;
  std::string map_path;
  std::string inspection_dir;
  std::string status_file;
  std::string estop_latch_file;
  std::string geofence_file;
};

CliConfig parseArgs(int argc, char **argv);

CommandSafetyConfig commandSafetyConfig(const CliConfig &cfg);
InputGateConfig inputGateConfig(const CliConfig &cfg);
rolling::SegmentExecutorConfig rollingSegmentExecutorConfig(const CliConfig &cfg);
StatusWriterConfig buildStatusWriterConfig(const CliConfig &cfg,
                                           const InputGateConfig &gate_cfg);
nav_kernel::LocalPlannerParams buildLocalPlannerParams(const CliConfig &cfg);
lingtu::nav::navigation::ExecutorConfig buildExecutorConfig(const CliConfig &cfg);

}  // namespace lingtu::nav::endpoint
