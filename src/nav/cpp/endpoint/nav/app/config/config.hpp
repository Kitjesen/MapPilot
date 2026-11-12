#pragma once

#include <lingtu/settings.hpp>

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "planning/global/far/planner.hpp"
#include "planning/global/contract.hpp"
#include "planning/local/planner.hpp"
#include "nav_kernel/velocity_smoother.hpp"
#include "navigation/executor.hpp"
#include "control/operator.hpp"
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
  GlobalPlannerBackend global_planner{
      std::string_view(::lingtu::settings::global::native_nav::global::planner) == "far"
          ? GlobalPlannerBackend::Far
          : GlobalPlannerBackend::OctoPlanner3D};
  nav_kernel::LocalPlannerBackend local_planner_backend{
      std::string_view(::lingtu::settings::control::standalone::native_nav::local_planner) == "scan"
          ? nav_kernel::LocalPlannerBackend::Scan
          : nav_kernel::LocalPlannerBackend::Cmu};
  int domain_id{0};
  std::vector<nav_kernel::LocalPlannerBackend> local_planners{
      nav_kernel::LocalPlannerBackend::Cmu, nav_kernel::LocalPlannerBackend::Scan};
  bool use_simulation_clock{false};
  double tick_hz{::lingtu::settings::control::native_nav::tick_hz};
  double control_loop_deadline_miss_ratio_limit{::lingtu::settings::control::native_nav::control_loop_deadline_miss_ratio_limit};
  double control_loop_p95_utilization_limit{::lingtu::settings::control::native_nav::control_loop_p95_utilization_limit};
  double corridor_lookahead_m{::lingtu::settings::follower::native_nav::corridor_lookahead_m};
  double segment_max_distance_m{::lingtu::settings::explore::parameters::segment_max_distance_m};
  std::size_t segment_max_waypoints{::lingtu::settings::explore::parameters::segment_max_waypoints};
  std::size_t segment_max_grid_cells{::lingtu::settings::explore::standalone::parameters::map_input_max_cells};
  double segment_risk_stop{::lingtu::settings::explore::parameters::risk_stop_threshold};
  double segment_risk_resume{::lingtu::settings::explore::parameters::risk_resume_threshold};
  double segment_map_max_age_s{::lingtu::settings::explore::standalone::parameters::map_input_max_age_s};
  double segment_future_tolerance_s{::lingtu::settings::explore::standalone::parameters::map_input_future_tolerance_s};
  double waypoint_reached_m{::lingtu::settings::follower::native_nav::waypoint_reached_m};
  double goal_reached_m{::lingtu::settings::follower::native_nav::goal_reached_m};
  double goal_height_tolerance_m{::lingtu::settings::follower::native_nav::goal_height_tolerance_m};
  double path_follower_goal_tolerance_m{::lingtu::settings::follower::native_nav::follower::goal_tolerance_m};
  double path_follower_lookahead_m{::lingtu::settings::follower::native_nav::follower::lookahead_m};
  double path_follower_max_speed_mps{::lingtu::settings::limits::native_nav::follower::max_speed_mps};
  double path_follower_min_speed_mps{::lingtu::settings::follower::native_nav::follower::min_speed_mps};
  double path_follower_max_accel_mps2{::lingtu::settings::limits::native_nav::follower::max_accel_mps2};
  double path_follower_max_yaw_rate_rad_s{::lingtu::settings::limits::native_nav::follower::max_yaw_rate_rad_s};
  double path_follower_max_yaw_accel_rad_s2{::lingtu::settings::limits::native_nav::follower::max_yaw_accel_rad_s2};
  double path_follower_heading_align_enter_rad{::lingtu::settings::follower::native_nav::follower::align_enter_rad};
  double path_follower_heading_align_exit_rad{::lingtu::settings::follower::native_nav::follower::align_exit_rad};
  nav_kernel::SplineFollowerParams scan_follower{};
  nav_kernel::ScanPlannerParams scan_planner{};
  nav_kernel::LocalPlannerParams cmu_planner{};
  nav_kernel::CmuFollowerProfile cmu_follower{};
  double local_collision_max_age_s{::lingtu::settings::local::parameters::local_collision_max_age_s};
  std::vector<nav_kernel::RecoveryAction> recovery_order = navigation::defaultRecoveryOrder();
  double recovery_blocked_interval_s{::lingtu::settings::recovery::native_nav::recovery::blocked_interval_s};
  double recovery_rotation_timeout_s{::lingtu::settings::recovery::native_nav::recovery::rotation_timeout_s};
  double recovery_translation_timeout_s{::lingtu::settings::recovery::native_nav::recovery::translation_timeout_s};
  int recovery_max_attempts{::lingtu::settings::recovery::standalone::native_nav::recovery_max_attempts};
  double recovery_translation_speed_mps{::lingtu::settings::recovery::native_nav::recovery::translation_speed_mps};
  double recovery_rotation_rate_rad_s{::lingtu::settings::recovery::native_nav::recovery::rotation_rate_rad_s};
  double recovery_min_rotation_rad{::lingtu::settings::recovery::native_nav::recovery::min_rotation_rad};
  double recovery_max_rotation_rad{::lingtu::settings::recovery::native_nav::recovery::max_rotation_rad};
  double recovery_rotation_candidate_step_rad{::lingtu::settings::recovery::native_nav::recovery::rotation_candidate_step_rad};
  double recovery_rotation_sample_step_rad{::lingtu::settings::recovery::native_nav::recovery::rotation_sample_step_rad};
  std::string product;
  std::string product_session_id;
  double status_s{::lingtu::settings::control::native_nav::status_s};
  double stop_confirmation_timeout_s{::lingtu::settings::control::native_nav::stop_confirmation_timeout_s};
  double teleop_cmd_max_age_s{::lingtu::settings::teleop::native_nav::teleop::cmd_max_age_s};
  double teleop_max_speed_mps{::lingtu::settings::teleop::native_nav::teleop::max_speed_mps};
  double teleop_max_yaw_rate{::lingtu::settings::teleop::native_nav::teleop::max_yaw_rate_rad_s};
  double teleop_min_motion_speed_mps{::lingtu::settings::teleop::native_nav::teleop::min_motion_speed_mps};
  double teleop_obstacle_margin_m{0.15};
  double teleop_traversability_hard_cost{::lingtu::settings::teleop::native_nav::teleop::traversability_hard_cost};
  double teleop_traversability_soft_cost{::lingtu::settings::teleop::native_nav::teleop::traversability_soft_cost};
  double teleop_planner_horizon_m{::lingtu::settings::teleop::native_nav::teleop::horizon_m};
  double teleop_planner_max_deviation_deg{::lingtu::settings::teleop::native_nav::teleop::max_deviation_deg};
  double traversability_max_age_s{::lingtu::settings::local::native_nav::traversability_max_age_s};
  double localization_health_max_age_s{::lingtu::settings::control::native_nav::localization_health_max_age_s};
  double driver_control_max_age_s{::lingtu::settings::control::native_nav::driver_control_max_age_s};
  double terrain_map_max_age_s{::lingtu::settings::local::native_nav::terrain_map_max_age_s};
  double odom_max_age_s{::lingtu::settings::control::native_nav::odom_max_age_s};
  double tf_max_age_s{::lingtu::settings::control::native_nav::tf_max_age_s};
  double cloud_max_age_s{::lingtu::settings::control::native_nav::cloud_max_age_s};
  double cloud_pose_max_gap_s{::lingtu::settings::control::native_nav::cloud_pose_max_gap_s};
  double input_future_tolerance_s{::lingtu::settings::control::native_nav::input_future_tolerance_s};
  double traversability_hard_cost{::lingtu::settings::local::native_nav::traversability_hard_cost};
  double traversability_soft_cost{::lingtu::settings::local::native_nav::traversability_soft_cost};
  double traversability_weight{::lingtu::settings::local::native_nav::traversability_weight};
  double local_planner_obstacle_height_min_m{::lingtu::settings::local::native_nav::local_planner_obstacle_height_min_m};
  double local_planner_obstacle_height_max_m{::lingtu::settings::local::native_nav::local_planner_obstacle_height_max_m};
  std::size_t max_obstacle_points{::lingtu::settings::local::native_nav::max_obstacle_points};
  std::size_t local_planner_threads{::lingtu::settings::local::native_nav::local_planner_threads};
  std::size_t local_planner_debug_candidate_limit{::lingtu::settings::local::native_nav::local_planner_debug_candidates};
  std::size_t local_map_debug_point_limit{::lingtu::settings::local::native_nav::local_map_debug_points};
  double obstacle_voxel_size_m{::lingtu::settings::local::native_nav::obstacle_voxel_size_m};
  double live_obstacle_decay_s{::lingtu::settings::local::native_nav::live_obstacle_decay_s};
  double live_obstacle_inflation_radius_m{0.12};
  double live_obstacle_ray_clear_max_range_m{::lingtu::settings::local::native_nav::live_obstacle_ray_clear_max_range_m};
  double live_obstacle_ray_clearing_interval_s{::lingtu::settings::local::native_nav::live_obstacle_ray_clearing_interval_s};
  std::size_t live_obstacle_max_clearing_rays{::lingtu::settings::local::native_nav::live_obstacle_max_clearing_rays};
  int live_obstacle_min_hits{::lingtu::settings::local::native_nav::live_obstacle_min_hits};
  std::size_t dynamic_min_cells{::lingtu::settings::local::native_nav::dynamic_min_cells};
  double dynamic_min_speed_mps{::lingtu::settings::local::native_nav::dynamic_min_speed_mps};
  std::size_t dynamic_confirm_frames{::lingtu::settings::local::native_nav::dynamic_confirm_frames};
  bool live_obstacle_ray_clearing{::lingtu::settings::local::native_nav::live_obstacle_ray_clearing};
  bool publish_cmd_vel{::lingtu::settings::control::native_nav::publish_cmd_vel};
  bool check_obstacle{true};
  bool use_traversability_cost{false};
  bool allow_teleop_takeover{::lingtu::settings::teleop::native_nav::teleop::takeover};
  bool teleop_local_planner{::lingtu::settings::teleop::native_nav::teleop::planner};
  bool velocity_smoother_enabled{::lingtu::settings::control::native_nav::smoothing};
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
void applyPlannerGeometry(CliConfig &cfg);

CommandSafetyConfig commandSafetyConfig(const CliConfig &cfg);
OperatorMotionAuthorityOptions operatorMotionAuthorityOptions(const CliConfig &cfg);
InputGateConfig inputGateConfig(const CliConfig &cfg);
rolling::SegmentExecutorConfig rollingSegmentExecutorConfig(const CliConfig &cfg);
StatusWriterConfig buildStatusWriterConfig(const CliConfig &cfg,
                                           const InputGateConfig &gate_cfg);
nav_kernel::LocalPlannerParams buildLocalPlannerParams(const CliConfig &cfg);
lingtu::nav::navigation::ExecutorConfig buildExecutorConfig(const CliConfig &cfg);

}  // namespace lingtu::nav::endpoint
