#pragma once

#include "global_planner_contract.hpp"
#include "input_gate.hpp"
#include "teleop_safety.hpp"

#include <cstddef>
#include <string>

namespace lingtu::nav::endpoint {

enum class ControlMode {
  Autonomy,
  Teleop,
  TeleopAvoid,
};

const char* controlModeName(ControlMode mode);

struct CliConfig {
  ControlMode control_mode{ControlMode::Autonomy};
  int domain_id{0};
  double tick_hz{20.0};
  double nav_max_speed_mps{0.4};
  double nav_max_accel_mps2{1.0};
  double corridor_lookahead_m{3.0};
  double status_s{5.0};
  double teleop_cmd_max_age_s{0.35};
  double teleop_max_speed_mps{0.4};
  double teleop_max_yaw_rate{1.0};
  double teleop_slow_distance_m{1.2};
  double teleop_stop_distance_m{0.55};
  double teleop_linear_slow_scale{0.35};
  double teleop_min_motion_speed_mps{0.03};
  double teleop_obstacle_height_min_m{0.10};
  double teleop_obstacle_height_max_m{1.20};
  double teleop_obstacle_margin_m{0.15};
  double teleop_traversability_hard_cost{80.0};
  double teleop_traversability_soft_cost{40.0};
  double teleop_planner_horizon_m{2.0};
  double teleop_planner_max_deviation_deg{55.0};
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
  std::size_t local_planner_debug_candidate_limit{0};
  std::size_t local_map_debug_point_limit{0};
  double obstacle_voxel_size_m{0.08};
  double obstacle_registered_share{0.55};
  double obstacle_terrain_share{0.30};
  double obstacle_terrain_ext_share{0.0};
  double live_obstacle_decay_s{0.45};
  double live_obstacle_inflation_radius_m{0.12};
  double live_obstacle_ray_clear_max_range_m{3.5};
  double live_obstacle_ray_clearing_interval_s{0.33};
  std::size_t live_obstacle_max_clearing_rays{160};
  int live_obstacle_min_hits{1};
  bool live_obstacle_ray_clearing{true};
  bool publish_cmd_vel{true};
  bool check_obstacle{true};
  bool use_traversability_cost{false};
  bool allow_teleop_takeover{false};
  bool teleop_local_planner{false};
  bool allow_legacy_motion_inputs{false};
  double vehicle_length_m{1.0};
  double vehicle_width_m{0.6};
  double sensor_offset_x_m{0.0};
  double sensor_offset_y_m{0.0};
  double sensor_offset_z_m{0.0};
  lingtu::nav::plan::GlobalPlannerOptions octoplanner_options{};
  std::string path_library_dir;
  std::string map_root;
  std::string map_path;
  std::string status_file;
  std::string estop_latch_file;
};

CliConfig parseArgs(int argc, char** argv);

TeleopSafetyConfig teleopSafetyConfig(const CliConfig& cfg);
CommandSafetyConfig commandSafetyConfig(const CliConfig& cfg);
InputGateConfig inputGateConfig(const CliConfig& cfg);

}  // namespace lingtu::nav::endpoint
