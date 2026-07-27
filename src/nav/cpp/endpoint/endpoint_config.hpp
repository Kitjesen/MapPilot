#pragma once

#include "endpoint_time.hpp"
#include "motion/teleop_safety.hpp"
#include "nav_endpoint_config.hpp"
#include "nav_loop.hpp"
#include "plan/input_gate.hpp"
#include "status/nav_status_writer.hpp"

namespace lingtu::nav::endpoint {

/// Build StatusWriterConfig from parsed CLI config.
/// Pure mapping — no side effects.
inline StatusWriterConfig buildStatusWriterConfig(const CliConfig &cfg,
                                                  const InputGateConfig &gate_cfg) {
  StatusWriterConfig out;
  out.control_mode = controlModeName(cfg.control_mode);
  out.profile = cfg.profile;
  out.config_fingerprint = cfg.config_fingerprint;
  out.domain_id = cfg.domain_id;
  out.tick_hz = cfg.tick_hz;
  out.max_speed_mps = cfg.path_follower_max_speed_mps;
  out.min_speed_mps = cfg.path_follower_min_speed_mps;
  out.max_accel_mps2 = cfg.path_follower_max_accel_mps2;
  out.follower_lookahead_m = cfg.path_follower_lookahead_m;
  out.follower_goal_tolerance_m = cfg.path_follower_goal_tolerance_m;
  out.waypoint_reached_m = cfg.waypoint_reached_m;
  out.goal_reached_m = cfg.goal_reached_m;
  out.corridor_lookahead_m = cfg.corridor_lookahead_m;
  out.nominal_dt_s = 1.0 / cfg.tick_hz;
  out.publish_cmd_vel = cfg.publish_cmd_vel;
  out.check_obstacle = cfg.check_obstacle;
  out.use_traversability_cost = cfg.use_traversability_cost;
  out.allow_teleop_takeover = cfg.allow_teleop_takeover;
  out.teleop_local_planner = cfg.teleop_local_planner;
  out.teleop_planner_horizon_m = cfg.teleop_planner_horizon_m;
  out.teleop_planner_max_deviation_deg = cfg.teleop_planner_max_deviation_deg;
  out.allow_legacy_motion_inputs = cfg.allow_legacy_motion_inputs;
  out.path_library_dir = cfg.path_library_dir;
  out.map_path = cfg.map_path;
  out.global_planner = globalPlannerBackendName(cfg.global_planner);
  out.status_file = cfg.status_file;
  out.max_obstacle_points = cfg.max_obstacle_points;
  out.local_planner_threads = cfg.local_planner_threads;
  out.stop_confirmation_timeout_s = cfg.stop_confirmation_timeout_s;
  out.local_planner_debug_candidate_limit = cfg.local_planner_debug_candidate_limit;
  out.local_map_debug_point_limit = cfg.local_map_debug_point_limit;
  out.obstacle_voxel_size_m = cfg.obstacle_voxel_size_m;
  out.obstacle_registered_share = cfg.obstacle_registered_share;
  out.obstacle_terrain_share = cfg.obstacle_terrain_share;
  out.obstacle_terrain_ext_share = cfg.obstacle_terrain_ext_share;
  out.sensor_offset_x_m = cfg.sensor_offset_x_m;
  out.sensor_offset_y_m = cfg.sensor_offset_y_m;
  out.sensor_offset_z_m = cfg.sensor_offset_z_m;
  out.live_obstacle_decay_s = cfg.live_obstacle_decay_s;
  out.live_obstacle_inflation_radius_m = cfg.live_obstacle_inflation_radius_m;
  out.live_obstacle_layer_inflation_radius_m = kLayerInflationM;
  out.live_obstacle_ray_clear_max_range_m = cfg.live_obstacle_ray_clear_max_range_m;
  out.live_obstacle_ray_clearing_interval_s = cfg.live_obstacle_ray_clearing_interval_s;
  out.live_obstacle_max_clearing_rays = cfg.live_obstacle_max_clearing_rays;
  out.live_obstacle_min_hits = cfg.live_obstacle_min_hits;
  out.live_obstacle_ray_clearing = cfg.live_obstacle_ray_clearing;
  out.odom_max_age_s = cfg.odom_max_age_s;
  out.tf_max_age_s = cfg.tf_max_age_s;
  out.cloud_max_age_s = cfg.cloud_max_age_s;
  out.cloud_pose_max_gap_s = cfg.cloud_pose_max_gap_s;
  out.input_future_tolerance_s = cfg.input_future_tolerance_s;
  out.input_recovery_frames = cfg.input_recovery_frames;
  out.octoplanner_options = cfg.octoplanner_options;
  out.far_options = cfg.far_options;
  // Input gate fields
  out.input_require_odom = gate_cfg.require_odom;
  out.input_require_cloud = gate_cfg.require_cloud;
  out.input_require_traversability = gate_cfg.require_traversability;
  out.input_require_localization_health = gate_cfg.require_localization_health;
  out.input_require_driver_control = gate_cfg.require_driver_control;
  out.traversability_max_age_s = gate_cfg.traversability_max_age_s;
  out.localization_health_max_age_s = gate_cfg.localization_health_max_age_s;
  out.driver_control_max_age_s = gate_cfg.driver_control_max_age_s;
  return out;
}

/// Build NavLoopConfig from parsed CLI config.
/// Pure mapping — no side effects.
inline lingtu::nav::plan::NavLoopConfig buildNavLoopConfig(const CliConfig &cfg,
                                                           double obstacle_margin_m) {
  lingtu::nav::plan::NavLoopConfig out;
  out.path_library_dir = cfg.path_library_dir;
  out.max_speed = cfg.path_follower_max_speed_mps;
  out.corridor_lookahead_m = cfg.corridor_lookahead_m;
  out.waypoint_reached_m = cfg.waypoint_reached_m;
  out.goal_reached_m = cfg.goal_reached_m;
  out.teleop_intent_horizon_m = cfg.teleop_planner_horizon_m;
  out.teleop_intent_max_deviation_deg = cfg.teleop_planner_max_deviation_deg;
  out.local_planner.autonomySpeed = cfg.path_follower_max_speed_mps;
  out.local_planner.maxSpeed = 1.0;
  out.local_planner.vehicleLength = cfg.vehicle_length_m;
  out.local_planner.vehicleWidth = cfg.vehicle_width_m;
  // NavLoop receives the map->body pose.  The LiDAR extrinsic is already
  // applied when sensor-origin geometry is constructed and must not shift
  // the body pose a second time inside LocalPlannerCore.
  out.local_planner.sensorOffsetX = 0.0;
  out.local_planner.sensorOffsetY = 0.0;
  out.local_planner.obstacleHeightMax = cfg.local_planner_obstacle_height_max_m;
  out.local_planner.footprintPadding = obstacle_margin_m;
  out.local_planner.useTraversabilityCost = cfg.use_traversability_cost;
  out.local_planner.traversabilityHardCost = cfg.traversability_hard_cost;
  out.local_planner.traversabilitySoftCost = cfg.traversability_soft_cost;
  out.local_planner.traversabilityWeight = cfg.traversability_weight;
  out.local_planner.scoringThreads = static_cast<int>(cfg.local_planner_threads);
  out.local_planner.debugCandidateLimit = static_cast<int>(cfg.local_planner_debug_candidate_limit);
  out.path_follower.maxSpeed = cfg.path_follower_max_speed_mps;
  out.path_follower.minSpeed = cfg.path_follower_min_speed_mps;
  out.path_follower.maxAccel = cfg.path_follower_max_accel_mps2;
  out.path_follower.baseLookAheadDis = cfg.path_follower_lookahead_m;
  out.path_follower.stopDisThre = cfg.path_follower_goal_tolerance_m;
  out.path_follower.nominalDt = 1.0 / cfg.tick_hz;
  return out;
}

}  // namespace lingtu::nav::endpoint
