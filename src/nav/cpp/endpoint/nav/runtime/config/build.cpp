#include "runtime/config/config.hpp"

#include <algorithm>
#include <limits>

#include "planning/rolling/segment.hpp"
#include "runtime/time.hpp"

namespace lingtu::nav::endpoint {

namespace {

nav_kernel::FollowerParams followerParams(const CliConfig &cfg) {
  nav_kernel::FollowerParams params;
  params.maxSpeed = cfg.path_follower_max_speed_mps;
  params.minSpeed = cfg.path_follower_min_speed_mps;
  params.maxAccel = cfg.path_follower_max_accel_mps2;
  params.maxYawRateRadS = cfg.path_follower_max_yaw_rate_rad_s;
  params.maxYawAccelRadS2 = cfg.path_follower_max_yaw_accel_rad_s2;
  params.headingAlignEnterRad = cfg.path_follower_heading_align_enter_rad;
  params.headingAlignExitRad = cfg.path_follower_heading_align_exit_rad;
  params.baseLookAheadDis = cfg.path_follower_lookahead_m;
  params.stopDisThre = cfg.path_follower_goal_tolerance_m;
  params.nominalDt = 1.0 / cfg.tick_hz;
  params.spline = cfg.scan_follower;
  params.spline.finishDistance = params.stopDisThre;
  return cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Cmu
             ? nav_kernel::cmuFollowerParams(params)
             : params;
}

}  // namespace

const char *controlModeName(ControlMode mode) {
  switch (mode) {
    case ControlMode::Autonomy:
      return "autonomy";
    case ControlMode::Teleop:
      return "teleop";
    case ControlMode::TeleopAvoid:
      return "teleop_avoid";
  }
  return "unknown";
}

const char *globalPlannerBackendName(GlobalPlannerBackend backend) {
  switch (backend) {
    case GlobalPlannerBackend::OctoPlanner3D:
      return "octoplanner3d";
    case GlobalPlannerBackend::Far:
      return "far";
  }
  return "unknown";
}

CommandSafetyConfig commandSafetyConfig(const CliConfig &cfg) {
  CommandSafetyConfig out;
  const bool autonomous = cfg.control_mode == ControlMode::Autonomy;
  out.cmd_max_age_s = cfg.teleop_cmd_max_age_s;
  out.max_speed_mps = autonomous ? cfg.path_follower_max_speed_mps
                                 : cfg.teleop_max_speed_mps;
  out.max_yaw_rate = autonomous ? cfg.path_follower_max_yaw_rate_rad_s
                                : cfg.teleop_max_yaw_rate;
  out.min_motion_speed_mps = cfg.teleop_min_motion_speed_mps;
  return out;
}

InputGateConfig inputGateConfig(const CliConfig &cfg) {
  InputGateConfig out;
  out.odom_max_age_s = cfg.odom_max_age_s;
  out.tf_max_age_s = cfg.tf_max_age_s;
  out.cloud_max_age_s = cfg.cloud_max_age_s;
  out.traversability_max_age_s = cfg.traversability_max_age_s;
  out.localization_health_max_age_s = cfg.localization_health_max_age_s;
  out.driver_control_max_age_s = cfg.driver_control_max_age_s;
  out.require_driver_control = true;
  out.future_tolerance_s = cfg.input_future_tolerance_s;
  out.recovery_frames = static_cast<std::uint32_t>(
      std::min<std::size_t>(cfg.input_recovery_frames,
                            std::numeric_limits<std::uint32_t>::max()));

  if (cfg.control_mode == ControlMode::Teleop) {
    out.require_odom = false;
    out.require_cloud = false;
    out.require_traversability = false;
    out.require_localization_health = false;
    return out;
  }

  out.require_odom = true;
  out.require_cloud = cfg.check_obstacle;
  out.require_traversability = cfg.use_traversability_cost;
  out.require_localization_health = cfg.control_mode == ControlMode::Autonomy;
  return out;
}

rolling::SegmentExecutorConfig rollingSegmentExecutorConfig(const CliConfig &cfg) {
  rolling::SegmentExecutorConfig out;
  out.segment.max_distance_m = cfg.segment_max_distance_m;
  out.segment.max_waypoints = cfg.segment_max_waypoints;
  out.map_input.max_grid_cells = cfg.segment_max_grid_cells;
  out.map_input.max_age_s = cfg.segment_map_max_age_s;
  out.risk.stop_threshold = static_cast<float>(cfg.segment_risk_stop);
  out.risk.resume_threshold = static_cast<float>(cfg.segment_risk_resume);
  return out;
}

/// Build StatusWriterConfig from parsed CLI config.
/// Pure mapping — no side effects.
StatusWriterConfig buildStatusWriterConfig(const CliConfig &cfg,
                                            const InputGateConfig &gate_cfg) {
  StatusWriterConfig out;
  const nav_kernel::FollowerParams follower = followerParams(cfg);
  out.control_mode = controlModeName(cfg.control_mode);
  out.product = cfg.product;
  out.product_session_id = cfg.product_session_id;
  out.domain_id = cfg.domain_id;
  out.tick_hz = cfg.tick_hz;
  out.max_speed_mps = follower.maxSpeed;
  out.min_speed_mps = follower.minSpeed;
  out.max_accel_mps2 = follower.maxAccel;
  out.follower_lookahead_m = follower.baseLookAheadDis;
  out.follower_goal_tolerance_m = follower.stopDisThre;
  out.waypoint_reached_m = cfg.waypoint_reached_m;
  out.goal_reached_m = cfg.goal_reached_m;
  out.corridor_lookahead_m = cfg.corridor_lookahead_m;
  out.nominal_dt_s = follower.nominalDt;
  out.publish_cmd_vel = cfg.publish_cmd_vel;
  out.check_obstacle = cfg.check_obstacle;
  out.use_traversability_cost = cfg.use_traversability_cost;
  out.allow_teleop_takeover = cfg.allow_teleop_takeover;
  out.teleop_local_planner = cfg.teleop_local_planner;
  out.teleop_planner_horizon_m = cfg.teleop_planner_horizon_m;
  out.teleop_planner_max_deviation_deg = cfg.teleop_planner_max_deviation_deg;
  out.map_path = cfg.map_path;
  out.global_planner = globalPlannerBackendName(cfg.global_planner);
  out.local_planner = nav_kernel::localPlannerBackendName(
      cfg.local_planner_backend);
  out.status_file = cfg.status_file;
  out.max_obstacle_points = cfg.max_obstacle_points;
  out.local_planner_threads = cfg.local_planner_threads;
  out.stop_confirmation_timeout_s = cfg.stop_confirmation_timeout_s;
  out.stop_confirmation_evidence =
      cfg.control_mode == ControlMode::Autonomy ? "driver_ack_and_odometry" : "driver_ack";
  out.local_planner_debug_candidate_limit = cfg.local_planner_debug_candidate_limit;
  out.local_map_debug_point_limit = cfg.local_map_debug_point_limit;
  out.planner_obstacle_height_min_m = cfg.teleop_obstacle_height_min_m;
  out.planner_obstacle_height_max_m = std::max(
      cfg.local_planner_obstacle_height_max_m,
      cfg.teleop_obstacle_height_max_m);
  out.obstacle_voxel_size_m = cfg.obstacle_voxel_size_m;
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

nav_kernel::LocalPlannerParams buildLocalPlannerParams(const CliConfig &cfg) {
  nav_kernel::LocalPlannerParams out;
  const double planner_obstacle_margin_m =
      cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Scan
          ? 0.0
          : cfg.teleop_obstacle_margin_m;
  out.backend = cfg.local_planner_backend;
  if (cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Cmu) {
    // Upstream Go2 autonomous navigation is forward-only.  Planner and
    // Follower must agree, otherwise a rearward candidate makes the follower
    // turn around to chase a path that will be replanned on the next frame.
    out.twoWayDrive = false;
  }
  out.autonomySpeed = cfg.path_follower_max_speed_mps;
  // CMU scales its path range by autonomySpeed / maxSpeed. Normalize against
  // the speed this Product can actually command, not an unrelated 1 m/s.
  out.maxSpeed = std::max(cfg.path_follower_max_speed_mps, 1e-9);
  out.adjacentRange = cfg.corridor_lookahead_m;
  out.vehicleLength = cfg.vehicle_length_m;
  out.vehicleWidth = cfg.vehicle_width_m;
  out.scan.cylinderRadius = cfg.collision_cylinder_radius_m;
  out.scan.cylinderOffset = cfg.collision_cylinder_offset_m;
  out.scan.bodyClearanceBelow = cfg.collision_clearance_below_m;
  out.scan.bodyClearanceAbove = cfg.collision_clearance_above_m;
  out.scan.maxAcceleration = cfg.path_follower_max_accel_mps2;
  // Executor receives the map->body pose.  The LiDAR extrinsic is already
  // applied when sensor-origin geometry is constructed and must not shift
  // the body pose a second time inside the local Planner, which accepts no
  // sensor-origin offset by design.
  out.obstacleHeightThre = cfg.teleop_obstacle_height_min_m;
  out.obstacleHeightMax = std::max(
      cfg.local_planner_obstacle_height_max_m,
      cfg.teleop_obstacle_height_max_m);
  // The CMU planner's fourth point value is terrain height above local ground.
  // Its official runtime enables terrain analysis and replaces raw registered
  // scans with the fresh terrain cloud before candidate scoring.
  out.useTerrainAnalysis =
      cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Cmu;
  // SCAN's configured double cylinder is already the complete hard footprint.
  // CMU uses the physical rectangle and one explicit configured margin.
  // Live-obstacle overlay radius is not robot geometry.
  out.footprintPadding = planner_obstacle_margin_m;
  // CMU already rejects colliding candidates through its correspondence table
  // and exact candidate sweep.  A second post-selection near-field gate can
  // otherwise report pathFound=true and then suppress that same path forever.
  if (cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Cmu) {
    out.nearFieldStopDis = 0.0;
  }
  out.useTraversabilityCost = cfg.use_traversability_cost;
  out.traversabilityHardCost = cfg.traversability_hard_cost;
  out.traversabilitySoftCost = cfg.traversability_soft_cost;
  out.traversabilityWeight = cfg.traversability_weight;
  out.scoringThreads = static_cast<int>(cfg.local_planner_threads);
  out.debugCandidateLimit = static_cast<int>(cfg.local_planner_debug_candidate_limit);
  return out;
}

/// Build ExecutorConfig from parsed CLI config.
/// Pure mapping — no side effects.
lingtu::nav::navigation::ExecutorConfig
buildExecutorConfig(const CliConfig &cfg) {
  lingtu::nav::navigation::ExecutorConfig out;
  const nav_kernel::LocalPlannerParams planner = buildLocalPlannerParams(cfg);
  out.max_speed = cfg.path_follower_max_speed_mps;
  out.corridor_lookahead_m = cfg.corridor_lookahead_m;
  out.waypoint_reached_m = cfg.waypoint_reached_m;
  out.goal_reached_m = cfg.goal_reached_m;
  out.goal_height_tolerance_m =
      cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Scan
          ? std::max(cfg.goal_reached_m, planner.scan.routeZTolerance)
          : cfg.goal_reached_m;
  out.teleop_intent_horizon_m = cfg.teleop_planner_horizon_m;
  out.teleop_intent_max_deviation_deg = cfg.teleop_planner_max_deviation_deg;
  out.follower = followerParams(cfg);
  out.recovery.behavior_order = cfg.recovery_order;
  out.recovery.blocked_interval_s = cfg.recovery_blocked_interval_s;
  out.recovery.rotation_timeout_s = cfg.recovery_rotation_timeout_s;
  out.recovery.translation_timeout_s = cfg.recovery_translation_timeout_s;
  out.recovery.max_attempts = cfg.recovery_max_attempts;
  out.recovery.translation_speed_mps = cfg.recovery_translation_speed_mps;
  out.recovery.rotation_rate_rad_s = cfg.recovery_rotation_rate_rad_s;
  out.recovery.min_rotation_rad = cfg.recovery_min_rotation_rad;
  out.recovery.max_rotation_rad = cfg.recovery_max_rotation_rad;
  out.recovery.rotation_candidate_step_rad =
      cfg.recovery_rotation_candidate_step_rad;
  out.recovery.rotation_sample_step_rad =
      cfg.recovery_rotation_sample_step_rad;
  return out;
}

}  // namespace lingtu::nav::endpoint
