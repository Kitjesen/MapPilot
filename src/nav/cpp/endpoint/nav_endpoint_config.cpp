#include "nav_endpoint_config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace lingtu::nav::endpoint {
namespace {

std::string envOrEmpty(const char *name) {
  const char *value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

bool parseBool(const std::string &raw, const char *name) {
  std::string value = raw;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::runtime_error(std::string(name) + " expects true/false or 1/0");
}

ControlMode parseControlMode(const std::string &raw, const char *name) {
  std::string value = raw;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "autonomy" || value == "autonomous" || value == "nav") {
    return ControlMode::Autonomy;
  }
  if (value == "teleop") {
    return ControlMode::Teleop;
  }
  if (value == "teleop_avoid") {
    return ControlMode::TeleopAvoid;
  }
  throw std::runtime_error(std::string(name) + " expects autonomy, teleop, or teleop_avoid");
}

GlobalPlannerBackend parseGlobalPlannerBackend(const std::string &raw, const char *name) {
  std::string value = raw;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "octoplanner3d" || value == "octo") {
    return GlobalPlannerBackend::OctoPlanner3D;
  }
  if (value == "far") {
    return GlobalPlannerBackend::Far;
  }
  throw std::runtime_error(std::string(name) + " expects octoplanner3d or far");
}

void applyEnvDouble(double &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = std::stod(value);
  }
}

void applyEnvInt(int &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = std::stoi(value);
  }
}

void applyEnvSize(std::size_t &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = static_cast<std::size_t>(std::stoull(value));
  }
}

void applyEnvBool(bool &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = parseBool(value, name);
  }
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
  out.cmd_max_age_s = cfg.teleop_cmd_max_age_s;
  out.max_speed_mps = cfg.teleop_max_speed_mps;
  out.max_yaw_rate = cfg.teleop_max_yaw_rate;
  out.slow_distance_m = cfg.teleop_slow_distance_m;
  out.stop_distance_m = cfg.teleop_stop_distance_m;
  out.linear_slow_scale = cfg.teleop_linear_slow_scale;
  out.min_motion_speed_mps = cfg.teleop_min_motion_speed_mps;
  out.obstacle_height_min_m = cfg.teleop_obstacle_height_min_m;
  out.obstacle_height_max_m = cfg.teleop_obstacle_height_max_m;
  out.obstacle_margin_m = cfg.teleop_obstacle_margin_m + cfg.live_obstacle_inflation_radius_m;
  out.traversability_hard_cost = cfg.teleop_traversability_hard_cost;
  out.traversability_soft_cost = cfg.teleop_traversability_soft_cost;
  out.vehicle_length_m = cfg.vehicle_length_m;
  out.vehicle_width_m = cfg.vehicle_width_m;
  out.check_obstacle = cfg.check_obstacle;
  out.use_traversability_cost = cfg.use_traversability_cost;
  return out;
}

TeleopSafetyConfig teleopSafetyConfig(const CliConfig &cfg) {
  return commandSafetyConfig(cfg);
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
      std::min<std::size_t>(cfg.input_recovery_frames, std::numeric_limits<std::uint32_t>::max()));

  if (cfg.control_mode == ControlMode::Teleop) {
    out.require_odom = false;
    out.require_cloud = false;
    out.require_traversability = false;
    out.require_localization_health = false;
    return out;
  }

  // Spatial obstacle checks require a pose, and both assisted teleop and
  // autonomy must stop when localization health is absent or unhealthy.
  out.require_odom = true;
  out.require_cloud = cfg.check_obstacle;
  out.require_traversability = cfg.use_traversability_cost;
  out.require_localization_health = true;
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

CliConfig parseArgs(int argc, char **argv) {
  CliConfig cfg;
  const std::string control_mode = envOrEmpty("LINGTU_NAV_CONTROL_MODE");
  if (!control_mode.empty()) {
    cfg.control_mode = parseControlMode(control_mode, "LINGTU_NAV_CONTROL_MODE");
  }
  const std::string global_planner = envOrEmpty("LINGTU_NAV_GLOBAL_PLANNER");
  if (!global_planner.empty()) {
    cfg.global_planner = parseGlobalPlannerBackend(global_planner, "LINGTU_NAV_GLOBAL_PLANNER");
  }
  cfg.path_library_dir = envOrEmpty("LINGTU_LOCAL_PLANNER_PATHS");
  cfg.map_root = envOrEmpty("NAV_MAP_DIR");
  cfg.map_path = envOrEmpty("LINGTU_ACTIVE_PLANNER_MAP");
  cfg.status_file = envOrEmpty("LINGTU_NAV_STATUS_FILE");
  cfg.estop_latch_file = envOrEmpty("LINGTU_NAV_ESTOP_LATCH_FILE");
  if (!envOrEmpty("LINGTU_NAV_PROFILE").empty()) {
    throw std::runtime_error("LINGTU_NAV_PROFILE is removed; use LINGTU_PRODUCT");
  }
  cfg.product = envOrEmpty("LINGTU_PRODUCT");
  cfg.config_fingerprint = envOrEmpty("LINGTU_NAV_CONFIG_FINGERPRINT");
  applyEnvDouble(cfg.nav_max_speed_mps, "LINGTU_NAV_MAX_SPEED_MPS");
  applyEnvDouble(cfg.nav_max_accel_mps2, "LINGTU_NAV_MAX_ACCEL_MPS2");
  applyEnvDouble(cfg.stop_confirmation_timeout_s, "LINGTU_NAV_STOP_CONFIRMATION_TIMEOUT_S");
  applyEnvDouble(cfg.corridor_lookahead_m, "LINGTU_NAV_CORRIDOR_LOOKAHEAD_M");
  applyEnvDouble(cfg.segment_max_distance_m, "LINGTU_NAV_SEGMENT_MAX_DISTANCE_M");
  applyEnvSize(cfg.segment_max_waypoints, "LINGTU_NAV_SEGMENT_MAX_WAYPOINTS");
  applyEnvSize(cfg.segment_max_grid_cells, "LINGTU_NAV_SEGMENT_MAX_GRID_CELLS");
  applyEnvDouble(cfg.segment_risk_stop, "LINGTU_NAV_SEGMENT_RISK_STOP");
  applyEnvDouble(cfg.segment_risk_resume, "LINGTU_NAV_SEGMENT_RISK_RESUME");
  applyEnvDouble(cfg.segment_map_max_age_s, "LINGTU_NAV_SEGMENT_MAP_MAX_AGE_S");
  cfg.path_follower_max_speed_mps = cfg.nav_max_speed_mps;
  cfg.path_follower_max_accel_mps2 = cfg.nav_max_accel_mps2;
  applyEnvDouble(cfg.waypoint_reached_m, "LINGTU_NAV_WAYPOINT_REACHED_M");
  applyEnvDouble(cfg.goal_reached_m, "LINGTU_NAV_GOAL_REACHED_M");
  applyEnvDouble(cfg.path_follower_goal_tolerance_m, "LINGTU_NAV_PATH_FOLLOWER_GOAL_TOLERANCE_M");
  applyEnvDouble(cfg.path_follower_lookahead_m, "LINGTU_NAV_PATH_FOLLOWER_LOOKAHEAD_M");
  applyEnvDouble(cfg.path_follower_max_speed_mps, "LINGTU_NAV_PATH_FOLLOWER_MAX_SPEED_MPS");
  applyEnvDouble(cfg.path_follower_min_speed_mps, "LINGTU_NAV_PATH_FOLLOWER_MIN_SPEED_MPS");
  applyEnvDouble(cfg.path_follower_max_accel_mps2, "LINGTU_NAV_PATH_FOLLOWER_MAX_ACCEL_MPS2");
  const std::string publish_cmd_vel = envOrEmpty("LINGTU_NAV_PUBLISH_CMD_VEL");
  if (!publish_cmd_vel.empty()) {
    cfg.publish_cmd_vel = parseBool(publish_cmd_vel, "LINGTU_NAV_PUBLISH_CMD_VEL");
  }
  const std::string check_obstacle = envOrEmpty("LINGTU_NAV_CHECK_OBSTACLE");
  if (!check_obstacle.empty()) {
    cfg.check_obstacle = parseBool(check_obstacle, "LINGTU_NAV_CHECK_OBSTACLE");
  }
  const std::string use_traversability = envOrEmpty("LINGTU_NAV_USE_TRAVERSABILITY_COST");
  if (!use_traversability.empty()) {
    cfg.use_traversability_cost =
        parseBool(use_traversability, "LINGTU_NAV_USE_TRAVERSABILITY_COST");
  }
  applyEnvDouble(cfg.traversability_max_age_s, "LINGTU_NAV_TRAVERSABILITY_MAX_AGE_S");
  applyEnvDouble(cfg.localization_health_max_age_s, "LINGTU_NAV_LOCALIZATION_HEALTH_MAX_AGE_S");
  applyEnvDouble(cfg.driver_control_max_age_s, "LINGTU_DRIVER_CONTROL_MAX_AGE_S");
  applyEnvDouble(cfg.terrain_map_max_age_s, "LINGTU_NAV_TERRAIN_MAP_MAX_AGE_S");
  applyEnvDouble(cfg.odom_max_age_s, "LINGTU_NAV_ODOM_MAX_AGE_S");
  applyEnvDouble(cfg.tf_max_age_s, "LINGTU_NAV_TF_MAX_AGE_S");
  applyEnvDouble(cfg.cloud_max_age_s, "LINGTU_NAV_CLOUD_MAX_AGE_S");
  applyEnvDouble(cfg.cloud_pose_max_gap_s, "LINGTU_NAV_CLOUD_POSE_MAX_GAP_S");
  applyEnvDouble(cfg.input_future_tolerance_s, "LINGTU_NAV_INPUT_FUTURE_TOLERANCE_S");
  applyEnvSize(cfg.input_recovery_frames, "LINGTU_NAV_INPUT_RECOVERY_FRAMES");
  applyEnvDouble(cfg.traversability_hard_cost, "LINGTU_NAV_TRAVERSABILITY_HARD_COST");
  applyEnvDouble(cfg.traversability_soft_cost, "LINGTU_NAV_TRAVERSABILITY_SOFT_COST");
  applyEnvDouble(cfg.traversability_weight, "LINGTU_NAV_TRAVERSABILITY_WEIGHT");
  applyEnvDouble(cfg.local_planner_obstacle_height_max_m,
                 "LINGTU_NAV_LOCAL_PLANNER_OBSTACLE_HEIGHT_MAX_M");
  applyEnvSize(cfg.max_obstacle_points, "LINGTU_NAV_DDS_MAX_OBSTACLE_POINTS");
  applyEnvSize(cfg.local_planner_threads, "LINGTU_NAV_LOCAL_PLANNER_THREADS");
  applyEnvSize(cfg.local_planner_debug_candidate_limit,
               "LINGTU_NAV_LOCAL_PLANNER_DEBUG_CANDIDATES");
  applyEnvSize(cfg.local_map_debug_point_limit, "LINGTU_NAV_LOCAL_MAP_DEBUG_POINTS");
  applyEnvDouble(cfg.obstacle_voxel_size_m, "LINGTU_NAV_OBSTACLE_VOXEL_SIZE_M");
  applyEnvDouble(cfg.obstacle_registered_share, "LINGTU_NAV_OBSTACLE_REGISTERED_SHARE");
  applyEnvDouble(cfg.obstacle_terrain_share, "LINGTU_NAV_OBSTACLE_TERRAIN_SHARE");
  applyEnvDouble(cfg.obstacle_terrain_ext_share, "LINGTU_NAV_OBSTACLE_TERRAIN_EXT_SHARE");
  applyEnvDouble(cfg.live_obstacle_decay_s, "LINGTU_NAV_LIVE_OBSTACLE_DECAY_S");
  applyEnvDouble(cfg.live_obstacle_inflation_radius_m,
                 "LINGTU_NAV_LIVE_OBSTACLE_INFLATION_RADIUS_M");
  applyEnvDouble(cfg.live_obstacle_ray_clear_max_range_m,
                 "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEAR_MAX_RANGE_M");
  applyEnvDouble(cfg.live_obstacle_ray_clearing_interval_s,
                 "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING_INTERVAL_S");
  applyEnvSize(cfg.live_obstacle_max_clearing_rays, "LINGTU_NAV_LIVE_OBSTACLE_MAX_CLEARING_RAYS");
  applyEnvInt(cfg.live_obstacle_min_hits, "LINGTU_NAV_LIVE_OBSTACLE_MIN_HITS");
  applyEnvBool(cfg.live_obstacle_ray_clearing, "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING");
  applyEnvBool(cfg.allow_teleop_takeover, "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER");
  applyEnvBool(cfg.teleop_local_planner, "LINGTU_TELEOP_LOCAL_PLANNER");
  applyEnvDouble(cfg.teleop_cmd_max_age_s, "LINGTU_TELEOP_CMD_MAX_AGE_S");
  applyEnvDouble(cfg.teleop_max_speed_mps, "LINGTU_TELEOP_MAX_SPEED_MPS");
  applyEnvDouble(cfg.teleop_max_yaw_rate, "LINGTU_TELEOP_MAX_YAW_RATE");
  applyEnvDouble(cfg.teleop_slow_distance_m, "LINGTU_TELEOP_SLOW_DISTANCE_M");
  applyEnvDouble(cfg.teleop_stop_distance_m, "LINGTU_TELEOP_STOP_DISTANCE_M");
  applyEnvDouble(cfg.teleop_linear_slow_scale, "LINGTU_TELEOP_LINEAR_SLOW_SCALE");
  applyEnvDouble(cfg.teleop_min_motion_speed_mps, "LINGTU_TELEOP_MIN_MOTION_SPEED_MPS");
  applyEnvDouble(cfg.teleop_obstacle_height_min_m, "LINGTU_TELEOP_OBSTACLE_HEIGHT_MIN_M");
  applyEnvDouble(cfg.teleop_obstacle_height_max_m, "LINGTU_TELEOP_OBSTACLE_HEIGHT_MAX_M");
  applyEnvDouble(cfg.teleop_obstacle_margin_m, "LINGTU_TELEOP_OBSTACLE_MARGIN_M");
  applyEnvDouble(cfg.teleop_traversability_hard_cost, "LINGTU_TELEOP_TRAVERSABILITY_HARD_COST");
  applyEnvDouble(cfg.teleop_traversability_soft_cost, "LINGTU_TELEOP_TRAVERSABILITY_SOFT_COST");
  applyEnvDouble(cfg.teleop_planner_horizon_m, "LINGTU_TELEOP_PLANNER_HORIZON_M");
  applyEnvDouble(cfg.teleop_planner_max_deviation_deg, "LINGTU_TELEOP_PLANNER_MAX_DEVIATION_DEG");
  applyEnvDouble(cfg.vehicle_length_m, "LINGTU_NAV_VEHICLE_LENGTH_M");
  applyEnvDouble(cfg.vehicle_width_m, "LINGTU_NAV_VEHICLE_WIDTH_M");
  applyEnvDouble(cfg.sensor_offset_x_m, "LINGTU_NAV_SENSOR_OFFSET_X_M");
  applyEnvDouble(cfg.sensor_offset_y_m, "LINGTU_NAV_SENSOR_OFFSET_Y_M");
  applyEnvDouble(cfg.sensor_offset_z_m, "LINGTU_NAV_SENSOR_OFFSET_Z_M");
  applyEnvDouble(cfg.octoplanner_options.robot_radius, "LINGTU_NAV_OCTO_ROBOT_RADIUS_M");
  applyEnvDouble(cfg.octoplanner_options.body_clearance_below_m,
                 "LINGTU_NAV_OCTO_BODY_CLEARANCE_BELOW_M");
  applyEnvDouble(cfg.octoplanner_options.body_clearance_above_m,
                 "LINGTU_NAV_OCTO_BODY_CLEARANCE_ABOVE_M");
  applyEnvInt(cfg.octoplanner_options.max_iterations, "LINGTU_NAV_OCTO_MAX_ITERATIONS");
  applyEnvInt(cfg.octoplanner_options.snap_search_radius_cells,
              "LINGTU_NAV_OCTO_SNAP_RADIUS_CELLS");
  applyEnvBool(cfg.octoplanner_options.require_ground_support,
               "LINGTU_NAV_OCTO_REQUIRE_GROUND_SUPPORT");
  applyEnvBool(cfg.octoplanner_options.strict_direct_ground_support,
               "LINGTU_NAV_OCTO_STRICT_GROUND_SUPPORT");
  applyEnvInt(cfg.octoplanner_options.ground_support_xy_radius_cells,
              "LINGTU_NAV_OCTO_GROUND_SUPPORT_XY_RADIUS_CELLS");
  applyEnvInt(cfg.octoplanner_options.ground_support_depth_cells,
              "LINGTU_NAV_OCTO_GROUND_SUPPORT_DEPTH_CELLS");
  applyEnvDouble(cfg.octoplanner_options.support_height_m, "LINGTU_NAV_OCTO_SUPPORT_HEIGHT_M");
  applyEnvDouble(cfg.octoplanner_options.support_height_tolerance_m,
                 "LINGTU_NAV_OCTO_SUPPORT_HEIGHT_TOLERANCE_M");
  applyEnvInt(cfg.octoplanner_options.support_patch_radius_cells,
              "LINGTU_NAV_OCTO_SUPPORT_PATCH_RADIUS_CELLS");
  applyEnvInt(cfg.octoplanner_options.support_patch_min_samples,
              "LINGTU_NAV_OCTO_SUPPORT_PATCH_MIN_SAMPLES");
  applyEnvBool(cfg.octoplanner_options.enable_preblocked_costmap,
               "LINGTU_NAV_OCTO_ENABLE_PREBLOCKED_COSTMAP");
  applyEnvInt(cfg.octoplanner_options.preblocked_costmap_radius_cells,
              "LINGTU_NAV_OCTO_PREBLOCKED_RADIUS_CELLS");
  applyEnvDouble(cfg.octoplanner_options.preblocked_costmap_weight,
                 "LINGTU_NAV_OCTO_PREBLOCKED_WEIGHT");
  applyEnvBool(cfg.octoplanner_options.lowest_traversable_only,
               "LINGTU_NAV_OCTO_LOWEST_TRAVERSABLE_ONLY");
  applyEnvDouble(cfg.octoplanner_options.floor_change_penalty,
                 "LINGTU_NAV_OCTO_FLOOR_CHANGE_PENALTY");
  applyEnvDouble(cfg.octoplanner_options.max_step_height, "LINGTU_NAV_OCTO_MAX_STEP_HEIGHT_M");
  applyEnvDouble(cfg.octoplanner_options.max_slope, "LINGTU_NAV_OCTO_MAX_SLOPE");
  applyEnvBool(cfg.octoplanner_options.same_floor_preference,
               "LINGTU_NAV_OCTO_SAME_FLOOR_PREFERENCE");
  applyEnvDouble(cfg.octoplanner_options.same_floor_z_tolerance,
                 "LINGTU_NAV_OCTO_SAME_FLOOR_Z_TOLERANCE_M");
  applyEnvDouble(cfg.octoplanner_options.max_same_floor_z_excursion,
                 "LINGTU_NAV_OCTO_MAX_SAME_FLOOR_Z_EXCURSION_M");
  applyEnvInt(cfg.octoplanner_options.obstacle_clearance_radius_cells,
              "LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_RADIUS_CELLS");
  applyEnvDouble(cfg.octoplanner_options.obstacle_clearance_weight,
                 "LINGTU_NAV_OCTO_OBSTACLE_CLEARANCE_WEIGHT");
  applyEnvDouble(cfg.octoplanner_options.terminal_goal_tolerance_m,
                 "LINGTU_NAV_OCTO_TERMINAL_GOAL_TOLERANCE_M");
  applyEnvDouble(cfg.octoplanner_options.terminal_goal_xy_tolerance_m,
                 "LINGTU_NAV_OCTO_TERMINAL_GOAL_XY_TOLERANCE_M");
  applyEnvDouble(cfg.octoplanner_options.terminal_goal_z_tolerance_m,
                 "LINGTU_NAV_OCTO_TERMINAL_GOAL_Z_TOLERANCE_M");
  applyEnvDouble(cfg.far_options.robot_radius_m, "LINGTU_NAV_FAR_ROBOT_RADIUS_M");
  applyEnvDouble(cfg.far_options.obstacle_clearance_m, "LINGTU_NAV_FAR_OBSTACLE_CLEARANCE_M");
  applyEnvDouble(cfg.far_options.max_visibility_distance_m,
                 "LINGTU_NAV_FAR_MAX_VISIBILITY_DISTANCE_M");
  applyEnvDouble(cfg.far_options.unknown_cost_multiplier, "LINGTU_NAV_FAR_UNKNOWN_COST_MULTIPLIER");
  applyEnvInt(cfg.far_options.corner_separation_cells, "LINGTU_NAV_FAR_CORNER_SEPARATION_CELLS");
  applyEnvInt(cfg.far_options.snap_search_radius_cells, "LINGTU_NAV_FAR_SNAP_RADIUS_CELLS");
  applyEnvSize(cfg.far_options.max_graph_nodes, "LINGTU_NAV_FAR_MAX_GRAPH_NODES");
  applyEnvSize(cfg.far_options.max_visibility_pairs, "LINGTU_NAV_FAR_MAX_VISIBILITY_PAIRS");
  applyEnvSize(cfg.far_options.max_search_expansions, "LINGTU_NAV_FAR_MAX_SEARCH_EXPANSIONS");
  applyEnvBool(cfg.far_options.allow_unknown_fallback, "LINGTU_NAV_FAR_ALLOW_UNKNOWN_FALLBACK");
  applyEnvBool(cfg.far_options.simplify_path, "LINGTU_NAV_FAR_SIMPLIFY_PATH");

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--control-mode") {
      cfg.control_mode = parseControlMode(next(), "--control-mode");
    } else if (arg == "--global-planner") {
      cfg.global_planner = parseGlobalPlannerBackend(next(), "--global-planner");
    } else if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
    } else if (arg == "--max-speed-mps") {
      cfg.nav_max_speed_mps = std::stod(next());
      cfg.path_follower_max_speed_mps = cfg.nav_max_speed_mps;
    } else if (arg == "--max-accel-mps2") {
      cfg.nav_max_accel_mps2 = std::stod(next());
      cfg.path_follower_max_accel_mps2 = cfg.nav_max_accel_mps2;
    } else if (arg == "--corridor-lookahead-m") {
      cfg.corridor_lookahead_m = std::stod(next());
    } else if (arg == "--segment-max-distance-m") {
      cfg.segment_max_distance_m = std::stod(next());
    } else if (arg == "--segment-max-waypoints") {
      cfg.segment_max_waypoints = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--segment-max-grid-cells") {
      cfg.segment_max_grid_cells = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--segment-risk-stop") {
      cfg.segment_risk_stop = std::stod(next());
    } else if (arg == "--segment-risk-resume") {
      cfg.segment_risk_resume = std::stod(next());
    } else if (arg == "--segment-map-max-age-s") {
      cfg.segment_map_max_age_s = std::stod(next());
    } else if (arg == "--profile") {
      throw std::runtime_error("--profile is removed for native field identity; use --product");
    } else if (arg == "--product") {
      cfg.product = next();
    } else if (arg == "--config-fingerprint") {
      cfg.config_fingerprint = next();
    } else if (arg == "--waypoint-reached-m") {
      cfg.waypoint_reached_m = std::stod(next());
    } else if (arg == "--goal-reached-m") {
      cfg.goal_reached_m = std::stod(next());
    } else if (arg == "--path-follower-goal-tolerance-m") {
      cfg.path_follower_goal_tolerance_m = std::stod(next());
    } else if (arg == "--path-follower-lookahead-m") {
      cfg.path_follower_lookahead_m = std::stod(next());
    } else if (arg == "--path-follower-max-speed-mps") {
      cfg.path_follower_max_speed_mps = std::stod(next());
    } else if (arg == "--path-follower-min-speed-mps") {
      cfg.path_follower_min_speed_mps = std::stod(next());
    } else if (arg == "--path-follower-max-accel-mps2") {
      cfg.path_follower_max_accel_mps2 = std::stod(next());
    } else if (arg == "--status-s") {
      cfg.status_s = std::stod(next());
    } else if (arg == "--stop-confirmation-timeout-s") {
      cfg.stop_confirmation_timeout_s = std::stod(next());
    } else if (arg == "--max-obstacle-points") {
      cfg.max_obstacle_points = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--local-planner-threads") {
      cfg.local_planner_threads = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--local-planner-debug-candidates") {
      cfg.local_planner_debug_candidate_limit = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--local-map-debug-points") {
      cfg.local_map_debug_point_limit = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--obstacle-voxel-size-m") {
      cfg.obstacle_voxel_size_m = std::stod(next());
    } else if (arg == "--obstacle-registered-share") {
      cfg.obstacle_registered_share = std::stod(next());
    } else if (arg == "--obstacle-terrain-share") {
      cfg.obstacle_terrain_share = std::stod(next());
    } else if (arg == "--obstacle-terrain-ext-share") {
      cfg.obstacle_terrain_ext_share = std::stod(next());
    } else if (arg == "--live-obstacle-decay-s") {
      cfg.live_obstacle_decay_s = std::stod(next());
    } else if (arg == "--live-obstacle-inflation-radius-m") {
      cfg.live_obstacle_inflation_radius_m = std::stod(next());
    } else if (arg == "--live-obstacle-ray-clear-max-range-m") {
      cfg.live_obstacle_ray_clear_max_range_m = std::stod(next());
    } else if (arg == "--live-obstacle-ray-clearing-interval-s") {
      cfg.live_obstacle_ray_clearing_interval_s = std::stod(next());
    } else if (arg == "--live-obstacle-max-clearing-rays") {
      cfg.live_obstacle_max_clearing_rays = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--live-obstacle-min-hits") {
      cfg.live_obstacle_min_hits = std::stoi(next());
    } else if (arg == "--live-obstacle-ray-clearing") {
      cfg.live_obstacle_ray_clearing = parseBool(next(), "--live-obstacle-ray-clearing");
    } else if (arg == "--path-library") {
      cfg.path_library_dir = next();
    } else if (arg == "--map-root") {
      cfg.map_root = next();
    } else if (arg == "--map") {
      cfg.map_path = next();
    } else if (arg == "--publish-cmd-vel") {
      cfg.publish_cmd_vel = parseBool(next(), "--publish-cmd-vel");
    } else if (arg == "--check-obstacle") {
      cfg.check_obstacle = parseBool(next(), "--check-obstacle");
    } else if (arg == "--use-traversability-cost") {
      cfg.use_traversability_cost = parseBool(next(), "--use-traversability-cost");
    } else if (arg == "--allow-teleop-takeover") {
      cfg.allow_teleop_takeover = parseBool(next(), "--allow-teleop-takeover");
    } else if (arg == "--teleop-local-planner") {
      cfg.teleop_local_planner = parseBool(next(), "--teleop-local-planner");
    } else if (arg == "--traversability-max-age-s") {
      cfg.traversability_max_age_s = std::stod(next());
    } else if (arg == "--localization-health-max-age-s") {
      cfg.localization_health_max_age_s = std::stod(next());
    } else if (arg == "--driver-control-max-age-s") {
      cfg.driver_control_max_age_s = std::stod(next());
    } else if (arg == "--terrain-map-max-age-s") {
      cfg.terrain_map_max_age_s = std::stod(next());
    } else if (arg == "--odom-max-age-s") {
      cfg.odom_max_age_s = std::stod(next());
    } else if (arg == "--tf-max-age-s") {
      cfg.tf_max_age_s = std::stod(next());
    } else if (arg == "--cloud-max-age-s") {
      cfg.cloud_max_age_s = std::stod(next());
    } else if (arg == "--cloud-pose-max-gap-s") {
      cfg.cloud_pose_max_gap_s = std::stod(next());
    } else if (arg == "--input-future-tolerance-s") {
      cfg.input_future_tolerance_s = std::stod(next());
    } else if (arg == "--input-recovery-frames") {
      cfg.input_recovery_frames = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--traversability-hard-cost") {
      cfg.traversability_hard_cost = std::stod(next());
    } else if (arg == "--traversability-soft-cost") {
      cfg.traversability_soft_cost = std::stod(next());
    } else if (arg == "--traversability-weight") {
      cfg.traversability_weight = std::stod(next());
    } else if (arg == "--local-planner-obstacle-height-max-m") {
      cfg.local_planner_obstacle_height_max_m = std::stod(next());
    } else if (arg == "--vehicle-length-m") {
      cfg.vehicle_length_m = std::stod(next());
    } else if (arg == "--vehicle-width-m") {
      cfg.vehicle_width_m = std::stod(next());
    } else if (arg == "--sensor-offset-x-m") {
      cfg.sensor_offset_x_m = std::stod(next());
    } else if (arg == "--sensor-offset-y-m") {
      cfg.sensor_offset_y_m = std::stod(next());
    } else if (arg == "--sensor-offset-z-m") {
      cfg.sensor_offset_z_m = std::stod(next());
    } else if (arg == "--teleop-slow-distance-m") {
      cfg.teleop_slow_distance_m = std::stod(next());
    } else if (arg == "--teleop-stop-distance-m") {
      cfg.teleop_stop_distance_m = std::stod(next());
    } else if (arg == "--teleop-min-motion-speed-mps") {
      cfg.teleop_min_motion_speed_mps = std::stod(next());
    } else if (arg == "--teleop-traversability-soft-cost") {
      cfg.teleop_traversability_soft_cost = std::stod(next());
    } else if (arg == "--teleop-traversability-hard-cost") {
      cfg.teleop_traversability_hard_cost = std::stod(next());
    } else if (arg == "--teleop-planner-horizon-m") {
      cfg.teleop_planner_horizon_m = std::stod(next());
    } else if (arg == "--teleop-planner-max-deviation-deg") {
      cfg.teleop_planner_max_deviation_deg = std::stod(next());
    } else if (arg == "--octo-robot-radius-m") {
      cfg.octoplanner_options.robot_radius = std::stod(next());
    } else if (arg == "--octo-body-clearance-below-m") {
      cfg.octoplanner_options.body_clearance_below_m = std::stod(next());
    } else if (arg == "--octo-body-clearance-above-m") {
      cfg.octoplanner_options.body_clearance_above_m = std::stod(next());
    } else if (arg == "--octo-max-iterations") {
      cfg.octoplanner_options.max_iterations = std::stoi(next());
    } else if (arg == "--octo-snap-radius-cells") {
      cfg.octoplanner_options.snap_search_radius_cells = std::stoi(next());
    } else if (arg == "--octo-require-ground-support") {
      cfg.octoplanner_options.require_ground_support =
          parseBool(next(), "--octo-require-ground-support");
    } else if (arg == "--octo-strict-ground-support") {
      cfg.octoplanner_options.strict_direct_ground_support =
          parseBool(next(), "--octo-strict-ground-support");
    } else if (arg == "--octo-ground-support-xy-radius-cells") {
      cfg.octoplanner_options.ground_support_xy_radius_cells = std::stoi(next());
    } else if (arg == "--octo-ground-support-depth-cells") {
      cfg.octoplanner_options.ground_support_depth_cells = std::stoi(next());
    } else if (arg == "--octo-support-height-m") {
      cfg.octoplanner_options.support_height_m = std::stod(next());
    } else if (arg == "--octo-support-height-tolerance-m") {
      cfg.octoplanner_options.support_height_tolerance_m = std::stod(next());
    } else if (arg == "--octo-support-patch-radius-cells") {
      cfg.octoplanner_options.support_patch_radius_cells = std::stoi(next());
    } else if (arg == "--octo-support-patch-min-samples") {
      cfg.octoplanner_options.support_patch_min_samples = std::stoi(next());
    } else if (arg == "--octo-enable-preblocked-costmap") {
      cfg.octoplanner_options.enable_preblocked_costmap =
          parseBool(next(), "--octo-enable-preblocked-costmap");
    } else if (arg == "--octo-preblocked-radius-cells") {
      cfg.octoplanner_options.preblocked_costmap_radius_cells = std::stoi(next());
    } else if (arg == "--octo-preblocked-weight") {
      cfg.octoplanner_options.preblocked_costmap_weight = std::stod(next());
    } else if (arg == "--octo-lowest-traversable-only") {
      cfg.octoplanner_options.lowest_traversable_only =
          parseBool(next(), "--octo-lowest-traversable-only");
    } else if (arg == "--octo-floor-change-penalty") {
      cfg.octoplanner_options.floor_change_penalty = std::stod(next());
    } else if (arg == "--octo-max-step-height-m") {
      cfg.octoplanner_options.max_step_height = std::stod(next());
    } else if (arg == "--octo-max-slope") {
      cfg.octoplanner_options.max_slope = std::stod(next());
    } else if (arg == "--octo-same-floor-preference") {
      cfg.octoplanner_options.same_floor_preference =
          parseBool(next(), "--octo-same-floor-preference");
    } else if (arg == "--octo-same-floor-z-tolerance-m") {
      cfg.octoplanner_options.same_floor_z_tolerance = std::stod(next());
    } else if (arg == "--octo-max-same-floor-z-excursion-m") {
      cfg.octoplanner_options.max_same_floor_z_excursion = std::stod(next());
    } else if (arg == "--octo-obstacle-clearance-radius-cells") {
      cfg.octoplanner_options.obstacle_clearance_radius_cells = std::stoi(next());
    } else if (arg == "--octo-obstacle-clearance-weight") {
      cfg.octoplanner_options.obstacle_clearance_weight = std::stod(next());
    } else if (arg == "--octo-terminal-goal-tolerance-m") {
      cfg.octoplanner_options.terminal_goal_tolerance_m = std::stod(next());
    } else if (arg == "--octo-terminal-goal-xy-tolerance-m") {
      cfg.octoplanner_options.terminal_goal_xy_tolerance_m = std::stod(next());
    } else if (arg == "--octo-terminal-goal-z-tolerance-m") {
      cfg.octoplanner_options.terminal_goal_z_tolerance_m = std::stod(next());
    } else if (arg == "--far-robot-radius-m") {
      cfg.far_options.robot_radius_m = std::stod(next());
    } else if (arg == "--far-obstacle-clearance-m") {
      cfg.far_options.obstacle_clearance_m = std::stod(next());
    } else if (arg == "--far-max-visibility-distance-m") {
      cfg.far_options.max_visibility_distance_m = std::stod(next());
    } else if (arg == "--far-unknown-cost-multiplier") {
      cfg.far_options.unknown_cost_multiplier = std::stod(next());
    } else if (arg == "--far-corner-separation-cells") {
      cfg.far_options.corner_separation_cells = std::stoi(next());
    } else if (arg == "--far-snap-radius-cells") {
      cfg.far_options.snap_search_radius_cells = std::stoi(next());
    } else if (arg == "--far-max-graph-nodes") {
      cfg.far_options.max_graph_nodes = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--far-max-visibility-pairs") {
      cfg.far_options.max_visibility_pairs = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--far-max-search-expansions") {
      cfg.far_options.max_search_expansions = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--far-allow-unknown-fallback") {
      cfg.far_options.allow_unknown_fallback = parseBool(next(), "--far-allow-unknown-fallback");
    } else if (arg == "--far-simplify-path") {
      cfg.far_options.simplify_path = parseBool(next(), "--far-simplify-path");
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--estop-latch-file") {
      cfg.estop_latch_file = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: navd --path-library DIR "
          "[--control-mode autonomy|teleop|teleop_avoid] "
          "[--product PRODUCT] "
          "[--global-planner octoplanner3d|far] "
          "[--allow-teleop-takeover true|false] "
          "[--teleop-local-planner true|false] "
          "[--map-root DIR] [--map PLANNER_ARTIFACT] [--domain-id N] [--tick-hz HZ] "
          "[--max-speed-mps MPS] [--max-accel-mps2 MPS2] "
          "[--corridor-lookahead-m M] "
          "[--segment-max-distance-m M] [--segment-max-waypoints N] "
          "[--segment-max-grid-cells N] [--segment-risk-stop C] "
          "[--segment-risk-resume C] [--segment-map-max-age-s S] "
          "[--max-obstacle-points N] [--publish-cmd-vel true|false] "
          "[--local-planner-debug-candidates N] [--local-map-debug-points N] "
          "[--obstacle-voxel-size-m M] [--live-obstacle-decay-s S] "
          "[--live-obstacle-inflation-radius-m M] "
          "[--live-obstacle-ray-clearing true|false] "
          "[--check-obstacle true|false] "
          "[--use-traversability-cost true|false] [--traversability-max-age-s S] "
          "[--localization-health-max-age-s S] "
          "[--driver-control-max-age-s S] "
          "[--terrain-map-max-age-s S] [--traversability-hard-cost C] "
          "[--input-future-tolerance-s S] "
          "[--traversability-soft-cost C] [--traversability-weight W] "
          "[--local-planner-obstacle-height-max-m M] "
          "[--vehicle-length-m M] [--vehicle-width-m M] [--sensor-offset-x-m M] "
          "[--sensor-offset-y-m M] [--sensor-offset-z-m M] "
          "[--teleop-slow-distance-m M] [--teleop-stop-distance-m M] "
          "[--teleop-min-motion-speed-mps MPS] "
          "[--teleop-traversability-soft-cost C] "
          "[--teleop-traversability-hard-cost C] "
          "[--octo-require-ground-support true|false] "
          "[--octo-support-height-m M] [--octo-max-step-height-m M] "
          "[--octo-max-slope DZ_PER_DXY] "
          "[--octo-ground-support-xy-radius-cells N] "
          "[--octo-obstacle-clearance-radius-cells N] [--status-file PATH] "
          "[--far-allow-unknown-fallback true|false] "
          "[--estop-latch-file PATH]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  // Resolve the backend-specific artifact after CLI parsing so
  // `--global-planner far` also selects LINGTU_ACTIVE_OCCUPANCY. A generic
  // LINGTU_ACTIVE_PLANNER_MAP or explicit --map remains authoritative.
  if (cfg.map_path.empty()) {
    cfg.map_path = cfg.global_planner == GlobalPlannerBackend::Far
                       ? envOrEmpty("LINGTU_ACTIVE_OCCUPANCY")
                       : envOrEmpty("LINGTU_ACTIVE_OCTOMAP");
  }

  if ((cfg.control_mode == ControlMode::Autonomy || cfg.teleop_local_planner) &&
      cfg.path_library_dir.empty()) {
    throw std::runtime_error(
        "path library is required; pass --path-library or set LINGTU_LOCAL_PLANNER_PATHS");
  }
  if (!std::isfinite(cfg.tick_hz) || !std::isfinite(cfg.nav_max_speed_mps) ||
      !std::isfinite(cfg.nav_max_accel_mps2) || !std::isfinite(cfg.waypoint_reached_m) ||
      !std::isfinite(cfg.goal_reached_m) || !std::isfinite(cfg.path_follower_goal_tolerance_m) ||
      !std::isfinite(cfg.path_follower_lookahead_m) ||
      !std::isfinite(cfg.path_follower_max_speed_mps) ||
      !std::isfinite(cfg.path_follower_min_speed_mps) ||
      !std::isfinite(cfg.path_follower_max_accel_mps2) ||
      !std::isfinite(cfg.corridor_lookahead_m) ||
      !std::isfinite(cfg.local_planner_obstacle_height_max_m) ||
      !std::isfinite(cfg.teleop_planner_horizon_m) ||
      !std::isfinite(cfg.driver_control_max_age_s) ||
      !std::isfinite(cfg.teleop_planner_max_deviation_deg)) {
    throw std::runtime_error(
        "tick_hz, motion limits, corridor_lookahead_m, local planner obstacle height, driver "
        "control max age, and teleop planner limits must be finite");
  }
  if (!rollingSegmentExecutorConfig(cfg).valid()) {
    throw std::runtime_error(
        "rolling segment policy is invalid: distance must be within [0.1, 100] m, waypoints "
        "within [2, 4096], grid cells within [1, 1048576], map age within [0.01, 10] s, "
        "and risk thresholds within [0, 100] with stop >= resume");
  }
  if (cfg.driver_control_max_age_s <= 0.0) {
    throw std::runtime_error("driver_control_max_age_s must be strictly positive");
  }
  cfg.tick_hz = std::max(1.0, cfg.tick_hz);
  if (!std::isfinite(cfg.stop_confirmation_timeout_s) || cfg.stop_confirmation_timeout_s < 0.5 ||
      cfg.stop_confirmation_timeout_s > 30.0) {
    throw std::runtime_error(
        "stop confirmation timeout must be finite and within [0.5, 30] seconds");
  }
  cfg.nav_max_speed_mps = std::max(0.0, cfg.nav_max_speed_mps);
  cfg.nav_max_accel_mps2 = std::max(0.0, cfg.nav_max_accel_mps2);
  cfg.waypoint_reached_m = std::max(0.01, cfg.waypoint_reached_m);
  cfg.goal_reached_m = std::max(0.01, cfg.goal_reached_m);
  cfg.path_follower_goal_tolerance_m = std::max(0.01, cfg.path_follower_goal_tolerance_m);
  cfg.path_follower_lookahead_m = std::max(0.01, cfg.path_follower_lookahead_m);
  cfg.path_follower_max_speed_mps = std::max(0.0, cfg.path_follower_max_speed_mps);
  cfg.path_follower_min_speed_mps = std::max(0.0, cfg.path_follower_min_speed_mps);
  cfg.path_follower_max_accel_mps2 = std::max(0.0, cfg.path_follower_max_accel_mps2);
  if (cfg.path_follower_min_speed_mps > cfg.path_follower_max_speed_mps) {
    throw std::runtime_error("path follower minimum speed must not exceed maximum speed");
  }
  if (cfg.path_follower_goal_tolerance_m > cfg.goal_reached_m) {
    throw std::runtime_error("path follower goal tolerance must not exceed goal reached threshold");
  }
  cfg.corridor_lookahead_m = std::max(0.2, cfg.corridor_lookahead_m);
  cfg.traversability_max_age_s = std::max(0.0, cfg.traversability_max_age_s);
  cfg.localization_health_max_age_s = std::max(0.0, cfg.localization_health_max_age_s);
  cfg.terrain_map_max_age_s = std::max(0.0, cfg.terrain_map_max_age_s);
  cfg.odom_max_age_s = std::max(0.0, cfg.odom_max_age_s);
  cfg.tf_max_age_s = std::max(0.0, cfg.tf_max_age_s);
  cfg.cloud_max_age_s = std::max(0.0, cfg.cloud_max_age_s);
  cfg.cloud_pose_max_gap_s = std::max(0.0, cfg.cloud_pose_max_gap_s);
  cfg.input_future_tolerance_s = std::max(0.0, cfg.input_future_tolerance_s);
  cfg.input_recovery_frames = std::max<std::size_t>(1, cfg.input_recovery_frames);
  cfg.traversability_hard_cost = std::max(0.0, cfg.traversability_hard_cost);
  cfg.traversability_soft_cost = std::max(0.0, cfg.traversability_soft_cost);
  cfg.traversability_weight = std::max(0.0, cfg.traversability_weight);
  cfg.local_planner_obstacle_height_max_m = std::max(0.2, cfg.local_planner_obstacle_height_max_m);
  cfg.local_planner_threads = std::clamp<std::size_t>(cfg.local_planner_threads, 1, 4);
  cfg.local_planner_debug_candidate_limit = std::min<std::size_t>(
      cfg.local_planner_debug_candidate_limit, static_cast<std::size_t>(nav_kernel::kRotDirs));
  cfg.local_map_debug_point_limit = std::min<std::size_t>(cfg.local_map_debug_point_limit, 4096);
  cfg.obstacle_voxel_size_m = std::max(0.02, cfg.obstacle_voxel_size_m);
  cfg.obstacle_registered_share = std::max(0.0, cfg.obstacle_registered_share);
  cfg.obstacle_terrain_share = std::max(0.0, cfg.obstacle_terrain_share);
  cfg.obstacle_terrain_ext_share = std::max(0.0, cfg.obstacle_terrain_ext_share);
  if (cfg.obstacle_registered_share <= 0.0 && cfg.obstacle_terrain_share <= 0.0 &&
      cfg.obstacle_terrain_ext_share <= 0.0) {
    cfg.obstacle_registered_share = 1.0;
  }
  cfg.live_obstacle_decay_s = std::max(0.0, cfg.live_obstacle_decay_s);
  cfg.live_obstacle_inflation_radius_m = std::max(0.0, cfg.live_obstacle_inflation_radius_m);
  cfg.live_obstacle_ray_clear_max_range_m = std::max(0.0, cfg.live_obstacle_ray_clear_max_range_m);
  cfg.live_obstacle_ray_clearing_interval_s =
      std::max(0.0, cfg.live_obstacle_ray_clearing_interval_s);
  cfg.live_obstacle_min_hits = std::max(1, cfg.live_obstacle_min_hits);
  cfg.teleop_cmd_max_age_s = std::max(0.0, cfg.teleop_cmd_max_age_s);
  cfg.teleop_max_speed_mps = std::max(0.0, cfg.teleop_max_speed_mps);
  cfg.teleop_max_yaw_rate = std::max(0.0, cfg.teleop_max_yaw_rate);
  cfg.teleop_slow_distance_m = std::max(0.0, cfg.teleop_slow_distance_m);
  cfg.teleop_stop_distance_m = std::max(0.0, cfg.teleop_stop_distance_m);
  cfg.teleop_linear_slow_scale = std::max(0.0, std::min(1.0, cfg.teleop_linear_slow_scale));
  cfg.teleop_min_motion_speed_mps = std::max(0.0, cfg.teleop_min_motion_speed_mps);
  cfg.teleop_obstacle_height_min_m = std::max(0.0, cfg.teleop_obstacle_height_min_m);
  cfg.teleop_obstacle_height_max_m =
      std::max(cfg.teleop_obstacle_height_min_m, cfg.teleop_obstacle_height_max_m);
  cfg.teleop_obstacle_margin_m = std::max(0.0, cfg.teleop_obstacle_margin_m);
  cfg.teleop_traversability_hard_cost = std::max(0.0, cfg.teleop_traversability_hard_cost);
  cfg.teleop_traversability_soft_cost = std::max(0.0, cfg.teleop_traversability_soft_cost);
  cfg.teleop_planner_horizon_m = std::max(0.5, cfg.teleop_planner_horizon_m);
  cfg.teleop_planner_max_deviation_deg =
      std::clamp(cfg.teleop_planner_max_deviation_deg, 0.0, 90.0);
  if (cfg.teleop_stop_distance_m > cfg.teleop_slow_distance_m) {
    throw std::runtime_error("teleop stop distance must not exceed slow distance");
  }
  if (cfg.teleop_traversability_soft_cost > cfg.teleop_traversability_hard_cost) {
    throw std::runtime_error("teleop traversability soft cost must not exceed hard cost");
  }
  if (cfg.traversability_soft_cost > cfg.traversability_hard_cost) {
    throw std::runtime_error("navigation traversability soft cost must not exceed hard cost");
  }
  cfg.vehicle_length_m = std::max(0.1, cfg.vehicle_length_m);
  cfg.vehicle_width_m = std::max(0.1, cfg.vehicle_width_m);
  auto &octo = cfg.octoplanner_options;
  octo.robot_radius = std::max(0.0, octo.robot_radius);
  octo.body_clearance_below_m = std::max(0.0, octo.body_clearance_below_m);
  octo.body_clearance_above_m = std::max(0.0, octo.body_clearance_above_m);
  octo.max_iterations = std::max(1, octo.max_iterations);
  octo.snap_search_radius_cells = std::max(0, octo.snap_search_radius_cells);
  octo.ground_support_xy_radius_cells = std::max(0, octo.ground_support_xy_radius_cells);
  octo.ground_support_depth_cells = std::max(1, octo.ground_support_depth_cells);
  octo.support_height_m = std::max(0.0, octo.support_height_m);
  octo.support_height_tolerance_m = std::max(0.0, octo.support_height_tolerance_m);
  octo.support_patch_radius_cells = std::max(0, octo.support_patch_radius_cells);
  octo.support_patch_min_samples = std::clamp(octo.support_patch_min_samples, 0, 5);
  octo.preblocked_costmap_radius_cells = std::max(0, octo.preblocked_costmap_radius_cells);
  octo.preblocked_costmap_weight = std::max(0.0, octo.preblocked_costmap_weight);
  octo.floor_change_penalty = std::max(0.0, octo.floor_change_penalty);
  octo.max_step_height = std::max(0.0, octo.max_step_height);
  octo.max_slope = std::max(0.0, octo.max_slope);
  octo.same_floor_z_tolerance = std::max(0.0, octo.same_floor_z_tolerance);
  octo.max_same_floor_z_excursion = std::max(0.0, octo.max_same_floor_z_excursion);
  octo.obstacle_clearance_radius_cells = std::max(0, octo.obstacle_clearance_radius_cells);
  octo.obstacle_clearance_weight = std::max(0.0, octo.obstacle_clearance_weight);
  octo.terminal_goal_tolerance_m = std::max(0.0, octo.terminal_goal_tolerance_m);
  octo.terminal_goal_xy_tolerance_m = std::max(0.0, octo.terminal_goal_xy_tolerance_m);
  octo.terminal_goal_z_tolerance_m = std::max(0.0, octo.terminal_goal_z_tolerance_m);
  // Construction performs the same strict validation used by the planning
  // runtime. Normalize kernel argument errors at the CLI boundary so callers
  // receive one stable startup-configuration error category.
  try {
    const lingtu::nav::plan::far::FarPlanner far_config_probe(cfg.far_options);
    (void)far_config_probe;
  } catch (const std::invalid_argument &exc) {
    throw std::runtime_error(std::string("invalid FAR configuration: ") + exc.what());
  }
  if (cfg.control_mode == ControlMode::Teleop) {
    cfg.check_obstacle = false;
    cfg.use_traversability_cost = false;
  } else if (cfg.control_mode == ControlMode::TeleopAvoid) {
    cfg.check_obstacle = true;
    cfg.use_traversability_cost = true;
  }
  if (cfg.teleop_local_planner && cfg.use_traversability_cost &&
      std::abs(cfg.traversability_hard_cost - cfg.teleop_traversability_hard_cost) > 1e-9) {
    throw std::runtime_error(
        "assisted teleop planner and final safety traversability hard costs must match");
  }
  return cfg;
}

}  // namespace lingtu::nav::endpoint
