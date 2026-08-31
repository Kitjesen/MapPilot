#include "runtime/config/config.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include "planning/rolling/segment.hpp"

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

nav_kernel::LocalPlannerBackend parseLocalPlannerBackend(
    const std::string& raw, const char* name) {
  std::string value = raw;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "cmu") return nav_kernel::LocalPlannerBackend::Cmu;
  if (value == "scan") return nav_kernel::LocalPlannerBackend::Scan;
  throw std::runtime_error(std::string(name) + " expects cmu or scan");
}

std::string trim(const std::string &raw);

std::vector<nav_kernel::RecoveryAction> parseRecoveryOrder(
    const std::string& raw, const char* name) {
  std::vector<nav_kernel::RecoveryAction> order;
  std::size_t start = 0;
  while (start <= raw.size()) {
    const std::size_t comma = raw.find(',', start);
    std::string value = trim(raw.substr(
        start, comma == std::string::npos ? std::string::npos : comma - start));
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) {
                     return static_cast<char>(std::tolower(c));
                   });
    nav_kernel::RecoveryAction action = nav_kernel::RecoveryAction::None;
    if (value == "translate") {
      action = nav_kernel::RecoveryAction::Translate;
    } else if (value == "rotate") {
      action = nav_kernel::RecoveryAction::Rotate;
    } else {
      throw std::runtime_error(
          std::string(name) + " expects a comma-separated subset of translate,rotate");
    }
    if (std::find(order.begin(), order.end(), action) != order.end()) {
      throw std::runtime_error(std::string(name) + " must not repeat an action");
    }
    order.push_back(action);
    if (comma == std::string::npos) break;
    start = comma + 1U;
  }
  if (order.empty()) {
    throw std::runtime_error(std::string(name) + " must not be empty");
  }
  return order;
}

nav_kernel::VelocityFeedbackMode parseVelocityFeedbackMode(const std::string &raw,
                                                           const char *name) {
  std::string value = raw;
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "open") {
    return nav_kernel::VelocityFeedbackMode::kOpenLoop;
  }
  if (value == "closed") {
    return nav_kernel::VelocityFeedbackMode::kClosedLoop;
  }
  throw std::runtime_error(std::string(name) + " expects open or closed");
}

std::string trim(const std::string &raw) {
  const auto first = std::find_if_not(raw.begin(), raw.end(),
                                      [](unsigned char c) { return std::isspace(c) != 0; });
  const auto last = std::find_if_not(raw.rbegin(), raw.rend(),
                                     [](unsigned char c) { return std::isspace(c) != 0; })
                        .base();
  return first < last ? std::string(first, last) : std::string{};
}

double parseDouble(const std::string &raw, const char *name) {
  const std::string value = trim(raw);
  std::size_t consumed = 0U;
  const double parsed = std::stod(value, &consumed);
  if (value.empty() || consumed != value.size()) {
    throw std::runtime_error(std::string(name) + " expects a number");
  }
  return parsed;
}

bool isGeometryOption(const std::string &arg) {
  return arg == "--vehicle-length-m" || arg == "--vehicle-width-m" ||
         arg == "--collision-cylinder-radius-m" ||
         arg == "--collision-cylinder-offset-m" ||
         arg == "--collision-clearance-below-m" ||
         arg == "--collision-clearance-above-m" ||
         arg == "--sensor-offset-x-m" || arg == "--sensor-offset-y-m" ||
         arg == "--sensor-offset-z-m";
}

void applyGeometryOption(CliConfig &cfg, const std::string &arg,
                         const std::string &raw) {
  const double value = parseDouble(raw, arg.c_str());
  if (arg == "--vehicle-length-m")
    cfg.vehicle_length_m = value;
  else if (arg == "--vehicle-width-m")
    cfg.vehicle_width_m = value;
  else if (arg == "--collision-cylinder-radius-m")
    cfg.collision_cylinder_radius_m = value;
  else if (arg == "--collision-cylinder-offset-m")
    cfg.collision_cylinder_offset_m = value;
  else if (arg == "--collision-clearance-below-m")
    cfg.collision_clearance_below_m = value;
  else if (arg == "--collision-clearance-above-m")
    cfg.collision_clearance_above_m = value;
  else if (arg == "--sensor-offset-x-m")
    cfg.sensor_offset_x_m = value;
  else if (arg == "--sensor-offset-y-m")
    cfg.sensor_offset_y_m = value;
  else
    cfg.sensor_offset_z_m = value;
}

bool isRecoveryOption(const std::string& arg) {
  return arg == "--recovery-order" ||
         arg == "--recovery-blocked-interval-s" ||
         arg == "--recovery-rotation-timeout-s" ||
         arg == "--recovery-translation-timeout-s" ||
         arg == "--recovery-max-attempts" ||
         arg == "--recovery-translation-speed-mps" ||
         arg == "--recovery-rotation-rate-rad-s" ||
         arg == "--recovery-min-rotation-rad" ||
         arg == "--recovery-max-rotation-rad" ||
         arg == "--recovery-rotation-candidate-step-rad" ||
         arg == "--recovery-rotation-sample-step-rad";
}

int parseInt(const std::string &raw, const char *name);

void applyRecoveryOption(CliConfig& cfg, const std::string& arg,
                         const std::string& raw) {
  if (arg == "--recovery-order")
    cfg.recovery_order = parseRecoveryOrder(raw, arg.c_str());
  else if (arg == "--recovery-blocked-interval-s")
    cfg.recovery_blocked_interval_s = parseDouble(raw, arg.c_str());
  else if (arg == "--recovery-rotation-timeout-s")
    cfg.recovery_rotation_timeout_s = parseDouble(raw, arg.c_str());
  else if (arg == "--recovery-translation-timeout-s")
    cfg.recovery_translation_timeout_s = parseDouble(raw, arg.c_str());
  else if (arg == "--recovery-max-attempts")
    cfg.recovery_max_attempts = parseInt(raw, arg.c_str());
  else if (arg == "--recovery-translation-speed-mps")
    cfg.recovery_translation_speed_mps = parseDouble(raw, arg.c_str());
  else if (arg == "--recovery-rotation-rate-rad-s")
    cfg.recovery_rotation_rate_rad_s = parseDouble(raw, arg.c_str());
  else if (arg == "--recovery-min-rotation-rad")
    cfg.recovery_min_rotation_rad = parseDouble(raw, arg.c_str());
  else if (arg == "--recovery-max-rotation-rad")
    cfg.recovery_max_rotation_rad = parseDouble(raw, arg.c_str());
  else if (arg == "--recovery-rotation-candidate-step-rad")
    cfg.recovery_rotation_candidate_step_rad = parseDouble(raw, arg.c_str());
  else
    cfg.recovery_rotation_sample_step_rad = parseDouble(raw, arg.c_str());
}

int parseInt(const std::string &raw, const char *name) {
  const std::string value = trim(raw);
  std::size_t consumed = 0U;
  const int parsed = std::stoi(value, &consumed);
  if (value.empty() || consumed != value.size()) {
    throw std::runtime_error(std::string(name) + " expects an integer");
  }
  return parsed;
}

std::int64_t parsePositiveInt64(const std::string &raw, const char *name) {
  const std::string value = trim(raw);
  std::size_t consumed = 0U;
  const auto parsed = std::stoll(value, &consumed);
  if (value.empty() || consumed != value.size() || parsed <= 0) {
    throw std::runtime_error(std::string(name) + " expects a positive integer");
  }
  return parsed;
}

std::size_t parseSize(const std::string &raw, const char *name) {
  const std::string value = trim(raw);
  if (value.empty() || value.front() == '-') {
    throw std::runtime_error(std::string(name) + " expects a non-negative integer");
  }
  std::size_t consumed = 0U;
  const auto parsed = std::stoull(value, &consumed);
  if (consumed != value.size() || parsed > std::numeric_limits<std::size_t>::max()) {
    throw std::runtime_error(std::string(name) + " expects a non-negative integer");
  }
  return static_cast<std::size_t>(parsed);
}

void applyEnvDouble(double &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = parseDouble(value, name);
  }
}

void applyEnvInt(int &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = parseInt(value, name);
  }
}

void applyEnvSize(std::size_t &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = parseSize(value, name);
  }
}

void applyEnvBool(bool &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = parseBool(value, name);
  }
}

void applyEnvRecoveryOrder(std::vector<nav_kernel::RecoveryAction>& target,
                           const char* name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = parseRecoveryOrder(value, name);
  }
}

void applyEnvVelocityFeedbackMode(nav_kernel::VelocityFeedbackMode &target, const char *name) {
  const std::string value = envOrEmpty(name);
  if (!value.empty()) {
    target = parseVelocityFeedbackMode(value, name);
  }
}

void validateVelocityAxis(const nav_kernel::VelocityAxisLimits &limits, const char *name) {
  if (!std::isfinite(limits.minimum) || !std::isfinite(limits.maximum) ||
      !std::isfinite(limits.acceleration) || !std::isfinite(limits.deceleration) ||
      !std::isfinite(limits.deadband)) {
    throw std::runtime_error(std::string("invalid velocity smoother ") + name +
                             " axis: limits must be finite");
  }
  if (limits.minimum > 0.0 || limits.maximum < 0.0 || limits.minimum > limits.maximum) {
    throw std::runtime_error(std::string("invalid velocity smoother ") + name +
                             " axis: limits must contain zero");
  }
  if (limits.acceleration <= 0.0 || limits.deceleration <= 0.0) {
    throw std::runtime_error(std::string("invalid velocity smoother ") + name +
                             " axis: rate limits must be positive");
  }
  if (limits.deadband < 0.0 ||
      limits.deadband > std::max(std::abs(limits.minimum), std::abs(limits.maximum))) {
    throw std::runtime_error(std::string("invalid velocity smoother ") + name +
                             " axis: deadband exceeds the axis range");
  }
}

void validateVelocitySmootherConfig(const CliConfig &cfg) {
  const auto &config = cfg.velocity_smoother;
  validateVelocityAxis(config.x, "x");
  validateVelocityAxis(config.y, "y");
  validateVelocityAxis(config.yaw, "yaw");
  if (!std::isfinite(config.target_timeout_s) || config.target_timeout_s <= 0.0) {
    throw std::runtime_error("invalid velocity smoother target timeout");
  }
  if (!std::isfinite(config.feedback_timeout_s) || config.feedback_timeout_s <= 0.0) {
    throw std::runtime_error("invalid velocity smoother feedback timeout");
  }
  if (!std::isfinite(config.max_step_s) || config.max_step_s <= 0.0) {
    throw std::runtime_error("invalid velocity smoother maximum step");
  }
  if (!std::isfinite(config.future_tolerance_s) || config.future_tolerance_s < 0.0) {
    throw std::runtime_error("invalid velocity smoother future tolerance");
  }
  if (cfg.velocity_smoother_enabled &&
      config.feedback_mode == nav_kernel::VelocityFeedbackMode::kClosedLoop) {
    throw std::runtime_error(
        "velocity smoother closed-loop feedback is unavailable in the native endpoint");
  }
}

std::filesystem::path normalizedAbsolutePath(const std::string &raw, const char *name) {
  const std::filesystem::path value(raw);
  if (raw.empty() || !value.is_absolute() || value.lexically_normal() != value) {
    throw std::runtime_error(std::string(name) + " must be a normalized absolute path");
  }
  return value;
}

}  // namespace

CliConfig parseArgs(int argc, char **argv) {
  CliConfig cfg;
  applyEnvInt(cfg.domain_id, "LINGTU_DDS_DOMAIN_ID");
  applyEnvDouble(cfg.tick_hz, "LINGTU_NAV_DDS_TICK_HZ");
  applyEnvDouble(cfg.status_s, "LINGTU_NAV_STATUS_S");
  const std::string control_mode = envOrEmpty("LINGTU_NAV_CONTROL_MODE");
  if (!control_mode.empty()) {
    cfg.control_mode = parseControlMode(control_mode, "LINGTU_NAV_CONTROL_MODE");
  }
  const std::string global_planner = envOrEmpty("NAV_GLOBAL_PLANNER");
  if (!global_planner.empty()) {
    cfg.global_planner = parseGlobalPlannerBackend(global_planner, "NAV_GLOBAL_PLANNER");
  }
  cfg.path_library_dir = envOrEmpty("LINGTU_LOCAL_PLANNER_PATHS");
  cfg.inspection_dir = envOrEmpty("LINGTU_INSPECTION_DIR");
  cfg.status_file = envOrEmpty("LINGTU_NAV_STATUS_FILE");
  cfg.estop_latch_file = envOrEmpty("LINGTU_NAV_ESTOP_LATCH_FILE");
  cfg.geofence_file = envOrEmpty("LINGTU_NAV_GEOFENCE_FILE");
  if (!envOrEmpty("LINGTU_NAV_PROFILE").empty()) {
    throw std::runtime_error("LINGTU_NAV_PROFILE is removed; use LINGTU_PRODUCT");
  }
  const std::string local_planner =
      envOrEmpty("LINGTU_NAV_LOCAL_PLANNER_BACKEND");
  if (!local_planner.empty()) {
    cfg.local_planner_backend = parseLocalPlannerBackend(
        local_planner, "LINGTU_NAV_LOCAL_PLANNER_BACKEND");
  }
  cfg.product = envOrEmpty("LINGTU_PRODUCT");
  cfg.product_session_id = envOrEmpty("LINGTU_PRODUCT_SESSION_ID");
  applyEnvDouble(cfg.nav_max_speed_mps, "LINGTU_NAV_MAX_SPEED_MPS");
  applyEnvDouble(cfg.nav_max_accel_mps2, "LINGTU_NAV_MAX_ACCEL_MPS2");
  applyEnvDouble(cfg.control_loop_deadline_miss_ratio_limit,
                 "LINGTU_NAV_CONTROL_LOOP_DEADLINE_MISS_RATIO_LIMIT");
  applyEnvDouble(cfg.control_loop_p95_utilization_limit,
                 "LINGTU_NAV_CONTROL_LOOP_P95_UTILIZATION_LIMIT");
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
  applyEnvDouble(cfg.path_follower_max_yaw_rate_rad_s,
                 "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_RATE_RAD_S");
  applyEnvDouble(cfg.path_follower_max_yaw_accel_rad_s2,
                 "LINGTU_NAV_PATH_FOLLOWER_MAX_YAW_ACCEL_RAD_S2");
  applyEnvDouble(cfg.path_follower_heading_align_enter_rad,
                 "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_ENTER_RAD");
  applyEnvDouble(cfg.path_follower_heading_align_exit_rad,
                 "LINGTU_NAV_PATH_FOLLOWER_HEADING_ALIGN_EXIT_RAD");
  applyEnvDouble(cfg.scan_follower.timeForward, "LINGTU_NAV_SCAN_TIME_FORWARD_S");
  applyEnvDouble(cfg.scan_follower.headingErrorThreshold,
                 "LINGTU_NAV_SCAN_HEADING_ERROR_RAD");
  applyEnvDouble(cfg.scan_follower.positionGain, "LINGTU_NAV_SCAN_POSITION_GAIN");
  applyEnvDouble(cfg.scan_follower.yawGain, "LINGTU_NAV_SCAN_YAW_GAIN");
  applyEnvDouble(cfg.scan_follower.maxVx, "LINGTU_NAV_SCAN_MAX_VX_MPS");
  applyEnvDouble(cfg.scan_follower.maxVy, "LINGTU_NAV_SCAN_MAX_VY_MPS");
  applyEnvDouble(cfg.scan_follower.maxYawRateRadS,
                 "LINGTU_NAV_SCAN_MAX_YAW_RATE_RAD_S");
  applyEnvRecoveryOrder(cfg.recovery_order, "LINGTU_NAV_RECOVERY_ORDER");
  applyEnvDouble(cfg.recovery_blocked_interval_s,
                 "LINGTU_NAV_RECOVERY_BLOCKED_INTERVAL_S");
  applyEnvDouble(cfg.recovery_rotation_timeout_s,
                 "LINGTU_NAV_RECOVERY_ROTATION_TIMEOUT_S");
  applyEnvDouble(cfg.recovery_translation_timeout_s,
                 "LINGTU_NAV_RECOVERY_TRANSLATION_TIMEOUT_S");
  applyEnvInt(cfg.recovery_max_attempts,
              "LINGTU_NAV_RECOVERY_MAX_ATTEMPTS");
  applyEnvDouble(cfg.recovery_translation_speed_mps,
                 "LINGTU_NAV_RECOVERY_TRANSLATION_SPEED_MPS");
  applyEnvDouble(cfg.recovery_rotation_rate_rad_s,
                 "LINGTU_NAV_RECOVERY_ROTATION_RATE_RAD_S");
  applyEnvDouble(cfg.recovery_min_rotation_rad,
                 "LINGTU_NAV_RECOVERY_MIN_ROTATION_RAD");
  applyEnvDouble(cfg.recovery_max_rotation_rad,
                 "LINGTU_NAV_RECOVERY_MAX_ROTATION_RAD");
  applyEnvDouble(cfg.recovery_rotation_candidate_step_rad,
                 "LINGTU_NAV_RECOVERY_ROTATION_CANDIDATE_STEP_RAD");
  applyEnvDouble(cfg.recovery_rotation_sample_step_rad,
                 "LINGTU_NAV_RECOVERY_ROTATION_SAMPLE_STEP_RAD");
  applyEnvBool(cfg.velocity_smoother_enabled, "LINGTU_NAV_SMOOTHER_ENABLED");
  applyEnvVelocityFeedbackMode(cfg.velocity_smoother.feedback_mode,
                               "LINGTU_NAV_SMOOTHER_FEEDBACK_MODE");
  applyEnvDouble(cfg.velocity_smoother.target_timeout_s,
                 "LINGTU_NAV_SMOOTHER_TARGET_TIMEOUT_S");
  applyEnvDouble(cfg.velocity_smoother.feedback_timeout_s,
                 "LINGTU_NAV_SMOOTHER_FEEDBACK_TIMEOUT_S");
  applyEnvDouble(cfg.velocity_smoother.max_step_s, "LINGTU_NAV_SMOOTHER_MAX_STEP_S");
  applyEnvDouble(cfg.velocity_smoother.future_tolerance_s,
                 "LINGTU_NAV_SMOOTHER_FUTURE_TOLERANCE_S");
  applyEnvBool(cfg.velocity_smoother.scale_velocities,
               "LINGTU_NAV_SMOOTHER_SCALE_VELOCITIES");
  applyEnvDouble(cfg.velocity_smoother.x.minimum, "LINGTU_NAV_SMOOTHER_X_MIN_MPS");
  applyEnvDouble(cfg.velocity_smoother.x.maximum, "LINGTU_NAV_SMOOTHER_X_MAX_MPS");
  applyEnvDouble(cfg.velocity_smoother.x.acceleration, "LINGTU_NAV_SMOOTHER_X_ACCEL_MPS2");
  applyEnvDouble(cfg.velocity_smoother.x.deceleration, "LINGTU_NAV_SMOOTHER_X_DECEL_MPS2");
  applyEnvDouble(cfg.velocity_smoother.x.deadband, "LINGTU_NAV_SMOOTHER_X_DEADBAND_MPS");
  applyEnvDouble(cfg.velocity_smoother.y.minimum, "LINGTU_NAV_SMOOTHER_Y_MIN_MPS");
  applyEnvDouble(cfg.velocity_smoother.y.maximum, "LINGTU_NAV_SMOOTHER_Y_MAX_MPS");
  applyEnvDouble(cfg.velocity_smoother.y.acceleration, "LINGTU_NAV_SMOOTHER_Y_ACCEL_MPS2");
  applyEnvDouble(cfg.velocity_smoother.y.deceleration, "LINGTU_NAV_SMOOTHER_Y_DECEL_MPS2");
  applyEnvDouble(cfg.velocity_smoother.y.deadband, "LINGTU_NAV_SMOOTHER_Y_DEADBAND_MPS");
  applyEnvDouble(cfg.velocity_smoother.yaw.minimum, "LINGTU_NAV_SMOOTHER_YAW_MIN_RADPS");
  applyEnvDouble(cfg.velocity_smoother.yaw.maximum, "LINGTU_NAV_SMOOTHER_YAW_MAX_RADPS");
  applyEnvDouble(cfg.velocity_smoother.yaw.acceleration,
                 "LINGTU_NAV_SMOOTHER_YAW_ACCEL_RADPS2");
  applyEnvDouble(cfg.velocity_smoother.yaw.deceleration,
                 "LINGTU_NAV_SMOOTHER_YAW_DECEL_RADPS2");
  applyEnvDouble(cfg.velocity_smoother.yaw.deadband,
                 "LINGTU_NAV_SMOOTHER_YAW_DEADBAND_RADPS");
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
  applyEnvDouble(cfg.live_obstacle_decay_s, "LINGTU_NAV_LIVE_OBSTACLE_DECAY_S");
  applyEnvDouble(cfg.live_obstacle_inflation_radius_m,
                 "LINGTU_NAV_LIVE_OBSTACLE_INFLATION_RADIUS_M");
  applyEnvDouble(cfg.live_obstacle_ray_clear_max_range_m,
                 "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEAR_MAX_RANGE_M");
  applyEnvDouble(cfg.live_obstacle_ray_clearing_interval_s,
                 "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING_INTERVAL_S");
  applyEnvSize(cfg.live_obstacle_max_clearing_rays, "LINGTU_NAV_LIVE_OBSTACLE_MAX_CLEARING_RAYS");
  applyEnvInt(cfg.live_obstacle_min_hits, "LINGTU_NAV_LIVE_OBSTACLE_MIN_HITS");
  applyEnvSize(cfg.dynamic_min_cells, "LINGTU_NAV_DYNAMIC_MIN_CELLS");
  applyEnvDouble(cfg.dynamic_min_speed_mps, "LINGTU_NAV_DYNAMIC_MIN_SPEED_MPS");
  applyEnvSize(cfg.dynamic_confirm_frames, "LINGTU_NAV_DYNAMIC_CONFIRM_FRAMES");
  applyEnvBool(cfg.live_obstacle_ray_clearing, "LINGTU_NAV_LIVE_OBSTACLE_RAY_CLEARING");
  applyEnvBool(cfg.allow_teleop_takeover, "LINGTU_NAV_ALLOW_TELEOP_TAKEOVER");
  applyEnvBool(cfg.teleop_local_planner, "LINGTU_TELEOP_LOCAL_PLANNER");
  applyEnvDouble(cfg.teleop_cmd_max_age_s, "LINGTU_TELEOP_CMD_MAX_AGE_S");
  applyEnvDouble(cfg.teleop_max_speed_mps, "LINGTU_TELEOP_MAX_SPEED_MPS");
  applyEnvDouble(cfg.teleop_max_yaw_rate, "LINGTU_TELEOP_MAX_YAW_RATE");
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
  applyEnvDouble(cfg.collision_cylinder_radius_m, "LINGTU_NAV_COLLISION_CYLINDER_RADIUS_M");
  applyEnvDouble(cfg.collision_cylinder_offset_m, "LINGTU_NAV_COLLISION_CYLINDER_OFFSET_M");
  applyEnvDouble(cfg.collision_clearance_below_m, "LINGTU_NAV_COLLISION_CLEARANCE_BELOW_M");
  applyEnvDouble(cfg.collision_clearance_above_m, "LINGTU_NAV_COLLISION_CLEARANCE_ABOVE_M");
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
    } else if (arg == "--local-planner") {
      cfg.local_planner_backend =
          parseLocalPlannerBackend(next(), "--local-planner");
    } else if (arg == "--domain-id") {
      cfg.domain_id = parseInt(next(), arg.c_str());
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = parseDouble(next(), arg.c_str());
    } else if (arg == "--control-loop-deadline-miss-ratio-limit") {
      cfg.control_loop_deadline_miss_ratio_limit = parseDouble(next(), arg.c_str());
    } else if (arg == "--control-loop-p95-utilization-limit") {
      cfg.control_loop_p95_utilization_limit = parseDouble(next(), arg.c_str());
    } else if (arg == "--max-speed-mps") {
      cfg.nav_max_speed_mps = parseDouble(next(), arg.c_str());
      cfg.path_follower_max_speed_mps = cfg.nav_max_speed_mps;
    } else if (arg == "--max-accel-mps2") {
      cfg.nav_max_accel_mps2 = parseDouble(next(), arg.c_str());
      cfg.path_follower_max_accel_mps2 = cfg.nav_max_accel_mps2;
    } else if (arg == "--corridor-lookahead-m") {
      cfg.corridor_lookahead_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--segment-max-distance-m") {
      cfg.segment_max_distance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--segment-max-waypoints") {
      cfg.segment_max_waypoints = parseSize(next(), arg.c_str());
    } else if (arg == "--segment-max-grid-cells") {
      cfg.segment_max_grid_cells = parseSize(next(), arg.c_str());
    } else if (arg == "--segment-risk-stop") {
      cfg.segment_risk_stop = parseDouble(next(), arg.c_str());
    } else if (arg == "--segment-risk-resume") {
      cfg.segment_risk_resume = parseDouble(next(), arg.c_str());
    } else if (arg == "--segment-map-max-age-s") {
      cfg.segment_map_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--profile") {
      throw std::runtime_error("--profile is removed for native field identity; use --product");
    } else if (arg == "--product") {
      cfg.product = next();
    } else if (arg == "--waypoint-reached-m") {
      cfg.waypoint_reached_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--goal-reached-m") {
      cfg.goal_reached_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-goal-tolerance-m") {
      cfg.path_follower_goal_tolerance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-lookahead-m") {
      cfg.path_follower_lookahead_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-max-speed-mps") {
      cfg.path_follower_max_speed_mps = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-min-speed-mps") {
      cfg.path_follower_min_speed_mps = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-max-accel-mps2") {
      cfg.path_follower_max_accel_mps2 = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-max-yaw-rate-rad-s") {
      cfg.path_follower_max_yaw_rate_rad_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-max-yaw-accel-rad-s2") {
      cfg.path_follower_max_yaw_accel_rad_s2 = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-heading-align-enter-rad") {
      cfg.path_follower_heading_align_enter_rad = parseDouble(next(), arg.c_str());
    } else if (arg == "--path-follower-heading-align-exit-rad") {
      cfg.path_follower_heading_align_exit_rad = parseDouble(next(), arg.c_str());
    } else if (isRecoveryOption(arg)) {
      applyRecoveryOption(cfg, arg, next());
    } else if (arg == "--status-s") {
      cfg.status_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--stop-confirmation-timeout-s") {
      cfg.stop_confirmation_timeout_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--max-obstacle-points") {
      cfg.max_obstacle_points = parseSize(next(), arg.c_str());
    } else if (arg == "--local-planner-threads") {
      cfg.local_planner_threads = parseSize(next(), arg.c_str());
    } else if (arg == "--local-planner-debug-candidates") {
      cfg.local_planner_debug_candidate_limit = parseSize(next(), arg.c_str());
    } else if (arg == "--local-map-debug-points") {
      cfg.local_map_debug_point_limit = parseSize(next(), arg.c_str());
    } else if (arg == "--obstacle-voxel-size-m") {
      cfg.obstacle_voxel_size_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--live-obstacle-decay-s") {
      cfg.live_obstacle_decay_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--live-obstacle-inflation-radius-m") {
      cfg.live_obstacle_inflation_radius_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--live-obstacle-ray-clear-max-range-m") {
      cfg.live_obstacle_ray_clear_max_range_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--live-obstacle-ray-clearing-interval-s") {
      cfg.live_obstacle_ray_clearing_interval_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--live-obstacle-max-clearing-rays") {
      cfg.live_obstacle_max_clearing_rays = parseSize(next(), arg.c_str());
    } else if (arg == "--live-obstacle-min-hits") {
      cfg.live_obstacle_min_hits = parseInt(next(), arg.c_str());
    } else if (arg == "--dynamic-min-cells") {
      cfg.dynamic_min_cells = parseSize(next(), arg.c_str());
    } else if (arg == "--dynamic-min-speed-mps") {
      cfg.dynamic_min_speed_mps = parseDouble(next(), arg.c_str());
    } else if (arg == "--dynamic-confirm-frames") {
      cfg.dynamic_confirm_frames = parseSize(next(), arg.c_str());
    } else if (arg == "--live-obstacle-ray-clearing") {
      cfg.live_obstacle_ray_clearing = parseBool(next(), "--live-obstacle-ray-clearing");
    } else if (arg == "--path-library") {
      cfg.path_library_dir = next();
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
      cfg.traversability_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--localization-health-max-age-s") {
      cfg.localization_health_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--driver-control-max-age-s") {
      cfg.driver_control_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--terrain-map-max-age-s") {
      cfg.terrain_map_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--odom-max-age-s") {
      cfg.odom_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--tf-max-age-s") {
      cfg.tf_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--cloud-max-age-s") {
      cfg.cloud_max_age_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--cloud-pose-max-gap-s") {
      cfg.cloud_pose_max_gap_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--input-future-tolerance-s") {
      cfg.input_future_tolerance_s = parseDouble(next(), arg.c_str());
    } else if (arg == "--input-recovery-frames") {
      cfg.input_recovery_frames = parseSize(next(), arg.c_str());
    } else if (arg == "--traversability-hard-cost") {
      cfg.traversability_hard_cost = parseDouble(next(), arg.c_str());
    } else if (arg == "--traversability-soft-cost") {
      cfg.traversability_soft_cost = parseDouble(next(), arg.c_str());
    } else if (arg == "--traversability-weight") {
      cfg.traversability_weight = parseDouble(next(), arg.c_str());
    } else if (arg == "--local-planner-obstacle-height-max-m") {
      cfg.local_planner_obstacle_height_max_m = parseDouble(next(), arg.c_str());
    } else if (isGeometryOption(arg)) {
      applyGeometryOption(cfg, arg, next());
    } else if (arg == "--teleop-min-motion-speed-mps") {
      cfg.teleop_min_motion_speed_mps = parseDouble(next(), arg.c_str());
    } else if (arg == "--teleop-traversability-soft-cost") {
      cfg.teleop_traversability_soft_cost = parseDouble(next(), arg.c_str());
    } else if (arg == "--teleop-traversability-hard-cost") {
      cfg.teleop_traversability_hard_cost = parseDouble(next(), arg.c_str());
    } else if (arg == "--teleop-planner-horizon-m") {
      cfg.teleop_planner_horizon_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--teleop-planner-max-deviation-deg") {
      cfg.teleop_planner_max_deviation_deg = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-robot-radius-m") {
      cfg.octoplanner_options.robot_radius = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-body-clearance-below-m") {
      cfg.octoplanner_options.body_clearance_below_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-body-clearance-above-m") {
      cfg.octoplanner_options.body_clearance_above_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-max-iterations") {
      cfg.octoplanner_options.max_iterations = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-snap-radius-cells") {
      cfg.octoplanner_options.snap_search_radius_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-require-ground-support") {
      cfg.octoplanner_options.require_ground_support =
          parseBool(next(), "--octo-require-ground-support");
    } else if (arg == "--octo-strict-ground-support") {
      cfg.octoplanner_options.strict_direct_ground_support =
          parseBool(next(), "--octo-strict-ground-support");
    } else if (arg == "--octo-ground-support-xy-radius-cells") {
      cfg.octoplanner_options.ground_support_xy_radius_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-ground-support-depth-cells") {
      cfg.octoplanner_options.ground_support_depth_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-support-height-m") {
      cfg.octoplanner_options.support_height_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-support-height-tolerance-m") {
      cfg.octoplanner_options.support_height_tolerance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-support-patch-radius-cells") {
      cfg.octoplanner_options.support_patch_radius_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-support-patch-min-samples") {
      cfg.octoplanner_options.support_patch_min_samples = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-enable-preblocked-costmap") {
      cfg.octoplanner_options.enable_preblocked_costmap =
          parseBool(next(), "--octo-enable-preblocked-costmap");
    } else if (arg == "--octo-preblocked-radius-cells") {
      cfg.octoplanner_options.preblocked_costmap_radius_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-preblocked-weight") {
      cfg.octoplanner_options.preblocked_costmap_weight = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-lowest-traversable-only") {
      cfg.octoplanner_options.lowest_traversable_only =
          parseBool(next(), "--octo-lowest-traversable-only");
    } else if (arg == "--octo-floor-change-penalty") {
      cfg.octoplanner_options.floor_change_penalty = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-max-step-height-m") {
      cfg.octoplanner_options.max_step_height = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-max-slope") {
      cfg.octoplanner_options.max_slope = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-same-floor-preference") {
      cfg.octoplanner_options.same_floor_preference =
          parseBool(next(), "--octo-same-floor-preference");
    } else if (arg == "--octo-same-floor-z-tolerance-m") {
      cfg.octoplanner_options.same_floor_z_tolerance = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-max-same-floor-z-excursion-m") {
      cfg.octoplanner_options.max_same_floor_z_excursion = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-obstacle-clearance-radius-cells") {
      cfg.octoplanner_options.obstacle_clearance_radius_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--octo-obstacle-clearance-weight") {
      cfg.octoplanner_options.obstacle_clearance_weight = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-terminal-goal-tolerance-m") {
      cfg.octoplanner_options.terminal_goal_tolerance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-terminal-goal-xy-tolerance-m") {
      cfg.octoplanner_options.terminal_goal_xy_tolerance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--octo-terminal-goal-z-tolerance-m") {
      cfg.octoplanner_options.terminal_goal_z_tolerance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--far-robot-radius-m") {
      cfg.far_options.robot_radius_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--far-obstacle-clearance-m") {
      cfg.far_options.obstacle_clearance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--far-max-visibility-distance-m") {
      cfg.far_options.max_visibility_distance_m = parseDouble(next(), arg.c_str());
    } else if (arg == "--far-unknown-cost-multiplier") {
      cfg.far_options.unknown_cost_multiplier = parseDouble(next(), arg.c_str());
    } else if (arg == "--far-corner-separation-cells") {
      cfg.far_options.corner_separation_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--far-snap-radius-cells") {
      cfg.far_options.snap_search_radius_cells = parseInt(next(), arg.c_str());
    } else if (arg == "--far-max-graph-nodes") {
      cfg.far_options.max_graph_nodes = parseSize(next(), arg.c_str());
    } else if (arg == "--far-max-visibility-pairs") {
      cfg.far_options.max_visibility_pairs = parseSize(next(), arg.c_str());
    } else if (arg == "--far-max-search-expansions") {
      cfg.far_options.max_search_expansions = parseSize(next(), arg.c_str());
    } else if (arg == "--far-simplify-path") {
      cfg.far_options.simplify_path = parseBool(next(), "--far-simplify-path");
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--estop-latch-file") {
      cfg.estop_latch_file = next();
    } else if (arg == "--geofence-file") {
      cfg.geofence_file = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: navd --path-library DIR "
          "[--control-mode autonomy|teleop|teleop_avoid] "
          "[--product PRODUCT] "
          "[--global-planner octoplanner3d|far] [--local-planner cmu|scan] "
          "[--allow-teleop-takeover true|false] "
          "[--teleop-local-planner true|false] "
          "[--map PLANNER_ARTIFACT] [--domain-id N] [--tick-hz HZ] "
          "[--control-loop-deadline-miss-ratio-limit RATIO] "
          "[--control-loop-p95-utilization-limit RATIO] "
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
          "[--dynamic-min-cells N] [--dynamic-min-speed-mps MPS] "
          "[--dynamic-confirm-frames N] "
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
          "[--estop-latch-file PATH] [--geofence-file PATH]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (cfg.map_path.empty()) {
    cfg.map_path = cfg.global_planner == GlobalPlannerBackend::Far
                       ? envOrEmpty("FAR_OCCUPANCY_PATH")
                       : envOrEmpty("OCTOPLANNER_MAP_PATH");
  }
  if (!cfg.map_path.empty()) {
    cfg.map_identity.map_id = envOrEmpty("LINGTU_MAP_ID");
    cfg.map_identity.frame_id = envOrEmpty("LINGTU_MAP_FRAME");
    cfg.map_identity.content_epoch = parsePositiveInt64(
        envOrEmpty("LINGTU_MAP_CONTENT_EPOCH"), "LINGTU_MAP_CONTENT_EPOCH");
    if (!cfg.map_identity.valid()) {
      throw std::runtime_error(
          "LINGTU_MAP_ID, LINGTU_MAP_CONTENT_EPOCH, and LINGTU_MAP_FRAME are required with a planner map");
    }
  }

  const std::string runtime_env = envOrEmpty("LINGTU_ENV");
  if (runtime_env == "sim") {
    const auto session_root =
        normalizedAbsolutePath(envOrEmpty("LINGTU_SESSION_ROOT"), "LINGTU_SESSION_ROOT");
    if (cfg.product_session_id.empty()) {
      throw std::runtime_error("LINGTU_PRODUCT_SESSION_ID is required in simulation");
    }
    if (cfg.product.empty() || std::any_of(cfg.product.begin(), cfg.product.end(),
                    [](unsigned char c) { return std::isspace(c) != 0; })) {
      throw std::runtime_error("LINGTU_PRODUCT must be non-empty text without whitespace in simulation");
    }
    if (cfg.status_file.empty()) {
      cfg.status_file = (session_root / "nav.status.json").string();
    }
    if (cfg.inspection_dir.empty()) {
      cfg.inspection_dir = (session_root / "inspection").string();
    }
  }

  if ((cfg.control_mode == ControlMode::Autonomy || cfg.teleop_local_planner) &&
      cfg.local_planner_backend == nav_kernel::LocalPlannerBackend::Cmu &&
      cfg.path_library_dir.empty()) {
    throw std::runtime_error(
        "path library is required; pass --path-library or set LINGTU_LOCAL_PLANNER_PATHS");
  }
  validateVelocitySmootherConfig(cfg);
  if (!std::isfinite(cfg.tick_hz) ||
      !std::isfinite(cfg.control_loop_deadline_miss_ratio_limit) ||
      !std::isfinite(cfg.control_loop_p95_utilization_limit) ||
      !std::isfinite(cfg.nav_max_speed_mps) ||
      !std::isfinite(cfg.nav_max_accel_mps2) || !std::isfinite(cfg.waypoint_reached_m) ||
      !std::isfinite(cfg.goal_reached_m) || !std::isfinite(cfg.path_follower_goal_tolerance_m) ||
      !std::isfinite(cfg.path_follower_lookahead_m) ||
      !std::isfinite(cfg.path_follower_max_speed_mps) ||
      !std::isfinite(cfg.path_follower_min_speed_mps) ||
      !std::isfinite(cfg.path_follower_max_accel_mps2) ||
      !std::isfinite(cfg.path_follower_max_yaw_rate_rad_s) ||
      !std::isfinite(cfg.path_follower_max_yaw_accel_rad_s2) ||
      !std::isfinite(cfg.path_follower_heading_align_enter_rad) ||
      !std::isfinite(cfg.path_follower_heading_align_exit_rad) ||
      !std::isfinite(cfg.scan_follower.timeForward) ||
      !std::isfinite(cfg.scan_follower.headingErrorThreshold) ||
      !std::isfinite(cfg.scan_follower.positionGain) ||
      !std::isfinite(cfg.scan_follower.yawGain) ||
      !std::isfinite(cfg.scan_follower.maxVx) ||
      !std::isfinite(cfg.scan_follower.maxVy) ||
      !std::isfinite(cfg.scan_follower.maxYawRateRadS) ||
      !std::isfinite(cfg.scan_follower.finishDistance) ||
      !std::isfinite(cfg.recovery_blocked_interval_s) ||
      !std::isfinite(cfg.recovery_rotation_timeout_s) ||
      !std::isfinite(cfg.recovery_translation_timeout_s) ||
      !std::isfinite(cfg.recovery_translation_speed_mps) ||
      !std::isfinite(cfg.recovery_rotation_rate_rad_s) ||
      !std::isfinite(cfg.recovery_min_rotation_rad) ||
      !std::isfinite(cfg.recovery_max_rotation_rad) ||
      !std::isfinite(cfg.recovery_rotation_candidate_step_rad) ||
      !std::isfinite(cfg.recovery_rotation_sample_step_rad) ||
      !std::isfinite(cfg.corridor_lookahead_m) ||
      !std::isfinite(cfg.local_planner_obstacle_height_max_m) ||
      !std::isfinite(cfg.teleop_planner_horizon_m) ||
      !std::isfinite(cfg.driver_control_max_age_s) ||
      !std::isfinite(cfg.dynamic_min_speed_mps) ||
      !std::isfinite(cfg.teleop_planner_max_deviation_deg)) {
    throw std::runtime_error(
        "tick_hz, control-loop health limits, motion limits, corridor_lookahead_m, local planner "
        "obstacle height, driver control max age, and teleop planner limits must be finite");
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
  if (cfg.control_loop_deadline_miss_ratio_limit < 0.0 ||
      cfg.control_loop_deadline_miss_ratio_limit > 1.0) {
    throw std::runtime_error(
        "control loop deadline miss ratio limit must be within [0, 1]");
  }
  if (cfg.control_loop_p95_utilization_limit <= 0.0) {
    throw std::runtime_error(
        "control loop P95 utilization limit must be strictly positive");
  }
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
  cfg.path_follower_max_yaw_rate_rad_s =
      std::max(0.0, cfg.path_follower_max_yaw_rate_rad_s);
  cfg.path_follower_max_yaw_accel_rad_s2 =
      std::max(0.0, cfg.path_follower_max_yaw_accel_rad_s2);
  cfg.path_follower_heading_align_enter_rad =
      std::clamp(cfg.path_follower_heading_align_enter_rad, 0.0, M_PI);
  cfg.path_follower_heading_align_exit_rad =
      std::clamp(cfg.path_follower_heading_align_exit_rad, 0.0, M_PI);
  cfg.scan_follower.timeForward = std::max(0.0, cfg.scan_follower.timeForward);
  cfg.scan_follower.headingErrorThreshold =
      std::clamp(cfg.scan_follower.headingErrorThreshold, 0.0, M_PI);
  cfg.scan_follower.positionGain = std::max(0.0, cfg.scan_follower.positionGain);
  cfg.scan_follower.yawGain = std::max(0.0, cfg.scan_follower.yawGain);
  cfg.scan_follower.maxVx = std::max(0.0, cfg.scan_follower.maxVx);
  cfg.scan_follower.maxVy = std::max(0.0, cfg.scan_follower.maxVy);
  cfg.scan_follower.maxYawRateRadS =
      std::clamp(cfg.scan_follower.maxYawRateRadS, 0.0, 1.0);
  cfg.scan_follower.finishDistance =
      std::max(0.01, cfg.scan_follower.finishDistance);
  if (cfg.path_follower_heading_align_exit_rad >=
      cfg.path_follower_heading_align_enter_rad) {
    throw std::runtime_error(
        "path follower heading alignment exit angle must be below enter angle");
  }
  if (cfg.recovery_blocked_interval_s < 0.0 ||
      cfg.recovery_rotation_timeout_s <= 0.0 ||
      cfg.recovery_translation_timeout_s <= 0.0 ||
      cfg.recovery_max_attempts < 0 ||
      cfg.recovery_translation_speed_mps < 0.0 ||
      cfg.recovery_rotation_rate_rad_s <= 0.0 ||
      cfg.recovery_min_rotation_rad <= 0.0 ||
      cfg.recovery_max_rotation_rad < cfg.recovery_min_rotation_rad ||
      cfg.recovery_max_rotation_rad > M_PI ||
      cfg.recovery_rotation_candidate_step_rad <= 0.0 ||
      cfg.recovery_rotation_sample_step_rad <= 0.0 ||
      cfg.recovery_rotation_sample_step_rad > cfg.recovery_min_rotation_rad) {
    throw std::runtime_error("invalid recovery motion configuration");
  }
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
  cfg.live_obstacle_decay_s = std::max(0.0, cfg.live_obstacle_decay_s);
  cfg.live_obstacle_inflation_radius_m = std::max(0.0, cfg.live_obstacle_inflation_radius_m);
  cfg.live_obstacle_ray_clear_max_range_m = std::max(0.0, cfg.live_obstacle_ray_clear_max_range_m);
  cfg.live_obstacle_ray_clearing_interval_s =
      std::max(0.0, cfg.live_obstacle_ray_clearing_interval_s);
  cfg.live_obstacle_min_hits = std::max(1, cfg.live_obstacle_min_hits);
  cfg.dynamic_min_cells = std::max<std::size_t>(1U, cfg.dynamic_min_cells);
  cfg.dynamic_min_speed_mps = std::max(0.0, cfg.dynamic_min_speed_mps);
  cfg.dynamic_confirm_frames = std::max<std::size_t>(2U, cfg.dynamic_confirm_frames);
  cfg.teleop_cmd_max_age_s = std::max(0.0, cfg.teleop_cmd_max_age_s);
  cfg.teleop_max_speed_mps = std::max(0.0, cfg.teleop_max_speed_mps);
  cfg.teleop_max_yaw_rate = std::max(0.0, cfg.teleop_max_yaw_rate);
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
  if (cfg.teleop_traversability_soft_cost > cfg.teleop_traversability_hard_cost) {
    throw std::runtime_error("teleop traversability soft cost must not exceed hard cost");
  }
  if (cfg.traversability_soft_cost > cfg.traversability_hard_cost) {
    throw std::runtime_error("navigation traversability soft cost must not exceed hard cost");
  }
  cfg.vehicle_length_m = std::max(0.1, cfg.vehicle_length_m);
  cfg.vehicle_width_m = std::max(0.1, cfg.vehicle_width_m);
  cfg.collision_cylinder_radius_m = std::max(0.05, cfg.collision_cylinder_radius_m);
  cfg.collision_cylinder_offset_m = std::max(0.0, cfg.collision_cylinder_offset_m);
  cfg.collision_clearance_below_m = std::max(0.0, cfg.collision_clearance_below_m);
  cfg.collision_clearance_above_m = std::max(0.0, cfg.collision_clearance_above_m);
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
    const lingtu::nav::plan::far_planner::FarPlanner far_config_probe(cfg.far_options);
    (void)far_config_probe;
  } catch (const std::invalid_argument &exc) {
    throw std::runtime_error(std::string("invalid FAR configuration: ") + exc.what());
  }
  if (cfg.control_mode == ControlMode::Teleop) {
    cfg.check_obstacle = false;
    cfg.use_traversability_cost = false;
  } else if (cfg.control_mode == ControlMode::TeleopAvoid) {
    cfg.check_obstacle = true;
  }
  return cfg;
}

}  // namespace lingtu::nav::endpoint
