#include "input/nav_input_state_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "frame_transform.hpp"
#include "nav_endpoint_messages.hpp"
namespace {

std::string ownedString(const char *value) {
  return value == nullptr ? std::string{} : std::string{value};
}

lingtu::nav::endpoint::SensorOrigin
sensorOriginFromBody(const lingtu::nav::endpoint::RigidTransform &body,
                     const nav_kernel::Vec3 &sensor_offset) {
  const auto offset = lingtu::nav::endpoint::rotatePoint(body.rotation, sensor_offset);
  return {
      body.translation.x + offset.x,
      body.translation.y + offset.y,
      body.translation.z + offset.z,
      true,
  };
}

}  // namespace

namespace lingtu::nav::endpoint {

NavInputStateProjector::NavInputStateProjector(EndpointState &state, InputGate &input_gate,
                                               TransformBuffer &pose_buffer,
                                               TransformBuffer &map_odom_buffer,
                                               LiveObstacleLayer &live_obstacles,
                                               NavInputStateProjectorConfig config,
                                               NavInputStateProjectorActions actions)
    : state_(state),
      input_gate_(input_gate),
      pose_buffer_(pose_buffer),
      map_odom_buffer_(map_odom_buffer),
      live_obstacles_(live_obstacles),
      config_(std::move(config)),
      actions_(std::move(actions)) {}

void NavInputStateProjector::projectTf(const lingtu_dds_TFMessage &message,
                                       double receive_steady_s) {
  const auto transform = mapOdomTransformFromTf(message);
  if (!transform || !std::isfinite(transform->stamp_s) || transform->stamp_s <= 0.0) {
    state_.frames.last_error = "map_odom_tf_invalid";
    ++state_.tf_count;
    return;
  }

  const auto stamp_decision =
      classifySourceOrder(state_.last_tf_s, transform->stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    state_.frames.last_error = state_.map_odom_epoch_start_s > 0.0 &&
                                       transform->stamp_s + 1e-9 < state_.map_odom_epoch_start_s
                                   ? "map_odom_tf_before_current_epoch"
                                   : "map_odom_tf_out_of_order";
    ++state_.tf_count;
    return;
  }

  const bool map_frame_jump =
      state_.map_odom_tf && transformJumpExceeds(*state_.map_odom_tf, *transform,
                                                 kMapOdomJumpTranslationM, kMapOdomJumpYawRad);
  if (map_frame_jump) {
    resetInputEpoch(transform->stamp_s, "map_frame_jump", true);
  } else if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
    resetInputEpoch(transform->stamp_s, "source_clock_rebase", false);
  }

  state_.map_odom_tf = *transform;
  map_odom_buffer_.push(transform->stamp_s, *transform);
  state_.last_tf_s = transform->stamp_s;
  state_.last_tf_receive_s = receive_steady_s;
  ++state_.tf_generation;
  if (!map_frame_jump && state_.odom_body) {
    state_.map_body = transformPose(*state_.map_odom_tf, *state_.odom_body);
  }
  if (!map_frame_jump && state_.odom_body_transform) {
    state_.map_body_transform = composeTransforms(*state_.map_odom_tf, *state_.odom_body_transform);
  }
  ++state_.tf_count;
}

void NavInputStateProjector::projectOdometry(const lingtu_dds_Odometry &message,
                                             double receive_steady_s) {
  const double stamp_s = headerStampSeconds(message.header);
  const std::string frame_id = headerFrameId(message.header);
  if (frame_id != "map" && frame_id != "odom") {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = frame_id.empty() ? "odom_frame_empty" : "odom_frame_unsupported";
    return;
  }

  const auto stamp_decision =
      classifySourceOrder(state_.last_odom_s, stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = "odom_stamp_invalid";
    return;
  }
  if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
    resetInputEpoch(stamp_s, "source_clock_rebase", false);
  }

  if (frame_id == "odom" && state_.map_odom_epoch_start_s > 0.0 &&
      stamp_s + 1e-9 < state_.map_odom_epoch_start_s) {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = "odom_before_current_map_epoch";
    return;
  }

  const auto next_odom_body_transform = rigidTransformFromOdometry(message);
  if (!next_odom_body_transform.valid) {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = "odom_pose_nonfinite";
    return;
  }

  const bool map_frame = frame_id == "map";
  std::optional<RigidTransform> next_map_body_transform;
  if (map_frame) {
    next_map_body_transform = next_odom_body_transform;
  } else {
    const auto source_map_odom =
        map_odom_buffer_.sample(stamp_s, config_.source_transform_max_gap_s);
    if (!source_map_odom) {
      ++state_.frames.odom_rejected;
      state_.frames.last_error = "odom_tf_gap_exceeded";
      return;
    }
    next_map_body_transform = composeTransforms(*source_map_odom, next_odom_body_transform);
  }

  const nav_kernel::Vec3 current_position{
      next_odom_body_transform.translation.x,
      next_odom_body_transform.translation.y,
      next_odom_body_transform.translation.z,
  };
  const auto &linear = message.twist.twist.linear;
  state_.last_odom_linear_speed_mps =
      state_.odom_speed_monitor.observe(stamp_s, frame_id, current_position.x, current_position.y,
                                        current_position.z, linear.x, linear.y, linear.z);
  const auto &angular = message.twist.twist.angular;
  state_.last_odom_angular_speed_radps =
      std::sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z);

  state_.odom_body = toPose(message);
  state_.odom_body_transform = next_odom_body_transform;
  state_.odom_requires_tf = !map_frame;
  state_.last_odom_s = stamp_s;
  state_.last_odom_receive_s = receive_steady_s;
  if (map_frame) {
    state_.map_body = *state_.odom_body;
    state_.map_body_transform = next_map_body_transform;
  } else {
    state_.map_body_transform = next_map_body_transform;
    state_.map_body = transformPose(*state_.map_body_transform, nav_kernel::Pose{});
  }
  pose_buffer_.push(stamp_s, *state_.map_body_transform);
  ++state_.odom_generation;
  ++state_.odom_count;
}

void NavInputStateProjector::projectDriverControl(const lingtu_dds_DriverControlState &message,
                                                  SteadyClock::time_point receive_time) {
  const double stamp_s = headerStampSeconds(message.header);
  const auto stamp_decision =
      classifySourceOrder(state_.driver_control_stamp_s, stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    state_.frames.last_error = "driver_control_stamp_invalid";
    return;
  }

  const std::string fsm = ownedString(message.fsm);
  const std::string owner = ownedString(message.owner);
  const std::string owner_id = ownedString(message.owner_id);
  state_.driver_control_stamp_s = stamp_s;
  state_.driver_control_receive_time = receive_time;
  state_.driver_control_received = true;
  state_.driver_control_reason = ownedString(message.reason);
  state_.driver_accepted_producer_boot_id = ownedString(message.accepted_producer_boot_id);
  state_.driver_accepted_output_sequence = message.accepted_output_sequence;
  state_.driver_last_command_accepted = message.last_command_accepted;
  state_.driver_control_ready = message.connected && message.ready && message.motors_enabled &&
                                !message.critical_fault && message.lease_valid && owner == "grpc" &&
                                owner_id == "lingtu-driver" &&
                                (fsm == "standing" || fsm == "walking");

  if (!state_.driver_control_ready && state_.driver_control_reason.empty()) {
    if (!message.connected) {
      state_.driver_control_reason = "disconnected";
    } else if (!message.motors_enabled) {
      state_.driver_control_reason = "motors_disabled";
    } else if (message.critical_fault) {
      state_.driver_control_reason = "motor_fault";
    } else if (!message.lease_valid || owner != "grpc" || owner_id != "lingtu-driver") {
      state_.driver_control_reason = "lease_not_owned";
    } else if (fsm != "standing" && fsm != "walking") {
      state_.driver_control_reason = "fsm_not_ready";
    } else {
      state_.driver_control_reason = "not_ready";
    }
  }
  ++state_.driver_control_generation;
}

double NavInputStateProjector::driverControlReceiveAge(SteadyClock::time_point now_time) const {
  if (!state_.driver_control_received) {
    return std::numeric_limits<double>::infinity();
  }
  return std::chrono::duration<double>(now_time - state_.driver_control_receive_time).count();
}

std::string NavInputStateProjector::driverControlBlocker(SteadyClock::time_point now_time) const {
  if (state_.driver_control_stamp_s <= 0.0 || !state_.driver_control_received) {
    return "driver_control_missing";
  }
  const double age_s = driverControlReceiveAge(now_time);
  if (config_.driver_control_max_age_s > 0.0 && age_s > config_.driver_control_max_age_s) {
    return "driver_control_stale";
  }
  if (!state_.driver_control_ready) {
    return state_.driver_control_reason.empty()
               ? "driver_control_not_ready"
               : std::string{"driver_control_"} + state_.driver_control_reason;
  }
  return {};
}

InputGateState NavInputStateProjector::evaluateInputGate(double now_steady_s,
                                                         SteadyClock::time_point now_time) {
  InputSnapshot snapshot;
  snapshot.now_s = now_steady_s;
  snapshot.odom_stamp_s = state_.last_odom_s;
  snapshot.odom_receive_s = state_.last_odom_receive_s;
  snapshot.odom_generation = state_.odom_generation;
  snapshot.odom_linear_speed_mps = state_.last_odom_linear_speed_mps;
  snapshot.tf_stamp_s = state_.last_tf_s;
  snapshot.tf_receive_s = state_.last_tf_receive_s;
  snapshot.tf_generation = state_.tf_generation;
  snapshot.cloud_stamp_s = state_.last_cloud_s;
  snapshot.cloud_receive_s = state_.last_cloud_receive_s;
  snapshot.cloud_generation = state_.cloud_generation;
  snapshot.traversability_stamp_s = state_.last_traversability_s;
  snapshot.traversability_receive_s = state_.last_traversability_receive_s;
  snapshot.traversability_generation = state_.traversability_generation;
  snapshot.localization_health_stamp_s = state_.localization_health.stamp_s;
  snapshot.localization_health_receive_s = state_.localization_health_receive_s;
  snapshot.localization_health_generation = state_.localization_health_generation;

  const double driver_age_s = driverControlReceiveAge(now_time);
  snapshot.driver_control_stamp_s = state_.driver_control_stamp_s;
  snapshot.driver_control_receive_s = state_.driver_control_received && std::isfinite(driver_age_s)
                                          ? now_steady_s - std::max(0.0, driver_age_s)
                                          : 0.0;
  snapshot.driver_control_generation = state_.driver_control_generation;
  snapshot.odom_requires_tf = state_.odom_requires_tf;
  snapshot.localization_healthy = state_.localization_health.healthy;
  snapshot.localization_state = state_.localization_health.state;
  snapshot.localization_reason = state_.localization_health.reason;
  snapshot.driver_control_ready = state_.driver_control_ready;
  snapshot.driver_control_reason = state_.driver_control_reason;

  state_.input_gate_state = input_gate_.evaluate(snapshot);
  return state_.input_gate_state;
}

void NavInputStateProjector::projectCloud(const lingtu_dds_PointCloud2 &message,
                                          double receive_steady_s, double receive_wall_s,
                                          TimingDiagnostics &timing) {
  ++state_.cloud_count;
  const auto convert_start = SteadyClock::now();
  const double stamp_s = headerStampSeconds(message.header);
  state_.cloud_sync.last_stamp_age_s = receive_wall_s - stamp_s;
  const auto stamp_decision =
      classifySourceOrder(state_.last_cloud_s, stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    ++state_.cloud_sync.stamp_rejected;
    state_.frames.last_error = "cloud_stamp_invalid";
    return;
  }
  if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }

  const std::string frame_id = headerFrameId(message.header);
  if (frame_id == "odom" && state_.map_odom_epoch_start_s > 0.0 &&
      stamp_s + 1e-9 < state_.map_odom_epoch_start_s) {
    ++state_.cloud_sync.pose_rejected;
    state_.frames.last_error = "cloud_before_current_map_epoch";
    return;
  }

  state_.cloud_sync.last_pose_gap_s = pose_buffer_.nearestGap(stamp_s);
  const auto cloud_pose = pose_buffer_.sample(stamp_s, config_.cloud_pose_max_gap_s);
  if (!cloud_pose) {
    ++state_.cloud_sync.pose_rejected;
    return;
  }

  std::optional<RigidTransform> cloud_map_odom;
  if (frame_id == "odom") {
    cloud_map_odom = map_odom_buffer_.sample(stamp_s, config_.source_transform_max_gap_s);
    if (!cloud_map_odom) {
      ++state_.cloud_sync.pose_rejected;
      state_.frames.last_error = "cloud_tf_gap_exceeded";
      return;
    }
  }

  auto xyzh = cloudToXyzh(message, 0, cloud_pose, cloud_map_odom);
  timing.cloud_convert_ms += elapsedMs(convert_start);
  if (xyzh.empty()) {
    return;
  }

  const auto motion_start = SteadyClock::now();
  state_.last_sensor_origin = sensorOriginFromBody(*cloud_pose, config_.sensor_offset);
  live_obstacles_.updateFromScan(state_.last_sensor_origin, xyzh, stamp_s);
  state_.latest_dynamic_clusters = live_obstacles_.dynamicClusters(32, stamp_s);
  timing.motion_update_last_ms = elapsedMs(motion_start);
  state_.obstacle_snapshot_dirty = true;
  state_.last_cloud_s = stamp_s;
  state_.last_cloud_receive_s = receive_steady_s;
  ++state_.cloud_generation;
}

void NavInputStateProjector::projectTerrainMap(const lingtu_dds_PointCloud2 &message,
                                               double receive_steady_s, TimingDiagnostics &timing) {
  const double stamp_s = headerStampSeconds(message.header);
  const auto stamp_decision =
      classifySourceOrder(state_.last_terrain_map_s, stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    state_.frames.last_error = "terrain_map_stamp_invalid";
    return;
  }
  if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }

  const auto convert_start = SteadyClock::now();
  auto xyzh = cloudToXyzh(message, 0, state_.map_body_transform, state_.map_odom_tf);
  timing.cloud_convert_ms += elapsedMs(convert_start);
  if (xyzh.empty()) {
    return;
  }

  state_.terrain_xyzh = std::move(xyzh);
  state_.last_terrain_map_s = stamp_s;
  state_.last_terrain_map_receive_s = receive_steady_s;
  ++state_.terrain_map_count;
}

void NavInputStateProjector::projectTerrainMapExt(const lingtu_dds_PointCloud2 &message,
                                                  double receive_steady_s,
                                                  TimingDiagnostics &timing) {
  const double stamp_s = headerStampSeconds(message.header);
  const auto stamp_decision =
      classifySourceOrder(state_.last_terrain_ext_s, stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    state_.frames.last_error = "terrain_map_ext_stamp_invalid";
    return;
  }
  if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }

  const auto convert_start = SteadyClock::now();
  auto xyzh = cloudToXyzh(message, 0, state_.map_body_transform, state_.map_odom_tf);
  timing.cloud_convert_ms += elapsedMs(convert_start);
  if (xyzh.empty()) {
    return;
  }

  state_.terrain_ext_xyzh = std::move(xyzh);
  state_.last_terrain_ext_s = stamp_s;
  state_.last_terrain_ext_receive_s = receive_steady_s;
  ++state_.terrain_map_ext_count;
}

void NavInputStateProjector::clearPlannerInputs(const lingtu_dds_Bool &message,
                                                PlannerInputClearSource source) {
  if (!message.data) {
    return;
  }

  clearPlannerInputState();
  live_obstacles_.clear();
  state_.latest_dynamic_clusters.clear();
  state_.obstacle_snapshot_dirty = false;
  state_.obstacle_xyzh.clear();

  const std::string reason = source == PlannerInputClearSource::kMap
                                 ? "execution_grid_map_cleared"
                                 : "execution_grid_cloud_cleared";
  state_.frames.last_error = reason;
  if (actions_.on_rolling_snapshot_invalidated) {
    actions_.on_rolling_snapshot_invalidated(reason);
  }

  if (source == PlannerInputClearSource::kMap) {
    ++state_.map_clearing_count;
  } else {
    ++state_.cloud_clearing_count;
  }
}

bool NavInputStateProjector::materializeLiveObstacleSnapshot(TimingDiagnostics &timing) {
  if (!config_.check_obstacle || !state_.obstacle_snapshot_dirty) {
    return false;
  }

  const auto snapshot_start = SteadyClock::now();
  state_.obstacle_xyzh = live_obstacles_.snapshot(config_.max_obstacle_points, state_.last_cloud_s);
  timing.obstacle_snapshot_last_ms = elapsedMs(snapshot_start);
  state_.obstacle_snapshot_dirty = false;
  return true;
}

void NavInputStateProjector::projectTraversability(const lingtu_dds_OccupancyGrid &message,
                                                   double receive_steady_s) {
  const double stamp_s = headerStampSeconds(message.header);
  const auto stamp_decision =
      classifySourceOrder(state_.last_traversability_s, stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    ++state_.frames.grid_rejected;
    state_.frames.last_error = "traversability_stamp_invalid";
    return;
  }
  if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }

  auto decoded = decodeGrid(message);
  if (!decoded.ok()) {
    ++state_.frames.grid_rejected;
    state_.frames.last_error = decoded.error;
    return;
  }

  decoded.value.generation = ++state_.traversability_generation;
  state_.traversability_grid = std::move(decoded.value);
  state_.last_traversability_s = stamp_s;
  state_.last_traversability_receive_s = receive_steady_s;
  ++state_.traversability_count;
}

void NavInputStateProjector::projectLocalizationHealth(const lingtu_dds_Text &message,
                                                       double receive_steady_s) {
  auto next_health = decodeLocalizationHealth(textData(message));
  if (!next_health.valid) {
    state_.frames.last_error = next_health.error;
    return;
  }

  const auto stamp_decision = classifySourceOrder(
      state_.localization_health.stamp_s, next_health.stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    state_.frames.last_error = "localization_health_stamp_invalid";
    return;
  }
  if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }

  state_.localization_health = std::move(next_health);
  state_.localization_health_receive_s = receive_steady_s;
  ++state_.localization_health_generation;
}

void NavInputStateProjector::resetInputEpoch(double epoch_start_s, const std::string &reason,
                                             bool clear_motion) {
  ++state_.frame_epoch;

  InputSnapshot epoch_boundary;
  epoch_boundary.odom_generation = state_.odom_generation;
  epoch_boundary.tf_generation = state_.tf_generation;
  epoch_boundary.cloud_generation = state_.cloud_generation;
  epoch_boundary.traversability_generation = state_.traversability_generation;
  epoch_boundary.localization_health_generation = state_.localization_health_generation;
  epoch_boundary.driver_control_generation = state_.driver_control_generation;
  input_gate_.beginRecoveryFrom(epoch_boundary);

  pose_buffer_.clear();
  map_odom_buffer_.clear();
  state_.map_odom_tf.reset();
  state_.odom_body.reset();
  state_.odom_body_transform.reset();
  state_.map_body.reset();
  state_.map_body_transform.reset();
  clearPlannerInputState();
  live_obstacles_.clear();
  state_.latest_dynamic_clusters.clear();
  state_.obstacle_xyzh.clear();
  state_.obstacle_snapshot_dirty = false;
  state_.last_sensor_origin = SensorOrigin{};
  state_.last_tf_s = 0.0;
  state_.last_tf_receive_s = 0.0;
  state_.last_odom_s = 0.0;
  state_.last_odom_receive_s = 0.0;
  state_.last_odom_linear_speed_mps = 0.0;
  state_.last_odom_angular_speed_radps = 0.0;
  state_.odom_speed_monitor.reset();
  state_.last_cloud_s = 0.0;
  state_.last_cloud_receive_s = 0.0;
  state_.cloud_sync = CloudSyncDiagnostics{};
  state_.localization_health = LocalizationHealthSample{};
  state_.localization_health_receive_s = 0.0;
  state_.map_odom_epoch_start_s = epoch_start_s;
  state_.frames.last_error = reason;

  if (actions_.on_epoch_reset) {
    actions_.on_epoch_reset(epoch_start_s, reason, clear_motion);
  }
}

void NavInputStateProjector::clearPlannerInputState() {
  state_.terrain_xyzh.clear();
  state_.terrain_ext_xyzh.clear();
  state_.planner_terrain_xyzh.clear();
  state_.traversability_grid = TraversabilityGrid{};
  state_.last_terrain_map_s = 0.0;
  state_.last_terrain_map_receive_s = 0.0;
  state_.last_terrain_ext_s = 0.0;
  state_.last_terrain_ext_receive_s = 0.0;
  state_.last_traversability_s = 0.0;
  state_.last_traversability_receive_s = 0.0;
}

}  // namespace lingtu::nav::endpoint
