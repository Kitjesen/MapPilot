#include "input/projector.hpp"

#include <cmath>
#include <utility>

#include "input/frame.hpp"

namespace lingtu::nav::endpoint {

InputProjector::InputProjector(EndpointState &state, InputGate &gate,
                               TransformBuffer &pose_buffer, TransformBuffer &map_odom_buffer,
                               MotionLayer &obstacles, InputConfig config, InputActions actions)
    : state_(state),
      gate_(gate),
      pose_buffer_(pose_buffer),
      map_odom_buffer_(map_odom_buffer),
      obstacles_(obstacles),
      config_(std::move(config)),
      actions_(std::move(actions)) {}

void InputProjector::apply(SensorBatch batch, TimingDiagnostics &timing) {
  for (const auto &sample : batch.transforms) {
    projectTf(sample, batch.receive_steady_s);
  }
  for (const auto &sample : batch.odometry) {
    projectOdometry(sample, batch.receive_steady_s);
  }
  if (batch.driver_control) {
    projectDriverControl(*batch.driver_control, batch.receive_steady_s);
  }
  if (batch.exploration_grid && actions_.on_execution_grid) {
    actions_.on_execution_grid(std::move(*batch.exploration_grid));
  }
  if (batch.obstacles) {
    projectCloud(std::move(*batch.obstacles), batch.receive_steady_s, batch.receive_wall_s, timing);
  }
  if (batch.terrain) {
    projectTerrainMap(std::move(*batch.terrain), batch.receive_steady_s, timing);
  }
  if (batch.terrain_extended) {
    projectTerrainMapExt(std::move(*batch.terrain_extended), batch.receive_steady_s, timing);
  }
  for (const auto &clear : batch.clears) {
    clearPlannerInputs(clear);
  }
  if (batch.traversability) {
    projectTraversability(std::move(*batch.traversability), batch.receive_steady_s);
  }
  if (batch.local_traversability) {
    projectLocalTraversability(std::move(*batch.local_traversability), batch.receive_steady_s);
  }
  if (batch.local_collision) {
    projectLocalCollision(std::move(*batch.local_collision), batch.receive_steady_s);
  }
  if (batch.localization_health) {
    projectLocalizationHealth(std::move(*batch.localization_health), batch.receive_steady_s);
  }
}

void InputProjector::projectTf(const InputSample<TransformSample> &sample,
                               double receive_steady_s) {
  if (!sample.ok()) {
    state_.frames.last_error = sample.error;
    ++state_.tf_count;
    return;
  }
  const auto &transform = sample.value.map_odom;
  if (!transform.valid || !std::isfinite(transform.stamp_s) || transform.stamp_s <= 0.0) {
    state_.frames.last_error = "map_odom_tf_invalid";
    ++state_.tf_count;
    return;
  }

  const auto stamp_decision =
      classifySourceOrder(state_.last_tf_s, transform.stamp_s, kSourceClockRebaseThresholdS);
  if (stamp_decision == SourceStampDecision::kReject) {
    state_.frames.last_error = state_.map_odom_epoch_start_s > 0.0 &&
                                       transform.stamp_s + 1e-9 < state_.map_odom_epoch_start_s
                                   ? "map_odom_tf_before_current_epoch"
                                   : "map_odom_tf_out_of_order";
    ++state_.tf_count;
    return;
  }

  const bool map_frame_jump =
      state_.map_odom_tf && transformJumpExceeds(*state_.map_odom_tf, transform,
                                                 kMapOdomJumpTranslationM, kMapOdomJumpYawRad);
  if (map_frame_jump) {
    resetEpoch(transform.stamp_s, "map_frame_jump", true);
  } else if (stamp_decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
    resetEpoch(transform.stamp_s, "source_clock_rebase", false);
  }

  state_.map_odom_tf = transform;
  map_odom_buffer_.push(transform.stamp_s, transform);
  state_.last_tf_s = transform.stamp_s;
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

void InputProjector::projectOdometry(const InputSample<OdometrySample> &sample,
                                     double receive_steady_s) {
  if (!sample.ok()) {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = sample.error;
    return;
  }
  const auto &message = sample.value;
  const double stamp_s = message.header.stamp_s;
  const std::string &frame_id = message.header.frame_id;

  if (state_.last_odom_s > 0.0 &&
      std::abs(stamp_s - state_.last_odom_s) <= kOdometryMinimumStampAdvanceS) {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = "odom_stamp_not_advanced";
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
    resetEpoch(stamp_s, "source_clock_rebase", false);
  }

  if (frame_id == "odom" && state_.map_odom_epoch_start_s > 0.0 &&
      stamp_s + 1e-9 < state_.map_odom_epoch_start_s) {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = "odom_before_current_map_epoch";
    return;
  }

  const auto &odom_body = message.body;
  if (!odom_body.valid) {
    ++state_.frames.odom_rejected;
    state_.frames.last_error = "odom_pose_nonfinite";
    return;
  }

  const bool map_frame = frame_id == "map";
  std::optional<RigidTransform> map_body;
  if (map_frame) {
    map_body = odom_body;
  } else {
    const auto map_odom = map_odom_buffer_.sample(stamp_s, config_.source_transform_max_gap_s);
    if (!map_odom) {
      ++state_.frames.odom_rejected;
      state_.frames.last_error = "odom_tf_gap_exceeded";
      return;
    }
    map_body = composeTransforms(*map_odom, odom_body);
  }

  const auto &position = odom_body.translation;
  const auto &linear = message.linear_velocity;
  state_.last_odom_linear_speed_mps = state_.odom_speed_monitor.observe(
      stamp_s, frame_id, position.x, position.y, position.z, linear.x, linear.y, linear.z);
  const auto &angular = message.angular_velocity;
  state_.last_odom_angular_speed_radps =
      std::sqrt(angular.x * angular.x + angular.y * angular.y + angular.z * angular.z);
  state_.odom_linear_velocity_body = {linear.x, linear.y, linear.z};
  state_.odom_yaw_rate = angular.z;
  state_.odom_velocity_valid = std::isfinite(linear.x) && std::isfinite(linear.y) &&
                               std::isfinite(linear.z) && std::isfinite(angular.z);

  state_.odom_body = message.pose;
  state_.odom_body_transform = odom_body;
  state_.odom_requires_tf = !map_frame;
  state_.last_odom_s = stamp_s;
  state_.last_odom_receive_s = receive_steady_s;
  state_.map_body_transform = map_body;
  state_.map_body = map_frame ? *state_.odom_body : transformPose(*map_body, nav_kernel::Pose{});
  pose_buffer_.push(stamp_s, *map_body);
  ++state_.odom_generation;
  ++state_.odom_count;
}

void InputProjector::resetEpoch(double epoch_start_s, const std::string &reason,
                                bool clear_motion) {
  ++state_.frame_epoch;

  InputSnapshot boundary;
  boundary.odom_generation = state_.odom_generation;
  boundary.tf_generation = state_.tf_generation;
  boundary.cloud_generation = state_.cloud_generation;
  boundary.traversability_generation = state_.traversability_generation;
  boundary.localization_health_generation = state_.localization_health_generation;
  boundary.driver_control_generation = state_.driver_control_generation;
  gate_.beginRecoveryFrom(boundary);

  pose_buffer_.clear();
  map_odom_buffer_.clear();
  state_.map_odom_tf.reset();
  state_.odom_body.reset();
  state_.odom_body_transform.reset();
  state_.map_body.reset();
  state_.map_body_transform.reset();
  clearPlanState();
  obstacles_.clear();
  state_.latest_dynamic_clusters.clear();
  state_.obstacle_xyzh.clear();
  state_.predicted_obstacle_xyzh.clear();
  state_.obstacle_snapshot_dirty = false;
  state_.last_sensor_origin = SensorOrigin{};
  state_.last_tf_s = 0.0;
  state_.last_tf_receive_s = 0.0;
  state_.last_odom_s = 0.0;
  state_.last_odom_receive_s = 0.0;
  state_.last_odom_linear_speed_mps = 0.0;
  state_.last_odom_angular_speed_radps = 0.0;
  state_.odom_linear_velocity_body = {};
  state_.odom_yaw_rate = 0.0;
  state_.odom_velocity_valid = false;
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

}  // namespace lingtu::nav::endpoint
