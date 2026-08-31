#include "input/projector.hpp"

#include <algorithm>
#include <cmath>

#include "input/frame.hpp"

namespace {

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

bool transformCloudToMap(
    lingtu::nav::endpoint::PointCloudSample &cloud,
    const std::optional<lingtu::nav::endpoint::RigidTransform> &map_body,
    const std::optional<lingtu::nav::endpoint::RigidTransform> &map_odom) {
  if (cloud.xyzh.empty() || cloud.xyzh.size() % 4U != 0U) {
    return false;
  }
  const bool map_frame = cloud.header.frame_id == "map";
  const bool odom_frame = cloud.header.frame_id == "odom";
  const bool body_frame = cloud.header.frame_id == "body" || cloud.header.frame_id == "base" ||
                          cloud.header.frame_id == "base_link";
  if ((!map_frame && !odom_frame && !body_frame) || (odom_frame && !map_odom) ||
      (body_frame && !map_body)) {
    return false;
  }

  for (std::size_t index = 0U; index < cloud.xyzh.size(); index += 4U) {
    const nav_kernel::Vec3 source{cloud.xyzh[index], cloud.xyzh[index + 1U],
                                  cloud.xyzh[index + 2U]};
    nav_kernel::Vec3 world = source;
    if (odom_frame) {
      world = lingtu::nav::endpoint::transformPoint(*map_odom, source);
    } else if (body_frame) {
      world = lingtu::nav::endpoint::transformPoint(*map_body, source);
    }
    cloud.xyzh[index] = static_cast<float>(world.x);
    cloud.xyzh[index + 1U] = static_cast<float>(world.y);
    cloud.xyzh[index + 2U] = static_cast<float>(world.z);
    if (!std::isfinite(cloud.xyzh[index + 3U])) {
      cloud.xyzh[index + 3U] = map_body
                                    ? static_cast<float>(world.z - map_body->translation.z)
                                    : static_cast<float>(world.z);
    }
  }
  return true;
}

}  // namespace

namespace lingtu::nav::endpoint {

void InputProjector::projectCloud(InputSample<PointCloudSample> sample,
                                  double receive_steady_s, double receive_wall_s,
  TimingDiagnostics &timing) {
  ++state_.cloud_count;
  auto message = std::move(sample.value);
  const auto convert_start = SteadyClock::now();
  const double stamp_s = message.header.stamp_s;
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
    obstacles_.clear();
    state_.latest_dynamic_clusters.clear();
    state_.obstacle_snapshot_dirty = false;
    state_.obstacle_xyzh.clear();
    state_.predicted_obstacle_xyzh.clear();
  }
  if (!sample.ok()) {
    return;
  }

  const std::string &frame_id = message.header.frame_id;
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

  std::optional<RigidTransform> map_odom;
  if (frame_id == "odom") {
    map_odom = map_odom_buffer_.sample(stamp_s, config_.source_transform_max_gap_s);
    if (!map_odom) {
      ++state_.cloud_sync.pose_rejected;
      state_.frames.last_error = "cloud_tf_gap_exceeded";
      return;
    }
  }

  if (!transformCloudToMap(message, cloud_pose, map_odom)) {
    timing.cloud_convert_ms += elapsedMs(convert_start);
    return;
  }
  auto xyzh = std::move(message.xyzh);
  timing.cloud_convert_ms += elapsedMs(convert_start);
  if (xyzh.empty()) {
    return;
  }

  const auto motion_start = SteadyClock::now();
  state_.last_sensor_origin = sensorOriginFromBody(*cloud_pose, config_.sensor_offset);
  obstacles_.updateFromScan(state_.last_sensor_origin, xyzh, stamp_s);
  state_.latest_dynamic_clusters = obstacles_.dynamicClusters(32, stamp_s);
  timing.motion_update_last_ms = elapsedMs(motion_start);
  state_.obstacle_snapshot_dirty = true;
  state_.last_cloud_s = stamp_s;
  state_.last_cloud_receive_s = receive_steady_s;
  ++state_.cloud_generation;
}

void InputProjector::projectTerrainMap(InputSample<PointCloudSample> sample,
                                       double receive_steady_s, TimingDiagnostics &timing) {
  projectTerrain(std::move(sample), receive_steady_s, timing, false);
}

void InputProjector::projectTerrainMapExt(InputSample<PointCloudSample> sample,
                                          double receive_steady_s, TimingDiagnostics &timing) {
  projectTerrain(std::move(sample), receive_steady_s, timing, true);
}

void InputProjector::projectTerrain(InputSample<PointCloudSample> sample,
                                    double receive_steady_s, TimingDiagnostics &timing,
                                    bool extended) {
  auto message = std::move(sample.value);
  double &last_stamp = extended ? state_.last_terrain_ext_s : state_.last_terrain_map_s;
  const double stamp_s = message.header.stamp_s;
  const auto decision = classifySourceOrder(last_stamp, stamp_s, kSourceClockRebaseThresholdS);
  if (decision == SourceStampDecision::kReject) {
    state_.frames.last_error =
        extended ? "terrain_map_ext_stamp_invalid" : "terrain_map_stamp_invalid";
    return;
  }
  if (decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }
  if (!sample.ok()) {
    return;
  }

  const auto convert_start = SteadyClock::now();
  if (!transformCloudToMap(message, state_.map_body_transform, state_.map_odom_tf)) {
    timing.cloud_convert_ms += elapsedMs(convert_start);
    return;
  }
  auto xyzh = std::move(message.xyzh);
  timing.cloud_convert_ms += elapsedMs(convert_start);
  if (xyzh.empty()) {
    return;
  }

  auto &points = extended ? state_.terrain_ext_xyzh : state_.terrain_xyzh;
  double &last_receive =
      extended ? state_.last_terrain_ext_receive_s : state_.last_terrain_map_receive_s;
  auto &count = extended ? state_.terrain_map_ext_count : state_.terrain_map_count;
  points = std::move(xyzh);
  last_stamp = stamp_s;
  last_receive = receive_steady_s;
  ++count;
}

void InputProjector::projectLocalCollision(InputSample<LocalCollisionMap> sample,
                                           double receive_steady_s) {
  if (!sample.ok()) {
    ++state_.local_collision_rejected;
    state_.frames.last_error = sample.error;
    return;
  }
  auto decoded = std::move(sample.value);

  const auto decision = classifySourceOrder(state_.last_local_collision_s, decoded.stamp_s,
                                            kSourceClockRebaseThresholdS);
  if (decision == SourceStampDecision::kReject) {
    ++state_.local_collision_rejected;
    state_.frames.last_error = "local_collision_stamp_invalid";
    return;
  }
  if (decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
    state_.local_collision_not_before_s = 0.0;
  }
  if (state_.local_collision_not_before_s > 0.0 &&
      decoded.stamp_s <= state_.local_collision_not_before_s + 1e-9) {
    ++state_.local_collision_rejected;
    state_.frames.last_error = "local_collision_before_clear";
    return;
  }

  auto &current = state_.local_collision_map;
  if (current.reset_epoch > decoded.reset_epoch ||
      (current.reset_epoch == decoded.reset_epoch &&
       (current.observation_sequence > decoded.observation_sequence ||
        current.generation > decoded.generation))) {
    ++state_.local_collision_rejected;
    state_.frames.last_error = "local_collision_identity_stale";
    return;
  }
  if (current.reset_epoch == decoded.reset_epoch &&
      current.observation_sequence == decoded.observation_sequence &&
      current.generation == decoded.generation) {
    current.live = current.live && decoded.live;
    current.complete = current.complete && decoded.complete;
    return;
  }

  decoded.receive_stamp_s = receive_steady_s;
  state_.local_collision_map = std::move(decoded);
  state_.last_local_collision_s = state_.local_collision_map.stamp_s;
  state_.last_local_collision_receive_s = receive_steady_s;
  ++state_.local_collision_count;
}

void InputProjector::clearPlannerInputs(const PlannerClearSample &sample) {
  if (!sample.requested) {
    return;
  }

  clearPlanState();
  obstacles_.clear();
  state_.latest_dynamic_clusters.clear();
  state_.obstacle_snapshot_dirty = false;
  state_.obstacle_xyzh.clear();
  state_.predicted_obstacle_xyzh.clear();

  const std::string reason = sample.source == ClearSource::Map ? "execution_grid_map_cleared"
                                                               : "execution_grid_cloud_cleared";
  state_.frames.last_error = reason;
  if (actions_.on_rolling_snapshot_invalidated) {
    actions_.on_rolling_snapshot_invalidated(reason);
  }

  if (sample.source == ClearSource::Map) {
    ++state_.map_clearing_count;
  } else {
    ++state_.cloud_clearing_count;
  }
}

bool InputProjector::materializeObstacles(TimingDiagnostics &timing) {
  if (!config_.check_obstacle || !state_.obstacle_snapshot_dirty) {
    return false;
  }

  const auto snapshot_start = SteadyClock::now();
  obstacles_.snapshot(state_.obstacle_xyzh, config_.max_obstacle_points, state_.last_cloud_s);
  obstacles_.snapshotPredictedDynamic(state_.predicted_obstacle_xyzh,
                                      kMaxDynamicPredictionPoints, state_.last_cloud_s);
  timing.obstacle_snapshot_last_ms = elapsedMs(snapshot_start);
  state_.obstacle_snapshot_dirty = false;
  return true;
}

void InputProjector::projectTraversability(InputSample<GridSample> sample,
                                           double receive_steady_s) {
  if (!sample.ok()) {
    ++state_.frames.grid_rejected;
    state_.frames.last_error = sample.error;
    return;
  }
  const double stamp_s = sample.value.header.stamp_s;
  const auto decision =
      classifySourceOrder(state_.last_traversability_s, stamp_s, kSourceClockRebaseThresholdS);
  if (decision == SourceStampDecision::kReject) {
    ++state_.frames.grid_rejected;
    state_.frames.last_error = "traversability_stamp_invalid";
    return;
  }
  if (decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
  }

  sample.value.grid.generation = ++state_.traversability_generation;
  state_.traversability_grid = std::move(sample.value.grid);
  state_.last_traversability_s = stamp_s;
  state_.last_traversability_receive_s = receive_steady_s;
  ++state_.traversability_count;
}

void InputProjector::projectLocalTraversability(InputSample<GridSample> sample,
                                                double receive_steady_s) {
  if (!sample.ok()) {
    ++state_.local_traversability_rejected;
    state_.frames.last_error = sample.error;
    return;
  }

  const double stamp_s = sample.value.header.stamp_s;
  const auto decision = classifySourceOrder(state_.last_local_traversability_s, stamp_s,
                                            kSourceClockRebaseThresholdS);
  if (decision == SourceStampDecision::kReject) {
    ++state_.local_traversability_rejected;
    state_.frames.last_error = "local_traversability_stamp_invalid";
    return;
  }
  if (decision == SourceStampDecision::kClockRebase) {
    ++state_.frames.clock_rebases;
    state_.local_traversability_not_before_s = 0.0;
  }
  if (state_.local_traversability_not_before_s > 0.0 &&
      stamp_s <= state_.local_traversability_not_before_s + 1e-9) {
    ++state_.local_traversability_rejected;
    state_.frames.last_error = "local_traversability_before_clear";
    return;
  }

  sample.value.grid.generation = ++state_.local_traversability_generation;
  state_.local_traversability_grid = std::move(sample.value.grid);
  state_.last_local_traversability_s = stamp_s;
  state_.last_local_traversability_receive_s = receive_steady_s;
  ++state_.local_traversability_count;
}

void InputProjector::clearPlanState() {
  state_.terrain_xyzh.clear();
  state_.terrain_ext_xyzh.clear();
  state_.traversability_grid = TraversabilityGrid{};
  state_.local_traversability_grid = TraversabilityGrid{};
  state_.local_collision_map = LocalCollisionMap{};
  state_.last_terrain_map_s = 0.0;
  state_.last_terrain_map_receive_s = 0.0;
  state_.last_terrain_ext_s = 0.0;
  state_.last_terrain_ext_receive_s = 0.0;
  state_.last_traversability_s = 0.0;
  state_.last_traversability_receive_s = 0.0;
  state_.local_traversability_not_before_s =
      std::max(state_.local_traversability_not_before_s, state_.last_local_traversability_s);
  state_.last_local_traversability_receive_s = 0.0;
  state_.local_collision_not_before_s =
      std::max(state_.local_collision_not_before_s, state_.last_local_collision_s);
  state_.last_local_collision_s = 0.0;
  state_.last_local_collision_receive_s = 0.0;
}

}  // namespace lingtu::nav::endpoint
