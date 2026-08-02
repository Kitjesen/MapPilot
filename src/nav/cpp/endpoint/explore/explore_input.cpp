#include "explore/explore_input.hpp"

#include <cmath>
#include <cstdint>
#include <string>

namespace lingtu::nav::endpoint {
namespace {

std::string ddsString(const char *value) {
  return value == nullptr ? std::string{} : std::string(value);
}

void reject(std::string *reason, const char *value) {
  if (reason != nullptr) {
    *reason = value;
  }
}

bool identityGridOrigin(const lingtu_dds_Quaternion &orientation) {
  if (!quaternionIsFiniteAndNonzero(orientation)) {
    return false;
  }
  const auto normalized = normalizeQuaternion(orientation);
  constexpr double kTolerance = 1e-6;
  return std::abs(normalized.x) <= kTolerance && std::abs(normalized.y) <= kTolerance &&
         std::abs(normalized.z) <= kTolerance &&
         std::abs(std::abs(normalized.w) - 1.0) <= kTolerance;
}

}  // namespace

bool sourceStampFresh(double source_stamp_s, double now_s, double maximum_age_s,
                      double future_tolerance_s) {
  if (!std::isfinite(source_stamp_s) || source_stamp_s <= 0.0 || !std::isfinite(now_s) ||
      now_s <= 0.0 || !std::isfinite(maximum_age_s) || maximum_age_s < 0.0 ||
      !std::isfinite(future_tolerance_s) || future_tolerance_s < 0.0) {
    return false;
  }
  const double age_s = now_s - source_stamp_s;
  return age_s >= -future_tolerance_s && age_s <= maximum_age_s;
}

std::optional<ExplorationSnapshot>
parseExplorationSnapshot(const lingtu_dds_ExplorationGrid &message,
                         const ExploreInputGateConfig &config, std::string *rejection_reason) {
  ExplorationSnapshot snapshot;
  snapshot.stamp_s = ddsStampSeconds(message.header.stamp);
  snapshot.identity.frame_id = ddsString(message.header.frame_id);
  snapshot.identity.session_id = ddsString(message.session_id);
  snapshot.identity.map_id = ddsString(message.map_id);
  snapshot.identity.map_version = message.map_version;
  snapshot.identity.artifact_hash = ddsString(message.artifact_hash);
  snapshot.identity.reset_epoch = message.reset_epoch;
  snapshot.identity.generation = message.generation;
  snapshot.identity.live = message.live;

  if (snapshot.identity.frame_id != "map") {
    reject(rejection_reason, "snapshot_frame_not_map");
    return std::nullopt;
  }
  if (!snapshot.identity.valid()) {
    reject(rejection_reason, "snapshot_identity_invalid");
    return std::nullopt;
  }
  if (!std::isfinite(snapshot.stamp_s) || snapshot.stamp_s <= 0.0) {
    reject(rejection_reason, "snapshot_stamp_invalid");
    return std::nullopt;
  }
  if (!std::isfinite(message.info.resolution) || message.info.resolution <= 0.0F ||
      message.info.width == 0U || message.info.height == 0U) {
    reject(rejection_reason, "snapshot_geometry_invalid");
    return std::nullopt;
  }
  if (!std::isfinite(message.info.origin.position.x) ||
      !std::isfinite(message.info.origin.position.y) ||
      !identityGridOrigin(message.info.origin.orientation)) {
    reject(rejection_reason, "snapshot_origin_unsupported");
    return std::nullopt;
  }
  const auto width = static_cast<std::size_t>(message.info.width);
  const auto height = static_cast<std::size_t>(message.info.height);
  if (config.max_grid_cells == 0U || width > config.max_grid_cells / height) {
    reject(rejection_reason, "snapshot_grid_resource_limit");
    return std::nullopt;
  }
  const std::size_t cell_count = width * height;
  if (cell_count > config.max_grid_cells || message.data._buffer == nullptr ||
      message.data._length != cell_count) {
    reject(rejection_reason, cell_count > config.max_grid_cells ? "snapshot_grid_resource_limit"
                                                                : "snapshot_data_size_mismatch");
    return std::nullopt;
  }

  snapshot.grid.width = static_cast<int>(message.info.width);
  snapshot.grid.height = static_cast<int>(message.info.height);
  snapshot.grid.resolution = message.info.resolution;
  snapshot.grid.origin_x = message.info.origin.position.x;
  snapshot.grid.origin_y = message.info.origin.position.y;
  snapshot.grid.cells.reserve(cell_count);
  for (std::size_t index = 0U; index < cell_count; ++index) {
    const auto raw = static_cast<std::uint8_t>(message.data._buffer[index]);
    if (raw == 0U) {
      snapshot.grid.cells.push_back(lingtu::explore::kFree);
    } else if (raw == 100U) {
      snapshot.grid.cells.push_back(lingtu::explore::kOccupied);
    } else if (raw == 255U) {
      snapshot.grid.cells.push_back(lingtu::explore::kUnknown);
    } else {
      reject(rejection_reason, "snapshot_not_trinary");
      return std::nullopt;
    }
  }
  reject(rejection_reason, "accepted");
  return snapshot;
}

std::optional<TimedMapPose> mapPoseFromOdometry(const lingtu_dds_Odometry &message,
                                                const std::optional<RigidTransform> &map_odom,
                                                double map_odom_receive_s, double now_s,
                                                const ExploreInputGateConfig &config,
                                                std::string *rejection_reason) {
  const std::string frame_id =
      message.header.frame_id == nullptr ? std::string{} : message.header.frame_id;
  const RigidTransform source_body = rigidTransformFromOdometry(message);
  if (!source_body.valid) {
    reject(rejection_reason, "odometry_pose_invalid");
    return std::nullopt;
  }
  if (!sourceStampFresh(source_body.stamp_s, now_s, config.odometry_max_age_s,
                        config.future_tolerance_s)) {
    reject(rejection_reason, "odometry_stale");
    return std::nullopt;
  }

  RigidTransform map_body = source_body;
  if (frame_id == "odom") {
    if (!map_odom.has_value() || !map_odom->valid) {
      reject(rejection_reason, "map_odom_transform_missing");
      return std::nullopt;
    }
    if (!sourceStampFresh(map_odom_receive_s, now_s, config.transform_max_age_s,
                          config.future_tolerance_s) ||
        !sourceStampFresh(map_odom->stamp_s, now_s, config.transform_max_age_s,
                          config.future_tolerance_s)) {
      reject(rejection_reason, "map_odom_transform_stale");
      return std::nullopt;
    }
    map_body = composeTransforms(*map_odom, source_body);
  } else if (frame_id != "map") {
    reject(rejection_reason, "odometry_frame_unsupported");
    return std::nullopt;
  }

  TimedMapPose result;
  result.pose = {
      map_body.translation.x,
      map_body.translation.y,
      map_body.yaw,
  };
  result.stamp_s = source_body.stamp_s;
  reject(rejection_reason, "accepted");
  return result;
}

bool snapshotFresh(const ExplorationSnapshot &snapshot, double now_s,
                   const ExploreInputGateConfig &config, std::string *rejection_reason) {
  if (!sourceStampFresh(snapshot.stamp_s, now_s, config.snapshot_max_age_s,
                        config.future_tolerance_s)) {
    reject(rejection_reason, "snapshot_stale");
    return false;
  }
  reject(rejection_reason, "accepted");
  return true;
}

}  // namespace lingtu::nav::endpoint
