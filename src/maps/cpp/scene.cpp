#include "lingtu/maps/scene.hpp"

#include <algorithm>
#include <stdexcept>

namespace lingtu::maps {

void SceneSnapshotBuilder::SetFrame(std::string frame_id, std::int64_t stamp_ns) {
  frame_id_ = frame_id.empty() ? "map" : std::move(frame_id);
  stamp_ns_ = stamp_ns;
}

void SceneSnapshotBuilder::SetHealth(SceneHealth health) {
  health_ = std::move(health);
}

void SceneSnapshotBuilder::SetRobotPose(ScenePose pose) {
  robot_pose_ = pose;
}

void SceneSnapshotBuilder::SetLiveCloud(const PointCloudView &cloud) {
  live_cloud_ = CopyCloud(cloud);
}

void SceneSnapshotBuilder::SetAccumulatedCloud(BlockGridSnapshot snapshot) {
  accumulated_cloud_ = std::move(snapshot);
}

void SceneSnapshotBuilder::SetFloorHeight(layers::Grid2D floor_height) {
  floor_height.validate("scene.floor_height");
  floor_height_ = std::move(floor_height);
}

void SceneSnapshotBuilder::SetTraversabilityCost(layers::Grid2D cost) {
  cost.validate("scene.traversability_cost");
  traversability_cost_ = std::move(cost);
}

void SceneSnapshotBuilder::SetPath(ScenePath path) {
  if (path.x_m.size() != path.y_m.size() || path.x_m.size() != path.z_m.size()) {
    throw std::invalid_argument("scene path coordinate arrays must have matching sizes");
  }
  path_ = std::move(path);
}

SceneSnapshot SceneSnapshotBuilder::Build() const {
  SceneSnapshot snapshot;
  snapshot.frame_id = frame_id_;
  snapshot.stamp_ns = stamp_ns_;
  snapshot.health = health_;
  snapshot.robot_pose = robot_pose_;
  snapshot.live_cloud = live_cloud_;
  snapshot.accumulated_cloud = accumulated_cloud_;
  snapshot.floor_height = floor_height_;
  snapshot.traversability_cost = traversability_cost_;
  snapshot.path = path_;
  if (snapshot.live_cloud.frame_id.empty()) {
    snapshot.live_cloud.frame_id = frame_id_;
  }
  if (snapshot.live_cloud.stamp_ns == 0) {
    snapshot.live_cloud.stamp_ns = stamp_ns_;
  }
  if (snapshot.accumulated_cloud.frame_id.empty()) {
    snapshot.accumulated_cloud.frame_id = frame_id_;
  }
  if (snapshot.accumulated_cloud.stamp_ns == 0) {
    snapshot.accumulated_cloud.stamp_ns = stamp_ns_;
  }
  if (snapshot.path.frame_id.empty()) {
    snapshot.path.frame_id = frame_id_;
  }
  if (snapshot.path.stamp_ns == 0) {
    snapshot.path.stamp_ns = stamp_ns_;
  }
  return snapshot;
}

OwnedPointCloud SceneSnapshotBuilder::CopyCloud(const PointCloudView &cloud) {
  OwnedPointCloud out;
  out.frame_id = cloud.frame_id.empty() ? "map" : cloud.frame_id;
  out.stamp_ns = cloud.stamp_ns;
  out.layout = cloud.layout;
  out.point_count = cloud.point_count;
  switch (cloud.layout) {
    case CloudLayout::kXyzF32Interleaved:
      if (cloud.point_count > 0U &&
          (cloud.interleaved.data == nullptr || cloud.interleaved.size < cloud.point_count * 3U)) {
        throw std::invalid_argument("live cloud xyz interleaved buffer is too small");
      }
      if (cloud.point_count > 0U) {
        out.interleaved.assign(cloud.interleaved.data,
                               cloud.interleaved.data + cloud.point_count * 3U);
      }
      break;
    case CloudLayout::kXyziF32Interleaved:
      if (cloud.point_count > 0U &&
          (cloud.interleaved.data == nullptr || cloud.interleaved.size < cloud.point_count * 4U)) {
        throw std::invalid_argument("live cloud xyzi interleaved buffer is too small");
      }
      if (cloud.point_count > 0U) {
        out.interleaved.assign(cloud.interleaved.data,
                               cloud.interleaved.data + cloud.point_count * 4U);
      }
      break;
    case CloudLayout::kXyzF32SoA:
    case CloudLayout::kXyziF32SoA:
      if (cloud.point_count > 0U &&
          (cloud.x.data == nullptr || cloud.y.data == nullptr || cloud.z.data == nullptr ||
           cloud.x.size < cloud.point_count || cloud.y.size < cloud.point_count ||
           cloud.z.size < cloud.point_count)) {
        throw std::invalid_argument("live cloud SoA xyz buffers are too small");
      }
      if (cloud.point_count > 0U) {
        out.x.assign(cloud.x.data, cloud.x.data + cloud.point_count);
        out.y.assign(cloud.y.data, cloud.y.data + cloud.point_count);
        out.z.assign(cloud.z.data, cloud.z.data + cloud.point_count);
      }
      if (cloud.layout == CloudLayout::kXyziF32SoA && cloud.intensity.data != nullptr &&
          cloud.intensity.size >= cloud.point_count) {
        out.intensity.assign(cloud.intensity.data, cloud.intensity.data + cloud.point_count);
      }
      break;
  }
  return out;
}

}  // namespace lingtu::maps
