#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lingtu/maps/block_grid.hpp"
#include "lingtu/maps/cloud.hpp"
#include "lingtu/maps/frame.hpp"
#include "lingtu/maps/layers/grid.hpp"

namespace lingtu::maps {

struct ScenePose {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  double qx{0.0};
  double qy{0.0};
  double qz{0.0};
  double qw{1.0};
};

struct ScenePath {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::vector<float> x_m;
  std::vector<float> y_m;
  std::vector<float> z_m;
};

struct SceneHealth {
  bool localization_ok{false};
  bool map_ok{false};
  bool planner_ok{false};
  std::uint32_t status_bits{0};
  std::string message;
};

struct SceneSnapshot {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  SceneHealth health;
  ScenePose robot_pose;
  OwnedPointCloud live_cloud;
  BlockGridSnapshot accumulated_cloud;
  layers::Grid2D floor_height;
  layers::Grid2D traversability_cost;
  ScenePath path;
};

class SceneSnapshotBuilder final {
 public:
  void SetFrame(std::string frame_id, std::int64_t stamp_ns);
  void SetHealth(SceneHealth health);
  void SetRobotPose(ScenePose pose);
  void SetLiveCloud(const PointCloudView &cloud);
  void SetAccumulatedCloud(BlockGridSnapshot snapshot);
  void SetFloorHeight(layers::Grid2D floor_height);
  void SetTraversabilityCost(layers::Grid2D cost);
  void SetPath(ScenePath path);

  SceneSnapshot Build() const;

 private:
  static OwnedPointCloud CopyCloud(const PointCloudView &cloud);
  std::string frame_id_{"map"};
  std::int64_t stamp_ns_{0};
  SceneHealth health_;
  ScenePose robot_pose_;
  OwnedPointCloud live_cloud_;
  BlockGridSnapshot accumulated_cloud_;
  layers::Grid2D floor_height_;
  layers::Grid2D traversability_cost_;
  ScenePath path_;
};

}  // namespace lingtu::maps
