#include "rolling_exploration_map.hpp"

#include "lingtu/maps/cloud.hpp"

#include <algorithm>
#include <stdexcept>

namespace lingtu::nav::endpoint {

lingtu::maps::layers::RollingOccupancyConfig
RollingExplorationMap::MakeCoreConfig(
    const RollingExplorationConfig& config) {
  lingtu::maps::layers::RollingOccupancyConfig core;
  core.size_x = config.size_x;
  core.size_y = config.size_y;
  core.size_z = config.size_z;
  core.resolution_m = config.resolution_m;
  core.max_ray_range_m = config.max_ray_range_m;
  core.roll_margin_x = std::max(1, config.size_x / 4);
  core.roll_margin_y = std::max(1, config.size_y / 4);
  core.roll_margin_z = std::max(1, config.size_z / 4);
  core.decay_after_ns = config.decay_after_ns;
  core.decay_factor = config.decay_factor;
  core.auto_roll = true;
  core.reject_out_of_order = true;
  return core;
}

RollingExplorationMap::RollingExplorationMap(
    RollingExplorationConfig config)
    : config_(config),
      occupancy_(MakeCoreConfig(config)) {
  if (config_.obstacle_min_z_m > config_.obstacle_max_z_m ||
      config_.inflation_radius_m < 0.0F) {
    throw std::invalid_argument(
        "rolling exploration projection configuration is invalid");
  }
}

void RollingExplorationMap::Reset(
    float center_x_m,
    float center_y_m,
    float center_z_m,
    std::int64_t stamp_ns) {
  occupancy_.Reset(
      "map",
      center_x_m,
      center_y_m,
      center_z_m,
      stamp_ns);
  initialized_ = true;
}

lingtu::maps::layers::ExplorationGridSnapshot
RollingExplorationMap::Update(
    const std::vector<float>& xyz,
    float sensor_origin_x_m,
    float sensor_origin_y_m,
    float sensor_origin_z_m,
    float robot_z_m,
    std::int64_t stamp_ns) {
  if (xyz.size() % 3U != 0U || stamp_ns < 0) {
    throw std::invalid_argument(
        "rolling exploration scan layout or timestamp is invalid");
  }
  if (!initialized_) {
    Reset(
        sensor_origin_x_m,
        sensor_origin_y_m,
        sensor_origin_z_m,
        stamp_ns);
  }

  lingtu::maps::PointCloudView cloud;
  cloud.frame_id = "map";
  cloud.stamp_ns = stamp_ns;
  cloud.layout =
      lingtu::maps::CloudLayout::kXyzF32Interleaved;
  cloud.point_count = xyz.size() / 3U;
  cloud.interleaved = {xyz.data(), xyz.size()};

  lingtu::maps::MapCloudFrame frame;
  frame.cloud = cloud;
  frame.sensor_origin_x_m = sensor_origin_x_m;
  frame.sensor_origin_y_m = sensor_origin_y_m;
  frame.sensor_origin_z_m = sensor_origin_z_m;
  frame.incremental = true;
  occupancy_.Update(frame);

  lingtu::maps::layers::ExplorationProjectionConfig projection;
  projection.robot_z_m = robot_z_m;
  projection.obstacle_min_z_m =
      config_.obstacle_min_z_m;
  projection.obstacle_max_z_m =
      config_.obstacle_max_z_m;
  projection.inflation_radius_m =
      config_.inflation_radius_m;
  return lingtu::maps::layers::ProjectExplorationGrid(
      occupancy_.Snapshot(),
      projection);
}

}  // namespace lingtu::nav::endpoint
