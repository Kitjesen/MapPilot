#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lingtu/maps/layers/exploration_projection.hpp"
#include "lingtu/maps/layers/rolling_occupancy.hpp"

namespace lingtu::nav::endpoint {

struct RollingExplorationConfig {
  int size_x{150};
  int size_y{150};
  int size_z{40};
  float resolution_m{0.20F};
  float max_ray_range_m{30.0F};
  float obstacle_min_z_m{0.10F};
  float obstacle_max_z_m{1.20F};
  float inflation_radius_m{0.45F};
  std::int64_t decay_after_ns{8'000'000'000LL};
  float decay_factor{0.85F};
};

class RollingExplorationMap {
 public:
  explicit RollingExplorationMap(RollingExplorationConfig config = {});

  void Reset(float center_x_m, float center_y_m, float center_z_m, std::int64_t stamp_ns);

  void Clear() noexcept { initialized_ = false; }

  [[nodiscard]] lingtu::maps::layers::ExplorationGridSnapshot
  Update(const std::vector<float> &xyz, float sensor_origin_x_m, float sensor_origin_y_m,
         float sensor_origin_z_m, float robot_z_m, std::int64_t stamp_ns);

  [[nodiscard]] bool initialized() const noexcept { return initialized_; }

 private:
  static lingtu::maps::layers::RollingOccupancyConfig
  MakeCoreConfig(const RollingExplorationConfig &config);

  RollingExplorationConfig config_;
  lingtu::maps::layers::RollingOccupancyGrid occupancy_;
  bool initialized_{false};
};

}  // namespace lingtu::nav::endpoint
