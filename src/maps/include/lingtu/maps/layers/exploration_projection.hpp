#pragma once

#include "lingtu/maps/layers/grid.hpp"
#include "lingtu/maps/layers/rolling_occupancy.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::maps::layers {

struct ExplorationProjectionConfig {
  float robot_z_m{0.0F};
  float obstacle_min_z_m{0.10F};
  float obstacle_max_z_m{1.20F};
  float inflation_radius_m{0.45F};
};

struct ExplorationGridSnapshot {
  std::string frame_id{"map"};
  std::int64_t stamp_ns{0};
  std::uint64_t generation{0U};
  Grid2D grid;
  std::vector<std::uint8_t> observed;
  std::size_t free_cells{0U};
  std::size_t occupied_cells{0U};
  std::size_t unknown_cells{0U};

  void Validate() const;
};

[[nodiscard]] ExplorationGridSnapshot ProjectExplorationGrid(
    const RollingOccupancySnapshot& occupancy,
    const ExplorationProjectionConfig& config);

}  // namespace lingtu::maps::layers
