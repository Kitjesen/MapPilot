#include "lingtu/maps/layers/exploration_projection.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lingtu::maps::layers {

void ExplorationGridSnapshot::Validate() const {
  if (frame_id.empty() || stamp_ns < 0 || generation == 0U) {
    throw std::invalid_argument(
        "exploration grid identity is incomplete");
  }
  grid.validate("exploration_grid");
  if (observed.size() != grid.data.size()) {
    throw std::invalid_argument(
        "exploration grid observed mask does not match geometry");
  }
  const std::size_t total =
      free_cells + occupied_cells + unknown_cells;
  if (total != grid.data.size()) {
    throw std::invalid_argument(
        "exploration grid cell counters do not match geometry");
  }
}

ExplorationGridSnapshot ProjectExplorationGrid(
    const RollingOccupancySnapshot& occupancy,
    const ExplorationProjectionConfig& config) {
  occupancy.Validate();
  if (!std::isfinite(config.robot_z_m) ||
      !std::isfinite(config.obstacle_min_z_m) ||
      !std::isfinite(config.obstacle_max_z_m) ||
      !std::isfinite(config.inflation_radius_m) ||
      config.obstacle_min_z_m > config.obstacle_max_z_m ||
      config.inflation_radius_m < 0.0F) {
    throw std::invalid_argument(
        "exploration projection configuration is invalid");
  }

  ExplorationGridSnapshot result;
  result.frame_id = occupancy.frame_id;
  result.stamp_ns = occupancy.stamp_ns;
  result.generation = occupancy.generation;
  result.grid = makeGrid2D(
      occupancy.size_y,
      occupancy.size_x,
      occupancy.resolution_m,
      occupancy.origin_x_m,
      occupancy.origin_y_m,
      0.0F);
  result.observed.assign(result.grid.data.size(), 0U);

  std::vector<std::uint8_t> occupied(
      result.grid.data.size(),
      0U);
  for (std::int32_t y = 0; y < occupancy.size_y; ++y) {
    for (std::int32_t x = 0; x < occupancy.size_x; ++x) {
      bool column_free = false;
      bool column_occupied = false;
      for (std::int32_t z = 0; z < occupancy.size_z; ++z) {
        const float center_z =
            occupancy.origin_z_m +
            (static_cast<float>(z) + 0.5F) *
                occupancy.resolution_m;
        const float relative_z = center_z - config.robot_z_m;
        if (relative_z < config.obstacle_min_z_m ||
            relative_z > config.obstacle_max_z_m) {
          continue;
        }
        const auto state = static_cast<OccupancyState>(
            occupancy.state[occupancy.Index(x, y, z)]);
        if (state == OccupancyState::kOccupied) {
          column_occupied = true;
          break;
        }
        if (state == OccupancyState::kFree) {
          column_free = true;
        }
      }

      const std::size_t position = static_cast<std::size_t>(
          result.grid.index(y, x));
      if (column_occupied) {
        result.grid.data[position] = 100.0F;
        result.observed[position] = 1U;
        occupied[position] = 1U;
      } else if (column_free) {
        result.grid.data[position] = 0.0F;
        result.observed[position] = 1U;
      }
    }
  }

  const int inflation_cells = static_cast<int>(
      std::ceil(
          static_cast<double>(config.inflation_radius_m) /
          result.grid.resolution));
  const double inflation_squared =
      static_cast<double>(config.inflation_radius_m) *
      static_cast<double>(config.inflation_radius_m);
  for (int row = 0; row < result.grid.rows; ++row) {
    for (int col = 0; col < result.grid.cols; ++col) {
      const std::size_t source = static_cast<std::size_t>(
          result.grid.index(row, col));
      if (occupied[source] == 0U) {
        continue;
      }
      for (int dr = -inflation_cells; dr <= inflation_cells; ++dr) {
        for (int dc = -inflation_cells; dc <= inflation_cells; ++dc) {
          const int target_row = row + dr;
          const int target_col = col + dc;
          if (target_row < 0 || target_row >= result.grid.rows ||
              target_col < 0 || target_col >= result.grid.cols) {
            continue;
          }
          const double dx =
              static_cast<double>(dc) * result.grid.resolution;
          const double dy =
              static_cast<double>(dr) * result.grid.resolution;
          if (dx * dx + dy * dy >
              inflation_squared + 1e-12) {
            continue;
          }
          const std::size_t target = static_cast<std::size_t>(
              result.grid.index(target_row, target_col));
          result.grid.data[target] = 100.0F;
          result.observed[target] = 1U;
        }
      }
    }
  }

  for (std::size_t index = 0;
       index < result.grid.data.size();
       ++index) {
    if (result.observed[index] == 0U) {
      ++result.unknown_cells;
    } else if (result.grid.data[index] >= 65.0F) {
      ++result.occupied_cells;
    } else {
      ++result.free_cells;
    }
  }
  result.Validate();
  return result;
}

}  // namespace lingtu::maps::layers
