#pragma once

#include "lingtu/maps/layers/grid.hpp"

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace lingtu::maps::layers {

inline std::vector<std::int8_t> encodeExplorationOccupancy(
    const Grid2D& cost_grid,
    const std::vector<std::uint8_t>& observed,
    float occupied_threshold = 65.0F) {
  cost_grid.validate("exploration cost grid");
  if (observed.size() != cost_grid.data.size()) {
    throw std::invalid_argument(
        "exploration observed mask size does not match grid");
  }

  std::vector<std::int8_t> cells(cost_grid.data.size(), -1);
  for (std::size_t index = 0; index < cost_grid.data.size(); ++index) {
    if (observed[index] == 0U) {
      continue;
    }
    const float cost = cost_grid.data[index];
    if (!std::isfinite(cost)) {
      continue;
    }
    cells[index] = cost >= occupied_threshold ? 100 : 0;
  }
  return cells;
}

}  // namespace lingtu::maps::layers
