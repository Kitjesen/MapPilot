#pragma once

#include "lingtu/maps/mapd/engine.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace lingtu::maps::mapd {

struct RegionCell {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const RegionCell &other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
};

// A mapd collision snapshot expressed in the integer-cell contract consumed by
// the Rust MLS adapter. The cell coordinates are local to origin_*_m.
struct RegionUpdate {
  std::string frame_id{"map"};
  double resolution_m{0.0};
  double origin_x_m{0.0};
  double origin_y_m{0.0};
  double origin_z_m{0.0};
  std::int32_t min_x{0};
  std::int32_t max_x{-1};
  std::int32_t min_y{0};
  std::int32_t max_y{-1};
  std::int32_t min_z{0};
  std::int32_t max_z{-1};
  std::int64_t map_content_epoch{0};
  std::uint64_t reset_epoch{0U};
  std::uint64_t sequence{0U};
  std::uint64_t generation{0U};
  std::vector<RegionCell> occupied;
  std::vector<RegionCell> known_free;
};

// Convert one complete mapd collision snapshot into the region contract. The
// measured occupied bits are used as occupied evidence, while inflated bits
// remain owned by local collision handling.
std::optional<RegionUpdate> BuildRegionUpdate(
  const Snapshot &snapshot,
  std::int64_t map_content_epoch);

}  // namespace lingtu::maps::mapd
