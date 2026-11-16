#include "lingtu/maps/mapd/region_update.hpp"

#include <cmath>

namespace lingtu::maps::mapd {
namespace {

bool BitSet(const std::vector<std::uint8_t> &bits, std::size_t index) {
  return (bits[index / 8U] & (std::uint8_t{1U} << (index % 8U))) != 0U;
}

}  // namespace

std::optional<RegionUpdate> BuildRegionUpdate(
  const Snapshot &snapshot,
  std::int64_t map_content_epoch) {
  const auto &collision = snapshot.collision;
  if (!collision.complete || !std::isfinite(collision.resolution_m) ||
      collision.resolution_m <= 0.0 || collision.size_x <= 0 ||
      collision.size_y <= 0 || collision.size_z <= 0 ||
      collision.measured_occupied_bits.size() !=
        (static_cast<std::size_t>(collision.size_x) *
         static_cast<std::size_t>(collision.size_y) *
         static_cast<std::size_t>(collision.size_z) + 7U) / 8U ||
      collision.known_free_bits.size() != collision.measured_occupied_bits.size()) {
    return std::nullopt;
  }

  RegionUpdate update;
  update.frame_id = snapshot.frame_id;
  update.resolution_m = collision.resolution_m;
  update.origin_x_m = collision.min_x_m;
  update.origin_y_m = collision.min_y_m;
  update.origin_z_m = collision.min_z_m;
  update.min_x = 0;
  update.max_x = collision.size_x - 1;
  update.min_y = 0;
  update.max_y = collision.size_y - 1;
  update.min_z = 0;
  update.max_z = collision.size_z - 1;
  update.map_content_epoch = map_content_epoch;
  update.reset_epoch = snapshot.reset_epoch;
  update.sequence = snapshot.sequence;
  update.generation = snapshot.generation;

  const auto cell_count = static_cast<std::size_t>(collision.size_x) *
      static_cast<std::size_t>(collision.size_y) *
      static_cast<std::size_t>(collision.size_z);
  update.occupied.reserve(collision.occupied_cells);
  update.known_free.reserve(cell_count / 4U);
  for (std::size_t index = 0; index < cell_count; ++index) {
    const auto x = static_cast<std::int32_t>(index % static_cast<std::size_t>(collision.size_x));
    const auto yz = index / static_cast<std::size_t>(collision.size_x);
    const auto y = static_cast<std::int32_t>(yz % static_cast<std::size_t>(collision.size_y));
    const auto z = static_cast<std::int32_t>(yz / static_cast<std::size_t>(collision.size_y));
    const RegionCell cell{x, y, z};
    if (BitSet(collision.measured_occupied_bits, index)) update.occupied.push_back(cell);
    if (BitSet(collision.known_free_bits, index)) update.known_free.push_back(cell);
  }
  return update;
}

}  // namespace lingtu::maps::mapd
