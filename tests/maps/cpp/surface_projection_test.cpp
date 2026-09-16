#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

#include "lingtu/maps/layers/ground_surface.hpp"
#include "lingtu/maps/layers/surface_projection.hpp"

using namespace lingtu::maps::layers;

namespace {

GroundSurfaceResult EstimateFineSurface(const Grid2D& fine) {
  const auto geometry = GroundSurfaceGeometry(fine);
  std::vector<float> xyz;
  xyz.reserve(fine.data.size() * 3U);
  for (int row = 0; row < fine.rows; ++row) {
    for (int col = 0; col < fine.cols; ++col) {
      const float z = fine.data[static_cast<std::size_t>(fine.index(row, col))];
      if (!std::isfinite(z)) continue;
      xyz.push_back(static_cast<float>(fine.originX + (col + 0.5) * fine.resolution));
      xyz.push_back(static_cast<float>(fine.originY + (row + 0.5) * fine.resolution));
      xyz.push_back(z);
    }
  }
  return EstimateGroundSurface(xyz, geometry);
}

}  // namespace

int main() {
  auto floor = makeGrid2D(40, 40, 0.05, -1.0, -1.0, -0.30F);
  RollingOccupancySnapshot occupancy;
  occupancy.size_x = occupancy.size_y = 40;
  occupancy.size_z = 40;
  occupancy.origin_x_m = occupancy.origin_y_m = -1.0;
  occupancy.origin_z_m = -0.5;
  occupancy.state.assign(40 * 40 * 40, static_cast<std::uint8_t>(OccupancyState::kUnknown));
  auto occupied = [&](int x, int y, int z) {
    occupancy.state[occupancy.Index(x, y, z)] = static_cast<std::uint8_t>(OccupancyState::kOccupied);
  };
  for (int y = 0; y < 40; ++y) {
    for (int x = 0; x < 40; ++x) {
      occupied(x, y, 4);  // Occupied floor voxel at -0.275 m.
      occupied(x, y, 35);  // Ceiling, well above the robot-height display band.
    }
  }
  const auto original_states = occupancy.state;
  const auto original_heights = floor.data;
  auto surface = EstimateFineSurface(floor);
  auto result = ProjectSupportSurface(surface, occupancy, 0, 0, 0);
  assert(result.rows == 10 && result.cols == 10);
  assert(std::all_of(result.data.begin(), result.data.end(), [](float v) { return v == 0; }));
  assert(occupancy.state == original_states && floor.data == original_heights);

  occupied(22, 22, 10);  // Obstacle at +0.025 m, above nearby ground.
  result = ProjectSupportSurface(surface, occupancy, 0, 0, 0);
  assert(result.data[result.index(5, 5)] == 100);
  assert(std::count(result.data.begin(), result.data.end(), 100.0F) == 1);

  const auto classified = result.data;
  for (std::size_t i = 0; i < floor.data.size(); ++i) {
    floor.data[i] += (i % 2 ? 0.015F : -0.015F);
  }
  surface = EstimateFineSurface(floor);
  assert(ProjectSupportSurface(surface, occupancy, 0, 0, 0).data == classified);

  // Free volume cannot manufacture a floor in an unobserved column.
  for (int y = 24; y < 28; ++y) {
    for (int x = 24; x < 28; ++x) {
      floor.data[floor.index(y, x)] = std::numeric_limits<float>::quiet_NaN();
    }
  }
  surface = EstimateFineSurface(floor);
  result = ProjectSupportSurface(surface, occupancy, 0, 0, 0);
  assert(result.data[result.index(6, 6)] == -1);

  // A raised object must not join the ground surface merely because its top is observed.
  for (int y = 28; y < 32; ++y) {
    for (int x = 24; x < 28; ++x) floor.data[floor.index(y, x)] = 0.15F;
  }
  occupied(26, 30, 13);
  surface = EstimateFineSurface(floor);
  result = ProjectSupportSurface(surface, occupancy, 0, 0, 0);
  assert(result.data[result.index(7, 6)] == 100);

  auto ceiling_only = floor;
  std::fill(ceiling_only.data.begin(), ceiling_only.data.end(), 1.25F);
  result = ProjectSupportSurface(EstimateFineSurface(ceiling_only), occupancy, 0, 0, 0);
  assert(std::all_of(result.data.begin(), result.data.end(), [](float v) { return v == -1; }));

  // Rolling the source by 5 cm must keep coarse cells aligned in world coordinates.
  auto rolled_floor = makeGrid2D(40, 40, 0.05, -0.95, -1.0, -0.30F);
  result = ProjectSupportSurface(EstimateFineSurface(rolled_floor), occupancy, 0, 0, 0);
  assert(std::abs(result.originX + 0.8) < 1e-7);
  const int obstacle_col = static_cast<int>(std::floor((0.125 - result.originX) / result.resolution));
  const int obstacle_row = static_cast<int>(std::floor((0.125 - result.originY) / result.resolution));
  assert(result.data[result.index(obstacle_row, obstacle_col)] == 100);
  assert(result.originX >= rolled_floor.originX);
  assert(result.originX + result.cols * result.resolution <=
         rolled_floor.originX + rolled_floor.cols * rolled_floor.resolution + 1e-7);
  auto blind_near_field = makeGrid2D(80, 80, 0.05, -2.0, -2.0, -0.30F);
  for (int y = 0; y < 80; ++y) {
    for (int x = 0; x < 80; ++x) {
      if (std::hypot(-2.0 + (x + 0.5) * 0.05, -2.0 + (y + 0.5) * 0.05) < 1.3) {
        blind_near_field.data[blind_near_field.index(y, x)] = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }
  result = ProjectSupportSurface(EstimateFineSurface(blind_near_field), occupancy, 0, 0, 0);
  assert(result.data[result.index(10, 10)] == -1);
  assert(result.data[result.index(10, 18)] == 0);

  // Height-band optimization must retain obstacles at both boundary edges.
  auto level_floor = makeGrid2D(40, 40, 0.05, -1.0, -1.0, 0.0F);
  for (double height : {0.11999, 0.12001, 0.79999, 0.80001}) {
    std::fill(occupancy.state.begin(), occupancy.state.end(),
              static_cast<std::uint8_t>(OccupancyState::kUnknown));
    occupancy.origin_z_m = height - 10.5 * occupancy.resolution_m;
    occupied(22, 22, 10);
    result = ProjectSupportSurface(EstimateFineSurface(level_floor), occupancy, 0, 0, 0.30);
    assert(result.data[result.index(5, 5)] ==
           (height > 0.12 && height <= 0.8 ? 100.0F : 0.0F));
  }
  // Clearing an occupied voxel must clear the projected obstacle immediately.
  occupancy.state[occupancy.Index(22, 22, 10)] =
      static_cast<std::uint8_t>(OccupancyState::kUnknown);
  occupancy.origin_z_m = -0.5;
  result = ProjectSupportSurface(EstimateFineSurface(level_floor), occupancy, 0, 0, 0.30);
  assert(result.data[result.index(5, 5)] == 0.0F);

  assert(ProjectSupportSurface({}, occupancy, 0, 0, 0).empty());
}
