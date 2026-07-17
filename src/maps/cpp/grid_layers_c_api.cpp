#include "lingtu/maps/c_api/grid_layers.h"

#include "lingtu/maps/layers/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <stdexcept>
#include <utility>
#include <vector>

using lingtu::maps::layers::ElevationMapResult;
using lingtu::maps::layers::Grid2D;
using lingtu::maps::layers::TerrainRiskParams;
using lingtu::maps::layers::TraversabilityParams;
using lingtu::maps::layers::buildElevationMap;
using lingtu::maps::layers::computeEsdf;
using lingtu::maps::layers::computeTerrainRisk;
using lingtu::maps::layers::fuseTraversabilityCost;
using lingtu::maps::layers::makeGrid2D;

namespace {

template <typename Fn>
int32_t Protect(Fn&& fn) {
  try {
    fn();
    return 0;
  } catch (const std::exception&) {
    return -2;
  } catch (...) {
    return -3;
  }
}

bool ValidSpec(const LingtuMapsGridSpec* spec) {
  return spec != nullptr &&
      spec->rows > 0 &&
      spec->cols > 0 &&
      spec->resolution_m > 0.0;
}

uint64_t CellCount(const LingtuMapsGridSpec& spec) {
  return static_cast<uint64_t>(spec.rows) * static_cast<uint64_t>(spec.cols);
}

void FillSpec(const Grid2D& grid, LingtuMapsGridSpec* out) {
  out->rows = grid.rows;
  out->cols = grid.cols;
  out->resolution_m = grid.resolution;
  out->origin_x_m = grid.originX;
  out->origin_y_m = grid.originY;
}

Grid2D GridFromArray(const float* data, const LingtuMapsGridSpec& spec) {
  Grid2D out = makeGrid2D(
      spec.rows,
      spec.cols,
      spec.resolution_m,
      spec.origin_x_m,
      spec.origin_y_m,
      0.0F);
  const uint64_t n = CellCount(spec);
  out.data.assign(data, data + n);
  return out;
}

Grid2D OptionalGridFromArray(const float* data, const LingtuMapsGridSpec& spec) {
  if (data == nullptr) {
    return {};
  }
  return GridFromArray(data, spec);
}

template <typename T>
void CopyVector(const std::vector<T>& src, T* dst) {
  std::copy(src.begin(), src.end(), dst);
}

TerrainRiskParams ToTerrainParams(const LingtuMapsTerrainRiskParams* in) {
  TerrainRiskParams out;
  if (in == nullptr) {
    return out;
  }
  out.maxSlopeDeg = in->max_slope_deg;
  out.softSlopeStartDeg = in->soft_slope_start_deg;
  out.criticalStepM = in->critical_step_m;
  out.roughnessCriticalM = in->roughness_critical_m;
  return out;
}

TraversabilityParams ToTraversabilityParams(const LingtuMapsTraversabilityParams* in) {
  TraversabilityParams out;
  if (in == nullptr) {
    return out;
  }
  out.lethal = in->lethal;
  out.inscribed = in->inscribed;
  out.maxSlopeDeg = in->max_slope_deg;
  out.softSlopeStartDeg = in->soft_slope_start_deg;
  out.safeDistance = in->safe_distance_m;
  out.proximityCap = in->proximity_cap;
  return out;
}

struct OccupancyConfig {
  double robot_x_m{0.0};
  double robot_y_m{0.0};
  double robot_yaw_rad{0.0};
  double resolution_m{0.2};
  double radius_m{30.0};
  double z_min_m{0.3};
  double z_max_m{2.0};
  double inflation_radius_m{0.25};
  double robot_clear_radius_m{0.6};
  double robot_clear_forward_m{0.0};
  double robot_clear_backward_m{0.0};
  double robot_clear_lateral_m{0.0};
  bool raycast_free_space{false};
  bool unknown_as_obstacle_for_costmap{false};
  uint32_t raycast_max_rays{1800};
  double raycast_free_inflation_radius_m{0.0};
};

constexpr int16_t kUnknown = -1;
constexpr int16_t kFree = 0;
constexpr int16_t kOccupied = 100;

int GridSize(const OccupancyConfig& cfg) {
  if (!(cfg.resolution_m > 0.0) || !(cfg.radius_m > 0.0)) {
    throw std::invalid_argument("occupancy resolution and radius must be positive");
  }
  const int size = static_cast<int>(std::floor((2.0 * cfg.radius_m) / cfg.resolution_m));
  if (size <= 0) {
    throw std::invalid_argument("computed occupancy grid is empty");
  }
  return size;
}

int CellIndex(int row, int col, int size) {
  return row * size + col;
}

std::vector<uint8_t> CircleKernel(double radius_m, double resolution_m, int& side) {
  const int r = std::max(1, static_cast<int>(std::ceil(radius_m / resolution_m)));
  side = r * 2 + 1;
  std::vector<uint8_t> kernel(static_cast<size_t>(side * side), 0);
  for (int y = -r; y <= r; ++y) {
    for (int x = -r; x <= r; ++x) {
      if (x * x + y * y <= r * r) {
        kernel[static_cast<size_t>((y + r) * side + (x + r))] = 1;
      }
    }
  }
  return kernel;
}

std::vector<uint8_t> Dilate(
    const std::vector<uint8_t>& input,
    int size,
    double radius_m,
    double resolution_m) {
  if (!(radius_m > 0.0)) {
    return input;
  }
  int kernel_side = 0;
  const auto kernel = CircleKernel(radius_m, resolution_m, kernel_side);
  const int pad = kernel_side / 2;
  std::vector<uint8_t> out(input.size(), 0);
  for (int row = 0; row < size; ++row) {
    for (int col = 0; col < size; ++col) {
      bool hit = false;
      for (int kr = 0; kr < kernel_side && !hit; ++kr) {
        const int rr = row + kr - pad;
        if (rr < 0 || rr >= size) {
          continue;
        }
        for (int kc = 0; kc < kernel_side; ++kc) {
          if (kernel[static_cast<size_t>(kr * kernel_side + kc)] == 0) {
            continue;
          }
          const int cc = col + kc - pad;
          if (cc < 0 || cc >= size) {
            continue;
          }
          if (input[static_cast<size_t>(CellIndex(rr, cc, size))] != 0) {
            hit = true;
            break;
          }
        }
      }
      out[static_cast<size_t>(CellIndex(row, col, size))] = hit ? 1 : 0;
    }
  }
  return out;
}

bool FootprintMask(double dx, double dy, const OccupancyConfig& cfg) {
  if (!(cfg.robot_clear_forward_m > 0.0) ||
      !(cfg.robot_clear_backward_m > 0.0) ||
      !(cfg.robot_clear_lateral_m > 0.0)) {
    return false;
  }
  const double c = std::cos(cfg.robot_yaw_rad);
  const double s = std::sin(cfg.robot_yaw_rad);
  const double body_x = c * dx + s * dy;
  const double body_y = -s * dx + c * dy;
  return body_x <= cfg.robot_clear_forward_m &&
      body_x >= -cfg.robot_clear_backward_m &&
      std::abs(body_y) <= cfg.robot_clear_lateral_m;
}

bool InsideRobotClear(double x, double y, const OccupancyConfig& cfg) {
  const double dx = x - cfg.robot_x_m;
  const double dy = y - cfg.robot_y_m;
  const double r = cfg.robot_clear_radius_m;
  return (dx * dx + dy * dy) < (r * r) || FootprintMask(dx, dy, cfg);
}

std::vector<uint8_t> RobotClearMask(
    int size,
    double origin_x,
    double origin_y,
    const OccupancyConfig& cfg) {
  std::vector<uint8_t> clear(static_cast<size_t>(size * size), 0);
  for (int row = 0; row < size; ++row) {
    const double y = origin_y + (static_cast<double>(row) + 0.5) * cfg.resolution_m;
    for (int col = 0; col < size; ++col) {
      const double x = origin_x + (static_cast<double>(col) + 0.5) * cfg.resolution_m;
      clear[static_cast<size_t>(CellIndex(row, col, size))] =
          InsideRobotClear(x, y, cfg) ? 1 : 0;
    }
  }
  return clear;
}

std::vector<std::pair<int, int>> Bresenham(int x0, int y0, int x1, int y1) {
  std::vector<std::pair<int, int>> cells;
  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0;
  int y = y0;
  while (true) {
    cells.emplace_back(x, y);
    if (x == x1 && y == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y += sy;
    }
  }
  return cells;
}

uint64_t DownsampleStep(uint64_t point_count, uint32_t max_points) {
  const uint32_t limit = std::max<uint32_t>(1U, max_points);
  if (point_count <= limit) {
    return 1U;
  }
  return std::max<uint64_t>(1U, (point_count + limit - 1U) / limit);
}

bool PointFinite(float x, float y, float z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

void CellForPoint(
    float x,
    float y,
    double origin_x,
    double origin_y,
    double resolution,
    int& col,
    int& row) {
  col = static_cast<int>(std::floor((static_cast<double>(x) - origin_x) / resolution));
  row = static_cast<int>(std::floor((static_cast<double>(y) - origin_y) / resolution));
}

bool InBounds(int row, int col, int size) {
  return row >= 0 && row < size && col >= 0 && col < size;
}

float SampleBilinear(
    const float* src,
    const LingtuMapsGridSpec& spec,
    double world_x,
    double world_y,
    float fill) {
  const double src_col = (world_x - spec.origin_x_m) / spec.resolution_m - 0.5;
  const double src_row = (world_y - spec.origin_y_m) / spec.resolution_m - 0.5;
  if (src_col < 0.0 || src_row < 0.0 ||
      src_col > static_cast<double>(spec.cols - 1) ||
      src_row > static_cast<double>(spec.rows - 1)) {
    return fill;
  }
  const int c0 = static_cast<int>(std::floor(src_col));
  const int r0 = static_cast<int>(std::floor(src_row));
  const int c1 = c0 + 1;
  const int r1 = r0 + 1;
  const double wc = src_col - static_cast<double>(c0);
  const double wr = src_row - static_cast<double>(r0);

  const auto value = [&](int row, int col) -> double {
    if (row < 0 || row >= spec.rows || col < 0 || col >= spec.cols) {
      return static_cast<double>(fill);
    }
    return static_cast<double>(src[static_cast<size_t>(row * spec.cols + col)]);
  };

  const double v00 = value(r0, c0);
  const double v01 = value(r0, c1);
  const double v10 = value(r1, c0);
  const double v11 = value(r1, c1);
  const double top = v00 * (1.0 - wc) + v01 * wc;
  const double bottom = v10 * (1.0 - wc) + v11 * wc;
  return static_cast<float>(top * (1.0 - wr) + bottom * wr);
}

void FillCounts(
    const std::vector<int16_t>& occupancy,
    LingtuMapsOccupancyCounts* out_counts) {
  if (out_counts == nullptr) {
    return;
  }
  out_counts->unknown = 0;
  out_counts->free = 0;
  out_counts->occupied = 0;
  for (const int16_t value : occupancy) {
    if (value < 0) {
      ++out_counts->unknown;
    } else if (value >= kOccupied) {
      ++out_counts->occupied;
    } else {
      ++out_counts->free;
    }
  }
}

void BuildProjectedOccupancy(
    const float* xyz,
    uint64_t point_count,
    int size,
    double origin_x,
    double origin_y,
    const OccupancyConfig& cfg,
    int8_t* out_occupancy,
    float* out_cost,
    LingtuMapsOccupancyCounts* out_counts) {
  std::vector<uint8_t> occupied(static_cast<size_t>(size * size), 0);
  const auto clear = RobotClearMask(size, origin_x, origin_y, cfg);

  for (uint64_t i = 0; i < point_count; ++i) {
    const float x = xyz[i * 3U + 0U];
    const float y = xyz[i * 3U + 1U];
    const float z = xyz[i * 3U + 2U];
    if (!PointFinite(x, y, z) || !(z > cfg.z_min_m && z < cfg.z_max_m)) {
      continue;
    }
    if (InsideRobotClear(x, y, cfg)) {
      continue;
    }
    int col = 0;
    int row = 0;
    CellForPoint(x, y, origin_x, origin_y, cfg.resolution_m, col, row);
    if (InBounds(row, col, size)) {
      occupied[static_cast<size_t>(CellIndex(row, col, size))] = 1;
    }
  }

  for (size_t i = 0; i < occupied.size(); ++i) {
    if (clear[i] != 0) {
      occupied[i] = 0;
    }
  }

  auto inflated = Dilate(occupied, size, cfg.inflation_radius_m, cfg.resolution_m);
  std::vector<int16_t> occupancy(static_cast<size_t>(size * size), kFree);
  for (size_t i = 0; i < occupancy.size(); ++i) {
    float cost = inflated[i] != 0 ? 100.0F : 0.0F;
    if (occupied[i] != 0) {
      cost = 100.0F;
    }
    if (clear[i] != 0) {
      cost = 0.0F;
    }
    out_cost[i] = cost;
    out_occupancy[i] = static_cast<int8_t>(cost);
    occupancy[i] = static_cast<int16_t>(out_occupancy[i]);
  }
  FillCounts(occupancy, out_counts);
}

void BuildRaycastOccupancy(
    const float* xyz,
    uint64_t point_count,
    int size,
    double origin_x,
    double origin_y,
    const OccupancyConfig& cfg,
    int8_t* out_occupancy,
    float* out_cost,
    LingtuMapsOccupancyCounts* out_counts) {
  std::vector<int16_t> grid(static_cast<size_t>(size * size), kUnknown);
  const auto clear = RobotClearMask(size, origin_x, origin_y, cfg);
  for (size_t i = 0; i < grid.size(); ++i) {
    if (clear[i] != 0) {
      grid[i] = kFree;
    }
  }

  int robot_col = 0;
  int robot_row = 0;
  CellForPoint(
      static_cast<float>(cfg.robot_x_m),
      static_cast<float>(cfg.robot_y_m),
      origin_x,
      origin_y,
      cfg.resolution_m,
      robot_col,
      robot_row);
  if (!InBounds(robot_row, robot_col, size)) {
    throw std::invalid_argument("robot origin is outside occupancy grid");
  }

  const uint64_t ray_step = DownsampleStep(point_count, cfg.raycast_max_rays);
  for (uint64_t i = 0; i < point_count; i += ray_step) {
    const float x = xyz[i * 3U + 0U];
    const float y = xyz[i * 3U + 1U];
    const float z = xyz[i * 3U + 2U];
    if (!PointFinite(x, y, z)) {
      continue;
    }
    int col = 0;
    int row = 0;
    CellForPoint(x, y, origin_x, origin_y, cfg.resolution_m, col, row);
    if (!InBounds(row, col, size)) {
      continue;
    }
    const auto cells = Bresenham(robot_col, robot_row, col, row);
    for (size_t ci = 0; ci + 1U < cells.size(); ++ci) {
      const int c = cells[ci].first;
      const int r = cells[ci].second;
      if (InBounds(r, c, size)) {
        grid[static_cast<size_t>(CellIndex(r, c, size))] = kFree;
      }
    }
  }

  if (cfg.raycast_free_inflation_radius_m > 0.0) {
    std::vector<uint8_t> free_mask(grid.size(), 0);
    for (size_t i = 0; i < grid.size(); ++i) {
      free_mask[i] = grid[i] == kFree ? 1 : 0;
    }
    auto widened_free = Dilate(
        free_mask,
        size,
        cfg.raycast_free_inflation_radius_m,
        cfg.resolution_m);
    for (size_t i = 0; i < grid.size(); ++i) {
      if (grid[i] == kUnknown && widened_free[i] != 0) {
        grid[i] = kFree;
      }
    }
  }

  const uint64_t occ_step = DownsampleStep(point_count, cfg.raycast_max_rays);
  for (uint64_t i = 0; i < point_count; i += occ_step) {
    const float x = xyz[i * 3U + 0U];
    const float y = xyz[i * 3U + 1U];
    const float z = xyz[i * 3U + 2U];
    if (!PointFinite(x, y, z) || !(z > cfg.z_min_m && z < cfg.z_max_m)) {
      continue;
    }
    if (InsideRobotClear(x, y, cfg)) {
      continue;
    }
    int col = 0;
    int row = 0;
    CellForPoint(x, y, origin_x, origin_y, cfg.resolution_m, col, row);
    if (InBounds(row, col, size)) {
      grid[static_cast<size_t>(CellIndex(row, col, size))] = kOccupied;
    }
  }

  std::vector<uint8_t> occ_mask(grid.size(), 0);
  for (size_t i = 0; i < grid.size(); ++i) {
    occ_mask[i] = grid[i] == kOccupied ? 1 : 0;
  }
  auto inflated = Dilate(occ_mask, size, cfg.inflation_radius_m, cfg.resolution_m);
  for (size_t i = 0; i < grid.size(); ++i) {
    if (inflated[i] != 0) {
      grid[i] = kOccupied;
    }
    if (clear[i] != 0) {
      grid[i] = kFree;
    }
  }

  for (size_t i = 0; i < grid.size(); ++i) {
    out_occupancy[i] = static_cast<int8_t>(grid[i]);
    if (grid[i] < 0) {
      out_cost[i] = cfg.unknown_as_obstacle_for_costmap ? 100.0F : 0.0F;
    } else {
      out_cost[i] = std::min(100.0F, std::max(0.0F, static_cast<float>(grid[i])));
    }
  }
  FillCounts(grid, out_counts);
}

}  // namespace

extern "C" {

int32_t lingtu_maps_build_elevation_map(
    const float* xyz,
    uint64_t point_count,
    double robot_x_m,
    double robot_y_m,
    double resolution_m,
    double radius_m,
    double z_floor_m,
    double z_ceil_m,
    LingtuMapsGridSpec* out_spec,
    float* out_min_z,
    float* out_max_z,
    float* out_clearance,
    uint8_t* out_valid,
    uint64_t capacity) {
  if ((xyz == nullptr && point_count > 0U) ||
      out_spec == nullptr ||
      out_min_z == nullptr ||
      out_max_z == nullptr ||
      out_clearance == nullptr ||
      out_valid == nullptr) {
    return -1;
  }

  return Protect([&]() {
    std::vector<float> points;
    points.assign(xyz, xyz + point_count * 3U);
    const auto result = buildElevationMap(
        points,
        robot_x_m,
        robot_y_m,
        resolution_m,
        radius_m,
        z_floor_m,
        z_ceil_m);
    FillSpec(result.maxZ, out_spec);
    const uint64_t n = static_cast<uint64_t>(result.maxZ.rows) *
        static_cast<uint64_t>(result.maxZ.cols);
    if (capacity < n) {
      throw std::length_error("elevation output capacity is too small");
    }
    CopyVector(result.minZ.data, out_min_z);
    CopyVector(result.maxZ.data, out_max_z);
    CopyVector(result.clearance.data, out_clearance);
    CopyVector(result.valid, out_valid);
  });
}

int32_t lingtu_maps_compute_esdf(
    const float* occupancy,
    const LingtuMapsGridSpec* spec,
    float obstacle_threshold,
    float* out_distance,
    float* out_grad_x,
    float* out_grad_y,
    uint64_t capacity) {
  if (occupancy == nullptr ||
      !ValidSpec(spec) ||
      out_distance == nullptr ||
      out_grad_x == nullptr ||
      out_grad_y == nullptr) {
    return -1;
  }
  if (capacity < CellCount(*spec)) {
    return -4;
  }
  return Protect([&]() {
    const auto result = computeEsdf(GridFromArray(occupancy, *spec), obstacle_threshold);
    CopyVector(result.distance.data, out_distance);
    CopyVector(result.gradX.data, out_grad_x);
    CopyVector(result.gradY.data, out_grad_y);
  });
}

int32_t lingtu_maps_compute_terrain_risk(
    const float* min_z,
    const float* max_z,
    const float* clearance,
    const uint8_t* valid,
    const LingtuMapsGridSpec* spec,
    const LingtuMapsTerrainRiskParams* params,
    float* out_risk,
    float* out_slope_deg,
    float* out_step_height,
    float* out_roughness,
    uint64_t capacity) {
  if (min_z == nullptr ||
      max_z == nullptr ||
      clearance == nullptr ||
      valid == nullptr ||
      !ValidSpec(spec) ||
      out_risk == nullptr ||
      out_slope_deg == nullptr ||
      out_step_height == nullptr ||
      out_roughness == nullptr) {
    return -1;
  }
  const uint64_t n = CellCount(*spec);
  if (capacity < n) {
    return -4;
  }
  return Protect([&]() {
    ElevationMapResult elevation;
    elevation.minZ = GridFromArray(min_z, *spec);
    elevation.maxZ = GridFromArray(max_z, *spec);
    elevation.clearance = GridFromArray(clearance, *spec);
    elevation.valid.assign(valid, valid + n);
    const auto result = computeTerrainRisk(elevation, ToTerrainParams(params));
    CopyVector(result.risk.data, out_risk);
    CopyVector(result.slopeDeg.data, out_slope_deg);
    CopyVector(result.stepHeight.data, out_step_height);
    CopyVector(result.roughness.data, out_roughness);
  });
}

int32_t lingtu_maps_fuse_traversability_cost(
    const float* cost,
    const float* slope_deg,
    const float* esdf_distance,
    const float* terrain_risk,
    const LingtuMapsGridSpec* spec,
    const LingtuMapsTraversabilityParams* params,
    float* out_cost,
    uint64_t capacity) {
  if (cost == nullptr || !ValidSpec(spec) || out_cost == nullptr) {
    return -1;
  }
  if (capacity < CellCount(*spec)) {
    return -4;
  }
  return Protect([&]() {
    const auto result = fuseTraversabilityCost(
        GridFromArray(cost, *spec),
        OptionalGridFromArray(slope_deg, *spec),
        OptionalGridFromArray(esdf_distance, *spec),
        OptionalGridFromArray(terrain_risk, *spec),
        ToTraversabilityParams(params));
    CopyVector(result.data, out_cost);
  });
}

int32_t lingtu_maps_build_occupancy_grid(
    const float* xyz,
    uint64_t point_count,
    double robot_x_m,
    double robot_y_m,
    double robot_yaw_rad,
    double resolution_m,
    double radius_m,
    double z_min_m,
    double z_max_m,
    double inflation_radius_m,
    double robot_clear_radius_m,
    double robot_clear_forward_m,
    double robot_clear_backward_m,
    double robot_clear_lateral_m,
    uint8_t raycast_free_space,
    uint8_t unknown_as_obstacle_for_costmap,
    uint32_t raycast_max_rays,
    double raycast_free_inflation_radius_m,
    LingtuMapsGridSpec* out_spec,
    int8_t* out_occupancy,
    float* out_cost,
    LingtuMapsOccupancyCounts* out_counts,
    uint64_t capacity) {
  if ((xyz == nullptr && point_count > 0U) ||
      out_spec == nullptr ||
      out_occupancy == nullptr ||
      out_cost == nullptr ||
      out_counts == nullptr) {
    return -1;
  }
  return Protect([&]() {
    OccupancyConfig cfg;
    cfg.robot_x_m = robot_x_m;
    cfg.robot_y_m = robot_y_m;
    cfg.robot_yaw_rad = robot_yaw_rad;
    cfg.resolution_m = resolution_m;
    cfg.radius_m = radius_m;
    cfg.z_min_m = z_min_m;
    cfg.z_max_m = z_max_m;
    cfg.inflation_radius_m = std::max(0.0, inflation_radius_m);
    cfg.robot_clear_radius_m = std::max(0.0, robot_clear_radius_m);
    cfg.robot_clear_forward_m = std::max(0.0, robot_clear_forward_m);
    cfg.robot_clear_backward_m = std::max(0.0, robot_clear_backward_m);
    cfg.robot_clear_lateral_m = std::max(0.0, robot_clear_lateral_m);
    cfg.raycast_free_space = raycast_free_space != 0U;
    cfg.unknown_as_obstacle_for_costmap = unknown_as_obstacle_for_costmap != 0U;
    cfg.raycast_max_rays = std::max<uint32_t>(1U, raycast_max_rays);
    cfg.raycast_free_inflation_radius_m = std::max(0.0, raycast_free_inflation_radius_m);

    const int size = GridSize(cfg);
    const uint64_t cells = static_cast<uint64_t>(size) * static_cast<uint64_t>(size);
    if (capacity < cells) {
      throw std::length_error("occupancy output capacity is too small");
    }
    out_spec->rows = size;
    out_spec->cols = size;
    out_spec->resolution_m = cfg.resolution_m;
    out_spec->origin_x_m = cfg.robot_x_m - cfg.radius_m;
    out_spec->origin_y_m = cfg.robot_y_m - cfg.radius_m;

    if (cfg.raycast_free_space) {
      BuildRaycastOccupancy(
          xyz,
          point_count,
          size,
          out_spec->origin_x_m,
          out_spec->origin_y_m,
          cfg,
          out_occupancy,
          out_cost,
          out_counts);
    } else {
      BuildProjectedOccupancy(
          xyz,
          point_count,
          size,
          out_spec->origin_x_m,
          out_spec->origin_y_m,
          cfg,
          out_occupancy,
          out_cost,
          out_counts);
    }
  });
}

int32_t lingtu_maps_resample_grid_bilinear(
    const float* src,
    const LingtuMapsGridSpec* src_spec,
    int32_t dst_rows,
    int32_t dst_cols,
    double dst_resolution_m,
    double dst_origin_x_m,
    double dst_origin_y_m,
    float fill,
    float* out,
    uint64_t capacity) {
  if (src == nullptr ||
      !ValidSpec(src_spec) ||
      dst_rows <= 0 ||
      dst_cols <= 0 ||
      !(dst_resolution_m > 0.0) ||
      out == nullptr) {
    return -1;
  }
  const uint64_t cells = static_cast<uint64_t>(dst_rows) * static_cast<uint64_t>(dst_cols);
  if (capacity < cells) {
    return -4;
  }
  return Protect([&]() {
    for (int row = 0; row < dst_rows; ++row) {
      const double y = dst_origin_y_m + (static_cast<double>(row) + 0.5) * dst_resolution_m;
      for (int col = 0; col < dst_cols; ++col) {
        const double x = dst_origin_x_m + (static_cast<double>(col) + 0.5) * dst_resolution_m;
        out[static_cast<size_t>(row * dst_cols + col)] =
            SampleBilinear(src, *src_spec, x, y, fill);
      }
    }
  });
}

}  // extern "C"
