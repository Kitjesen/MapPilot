#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "lingtu/maps/layers/grid.hpp"

namespace lingtu::maps::layers {

// Geometric surface evidence, not a declaration of traversability. Callers own
// robot-relative ground seeding, connectivity and collision policy.
struct GroundSurfaceConfig {
  double sample_resolution_m{0.05};
  int min_support_columns{3};
  double min_cluster_fraction{0.35};
  double cluster_height_m{0.12};
  double max_slope_deg{35.0};
  double max_residual_m{0.04};
  int neighbor_radius_cells{1};
};

struct GroundSurfaceResult {
  Grid2D height;
  Grid2D gradient_x;
  Grid2D gradient_y;
  Grid2D slope_deg;
  Grid2D roughness_m;
  // Spatial fit residual variance, not posterior localization uncertainty.
  Grid2D variance_m2;
  Grid2D support_count;
};

// Keep complete world-aligned cells inside the observed rolling window.
inline Grid2D GroundSurfaceGeometry(const Grid2D& source, double resolution = 0.20) {
  if (source.empty()) return {};
  const double x = std::ceil(source.originX/resolution-1e-7)*resolution;
  const double y = std::ceil(source.originY/resolution-1e-7)*resolution;
  const int cols = std::max(0, static_cast<int>(std::floor(
      (source.originX+source.cols*source.resolution-x)/resolution+1e-7)));
  const int rows = std::max(0, static_cast<int>(std::floor(
      (source.originY+source.rows*source.resolution-y)/resolution+1e-7)));
  return makeGrid2D(rows, cols, resolution, x, y,
      std::numeric_limits<float>::quiet_NaN());
}

inline GroundSurfaceResult EstimateGroundSurface(
    const std::vector<float>& xyz, const Grid2D& geometry,
    const GroundSurfaceConfig& config = {});

namespace ground_detail {
struct Point {
  double x, y, z;
  int column_x, column_y;
  // Within-column covariance XX, YY, ZZ, XY, XZ, YZ. One support vote must not
  // erase the roughness represented by multiple returns at that location.
  std::array<double, 6> covariance{};
};

struct Plane {
  double x{0}, y{0}, z{0}, rms{0};
  bool valid{false};
  double At(double px, double py) const { return x * px + y * py + z; }
};

inline double WithinResidualVariance(const Point& p, const Plane& plane) {
  const auto& v = p.covariance;
  return std::max(0.0, v[2]+plane.x*plane.x*v[0]+plane.y*plane.y*v[1]
      +2*plane.x*plane.y*v[3]-2*plane.x*v[4]-2*plane.y*v[5]);
}

// The points are relative to a cell center, avoiding ill-conditioned fits in
// large map coordinates. XY covariance must span an area, not just a line.
inline Plane Fit(const std::vector<Point>& points) {
  Plane plane;
  if (points.size() < 3U) return plane;
  double mx = 0, my = 0, mz = 0;
  for (const auto& p : points) { mx += p.x; my += p.y; mz += p.z; }
  const double n = static_cast<double>(points.size());
  mx /= n; my /= n; mz /= n;
  double xx = 0, yy = 0, xy = 0, xz = 0, yz = 0;
  for (const auto& p : points) {
    const double x = p.x - mx, y = p.y - my, z = p.z - mz;
    xx += x*x; yy += y*y; xy += x*y; xz += x*z; yz += y*z;
  }
  const double eigen_min = (xx + yy - std::hypot(xx - yy, 2*xy)) / (2*n);
  if (eigen_min < 1e-5) return plane;
  const double determinant = xx*yy - xy*xy;
  plane.x = (xz*yy - yz*xy) / determinant;
  plane.y = (yz*xx - xz*xy) / determinant;
  plane.z = mz - plane.x*mx - plane.y*my;
  double residual = 0;
  for (const auto& p : points) {
    const double d = p.z - plane.At(p.x, p.y);
    residual += d*d + WithinResidualVariance(p, plane);
  }
  plane.rms = std::sqrt(residual/n);
  plane.valid = true;
  return plane;
}

inline bool SameColumn(const Point& a, const Point& b) {
  return a.column_x == b.column_x && a.column_y == b.column_y;
}

// A height cluster gets one vote per observed XY column. Repeated returns or a
// vertical wall cannot outvote the surrounding surface just by point density.
inline std::vector<Point> LowestSupportedCluster(
    std::vector<Point> points, const GroundSurfaceConfig& config) {
  std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
    return a.z < b.z;
  });
  std::vector<Point> columns;
  for (const auto& p : points) {
    if (std::none_of(columns.begin(), columns.end(), [&](const Point& q) {
      return SameColumn(p, q);
    })) columns.push_back(p);
  }
  const auto required = static_cast<std::size_t>(std::max(config.min_support_columns,
      static_cast<int>(std::ceil(config.min_cluster_fraction*columns.size()))));
  if (columns.size() < required) return {};
  for (std::size_t begin = 0; begin < points.size(); ++begin) {
    columns.clear();
    std::vector<int> counts;
    for (std::size_t end = begin; end < points.size() &&
         points[end].z - points[begin].z <= config.cluster_height_m; ++end) {
      const auto& p = points[end];
      auto found = std::find_if(columns.begin(), columns.end(), [&](const Point& q) {
        return SameColumn(p, q);
      });
      if (found == columns.end()) {
        columns.push_back(p); counts.push_back(1);
      } else {
        const int n = ++counts[static_cast<std::size_t>(found-columns.begin())];
        const double dx = p.x-found->x, dy = p.y-found->y, dz = p.z-found->z;
        found->x += (p.x-found->x)/n;
        found->y += (p.y-found->y)/n;
        found->z += (p.z-found->z)/n;
        found->covariance[0] += dx*(p.x-found->x);
        found->covariance[1] += dy*(p.y-found->y);
        found->covariance[2] += dz*(p.z-found->z);
        found->covariance[3] += dx*(p.y-found->y);
        found->covariance[4] += dx*(p.z-found->z);
        found->covariance[5] += dy*(p.z-found->z);
      }
    }
    if (columns.size() < required) continue;
    for (std::size_t i = 0; i < columns.size(); ++i)
      for (auto& covariance : columns[i].covariance) covariance /= counts[i];
    const auto plane = Fit(columns);
    if (plane.valid && plane.rms <= config.max_residual_m &&
        std::hypot(plane.x,plane.y) <= std::tan(config.max_slope_deg*M_PI/180.0)) return columns;
  }
  return {};
}
}  // namespace ground_detail

inline GroundSurfaceResult EstimateGroundSurface(
    const std::vector<float>& xyz, const Grid2D& geometry,
    const GroundSurfaceConfig& config) {
  geometry.validate("ground surface");
  if (xyz.size()%3U != 0U || !(config.sample_resolution_m > 0) ||
      config.min_support_columns < 3 || !(config.cluster_height_m > 0) ||
      !(config.max_residual_m > 0) || !(config.min_cluster_fraction > 0) ||
      config.min_cluster_fraction > 1 || config.neighbor_radius_cells < 0 ||
      config.neighbor_radius_cells > 3 || !(config.max_slope_deg > 0) ||
      config.max_slope_deg >= 90) throw std::invalid_argument("invalid ground surface input");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto blank = [&] (float fill) { return makeGrid2D(geometry.rows, geometry.cols,
      geometry.resolution, geometry.originX, geometry.originY, fill); };
  GroundSurfaceResult result{blank(nan), blank(nan), blank(nan), blank(nan),
      blank(nan), blank(nan), blank(0)};
  if (geometry.empty()) return result;
  using ground_detail::Point;
  using ground_detail::Plane;
  std::vector<std::vector<Point>> cells(geometry.data.size());
  for (std::size_t i = 0; i < xyz.size(); i += 3U) {
    const double x = xyz[i], y = xyz[i+1], z = xyz[i+2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const double gx = (x-geometry.originX)/geometry.resolution;
    const double gy = (y-geometry.originY)/geometry.resolution;
    if (gx < 0 || gy < 0 || gx >= geometry.cols || gy >= geometry.rows) continue;
    const int c = static_cast<int>(std::floor(gx)), r = static_cast<int>(std::floor(gy));
    cells[geometry.index(r,c)].push_back({x-(geometry.originX+(c+.5)*geometry.resolution),
        y-(geometry.originY+(r+.5)*geometry.resolution), z,
        static_cast<int>(std::floor((x-geometry.originX)/config.sample_resolution_m)),
        static_cast<int>(std::floor((y-geometry.originY)/config.sample_resolution_m))});
  }
  std::vector<Plane> initial(cells.size());
  const double slope_limit = std::tan(config.max_slope_deg*M_PI/180.0);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    cells[i] = ground_detail::LowestSupportedCluster(std::move(cells[i]), config);
    initial[i] = ground_detail::Fit(cells[i]);
    if (initial[i].rms > config.max_residual_m ||
        std::hypot(initial[i].x, initial[i].y) > slope_limit) initial[i].valid = false;
  }
  for (int r = 0; r < geometry.rows; ++r) for (int c = 0; c < geometry.cols; ++c) {
    const int index = geometry.index(r,c);
    if (!initial[index].valid) continue;
    std::vector<Point> neighborhood;
    const int radius = config.neighbor_radius_cells;
    for (int nr = std::max(0,r-radius); nr <= std::min(geometry.rows-1,r+radius); ++nr)
      for (int nc = std::max(0,c-radius); nc <= std::min(geometry.cols-1,c+radius); ++nc) {
        const int neighbor = geometry.index(nr,nc);
        if (!initial[neighbor].valid) continue;
        const double dx = (nc-c)*geometry.resolution, dy = (nr-r)*geometry.resolution;
        // Do not average opposite sides of a ledge into a fictitious ramp.
        if (std::abs(initial[index].At(dx,dy)-initial[neighbor].z) > config.max_residual_m ||
            std::abs(initial[neighbor].At(-dx,-dy)-initial[index].z) > config.max_residual_m) continue;
        for (auto p : cells[neighbor]) {
          p.x += dx; p.y += dy;
          if (std::abs(p.z-initial[index].At(p.x,p.y)) <= 2*config.max_residual_m)
            neighborhood.push_back(p);
        }
      }
    Plane plane = initial[index];
    for (int iteration = 0; iteration < 2; ++iteration) {
      std::vector<Point> inliers;
      for (const auto& p : neighborhood)
        if (std::abs(p.z-plane.At(p.x,p.y)) <= config.max_residual_m) inliers.push_back(p);
      auto fitted = ground_detail::Fit(inliers);
      if (!fitted.valid) { plane.valid = false; break; }
      plane = fitted;
    }
    if (!plane.valid || plane.rms > config.max_residual_m ||
        std::hypot(plane.x,plane.y) > slope_limit) continue;
    int support = 0;
    double sum_residual = 0;
    for (const auto& p : cells[index]) {
      const double d = p.z-plane.At(p.x,p.y);
      if (std::abs(d) <= config.max_residual_m) {
        ++support;
        sum_residual += d*d + ground_detail::WithinResidualVariance(p, plane);
      }
    }
    // Neighbor predictions can refine observed ground, never create support in a hole.
    if (support < config.min_support_columns || support*2 < static_cast<int>(cells[index].size())) continue;
    if (sum_residual/support > config.max_residual_m*config.max_residual_m) continue;
    result.height.data[index] = static_cast<float>(plane.z);
    result.gradient_x.data[index] = static_cast<float>(plane.x);
    result.gradient_y.data[index] = static_cast<float>(plane.y);
    result.slope_deg.data[index] = static_cast<float>(std::atan(std::hypot(plane.x,plane.y))*180/M_PI);
    result.variance_m2.data[index] = static_cast<float>(sum_residual/support);
    result.roughness_m.data[index] = std::sqrt(result.variance_m2.data[index]);
    result.support_count.data[index] = static_cast<float>(support);
  }
  return result;
}

}  // namespace lingtu::maps::layers
