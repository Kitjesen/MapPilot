#include "lingtu/maps/build/import_check.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <sstream>
#include <unordered_set>

#include "lingtu/maps/json.hpp"

namespace lingtu::maps {
namespace {

constexpr double kMaxFloorTiltDeg = 3.0;
constexpr double kMinFloorShare = 0.05;
constexpr double kMinFloorFill = 0.5;
constexpr double kMinExtentXyM = 1.0;
constexpr double kMaxExtentXyM = 2000.0;
constexpr double kMaxExtentZM = 100.0;
// Floor flatness, independent of the map resolution: a looser band would let a
// horizontal slice through walls outnumber the floor.
constexpr double kFloorToleranceM = 0.05;
constexpr std::size_t kSampleSize = 20000;
constexpr int kRansacIterations = 400;
constexpr double kPi = 3.14159265358979323846;

double Percentile(std::vector<double> values, double q) {
  const auto index = static_cast<std::size_t>(q * static_cast<double>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index),
                   values.end());
  return values[index];
}

// Least-squares plane z = a*x + b*y + c through the given points.
bool FitHeightPlane(const std::vector<PointXyz>& points, std::array<double, 3>* plane) {
  double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0, sxz = 0, syz = 0, sz = 0;
  const double n = static_cast<double>(points.size());
  for (const auto& p : points) {
    sxx += p.x * p.x; sxy += p.x * p.y; sx += p.x; syy += p.y * p.y; sy += p.y;
    sxz += p.x * p.z; syz += p.y * p.z; sz += p.z;
  }
  // Cramer's rule on [sxx sxy sx; sxy syy sy; sx sy n] [a b c]' = [sxz syz sz]'.
  const auto det3 = [](double a, double b, double c, double d, double e, double f, double g,
                       double h, double i) {
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  };
  const double det = det3(sxx, sxy, sx, sxy, syy, sy, sx, sy, n);
  if (std::abs(det) < 1e-9) return false;
  (*plane)[0] = det3(sxz, sxy, sx, syz, syy, sy, sz, sy, n) / det;
  (*plane)[1] = det3(sxx, sxz, sx, sxy, syz, sy, sx, sz, n) / det;
  (*plane)[2] = det3(sxx, sxy, sxz, sxy, syy, syz, sx, sy, sz) / det;
  return true;
}

std::string Format(double value) {
  std::ostringstream out;
  out.precision(3);
  out << value;
  return out.str();
}

}  // namespace

PointCloudNavigationCheck CheckPointCloudForNavigation(const std::vector<PointXyz>& points,
                                                       double resolution) {
  PointCloudNavigationCheck check;
  if (points.size() < 3U) {
    check.blockers.push_back("point cloud has fewer than 3 points");
    return check;
  }
  const std::size_t stride = std::max<std::size_t>(1U, points.size() / kSampleSize);
  std::vector<PointXyz> sample;
  for (std::size_t i = 0; i < points.size(); i += stride) sample.push_back(points[i]);

  std::vector<double> xs, ys, zs;
  for (const auto& p : sample) { xs.push_back(p.x); ys.push_back(p.y); zs.push_back(p.z); }
  check.extent_xy_m = std::max(Percentile(xs, 0.99) - Percentile(xs, 0.01),
                               Percentile(ys, 0.99) - Percentile(ys, 0.01));
  check.extent_z_m = Percentile(zs, 0.99) - Percentile(zs, 0.01);
  if (check.extent_xy_m < kMinExtentXyM || check.extent_xy_m > kMaxExtentXyM) {
    check.blockers.push_back("map spans " + Format(check.extent_xy_m) +
                             " in x/y; expected 1 to 2000 m, check the units");
  }
  if (check.extent_z_m > kMaxExtentZM) {
    check.blockers.push_back("map spans " + Format(check.extent_z_m) +
                             " in z; expected at most 100 m, check the units and the up axis");
  }

  // RANSAC over near-horizontal planes (normal within 15 degrees of z). The
  // lowest sufficiently supported plane is treated as the floor, while a
  // narrow wall slice is rejected because it has no 2-D support footprint.
  const double tolerance = kFloorToleranceM;
  const double min_normal_z = std::cos(15.0 * kPi / 180.0);
  std::mt19937 random(7U);
  std::uniform_int_distribution<std::size_t> pick(0U, sample.size() - 1U);
  const double reference_x = Percentile(xs, 0.5);
  const double reference_y = Percentile(ys, 0.5);
  const double minimum_observed_z = Percentile(zs, 0.01);
  const std::size_t min_floor_inliers = std::max<std::size_t>(
      3U, static_cast<std::size_t>(
              std::ceil(kMinFloorShare * static_cast<double>(sample.size()))));
  std::size_t best_inliers = 0;
  double best_height = std::numeric_limits<double>::infinity();
  std::array<double, 4> best{};
  for (int iteration = 0; iteration < kRansacIterations; ++iteration) {
    const auto& p0 = sample[pick(random)];
    const auto& p1 = sample[pick(random)];
    const auto& p2 = sample[pick(random)];
    const double ux = p1.x - p0.x, uy = p1.y - p0.y, uz = p1.z - p0.z;
    const double vx = p2.x - p0.x, vy = p2.y - p0.y, vz = p2.z - p0.z;
    double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const double length = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (length < 1e-9) continue;
    nx /= length; ny /= length; nz /= length;
    if (std::abs(nz) < min_normal_z) continue;
    const double d = -(nx * p0.x + ny * p0.y + nz * p0.z);
    std::size_t inliers = 0;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& p : sample) {
      if (std::abs(nx * p.x + ny * p.y + nz * p.z + d) <= tolerance) {
        ++inliers;
        min_x = std::min(min_x, static_cast<double>(p.x));
        max_x = std::max(max_x, static_cast<double>(p.x));
        min_y = std::min(min_y, static_cast<double>(p.y));
        max_y = std::max(max_y, static_cast<double>(p.y));
      }
    }
    const double reference_height =
        -(nx * reference_x + ny * reference_y + d) / nz;
    const double min_floor_span = std::max(0.20, resolution * 4.0);
    const bool has_area_support =
        inliers >= min_floor_inliers && max_x - min_x >= min_floor_span &&
        max_y - min_y >= min_floor_span &&
        reference_height + tolerance >= minimum_observed_z;
    if (has_area_support &&
        (reference_height < best_height - 1e-6 ||
         (std::abs(reference_height - best_height) <= 1e-6 &&
          inliers > best_inliers))) {
      best_inliers = inliers;
      best_height = reference_height;
      best = {nx, ny, nz, d};
    }
  }
  check.floor_share = static_cast<double>(best_inliers) / static_cast<double>(sample.size());
  if (check.floor_share < kMinFloorShare) {
    check.blockers.push_back("no level floor plane found; z must point up");
    return check;
  }

  std::vector<PointXyz> floor_sample;
  for (const auto& p : sample) {
    if (std::abs(best[0] * p.x + best[1] * p.y + best[2] * p.z + best[3]) <= tolerance)
      floor_sample.push_back(p);
  }
  std::array<double, 3> plane{};
  if (!FitHeightPlane(floor_sample, &plane)) {
    check.blockers.push_back("floor plane is degenerate");
    return check;
  }
  check.floor_tilt_deg = std::atan(std::hypot(plane[0], plane[1])) * 180.0 / kPi;
  if (check.floor_tilt_deg > kMaxFloorTiltDeg) {
    check.blockers.push_back("floor is tilted " + Format(check.floor_tilt_deg) +
                             " degrees; level the map so z points up");
  }

  // Floor density: cells of the floor at the map resolution whose neighbours
  // are mostly present. Points spaced wider than a cell leave isolated cells.
  std::unordered_set<std::uint64_t> cells;
  const auto cell_key = [](std::int64_t x, std::int64_t y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32U) |
           static_cast<std::uint32_t>(y);
  };
  const double vertical_tolerance = tolerance * std::sqrt(1.0 + plane[0] * plane[0] + plane[1] * plane[1]);
  for (const auto& p : points) {
    if (std::abs(p.z - (plane[0] * p.x + plane[1] * p.y + plane[2])) > vertical_tolerance) continue;
    cells.insert(cell_key(static_cast<std::int64_t>(std::floor(p.x / resolution)),
                          static_cast<std::int64_t>(std::floor(p.y / resolution))));
  }
  std::size_t interior = 0;
  for (const auto key : cells) {
    const std::int64_t x = static_cast<std::int32_t>(key >> 32U);
    const std::int64_t y = static_cast<std::int32_t>(key & 0xffffffffU);
    int neighbours = 0;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        if ((dx != 0 || dy != 0) && cells.count(cell_key(x + dx, y + dy)) != 0U) ++neighbours;
    if (neighbours >= 6) ++interior;
  }
  check.floor_fill = cells.empty() ? 0.0
                                   : static_cast<double>(interior) / static_cast<double>(cells.size());
  if (check.floor_fill < kMinFloorFill) {
    check.blockers.push_back("floor is too sparse for " + Format(resolution) +
                             " m cells; import a denser cloud or build at a coarser resolution");
  }
  return check;
}

std::string PointCloudNavigationCheckJson(const PointCloudNavigationCheck& check) {
  std::ostringstream out;
  out << "{\"ok\":" << (check.ok() ? "true" : "false")
      << ",\"floor_tilt_deg\":" << check.floor_tilt_deg
      << ",\"floor_share\":" << check.floor_share
      << ",\"floor_fill\":" << check.floor_fill
      << ",\"extent_xy_m\":" << check.extent_xy_m
      << ",\"extent_z_m\":" << check.extent_z_m << ",\"blockers\":[";
  for (std::size_t i = 0; i < check.blockers.size(); ++i) {
    out << (i == 0U ? "" : ",") << JsonString(check.blockers[i]);
  }
  out << "]}";
  return out.str();
}

}  // namespace lingtu::maps
