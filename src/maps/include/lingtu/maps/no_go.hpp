#pragma once

#include <array>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::maps {

inline constexpr char kNoGoFilename[] = "no_go.json";

struct NoGoRegion {
  std::string id;
  std::array<double, 3> min;
  std::array<double, 3> max;
  std::string label;
  std::string kind{"no_go"};
};

void ValidateNoGoRegion(const NoGoRegion& region);
std::vector<NoGoRegion> ReadNoGoRegions(const std::filesystem::path& map_dir);
std::string NoGoRegionsJson(const std::vector<NoGoRegion>& regions);

inline bool IntersectsNoGo(const NoGoRegion& region,
                          const std::array<double, 3>& min,
                          const std::array<double, 3>& max) {
  for (int axis = 0; axis < 3; ++axis)
    if (max[axis] < region.min[axis] || min[axis] > region.max[axis]) return false;
  return true;
}

inline bool SegmentIntersectsNoGo(const NoGoRegion& region,
                                 const std::array<double, 3>& from,
                                 const std::array<double, 3>& to,
                                 const std::array<double, 3>& margin = {}) {
  double enter = 0, leave = 1;
  for (int axis = 0; axis < 3; ++axis) {
    const double minimum = region.min[axis] - margin[axis];
    const double maximum = region.max[axis] + margin[axis];
    const double delta = to[axis] - from[axis];
    if (delta == 0) {
      if (from[axis] < minimum || from[axis] > maximum) return false;
      continue;
    }
    double first = (minimum - from[axis]) / delta;
    double last = (maximum - from[axis]) / delta;
    if (first > last) { const double swap = first; first = last; last = swap; }
    if (first > enter) enter = first;
    if (last < leave) leave = last;
    if (enter > leave) return false;
  }
  return true;
}

inline bool SweptCylinderIntersectsNoGo(const NoGoRegion& region,
    const std::array<double, 3>& from, const std::array<double, 3>& to,
    double radius, double below, double above) {
  double enter = 0, leave = 1;
  const double dz = to[2] - from[2];
  if (dz == 0) {
    if (from[2] + above < region.min[2] || from[2] - below > region.max[2]) return false;
  } else {
    double first = (region.min[2] - above - from[2]) / dz;
    double last = (region.max[2] + below - from[2]) / dz;
    if (first > last) std::swap(first, last);
    enter = std::max(enter, first);
    leave = std::min(leave, last);
    if (enter > leave) return false;
  }
  const std::array<double, 3> a{from[0] + enter * (to[0] - from[0]),
      from[1] + enter * (to[1] - from[1]), 0};
  const std::array<double, 3> b{from[0] + leave * (to[0] - from[0]),
      from[1] + leave * (to[1] - from[1]), 0};
  const NoGoRegion flat{{}, {region.min[0], region.min[1], -1},
      {region.max[0], region.max[1], 1}};
  if (SegmentIntersectsNoGo(flat, a, b)) return true;
  const double radius2 = radius * radius;
  for (const auto& p : {a, b}) {
    const double dx = p[0] - std::clamp(p[0], region.min[0], region.max[0]);
    const double dy = p[1] - std::clamp(p[1], region.min[1], region.max[1]);
    if (dx * dx + dy * dy <= radius2) return true;
  }
  const double dx = b[0] - a[0], dy = b[1] - a[1];
  const double length2 = dx * dx + dy * dy;
  for (const double x : {region.min[0], region.max[0]})
    for (const double y : {region.min[1], region.max[1]}) {
      const double t = length2 > 0 ? std::clamp(((x-a[0])*dx + (y-a[1])*dy) / length2, 0.0, 1.0) : 0;
      const double ex = x - a[0] - t * dx, ey = y - a[1] - t * dy;
      if (ex * ex + ey * ey <= radius2) return true;
    }
  return false;
}

}  // namespace lingtu::maps
