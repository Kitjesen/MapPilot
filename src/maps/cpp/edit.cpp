#include "lingtu/maps/edit.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace lingtu::maps {
namespace {

using Selection = std::vector<std::size_t>;

double Coordinate(const PointXyz& point, int axis) {
  return axis == 0 ? point.x : axis == 1 ? point.y : point.z;
}

double DistanceSquared(const PointXyz& a, const PointXyz& b) {
  const double x = static_cast<double>(a.x) - b.x;
  const double y = static_cast<double>(a.y) - b.y;
  const double z = static_cast<double>(a.z) - b.z;
  return x*x + y*y + z*z;
}

bool InBounds(const PointXyz& point, const PcdBounds& bounds) {
  return point.x >= bounds.min_x && point.x <= bounds.max_x &&
         point.y >= bounds.min_y && point.y <= bounds.max_y &&
         point.z >= bounds.min_z && point.z <= bounds.max_z;
}

class PointIndex {
 public:
  PointIndex(const std::vector<PointXyz>& points, const Selection& selected)
      : points_(points), selected_(selected), order_(selected.size()), axis_(selected.size()) {
    std::iota(order_.begin(), order_.end(), std::size_t{0});
    Build(0, order_.size());
  }

  double MeanDistance(std::size_t query, std::size_t count) const {
    std::priority_queue<double> distances;
    Nearest(query, count, 0, order_.size(), distances);
    double sum = 0;
    const auto size = distances.size();
    while (!distances.empty()) {
      sum += std::sqrt(distances.top());
      distances.pop();
    }
    return sum / static_cast<double>(size);
  }

  Selection WithinRadius(std::size_t query, double radius,
                         std::size_t limit = std::numeric_limits<std::size_t>::max()) const {
    Selection result;
    Radius(query, radius*radius, limit, 0, order_.size(), result);
    return result;
  }

 private:
  const PointXyz& Point(std::size_t index) const {
    return points_[selected_[index]];
  }

  void Build(std::size_t first, std::size_t last) {
    if (first == last) return;
    const auto middle = first + (last - first) / 2;
    const auto& point = Point(order_[first]);
    std::array<double, 3> minimum{point.x, point.y, point.z};
    auto maximum = minimum;
    for (std::size_t i = first + 1; i < last; ++i) {
      for (int axis = 0; axis < 3; ++axis) {
        const double value = Coordinate(Point(order_[i]), axis);
        minimum[axis] = std::min(minimum[axis], value);
        maximum[axis] = std::max(maximum[axis], value);
      }
    }
    int axis = 0;
    for (int candidate = 1; candidate < 3; ++candidate)
      if (maximum[candidate] - minimum[candidate] > maximum[axis] - minimum[axis])
        axis = candidate;
    axis_[middle] = static_cast<std::uint8_t>(axis);
    std::nth_element(order_.begin() + first, order_.begin() + middle,
        order_.begin() + last, [&](std::size_t a, std::size_t b) {
          const double left = Coordinate(Point(a), axis);
          const double right = Coordinate(Point(b), axis);
          return left == right ? a < b : left < right;
        });
    Build(first, middle);
    Build(middle + 1, last);
  }

  void Nearest(std::size_t query, std::size_t count, std::size_t first,
               std::size_t last, std::priority_queue<double>& found) const {
    if (first == last) return;
    const auto middle = first + (last - first) / 2;
    const auto index = order_[middle];
    const auto& point = Point(index);
    if (index != query) {
      const double distance = DistanceSquared(Point(query), point);
      if (found.size() < count) found.push(distance);
      else if (distance < found.top()) {
        found.pop();
        found.push(distance);
      }
    }
    const double delta = Coordinate(Point(query), axis_[middle]) - Coordinate(point, axis_[middle]);
    const bool left = delta < 0;
    Nearest(query, count, left ? first : middle + 1,
            left ? middle : last, found);
    if (found.size() < count || delta*delta < found.top())
      Nearest(query, count, left ? middle + 1 : first,
              left ? last : middle, found);
  }

  void Radius(std::size_t query, double radiusSquared, std::size_t limit,
              std::size_t first, std::size_t last, Selection& found) const {
    if (first == last || found.size() >= limit) return;
    const auto middle = first + (last - first) / 2;
    const auto index = order_[middle];
    const auto& point = Point(index);
    if (DistanceSquared(Point(query), point) <= radiusSquared) found.push_back(index);
    const double delta = Coordinate(Point(query), axis_[middle]) - Coordinate(point, axis_[middle]);
    const bool left = delta < 0;
    Radius(query, radiusSquared, limit, left ? first : middle + 1,
           left ? middle : last, found);
    if (delta*delta <= radiusSquared)
      Radius(query, radiusSquared, limit, left ? middle + 1 : first,
             left ? last : middle, found);
  }

  const std::vector<PointXyz>& points_;
  const Selection& selected_;
  Selection order_;
  std::vector<std::uint8_t> axis_;
};

struct VoxelKey {
  std::int64_t x, y, z;
  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash {
  std::size_t operator()(const VoxelKey& key) const {
    const auto x = std::hash<std::int64_t>{}(key.x);
    const auto y = std::hash<std::int64_t>{}(key.y);
    const auto z = std::hash<std::int64_t>{}(key.z);
    return x ^ (y << 1U) ^ (z << 2U);
  }
};

void Voxel(const std::vector<PointXyz>& points, Selection& selected, double size) {
  std::unordered_set<VoxelKey, VoxelHash> seen;
  const double inverse = 1.0 / size;
  selected.erase(std::remove_if(selected.begin(), selected.end(), [&](std::size_t i) {
    const auto& point = points[i];
    return !seen.insert({static_cast<std::int64_t>(std::floor(point.x * inverse)),
                         static_cast<std::int64_t>(std::floor(point.y * inverse)),
                         static_cast<std::int64_t>(std::floor(point.z * inverse))}).second;
  }), selected.end());
}

void Statistical(const std::vector<PointXyz>& points, Selection& selected,
                 const SourceEditOperation& operation) {
  if (selected.size() < 2) return;
  const PointIndex index(points, selected);
  const auto count = std::min<std::size_t>(operation.neighbors, selected.size() - 1);
  std::vector<double> distances(selected.size());
  for (std::size_t i = 0; i < selected.size(); ++i)
    distances[i] = index.MeanDistance(i, count);
  const double mean = std::accumulate(distances.begin(), distances.end(), 0.0) /
                      static_cast<double>(distances.size());
  double variance = 0;
  for (double distance : distances) variance += (distance - mean) * (distance - mean);
  const double threshold = mean + operation.std_ratio *
      std::sqrt(variance / static_cast<double>(distances.size() - 1));
  Selection kept;
  kept.reserve(selected.size());
  for (std::size_t i = 0; i < selected.size(); ++i)
    if (distances[i] <= threshold) kept.push_back(selected[i]);
  selected = std::move(kept);
}

void RadiusFilter(const std::vector<PointXyz>& points, Selection& selected,
                  const SourceEditOperation& operation) {
  const PointIndex index(points, selected);
  Selection kept;
  kept.reserve(selected.size());
  for (std::size_t i = 0; i < selected.size(); ++i)
    if (index.WithinRadius(i, operation.radius, operation.min_points).size() >=
        static_cast<std::size_t>(operation.min_points)) kept.push_back(selected[i]);
  selected = std::move(kept);
}

void Clusters(const std::vector<PointXyz>& points, Selection& selected,
              const SourceEditOperation& operation) {
  const PointIndex index(points, selected);
  constexpr auto unassigned = std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> labels(selected.size(), unassigned);
  std::vector<bool> visited(selected.size(), false);
  std::vector<std::size_t> sizes;
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (visited[i]) continue;
    visited[i] = true;
    auto pending = index.WithinRadius(i, operation.radius);
    if (pending.size() < static_cast<std::size_t>(operation.neighbors)) continue;
    const auto label = sizes.size();
    std::size_t size = 1;
    labels[i] = label;
    for (std::size_t next = 0; next < pending.size(); ++next) {
      const auto point = pending[next];
      if (labels[point] == unassigned) {
        labels[point] = label;
        ++size;
      }
      if (visited[point]) continue;
      visited[point] = true;
      const auto neighbors = index.WithinRadius(point, operation.radius);
      if (neighbors.size() >= static_cast<std::size_t>(operation.neighbors)) {
        for (auto neighbor : neighbors) {
          if (labels[neighbor] != unassigned) continue;
          labels[neighbor] = label;
          ++size;
          pending.push_back(neighbor);
        }
      }
    }
    sizes.push_back(size);
  }
  Selection kept;
  kept.reserve(selected.size());
  for (std::size_t i = 0; i < selected.size(); ++i)
    if (labels[i] != unassigned &&
        sizes[labels[i]] >= static_cast<std::size_t>(operation.min_cluster_points))
      kept.push_back(selected[i]);
  selected = std::move(kept);
}

struct Plane {
  double x{0}, y{0}, z{0}, offset{0};

  bool Fit(const PointXyz& a, const PointXyz& b, const PointXyz& c) {
    const double ux = static_cast<double>(b.x) - a.x;
    const double uy = static_cast<double>(b.y) - a.y;
    const double uz = static_cast<double>(b.z) - a.z;
    const double vx = static_cast<double>(c.x) - a.x;
    const double vy = static_cast<double>(c.y) - a.y;
    const double vz = static_cast<double>(c.z) - a.z;
    x = uy*vz - uz*vy;
    y = uz*vx - ux*vz;
    z = ux*vy - uy*vx;
    const double norm = std::sqrt(x*x + y*y + z*z);
    if (norm <= 1e-12) return false;
    x /= norm;
    y /= norm;
    z /= norm;
    offset = -(x*a.x + y*a.y + z*a.z);
    return true;
  }

  bool Contains(const PointXyz& point, double threshold) const {
    return std::abs(x*point.x + y*point.y + z*point.z + offset) <= threshold;
  }
};

void PlaneCrop(const std::vector<PointXyz>& points, Selection& selected,
               const SourceEditOperation& operation) {
  constexpr std::size_t minInliers = 80;
  constexpr std::size_t sampleLimit = 8192;
  std::mt19937 random(0);
  // Bound candidate scoring by a uniform sample, then measure accepted planes on all points.
  Selection sample;
  sample.reserve(std::min(selected.size(), sampleLimit));
  for (std::size_t i = 0; i < selected.size(); ++i) {
    if (i < sampleLimit) sample.push_back(selected[i]);
    else {
      const auto slot = std::uniform_int_distribution<std::size_t>(0, i)(random);
      if (slot < sampleLimit) sample[slot] = selected[i];
    }
  }
  Selection remaining = selected;
  std::vector<PcdBounds> boxes;
  for (int plane = 0; plane < 6 && remaining.size() >= minInliers && sample.size() >= 3; ++plane) {
    Plane best;
    std::size_t bestCount = 0;
    std::uniform_int_distribution<std::size_t> pick(0, sample.size() - 1);
    for (int candidate = 0; candidate < 1000; ++candidate) {
      const auto a = pick(random), b = pick(random), c = pick(random);
      if (a == b || a == c || b == c) continue;
      Plane fitted;
      if (!fitted.Fit(points[sample[a]], points[sample[b]], points[sample[c]])) continue;
      std::size_t count = 0;
      for (auto i : sample) if (fitted.Contains(points[i], operation.distance_threshold)) ++count;
      if (count > bestCount) {
        best = fitted;
        bestCount = count;
      }
    }
    if (bestCount < 3) break;
    const double infinity = std::numeric_limits<double>::infinity();
    PcdBounds bounds{true, infinity, infinity, infinity, -infinity, -infinity, -infinity};
    Selection outside;
    outside.reserve(remaining.size());
    std::size_t count = 0;
    for (auto i : remaining) {
      const auto& point = points[i];
      if (!best.Contains(point, operation.distance_threshold)) {
        outside.push_back(i);
        continue;
      }
      ++count;
      bounds.min_x = std::min<double>(bounds.min_x, point.x);
      bounds.min_y = std::min<double>(bounds.min_y, point.y);
      bounds.min_z = std::min<double>(bounds.min_z, point.z);
      bounds.max_x = std::max<double>(bounds.max_x, point.x);
      bounds.max_y = std::max<double>(bounds.max_y, point.y);
      bounds.max_z = std::max<double>(bounds.max_z, point.z);
    }
    if (count < minInliers) break;
    bounds.min_x -= operation.margin;
    bounds.min_y -= operation.margin;
    bounds.min_z -= operation.margin;
    bounds.max_x += operation.margin;
    bounds.max_y += operation.margin;
    bounds.max_z += operation.margin;
    boxes.push_back(bounds);
    remaining = std::move(outside);
    sample.erase(std::remove_if(sample.begin(), sample.end(), [&](std::size_t i) {
      return best.Contains(points[i], operation.distance_threshold);
    }), sample.end());
  }
  if (boxes.empty()) throw std::runtime_error("plane_crop found no plane with at least 80 inliers");
  selected.erase(std::remove_if(selected.begin(), selected.end(), [&](std::size_t i) {
    return std::none_of(boxes.begin(), boxes.end(), [&](const PcdBounds& bounds) {
      return InBounds(points[i], bounds);
    });
  }), selected.end());
}

}  // namespace

void ValidateSourceEdits(const std::vector<SourceEditOperation>& operations) {
  const auto positive = [](double value) { return std::isfinite(value) && value > 0; };
  for (const auto& edit : operations) {
    const auto& name = edit.operation;
    bool valid = false;
    if (name == "delete_inside" || name == "keep_inside" || name == "add_plane") {
      const auto& b = edit.bounds;
      valid = b.enabled && std::isfinite(b.min_x) && std::isfinite(b.min_y) &&
          std::isfinite(b.min_z) && std::isfinite(b.max_x) && std::isfinite(b.max_y) &&
          std::isfinite(b.max_z) && b.min_x <= b.max_x && b.min_y <= b.max_y && b.min_z <= b.max_z;
      if (name == "add_plane") {
        const int axis = edit.axis == "x" ? 0 : edit.axis == "y" ? 1 : 2;
        const double minimum[] = {b.min_x, b.min_y, b.min_z};
        const double maximum[] = {b.max_x, b.max_y, b.max_z};
        valid = valid && (edit.axis == "x" || edit.axis == "y" || edit.axis == "z") &&
            std::isfinite(edit.offset) && positive(edit.spacing) &&
            edit.offset >= minimum[axis] && edit.offset <= maximum[axis];
        double count = 1;
        for (int i = 0; i < 3; ++i) if (i != axis) {
          valid = valid && maximum[i] > minimum[i] &&
              std::abs(minimum[i]) <= std::numeric_limits<float>::max() &&
              std::abs(maximum[i]) <= std::numeric_limits<float>::max();
          count *= std::ceil((maximum[i] - minimum[i]) / edit.spacing) + 1;
        }
        valid = valid && std::abs(edit.offset) <= std::numeric_limits<float>::max() &&
            std::isfinite(count) && count <= 1000000;
      }
    } else if (name == "voxel") valid = positive(edit.voxel_size);
    else if (name == "statistical") valid = edit.neighbors > 0 && positive(edit.std_ratio);
    else if (name == "radius") valid = positive(edit.radius) && edit.min_points > 0;
    else if (name == "clusters")
      valid = positive(edit.radius) && edit.neighbors > 0 && edit.min_cluster_points > 0;
    else if (name == "plane_crop")
      valid = positive(edit.distance_threshold) && std::isfinite(edit.margin) && edit.margin >= 0;
    if (!valid) throw std::invalid_argument("invalid point cloud edit: " + name);
  }
}

namespace {
void FilterSelection(const std::vector<PointXyz>& points, Selection& selected,
                     const SourceEditOperation& edit) {
    if (edit.operation == "delete_inside" || edit.operation == "keep_inside") {
      selected.erase(std::remove_if(selected.begin(), selected.end(), [&](std::size_t i) {
        const bool inside = InBounds(points[i], edit.bounds);
        return edit.operation == "delete_inside" ? inside : !inside;
      }), selected.end());
    } else if (edit.operation == "voxel") Voxel(points, selected, edit.voxel_size);
    else if (edit.operation == "statistical") Statistical(points, selected, edit);
    else if (edit.operation == "radius") RadiusFilter(points, selected, edit);
    else if (edit.operation == "clusters") Clusters(points, selected, edit);
    else if (edit.operation == "plane_crop") PlaneCrop(points, selected, edit);
    else throw std::invalid_argument("unsupported subset edit: " + edit.operation);
}
}  // namespace

Selection SelectSourceEdits(const std::vector<PointXyz>& points,
                            const std::vector<SourceEditOperation>& operations) {
  Selection selected(points.size());
  std::iota(selected.begin(), selected.end(), std::size_t{0});
  for (const auto& edit : operations) FilterSelection(points, selected, edit);
  return selected;
}

Selection ApplySourceEdits(std::vector<PointXyz>& points,
                          const std::vector<SourceEditOperation>& operations) {
  Selection selected(points.size());
  std::iota(selected.begin(), selected.end(), std::size_t{0});
  for (const auto& edit : operations) {
    if (edit.operation != "add_plane") {
      FilterSelection(points, selected, edit);
      continue;
    }
    const auto& b = edit.bounds;
    const double minimum[] = {b.min_x, b.min_y, b.min_z};
    const double maximum[] = {b.max_x, b.max_y, b.max_z};
    const int axis = edit.axis == "x" ? 0 : edit.axis == "y" ? 1 : 2;
    const int u = axis == 0 ? 1 : 0, v = axis == 2 ? 1 : 2;
    const auto nu = static_cast<std::size_t>(std::ceil((maximum[u] - minimum[u]) / edit.spacing));
    const auto nv = static_cast<std::size_t>(std::ceil((maximum[v] - minimum[v]) / edit.spacing));
    for (std::size_t i = 0; i <= nu; ++i) for (std::size_t j = 0; j <= nv; ++j) {
      double p[3];
      p[axis] = edit.offset;
      p[u] = minimum[u] + (maximum[u] - minimum[u]) * i / nu;
      p[v] = minimum[v] + (maximum[v] - minimum[v]) * j / nv;
      selected.push_back(points.size());
      points.push_back({static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
    }
  }
  return selected;
}

}  // namespace lingtu::maps
