#include "lingtu/maps/semantic/stairs.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <utility>

namespace lingtu::maps {
namespace {

struct Sample { double s, t, z; };
using Cell = std::pair<int, int>;
struct SurfaceCell {
  Cell cell;
  Sample mean;
  double s_min, s_max, t_min, t_max;
  std::size_t count;
  bool horizontal{false};
  bool visited{false};
};

bool Finite(const PointXyz& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// Fit z = a*s + b*t + c with centered coordinates. A wall/line has no XY
// support area. Fit on cell observations, so point density is not a plane vote.
double MedianHeight(const std::vector<const SurfaceCell*>& neighbors) {
  std::vector<double> heights;
  heights.reserve(neighbors.size());
  for (const auto* node : neighbors) heights.push_back(node->mean.z);
  const auto middle = heights.begin()+heights.size()/2;
  std::nth_element(heights.begin(),middle,heights.end());
  return *middle;
}

bool Horizontal(const std::vector<const SurfaceCell*>& input, double tolerance) {
  if (input.size() < 5) return false;
  const double height = MedianHeight(input);
  std::vector<const SurfaceCell*> neighbors;
  for (const auto* node : input) {
    if (std::abs(node->mean.z-height) <= 0.5*tolerance) neighbors.push_back(node);
  }
  if (neighbors.size() < 5) return false;
  Sample mean{};
  for (const auto* node : neighbors) {
    mean.s += node->mean.s; mean.t += node->mean.t; mean.z += node->mean.z;
  }
  mean.s /= neighbors.size(); mean.t /= neighbors.size(); mean.z /= neighbors.size();
  double ss = 0, tt = 0, st = 0, sz = 0, tz = 0;
  for (const auto* node : neighbors) {
    const double s = node->mean.s-mean.s, t = node->mean.t-mean.t;
    const double z = node->mean.z-mean.z;
    ss += s*s; tt += t*t; st += s*t; sz += s*z; tz += t*z;
  }
  const double determinant = ss*tt-st*st;
  if (determinant <= 1e-12) return false;
  const double a = (sz*tt-tz*st)/determinant;
  const double b = (tz*ss-sz*st)/determinant;
  if (std::hypot(a,b) > 0.12) return false;
  double residual = 0;
  for (const auto* node : neighbors) {
    const double dz = node->mean.z-mean.z-a*(node->mean.s-mean.s)-b*(node->mean.t-mean.t);
    residual += dz*dz;
  }
  return std::sqrt(residual/neighbors.size()) <= tolerance;
}

bool Contains(const StairSurface& surface, double s, double t, double z,
              double horizontal_tolerance, double vertical_tolerance) {
  return s >= surface.s_min-horizontal_tolerance && s <= surface.s_max+horizontal_tolerance &&
      t >= surface.lateral_min-horizontal_tolerance && t <= surface.lateral_max+horizontal_tolerance &&
      std::abs(z-surface.surface_z) <= vertical_tolerance;
}

bool Follows(const StairSurface& previous, const StairSurface& next,
             const StairRecognitionOptions& options, bool landing_transition) {
  const double rise = next.surface_z-previous.surface_z;
  const double gap = next.s_min-previous.s_max;
  const double lateral_overlap = std::min(previous.lateral_max, next.lateral_max)-
      std::max(previous.lateral_min, next.lateral_min);
  // A measured riser at the configured limit carries the same height error
  // as its fitted surfaces. A smaller observed landing lip may join the
  // flight, but does not count as a stair riser below.
  const double minimum = landing_transition ? options.height_tolerance_m :
      options.min_rise_m-options.height_tolerance_m;
  return rise >= minimum && rise <= options.max_rise_m+options.height_tolerance_m &&
      next.s_max > previous.s_max+options.resolution_m &&
      next.s_min > previous.s_min+options.resolution_m &&
      gap >= -2*options.resolution_m && gap <= 2.5*options.resolution_m &&
      lateral_overlap >= options.min_width_m;
}

}  // namespace

StairGeometry RecognizeStairGeometry(
    const std::vector<PointXyz>& points, const PcdBounds& bounds,
    const PointXyz& lower, const PointXyz& upper,
    const StairRecognitionOptions& options) {
  StairGeometry result;
  result.origin = lower;
  result.lower = lower;
  result.upper = upper;
  const double length = std::hypot(upper.x-lower.x, upper.y-lower.y);
  if (!Finite(lower) || !Finite(upper) || length < options.resolution_m || upper.z <= lower.z ||
      !std::isfinite(options.resolution_m) || options.resolution_m <= 0 ||
      !std::isfinite(options.height_tolerance_m) || options.height_tolerance_m <= 0 ||
      options.min_rise_m <= 2*options.height_tolerance_m || options.max_rise_m < options.min_rise_m) {
    result.reason = "invalid_stair_region";
    return result;
  }
  result.axis_x = (upper.x-lower.x)/length;
  result.axis_y = (upper.y-lower.y)/length;
  const double resolution = options.resolution_m;
  std::map<Cell, std::vector<Sample>> columns;
  for (const auto& point : points) {
    if (!Finite(point)) continue;
    if (bounds.enabled && (point.x < bounds.min_x || point.x > bounds.max_x ||
        point.y < bounds.min_y || point.y > bounds.max_y ||
        point.z < bounds.min_z || point.z > bounds.max_z)) continue;
    const double dx = point.x-lower.x, dy = point.y-lower.y;
    Sample sample{dx*result.axis_x+dy*result.axis_y,
                  -dx*result.axis_y+dy*result.axis_x, point.z};
    const Cell cell{static_cast<int>(std::floor(sample.s/resolution)),
                    static_cast<int>(std::floor(sample.t/resolution))};
    columns[cell].push_back(sample);
  }

  std::vector<SurfaceCell> nodes;
  std::map<Cell, std::vector<std::size_t>> cells;
  for (auto& [cell, samples] : columns) {
    std::sort(samples.begin(), samples.end(), [](const Sample& a, const Sample& b) { return a.z < b.z; });
    for (std::size_t begin = 0; begin < samples.size();) {
      std::size_t end = begin+1;
      while (end < samples.size() && samples[end].z-samples[begin].z <= 2*options.height_tolerance_m) ++end;
      SurfaceCell node{cell, {}, samples[begin].s, samples[begin].s,
                       samples[begin].t, samples[begin].t, end-begin};
      for (std::size_t i = begin; i < end; ++i) {
        node.mean.s += samples[i].s; node.mean.t += samples[i].t; node.mean.z += samples[i].z;
        node.s_min = std::min(node.s_min, samples[i].s); node.s_max = std::max(node.s_max, samples[i].s);
        node.t_min = std::min(node.t_min, samples[i].t); node.t_max = std::max(node.t_max, samples[i].t);
      }
      node.mean.s /= node.count; node.mean.t /= node.count; node.mean.z /= node.count;
      cells[cell].push_back(nodes.size());
      nodes.push_back(node);
      begin = end;
    }
  }
  const auto neighbors = [&](std::size_t index) {
    std::vector<std::size_t> output;
    const auto& node = nodes[index];
    // Voxel observations straddle this flight-aligned grid after rotation or
    // downsampling. Use observed extents, not means or one-cell index distance.
    for (int ds = -2; ds <= 2; ++ds) {
      for (int dt = -2; dt <= 2; ++dt) {
        const auto found = cells.find({node.cell.first+ds, node.cell.second+dt});
        if (found == cells.end()) continue;
        std::size_t best = nodes.size();
        double distance = options.height_tolerance_m;
        for (auto candidate : found->second) {
          const auto& other = nodes[candidate];
          if (other.s_min-node.s_max > 1.6*resolution || node.s_min-other.s_max > 1.6*resolution ||
              other.t_min-node.t_max > 1.6*resolution || node.t_min-other.t_max > 1.6*resolution) continue;
          const double dz = std::abs(nodes[candidate].mean.z-node.mean.z);
          if (dz <= distance) { distance = dz; best = candidate; }
        }
        if (best != nodes.size()) output.push_back(best);
      }
    }
    return output;
  };
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    std::vector<const SurfaceCell*> local;
    for (auto index : neighbors(i)) local.push_back(&nodes[index]);
    nodes[i].horizontal = !local.empty() &&
        std::abs(nodes[i].mean.z-MedianHeight(local)) <= 0.5*options.height_tolerance_m &&
        Horizontal(local, options.height_tolerance_m);
  }

  std::vector<StairSurface> patches;
  for (std::size_t start = 0; start < nodes.size(); ++start) {
    if (!nodes[start].horizontal || nodes[start].visited) continue;
    std::queue<std::size_t> pending;
    pending.push(start); nodes[start].visited = true;
    const auto& first = nodes[start];
    StairSurface patch{first.s_min, first.s_max, first.t_min, first.t_max};
    std::vector<const SurfaceCell*> component;
    std::vector<std::size_t> component_indices;
    double sum_z = 0;
    while (!pending.empty()) {
      const auto index = pending.front(); pending.pop();
      const auto& node = nodes[index]; component.push_back(&node);
      component_indices.push_back(index);
      patch.s_min = std::min(patch.s_min,node.s_min); patch.s_max = std::max(patch.s_max,node.s_max);
      patch.lateral_min = std::min(patch.lateral_min,node.t_min);
      patch.lateral_max = std::max(patch.lateral_max,node.t_max);
      patch.observed_points += node.count; sum_z += node.mean.z;
      for (auto adjacent : neighbors(index)) {
        // Pairwise closeness alone can flood through riser observations and
        // merge successive low treads into one gradually rising component.
        if (nodes[adjacent].horizontal && !nodes[adjacent].visited &&
            std::abs(nodes[adjacent].mean.z-first.mean.z) <= options.height_tolerance_m) {
          nodes[adjacent].visited = true; pending.push(adjacent);
        }
      }
    }
    patch.surface_z = sum_z/component.size();
    const double depth = patch.s_max-patch.s_min;
    const double width = patch.lateral_max-patch.lateral_min;
    if (depth >= options.min_tread_depth_m && width >= options.min_width_m &&
        component.size()*resolution*resolution >= 0.5*depth*width &&
        Horizontal(component, options.height_tolerance_m)) {
      // A riser contaminates the local normal of boundary cells. Recover only
      // their observed near-plane samples after fitting the tread interior.
      // Do not extend through absent cells or extrapolate a landing surface.
      std::set<std::size_t> boundary;
      for (auto index : component_indices) {
        for (auto adjacent : neighbors(index)) {
          if (!nodes[adjacent].horizontal &&
              std::abs(nodes[adjacent].mean.z-patch.surface_z) <= options.height_tolerance_m) boundary.insert(adjacent);
        }
      }
      for (auto index : boundary) {
        const auto& node = nodes[index];
        patch.s_min = std::min(patch.s_min,node.s_min); patch.s_max = std::max(patch.s_max,node.s_max);
        patch.lateral_min = std::min(patch.lateral_min,node.t_min);
        patch.lateral_max = std::max(patch.lateral_max,node.t_max);
        patch.observed_points += node.count;
      }
      patches.push_back(patch);
    }
  }
  if (patches.size() < 3) return result;
  std::sort(patches.begin(), patches.end(), [](const auto& a, const auto& b) { return a.surface_z < b.surface_z; });
  std::vector<int> parent(patches.size(), -1), chain_size(patches.size(), 0),
      stair_risers(patches.size(), 0);
  int end = -1;
  for (std::size_t i = 0; i < patches.size(); ++i) {
    if (Contains(patches[i],0,0,lower.z,2*resolution,2*options.height_tolerance_m)) chain_size[i] = 1;
    for (std::size_t j = 0; j < i; ++j) {
      const bool landing_transition = chain_size[j] == 1 ||
          Contains(patches[i],length,0,upper.z,2*resolution,2*options.height_tolerance_m);
      if (chain_size[j] > 0 && chain_size[j]+1 > chain_size[i] &&
          Follows(patches[j],patches[i],options,landing_transition)) {
        chain_size[i] = chain_size[j]+1; parent[i] = static_cast<int>(j);
        stair_risers[i] = stair_risers[j] +
            (patches[i].surface_z-patches[j].surface_z >=
             options.min_rise_m-options.height_tolerance_m ? 1 : 0);
      }
    }
    if (chain_size[i] >= 3 && stair_risers[i] >= 2 &&
        Contains(patches[i],length,0,upper.z,2*resolution,2*options.height_tolerance_m) &&
        (end < 0 || chain_size[i] > chain_size[end])) end = static_cast<int>(i);
  }
  if (end < 0) { result.reason = "no_tread_chain"; return result; }
  for (int i = end; i >= 0; i = parent[i]) result.surfaces.push_back(patches[i]);
  std::reverse(result.surfaces.begin(), result.surfaces.end());
  double lateral_min = -std::numeric_limits<double>::infinity();
  double lateral_max = std::numeric_limits<double>::infinity();
  for (const auto& patch : result.surfaces) {
    lateral_min = std::max(lateral_min,patch.lateral_min);
    lateral_max = std::min(lateral_max,patch.lateral_max);
  }
  result.width_m = std::max(0.0,lateral_max-lateral_min);
  if (result.width_m < options.min_width_m) {
    result.surfaces.clear(); result.reason = "no_tread_chain"; return result;
  }
  result.lower.z = static_cast<float>(result.surfaces.front().surface_z);
  result.upper.z = static_cast<float>(result.surfaces.back().surface_z);
  result.mean_rise_m = (result.upper.z-result.lower.z)/(result.surfaces.size()-1);
  double run = 0;
  for (std::size_t i = 1; i+1 < result.surfaces.size(); ++i) run += result.surfaces[i].s_max-result.surfaces[i].s_min;
  result.mean_run_m = run/(result.surfaces.size()-2);
  result.recognized = true; result.reason = "stairs";
  return result;
}

}  // namespace lingtu::maps
