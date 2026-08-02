#include "motion/motion_layer.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>

namespace lingtu::nav::endpoint {
namespace {

double positiveOr(double value, double fallback) {
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

bool sameFrame(double a, double b) {
  return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1e-9;
}

}  // namespace

std::size_t MotionLayer::VoxelKeyHash::operator()(const VoxelKey &key) const {
  std::size_t h = 1469598103934665603ull;
  auto mix = [&](int value) {
    h ^= static_cast<std::size_t>(value) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
  };
  mix(key.x);
  mix(key.y);
  mix(key.z);
  return h;
}

MotionLayer::MotionLayer(MotionLayerConfig config) {
  configure(config);
}

void MotionLayer::configure(MotionLayerConfig config) {
  config.voxel_size_m = positiveOr(config.voxel_size_m, 0.08);
  config.decay_s = std::max(0.0, config.decay_s);
  config.inflation_radius_m = std::max(0.0, config.inflation_radius_m);
  config.ray_clear_max_range_m = std::max(0.0, config.ray_clear_max_range_m);
  config.ray_clearing_interval_s = std::max(0.0, config.ray_clearing_interval_s);
  config.max_clearing_rays = std::max<std::size_t>(1, config.max_clearing_rays);
  config.min_hits = std::max(1, config.min_hits);
  config.static_min_frames = std::max<std::uint32_t>(1, config.static_min_frames);
  config.free_min_frames = std::max<std::uint32_t>(1, config.free_min_frames);
  config.static_free_min_frames = std::max(config.free_min_frames, config.static_free_min_frames);
  config.dynamic_min_cells = std::max<std::size_t>(1, config.dynamic_min_cells);
  config.dynamic_free_min_frames = std::max<std::uint32_t>(1, config.dynamic_free_min_frames);
  config.dynamic_min_speed_mps = std::max(0.0, config.dynamic_min_speed_mps);
  config.dynamic_max_speed_mps =
      std::max(config.dynamic_min_speed_mps, config.dynamic_max_speed_mps);
  config.dynamic_max_z_speed_mps = std::max(0.0, config.dynamic_max_z_speed_mps);
  config.dynamic_min_dir_cos = std::clamp(config.dynamic_min_dir_cos, -1.0, 1.0);
  config.dynamic_min_height_m = std::max(0.0, config.dynamic_min_height_m);
  config.dynamic_max_height_m = std::max(config.dynamic_min_height_m, config.dynamic_max_height_m);
  config.dynamic_confirm_frames = std::max<std::uint32_t>(2, config.dynamic_confirm_frames);
  config.dynamic_track_ttl_s = std::max(0.0, config.dynamic_track_ttl_s);
  config.dynamic_match_distance_m = positiveOr(config.dynamic_match_distance_m, 0.75);
  config_ = config;
  refreshStats();
}

MotionLayer::VoxelKey MotionLayer::makeKey(float x, float y, float z) const {
  return makeKeyForSize(x, y, z, config_.voxel_size_m);
}

MotionLayer::VoxelKey MotionLayer::makeKeyForSize(float x, float y, float z,
                                                  double voxel_size_m) const {
  return {
      static_cast<int>(std::floor(static_cast<double>(x) / voxel_size_m)),
      static_cast<int>(std::floor(static_cast<double>(y) / voxel_size_m)),
      static_cast<int>(std::floor(static_cast<double>(z) / voxel_size_m)),
  };
}

MotionLayer::Cell MotionLayer::cellAtKey(const VoxelKey &key, double stamp_s) const {
  const float voxel = static_cast<float>(config_.voxel_size_m);
  Cell cell{
      (static_cast<float>(key.x) + 0.5f) * voxel,
      (static_cast<float>(key.y) + 0.5f) * voxel,
      (static_cast<float>(key.z) + 0.5f) * voxel,
      0.0f,
      -1.0,
      stamp_s,
      stamp_s,
      0,
      0,
      1,
      CellState::Free,
  };
  cell.free_observations = 1;
  return cell;
}

bool MotionLayer::isObstacle(CellState state) {
  return state == CellState::Occupied || state == CellState::Static;
}

bool MotionLayer::isStale(const Cell &cell, double now_s) const {
  if (config_.decay_s <= 0.0) {
    return false;
  }
  const double age_source = cell.state == CellState::Free || cell.state == CellState::Cleared
                                ? cell.last_free_s
                                : cell.last_hit_s;
  return age_source >= 0.0 && now_s - age_source > config_.decay_s;
}

void MotionLayer::markHit(const Cell &sample, double stamp_s) {
  auto &cell = cells_[makeKey(sample.x, sample.y, sample.z)];
  const bool new_hit_frame = !sameFrame(cell.last_hit_s, stamp_s);
  if (cell.hits <= 0 || sample.height > cell.height || cell.state == CellState::Free ||
      cell.state == CellState::Cleared) {
    cell.x = sample.x;
    cell.y = sample.y;
    cell.z = sample.z;
    cell.height = sample.height;
  }
  cell.last_hit_s = stamp_s;
  cell.last_update_s = stamp_s;
  cell.hits = std::min(cell.hits + 1, 1000000);
  if (new_hit_frame) {
    cell.hit_frames = std::min<std::uint32_t>(cell.hit_frames + 1, 1000000);
  }
  cell.free_frames = 0;
  cell.state =
      cell.hit_frames >= config_.static_min_frames ? CellState::Static : CellState::Occupied;
}

void MotionLayer::markFree(const VoxelKey &key, double stamp_s) {
  auto [it, inserted] = cells_.emplace(key, cellAtKey(key, stamp_s));
  Cell &cell = it->second;
  const bool had_obstacle = isObstacle(cell.state);
  if (!inserted) {
    if (!sameFrame(cell.last_free_s, stamp_s)) {
      cell.free_frames = std::min<std::uint32_t>(cell.free_frames + 1, 1000000);
      cell.free_observations = std::min<std::uint32_t>(cell.free_observations + 1, 1000000);
    }
    cell.last_free_s = stamp_s;
    cell.last_update_s = stamp_s;
  }
  if (cell.hits <= 0) {
    cell.state = CellState::Free;
    return;
  }
  if (sameFrame(cell.last_hit_s, stamp_s)) {
    return;
  }
  const std::uint32_t required_free =
      cell.state == CellState::Static ? config_.static_free_min_frames : config_.free_min_frames;
  if (cell.free_frames >= required_free && cell.last_free_s >= cell.last_hit_s) {
    cell.state = CellState::Cleared;
    if (had_obstacle) {
      ++stats_.ray_cleared_cells;
    }
  }
}

void MotionLayer::collectRayFreeKeys(const SensorOrigin &origin, const Cell &endpoint,
                                     std::unordered_set<VoxelKey, VoxelKeyHash> &keys) const {
  if (!config_.ray_clearing_enabled || !origin.valid || config_.ray_clear_max_range_m <= 0.0) {
    return;
  }
  const double dx = static_cast<double>(endpoint.x) - origin.x;
  const double dy = static_cast<double>(endpoint.y) - origin.y;
  const double dz = static_cast<double>(endpoint.z) - origin.z;
  const double range = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!std::isfinite(range) || range <= config_.voxel_size_m) {
    return;
  }
  const double clear_range = std::min(config_.ray_clear_max_range_m, range - config_.voxel_size_m);
  if (clear_range <= 0.0) {
    return;
  }
  const double inv_range = 1.0 / range;
  const double ux = dx * inv_range;
  const double uy = dy * inv_range;
  const double uz = dz * inv_range;
  const double step = std::max(config_.voxel_size_m * 0.75, 0.04);
  for (double r = config_.voxel_size_m; r < clear_range; r += step) {
    const float x = static_cast<float>(origin.x + ux * r);
    const float y = static_cast<float>(origin.y + uy * r);
    const float z = static_cast<float>(origin.z + uz * r);
    keys.insert(makeKey(x, y, z));
  }
}

void MotionLayer::update(const std::vector<float> &xyzh, double stamp_s) {
  const std::size_t count = xyzh.size() / 4;
  current_hit_keys_.clear();
  current_hit_keys_.reserve(count);
  current_dynamic_keys_.clear();
  current_stamp_s_ = stamp_s;
  cells_.reserve(cells_.size() + count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t base = i * 4;
    const Cell sample{
        xyzh[base + 0],
        xyzh[base + 1],
        xyzh[base + 2],
        xyzh[base + 3],
        stamp_s,
        -1.0,
        stamp_s,
        1,
        1,
        0,
        CellState::Occupied,
    };
    if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z) ||
        !std::isfinite(sample.height)) {
      continue;
    }
    current_hit_keys_.insert(makeKey(sample.x, sample.y, sample.z));
    markHit(sample, stamp_s);
  }
  prune(stamp_s);
}

void MotionLayer::updateFromScan(const SensorOrigin &origin, const std::vector<float> &xyzh,
                                 double stamp_s) {
  if (origin.valid && std::isfinite(origin.x) && std::isfinite(origin.y) &&
      std::isfinite(origin.z)) {
    last_sensor_origin_ = origin;
  }
  const std::size_t count = xyzh.size() / 4;
  current_hit_keys_.clear();
  current_dynamic_keys_.clear();
  current_stamp_s_ = stamp_s;
  if (count == 0) {
    prune(stamp_s);
    return;
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> hit_keys;
  hit_keys.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t base = i * 4;
    const float x = xyzh[base + 0];
    const float y = xyzh[base + 1];
    const float z = xyzh[base + 2];
    const float height = xyzh[base + 3];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(height)) {
      continue;
    }
    hit_keys.insert(makeKey(x, y, z));
  }

  const std::size_t ray_stride =
      count > config_.max_clearing_rays
          ? static_cast<std::size_t>(std::ceil(static_cast<double>(count) /
                                               static_cast<double>(config_.max_clearing_rays)))
          : 1;
  const bool run_clearing = config_.ray_clearing_enabled && origin.valid &&
                            (last_ray_clearing_s_ < 0.0 ||
                             stamp_s - last_ray_clearing_s_ >= config_.ray_clearing_interval_s);
  if (run_clearing) {
    std::unordered_set<VoxelKey, VoxelKeyHash> free_keys;
    free_keys.reserve(config_.max_clearing_rays * 64);
    for (std::size_t i = 0; i < count; i += ray_stride) {
      const std::size_t base = i * 4;
      const Cell endpoint{
          xyzh[base + 0],
          xyzh[base + 1],
          xyzh[base + 2],
          xyzh[base + 3],
          stamp_s,
          -1.0,
          stamp_s,
          1,
          1,
          0,
          CellState::Occupied,
      };
      if (!std::isfinite(endpoint.x) || !std::isfinite(endpoint.y) || !std::isfinite(endpoint.z) ||
          !std::isfinite(endpoint.height)) {
        continue;
      }
      collectRayFreeKeys(origin, endpoint, free_keys);
      ++stats_.raycast_rays;
    }
    stats_.raycast_voxels += free_keys.size();
    for (const auto &key : free_keys) {
      if (hit_keys.find(key) != hit_keys.end()) {
        continue;
      }
      markFree(key, stamp_s);
    }
    last_ray_clearing_s_ = stamp_s;
  }

  cells_.reserve(cells_.size() + count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::size_t base = i * 4;
    const Cell sample{
        xyzh[base + 0],
        xyzh[base + 1],
        xyzh[base + 2],
        xyzh[base + 3],
        stamp_s,
        -1.0,
        stamp_s,
        1,
        1,
        0,
        CellState::Occupied,
    };
    if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z) ||
        !std::isfinite(sample.height)) {
      continue;
    }
    markHit(sample, stamp_s);
  }
  current_hit_keys_ = std::move(hit_keys);
  prune(stamp_s);
}

void MotionLayer::prune(double now_s) {
  constexpr double kPruneIntervalS = 0.05;
  if (last_prune_s_ >= 0.0 && now_s >= last_prune_s_ && now_s - last_prune_s_ < kPruneIntervalS) {
    return;
  }
  last_prune_s_ = now_s;
  ++stats_.prune_passes;
  if (config_.decay_s <= 0.0) {
    refreshStats();
    return;
  }
  for (auto it = cells_.begin(); it != cells_.end();) {
    if (isStale(it->second, now_s)) {
      it = cells_.erase(it);
    } else {
      ++it;
    }
  }
  refreshStats();
}

std::vector<float> MotionLayer::snapshot(std::size_t max_points, double now_s) {
  prune(now_s);
  std::vector<float> out;
  if (cells_.empty()) {
    return out;
  }
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> output_cells;
  const int inflation_steps =
      static_cast<int>(std::ceil(config_.inflation_radius_m / config_.voxel_size_m));
  if (inflation_steps == 0) {
    std::vector<const Cell *> obstacles;
    obstacles.reserve(stats_.obstacle_cells);
    for (const auto &entry : cells_) {
      const Cell &cell = entry.second;
      if (isObstacle(cell.state) && cell.hits >= config_.min_hits) {
        obstacles.push_back(&cell);
      }
    }
    if (obstacles.empty()) {
      return out;
    }
    const auto distance_squared = [&](const Cell *cell) {
      const double origin_x = last_sensor_origin_.valid ? last_sensor_origin_.x : 0.0;
      const double origin_y = last_sensor_origin_.valid ? last_sensor_origin_.y : 0.0;
      const double origin_z = last_sensor_origin_.valid ? last_sensor_origin_.z : 0.0;
      const double dx = static_cast<double>(cell->x) - origin_x;
      const double dy = static_cast<double>(cell->y) - origin_y;
      const double dz = static_cast<double>(cell->z) - origin_z;
      return dx * dx + dy * dy + dz * dz;
    };
    if (max_points == 0 || obstacles.size() <= max_points) {
      out.reserve(obstacles.size() * 4);
      for (const Cell *cell : obstacles) {
        out.push_back(cell->x);
        out.push_back(cell->y);
        out.push_back(cell->z);
        out.push_back(cell->height);
      }
      return out;
    }

    std::partial_sort(obstacles.begin(),
                      obstacles.begin() + static_cast<std::ptrdiff_t>(max_points), obstacles.end(),
                      [&](const Cell *lhs, const Cell *rhs) {
                        return distance_squared(lhs) < distance_squared(rhs);
                      });
    out.reserve(max_points * 4);
    for (std::size_t i = 0; i < max_points; ++i) {
      const Cell &cell = *obstacles[i];
      out.push_back(cell.x);
      out.push_back(cell.y);
      out.push_back(cell.z);
      out.push_back(cell.height);
    }
    return out;
  }
  output_cells.reserve(cells_.size());
  for (const auto &entry : cells_) {
    const auto &cell = entry.second;
    if (!isObstacle(cell.state) || cell.hits < config_.min_hits) {
      continue;
    }
    const auto base_key = makeKey(cell.x, cell.y, cell.z);
    for (int ix = -inflation_steps; ix <= inflation_steps; ++ix) {
      for (int iy = -inflation_steps; iy <= inflation_steps; ++iy) {
        const double offset = std::hypot(static_cast<double>(ix) * config_.voxel_size_m,
                                         static_cast<double>(iy) * config_.voxel_size_m);
        if (offset > config_.inflation_radius_m + 1e-9) {
          continue;
        }
        const VoxelKey key{base_key.x + ix, base_key.y + iy, base_key.z};
        Cell inflated = cell;
        inflated.x = static_cast<float>(cell.x + ix * config_.voxel_size_m);
        inflated.y = static_cast<float>(cell.y + iy * config_.voxel_size_m);
        auto [it, inserted] = output_cells.emplace(key, inflated);
        if (!inserted && inflated.height > it->second.height) {
          it->second = inflated;
        }
      }
    }
  }
  if (output_cells.empty()) {
    return out;
  }
  if (max_points > 0 && output_cells.size() > max_points) {
    std::unordered_map<VoxelKey, Cell, VoxelKeyHash> reduced;
    const double initial_ratio =
        std::cbrt(static_cast<double>(output_cells.size()) / static_cast<double>(max_points));
    double coarse_size = config_.voxel_size_m * std::max(1.0, initial_ratio);
    for (int attempt = 0; attempt < 8; ++attempt) {
      reduced.clear();
      reduced.reserve(std::min(output_cells.size(), max_points * 2));
      for (const auto &entry : output_cells) {
        const auto &cell = entry.second;
        const VoxelKey key = makeKeyForSize(cell.x, cell.y, cell.z, coarse_size);
        auto [it, inserted] = reduced.emplace(key, cell);
        if (!inserted && cell.height > it->second.height) {
          it->second = cell;
        }
      }
      if (reduced.size() <= max_points) {
        output_cells.swap(reduced);
        break;
      }
      coarse_size *= 1.25;
    }
  }
  const std::size_t keep =
      max_points == 0 ? output_cells.size() : std::min(max_points, output_cells.size());
  out.reserve(keep * 4);
  std::size_t added = 0;
  for (const auto &entry : output_cells) {
    if (max_points > 0 && added >= keep) {
      break;
    }
    const auto &cell = entry.second;
    out.push_back(cell.x);
    out.push_back(cell.y);
    out.push_back(cell.z);
    out.push_back(cell.height);
    ++added;
  }
  return out;
}

std::vector<float> MotionLayer::snapshotDynamic(std::size_t max_points, double now_s) {
  prune(now_s);
  dynamicClusters(0, now_s);
  std::vector<float> out;
  if (current_dynamic_keys_.empty()) {
    return out;
  }
  const std::size_t keep = max_points == 0 ? current_dynamic_keys_.size()
                                           : std::min(max_points, current_dynamic_keys_.size());
  out.reserve(keep * 4);
  std::size_t added = 0;
  for (const auto &key : current_dynamic_keys_) {
    if (max_points > 0 && added >= keep) {
      break;
    }
    const auto it = cells_.find(key);
    if (it == cells_.end()) {
      continue;
    }
    const auto &cell = it->second;
    out.push_back(cell.x);
    out.push_back(cell.y);
    out.push_back(cell.z);
    out.push_back(cell.height);
    ++added;
  }
  return out;
}

MotionCellQuery MotionLayer::query(float x, float y, float z, double now_s) const {
  const auto it = cells_.find(makeKey(x, y, z));
  if (it == cells_.end() || isStale(it->second, now_s)) {
    return MotionCellQuery{};
  }
  const Cell &cell = it->second;
  MotionCellQuery out;
  out.state = cell.state;
  out.x = cell.x;
  out.y = cell.y;
  out.z = cell.z;
  out.height = cell.height;
  out.hits = cell.hits;
  out.hit_frames = cell.hit_frames;
  out.free_frames = cell.free_frames;
  out.last_hit_s = cell.last_hit_s;
  out.last_free_s = cell.last_free_s;
  return out;
}

std::vector<DynamicCluster> MotionLayer::dynamicClusters(std::size_t max_clusters, double now_s) {
  prune(now_s);
  if (current_stamp_s_ >= 0.0 && now_s >= current_stamp_s_ &&
      now_s - current_stamp_s_ > config_.dynamic_track_ttl_s) {
    current_dynamic_keys_.clear();
    cluster_tracks_.clear();
    last_dynamic_clusters_.clear();
    refreshStats();
    return {};
  }
  if (sameFrame(last_cluster_update_s_, current_stamp_s_)) {
    if (max_clusters == 0 || last_dynamic_clusters_.size() <= max_clusters) {
      return last_dynamic_clusters_;
    }
    return std::vector<DynamicCluster>(last_dynamic_clusters_.begin(),
                                       last_dynamic_clusters_.begin() +
                                           static_cast<std::ptrdiff_t>(max_clusters));
  }
  last_cluster_update_s_ = current_stamp_s_;
  current_dynamic_keys_.clear();
  last_dynamic_clusters_.clear();

  std::unordered_map<VoxelKey, const Cell *, VoxelKeyHash> candidate_cells;
  candidate_cells.reserve(current_hit_keys_.size());
  for (const auto &key : current_hit_keys_) {
    const auto it = cells_.find(key);
    if (it == cells_.end()) {
      continue;
    }
    const Cell &cell = it->second;
    if (cell.state == CellState::Occupied && sameFrame(cell.last_hit_s, current_stamp_s_) &&
        cell.free_observations >= config_.dynamic_free_min_frames &&
        cell.height >= config_.dynamic_min_height_m &&
        cell.height <= config_.dynamic_max_height_m) {
      candidate_cells.emplace(key, &cell);
    }
  }

  struct Candidate {
    DynamicCluster cluster;
    std::vector<VoxelKey> keys;
  };
  std::vector<Candidate> candidates;

  std::unordered_set<VoxelKey, VoxelKeyHash> visited;
  visited.reserve(candidate_cells.size());
  std::deque<VoxelKey> queue;
  for (const auto &entry : candidate_cells) {
    const VoxelKey &start = entry.first;
    if (visited.find(start) != visited.end()) {
      continue;
    }
    visited.insert(start);
    queue.clear();
    queue.push_back(start);
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    Candidate candidate;
    while (!queue.empty()) {
      const VoxelKey key = queue.front();
      queue.pop_front();
      const auto cell_it = candidate_cells.find(key);
      if (cell_it == candidate_cells.end()) {
        continue;
      }
      const Cell &cell = *cell_it->second;
      sx += cell.x;
      sy += cell.y;
      sz += cell.z;
      candidate.keys.push_back(key);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            const VoxelKey next{key.x + dx, key.y + dy, key.z + dz};
            if (visited.find(next) != visited.end()) {
              continue;
            }
            if (candidate_cells.find(next) == candidate_cells.end()) {
              continue;
            }
            visited.insert(next);
            queue.push_back(next);
          }
        }
      }
    }
    if (candidate.keys.size() < config_.dynamic_min_cells) {
      continue;
    }
    candidate.cluster.x = sx / static_cast<double>(candidate.keys.size());
    candidate.cluster.y = sy / static_cast<double>(candidate.keys.size());
    candidate.cluster.z = sz / static_cast<double>(candidate.keys.size());
    candidate.cluster.cells = candidate.keys.size();
    candidates.push_back(std::move(candidate));
  }

  std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
    return a.cluster.cells > b.cluster.cells;
  });

  std::vector<bool> matched(cluster_tracks_.size(), false);
  std::vector<ClusterTrack> next_tracks;
  next_tracks.reserve(candidates.size() + cluster_tracks_.size());
  for (auto &candidate : candidates) {
    auto &cluster = candidate.cluster;
    double best_distance = std::numeric_limits<double>::infinity();
    std::size_t best_index = cluster_tracks_.size();
    for (std::size_t i = 0; i < cluster_tracks_.size(); ++i) {
      if (matched[i]) {
        continue;
      }
      const auto &track = cluster_tracks_[i];
      const double predict_dt = std::max(0.0, current_stamp_s_ - track.last_seen_s);
      const double dx = cluster.x - (track.x + track.vx * predict_dt);
      const double dy = cluster.y - (track.y + track.vy * predict_dt);
      const double dz = cluster.z - (track.z + track.vz * predict_dt);
      const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (distance < best_distance) {
        best_distance = distance;
        best_index = i;
      }
    }
    ClusterTrack track;
    if (best_index < cluster_tracks_.size() && best_distance <= config_.dynamic_match_distance_m) {
      const auto &previous = cluster_tracks_[best_index];
      matched[best_index] = true;
      const double dt = std::max(1e-3, current_stamp_s_ - previous.last_seen_s);
      const double raw_vx = (cluster.x - previous.x) / dt;
      const double raw_vy = (cluster.y - previous.y) / dt;
      const double raw_vz = (cluster.z - previous.z) / dt;
      const double raw_speed = std::hypot(raw_vx, raw_vy);
      const double previous_speed = std::hypot(previous.vx, previous.vy);
      const double direction_cos =
          previous_speed > 1e-6 && raw_speed > 1e-6
              ? (previous.vx * raw_vx + previous.vy * raw_vy) / (previous_speed * raw_speed)
              : 1.0;
      const bool plausible_motion =
          raw_speed >= config_.dynamic_min_speed_mps &&
          raw_speed <= config_.dynamic_max_speed_mps &&
          std::abs(raw_vz) <= config_.dynamic_max_z_speed_mps &&
          (previous.moving_frames == 0 || direction_cos >= config_.dynamic_min_dir_cos);
      cluster.id = previous.id;
      cluster.vx = previous.observations > 1 ? 0.5 * previous.vx + 0.5 * raw_vx : raw_vx;
      cluster.vy = previous.observations > 1 ? 0.5 * previous.vy + 0.5 * raw_vy : raw_vy;
      cluster.vz = previous.observations > 1 ? 0.5 * previous.vz + 0.5 * raw_vz : raw_vz;
      cluster.age_s = previous.age_s + dt;
      track.id = previous.id;
      track.age_s = cluster.age_s;
      track.observations = previous.observations + 1;
      track.moving_frames = plausible_motion ? previous.moving_frames + 1 : 0;
      const std::uint32_t required_motion_frames = config_.dynamic_confirm_frames - 1;
      track.confirmed = track.observations >= config_.dynamic_confirm_frames &&
                        track.moving_frames >= required_motion_frames;
    } else {
      cluster.id = next_cluster_id_++;
      cluster.age_s = 0.0;
      track.id = cluster.id;
      track.age_s = 0.0;
      track.observations = 1;
      track.moving_frames = 0;
      track.confirmed = false;
    }
    track.x = cluster.x;
    track.y = cluster.y;
    track.z = cluster.z;
    track.vx = cluster.vx;
    track.vy = cluster.vy;
    track.vz = cluster.vz;
    track.last_seen_s = current_stamp_s_;
    next_tracks.push_back(track);
    if (track.confirmed) {
      cluster.confidence =
          std::min(1.0, 0.5 * static_cast<double>(cluster.cells) /
                                static_cast<double>(config_.dynamic_min_cells) +
                            0.5 * static_cast<double>(track.observations) /
                                static_cast<double>(config_.dynamic_confirm_frames));
      last_dynamic_clusters_.push_back(cluster);
      current_dynamic_keys_.insert(candidate.keys.begin(), candidate.keys.end());
    }
  }
  for (std::size_t i = 0; i < cluster_tracks_.size(); ++i) {
    if (i < matched.size() && matched[i]) {
      continue;
    }
    const auto &track = cluster_tracks_[i];
    if (current_stamp_s_ - track.last_seen_s <= config_.dynamic_track_ttl_s) {
      next_tracks.push_back(track);
    }
  }
  cluster_tracks_.swap(next_tracks);
  std::sort(last_dynamic_clusters_.begin(), last_dynamic_clusters_.end(),
            [](const DynamicCluster &a, const DynamicCluster &b) { return a.cells > b.cells; });
  refreshStats();
  if (max_clusters > 0 && last_dynamic_clusters_.size() > max_clusters) {
    return std::vector<DynamicCluster>(last_dynamic_clusters_.begin(),
                                       last_dynamic_clusters_.begin() +
                                           static_cast<std::ptrdiff_t>(max_clusters));
  }
  return last_dynamic_clusters_;
}

void MotionLayer::clear() {
  cells_.clear();
  current_hit_keys_.clear();
  current_dynamic_keys_.clear();
  cluster_tracks_.clear();
  last_dynamic_clusters_.clear();
  last_ray_clearing_s_ = -1.0;
  last_prune_s_ = -1.0;
  current_stamp_s_ = -1.0;
  last_cluster_update_s_ = -1.0;
  next_cluster_id_ = 1;
  last_sensor_origin_ = SensorOrigin{};
  stats_ = MotionLayerStats{};
}

std::size_t MotionLayer::size() const {
  return stats_.obstacle_cells;
}

MotionLayerStats MotionLayer::stats() const {
  return stats_;
}

void MotionLayer::refreshStats() {
  const std::size_t raycast_rays = stats_.raycast_rays;
  const std::size_t raycast_voxels = stats_.raycast_voxels;
  const std::size_t ray_cleared_cells = stats_.ray_cleared_cells;
  const std::size_t prune_passes = stats_.prune_passes;
  stats_ = MotionLayerStats{};
  stats_.raycast_rays = raycast_rays;
  stats_.raycast_voxels = raycast_voxels;
  stats_.ray_cleared_cells = ray_cleared_cells;
  stats_.prune_passes = prune_passes;
  stats_.cells = cells_.size();
  stats_.dynamic_cells = current_dynamic_keys_.size();
  for (const auto &entry : cells_) {
    switch (entry.second.state) {
      case CellState::Free:
        ++stats_.free_cells;
        break;
      case CellState::Occupied:
        ++stats_.occupied_cells;
        ++stats_.obstacle_cells;
        break;
      case CellState::Static:
        ++stats_.static_cells;
        ++stats_.obstacle_cells;
        break;
      case CellState::Cleared:
        ++stats_.cleared_cells;
        break;
      case CellState::Unknown:
        ++stats_.unknown_cells;
        break;
    }
  }
}

}  // namespace lingtu::nav::endpoint
