#include "lingtu/maps/layers/semantic_occupancy.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace lingtu::maps::layers {

SemanticGenerationMismatch::SemanticGenerationMismatch(std::uint64_t expected_generation,
                                                       std::uint64_t actual_generation)
    : std::runtime_error("semantic map generation mismatch: expected " +
                         std::to_string(expected_generation) + ", actual " +
                         std::to_string(actual_generation)),
      expected_generation_(expected_generation),
      actual_generation_(actual_generation) {}

namespace {

constexpr std::size_t kSemanticBinCount = 4U;

bool IsFinite(float value) {
  return std::isfinite(static_cast<double>(value));
}

float ProbabilityFromLogOdds(float log_odds) {
  return 1.0F / (1.0F + std::exp(-log_odds));
}

std::uint64_t Mix(std::uint64_t value) {
  value ^= value >> 33U;
  value *= 0xff51afd7ed558ccdULL;
  value ^= value >> 33U;
  value *= 0xc4ceb9fe1a85ec53ULL;
  value ^= value >> 33U;
  return value;
}

struct VoxelKey {
  std::int32_t x{0};
  std::int32_t y{0};
  std::int32_t z{0};

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const {
    const auto x = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.x));
    const auto y = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.y));
    const auto z = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.z));
    return static_cast<std::size_t>(Mix(x) ^ (Mix(y) << 1U) ^ (Mix(z) << 2U));
  }
};

VoxelKey ToKey(float x_m, float y_m, float z_m, float voxel_size_m) {
  const auto quantize = [voxel_size_m](float value) {
    const double scaled =
        std::floor(static_cast<double>(value) / static_cast<double>(voxel_size_m));
    const double min_index = static_cast<double>(std::numeric_limits<std::int32_t>::min());
    const double max_index = static_cast<double>(std::numeric_limits<std::int32_t>::max());
    if (!std::isfinite(scaled) || scaled < min_index || scaled > max_index) {
      throw std::out_of_range("semantic occupancy coordinate exceeds voxel index range");
    }
    return static_cast<std::int32_t>(scaled);
  };
  return {quantize(x_m), quantize(y_m), quantize(z_m)};
}

std::size_t IndexDistance(std::int32_t lhs, std::int32_t rhs) {
  const std::int64_t delta = static_cast<std::int64_t>(lhs) - static_cast<std::int64_t>(rhs);
  return static_cast<std::size_t>(delta < 0 ? -delta : delta);
}

bool ReadPoint(const PointCloudView &cloud, std::size_t index, float *x_m, float *y_m, float *z_m) {
  if (x_m == nullptr || y_m == nullptr || z_m == nullptr) {
    return false;
  }
  switch (cloud.layout) {
    case CloudLayout::kXyzF32Interleaved: {
      const std::size_t base = index * 3U;
      if (cloud.interleaved.data == nullptr || base + 2U >= cloud.interleaved.size) {
        return false;
      }
      *x_m = cloud.interleaved.data[base];
      *y_m = cloud.interleaved.data[base + 1U];
      *z_m = cloud.interleaved.data[base + 2U];
      return true;
    }
    case CloudLayout::kXyziF32Interleaved: {
      const std::size_t base = index * 4U;
      if (cloud.interleaved.data == nullptr || base + 2U >= cloud.interleaved.size) {
        return false;
      }
      *x_m = cloud.interleaved.data[base];
      *y_m = cloud.interleaved.data[base + 1U];
      *z_m = cloud.interleaved.data[base + 2U];
      return true;
    }
    case CloudLayout::kXyzF32SoA:
    case CloudLayout::kXyziF32SoA:
      if (cloud.x.data == nullptr || cloud.y.data == nullptr || cloud.z.data == nullptr ||
          index >= cloud.x.size || index >= cloud.y.size || index >= cloud.z.size) {
        return false;
      }
      *x_m = cloud.x.data[index];
      *y_m = cloud.y.data[index];
      *z_m = cloud.z.data[index];
      return true;
  }
  return false;
}

struct ObservedPoint {
  float x_m{0.0F};
  float y_m{0.0F};
  float z_m{0.0F};
  std::uint16_t label{0U};
  VoxelKey key;
};

struct SemanticBin {
  std::uint16_t label{0U};
  std::uint32_t count{0U};
};

struct VoxelCell {
  float log_odds{0.0F};
  std::uint32_t hit_count{0U};
  std::uint32_t miss_count{0U};
  std::uint32_t point_count{0U};
  std::array<float, 3U> mean{};
  std::array<float, 6U> m2{};
  std::array<SemanticBin, kSemanticBinCount> semantic_bins{};
  std::uint32_t semantic_observations{0U};
  std::uint64_t last_update_sequence{0U};
};

std::shared_ptr<const SemanticMapChunkSoA> BuildChunkData(
    const std::vector<SemanticVoxel> &voxels) {
  auto data = std::make_shared<SemanticMapChunkSoA>();
  const std::size_t size = voxels.size();
#define LINGTU_RESERVE_SEMANTIC_FIELD(field) data->field.reserve(size)
  LINGTU_RESERVE_SEMANTIC_FIELD(index_x);
  LINGTU_RESERVE_SEMANTIC_FIELD(index_y);
  LINGTU_RESERVE_SEMANTIC_FIELD(index_z);
  LINGTU_RESERVE_SEMANTIC_FIELD(center_x_m);
  LINGTU_RESERVE_SEMANTIC_FIELD(center_y_m);
  LINGTU_RESERVE_SEMANTIC_FIELD(center_z_m);
  LINGTU_RESERVE_SEMANTIC_FIELD(occupancy_probability);
  LINGTU_RESERVE_SEMANTIC_FIELD(hit_count);
  LINGTU_RESERVE_SEMANTIC_FIELD(miss_count);
  LINGTU_RESERVE_SEMANTIC_FIELD(point_count);
  LINGTU_RESERVE_SEMANTIC_FIELD(mean_x_m);
  LINGTU_RESERVE_SEMANTIC_FIELD(mean_y_m);
  LINGTU_RESERVE_SEMANTIC_FIELD(mean_z_m);
  LINGTU_RESERVE_SEMANTIC_FIELD(covariance_xx);
  LINGTU_RESERVE_SEMANTIC_FIELD(covariance_xy);
  LINGTU_RESERVE_SEMANTIC_FIELD(covariance_xz);
  LINGTU_RESERVE_SEMANTIC_FIELD(covariance_yy);
  LINGTU_RESERVE_SEMANTIC_FIELD(covariance_yz);
  LINGTU_RESERVE_SEMANTIC_FIELD(covariance_zz);
  LINGTU_RESERVE_SEMANTIC_FIELD(dominant_label);
  LINGTU_RESERVE_SEMANTIC_FIELD(semantic_confidence);
#undef LINGTU_RESERVE_SEMANTIC_FIELD

  for (const auto &voxel : voxels) {
    data->index_x.push_back(voxel.index_x);
    data->index_y.push_back(voxel.index_y);
    data->index_z.push_back(voxel.index_z);
    data->center_x_m.push_back(voxel.center_x_m);
    data->center_y_m.push_back(voxel.center_y_m);
    data->center_z_m.push_back(voxel.center_z_m);
    data->occupancy_probability.push_back(voxel.occupancy_probability);
    data->hit_count.push_back(voxel.hit_count);
    data->miss_count.push_back(voxel.miss_count);
    data->point_count.push_back(voxel.point_count);
    data->mean_x_m.push_back(voxel.mean_x_m);
    data->mean_y_m.push_back(voxel.mean_y_m);
    data->mean_z_m.push_back(voxel.mean_z_m);
    data->covariance_xx.push_back(voxel.covariance_xx);
    data->covariance_xy.push_back(voxel.covariance_xy);
    data->covariance_xz.push_back(voxel.covariance_xz);
    data->covariance_yy.push_back(voxel.covariance_yy);
    data->covariance_yz.push_back(voxel.covariance_yz);
    data->covariance_zz.push_back(voxel.covariance_zz);
    data->dominant_label.push_back(voxel.dominant_label);
    data->semantic_confidence.push_back(voxel.semantic_confidence);
  }
  return data;
}

void ResetGeometry(VoxelCell *cell) {
  cell->point_count = 0U;
  cell->mean = {};
  cell->m2 = {};
  cell->semantic_bins = {};
  cell->semantic_observations = 0U;
}

void UpdateGeometry(VoxelCell *cell, const ObservedPoint &point) {
  ++cell->point_count;
  const float count = static_cast<float>(cell->point_count);
  const float dx = point.x_m - cell->mean[0U];
  const float dy = point.y_m - cell->mean[1U];
  const float dz = point.z_m - cell->mean[2U];
  cell->mean[0U] += dx / count;
  cell->mean[1U] += dy / count;
  cell->mean[2U] += dz / count;
  const float dx2 = point.x_m - cell->mean[0U];
  const float dy2 = point.y_m - cell->mean[1U];
  const float dz2 = point.z_m - cell->mean[2U];
  cell->m2[0U] += dx * dx2;
  cell->m2[1U] += dx * dy2;
  cell->m2[2U] += dx * dz2;
  cell->m2[3U] += dy * dy2;
  cell->m2[4U] += dy * dz2;
  cell->m2[5U] += dz * dz2;
}

void UpdateSemantic(VoxelCell *cell, std::uint16_t label, std::uint16_t unknown_label) {
  if (label == unknown_label) {
    return;
  }
  ++cell->semantic_observations;
  for (auto &bin : cell->semantic_bins) {
    if (bin.count > 0U && bin.label == label) {
      ++bin.count;
      return;
    }
  }
  for (auto &bin : cell->semantic_bins) {
    if (bin.count == 0U) {
      bin.label = label;
      bin.count = 1U;
      return;
    }
  }
  for (auto &bin : cell->semantic_bins) {
    --bin.count;
    if (bin.count == 0U) {
      bin.label = unknown_label;
    }
  }
}

template <typename Fn>
void TraceFreeVoxels(float origin_x_m, float origin_y_m, float origin_z_m,
                     const ObservedPoint &endpoint, float voxel_size_m, std::size_t max_voxels,
                     Fn &&visit) {
  VoxelKey current = ToKey(origin_x_m, origin_y_m, origin_z_m, voxel_size_m);
  const VoxelKey target = endpoint.key;
  if (current == target) {
    return;
  }

  const float dx = endpoint.x_m - origin_x_m;
  const float dy = endpoint.y_m - origin_y_m;
  const float dz = endpoint.z_m - origin_z_m;
  const auto axis = [voxel_size_m](float origin, float delta, std::int32_t index) {
    struct AxisState {
      std::int32_t step;
      float t_max;
      float t_delta;
    };
    if (delta > 0.0F) {
      const float boundary = (static_cast<float>(index) + 1.0F) * voxel_size_m;
      return AxisState{1, (boundary - origin) / delta, voxel_size_m / delta};
    }
    if (delta < 0.0F) {
      const float boundary = static_cast<float>(index) * voxel_size_m;
      return AxisState{-1, (boundary - origin) / delta, -voxel_size_m / delta};
    }
    return AxisState{0, std::numeric_limits<float>::infinity(),
                     std::numeric_limits<float>::infinity()};
  };

  auto x = axis(origin_x_m, dx, current.x);
  auto y = axis(origin_y_m, dy, current.y);
  auto z = axis(origin_z_m, dz, current.z);
  const std::size_t required_steps = IndexDistance(target.x, current.x) +
                                     IndexDistance(target.y, current.y) +
                                     IndexDistance(target.z, current.z) + 3U;
  const std::size_t max_steps = std::min(required_steps, max_voxels);

  for (std::size_t step = 0U; step < max_steps && !(current == target); ++step) {
    visit(current);
    const float next_t = std::min(x.t_max, std::min(y.t_max, z.t_max));
    constexpr float kTieTolerance = 1e-6F;
    if (x.t_max <= next_t + kTieTolerance) {
      current.x += x.step;
      x.t_max += x.t_delta;
    }
    if (y.t_max <= next_t + kTieTolerance) {
      current.y += y.step;
      y.t_max += y.t_delta;
    }
    if (z.t_max <= next_t + kTieTolerance) {
      current.z += z.step;
      z.t_max += z.t_delta;
    }
  }
}

}  // namespace

struct SemanticOccupancyLayerCore::Impl {
  explicit Impl(SemanticOccupancyConfig in_config) : config(std::move(in_config)) {}

  SemanticOccupancyConfig config;
  mutable std::shared_mutex mutex;
  std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> voxels;
  SemanticOccupancyUpdateStats last_stats;
  std::string frame_id;
  std::string taxonomy;
  std::uint32_t taxonomy_version{0U};
  std::uint64_t update_sequence{0U};
  std::uint64_t last_observation_sequence{0U};
  std::uint64_t generation{0U};

  SemanticVoxel Export(const VoxelKey &key, const VoxelCell &cell) const {
    SemanticVoxel out;
    out.index_x = key.x;
    out.index_y = key.y;
    out.index_z = key.z;
    out.center_x_m = (static_cast<float>(key.x) + 0.5F) * config.voxel_size_m;
    out.center_y_m = (static_cast<float>(key.y) + 0.5F) * config.voxel_size_m;
    out.center_z_m = (static_cast<float>(key.z) + 0.5F) * config.voxel_size_m;
    out.occupancy_probability = ProbabilityFromLogOdds(cell.log_odds);
    out.occupied = out.occupancy_probability > config.occupied_probability;
    out.hit_count = cell.hit_count;
    out.miss_count = cell.miss_count;
    out.point_count = cell.point_count;
    out.mean_x_m = cell.mean[0U];
    out.mean_y_m = cell.mean[1U];
    out.mean_z_m = cell.mean[2U];
    if (cell.point_count > 1U) {
      const float denominator = static_cast<float>(cell.point_count - 1U);
      out.covariance_xx = cell.m2[0U] / denominator;
      out.covariance_xy = cell.m2[1U] / denominator;
      out.covariance_xz = cell.m2[2U] / denominator;
      out.covariance_yy = cell.m2[3U] / denominator;
      out.covariance_yz = cell.m2[4U] / denominator;
      out.covariance_zz = cell.m2[5U] / denominator;
    }
    const SemanticBin *dominant = nullptr;
    for (const auto &bin : cell.semantic_bins) {
      if (bin.count > 0U && (dominant == nullptr || bin.count > dominant->count)) {
        dominant = &bin;
      }
    }
    if (dominant != nullptr && cell.semantic_observations > 0U) {
      out.dominant_label = dominant->label;
      out.semantic_confidence =
          static_cast<float>(dominant->count) / static_cast<float>(cell.semantic_observations);
    }
    return out;
  }

  std::size_t EnforceLimit() {
    if (voxels.size() <= config.max_voxels) {
      return 0U;
    }
    std::vector<VoxelKey> order;
    order.reserve(voxels.size());
    for (const auto &item : voxels) {
      order.push_back(item.first);
    }
    std::sort(order.begin(), order.end(), [this](const VoxelKey &lhs, const VoxelKey &rhs) {
      const auto &a = voxels.at(lhs);
      const auto &b = voxels.at(rhs);
      const bool a_occupied = ProbabilityFromLogOdds(a.log_odds) > config.occupied_probability;
      const bool b_occupied = ProbabilityFromLogOdds(b.log_odds) > config.occupied_probability;
      if (a_occupied != b_occupied) {
        return a_occupied;
      }
      if (a.last_update_sequence != b.last_update_sequence) {
        return a.last_update_sequence > b.last_update_sequence;
      }
      const float a_strength = std::fabs(a.log_odds);
      const float b_strength = std::fabs(b.log_odds);
      if (a_strength != b_strength) {
        return a_strength > b_strength;
      }
      if (lhs.x != rhs.x)
        return lhs.x < rhs.x;
      if (lhs.y != rhs.y)
        return lhs.y < rhs.y;
      return lhs.z < rhs.z;
    });
    const std::size_t remove_count = order.size() - config.max_voxels;
    for (std::size_t i = config.max_voxels; i < order.size(); ++i) {
      voxels.erase(order[i]);
    }
    return remove_count;
  }
};

SemanticOccupancyLayerCore::SemanticOccupancyLayerCore(SemanticOccupancyConfig config)
    : impl_(std::make_unique<Impl>(config)) {
  if (!(config.voxel_size_m > 0.0F) || !IsFinite(config.voxel_size_m)) {
    throw std::invalid_argument("SemanticOccupancyConfig.voxel_size_m must be finite and > 0");
  }
  if (config.max_range_m < 0.0F || !IsFinite(config.max_range_m)) {
    throw std::invalid_argument("SemanticOccupancyConfig.max_range_m must be finite and >= 0");
  }
  if (!IsFinite(config.min_z_m) || !IsFinite(config.max_z_m) || config.min_z_m > config.max_z_m) {
    throw std::invalid_argument("SemanticOccupancyConfig z range is invalid");
  }
  if (!(config.hit_log_odds > 0.0F) || !(config.miss_log_odds < 0.0F) ||
      !IsFinite(config.hit_log_odds) || !IsFinite(config.miss_log_odds)) {
    throw std::invalid_argument("SemanticOccupancyConfig hit/miss log odds are invalid");
  }
  if (!IsFinite(config.min_log_odds) || !IsFinite(config.max_log_odds) ||
      !(config.min_log_odds < 0.0F) || !(config.max_log_odds > 0.0F) ||
      config.min_log_odds >= config.max_log_odds) {
    throw std::invalid_argument("SemanticOccupancyConfig log odds bounds are invalid");
  }
  if (!(config.occupied_probability > 0.0F && config.occupied_probability < 1.0F) ||
      !IsFinite(config.occupied_probability)) {
    throw std::invalid_argument("SemanticOccupancyConfig.occupied_probability must be in (0, 1)");
  }
  if (config.max_rays_per_update == 0U || config.max_ray_voxels_per_ray == 0U ||
      config.max_voxels == 0U || config.max_query_voxel_checks == 0U ||
      config.max_query_results == 0U) {
    throw std::invalid_argument("SemanticOccupancyConfig limits must be positive");
  }
}

SemanticOccupancyLayerCore::~SemanticOccupancyLayerCore() = default;

void SemanticOccupancyLayerCore::Reset() {
  std::unique_lock<std::shared_mutex> lock(impl_->mutex);
  impl_->voxels.clear();
  impl_->last_stats = {};
  impl_->frame_id.clear();
  impl_->taxonomy.clear();
  impl_->taxonomy_version = 0U;
  impl_->last_observation_sequence = 0U;
  impl_->update_sequence = 0U;
  if (impl_->generation == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("semantic map generation exhausted");
  }
  ++impl_->generation;
  impl_->last_stats.generation_before = impl_->generation - 1U;
  impl_->last_stats.generation_after = impl_->generation;
  impl_->last_stats.applied = true;
}

SemanticOccupancyUpdateStats
SemanticOccupancyLayerCore::Update(const SemanticObservationFrame &observation) {
  const PointCloudView &cloud = observation.frame.cloud;
  const std::string incoming_frame = cloud.frame_id.empty() ? "map" : cloud.frame_id;
  const bool has_labels = observation.labels.size > 0U;
  if (has_labels &&
      (observation.labels.data == nullptr || observation.labels.size != cloud.point_count)) {
    throw std::invalid_argument("semantic label count must equal point count or be empty");
  }
  if (has_labels &&
      (observation.labels.stamp_ns != cloud.stamp_ns ||
       observation.labels.frame_id != incoming_frame || observation.labels.taxonomy.empty() ||
       observation.labels.taxonomy_version == 0U)) {
    throw std::invalid_argument(
        "semantic labels require matching stamp/frame and a versioned taxonomy");
  }

  SemanticOccupancyUpdateStats stats;
  stats.input_points = cloud.point_count;
  stats.replaced_full_map = observation.frame.full_map;
  std::vector<ObservedPoint> points;
  points.reserve(cloud.point_count);
  const float max_range_sq = impl_->config.max_range_m * impl_->config.max_range_m;
  const bool check_range = impl_->config.max_range_m > 0.0F && !observation.frame.full_map;
  const bool needs_sensor_origin =
      check_range || (impl_->config.raycast_free_space && !observation.frame.full_map);
  if (needs_sensor_origin && (!IsFinite(observation.frame.sensor_origin_x_m) ||
                              !IsFinite(observation.frame.sensor_origin_y_m) ||
                              !IsFinite(observation.frame.sensor_origin_z_m))) {
    throw std::invalid_argument("semantic occupancy sensor origin must be finite");
  }
  for (std::size_t i = 0U; i < cloud.point_count; ++i) {
    float x_m = 0.0F;
    float y_m = 0.0F;
    float z_m = 0.0F;
    if (!ReadPoint(cloud, i, &x_m, &y_m, &z_m) || !IsFinite(x_m) || !IsFinite(y_m) ||
        !IsFinite(z_m)) {
      continue;
    }
    if (z_m < impl_->config.min_z_m || z_m > impl_->config.max_z_m) {
      continue;
    }
    if (check_range) {
      const float dx = x_m - observation.frame.sensor_origin_x_m;
      const float dy = y_m - observation.frame.sensor_origin_y_m;
      const float dz = z_m - observation.frame.sensor_origin_z_m;
      if (dx * dx + dy * dy + dz * dz > max_range_sq) {
        continue;
      }
    }
    const std::uint16_t label =
        observation.labels.size == 0U ? impl_->config.unknown_label : observation.labels.data[i];
    points.push_back({x_m, y_m, z_m, label, ToKey(x_m, y_m, z_m, impl_->config.voxel_size_m)});
  }
  stats.accepted_points = points.size();

  std::unique_lock<std::shared_mutex> lock(impl_->mutex);
  stats.generation_before = impl_->generation;
  stats.generation_after = impl_->generation;
  if (observation.expected_generation != kAnySemanticMapGeneration &&
      observation.expected_generation != impl_->generation) {
    throw SemanticGenerationMismatch(observation.expected_generation, impl_->generation);
  }
  if (observation.sequence != 0U && impl_->last_observation_sequence != 0U &&
      observation.sequence <= impl_->last_observation_sequence) {
    stats.duplicate_sequence = observation.sequence == impl_->last_observation_sequence;
    stats.stale_sequence = observation.sequence < impl_->last_observation_sequence;
    stats.total_voxels = impl_->voxels.size();
    impl_->last_stats = stats;
    return stats;
  }
  if (impl_->generation == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("semantic map generation exhausted");
  }
  if (observation.frame.full_map) {
    impl_->voxels.clear();
    impl_->frame_id.clear();
    impl_->taxonomy.clear();
    impl_->taxonomy_version = 0U;
  }
  if (!observation.frame.full_map && !impl_->frame_id.empty() &&
      impl_->frame_id != incoming_frame) {
    throw std::invalid_argument("semantic occupancy frame mismatch: expected " + impl_->frame_id +
                                ", got " + incoming_frame);
  }
  if (!observation.frame.full_map && has_labels && !impl_->taxonomy.empty() &&
      (impl_->taxonomy != observation.labels.taxonomy ||
       impl_->taxonomy_version != observation.labels.taxonomy_version)) {
    throw std::invalid_argument("semantic occupancy taxonomy changed during incremental update");
  }
  impl_->frame_id = incoming_frame;
  if (has_labels && impl_->taxonomy.empty()) {
    impl_->taxonomy = observation.labels.taxonomy;
    impl_->taxonomy_version = observation.labels.taxonomy_version;
  }
  const std::uint64_t sequence = ++impl_->update_sequence;

  std::unordered_set<VoxelKey, VoxelKeyHash> hit_keys;
  hit_keys.reserve(points.size());
  for (const auto &point : points) {
    hit_keys.insert(point.key);
  }
  stats.hit_voxels = hit_keys.size();

  if (impl_->config.raycast_free_space && !observation.frame.full_map && !points.empty()) {
    const std::size_t stride =
        std::max<std::size_t>(1U, (points.size() + impl_->config.max_rays_per_update - 1U) /
                                      impl_->config.max_rays_per_update);
    std::unordered_set<VoxelKey, VoxelKeyHash> free_keys;
    free_keys.reserve(std::min(points.size(), impl_->config.max_voxels));
    for (std::size_t i = 0U; i < points.size(); i += stride) {
      ++stats.rays_traced;
      const VoxelKey origin_key =
          ToKey(observation.frame.sensor_origin_x_m, observation.frame.sensor_origin_y_m,
                observation.frame.sensor_origin_z_m, impl_->config.voxel_size_m);
      const std::size_t required_steps = IndexDistance(points[i].key.x, origin_key.x) +
                                         IndexDistance(points[i].key.y, origin_key.y) +
                                         IndexDistance(points[i].key.z, origin_key.z) + 3U;
      if (required_steps > impl_->config.max_ray_voxels_per_ray) {
        ++stats.truncated_rays;
      }
      TraceFreeVoxels(observation.frame.sensor_origin_x_m, observation.frame.sensor_origin_y_m,
                      observation.frame.sensor_origin_z_m, points[i], impl_->config.voxel_size_m,
                      impl_->config.max_ray_voxels_per_ray, [&](const VoxelKey &key) {
                        if (hit_keys.find(key) != hit_keys.end()) {
                          return;
                        }
                        if (free_keys.size() < impl_->config.max_voxels) {
                          free_keys.insert(key);
                        }
                      });
    }
    for (const VoxelKey &key : free_keys) {
      auto &cell = impl_->voxels[key];
      const float previous_log_odds = cell.log_odds;
      cell.log_odds =
          std::max(impl_->config.min_log_odds, cell.log_odds + impl_->config.miss_log_odds);
      ++cell.miss_count;
      cell.last_update_sequence = sequence;
      if (previous_log_odds > 0.0F && cell.log_odds <= 0.0F) {
        ResetGeometry(&cell);
      }
    }
    stats.free_voxel_updates = free_keys.size();
  }

  std::unordered_set<VoxelKey, VoxelKeyHash> evidence_applied;
  evidence_applied.reserve(hit_keys.size());
  for (const auto &point : points) {
    auto &cell = impl_->voxels[point.key];
    if (evidence_applied.insert(point.key).second) {
      cell.log_odds =
          std::min(impl_->config.max_log_odds, cell.log_odds + impl_->config.hit_log_odds);
      ++cell.hit_count;
    }
    cell.last_update_sequence = sequence;
    UpdateGeometry(&cell, point);
    UpdateSemantic(&cell, point.label, impl_->config.unknown_label);
  }

  stats.pruned_voxels = impl_->EnforceLimit();
  stats.total_voxels = impl_->voxels.size();
  if (observation.sequence != 0U) {
    impl_->last_observation_sequence = observation.sequence;
  }
  ++impl_->generation;
  stats.applied = true;
  stats.generation_after = impl_->generation;
  impl_->last_stats = stats;
  return stats;
}

SemanticOccupancyUpdateStats SemanticOccupancyLayerCore::Replace(
    const SemanticMapChunk &chunk, std::uint64_t expected_generation) {
  if (chunk.data == nullptr || !chunk.complete || chunk.offset != 0U ||
      chunk.total_voxels != chunk.Size() || chunk.generation == 0U ||
      chunk.frame_id.empty() || !(chunk.voxel_size_m > 0.0F) ||
      !IsFinite(chunk.voxel_size_m)) {
    throw std::invalid_argument("semantic map replacement chunk is incomplete or invalid");
  }
  if (std::fabs(chunk.voxel_size_m - impl_->config.voxel_size_m) > 1e-6F) {
    throw std::invalid_argument("semantic map replacement voxel size does not match core config");
  }
  if (chunk.taxonomy.empty() != (chunk.taxonomy_version == 0U)) {
    throw std::invalid_argument("semantic map replacement taxonomy is inconsistent");
  }
  const auto &data = *chunk.data;
  const std::size_t count = chunk.Size();
#define LINGTU_REQUIRE_REPLACEMENT_FIELD(name) \
  if (data.name.size() != count) { \
    throw std::invalid_argument("semantic map replacement field length mismatch: " #name); \
  }
  LINGTU_REQUIRE_REPLACEMENT_FIELD(index_x)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(index_y)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(index_z)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(center_x_m)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(center_y_m)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(center_z_m)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(occupancy_probability)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(hit_count)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(miss_count)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(point_count)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(mean_x_m)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(mean_y_m)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(mean_z_m)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(covariance_xx)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(covariance_xy)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(covariance_xz)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(covariance_yy)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(covariance_yz)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(covariance_zz)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(dominant_label)
  LINGTU_REQUIRE_REPLACEMENT_FIELD(semantic_confidence)
#undef LINGTU_REQUIRE_REPLACEMENT_FIELD

  std::unordered_map<VoxelKey, VoxelCell, VoxelKeyHash> staged;
  staged.reserve(count);
  for (std::size_t i = 0U; i < count; ++i) {
    const float probability = data.occupancy_probability[i];
    const float confidence = data.semantic_confidence[i];
    if (!IsFinite(probability) || probability <= 0.0F || probability >= 1.0F ||
        !IsFinite(confidence) || confidence < 0.0F || confidence > 1.0F ||
        !IsFinite(data.mean_x_m[i]) || !IsFinite(data.mean_y_m[i]) ||
        !IsFinite(data.mean_z_m[i]) || !IsFinite(data.covariance_xx[i]) ||
        !IsFinite(data.covariance_xy[i]) || !IsFinite(data.covariance_xz[i]) ||
        !IsFinite(data.covariance_yy[i]) || !IsFinite(data.covariance_yz[i]) ||
        !IsFinite(data.covariance_zz[i]) || data.covariance_xx[i] < 0.0F ||
        data.covariance_yy[i] < 0.0F || data.covariance_zz[i] < 0.0F) {
      throw std::invalid_argument("semantic map replacement contains invalid voxel values");
    }
    if (data.dominant_label[i] != impl_->config.unknown_label && chunk.taxonomy.empty()) {
      throw std::invalid_argument("semantic map labels require a versioned taxonomy");
    }
    VoxelCell cell;
    cell.log_odds = std::clamp(
        std::log(probability / (1.0F - probability)), impl_->config.min_log_odds,
        impl_->config.max_log_odds);
    cell.hit_count = data.hit_count[i];
    cell.miss_count = data.miss_count[i];
    cell.point_count = data.point_count[i];
    cell.mean = {data.mean_x_m[i], data.mean_y_m[i], data.mean_z_m[i]};
    const float covariance_scale =
        cell.point_count > 1U ? static_cast<float>(cell.point_count - 1U) : 0.0F;
    cell.m2 = {
        data.covariance_xx[i] * covariance_scale,
        data.covariance_xy[i] * covariance_scale,
        data.covariance_xz[i] * covariance_scale,
        data.covariance_yy[i] * covariance_scale,
        data.covariance_yz[i] * covariance_scale,
        data.covariance_zz[i] * covariance_scale,
    };
    if (data.dominant_label[i] != impl_->config.unknown_label && confidence > 0.0F) {
      cell.semantic_observations = std::max<std::uint32_t>(1U, cell.point_count);
      cell.semantic_bins[0U].label = data.dominant_label[i];
      cell.semantic_bins[0U].count = std::max<std::uint32_t>(
          1U, static_cast<std::uint32_t>(std::round(
                  confidence * static_cast<float>(cell.semantic_observations))));
    }
    const VoxelKey key{data.index_x[i], data.index_y[i], data.index_z[i]};
    if (!staged.emplace(key, std::move(cell)).second) {
      throw std::invalid_argument("semantic map replacement contains duplicate voxel keys");
    }
  }

  std::unique_lock<std::shared_mutex> lock(impl_->mutex);
  if (expected_generation != kAnySemanticMapGeneration &&
      expected_generation != impl_->generation) {
    throw SemanticGenerationMismatch(expected_generation, impl_->generation);
  }
  if (impl_->generation == std::numeric_limits<std::uint64_t>::max()) {
    throw std::overflow_error("semantic map generation exhausted");
  }
  SemanticOccupancyUpdateStats stats;
  stats.generation_before = impl_->generation;
  impl_->voxels.swap(staged);
  impl_->frame_id = chunk.frame_id;
  impl_->taxonomy = chunk.taxonomy;
  impl_->taxonomy_version = chunk.taxonomy_version;
  impl_->last_observation_sequence = 0U;
  impl_->update_sequence = 0U;
  impl_->generation = std::max(chunk.generation, impl_->generation + 1U);
  stats.generation_after = impl_->generation;
  stats.total_voxels = impl_->voxels.size();
  stats.replaced_full_map = true;
  stats.applied = true;
  impl_->last_stats = stats;
  return stats;
}

std::optional<SemanticVoxel> SemanticOccupancyLayerCore::Lookup(float x_m, float y_m,
                                                                float z_m) const {
  if (!IsFinite(x_m) || !IsFinite(y_m) || !IsFinite(z_m)) {
    return std::nullopt;
  }
  const VoxelKey key = ToKey(x_m, y_m, z_m, impl_->config.voxel_size_m);
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  const auto it = impl_->voxels.find(key);
  if (it == impl_->voxels.end()) {
    return std::nullopt;
  }
  return impl_->Export(key, it->second);
}

std::vector<SemanticVoxel>
SemanticOccupancyLayerCore::QueryRadius(float center_x_m, float center_y_m, float center_z_m,
                                        float radius_m, float min_occupancy_probability) const {
  const SemanticMapChunk chunk = QueryRadiusChunk(center_x_m, center_y_m, center_z_m, radius_m,
                                                   min_occupancy_probability);
  std::vector<SemanticVoxel> result;
  result.reserve(chunk.Size());
  if (chunk.data == nullptr) {
    return result;
  }
  const auto &data = *chunk.data;
  for (std::size_t i = 0U; i < chunk.Size(); ++i) {
    SemanticVoxel voxel;
    voxel.index_x = data.index_x[i];
    voxel.index_y = data.index_y[i];
    voxel.index_z = data.index_z[i];
    voxel.center_x_m = data.center_x_m[i];
    voxel.center_y_m = data.center_y_m[i];
    voxel.center_z_m = data.center_z_m[i];
    voxel.occupancy_probability = data.occupancy_probability[i];
    voxel.occupied = voxel.occupancy_probability > impl_->config.occupied_probability;
    voxel.hit_count = data.hit_count[i];
    voxel.miss_count = data.miss_count[i];
    voxel.point_count = data.point_count[i];
    voxel.mean_x_m = data.mean_x_m[i];
    voxel.mean_y_m = data.mean_y_m[i];
    voxel.mean_z_m = data.mean_z_m[i];
    voxel.covariance_xx = data.covariance_xx[i];
    voxel.covariance_xy = data.covariance_xy[i];
    voxel.covariance_xz = data.covariance_xz[i];
    voxel.covariance_yy = data.covariance_yy[i];
    voxel.covariance_yz = data.covariance_yz[i];
    voxel.covariance_zz = data.covariance_zz[i];
    voxel.dominant_label = data.dominant_label[i];
    voxel.semantic_confidence = data.semantic_confidence[i];
    result.push_back(voxel);
  }
  return result;
}

SemanticMapChunk SemanticOccupancyLayerCore::QueryRadiusChunk(
    float center_x_m, float center_y_m, float center_z_m, float radius_m,
    float min_occupancy_probability) const {
  if (!IsFinite(center_x_m) || !IsFinite(center_y_m) || !IsFinite(center_z_m) ||
      !IsFinite(radius_m) || radius_m < 0.0F || !IsFinite(min_occupancy_probability) ||
      min_occupancy_probability < 0.0F || min_occupancy_probability > 1.0F) {
    throw std::invalid_argument("semantic occupancy query parameters are invalid");
  }
  const VoxelKey min_key = ToKey(center_x_m - radius_m, center_y_m - radius_m,
                                 center_z_m - radius_m, impl_->config.voxel_size_m);
  const VoxelKey max_key = ToKey(center_x_m + radius_m, center_y_m + radius_m,
                                 center_z_m + radius_m, impl_->config.voxel_size_m);
  const std::uint64_t span_x =
      static_cast<std::uint64_t>(static_cast<std::int64_t>(max_key.x) - min_key.x + 1);
  const std::uint64_t span_y =
      static_cast<std::uint64_t>(static_cast<std::int64_t>(max_key.y) - min_key.y + 1);
  const std::uint64_t span_z =
      static_cast<std::uint64_t>(static_cast<std::int64_t>(max_key.z) - min_key.z + 1);
  const std::uint64_t query_limit = impl_->config.max_query_voxel_checks;
  if (span_x > query_limit || span_y > query_limit / span_x ||
      span_z > query_limit / (span_x * span_y)) {
    throw std::length_error("semantic occupancy query exceeds configured voxel-check limit");
  }
  const float radius_sq = radius_m * radius_m;
  std::vector<std::pair<float, SemanticVoxel>> matches;
  SemanticMapChunk chunk;

  {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex);
    chunk.generation = impl_->generation;
    chunk.voxel_size_m = impl_->config.voxel_size_m;
    chunk.frame_id = impl_->frame_id;
    chunk.taxonomy = impl_->taxonomy;
    chunk.taxonomy_version = impl_->taxonomy_version;
    matches.reserve(std::min(impl_->config.max_query_results, impl_->voxels.size()));
    for (std::int64_t x = min_key.x; x <= max_key.x; ++x) {
      for (std::int64_t y = min_key.y; y <= max_key.y; ++y) {
        for (std::int64_t z = min_key.z; z <= max_key.z; ++z) {
          const VoxelKey key{static_cast<std::int32_t>(x), static_cast<std::int32_t>(y),
                             static_cast<std::int32_t>(z)};
          const auto it = impl_->voxels.find(key);
          if (it == impl_->voxels.end()) {
            continue;
          }
          SemanticVoxel voxel = impl_->Export(key, it->second);
          if (voxel.occupancy_probability < min_occupancy_probability) {
            continue;
          }
          const float dx = voxel.center_x_m - center_x_m;
          const float dy = voxel.center_y_m - center_y_m;
          const float dz = voxel.center_z_m - center_z_m;
          const float distance_sq = dx * dx + dy * dy + dz * dz;
          if (distance_sq <= radius_sq) {
            if (matches.size() >= impl_->config.max_query_results) {
              throw std::length_error("semantic occupancy query exceeds configured result limit");
            }
            matches.emplace_back(distance_sq, std::move(voxel));
          }
        }
      }
    }
  }
  std::sort(matches.begin(), matches.end(), [](const auto &lhs, const auto &rhs) {
    if (lhs.first != rhs.first) {
      return lhs.first < rhs.first;
    }
    if (lhs.second.index_x != rhs.second.index_x) {
      return lhs.second.index_x < rhs.second.index_x;
    }
    if (lhs.second.index_y != rhs.second.index_y) {
      return lhs.second.index_y < rhs.second.index_y;
    }
    return lhs.second.index_z < rhs.second.index_z;
  });
  std::vector<SemanticVoxel> voxels;
  voxels.reserve(matches.size());
  for (auto &match : matches) {
    voxels.push_back(std::move(match.second));
  }
  chunk.total_voxels = voxels.size();
  chunk.complete = true;
  chunk.data = BuildChunkData(voxels);
  return chunk;
}

SemanticMapChunk SemanticOccupancyLayerCore::SnapshotChunk(
    std::size_t offset, std::size_t limit, float min_occupancy_probability) const {
  if (limit == 0U || !IsFinite(min_occupancy_probability) ||
      min_occupancy_probability < 0.0F || min_occupancy_probability > 1.0F) {
    throw std::invalid_argument("semantic occupancy snapshot parameters are invalid");
  }
  std::vector<SemanticVoxel> voxels;
  SemanticMapChunk chunk;
  {
    std::shared_lock<std::shared_mutex> lock(impl_->mutex);
    chunk.generation = impl_->generation;
    chunk.voxel_size_m = impl_->config.voxel_size_m;
    chunk.frame_id = impl_->frame_id;
    chunk.taxonomy = impl_->taxonomy;
    chunk.taxonomy_version = impl_->taxonomy_version;
    voxels.reserve(impl_->voxels.size());
    for (const auto &item : impl_->voxels) {
      SemanticVoxel voxel = impl_->Export(item.first, item.second);
      if (voxel.occupancy_probability >= min_occupancy_probability) {
        voxels.push_back(std::move(voxel));
      }
    }
  }
  std::sort(voxels.begin(), voxels.end(), [](const SemanticVoxel &lhs, const SemanticVoxel &rhs) {
    if (lhs.index_x != rhs.index_x) {
      return lhs.index_x < rhs.index_x;
    }
    if (lhs.index_y != rhs.index_y) {
      return lhs.index_y < rhs.index_y;
    }
    return lhs.index_z < rhs.index_z;
  });
  chunk.total_voxels = voxels.size();
  chunk.offset = std::min(offset, voxels.size());
  const std::size_t page_size = std::min(limit, voxels.size() - chunk.offset);
  const std::size_t end = chunk.offset + page_size;
  std::vector<SemanticVoxel> page;
  page.reserve(end - chunk.offset);
  for (std::size_t i = chunk.offset; i < end; ++i) {
    page.push_back(std::move(voxels[i]));
  }
  chunk.complete = end >= voxels.size();
  chunk.data = BuildChunkData(page);
  return chunk;
}

std::uint64_t SemanticOccupancyLayerCore::Generation() const {
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  return impl_->generation;
}

SemanticMapMetadata SemanticOccupancyLayerCore::Metadata() const {
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  SemanticMapMetadata metadata;
  metadata.generation = impl_->generation;
  metadata.voxel_count = impl_->voxels.size();
  metadata.voxel_size_m = impl_->config.voxel_size_m;
  metadata.frame_id = impl_->frame_id;
  metadata.taxonomy = impl_->taxonomy;
  metadata.taxonomy_version = impl_->taxonomy_version;
  return metadata;
}

std::size_t SemanticOccupancyLayerCore::VoxelCount() const {
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  return impl_->voxels.size();
}

SemanticOccupancyUpdateStats SemanticOccupancyLayerCore::LastStats() const {
  std::shared_lock<std::shared_mutex> lock(impl_->mutex);
  return impl_->last_stats;
}

}  // namespace lingtu::maps::layers
