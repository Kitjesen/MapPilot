#include "lingtu/maps/layers/voxel.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace lingtu::maps::layers {

namespace {

bool IsFinite(float value) {
  return std::isfinite(static_cast<double>(value));
}

std::uint64_t Mix(std::uint64_t value) {
  value ^= value >> 33U;
  value *= 0xff51afd7ed558ccdULL;
  value ^= value >> 33U;
  value *= 0xc4ceb9fe1a85ec53ULL;
  value ^= value >> 33U;
  return value;
}

}  // namespace

std::size_t VoxelLayerCore::VoxelKeyHash::operator()(const VoxelKey& key) const {
  const auto x = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.x));
  const auto y = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.y));
  const auto z = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.z));
  return static_cast<std::size_t>(Mix(x) ^ (Mix(y) << 1U) ^ (Mix(z) << 2U));
}

std::size_t VoxelLayerCore::ColumnKeyHash::operator()(const ColumnKey& key) const {
  const auto x = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.x));
  const auto y = static_cast<std::uint64_t>(static_cast<std::int64_t>(key.y));
  return static_cast<std::size_t>(Mix(x) ^ (Mix(y) << 1U));
}

VoxelLayerCore::VoxelLayerCore(VoxelLayerConfig config) : config_(config) {
  if (!(config_.voxel_size_m > 0.0F) || !IsFinite(config_.voxel_size_m)) {
    throw std::invalid_argument("VoxelLayerConfig.voxel_size_m must be finite and > 0");
  }
  if (config_.max_range_m < 0.0F || !IsFinite(config_.max_range_m)) {
    throw std::invalid_argument("VoxelLayerConfig.max_range_m must be finite and >= 0");
  }
  if (!IsFinite(config_.min_z_m) || !IsFinite(config_.max_z_m) ||
      config_.min_z_m > config_.max_z_m) {
    throw std::invalid_argument("VoxelLayerConfig z range is invalid");
  }
  if (config_.decay_rate < 0.0F || config_.decay_rate >= 1.0F ||
      !IsFinite(config_.decay_rate)) {
    throw std::invalid_argument("VoxelLayerConfig.decay_rate must be in [0, 1)");
  }
  if (config_.prune_below_count < 0.0F || !IsFinite(config_.prune_below_count)) {
    throw std::invalid_argument("VoxelLayerConfig.prune_below_count must be finite and >= 0");
  }
  last_stats_.column_carving = config_.column_carving;
}

void VoxelLayerCore::Reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  voxels_.clear();
  last_stats_ = {};
  last_stats_.frame_id = last_frame_id_;
  last_stats_.column_carving = config_.column_carving;
}

void VoxelLayerCore::Update(const MapCloudFrame& frame) {
  const PointCloudView& cloud = frame.cloud;

  std::unordered_map<VoxelKey, float, VoxelKeyHash> frame_counts;
  std::unordered_map<ColumnKey, bool, ColumnKeyHash> observed_columns;
  frame_counts.reserve(cloud.point_count);
  observed_columns.reserve(cloud.point_count);

  VoxelUpdateStats stats;
  stats.input_points = cloud.point_count;
  stats.frame_id = cloud.frame_id.empty() ? last_frame_id_ : cloud.frame_id;
  stats.column_carving = config_.column_carving;

  const float max_range_sq = config_.max_range_m * config_.max_range_m;
  const bool check_range = config_.max_range_m > 0.0F;

  for (std::size_t i = 0; i < cloud.point_count; ++i) {
    float x_m = 0.0F;
    float y_m = 0.0F;
    float z_m = 0.0F;
    if (!ReadPoint(cloud, i, &x_m, &y_m, &z_m)) {
      continue;
    }
    if (!IsFinite(x_m) || !IsFinite(y_m) || !IsFinite(z_m)) {
      continue;
    }
    if (z_m < config_.min_z_m || z_m > config_.max_z_m) {
      continue;
    }
    if (check_range) {
      const float dx = x_m - frame.sensor_origin_x_m;
      const float dy = y_m - frame.sensor_origin_y_m;
      const float dz = z_m - frame.sensor_origin_z_m;
      const float dist_sq = dx * dx + dy * dy + dz * dz;
      if (dist_sq > max_range_sq) {
        continue;
      }
    }

    const VoxelKey key = ToKey(x_m, y_m, z_m, config_.voxel_size_m);
    frame_counts[key] += 1.0F;
    observed_columns[{key.x, key.y}] = true;
    ++stats.accepted_points;
  }

  stats.input_voxels = frame_counts.size();
  stats.input_columns = observed_columns.size();
  if (frame_counts.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_stats_ = stats;
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  last_frame_id_ = stats.frame_id;
  last_stamp_ns_ = cloud.stamp_ns;
  if (config_.column_carving && !observed_columns.empty()) {
    stats.carved_columns = observed_columns.size();
    stats.carved_voxels = CarveColumnsUnlocked(observed_columns);
  }
  for (const auto& item : frame_counts) {
    voxels_[item.first] += item.second;
  }
  stats.total_voxels = voxels_.size();
  last_stats_ = stats;
}

OwnedPointCloud VoxelLayerCore::SnapshotCloud() const {
  std::lock_guard<std::mutex> lock(mutex_);
  OwnedPointCloud cloud;
  cloud.frame_id = last_frame_id_;
  cloud.stamp_ns = last_stamp_ns_;
  cloud.layout = CloudLayout::kXyzF32SoA;
  cloud.point_count = voxels_.size();
  cloud.x.reserve(voxels_.size());
  cloud.y.reserve(voxels_.size());
  cloud.z.reserve(voxels_.size());

  for (const auto& item : voxels_) {
    const VoxelKey& key = item.first;
    cloud.x.push_back((static_cast<float>(key.x) + 0.5F) * config_.voxel_size_m);
    cloud.y.push_back((static_cast<float>(key.y) + 0.5F) * config_.voxel_size_m);
    cloud.z.push_back((static_cast<float>(key.z) + 0.5F) * config_.voxel_size_m);
  }
  return cloud;
}

void VoxelLayerCore::Decay() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (voxels_.empty() || config_.decay_rate == 0.0F) {
    return;
  }

  const float factor = 1.0F - config_.decay_rate;
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    it->second *= factor;
    if (it->second < config_.prune_below_count) {
      it = voxels_.erase(it);
    } else {
      ++it;
    }
  }
  last_stats_.total_voxels = voxels_.size();
}

bool VoxelLayerCore::Contains(float x_m, float y_m, float z_m) const {
  return CountAt(x_m, y_m, z_m) >= 1.0F;
}

float VoxelLayerCore::CountAt(float x_m, float y_m, float z_m) const {
  const VoxelKey key = ToKey(x_m, y_m, z_m, config_.voxel_size_m);
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = voxels_.find(key);
  return it == voxels_.end() ? 0.0F : it->second;
}

std::size_t VoxelLayerCore::VoxelCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return voxels_.size();
}

VoxelUpdateStats VoxelLayerCore::LastStats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return last_stats_;
}

VoxelLayerCore::VoxelKey VoxelLayerCore::ToKey(
    float x_m,
    float y_m,
    float z_m,
    float voxel_size_m) {
  return {
      static_cast<std::int32_t>(std::floor(x_m / voxel_size_m)),
      static_cast<std::int32_t>(std::floor(y_m / voxel_size_m)),
      static_cast<std::int32_t>(std::floor(z_m / voxel_size_m)),
  };
}

bool VoxelLayerCore::ReadPoint(
    const PointCloudView& cloud,
    std::size_t index,
    float* x_m,
    float* y_m,
    float* z_m) {
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

std::size_t VoxelLayerCore::CarveColumnsUnlocked(
    const std::unordered_map<ColumnKey, bool, ColumnKeyHash>& columns) {
  std::size_t removed = 0;
  for (auto it = voxels_.begin(); it != voxels_.end();) {
    if (columns.find({it->first.x, it->first.y}) != columns.end()) {
      it = voxels_.erase(it);
      ++removed;
    } else {
      ++it;
    }
  }
  return removed;
}

}  // namespace lingtu::maps::layers
