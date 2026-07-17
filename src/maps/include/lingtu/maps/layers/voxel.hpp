#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "lingtu/maps/cloud.hpp"

namespace lingtu::maps::layers {

struct VoxelLayerConfig {
  float voxel_size_m{0.05F};
  float max_range_m{20.0F};
  float min_z_m{-0.5F};
  float max_z_m{3.0F};
  float decay_rate{0.01F};
  float prune_below_count{1.0F};
  bool column_carving{true};
};

struct VoxelUpdateStats {
  std::size_t input_points{0};
  std::size_t accepted_points{0};
  std::size_t input_voxels{0};
  std::size_t input_columns{0};
  std::size_t carved_columns{0};
  std::size_t carved_voxels{0};
  std::size_t total_voxels{0};
  std::string frame_id{"map"};
  bool column_carving{true};
};

class VoxelLayer {
 public:
  virtual ~VoxelLayer() = default;
  virtual void Reset() = 0;
  virtual void Update(const MapCloudFrame& frame) = 0;
  virtual OwnedPointCloud SnapshotCloud() const = 0;
};

class VoxelLayerCore final : public VoxelLayer {
 public:
  explicit VoxelLayerCore(VoxelLayerConfig config = {});

  void Reset() override;
  void Update(const MapCloudFrame& frame) override;
  OwnedPointCloud SnapshotCloud() const override;

  void Decay();
  bool Contains(float x_m, float y_m, float z_m) const;
  float CountAt(float x_m, float y_m, float z_m) const;
  std::size_t VoxelCount() const;
  VoxelUpdateStats LastStats() const;

 private:
  struct VoxelKey {
    std::int32_t x{0};
    std::int32_t y{0};
    std::int32_t z{0};

    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct ColumnKey {
    std::int32_t x{0};
    std::int32_t y{0};

    bool operator==(const ColumnKey& other) const {
      return x == other.x && y == other.y;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const;
  };

  struct ColumnKeyHash {
    std::size_t operator()(const ColumnKey& key) const;
  };

  static VoxelKey ToKey(float x_m, float y_m, float z_m, float voxel_size_m);
  static bool ReadPoint(
      const PointCloudView& cloud,
      std::size_t index,
      float* x_m,
      float* y_m,
      float* z_m);

  std::size_t CarveColumnsUnlocked(
      const std::unordered_map<ColumnKey, bool, ColumnKeyHash>& columns);

  VoxelLayerConfig config_;
  mutable std::mutex mutex_;
  std::unordered_map<VoxelKey, float, VoxelKeyHash> voxels_;
  VoxelUpdateStats last_stats_;
  std::string last_frame_id_{"map"};
  std::int64_t last_stamp_ns_{0};
};

}  // namespace lingtu::maps::layers
