#pragma once

#include <optional>
#include <unordered_map>
#include <vector>

#include "cleaner.hpp"
#include "core/types.hpp"

namespace lingtu::map_cleaning {

// Index only map points; free space is evidence, not a new persistent map.
class VisibilityEvidence {
 public:
  VisibilityEvidence(const std::vector<PointXYZI>& points, const StaticCleanerOptions& options);
  void observe(const std::vector<PointXYZI>& scan, const Pose& pose);
  bool contradicted(std::size_t index) const;
  bool onSupportedSurface(std::size_t index) const;
  std::vector<bool> supportedSurfaces() const;

 private:
  struct Cell {
    std::vector<std::size_t> points;
    std::size_t remaining{0};
    std::size_t ray{0};
    std::uint32_t frame{0};
  };
  const std::vector<PointXYZI>& points_;
  const StaticCleanerOptions& options_;
  std::unordered_map<VoxelKey, Cell, VoxelKeyHash> cells_;
  // Bucket geometry is fixed; each query still reads the current hit/free evidence.
  mutable std::optional<VoxelKey> support_key_;
  mutable std::vector<const std::vector<std::size_t>*> support_cells_;
  std::vector<std::uint32_t> free_frames_;
  std::vector<bool> seen_;
  std::vector<std::uint32_t> hit_frames_;
  // A stamp equal to frame_ marks the point in the current frame; 0 is never current.
  std::uint32_t frame_{0};
  std::vector<std::uint32_t> occupied_stamp_;
  std::vector<std::uint32_t> free_stamp_;
  std::vector<std::uint8_t> free_coverage_;
  std::vector<std::size_t> occupied_;
  std::vector<std::size_t> free_;
};

}  // namespace lingtu::map_cleaning
