#pragma once

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

 private:
  const std::vector<PointXYZI>& points_;
  const StaticCleanerOptions& options_;
  std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> cells_;
  std::vector<std::uint32_t> free_frames_;
  std::vector<bool> seen_;
  std::vector<std::uint32_t> hit_frames_;
};

}  // namespace lingtu::map_cleaning
