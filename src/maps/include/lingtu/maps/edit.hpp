#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "lingtu/maps/io/pcd.hpp"

namespace lingtu::maps {

struct SourceEditOperation {
  std::string operation;
  PcdBounds bounds;
  double voxel_size{0.1};
  int neighbors{20};
  double std_ratio{1.5};
  double radius{0.2};
  int min_points{12};
  int min_cluster_points{100};
  double distance_threshold{0.05};
  double margin{0.5};
  std::string axis{"z"};
  double offset{0.0};
  double spacing{0.05};
};

void ValidateSourceEdits(const std::vector<SourceEditOperation>& operations);

// Operations are validated at the request boundary. Results retain source order
// and source indices, including distinct records with identical coordinates.
std::vector<std::size_t> SelectSourceEdits(
    const std::vector<PointXyz>& points,
    const std::vector<SourceEditOperation>& operations);

// New points are appended, so callers can retain measured/manual provenance
// through the returned source indices and subsequent subset operations.
std::vector<std::size_t> ApplySourceEdits(
    std::vector<PointXyz>& points,
    const std::vector<SourceEditOperation>& operations);

}  // namespace lingtu::maps
