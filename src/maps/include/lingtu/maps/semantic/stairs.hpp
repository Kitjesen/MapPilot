#pragma once

#include "lingtu/maps/io/pcd.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace lingtu::maps {

// Observed horizontal surface, expressed in a single flight's local axes.
// s points from lower to upper; lateral is the left perpendicular in map XY.
// Bounds describe observations, not a filled or collision-free rectangle.
struct StairSurface {
  double s_min{0.0};
  double s_max{0.0};
  double lateral_min{0.0};
  double lateral_max{0.0};
  double surface_z{0.0};
  std::size_t observed_points{0};
};

struct StairGeometry {
  bool recognized{false};
  std::string reason{"insufficient_geometry"};
  PointXyz origin;
  double axis_x{1.0};
  double axis_y{0.0};
  // Input XY on the landing, refined with its measured surface Z.
  PointXyz lower;
  PointXyz upper;
  // Includes both observed landings; surfaces[1..size-2] are treads.
  std::vector<StairSurface> surfaces;
  double width_m{0.0};
  double mean_rise_m{0.0};
  double mean_run_m{0.0};
};

struct StairRecognitionOptions {
  double resolution_m{0.05};
  double height_tolerance_m{0.025};
  double min_tread_depth_m{0.12};
  double min_width_m{0.30};
  double min_rise_m{0.06};
  double max_rise_m{0.30};
};

// Region/entrance-guided recognition of one straight flight. Distinct surfaces
// at the same XY remain separate. This never modifies source points, infers
// free space or establishes the selected robot's ability to traverse stairs.
StairGeometry RecognizeStairGeometry(
    const std::vector<PointXyz>& points, const PcdBounds& bounds,
    const PointXyz& lower, const PointXyz& upper,
    const StairRecognitionOptions& options = {});

}  // namespace lingtu::maps
