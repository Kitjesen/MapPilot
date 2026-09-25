#pragma once

#include "localization/opt/map.hpp"
#include "localization/opt/poses.hpp"

#include "lingtu_pose_graph_opt.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::localization::opt {

struct GeometricConstraint {
  std::size_t from_index = 0;
  std::size_t to_index = 0;
  Pose pose_from_to;
  // SE(3) tangent information order is fixed by the pose-graph ABI:
  // [omega_x, omega_y, omega_z, upsilon_x, upsilon_y, upsilon_z].
  // A caller must provide every axis explicitly; zero defaults fail closed.
  // Packed row-major upper triangle of the full 6x6 information matrix.
  std::array<double, 21> information_upper{};
};

struct OptimizeOptions {
  std::string strategy = "pgo";
  std::size_t max_iterations = 30;
  std::vector<GeometricConstraint> geometric_constraints;
  // Original LIO attitudes, independent of warm-started optimizer estimates.
  // Empty is reserved for generic graphs without gravity measurements.
  std::vector<Keyframe> gravity_reference;
  double max_gravity_error_rad = 0.03;
};

struct GraphSolution {
  bool ok = false;
  std::string code;
  std::string message;
  std::vector<Keyframe> keyframes;
  lt_pose_graph_opt_report report{};
};

// Nodes reachable from the fixed first pose through measured edges only.
std::vector<std::size_t> connected_pose_indices(
    std::size_t pose_count, const std::vector<GeometricConstraint>& constraints,
    std::size_t root = 0);

// Pure in-memory solve. Input poses are estimates, never synthesized factors.
// Both the save-time writer and the online worker use this quality gate.
GraphSolution optimize_graph(const std::vector<Keyframe>& keyframes,
                             const OptimizeOptions& options);

Result optimize_map(const Map& map, const OptimizeOptions& options);


}  // namespace lingtu::localization::opt
