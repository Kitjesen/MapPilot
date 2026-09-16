#pragma once

#include "localization/opt/map.hpp"

#include "lingtu_pose_graph_opt.h"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace lingtu::localization::opt {

struct Pose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double qw = 1.0;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
};

struct Keyframe {
  std::string patch_name;
  Pose pose;
};

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
};

struct GraphSolution {
  bool ok = false;
  std::string code;
  std::string message;
  std::vector<Keyframe> keyframes;
  lt_pose_graph_opt_report report{};
};

// Pure in-memory solve. Input poses are estimates, never synthesized factors.
// Both the save-time writer and the online worker use this quality gate.
GraphSolution optimize_graph(const std::vector<Keyframe>& keyframes,
                             const OptimizeOptions& options);

Result optimize_map(const Map& map, const OptimizeOptions& options);

std::vector<Keyframe> read_poses(const std::filesystem::path& path);

}  // namespace lingtu::localization::opt
