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
  std::array<double, 6> information_diagonal{};
};

struct OptimizeOptions {
  std::string strategy = "pgo";
  std::size_t max_iterations = 30;
  double prior_weight = 1000000.0;
  double chain_rot_weight = 10000.0;
  double chain_trans_weight = 10000.0;
  double skip_rot_weight = 1000.0;
  double skip_trans_weight = 1000.0;
  std::size_t skip_stride = 0;
  std::vector<GeometricConstraint> geometric_constraints;
};

Result optimize_map(const Map& map, const OptimizeOptions& options);

std::vector<Keyframe> read_poses(const std::filesystem::path& path);

}  // namespace lingtu::localization::opt
