#pragma once

#include "localization/opt/graph.hpp"

#include <filesystem>
#include <vector>

namespace lingtu::localization::opt {

// Format:
// LINGTU_PGO_CONSTRAINTS_V1
// T_from_to tx ty tz qw qx qy qz
// RIGHT_TANGENT omega_x omega_y omega_z upsilon_x upsilon_y upsilon_z
// UPPER_TRIANGLE row_major 21
// followed by every independent graph factor: from to, pose values, then the
// packed upper triangle. This includes odometry adjacency as well as loops;
// poses.txt is only the initial estimate and never synthesizes factors.
std::vector<GeometricConstraint> read_constraints(const std::filesystem::path& path);

bool valid_information_upper(const std::array<double, 21>& upper);

}  // namespace lingtu::localization::opt
