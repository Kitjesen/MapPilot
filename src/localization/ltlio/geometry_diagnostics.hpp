#pragma once

#include <array>
#include <string>

namespace lingtu::ltlio {

// LiDAR-only geometry, before adding the inertial prior. Each principal
// direction is [scaled rotation, translation], both expressed in IMU axes.
// A rotation increment is multiplied by rotation_scale_m in this basis.
struct GeometryDiagnostics {
    bool available = false;
    bool detected = false;
    int degenerate_dof_count = 0;
    double condition_number = 0.0;
    double min_eigenvalue = 0.0;
    double max_eigenvalue = 0.0;
    double rotation_scale_m = 0.0;
    std::array<double, 6> eigenvalues{};
    std::array<std::array<double, 6>, 6> principal_directions{};
    std::array<std::string, 6> categories{};
    std::array<double, 6> support_ratios{};
    // True only when an accepted scan used the direction-aware LiDAR model.
    // This is not the ordinary LiDAR update acceptance flag.
    bool update_applied = false;
    std::string guidance = "awaiting_lidar_geometry";
};

}  // namespace lingtu::ltlio
