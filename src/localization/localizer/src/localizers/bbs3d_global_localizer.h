#pragma once

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

#include "commons.h"

namespace cpu { class BBS3D; }  // fwd-decl from libcpu_bbs3d

/**
 * Branch-and-bound global localization in a pre-built point cloud map.
 *
 * One-shot, no initial guess required. Pose returned in map frame is used
 * to seed ICPLocalizer.align() for final refinement.
 *
 * Reference: Aoki et al. "3D-BBS: Global Localization for 3D Point Cloud
 * Scan Matching Using Branch-and-Bound Algorithm", ICRA 2024.
 */
class BBS3DGlobalLocalizer {
public:
    struct Config {
        double min_level_res       = 1.0;   // coarsest voxel (m) - wider = faster on CPU
        int    max_level           = 5;     // pyramid levels (wider search range)
        double score_threshold     = 0.0;   // 0 = disabled; else inlier-ratio gate
        double trans_search_margin = 1.0;   // expand map bbox by N m each side
        double roll_pitch_margin   = 0.2;   // rad (~11 deg); IMU should mostly level it
        double yaw_search_min      = -3.14159;
        double yaw_search_max      =  3.14159;
        int    num_threads         = 1;     // Keep CPU BBS3D in-process recovery crash-isolated.
        int    timeout_ms          = 30000; // 30 s; CPU version needs more headroom
        Config() {}
    };

    struct Result {
        bool success = false;
        M4F  pose    = M4F::Identity();
        double score_percentage = 0.0;
        double elapsed_ms = 0.0;
        std::string message;
    };

    explicit BBS3DGlobalLocalizer(const Config& cfg = Config());
    ~BBS3DGlobalLocalizer();

    // True only when this binary was linked with cpu_bbs3d.
    bool available() const;

    // Build the multi-level voxel map from a PCD cloud. Expensive (~seconds).
    // Call once at startup after loadMap; redo only if map changes.
    bool set_map(const CloudType::Ptr& map_cloud);

    bool has_map() const { return has_map_; }

    // One-shot global localization. scan = current LiDAR scan (body frame,
    // gravity-aligned preferred).
    Result localize(const CloudType::Ptr& scan_cloud);

private:
    Config cfg_;
    std::unique_ptr<cpu::BBS3D> bbs_;
    bool has_map_ = false;
};
