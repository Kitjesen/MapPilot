#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "localization/opt/graph.hpp"
#include "localization/opt/map.hpp"

namespace lingtu::localization::opt {

struct LoopConstraintOptions {
  std::size_t min_index_separation = 20;
  double min_path_separation_m = 8.0;
  double candidate_xy_radius_m = 6.0;
  double candidate_max_abs_z_m = 4.0;
  std::size_t submap_half_window = 2;
  double voxel_size_m = 0.25;
  std::size_t max_points_per_submap = 5000;
  std::size_t angular_bins = 36;
  std::size_t radial_bins = 12;
  std::size_t height_bins = 8;
  double descriptor_max_radius_m = 25.0;
  double descriptor_min_z_m = -3.0;
  double descriptor_max_z_m = 5.0;
  double descriptor_min_similarity = 0.55;
  // Distinguish the best historical place from the next distinct place.
  double descriptor_min_margin = 0.02;
  // Reject rotationally symmetric descriptors before ICP.
  double descriptor_min_yaw_margin = 0.01;
  std::size_t yaw_hypotheses = 3;
  // Bound descriptor work before candidate scoring in revisited map cells.
  std::size_t max_spatial_candidates_per_anchor = 128;
  std::size_t max_candidates_per_anchor = 3;
  std::size_t max_total_candidates = 64;
  double max_correspondence_distance_m = 1.2;
  std::size_t max_icp_iterations = 12;
  double trim_fraction = 0.70;
  std::size_t min_inliers = 80;
  double min_inlier_ratio = 0.25;
  double max_rmse_m = 0.35;
  double max_p95_m = 0.65;
  double min_xy_span_m = 1.5;
  double min_z_span_m = 0.25;
  double normal_radius_m = 0.65;
  std::size_t normal_min_neighbors = 8;
  double normal_max_small_to_middle_ratio = 0.20;
  double normal_min_middle_to_large_ratio = 0.05;
  double normal_min_middle_variance_m2 = 0.005;
  std::size_t min_planar_inliers = 60;
  double min_planar_inlier_ratio = 0.30;
  double min_point_to_plane_eigenvalue = 0.01;
  double max_point_to_plane_condition = 100.0;
  // Retained as a diagnostic-only legacy point-to-point condition gate.
  double max_hessian_condition = 1.0e6;
  double max_correction_xy_m = 3.0;
  double max_correction_z_m = 2.0;
  double max_correction_yaw_rad = 0.70;
  double max_inverse_translation_m = 0.25;
  double max_inverse_yaw_rad = 0.08;
  bool require_consensus = true;
  std::size_t min_consistent_matches = 2;
  std::size_t consensus_anchor_tolerance = 3;
  std::size_t consensus_step_tolerance = 1;
  double consensus_translation_m = 0.40;
  double consensus_yaw_rad = 0.10;
};

struct LoopCandidateDiagnostic {
  std::size_t from_index = 0;
  std::size_t to_index = 0;
  bool accepted = false;
  std::string reason;
  double descriptor_score = 0.0;
  double descriptor_margin = 0.0;
  double descriptor_yaw_margin = 0.0;
  double yaw_hypothesis_rad = 0.0;
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  std::size_t inliers = 0;
  double inlier_ratio = 0.0;
  double rmse_m = -1.0;
  double p50_m = -1.0;
  double p95_m = -1.0;
  double hessian_condition = -1.0;
  std::size_t planar_inliers = 0;
  double planar_inlier_ratio = 0.0;
  std::array<double, 4> point_to_plane_eigenvalues{};
  std::array<double, 4> point_to_plane_weak_mode{};
  double point_to_plane_condition = -1.0;
  double correction_xy_m = -1.0;
  double correction_z_m = -1.0;
  double correction_yaw_rad = -1.0;
  double inverse_translation_error_m = -1.0;
  double inverse_yaw_error_rad = -1.0;
  std::size_t iterations = 0;
};

struct LoopConstraintReport {
  std::string schema_version = "lingtu.loop_constraints.shadow.v3";
  std::string algorithm_version = "lingtu.loop_verify.2";
  std::string threshold_version = "lingtu.loop_thresholds.v3";
  std::string frame_convention = "patch=body_local,pose=T_map_body,constraint=T_from_to";
  std::string information_convention = "shadow_only;information_diagonal=zero;not_graph_compatible";
  std::string options_fingerprint;
  LoopConstraintOptions options;
  std::string poses_fingerprint;
  std::string patches_fingerprint;
  std::size_t pose_count = 0;
  std::size_t patch_count = 0;
  std::size_t candidate_count = 0;
  std::size_t geometrically_verified_count = 0;
  std::size_t accepted_constraint_count = 0;
  std::size_t consensus_support_count = 0;
  std::size_t rejected_count = 0;
  double elapsed_ms = 0.0;
  std::vector<LoopCandidateDiagnostic> candidates;
};

struct LoopConstraintResult {
  bool ok = false;
  std::string code;
  std::string message;
  std::vector<GeometricConstraint> constraints;
  LoopConstraintReport report;
};

LoopConstraintResult generate_loop_constraints(const Map &map,
                                               const std::vector<Keyframe> &keyframes,
                                               const LoopConstraintOptions &options = {});

bool write_loop_constraints_report(const std::filesystem::path &path,
                                   const LoopConstraintResult &result,
                                   std::string *error = nullptr);

}  // namespace lingtu::localization::opt
