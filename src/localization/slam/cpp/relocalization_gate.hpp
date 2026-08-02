#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "slam.hpp"

namespace lingtu::slam {

struct RelocalizationGateConfig {
  double max_fitness{0.5};
  int min_inliers{30};
  int min_evaluated_points{30};
  double max_pos_cov_trace{1.0};
  double max_alignment_translation_m{1.0};
  double max_alignment_yaw_rad{0.2617993877991494};
  // map and odom are gravity-aligned frames. A non-planar map->odom
  // transform is therefore an invalid registration result, not terrain
  // attitude. Apply this gate to initial relocalization as well as periodic
  // alignment updates so a degenerate 6-DoF ICP solution cannot tilt the
  // entire saved map.
  double max_alignment_tilt_rad{0.08726646259971647};
  bool require_alignment_degeneracy_metrics{true};
};

struct RelocalizationGateInput {
  bool converged{false};
  double fitness{-1.0};
  int inliers{-1};
  int evaluated_points{-1};
  double pos_cov_trace{-1.0};
  bool alignment_update{false};
  Pose3d current_map_odom;
  Pose3d candidate_map_odom;
};

struct RelocalizationGateDecision {
  bool accepted{false};
  std::string reason;
  double correction_translation_m{0.0};
  double correction_yaw_rad{0.0};
  double alignment_tilt_rad{0.0};
};

namespace relocalization_gate_detail {

inline bool FinitePose(const Pose3d &pose) {
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
         std::isfinite(pose.qx) && std::isfinite(pose.qy) && std::isfinite(pose.qz) &&
         std::isfinite(pose.qw);
}

inline double Yaw(const Pose3d &pose) {
  const double norm =
      std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
  if (!std::isfinite(norm) || norm <= 1e-12) {
    return 0.0;
  }
  const double x = pose.qx / norm;
  const double y = pose.qy / norm;
  const double z = pose.qz / norm;
  const double w = pose.qw / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

inline double Tilt(const Pose3d &pose) {
  const double norm =
      std::sqrt(pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
  if (!std::isfinite(norm) || norm <= 1e-12) {
    return std::numeric_limits<double>::infinity();
  }
  const double x = pose.qx / norm;
  const double y = pose.qy / norm;
  const double rotated_z_dot_world_z =
      std::clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0);
  return std::acos(rotated_z_dot_world_z);
}

inline double AngleDistance(double lhs, double rhs) {
  constexpr double kPi = 3.14159265358979323846;
  double delta = std::fmod(lhs - rhs, 2.0 * kPi);
  if (delta > kPi) {
    delta -= 2.0 * kPi;
  } else if (delta < -kPi) {
    delta += 2.0 * kPi;
  }
  return std::abs(delta);
}

}  // namespace relocalization_gate_detail

inline RelocalizationGateDecision EvaluateRelocalizationGate(const RelocalizationGateConfig &config,
                                                             const RelocalizationGateInput &input) {
  RelocalizationGateDecision decision;
  if (!input.converged) {
    decision.reason = "relocalization_not_converged";
    return decision;
  }
  if (!std::isfinite(input.fitness) || input.fitness < 0.0 ||
      input.fitness > std::max(0.0, config.max_fitness)) {
    decision.reason = "relocalization_fitness_rejected";
    return decision;
  }
  if (input.inliers >= 0 && input.inliers < std::max(0, config.min_inliers)) {
    decision.reason = "relocalization_inliers_rejected";
    return decision;
  }
  const int min_evaluated_points = std::max(0, config.min_evaluated_points);
  if (input.evaluated_points < 0 && min_evaluated_points > 0) {
    decision.reason = "relocalization_evaluated_points_unavailable";
    return decision;
  }
  if (input.evaluated_points >= 0 &&
      input.evaluated_points < min_evaluated_points) {
    decision.reason = "relocalization_evaluated_points_rejected";
    return decision;
  }
  if (input.pos_cov_trace >= 0.0 &&
      (!std::isfinite(input.pos_cov_trace) ||
       input.pos_cov_trace > std::max(0.0, config.max_pos_cov_trace))) {
    decision.reason = "relocalization_covariance_rejected";
    return decision;
  }
  if (!relocalization_gate_detail::FinitePose(input.candidate_map_odom)) {
    decision.reason = "relocalization_transform_invalid";
    return decision;
  }
  decision.alignment_tilt_rad =
      relocalization_gate_detail::Tilt(input.candidate_map_odom);
  if (!std::isfinite(decision.alignment_tilt_rad)) {
    decision.reason = "relocalization_transform_invalid";
    return decision;
  }
  if (decision.alignment_tilt_rad > std::max(0.0, config.max_alignment_tilt_rad)) {
    decision.reason = "relocalization_tilt_rejected";
    return decision;
  }
  if (config.require_alignment_degeneracy_metrics &&
      (input.inliers < 0 || input.evaluated_points < 0 ||
       input.pos_cov_trace < 0.0)) {
    decision.reason = "relocalization_degeneracy_metrics_unavailable";
    return decision;
  }
  if (!input.alignment_update) {
    decision.accepted = true;
    decision.reason = "accepted";
    return decision;
  }
  if (!relocalization_gate_detail::FinitePose(input.current_map_odom)) {
    decision.reason = "relocalization_current_transform_invalid";
    return decision;
  }
  const double dx = input.candidate_map_odom.x - input.current_map_odom.x;
  const double dy = input.candidate_map_odom.y - input.current_map_odom.y;
  const double dz = input.candidate_map_odom.z - input.current_map_odom.z;
  decision.correction_translation_m = std::sqrt(dx * dx + dy * dy + dz * dz);
  decision.correction_yaw_rad = relocalization_gate_detail::AngleDistance(
      relocalization_gate_detail::Yaw(input.candidate_map_odom),
      relocalization_gate_detail::Yaw(input.current_map_odom));
  if (decision.correction_translation_m > std::max(0.0, config.max_alignment_translation_m)) {
    decision.reason = "relocalization_translation_jump_rejected";
    return decision;
  }
  if (decision.correction_yaw_rad > std::max(0.0, config.max_alignment_yaw_rad)) {
    decision.reason = "relocalization_yaw_jump_rejected";
    return decision;
  }
  decision.accepted = true;
  decision.reason = "accepted";
  return decision;
}

}  // namespace lingtu::slam
