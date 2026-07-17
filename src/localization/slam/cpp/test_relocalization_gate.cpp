#include <cmath>
#include <stdexcept>

#include "relocalization_gate.hpp"

namespace {

lingtu::slam::Pose3d Pose(double x, double yaw, double roll = 0.0, double pitch = 0.0) {
  lingtu::slam::Pose3d pose;
  pose.x = x;
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  pose.qx = sr * cp * cy - cr * sp * sy;
  pose.qy = cr * sp * cy + sr * cp * sy;
  pose.qz = cr * cp * sy - sr * sp * cy;
  pose.qw = cr * cp * cy + sr * sp * sy;
  return pose;
}

void Require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main() {
  using lingtu::slam::EvaluateRelocalizationGate;
  using lingtu::slam::RelocalizationGateConfig;
  using lingtu::slam::RelocalizationGateInput;

  const RelocalizationGateConfig config;
  RelocalizationGateInput initial;
  initial.converged = true;
  initial.fitness = 0.1;
  initial.inliers = 100;
  initial.pos_cov_trace = 0.05;
  initial.candidate_map_odom = Pose(5.0, 1.0);
  Require(EvaluateRelocalizationGate(config, initial).accepted,
          "initial relocalization should be accepted");

  initial.candidate_map_odom = Pose(5.0, 1.0, 0.2, 0.0);
  Require(EvaluateRelocalizationGate(config, initial).reason ==
              "relocalization_tilt_rejected",
          "initial relocalization must reject tilted map alignment");
  initial.candidate_map_odom = Pose(5.0, 1.0, 0.0, 0.2);
  Require(EvaluateRelocalizationGate(config, initial).reason ==
              "relocalization_tilt_rejected",
          "initial relocalization must reject pitched map alignment");
  initial.candidate_map_odom = Pose(5.0, 1.0);
  initial.inliers = -1;
  initial.pos_cov_trace = -1.0;
  Require(EvaluateRelocalizationGate(config, initial).reason ==
              "relocalization_degeneracy_metrics_unavailable",
          "initial relocalization must require degeneracy metrics");
  initial.inliers = 100;
  initial.pos_cov_trace = 0.05;

  RelocalizationGateInput update;
  update.converged = true;
  update.fitness = 0.1;
  update.inliers = 100;
  update.pos_cov_trace = 0.05;
  update.alignment_update = true;
  update.current_map_odom = Pose(0.0, 0.0);
  update.candidate_map_odom = Pose(0.2, 0.05);
  Require(EvaluateRelocalizationGate(config, update).accepted,
          "bounded alignment update should be accepted");

  update.candidate_map_odom = Pose(0.2, 0.05, 0.1, 0.0);
  Require(EvaluateRelocalizationGate(config, update).reason ==
              "relocalization_tilt_rejected",
          "periodic alignment must reject tilted map alignment");
  update.candidate_map_odom = Pose(0.2, 0.05);

  update.inliers = -1;
  update.pos_cov_trace = -1.0;
  Require(EvaluateRelocalizationGate(config, update).reason ==
              "relocalization_degeneracy_metrics_unavailable",
          "alignment update must require degeneracy metrics");
  update.inliers = 10;
  update.pos_cov_trace = 0.05;
  Require(EvaluateRelocalizationGate(config, update).reason == "relocalization_inliers_rejected",
          "low-inlier alignment must be rejected");
  update.inliers = 100;
  update.pos_cov_trace = 2.0;
  Require(EvaluateRelocalizationGate(config, update).reason == "relocalization_covariance_rejected",
          "high-covariance alignment must be rejected");
  update.pos_cov_trace = 0.05;
  update.candidate_map_odom = Pose(2.0, 0.0);
  Require(EvaluateRelocalizationGate(config, update).reason ==
              "relocalization_translation_jump_rejected",
          "large translation correction must be rejected");
  update.candidate_map_odom = Pose(0.1, 0.5);
  Require(EvaluateRelocalizationGate(config, update).reason == "relocalization_yaw_jump_rejected",
          "large yaw correction must be rejected");
  return 0;
}
