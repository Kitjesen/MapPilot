// SPDX-License-Identifier: MIT
// LingTu portable camera-LiDAR optimizer bridge.

#pragma once

#include <cstddef>
#include <cstdint>

extern "C" {

struct LingtuCameraLidarPose3 {
  double t_xyz[3];
  double q_wxyz[4];
};

struct LingtuCameraLidarCtIcpCorrespondence {
  double time;
  double source_xyz[3];
  double target_xyz[3];
  double target_normal[3];
};

struct LingtuCameraLidarCtGicpCorrespondence {
  double time;
  double source_xyz[3];
  double target_xyz[3];
  double source_cov_row_major[9];
  double target_cov_row_major[9];
};

struct LingtuCameraLidarCtGicpSourcePoint {
  double time;
  double xyz[3];
  double cov_row_major[9];
};

struct LingtuCameraLidarCtGicpTargetPoint {
  double xyz[3];
  double cov_row_major[9];
};

struct LingtuCameraLidarTwoPoseLinearization {
  double cost;
  std::uint64_t used_correspondences;
  double hessian_12x12_row_major[144];
  double gradient_12[12];
  double rhs_12[12];
};

struct LingtuCameraLidarTwoPoseOptimizerConfig {
  std::uint32_t max_iterations;
  double derivative_eps;
  double initial_lambda;
  double lambda_factor;
  double step_tolerance;
  double relative_cost_tolerance;
  double prior_precision;
  double between_precision;
};

struct LingtuCameraLidarTwoPoseOptimizationResult {
  LingtuCameraLidarPose3 pose0;
  LingtuCameraLidarPose3 pose1;
  double initial_cost;
  double final_cost;
  std::uint32_t iterations;
  std::uint32_t accepted_steps;
  double last_lambda;
  std::uint64_t used_correspondences;
};

std::uint32_t lingtu_camera_lidar_optimizer_abi_version();
std::size_t lingtu_camera_lidar_optimizer_sizeof_pose3();
std::size_t lingtu_camera_lidar_optimizer_sizeof_ct_icp_correspondence();
std::size_t lingtu_camera_lidar_optimizer_sizeof_ct_gicp_correspondence();
std::size_t lingtu_camera_lidar_optimizer_sizeof_ct_gicp_source_point();
std::size_t lingtu_camera_lidar_optimizer_sizeof_ct_gicp_target_point();
std::size_t lingtu_camera_lidar_optimizer_sizeof_two_pose_linearization();
std::size_t lingtu_camera_lidar_optimizer_sizeof_two_pose_optimizer_config();
std::size_t lingtu_camera_lidar_optimizer_sizeof_two_pose_optimization_result();

int lingtu_camera_lidar_optimizer_build_ct_gicp_correspondences(
  const LingtuCameraLidarPose3* pose0,
  const LingtuCameraLidarPose3* pose1,
  const LingtuCameraLidarCtGicpSourcePoint* sources,
  std::size_t source_count,
  const LingtuCameraLidarCtGicpTargetPoint* targets,
  std::size_t target_count,
  double max_correspondence_distance,
  LingtuCameraLidarCtGicpCorrespondence* output,
  std::size_t output_capacity,
  std::size_t* output_count);

int lingtu_camera_lidar_optimizer_linearize_ct_icp(
  const LingtuCameraLidarPose3* pose0,
  const LingtuCameraLidarPose3* pose1,
  const LingtuCameraLidarCtIcpCorrespondence* correspondences,
  std::size_t correspondence_count,
  double derivative_eps,
  LingtuCameraLidarTwoPoseLinearization* output);

int lingtu_camera_lidar_optimizer_linearize_ct_gicp(
  const LingtuCameraLidarPose3* pose0,
  const LingtuCameraLidarPose3* pose1,
  const LingtuCameraLidarCtGicpCorrespondence* correspondences,
  std::size_t correspondence_count,
  double derivative_eps,
  LingtuCameraLidarTwoPoseLinearization* output);

int lingtu_camera_lidar_optimizer_optimize_ct_gicp_two_pose(
  const LingtuCameraLidarPose3* initial_pose0,
  const LingtuCameraLidarPose3* initial_pose1,
  const LingtuCameraLidarPose3* prior_pose0,
  const LingtuCameraLidarPose3* between_measurement,
  const LingtuCameraLidarCtGicpCorrespondence* correspondences,
  std::size_t correspondence_count,
  const LingtuCameraLidarTwoPoseOptimizerConfig* config,
  LingtuCameraLidarTwoPoseOptimizationResult* output);

int lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose(
  const LingtuCameraLidarPose3* initial_pose0,
  const LingtuCameraLidarPose3* initial_pose1,
  const LingtuCameraLidarPose3* prior_pose0,
  const LingtuCameraLidarPose3* between_measurement,
  const LingtuCameraLidarCtGicpSourcePoint* sources,
  std::size_t source_count,
  const LingtuCameraLidarCtGicpTargetPoint* targets,
  std::size_t target_count,
  double max_correspondence_distance,
  std::uint32_t outer_iterations,
  const LingtuCameraLidarTwoPoseOptimizerConfig* config,
  LingtuCameraLidarTwoPoseOptimizationResult* output);

}  // extern "C"

namespace vlcal::rust_camera_lidar_optimizer {

constexpr int kOk = 0;
constexpr int kInsufficientOutputCapacity = -8;
constexpr double kDefaultDerivativeEps = 1e-6;
constexpr double kDefaultInitialLambda = 1e-3;
constexpr double kDefaultLambdaFactor = 10.0;
constexpr double kDefaultStepTolerance = 1e-7;
constexpr double kDefaultRelativeCostTolerance = 1e-8;

inline bool abi_compatible() {
  return lingtu_camera_lidar_optimizer_abi_version() == 2 &&
         lingtu_camera_lidar_optimizer_sizeof_pose3() == sizeof(LingtuCameraLidarPose3) &&
         lingtu_camera_lidar_optimizer_sizeof_ct_icp_correspondence() ==
           sizeof(LingtuCameraLidarCtIcpCorrespondence) &&
         lingtu_camera_lidar_optimizer_sizeof_ct_gicp_correspondence() ==
           sizeof(LingtuCameraLidarCtGicpCorrespondence) &&
         lingtu_camera_lidar_optimizer_sizeof_ct_gicp_source_point() ==
           sizeof(LingtuCameraLidarCtGicpSourcePoint) &&
         lingtu_camera_lidar_optimizer_sizeof_ct_gicp_target_point() ==
           sizeof(LingtuCameraLidarCtGicpTargetPoint) &&
         lingtu_camera_lidar_optimizer_sizeof_two_pose_linearization() ==
           sizeof(LingtuCameraLidarTwoPoseLinearization) &&
         lingtu_camera_lidar_optimizer_sizeof_two_pose_optimizer_config() ==
           sizeof(LingtuCameraLidarTwoPoseOptimizerConfig) &&
         lingtu_camera_lidar_optimizer_sizeof_two_pose_optimization_result() ==
           sizeof(LingtuCameraLidarTwoPoseOptimizationResult);
}

template <typename Matrix4Like>
inline void copy_covariance3_row_major(const Matrix4Like& matrix, double* output) {
  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      output[row * 3 + col] = matrix(row, col);
    }
  }
}

}  // namespace vlcal::rust_camera_lidar_optimizer
