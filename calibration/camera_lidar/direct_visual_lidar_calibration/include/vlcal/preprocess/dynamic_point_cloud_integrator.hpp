#pragma once

#include <random>
#include <thread>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vlcal/common/frame_cpu.hpp>
#include <vlcal/common/vector3i_hash.hpp>
#include <vlcal/common/concurrent_queue.hpp>
#include <vlcal/preprocess/point_cloud_integrator.hpp>

namespace Bonxai {
template <typename T>
class VoxelGrid;
}

namespace vlcal {

class iVox;

struct DynamicPointCloudIntegratorParams {
public:
  DynamicPointCloudIntegratorParams();
  ~DynamicPointCloudIntegratorParams();

  bool visualize;         ///< If true, show integrated points
  bool verbose;           ///< If true, print out optimization progress
  int num_threads;        ///< Number of threads
  int k_neighbors;        ///< Number of neighbor points for covariance estimation
  int target_num_points;  ///< Target number of points for downsampling

  double min_distance;      ///< Points closer than this will be discarded
  double voxel_resolution;  ///< Dense target model downsampling resolution
};

/**
 * @brief Dynamic LiDAR point integrator for spinning LiDARs
 */
class DynamicPointCloudIntegrator : public PointCloudIntegrator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicPointCloudIntegrator(const DynamicPointCloudIntegratorParams& params = DynamicPointCloudIntegratorParams());
  ~DynamicPointCloudIntegrator();

  virtual void insert_points(const Frame::ConstPtr& raw_points) override;
  virtual Frame::ConstPtr get_points() override;

private:
  void voxelgrid_task();

private:
  const DynamicPointCloudIntegratorParams params;

  std::mt19937 mt;

  Eigen::Isometry3d last_T_odom_lidar_begin;
  Eigen::Isometry3d last_T_odom_lidar_end;
  std::shared_ptr<iVox> target_ivox;

  std::thread voxelgrid_thread;
  ConcurrentQueue<std::tuple<Frame::ConstPtr, Eigen::Isometry3d, Eigen::Isometry3d>> alignment_results;
  std::unordered_map<Eigen::Vector3i, Eigen::Vector4d, Vector3iHash> voxelgrid;
};
}  // namespace vlcal
