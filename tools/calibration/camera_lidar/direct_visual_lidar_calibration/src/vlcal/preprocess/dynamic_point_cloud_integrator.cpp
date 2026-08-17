#include <vlcal/preprocess/dynamic_point_cloud_integrator.hpp>

#ifdef LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER
#include <vlcal/common/rust_camera_lidar_optimizer.hpp>
#endif

#include <vlcal/common/ivox.hpp>
#include <vlcal/common/kdtree2.hpp>
#include <vlcal/common/frame_cpu.hpp>
#include <vlcal/common/vector3i_hash.hpp>

#include <vlcal/common/cloud_covariance_estimation.hpp>

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <guik/viewer/light_viewer.hpp>

#include <sophus/se3.hpp>

namespace {
using Vector6d = Eigen::Matrix<double, 6, 1>;

Eigen::Isometry3d from_sophus_pose(const Sophus::SE3d& pose) {
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  output.matrix() = pose.matrix();
  return output;
}

Eigen::Isometry3d interpolate_rt(const Eigen::Isometry3d& begin, const Eigen::Isometry3d& end, double t) {
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q_begin(begin.linear());
  Eigen::Quaterniond q_end(end.linear());
  q_begin.normalize();
  q_end.normalize();
  output.linear() = q_begin.slerp(t, q_end).normalized().toRotationMatrix();
  output.translation() = (1.0 - t) * begin.translation() + t * end.translation();
  return output;
}

std::vector<double> normalized_time_table(const vlcal::Frame& source, std::vector<int>* time_indices) {
  if (!source.has_times()) {
    throw std::runtime_error("dynamic point cloud integration requires per-point LiDAR timestamps.");
  }
  if (source.size() == 0) {
    throw std::runtime_error("dynamic point cloud integration requires a non-empty LiDAR frame.");
  }
  if (time_indices == nullptr) {
    throw std::invalid_argument("time_indices output must not be null.");
  }

  std::vector<double> time_table;
  time_table.reserve(std::max<size_t>(1, source.size() / 10));
  time_indices->clear();
  time_indices->reserve(source.size());

  const double time_eps = 1e-3;
  for (size_t i = 0; i < source.size(); i++) {
    const double t = source.times[i];
    if (time_table.empty() || t - time_table.back() > time_eps) {
      time_table.push_back(t);
    }
    time_indices->push_back(static_cast<int>(time_table.size() - 1));
  }

  const double max_time = std::max(1e-9, time_table.back());
  for (auto& t : time_table) {
    t /= max_time;
  }
  return time_table;
}

std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> source_pose_table(
  const Eigen::Isometry3d& begin,
  const Eigen::Isometry3d& end,
  const std::vector<double>& time_table) {
  const Sophus::SE3d T_begin(begin.matrix());
  const Sophus::SE3d T_end(end.matrix());
  const Vector6d velocity = (T_begin.inverse() * T_end).log();

  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
  poses.reserve(time_table.size());
  for (const double t : time_table) {
    poses.push_back(from_sophus_pose(T_begin * Sophus::SE3d::exp(t * velocity)));
  }
  return poses;
}

std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> deskew_points(
  const vlcal::Frame& source,
  const Eigen::Isometry3d& begin,
  const Eigen::Isometry3d& end) {
  std::vector<int> time_indices;
  const auto time_table = normalized_time_table(source, &time_indices);
  const auto poses = source_pose_table(begin, end, time_table);

  std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> deskewed(source.size());
  for (size_t i = 0; i < source.size(); i++) {
    deskewed[i] = poses[time_indices[i]].matrix() * source.points[i];
  }
  return deskewed;
}

#ifdef LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER
LingtuCameraLidarPose3 to_ffi_pose(const Eigen::Isometry3d& pose) {
  Eigen::Quaterniond q(pose.linear());
  q.normalize();
  const Eigen::Vector3d t = pose.translation();
  return LingtuCameraLidarPose3{{t.x(), t.y(), t.z()}, {q.w(), q.x(), q.y(), q.z()}};
}

Eigen::Isometry3d from_ffi_pose(const LingtuCameraLidarPose3& pose) {
  Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(pose.q_wxyz[0], pose.q_wxyz[1], pose.q_wxyz[2], pose.q_wxyz[3]);
  q.normalize();
  output.linear() = q.toRotationMatrix();
  output.translation() << pose.t_xyz[0], pose.t_xyz[1], pose.t_xyz[2];
  return output;
}

std::vector<LingtuCameraLidarCtGicpSourcePoint> build_rust_source_points(const vlcal::Frame& source) {
  if (!source.has_points() || !source.has_covs() || !source.has_times()) {
    return {};
  }

  std::vector<int> time_indices;
  const auto time_table = normalized_time_table(source, &time_indices);

  std::vector<LingtuCameraLidarCtGicpSourcePoint> sources;
  sources.reserve(source.size());
  for (size_t i = 0; i < source.size(); i++) {
    const auto& source_pt = source.points[i];
    const auto& source_cov = source.covs[i];

    LingtuCameraLidarCtGicpSourcePoint point{};
    point.time = time_table[time_indices[i]];
    for (int axis = 0; axis < 3; axis++) {
      point.xyz[axis] = source_pt[axis];
    }
    vlcal::rust_camera_lidar_optimizer::copy_covariance3_row_major(source_cov, point.cov_row_major);
    sources.push_back(point);
  }
  return sources;
}

std::vector<LingtuCameraLidarCtGicpTargetPoint> build_rust_target_points(const std::shared_ptr<vlcal::iVox>& target_ivox) {
  if (!target_ivox || !target_ivox->has_points() || !target_ivox->has_covs()) {
    return {};
  }

  const auto target_points = target_ivox->voxel_points();
  const auto target_covs = target_ivox->voxel_covs();
  if (target_points.empty() || target_points.size() != target_covs.size()) {
    return {};
  }

  std::vector<LingtuCameraLidarCtGicpTargetPoint> targets;
  targets.reserve(target_points.size());
  for (size_t i = 0; i < target_points.size(); i++) {
    LingtuCameraLidarCtGicpTargetPoint point{};
    for (int axis = 0; axis < 3; axis++) {
      point.xyz[axis] = target_points[i][axis];
    }
    vlcal::rust_camera_lidar_optimizer::copy_covariance3_row_major(target_covs[i], point.cov_row_major);
    targets.push_back(point);
  }
  return targets;
}

bool optimize_dynamic_ct_gicp_with_rust(
  const std::shared_ptr<vlcal::iVox>& target_ivox,
  const vlcal::Frame& source,
  const Eigen::Isometry3d& prior_begin,
  Eigen::Isometry3d* optimized_begin,
  Eigen::Isometry3d* optimized_end,
  int num_threads) {
  if (optimized_begin == nullptr || optimized_end == nullptr) {
    return false;
  }
  if (!vlcal::rust_camera_lidar_optimizer::abi_compatible()) {
    return false;
  }

  (void)num_threads;
  const auto rust_sources = build_rust_source_points(source);
  const auto rust_targets = build_rust_target_points(target_ivox);
  if (rust_sources.empty() || rust_targets.empty()) {
    return false;
  }

  const auto prior_pose = to_ffi_pose(prior_begin);
  const auto between_measurement = to_ffi_pose(Eigen::Isometry3d::Identity());
  const LingtuCameraLidarTwoPoseOptimizerConfig config{
    static_cast<std::uint32_t>(10),
    vlcal::rust_camera_lidar_optimizer::kDefaultDerivativeEps,
    vlcal::rust_camera_lidar_optimizer::kDefaultInitialLambda,
    vlcal::rust_camera_lidar_optimizer::kDefaultLambdaFactor,
    vlcal::rust_camera_lidar_optimizer::kDefaultStepTolerance,
    vlcal::rust_camera_lidar_optimizer::kDefaultRelativeCostTolerance,
    1e3,
    1e5};

  const auto initial_begin = to_ffi_pose(*optimized_begin);
  const auto initial_end = to_ffi_pose(*optimized_end);
  LingtuCameraLidarTwoPoseOptimizationResult result{};
  const int rust_status = lingtu_camera_lidar_optimizer_optimize_ct_gicp_dynamic_two_pose(
    &initial_begin,
    &initial_end,
    &prior_pose,
    &between_measurement,
    rust_sources.data(),
    rust_sources.size(),
    rust_targets.data(),
    rust_targets.size(),
    1.0,
    static_cast<std::uint32_t>(3),
    &config,
    &result);
  if (rust_status != vlcal::rust_camera_lidar_optimizer::kOk || result.used_correspondences == 0) {
    return false;
  }

  *optimized_begin = from_ffi_pose(result.pose0);
  *optimized_end = from_ffi_pose(result.pose1);
  return true;
}
#endif
}  // namespace

namespace vlcal {

DynamicPointCloudIntegratorParams::DynamicPointCloudIntegratorParams() {
  visualize = false;
  verbose = false;
  num_threads = 16;
  k_neighbors = 20;
  target_num_points = 10000;
  voxel_resolution = 0.05;
  min_distance = 1.0;
}

DynamicPointCloudIntegratorParams::~DynamicPointCloudIntegratorParams() {}

DynamicPointCloudIntegrator::DynamicPointCloudIntegrator(const DynamicPointCloudIntegratorParams& params) : params(params) {
  last_T_odom_lidar_begin = Eigen::Isometry3d::Identity();
  last_T_odom_lidar_end = Eigen::Isometry3d::Identity();

  target_ivox.reset(new iVox(1.0, 0.05, 100));

  voxelgrid_thread = std::thread([this] { voxelgrid_task(); });
}

DynamicPointCloudIntegrator::~DynamicPointCloudIntegrator() {
  alignment_results.submit_end_of_data();
  if (voxelgrid_thread.joinable()) {
    voxelgrid_thread.join();
  }
}

void DynamicPointCloudIntegrator::insert_points(const Frame::ConstPtr& raw_points_) {
  auto raw_points = sort_by_time(raw_points_);
  auto points = randomgrid_sampling(raw_points, 0.5, static_cast<double>(params.target_num_points) / raw_points->size(), mt);

  std::vector<int> neighbors(points->size() * params.k_neighbors);

  // Find kNN
  KdTree2<Frame> tree(points);
#pragma omp parallel for num_threads(params.num_threads)
  for (int i = 0; i < points->size(); i++) {
    std::vector<size_t> k_indices(params.k_neighbors);
    std::vector<double> k_sq_dists(params.k_neighbors);
    tree.knn_search(points->points[i].data(), params.k_neighbors, k_indices.data(), k_sq_dists.data());
    std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + params.k_neighbors * i);
  }

  // Estimate covariances
  CloudCovarianceEstimation covariance_estimation(params.num_threads);
  points->add_covs(covariance_estimation.estimate(points->points_storage, neighbors));

  if (!target_ivox->has_points()) {
    // Handling the first frame
    target_ivox->insert(*points);
    alignment_results.push(std::make_tuple(raw_points, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity()));
    return;
  }

  const double scan_duration = raw_points->times[raw_points->size() - 1];
  const Sophus::SE3d last_T_begin(last_T_odom_lidar_begin.matrix());
  const Sophus::SE3d last_T_end(last_T_odom_lidar_end.matrix());
  const Vector6d pred_v_odom_lidar = (last_T_begin.inverse() * last_T_end).log() / scan_duration;
  const Eigen::Isometry3d pred_T_odom_lidar_begin = last_T_odom_lidar_end;
  const Eigen::Isometry3d pred_T_odom_lidar_end = from_sophus_pose(last_T_end * Sophus::SE3d::exp(0.75 * pred_v_odom_lidar * scan_duration));

  Eigen::Isometry3d optimized_T_odom_lidar_begin = pred_T_odom_lidar_begin;
  Eigen::Isometry3d optimized_T_odom_lidar_end = pred_T_odom_lidar_end;

  bool optimized_with_rust = false;
#ifdef LINGTU_CAMERA_LIDAR_USE_RUST_OPTIMIZER
  optimized_with_rust = optimize_dynamic_ct_gicp_with_rust(
    target_ivox,
    *points,
    pred_T_odom_lidar_begin,
    &optimized_T_odom_lidar_begin,
    &optimized_T_odom_lidar_end,
    params.num_threads);
#endif

  if (!optimized_with_rust) {
    throw std::runtime_error(
      "Dynamic point cloud integration requires the Rust CT-GICP optimizer, but it "
      "did not return a valid begin/end pose update.");
  }

  last_T_odom_lidar_begin = optimized_T_odom_lidar_begin;
  last_T_odom_lidar_end = optimized_T_odom_lidar_end;

  auto deskewed = std::make_shared<FrameCPU>(
    deskew_points(*points, optimized_T_odom_lidar_begin, optimized_T_odom_lidar_end));
  deskewed->add_covs(covariance_estimation.estimate(deskewed->points_storage, neighbors));
  deskewed->add_intensities(points->intensities, points->size());
  target_ivox->insert(*deskewed);

  alignment_results.push(std::make_tuple(raw_points, optimized_T_odom_lidar_begin, optimized_T_odom_lidar_end));

  if (params.visualize) {
    auto viewer = guik::LightViewer::instance();
    viewer->update_drawable("coord", glk::Primitives::coordinate_system(), guik::VertexColor(last_T_odom_lidar_end.matrix()));
    viewer->update_drawable("points", std::make_shared<glk::PointCloudBuffer>(deskewed->points, deskewed->size()), guik::FlatOrange());
    viewer->update_drawable("target", std::make_shared<glk::PointCloudBuffer>(target_ivox->voxel_points()), guik::Rainbow());
    viewer->spin_once();
  }
}

void DynamicPointCloudIntegrator::voxelgrid_task() {
  while (true) {
    auto data = alignment_results.pop_lock();
    if (!data) {
      break;
    }

    const auto& raw_points = std::get<0>(*data);
    const auto& T_odom_lidar_begin = std::get<1>(*data);
    const auto& T_odom_lidar_end = std::get<2>(*data);
    const double max_timestamp = raw_points->times[raw_points->size() - 1];

    double last_t = -1.0;
    Eigen::Isometry3d T_odom_lidar = T_odom_lidar_begin;

    const double time_eps = 1e-4;
    for (int i = 0; i < raw_points->size(); i++) {
      const double t = raw_points->times[i] / max_timestamp;

      if (t - last_t > time_eps) {
        last_t = t;
        T_odom_lidar = interpolate_rt(T_odom_lidar_begin, T_odom_lidar_end, t);
      }

      const Eigen::Vector4d pt = T_odom_lidar.matrix() * raw_points->points[i];

      if (pt.head<3>().norm() < params.min_distance) {
        continue;
      }

      const Eigen::Vector3i coord = (pt / params.voxel_resolution).array().floor().cast<int>().head<3>();
      voxelgrid[coord] = Eigen::Vector4d(pt[0], pt[1], pt[2], raw_points->intensities[i]);
    }

    /*
    std::vector<Eigen::Vector3f> points;
    std::vector<float> intensities;

    voxelgrid->forEachCell([&](const Eigen::Vector4d& value, const auto& coord) {
      points.emplace_back(value.head<3>().cast<float>());
      intensities.emplace_back(value.w());
    });

    auto viewer = guik::LightViewer::instance();
    auto cloud_buffer = std::make_shared<glk::PointCloudBuffer>(points);
    cloud_buffer->add_intensity(glk::COLORMAP::TURBO, intensities, 1.0f / (*std::max_element(intensities.begin(), intensities.end())));
    viewer->update_drawable("points", cloud_buffer, guik::VertexColor());
    viewer->spin_once();
    */
  }
}

Frame::ConstPtr DynamicPointCloudIntegrator::get_points() {
  alignment_results.submit_end_of_data();
  voxelgrid_thread.join();

  std::vector<Eigen::Vector3f> points;
  std::vector<float> intensities;

  points.reserve(voxelgrid.size());
  intensities.reserve(voxelgrid.size());

  for (const auto& voxel : voxelgrid) {
    points.emplace_back(voxel.second.cast<float>().head<3>());
    intensities.emplace_back(voxel.second.w());
  }

  auto frame = std::make_shared<FrameCPU>(points);
  frame->add_intensities(intensities);
  return frame;
}

}  // namespace vlcal
