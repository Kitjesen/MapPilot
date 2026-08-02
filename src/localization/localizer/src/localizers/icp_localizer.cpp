#include "icp_localizer.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <Eigen/Cholesky>
#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <Eigen/Eigenvalues>
#include <pcl/kdtree/kdtree_flann.h>

namespace {

constexpr std::size_t kMinGicpPoints = 20U;

CloudType::Ptr finiteCloud(const CloudType::Ptr &cloud)
{
    auto finite = std::make_shared<CloudType>();
    if (!cloud) {
        return finite;
    }
    finite->reserve(cloud->size());
    for (const PointType &point : cloud->points) {
        if (pcl::isFinite(point)) {
            finite->push_back(point);
        }
    }
    finite->width = static_cast<std::uint32_t>(finite->size());
    finite->height = 1U;
    finite->is_dense = true;
    return finite;
}

Eigen::Matrix3d skew(const Eigen::Vector3d &point)
{
    Eigen::Matrix3d out;
    out << 0.0, -point.z(), point.y(),
           point.z(), 0.0, -point.x(),
           -point.y(), point.x(), 0.0;
    return out;
}

}  // namespace

#ifdef LINGTU_ENABLE_SMALL_GICP
namespace {

double positionCovarianceTrace(const Eigen::Matrix<double, 6, 6> &hessian)
{
    if (!hessian.allFinite()) {
        return -1.0;
    }
    const double det = hessian.determinant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        return -1.0;
    }
    const Eigen::Matrix<double, 6, 6> cov = hessian.inverse();
    const double trace = cov.block<3, 3>(3, 3).trace();
    return std::isfinite(trace) ? trace : -1.0;
}

}  // namespace
#else
namespace {

struct PclRegistrationDiagnostics
{
    int inliers = -1;
    double position_covariance_trace = -1.0;
};

PclRegistrationDiagnostics computePclDiagnostics(
    const CloudType::ConstPtr &source,
    const CloudType::ConstPtr &target,
    const Eigen::Matrix4f &source_to_target,
    double max_correspondence_distance_m)
{
    PclRegistrationDiagnostics diagnostics;
    if (!source || !target || source->empty() || target->empty()) {
        return diagnostics;
    }

    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(target);
    const double max_distance_sq =
        max_correspondence_distance_m * max_correspondence_distance_m;
    Eigen::Matrix<double, 6, 6> hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    double squared_error = 0.0;
    int inliers = 0;
    std::vector<int> indices(1);
    std::vector<float> distances_sq(1);

    for (const PointType &source_point : source->points) {
        if (!pcl::isFinite(source_point)) {
            continue;
        }
        const Eigen::Vector4f homogeneous(
            source_point.x, source_point.y, source_point.z, 1.0F);
        const Eigen::Vector4f transformed = source_to_target * homogeneous;
        if (!transformed.allFinite()) {
            continue;
        }
        PointType query;
        query.x = transformed.x();
        query.y = transformed.y();
        query.z = transformed.z();
        if (tree.nearestKSearch(query, 1, indices, distances_sq) != 1 ||
            !std::isfinite(distances_sq[0]) ||
            static_cast<double>(distances_sq[0]) > max_distance_sq) {
            continue;
        }

        const Eigen::Vector3d point(
            static_cast<double>(query.x),
            static_cast<double>(query.y),
            static_cast<double>(query.z));
        Eigen::Matrix<double, 3, 6> jacobian;
        jacobian.block<3, 3>(0, 0) = -skew(point);
        jacobian.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        hessian.noalias() += jacobian.transpose() * jacobian;
        squared_error += static_cast<double>(distances_sq[0]);
        ++inliers;
    }

    diagnostics.inliers = inliers;
    if (inliers < 6 || !hessian.allFinite()) {
        return diagnostics;
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(hessian);
    if (solver.info() != Eigen::Success) {
        return diagnostics;
    }
    const auto eigenvalues = solver.eigenvalues();
    const double min_eigenvalue = eigenvalues.minCoeff();
    const double max_eigenvalue = eigenvalues.maxCoeff();
    if (!std::isfinite(min_eigenvalue) || !std::isfinite(max_eigenvalue) ||
        max_eigenvalue <= 0.0 || min_eigenvalue / max_eigenvalue < 1e-9) {
        return diagnostics;
    }

    const double degrees_of_freedom = std::max(1, 3 * inliers - 6);
    const double residual_variance =
        std::max(1e-6, squared_error / degrees_of_freedom);
    const Eigen::Matrix<double, 6, 6> inverse_hessian =
        solver.eigenvectors() *
        eigenvalues.cwiseInverse().asDiagonal() *
        solver.eigenvectors().transpose();
    const double trace =
        (residual_variance * inverse_hessian).block<3, 3>(3, 3).trace();
    if (std::isfinite(trace) && trace >= 0.0) {
        diagnostics.position_covariance_trace = trace;
    }
    return diagnostics;
}

}  // namespace
#endif

namespace {

struct FixedTransformDiagnostics
{
    double fitness = -1.0;
    int inliers = -1;
    int evaluated_points = 0;
    double position_covariance_trace = -1.0;
};

FixedTransformDiagnostics evaluateFixedTransform(
    const CloudType::ConstPtr &source,
    pcl::KdTreeFLANN<PointType> &tree,
    const Eigen::Matrix4f &source_to_target,
    double max_correspondence_distance_m,
    const Eigen::Vector3f &target_min_bound,
    const Eigen::Vector3f &target_max_bound)
{
    FixedTransformDiagnostics diagnostics;
    const CloudType::ConstPtr target = tree.getInputCloud();
    if (!source || !target || source->empty() || target->empty() ||
        !source_to_target.allFinite() || !(max_correspondence_distance_m > 0.0) ||
        !target_min_bound.allFinite() || !target_max_bound.allFinite()) {
        return diagnostics;
    }

    const double max_distance_sq =
        max_correspondence_distance_m * max_correspondence_distance_m;
    const Eigen::Vector3f support_margin =
        Eigen::Vector3f::Constant(static_cast<float>(max_correspondence_distance_m));
    const Eigen::Vector3f support_min = target_min_bound - support_margin;
    const Eigen::Vector3f support_max = target_max_bound + support_margin;
    Eigen::Matrix<double, 6, 6> hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    double squared_error = 0.0;
    int inliers = 0;
    std::vector<int> indices(1);
    std::vector<float> distances_sq(1);

    for (const PointType &source_point : source->points) {
        if (!pcl::isFinite(source_point)) {
            continue;
        }
        const Eigen::Vector4f homogeneous(
            source_point.x, source_point.y, source_point.z, 1.0F);
        const Eigen::Vector4f transformed = source_to_target * homogeneous;
        if (!transformed.allFinite()) {
            continue;
        }
        PointType query;
        query.x = transformed.x();
        query.y = transformed.y();
        query.z = transformed.z();
        const Eigen::Vector3f query_position(query.x, query.y, query.z);
        if ((query_position.array() < support_min.array()).any() ||
            (query_position.array() > support_max.array()).any()) {
            continue;
        }
        ++diagnostics.evaluated_points;
        if (tree.nearestKSearch(query, 1, indices, distances_sq) != 1 ||
            !std::isfinite(distances_sq[0]) ||
            static_cast<double>(distances_sq[0]) > max_distance_sq) {
            continue;
        }

        const Eigen::Vector3d point(
            static_cast<double>(query.x),
            static_cast<double>(query.y),
            static_cast<double>(query.z));
        Eigen::Matrix<double, 3, 6> jacobian;
        jacobian.block<3, 3>(0, 0) = -skew(point);
        jacobian.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        hessian.noalias() += jacobian.transpose() * jacobian;
        squared_error += static_cast<double>(distances_sq[0]);
        ++inliers;
    }

    diagnostics.inliers = inliers;
    if (inliers <= 0) {
        return diagnostics;
    }
    diagnostics.fitness = squared_error / static_cast<double>(inliers);
    if (inliers < 6 || !hessian.allFinite()) {
        return diagnostics;
    }
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(hessian);
    if (solver.info() != Eigen::Success) {
        return diagnostics;
    }
    const auto eigenvalues = solver.eigenvalues();
    const double min_eigenvalue = eigenvalues.minCoeff();
    const double max_eigenvalue = eigenvalues.maxCoeff();
    if (!std::isfinite(min_eigenvalue) || !std::isfinite(max_eigenvalue) ||
        max_eigenvalue <= 0.0 || min_eigenvalue / max_eigenvalue < 1e-9) {
        return diagnostics;
    }
    const double degrees_of_freedom = std::max(1, 3 * inliers - 6);
    const double residual_variance =
        std::max(1e-6, squared_error / degrees_of_freedom);
    const Eigen::Matrix<double, 6, 6> inverse_hessian =
        solver.eigenvectors() * eigenvalues.cwiseInverse().asDiagonal() *
        solver.eigenvectors().transpose();
    const double trace =
        (residual_variance * inverse_hessian).block<3, 3>(3, 3).trace();
    if (std::isfinite(trace) && trace >= 0.0) {
        diagnostics.position_covariance_trace = trace;
    }
    return diagnostics;
}

}  // namespace

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);

#ifdef LINGTU_ENABLE_SMALL_GICP
    m_rough_icp.setRegistrationType("GICP");
    m_refine_icp.setRegistrationType("GICP");
    m_rough_icp.setNumThreads(std::max(1, m_config.num_threads));
    m_refine_icp.setNumThreads(std::max(1, m_config.num_threads));
#endif
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    if (reader.read(path, *cloud) < 0) {
        return false;
    }
    return setMap(cloud);
}

bool ICPLocalizer::setMap(const CloudType::Ptr &cloud)
{
    const CloudType::Ptr finite = finiteCloud(cloud);
    if (finite->size() < kMinGicpPoints) {
        return false;
    }
    auto refine_target = std::make_shared<CloudType>();
    auto rough_target = std::make_shared<CloudType>();
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(finite);
        m_voxel_filter.filter(*refine_target);
    }
    else
    {
        pcl::copyPointCloud(*finite, *refine_target);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(finite);
        m_voxel_filter.filter(*rough_target);
    }
    else
    {
        pcl::copyPointCloud(*finite, *rough_target);
    }

    if (rough_target->size() < kMinGicpPoints ||
        refine_target->size() < kMinGicpPoints) {
        return false;
    }
    PointType min_point;
    PointType max_point;
    pcl::getMinMax3D(*refine_target, min_point, max_point);
    const Eigen::Vector3f refine_min_bound =
        Eigen::Vector3f(min_point.x, min_point.y, min_point.z);
    const Eigen::Vector3f refine_max_bound =
        Eigen::Vector3f(max_point.x, max_point.y, max_point.z);
    const bool refine_bounds_valid =
        refine_min_bound.allFinite() && refine_max_bound.allFinite() &&
        (refine_min_bound.array() <= refine_max_bound.array()).all();
    if (!refine_bounds_valid) {
        return false;
    }

    // RegistrationPCL caches search structures by shared_ptr identity. Publish
    // immutable snapshots so a changed map cannot reuse an index built for the
    // previous point count.
    m_refine_tgt = std::move(refine_target);
    m_rough_tgt = std::move(rough_target);
    m_refine_min_bound = refine_min_bound;
    m_refine_max_bound = refine_max_bound;
    m_refine_bounds_valid = true;

    // Build the target search structures once when the map changes. Rebuilding
    // them for every alignment adds avoidable O(N log N) work to the hot path.
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputTarget(m_refine_tgt);
    m_refine_tree.setInputCloud(m_refine_tgt);
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    const CloudType::Ptr finite = finiteCloud(cloud);
    auto refine_input = std::make_shared<CloudType>();
    auto rough_input = std::make_shared<CloudType>();
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(finite);
        m_voxel_filter.filter(*refine_input);
    }
    else
    {
        pcl::copyPointCloud(*finite, *refine_input);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(finite);
        m_voxel_filter.filter(*rough_input);
    }
    else
    {
        pcl::copyPointCloud(*finite, *rough_input);
    }

    // RegistrationPCL skips rebuilding source search structures when the
    // shared_ptr is unchanged. Replacing snapshots prevents stale indices when
    // filtering changes the point count between scans.
    m_refine_inp = std::move(refine_input);
    m_rough_inp = std::move(rough_input);
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr rough_aligned_cloud(new CloudType);
    CloudType::Ptr refine_aligned_cloud(new CloudType);
    m_last_evaluated_points = 0;
    if (!guess.allFinite() ||
        !m_refine_bounds_valid ||
        m_refine_tgt->size() < kMinGicpPoints ||
        m_rough_tgt->size() < kMinGicpPoints ||
        m_refine_inp->size() < kMinGicpPoints ||
        m_rough_inp->size() < kMinGicpPoints) {
        m_last_fitness_score = -1.0;
        m_last_iterations = -1;
        m_last_inliers = -1;
        m_last_converged = false;
        m_last_pos_cov_trace = -1.0;
        return false;
    }
    try {
        // setInputTarget + setMaximumIterations were done in loadMap()
        // (KD-tree already cached).
        m_rough_icp.setInputSource(m_rough_inp);
        m_rough_icp.align(*rough_aligned_cloud, guess);
        const M4F rough_transform = m_rough_icp.getFinalTransformation();
        const double rough_fitness = m_rough_icp.getFitnessScore();
        if (!m_rough_icp.hasConverged() || !rough_transform.allFinite() ||
            !std::isfinite(rough_fitness) ||
            rough_fitness > m_config.rough_score_thresh) {
            m_last_fitness_score = m_rough_icp.getFitnessScore();
            m_last_iterations = -1;
            m_last_inliers = -1;
            m_last_converged = false;
            m_last_pos_cov_trace = -1.0;
            return false;
        }
        m_refine_icp.setInputSource(m_refine_inp);
        m_refine_icp.align(*refine_aligned_cloud, rough_transform);
        const double backend_fitness = m_refine_icp.getFitnessScore();
        m_last_fitness_score = backend_fitness;
        m_last_iterations = -1;
        m_last_inliers = -1;
        m_last_converged = m_refine_icp.hasConverged();
        m_last_pos_cov_trace = -1.0;
        const M4F refine_transform = m_refine_icp.getFinalTransformation();
        if (!m_last_converged || !refine_transform.allFinite() ||
            !std::isfinite(backend_fitness)) {
            return false;
        }
        const double correspondence_distance = std::max(
            0.15,
            3.0 * std::max(
                m_config.refine_scan_resolution,
                m_config.refine_map_resolution));
#ifdef LINGTU_ENABLE_SMALL_GICP
        const auto &detail = m_refine_icp.getRegistrationResult();
        m_last_iterations = static_cast<int>(detail.iterations);
        m_last_inliers = static_cast<int>(detail.num_inliers);
        m_last_converged = detail.converged;
        m_last_pos_cov_trace = positionCovarianceTrace(detail.H);
#else
        const PclRegistrationDiagnostics diagnostics = computePclDiagnostics(
            m_refine_inp,
            m_refine_tgt,
            refine_transform,
            correspondence_distance);
        m_last_inliers = diagnostics.inliers;
        m_last_pos_cov_trace = diagnostics.position_covariance_trace;
#endif
        const FixedTransformDiagnostics support_diagnostics =
            evaluateFixedTransform(
                m_refine_inp,
                m_refine_tree,
                refine_transform,
                correspondence_distance,
                m_refine_min_bound,
                m_refine_max_bound);
        m_last_evaluated_points = support_diagnostics.evaluated_points;
        m_last_inliers = support_diagnostics.inliers;
        m_last_fitness_score = support_diagnostics.fitness;
        if (m_last_pos_cov_trace < 0.0) {
            m_last_pos_cov_trace =
                support_diagnostics.position_covariance_trace;
        }

        if (!m_last_converged ||
            m_last_evaluated_points < static_cast<int>(kMinGicpPoints) ||
            m_last_inliers < static_cast<int>(kMinGicpPoints) ||
            !std::isfinite(m_last_fitness_score) ||
            m_last_fitness_score < 0.0 ||
            m_last_fitness_score > m_config.refine_score_thresh) {
            return false;
        }
        guess = refine_transform;
        return true;
    } catch (const std::exception &e) {
        std::cerr << "[ICPLocalizer] align exception: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "[ICPLocalizer] align unknown exception" << std::endl;
    }
    m_last_fitness_score = -1.0;
    m_last_iterations = -1;
    m_last_inliers = -1;
    m_last_converged = false;
    m_last_pos_cov_trace = -1.0;
    m_last_evaluated_points = 0;
    return false;
}

bool ICPLocalizer::evaluate(
    const M4F &transform,
    double max_correspondence_distance_m)
{
    m_last_evaluated_points = 0;
    if (!m_refine_bounds_valid) {
        m_last_fitness_score = -1.0;
        m_last_iterations = 0;
        m_last_inliers = -1;
        m_last_pos_cov_trace = -1.0;
        m_last_converged = false;
        return false;
    }
    const FixedTransformDiagnostics diagnostics = evaluateFixedTransform(
        m_refine_inp,
        m_refine_tree,
        transform,
        max_correspondence_distance_m,
        m_refine_min_bound,
        m_refine_max_bound);
    m_last_evaluated_points = diagnostics.evaluated_points;
    m_last_fitness_score = diagnostics.fitness;
    m_last_iterations = 0;
    m_last_inliers = diagnostics.inliers;
    m_last_pos_cov_trace = diagnostics.position_covariance_trace;
    m_last_converged =
        m_last_evaluated_points >= static_cast<int>(kMinGicpPoints) &&
        diagnostics.inliers >= static_cast<int>(kMinGicpPoints) &&
        std::isfinite(diagnostics.fitness) && diagnostics.fitness >= 0.0;
    return m_last_converged;
}

bool ICPLocalizer::alignPlanar(
    M4F &guess,
    double max_correspondence_distance_m,
    int max_iterations,
    double max_xy_m,
    double max_z_m,
    double max_yaw_rad)
{
    if (!guess.allFinite() || !(max_correspondence_distance_m > 0.0) ||
        max_iterations <= 0 || max_xy_m < 0.0 || max_z_m < 0.0 ||
        max_yaw_rad < 0.0 ||
        m_refine_inp->size() < kMinGicpPoints ||
        m_refine_tgt->size() < kMinGicpPoints) {
        return false;
    }
    if (!evaluate(guess, max_correspondence_distance_m)) {
        return false;
    }

    const M4F seed = guess;
    M4F best = seed;
    double best_fitness = m_last_fitness_score;
    int completed_iterations = 0;
    const double max_distance_sq =
        max_correspondence_distance_m * max_correspondence_distance_m;
    const double huber_threshold =
        std::max(0.02, 0.5 * max_correspondence_distance_m);
    const CloudType::ConstPtr target = m_refine_tree.getInputCloud();
    if (!target || target->empty()) {
        return false;
    }

    M4F current = seed;
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        Eigen::Matrix4d hessian = Eigen::Matrix4d::Zero();
        Eigen::Vector4d gradient = Eigen::Vector4d::Zero();
        double weighted_squared_error = 0.0;
        int inliers = 0;
        std::vector<int> indices(1);
        std::vector<float> distances_sq(1);

        for (const PointType &source_point : m_refine_inp->points) {
            const Eigen::Vector4f homogeneous(
                source_point.x, source_point.y, source_point.z, 1.0F);
            const Eigen::Vector4f transformed = current * homogeneous;
            if (!transformed.allFinite()) {
                continue;
            }
            PointType query;
            query.x = transformed.x();
            query.y = transformed.y();
            query.z = transformed.z();
            if (m_refine_tree.nearestKSearch(query, 1, indices, distances_sq) != 1 ||
                !std::isfinite(distances_sq[0]) ||
                static_cast<double>(distances_sq[0]) > max_distance_sq) {
                continue;
            }

            const PointType &match = target->points[static_cast<std::size_t>(indices[0])];
            const Eigen::Vector3d residual(
                static_cast<double>(match.x - query.x),
                static_cast<double>(match.y - query.y),
                static_cast<double>(match.z - query.z));
            const double residual_norm = residual.norm();
            const double weight = residual_norm > huber_threshold
                ? huber_threshold / residual_norm
                : 1.0;
            const Eigen::Vector3f rotated =
                current.block<3, 3>(0, 0) * homogeneous.head<3>();
            Eigen::Matrix<double, 3, 4> jacobian;
            jacobian << -static_cast<double>(rotated.y()), 1.0, 0.0, 0.0,
                         static_cast<double>(rotated.x()), 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0, 1.0;
            hessian.noalias() += weight * jacobian.transpose() * jacobian;
            gradient.noalias() += weight * jacobian.transpose() * residual;
            weighted_squared_error += weight * residual.squaredNorm();
            ++inliers;
        }

        if (inliers < static_cast<int>(kMinGicpPoints) ||
            !hessian.allFinite() || !gradient.allFinite()) {
            break;
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(hessian);
        if (solver.info() != Eigen::Success) {
            break;
        }
        const auto eigenvalues = solver.eigenvalues();
        const double min_eigenvalue = eigenvalues.minCoeff();
        const double max_eigenvalue = eigenvalues.maxCoeff();
        if (!std::isfinite(min_eigenvalue) || !std::isfinite(max_eigenvalue) ||
            min_eigenvalue <= 0.0 || max_eigenvalue <= 0.0 ||
            min_eigenvalue / max_eigenvalue < 1e-9) {
            break;
        }
        const Eigen::Matrix3d translation_hessian =
            hessian.block<3, 3>(1, 1);
        const Eigen::LDLT<Eigen::Matrix3d> translation_ldlt(translation_hessian);
        if (translation_ldlt.info() != Eigen::Success) {
            break;
        }
        const Eigen::Vector3d yaw_translation = hessian.block<1, 3>(0, 1).transpose();
        const double yaw_information = hessian(0, 0) -
            yaw_translation.dot(translation_ldlt.solve(yaw_translation));
        const double residual_variance = weighted_squared_error /
            static_cast<double>(std::max(1, 3 * inliers - 4));
        const double yaw_variance = yaw_information > 0.0
            ? residual_variance / yaw_information
            : std::numeric_limits<double>::infinity();
        constexpr double kMaxYawStddevRad = 0.017453292519943295;
        if (!std::isfinite(yaw_variance) || yaw_variance < 0.0 ||
            std::sqrt(yaw_variance) > kMaxYawStddevRad) {
            break;
        }

        Eigen::Vector4d delta = solver.eigenvectors() *
            eigenvalues.cwiseInverse().asDiagonal() *
            solver.eigenvectors().transpose() * gradient;
        if (!delta.allFinite()) {
            break;
        }
        delta[0] = std::clamp(delta[0], -0.5 * max_yaw_rad, 0.5 * max_yaw_rad);
        Eigen::Vector2d translation_xy = delta.segment<2>(1);
        const double per_iteration_xy = 0.5 * max_xy_m;
        if (translation_xy.norm() > per_iteration_xy && translation_xy.norm() > 0.0) {
            translation_xy *= per_iteration_xy / translation_xy.norm();
            delta.segment<2>(1) = translation_xy;
        }
        delta[3] = std::clamp(delta[3], -0.5 * max_z_m, 0.5 * max_z_m);

        const float c = static_cast<float>(std::cos(delta[0]));
        const float s = static_cast<float>(std::sin(delta[0]));
        Eigen::Matrix3f yaw_increment = Eigen::Matrix3f::Identity();
        yaw_increment(0, 0) = c;
        yaw_increment(0, 1) = -s;
        yaw_increment(1, 0) = s;
        yaw_increment(1, 1) = c;
        M4F candidate = current;
        candidate.block<3, 3>(0, 0) =
            yaw_increment * current.block<3, 3>(0, 0);
        candidate(0, 3) += static_cast<float>(delta[1]);
        candidate(1, 3) += static_cast<float>(delta[2]);
        candidate(2, 3) += static_cast<float>(delta[3]);
        const Eigen::Vector3d total_translation =
            (candidate.block<3, 1>(0, 3) - seed.block<3, 1>(0, 3)).cast<double>();
        const Eigen::Matrix3f correction_rotation =
            candidate.block<3, 3>(0, 0) * seed.block<3, 3>(0, 0).transpose();
        const double correction_yaw = std::abs(std::atan2(
            static_cast<double>(correction_rotation(1, 0)),
            static_cast<double>(correction_rotation(0, 0))));
        if (!candidate.allFinite() || total_translation.head<2>().norm() > max_xy_m ||
            std::abs(total_translation.z()) > max_z_m || correction_yaw > max_yaw_rad) {
            break;
        }
        if (!evaluate(candidate, max_correspondence_distance_m)) {
            break;
        }
        ++completed_iterations;
        if (m_last_fitness_score <= best_fitness) {
            best = candidate;
            best_fitness = m_last_fitness_score;
            current = candidate;
        } else {
            break;
        }
        if (std::abs(delta[0]) < 1e-5 && delta.tail<3>().norm() < 1e-4) {
            break;
        }
    }

    const Eigen::Vector3d best_translation =
        (best.block<3, 1>(0, 3) - seed.block<3, 1>(0, 3)).cast<double>();
    const Eigen::Matrix3f best_rotation =
        best.block<3, 3>(0, 0) * seed.block<3, 3>(0, 0).transpose();
    const double best_yaw = std::abs(std::atan2(
        static_cast<double>(best_rotation(1, 0)),
        static_cast<double>(best_rotation(0, 0))));
    if (best_translation.head<2>().norm() < 0.002 &&
        std::abs(best_translation.z()) < 0.002 &&
        best_yaw < 0.00034906585039886593) {
        best = seed;
    }
    guess = best;
    if (!evaluate(best, max_correspondence_distance_m)) {
        return false;
    }
    m_last_iterations = completed_iterations;
    return true;
}
