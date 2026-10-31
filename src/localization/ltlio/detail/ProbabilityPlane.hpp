// SPDX-License-Identifier: GPL-2.0-or-later
// Plane-parameter Jacobians adapted from GenZ-LIO / VoxelMap (GPL v2),
// cocel-postech/genz-lio bf444f7a747ed3da8548b56b49c14f207e8d49bb,
// core/VoxelMap.hpp::initPlane and core/HybridMetricUpdate.cpp::fillPlaneRows.
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lingtu::ltlio::detail {

struct ProbabilityPlane {
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    // Joint covariance of [unit normal, center], including their cross terms.
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
    double radius = 0.0;
    std::size_t support_points = 0;
};

inline Eigen::Matrix3d probabilitySkew(const Eigen::Vector3d& point) {
    Eigen::Matrix3d result;
    result << 0.0, -point.z(), point.y(), point.z(), 0.0, -point.x(),
              -point.y(), point.x(), 0.0;
    return result;
}

inline Eigen::Matrix3d matchingPointPoseCovariance(
        const Eigen::Vector3d& point_imu, const Eigen::Matrix3d& rotation,
        const Eigen::MatrixXd& state_covariance) {
    Eigen::Matrix<double, 3, 6> jacobian;
    jacobian.leftCols<3>() = -rotation * probabilitySkew(point_imu);
    jacobian.rightCols<3>().setIdentity();
    // State indices [R,P] are 0..5. The active elevator transport height is
    // absent: the matching coordinates are R*p_imu + state.p until mode exit.
    // Retain rotation/translation cross covariance, not only diagonal blocks.
    return jacobian * state_covariance.topLeftCorner<6, 6>() * jacobian.transpose();
}

inline bool fitProbabilityPlane(const std::vector<Eigen::Vector3d>& points,
        const std::vector<Eigen::Matrix3d>& point_covariances,
        double max_rmse, ProbabilityPlane& plane) {
    if (points.size() < 4 || points.size() != point_covariances.size()) return false;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    Eigen::Vector3d center;
    int skip_index = -1;
    int count = static_cast<int>(points.size());
    const auto fit = [&]() {
        center.setZero();
        for (int i = 0; i < static_cast<int>(points.size()); ++i)
            if (i != skip_index) center += points[i];
        center /= count;
        Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
        for (int i = 0; i < static_cast<int>(points.size()); ++i) {
            if (i == skip_index) continue;
            const Eigen::Vector3d delta = points[i] - center;
            scatter.noalias() += delta * delta.transpose();
        }
        solver.compute(scatter / count);
        return solver.info() == Eigen::Success && solver.eigenvalues().allFinite();
    };
    if (!fit()) return false;
    // Preserve LTLIO's thickness threshold and one-outlier fit policy.
    if (std::sqrt(std::max(0.0, solver.eigenvalues()[0])) > max_rmse) {
        if (count <= 4) return false;
        double largest_distance = -1.0;
        for (int i = 0; i < count; ++i) {
            const double distance = std::pow(solver.eigenvectors().col(0).dot(points[i] - center), 2);
            if (distance > largest_distance) { largest_distance = distance; skip_index = i; }
        }
        --count;
        if (!fit()) return false;
        if (std::sqrt(std::max(0.0, solver.eigenvalues()[0])) > max_rmse) return false;
    }
    const Eigen::Vector3d eigenvalues = solver.eigenvalues();
    const Eigen::Matrix3d eigenvectors = solver.eigenvectors();
    const double gap_tolerance = 64.0 * std::numeric_limits<double>::epsilon() *
        std::max(0.0, eigenvalues[2]);
    if (eigenvalues[1] - std::max(0.0, eigenvalues[0]) <= gap_tolerance ||
        eigenvalues[1] < 2.0 * eigenvalues[0]) return false;
    const double sign = eigenvectors.col(0).dot(center) > 0.0 ? -1.0 : 1.0;
    plane.normal = sign * eigenvectors.col(0);
    plane.center = center;
    plane.support_points = static_cast<std::size_t>(count);
    plane.radius = std::sqrt(std::max(0.0, eigenvalues[2]));
    plane.covariance.setZero();
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        if (i == skip_index) continue;
        Eigen::Matrix3d normal_jacobian = Eigen::Matrix3d::Zero();
        const Eigen::Vector3d delta = points[i] - center;
        for (int axis = 1; axis < 3; ++axis) {
            const Eigen::Vector3d tangent = eigenvectors.col(axis);
            const Eigen::RowVector3d derivative = delta.transpose() *
                (tangent * eigenvectors.col(0).transpose() +
                 eigenvectors.col(0) * tangent.transpose()) /
                (count * (eigenvalues[0] - eigenvalues[axis]));
            normal_jacobian.noalias() += sign * tangent * derivative;
        }
        Eigen::Matrix<double, 6, 3> jacobian;
        jacobian.topRows<3>() = normal_jacobian;
        jacobian.bottomRows<3>() = Eigen::Matrix3d::Identity() / count;
        plane.covariance.noalias() += jacobian * point_covariances[i] * jacobian.transpose();
    }
    return plane.covariance.allFinite();
}

inline double planeResidualVariance(const ProbabilityPlane& plane,
        const Eigen::Vector3d& point_world, const Eigen::Matrix3d& measurement_covariance) {
    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.leftCols<3>() = (point_world - plane.center).transpose();
    jacobian.rightCols<3>() = -plane.normal.transpose();
    return (jacobian * plane.covariance * jacobian.transpose()).value() +
        (plane.normal.transpose() * measurement_covariance * plane.normal).value();
}

inline double scaledPlaneInformation(double prior_scale, double residual_variance) {
    if (!(residual_variance > 0.0) || !std::isfinite(residual_variance)) return 0.0;
    // Author physical precision bounds. LTLIO solver information is w/s;
    // multiply by s here so it sees 1/R exactly once.
    return prior_scale * std::clamp(1.0 / residual_variance, 1e-5, 1e5);
}

}  // namespace lingtu::ltlio::detail
