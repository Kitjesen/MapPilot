#pragma once

#include "../geometry_diagnostics.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace lingtu::ltlio::detail {

using GeometryJacobian = Eigen::Matrix<double, Eigen::Dynamic, 6, Eigen::RowMajor>;

// Geometry diagnostics inspired by localizability analysis, not an X-ICP
// reproduction. RMS lever-arm scaling separates scene scale from rank, and
// averaging prevents repeated sampling from manufacturing information.
inline GeometryDiagnostics analyzeLocalizability(
    const GeometryJacobian& pose_jacobian,
    const std::vector<double>& inverse_variances,
    const Eigen::Matrix3d& world_from_imu) {
    GeometryDiagnostics result;
    if (pose_jacobian.rows() < 10 || !pose_jacobian.allFinite() ||
        !world_from_imu.allFinite() ||
        inverse_variances.size() != static_cast<std::size_t>(pose_jacobian.rows()))
        return result;

    double weight_sum = 0.0;
    double rotation_energy = 0.0;
    double translation_energy = 0.0;
    for (Eigen::Index row = 0; row < pose_jacobian.rows(); ++row) {
        const double weight = inverse_variances[static_cast<std::size_t>(row)];
        if (!std::isfinite(weight) || weight <= 0.0) return result;
        weight_sum += weight;
        rotation_energy += weight * pose_jacobian.row(row).head<3>().squaredNorm();
        translation_energy += weight * pose_jacobian.row(row).tail<3>().squaredNorm();
    }
    if (!std::isfinite(weight_sum) || !std::isfinite(rotation_energy) ||
        !std::isfinite(translation_energy) || translation_energy <= 0.0)
        return result;
    const double rotation_scale = rotation_energy > translation_energy * 1e-12
        ? std::sqrt(rotation_energy / translation_energy) : 1.0;

    GeometryJacobian scaled = pose_jacobian;
    scaled.leftCols<3>() /= rotation_scale;
    // The estimator rotation error is in IMU axes; its translation error is
    // in world axes. Put both in the same frame before analysing mixed modes.
    scaled.rightCols<3>() = (pose_jacobian.rightCols<3>() * world_from_imu).eval();
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    for (Eigen::Index row = 0; row < scaled.rows(); ++row)
        information.noalias() += inverse_variances[static_cast<std::size_t>(row)] *
            scaled.row(row).transpose() * scaled.row(row) / weight_sum;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigen(information);
    if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
        eigen.eigenvalues().maxCoeff() <= 0.0)
        return result;

    result.rotation_scale_m = rotation_scale;
    result.min_eigenvalue = std::max(0.0, eigen.eigenvalues()(0));
    result.max_eigenvalue = eigen.eigenvalues()(5);
    result.condition_number = result.min_eigenvalue > result.max_eigenvalue * 1e-12
        ? result.max_eigenvalue / result.min_eigenvalue
        : std::numeric_limits<double>::infinity();
    // Initial diagnostic thresholds, deliberately independent of point count.
    // They do not alter filter updates or establish field qualification.
    constexpr double kMinimumAlignment = 0.17364817766693033;  // cos(80 deg).
    constexpr double kUnobservableRelativeInformation = 1e-4;
    constexpr double kPartialRelativeInformation = 0.02;
    for (int mode = 0; mode < 6; ++mode) {
        Eigen::Matrix<double, 6, 1> direction = eigen.eigenvectors().col(mode);
        Eigen::Index sign_axis;
        direction.cwiseAbs().maxCoeff(&sign_axis);
        if (direction(sign_axis) < 0.0) direction = -direction;
        double support = 0.0;
        for (Eigen::Index row = 0; row < scaled.rows(); ++row) {
            const double norm = scaled.row(row).norm();
            if (norm > 0.0 && std::abs(scaled.row(row).dot(direction)) / norm >= kMinimumAlignment)
                support += inverse_variances[static_cast<std::size_t>(row)];
        }
        const double value = std::max(0.0, eigen.eigenvalues()(mode));
        const double relative_information = value / result.max_eigenvalue;
        const double support_ratio = support / weight_sum;
        result.eigenvalues[static_cast<std::size_t>(mode)] = value;
        result.support_ratios[static_cast<std::size_t>(mode)] = support_ratio;
        for (int axis = 0; axis < 6; ++axis)
            result.principal_directions[static_cast<std::size_t>(mode)][static_cast<std::size_t>(axis)] =
                direction(axis);
        auto& category = result.categories[static_cast<std::size_t>(mode)];
        if (relative_information <= kUnobservableRelativeInformation || support_ratio < 0.01)
            category = "UNOBSERVABLE";
        else if (relative_information < kPartialRelativeInformation || support_ratio < 0.1)
            category = "PARTIAL";
        else
            category = "OBSERVABLE";
        if (category != "OBSERVABLE") ++result.degenerate_dof_count;
    }
    result.available = true;
    result.detected = result.degenerate_dof_count > 0;
    result.guidance = result.detected ? "turn_or_retrace_for_features" : "none";
    return result;
}

}  // namespace lingtu::ltlio::detail
