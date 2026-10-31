// SPDX-License-Identifier: GPL-2.0-only
// Scalar point residual and bounded voxel search adapted from GenZ-LIO,
// cocel-postech/genz-lio bf444f7a747ed3da8548b56b49c14f207e8d49bb,
// core/HybridMetricUpdate.cpp and core/VoxelPrunedSearch.cpp.
#pragma once

#include "ProbabilityPlane.hpp"
#include <Eigen/Geometry>
#include <cstddef>
#include <vector>

namespace lingtu::ltlio::detail {

struct HybridMapPoint {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

struct HybridPointBucket {
    Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
    Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
    std::vector<HybridMapPoint> points;
};

struct HybridPointMatch {
    bool found = false;
    HybridMapPoint target;
    std::size_t candidate_points = 0;
    std::size_t visited_buckets = 0;
    std::size_t evaluated_points = 0;
};

inline double pointBucketDistanceSquared(const Eigen::Vector3d& query,
                                        const HybridPointBucket& bucket) {
    return (query.cwiseMax(bucket.minimum).cwiseMin(bucket.maximum) - query).squaredNorm();
}

inline HybridPointMatch findHybridPointMatch(const Eigen::Vector3d& query,
        const std::vector<HybridPointBucket>& buckets,
        const std::vector<std::size_t>& candidates, double max_distance_squared,
        bool prune) {
    HybridPointMatch result;
    double closest = max_distance_squared;
    for (const auto index : candidates) {
        const auto& bucket = buckets[index];
        // Statistical support is independent of the traversal optimization.
        // Otherwise enabling pruning would silently change R_disc and the estimate.
        result.candidate_points += bucket.points.size();
        if (prune && pointBucketDistanceSquared(query, bucket) >= closest) continue;
        ++result.visited_buckets;
        for (const auto& point : bucket.points) {
            ++result.evaluated_points;
            const double distance = (point.position - query).squaredNorm();
            if (distance >= closest) continue;
            closest = distance;
            result.target = point;
            result.found = true;
        }
    }
    return result;
}

inline double hybridPointGateSquared(const Eigen::Vector3d& point_lidar,
        const Eigen::Quaterniond& correction_rotation,
        const Eigen::Vector3d& correction_translation, double initial_threshold) {
    const double range = std::clamp(point_lidar.norm(), 0.1, 100.0);
    const double rotation_motion = 2.0 * range *
        std::sin(0.5 * Eigen::AngleAxisd(correction_rotation).angle());
    const double motion = correction_translation.norm() + rotation_motion;
    const double sigma = std::max(initial_threshold, motion);
    return 2.5 * 2.5 * sigma * sigma;
}

inline double hybridDiscretizationVariance(std::size_t candidate_roots,
                                          std::size_t candidate_points) {
    // Probability map roots are 1 m on each axis. Root support is counted before
    // pruning, so exact and pruned searches describe the same measurement model.
    return candidate_points > 0 ? static_cast<double>(candidate_roots) /
        static_cast<double>(candidate_points) : 0.0;
}

inline bool linearizeHybridPoint(const Eigen::Vector3d& query,
        const HybridMapPoint& target, const Eigen::Matrix3d& query_sensor_covariance,
        double discretization_variance, double weight,
        Eigen::Vector3d& direction, double& residual, double& variance) {
    const Eigen::Vector3d difference = query - target.position;
    residual = difference.norm();
    if (!(residual > 1e-9) || !std::isfinite(residual)) return false;
    direction = difference / residual;
    variance = weight * ((direction.transpose() *
        (query_sensor_covariance + target.covariance) * direction).value() +
        discretization_variance);
    return variance > 0.0 && std::isfinite(variance);
}

}  // namespace lingtu::ltlio::detail
