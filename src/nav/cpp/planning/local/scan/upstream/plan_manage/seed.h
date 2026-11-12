#pragma once

// Seed construction extracted from LingTu's SCAN manager.
// Upstream base: SCAN-Planner 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// Velocity ramps and 3-D reference seeding are local adaptations.
// SPDX-License-Identifier: Apache-2.0

#include "planning/local/scan/upstream/plan_manage/plan_container.hpp"

namespace nav_kernel::local::scan::upstream::seed {

Eigen::VectorXd referenceDurations(const std::vector<Eigen::Vector3d> &points,
    const Eigen::Vector3d &startVelocity, const Eigen::Vector3d &endVelocity,
    double maxVelocity, double maxAcceleration);

bool boundedVelocitySeed(const PlanParameters &params,
                      const Eigen::Vector3d &start,
                      const Eigen::Vector3d &start_velocity,
                      const Eigen::Vector3d &start_acceleration,
                      const Eigen::Vector3d &target,
                      const Eigen::Vector3d &target_velocity,
                      PolynomialTraj &trajectory, double &duration,
                      double &interval);

void applyLinearZReference(std::vector<Eigen::Vector3d> &points,
                           const double start_z, const double target_z);

bool applyReferenceSeed(std::vector<Eigen::Vector3d> &points, double &interval,
                        GlobalTrajData &reference,
                        const std::pair<double,double> &window,
                        const Eigen::Vector3d &start, const Eigen::Vector3d &target,
                        const double spacing,
                        const std::vector<Eigen::Vector3d> *planarReference,
                        double &referenceProgress);

}  // namespace nav_kernel::local::scan::upstream::seed
