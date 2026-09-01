#pragma once

// Ported from SCAN-Planner plan_manage/closed_loop_controller.cpp at
// commit 348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// ROS messages, timers and publishers are replaced by value inputs/outputs;
// the controller equations and execution-clock behavior are unchanged.
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>

#include <Eigen/Eigen>

#include "nav_kernel/types.hpp"
#include "planning/local/scan/upstream/bspline_opt/uniform_bspline.h"

namespace nav_kernel::local::scan::upstream {

struct BsplineTrajectory {
  Eigen::MatrixXd positionPoints{};
  Eigen::VectorXd knots{};
  int order{3};
  std::int64_t trajectoryId{0};
  double startTimeS{0.0};
};

struct ClosedLoopControllerParams {
  double timeForward{0.8};
  double headingErrorThreshold{0.8};
  double positionGain{0.8};
  double yawGain{1.5};
  double maxVx{0.75};
  double maxVy{0.35};
  double maxYawRate{1.0};
  double finishDistance{0.15};
};

struct ClosedLoopControllerState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
  double nowS{0.0};
};

struct ClosedLoopControllerOutput {
  Twist command{};
  double yawError{0.0};
  double endDistance{0.0};
  double executionTimeS{0.0};
  bool executionFrozen{false};
  bool finished{false};
};

class ClosedLoopController {
 public:
  bool setTrajectory(const BsplineTrajectory &trajectory, double nowS);
  ClosedLoopControllerOutput step(const ClosedLoopControllerState &state,
                                  const ClosedLoopControllerParams &params);
  void reset() noexcept;

  [[nodiscard]] bool hasTrajectory() const noexcept;
  [[nodiscard]] std::int64_t trajectoryId() const noexcept;

 private:
  [[nodiscard]] double desiredYaw(double time,
                                  const Eigen::Vector3d &desiredPosition,
                                  double currentYaw,
                                  double timeForward) const;

  bool receiveTrajectory_{false};
  std::int64_t trajectoryId_{0};
  UniformBspline positionTrajectory_{};
  UniformBspline velocityTrajectory_{};
  UniformBspline accelerationTrajectory_{};
  double durationS_{0.0};
  double executionTimeS_{0.0};
  double lastUpdateTimeS_{0.0};
};

}  // namespace nav_kernel::local::scan::upstream
