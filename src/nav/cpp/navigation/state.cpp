#include "navigation/executor.hpp"

#include <algorithm>
#include <cmath>

namespace lingtu::nav::navigation {
namespace {

double wrappedAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

nav_kernel::LocalKinematicState Executor::planningKinematics(
    const nav_kernel::Pose& planning_body,
    const ExecutionObservation& observation,
    double timestamp_s) {
  nav_kernel::LocalKinematicState state;
  if (!observation.body_velocity_valid ||
      !std::isfinite(observation.body_linear_velocity.x) ||
      !std::isfinite(observation.body_linear_velocity.y) ||
      !std::isfinite(observation.body_linear_velocity.z) ||
      !std::isfinite(observation.body_yaw_rate)) {
    previous_kinematics_time_s_ = -1.0;
    return state;
  }
  const double c = std::cos(planning_body.yaw);
  const double s = std::sin(planning_body.yaw);
  state.linearVelocity = {
      c * observation.body_linear_velocity.x -
          s * observation.body_linear_velocity.y,
      s * observation.body_linear_velocity.x +
          c * observation.body_linear_velocity.y,
      observation.body_linear_velocity.z,
  };
  state.yawRate = observation.body_yaw_rate;
  if (previous_kinematics_time_s_ >= 0.0 &&
      previous_kinematics_frame_epoch_ == observation.frame_epoch &&
      timestamp_s > previous_kinematics_time_s_) {
    const double dt = timestamp_s - previous_kinematics_time_s_;
    state.linearAcceleration = {
        (state.linearVelocity.x - previous_planning_velocity_.x) / dt,
        (state.linearVelocity.y - previous_planning_velocity_.y) / dt,
        (state.linearVelocity.z - previous_planning_velocity_.z) / dt,
    };
  }
  previous_planning_velocity_ = state.linearVelocity;
  previous_kinematics_time_s_ = timestamp_s;
  previous_kinematics_frame_epoch_ = observation.frame_epoch;
  state.valid = true;
  return state;
}

double Executor::goalYawError(const nav_kernel::Pose& odom_map_body) const {
  if (!final_yaw_) return 0.0;
  return wrappedAngle(*final_yaw_ - odom_map_body.yaw);
}

bool Executor::autonomyMotionStalled(
    const nav_kernel::Pose& odom_map_body, double timestamp_s,
    const nav_kernel::LocalKinematicState& kinematics) {
  if (!autonomy_motion_expected_) return false;
  if (!autonomy_progress_valid_ ||
      !std::isfinite(timestamp_s) ||
      timestamp_s < autonomy_progress_time_s_) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    autonomy_progress_valid_ = true;
    return false;
  }

  const double interval = std::max(0.1, config_.recovery.blocked_interval_s);
  const double linear_threshold = config_.recovery.stuck_linear_progress_m;
  const double yaw_threshold = config_.recovery.stuck_yaw_progress_rad;
  const bool observed_linear_motion =
      kinematics.valid && linear_threshold > 0.0 &&
      std::hypot(kinematics.linearVelocity.x, kinematics.linearVelocity.y) >=
          2.0 * linear_threshold / interval;
  const bool observed_yaw_motion =
      kinematics.valid && yaw_threshold > 0.0 &&
      std::abs(kinematics.yawRate) >= 2.0 * yaw_threshold / interval;
  if (observed_linear_motion || observed_yaw_motion) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    return false;
  }

  const double linear_progress = nav_kernel::distance3D(
      autonomy_progress_pose_.position, odom_map_body.position);
  const double yaw_progress = std::abs(
      wrappedAngle(odom_map_body.yaw - autonomy_progress_pose_.yaw));
  if (linear_progress >= config_.recovery.stuck_linear_progress_m ||
      yaw_progress >= config_.recovery.stuck_yaw_progress_rad) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    return false;
  }

  return timestamp_s - autonomy_progress_time_s_ >=
         std::max(0.0, config_.recovery.blocked_interval_s);
}

void Executor::setAutonomyMotionExpected(
    bool expected,
    const nav_kernel::Pose& odom_map_body,
    double timestamp_s) {
  if (!expected) {
    resetAutonomyProgress();
    return;
  }
  if (!autonomy_motion_expected_ || !autonomy_progress_valid_) {
    autonomy_progress_pose_ = odom_map_body;
    autonomy_progress_time_s_ = timestamp_s;
    autonomy_progress_valid_ = true;
  }
  autonomy_motion_expected_ = true;
}

void Executor::resetAutonomyProgress() {
  autonomy_motion_expected_ = false;
  autonomy_progress_valid_ = false;
  autonomy_progress_pose_ = {};
  autonomy_progress_time_s_ = 0.0;
}

bool Executor::recoveryObservationAdvanced(
    const ExecutionObservation& observation) const {
  if (!recovery_observation_waiting_ ||
      observation.frame_epoch != recovery_observation_baseline_.frame_epoch) {
    return false;
  }
  const double barrier_stamp = recovery_observation_baseline_.odom_stamp_s;
  const auto source_after_rotation = [barrier_stamp](double stamp_s) {
    return std::isfinite(stamp_s) && std::isfinite(barrier_stamp) &&
           stamp_s > barrier_stamp + 1e-9;
  };
  if (local_planner_.params().checkObstacle &&
      (observation.cloud_generation <=
           recovery_observation_baseline_.cloud_generation ||
       !source_after_rotation(observation.cloud_stamp_s))) {
    return false;
  }
  if (local_planner_.params().useTraversabilityCost &&
      (observation.traversability_generation <=
           recovery_observation_baseline_.traversability_generation ||
       !source_after_rotation(observation.traversability_stamp_s))) {
    return false;
  }
  return true;
}

void Executor::clearRecoveryObservationWait() {
  recovery_observation_waiting_ = false;
  recovery_observation_baseline_ = {};
}

}  // namespace lingtu::nav::navigation
