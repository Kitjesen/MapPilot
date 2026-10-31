#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <optional>
#include <vector>

namespace lingtu::localization::leg_odometry {

struct RevoluteJoint {
  // Fixed transform before this joint's rotation. Axis is in joint coordinates.
  Eigen::Isometry3d parent_from_joint = Eigen::Isometry3d::Identity();
  Eigen::Vector3d axis_joint = Eigen::Vector3d::UnitZ();
};

struct LegChain {
  std::vector<RevoluteJoint> joints;
  Eigen::Isometry3d last_link_from_tip = Eigen::Isometry3d::Identity();
};

struct Kinematics {
  Eigen::Vector3d point_position_body = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 3, Eigen::Dynamic> position_jacobian_body;
};

// All translations are metres, q is radians. A point in the tip frame must be
// selected explicitly when it differs from the frame origin. No contact or
// robot dimensions are inferred. Malformed model/state inputs throw invalid_argument.
Kinematics forwardKinematics(
    const LegChain& chain, const Eigen::VectorXd& q_rad,
    const Eigen::Vector3d& point_tip = Eigen::Vector3d::Zero());

enum class ContactState { Unknown, Swing, Stance, Slipping };

struct ContactObservation {
  Eigen::VectorXd q_rad;
  Eigen::VectorXd dq_rps;
  // Bias-corrected angular velocity expressed in the chain's body frame.
  Eigen::Vector3d unbiased_angular_velocity_body_rps = Eigen::Vector3d::Zero();
  // Contact classification is supplied at joint_stamp_s. Stance means the
  // caller is testing the no-slip assumption for this selected material point.
  ContactState contact = ContactState::Unknown;
  double joint_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double gyro_stamp_s = std::numeric_limits<double>::quiet_NaN();
  // Known common clock domain; samples can still differ by max_skew_s.
  bool timestamps_aligned = false;
};

struct EvaluationWindow {
  // Freshness check time, not the time of the predicted state or measurement.
  double query_stamp_s = std::numeric_limits<double>::quiet_NaN();
  double max_age_s = 0.0;
  double max_skew_s = 0.0;
};

enum class ContactVelocityStatus {
  Available,
  UnknownContact,
  Swing,
  Slipping,
  UnalignedTime,
  InvalidTime,
  FutureSample,
  StaleSample,
  SampleSkew,
  InvalidInput,
};

const char* contactVelocityStatusName(ContactVelocityStatus status) noexcept;

struct ContactVelocity {
  double measurement_stamp_s = std::numeric_limits<double>::quiet_NaN();
  Kinematics kinematics;
  Eigen::Vector3d velocity_at_origin_body_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d contact_velocity_residual_body_mps = Eigen::Vector3d::Zero();
};

struct ContactVelocityResult {
  ContactVelocityStatus status = ContactVelocityStatus::InvalidInput;
  // Unavailable input never fabricates a zero-velocity observation.
  std::optional<ContactVelocity> measurement;
  bool usable() const noexcept { return status == ContactVelocityStatus::Available; }
};

// Local diagnostic under an externally supplied no-slip contact assumption:
//   v_origin = -omega x (p_point - r_origin) - J(q) dq.
// origin is a reference point fixed to the body (e.g. its IMU mount), expressed
// in the chain's body frame. A point moving relative to the body needs its own
// relative velocity and is not supported by this diagnostic.
// predicted_velocity_body_mps must refer to that same point and frame at
// observation.joint_stamp_s. The gyro is merely accepted within max_skew_s;
// this function does not interpolate samples or propagate a state from query time.
// This returns no measurement covariance and uses caller-supplied gyro/contact
// information. It is not an independent fusion factor or a contact detector.
ContactVelocityResult evaluateContactVelocity(
    const LegChain& chain, const ContactObservation& observation,
    const EvaluationWindow& window, const Eigen::Vector3d& point_tip,
    const Eigen::Vector3d& velocity_origin_body,
    const Eigen::Vector3d& predicted_velocity_body_mps);

}  // namespace lingtu::localization::leg_odometry
