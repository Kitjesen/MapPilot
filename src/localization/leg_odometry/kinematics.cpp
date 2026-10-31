#include "kinematics.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::localization::leg_odometry {
namespace {

bool rigid(const Eigen::Isometry3d& transform) {
  if (!transform.matrix().allFinite()) return false;
  const auto& rotation = transform.linear();
  return (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() < 1e-9 &&
         std::abs(rotation.determinant() - 1.0) < 1e-9 &&
         (transform.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).cwiseAbs().maxCoeff() < 1e-9;
}

ContactVelocityResult unavailable(ContactVelocityStatus status) {
  return {status, std::nullopt};
}

}  // namespace

Kinematics forwardKinematics(const LegChain& chain, const Eigen::VectorXd& q_rad,
                            const Eigen::Vector3d& point_tip) {
  if (chain.joints.empty() || q_rad.size() != static_cast<Eigen::Index>(chain.joints.size()) ||
      !q_rad.allFinite() || !point_tip.allFinite() || !rigid(chain.last_link_from_tip))
    throw std::invalid_argument("invalid leg model or joint position");

  Eigen::Isometry3d body_from_link = Eigen::Isometry3d::Identity();
  std::vector<Eigen::Vector3d> origins;
  std::vector<Eigen::Vector3d> axes;
  origins.reserve(chain.joints.size());
  axes.reserve(chain.joints.size());
  for (std::size_t i = 0; i < chain.joints.size(); ++i) {
    const auto& joint = chain.joints[i];
    const double axis_norm = joint.axis_joint.norm();
    if (!rigid(joint.parent_from_joint) || !joint.axis_joint.allFinite() ||
        !std::isfinite(axis_norm) || axis_norm <= 0.0)
      throw std::invalid_argument("invalid revolute joint transform or axis");
    const Eigen::Vector3d axis = joint.axis_joint / axis_norm;
    body_from_link = body_from_link * joint.parent_from_joint;
    origins.push_back(body_from_link.translation());
    axes.push_back(body_from_link.linear() * axis);
    body_from_link.rotate(Eigen::AngleAxisd(q_rad[static_cast<Eigen::Index>(i)], axis));
  }

  Kinematics result;
  result.point_position_body = body_from_link * chain.last_link_from_tip * point_tip;
  result.position_jacobian_body.resize(3, q_rad.size());
  for (Eigen::Index i = 0; i < q_rad.size(); ++i)
    result.position_jacobian_body.col(i) =
        axes[static_cast<std::size_t>(i)].cross(result.point_position_body - origins[static_cast<std::size_t>(i)]);
  if (!result.point_position_body.allFinite() || !result.position_jacobian_body.allFinite())
    throw std::invalid_argument("nonfinite leg kinematics");
  return result;
}

const char* contactVelocityStatusName(ContactVelocityStatus status) noexcept {
  switch (status) {
    case ContactVelocityStatus::Available: return "available";
    case ContactVelocityStatus::UnknownContact: return "unknown_contact";
    case ContactVelocityStatus::Swing: return "swing";
    case ContactVelocityStatus::Slipping: return "slipping";
    case ContactVelocityStatus::UnalignedTime: return "unaligned_time";
    case ContactVelocityStatus::InvalidTime: return "invalid_time";
    case ContactVelocityStatus::FutureSample: return "future_sample";
    case ContactVelocityStatus::StaleSample: return "stale_sample";
    case ContactVelocityStatus::SampleSkew: return "sample_skew";
    case ContactVelocityStatus::InvalidInput: return "invalid_input";
  }
  return "invalid_input";
}

ContactVelocityResult evaluateContactVelocity(
    const LegChain& chain, const ContactObservation& observation,
    const EvaluationWindow& window, const Eigen::Vector3d& point_tip,
    const Eigen::Vector3d& velocity_origin_body,
    const Eigen::Vector3d& predicted_velocity_body_mps) {
  switch (observation.contact) {
    case ContactState::Unknown: return unavailable(ContactVelocityStatus::UnknownContact);
    case ContactState::Swing: return unavailable(ContactVelocityStatus::Swing);
    case ContactState::Slipping: return unavailable(ContactVelocityStatus::Slipping);
    case ContactState::Stance: break;
    default: return unavailable(ContactVelocityStatus::UnknownContact);
  }
  if (!observation.timestamps_aligned) return unavailable(ContactVelocityStatus::UnalignedTime);
  if (!std::isfinite(window.query_stamp_s) || !std::isfinite(window.max_age_s) ||
      !std::isfinite(window.max_skew_s) || window.max_age_s < 0.0 || window.max_skew_s < 0.0 ||
      !std::isfinite(observation.joint_stamp_s) || !std::isfinite(observation.gyro_stamp_s))
    return unavailable(ContactVelocityStatus::InvalidTime);
  if (observation.joint_stamp_s > window.query_stamp_s || observation.gyro_stamp_s > window.query_stamp_s)
    return unavailable(ContactVelocityStatus::FutureSample);
  if (window.query_stamp_s - observation.joint_stamp_s > window.max_age_s ||
      window.query_stamp_s - observation.gyro_stamp_s > window.max_age_s)
    return unavailable(ContactVelocityStatus::StaleSample);
  if (std::abs(observation.joint_stamp_s - observation.gyro_stamp_s) > window.max_skew_s)
    return unavailable(ContactVelocityStatus::SampleSkew);
  if (observation.dq_rps.size() != observation.q_rad.size() || !observation.dq_rps.allFinite() ||
      !observation.unbiased_angular_velocity_body_rps.allFinite() || !velocity_origin_body.allFinite() ||
      !predicted_velocity_body_mps.allFinite())
    return unavailable(ContactVelocityStatus::InvalidInput);

  ContactVelocity value;
  value.measurement_stamp_s = observation.joint_stamp_s;
  try {
    value.kinematics = forwardKinematics(chain, observation.q_rad, point_tip);
  } catch (const std::invalid_argument&) {
    return unavailable(ContactVelocityStatus::InvalidInput);
  }
  const Eigen::Vector3d point_velocity_relative =
      observation.unbiased_angular_velocity_body_rps.cross(value.kinematics.point_position_body - velocity_origin_body) +
      value.kinematics.position_jacobian_body * observation.dq_rps;
  value.velocity_at_origin_body_mps = -point_velocity_relative;
  value.contact_velocity_residual_body_mps = predicted_velocity_body_mps + point_velocity_relative;
  if (!value.velocity_at_origin_body_mps.allFinite() || !value.contact_velocity_residual_body_mps.allFinite())
    return unavailable(ContactVelocityStatus::InvalidInput);
  return {ContactVelocityStatus::Available, std::move(value)};
}

}  // namespace lingtu::localization::leg_odometry
