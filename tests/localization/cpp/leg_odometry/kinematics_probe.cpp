#include "kinematics.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace leg = lingtu::localization::leg_odometry;

namespace {
void readVector(Eigen::Vector3d& vector) {
  for (int i = 0; i < 3; ++i) std::cin >> vector[i];
}

void readTransform(Eigen::Isometry3d& transform) {
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col) std::cin >> transform.matrix()(row, col);
}

template <typename Derived>
void writeVector(const Eigen::MatrixBase<Derived>& vector) {
  std::cout << '[';
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    if (i) std::cout << ',';
    std::cout << vector[i];
  }
  std::cout << ']';
}

leg::ContactState contactState(const std::string& value) {
  if (value == "stance") return leg::ContactState::Stance;
  if (value == "swing") return leg::ContactState::Swing;
  if (value == "slipping") return leg::ContactState::Slipping;
  if (value == "unknown") return leg::ContactState::Unknown;
  throw std::invalid_argument("unknown contact state");
}
}  // namespace

// Test-only numeric protocol; one complete case produces one JSON line. Model
// transforms, axes and state are supplied by the external URDF/oracle harness.
int main() {
  try {
    std::cout << std::setprecision(17);
    int count = 0;
    while (std::cin >> count) {
      if (count <= 0) throw std::invalid_argument("empty chain");
      leg::LegChain chain;
      chain.joints.resize(static_cast<std::size_t>(count));
      for (auto& joint : chain.joints) {
        readTransform(joint.parent_from_joint);
        readVector(joint.axis_joint);
      }
      readTransform(chain.last_link_from_tip);
      Eigen::Vector3d point, origin, predicted;
      readVector(point);
      leg::ContactObservation observation;
      observation.q_rad.resize(count);
      observation.dq_rps.resize(count);
      for (int i = 0; i < count; ++i) std::cin >> observation.q_rad[i];
      for (int i = 0; i < count; ++i) std::cin >> observation.dq_rps[i];
      readVector(observation.unbiased_angular_velocity_body_rps);
      readVector(origin);
      readVector(predicted);
      leg::EvaluationWindow window;
      int aligned = 0;
      std::string contact;
      std::cin >> observation.joint_stamp_s >> observation.gyro_stamp_s
               >> window.query_stamp_s >> window.max_age_s >> window.max_skew_s
               >> aligned >> contact;
      if (!std::cin) throw std::invalid_argument("incomplete probe input");
      observation.timestamps_aligned = aligned != 0;
      observation.contact = contactState(contact);
      const auto kinematics = leg::forwardKinematics(chain, observation.q_rad, point);
      const auto result = leg::evaluateContactVelocity(
          chain, observation, window, point, origin, predicted);
      std::cout << "{\"point\":";
      writeVector(kinematics.point_position_body);
      std::cout << ",\"jacobian\":[";
      for (int row = 0; row < 3; ++row) {
        if (row) std::cout << ',';
        writeVector(kinematics.position_jacobian_body.row(row).transpose());
      }
      std::cout << "],\"status\":\"" << leg::contactVelocityStatusName(result.status)
                << "\",\"usable\":" << (result.usable() ? "true" : "false");
      if (result.measurement) {
        std::cout << ",\"velocity\":";
        writeVector(result.measurement->velocity_at_origin_body_mps);
        std::cout << ",\"residual\":";
        writeVector(result.measurement->contact_velocity_residual_body_mps);
      }
      std::cout << "}\n";
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
