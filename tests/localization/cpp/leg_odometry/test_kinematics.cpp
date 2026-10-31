#include "localization/leg_odometry/kinematics.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace leg = lingtu::localization::leg_odometry;
namespace {

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

template <class A, class B>
void near(const A& actual, const B& expected, double tolerance, const char* message) {
  require((actual - expected).norm() < tolerance, message);
}

leg::LegChain exampleChain() {
  leg::LegChain chain;
  for (int i = 0; i < 3; ++i) {
    leg::RevoluteJoint joint;
    joint.parent_from_joint.translation() = Eigen::Vector3d(.13 * (i + 1), -.07 * i, -.18);
    joint.parent_from_joint.linear() =
        (Eigen::AngleAxisd(.17 * (i + 1), Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(-.11 * i, Eigen::Vector3d::UnitX())).toRotationMatrix();
    joint.axis_joint = i == 0 ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0, 2, 0);
    chain.joints.push_back(joint);
  }
  chain.last_link_from_tip.translation() = Eigen::Vector3d(.025, -.012, -.22);
  chain.last_link_from_tip.linear() = Eigen::AngleAxisd(.31, Eigen::Vector3d::UnitY()).toRotationMatrix();
  return chain;
}

leg::ContactObservation observation() {
  leg::ContactObservation sample;
  sample.q_rad = Eigen::Vector3d(.2, .7, -1.4);
  sample.dq_rps = Eigen::Vector3d(.3, -.4, .1);
  sample.unbiased_angular_velocity_body_rps = Eigen::Vector3d(.2, -.1, .3);
  sample.contact = leg::ContactState::Stance;
  sample.joint_stamp_s = 10.0;
  sample.gyro_stamp_s = 10.002;
  sample.timestamps_aligned = true;
  return sample;
}

void testKnownTransformAndPoint() {
  leg::LegChain chain;
  leg::RevoluteJoint joint;
  const double pi = std::acos(-1.0);
  joint.parent_from_joint.translation() = Eigen::Vector3d(.2, -.1, .3);
  joint.parent_from_joint.linear() = Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitX()).toRotationMatrix();
  chain.joints.push_back(joint);
  chain.last_link_from_tip.translation() = Eigen::Vector3d(1, 0, 0);
  chain.last_link_from_tip.linear() = Eigen::AngleAxisd(pi / 2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto value = leg::forwardKinematics(chain, Eigen::VectorXd::Constant(1, pi / 2), Eigen::Vector3d(0, 0, .2));
  near(value.point_position_body, Eigen::Vector3d(.2, -.1, 1.5), 1e-12, "fixed origins or selected tip point ignored");
  near(value.position_jacobian_body.col(0), Eigen::Vector3d(-1.2, 0, 0), 1e-12, "joint axis was not rotated into body");
}

void testJacobianAgainstFiniteDifference() {
  const auto chain = exampleChain();
  const Eigen::Vector3d point(.014, -.018, -.021);
  for (int trial = 0; trial < 12; ++trial) {
    Eigen::VectorXd q = Eigen::Vector3d(.11 * trial - .5, .7 - .09 * trial, -.8 + .03 * trial);
    const auto actual = leg::forwardKinematics(chain, q, point);
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      Eigen::VectorXd plus = q, minus = q;
      plus[i] += 1e-6;
      minus[i] -= 1e-6;
      const Eigen::Vector3d derivative =
          (leg::forwardKinematics(chain, plus, point).point_position_body -
           leg::forwardKinematics(chain, minus, point).point_position_body) / 2e-6;
      near(actual.position_jacobian_body.col(i), derivative, 2e-9, "Jacobian differs from independent finite displacement");
    }
  }
}

void testMovingBodyAndLeverArm() {
  const auto chain = exampleChain();
  auto sample = observation();
  const leg::EvaluationWindow window{10.005, .02, .005};
  const Eigen::Vector3d point(.014, -.018, -.021), origin(.09, -.04, .07);
  const auto geometry = leg::forwardKinematics(chain, sample.q_rad, point);
  const Eigen::Vector3d expected =
      -sample.unbiased_angular_velocity_body_rps.cross(geometry.point_position_body - origin) -
      geometry.position_jacobian_body * sample.dq_rps;
  require(expected.norm() > .05, "moving support fixture accidentally has a stationary body");
  const auto result = leg::evaluateContactVelocity(chain, sample, window, point, origin, expected);
  require(result.usable() && result.measurement.has_value(), "valid aligned stance unavailable");
  require(result.measurement->measurement_stamp_s == sample.joint_stamp_s &&
              result.measurement->measurement_stamp_s != window.query_stamp_s,
          "measurement time was replaced with the freshness query time");
  near(result.measurement->velocity_at_origin_body_mps, expected, 1e-12, "stance incorrectly forced body velocity to zero");
  near(result.measurement->contact_velocity_residual_body_mps, Eigen::Vector3d::Zero(), 1e-12, "no-slip residual is not zero");
  const auto body_origin = leg::evaluateContactVelocity(chain, sample, window, point, Eigen::Vector3d::Zero(), expected);
  near(result.measurement->velocity_at_origin_body_mps - body_origin.measurement->velocity_at_origin_body_mps,
       sample.unbiased_angular_velocity_body_rps.cross(origin), 1e-12, "velocity origin lever arm was ignored");
  const Eigen::Vector3d perturbation(.1, -.02, .03);
  const auto mismatched = leg::evaluateContactVelocity(chain, sample, window, point, origin, expected + perturbation);
  near(mismatched.measurement->contact_velocity_residual_body_mps, perturbation, 1e-12, "residual sign or reference frame is wrong");
}

void testQualityAndInvalidInputs() {
  const auto chain = exampleChain();
  const leg::EvaluationWindow window{10.005, .02, .005};
  auto sample = observation();
  auto check = [&](leg::ContactVelocityStatus expected) {
    const auto result = leg::evaluateContactVelocity(chain, sample, window, Eigen::Vector3d::Zero(),
                                                     Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    require(result.status == expected && !result.measurement && !result.usable(), "invalid evidence fabricated a measurement");
  };
  sample.contact = leg::ContactState::Unknown; check(leg::ContactVelocityStatus::UnknownContact);
  sample.contact = leg::ContactState::Swing; check(leg::ContactVelocityStatus::Swing);
  sample.contact = leg::ContactState::Slipping; check(leg::ContactVelocityStatus::Slipping);
  sample = observation(); sample.timestamps_aligned = false; check(leg::ContactVelocityStatus::UnalignedTime);
  sample = observation(); sample.gyro_stamp_s = std::numeric_limits<double>::quiet_NaN(); check(leg::ContactVelocityStatus::InvalidTime);
  sample = observation(); sample.joint_stamp_s = 10.006; check(leg::ContactVelocityStatus::FutureSample);
  sample = observation(); sample.gyro_stamp_s = 9.98; check(leg::ContactVelocityStatus::StaleSample);
  sample = observation(); sample.gyro_stamp_s = 9.99; check(leg::ContactVelocityStatus::SampleSkew);
  sample = observation(); sample.dq_rps.resize(2); check(leg::ContactVelocityStatus::InvalidInput);
  sample = observation(); sample.q_rad[0] = std::numeric_limits<double>::infinity(); check(leg::ContactVelocityStatus::InvalidInput);
  sample = observation(); sample.dq_rps[0] = std::numeric_limits<double>::quiet_NaN(); check(leg::ContactVelocityStatus::InvalidInput);
  auto bad = chain;
  bad.joints[0].parent_from_joint.linear()(0, 0) = 2;
  bool rejected = false;
  try { leg::forwardKinematics(bad, observation().q_rad); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "nonrigid model transform accepted");
  bad = chain; bad.joints[1].axis_joint.setZero(); rejected = false;
  try { leg::forwardKinematics(bad, observation().q_rad); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "undefined joint axis accepted");

  const leg::LegChain missing_model;
  rejected = false;
  try { leg::forwardKinematics(missing_model, Eigen::VectorXd{}); }
  catch (const std::invalid_argument&) { rejected = true; }
  require(rejected, "missing model fabricated rigid-foot kinematics");
  sample = observation();
  sample.q_rad.resize(0);
  sample.dq_rps.resize(0);
  const auto missing = leg::evaluateContactVelocity(missing_model, sample, window,
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  require(missing.status == leg::ContactVelocityStatus::InvalidInput && !missing.measurement,
          "missing model fabricated a zero-velocity measurement");
}

}  // namespace

int main() {
  try {
    testKnownTransformAndPoint();
    testJacobianAgainstFiniteDifference();
    testMovingBodyAndLeverArm();
    testQualityAndInvalidInputs();
    std::cout << "leg kinematics: transforms, selected points, Jacobian, moving stance, lever arm and input quality passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
