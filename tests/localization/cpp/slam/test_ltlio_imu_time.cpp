#include "ltlio/detail/Context.h"
#include "ltlio/detail/IMUProcess.h"

#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
namespace lio = lingtu::ltlio;
namespace detail = lingtu::ltlio::detail;
constexpr double kInterval = 0.01;
const Eigen::Vector3d kLeftAcceleration(0.2, -0.1, 9.81);
const Eigen::Vector3d kRightAcceleration(0.6, 0.3, 9.9);
const Eigen::Vector3d kLeftGyro(0.1, 0.2, 0.3);
const Eigen::Vector3d kRightGyro(0.3, 0.4, 0.5);

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double error, double tolerance, const std::string& message) {
  require(std::isfinite(error) && std::abs(error) <= tolerance,
          message + ": error=" + std::to_string(error));
}

detail::State initialState() {
  detail::State state;
  state.q = Eigen::Quaterniond::Identity();
  state.p.setZero();
  state.v.setZero();
  state.ba.setZero();
  state.bw.setZero();
  state.g = Eigen::Vector3d(0.0, 0.0, -9.81);
  state.time = 0.0;
  state.P = Eigen::MatrixXd::Identity(detail::STATE_TOTAL, detail::STATE_TOTAL) * 0.001;
  return state;
}

std::unique_ptr<detail::Context> context(bool elevator_enabled = false) {
  lio::Config settings;
  settings.elevator_enabled = elevator_enabled;
  settings.acc_noise_variance = 0.01;
  settings.gyro_noise_variance = 0.001;
  return std::make_unique<detail::Context>(settings);
}

void sameMean(const detail::State& actual, const detail::State& expected,
              const std::string& label) {
  near(actual.q.angularDistance(expected.q), 1e-12, label + " rotation");
  near((actual.p - expected.p).norm(), 1e-12, label + " position");
  near((actual.v - expected.v).norm(), 1e-12, label + " velocity");
  near((actual.ba - expected.ba).norm(), 1e-12, label + " accelerometer bias");
  near((actual.bw - expected.bw).norm(), 1e-12, label + " gyro bias");
  near((actual.g - expected.g).norm(), 1e-12, label + " gravity");
  near(actual.z - expected.z, 1e-12, label + " transport height");
  near(actual.vz - expected.vz, 1e-12, label + " transport velocity");
  near(actual.az - expected.az, 1e-12, label + " transport acceleration");
  near(actual.time - expected.time, 1e-12, label + " timestamp");
  require(actual.in_elevator == expected.in_elevator, label + " transport mode");
}

void sameState(const detail::State& actual, const detail::State& expected,
               const std::string& label) {
  sameMean(actual, expected, label);
  near((actual.P - expected.P).norm(), 1e-12, label + " covariance");
}

detail::State partialIntegration(double query_time) {
  auto settings = context();
  detail::IMUProcess partial(*settings);
  partial.set_init_state(initialState(), 0.0);
  partial.integration(kLeftAcceleration, kLeftGyro, 0.0);
  const double ratio = query_time / kInterval;
  // The public integration path averages these endpoints using predictOnce.
  partial.integration((1.0 - ratio) * kLeftAcceleration + ratio * kRightAcceleration,
                      (1.0 - ratio) * kLeftGyro + ratio * kRightGyro, query_time);
  detail::State expected;
  partial.get_state_at_t(expected, query_time);
  return expected;
}

void sameBoundary(const detail::Pose& pose, const detail::State& state,
                  double offset, const std::string& label) {
  near(pose.q.angularDistance(state.q), 1e-12, label + " rotation");
  near((pose.p - state.p).norm(), 1e-12, label + " position");
  near((pose.v - state.v).norm(), 1e-12, label + " velocity");
  near(pose.offset_time - offset, 1e-12, label + " offset");
}

void checkOrdinaryQueriesAndDeskew() {
  auto settings = context();
  detail::IMUProcess imu(*settings);
  imu.set_init_state(initialState(), 0.0);
  imu.integration(kLeftAcceleration, kLeftGyro, 0.0);
  imu.integration(kRightAcceleration, kRightGyro, kInterval);

  const double query_times[] = {0.00499999, 0.00500001};
  detail::State states[2];
  for (int i = 0; i < 2; ++i) {
    imu.get_state_at_t(states[i], query_times[i]);
    sameState(states[i], partialIntegration(query_times[i]),
              "ordinary query " + std::to_string(i));
  }
  near((states[1].P - states[0].P).norm(), 1e-7,
       "covariance jumps at the IMU interval midpoint");

  std::vector<detail::Pose> poses;
  imu.get_imu_pose_from_t1_to_t2(poses, query_times[0], query_times[1]);
  require(poses.size() == 2, "interior deskew interval must have two boundaries");
  sameBoundary(poses.front(), states[0], 0.0, "deskew start");
  sameBoundary(poses.back(), states[1], query_times[1] - query_times[0], "deskew end");

  detail::State exact;
  imu.get_state_at_t(exact, 0.0);
  sameState(exact, imu.states_imu[0], "exact oldest sample");
  imu.get_state_at_t(exact, kInterval);
  sameState(exact, imu.states_imu[1], "exact newest sample");
  imu.get_imu_pose_from_t1_to_t2(poses, 0.0, kInterval);
  require(poses.size() == 2, "exact deskew interval must retain both samples");
  sameBoundary(poses.front(), imu.states_imu[0], 0.0, "exact deskew start");
  sameBoundary(poses.back(), imu.states_imu[1], kInterval, "exact deskew end");
}

void checkModeChangeOccursAtSample() {
  auto settings = context(true);
  detail::IMUProcess imu(*settings);
  imu.set_init_state(initialState(), 0.0);
  imu.integration(kLeftAcceleration, kLeftGyro, 0.0);
  settings->ELEVATOR_TRIGGER = true;
  imu.integration(kRightAcceleration, kRightGyro, kInterval);
  require(imu.states_imu[1].in_elevator, "fixture must enter at the right sample");

  for (const double query : {0.004, 0.006}) {
    detail::State state;
    imu.get_state_at_t(state, query);
    sameState(state, partialIntegration(query), "before transport entry sample");
  }
  detail::State at_entry;
  imu.get_state_at_t(at_entry, kInterval);
  sameState(at_entry, imu.states_imu[1], "exact transport entry sample");
  require(at_entry.in_elevator, "exact entry query must retain the mode reset");
}
}  // namespace

int main() {
  try {
    checkOrdinaryQueriesAndDeskew();
    checkModeChangeOccursAtSample();
    std::cout << "LTLIO same-time IMU mean/covariance and deskew boundaries passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
