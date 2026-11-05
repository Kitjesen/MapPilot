#include "ltlio/frontend.hpp"
#include "ltlio/detail/ElevatorProcess.h"
#include "ltlio/detail/ElevatorSelfExit.h"
#include "ltlio/detail/ESEKF.h"
#include "ltlio/detail/IMUProcess.h"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {
namespace lio = lingtu::ltlio;
namespace detail = lingtu::ltlio::detail;
constexpr int kSize = detail::STATE_TOTAL;
using Matrix = Eigen::Matrix<double, kSize, kSize>;
using Delta = Eigen::Matrix<double, kSize, 1>;
constexpr double kGravity = 9.80665;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const std::string& message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          message + ": actual=" + std::to_string(actual) +
              " expected=" + std::to_string(expected));
}

detail::State correlatedState() {
  detail::State state;
  state.q = Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitY());
  state.p = {1.0, -2.0, 0.4};
  state.v = {0.1, -0.1, 0.2};
  state.bw = {0.002, -0.001, 0.003};
  state.ba = {0.01, -0.02, 0.03};
  state.g = {0.02, -0.01, -kGravity};
  state.z = 3.2;
  state.vz = 0.35;
  state.az = -0.12;
  state.time = 12.0;
  state.in_elevator = true;
  Matrix factor = Matrix::Identity() * 0.15;
  factor(detail::Z, detail::Z) = 0.8;
  factor(detail::VZ, detail::VZ) = 0.4;
  factor(detail::AZ, detail::AZ) = 0.2;
  factor(detail::P + 2, detail::VZ) = 0.8;
  factor(detail::Z, detail::VZ) = 1.4;
  factor(detail::Z, detail::AZ) = 0.7;
  factor(detail::R, detail::VZ) = 0.1;
  factor(detail::R + 1, detail::AZ) = -0.08;
  factor(detail::V + 1, detail::VZ) = 0.12;
  factor(detail::BW + 2, detail::VZ) = 0.03;
  factor(detail::G, detail::AZ) = 0.05;
  state.P = factor * factor.transpose();
  require(state.P.llt().info() == Eigen::Success, "correlated covariance fixture is not SPD");
  return state;
}

double worldHeight(const detail::State& state) {
  return state.p.z() + (state.in_elevator ? state.z : 0.0);
}

Eigen::Matrix<double, 3, kSize> worldPositionJacobian(bool has_transport_offset) {
  Eigen::Matrix<double, 3, kSize> jacobian =
      Eigen::Matrix<double, 3, kSize>::Zero();
  jacobian.block<3, 3>(0, detail::P).setIdentity();
  if (has_transport_offset) jacobian(2, detail::Z) = 1.0;
  return jacobian;
}

double worldHeightVariance(const detail::State& state) {
  const Eigen::Matrix<double, 1, kSize> row = worldPositionJacobian(state.in_elevator).row(2);
  return (row * state.P * row.transpose())(0, 0);
}

struct ExpectedUpdate {
  Delta delta;
  Matrix covariance;
};

ExpectedUpdate referenceStopUpdate(const detail::State& prior) {
  // Independently condition the full joint Gaussian on zero transport speed
  // and acceleration. The subtractive covariance expression is equivalent to
  // Joseph's form, without using production's gain or injection helpers.
  const Matrix covariance = prior.P;
  Eigen::Matrix<double, kSize, 2> cross;
  cross.col(0) = covariance.col(detail::VZ);
  cross.col(1) = covariance.col(detail::AZ);
  const double a = covariance(detail::VZ, detail::VZ) + 1e-5;
  const double b = covariance(detail::VZ, detail::AZ);
  const double c = covariance(detail::AZ, detail::AZ) + 1e-4;
  Eigen::Matrix2d inverse;
  inverse << c, -b, -b, a;
  inverse /= a * c - b * b;
  ExpectedUpdate expected{cross * inverse * Eigen::Vector2d(-prior.vz, -prior.az),
                          covariance - cross * inverse * cross.transpose()};
  const auto theta = expected.delta.head<3>();
  Eigen::Matrix3d skew;
  skew << 0.0, -theta.z(), theta.y(), theta.z(), 0.0, -theta.x(),
      -theta.y(), theta.x(), 0.0;
  Matrix reset = Matrix::Identity();
  reset.block<3, 3>(detail::R, detail::R) -= 0.5 * skew;
  expected.covariance = reset * expected.covariance * reset.transpose();
  return expected;
}

void checkUpdate(const detail::State& before, const detail::State& after,
                 const ExpectedUpdate& expected) {
  const auto theta = expected.delta.head<3>();
  Eigen::Quaterniond expected_rotation = before.q;
  if (theta.norm() > 0.0)
    expected_rotation = before.q * Eigen::Quaterniond(Eigen::AngleAxisd(theta.norm(), theta.normalized()));
  near(after.q.angularDistance(expected_rotation), 0.0, 1e-11,
       "stop update lost its rotation cross-covariance correction");
  near((after.p - before.p - expected.delta.segment<3>(detail::P)).norm(), 0.0, 1e-11,
       "stop position mean differs from full conditional update");
  near((after.v - before.v - expected.delta.segment<3>(detail::V)).norm(), 0.0, 1e-11,
       "stop velocity mean differs from full conditional update");
  near((after.bw - before.bw - expected.delta.segment<3>(detail::BW)).norm(), 0.0, 1e-11,
       "stop gyro bias lost its cross-covariance correction");
  near((after.ba - before.ba - expected.delta.segment<3>(detail::BA)).norm(), 0.0, 1e-11,
       "stop accelerometer bias differs from independent update");
  near((after.g - before.g - expected.delta.segment<3>(detail::G)).norm(), 0.0, 1e-11,
       "stop gravity lost its cross-covariance correction");
  near(after.z, before.z + expected.delta(detail::Z), 1e-11, "stop transport-height injection differs");
  near(after.vz, before.vz + expected.delta(detail::VZ), 1e-11, "stop transport-speed injection differs");
  near(after.az, before.az + expected.delta(detail::AZ), 1e-11, "stop acceleration injection differs");
  near((after.P - expected.covariance).norm(), 0.0, 1e-10,
       "stop covariance differs from conditional-Gaussian/SO3-reset oracle");
  near((after.P - after.P.transpose()).norm(), 0.0, 1e-12, "stop covariance is not symmetric");
  require(after.P.llt().info() == Eigen::Success, "stop covariance lost positive definiteness");
}

Eigen::Isometry3d worldPose(const detail::State& state) {
  auto pose = Eigen::Isometry3d::Identity();
  pose.linear() = state.q.toRotationMatrix();
  pose.translation() = state.p;
  if (state.in_elevator) pose.translation().z() += state.z;
  return pose;
}

Eigen::Isometry3d initialCorrection() {
  auto correction = Eigen::Isometry3d::Identity();
  correction.linear() = (Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitY())).toRotationMatrix();
  correction.translation() = Eigen::Vector3d(2.0, -1.0, 0.7);
  return correction;
}

void checkStopPosteriorAndContinuousOdometry() {
  auto context = std::make_unique<detail::Context>(lio::Config{});
  context->world_from_odom = initialCorrection();
  detail::ElevatorProcess process(*context);
  const auto before = correlatedState();
  const auto expected = referenceStopUpdate(before);
  require(expected.delta.head<3>().norm() > 1e-3,
          "stop fixture did not exercise orientation cross-covariance/reset");
  const Eigen::Isometry3d initial_odom = context->world_from_odom.inverse() * worldPose(before);
  auto after = before;
  for (int i = 0; i < 4; ++i) {
    const auto prior = after;
    const auto correction_before = context->world_from_odom;
    process.applyZeroVelocityUpdateZ(after);
    checkUpdate(prior, after, referenceStopUpdate(prior));
    const Eigen::Isometry3d correction_expected = worldPose(after) *
        worldPose(prior).inverse() * correction_before;
    near((context->world_from_odom.matrix() - correction_expected.matrix()).norm(),
         0.0, 1e-11, "repeated stop correction composed in the wrong frame/order");
    const Eigen::Isometry3d odom_after = context->world_from_odom.inverse() * worldPose(after);
    near((odom_after.matrix() - initial_odom.matrix()).norm(), 0.0, 1e-11,
         "full posterior stop correction leaked into continuous odometry pose");
  }
  require(std::abs(worldHeight(after) - worldHeight(before)) > 0.1,
          "continuity handling suppressed the posterior height correction");
  require(worldHeightVariance(after) < 0.5 * worldHeightVariance(before),
          "continuity handling discarded posterior height information");
  require(std::abs(after.vz) < std::abs(before.vz) && std::abs(after.az) < std::abs(before.az),
          "full stop update did not settle observed speed/acceleration");

  auto zero = before;
  zero.vz = zero.az = 0.0;
  const auto zero_before = zero;
  const auto correction_before = context->world_from_odom;
  process.applyZeroVelocityUpdateZ(zero);
  checkUpdate(zero_before, zero, referenceStopUpdate(zero_before));
  near((zero.p - zero_before.p).norm() + std::abs(zero.z - zero_before.z), 0.0, 1e-12,
       "zero-innovation stop observation changed its mean");
  near((context->world_from_odom.matrix() - correction_before.matrix()).norm(),
       0.0, 1e-11, "zero-innovation update moved the map/odom correction");
}

void checkOrdinaryEkfIsUnchanged() {
  const auto before = correlatedState();
  auto after = before;
  Eigen::Matrix<double, 2, kSize> observation = Eigen::Matrix<double, 2, kSize>::Zero();
  observation(0, detail::VZ) = observation(1, detail::AZ) = 1.0;
  Eigen::Matrix2d noise = Eigen::Matrix2d::Zero();
  noise(0, 0) = 1e-5;
  noise(1, 1) = 1e-4;
  detail::ekfUpdate<2>(after, observation, Eigen::Vector2d(-before.vz, -before.az), noise);
  checkUpdate(before, after, referenceStopUpdate(before));
  require(std::abs(worldHeight(after) - worldHeight(before)) > 0.1,
          "generic EKF was accidentally changed to freeze correlated world height");
}

void checkExitUpstreamCovariancePolicy() {
  auto context = std::make_unique<detail::Context>(lio::Config{});
  context->world_from_odom = initialCorrection();
  detail::ElevatorProcess process(*context);
  const auto before = correlatedState();
  // Upstream exit anchors transport displacement into INS position while
  // retaining the INS position covariance, rather than propagating Z through
  // a full coordinate-change Jacobian. This is an explicit approximation;
  // world position variance is not expected to be continuous across exit.
  // LingTu also folds actual transport velocity to preserve manual moving exit.
  Matrix fold = Matrix::Identity();
  fold(detail::V + 2, detail::VZ) = 1.0;
  fold.bottomRows<3>().setZero();
  Matrix expected = fold * before.P * fold.transpose();
  expected(detail::Z, detail::Z) = 1e-4;
  expected(detail::VZ, detail::VZ) = 1e-3;
  expected(detail::AZ, detail::AZ) = 1e-2;
  auto after = before;
  process.ElevatorModeExit(after);
  require(!after.in_elevator && after.z == 0.0 && after.vz == 0.0 && after.az == 0.0,
          "exit did not retire transport coordinates after folding the full posterior");
  near(worldHeight(after), worldHeight(before), 1e-12, "exit changed world-height mean");
  near(after.v.z(), before.v.z() + before.vz, 1e-12, "exit discarded world vertical velocity");
  near(after.p.x(), before.p.x(), 1e-12, "exit changed horizontal position");
  near(after.p.y(), before.p.y(), 1e-12, "exit changed horizontal position");
  near(after.q.angularDistance(before.q), 0.0, 1e-12, "coordinate fold changed orientation");
  near((after.P - expected).norm(), 0.0, 1e-10,
       "exit differs from upstream INS covariance anchoring plus velocity fold");
  near((after.P.block<3, 3>(detail::P, detail::P) -
        before.P.block<3, 3>(detail::P, detail::P)).norm(),
       0.0, 1e-11, "exit did not retain the upstream INS position covariance");
  require(after.P.llt().info() == Eigen::Success, "exit covariance is not positive definite");
  near((context->world_from_odom.matrix() - initialCorrection().matrix()).norm(),
       0.0, 1e-12, "coordinate-only exit added an unobserved stop correction");

  const auto stopped = after;
  process.ElevatorModeEnter(after, Eigen::Vector3d(0.0, 0.0, kGravity));
  require(after.in_elevator && after.vz == 0.0 && after.az == 0.0,
          "next entry did not start a fresh dynamic transport episode");
  near(worldHeight(after), worldHeight(stopped), 1e-12, "next entry erased the previous floor elevation");
  near((after.P.topLeftCorner<18, 18>() - stopped.P.topLeftCorner<18, 18>()).norm(),
       0.0, 1e-11, "next entry discarded previously folded pose/velocity covariance");
}

lio::Config frontendConfig() {
  lio::Config settings;
  settings.imu_init_samples = 20;
  settings.point_filter_num = 1;
  settings.blind_m = 0.1;
  settings.scan_resolution = 0.1;
  settings.elevator_enabled = true;
  settings.automatic_door = true;
  return settings;
}

lio::TimedCloud cabin(double stamp, bool open) {
  lio::TimedCloud cloud;
  cloud.stamp_s = stamp;
  cloud.end_stamp_s = stamp + 0.02;
  for (int plane = 0; plane < 3; ++plane)
    for (int a = -6; a <= 6; ++a)
      for (int b = -6; b <= 6; ++b) {
        lio::TimedPoint point;
        if (plane == 0) { point.x = 1.1F; point.y = a * 0.1F; point.z = b * 0.1F; }
        else if (plane == 1) { point.x = a * 0.1F; point.y = 1.4F; point.z = b * 0.1F; }
        else { point.x = a * 0.1F; point.y = b * 0.1F; point.z = -0.8F; }
        cloud.points.push_back(point);
      }
  if (open) {
    // More than 6% far returns change the actual 94th percentile while the
    // original near surfaces still support real matching across the door view.
    const auto far_count = cloud.points.size() / 5;
    for (std::size_t i = 0; i < far_count; ++i) {
      lio::TimedPoint point;
      point.x = 6.0F;
      point.y = static_cast<float>(i % 10) * 0.05F;
      point.z = static_cast<float>(i / 10) * 0.05F;
      cloud.points.push_back(point);
    }
  }
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
    cloud.points[i].line = static_cast<std::uint8_t>(i % 4);
  }
  return cloud;
}

detail::State localObservation(double height_variance, double height_position_cross) {
  auto context = std::make_unique<detail::Context>(frontendConfig());
  detail::EskfEstimator estimator(*context);
  detail::PointCloudXYZI::Ptr map(new detail::PointCloudXYZI);
  for (const auto& input : cabin(0.0, false).points) {
    detail::PointType point;
    point.x = input.x; point.y = input.y; point.z = input.z;
    map->push_back(point);
  }
  context->ikd_map->add(map);
  auto state = correlatedState();
  state.q = Eigen::Quaterniond::Identity();
  state.p = Eigen::Vector3d(0.0, 0.0, 0.02);
  state.v.setZero(); state.bw.setZero(); state.ba.setZero();
  state.g = Eigen::Vector3d(0.0, 0.0, -kGravity);
  state.vz = state.az = 0.0;
  state.P = Matrix::Identity() * 0.01;
  state.P(detail::Z, detail::Z) = height_variance;
  state.P(detail::Z, detail::P + 2) = state.P(detail::P + 2, detail::Z) = height_position_cross;
  estimator.set_cur_state(state);
  auto observation = *map;
  estimator.UpdateByLidarIESKF(observation);
  require(estimator.lastUpdateAccepted() && estimator.validPointCount() >= 100,
          "active-cabin observability fixture did not perform real point-to-plane matching");
  const auto result = estimator.get_cur_state();
  require(std::abs(result.p.z()) < 0.01,
          "local point-to-plane observation did not correct local position");
  return result;
}

void checkLocalHeightObservability() {
  const auto narrow = localObservation(0.02, 0.0);
  const auto broad = localObservation(64.0, 0.0);
  near((narrow.p - broad.p).norm(), 0.0, 1e-10,
       "pure global-height uncertainty changed the local geometric correction");
  near((narrow.P.topLeftCorner<18, 18>() - broad.P.topLeftCorner<18, 18>()).norm(),
       0.0, 1e-10, "local matching consumed independent global-height variance");
  near(broad.z, 3.2, 1e-11, "local geometry directly observed independent cumulative height");
  near(broad.P(detail::Z, detail::Z), 64.0, 1e-9,
       "local matching collapsed the independent shared height uncertainty");
  const auto correlated = localObservation(1.0, 0.05);
  require(std::abs(correlated.z - 3.2) > 1e-3 && correlated.P(detail::Z, detail::Z) < 1.0,
          "full LiDAR posterior incorrectly froze legitimate Z/position cross-updates");
}

struct Sequence {
  int imu = 0;
  int frame = 0;
};

lio::Result step(lio::Frontend& frontend, Sequence& sequence, bool open = false) {
  const auto cloud = cabin(0.20 + sequence.frame++ * 0.05, open);
  while (sequence.imu * 0.005 <= cloud.end_stamp_s + 0.006) {
    lio::ImuSample imu;
    imu.stamp_s = sequence.imu++ * 0.005;
    imu.acceleration = {0.0, 0.0, kGravity};
    frontend.feedImu(imu);
  }
  frontend.feedCloud(cloud);
  const auto result = frontend.processNext();
  require(result.has_value(), "covered cabin scan did not produce a result");
  if (sequence.frame > 5)
    require(result->accepted, "cabin/door fixture failed real point-to-plane matching");
  const auto jacobian = worldPositionJacobian(result->state.in_elevator);
  near((result->world_position_covariance -
        jacobian * result->state.covariance * jacobian.transpose()).norm(),
       0.0, 1e-10, "public world covariance double-counted dormant transport variance");
  return *result;
}

enum class Observation { Room, Rejected, Empty };

lio::Result timedStep(lio::Frontend& frontend, Sequence& sequence, double end,
                     Observation kind = Observation::Room, double sensor_height = 0.0) {
  auto cloud = cabin(end - 0.02, false);
  cloud.end_stamp_s = end;
  if (kind == Observation::Rejected) cloud.points.resize(2);
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    auto& point = cloud.points[i];
    point.z -= static_cast<float>(sensor_height);
    if (kind == Observation::Empty) point.x = point.y = point.z = 0.0F;
    point.offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
  }
  // Queue the scan before its covering IMU. Include that right-hand sample
  // explicitly rather than relying on a floating-point half-open end bound.
  frontend.feedCloud(cloud);
  // Division can round an end just above an IMU tick back to an integer.
  // Queue the following real sample too; the frontend stops consumption at
  // the first sample that actually covers the exact scan endpoint.
  const int covering_imu = static_cast<int>(std::ceil(end / 0.005)) + 1;
  while (sequence.imu <= covering_imu) {
    lio::ImuSample sample;
    sample.stamp_s = sequence.imu++ * 0.005;
    // A small sensor bias creates a nonzero waiting innovation without
    // reaching the transport acceleration/speed transition thresholds.
    sample.acceleration = Eigen::Vector3d(0.0, 0.0, kGravity + 0.02);
    frontend.feedImu(sample);
  }
  std::optional<lio::Result> callback;
  const auto result = frontend.processNext([&](const lio::Result& candidate) -> std::optional<lio::PosePrior> {
    callback = candidate;
    return std::nullopt;
  });
  require(result.has_value(), "covered causal fixture did not return its observation: end=" +
      std::to_string(end) + " last_imu=" + std::to_string((sequence.imu - 1) * 0.005) +
      " queued_clouds=" + std::to_string(frontend.bufferedClouds()) +
      " queued_imus=" + std::to_string(frontend.bufferedImus()));
  if (kind == Observation::Room) {
    require(result->accepted && callback.has_value(), "causal room fixture failed actual matching");
    near((callback->world_from_odom.matrix() - result->world_from_odom.matrix()).norm(),
         0.0, 1e-12, "same-scan pose-prior callback saw an obsolete correction frame");
  } else {
    require(!result->accepted && !callback, "rejected/empty scan consumed an accepted-scan callback");
  }
  return *result;
}

void checkWaitingCausality(bool reject_before_commit, bool manual_exit) {
  auto settings = frontendConfig();
  settings.automatic_door = false;
  lio::Frontend frontend(settings);
  Sequence sequence;
  for (int i = 0; i < 8; ++i) step(frontend, sequence);
  // The earlier static helper already consumed its right-hand covering IMU;
  // the next sample is the first that can enter IDLE transport.
  const double entered_at = sequence.imu * 0.005;
  frontend.requestElevator(true);
  int request_index = sequence.imu;
  while (request_index * 0.005 - entered_at < 3.0) ++request_index;
  const double requested_at = request_index * 0.005;
  lio::Result previous;
  for (int index = 0; 0.62 + index * 0.05 < requested_at - 0.03; ++index)
    previous = timedStep(frontend, sequence, 0.62 + index * 0.05);
  require(previous.state.in_elevator && previous.elevator.phase == 0,
          "waiting fixture left the idle cabin before the causal boundary");
  const auto correction = previous.world_from_odom;
  const auto early = timedStep(frontend, sequence, requested_at - 0.002);
  near((early.world_from_odom.matrix() - correction.matrix()).norm(), 0.0, 1e-12,
       "future covering IMU applied a waiting correction before scan end");
  if (manual_exit) {
    frontend.requestElevator(false);
    const auto exited = timedStep(frontend, sequence, requested_at + 0.048);
    require(!exited.state.in_elevator && exited.elevator.exits == 1,
            "manual exit did not commit after the pending waiting event");
    near((exited.world_from_odom.matrix() - correction.matrix()).norm(), 0.0, 1e-11,
         "manual exit consumed a pending zero-speed update without waiting evidence");
    return;
  }
  if (reject_before_commit) {
    const auto rejected = timedStep(frontend, sequence, requested_at + 0.018, Observation::Rejected);
    near((rejected.world_from_odom.matrix() - correction.matrix()).norm(), 0.0, 1e-12,
         "rejected scan applied a pending waiting update");
    timedStep(frontend, sequence, requested_at + 0.038, Observation::Empty);
  }
  const auto committed = timedStep(frontend, sequence, requested_at + 0.068);
  require(committed.state.in_elevator && committed.elevator.phase == 0 &&
              (committed.world_from_odom.matrix() - correction.matrix()).norm() > 1e-7,
          "accepted scan failed to commit the pending nonzero waiting posterior");

}

void checkClosedEpisodeRearmAndManualOverride() {
  lio::Frontend frontend(frontendConfig());
  Sequence sequence;
  lio::Result result;
  for (int i = 0; i < 12; ++i) result = step(frontend, sequence);
  require(result.elevator.entries == 0 && !result.state.in_elevator,
          "short initial closed view unexpectedly entered transport");
  frontend.requestElevator(true);
  for (int i = 0; i < 8; ++i) result = step(frontend, sequence);
  require(result.state.in_elevator && result.elevator.entries == 1,
          "manual entry did not reach the real scan state");
  frontend.requestElevator(false);
  result = step(frontend, sequence);
  require(!result.state.in_elevator && result.elevator.exits == 1,
          "manual exit was not committed by the accepted scan");
  for (int i = 0; i < 130; ++i) {
    result = step(frontend, sequence);
    require(!result.state.in_elevator && result.elevator.entries == 1,
            "the same closed cabin episode caused automatic re-entry");
  }
  for (int i = 0; i < 3; ++i) result = step(frontend, sequence, true);
  require(result.elevator.range_quantile_m > 3.0,
          "open-door fixture never supplied independent rearming evidence");
  for (int i = 0; i < 30; ++i) {
    result = step(frontend, sequence);
    require(!result.state.in_elevator, "fresh closed episode bypassed its hold time");
  }
  for (int i = 0; i < 20; ++i) result = step(frontend, sequence);
  require(result.state.in_elevator && result.elevator.entries == 2,
          "a new open-to-closed episode could not begin the next trip");
  frontend.requestElevator(false);
  result = step(frontend, sequence);
  require(!result.state.in_elevator && result.elevator.exits == 2,
          "second trip exit was not committed");
  frontend.requestElevator(true);
  for (int i = 0; i < 3; ++i) result = step(frontend, sequence);
  require(result.state.in_elevator && result.elevator.entries == 3,
          "door rearm latch prevented an explicit manual next trip");
}

void checkOrdinaryFrontendUnaffected() {
  auto settings = frontendConfig();
  settings.automatic_door = false;
  lio::Frontend dormant(settings);
  settings.elevator_enabled = false;
  lio::Frontend disabled(settings);
  disabled.requestElevator(true);
  Sequence a, b;
  for (int i = 0; i < 50; ++i) {
    const auto reference = step(dormant, a, i >= 25);
    const auto actual = step(disabled, b, i >= 25);
    require(reference.accepted == actual.accepted &&
                reference.scan_end_s == actual.scan_end_s &&
                !actual.state.in_elevator && actual.elevator.entries == 0,
            "disabled elevator changed ordinary scan state or entered a trip");
    near((actual.world_position - reference.world_position).norm(), 0.0, 1e-11,
         "ordinary position changed with no transport event");
    near(actual.state.orientation.angularDistance(reference.state.orientation), 0.0, 1e-11,
         "ordinary orientation changed with no transport event");
    near((actual.state.covariance - reference.state.covariance).norm(), 0.0, 1e-10,
         "ordinary covariance changed with no transport event");
    near((actual.world_from_odom.matrix() - Eigen::Matrix4d::Identity()).norm(),
         0.0, 1e-12, "ordinary processing invented a transport map/odom correction");
  }
}

void checkDetectorRejectsDuplicateTime() {
  auto context = std::make_unique<detail::Context>(frontendConfig());
  detail::ElevatorSelfExit detector(*context);
  detail::State state;
  state.in_elevator = true;

  detail::ElevatorSelfExit::ImuFrame first;
  first.timestamp = 1.0;
  first.acc_z = 0.25;
  first.vel_z = 0.35;
  first.in_elevator = true;
  detector.process(first, state);
  const auto first_debug = detector.debugInfo();

  for (int i = 0; i < 400; ++i) {
    auto duplicate = first;
    duplicate.acc_z = -8.0 - i;
    duplicate.vel_z = (i % 2 == 0) ? -2.0 : 2.0;
    detector.process(duplicate, state);
  }
  require(detector.eleState() == 0 &&
              detector.debugInfo().acc_z == first_debug.acc_z &&
              detector.debugInfo().vel_z == first_debug.vel_z &&
              detector.debugInfo().std_dev == first_debug.std_dev,
          "duplicate IMU timestamps changed the exit detector");

  // Together with the first accepted sample, 299 strictly newer samples fill
  // the real 300-sample window and must retain the historical phase behavior.
  for (int i = 1; i <= 299; ++i) {
    auto sample = first;
    sample.timestamp = 1.0 + i * 0.005;
    sample.vel_z = (i % 2 == 0) ? 0.0 : 0.35;
    detector.process(sample, state);
  }
  require(detector.eleState() == 1,
          "strictly increasing IMU samples no longer drive the detector phase");

  detector.reset();
  require(detector.eleState() == 0 && detector.debugInfo().state == 0 &&
              detector.debugInfo().acc_z == 0.0 &&
              detector.debugInfo().vel_z == 0.0 &&
              detector.debugInfo().std_dev == 0.0,
          "reset retained the previous transport diagnostics");
  auto restarted = first;
  restarted.timestamp = 0.5;
  restarted.acc_z = 0.1;
  restarted.vel_z = 0.15;
  detector.process(restarted, state);
  require(detector.debugInfo().acc_z == restarted.acc_z &&
              detector.debugInfo().vel_z == restarted.vel_z,
          "reset did not start a fresh detector time base");
}

void checkDuplicateOpenCloudDoesNotRearmDoor() {
  auto context = std::make_unique<detail::Context>(frontendConfig());
  context->elevator_door_cooldown_time = 0.0;
  context->elevator_door_time_threshold = 2.0;
  detail::ElevatorProcess door(*context);
  auto state = correlatedState();
  state.in_elevator = false;

  const auto closed_input = cabin(0.0, false);
  const auto open_input = cabin(0.0, true);
  const auto closed = selectPoints(closed_input, frontendConfig());
  const auto open = selectPoints(open_input, frontendConfig());

  door.processDoorDetector(*closed, 10.0, state, true);
  door.ElevatorModeEnter(state, Eigen::Vector3d::Zero());
  door.ElevatorModeExit(state);
  context->ELEVATOR_TRIGGER = false;

  // The open observation repeats the last accepted timestamp.  It cannot be
  // used as fresh evidence to re-arm the just-consumed closed-door episode.
  door.processDoorDetector(*open, 10.0, state, true);
  door.processDoorDetector(*closed, 12.0, state, true);
  door.processDoorDetector(*closed, 14.1, state, true);
  require(!context->ELEVATOR_TRIGGER,
          "duplicate open cloud re-armed automatic elevator entry");
}

void checkRotationErrorLogarithm() {
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 3.0).normalized();
  const auto reference = correlatedState();
  // A known relative axis/angle is an independent oracle for the rotation
  // error used by both IESKF update branches, including a nonidentity base.
  for (const double angle : {0.0, 1e-9, -1e-9, 0.0005, 0.3, -0.3, 1.0, -1.0,
                             3.14159265358979323846 - 1e-7}) {
    auto current = reference;
    current.q = reference.q * Eigen::Quaterniond(Eigen::AngleAxisd(angle, axis));
    const Delta error = detail::EskfEstimator::getErrorState(current, reference);
    near((error.head<3>() - angle * axis).norm(), 0.0, 1e-10,
         "IESKF rotation error is not the relative SO(3) logarithm at angle " +
             std::to_string(angle));
    near(error.tail<kSize - 3>().norm(), 0.0, 1e-12,
         "a pure rotation difference changed another error-state component");
    const Delta reverse = detail::EskfEstimator::getErrorState(reference, current);
    near((reverse.head<3>() + angle * axis).norm(), 0.0, 1e-10,
         "inverse rotation error has the wrong sign or magnitude");
  }
}

void checkImuPoseRangeTiming(bool oldest_only = false) {
  auto settings = frontendConfig();
  settings.elevator_enabled = false;
  auto context = std::make_unique<detail::Context>(settings);
  detail::IMUProcess imu(*context);
  auto initial = correlatedState();
  initial.q = Eigen::Quaterniond::Identity();
  initial.p.setZero();
  initial.v = Eigen::Vector3d(0.6, -0.2, 0.1);
  initial.bw.setZero();
  initial.ba.setZero();
  initial.g = Eigen::Vector3d(0.0, 0.0, -kGravity);
  initial.z = initial.vz = initial.az = initial.time = 0.0;
  initial.in_elevator = false;
  imu.set_init_state(initial, 0.0);

  const auto check_range = [&](double start, double end) {
    std::vector<detail::Pose> poses;
    imu.get_imu_pose_from_t1_to_t2(poses, start, end);
    require(poses.size() >= 2, "covered IMU range did not contain both endpoints");
    near(poses.front().offset_time, 0.0, 1e-12,
         "IMU range begins at a different time origin");
    near(poses.back().offset_time, end - start, 1e-12,
         "IMU range endpoint lost the scan-start time origin");
    for (std::size_t i = 0; i < poses.size(); ++i) {
      if (i > 0)
        require(poses[i].offset_time > poses[i - 1].offset_time,
                "IMU deskew intervals are not strictly chronological");
      near((poses[i].p - initial.v * (start + poses[i].offset_time)).norm(),
           0.0, 1e-10, "IMU pose timestamp disagrees with constant-speed truth");
    }
    // This return lies in the final IMU interval, which deskew would skip if
    // the tail were accidentally timestamped relative to the scan end.
    const double point_time = end - start - 0.0005;
    int covering_intervals = 0;
    for (std::size_t i = 1; i < poses.size(); ++i) {
      const auto& head = poses[i - 1];
      const auto& tail = poses[i];
      if (point_time <= head.offset_time || point_time > tail.offset_time) continue;
      ++covering_intervals;
      const double dt = point_time - head.offset_time;
      const Eigen::Vector3d landmark(3.0, -1.0, 0.4);
      const Eigen::Vector3d raw_return = landmark - initial.v * (start + point_time);
      const Eigen::Vector3d world_point =
          raw_return + head.p + head.v * dt + 0.5 * tail.acc * dt * dt;
      near((world_point - landmark).norm(), 0.0, 1e-10,
           "last-interval LiDAR return does not recover its fixed world landmark");
      const Eigen::Vector3d deskewed = world_point - poses.back().p;
      near((deskewed - (landmark - initial.v * end)).norm(), 0.0, 1e-10,
           "last-interval deskew does not express the return at scan end");
    }
    require(covering_intervals == 1, "last-interval LiDAR return is not covered exactly once");
  };

  for (int i = 0; i <= detail::IMUProcess::MAX_LEN + 3; ++i) {
    imu.integration(Eigen::Vector3d(0.0, 0.0, kGravity), Eigen::Vector3d::Zero(), i * 0.005);
    if (i == 5) {
      if (!oldest_only) {
        check_range(0.005, 0.020);
        check_range(0.0075, 0.0225);
        check_range(0.0075, 0.020);
        check_range(0.005, 0.0225);
      }
      check_range(0.0, 0.0225);
    }
  }
  // The interval spans the physical array wrap, not just an unwrapped prefix.
  const double wrap = detail::IMUProcess::MAX_LEN * 0.005;
  if (!oldest_only) {
    check_range(wrap - 0.010, wrap + 0.010);
    check_range(wrap - 0.0075, wrap + 0.0125);
  }
  check_range(0.020, 0.0375);
}

void checkPosteriorRetimeInterpolatesImu() {
  for (const bool wrapped : {false, true}) {
    auto settings = frontendConfig();
    settings.elevator_enabled = false;
    auto context = std::make_unique<detail::Context>(settings);
    detail::IMUProcess imu(*context);
    auto initial = correlatedState();
    initial.q = Eigen::Quaterniond::Identity();
    initial.p.setZero();
    initial.v.setZero();
    initial.bw.setZero();
    initial.ba.setZero();
    initial.g = Eigen::Vector3d(0.0, 0.0, -kGravity);
    initial.z = initial.vz = initial.az = initial.time = 0.0;
    initial.in_elevator = false;
    imu.set_init_state(initial, 0.0);
    const int left_sample = wrapped ? detail::IMUProcess::MAX_LEN - 1 : 0;
    for (int i = 0; i <= left_sample; ++i)
      imu.integration(Eigen::Vector3d(0.0, 0.0, kGravity),
                      Eigen::Vector3d::Zero(), i * 0.005);
    const double start = left_sample * 0.005;
    const auto integrate = [&](double elapsed) {
      // Vertical acceleration and yaw rate vary linearly, so their integrals
      // are exact trapezoids and yaw does not rotate the acceleration axis.
      imu.integration(Eigen::Vector3d(0.0, 0.0, kGravity + 200.0 * elapsed),
                      Eigen::Vector3d(0.0, 0.0, 20.0 * elapsed), start + elapsed);
    };
    const auto truth = [&](double elapsed) {
      auto result = initial;
      result.time = start + elapsed;
      result.q = Eigen::AngleAxisd(10.0 * elapsed * elapsed, Eigen::Vector3d::UnitZ());
      result.v.z() = 100.0 * elapsed * elapsed;
      result.p.z() = (200.0 / 6.0) * elapsed * elapsed * elapsed;
      return result;
    };
    const auto check_end = [&](double elapsed, const std::string& phase) {
      detail::State actual;
      imu.get_state_at_t(actual, start + elapsed);
      const auto expected = truth(elapsed);
      near(actual.v.z(), expected.v.z(), 1e-10,
           phase + " reintegration uses acceleration from the wrong timestamp");
      near(actual.q.angularDistance(expected.q), 0.0, 1e-10,
           phase + " reintegration uses angular velocity from the wrong timestamp");
    };

    integrate(0.010);
    auto middle_posterior = truth(0.005);
    imu.set_state_at_t(middle_posterior, middle_posterior.time);
    check_end(0.010, wrapped ? "ring-crossing interpolated" : "interpolated");

    // Committing a posterior at an actual IMU sample must keep that sample's
    // acceleration and angular rate, not replace them with an interpolant.
    auto exact_posterior = truth(0.010);
    imu.set_state_at_t(exact_posterior, exact_posterior.time);
    integrate(0.020);
    check_end(0.020, wrapped ? "ring-crossing exact" : "exact");
  }
}

void checkTransportCovarianceNullDirection() {
  // In the cabin frame, equal and opposite vertical accelerometer-bias and
  // transport-acceleration errors cancel in both position and velocity.
  Delta direction = Delta::Zero();
  direction(detail::BA + 2) = 1.0;
  direction(detail::AZ) = -1.0;
  for (const bool active : {false, true}) {
    auto settings = frontendConfig();
    settings.elevator_enabled = active;
    auto base_context = std::make_unique<detail::Context>(settings);
    auto uncertain_context = std::make_unique<detail::Context>(settings);
    auto shifted_context = std::make_unique<detail::Context>(settings);
    for (auto* context : {base_context.get(), uncertain_context.get(), shifted_context.get()}) {
      context->ELEVATOR_TRIGGER = active;
      context->elevator_strong_prior_enable = false;
    }
    detail::IMUProcess base(*base_context);
    detail::IMUProcess uncertain(*uncertain_context);
    detail::IMUProcess shifted(*shifted_context);
    auto initial = correlatedState();
    initial.q = Eigen::Quaterniond::Identity();
    initial.p.setZero();
    initial.v.setZero();
    initial.bw.setZero();
    initial.ba.setZero();
    initial.g = Eigen::Vector3d(0.0, 0.0, -kGravity);
    initial.z = initial.vz = initial.az = 0.0;
    initial.time = 1.0;
    initial.in_elevator = active;
    initial.P = Matrix::Identity() * 0.01;
    auto extra_uncertainty = initial;
    extra_uncertainty.P += direction * direction.transpose();
    auto shifted_mean = initial;
    shifted_mean.ba.z() += 1.0;
    shifted_mean.az -= 1.0;
    base.set_init_state(initial, initial.time);
    uncertain.set_init_state(extra_uncertainty, initial.time);
    shifted.set_init_state(shifted_mean, initial.time);
    double time = initial.time;
    const auto integrate = [&] {
      for (auto* imu : {&base, &uncertain, &shifted})
        imu->integration(Eigen::Vector3d(0.0, 0.0, kGravity),
                         Eigen::Vector3d::Zero(), time);
    };
    integrate();
    double off_position_sensitivity = 0.0;
    double off_velocity_sensitivity = 0.0;
    for (const double dt : {0.005, 0.006, 0.004, 0.005, 0.006, 0.004}) {
      time += dt;
      integrate();
      detail::State base_state, uncertain_state, shifted_state;
      base.get_state_at_t(base_state, time);
      uncertain.get_state_at_t(uncertain_state, time);
      shifted.get_state_at_t(shifted_state, time);
      require(base_state.in_elevator == active && uncertain_state.in_elevator == active,
              "covariance fixture unexpectedly changed transport mode");
      const Eigen::Matrix<double, 6, 6> additional_observation_covariance =
          (uncertain_state.P - base_state.P).block<6, 6>(detail::P, detail::P);
      if (active) {
        near((shifted_state.p - base_state.p).norm(), 0.0, 1e-12,
             "transport null direction changed nominal cabin position");
        near((shifted_state.v - base_state.v).norm(), 0.0, 1e-12,
             "transport null direction changed nominal cabin velocity");
        if (additional_observation_covariance.norm() > 1e-12)
          std::cerr << "spurious cabin covariance norm=" << std::scientific
                    << additional_observation_covariance.norm() << '\n';
        near(additional_observation_covariance.norm(), 0.0, 1e-12,
             "transport propagation made an invisible BA/AZ direction observable in cabin P/V");
      } else {
        // Preserve the existing ordinary-mode first-order covariance policy:
        // AZ is inactive, while BA remains observable through acceleration.
        off_position_sensitivity += off_velocity_sensitivity * dt;
        off_velocity_sensitivity -= dt;
        Eigen::Matrix<double, 6, 1> observed = Eigen::Matrix<double, 6, 1>::Zero();
        observed(2) = off_position_sensitivity;
        observed(5) = off_velocity_sensitivity;
        near((additional_observation_covariance - observed * observed.transpose()).norm(),
             0.0, 1e-12, "transport covariance fix changed ordinary-mode propagation");
      }
    }
  }
}

} // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--rotation-log") {
      checkRotationErrorLogarithm();
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--imu-pose-range") {
      checkImuPoseRangeTiming();
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--imu-oldest-range") {
      checkImuPoseRangeTiming(true);
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--imu-retime") {
      checkPosteriorRetimeInterpolatesImu();
      return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--transport-covariance") {
      checkTransportCovarianceNullDirection();
      return 0;
    }
    checkRotationErrorLogarithm();
    checkImuPoseRangeTiming();
    checkPosteriorRetimeInterpolatesImu();
    checkTransportCovarianceNullDirection();
    checkStopPosteriorAndContinuousOdometry();
    checkOrdinaryEkfIsUnchanged();
    checkExitUpstreamCovariancePolicy();
    checkLocalHeightObservability();
    checkWaitingCausality(false, false);
    checkWaitingCausality(true, false);
    checkWaitingCausality(false, true);
    checkClosedEpisodeRearmAndManualOverride();
    checkOrdinaryFrontendUnaffected();
    checkDetectorRejectsDuplicateTime();
    checkDuplicateOpenCloudDoesNotRearmDoor();
    std::cout << "LTLIO full stop posterior, continuous odometry, exit covariance, trip rearm and ordinary-mode contracts passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
