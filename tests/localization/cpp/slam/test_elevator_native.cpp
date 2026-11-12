#include "ltlio/frontend.hpp"
#include "slam.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
namespace native = lingtu::ltlio;
namespace slam = lingtu::slam;
constexpr double kGravity = 9.80665;
constexpr double kImuDt = 0.005;
constexpr double kScanDuration = 0.020;
constexpr int kFrames = 12;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double actual, double expected, double tolerance, const char* message) {
  if (std::abs(actual - expected) > tolerance || !std::isfinite(actual)) {
    std::ostringstream detail;
    detail.precision(17);
    detail << message << ": actual=" << actual << " expected=" << expected;
    throw std::runtime_error(detail.str());
  }
}

native::Config config() {
  native::Config value;
  value.imu_init_samples = 20;
  value.point_filter_num = 1;
  value.blind_m = 0.1;
  value.elevator_enabled = true;
  value.automatic_door = false;
  return value;
}

native::TimedCloud room(double stamp, float intensity, double scale = 1.0,
                        double duration = kScanDuration) {
  native::TimedCloud cloud;
  cloud.stamp_s = stamp;
  cloud.end_stamp_s = stamp + duration;
  for (int surface = 0; surface < 3; ++surface) {
    for (int a = -8; a <= 8; ++a) {
      for (int b = -8; b <= 8; ++b) {
        native::TimedPoint p;
        if (surface == 0) {
          p.x = static_cast<float>(3.0 * scale);
          p.y = a * 0.09F;
          p.z = b * 0.09F;
        } else if (surface == 1) {
          p.x = a * 0.09F;
          p.y = static_cast<float>(4.0 * scale);
          p.z = b * 0.09F;
        } else {
          p.x = a * 0.09F;
          p.y = b * 0.09F;
          p.z = static_cast<float>(-2.0 * scale);
        }
        p.intensity = intensity;
        cloud.points.push_back(p);
      }
    }
  }
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].offset_ns = static_cast<std::uint64_t>(std::llround(
        duration * 1e9 * static_cast<double>(i) / (cloud.points.size() - 1)));
    cloud.points[i].line = static_cast<std::uint8_t>(i % 4);
  }
  return cloud;
}

struct ImuCursor {
  int index = 0;
  double gravity = kGravity;
  Eigen::Vector3d bias = Eigen::Vector3d::Zero();
};

native::ImuSample sample(double stamp, double gravity,
                         const Eigen::Vector3d& gyro) {
  native::ImuSample value;
  value.stamp_s = stamp;
  value.acceleration = Eigen::Vector3d(0.0, 0.0, gravity);
  value.angular_velocity_rps = gyro;
  return value;
}

native::Result step(native::Frontend& frontend, ImuCursor& cursor, int frame,
                    float intensity, double scale = 1.0) {
  auto scan = room(0.20 + frame * 0.025, intensity, scale);
  while (cursor.index * kImuDt <= scan.end_stamp_s + 0.006 + 1e-12) {
    frontend.feedImu(sample(cursor.index * kImuDt, cursor.gravity, cursor.bias));
    ++cursor.index;
  }
  frontend.feedCloud(scan);
  auto result = frontend.processNext();
  require(result.has_value(), "covered native scan did not return a result");
  near(result->scan_end_s, scan.end_stamp_s, 1e-12, "native scan end changed");
  near(result->state.stamp_s, scan.end_stamp_s, 1e-12,
       "native state is not at the reported scan end");
  return *result;
}

void sameResult(const native::Result& actual, const native::Result& expected) {
  require(actual.initialized == expected.initialized &&
          actual.map_ready == expected.map_ready &&
          actual.attempted == expected.attempted && actual.accepted == expected.accepted &&
          actual.effective_points == expected.effective_points,
          "interleaved/reset native processing changed scan status");
  near(actual.state.orientation.angularDistance(expected.state.orientation), 0.0, 1e-9,
       "another instance/reset changed orientation");
  near((actual.world_position - expected.world_position).norm(), 0.0, 1e-9,
       "another instance/reset changed world position");
  near((actual.world_velocity - expected.world_velocity).norm(), 0.0, 1e-9,
       "another instance/reset changed world velocity");
  near((actual.state.gyro_bias - expected.state.gyro_bias).norm(), 0.0, 1e-12,
       "another instance/reset changed gyro initialization");
  near((actual.state.gravity - expected.state.gravity).norm(), 0.0, 1e-12,
       "another instance/reset changed gravity initialization");
  near((actual.state.covariance - expected.state.covariance).norm(), 0.0, 1e-9,
       "another instance/reset changed posterior covariance");
  require(actual.state.in_elevator == expected.state.in_elevator &&
          actual.elevator.entries == expected.elevator.entries &&
          actual.elevator.exits == expected.elevator.exits &&
          actual.elevator.phase == expected.elevator.phase,
          "another instance/reset changed transport state");
}

void onlyIntensity(const native::Cloud& cloud, float value, const char* message) {
  require(!cloud.empty(), "native map snapshot is unexpectedly empty");
  for (const auto& p : cloud) near(p.intensity, value, 1e-3, message);
}

void onRoomPlanes(const Eigen::Vector3d& p, double scale, const char* message) {
  const double residual = std::min({std::abs(p.x() - 3.0 * scale),
                                    std::abs(p.y() - 4.0 * scale),
                                    std::abs(p.z() + 2.0 * scale)});
  near(residual, 0.0, 3e-4, message);
}

std::vector<native::Result> reference(const ::lingtu::ltlio::Config& settings,
                                     ImuCursor cursor, float intensity, int count) {
  native::Frontend frontend(settings);
  std::vector<native::Result> result;
  for (int frame = 0; frame < count; ++frame)
    result.push_back(step(frontend, cursor, frame, intensity));
  require(result.back().map_ready &&
          std::any_of(result.begin(), result.end(), [](const native::Result& value) {
            return value.attempted && value.accepted && value.effective_points > 0;
          }), "native fixture did not perform real accepted LiDAR matching");
  return result;
}

void testInstanceIsolationAndReset() {
  auto a_config = config();
  a_config.imu_from_lidar_rotation =
      Eigen::AngleAxisd(0.32, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  a_config.imu_from_lidar_translation = Eigen::Vector3d(0.35, -0.20, 0.10);
  auto b_config = config();
  b_config.imu_from_lidar_rotation =
      (Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitX()) *
       Eigen::AngleAxisd(-0.6, Eigen::Vector3d::UnitZ())).toRotationMatrix();
  b_config.imu_from_lidar_translation = Eigen::Vector3d(-0.22, 0.11, 0.06);
  ImuCursor a_imu;
  a_imu.bias = Eigen::Vector3d(0.002, -0.001, 0.003);
  ImuCursor b_imu;
  b_imu.gravity = 9.70;
  b_imu.bias = Eigen::Vector3d(0.02, -0.03, 0.04);
  const auto a_reference = reference(a_config, a_imu, 11.0F, kFrames);
  const auto b_reference = reference(b_config, b_imu, 201.0F, kFrames + 1);
  native::Frontend a(a_config), b(b_config);
  require(!a.initialized() && a.mapSnapshot()->empty(), "new frontend retained state");
  native::Cloud::ConstPtr retained_scan;
  pcl::PointXYZINormal retained_point;
  std::size_t retained_count = 0;
  for (int frame = 0; frame < kFrames; ++frame) {
    const auto ar = step(a, a_imu, frame, 11.0F);
    const auto br = step(b, b_imu, frame, 201.0F);
    sameResult(ar, a_reference[frame]);
    sameResult(br, b_reference[frame]);
    if (frame == 6) {
      require(ar.deskewed_cloud && !ar.deskewed_cloud->empty(), "native scan snapshot missing");
      retained_scan = ar.deskewed_cloud;
      retained_point = retained_scan->front();
      retained_count = retained_scan->size();
    }
  }
  auto a_map = a.mapSnapshot();
  onlyIntensity(*a_map, 11.0F, "second instance leaked into the first map");
  onlyIntensity(*b.mapSnapshot(), 201.0F, "first instance leaked into the second map");
  for (const auto& p : *a_map) {
    const Eigen::Vector3d lidar = a_config.imu_from_lidar_rotation.transpose() *
        (Eigen::Vector3d(p.x, p.y, p.z) - a_config.imu_from_lidar_translation);
    onRoomPlanes(lidar, 1.0, "native map omitted or doubled fixed LiDAR extrinsics");
  }
  for (const auto& p : *retained_scan)
    onRoomPlanes(Eigen::Vector3d(p.x, p.y, p.z), 1.0,
                 "deskew snapshot was transformed out of LiDAR coordinates");
  a_map->front().intensity = 1234.0F;
  onlyIntensity(*a.mapSnapshot(), 11.0F, "mapSnapshot aliases the live map");

  // All three pending inputs must disappear, including the manual request.
  a.feedCloud(room(10.0, 53.0F));
  a.feedImu(sample(10.01, kGravity, Eigen::Vector3d::Ones()));
  a.requestElevator(true);
  a.reset();
  require(!a.initialized() && a.mapSnapshot()->empty(), "reset retained initialization or map");
  require(!a.processNext().has_value(), "reset retained a queued scan or IMU work item");
  a_imu.index = 0;
  for (int frame = 0; frame < kFrames; ++frame)
    sameResult(step(a, a_imu, frame, 11.0F), a_reference[frame]);
  sameResult(step(b, b_imu, kFrames, 201.0F), b_reference[kFrames]);

  a.reset();
  a_imu.index = 0;
  for (int frame = 0; frame < kFrames; ++frame) step(a, a_imu, frame, 71.0F, 1.4);
  onlyIntensity(*a.mapSnapshot(), 71.0F, "old native map survived reset into a different scene");
  require(retained_scan->size() == retained_count &&
          retained_scan->front().x == retained_point.x &&
          retained_scan->front().y == retained_point.y &&
          retained_scan->front().z == retained_point.z &&
          retained_scan->front().intensity == retained_point.intensity,
          "later frames/reset mutated an earlier result's scan snapshot");
}

void testScanEndInterpolation() {
  auto settings = config();
  settings.elevator_enabled = false;
  native::Frontend frontend(settings);
  for (int i = 0; i <= 23; ++i)
    frontend.feedImu(sample(i * kImuDt, kGravity,
        Eigen::Vector3d(0.0, 0.0, i < 20 ? 0.0 : 0.8)));
  frontend.feedCloud(room(0.100, 31.0F, 1.0, 0.0175));
  require(!frontend.processNext(), "uncovered scan was processed without an IMU right bracket");
  frontend.feedImu(sample(0.120, kGravity, Eigen::Vector3d(0.0, 0.0, 0.8)));
  frontend.feedImu(sample(0.125, kGravity, Eigen::Vector3d(0.0, 0.0, 9.0)));
  const auto result = frontend.processNext();
  require(result && result->initialized && !result->attempted,
          "interpolation fixture must run before the first LiDAR correction");
  near(result->state.stamp_s, 0.1175, 1e-12, "interpolated state timestamp is wrong");
  // Initialization consumes 0..0.095. The first history sample at 0.100 seeds
  // the predictor; the constant angular rate then integrates exactly on SO(2).
  const auto rotation = result->state.orientation.toRotationMatrix();
  near(std::atan2(rotation(1, 0), rotation(0, 0)), 0.8 * (0.1175 - 0.100), 1e-8,
       "scan result used the latest propagated IMU state instead of interpolation");
  near(result->causal_gyro_stamp_s, 0.115, 1e-12,
       "scan gyro timestamp came from a future sample");
  near(result->causal_gyro_rps.z(), 0.8, 1e-12, "future gyro leaked into scan output");
}

void testEmptyFrameKeepsScanOrder() {
  auto settings = config();
  settings.elevator_enabled = false;
  native::Frontend frontend(settings);
  for (int i = 0; i <= 23; ++i)
    frontend.feedImu(sample(i * kImuDt, kGravity, Eigen::Vector3d::Zero()));
  const auto normal = room(0.100, 31.0F, 1.0, 0.0175);
  frontend.feedCloud(normal);
  auto blind = room(0.125, 41.0F);
  for (auto& point : blind.points) point.x = point.y = point.z = 0.0F;
  frontend.feedCloud(blind);
  require(frontend.bufferedClouds() == 2,
          "normal and fully blind scans were not both queued");
  require(!frontend.processNext(),
          "later blind scan bypassed an earlier scan waiting for the IMU right bracket");
  frontend.feedImu(sample(0.120, kGravity, Eigen::Vector3d::Zero()));
  const auto first = frontend.processNext();
  require(first && first->deskewed_cloud && !first->deskewed_cloud->empty(),
          "covered normal scan was not returned before the blind scan");
  near(first->scan_end_s, normal.end_stamp_s, 1e-12,
       "normal scan lost its position in the result sequence");
  const auto second = frontend.processNext();
  require(second && second->initialized && !second->attempted && !second->accepted &&
          second->input_points == blind.points.size() && !second->deskewed_cloud,
          "fully blind scan did not return its rejected intake status");
  near(second->scan_end_s, blind.end_stamp_s, 1e-12,
       "blind scan lost its original frame end");
  require(frontend.bufferedClouds() == 0 && !frontend.processNext(),
          "ordered scan results were returned more than once");
}

void testUnsupportedOnlineExtrinsics() {
  auto settings = config();
  settings.estimate_extrinsics = true;
  bool rejected = false;
  try {
    native::Frontend unsupported(settings);
  } catch (const std::exception&) {
    rejected = true;
  }
  require(rejected, "unsupported online extrinsic estimation was silently ignored");
}

void testSparseFrameDoesNotReuseEffectivePoints() {
  auto settings = config();
  settings.elevator_enabled = false;
  native::Frontend frontend(settings);
  ImuCursor cursor;
  native::Result previous;
  for (int frame = 0; frame < kFrames; ++frame)
    previous = step(frontend, cursor, frame, 41.0F);
  require(previous.attempted && previous.accepted && previous.effective_points > 0,
          "sparse-frame fixture did not establish a real successful large scan");

  auto sparse = room(0.20 + kFrames * 0.025, 41.0F);
  std::vector<native::TimedPoint> points;
  // Upstream voxelizes only when the intake cloud has more than ten points.
  // Many repeated returns pass that gate but form just three voxel centroids,
  // reaching ESEKF's separate sampled-point-count early return.
  constexpr std::size_t raw_points = 90;
  for (std::size_t i = 0; i < raw_points; ++i) {
    auto point = sparse.points[(i % 3) * 17 * 17 + 19];
    point.offset_ns = i * 20'000'000ULL / (raw_points - 1);
    points.push_back(point);
  }
  sparse.points = points;
  while (cursor.index * kImuDt <= sparse.end_stamp_s + 0.006 + 1e-12) {
    frontend.feedImu(sample(cursor.index * kImuDt, cursor.gravity, cursor.bias));
    ++cursor.index;
  }
  frontend.feedCloud(sparse);
  const auto result = frontend.processNext();
  require(result && result->map_ready && result->attempted &&
          result->input_points == raw_points && result->sampled_points > 0 &&
          result->sampled_points < 10,
          "dense raw scan did not reach the voxel-sparse ESEKF path");
  near(result->scan_end_s, sparse.end_stamp_s, 1e-12,
       "sparse scan returned the previous scan result");
  require(result->effective_points == 0 && !result->accepted,
          "sparse scan reused the previous effective-point count or reported acceptance");

  previous = step(frontend, cursor, kFrames + 1, 41.0F);
  require(previous.accepted && previous.sampled_points >= 10 && previous.effective_points > 0,
          "raw-small fixture did not restore a successful large scan first");
  auto raw_small = room(0.20 + (kFrames + 2) * 0.025, 41.0F);
  points.clear();
  for (std::size_t i = 0; i < 9; ++i) {
    auto point = raw_small.points[i * (raw_small.points.size() - 1) / 8];
    point.offset_ns = i * 2'500'000;
    points.push_back(point);
  }
  raw_small.points = points;
  while (cursor.index * kImuDt <= raw_small.end_stamp_s + 0.006 + 1e-12) {
    frontend.feedImu(sample(cursor.index * kImuDt, cursor.gravity, cursor.bias));
    ++cursor.index;
  }
  frontend.feedCloud(raw_small);
  const auto raw_small_result = frontend.processNext();
  require(raw_small_result && raw_small_result->map_ready &&
          raw_small_result->input_points == 9,
          "raw-small scan did not return its own result");
  near(raw_small_result->scan_end_s, raw_small.end_stamp_s, 1e-12,
       "raw-small scan returned the previous scan timestamp");
  require(raw_small_result->sampled_points == 0 &&
          raw_small_result->effective_points == 0 && !raw_small_result->accepted,
          "raw-small scan reused the preceding sampled cloud or matching result");
}

void testRangeFilterPreservesRawEnd() {
  auto settings = config();
  settings.elevator_enabled = false;
  settings.max_range_m = 3.5;
  native::Frontend frontend(settings);
  ImuCursor cursor;
  for (int frame = 0; frame < kFrames; ++frame) step(frontend, cursor, frame, 31.0F);
  auto scan = room(0.20 + kFrames * 0.025, 31.0F, 1.0, 0.030);
  auto tail = scan.points.back();
  tail.x = 20.0F;
  tail.offset_ns = 30'000'000;
  scan.points.push_back(tail);
  while (cursor.index * kImuDt <= scan.end_stamp_s + 0.006 + 1e-12)
    frontend.feedImu(sample(cursor.index++ * kImuDt, cursor.gravity, cursor.bias));
  frontend.feedCloud(scan);
  const auto result = frontend.processNext();
  require(result && result->deskewed_cloud && !result->deskewed_cloud->empty(),
          "range filtering removed every nearby observation");
  near(result->scan_end_s, scan.end_stamp_s, 1e-12,
       "range filtering shortened the raw frame end");
  require(result->deskewed_cloud->size() < scan.points.size() - 1,
          "upper range limit did not remove far returns");
  for (const auto& point : *result->deskewed_cloud)
    require(Eigen::Vector3d(point.x, point.y, point.z).norm() <= 3.5 + 1e-4,
            "a far return survived the configured intake range");
}

void testCorrectStateReintegratesHistory() {
  auto settings = config();
  settings.elevator_enabled = false;
  native::Frontend frontend(settings);
  ImuCursor cursor;
  native::Result last;
  for (int frame = 0; frame < kFrames; ++frame) last = step(frontend, cursor, frame, 51.0F);
  const Eigen::Vector3d velocity(0.4, 0.0, 0.0);
  require(frontend.correctState(last.scan_end_s, last.state.orientation,
                               last.world_position, velocity),
          "covered external pose/velocity correction was rejected");
  require(!frontend.correctState(-1.0, last.state.orientation, Eigen::Vector3d::Zero()),
          "external correction accepted a time outside IMU history");
  auto sparse = room(0.20 + kFrames * 0.025, 51.0F);
  const auto point = sparse.points[20];
  sparse.points.assign(90, point);
  for (std::size_t i = 0; i < sparse.points.size(); ++i)
    sparse.points[i].offset_ns = i * 20'000'000ULL / (sparse.points.size() - 1);
  while (cursor.index * kImuDt <= sparse.end_stamp_s + 0.006 + 1e-12)
    frontend.feedImu(sample(cursor.index++ * kImuDt, cursor.gravity, cursor.bias));
  frontend.feedCloud(sparse);
  const auto next = frontend.processNext();
  require(next && next->attempted && !next->accepted,
          "correction fixture unexpectedly had a LiDAR update to hide its prior");
  near((next->world_velocity - velocity).norm(), 0.0, 1e-7,
       "external velocity correction did not reach IMU prediction");
  near((next->world_position - last.world_position -
        velocity * (next->scan_end_s - last.scan_end_s)).norm(), 0.0, 1e-6,
       "correctState only changed output instead of reintegrating IMU history");
}

void testConstantVelocityIsNotStationary() {
  auto settings = config();
  settings.elevator_enabled = false;
  settings.imu_static_acc_thresh = 0.22;
  settings.imu_static_gyro_thresh = 0.022;
  settings.scan_resolution = 0.05;
  native::Frontend frontend(settings);
  ImuCursor cursor;
  native::Result last;
  for (int frame = 0; frame < kFrames; ++frame) last = step(frontend, cursor, frame, 61.0F);
  const Eigen::Vector3d velocity(0.4, 0.0, 0.0);
  const auto origin = last.world_position;
  const double start = last.scan_end_s;
  require(frontend.correctState(start, last.state.orientation, origin, velocity),
          "constant-velocity fixture could not establish its initial motion");
  for (int frame = kFrames; frame < kFrames + 60; ++frame) {
    auto scan = room(0.20 + frame * 0.025, 61.0F);
    for (auto& point : scan.points)
      point.x -= static_cast<float>(velocity.x() *
          (scan.stamp_s + point.offset_ns * 1e-9 - start));
    while (cursor.index * kImuDt <= scan.end_stamp_s + 0.006 + 1e-12)
      frontend.feedImu(sample(cursor.index++ * kImuDt, cursor.gravity, cursor.bias));
    frontend.feedCloud(scan);
    const auto next = frontend.processNext();
    require(next && next->accepted, "moving three-plane fixture lost real LiDAR matching");
    last = *next;
  }
  require(last.world_velocity.x() > 0.2,
          "steady IMU falsely clamped independently observed constant motion to zero");
  near(last.world_position.x() - origin.x(), velocity.x() * (last.scan_end_s - start),
       0.03, "stationary constraints corrupted constant-speed translation");
}

void writeMatrix(std::ostream& output, const Eigen::Matrix3d& rotation) {
  output << '[';
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) {
      if (row != 0 || col != 0) output << ',';
      output << rotation(row, col);
    }
  output << "]\n";
}

void testInitialMapBeforeHistory() {
  native::Frontend early(config()), covered(config());
  int imu_index = 0;
  const auto advance_both = [&](double until) {
    while (imu_index * kImuDt <= until + 1e-12) {
      const auto input = sample(imu_index * kImuDt, kGravity, Eigen::Vector3d::Zero());
      early.feedImu(input);
      covered.feedImu(input);
      ++imu_index;
    }
  };
  advance_both(0.125);
  require(early.initialized() && covered.initialized(), "initial-map fixture did not initialize IMU");

  // In Office1 the first cloud delivered after initialization ends just before
  // the initializing IMU. Upstream accumulates it as raw initial-map geometry.
  const auto initial_early = room(0.074, 41.0F);
  const auto initial_covered = room(0.101, 41.0F);
  require(initial_early.end_stamp_s < early.initialState()->stamp_s,
          "initial-map fixture does not precede the initializing IMU");
  early.feedCloud(initial_early);
  covered.feedCloud(initial_covered);
  const auto first_early = early.processNext();
  const auto first_covered = covered.processNext();
  require(first_early && first_covered && first_early->initialized &&
          !first_early->attempted && !first_early->map_ready && early.droppedClouds() == 0,
          "first raw map frame was incorrectly discarded for missing IMU history");

  for (int frame = 1; frame < 5; ++frame) {
    const auto input = room(0.130 + (frame - 1) * 0.025, 41.0F);
    advance_both(input.end_stamp_s + kImuDt);
    early.feedCloud(input);
    covered.feedCloud(input);
    const auto a = early.processNext();
    const auto b = covered.processNext();
    require(a && b && !a->attempted && !b->attempted &&
            a->map_ready == (frame == 4) && b->map_ready == (frame == 4),
            "initial map did not become ready on exactly the fifth raw frame");
  }
  const auto early_map = early.mapSnapshot();
  const auto covered_map = covered.mapSnapshot();
  require(!early_map->empty() && early_map->size() == covered_map->size(),
          "initial timing changed the raw initial-map geometry");
  for (std::size_t i = 0; i < early_map->size(); ++i) {
    near((early_map->points[i].getVector3fMap() - covered_map->points[i].getVector3fMap()).norm(),
         0.0, 1e-9, "first raw map frame was incorrectly deskewed or transformed");
  }
  const auto scan = room(0.230, 41.0F);
  advance_both(scan.end_stamp_s + kImuDt);
  early.feedCloud(scan);
  covered.feedCloud(scan);
  const auto a = early.processNext();
  const auto b = covered.processNext();
  require(a && b && a->accepted && b->accepted && early.droppedClouds() == 0 &&
          covered.droppedClouds() == 0,
          "sixth raw frame did not perform the first real LiDAR update");
  sameResult(*a, *b);
}

void testExpiredElevatorScan() {
  native::Frontend frontend(config());
  ImuCursor cursor;
  native::Result previous;
  for (int frame = 0; frame < kFrames; ++frame)
    previous = step(frontend, cursor, frame, 41.0F);
  require(previous.accepted, "history fixture did not establish LiDAR matching");

  frontend.requestElevator(true);
  // Continue real IMU input while LiDAR is delayed until the 1000-state ring
  // has wrapped. Manual mode keeps the automatic door detector out of this test.
  for (int i = 0; i < 1203; ++i, ++cursor.index)
    frontend.feedImu(sample(cursor.index * kImuDt, kGravity, Eigen::Vector3d::Zero()));
  const double oldest = (cursor.index - 1000) * kImuDt;
  const double latest = (cursor.index - 1) * kImuDt;
  auto expired = room(previous.scan_end_s + 0.025, 41.0F);
  require(expired.end_stamp_s < oldest, "history fixture did not expire its delayed scan");
  frontend.feedCloud(expired);
  auto boundary = room(oldest - kScanDuration, 41.0F);
  boundary.end_stamp_s = oldest;
  frontend.feedCloud(boundary);
  auto current = room(latest - kScanDuration, 41.0F);
  current.end_stamp_s = latest;
  frontend.feedCloud(current);

  const auto first = frontend.processNext();
  require(first && first->accepted && first->state.in_elevator && first->elevator.entries == 1,
          "oldest retained elevator scan failed after the expired scan was discarded");
  near(first->scan_end_s, oldest, 1e-12, "expired scan relabeled the latest pose as old data");
  near(first->state.stamp_s, oldest, 1e-12, "oldest retained state has the wrong timestamp");
  require(first->state.covariance.allFinite() && frontend.droppedClouds() == 1,
          "expired elevator scan corrupted state or was not counted as dropped");
  const auto second = frontend.processNext();
  require(second && second->accepted && second->state.in_elevator &&
          second->state.covariance.allFinite(),
          "current elevator scan did not recover after expired history");
  near(second->scan_end_s, latest, 1e-12, "current scan was replaced by an old pose");
  require(frontend.droppedClouds() == 1 && frontend.bufferedClouds() == 0 &&
          !frontend.processNext(), "expired history caused extra drops or retained scan work");
}

slam::LidarFrame backendCloud(const native::TimedCloud& source) {
  slam::LidarFrame cloud;
  cloud.stamp_s = source.stamp_s;
  cloud.frame_id = "lidar";
  for (const auto& p : source.points) {
    slam::PointXYZIT point;
    point.x = p.x; point.y = p.y; point.z = p.z; point.intensity = p.intensity;
    point.offset_time_ns = static_cast<std::int64_t>(p.offset_ns);
    point.line = p.line; point.tag = p.tag;
    cloud.points.push_back(point);
  }
  return cloud;
}

slam::SlamOutputs backendStepCloud(slam::ISlamBackend& backend, int& imu_index,
                                  const native::TimedCloud& scan) {
  while (imu_index * kImuDt <= scan.end_stamp_s + 0.006 + 1e-12) {
    slam::ImuSample imu;
    imu.stamp_s = imu_index++ * kImuDt;
    imu.az = kGravity;
    require(backend.feedImu(imu).ok, "official backend rejected SI IMU sample");
  }
  require(backend.feedLidar(backendCloud(scan)).ok, "official backend rejected scan");
  require(backend.tick().ok, "official backend tick failed");
  return backend.outputs();
}

slam::SlamOutputs backendStep(slam::ISlamBackend& backend, int& imu_index, int frame,
                              float intensity, double scale = 1.0) {
  return backendStepCloud(backend, imu_index,
      room(0.20 + frame * 0.025, intensity, scale));
}

void checkSavedMapLocalization(const slam::SlamConfig& settings,
                               const std::filesystem::path& map_path) {
  auto backend = slam::makeLtlioBackend();
  require(backend && backend->configure(settings).ok,
          "official localization configuration failed");
  require(backend->loadMap(map_path.string()).ok,
          "official saved map did not load through ISlamBackend");
  require(backend->setMode(slam::SlamMode::Localization, "").ok,
          "official localization mode failed after explicit map loading");
  auto output = backend->outputs();
  require(output.map_loaded && output.saved_map_points > 0 &&
          output.relocalization_supported && output.saved_map_relocalization_supported &&
          output.state == slam::SlamState::Localizing && !output.map_odom_tf &&
          !output.odometry_odom_body,
          "loaded official map did not enable the saved-map localization interface");
  int imu_index = 0;
  for (int frame = 0; frame < kFrames; ++frame)
    output = backendStep(*backend, imu_index, frame, 87.0F, 1.6);
  require(output.odometry_odom_body && output.registered_cloud_body &&
          output.ltlio_lidar_update.accepted,
          "localization fixture never established real odometry before the time jump");
  const auto local_pose_before_alignment = *output.odometry_odom_body;
  const auto alignment = backend->relocalize(slam::Pose3d{});
  require(alignment.ok, "official saved-map alignment failed: " + alignment.message);
  output = backend->outputs();
  require(output.state == slam::SlamState::Tracking && output.map_odom_tf &&
          output.relocalization_map_body && output.odometry_odom_body &&
          output.relocalization_state == "completed" &&
          output.relocalization_refine_converged &&
          output.relocalization_refine_inliers > 0,
          "official saved-map alignment did not publish a measured map transform");
  near(Eigen::Vector3d(output.relocalization_map_body->x,
                      output.relocalization_map_body->y,
                      output.relocalization_map_body->z).norm(),
       0.0, 0.03, "saved-map alignment misplaced the static navigation body");
  near(Eigen::Vector3d(output.odometry_odom_body->x - local_pose_before_alignment.x,
                      output.odometry_odom_body->y - local_pose_before_alignment.y,
                      output.odometry_odom_body->z - local_pose_before_alignment.z).norm(),
       0.0, 1e-12, "saved-map alignment rewrote the local odometry position");
  const Eigen::Quaterniond before_q(local_pose_before_alignment.qw,
      local_pose_before_alignment.qx, local_pose_before_alignment.qy,
      local_pose_before_alignment.qz);
  const auto& aligned_odom = *output.odometry_odom_body;
  near(before_q.angularDistance(Eigen::Quaterniond(aligned_odom.qw,
           aligned_odom.qx, aligned_odom.qy, aligned_odom.qz)),
       0.0, 1e-12, "saved-map alignment rewrote the local odometry rotation");
  const auto previous_epoch = output.source_epoch;
  slam::ImuSample jumped_imu;
  jumped_imu.stamp_s = 100.0;
  jumped_imu.az = kGravity;
  require(backend->feedImu(jumped_imu).ok, "official localization time jump was rejected");
  const auto require_unaligned = [](const slam::SlamOutputs& value) {
    require(value.state == slam::SlamState::Lost &&
            value.reason == "sensor_time_jump_relocalization_required" &&
            !value.odometry_odom_body && !value.odometry_twist_body &&
            !value.registered_cloud_body && !value.map_cloud_map && !value.map_odom_tf,
            "localization time jump exposed old or unaligned navigation output");
  };
  output = backend->outputs();
  require(output.source_epoch != previous_epoch && output.map_loaded &&
          output.saved_map_relocalization_supported,
          "time jump discarded the loaded map or retained the old source epoch");
  require_unaligned(output);
  imu_index = 20001;
  // Exercise both map accumulation and the first new matched scan. Neither
  // may restore navigation output before saved-map alignment is established.
  for (int frame = 0; frame < 6; ++frame) {
    output = backendStepCloud(*backend, imu_index,
        room(100.20 + frame * 0.025, 87.0F, 1.6));
    require_unaligned(output);
  }
  require(output.observation_sequence > 0,
          "time-jump fixture did not exercise private scan progress");
  const auto recovery_started = backend->startRelocalizeAsync(slam::Pose3d{});
  require(recovery_started.ok,
          "time-jump real relocalization did not start: " + recovery_started.message);
  const auto recovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  std::optional<slam::Status> recovery;
  while (std::chrono::steady_clock::now() < recovery_deadline) {
    recovery = backend->pollRelocalizeAsync();
    if (recovery) break;
    require(backend->tick().ok, "time-jump relocalization blocked scan polling");
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(recovery && recovery->ok,
          "time-jump real relocalization failed: " +
              (recovery ? recovery->message : std::string("timeout")));
  output = backend->outputs();
  require(output.state == slam::SlamState::Tracking && output.map_odom_tf &&
          output.odometry_odom_body && output.registered_cloud_body &&
          output.relocalization_refine_converged,
          "time-jump relocalization did not restore measured navigation output");
  near(output.stamp_s, 100.20 + 5 * 0.025 + kScanDuration, 1e-12,
       "time-jump recovery resurrected a pre-jump scan");
  const auto jumped_epoch = output.source_epoch;
  require(backend->reset().ok, "official localization reset failed");
  output = backend->outputs();
  require(output.source_epoch != jumped_epoch && !output.map_loaded &&
          !output.saved_map_relocalization_supported && !output.odometry_odom_body &&
          !output.registered_cloud_body && !output.map_cloud_map && !output.map_odom_tf,
          "localization reset retained old map readiness or odometry");
  require(backend->setMode(slam::SlamMode::Localization, map_path.string()).ok,
          "official localization did not reload the saved map after reset");
  output = backend->outputs();
  require(output.map_loaded && output.saved_map_relocalization_supported &&
          !output.odometry_odom_body && !output.registered_cloud_body,
          "map reload either failed readiness or resurrected an old scan");
}

void testConvergenceDiagnostics() {
  auto settings = config();
  settings.elevator_enabled = false;
  settings.max_range_m = 50.0; // Let the 40 m fixture reach matching rather than range filtering.
  native::Frontend frontend(settings);
  ImuCursor cursor;
  native::Result result;
  for (int frame = 0; frame < kFrames; ++frame) {
    result = step(frontend, cursor, frame, 43.0F);
    if (!result.attempted)
      require(result.iterations == 0 && !result.converged,
              "map initialization claimed an evaluated/converged LiDAR update");
  }
  require(result.accepted && result.converged &&
          result.iterations >= 2 && result.iterations <= 4,
          "static matching did not report its actual converged iterations");

  auto small = room(0.20 + kFrames * 0.025, 43.0F);
  small.points.resize(9);
  small.points.back().offset_ns = 20'000'000;
  while (cursor.index * kImuDt <= small.end_stamp_s + 0.006 + 1e-12) {
    frontend.feedImu(sample(cursor.index * kImuDt, cursor.gravity, cursor.bias));
    ++cursor.index;
  }
  frontend.feedCloud(small);
  const auto skipped = frontend.processNext();
  require(skipped && skipped->attempted && !skipped->accepted &&
          skipped->iterations == 0 && !skipped->converged,
          "small scan reused the preceding convergence result");

  const auto no_match = step(frontend, cursor, kFrames + 1, 43.0F, 10.0);
  require(no_match.attempted && !no_match.accepted &&
          no_match.iterations == 1 && !no_match.converged,
          "failed nearest-neighbor match claimed convergence or lost its attempted iteration");
}

void testBackendImuGap(const std::filesystem::path& directory) {
  std::filesystem::create_directories(directory);
  const auto path = directory / "official-imu-gap.yaml";
  {
    std::ofstream output(path);
    output << "elevator_mode: off\nimu_init_num: 20\n"
              "lidar_filter_num: 1\nlidar_min_range: 0.1\nmax_imu_gap_s: 0.25\n"
              "sam:\n  min_time_s: 1000000000.0\n";
    require(static_cast<bool>(output), "cannot write official IMU-gap configuration");
  }
  auto backend = slam::makeLtlioBackend();
  slam::SlamConfig settings;
  settings.config_path = path.string();
  require(backend && backend->configure(settings).ok,
          "official IMU-gap configuration failed");
  require(backend->setMode(slam::SlamMode::Mapping, "").ok,
          "official IMU-gap mapping mode failed");
  int imu_index = 0;
  for (int frame = 0; frame < kFrames; ++frame)
    backendStep(*backend, imu_index, frame, 31.0F);
  const auto before = backend->outputs();
  require(before.state == slam::SlamState::Tracking && before.odometry_odom_body,
          "official IMU-gap fixture never established tracking");

  // A filtered-empty intake notification must not bypass the same gap health
  // check used when a normal scan is waiting for its covering IMU.
  auto blind = room(before.last_imu_s + 0.30, 31.0F);
  for (auto& point : blind.points) point.x = point.y = point.z = 0.01F;
  require(backend->feedLidar(backendCloud(blind)).ok && backend->tick().ok,
          "official IMU-gap blind scan intake failed");
  const auto blind_waiting = backend->outputs();
  require(blind_waiting.state == slam::SlamState::Degraded &&
          blind_waiting.reason == "imu_gap_waiting_for_scan" &&
          blind_waiting.stamp_s == before.stamp_s &&
          blind_waiting.observation_sequence == before.observation_sequence,
          "filtered-empty scan bypassed IMU-gap health or reused an old estimate");

  for (int frame = 7; frame <= 10; ++frame) {
    const auto scan = room(before.last_imu_s + frame * 0.05, 31.0F);
    require(backend->feedLidar(backendCloud(scan)).ok,
            "official IMU-gap scan intake failed");
    require(backend->tick().ok, "official IMU-gap waiting tick failed");
  }
  const auto waiting = backend->outputs();
  require(waiting.stamp_s == before.stamp_s &&
          waiting.observation_sequence == before.observation_sequence,
          "official IMU gap published or retimestamped an old observation");
  require(waiting.state == slam::SlamState::Degraded &&
          waiting.confidence == 0.0 && waiting.localization_quality == 0.0 &&
          waiting.reason == "imu_gap_waiting_for_scan",
          "official IMU gap incorrectly retained healthy tracking");

  const auto recovery_scan = room(before.last_imu_s + 0.60, 31.0F);
  auto recovered = backendStepCloud(*backend, imu_index, recovery_scan);
  // Both retained queued scans and a policy which discards stale scans may
  // recover, but the current covered scan must eventually become observable.
  for (int tick = 0; tick < 16 && recovered.stamp_s < recovery_scan.end_stamp_s; ++tick) {
    require(backend->tick().ok, "official IMU-gap recovery tick failed");
    recovered = backend->outputs();
  }
  require(recovered.state == slam::SlamState::Tracking &&
          recovered.ltlio_lidar_update.accepted && recovered.odometry_odom_body &&
          recovered.observation_sequence > before.observation_sequence,
          "official IMU-gap recovery required a frontend reset");
  near(recovered.stamp_s, recovery_scan.end_stamp_s, 1e-12,
       "official IMU-gap recovery did not reach the current scan");
}

void testBackendFramesAndReset(const std::filesystem::path& directory) {
  std::filesystem::create_directories(directory);
  const auto path = directory / "official.yaml";
  const Eigen::Matrix3d body_from_imu =
      Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d body_imu_translation(0.38, -0.02, 0.10);
  const Eigen::Matrix3d imu_from_lidar =
      (Eigen::AngleAxisd(-0.3, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY())).toRotationMatrix();
  const Eigen::Vector3d imu_lidar_translation(0.12, 0.03, -0.04);
  {
    std::ofstream output(path);
    output.precision(17);
    output << "elevator_mode: off\nimu_init_num: 20\n"
              "lidar_filter_num: 1\nlidar_min_range: 0.1\n"
              "navigation_body_from_imu_translation: [0.38,-0.02,0.10]\n"
              "navigation_body_from_imu_rotation: ";
    writeMatrix(output, body_from_imu);
    output << "t_il: [0.12,0.03,-0.04]\nr_il: ";
    writeMatrix(output, imu_from_lidar);
    output << "patch_min_interval_s: 0.0\npatch_min_translation_m: 0.0\n"
              "patch_min_rotation_rad: 0.0\nsam:\n  min_time_s: 1000000000.0\n";
    require(static_cast<bool>(output), "cannot write native backend fixture configuration");
  }
  auto backend = slam::makeLtlioBackend();
  slam::SlamConfig settings;
  settings.config_path = path.string();
  require(backend && backend->configure(settings).ok, "official estimator configuration failed");
  require(backend->setMode(slam::SlamMode::Mapping, "").ok, "official mapping mode failed");
  int imu_index = 0;
  slam::SlamOutputs result;
  for (int frame = 0; frame < kFrames; ++frame)
    result = backendStep(*backend, imu_index, frame, 17.0F);
  require(result.state == slam::SlamState::Tracking && result.odometry_odom_body &&
          result.registered_cloud_body && result.map_cloud_map &&
          result.ltlio_lidar_update.attempted && result.ltlio_lidar_update.accepted,
          "official backend did not publish a real accepted scan");
  const auto& pose = *result.odometry_odom_body;
  near(Eigen::Vector3d(pose.x, pose.y, pose.z).norm(), 0.0, 2e-4,
       "initial navigation-body translation was not anchored to odom origin");
  const Eigen::Quaterniond orientation(pose.qw, pose.qx, pose.qy, pose.qz);
  near(orientation.angularDistance(Eigen::Quaterniond::Identity()), 0.0, 2e-4,
       "initial navigation-body yaw was not anchored");
  const Eigen::Matrix3d body_from_lidar = body_from_imu * imu_from_lidar;
  const Eigen::Vector3d body_lidar_translation =
      body_imu_translation + body_from_imu * imu_lidar_translation;
  require(result.registered_cloud_body->frame_id == settings.body_frame,
          "registered cloud lost the configured body frame");
  for (const auto& p : result.registered_cloud_body->points)
    onRoomPlanes(body_from_lidar.transpose() *
        (Eigen::Vector3d(p.x, p.y, p.z) - body_lidar_translation), 1.0,
        "body cloud omitted or doubled the fixed mounting transforms");
  require(result.registered_cloud_body->points.size() == result.map_cloud_map->points.size(),
          "body and world scan outputs do not describe the same scan");
  for (std::size_t i = 0; i < result.map_cloud_map->points.size(); ++i) {
    const auto& p = result.registered_cloud_body->points[i];
    const auto& world = result.map_cloud_map->points[i];
    const Eigen::Vector3d expected = orientation * Eigen::Vector3d(p.x, p.y, p.z) +
                                   Eigen::Vector3d(pose.x, pose.y, pose.z);
    near((expected - Eigen::Vector3d(world.x, world.y, world.z)).norm(), 0.0, 3e-5,
         "world scan and body pose use inconsistent output anchors");
  }
  auto invalid_tail_scan = room(0.20 + kFrames * 0.025, 17.0F);
  auto tail = invalid_tail_scan.points.back();
  tail.x = std::numeric_limits<float>::quiet_NaN();
  tail.offset_ns = 30'000'000;
  invalid_tail_scan.points.push_back(tail);
  invalid_tail_scan.end_stamp_s = invalid_tail_scan.stamp_s + 0.030;
  const auto before_tail_sequence = result.observation_sequence;
  result = backendStepCloud(*backend, imu_index, invalid_tail_scan);
  require(result.ltlio_lidar_update.accepted &&
          result.observation_sequence == before_tail_sequence + 1,
          "invalid-tail fixture did not process its valid scan points");
  near(result.scan_end_s, invalid_tail_scan.end_stamp_s, 1e-12,
       "dropping an invalid tail point shortened the original scan end");
  near(result.stamp_s, invalid_tail_scan.end_stamp_s, 1e-12,
       "invalid-tail scan published a state at the filtered point's end time");
  require(backend->saveMap((directory / "before" / "map.pcd").string()).ok,
          "official backend could not save its map");
  const auto before_reset = backend->outputs();
  require(before_reset.saved_map_cloud_map && !before_reset.saved_map_cloud_map->points.empty(),
          "official saved map output is missing");
  for (const auto& p : before_reset.saved_map_cloud_map->points)
    onRoomPlanes(body_from_lidar.transpose() *
        (Eigen::Vector3d(p.x, p.y, p.z) - body_lidar_translation), 1.0,
        "saved map omitted the native-world to odom anchor");

  require(backend->reset().ok, "official backend reset failed");
  const auto reset = backend->outputs();
  require(reset.source_epoch != before_reset.source_epoch &&
          !reset.odometry_odom_body && !reset.registered_cloud_body &&
          !reset.saved_map_cloud_map && !reset.global_map_cloud,
          "backend reset retained an old output or epoch");
  imu_index = 0;
  for (int frame = 0; frame < kFrames; ++frame)
    result = backendStep(*backend, imu_index, frame, 87.0F, 1.6);
  require(result.state == slam::SlamState::Tracking,
          "official backend did not reinitialize after reset");
  require(backend->saveMap((directory / "after" / "map.pcd").string()).ok,
          "official backend could not save the replacement scene");
  const auto replacement = backend->outputs();
  require(replacement.saved_map_cloud_map && !replacement.saved_map_cloud_map->points.empty(),
          "replacement saved map is empty");
  for (const auto& p : replacement.saved_map_cloud_map->points)
    near(p.intensity, 87.0, 1e-3, "old map/patch survived backend reset");
  for (const auto& p : before_reset.saved_map_cloud_map->points)
    near(p.intensity, 17.0, 1e-3, "reset mutated an already returned saved-map snapshot");
  checkSavedMapLocalization(settings, directory / "after" / "map.pcd");
}

void testInitialPoseBeforeInitialization(const std::filesystem::path& directory) {
  const auto path = directory / "initial-pose.yaml";
  {
    std::ofstream output(path);
    output << "elevator_mode: off\nimu_init_num: 20\nlidar_filter_num: 1\n"
              "lidar_min_range: 0.1\nsam:\n  min_time_s: 1000000000.0\n";
  }
  slam::SlamConfig settings;
  settings.config_path = path.string();
  auto backend = slam::makeLtlioBackend();
  require(backend && backend->configure(settings).ok &&
              backend->setMode(slam::SlamMode::Mapping, "").ok,
          "initial-pose mapping fixture failed to configure");
  const Eigen::Quaterniond rotation(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d translation(1.5, -2.0, 0.6);
  const slam::Pose3d seed{translation.x(), translation.y(), translation.z(),
                        rotation.x(), rotation.y(), rotation.z(), rotation.w()};
  require(backend->setInitialPose(seed).ok, "pre-initialization pose was rejected");
  int imu_index = 0;
  slam::SlamOutputs output;
  for (int frame = 0; frame < kFrames; ++frame)
    output = backendStep(*backend, imu_index, frame, 73.0F);
  require(output.ltlio_lidar_update.accepted && output.odometry_odom_body &&
              output.map_cloud_map && output.registered_cloud_body,
          "explicit initial pose did not survive real frontend initialization");
  const auto& pose = *output.odometry_odom_body;
  near((Eigen::Vector3d(pose.x, pose.y, pose.z) - translation).norm(), 0.0, 2e-4,
       "default body-origin anchoring overwrote the requested initial position");
  near(rotation.angularDistance(Eigen::Quaterniond(pose.qw, pose.qx, pose.qy, pose.qz)),
       0.0, 2e-4, "default yaw anchoring overwrote the requested initial heading");
  require(backend->saveMap((directory / "initial-pose" / "map.pcd").string()).ok,
          "map could not be saved from an explicit initial pose");
  const auto saved = backend->outputs().saved_map_cloud_map;
  require(saved && !saved->points.empty(), "initial-pose saved map is empty");
  for (const auto& p : saved->points)
    onRoomPlanes(rotation.conjugate() * (Eigen::Vector3d(p.x, p.y, p.z) - translation),
                 1.0, "saved map used a different initial pose than odometry");
}
}  // namespace

int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("lingtu-elevator-native-" + std::to_string(
          std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    testUnsupportedOnlineExtrinsics();
    testScanEndInterpolation();
    testEmptyFrameKeepsScanOrder();
    testInstanceIsolationAndReset();
    testSparseFrameDoesNotReuseEffectivePoints();
    testRangeFilterPreservesRawEnd();
    testCorrectStateReintegratesHistory();
    testConstantVelocityIsNotStationary();
    testInitialMapBeforeHistory();
    testExpiredElevatorScan();
    testConvergenceDiagnostics();
    testBackendImuGap(directory);
    testBackendFramesAndReset(directory);
    testInitialPoseBeforeInitialization(directory);
    std::filesystem::remove_all(directory);
    std::cout << "Official native frontend isolation, interpolation, reset and frame contracts passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\nTest files: " << directory << '\n';
    return 1;
  }
}
