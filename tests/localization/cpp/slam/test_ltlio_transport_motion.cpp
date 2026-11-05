#include "ltlio/frontend.hpp"
#include "slam.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
namespace lio = lingtu::ltlio;
namespace slam = lingtu::slam;
constexpr double kGravity = 9.80665;
constexpr double kImuDt = 0.005;
constexpr double kScanDt = 0.05;
constexpr double kScanDuration = 0.02;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

struct Ride {
  double start = 0.0;
  double direction = 1.0;
  static constexpr double acceleration = 0.7;
  static constexpr double ramp = 1.5;
  static constexpr double cruise = 3.0;

  double finish() const { return start + 2 * ramp + cruise; }
  double totalHeight() const { return direction * acceleration * ramp * (ramp + cruise); }

  double acc(double timestamp) const {
    const double elapsed = timestamp - start;
    if (elapsed < 0.0) return 0.0;
    if (elapsed < ramp) return direction * acceleration;
    if (elapsed < ramp + cruise) return 0.0;
    if (elapsed < 2 * ramp + cruise) return -direction * acceleration;
    return 0.0;
  }

  double height(double timestamp) const {
    const double elapsed = std::max(0.0, timestamp - start);
    const double speed = acceleration * ramp;
    if (elapsed < ramp) return direction * 0.5 * acceleration * elapsed * elapsed;
    if (elapsed < ramp + cruise)
      return direction * (0.5 * acceleration * ramp * ramp + speed * (elapsed - ramp));
    if (elapsed < 2 * ramp + cruise) {
      const double braking = elapsed - ramp - cruise;
      return direction * (0.5 * acceleration * ramp * ramp + speed * cruise +
                           speed * braking - 0.5 * acceleration * braking * braking);
    }
    return totalHeight();
  }
};

lio::TimedCloud cabin(double stamp, bool landing = false) {
  lio::TimedCloud cloud;
  cloud.stamp_s = stamp;
  cloud.end_stamp_s = stamp + kScanDuration;
  for (int plane = 0; plane < 3; ++plane)
    for (int a = -6; a <= 6; ++a)
      for (int b = -6; b <= 6; ++b) {
        lio::TimedPoint point;
        if (plane == 0) { point.x = 1.1F; point.y = a * 0.1F; point.z = b * 0.1F; }
        else if (plane == 1) { point.x = a * 0.1F; point.y = 1.4F; point.z = b * 0.1F; }
        else { point.x = a * 0.1F; point.y = b * 0.1F; point.z = -0.8F; }
        cloud.points.push_back(point);
      }
  if (landing) {
    // A newly visible landing-floor patch is absent from the original floor
    // and all moving cabin scans. Intensity identifies this observation when
    // inspecting the persistent building map after transport has ended.
    for (int x = 0; x <= 12; ++x)
      for (int y = 0; y <= 12; ++y) {
        lio::TimedPoint point;
        point.x = 2.0F + x * 0.1F;
        point.y = 1.8F + y * 0.1F;
        point.z = -0.8F;
        point.intensity = 233.0F;
        cloud.points.push_back(point);
      }
  }
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].line = static_cast<std::uint8_t>(i % 4);
    cloud.points[i].offset_ns = i * 20'000'000ULL / (cloud.points.size() - 1);
  }
  return cloud;
}

lio::Config config() {
  lio::Config value;
  value.imu_init_samples = 20;
  value.point_filter_num = 1;
  value.blind_m = 0.1;
  value.scan_resolution = 0.1;
  value.elevator_enabled = true;
  value.automatic_door = false;
  return value;
}

Eigen::Isometry3d pose(const slam::Pose3d& value) {
  auto result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::Quaterniond(value.qw, value.qx, value.qy, value.qz).toRotationMatrix();
  result.translation() = Eigen::Vector3d(value.x, value.y, value.z);
  return result;
}

slam::Pose3d pose(const Eigen::Isometry3d& value) {
  const Eigen::Quaterniond rotation(value.linear());
  return {value.translation().x(), value.translation().y(), value.translation().z(),
          rotation.x(), rotation.y(), rotation.z(), rotation.w()};
}

Eigen::Isometry3d worldPose(const lio::Result& value) {
  auto result = Eigen::Isometry3d::Identity();
  result.linear() = value.state.orientation.toRotationMatrix();
  result.translation() = value.world_position;
  return result;
}

Eigen::Isometry3d odomPose(const lio::Result& value) {
  return value.world_from_odom.inverse() * worldPose(value);
}

Eigen::Isometry3d bodyFromImu() {
  auto mount = Eigen::Isometry3d::Identity();
  mount.linear() = Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  mount.translation() = Eigen::Vector3d(0.11, -0.07, 0.16);
  return mount;
}

Eigen::Isometry3d bodyWorldPose(const lio::Result& value) {
  // The initialized IMU is level and at identity. Mapping starts the body at
  // the origin and removes its initial yaw, so this mount is also the anchor.
  return bodyFromImu() * worldPose(value) * bodyFromImu().inverse();
}

void checkCloudChain(const slam::SlamOutputs& output, const Eigen::Isometry3d& world_body) {
  require(output.registered_cloud_body && output.map_cloud_map &&
              output.registered_cloud_body->frame_id == "body" &&
              output.map_cloud_map->frame_id == "map" &&
              output.registered_cloud_body->points.size() == output.map_cloud_map->points.size(),
          "transport output clouds do not share the declared body/map observation");
  for (std::size_t i = 0; i < output.registered_cloud_body->points.size(); ++i) {
    const auto& body = output.registered_cloud_body->points[i];
    const auto& world = output.map_cloud_map->points[i];
    const Eigen::Vector3d expected = world_body * Eigen::Vector3d(body.x, body.y, body.z);
    require((expected - Eigen::Vector3d(world.x, world.y, world.z)).norm() < 2e-5,
            "published map cloud disagrees with the actual map/odom/body chain");
  }
}

void checkBackendChain(const slam::SlamOutputs& output, const lio::Result& reference) {
  require(output.state == slam::SlamState::Tracking && output.ltlio_lidar_update.accepted &&
              output.odometry_odom_body && output.map_odom_tf,
          "same real sensor stream did not produce matched backend odometry: " + output.reason);
  require(output.map_odom_tf->frame_id == "map" && output.map_odom_tf->child_frame_id == "odom",
          "transport correction was published under the wrong TF frame IDs");
  const auto actual_odom = pose(*output.odometry_odom_body);
  const auto actual_correction = pose(output.map_odom_tf->pose);
  const Eigen::Isometry3d expected_odom = bodyFromImu() * odomPose(reference) * bodyFromImu().inverse();
  const Eigen::Isometry3d expected_correction = bodyFromImu() * reference.world_from_odom * bodyFromImu().inverse();
  require((actual_odom.matrix() - expected_odom.matrix()).norm() < 1e-5,
          "backend exposed corrected world pose as continuous odometry");
  require((actual_correction.matrix() - expected_correction.matrix()).norm() < 1e-5,
          "backend omitted or duplicated the frontend correction in map->odom");
  const Eigen::Isometry3d actual_world = actual_correction * actual_odom;
  require((actual_world.matrix() - bodyWorldPose(reference).matrix()).norm() < 1e-5,
          "backend map->odom->body lost the full posterior world pose");
  checkCloudChain(output, actual_world);
}

std::filesystem::path writeBackendConfig(const std::filesystem::path& directory) {
  std::filesystem::create_directories(directory);
  const auto path = directory / "transport.yaml";
  std::ofstream file(path);
  const Eigen::Matrix3d rotation = bodyFromImu().rotation();
  file << std::setprecision(17)
       << "navigation_body_from_imu_translation: [0.11, -0.07, 0.16]\n"
       << "navigation_body_from_imu_rotation: [";
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      file << (row || col ? ", " : "") << rotation(row, col);
  file << "]\n"
       << "elevator_mode: manual\n"
       << "lidar_filter_num: 1\nlidar_min_range: 0.1\n"
       << "scan_resolution: 0.1\nmap_resolution: 0.05\nimu_init_num: 20\n"
       << "stationary_thresh: 0.0\n"
       << "patch_min_interval_s: 0.0\npatch_min_translation_m: 0.0\npatch_min_rotation_rad: 0.0\n"
       << "relocalization_min_inliers: 100\nrelocalization_min_evaluated_points: 100\n"
       // No loop closure is available within this short physical ride. This
       // isolates frontend correction wiring from a separate SAM map alignment.
       << "sam:\n  min_time_s: 1000000000.0\n";
  require(static_cast<bool>(file), "could not write sensor-equivalent backend configuration");
  return path;
}

std::unique_ptr<slam::ISlamBackend> backend(const std::filesystem::path& path,
                                          slam::SlamMode mode = slam::SlamMode::Mapping,
                                          const std::filesystem::path& saved_map = {}) {
  auto result = slam::makeLtlioBackend();
  slam::SlamConfig settings;
  settings.config_path = path.string();
  require(result && result->configure(settings).ok, "dynamic backend configuration failed");
  if (!saved_map.empty())
    require(result->loadMap(saved_map.string()).ok, "could not load the transported building map");
  require(result->setMode(mode, "").ok, "dynamic backend mode change failed");
  return result;
}

struct SensorStream {
  int imu = 0;
  int frame = 0;
  slam::ISlamBackend* mirror = nullptr;
  bool verify_link = true;
  double nextScanStart() const { return 0.2 + frame * kScanDt; }

  lio::Result step(lio::Frontend& frontend, const std::optional<Ride>& ride = std::nullopt,
                   bool landing = false) {
    const auto cloud = cabin(nextScanStart(), landing);
    ++frame;
    while (imu * kImuDt <= cloud.end_stamp_s + 0.006) {
      lio::ImuSample sample;
      sample.stamp_s = imu++ * kImuDt;
      sample.acceleration = {0.0, 0.0, kGravity + (ride ? ride->acc(sample.stamp_s) : 0.0)};
      frontend.feedImu(sample);
      if (mirror) {
        slam::ImuSample input;
        input.stamp_s = sample.stamp_s;
        input.az = sample.acceleration.z();
        require(mirror->feedImu(input).ok, "backend rejected physical ride IMU");
      }
    }
    frontend.feedCloud(cloud);
    if (mirror) {
      slam::LidarFrame input;
      input.stamp_s = cloud.stamp_s;
      input.frame_id = "lidar";
      for (const auto& point : cloud.points)
        input.points.push_back({point.x, point.y, point.z, point.intensity,
                                static_cast<std::int64_t>(point.offset_ns), point.line, point.tag});
      require(mirror->feedLidar(input).ok && mirror->tick().ok,
              "backend rejected physical cabin observation");
    }
    const auto result = frontend.processNext();
    require(result.has_value(), "covered dynamic cabin scan did not produce a result");
    if (frame > 5)
      require(result->accepted, "dynamic cabin scan failed actual LiDAR matching: " + result->rejection_reason);
    require(result->state.covariance.allFinite() && result->world_position.allFinite() &&
                result->world_velocity.allFinite() && result->world_from_odom.matrix().allFinite(),
            "dynamic transport produced a nonfinite estimate");
    Eigen::Vector3d expected_position = result->state.position;
    if (result->state.in_elevator) expected_position.z() += result->state.transport_height;
    require((result->world_position - expected_position).norm() < 1e-11,
            "published world pose omitted the active transport displacement");
    Eigen::Matrix<double, 3, 21> world_jacobian = Eigen::Matrix<double, 3, 21>::Zero();
    world_jacobian.block<3, 3>(0, 3).setIdentity();
    if (result->state.in_elevator) world_jacobian(2, 18) = 1.0;
    require((result->world_position_covariance - world_jacobian * result->state.covariance *
              world_jacobian.transpose()).norm() < 1e-10,
            "world covariance omitted active height or double-counted its folded uncertainty");
    if (mirror && verify_link && result->accepted) checkBackendChain(mirror->outputs(), *result);
    return *result;
  }
};

lio::Result initialize(lio::Frontend& frontend, SensorStream& stream, bool landing = false) {
  lio::Result result;
  for (int i = 0; i < 15; ++i) result = stream.step(frontend, std::nullopt, landing);
  require(result.accepted && !result.state.in_elevator && result.elevator.entries == 0,
          "dynamic-ride fixture did not establish stationary matching first");
  return result;
}

double continuityResidual(const lio::Result& before, const lio::Result& after) {
  const double dt = after.scan_end_s - before.scan_end_s;
  const Eigen::Vector3d velocity_before = before.world_from_odom.linear().transpose() * before.world_velocity;
  const Eigen::Vector3d velocity_after = after.world_from_odom.linear().transpose() * after.world_velocity;
  const Eigen::Vector3d integrated = odomPose(before).translation() +
      0.5 * (velocity_before + velocity_after) * dt;
  return (odomPose(after).translation() - integrated).norm();
}

void checkManualExitRetainsRealMotion() {
  lio::Frontend frontend(config());
  SensorStream stream;
  initialize(frontend, stream);
  const Ride ride{stream.nextScanStart() + 0.5, 1.0};
  frontend.requestElevator(true);
  lio::Result before;
  do {
    before = stream.step(frontend, ride);
  } while (before.scan_end_s < ride.start + Ride::ramp + 0.15);
  require(before.state.in_elevator && before.elevator.entries == 1 &&
              !before.elevator.stop_confirmed && before.world_velocity.z() > 0.35 &&
              before.state.transport_velocity > 0.35,
          "manual-exit fixture never estimated genuine nonzero transport motion");
  frontend.requestElevator(false);
  const auto after = stream.step(frontend, ride);
  require(!after.state.in_elevator && after.elevator.exits == 1,
          "manual exit did not commit at the matched scan");
  require(after.state.transport_height == 0.0 && after.state.transport_velocity == 0.0 &&
              after.state.transport_acceleration == 0.0,
          "manual exit retained transport coordinates after the full fold");
  require(after.world_velocity.z() > 0.35 &&
              std::abs(after.world_velocity.z() - before.world_velocity.z()) < 0.15,
          "manual exit without stop evidence erased the measured world velocity");
  require(continuityResidual(before, after) < 0.025,
          "manual coordinate exit added a discontinuous odometry correction");
  require((after.world_from_odom.matrix() - before.world_from_odom.matrix()).norm() < 1e-11,
          "manual moving exit fabricated a stop correction without stop evidence");
  std::cout << "manual moving exit: world velocity " << before.world_velocity.z()
            << " -> " << after.world_velocity.z()
            << ", odometry residual " << continuityResidual(before, after) << " m\n";
}

Eigen::Vector3d markerCenter(const slam::Cloud& cloud) {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0;
  for (const auto& point : cloud.points) {
    if (point.intensity < 232.0F) continue;
    sum += Eigen::Vector3d(point.x, point.y, point.z);
    ++count;
  }
  require(count >= 100, "new landing floor was not retained in the real cloud/map");
  return sum / static_cast<double>(count);
}

void checkSavedPose(const std::filesystem::path& path, bool patch_index,
                    const Eigen::Isometry3d& expected) {
  std::ifstream file(path);
  require(static_cast<bool>(file), "saved posterior pose file is missing: " + path.string());
  std::string line, last;
  while (std::getline(file, line)) if (!line.empty()) last = line;
  std::istringstream row(last);
  std::string key;
  slam::Pose3d value;
  row >> key >> value.x >> value.y >> value.z;
  if (patch_index) row >> value.qw >> value.qx >> value.qy >> value.qz;
  else row >> value.qx >> value.qy >> value.qz >> value.qw;
  require(static_cast<bool>(row), "saved posterior pose record is malformed");
  require((pose(value).matrix() - expected.matrix()).norm() < 1e-8,
          "saved trajectory/patch pose used continuous odometry instead of the posterior map pose");
}

void checkLandingPersistence(slam::ISlamBackend& mapped, lio::Frontend& frontend,
                             SensorStream& stream, const Ride& ride,
                             const std::filesystem::path& directory,
                             const std::filesystem::path& config_path) {
  const auto landing = stream.step(frontend, ride, true);
  require(!landing.state.in_elevator, "landing observation was still part of moving transport");
  require(landing.state.transport_height == 0.0 && ride.direction * landing.world_position.z() > 1.0,
          "landing representation did not retain the completed trip in ordinary world position");
  const auto output = mapped.outputs();
  const Eigen::Vector3d body_center = markerCenter(*output.registered_cloud_body);
  const Eigen::Isometry3d expected_pose = bodyWorldPose(landing);
  const Eigen::Vector3d expected_center = expected_pose * body_center;
  const Eigen::Vector3d wrong_odom_center = pose(*output.odometry_odom_body) * body_center;
  require((expected_center - wrong_odom_center).norm() > 1e-4,
          "ride did not exercise a measurable posterior/continuous-odometry difference");
  const auto map_path = directory / "map.pcd";
  const auto saved = mapped.saveMap(map_path.string());
  require(saved.ok, "transport map save failed: " + saved.message);
  const auto stored = mapped.outputs().saved_map_cloud_map;
  require(stored && std::filesystem::exists(map_path), "saved transport map is absent");
  require((markerCenter(*stored) - expected_center).norm() < 2e-4,
          "saved landing geometry discarded the posterior map-height correction");
  for (const auto* name : {"trajectory.txt", "trajectory.raw.txt"})
    checkSavedPose(directory / name, false, expected_pose);
  for (const auto* name : {"poses.txt", "poses.raw.txt"})
    checkSavedPose(directory / name, true, expected_pose);

  auto localized = backend(config_path, slam::SlamMode::Localization, map_path);
  lio::Frontend local_reference(config());
  SensorStream local_stream;
  local_stream.mirror = localized.get();
  local_stream.verify_link = false;
  initialize(local_reference, local_stream, true);
  const auto before = localized->outputs();
  require(before.odometry_odom_body && before.ltlio_lidar_update.accepted && !before.map_odom_tf,
          "reloaded map did not preserve unaligned local matching before real registration");
  const auto aligned = localized->relocalize(pose(expected_pose));
  require(aligned.ok, "saved landing seeded ICP failed: " + aligned.message);
  const auto after = localized->outputs();
  require(after.odometry_odom_body && after.map_odom_tf && after.relocalization_refine_converged &&
              after.relocalization_refine_inliers >= 100,
          "landing relocalization did not produce actual geometric alignment evidence");
  require((pose(*after.odometry_odom_body).matrix() - pose(*before.odometry_odom_body).matrix()).norm() < 1e-12,
          "landing relocalization rewrote continuous local odometry");
  const Eigen::Isometry3d recovered = pose(after.map_odom_tf->pose) * pose(*after.odometry_odom_body);
  require((recovered.translation() - expected_pose.translation()).norm() < 0.01 &&
              Eigen::Quaterniond(recovered.linear()).angularDistance(Eigen::Quaterniond(expected_pose.linear())) < 0.01,
          "reloaded landing map aligned into the wrong posterior floor/frame");
  checkCloudChain(after, recovered);

  const slam::Pose3d accepted_map_odom = after.map_odom_tf->pose;
  local_reference.requestElevator(true);
  require(localized->setElevator(true).ok,
          "localization backend rejected a transport entry request");
  const auto active = local_stream.step(local_reference, std::nullopt, true);
  require(active.state.in_elevator && localized->outputs().elevator.active,
          "localization transport fixture did not enter the cabin state");
  const auto during_transport = localized->relocalize(pose(expected_pose));
  require(!during_transport.ok && during_transport.message == "waiting_for_elevator_exit",
          "synchronous relocalization consumed a cabin scan");
  const auto async_during_transport = localized->startRelocalizeAsync(
      pose(expected_pose), slam::RelocalizationSearch::Automatic);
  require(!async_during_transport.ok &&
              async_during_transport.message == "waiting_for_elevator_exit",
          "asynchronous relocalization consumed a cabin scan");
  const auto waiting = localized->outputs();
  require(waiting.state == slam::SlamState::Tracking && waiting.odometry_odom_body &&
              waiting.map_odom_tf && waiting.relocalization_state == "waiting" &&
              waiting.last_relocalization_message == "waiting_for_elevator_exit",
          "transport wait discarded tracking or hid its relocalization reason");
  require((pose(waiting.map_odom_tf->pose).matrix() - pose(accepted_map_odom).matrix()).norm() < 1e-12,
          "rejected cabin relocalization changed the accepted map alignment");

  local_reference.requestElevator(false);
  require(localized->setElevator(false).ok,
          "localization backend rejected a transport exit request");
  const auto exited = local_stream.step(local_reference, std::nullopt, true);
  const auto exited_output = localized->outputs();
  require(!exited.state.in_elevator && !exited_output.elevator.active,
          "localization transport fixture did not commit its exit");

  const auto exit_scan = localized->startRelocalizeAsync(
      pose(expected_pose), slam::RelocalizationSearch::Automatic);
  require(!exit_scan.ok && exit_scan.message == "waiting_for_scan",
          "relocalization reused the scan that committed transport exit");
  const auto fresh_building_scan = local_stream.step(local_reference, std::nullopt, true);
  require(!fresh_building_scan.state.in_elevator &&
              localized->outputs().observation_sequence > exited_output.observation_sequence,
          "localization fixture did not publish a fresh post-transport scan");

  const auto started = localized->startRelocalizeAsync(
      pose(expected_pose), slam::RelocalizationSearch::Automatic);
  require(started.ok, "building-scan async relocalization did not start: " + started.message);
  std::optional<slam::Status> recovered_async;
  const auto recovery_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!recovered_async && std::chrono::steady_clock::now() < recovery_deadline) {
    recovered_async = localized->pollRelocalizeAsync();
    if (!recovered_async) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(recovered_async.has_value() && recovered_async->ok,
          "fresh post-transport building scan did not restore async map matching");
  const auto recovered_output = localized->outputs();
  require(recovered_output.state == slam::SlamState::Tracking &&
              recovered_output.map_odom_tf &&
              recovered_output.relocalization_state == "completed",
          "fresh post-transport map match did not return to Tracking");
  const slam::Pose3d alignment_before_cross = recovered_output.map_odom_tf->pose;

  const auto crossing = localized->startRelocalizeAsync(
      pose(expected_pose), slam::RelocalizationSearch::Automatic);
  require(crossing.ok, "cross-trip async relocalization did not start: " + crossing.message);
  local_reference.requestElevator(true);
  require(localized->setElevator(true).ok,
          "localization backend rejected entry across async relocalization");
  const auto crossed_active = local_stream.step(local_reference, std::nullopt, true);
  require(crossed_active.state.in_elevator && localized->outputs().elevator.active,
          "async relocalization fixture did not cross into transport");
  local_reference.requestElevator(false);
  require(localized->setElevator(false).ok,
          "localization backend rejected exit across async relocalization");
  const auto crossed_exit = local_stream.step(local_reference, std::nullopt, true);
  require(!crossed_exit.state.in_elevator && !localized->outputs().elevator.active,
          "async relocalization fixture did not finish the crossed trip");

  std::optional<slam::Status> stale;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!stale && std::chrono::steady_clock::now() < deadline) {
    stale = localized->pollRelocalizeAsync();
    if (!stale) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(stale.has_value() && !stale->ok &&
              stale->message == "async_relocalization_transport_changed",
          "async relocalization committed a result across a transport trip");
  const auto after_stale = localized->outputs();
  require(after_stale.state == slam::SlamState::Tracking && after_stale.map_odom_tf &&
              after_stale.relocalization_state == "waiting" &&
              after_stale.last_relocalization_message ==
                  "async_relocalization_transport_changed",
          "cross-trip async rejection discarded tracking or its wait reason");
  require((pose(after_stale.map_odom_tf->pose).matrix() -
           pose(alignment_before_cross).matrix()).norm() < 1e-12,
          "cross-trip async result changed the accepted map alignment");
}

void checkCompleteRide(double direction, const std::filesystem::path& directory) {
  lio::Frontend frontend(config());
  const auto config_path = writeBackendConfig(directory);
  auto mapped = backend(config_path);
  SensorStream stream;
  stream.mirror = mapped.get();
  const auto initial = initialize(frontend, stream);
  auto backend_previous = mapped->outputs();
  const auto source_epoch = backend_previous.source_epoch;
  const Ride ride{stream.nextScanStart() + 0.5, direction};
  frontend.requestElevator(true);
  require(mapped->setElevator(true).ok, "backend rejected elevator entry after real initialization");
  std::array<bool, 5> phases{};
  bool confirmed = false;
  int max_stop_updates = 0;
  double peak_transport_speed = 0.0;
  double largest_stop_residual = 0.0;
  double largest_height_error = 0.0;
  auto previous = initial;
  lio::Result result;
  do {
    result = stream.step(frontend, ride);
    const auto backend_current = mapped->outputs();
    require(backend_current.source_epoch == source_epoch,
            "stop correction incorrectly reset the map/odometry source epoch");
    const Eigen::Isometry3d correction_before = pose(backend_previous.map_odom_tf->pose);
    const Eigen::Isometry3d correction_after = pose(backend_current.map_odom_tf->pose);
    const bool large_correction =
        (correction_after.translation() - correction_before.translation()).norm() > 0.05 ||
        Eigen::Quaterniond(correction_after.linear()).angularDistance(
            Eigen::Quaterniond(correction_before.linear())) > 3.14159265358979323846 / 180.0;
    if (large_correction)
      require(backend_current.map_frame_jump_sequence > backend_previous.map_frame_jump_sequence,
              "posterior frame correction was not announced to map consumers");
    else
      require(backend_current.map_frame_jump_sequence == backend_previous.map_frame_jump_sequence,
              "small posterior correction unnecessarily invalidated map consumers");
    backend_previous = backend_current;
    require(result.elevator.phase >= 0 && result.elevator.phase < 5,
            "transport state machine reported an unknown phase");
    phases[result.elevator.phase] = true;
    confirmed = confirmed || result.elevator.stop_confirmed;
    max_stop_updates = std::max(max_stop_updates, result.elevator.stop_updates);
    peak_transport_speed = std::max(peak_transport_speed,
        direction * result.state.transport_velocity);
    largest_height_error = std::max(largest_height_error,
        std::abs(result.world_position.z() - initial.world_position.z() - ride.height(result.scan_end_s)));
    const bool exit_commit = previous.state.in_elevator && !result.state.in_elevator;
    if (result.elevator.stop_confirmed || previous.elevator.stop_confirmed ||
        result.elevator.stop_updates > 0 || previous.elevator.stop_updates > 0 || exit_commit)
      largest_stop_residual = std::max(largest_stop_residual, continuityResidual(previous, result));
    previous = result;
  } while (result.scan_end_s < ride.finish() + 5.0);
  require(std::all_of(phases.begin(), phases.end(), [](bool seen) { return seen; }),
          "physical ride did not traverse idle/accelerate/cruise/decelerate/stop-check");
  require(confirmed && max_stop_updates >= 2,
          "physical stop never reached confirmation and repeated matched stop updates");
  require(result.elevator.entries == 1 && result.elevator.exits == 1 && !result.state.in_elevator,
          "physical ride did not finish one automatic exit");
  require(peak_transport_speed > 0.5,
          "constant cabin geometry hid the actual elevator motion entirely");
  require(std::abs(result.world_velocity.z()) < 0.1,
          "completed stopped ride retained substantial world velocity");
  require(largest_stop_residual < 0.025,
          "confirmed stop or exit introduced an extra continuous-odometry jump");
  const double displacement = result.world_position.z() - initial.world_position.z();
  const double error = std::abs(displacement - ride.totalHeight());
  // This broad bound checks genuine direction/transport completion. Numerical
  // accuracy is separately reported, rather than calling mode transitions a
  // calibrated height-accuracy result or tuning the bound after seeing a run.
  require(error < 0.2 * std::abs(ride.totalHeight()),
          "completed ride did not recover the physical transport displacement");
  std::cout << (direction > 0 ? "ascending" : "descending")
            << " ride: truth=" << ride.totalHeight() << " m, estimated=" << displacement
            << " m, final_error=" << error << " m, largest_error=" << largest_height_error
            << " m, peak_transport_speed=" << peak_transport_speed
            << " m/s, stop_odom_residual=" << largest_stop_residual << " m\n";
  checkLandingPersistence(*mapped, frontend, stream, ride, directory, config_path);
}
} // namespace

int main() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("lingtu-transport-motion-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    checkManualExitRetainsRealMotion();
    checkCompleteRide(1.0, directory / "ascending");
    checkCompleteRide(-1.0, directory / "descending");
    std::filesystem::remove_all(directory);
    std::cout << "LTLIO real-IMU up/down transport, continuous odom/TF, map save/reload and landing ICP passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\nTest files: " << directory << '\n';
    return 1;
  }
}
