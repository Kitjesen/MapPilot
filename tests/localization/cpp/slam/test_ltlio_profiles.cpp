#include "slam.hpp"
#include "profile.hpp"
#include "ltlio/frontend.hpp"

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
using namespace lingtu::slam;
constexpr double kGravity = 9.80665;
constexpr double kImuDt = 0.005;
constexpr double kScanDuration = 0.05;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void near(double value, double expected, double tolerance, const std::string& message) {
  require(std::isfinite(value) && std::abs(value - expected) <= tolerance,
          message + ": actual=" + std::to_string(value) +
              " expected=" + std::to_string(expected));
}

Eigen::Matrix3d matrix(const YAML::Node& node) {
  require(node.IsSequence() && node.size() == 9, "profile rotation is not a 3x3 matrix");
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      result(row, col) = node[row * 3 + col].as<double>();
  return result;
}

Eigen::Vector3d vector(const YAML::Node& node) {
  require(node.IsSequence() && node.size() == 3, "profile translation is not a 3-vector");
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

struct Sensor {
  SlamConfig config;
  Eigen::Matrix3d body_from_imu;
  Eigen::Matrix3d body_from_lidar;
  Eigen::Vector3d body_lidar_translation;
  Eigen::Vector3d raw_static_acceleration;
  int initial_samples = 0;
};

Sensor sensor(const std::filesystem::path& profile) {
  const auto yaml = readProfile(profile);
  Sensor value;
  value.config.config_path = profile.string();
  value.body_from_imu = matrix(yaml["navigation_body_from_imu_rotation"]);
  value.body_from_lidar = value.body_from_imu * matrix(yaml["r_il"]);
  value.body_lidar_translation = vector(yaml["navigation_body_from_imu_translation"]) +
      value.body_from_imu * vector(yaml["t_il"]);
  value.raw_static_acceleration = value.body_from_imu.transpose() *
      Eigen::Vector3d(0.0, 0.0, kGravity / yaml["acc_scale"].as<double>());
  value.initial_samples = yaml["imu_init_num"].as<int>();
  return value;
}

Eigen::Vector3d translation(const Pose3d& pose) { return {pose.x, pose.y, pose.z}; }
Eigen::Quaterniond rotation(const Pose3d& pose) { return {pose.qw, pose.qx, pose.qy, pose.qz}; }

LidarFrame room(const Sensor& mount, double stamp) {
  LidarFrame frame;
  frame.stamp_s = stamp;
  frame.frame_id = "lidar";
  // Three separated, asymmetrically placed planes constrain all six pose DOFs.
  // Their dense extent supports the unmodified field >=100-point ICP gate.
  for (int surface = 0; surface < 3; ++surface)
    for (int a = -15; a <= 15; ++a)
      for (int b = -15; b <= 15; ++b) {
        Eigen::Vector3d body;
        if (surface == 0) body = {3.0, a * 0.10, b * 0.10};
        else if (surface == 1) body = {a * 0.10, 4.0, b * 0.10};
        else body = {a * 0.10, b * 0.10, -0.8};
        const Eigen::Vector3d lidar = mount.body_from_lidar.transpose() *
            (body - mount.body_lidar_translation);
        PointXYZIT point;
        point.x = static_cast<float>(lidar.x());
        point.y = static_cast<float>(lidar.y());
        point.z = static_cast<float>(lidar.z());
        point.intensity = static_cast<float>(10 * (surface + 1));
        frame.points.push_back(point);
      }
  for (std::size_t i = 0; i < frame.points.size(); ++i) {
    frame.points[i].offset_time_ns = static_cast<std::int64_t>(
        i * 50'000'000ULL / (frame.points.size() - 1));
    frame.points[i].line = static_cast<std::uint8_t>(i % 4);
  }
  return frame;
}

void feedStaticImu(ISlamBackend& backend, const Sensor& mount, double stamp) {
  ImuSample sample;
  sample.stamp_s = stamp;
  sample.ax = mount.raw_static_acceleration.x();
  sample.ay = mount.raw_static_acceleration.y();
  sample.az = mount.raw_static_acceleration.z();
  require(backend.feedImu(sample).ok, "profile rejected calibrated stationary IMU");
}

SlamOutputs initializeRoom(ISlamBackend& backend, const Sensor& mount,
                           SlamState expected_state = SlamState::Tracking) {
  int imu_index = 0;
  const double first = mount.initial_samples * kImuDt + 0.1;
  SlamOutputs result;
  for (int frame = 0; frame < 14; ++frame) {
    const double stamp = first + frame * 0.06;
    while (imu_index * kImuDt <= stamp + kScanDuration + 0.006)
      feedStaticImu(backend, mount, imu_index++ * kImuDt);
    require(backend.feedLidar(room(mount, stamp)).ok, "profile rejected raw room cloud");
    require(backend.tick().ok, "profile processing failed");
    result = backend.outputs();
  }
  require(result.state == expected_state && result.ltlio_lidar_update.accepted &&
              result.odometry_odom_body && result.registered_cloud_body && result.map_cloud_map,
          "profile did not reach expected state with real matched LiDAR: " + result.reason);
  require(!result.odom_prior_active && !result.odom_prior_enabled,
          "field-profile fixture accidentally used external/truth odometry");
  return result;
}

void onRoom(const Eigen::Vector3d& point, const std::string& message) {
  const double residual = std::min({std::abs(point.x() - 3.0),
                                    std::abs(point.y() - 4.0),
                                    std::abs(point.z() + 0.8)});
  // PCL clouds and repeated rigid transforms use float coordinates. 0.2 mm
  // is above their rounding scale and below any mounting/calibration error.
  near(residual, 0.0, 2e-4, message);
}

void checkCloudFrames(const SlamOutputs& output, const Sensor& mount) {
  require(output.registered_cloud_body->frame_id == mount.config.body_frame &&
              output.map_cloud_map->frame_id == mount.config.map_frame,
          "profile changed configured cloud frame identifiers");
  require(output.registered_cloud_body->points.size() == output.map_cloud_map->points.size(),
          "profile body/world clouds describe different observations");
  const auto& pose = *output.odometry_odom_body;
  for (std::size_t i = 0; i < output.registered_cloud_body->points.size(); ++i) {
    const auto& body = output.registered_cloud_body->points[i];
    const auto& world = output.map_cloud_map->points[i];
    const Eigen::Vector3d body_point(body.x, body.y, body.z);
    onRoom(body_point, "profile mount was omitted or applied twice in the body cloud");
    const Eigen::Vector3d expected = rotation(pose) * body_point + translation(pose);
    near((expected - Eigen::Vector3d(world.x, world.y, world.z)).norm(), 0.0, 2e-5,
         "profile odometry and registered point coordinates disagree");
  }
}

void checkFieldProfile(const std::filesystem::path& profile,
                       const std::filesystem::path& output_directory) {
  const auto mount = sensor(profile);
  auto backend = makeLtlioBackend();
  require(backend && backend->configure(mount.config).ok,
          "actual robot profile failed to configure: " + profile.string());
  require(backend->setMode(SlamMode::Mapping, "").ok, "field profile mapping mode failed");
  const auto mapped = initializeRoom(*backend, mount);
  near(translation(*mapped.odometry_odom_body).norm(), 0.0, 1e-3,
       "actual robot mount did not initialize the body at odom origin");
  near(rotation(*mapped.odometry_odom_body).angularDistance(Eigen::Quaterniond::Identity()),
       0.0, 1e-3, "actual robot mount left an unexpected body tilt/yaw");
  checkCloudFrames(mapped, mount);

  const auto map_path = output_directory / "map.pcd";
  const auto saved = backend->saveMap(map_path.string());
  require(saved.ok, "actual robot profile could not save its map: " + saved.message);
  const auto stored = backend->outputs().saved_map_cloud_map;
  require(stored && !stored->points.empty() && std::filesystem::exists(map_path),
          "actual robot profile produced no saved cloud");
  for (const auto& point : stored->points)
    onRoom({point.x, point.y, point.z}, "saved field map uses the wrong sensor/body frame");

  auto localized = makeLtlioBackend();
  require(localized && localized->configure(mount.config).ok &&
              localized->loadMap(map_path.string()).ok &&
              localized->setMode(SlamMode::Localization, "").ok,
          "new backend could not reload the actual-profile map");
  const auto before = initializeRoom(*localized, mount, SlamState::Localizing);
  require(!before.map_odom_tf, "saved map was aligned without a measured registration");
  const auto aligned = localized->relocalize(Pose3d{});
  require(aligned.ok, "actual-profile seeded registration failed: " + aligned.message);
  const auto after = localized->outputs();
  require(after.state == SlamState::Tracking && after.map_odom_tf &&
              after.relocalization_refine_converged && after.relocalization_map_body &&
              after.relocalization_refine_inliers >= 100 && after.odometry_odom_body,
          "actual-profile relocalization did not satisfy the unchanged field evidence gate");
  near(translation(*after.relocalization_map_body).norm(), 0.0, 0.005,
       "saved-map registration misplaced the static body");
  near((translation(*after.odometry_odom_body) -
        translation(*before.odometry_odom_body)).norm(), 0.0, 1e-12,
       "global alignment rewrote field-profile local odometry");
  near(rotation(*before.odometry_odom_body).angularDistance(rotation(*after.odometry_odom_body)),
       0.0, 1e-12, "global alignment rewrote field-profile odometry orientation");
  std::cout << profile.string() << ": real matching, map save/reload and seeded ICP passed\n";
}

void checkTruthProfile(const std::filesystem::path& profile) {
  const auto mount = sensor(profile);
  auto backend = makeLtlioBackend();
  require(backend && backend->configure(mount.config).ok &&
              backend->setMode(SlamMode::Mapping, "").ok,
          "actual truth-bypass profile failed to configure");
  OdomSample prior;
  prior.stamp_s = 0.35;
  const Eigen::Quaterniond heading(Eigen::AngleAxisd(0.45, Eigen::Vector3d::UnitZ()));
  prior.odom_body = {1.2, -0.8, 0.4, heading.x(), heading.y(), heading.z(), heading.w()};
  const Eigen::Vector3d world_velocity = heading * Eigen::Vector3d(0.25, 0.1, 0.0);
  prior.has_velocity = true;
  prior.vx = world_velocity.x(); prior.vy = world_velocity.y(); prior.vz = world_velocity.z();
  require(backend->feedVisualOdom(prior).ok, "truth profile rejected typed pose prior");
  feedStaticImu(*backend, mount, 0.25);
  feedStaticImu(*backend, mount, 0.40);
  require(backend->feedLidar(room(mount, 0.30)).ok && backend->tick().ok,
          "truth profile rejected its covered scan");
  const auto output = backend->outputs();
  require(output.state == SlamState::Tracking && output.odom_prior_active &&
              output.reason == "tracking_with_odom_prior_bypass" &&
              !output.ltlio_lidar_update.attempted && output.odometry_odom_body &&
              output.registered_cloud_body && output.map_cloud_map && output.odometry_twist_body,
          "truth profile mixed estimation with the explicit bypass input");
  near((translation(*output.odometry_odom_body) - translation(prior.odom_body)).norm(),
       0.0, 1e-12, "truth profile changed external body pose");
  near(rotation(*output.odometry_odom_body).angularDistance(heading), 0.0, 1e-12,
       "truth profile reapplied a sensor mount to body orientation");
  near(output.odometry_twist_body->vx, 0.25, 1e-12, "truth velocity was not expressed in body");
  near(output.odometry_twist_body->vy, 0.1, 1e-12, "truth lateral velocity frame changed");
  checkCloudFrames(output, mount);
  std::cout << profile.string() << ": explicit truth input/frame contract passed\n";
}

std::filesystem::path noiseProfile(const std::filesystem::path& profile,
                                  const std::filesystem::path& directory,
                                  const std::string& name,
                                  const YAML::Node& densities) {
  auto yaml = readProfile(profile);
  for (const char* key : {"na", "ng", "nba", "nbg"}) yaml.remove(key);
  for (const auto& entry : densities)
    yaml[entry.first.as<std::string>()] = entry.second;
  std::filesystem::create_directories(directory);
  const auto path = directory / (name + ".yaml");
  {
    std::ofstream file(path);
    file << yaml;
    require(file.good(), "could not write calibration test profile");
  }
  return path;
}

double calibratedCovariance(const std::filesystem::path& path) {
  const auto mount = sensor(path);
  auto backend = makeLtlioBackend();
  require(backend && backend->configure(mount.config).ok &&
              backend->setMode(SlamMode::Mapping, "").ok,
          "calibrated IMU profile could not initialize");
  return initializeRoom(*backend, mount).ltlio_pos_cov_trace;
}

void checkCalibratedImuNoise(const std::filesystem::path& profile,
                             const std::filesystem::path& directory,
                             const std::string& selected_key = "") {
  const lingtu::ltlio::Config defaults;
  require(defaults.acc_noise_variance == 1e-4 && defaults.gyro_noise_variance == 1e-5 &&
              defaults.acc_bias_noise_variance == 1e-6 && defaults.gyro_bias_noise_variance == 1e-6,
          "omitted calibration changed the exact upstream process-noise defaults");
  const double baseline = calibratedCovariance(noiseProfile(profile, directory, "default", {}));
  require(std::isfinite(baseline) && baseline > 0.0, "default covariance is invalid");
  YAML::Node official;
  official["na"] = 0.01;
  official["ng"] = std::sqrt(1e-5);
  official["nba"] = 0.001;
  official["nbg"] = 0.001;
  const double explicit_defaults = calibratedCovariance(
      noiseProfile(profile, directory, "official", official));
  near(explicit_defaults, baseline, 1e-13,
       "explicit equivalent densities changed the default covariance");
  for (const char* key : {"na", "ng", "nba", "nbg"}) {
    if (!selected_key.empty() && selected_key != key) continue;
    YAML::Node calibration;
    calibration[key] = 0.1;
    const double calibrated = calibratedCovariance(
        noiseProfile(profile, directory, key, calibration));
    require(std::isfinite(calibrated) && calibrated > 0.0 &&
                std::abs(calibrated - baseline) > 1e-12,
            std::string("calibrated ") + key + " had no effect on propagated covariance");
    std::cout << key << ": default covariance=" << baseline
              << ", calibrated covariance=" << calibrated << '\n';
    // yaml-cpp 0.6 double assignment emits nonstandard "nan"/"inf" scalars.
    // Canonical YAML numbers reach the intended semantic validation on every SDK.
    for (const char* invalid : {"-0.1", ".inf", ".nan"}) {
      calibration[key] = YAML::Load(invalid);
      SlamConfig config;
      config.config_path = noiseProfile(profile, directory, "invalid", calibration).string();
      auto backend = makeLtlioBackend();
      const auto configured = backend->configure(config);
      require(!configured.ok && configured.message == std::string("invalid_imu_noise_density: ") + key,
              std::string("invalid calibrated density was accepted: ") + key);
    }
  }
}

void checkElevatorEntryProfile(const std::filesystem::path& profile,
                              const std::filesystem::path& directory) {
  std::filesystem::create_directories(directory);
  const auto base = readProfile(profile);
  auto write = [&](const std::string& name, const YAML::Node& yaml) {
    const auto path = directory / (name + ".yaml");
    std::ofstream file(path);
    file << yaml;
    require(file.good(), "could not write elevator entry test profile");
    return path;
  };
  for (const char* key : {"range_m", "hold_s", "motion_m"}) {
    for (const char* invalid : {"0.0", ".nan"}) {
      auto yaml = YAML::Clone(base);
      yaml["elevator_mode"] = "auto";
      yaml["elevator_entry"][key] = YAML::Load(invalid);
      SlamConfig config;
      config.config_path = write("invalid", yaml).string();
      auto backend = makeLtlioBackend();
      const auto configured = backend->configure(config);
      require(!configured.ok && configured.message ==
                  "invalid_ltlio_config: Invalid elevator entry configuration",
              std::string("backend ignored invalid elevator_entry.") + key);
    }
  }
  for (const double range : {10.0, 0.5}) {
    auto yaml = YAML::Clone(base);
    yaml["elevator_mode"] = "auto";
    yaml["elevator_entry"]["range_m"] = range;
    yaml["elevator_entry"]["hold_s"] = 0.2;
    yaml["elevator_entry"]["motion_m"] = 0.07;
    const auto mount = sensor(write(range > 1.0 ? "wide" : "narrow", yaml));
    auto backend = makeLtlioBackend();
    require(backend && backend->configure(mount.config).ok &&
                backend->setMode(SlamMode::Mapping, "").ok,
            "nondefault elevator entry profile failed to configure");
    const auto output = initializeRoom(*backend, mount);
    // The same accepted room scans run for less than the default 2 s hold.
    // A 0.2 s hold enters at 10 m, while a 0.5 m range excludes this room.
    require(output.elevator.active == (range > 1.0),
            "backend did not apply elevator_entry range/hold to matched scans");
  }
  std::cout << "elevator_entry: backend YAML validation and accepted-scan behavior passed\n";
}
void checkSharedProfile(const std::filesystem::path& source, const std::filesystem::path& output) {
  std::filesystem::create_directories(output);
  YAML::Node common;
  common["slam"] = readProfile(source);
  common["slam"]["elevator_mode"] = "invalid";
  common["slam"]["elevator_entry"]["hold_s"] = 3.25;
  const auto common_path = output / "common.yaml";
  std::ofstream(common_path) << common;
  YAML::Node profile;
  profile["defaults"] = "common.yaml";
  const auto path = output / "robot.yaml";
  std::ofstream(path) << profile;
  SlamConfig config;
  config.config_path = path.string();
  auto backend = makeLtlioBackend();
  require(backend->configure(config).message == "invalid_elevator_mode",
          "native backend did not read the common algorithm settings");
  profile["elevator_mode"] = "off";
  profile["elevator_entry"]["range_m"] = 4.25;
  std::ofstream(path) << profile;
  require(backend->configure(config).ok, "installation override did not replace common setting");
  near(readProfile(path)["scan_resolution"].as<double>(), 0.25, 1e-12,
       "common scan resolution was not retained with an installation override");
  const auto entry = readProfile(path)["elevator_entry"];
  near(entry["hold_s"].as<double>(), 3.25, 1e-12,
       "partial elevator override discarded the shared hold time");
  near(entry["range_m"].as<double>(), 4.25, 1e-12,
       "installation elevator range did not override the common setting");
}
} // namespace

int main(int argc, char** argv) {
  const auto repository = std::filesystem::path(__FILE__).parent_path().parent_path()
      .parent_path().parent_path().parent_path();
  const auto output = std::filesystem::temp_directory_path() /
      ("lingtu-profiles-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  try {
    const auto go2 = repository / "config/robots/unitree/go2/sensors/mid360_ltlio.yaml";
    checkSharedProfile(go2, output / "shared");
    if (argc > 1) {
      if (std::string(argv[1]) == "--elevator-entry" && argc == 2) {
        checkElevatorEntryProfile(go2, output / "elevator-entry");
        std::filesystem::remove_all(output);
        return 0;
      }
      require(std::string(argv[1]) == "--imu-noise" && argc == 3,
              "expected --imu-noise na|ng|nba|nbg");
      const std::string key = argv[2];
      require(key == "na" || key == "ng" || key == "nba" || key == "nbg",
              "unknown IMU noise key");
      checkCalibratedImuNoise(go2, output / "noise", key);
      std::filesystem::remove_all(output);
      return 0;
    }
    checkElevatorEntryProfile(go2, output / "elevator-entry");
    checkCalibratedImuNoise(go2, output / "noise");
    checkFieldProfile(repository / "config/robots/unitree/go2/sensors/mid360_ltlio.yaml", output / "go2");
    checkFieldProfile(repository / "config/robots/doso/thunder_v4/sensors/mid360_ltlio.yaml", output / "thunder");
    checkTruthProfile(repository / "src/localization/ltlio/config/sim_mid360.yaml");
    std::filesystem::remove_all(output);
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\nTest files: " << output << '\n';
    return 1;
  }
}
