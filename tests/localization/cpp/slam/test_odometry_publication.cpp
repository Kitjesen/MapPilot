// Exercise the actual DDS conversion and Fast-LIO backend without a participant.
#define main lingtu_slam_runtime_entry
#include "cyclone_runtime.cpp"
#undef main

namespace {

void require(bool condition, const char* reason) {
  if (!condition) throw std::runtime_error(reason);
}

void requireNear(double actual, double expected, const char* reason, double tolerance = 1e-9) {
  if (!std::isfinite(actual) || std::abs(actual - expected) > tolerance) {
    std::fprintf(stderr, "actual %.12f expected %.12f\n", actual, expected);
    throw std::runtime_error(reason);
  }
}

std::unique_ptr<ISlamBackend> makeBackend(bool bypass, bool prior_enabled = false) {
  const auto path = std::filesystem::temp_directory_path() / "lingtu_odometry_twist.yaml";
  {
    std::ofstream config(path);
    config << "lidar_filter_num: 1\nimu_init_num: 20\n"
              "navigation_body_from_imu_translation: [0.38, -0.02, 0.10]\n"
              "navigation_body_from_imu_rotation: [-1, 0, 0, 0, 1, 0, 0, 0, -1]\n";
    if (bypass || prior_enabled) config << "odom_prior_enabled: true\n";
    if (bypass) config << "odom_prior_bypass_fastlio: true\n";
  }
  auto backend = makeFastLioBackend();
  SlamConfig config;
  config.config_path = path.string();
  require(backend && backend->configure(config).ok, "backend configure failed");
  std::filesystem::remove(path);
  require(backend->setMode(SlamMode::Mapping, "").ok, "mapping mode failed");
  return backend;
}

LidarFrame scan(double stamp_s) {
  LidarFrame frame;
  frame.stamp_s = stamp_s;
  frame.frame_id = "lidar";
  for (int i = 0; i < 240; ++i) {
    const float a = static_cast<float>((i / 3) % 10 - 5) * 0.2F;
    const float b = static_cast<float>((i / 3) / 10 - 4) * 0.2F;
    PointXYZIT point;
    if (i % 3 == 0) { point.x = 3.0F; point.y = a; point.z = b; }
    else if (i % 3 == 1) { point.x = 2.0F + a; point.y = 2.0F; point.z = b; }
    else { point.x = 2.0F + a; point.y = b; point.z = 1.5F; }
    frame.points.push_back(point);
  }
  return frame;
}

void feedImu(ISlamBackend& backend, double stamp_s, double gz) {
  ImuSample imu;
  imu.stamp_s = stamp_s;
  imu.az = -9.81;
  imu.gz = gz;
  require(backend.feedImu(imu).ok, "IMU feed failed");
}

SlamOutputs tickScan(ISlamBackend& backend, double stamp_s) {
  require(backend.feedLidar(scan(stamp_s)).ok, "scan feed failed");
  require(backend.tick().ok, "scan tick failed");
  return backend.outputs();
}

lingtu_dds_Odometry messageFrom(const SlamOutputs& out) {
  require(out.odometry_odom_body.has_value(), "accepted body pose missing");
  const auto message = toDdsOdom(*out.odometry_odom_body, out.stamp_s,
                                out.odometry_twist_body, "odom", "body");
  require(message.has_value(), "accepted synchronized twist missing");
  require(std::string(message->header.frame_id) == "odom" &&
              std::string(message->child_frame_id) == "body", "Odometry frame contract changed");
  requireNear(stampSeconds(message->header.stamp), out.stamp_s, "Odometry source stamp changed");
  return *message;
}

void testWorldVelocityMustNotBePublishedAsBodyVelocity() {
  auto backend = makeBackend(true);
  OdomSample prior;
  prior.stamp_s = 0.15;
  prior.odom_body.qz = prior.odom_body.qw = std::sqrt(0.5);
  prior.has_velocity = true;
  prior.vy = 0.5;
  require(backend->feedVisualOdom(prior).ok, "world-frame prior rejected");
  feedImu(*backend, 0.10, -0.2);
  feedImu(*backend, 0.20, -9.0);  // Future sample must not leak into the earlier scan.
  const auto out = tickScan(*backend, 0.15);
  const auto message = messageFrom(out);
  requireNear(message.twist.twist.linear.x, 0.5,
              "yaw90 forward velocity was published as body lateral velocity");
  requireNear(message.twist.twist.linear.y, 0.0, "body lateral velocity changed");
  requireNear(message.twist.twist.angular.z, 0.2, "synchronized angular velocity missing");
  requireNear(out.fastlio_velocity_y, 0.5, "filter diagnostic velocity changed frame");

  prior.stamp_s = 0.16;
  require(backend->feedVisualOdom(prior).ok, "second prior rejected");
  const auto missing_imu = tickScan(*backend, 0.16);
  require(missing_imu.odometry_odom_body.has_value(), "missing gyro erased diagnostic pose");
  require(!missing_imu.odometry_twist_body, "missing gyro became a valid zero twist");
  require(!toDdsOdom(*missing_imu.odometry_odom_body, missing_imu.stamp_s,
                     missing_imu.odometry_twist_body, "odom", "body"),
          "missing gyro refreshed navigation Odometry");
  prior.stamp_s = 0.25;
  require(backend->feedVisualOdom(prior).ok, "recovery prior rejected");
  feedImu(*backend, 0.30, -9.0);
  requireNear(messageFrom(tickScan(*backend, 0.25)).twist.twist.angular.z, 9.0,
              "fresh synchronized gyro failed to restore publication");
  require(backend->reset().ok, "reset failed");
  require(!backend->outputs().odometry_twist_body, "reset retained an old twist");
}

void testFullAttitudeConversionAndLegacyPriorContract() {
  auto backend = makeBackend(true);
  OdomSample prior;
  prior.stamp_s = 0.15;
  // yaw=90deg, pitch=30deg, roll=20deg. Full child-frame velocity is required;
  // the planner's later yaw-only projection is a distinct planar approximation.
  prior.odom_body.qx = -0.06162841671621935;
  prior.odom_body.qy = 0.2988362387301198;
  prior.odom_body.qz = 0.6408563820557885;
  prior.odom_body.qw = 0.7044160264027587;
  prior.has_velocity = true;
  prior.vx = 0.1623732907437246;
  prior.vy = 0.42327841642608294;
  prior.vz = -0.06686027700272759;
  require(backend->feedVisualOdom(prior).ok, "tilted prior rejected");
  feedImu(*backend, 0.1, -0.2);
  feedImu(*backend, 0.2, -0.2);
  const auto message = messageFrom(tickScan(*backend, 0.15));
  requireNear(message.twist.twist.linear.x, 0.4, "roll/pitch lost in body forward velocity");
  requireNear(message.twist.twist.linear.y, -0.1, "roll/pitch lost in body lateral velocity");
  requireNear(message.twist.twist.linear.z, 0.2, "roll/pitch lost in body vertical velocity");

  // The separate /odom_prior producer still serializes world-frame velocities.
  lingtu_dds_Odometry legacy{};
  legacy.pose.pose = toDdsPose(prior.odom_body);
  legacy.twist.twist.linear.x = prior.vx;
  legacy.twist.twist.linear.y = prior.vy;
  legacy.twist.twist.linear.z = prior.vz;
  const auto decoded = toOdomSample(legacy);
  requireNear(decoded.vx, prior.vx, "legacy prior world velocity rotated twice");
  requireNear(decoded.vy, prior.vy, "legacy prior world velocity rotated twice");
}

void testNormalFastLioBiasMountingAndBodyOrigin() {
  auto backend = makeBackend(false);
  for (int i = 0; i < 20; ++i) feedImu(*backend, i * 0.01, -0.035);
  feedImu(*backend, 0.20, -0.235);
  const auto initializing = tickScan(*backend, 0.20);
  require(!initializing.odometry_twist_body, "initializing filter fabricated twist");
  for (int i = 21; i <= 31; ++i) feedImu(*backend, i * 0.01, -0.235);
  const auto out = tickScan(*backend, 0.30);
  const auto message = messageFrom(out);
  require(!out.odom_prior_active, "normal Fast-LIO test used odometry bypass");
  requireNear(message.twist.twist.angular.z, 0.2, "gyro bias or mounting rotation omitted", 1e-6);
  // The IMU rotates about its own fixed origin: the body origin therefore has
  // velocity -omega x [0.38,-0.02,0.10], not zero.
  requireNear(out.fastlio_velocity_x, 0.0, "test IMU origin unexpectedly moved", 1e-6);
  requireNear(out.fastlio_velocity_y, 0.0, "test IMU origin unexpectedly moved", 1e-6);
  requireNear(message.twist.twist.linear.x, -0.004, "IMU lever arm x omitted", 1e-6);
  requireNear(message.twist.twist.linear.y, -0.076, "IMU lever arm y omitted", 1e-6);
  requireNear(message.twist.twist.linear.z, 0.0, "body vertical velocity changed", 1e-6);
}

void testPriorWithoutVelocityDoesNotPromoteSanitizedZero() {
  for (const bool invalid_velocity : {false, true}) {
    auto backend = makeBackend(false, true);
    for (int i = 0; i < 20; ++i) feedImu(*backend, i * 0.01, 0.0);
    feedImu(*backend, 0.20, 0.0);
    (void)tickScan(*backend, 0.20);
    for (int i = 21; i <= 31; ++i) {
      ImuSample imu;
      imu.stamp_s = i * 0.01;
      imu.az = -9.81;
      imu.ax = invalid_velocity ? 100.0 : 0.0;
      require(backend->feedImu(imu).ok, "fallback IMU rejected");
    }
    OdomSample prior;
    prior.stamp_s = 0.3;
    prior.has_velocity = false;
    require(backend->feedVisualOdom(prior).ok, "pose-only prior rejected");
    const auto out = tickScan(*backend, 0.30);
    require(out.odometry_odom_body.has_value(), "pose-only prior removed diagnostic pose");
    require(out.odom_prior_active, "pose-only prior fixture did not activate");
    if (invalid_velocity) {
      requireNear(out.fastlio_velocity_x, 0.0, "existing invalid-state sanitization changed");
      require(!out.odometry_twist_body, "sanitized invalid velocity became an observed zero twist");
    } else {
      requireNear(messageFrom(out).twist.twist.linear.x, 0.0,
                  "valid filter velocity lost during pose-only prior fallback", 1e-6);
    }
  }
}

void testOnlineMappingUsesAcceptedBodyScan() {
  auto backend = makeBackend(true);
  OdomSample prior;
  prior.stamp_s = .15;
  prior.odom_body.x = 1.0;
  require(backend->feedVisualOdom(prior).ok, "mapping prior rejected");
  feedImu(*backend, .10, 0);
  feedImu(*backend, .20, 0);
  const auto initial = tickScan(*backend, .15);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  SlamOutputs out;
  do {
    require(backend->tick().ok, "mapping background poll failed");
    out = backend->outputs();
    if (out.global_map_cloud) break;
    require(std::chrono::steady_clock::now() < deadline, "mapping output timed out");
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  } while (true);
  require(out.global_map_keyframes == 1 && out.global_map_registered_keyframes == 1,
          "accepted scan did not reach the resident mapping backend");
  require(out.global_map_revision > 0 && !out.global_map_cloud->points.empty(),
          "resident global map is empty");
  require(out.global_map_cloud->frame_id == "map", "global map display frame changed");
  requireNear(out.odometry_odom_body->x, initial.odometry_odom_body->x,
              "background mapping mutated odometry");
  require(out.source_epoch == initial.source_epoch && !out.map_frame_jump,
          "mapping invented a navigation transform jump");
  require(globalMappingJson(out).find("\"keyframes\":1") != std::string::npos,
          "global mapping status was not serialized");
  require(backend->reset().ok, "mapping reset failed");
  require(!backend->outputs().global_map_cloud, "mapping reset retained previous global map");
}

}  // namespace

int main() {
  try {
    testWorldVelocityMustNotBePublishedAsBodyVelocity();
    testFullAttitudeConversionAndLegacyPriorContract();
    testNormalFastLioBiasMountingAndBodyOrigin();
    testPriorWithoutVelocityDoesNotPromoteSanitizedZero();
    testOnlineMappingUsesAcceptedBodyScan();
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "Odometry publication regression failed: %s\n", error.what());
    return 1;
  }
}
