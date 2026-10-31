#ifdef NDEBUG
#undef NDEBUG
#endif
// Test the production JSON and DDS projections from one SLAM snapshot.
#define main lingtu_slam_runtime_entry
#include "cyclone_runtime.cpp"
#undef main

#include <cassert>
#include <iostream>

void checkMapObservationHealth(SlamOutputs out, const Pose3d& mount, bool stale) {
  out.state = lingtu::slam::SlamState::Tracking;
  out.confidence = 0.8;
  out.localization_quality = 0.75;
  RuntimeMapTrackingStatus tracking;
  tracking.enabled = true;
  tracking.successes = 1;
  tracking.consecutive_failures = stale ? 0 : 3;
  tracking.last_success_age_s = stale ? 120.0 : 0.1;
  const auto health = projectedHealth(out, tracking);
  assert(health.state == lingtu::slam::SlamState::Degraded);
  const auto observation = toDdsMapObservation(out, mount, SlamMode::Localization, health);
  assert(observation);
  assert(observation->pose_state == toString(health.state));
  assert(observation->pose_reason == health.reason);
  assert(observation->msg.pose_confidence == 0.0F);
  assert(observation->msg.localization_quality == 0.0F);
  assert(projectedLocalizationQuality(out, health) == 0.0);
  const auto snapshot = statusSnapshotJson(
      out, "ltlio", "localization", {}, tracking, {}, mount, {});
  assert(snapshot.find("\"localization_quality\":0.000000") != std::string::npos);

  const auto mapping_health = projectedHealth(out, {});
  const auto mapping = toDdsMapObservation(out, mount, SlamMode::Mapping, mapping_health);
  assert(mapping);
  assert(mapping->pose_state == "MAPPING");
  assert(std::abs(mapping->msg.pose_confidence - 0.8F) < 1e-6F);
  assert(std::abs(mapping->msg.localization_quality - 0.75F) < 1e-6F);
  assert(projectedLocalizationQuality(out, mapping_health) == 0.75);
}

int main(int argc, char** argv) {
  SlamOutputs out;
  out.stamp_s = 123;
  out.source_epoch = 1;
  out.observation_sequence = 1;
  out.odometry_odom_body = Pose3d{2, 1, 0.5};
  out.state_estimation_at_scan = *out.odometry_odom_body;
  out.map_odom_tf = Transform3d{"map", "odom", {10, -2, 3, 0, 0, std::sqrt(0.5), std::sqrt(0.5)}};
  out.odometry_twist_body = lingtu::slam::BodyTwist{0.5, 0.1, 0.2, 0.3, 0.4, 0.7};
  auto cloud = std::make_shared<Cloud>();
  cloud->stamp_s = out.stamp_s;
  cloud->frame_id = "body";
  out.registered_cloud_body = cloud;
  const Pose3d mount{0.16, 0, 0.12};
  if (argc > 1) {
    const std::string scenario = argv[1];
    assert(scenario == "--health-failures" || scenario == "--health-stale");
    checkMapObservationHealth(out, mount, scenario == "--health-stale");
    return 0;
  }
  checkMapObservationHealth(out, mount, false);
  checkMapObservationHealth(out, mount, true);
  lingtu::slam::TravelDistance distance;
  SlamOutputs previous = out;
  previous.stamp_s -= 0.1;
  previous.odometry_odom_body->x -= 0.3;
  previous.odometry_odom_body->y -= 0.4;
  distance.update(previous);
  distance.update(out);
  auto json = statusSnapshotJson(out, "ltlio", "localization", {}, {}, {}, mount, distance);
  assert(json.find("\"map_odometry\":{\"frame_id\":\"map\"") != std::string::npos);
  assert(json.find("\"vx\":0.500000") != std::string::npos);
  assert(json.find("\"wz\":0.700000") != std::string::npos);
  assert(json.find("\"session_distance_m\":0.500000") != std::string::npos);
  assert(json.find("\"session_distance_source\":\"slam_odom\"") != std::string::npos);
  const auto observation = toDdsMapObservation(out, mount, SlamMode::Localization, projectedHealth(out, {}));
  assert(observation);
  assert(std::abs(observation->msg.map_sensor.translation.x - 9) < 1e-12);
  assert(std::abs(observation->msg.sensor_origin.y - 0.16) < 1e-12);
  assert(std::abs(observation->msg.sensor_origin.z - 3.62) < 1e-12);
  out.elevator.mode = "auto";
  out.elevator.active = true;
  out.elevator.height_m = 3.5;
  out.state = lingtu::slam::SlamState::Tracking;
  out.confidence = 0.8;
  RuntimeMapTrackingStatus tracking;
  tracking.enabled = true;
  tracking.period_s = 5.0;
  tracking.successes = 1;
  tracking.last_success_age_s = 120.0;
  assert(projectedHealth(out, tracking).state == lingtu::slam::SlamState::Tracking);
  assert(isTrackAgainstMapInputWait("waiting_for_elevator_exit"));
  assert(isTrackAgainstMapInputWait("async_relocalization_transport_changed"));
  assert(!isTrackAgainstMapInputWait("alignment_failed"));
  json = statusSnapshotJson(out, "ltlio", "localization", {}, {}, {}, mount, distance);
  assert(json.find("\"elevator\":{\"mode\":\"auto\"") != std::string::npos);
  assert(json.find("\"height_m\":3.500000") != std::string::npos);
  assert(!toDdsMapObservation(out, mount, SlamMode::Mapping, projectedHealth(out, {})));
  assert(out.registered_cloud_body == cloud);
  out.elevator.active = false;
  assert(projectedHealth(out, tracking).state == lingtu::slam::SlamState::Degraded);
  assert(toDdsMapObservation(out, mount, SlamMode::Mapping, projectedHealth(out, {})));
  std::cout << json << '\n';
  out.map_odom_tf.reset();
  json = statusSnapshotJson(out, "ltlio", "localization", {}, {}, {}, mount, distance);
  assert(json.find("\"map_odometry\":null") != std::string::npos);
  assert(json.find("\"map_scan_pose\":null") != std::string::npos);
  assert(!toDdsMapObservation(out, mount, SlamMode::Localization, projectedHealth(out, {})));
}
