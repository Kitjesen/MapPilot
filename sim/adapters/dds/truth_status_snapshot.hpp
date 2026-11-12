#pragma once

#include "native/navigation_fixture_observation.hpp"

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace lingtu::sim::dds_adapter {

// The explicit navigation fixture publishes world/map truth with identity
// T_map_from_odom. This is a display/evidence bridge, never a SLAM estimate.
inline std::string truthStatusSnapshot(
    std::uint64_t timestamp_ns,
    const lingtu::drivers::lidar::OdomPrior& pose,
    const std::string& runtime_id,
    std::uint64_t source_epoch,
    std::uint64_t sequence) {
  const double stamp_s = static_cast<double>(timestamp_ns) / 1000000000.0;
  const auto velocity = pose.has_velocity != 0
      ? lingtu::drivers::lidar::NavigationFixtureObservationState::bodyVelocity(pose)
      : std::array<double, 3>{0.0, 0.0, 0.0};
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"schema_version\":\"lingtu.slam.status_snapshot.v1\","
      << "\"backend\":\"mujoco_truth\",\"source\":\"mujoco_truth\","
      << "\"runtime_instance_id\":\"" << runtime_id << "\","
      << "\"source_epoch\":" << source_epoch << ",\"observation_sequence\":" << sequence << ','
      << "\"stamp_s\":" << stamp_s << ",\"state\":\"TRACKING\","
      << "\"mode\":\"localization\",\"alive\":true,\"has_odom\":true,"
      << "\"localization_quality\":1,\"saved_map_relocalization_supported\":false,"
      << "\"registered_cloud_frame_id\":\"body\",\"map_cloud_frame_id\":\"map\","
      << "\"map_odom_tf\":{\"valid\":true,\"frame_id\":\"map\",\"child_frame_id\":\"odom\","
      << "\"stamp_s\":" << stamp_s << ",\"tx\":0,\"ty\":0,\"tz\":0,\"qx\":0,\"qy\":0,\"qz\":0,\"qw\":1},"
      << "\"map_odometry\":{\"frame_id\":\"map\",\"child_frame_id\":\"body\",\"pose\":{"
      << "\"x\":" << pose.x << ",\"y\":" << pose.y << ",\"z\":" << pose.z
      << ",\"qx\":" << pose.qx << ",\"qy\":" << pose.qy << ",\"qz\":" << pose.qz << ",\"qw\":" << pose.qw
      << "},\"twist\":{\"vx\":" << velocity[0] << ",\"vy\":" << velocity[1] << ",\"vz\":" << velocity[2]
      << ",\"wx\":" << (pose.has_angular_velocity ? pose.wx : 0.0)
      << ",\"wy\":" << (pose.has_angular_velocity ? pose.wy : 0.0)
      << ",\"wz\":" << (pose.has_angular_velocity ? pose.wz : 0.0) << "}}}";
  return out.str();
}

}  // namespace lingtu::sim::dds_adapter
