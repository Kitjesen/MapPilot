#pragma once

#include "native/module.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace lingtu::drivers::lidar {

struct NavigationFixtureObservation {
  std::uint64_t sequence{0};
  std::uint64_t reset_epoch{0};
  std::uint64_t timestamp_ns{0};
  OdomPrior pose{};
};

// Pairs a registered body-frame cloud with the exact truth pose record that
// immediately preceded it. This state exists only for the explicit MuJoCo
// navigation fixture and must never be used as a field SLAM replacement.
class NavigationFixtureObservationState {
 public:
  // MuJoCo free-joint translation velocity is world-frame; Odometry twist
  // belongs to child_frame_id (body). Keep the prior/scan pose in world-frame.
  static std::array<double, 3> bodyVelocity(const OdomPrior& pose) noexcept {
    const double x = pose.qx, y = pose.qy, z = pose.qz, w = pose.qw;
    const double s = 2.0 / (x * x + y * y + z * z + w * w);
    return {
        (1.0 - s * (y * y + z * z)) * pose.vx + s * (x * y + w * z) * pose.vy + s * (x * z - w * y) * pose.vz,
        s * (x * y - w * z) * pose.vx + (1.0 - s * (x * x + z * z)) * pose.vy + s * (y * z + w * x) * pose.vz,
        s * (x * z + w * y) * pose.vx + s * (y * z - w * x) * pose.vy + (1.0 - s * (x * x + y * y)) * pose.vz};
  }

  // Older captured poses still pair scans, but must not rewind live odometry.
  bool recordPose(std::uint64_t timestamp_ns, const OdomPrior& pose) {
    if (timestamp_ns == 0 || !validPose(pose)) {
      invalidate();
      throw std::invalid_argument("navigation fixture pose is invalid");
    }
    pose_timestamp_ns_ = timestamp_ns;
    pose_ = pose;
    has_pose_ = true;
    const bool publish_live = timestamp_ns > latest_pose_timestamp_ns_;
    if (publish_live)
      latest_pose_timestamp_ns_ = timestamp_ns;
    return publish_live;
  }

  NavigationFixtureObservation matchScan(std::uint64_t timestamp_ns) {
    if (!has_pose_) {
      throw std::runtime_error("navigation fixture scan has no matching pose");
    }
    if (timestamp_ns == 0 || timestamp_ns != pose_timestamp_ns_) {
      throw std::runtime_error("navigation fixture scan timestamp does not match pose");
    }
    if (sequence_ == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("navigation fixture observation sequence exhausted");
    }
    if (reset_epoch_ == 0) {
      reset_epoch_ = timestamp_ns;
    }
    ++sequence_;
    return NavigationFixtureObservation{
        sequence_, reset_epoch_, timestamp_ns, pose_};
  }

  void invalidate() noexcept {
    has_pose_ = false;
    pose_timestamp_ns_ = 0;
    pose_ = OdomPrior{};
  }

 private:
  static bool validPose(const OdomPrior& pose) noexcept {
    const double values[] = {
        pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw};
    for (const double value : values) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    const double norm = pose.qx * pose.qx + pose.qy * pose.qy +
                        pose.qz * pose.qz + pose.qw * pose.qw;
    return std::isfinite(norm) && norm > 1e-12;
  }

  bool has_pose_{false};
  std::uint64_t pose_timestamp_ns_{0};
  std::uint64_t latest_pose_timestamp_ns_{0};
  std::uint64_t reset_epoch_{0};
  std::uint64_t sequence_{0};
  OdomPrior pose_{};
};

}  // namespace lingtu::drivers::lidar
