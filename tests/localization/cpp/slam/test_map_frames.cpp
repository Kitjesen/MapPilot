#ifdef NDEBUG
#undef NDEBUG
#endif
#include "frames.hpp"
#include <cassert>
#include <cmath>

int main() {
  using namespace lingtu::slam;
  SlamOutputs out;
  out.odometry_odom_body = Pose3d{2, 1, 0.5, std::sin(0.2), 0, 0, std::cos(0.2)};
  out.state_estimation_at_scan = *out.odometry_odom_body;
  const Pose3d mount{0.16, 0, 0.12};
  assert(!resolveMapFrames(out, mount).body);
  out.map_odom_tf = Transform3d{"map", "odom", {10, -2, 3, 0, 0, std::sqrt(0.5), std::sqrt(0.5)}};
  auto frames = resolveMapFrames(out, mount);
  assert(frames.body && frames.scan_body && frames.scan_lidar);
  assert(std::abs(frames.body->x - 9) < 1e-12);
  assert(std::abs(frames.body->y) < 1e-12);
  assert(std::abs(frames.body->z - 3.5) < 1e-12);
  assert(std::abs(frames.body->qx - std::sqrt(0.5) * std::sin(0.2)) < 1e-12);
  assert(std::abs(frames.body->qy - std::sqrt(0.5) * std::sin(0.2)) < 1e-12);
  // The lidar ray starts at the mounted sensor, including the current body tilt.
  assert(std::abs(frames.scan_lidar->x - (9 + 0.12 * std::sin(0.4))) < 1e-12);
  assert(std::abs(frames.scan_lidar->y - 0.16) < 1e-12);
  assert(std::abs(frames.scan_lidar->z - (3.5 + 0.12 * std::cos(0.4))) < 1e-12);
  out.map_odom_tf->pose.x += 1;
  frames = resolveMapFrames(out, mount);
  assert(std::abs(frames.body->x - 10) < 1e-12);
  assert(std::abs(frames.scan_body->x - 10) < 1e-12);
  out.map_odom_tf.reset();
  frames = resolveMapFrames(out, mount);
  assert(!frames.body && !frames.scan_body && !frames.scan_lidar);
}
