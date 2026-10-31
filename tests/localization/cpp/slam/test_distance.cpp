#ifdef NDEBUG
#undef NDEBUG
#endif
#include "distance.hpp"

#include <cassert>
#include <limits>

using namespace lingtu::slam;

int main() {
  TravelDistance distance;
  assert(!distance.meters());
  SlamOutputs out;
  out.source_epoch = 1;
  out.stamp_s = 1.0;
  out.odometry_odom_body = Pose3d{};
  distance.update(out);
  assert(*distance.meters() == 0.0);

  out.stamp_s = 1.1;
  out.odometry_odom_body = Pose3d{0.3, 0.4, 0.5};
  distance.update(out);
  assert(std::abs(*distance.meters() - 0.5) < 1e-12);
  distance.update(out);  // A repeated snapshot contributes nothing.
  assert(std::abs(*distance.meters() - 0.5) < 1e-12);

  // Global corrections, including small loop closures, are not travel.
  out.stamp_s = 1.2;
  out.map_odom_tf = Transform3d{"map", "odom", {0.4, 0, 0}};
  distance.update(out);
  out.stamp_s = 1.3;
  out.map_odom_tf = Transform3d{"map", "odom", {10, -2, 3}};
  distance.update(out);
  assert(std::abs(*distance.meters() - 0.5) < 1e-12);

  out.stamp_s = 1.4;
  out.odometry_odom_body = Pose3d{0.3, 0.9, 0.5};
  distance.update(out);
  assert(std::abs(*distance.meters() - 1.0) < 1e-12);

  // Discontinuities reanchor without inventing a connecting journey.
  out.source_epoch = 2;
  out.odometry_odom_body = Pose3d{100, 100, 0};
  distance.update(out);
  out.stamp_s = 5.0;
  out.odometry_odom_body = Pose3d{200, 200, 0};
  distance.update(out);
  out.stamp_s = 2.0;
  out.odometry_odom_body = Pose3d{300, 300, 0};
  distance.update(out);
  assert(std::abs(*distance.meters() - 1.0) < 1e-12);

  out.odometry_odom_body.reset();
  distance.update(out);
  assert(!distance.meters());
  out.stamp_s = 2.1;
  out.odometry_odom_body = Pose3d{};
  distance.update(out);
  assert(std::abs(*distance.meters() - 1.0) < 1e-12);
  out.stamp_s = 2.2;
  out.odometry_odom_body->x = std::numeric_limits<double>::quiet_NaN();
  distance.update(out);
  assert(!distance.meters());

  TravelDistance restarted;
  out.odometry_odom_body = Pose3d{};
  restarted.update(out);
  assert(*restarted.meters() == 0.0);

  // Stationary SLAM jitter must not become a path merely because it is sampled often.
  TravelDistance stationary;
  out = SlamOutputs{};
  out.source_epoch = 1;
  out.stamp_s = 1.0;
  out.odometry_odom_body = Pose3d{};
  stationary.update(out);
  for (int i = 1; i <= 2000; ++i) {
    out.stamp_s = 1.0 + i * 0.1;
    out.odometry_odom_body = Pose3d{
        0.008 * std::sin(i * 0.7), 0.006 * std::cos(i * 0.9), 0.0};
    stationary.update(out);
  }
  assert(*stationary.meters() == 0.0);

  // Keep small increments against one anchor, so slow travel is not discarded.
  TravelDistance slow;
  out.stamp_s = 1.0;
  out.odometry_odom_body = Pose3d{};
  slow.update(out);
  for (int i = 1; i <= 1000; ++i) {
    out.stamp_s = 1.0 + i * 0.02;
    out.odometry_odom_body->x = i * 0.001;
    slow.update(out);
  }
  assert(*slow.meters() > 0.95 && *slow.meters() <= 1.0);
  for (int i = 1; i <= 1000; ++i) {
    out.stamp_s = 21.0 + i * 0.02;
    out.odometry_odom_body->x = 1.0 - i * 0.001;
    slow.update(out);
  }
  assert(*slow.meters() > 1.9 && *slow.meters() <= 2.0);

  // Heading changes alone add nothing; the total remains a planar travel estimate.
  const double before_rotation = *slow.meters();
  for (int i = 1; i <= 100; ++i) {
    out.stamp_s += 0.1;
    out.odometry_odom_body->qz = std::sin(i * 0.01);
    out.odometry_odom_body->qw = std::cos(i * 0.01);
    slow.update(out);
  }
  assert(*slow.meters() == before_rotation);

  // A discontinuity clears an uncommitted segment, without adding a bridge.
  for (int boundary = 0; boundary < 4; ++boundary) {
    TravelDistance interrupted;
    out = SlamOutputs{};
    out.source_epoch = 1;
    out.stamp_s = 1.0;
    out.odometry_odom_body = Pose3d{};
    interrupted.update(out);
    out.stamp_s = 1.1;
    out.odometry_odom_body->x = 0.04;
    interrupted.update(out);
    assert(*interrupted.meters() == 0.0);
    if (boundary == 0) out.stamp_s += 3.0;
    if (boundary == 1) out.source_epoch += 1;
    if (boundary == 2) out.stamp_s -= 1.0;
    if (boundary == 3) {
      out.odometry_odom_body.reset();
      interrupted.update(out);
      assert(!interrupted.meters());
      out.odometry_odom_body = Pose3d{};
    }
    out.odometry_odom_body->x = 10.0;
    interrupted.update(out);
    assert(*interrupted.meters() == 0.0);
    out.stamp_s += 0.1;
    out.odometry_odom_body->x = 10.1;
    interrupted.update(out);
    assert(std::abs(*interrupted.meters() - 0.1) < 1e-12);
  }
}
