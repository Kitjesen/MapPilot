#include "pose_buffer.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double kPi = 3.14159265358979323846;

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

nav_kernel::Pose pose(double x, double yaw) {
  nav_kernel::Pose out;
  out.position.x = x;
  out.yaw = yaw;
  return out;
}

void testInterpolatesPositionAndWrappedYaw() {
  lingtu::nav::endpoint::PoseBuffer buffer;
  buffer.push(1.0, pose(0.0, 170.0 * kPi / 180.0));
  buffer.push(1.2, pose(2.0, -170.0 * kPi / 180.0));
  const auto sample = buffer.sample(1.1, 0.15);
  require(sample.has_value(), "bracketed pose must interpolate");
  require(std::abs(sample->position.x - 1.0) < 1e-6, "position interpolation is wrong");
  require(std::abs(std::abs(sample->yaw) - kPi) < 1e-6, "yaw must use the shortest wrap");
}

void testRejectsUnboundedExtrapolation() {
  lingtu::nav::endpoint::PoseBuffer buffer;
  buffer.push(2.0, pose(2.0, 0.0));
  require(buffer.sample(2.04, 0.05).has_value(), "near pose may be held briefly");
  require(!buffer.sample(2.20, 0.05).has_value(), "stale pose must not be extrapolated");
}

void testAcceptsOutOfOrderSamples() {
  lingtu::nav::endpoint::PoseBuffer buffer;
  buffer.push(3.2, pose(2.0, 0.2));
  buffer.push(3.0, pose(0.0, 0.0));
  const auto sample = buffer.sample(3.1, 0.15);
  require(sample.has_value(), "out-of-order odometry must remain sampleable");
  require(std::abs(sample->position.x - 1.0) < 1e-6, "out-of-order interpolation is wrong");
  require(std::abs(buffer.nearestGap(3.1) - 0.1) < 1e-6, "nearest pose gap is wrong");
}

void testExactSampleDoesNotNeedASecondBracket() {
  lingtu::nav::endpoint::PoseBuffer buffer;
  buffer.push(4.0, pose(0.0, 0.0));
  buffer.push(5.0, pose(1.0, 0.1));
  const auto sample = buffer.sample(5.0, 0.01);
  require(sample.has_value(), "an exact timestamp must not require a nearby older bracket");
  require(std::abs(sample->position.x - 1.0) < 1e-6, "exact timestamp returned wrong pose");
}

}  // namespace

int main() {
  testInterpolatesPositionAndWrappedYaw();
  testRejectsUnboundedExtrapolation();
  testAcceptsOutOfOrderSamples();
  testExactSampleDoesNotNeedASecondBracket();
  std::cout << "test_pose_buffer passed\n";
  return 0;
}
