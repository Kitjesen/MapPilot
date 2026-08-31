#include "native/navigation_fixture_observation.hpp"

#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using lingtu::drivers::lidar::NavigationFixtureObservationState;
using lingtu::drivers::lidar::OdomPrior;

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

template <typename Callable>
void requireThrows(Callable&& callable, const char* message) {
  try {
    callable();
  } catch (const std::exception&) {
    return;
  }
  throw std::runtime_error(message);
}

OdomPrior pose(double x = 1.0) {
  OdomPrior value{};
  value.x = x;
  value.y = 2.0;
  value.z = 0.4;
  value.qw = 1.0;
  return value;
}

}  // namespace

int main() {
  try {
    NavigationFixtureObservationState state;
    requireThrows(
        [&] { static_cast<void>(state.matchScan(10)); },
        "scan without pose was accepted");

    state.recordPose(10, pose());
    requireThrows(
        [&] { static_cast<void>(state.matchScan(11)); },
        "mismatched scan timestamp was accepted");
    const auto first = state.matchScan(10);
    require(first.sequence == 1, "first sequence is not one");
    require(first.reset_epoch == 10, "reset epoch is not bound to first scan");
    require(first.timestamp_ns == 10, "scan timestamp changed");
    require(first.pose.x == 1.0, "pose snapshot changed");

    state.recordPose(20, pose(3.0));
    const auto second = state.matchScan(20);
    require(second.sequence == 2, "sequence did not advance");
    require(second.reset_epoch == first.reset_epoch, "epoch changed within one fixture boot");
    require(second.pose.x == 3.0, "replacement pose was not used");

    OdomPrior invalid = pose();
    invalid.qw = std::numeric_limits<double>::quiet_NaN();
    requireThrows(
        [&] { state.recordPose(30, invalid); },
        "non-finite pose was accepted");
    requireThrows(
        [&] { static_cast<void>(state.matchScan(30)); },
        "invalid pose did not clear previous evidence");

    std::cout << "navigation fixture observation tests passed\n";
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
