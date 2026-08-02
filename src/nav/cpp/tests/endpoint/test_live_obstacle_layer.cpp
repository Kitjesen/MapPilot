#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "plan/live_obstacle_layer.hpp"

namespace {

std::vector<float> point(float x, float y, float z, float h) {
  return {x, y, z, h};
}

bool containsNear(const std::vector<float> &xyzh, float x, float y, float eps) {
  const std::size_t n = xyzh.size() / 4;
  for (std::size_t i = 0; i < n; ++i) {
    const float dx = xyzh[i * 4 + 0] - x;
    const float dy = xyzh[i * 4 + 1] - y;
    if (std::hypot(dx, dy) <= eps) {
      return true;
    }
  }
  return false;
}

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testMinHits() {
  lingtu::nav::endpoint::LiveObstacleLayer layer({
      0.10,
      1.0,
      0.0,
      4.0,
      0.0,
      100,
      2,
      false,
  });
  layer.update(point(1.0f, 0.0f, 0.2f, 0.3f), 1.0);
  require(layer.snapshot(100, 1.0).empty(), "single hit must be filtered");
  layer.update(point(1.0f, 0.0f, 0.2f, 0.4f), 1.1);
  require(!layer.snapshot(100, 1.1).empty(), "two hits must become occupied");
}

void testInflation() {
  lingtu::nav::endpoint::LiveObstacleLayer layer({
      0.10,
      1.0,
      0.20,
      4.0,
      0.0,
      100,
      1,
      false,
  });
  layer.update(point(1.0f, 1.0f, 0.2f, 0.4f), 1.0);
  const auto inflated = layer.snapshot(100, 1.0);
  require(inflated.size() / 4 > 1, "inflation must add occupied neighbors");
  require(containsNear(inflated, 1.0f, 1.0f, 0.02f), "inflation must keep source cell");
}

void testRayClearing() {
  lingtu::nav::endpoint::LiveObstacleLayer layer({
      0.10,
      10.0,
      0.0,
      4.0,
      0.0,
      100,
      1,
      true,
  });
  const lingtu::nav::endpoint::SensorOrigin origin{0.0, 0.0, 0.0, true};
  layer.updateFromScan(origin, point(1.0f, 0.0f, 0.0f, 0.4f), 1.0);
  require(containsNear(layer.snapshot(100, 1.0), 1.0f, 0.0f, 0.15f),
          "first endpoint must be occupied");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.1);
  require(containsNear(layer.snapshot(100, 1.1), 1.0f, 0.0f, 0.05f),
          "old endpoint must wait for repeated free evidence");
  layer.updateFromScan(origin, point(2.0f, 0.0f, 0.0f, 0.4f), 1.2);
  const auto after_clear = layer.snapshot(100, 1.2);
  require(!containsNear(after_clear, 1.0f, 0.0f, 0.05f), "old endpoint must be cleared");
  require(containsNear(after_clear, 2.0f, 0.0f, 0.15f), "new endpoint must be occupied");
}

}  // namespace

int main() {
  testMinHits();
  testInflation();
  testRayClearing();
  std::cout << "test_live_obstacle_layer passed\n";
  return 0;
}
