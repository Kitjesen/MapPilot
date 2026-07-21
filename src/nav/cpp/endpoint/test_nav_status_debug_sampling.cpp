#include "nav_status_debug_sampling.hpp"

#include <stdexcept>
#include <vector>

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void testNearestOriginSamplingWinsOverSourceOrder() {
  const std::vector<std::size_t> candidates{0, 1, 2, 3, 4, 5};
  const std::vector<double> distance_squared{25.0, 4.0, 16.0, 1.0, 9.0, 36.0};
  const auto sampled = lingtu::nav::endpoint::detail::sampleDebugIndices(
      candidates,
      3,
      true,
      [&](std::size_t index) { return distance_squared[index]; });

  require(
      sampled == std::vector<std::size_t>({3, 1, 4}),
      "local-map debug sampling must retain the nearest origin cells");
}

void testMissingOriginKeepsDeterministicUniformFallback() {
  const std::vector<std::size_t> candidates{0, 1, 2, 3, 4, 5};
  const auto sampled = lingtu::nav::endpoint::detail::sampleDebugIndices(
      candidates,
      3,
      false,
      [](std::size_t) { return 0.0; });

  require(
      sampled == std::vector<std::size_t>({0, 2, 5}),
      "missing sensor origin must use the deterministic uniform fallback");
}

}  // namespace

int main() {
  testNearestOriginSamplingWinsOverSourceOrder();
  testMissingOriginKeepsDeterministicUniformFallback();
  return 0;
}
