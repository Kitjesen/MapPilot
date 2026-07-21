#include "dds_drain_policy.hpp"

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <vector>

namespace {

using lingtu::nav::endpoint::DdsDrainProfile;
using lingtu::nav::endpoint::drainBatches;
using lingtu::nav::endpoint::drainBudget;

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void testDefaultReaderTakesAtMostOneBatch() {
  const auto budget = drainBudget(DdsDrainProfile::kDefault);
  std::size_t calls = 0;
  const std::size_t drained = drainBatches(budget, [&](std::size_t capacity) {
    ++calls;
    require(capacity == 16, "default reader batch capacity must be 16");
    return static_cast<std::ptrdiff_t>(capacity);
  });

  require(budget.max_samples() == 16, "default reader budget must be 16 samples");
  require(calls == 1, "default reader must stop after one full batch");
  require(drained == 16, "default reader must report one full batch");
}

void testTfReaderTakesAtMostEightBatches() {
  const auto budget = drainBudget(DdsDrainProfile::kTransform);
  std::size_t calls = 0;
  const std::size_t drained = drainBatches(budget, [&](std::size_t capacity) {
    ++calls;
    require(capacity == 16, "TF reader batch capacity must remain 16");
    return static_cast<std::ptrdiff_t>(capacity);
  });

  require(budget.max_samples() == 128, "TF reader budget must cover 128 samples");
  require(calls == 8, "TF reader must stop after eight full batches");
  require(drained == 128, "TF reader must report eight full batches");
}

void testShortBatchStopsWithoutPollingAgain() {
  const auto budget = drainBudget(DdsDrainProfile::kTransform);
  const std::vector<std::ptrdiff_t> counts{16, 7, 16};
  std::size_t calls = 0;
  const std::size_t drained = drainBatches(budget, [&](std::size_t) {
    return counts.at(calls++);
  });

  require(calls == 2, "a short batch must end the bounded drain");
  require(drained == 23, "short final batch must be included in the count");
}

void testErrorStopsWithoutPollingAgain() {
  const auto budget = drainBudget(DdsDrainProfile::kTransform);
  std::size_t calls = 0;
  const std::size_t drained = drainBatches(budget, [&](std::size_t) {
    ++calls;
    return static_cast<std::ptrdiff_t>(-1);
  });

  require(calls == 1, "a take error must stop the bounded drain");
  require(drained == 0, "a take error must not count samples");
}

}  // namespace

int main() {
  testDefaultReaderTakesAtMostOneBatch();
  testTfReaderTakesAtMostEightBatches();
  testShortBatchStopsWithoutPollingAgain();
  testErrorStopsWithoutPollingAgain();
  std::cout << "test_dds_drain_policy passed\n";
  return 0;
}
