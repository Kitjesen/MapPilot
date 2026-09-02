#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>
#include <vector>

#include "traversability/latest_slow_terrain_worker.hpp"

namespace {

struct Input {
  std::uint64_t generation{0};
};

struct Result {
  std::uint64_t generation{0};
};

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_latest_slow_terrain_worker failed: %s\n", message);
    std::exit(1);
  }
}

void testSubmitDoesNotBlockAndPendingKeepsNewest() {
  std::mutex mutex;
  std::vector<std::uint64_t> processed;
  std::atomic_bool first_started{false};
  std::atomic_int processed_count{0};

  auto worker = lingtu::nav::endpoint::makeLatestOnlyWorker<Input, Result>(
      [&](Input input) {
        {
          std::lock_guard<std::mutex> lock(mutex);
          processed.push_back(input.generation);
        }
        if (input.generation == 1U) {
          first_started = true;
          std::this_thread::sleep_for(std::chrono::milliseconds(120));
        }
        ++processed_count;
        return Result{input.generation};
      },
      []() {});

  worker.submit(Input{1U});
  for (int attempt = 0; attempt < 100 && !first_started; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  require(first_started, "worker must start the first slow input");

  const auto submit_start = std::chrono::steady_clock::now();
  worker.submit(Input{2U});
  worker.submit(Input{3U});
  const auto submit_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - submit_start)
                             .count();
  require(submit_ms < 30.0, "submitting while the worker is busy must not block");

  for (int attempt = 0; attempt < 300 && processed_count.load() < 2; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  worker.stop();

  std::lock_guard<std::mutex> lock(mutex);
  require(processed.size() == 2U, "busy worker must process first input plus newest pending input");
  require(processed[0] == 1U, "first input should be processed");
  require(processed[1] == 3U, "newest pending input must overwrite older pending input");
}

void testResultRejectionOnlyPreventsGenerationRegression() {
  require(lingtu::nav::endpoint::slowTerrainResultCanUpdate(7U, 6U),
          "newer result must remain consumable after a rolling-grid origin change");
  require(!lingtu::nav::endpoint::slowTerrainResultCanUpdate(6U, 6U),
          "equal-generation result must be rejected as stale");
  require(!lingtu::nav::endpoint::slowTerrainResultCanUpdate(5U, 6U),
          "older result must be rejected as regressing");
}

}  // namespace

int main() {
  testSubmitDoesNotBlockAndPendingKeepsNewest();
  testResultRejectionOnlyPreventsGenerationRegression();
  std::puts("test_latest_slow_terrain_worker passed");
  return 0;
}
