#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "status/active_inspection_map_cache.hpp"

namespace {

using Cache = lingtu::nav::endpoint::ActiveInspectionMapCache;
using Config = lingtu::nav::endpoint::ActiveInspectionMapCacheConfig;
using Identity = lingtu::nav::endpoint::ActiveInspectionMapIdentity;

double steadySeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

Config quietConfig(double stale_after_s = 5.0) {
  return Config{30.0, stale_after_s};
}

void testConstructionTriggersFirstRead() {
  std::atomic<std::uint64_t> calls{0};
  Cache cache(
      [&]() -> std::optional<Identity> {
        ++calls;
        return Identity{"field", 7};
      },
      quietConfig());

  cache.flush();
  const auto snapshot = cache.snapshot(steadySeconds());
  require(calls.load() == 1, "construction must immediately schedule one read");
  require(snapshot.fresh, "the first completed read must be fresh");
  require(snapshot.reason == "ok", "a fresh identity must report ok");
  require(snapshot.identity.has_value(), "the first read must publish an identity");
  require(snapshot.identity->map_id == "field", "map id must be preserved");
  require(snapshot.identity->version == 7, "map version must be preserved");
  require(snapshot.sequence == 1, "the first identity must start sequence one");
}

void testReaderRunsOutsideCacheLock() {
  std::mutex gate_mutex;
  std::condition_variable gate;
  bool entered = false;
  bool release = false;
  Cache cache(
      [&]() -> std::optional<Identity> {
        std::unique_lock<std::mutex> lock(gate_mutex);
        entered = true;
        gate.notify_all();
        gate.wait(lock, [&]() { return release; });
        return Identity{"field", 1};
      },
      quietConfig());

  {
    std::unique_lock<std::mutex> lock(gate_mutex);
    require(gate.wait_for(lock, std::chrono::seconds(2), [&]() { return entered; }),
            "the initial reader call must start");
  }
  const auto start = std::chrono::steady_clock::now();
  const auto snapshot = cache.snapshot(steadySeconds());
  const auto diagnostics = cache.diagnostics();
  const auto elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

  {
    std::lock_guard<std::mutex> lock(gate_mutex);
    release = true;
  }
  gate.notify_all();
  cache.flush();

  require(elapsed_ms < 200.0, "snapshot must not wait for a blocked reader");
  require(!snapshot.fresh, "an in-flight first read must fail closed");
  require(snapshot.reason == "not_refreshed",
          "an in-flight first read must explain that it has not refreshed");
  require(diagnostics.reading, "diagnostics must expose the in-flight read");
}

void testIdentityChangesAdvanceSequenceOnlyOnce() {
  std::mutex identity_mutex;
  Identity current{"field-a", 1};
  Cache cache(
      [&]() -> std::optional<Identity> {
        std::lock_guard<std::mutex> lock(identity_mutex);
        return current;
      },
      quietConfig());
  cache.flush();

  {
    std::lock_guard<std::mutex> lock(identity_mutex);
    current = Identity{"field-b", 2};
  }
  cache.requestRefresh();
  cache.flush();
  const auto changed = cache.snapshot(steadySeconds());
  require(changed.sequence == 2, "an identity change must advance sequence");
  require(changed.identity.has_value() && changed.identity->map_id == "field-b",
          "the changed identity must be visible atomically");

  cache.requestRefresh();
  cache.flush();
  const auto unchanged = cache.snapshot(steadySeconds());
  require(unchanged.sequence == 2, "refreshing the same identity must not advance sequence");
  require(cache.diagnostics().identity_changes == 2,
          "diagnostics must count initial and changed identities");
}

void testReaderExceptionIsContainedAndCanRecover() {
  std::atomic<bool> fail{true};
  Cache cache(
      [&]() -> std::optional<Identity> {
        if (fail.load())
          throw std::runtime_error("map store unavailable");
        return Identity{"recovered", 3};
      },
      quietConfig());

  cache.flush();
  const auto failed = cache.snapshot(steadySeconds());
  const auto failed_diagnostics = cache.diagnostics();
  require(!failed.fresh, "a failed first read must fail closed");
  require(failed.reason == "reader_failure", "a failed first read must report reader_failure");
  require(failed_diagnostics.failures == 1, "reader failures must be counted");
  require(failed_diagnostics.last_error == "map store unavailable",
          "the last reader error must be retained for diagnostics");

  fail.store(false);
  cache.requestRefresh();
  cache.flush();
  const auto recovered = cache.snapshot(steadySeconds());
  const auto recovered_diagnostics = cache.diagnostics();
  require(recovered.fresh, "a later successful read must recover the cache");
  require(recovered_diagnostics.successes == 1, "the recovery must count as a successful refresh");
  require(recovered_diagnostics.last_error.empty(),
          "a successful refresh must clear the last reader error");
}

void testMissingAndStaleIdentityFailClosed() {
  std::atomic<bool> has_identity{false};
  Cache cache(
      [&]() -> std::optional<Identity> {
        if (!has_identity.load())
          return std::nullopt;
        return Identity{"field", 9};
      },
      quietConfig(0.5));

  cache.flush();
  const auto missing = cache.snapshot(steadySeconds());
  require(!missing.fresh, "a missing active map must fail closed");
  require(missing.reason == "no_active_map", "a successful empty read must report no_active_map");

  has_identity.store(true);
  cache.requestRefresh();
  cache.flush();
  const auto fresh = cache.snapshot(steadySeconds());
  require(fresh.fresh, "a newly refreshed identity must be fresh");
  const auto stale = cache.snapshot(fresh.refreshed_at_s + 0.6);
  require(!stale.fresh, "an expired identity must fail closed");
  require(stale.reason == "stale", "an expired identity must report stale");
  require(stale.identity.has_value(), "a stale snapshot may retain identity only for diagnostics");
}

void testConfiguredIntervalPollsWithoutManualRefresh() {
  std::mutex calls_mutex;
  std::condition_variable calls_ready;
  std::uint64_t calls = 0;
  Cache cache(
      [&]() -> std::optional<Identity> {
        std::lock_guard<std::mutex> lock(calls_mutex);
        ++calls;
        calls_ready.notify_all();
        return Identity{"field", 1};
      },
      Config{0.02, 1.0});

  bool observed_periodic_read = false;
  {
    std::unique_lock<std::mutex> lock(calls_mutex);
    observed_periodic_read =
        calls_ready.wait_for(lock, std::chrono::seconds(2), [&]() { return calls >= 2; });
  }
  cache.stop();

  require(observed_periodic_read,
          "the configured interval must trigger reads without manual refresh");
}

void testStopAndDestructionInterruptLongPollingWait() {
  Cache cache([]() -> std::optional<Identity> { return Identity{"field", 1}; }, quietConfig());
  cache.flush();

  const auto stop_start = std::chrono::steady_clock::now();
  cache.stop();
  cache.stop();
  const auto stop_elapsed_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - stop_start)
          .count();
  const auto snapshot = cache.snapshot(steadySeconds());

  require(stop_elapsed_ms < 500.0,
          "stop must interrupt the polling wait instead of sleeping to deadline");
  require(!cache.diagnostics().running, "stop must join the worker");
  require(!snapshot.fresh, "a stopped cache must fail closed immediately");
  require(snapshot.reason == "stopped", "a stopped cache must report stopped");

  const auto destruction_start = std::chrono::steady_clock::now();
  {
    Cache destructing_cache([]() -> std::optional<Identity> { return Identity{"field", 2}; },
                            quietConfig());
    destructing_cache.flush();
  }
  const auto destruction_elapsed_ms = std::chrono::duration<double, std::milli>(
                                          std::chrono::steady_clock::now() - destruction_start)
                                          .count();
  require(destruction_elapsed_ms < 500.0,
          "destruction must interrupt the polling wait and join promptly");
}

}  // namespace

int main() {
  testConstructionTriggersFirstRead();
  testReaderRunsOutsideCacheLock();
  testIdentityChangesAdvanceSequenceOnlyOnce();
  testReaderExceptionIsContainedAndCanRecover();
  testMissingAndStaleIdentityFailClosed();
  testConfiguredIntervalPollsWithoutManualRefresh();
  testStopAndDestructionInterruptLongPollingWait();
  return 0;
}
