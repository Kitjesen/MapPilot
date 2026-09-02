#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "status/control_loop_health.hpp"

namespace {

using lingtu::nav::endpoint::ControlLoopHealth;
using lingtu::nav::endpoint::ControlLoopHealthConfig;
using lingtu::nav::endpoint::ControlLoopSample;

void require(bool condition, const char *message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
void requireNear(double actual, double expected, const char *message) {
  require(std::abs(actual - expected) < 1e-12, message);
}

void requireInvalidConfig(ControlLoopHealthConfig config, const char *message) {
  try {
    ControlLoopHealth health(config);
    (void)health;
  } catch (const std::invalid_argument &) {
    return;
  }
  require(false, message);
}

void observeAll(ControlLoopHealth &health, std::initializer_list<ControlLoopSample> samples) {
  for (const auto &sample : samples) {
    require(health.observe(sample), "a health-rule fixture sample must be accepted");
  }
}

void testStartsInWarmup() {
  ControlLoopHealthConfig defaults;
  ControlLoopHealth health(defaults);

  const auto snapshot = health.snapshot();
  require(defaults.window_size == 600, "the default window must retain 600 samples");
  require(defaults.minimum_samples == 100, "the default warmup must require 100 samples");
  require(defaults.deadline_miss_ratio_limit == 0.05,
          "the default deadline miss ratio limit must be conservative");
  require(defaults.p95_utilization_limit == 0.90,
          "the default p95 utilization limit must be conservative");
  require(defaults.consecutive_miss_limit == 3, "the default streak limit must be three");

  require(!snapshot.ready, "an empty monitor must not be ready");
  require(!snapshot.healthy, "warming up must not be reported as healthy");
  require(snapshot.reason == "warming_up", "an empty monitor must report warming_up");
  require(snapshot.period_ms == 50.0, "the default period must match the 20 Hz nav loop");
  require(snapshot.window_samples == 0, "an empty monitor must have no window samples");
  require(snapshot.total_samples == 0, "an empty monitor must have no accepted samples");
}
void testAcceptsAndSummarizesOneSample() {
  ControlLoopHealthConfig config;
  config.period_ms = 10.0;
  config.window_size = 4;
  config.minimum_samples = 3;
  ControlLoopHealth health(config);

  require(health.observe(ControlLoopSample{8.0, 3.0, 1.0}),
          "a finite non-negative sample must be accepted");
  const auto snapshot = health.snapshot();

  require(!snapshot.ready, "one of three required samples must still be warming up");
  require(snapshot.reason == "warming_up", "accepted warmup samples must retain warming_up");
  require(snapshot.window_samples == 1, "an accepted sample must enter the window");
  require(snapshot.total_samples == 1, "an accepted sample must increment the lifetime count");
  require(snapshot.loop_ms.mean == 8.0, "loop mean must summarize the observed cycle");
  require(snapshot.loop_ms.max == 8.0, "loop maximum must summarize the observed cycle");
  require(snapshot.work_ms.mean == 5.0, "work must exclude the measured sleep time");
  require(snapshot.work_ms.max == 5.0, "work maximum must summarize the window");
  require(snapshot.overrun_ms.mean == 1.0, "overrun mean must summarize the window");
  require(snapshot.overrun_ms.max == 1.0, "overrun maximum must summarize the window");
  require(snapshot.deadline_misses == 1, "positive overrun must count as a deadline miss");
  require(snapshot.deadline_miss_ratio == 1.0, "miss ratio must use the accepted window");
  require(snapshot.current_miss_streak == 1, "a first miss must begin the current streak");
  require(snapshot.max_miss_streak == 1, "a first miss must set the lifetime maximum streak");
  require(snapshot.p95_utilization == 0.5, "utilization must divide work by period");
  require(snapshot.max_utilization == 0.5, "maximum utilization must summarize the window");
}

void testUsesNearestRankPercentilesAndBecomesHealthy() {
  ControlLoopHealthConfig config;
  config.period_ms = 200.0;
  config.window_size = 100;
  config.minimum_samples = 100;
  ControlLoopHealth health(config);

  for (int value = 1; value <= 100; ++value) {
    require(health.observe(ControlLoopSample{static_cast<double>(value + 10), 10.0, 0.0}),
            "every finite sample in the percentile fixture must be accepted");
  }

  const auto snapshot = health.snapshot();

  requireNear(snapshot.work_ms.mean, 50.5, "work mean must cover the whole window");
  requireNear(snapshot.work_ms.p50, 50.0, "p50 must use nearest rank");
  requireNear(snapshot.work_ms.p95, 95.0, "p95 must use nearest rank");
  requireNear(snapshot.work_ms.p99, 99.0, "p99 must use nearest rank");
  requireNear(snapshot.work_ms.max, 100.0, "work max must retain the largest sample");
  requireNear(snapshot.loop_ms.mean, 60.5, "loop mean must cover the whole window");
  requireNear(snapshot.loop_ms.p50, 60.0, "loop p50 must use nearest rank");
  requireNear(snapshot.loop_ms.p95, 105.0, "loop p95 must use nearest rank");
  requireNear(snapshot.loop_ms.p99, 109.0, "loop p99 must use nearest rank");
  requireNear(snapshot.loop_ms.max, 110.0, "loop max must retain the largest sample");
  require(snapshot.overrun_ms.mean == 0.0, "zero overruns must have zero mean");
  require(snapshot.overrun_ms.p50 == 0.0, "zero overruns must have zero p50");
  require(snapshot.overrun_ms.p95 == 0.0, "zero overruns must have zero p95");
  require(snapshot.overrun_ms.p99 == 0.0, "zero overruns must have zero p99");
  require(snapshot.overrun_ms.max == 0.0, "zero overruns must have zero max");
  requireNear(snapshot.p95_utilization, 0.475, "p95 utilization must use work p95");
  requireNear(snapshot.max_utilization, 0.5, "max utilization must use maximum work");
  require(snapshot.deadline_misses == 0, "zero overruns must not miss deadlines");
  require(snapshot.deadline_miss_ratio == 0.0, "a miss-free window must have zero ratio");
  require(snapshot.ready, "minimum_samples accepted samples must make the snapshot ready");
  require(snapshot.healthy, "a ready snapshot within all limits must be healthy");
  require(snapshot.reason == "healthy", "a healthy snapshot must explain itself");
}

void testRejectsInvalidConfiguration() {
  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ControlLoopHealthConfig invalid;

  invalid.period_ms = 0.0;
  requireInvalidConfig(invalid, "zero period must be rejected");
  invalid = {};
  invalid.period_ms = -1.0;
  requireInvalidConfig(invalid, "negative period must be rejected");
  invalid = {};
  invalid.period_ms = infinity;
  requireInvalidConfig(invalid, "infinite period must be rejected");
  invalid = {};
  invalid.period_ms = nan;
  requireInvalidConfig(invalid, "NaN period must be rejected");
  invalid = {};
  invalid.window_size = 0;
  requireInvalidConfig(invalid, "an empty window must be rejected");
  invalid = {};
  invalid.minimum_samples = 0;
  requireInvalidConfig(invalid, "zero minimum_samples must be rejected");
  invalid = {};
  invalid.minimum_samples = invalid.window_size + 1;
  requireInvalidConfig(invalid, "minimum_samples beyond the window must be rejected");
  invalid = {};
  invalid.deadline_miss_ratio_limit = -0.01;
  requireInvalidConfig(invalid, "negative miss ratio limit must be rejected");
  invalid = {};
  invalid.deadline_miss_ratio_limit = nan;
  requireInvalidConfig(invalid, "NaN miss ratio limit must be rejected");
  invalid = {};
  invalid.p95_utilization_limit = -0.01;
  requireInvalidConfig(invalid, "negative utilization limit must be rejected");
  invalid = {};
  invalid.p95_utilization_limit = infinity;
  requireInvalidConfig(invalid, "infinite utilization limit must be rejected");
  invalid = {};
  invalid.consecutive_miss_limit = 0;
  requireInvalidConfig(invalid, "zero consecutive miss limit must be rejected");
}

void testRejectsInvalidSamplesWithoutPollutingState() {
  ControlLoopHealthConfig config;
  config.period_ms = 10.0;
  config.window_size = 4;
  config.minimum_samples = 4;
  ControlLoopHealth health(config);
  require(health.observe(ControlLoopSample{6.0, 1.0, 1.0}), "the baseline sample must be accepted");
  const auto before = health.snapshot();

  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const ControlLoopSample invalid_samples[] = {
      {-1.0, 0.0, 0.0}, {0.0, -1.0, 0.0},     {0.0, 0.0, -1.0},
      {nan, 0.0, 0.0},  {0.0, infinity, 0.0}, {0.0, 0.0, nan},
  };
  for (const auto &sample : invalid_samples) {
    require(!health.observe(sample), "an invalid sample must be rejected");
  }

  const auto after_invalid = health.snapshot();
  require(after_invalid.window_samples == before.window_samples,
          "invalid samples must not change the window");
  require(after_invalid.total_samples == before.total_samples,
          "invalid samples must not change the lifetime count");
  require(after_invalid.deadline_misses == before.deadline_misses,
          "invalid samples must not change miss counts");
  require(after_invalid.current_miss_streak == before.current_miss_streak,
          "invalid samples must not change the current streak");
  require(after_invalid.max_miss_streak == before.max_miss_streak,
          "invalid samples must not change the maximum streak");
  require(after_invalid.loop_ms.mean == before.loop_ms.mean,
          "invalid samples must not contaminate loop statistics");
  require(after_invalid.work_ms.mean == before.work_ms.mean,
          "invalid samples must not contaminate work statistics");
  require(after_invalid.overrun_ms.mean == before.overrun_ms.mean,
          "invalid samples must not contaminate overrun statistics");

  require(health.observe(ControlLoopSample{1.0, 2.0, 0.0}),
          "sleep greater than loop is still finite and non-negative");
  const auto clamped = health.snapshot();
  require(clamped.total_samples == 2, "a later valid sample must still be accepted");
  requireNear(clamped.work_ms.mean, 2.5, "negative derived work must clamp to zero");
  require(clamped.current_miss_streak == 0, "a non-miss must reset the current streak");
  require(clamped.max_miss_streak == 1, "resetting must retain the lifetime max streak");
}

void testHealthReasonsFollowConfiguredPriority() {
  ControlLoopHealthConfig config;
  config.period_ms = 10.0;
  config.window_size = 4;
  config.minimum_samples = 4;
  config.deadline_miss_ratio_limit = 0.25;
  config.p95_utilization_limit = 0.50;
  config.consecutive_miss_limit = 2;

  {
    ControlLoopHealth health(config);
    observeAll(health, {{1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {9.0, 0.0, 1.0}, {9.0, 0.0, 1.0}});
    const auto snapshot = health.snapshot();
    require(snapshot.ready, "the consecutive-miss fixture must be ready");
    require(!snapshot.healthy, "consecutive misses at the limit must be unhealthy");
    require(snapshot.reason == "consecutive_deadline_misses",
            "consecutive misses must take priority over ratio and utilization");
  }

  {
    ControlLoopHealth health(config);
    observeAll(health, {{9.0, 0.0, 1.0}, {9.0, 0.0, 0.0}, {9.0, 0.0, 1.0}, {9.0, 0.0, 0.0}});
    const auto snapshot = health.snapshot();
    require(!snapshot.healthy, "a miss ratio above the limit must be unhealthy");
    require(snapshot.reason == "deadline_miss_ratio_high",
            "miss ratio must take priority over utilization");
  }

  {
    ControlLoopHealth health(config);
    observeAll(health, {{9.0, 0.0, 0.0}, {9.0, 0.0, 0.0}, {9.0, 0.0, 0.0}, {9.0, 0.0, 0.0}});
    const auto snapshot = health.snapshot();
    require(!snapshot.healthy, "p95 utilization above the limit must be unhealthy");
    require(snapshot.reason == "p95_utilization_high",
            "utilization must explain an otherwise healthy deadline window");
  }

  {
    ControlLoopHealth health(config);
    observeAll(health, {{5.0, 0.0, 1.0}, {5.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, {5.0, 0.0, 0.0}});
    const auto snapshot = health.snapshot();
    require(snapshot.healthy, "ratio and utilization equal to their limits must be healthy");
    require(snapshot.reason == "healthy", "an at-limit healthy window must report healthy");
  }
}

void testWindowEvictsOldestSampleButKeepsLifetimeCounters() {
  ControlLoopHealthConfig config;
  config.period_ms = 10.0;
  config.window_size = 3;
  config.minimum_samples = 3;
  config.deadline_miss_ratio_limit = 1.0;
  config.p95_utilization_limit = 1.0;
  config.consecutive_miss_limit = 10;
  ControlLoopHealth health(config);

  observeAll(health, {{10.0, 0.0, 1.0}, {8.0, 0.0, 1.0}, {6.0, 0.0, 0.0}, {4.0, 0.0, 0.0}});
  const auto snapshot = health.snapshot();

  require(snapshot.window_samples == 3, "the window must stay at its configured size");
  require(snapshot.total_samples == 4, "eviction must not reduce the lifetime sample count");
  requireNear(snapshot.work_ms.mean, 6.0, "eviction must remove the oldest work sample");
  requireNear(snapshot.work_ms.p50, 6.0, "rolling p50 must use retained samples only");
  requireNear(snapshot.work_ms.p95, 8.0, "rolling p95 must use retained samples only");
  requireNear(snapshot.work_ms.p99, 8.0, "rolling p99 must use retained samples only");
  requireNear(snapshot.work_ms.max, 8.0, "rolling max must exclude an evicted maximum");
  require(snapshot.deadline_misses == 1, "deadline misses must describe the window");
  requireNear(snapshot.deadline_miss_ratio, 1.0 / 3.0,
              "deadline miss ratio must describe the window");
  require(snapshot.current_miss_streak == 0, "the latest non-miss must reset the current streak");
  require(snapshot.max_miss_streak == 2, "the maximum streak must remain a lifetime counter");
}

void testDefaultWindowAndWarmupSizes() {
  ControlLoopHealth health;
  for (int sample = 0; sample < 99; ++sample) {
    require(health.observe(ControlLoopSample{10.0, 5.0, 0.0}),
            "default warmup samples must be accepted");
  }
  require(!health.snapshot().ready, "the default warmup must require 100 samples");

  require(health.observe(ControlLoopSample{10.0, 5.0, 0.0}),
          "the hundredth default sample must be accepted");
  require(health.snapshot().ready, "the default warmup must finish at 100 samples");

  for (int sample = 0; sample < 501; ++sample) {
    require(health.observe(ControlLoopSample{10.0, 5.0, 0.0}),
            "default rolling-window samples must be accepted");
  }
  const auto snapshot = health.snapshot();
  require(snapshot.window_samples == 600, "the default window must retain 600 samples");
  require(snapshot.total_samples == 601, "the lifetime count must exceed the default window");
}

}  // namespace

int main() {
  testStartsInWarmup();
  testAcceptsAndSummarizesOneSample();
  testUsesNearestRankPercentilesAndBecomesHealthy();
  testRejectsInvalidConfiguration();
  testRejectsInvalidSamplesWithoutPollutingState();
  testHealthReasonsFollowConfiguredPriority();
  testWindowEvictsOldestSampleButKeepsLifetimeCounters();
  testDefaultWindowAndWarmupSizes();
  return 0;
}
