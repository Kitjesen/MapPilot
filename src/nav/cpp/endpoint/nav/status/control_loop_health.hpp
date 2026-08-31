#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

namespace lingtu::nav::endpoint {

struct ControlLoopHealthConfig {
  double period_ms{50.0};
  std::size_t window_size{600};
  std::size_t minimum_samples{100};
  double deadline_miss_ratio_limit{0.05};
  double p95_utilization_limit{0.90};
  std::size_t consecutive_miss_limit{3};
};

struct ControlLoopSample {
  double loop_ms{0.0};
  double sleep_ms{0.0};
  double overrun_ms{0.0};
};

struct MetricDistribution {
  double mean{0.0};
  double p50{0.0};
  double p95{0.0};
  double p99{0.0};
  double max{0.0};
};

// Distributions and deadline counts cover the retained window. Total samples
// and miss streaks are lifetime diagnostics.
struct ControlLoopHealthSnapshot {
  bool ready{false};
  bool healthy{false};
  std::string reason{"warming_up"};
  double period_ms{0.0};
  std::size_t window_samples{0};
  std::uint64_t total_samples{0};
  MetricDistribution loop_ms{};
  MetricDistribution work_ms{};
  MetricDistribution overrun_ms{};
  std::size_t deadline_misses{0};
  double deadline_miss_ratio{0.0};
  std::size_t current_miss_streak{0};
  std::size_t max_miss_streak{0};
  double p95_utilization{0.0};
  double max_utilization{0.0};
};

class ControlLoopHealth {
 public:
  // Throws std::invalid_argument when the configuration violates its invariants.
  explicit ControlLoopHealth(ControlLoopHealthConfig config = {});

  // Returns false without changing state for a non-finite or negative sample.
  // Work is derived as max(0, loop_ms - sleep_ms).
  bool observe(ControlLoopSample sample);
  ControlLoopHealthSnapshot snapshot() const;

 private:
  struct StoredSample {
    double work_ms{0.0};
    double loop_ms{0.0};
    double overrun_ms{0.0};
    double utilization{0.0};
    bool deadline_miss{false};
  };

  ControlLoopHealthConfig config_;
  std::deque<StoredSample> window_;
  std::uint64_t total_samples_{0};
  std::size_t current_miss_streak_{0};
  std::size_t max_miss_streak_{0};
};

}  // namespace lingtu::nav::endpoint
