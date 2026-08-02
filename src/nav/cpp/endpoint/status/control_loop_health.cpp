#include "status/control_loop_health.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace lingtu::nav::endpoint {
namespace {

double nearestRank(const std::vector<double> &sorted_values, double quantile) {
  const std::size_t rank = static_cast<std::size_t>(std::ceil(quantile * sorted_values.size()));
  return sorted_values[rank - 1];
}

MetricDistribution summarize(std::vector<double> values) {
  MetricDistribution result;
  if (values.empty()) {
    return result;
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    result.mean += (values[index] - result.mean) / static_cast<double>(index + 1);
  }
  std::sort(values.begin(), values.end());
  result.p50 = nearestRank(values, 0.50);
  result.p95 = nearestRank(values, 0.95);
  result.p99 = nearestRank(values, 0.99);
  result.max = values.back();
  return result;
}

}  // namespace

ControlLoopHealth::ControlLoopHealth(ControlLoopHealthConfig config) : config_(std::move(config)) {
  if (!std::isfinite(config_.period_ms) || config_.period_ms <= 0.0) {
    throw std::invalid_argument("control loop period must be finite and positive");
  }
  if (config_.window_size == 0) {
    throw std::invalid_argument("control loop health window must not be empty");
  }
  if (config_.minimum_samples == 0 || config_.minimum_samples > config_.window_size) {
    throw std::invalid_argument("control loop minimum samples must fit the window");
  }
  if (!std::isfinite(config_.deadline_miss_ratio_limit) ||
      config_.deadline_miss_ratio_limit < 0.0) {
    throw std::invalid_argument("deadline miss ratio limit must be finite and non-negative");
  }
  if (!std::isfinite(config_.p95_utilization_limit) || config_.p95_utilization_limit < 0.0) {
    throw std::invalid_argument("utilization limit must be finite and non-negative");
  }
  if (config_.consecutive_miss_limit == 0) {
    throw std::invalid_argument("consecutive miss limit must be positive");
  }
}

bool ControlLoopHealth::observe(ControlLoopSample sample) {
  if (!std::isfinite(sample.loop_ms) || !std::isfinite(sample.sleep_ms) ||
      !std::isfinite(sample.overrun_ms) || sample.loop_ms < 0.0 || sample.sleep_ms < 0.0 ||
      sample.overrun_ms < 0.0) {
    return false;
  }
  const double work_ms = std::max(0.0, sample.loop_ms - sample.sleep_ms);
  const StoredSample stored{work_ms, sample.loop_ms, sample.overrun_ms, work_ms / config_.period_ms,
                            sample.overrun_ms > 0.0};
  window_.push_back(stored);
  if (window_.size() > config_.window_size) {
    window_.pop_front();
  }
  ++total_samples_;
  if (stored.deadline_miss) {
    ++current_miss_streak_;
    max_miss_streak_ = std::max(max_miss_streak_, current_miss_streak_);
  } else {
    current_miss_streak_ = 0;
  }
  return true;
}

ControlLoopHealthSnapshot ControlLoopHealth::snapshot() const {
  ControlLoopHealthSnapshot result;
  result.period_ms = config_.period_ms;
  result.window_samples = window_.size();
  result.total_samples = total_samples_;
  result.current_miss_streak = current_miss_streak_;
  result.max_miss_streak = max_miss_streak_;

  std::vector<double> loop_values;
  std::vector<double> work_values;
  std::vector<double> overrun_values;
  std::vector<double> utilization_values;
  loop_values.reserve(window_.size());
  work_values.reserve(window_.size());
  overrun_values.reserve(window_.size());
  utilization_values.reserve(window_.size());

  for (const auto &sample : window_) {
    loop_values.push_back(sample.loop_ms);
    work_values.push_back(sample.work_ms);
    overrun_values.push_back(sample.overrun_ms);
    utilization_values.push_back(sample.utilization);
    if (sample.deadline_miss) {
      ++result.deadline_misses;
    }
  }

  result.loop_ms = summarize(std::move(loop_values));
  result.work_ms = summarize(std::move(work_values));
  result.overrun_ms = summarize(std::move(overrun_values));
  const auto utilization = summarize(std::move(utilization_values));
  result.p95_utilization = utilization.p95;
  result.max_utilization = utilization.max;

  if (!window_.empty()) {
    result.deadline_miss_ratio =
        static_cast<double>(result.deadline_misses) / static_cast<double>(window_.size());
  }

  result.ready = window_.size() >= config_.minimum_samples;
  if (!result.ready) {
    return result;
  }

  if (current_miss_streak_ >= config_.consecutive_miss_limit) {
    result.reason = "consecutive_deadline_misses";
  } else if (result.deadline_miss_ratio > config_.deadline_miss_ratio_limit) {
    result.reason = "deadline_miss_ratio_high";
  } else if (result.p95_utilization > config_.p95_utilization_limit) {
    result.reason = "p95_utilization_high";
  } else {
    result.healthy = true;
    result.reason = "healthy";
  }
  return result;
}

}  // namespace lingtu::nav::endpoint
