#include "status/active_inspection_map_cache.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

using SteadyClock = std::chrono::steady_clock;

double steadySeconds() {
  return std::chrono::duration<double>(SteadyClock::now().time_since_epoch()).count();
}

bool sameIdentity(const std::optional<ActiveInspectionMapIdentity> &left,
                  const std::optional<ActiveInspectionMapIdentity> &right) {
  if (left.has_value() != right.has_value())
    return false;
  if (!left.has_value())
    return true;
  return left->map_id == right->map_id && left->version == right->version;
}

SteadyClock::duration pollDuration(double seconds) {
  const auto duration =
      std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(seconds));
  return std::max(duration, SteadyClock::duration{1});
}

}  // namespace

struct ActiveInspectionMapCache::Impl {
  Impl(Reader target_reader, ActiveInspectionMapCacheConfig target_config)
      : reader(std::move(target_reader)), config(target_config) {
    if (!reader) {
      throw std::invalid_argument("active inspection map reader must be callable");
    }
    if (!std::isfinite(config.poll_interval_s) || config.poll_interval_s <= 0.0) {
      throw std::invalid_argument("active inspection map poll interval must be positive");
    }
    if (!std::isfinite(config.stale_after_s) || config.stale_after_s <= 0.0) {
      throw std::invalid_argument("active inspection map stale threshold must be positive");
    }

    poll_interval = pollDuration(config.poll_interval_s);
    stats.running = true;
    requested_generation = 1;
    worker = std::thread([this]() { run(); });
  }

  ~Impl() { stop(); }

  void requestRefresh() {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (stopping || !stats.running)
        return;
      ++requested_generation;
    }
    work_ready.notify_one();
  }

  void flush() {
    std::unique_lock<std::mutex> lock(mutex);
    const std::uint64_t target_generation = requested_generation;
    idle.wait(lock, [this, target_generation]() {
      return !stats.running || (completed_generation >= target_generation && !stats.reading);
    });
  }

  void stop() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex);
    {
      std::lock_guard<std::mutex> lock(mutex);
      stopping = true;
    }
    work_ready.notify_all();
    idle.notify_all();
    if (worker.joinable()) {
      worker.join();
    }
  }

  ActiveInspectionMapSnapshot snapshot(double now_steady_s) const {
    ActiveInspectionMapSnapshot result;
    bool running = false;
    bool has_success = false;
    std::uint64_t failures = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      result.identity = identity;
      result.sequence = sequence;
      result.refreshed_at_s = refreshed_at_s;
      running = stats.running && !stopping;
      has_success = has_successful_refresh;
      failures = stats.failures;
    }

    if (!running) {
      result.reason = "stopped";
      return result;
    }
    if (!has_success) {
      result.reason = failures > 0 ? "reader_failure" : "not_refreshed";
      return result;
    }
    if (!result.identity.has_value()) {
      result.reason = "no_active_map";
      return result;
    }
    if (!std::isfinite(now_steady_s)) {
      result.reason = "invalid_time";
      return result;
    }

    const double age_s = std::max(0.0, now_steady_s - result.refreshed_at_s);
    if (age_s > config.stale_after_s) {
      result.reason = "stale";
      return result;
    }
    result.fresh = true;
    result.reason = "ok";
    return result;
  }

  ActiveInspectionMapCacheDiagnostics diagnostics() const {
    std::lock_guard<std::mutex> lock(mutex);
    return stats;
  }

  void run() noexcept {
    std::unique_lock<std::mutex> lock(mutex);
    auto next_poll = SteadyClock::now();
    while (!stopping) {
      work_ready.wait_until(lock, next_poll, [this]() {
        return stopping || completed_generation < requested_generation;
      });
      if (stopping)
        break;

      const std::uint64_t target_generation = requested_generation;
      stats.reading = true;
      ++stats.attempts;
      lock.unlock();

      std::optional<ActiveInspectionMapIdentity> next_identity;
      std::string error;
      bool succeeded = false;
      try {
        next_identity = reader();
        succeeded = true;
      } catch (const std::exception &exception) {
        error = exception.what();
        if (error.empty())
          error = "reader_exception";
      } catch (...) {
        error = "unknown_reader_exception";
      }
      const double completed_at_s = steadySeconds();

      lock.lock();
      if (succeeded) {
        ++stats.successes;
        stats.last_error.clear();
        has_successful_refresh = true;
        refreshed_at_s = completed_at_s;
        if (!sameIdentity(identity, next_identity)) {
          identity = std::move(next_identity);
          ++sequence;
          ++stats.identity_changes;
        }
      } else {
        ++stats.failures;
        stats.last_error = std::move(error);
      }
      completed_generation = std::max(completed_generation, target_generation);
      stats.reading = false;
      next_poll = SteadyClock::now() + poll_interval;
      idle.notify_all();
    }

    stats.reading = false;
    stats.running = false;
    idle.notify_all();
  }

  Reader reader;
  ActiveInspectionMapCacheConfig config;
  SteadyClock::duration poll_interval;
  mutable std::mutex mutex;
  std::mutex lifecycle_mutex;
  std::condition_variable work_ready;
  std::condition_variable idle;
  std::optional<ActiveInspectionMapIdentity> identity;
  ActiveInspectionMapCacheDiagnostics stats;
  std::uint64_t requested_generation{0};
  std::uint64_t completed_generation{0};
  std::uint64_t sequence{0};
  double refreshed_at_s{0.0};
  bool has_successful_refresh{false};
  bool stopping{false};
  std::thread worker;
};

ActiveInspectionMapCache::ActiveInspectionMapCache(Reader reader,
                                                   ActiveInspectionMapCacheConfig config)
    : impl_(std::make_unique<Impl>(std::move(reader), config)) {}

ActiveInspectionMapCache::~ActiveInspectionMapCache() = default;

void ActiveInspectionMapCache::requestRefresh() {
  impl_->requestRefresh();
}

void ActiveInspectionMapCache::flush() {
  impl_->flush();
}

void ActiveInspectionMapCache::stop() {
  impl_->stop();
}

ActiveInspectionMapSnapshot ActiveInspectionMapCache::snapshot(double now_steady_s) const {
  return impl_->snapshot(now_steady_s);
}

ActiveInspectionMapCacheDiagnostics ActiveInspectionMapCache::diagnostics() const {
  return impl_->diagnostics();
}

}  // namespace lingtu::nav::endpoint
