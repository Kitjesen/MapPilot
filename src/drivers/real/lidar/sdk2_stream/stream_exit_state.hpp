#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>

namespace lingtu::drivers::lidar {

struct StreamExitDecision {
  bool timestamp_fault = false;
  bool flush_final_batch = true;
  int return_code = 0;
};

class StreamExitState {
 public:
  static_assert(
      std::atomic<bool>::is_always_lock_free,
      "signal-stop requires lock-free atomic<bool>");

  // Returns true only for the first fatal timestamp request.
  bool request_timestamp_fault() noexcept {
    bool first = false;
    {
      std::lock_guard<std::mutex> lock(wait_mutex_);
      first = !timestamp_fault_.exchange(true, std::memory_order_acq_rel);
      quit_.store(true, std::memory_order_release);
    }
    wait_cv_.notify_all();
    return first;
  }

  // Called from the process signal handler: keep this to one lock-free store.
  void request_signal_stop() noexcept {
    quit_.store(true, std::memory_order_release);
  }

  bool quit_requested() const noexcept {
    return quit_.load(std::memory_order_acquire);
  }

  void wait_for_stop() {
    std::unique_lock<std::mutex> lock(wait_mutex_);
    while (!wait_cv_.wait_for(
        lock,
        std::chrono::milliseconds(100),
        [this] { return quit_.load(std::memory_order_acquire); })) {
    }
  }

  StreamExitDecision snapshot_exit_decision() const noexcept {
    const bool timestamp_fault =
        timestamp_fault_.load(std::memory_order_acquire);
    return StreamExitDecision{
        timestamp_fault,
        !timestamp_fault,
        timestamp_fault ? 1 : 0};
  }

 private:
  std::condition_variable wait_cv_;
  std::mutex wait_mutex_;
  std::atomic<bool> quit_{false};
  std::atomic<bool> timestamp_fault_{false};
};

}  // namespace lingtu::drivers::lidar
