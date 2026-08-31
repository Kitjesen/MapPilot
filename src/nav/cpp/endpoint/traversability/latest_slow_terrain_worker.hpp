#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>

namespace lingtu::nav::endpoint {

template <typename Input, typename Result, typename Processor, typename Resetter>
class LatestOnlyWorker {
 public:
  LatestOnlyWorker(Processor processor, Resetter resetter)
      : processor_(std::move(processor)), resetter_(std::move(resetter)),
        worker_([this]() { run(); }) {}

  LatestOnlyWorker(const LatestOnlyWorker &) = delete;
  LatestOnlyWorker &operator=(const LatestOnlyWorker &) = delete;

  ~LatestOnlyWorker() { stop(); }

  void submit(Input input) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = std::move(input);
    }
    cv_.notify_one();
  }

  void requestReset() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_.reset();
      latest_result_.reset();
      reset_requested_ = true;
    }
    cv_.notify_one();
  }

  std::optional<Result> takeLatestResult() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = std::move(latest_result_);
    latest_result_.reset();
    return result;
  }

  void stop() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stopping_) {
        return;
      }
      stopping_ = true;
      pending_.reset();
    }
    cv_.notify_one();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void run() {
    for (;;) {
      std::optional<Input> input;
      bool reset = false;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&]() { return stopping_ || reset_requested_ || pending_.has_value(); });
        if (stopping_) {
          return;
        }
        if (reset_requested_) {
          pending_.reset();
          latest_result_.reset();
          reset_requested_ = false;
          reset = true;
        } else {
          input = std::move(pending_);
          pending_.reset();
        }
      }
      if (reset) {
        resetter_();
        continue;
      }
      if (!input) {
        continue;
      }
      Result result = processor_(std::move(*input));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reset_requested_ || stopping_) {
          continue;
        }
        latest_result_ = std::move(result);
      }
    }
  }

  Processor processor_;
  Resetter resetter_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Input> pending_;
  std::optional<Result> latest_result_;
  bool reset_requested_{false};
  bool stopping_{false};
  std::thread worker_;
};

template <typename Input, typename Result, typename Processor, typename Resetter>
auto makeLatestOnlyWorker(Processor processor, Resetter resetter) {
  return LatestOnlyWorker<Input, Result, std::decay_t<Processor>, std::decay_t<Resetter>>(
      std::move(processor), std::move(resetter));
}

inline bool slowTerrainResultCanUpdate(std::uint64_t result_generation,
                                       std::uint64_t last_applied_generation) {
  return result_generation > last_applied_generation;
}

}  // namespace lingtu::nav::endpoint
