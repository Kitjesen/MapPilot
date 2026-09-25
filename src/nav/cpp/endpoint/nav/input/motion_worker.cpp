#include "input/motion_worker.hpp"

#include <exception>
#include <utility>

#include "runtime/time.hpp"

namespace lingtu::nav::endpoint {

MotionWorker::MotionWorker(MotionLayer layer, std::size_t max_points, bool snapshots_enabled)
    : layer_(std::move(layer)),
      max_points_(max_points),
      snapshots_enabled_(snapshots_enabled),
      thread_([this] { run(); }) {}

MotionWorker::~MotionWorker() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_one();
  thread_.join();
}

void MotionWorker::submit(MotionObservation observation) {
  std::optional<MotionObservation> replaced;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Keep the newest waiting frame, not a queue of increasingly stale scans.
    replaced.swap(pending_);
    pending_.emplace(std::move(observation));
  }
  wake_.notify_one();
}

std::optional<MotionResult> MotionWorker::poll() {
  std::optional<MotionResult> result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result.swap(completed_);
  }
  return result;
}

void MotionWorker::clear() {
  std::optional<MotionObservation> discarded_input;
  std::optional<MotionResult> discarded_result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++epoch_;
    discarded_input.swap(pending_);
    discarded_result.swap(completed_);
  }
  wake_.notify_one();
}

void MotionWorker::run() {
  std::uint64_t layer_epoch = 0;
  for (;;) {
    std::optional<MotionObservation> observation;
    std::uint64_t job_epoch;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [&] { return stopping_ || pending_ || epoch_ != layer_epoch; });
      if (stopping_) {
        return;
      }
      job_epoch = epoch_;
      observation.swap(pending_);
    }
    if (job_epoch != layer_epoch) {
      layer_.clear();
      layer_epoch = job_epoch;
    }
    if (!observation) {
      continue;
    }

    MotionResult result;
    result.stamp_s = observation->stamp_s;
    result.receive_s = observation->receive_s;
    result.origin = observation->origin;
    try {
      const auto update_start = SteadyClock::now();
      layer_.updateFromScan(observation->origin, observation->xyzh, observation->stamp_s);
      result.clusters = layer_.dynamicClusters(32, observation->stamp_s);
      result.update_ms = elapsedMs(update_start);
      const auto snapshot_start = SteadyClock::now();
      if (snapshots_enabled_) {
        layer_.snapshot(result.obstacles, max_points_, observation->stamp_s);
        layer_.snapshotPredictedDynamic(result.predicted_points, kMaxDynamicPredictionPoints,
                                        observation->stamp_s);
        result.predictions = layer_.predictedVolumes(observation->stamp_s);
      }
      result.stats = layer_.stats();
      result.snapshot_ms = elapsedMs(snapshot_start);
    } catch (const std::exception &error) {
      // A failed frame cannot refresh the control gate or publish partial geometry.
      layer_.clear();
      result.error = error.what();
    }

    std::optional<MotionResult> replaced;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!stopping_ && epoch_ == job_epoch) {
        replaced.swap(completed_);
        completed_.emplace(std::move(result));
      }
    }
  }
}

}  // namespace lingtu::nav::endpoint
