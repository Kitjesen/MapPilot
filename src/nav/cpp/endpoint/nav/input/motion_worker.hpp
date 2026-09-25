#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "input/obstacle.hpp"

namespace lingtu::nav::endpoint {

struct MotionObservation {
  std::vector<float> xyzh;
  SensorOrigin origin;
  double stamp_s{0.0};
  double receive_s{0.0};
};

struct MotionResult {
  double stamp_s{0.0};
  double receive_s{0.0};
  SensorOrigin origin;
  std::vector<float> obstacles;
  std::vector<float> predicted_points;
  std::vector<nav_kernel::PredictedObstacle> predictions;
  std::vector<DynamicCluster> clusters;
  MotionLayerStats stats;
  double update_ms{0.0};
  double snapshot_ms{0.0};
  std::string error;
};

// One owner for the mutable voxel/tracking layer. The control thread only moves
// observations and completed snapshots; it never waits for point-cloud work.
class MotionWorker {
 public:
  MotionWorker(MotionLayer layer, std::size_t max_points, bool snapshots_enabled = true);
  ~MotionWorker();
  MotionWorker(const MotionWorker &) = delete;
  MotionWorker &operator=(const MotionWorker &) = delete;

  void submit(MotionObservation observation);
  std::optional<MotionResult> poll();
  // Invalidates pending AND in-flight results without waiting for processing.
  void clear();

 private:
  void run();

  MotionLayer layer_;
  const std::size_t max_points_;
  const bool snapshots_enabled_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_{false};
  std::uint64_t epoch_{0};
  std::optional<MotionObservation> pending_;
  std::optional<MotionResult> completed_;
  std::thread thread_;
};

}  // namespace lingtu::nav::endpoint
