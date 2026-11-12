#pragma once

#include "localization/map_optimization/poses.hpp"
#include "localization/map_optimization/cloud.hpp"
#include "localization/online_mapping/backend.hpp"

#include <deque>
#include <future>
#include <memory>

namespace lingtu::localization::online_mapping {

struct MappingFrame {
  map_optimization::Keyframe keyframe;
  double stamp_s = 0.0;
  std::vector<map_optimization::Point> body_cloud;
  bool relocalized = false;
};

struct WorkerOptions {
  std::size_t max_keyframes = 3000;
  std::size_t max_pending = 4;
  std::size_t preview_points = 200000;
  double preview_voxel_m = 0.12;
  online_mapping::Config online_mapping;
};

struct Snapshot {
  std::uint64_t source_epoch = 0;
  std::uint64_t revision = 0;
  double stamp_s = 0.0;
  std::string code = "waiting_for_keyframes";
  std::size_t registered_keyframes = 0;
  std::size_t registration_rejections = 0;
  std::size_t sequential_constraints = 0;
  std::size_t loop_constraints = 0;
  std::size_t optimizations = 0;
  std::size_t optimization_failures = 0;
  online_mapping::Config online_mapping_config;
  std::vector<online_mapping::LoopResult> loop_records;
  std::vector<std::size_t> segment_starts;
  double worker_ms = 0;
  double preview_ms = 0;
  std::size_t cloud_bytes = 0;
  std::size_t pending_high_water = 0;
  // Both outputs are in the continuous odometry frame at the newest anchor.
  // Historical geometry is corrected, while current odometry never jumps.
  std::vector<map_optimization::Keyframe> keyframes;
  std::vector<map_optimization::Point> cloud;
  map_optimization::Pose global_from_odom;
};

struct EnqueueResult {
  bool ok = false;
  std::string code;
};

// Single runtime-thread owner. Registration, loop detection, solving and cloud
// reconstruction all execute in one background job, never in the LIO tick.
class Worker {
 public:
  explicit Worker(WorkerOptions options = {});
  ~Worker();
  void reset(std::uint64_t source_epoch);
  void configure(const Config& config);
  const Config& config() const { return options_.online_mapping; }
  EnqueueResult enqueue(MappingFrame frame);
  std::shared_ptr<const Snapshot> restore(std::vector<MappingFrame> frames,
      const std::vector<LoopResult>& records, const Config& config, std::uint64_t source_epoch);
  std::shared_ptr<const Snapshot> update();
  bool busy() const { return job_.valid() || !pending_.empty(); }
  bool canEnqueue() const {
    return queued_frames_ < options_.max_keyframes &&
           pending_.size() < options_.max_pending;
  }
  bool capacityReached() const { return queued_frames_ >= options_.max_keyframes; }
  std::size_t capacity() const { return options_.max_keyframes; }
  std::size_t dropped_frames() const { return dropped_frames_; }
  std::size_t optimization_failures() const { return optimization_failures_; }

 private:
  struct State;
  struct Completed {
    std::uint64_t generation;
    std::shared_ptr<State> state;
    std::shared_ptr<Snapshot> snapshot;
  };
  WorkerOptions options_;
  std::uint64_t epoch_ = 0;
  std::uint64_t generation_ = 0;
  std::size_t queued_frames_ = 0;
  std::size_t dropped_frames_ = 0;
  std::size_t optimization_failures_ = 0;
  std::size_t pending_high_water_ = 0;
  std::deque<MappingFrame> pending_;
  std::shared_ptr<State> state_;
  std::future<Completed> job_;
};

}  // namespace lingtu::localization::online_mapping
