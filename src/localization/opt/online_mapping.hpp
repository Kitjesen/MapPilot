#pragma once

#include "localization/opt/poses.hpp"
#include "localization/opt/map.hpp"
#include "localization/opt/cloud.hpp"
#include "localization/sam/backend.hpp"

#include <deque>
#include <future>
#include <memory>

namespace lingtu::localization::opt {

struct MappingFrame {
  Keyframe keyframe;
  double stamp_s = 0.0;
  std::vector<Point> body_cloud;
};

struct OnlineMappingOptions {
  std::size_t max_keyframes = 3000;
  std::size_t max_pending = 4;
  std::size_t preview_points = 200000;
  double preview_voxel_m = 0.12;
  sam::Config sam;
};

struct OnlineMappingSnapshot {
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
  sam::Config sam_config;
  std::vector<sam::LoopResult> loop_records;
  double worker_ms = 0;
  double preview_ms = 0;
  std::size_t cloud_bytes = 0;
  std::size_t pending_high_water = 0;
  // Both outputs are in the continuous odometry frame at the newest anchor.
  // Historical geometry is corrected, while current odometry never jumps.
  std::vector<Keyframe> keyframes;
  std::vector<Point> cloud;
  Pose global_from_odom;
};

// Single runtime-thread owner. Registration, loop detection, solving and cloud
// reconstruction all execute in one background job, never in the LIO tick.
class OnlineMapping {
 public:
  explicit OnlineMapping(OnlineMappingOptions options = {});
  ~OnlineMapping();
  void reset(std::uint64_t source_epoch);
  void configureSam(const sam::Config& config);
  const sam::Config& samConfig() const { return options_.sam; }
  Result enqueue(MappingFrame frame);
  std::shared_ptr<const OnlineMappingSnapshot> poll();
  bool busy() const { return job_.valid() || !pending_.empty(); }
  bool canEnqueue() const {
    return queued_frames_ < options_.max_keyframes &&
           pending_.size() < options_.max_pending;
  }
  bool capacityReached() const { return queued_frames_ >= options_.max_keyframes; }
  std::size_t dropped_frames() const { return dropped_frames_; }

 private:
  struct State;
  struct Completed {
    std::uint64_t generation;
    std::shared_ptr<State> state;
    std::shared_ptr<OnlineMappingSnapshot> snapshot;
  };
  OnlineMappingOptions options_;
  std::uint64_t epoch_ = 0;
  std::uint64_t generation_ = 0;
  std::size_t queued_frames_ = 0;
  std::size_t dropped_frames_ = 0;
  std::size_t pending_high_water_ = 0;
  std::deque<MappingFrame> pending_;
  std::shared_ptr<State> state_;
  std::future<Completed> job_;
};

}  // namespace lingtu::localization::opt
