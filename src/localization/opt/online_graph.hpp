#pragma once

#include "localization/opt/graph.hpp"

#include <cstdint>
#include <future>
#include <optional>
#include <set>
#include <utility>

namespace lingtu::localization::opt {

struct OnlineGraphLimits {
  // The solver is batch (sparse with dense fallback), not iSAM2.
  std::size_t max_keyframes = 256;
  std::size_t max_constraints = 4096;
  std::size_t max_iterations = 30;
};

struct OnlineGraphUpdate {
  std::uint64_t source_epoch = 0;
  std::uint64_t revision = 0;
  GraphSolution solution;
  // Valid only on success, at the last keyframe in this update's prefix.
  Pose map_from_odom;
};

// Owned by one runtime thread. Only an immutable graph copy crosses to the
// worker. This consumes already measured/verified factors; it is NOT a place
// recognizer. It never modifies Fast-LIO state or publishes motion transforms.
class OnlinePoseGraph {
 public:
  explicit OnlinePoseGraph(OnlineGraphLimits limits = {});
  void reset(std::uint64_t source_epoch);
  Result append(Keyframe keyframe,
                const std::optional<GeometricConstraint>& measured_adjacent = std::nullopt);
  Result add_verified_loop(const GeometricConstraint& loop);
  // At most one job; busy/unchanged requests leave the graph available to retry.
  Result start_optimization();
  std::optional<OnlineGraphUpdate> poll();
  bool busy() const { return job_.valid(); }
  const std::vector<Keyframe>& odometry_keyframes() const { return keyframes_; }

 private:
  OnlineGraphLimits limits_;
  std::uint64_t source_epoch_ = 0;
  std::uint64_t generation_ = 0;
  std::uint64_t revision_ = 0;
  std::size_t loops_ = 0;
  std::uint64_t last_submitted_revision_ = 0;
  std::vector<Keyframe> keyframes_;
  std::vector<GeometricConstraint> constraints_;
  std::set<std::string> names_;
  std::set<std::pair<std::size_t, std::size_t>> loop_pairs_;
  struct JobResult {
    std::uint64_t generation = 0;
    OnlineGraphUpdate update;
  };
  std::future<JobResult> job_;
};

}  // namespace lingtu::localization::opt
