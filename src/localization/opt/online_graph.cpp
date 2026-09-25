#include "localization/opt/online_graph.hpp"

#include "localization/opt/constraints.hpp"
#include "localization/opt/pose_math.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace lingtu::localization::opt {
namespace {
Result status(bool ok, const std::string& code) {
  Result result;
  result.ok = ok;
  result.code = code;
  result.message = code;
  return result;
}

bool valid_pose(const Pose& pose) {
  const double norm = std::sqrt(square(pose.qw) + square(pose.qx) +
                                square(pose.qy) + square(pose.qz));
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
         std::isfinite(norm) && norm >= 0.9 && norm <= 1.1;
}

bool valid_factor(const GeometricConstraint& factor) {
  return valid_pose(factor.pose_from_to) && valid_information_upper(factor.information_upper);
}
}  // namespace

OnlinePoseGraph::OnlinePoseGraph(OnlineGraphLimits limits) : limits_(limits) {
  if (limits_.max_keyframes < 3 || limits_.max_keyframes > 256 ||
      limits_.max_constraints < 3 || limits_.max_constraints > 4096 ||
      limits_.max_iterations == 0 || limits_.max_iterations > 200) {
    throw std::invalid_argument("online graph limits exceed the validated solver budget");
  }
}

void OnlinePoseGraph::reset(std::uint64_t source_epoch) {
  source_epoch_ = source_epoch;
  ++generation_;
  revision_ = 0;
  loops_ = 0;
  last_submitted_revision_ = 0;
  keyframes_.clear();
  constraints_.clear();
  names_.clear();
  loop_pairs_.clear();
  // Do not destroy an active std::async future here: that would join on the
  // sensor thread. poll discards the old generation when it completes.
}

Result OnlinePoseGraph::append(
    Keyframe keyframe, const std::optional<GeometricConstraint>& measured_adjacent) {
  if (keyframes_.size() >= limits_.max_keyframes ||
      (!keyframes_.empty() && constraints_.size() >= limits_.max_constraints)) {
    return status(false, "online_graph_capacity_reached");
  }
  if (keyframe.patch_name.empty() || names_.count(keyframe.patch_name) ||
      !valid_pose(keyframe.pose)) {
    return status(false, "invalid_online_keyframe");
  }
  if (keyframes_.empty()) {
    if (measured_adjacent) return status(false, "unexpected_first_keyframe_factor");
  } else if (!measured_adjacent ||
             measured_adjacent->from_index != keyframes_.size() - 1 ||
             measured_adjacent->to_index != keyframes_.size() ||
             !valid_factor(*measured_adjacent)) {
    return status(false, "measured_adjacent_factor_required");
  }
  names_.insert(keyframe.patch_name);
  keyframes_.push_back(std::move(keyframe));
  if (measured_adjacent) constraints_.push_back(*measured_adjacent);
  ++revision_;
  return status(true, "keyframe_added");
}

Result OnlinePoseGraph::add_verified_loop(const GeometricConstraint& loop) {
  if (constraints_.size() >= limits_.max_constraints) {
    return status(false, "online_graph_capacity_reached");
  }
  if (loop.from_index >= keyframes_.size() || loop.to_index >= keyframes_.size() ||
      loop.from_index == loop.to_index || !valid_factor(loop)) {
    return status(false, "invalid_verified_loop");
  }
  const auto pair = std::minmax(loop.from_index, loop.to_index);
  if (pair.second - pair.first < 2) return status(false, "loop_is_adjacent_edge");
  if (!loop_pairs_.insert(pair).second) return status(false, "duplicate_loop");
  constraints_.push_back(loop);
  ++loops_;
  ++revision_;
  return status(true, "verified_loop_added");
}

Result OnlinePoseGraph::start_optimization() {
  if (job_.valid()) return status(false, "online_optimizer_busy");
  if (loops_ == 0) return status(true, "waiting_for_verified_loop");
  if (last_submitted_revision_ == revision_) return status(true, "graph_unchanged");
  OptimizeOptions options;
  options.max_iterations = limits_.max_iterations;
  options.geometric_constraints = constraints_;
  options.gravity_reference = keyframes_;
  const auto epoch = source_epoch_;
  const auto revision = revision_;
  const auto generation = generation_;
  // Copy the accepted prefix; new keyframes can arrive while it is optimized.
  auto keyframes = keyframes_;
  job_ = std::async(std::launch::async,
      [keyframes = std::move(keyframes), options = std::move(options), epoch, revision,
       generation]() {
        JobResult result;
        result.generation = generation;
        result.update.source_epoch = epoch;
        result.update.revision = revision;
        try {
          result.update.solution = optimize_graph(keyframes, options);
          if (result.update.solution.ok) {
            result.update.map_from_odom = gravity_preserving_alignment(
                result.update.solution.keyframes.back().pose,
                keyframes.back().pose);
          }
        } catch (const std::exception& error) {
          result.update.solution.code = "online_optimizer_failed";
          result.update.solution.message = error.what();
        }
        return result;
      });
  last_submitted_revision_ = revision_;
  return status(true, "optimization_started");
}

std::optional<OnlineGraphUpdate> OnlinePoseGraph::poll() {
  if (!job_.valid() || job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return std::nullopt;
  }
  auto result = job_.get();
  if (result.generation != generation_) return std::nullopt;
  return std::move(result.update);
}
}  // namespace lingtu::localization::opt
