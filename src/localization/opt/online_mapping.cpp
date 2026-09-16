#include "localization/opt/online_mapping.hpp"
#include "localization/opt/pose_math.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <stdexcept>

namespace lingtu::localization::opt {
struct OnlineMapping::State {
  std::vector<MappingFrame> frames;
  std::vector<Keyframe> graph;
  std::vector<std::size_t> anchors;
  std::vector<GeometricConstraint> constraints;
  std::set<std::pair<std::size_t, std::size_t>> loops;
  LoopVerificationCache verification_cache;
  std::vector<Keyframe> optimized;
  std::size_t rejected = 0;
  std::size_t optimizations = 0;
  std::size_t optimization_failures = 0;
  lt_pose_graph_opt_report last_optimization{};
  std::uint64_t revision = 0;
  std::string code = "accumulating";

  void accept(MappingFrame frame, const OnlineMappingOptions& options) {
    frame.body_cloud = sample_mapping_cloud(frame.body_cloud,
        options.verification.voxel_size_m, options.verification.max_points_per_submap);
    frames.push_back(std::move(frame));
    graph.push_back(frames.back().keyframe);
    const auto index = frames.size() - 1;
    anchors.push_back(index);
    if (graph.size() > 1) {
      const auto sequential = generate_sequential_constraint([&](std::size_t i) {
        return frames.at(anchors.at(i)).body_cloud;
      }, graph, graph.size() - 2, options.verification);
      if (!sequential.ok) {
        ++rejected;
        graph.pop_back();
        anchors.pop_back();
        code = sequential.code;
        return;
      }
      constraints.push_back(sequential.constraint);
    }
    code = "accumulating";
  }

  void close_loops(const OnlineMappingOptions& options, std::size_t previous_size) {
    if (graph.size() == previous_size || graph.size() <= options.verification.min_index_separation)
      return;
    // Revisit recent anchors for multi-frame consensus. Old high-score loops
    // must not consume the bounded candidate budget forever.
    const auto window = options.verification.consensus_anchor_tolerance +
                        options.verification.min_consistent_matches;
    const auto first = previous_size > window ? previous_size - window : 0;
    const auto verified = generate_loop_constraints([&](std::size_t i) {
      return frames.at(anchors.at(i)).body_cloud;
    }, graph, options.verification, first, &verification_cache);
    if (!verified.ok) { code = verified.code; return; }
    const auto previous_constraint_count = constraints.size();
    std::vector<std::pair<std::size_t, std::size_t>> added;
    for (const auto& loop : verified.constraints) {
      if (loops.emplace(loop.from_index, loop.to_index).second) {
        constraints.push_back(loop);
        added.emplace_back(loop.from_index, loop.to_index);
      }
    }
    if (added.empty()) return;
    OptimizeOptions solve;
    solve.max_iterations = options.max_iterations;
    solve.geometric_constraints = constraints;
    auto estimates = graph;
    if (!optimized.empty()) {
      const auto last = optimized.size() - 1;
      const auto correction = compose_pose(optimized[last].pose, inverse_pose(graph[last].pose));
      for (std::size_t i = 0; i < estimates.size(); ++i)
        estimates[i].pose = i < optimized.size() ? optimized[i].pose
            : compose_pose(correction, graph[i].pose);
    }
    auto solution = optimize_graph(estimates, solve);
    last_optimization = solution.report;
    code = solution.code;
    if (solution.ok) {
      optimized = std::move(solution.keyframes);
      ++optimizations;
      code = "loop_corrected";
    } else {
      ++optimization_failures;
      constraints.resize(previous_constraint_count);
      for (const auto& pair : added) loops.erase(pair);
    }
  }

  std::shared_ptr<OnlineMappingSnapshot> snapshot(
      std::uint64_t epoch, const OnlineMappingOptions& options) {
    auto result = std::make_shared<OnlineMappingSnapshot>();
    result->source_epoch = epoch;
    result->revision = ++revision;
    result->code = code;
    result->registered_keyframes = graph.size();
    result->registration_rejections = rejected;
    result->loop_constraints = loops.size();
    result->optimizations = optimizations;
    result->optimization_failures = optimization_failures;
    result->last_optimization = last_optimization;
    if (frames.empty()) return result;
    result->stamp_s = frames.back().stamp_s;
    if (!optimized.empty()) {
      const auto last = optimized.size() - 1;
      result->global_from_odom = compose_pose(optimized[last].pose, inverse_pose(graph[last].pose));
    }
    const auto odom_from_global = inverse_pose(result->global_from_odom);
    Pose correction;
    std::size_t anchor = 0;
    std::vector<Point> cloud;
    for (std::size_t i = 0; i < frames.size(); ++i) {
      if (anchor < anchors.size() && anchors[anchor] == i) {
        if (anchor < optimized.size())
          correction = compose_pose(optimized[anchor].pose, inverse_pose(graph[anchor].pose));
        ++anchor;
      }
      auto keyframe = frames[i].keyframe;
      keyframe.pose = compose_pose(odom_from_global, compose_pose(correction, keyframe.pose));
      result->keyframes.push_back(keyframe);
      for (const auto& point : frames[i].body_cloud) {
        const auto xyz = rotate_vector(keyframe.pose, point.x, point.y, point.z);
        cloud.push_back({static_cast<float>(xyz[0] + keyframe.pose.x),
                         static_cast<float>(xyz[1] + keyframe.pose.y),
                         static_cast<float>(xyz[2] + keyframe.pose.z), point.intensity});
      }
    }
    result->cloud = sample_mapping_cloud(cloud, options.preview_voxel_m, options.preview_points);
    return result;
  }
};

OnlineMapping::OnlineMapping(OnlineMappingOptions options) : options_(std::move(options)) {
  if (options_.max_keyframes < 3 || options_.max_keyframes > 3000 ||
      options_.max_pending == 0 || options_.max_pending > 32 ||
      options_.max_iterations == 0 || options_.max_iterations > 200 ||
      options_.preview_points == 0 || !std::isfinite(options_.preview_voxel_m) ||
      options_.preview_voxel_m <= 0)
    throw std::invalid_argument("invalid online mapping limits");
  reset(0);
}
OnlineMapping::~OnlineMapping() = default;

void OnlineMapping::reset(std::uint64_t source_epoch) {
  epoch_ = source_epoch;
  ++generation_;
  queued_frames_ = 0;
  dropped_frames_ = 0;
  pending_.clear();
  state_ = std::make_shared<State>();
  // Retain the future until poll: destroying an async job would block LIO.
}

Result OnlineMapping::enqueue(MappingFrame frame) {
  Result result;
  const auto& p = frame.keyframe.pose;
  const double q = p.qw*p.qw + p.qx*p.qx + p.qy*p.qy + p.qz*p.qz;
  if (frame.keyframe.patch_name.empty() || frame.body_cloud.empty() ||
      !std::isfinite(frame.stamp_s) || !std::isfinite(p.x) ||
      !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(q) ||
      q < .81 || q > 1.21) {
    result.code = "invalid_mapping_frame";
  } else if (queued_frames_ >= options_.max_keyframes) {
    result.code = "online_mapping_capacity_reached";
  } else if (pending_.size() >= options_.max_pending) {
    result.code = "online_mapping_backpressure";
  } else {
    pending_.push_back(std::move(frame));
    ++queued_frames_;
    result.ok = true;
    result.code = "mapping_frame_queued";
  }
  if (!result.ok) ++dropped_frames_;
  result.message = result.code;
  return result;
}

std::shared_ptr<const OnlineMappingSnapshot> OnlineMapping::poll() {
  std::shared_ptr<const OnlineMappingSnapshot> snapshot;
  if (job_.valid()) {
    if (job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return {};
    auto done = job_.get();
    if (done.generation == generation_) {
      state_ = std::move(done.state);
      snapshot = std::move(done.snapshot);
    }
  }
  if (!pending_.empty()) {
    auto frames = std::move(pending_);
    pending_.clear();
    auto state = std::move(state_);
    const auto options = options_;
    const auto epoch = epoch_;
    const auto generation = generation_;
    job_ = std::async(std::launch::async,
        [state, frames = std::move(frames), options, epoch, generation]() mutable {
      std::shared_ptr<OnlineMappingSnapshot> result;
      try {
        const auto previous_size = state->graph.size();
        for (auto& frame : frames) state->accept(std::move(frame), options);
        state->close_loops(options, previous_size);
        result = state->snapshot(epoch, options);
      } catch (const std::exception& exception) {
        result = std::make_shared<OnlineMappingSnapshot>();
        result->source_epoch = epoch;
        result->code = std::string("online_mapping_failed: ") + exception.what();
      }
      return Completed{generation, state, result};
    });
  }
  return snapshot;
}
}  // namespace lingtu::localization::opt
