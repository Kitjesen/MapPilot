#include "localization/opt/online_mapping.hpp"
#include "localization/sam/backend.hpp"
#include "localization/opt/pose_math.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <tuple>

namespace lingtu::localization::opt {
struct OnlineMapping::State {
  struct PreviewCell {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double intensity = 0.0;
    std::size_t count = 0;

    void add(const Point& point) {
      x += point.x;
      y += point.y;
      z += point.z;
      intensity += point.intensity;
      ++count;
    }

    void merge(const PreviewCell& other) {
      x += other.x;
      y += other.y;
      z += other.z;
      intensity += other.intensity;
      count += other.count;
    }
  };
  using PreviewKey = std::tuple<std::int64_t, std::int64_t, std::int64_t>;

  std::vector<MappingFrame> frames;
  std::vector<Keyframe> graph;
  sam::Backend backend;
  explicit State(const sam::Config& config) : backend(config) {}
  std::map<std::size_t, Keyframe> optimized;
  std::size_t sequential_count = 0;
  std::size_t rejected = 0;
  std::size_t optimizations = 0;
  std::size_t optimization_failures = 0;
  std::uint64_t revision = 0;
  std::string code = "accumulating";
  std::map<PreviewKey, PreviewCell> preview_cells;
  std::size_t preview_frame_count = 0;
  bool preview_dirty = false;
  bool failed = false;
  Pose preview_anchor;
  double preview_resolution_m = 0.0;

  static PreviewKey previewKey(double x, double y, double z, double resolution) {
    return {static_cast<std::int64_t>(std::floor(x / resolution)),
            static_cast<std::int64_t>(std::floor(y / resolution)),
            static_cast<std::int64_t>(std::floor(z / resolution))};
  }

  void addPreviewPoint(const Point& point, std::size_t max_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      return;
    preview_cells[previewKey(point.x, point.y, point.z, preview_resolution_m)].add(point);
    while (preview_cells.size() > max_points) {
      preview_resolution_m *= 2.0;
      std::map<PreviewKey, PreviewCell> reduced;
      for (const auto& [key, cell] : preview_cells) {
        const double count = static_cast<double>(cell.count);
        reduced[previewKey(cell.x / count, cell.y / count, cell.z / count,
                           preview_resolution_m)].merge(cell);
      }
      preview_cells.swap(reduced);
    }
  }

  void accept(MappingFrame frame, const OnlineMappingOptions&) {
    if (failed) throw std::runtime_error("LIO-SAM worker failed; new session required");
    sam::Cloud::Ptr cloud(new sam::Cloud);
    cloud->reserve(frame.body_cloud.size());
    for (const auto& point : frame.body_cloud) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      pcl::PointXYZI p; p.x=point.x; p.y=point.y; p.z=point.z; p.intensity=point.intensity;
      cloud->push_back(p);
    }
    const auto& p = frame.keyframe.pose;
    Eigen::Quaterniond q(p.qw,p.qx,p.qy,p.qz);
    const gtsam::Pose3 odom(gtsam::Rot3(q.normalized()), {p.x,p.y,p.z});
    const auto loop = backend.append({frame.stamp_s, odom, cloud});
    // Backend owns the sole full cloud retained by this worker. The capture
    // journal independently owns the original scan for lossless saving.
    std::vector<Point>().swap(frame.body_cloud);
    frames.push_back(std::move(frame));
    graph.push_back(frames.back().keyframe);
    sequential_count = frames.size()-1;
    if (loop.accepted) ++optimizations;
    for (std::size_t i=0;i<frames.size();++i) {
      const auto& pose=backend.poses()[i];
      const auto rotation=pose.rotation().toQuaternion();
      if (i < optimized.size()) {
        const auto& old = optimized.at(i).pose;
        const auto delta = compose_pose({pose.x(),pose.y(),pose.z(),rotation.w(),rotation.x(),rotation.y(),rotation.z()}, inverse_pose(old));
        preview_dirty |= std::abs(delta.x)+std::abs(delta.y)+std::abs(delta.z)+
                         std::abs(delta.qx)+std::abs(delta.qy)+std::abs(delta.qz) > 1e-9;
      }
      optimized[i] = {graph[i].patch_name, {pose.x(),pose.y(),pose.z(),
                      rotation.w(),rotation.x(),rotation.y(),rotation.z()}};
    }
    code = loop.accepted ? "loop_corrected" : "accumulating";
  }

  std::shared_ptr<OnlineMappingSnapshot> snapshot(
      std::uint64_t epoch, const OnlineMappingOptions& options) {
    const auto started = std::chrono::steady_clock::now();
    auto result = std::make_shared<OnlineMappingSnapshot>();
    result->source_epoch = epoch;
    result->revision = ++revision;
    result->code = code;
    result->registered_keyframes = backend.poses().size();
    if (result->registered_keyframes != graph.size()) result->code = "graph_disconnected";
    result->registration_rejections = rejected;
    result->sequential_constraints = sequential_count;
    result->loop_constraints = backend.loops();
    result->optimizations = optimizations;
    result->optimization_failures = optimization_failures;
    result->sam_config = options.sam;
    result->loop_records = backend.records();
    result->cloud_bytes = backend.cloudBytes();
    if (frames.empty()) return result;
    result->stamp_s = frames.back().stamp_s;
    if (!optimized.empty()) {
      const auto& last = *optimized.rbegin();
      result->global_from_odom = compose_pose(last.second.pose, inverse_pose(graph[last.first].pose));
    }
    const auto odom_from_global = inverse_pose(result->global_from_odom);
    const auto anchor_delta = compose_pose(odom_from_global, inverse_pose(preview_anchor));
    preview_dirty |= std::abs(anchor_delta.x)+std::abs(anchor_delta.y)+std::abs(anchor_delta.z)+
                     std::abs(anchor_delta.qx)+std::abs(anchor_delta.qy)+std::abs(anchor_delta.qz)>1e-9;
    preview_anchor = odom_from_global;
    if (preview_dirty) {
      preview_cells.clear();
      preview_frame_count = 0;
      preview_dirty = false;
      preview_resolution_m = options.preview_voxel_m;
    }
    if (preview_resolution_m == 0.0) preview_resolution_m = options.preview_voxel_m;
    Pose correction;
    for (std::size_t i = 0; i < frames.size(); ++i) {
      const auto found = optimized.find(i);
      if (found != optimized.end())
        correction = compose_pose(found->second.pose, inverse_pose(graph[i].pose));
      auto keyframe = frames[i].keyframe;
      keyframe.pose = compose_pose(odom_from_global, compose_pose(correction, keyframe.pose));
      result->keyframes.push_back(keyframe);
      if (i >= preview_frame_count) {
        for (const auto& point : *backend.cloud(i)) {
          const auto xyz = rotate_vector(keyframe.pose, point.x, point.y, point.z);
          addPreviewPoint({static_cast<float>(xyz[0] + keyframe.pose.x),
                           static_cast<float>(xyz[1] + keyframe.pose.y),
                           static_cast<float>(xyz[2] + keyframe.pose.z), point.intensity},
                          options.preview_points);
        }
      }
    }
    preview_frame_count = frames.size();
    result->cloud.reserve(preview_cells.size());
    for (const auto& [key, cell] : preview_cells) {
      const double scale = 1.0 / static_cast<double>(cell.count);
      result->cloud.push_back({static_cast<float>(cell.x * scale),
                               static_cast<float>(cell.y * scale),
                               static_cast<float>(cell.z * scale),
                               static_cast<float>(cell.intensity * scale)});
    }
    result->preview_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now()-started).count();
    return result;
  }
};

OnlineMapping::OnlineMapping(OnlineMappingOptions options) : options_(std::move(options)) {
  if (options_.max_keyframes < 3 || options_.max_keyframes > 3000 ||
      options_.max_pending == 0 || options_.max_pending > 32 ||
      options_.preview_points == 0 || !std::isfinite(options_.preview_voxel_m) ||
      options_.preview_voxel_m <= 0)
    throw std::invalid_argument("invalid online mapping limits");
  reset(0);
}
OnlineMapping::~OnlineMapping() = default;

void OnlineMapping::configureSam(const sam::Config& config) {
  sam::Backend validated(config);
  options_.sam = config;
  reset(epoch_);
}

void OnlineMapping::reset(std::uint64_t source_epoch) {
  epoch_ = source_epoch;
  ++generation_;
  queued_frames_ = 0;
  dropped_frames_ = 0;
  pending_high_water_ = 0;
  pending_.clear();
  state_ = std::make_shared<State>(options_.sam);
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
    pending_high_water_ = std::max(pending_high_water_, pending_.size());
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
    const auto high_water = pending_high_water_;
    job_ = std::async(std::launch::async,
        [state, frames = std::move(frames), options, epoch, generation, high_water]() mutable {
      const auto started = std::chrono::steady_clock::now();
      std::shared_ptr<OnlineMappingSnapshot> result;
      try {
        for (auto& frame : frames) state->accept(std::move(frame), options);
        result = state->snapshot(epoch, options);
      } catch (const std::exception& exception) {
        state->failed = true;
        ++state->optimization_failures;
        result = std::make_shared<OnlineMappingSnapshot>();
        result->source_epoch = epoch;
        result->optimization_failures = state->optimization_failures;
        result->code = std::string("online_mapping_failed: ") + exception.what();
      }
      result->worker_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now()-started).count();
      result->pending_high_water = high_water;
      return Completed{generation, state, result};
    });
  }
  return snapshot;
}
}  // namespace lingtu::localization::opt
