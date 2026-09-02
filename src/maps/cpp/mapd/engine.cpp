#include "lingtu/maps/mapd/engine.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lingtu::maps::mapd {
namespace {

bool IsFinite(double value) {
  return std::isfinite(value);
}

std::int64_t SteadyTimeNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

bool ReadPoint(
    const PointCloudView& cloud,
    std::size_t index,
    float* x,
    float* y,
    float* z) {
  if (x == nullptr || y == nullptr || z == nullptr) {
    return false;
  }
  switch (cloud.layout) {
    case CloudLayout::kXyzF32Interleaved: {
      const std::size_t offset = index * 3U;
      if (cloud.interleaved.data == nullptr ||
          offset + 2U >= cloud.interleaved.size) {
        return false;
      }
      *x = cloud.interleaved.data[offset];
      *y = cloud.interleaved.data[offset + 1U];
      *z = cloud.interleaved.data[offset + 2U];
      return true;
    }
    case CloudLayout::kXyziF32Interleaved: {
      const std::size_t offset = index * 4U;
      if (cloud.interleaved.data == nullptr ||
          offset + 2U >= cloud.interleaved.size) {
        return false;
      }
      *x = cloud.interleaved.data[offset];
      *y = cloud.interleaved.data[offset + 1U];
      *z = cloud.interleaved.data[offset + 2U];
      return true;
    }
    case CloudLayout::kXyzF32SoA:
    case CloudLayout::kXyziF32SoA:
      if (cloud.x.data == nullptr || cloud.y.data == nullptr ||
          cloud.z.data == nullptr || index >= cloud.x.size ||
          index >= cloud.y.size || index >= cloud.z.size) {
        return false;
      }
      *x = cloud.x.data[index];
      *y = cloud.y.data[index];
      *z = cloud.z.data[index];
      return true;
  }
  return false;
}

bool IdentityAtLeast(
    std::uint64_t epoch,
    std::uint64_t sequence,
    std::uint64_t expected_epoch,
    std::uint64_t expected_sequence) {
  return epoch > expected_epoch ||
      (epoch == expected_epoch && sequence >= expected_sequence);
}

std::uint64_t ColumnKey(float x, float y, float cell_size) {
  const auto ix = static_cast<std::int32_t>(std::floor(x / cell_size));
  const auto iy = static_cast<std::int32_t>(std::floor(y / cell_size));
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32U) |
      static_cast<std::uint32_t>(iy);
}

std::string UpperAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::toupper(character));
  });
  return value;
}

Snapshot::CollisionLayer BuildCollisionLayer(
    layers::RollingInflatedSnapshot occupancy) {
  occupancy.Validate();
  Snapshot::CollisionLayer layer;
  layer.generation = occupancy.generation;
  layer.resolution_m = occupancy.resolution_m;
  layer.size_x = occupancy.size_x;
  layer.size_y = occupancy.size_y;
  layer.size_z = occupancy.size_z;
  layer.min_x_m = occupancy.origin_x_m;
  layer.min_y_m = occupancy.origin_y_m;
  layer.min_z_m = occupancy.origin_z_m;
  layer.max_x_m = occupancy.origin_x_m +
      static_cast<float>(occupancy.size_x) * occupancy.resolution_m;
  layer.max_y_m = occupancy.origin_y_m +
      static_cast<float>(occupancy.size_y) * occupancy.resolution_m;
  layer.max_z_m = occupancy.origin_z_m +
      static_cast<float>(occupancy.size_z) * occupancy.resolution_m;
  layer.occupied_cells = occupancy.occupied_cells;
  layer.complete = true;
  layer.occupied_bits = std::move(occupancy.occupied_bits);
  return layer;
}

}  // namespace

LiveMapEngine::LiveMapEngine(Config config)
    : config_(std::move(config)),
      voxel_(config_.voxel),
      occupancy_(config_.occupancy),
      accumulated_(config_.accumulated) {
  if (config_.max_points_per_observation == 0U ||
      !IsFinite(config_.min_range_m) || !IsFinite(config_.max_range_m) ||
      config_.min_range_m < 0.0F ||
      config_.max_range_m <= config_.min_range_m ||
      !IsFinite(config_.min_height_from_sensor_m) ||
      !IsFinite(config_.max_height_from_sensor_m) ||
      config_.min_height_from_sensor_m > config_.max_height_from_sensor_m ||
      !IsFinite(config_.column_carving_min_height_from_sensor_m) ||
      !IsFinite(config_.column_carving_max_height_from_sensor_m) ||
      config_.column_carving_min_height_from_sensor_m >
          config_.column_carving_max_height_from_sensor_m ||
      !IsFinite(config_.occupancy_min_height_from_sensor_m) ||
      !IsFinite(config_.occupancy_max_height_from_sensor_m) ||
      config_.occupancy_min_height_from_sensor_m >
          config_.occupancy_max_height_from_sensor_m ||
      config_.decay_period.count() <= 0 ||
      config_.stale_after.count() <= 0 ||
      !IsFinite(config_.accumulated_decay_factor) ||
      config_.accumulated_decay_factor < 0.0F ||
      config_.accumulated_decay_factor > 1.0F ||
      !IsFinite(config_.accumulated_snapshot_radius_m) ||
      config_.accumulated_snapshot_radius_m <= 0.0F ||
      !IsFinite(config_.accumulated_snapshot_min_z_from_sensor_m) ||
      !IsFinite(config_.accumulated_snapshot_max_z_from_sensor_m) ||
      config_.accumulated_snapshot_min_z_from_sensor_m >
          config_.accumulated_snapshot_max_z_from_sensor_m ||
      config_.max_accumulated_snapshot_cells == 0U ||
      !IsFinite(config_.voxel_snapshot_radius_m) ||
      config_.voxel_snapshot_radius_m <= 0.0F ||
      !IsFinite(config_.voxel_snapshot_min_z_from_sensor_m) ||
      !IsFinite(config_.voxel_snapshot_max_z_from_sensor_m) ||
      config_.voxel_snapshot_min_z_from_sensor_m >
          config_.voxel_snapshot_max_z_from_sensor_m ||
      config_.max_voxel_snapshot_points == 0U) {
    throw std::invalid_argument("mapd LiveMapEngine configuration is invalid");
  }
  snapshot_.frame_id = "map";
}

LiveMapEngine::~LiveMapEngine() {
  Stop();
}

void LiveMapEngine::Start() {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (running_) {
    return;
  }
  stop_requested_ = false;
  running_ = true;
  worker_ = std::thread(&LiveMapEngine::Run, this);
}

void LiveMapEngine::Stop() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!running_) {
      return;
    }
    stop_requested_ = true;
  }
  queue_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

SubmitResult LiveMapEngine::Submit(Observation observation) {
  std::string reason;
  if (!ValidateObservation(observation, config_, &reason)) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    ++invalid_observations_;
    return {SubmitCode::kInvalid, std::move(reason)};
  }

  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (!running_ || stop_requested_) {
    return {SubmitCode::kStopped, "mapd engine is not running"};
  }
  if (observation.reset_epoch < highest_epoch_ ||
      (observation.reset_epoch == highest_epoch_ &&
       observation.sequence <= highest_sequence_)) {
    ++stale_observations_;
    return {SubmitCode::kStale, "observation identity is stale or duplicate"};
  }

  const bool replaced = has_pending_;
  pending_ = std::move(observation);
  has_pending_ = true;
  highest_epoch_ = pending_.reset_epoch;
  highest_sequence_ = pending_.sequence;
  ++accepted_observations_;
  if (replaced) {
    ++replaced_observations_;
  }
  queue_cv_.notify_one();
  return {
      replaced ? SubmitCode::kReplacedPending : SubmitCode::kAccepted,
      replaced ? "newer observation replaced the pending sample" : ""};
}

State LiveMapEngine::GetState() const {
  const std::int64_t now_ns = SteadyTimeNs();
  QueueState queue;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue = QueueStateLocked();
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  return BuildStateLocked(now_ns, queue);
}

Snapshot LiveMapEngine::GetSnapshot() const {
  std::lock_guard<std::mutex> lock(data_mutex_);
  EnsureCompleteSnapshotLocked();
  return snapshot_;
}

EngineView LiveMapEngine::GetView(SnapshotDetail detail) const {
  const std::int64_t now_ns = SteadyTimeNs();
  QueueState queue;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue = QueueStateLocked();
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (detail == SnapshotDetail::kRealtime) {
    EnsureRealtimeSnapshotLocked();
  } else {
    EnsureCompleteSnapshotLocked();
  }
  EngineView view;
  view.state = BuildStateLocked(now_ns, queue);
  view.snapshot = detail == SnapshotDetail::kRealtime
      ? RealtimeSnapshotLocked()
      : snapshot_;
  return view;
}

LiveMapEngine::QueueState LiveMapEngine::QueueStateLocked() const {
  return {
      running_,
      stop_requested_,
      has_pending_,
      accepted_observations_,
      replaced_observations_,
      stale_observations_,
      invalid_observations_,
  };
}

State LiveMapEngine::BuildStateLocked(std::int64_t now_ns, const QueueState& queue) const {
  State state;
  state.running = queue.running && !queue.stop_requested;
  state.live = state.running && last_processed_steady_ns_ > 0 &&
      now_ns - last_processed_steady_ns_ <=
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              config_.stale_after)
              .count();
  state.extended_layers_enabled = config_.build_extended_layers;
  state.reset_epoch = processed_epoch_;
  state.sequence = processed_sequence_;
  state.generation = generation_;
  state.realtime_snapshot_generation = realtime_snapshot_generation_;
  state.complete_snapshot_generation = complete_snapshot_generation_;
  state.realtime_snapshot_builds = realtime_snapshot_builds_;
  state.complete_snapshot_builds = complete_snapshot_builds_;
  state.accepted_observations = queue.accepted_observations;
  state.processed_observations = processed_observations_;
  state.replaced_observations = queue.replaced_observations;
  state.stale_observations = queue.stale_observations;
  state.invalid_observations = queue.invalid_observations;
  state.epoch_resets = epoch_resets_;
  state.queue_depth = queue.has_pending ? 1U : 0U;
  state.live_points = snapshot_.live_cloud.point_count;
  const bool realtime_current =
      generation_ > 0U && realtime_snapshot_generation_ == generation_;
  const bool complete_current =
      generation_ > 0U && complete_snapshot_generation_ == generation_;
  state.voxel_points =
      realtime_current ? snapshot_.voxel_cloud.point_count : 0U;
  state.voxel_cells = voxel_total_cells_;
  state.voxel_snapshot_omitted_cells =
      realtime_current ? voxel_snapshot_omitted_cells_ : 0U;
  state.voxel_capacity_rejections = voxel_capacity_rejections_;
  state.accumulated_cells = accumulated_total_cells_;
  state.accumulated_snapshot_cells =
      complete_current ? snapshot_.accumulated_cloud.Size() : 0U;
  state.accumulated_capacity_rejections =
      accumulated_capacity_rejections_;
  state.capacity_limited = capacity_limited_ ||
      (realtime_current && !snapshot_.collision.complete);
  state.pose_quality = pose_quality_;
  state.pose_state = pose_state_;
  state.pose_reason = pose_reason_;
  state.last_error = last_error_;
  return state;
}

bool LiveMapEngine::WaitUntilProcessed(
    std::uint64_t reset_epoch,
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) const {
  std::unique_lock<std::mutex> lock(data_mutex_);
  return processed_cv_.wait_for(lock, timeout, [&] {
    return IdentityAtLeast(
        processed_epoch_, processed_sequence_, reset_epoch, sequence);
  });
}

bool LiveMapEngine::ValidateObservation(
    const Observation& observation,
    const Config& config,
    std::string* reason) {
  const auto fail = [reason](const char* message) {
    if (reason != nullptr) {
      *reason = message;
    }
    return false;
  };
  if (observation.reset_epoch == 0U || observation.sequence == 0U) {
    return fail("reset_epoch and sequence must be non-zero");
  }
  if (observation.stamp_ns <= 0 || observation.map_frame.empty() ||
      observation.sensor_frame.empty()) {
    return fail("timestamp, map frame, and sensor frame are required");
  }
  const Pose& pose = observation.map_sensor;
  if (!IsFinite(pose.x) || !IsFinite(pose.y) || !IsFinite(pose.z) ||
      !IsFinite(pose.qx) || !IsFinite(pose.qy) || !IsFinite(pose.qz) ||
      !IsFinite(pose.qw) || !IsFinite(observation.sensor_origin_x_m) ||
      !IsFinite(observation.sensor_origin_y_m) ||
      !IsFinite(observation.sensor_origin_z_m) ||
      !IsFinite(observation.pose_quality)) {
    return fail("observation pose contains a non-finite value");
  }
  const double quaternion_norm =
      pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz +
      pose.qw * pose.qw;
  if (quaternion_norm < 1.0e-12) {
    return fail("map_sensor quaternion is invalid");
  }
  const std::string pose_state = UpperAscii(observation.pose_state);
  if (pose_state.empty()) {
    return fail("pose_state is required");
  }
  if (pose_state != "MAPPING" && pose_state != "LOCALIZING" &&
      pose_state != "TRACKING" &&
      !(config.accept_degraded_pose && pose_state == "DEGRADED")) {
    return fail("pose_state is not safe for live map integration");
  }

  const OwnedPointCloud& cloud = observation.scan;
  if (cloud.point_count == 0U ||
      cloud.point_count > config.max_points_per_observation) {
    return fail("observation point count is outside the configured limit");
  }
  if (!cloud.frame_id.empty() && cloud.frame_id != observation.sensor_frame) {
    return fail("scan frame does not match sensor_frame");
  }
  if (cloud.stamp_ns > 0 && cloud.stamp_ns != observation.stamp_ns) {
    return fail("scan timestamp does not match observation timestamp");
  }

  const std::size_t count = cloud.point_count;
  switch (cloud.layout) {
    case CloudLayout::kXyzF32Interleaved:
      if (cloud.interleaved.size() != count * 3U) {
        return fail("XYZ scan payload length is invalid");
      }
      break;
    case CloudLayout::kXyziF32Interleaved:
      if (cloud.interleaved.size() != count * 4U) {
        return fail("XYZI scan payload length is invalid");
      }
      break;
    case CloudLayout::kXyzF32SoA:
      if (cloud.x.size() != count || cloud.y.size() != count ||
          cloud.z.size() != count) {
        return fail("XYZ SoA scan payload length is invalid");
      }
      break;
    case CloudLayout::kXyziF32SoA:
      if (cloud.x.size() != count || cloud.y.size() != count ||
          cloud.z.size() != count || cloud.intensity.size() != count) {
        return fail("XYZI SoA scan payload length is invalid");
      }
      break;
  }
  if (reason != nullptr) {
    reason->clear();
  }
  return true;
}

OwnedPointCloud LiveMapEngine::TransformObservation(
    const Observation& observation,
    const Config& config) {
  const Pose& pose = observation.map_sensor;
  const double inverse_norm = 1.0 / std::sqrt(
      pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz +
      pose.qw * pose.qw);
  const double qx = pose.qx * inverse_norm;
  const double qy = pose.qy * inverse_norm;
  const double qz = pose.qz * inverse_norm;
  const double qw = pose.qw * inverse_norm;

  const double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
  const double r01 = 2.0 * (qx * qy - qz * qw);
  const double r02 = 2.0 * (qx * qz + qy * qw);
  const double r10 = 2.0 * (qx * qy + qz * qw);
  const double r11 = 1.0 - 2.0 * (qx * qx + qz * qz);
  const double r12 = 2.0 * (qy * qz - qx * qw);
  const double r20 = 2.0 * (qx * qz - qy * qw);
  const double r21 = 2.0 * (qy * qz + qx * qw);
  const double r22 = 1.0 - 2.0 * (qx * qx + qy * qy);

  const float min_range_sq = config.min_range_m * config.min_range_m;
  const float max_range_sq = config.max_range_m * config.max_range_m;
  OwnedPointCloud transformed;
  transformed.frame_id = observation.map_frame;
  transformed.stamp_ns = observation.stamp_ns;
  transformed.layout = CloudLayout::kXyzF32Interleaved;
  transformed.interleaved.reserve(observation.scan.point_count * 3U);

  const PointCloudView source = observation.scan.View();
  for (std::size_t index = 0U; index < source.point_count; ++index) {
    float sx = 0.0F;
    float sy = 0.0F;
    float sz = 0.0F;
    if (!ReadPoint(source, index, &sx, &sy, &sz) ||
        !IsFinite(sx) || !IsFinite(sy) || !IsFinite(sz)) {
      continue;
    }
    const float mx = static_cast<float>(
        r00 * sx + r01 * sy + r02 * sz + pose.x);
    const float my = static_cast<float>(
        r10 * sx + r11 * sy + r12 * sz + pose.y);
    const float mz = static_cast<float>(
        r20 * sx + r21 * sy + r22 * sz + pose.z);
    const float dx = mx - observation.sensor_origin_x_m;
    const float dy = my - observation.sensor_origin_y_m;
    const float dz = mz - observation.sensor_origin_z_m;
    const float range_sq = dx * dx + dy * dy + dz * dz;
    if (range_sq < min_range_sq || range_sq > max_range_sq ||
        dz < config.min_height_from_sensor_m ||
        dz > config.max_height_from_sensor_m) {
      continue;
    }
    transformed.interleaved.push_back(mx);
    transformed.interleaved.push_back(my);
    transformed.interleaved.push_back(mz);
  }
  transformed.point_count = transformed.interleaved.size() / 3U;
  return transformed;
}

layers::Grid2D LiveMapEngine::ProjectOccupancy(
    const layers::RollingOccupancySnapshot& occupancy,
    float sensor_z_m,
    const Config& config) {
  occupancy.Validate();
  layers::Grid2D grid = layers::makeGrid2D(
      occupancy.size_y,
      occupancy.size_x,
      occupancy.resolution_m,
      occupancy.origin_x_m,
      occupancy.origin_y_m,
      -1.0F);
  const float min_z =
      sensor_z_m + config.occupancy_min_height_from_sensor_m;
  const float max_z =
      sensor_z_m + config.occupancy_max_height_from_sensor_m;
  const float inverse_resolution = 1.0F / occupancy.resolution_m;
  const std::int32_t min_layer = std::clamp(
      static_cast<std::int32_t>(std::ceil(
          (min_z - occupancy.origin_z_m) * inverse_resolution - 0.5F)),
      0,
      occupancy.size_z);
  const std::int32_t max_layer_exclusive = std::clamp(
      static_cast<std::int32_t>(std::floor(
          (max_z - occupancy.origin_z_m) * inverse_resolution - 0.5F)) + 1,
      0,
      occupancy.size_z);
  for (std::int32_t y = 0; y < occupancy.size_y; ++y) {
    for (std::int32_t x = 0; x < occupancy.size_x; ++x) {
      bool observed_free = false;
      bool occupied = false;
      for (std::int32_t z = min_layer; z < max_layer_exclusive; ++z) {
        const auto state = static_cast<layers::OccupancyState>(
            occupancy.state[occupancy.Index(x, y, z)]);
        occupied = occupied || state == layers::OccupancyState::kOccupied;
        observed_free =
            observed_free || state == layers::OccupancyState::kFree;
      }
      const std::size_t index =
          static_cast<std::size_t>(grid.index(y, x));
      grid.data[index] = occupied ? 100.0F : (observed_free ? 0.0F : -1.0F);
    }
  }
  return grid;
}

layers::ElevationMapResult LiveMapEngine::ProjectElevation(
    const PointCloudView& cloud,
    const layers::Grid2D& geometry) {
  geometry.validate("mapd elevation geometry");
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  layers::ElevationMapResult result;
  result.minZ = layers::makeGrid2D(
      geometry.rows,
      geometry.cols,
      geometry.resolution,
      geometry.originX,
      geometry.originY,
      inf);
  result.maxZ = layers::makeGrid2D(
      geometry.rows,
      geometry.cols,
      geometry.resolution,
      geometry.originX,
      geometry.originY,
      -inf);
  result.clearance = layers::makeGrid2D(
      geometry.rows,
      geometry.cols,
      geometry.resolution,
      geometry.originX,
      geometry.originY,
      nan);
  result.valid.assign(
      static_cast<std::size_t>(geometry.rows * geometry.cols), 0U);

  for (std::size_t point = 0U; point < cloud.point_count; ++point) {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    if (!ReadPoint(cloud, point, &x, &y, &z) ||
        !IsFinite(x) || !IsFinite(y) || !IsFinite(z)) {
      continue;
    }
    const int col = static_cast<int>(
        std::floor((x - geometry.originX) / geometry.resolution));
    const int row = static_cast<int>(
        std::floor((y - geometry.originY) / geometry.resolution));
    if (row < 0 || row >= geometry.rows ||
        col < 0 || col >= geometry.cols) {
      continue;
    }
    const std::size_t index =
        static_cast<std::size_t>(geometry.index(row, col));
    result.valid[index] = 1U;
    result.minZ.data[index] = std::min(result.minZ.data[index], z);
    result.maxZ.data[index] = std::max(result.maxZ.data[index], z);
  }
  for (std::size_t index = 0U; index < result.valid.size(); ++index) {
    if (result.valid[index] == 0U) {
      result.minZ.data[index] = nan;
      result.maxZ.data[index] = nan;
      continue;
    }
    result.clearance.data[index] =
        result.maxZ.data[index] - result.minZ.data[index];
  }
  return result;
}

void LiveMapEngine::Run() {
  auto next_decay = std::chrono::steady_clock::now() + config_.decay_period;
  for (;;) {
    std::optional<Observation> observation;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_until(lock, next_decay, [&] {
        return stop_requested_ || has_pending_;
      });
      if (stop_requested_) {
        running_ = false;
        has_pending_ = false;
        break;
      }
      if (has_pending_) {
        observation.emplace(std::move(pending_));
        pending_ = {};
        has_pending_ = false;
      }
    }

    if (observation.has_value()) {
      try {
        Process(std::move(*observation));
      } catch (const std::exception& error) {
        {
          std::lock_guard<std::mutex> queue_lock(queue_mutex_);
          ++invalid_observations_;
        }
        {
          std::lock_guard<std::mutex> data_lock(data_mutex_);
          last_error_ = error.what();
        }
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_decay) {
      try {
        Decay(SteadyTimeNs());
      } catch (const std::exception& error) {
        std::lock_guard<std::mutex> data_lock(data_mutex_);
        last_error_ = error.what();
      }
      do {
        next_decay += config_.decay_period;
      } while (next_decay <= now);
    }
  }
  processed_cv_.notify_all();
}

void LiveMapEngine::Process(Observation observation) {
  OwnedPointCloud transformed = TransformObservation(observation, config_);
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (processed_epoch_ != observation.reset_epoch) {
    ResetForEpoch(observation);
  }

  MapCloudFrame frame;
  frame.cloud = transformed.View();
  frame.decay_stamp_ns = SteadyTimeNs();
  frame.sensor_origin_x_m = observation.sensor_origin_x_m;
  frame.sensor_origin_y_m = observation.sensor_origin_y_m;
  frame.sensor_origin_z_m = observation.sensor_origin_z_m;
  frame.column_carving_z_range_enabled = true;
  frame.column_carving_min_z_m =
      observation.sensor_origin_z_m +
      config_.column_carving_min_height_from_sensor_m;
  frame.column_carving_max_z_m =
      observation.sensor_origin_z_m +
      config_.column_carving_max_height_from_sensor_m;
  frame.incremental = true;
  if (config_.build_extended_layers) {
    voxel_.Update(frame);
    const auto voxel_stats = voxel_.LastStats();
    voxel_total_cells_ = voxel_.VoxelCount();
    voxel_capacity_rejections_ += voxel_stats.capacity_rejected_voxels;
    capacity_limited_ =
        capacity_limited_ || voxel_stats.capacity_rejected_voxels > 0U;
  }
  occupancy_.Update(frame);

  if (config_.build_extended_layers) {
    accumulated_.SetFrame(observation.map_frame);
    accumulated_.SetStampNs(observation.stamp_ns);
  }
  if (config_.build_extended_layers && transformed.point_count > 0U) {
    if (config_.accumulated_column_carving) {
      std::unordered_set<std::uint64_t> columns;
      std::vector<float> columns_xy;
      columns.reserve(transformed.point_count);
      columns_xy.reserve(transformed.point_count * 2U);
      for (std::size_t point = 0U; point < transformed.point_count; ++point) {
        const float x = transformed.interleaved[point * 3U];
        const float y = transformed.interleaved[point * 3U + 1U];
        const auto key = ColumnKey(x, y, config_.accumulated.cell_size_m);
        if (columns.insert(key).second) {
          columns_xy.push_back(x);
          columns_xy.push_back(y);
        }
      }
      static_cast<void>(accumulated_.ClearColumns(
          columns_xy.data(),
          columns_xy.size() / 2U,
          observation.sensor_origin_z_m +
              config_.column_carving_min_height_from_sensor_m,
          observation.sensor_origin_z_m +
              config_.column_carving_max_height_from_sensor_m));
    }
    std::vector<float> origins(transformed.point_count * 3U);
    for (std::size_t point = 0U; point < transformed.point_count; ++point) {
      origins[point * 3U] = observation.sensor_origin_x_m;
      origins[point * 3U + 1U] = observation.sensor_origin_y_m;
      origins[point * 3U + 2U] = observation.sensor_origin_z_m;
    }
    const auto accumulated_stats = accumulated_.InsertRays(
        origins.data(),
        transformed.interleaved.data(),
        transformed.point_count,
        config_.max_range_m);
    accumulated_capacity_rejections_ +=
        accumulated_stats.capacity_rejections;
    capacity_limited_ =
        capacity_limited_ || accumulated_stats.capacity_rejections > 0U;
  }
  accumulated_total_cells_ =
      config_.build_extended_layers ? accumulated_.CellCount() : 0U;

  processed_epoch_ = observation.reset_epoch;
  processed_sequence_ = observation.sequence;
  ++processed_observations_;
  ++generation_;
  last_processed_steady_ns_ = frame.decay_stamp_ns;
  pose_quality_ = observation.pose_quality;
  pose_state_ = UpperAscii(observation.pose_state);
  pose_reason_ = observation.pose_reason;
  last_error_.clear();
  snapshot_.frame_id = observation.map_frame;
  snapshot_.stamp_ns = observation.stamp_ns;
  snapshot_.reset_epoch = observation.reset_epoch;
  snapshot_.sequence = observation.sequence;
  snapshot_.generation = generation_;
  snapshot_.map_sensor = observation.map_sensor;
  snapshot_.live_cloud = std::move(transformed);
  processed_cv_.notify_all();
}

void LiveMapEngine::Decay(std::int64_t now_ns) {
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (processed_epoch_ == 0U) {
    return;
  }
  const std::size_t voxel_before = voxel_total_cells_;
  if (config_.build_extended_layers) {
    voxel_.Decay();
    voxel_total_cells_ = voxel_.VoxelCount();
  }
  const std::size_t occupancy_changed = occupancy_.Decay(now_ns);
  const std::uint64_t accumulated_generation = accumulated_.Generation();
  if (config_.build_extended_layers && config_.accumulated_decay_factor < 1.0F) {
    static_cast<void>(
        accumulated_.Decay(config_.accumulated_decay_factor));
  }
  const bool accumulated_changed = config_.build_extended_layers &&
      accumulated_.Generation() != accumulated_generation;
  if (voxel_before == voxel_total_cells_ &&
      occupancy_changed == 0U && !accumulated_changed) {
    return;
  }
  accumulated_total_cells_ =
      config_.build_extended_layers ? accumulated_.CellCount() : 0U;
  ++generation_;
  snapshot_.generation = generation_;
}

void LiveMapEngine::ResetForEpoch(const Observation& observation) {
  voxel_.Reset();
  occupancy_.Reset(
      observation.map_frame,
      observation.sensor_origin_x_m,
      observation.sensor_origin_y_m,
      observation.sensor_origin_z_m,
      observation.stamp_ns);
  accumulated_.Reset();
  accumulated_.SetFrame(observation.map_frame);
  accumulated_.SetStampNs(observation.stamp_ns);
  accumulated_total_cells_ = 0U;
  voxel_total_cells_ = 0U;
  voxel_snapshot_omitted_cells_ = 0U;
  voxel_capacity_rejections_ = 0U;
  accumulated_capacity_rejections_ = 0U;
  capacity_limited_ = false;
  pose_quality_ = observation.pose_quality;
  pose_state_ = UpperAscii(observation.pose_state);
  pose_reason_ = observation.pose_reason;
  snapshot_ = {};
  snapshot_.frame_id = observation.map_frame;
  realtime_snapshot_generation_ = 0U;
  complete_snapshot_generation_ = 0U;
  processed_epoch_ = observation.reset_epoch;
  processed_sequence_ = 0U;
  ++epoch_resets_;
}

void LiveMapEngine::EnsureRealtimeSnapshotLocked() const {
  if (generation_ == 0U || realtime_snapshot_generation_ == generation_) {
    return;
  }
  BuildRealtimeSnapshotLocked();
}

void LiveMapEngine::BuildRealtimeSnapshotLocked() const {
  if (config_.build_extended_layers) {
    layers::VoxelSnapshotRequest voxel_request;
    voxel_request.center_x_m = static_cast<float>(snapshot_.map_sensor.x);
    voxel_request.center_y_m = static_cast<float>(snapshot_.map_sensor.y);
    voxel_request.radius_m = config_.voxel_snapshot_radius_m;
    voxel_request.min_z_m =
        static_cast<float>(snapshot_.map_sensor.z) +
        config_.voxel_snapshot_min_z_from_sensor_m;
    voxel_request.max_z_m =
        static_cast<float>(snapshot_.map_sensor.z) +
        config_.voxel_snapshot_max_z_from_sensor_m;
    voxel_request.max_points = config_.max_voxel_snapshot_points;
    layers::VoxelSnapshotStats voxel_stats;
    snapshot_.voxel_cloud = voxel_.SnapshotCloud(voxel_request, &voxel_stats);
    voxel_snapshot_omitted_cells_ = voxel_stats.omitted_voxels;
  } else {
    snapshot_.voxel_cloud = {};
    voxel_snapshot_omitted_cells_ = 0U;
  }
  snapshot_.collision = BuildCollisionLayer(occupancy_.InflatedSnapshot());
  realtime_snapshot_generation_ = generation_;
  ++realtime_snapshot_builds_;
}

void LiveMapEngine::EnsureCompleteSnapshotLocked() const {
  if (generation_ == 0U || complete_snapshot_generation_ == generation_) {
    return;
  }
  if (!config_.build_extended_layers) {
    if (realtime_snapshot_generation_ != generation_) {
      BuildRealtimeSnapshotLocked();
    }
    snapshot_.accumulated_cloud = {};
    snapshot_.occupancy = {};
    snapshot_.elevation = {};
    snapshot_.esdf = {};
    complete_snapshot_generation_ = generation_;
    ++complete_snapshot_builds_;
    return;
  }
  const auto occupancy_snapshot = occupancy_.Snapshot();
  if (realtime_snapshot_generation_ != generation_) {
    BuildRealtimeSnapshotLocked();
  }
  BlockGridRoi accumulated_roi;
  accumulated_roi.enabled = true;
  accumulated_roi.min_x_m =
      static_cast<float>(snapshot_.map_sensor.x) -
      config_.accumulated_snapshot_radius_m;
  accumulated_roi.max_x_m =
      static_cast<float>(snapshot_.map_sensor.x) +
      config_.accumulated_snapshot_radius_m;
  accumulated_roi.min_y_m =
      static_cast<float>(snapshot_.map_sensor.y) -
      config_.accumulated_snapshot_radius_m;
  accumulated_roi.max_y_m =
      static_cast<float>(snapshot_.map_sensor.y) +
      config_.accumulated_snapshot_radius_m;
  accumulated_roi.min_z_m =
      static_cast<float>(snapshot_.map_sensor.z) +
      config_.accumulated_snapshot_min_z_from_sensor_m;
  accumulated_roi.max_z_m =
      static_cast<float>(snapshot_.map_sensor.z) +
      config_.accumulated_snapshot_max_z_from_sensor_m;
  accumulated_roi.max_cells =
      config_.max_accumulated_snapshot_cells;
  snapshot_.accumulated_cloud = accumulated_.Snapshot(accumulated_roi);
  snapshot_.occupancy = ProjectOccupancy(
      occupancy_snapshot,
      static_cast<float>(snapshot_.map_sensor.z),
      config_);
  snapshot_.elevation =
      ProjectElevation(snapshot_.voxel_cloud.View(), snapshot_.occupancy);

  layers::Grid2D collision_cost = snapshot_.occupancy;
  for (float& value : collision_cost.data) {
    if (value < 0.0F) {
      value = 100.0F;
    }
  }
  snapshot_.esdf = layers::computeEsdf(collision_cost, 50.0F);
  complete_snapshot_generation_ = generation_;
  ++complete_snapshot_builds_;
}

Snapshot LiveMapEngine::RealtimeSnapshotLocked() const {
  Snapshot realtime;
  realtime.frame_id = snapshot_.frame_id;
  realtime.stamp_ns = snapshot_.stamp_ns;
  realtime.reset_epoch = snapshot_.reset_epoch;
  realtime.sequence = snapshot_.sequence;
  realtime.generation = snapshot_.generation;
  realtime.map_sensor = snapshot_.map_sensor;
  realtime.live_cloud = snapshot_.live_cloud;
  realtime.voxel_cloud = snapshot_.voxel_cloud;
  realtime.collision = snapshot_.collision;
  return realtime;
}

}  // namespace lingtu::maps::mapd
