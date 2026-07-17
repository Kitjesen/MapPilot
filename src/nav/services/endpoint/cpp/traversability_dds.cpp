#include "message/cpp/dds_qos_profiles.hpp"
#include "message/cpp/dds_topics.hpp"
#include "message/cpp/snapshot_file.hpp"
#include "lingtu/maps/layers/grid.hpp"
#include "dds_drain_policy.hpp"
#include "frame_transform.hpp"
#include "grid_inflation.hpp"
#include "observed_free_cache.hpp"
#include "observed_safety_grid.hpp"
#include "point_cloud_layout.hpp"
#include "safety_grid_probe.hpp"
#include "terrain_risk.hpp"
#include "transform_buffer.hpp"
#include "traversability_geometry.hpp"
#include "nav_kernel/dynamic_clear_core.hpp"
#include "nav_kernel/terrain_core.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

namespace map_layers = lingtu::maps::layers;
namespace nav_endpoint = lingtu::nav::endpoint;

std::atomic_bool g_running{true};

constexpr double kNativeTeleopStraightProbeHorizonM = 1.2;
constexpr double kNativeTeleopTerrainMinimumStepM = 0.20;

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

using SteadyClock = std::chrono::steady_clock;

double elapsedMs(
    SteadyClock::time_point start,
    SteadyClock::time_point end = SteadyClock::now()) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

void fillHeader(lingtu_dds_Header& header, double stamp_s, const char* frame_id) {
  if (!std::isfinite(stamp_s) || stamp_s <= 0.0) {
    stamp_s = nowSeconds();
  }
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char*>(frame_id);
}

dds_entity_t checked(dds_return_t value, const char* what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void logDdsError(dds_return_t value, const char* what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

template <typename T, typename Handler>
void drainReader(
    dds_entity_t reader,
    const dds_topic_descriptor_t& descriptor,
    Handler&& handler,
    nav_endpoint::DdsDrainBudget budget = {8, 1}) {
  constexpr std::size_t kMaxSamples = 8;
  budget.batch_size = std::min(budget.batch_size, kMaxSamples);
  nav_endpoint::drainBatches(
      budget,
      [&](std::size_t capacity) -> std::ptrdiff_t {
        void* samples[kMaxSamples];
        dds_sample_info_t infos[kMaxSamples];
        for (auto& sample : samples) {
          sample = dds_alloc(sizeof(T));
          std::memset(sample, 0, sizeof(T));
        }
        const dds_return_t count =
            dds_take(reader, samples, infos, capacity, capacity);
        if (count >= 0) {
          for (dds_return_t i = 0; i < count; ++i) {
            if (infos[i].valid_data) {
              handler(*static_cast<T*>(samples[i]));
            }
          }
        } else {
          logDdsError(count, "dds_take");
        }
        for (auto& sample : samples) {
          dds_sample_free(sample, &descriptor, DDS_FREE_ALL);
        }
        return static_cast<std::ptrdiff_t>(count);
      }
  );
}

std::string jsonEscape(const std::string& input) {
  std::string out;
  out.reserve(input.size() + 8);
  for (const char value : input) {
    switch (value) {
      case '\\':
        out += "\\\\";
        break;
      case '"':
        out += "\\\"";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += value;
        break;
    }
  }
  return out;
}

std::string headerFrameId(const lingtu_dds_Header& header) {
  return header.frame_id == nullptr ? std::string{} : std::string(header.frame_id);
}

nav_endpoint::RigidTransform transformFromOdometry(const lingtu_dds_Odometry& msg) {
  return nav_endpoint::rigidTransformFromOdometry(msg);
}

std::optional<nav_endpoint::RigidTransform> mapBodyTransform(
    const nav_endpoint::RigidTransform& pose,
    const std::string& pose_frame,
    const std::optional<nav_endpoint::RigidTransform>& map_odom) {
  if (!pose.valid) {
    return std::nullopt;
  }
  if (pose_frame == "map") {
    return pose;
  }
  if (pose_frame == "odom" && map_odom && map_odom->valid) {
    return nav_endpoint::composeTransforms(*map_odom, pose);
  }
  return std::nullopt;
}

struct FieldOffsets {
  int x{-1};
  int y{-1};
  int z{-1};
  bool invalid{false};
};

FieldOffsets fieldOffsets(const lingtu_dds_PointCloud2& msg) {
  FieldOffsets offsets;
  if (msg.fields._length > 0 && msg.fields._buffer == nullptr) {
    offsets.invalid = true;
    return offsets;
  }
  for (std::uint32_t i = 0; i < msg.fields._length; ++i) {
    const auto& field = msg.fields._buffer[i];
    const std::string name = field.name ? field.name : "";
    if (name != "x" && name != "y" && name != "z") {
      continue;
    }
    if (!nav_endpoint::pointFieldIsScalarFloat32(
            field.datatype,
            field.count,
            field.offset,
            msg.point_step)) {
      offsets.invalid = true;
      continue;
    }
    if (name == "x") {
      offsets.x = static_cast<int>(field.offset);
    } else if (name == "y") {
      offsets.y = static_cast<int>(field.offset);
    } else if (name == "z") {
      offsets.z = static_cast<int>(field.offset);
    }
  }
  return offsets;
}

float readFloat(const std::uint8_t* data) {
  float value = 0.0f;
  std::memcpy(&value, data, sizeof(float));
  return value;
}

void writeFloat(std::vector<std::uint8_t>& data, std::size_t offset, float value) {
  std::memcpy(data.data() + offset, &value, sizeof(float));
}

struct MapCloudResult {
  std::vector<float> xyz;
  std::string input_frame;
  std::string reason{"not_processed"};
  bool accepted{false};
};

MapCloudResult cloudToMapXyz(
    const lingtu_dds_PointCloud2& msg,
    const std::optional<nav_endpoint::RigidTransform>& map_body,
    const std::optional<nav_endpoint::RigidTransform>& map_odom,
    std::size_t max_points) {
  MapCloudResult result;
  result.input_frame = headerFrameId(msg.header);
  const FieldOffsets offsets = fieldOffsets(msg);
  if (offsets.invalid || offsets.x < 0 || offsets.y < 0 || offsets.z < 0 ||
      msg.data._buffer == nullptr || msg.is_bigendian) {
    result.reason = "invalid_cloud_layout";
    return result;
  }
  const std::size_t rows = std::max<std::uint32_t>(1, msg.height);
  const std::size_t cols = msg.width;
  const std::size_t point_step = msg.point_step;
  const std::size_t row_step = msg.row_step > 0 ? msg.row_step : point_step * cols;
  if (!nav_endpoint::pointCloudStorageIsValid(
          msg.width,
          msg.height,
          msg.point_step,
          msg.row_step,
          msg.data._length)) {
    result.reason = "invalid_cloud_size";
    return result;
  }
  const std::size_t count = rows * cols;
  if (count == 0) {
    result.reason = "empty_cloud";
    result.accepted = true;
    return result;
  }
  const std::size_t stride = max_points > 0 && count > max_points
      ? static_cast<std::size_t>(std::ceil(static_cast<double>(count) / max_points))
      : 1;
  std::optional<nav_endpoint::RigidTransform> transform;
  if (result.input_frame == "map") {
    // No transform is required.
  } else if (result.input_frame == "odom") {
    if (!map_odom || !map_odom->valid) {
      result.reason = "map_odom_tf_missing";
      return result;
    }
    transform = map_odom;
  } else if (
      result.input_frame == "body" || result.input_frame == "base_link" ||
      result.input_frame == "base") {
    if (!map_body || !map_body->valid) {
      result.reason = "map_body_pose_missing";
      return result;
    }
    transform = map_body;
  } else {
    result.reason = result.input_frame.empty() ? "cloud_frame_empty" : "cloud_frame_unsupported";
    return result;
  }
  result.xyz.reserve((count / stride + 1) * 3);
  for (std::size_t i = 0; i < count; i += stride) {
    const std::size_t row = i / cols;
    const std::size_t col = i % cols;
    const auto* base = msg.data._buffer + row * row_step + col * point_step;
    const float x = readFloat(base + offsets.x);
    const float y = readFloat(base + offsets.y);
    const float z = readFloat(base + offsets.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (!transform) {
      result.xyz.push_back(x);
      result.xyz.push_back(y);
      result.xyz.push_back(z);
    } else {
      const auto point = nav_endpoint::transformPoint(*transform, {x, y, z});
      result.xyz.push_back(static_cast<float>(point.x));
      result.xyz.push_back(static_cast<float>(point.y));
      result.xyz.push_back(static_cast<float>(point.z));
    }
  }
  result.reason = "accepted";
  result.accepted = true;
  return result;
}

struct CliConfig {
  int domain_id{0};
  double publish_hz{10.0};
  double slow_hz{1.0};
  double tick_hz{50.0};
  double tf_max_age_s{1.0};
  double cloud_pose_max_gap_s{0.10};
  double resolution{0.2};
  double radius{15.0};
  double z_min{-1.5};
  double z_max{1.2};
  double obstacle_min_z{0.10};
  double robot_radius{0.45};
  double observed_free_ttl_s{0.60};
  double terrain_soft_height_m{0.08};
  double terrain_hard_height_m{0.20};
  double terrain_soft_slope_deg{12.0};
  double terrain_hard_slope_deg{28.0};
  double sensor_offset_x_m{0.0};
  double sensor_offset_y_m{0.0};
  double sensor_offset_z_m{0.0};
  double terrain_decay_s{2.0};
  double terrain_no_decay_radius{4.0};
  double terrain_dis_ratio_z{0.2};
  double terrain_quantile{0.25};
  double vehicle_height{1.5};
  int terrain_min_block_points{10};
  bool terrain_clear_dy_obs{false};
  double terrain_min_dy_obs_dis{0.3};
  double terrain_min_dy_obs_angle{0.0};
  double terrain_min_dy_obs_rel_z{-0.5};
  double terrain_abs_dy_obs_rel_z_thre{0.2};
  double terrain_min_dy_obs_vfov{-16.0};
  double terrain_max_dy_obs_vfov{16.0};
  int terrain_min_dy_obs_point_num{1};
  bool terrain_no_data_obstacle{false};
  int terrain_no_data_block_skip_num{0};
  bool dynamic_clear{true};
  double dynamic_clear_voxel_size{0.20};
  double dynamic_clear_weak_ttl_s{0.80};
  double dynamic_clear_static_ttl_s{3.0};
  int dynamic_clear_static_min_hits{3};
  int dynamic_clear_static_min_frames{2};
  bool dynamic_clear_raycast{true};
  int dynamic_clear_raycast_min_frames{2};
  double dynamic_clear_raycast_max_range{6.0};
  std::size_t max_points{20000};
  std::size_t terrain_cache_max_points{80000};
  std::string status_file;
};

struct TimingDiagnostics {
  double loop_ms{0.0};
  double input_callbacks_ms{0.0};
  double cloud_convert_ms{0.0};
  double fast_occupancy_ms{0.0};
  double slow_terrain_ms{0.0};
  double terrain_core_ms{0.0};
  double dynamic_clear_ms{0.0};
  double terrain_pack_ms{0.0};
  double dds_write_ms{0.0};
  double sleep_ms{0.0};
  double overrun_ms{0.0};
};

struct SafetyGridDiagnostics {
  double origin_x{0.0};
  double origin_y{0.0};
  double alignment_residual_x{0.0};
  double alignment_residual_y{0.0};
  std::size_t total_cells{0};
  std::size_t observed_free_cache_cells{0};
  std::size_t observed_free_applied_cells{0};
  std::size_t observed_before_overlays_cells{0};
  std::size_t unknown_before_overlays_cells{0};
  std::size_t occupancy_source_cells{0};
  std::size_t fused_zero_cells{0};
  std::size_t fused_soft_cells{0};
  std::size_t fused_hard_cells{0};
};

std::string envOrEmpty(const char* name) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
}

bool parseBool(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value == "1" || value == "true" || value == "yes" || value == "on";
}

CliConfig parseArgs(int argc, char** argv) {
  CliConfig cfg;
  cfg.status_file = envOrEmpty("LINGTU_TRAVERSABILITY_STATUS_FILE");
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error("missing value for " + arg);
      }
      return argv[++i];
    };
    if (arg == "--domain-id") {
      cfg.domain_id = std::stoi(next());
    } else if (arg == "--publish-hz") {
      cfg.publish_hz = std::stod(next());
    } else if (arg == "--slow-hz") {
      cfg.slow_hz = std::stod(next());
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
    } else if (arg == "--tf-max-age-s") {
      cfg.tf_max_age_s = std::stod(next());
    } else if (arg == "--cloud-pose-max-gap-s") {
      cfg.cloud_pose_max_gap_s = std::stod(next());
    } else if (arg == "--resolution") {
      cfg.resolution = std::stod(next());
    } else if (arg == "--radius") {
      cfg.radius = std::stod(next());
    } else if (arg == "--z-min") {
      cfg.z_min = std::stod(next());
    } else if (arg == "--z-max") {
      cfg.z_max = std::stod(next());
    } else if (arg == "--obstacle-min-z") {
      cfg.obstacle_min_z = std::stod(next());
    } else if (arg == "--robot-radius") {
      cfg.robot_radius = std::stod(next());
    } else if (arg == "--observed-free-ttl-s") {
      cfg.observed_free_ttl_s = std::stod(next());
    } else if (arg == "--terrain-soft-height-m") {
      cfg.terrain_soft_height_m = std::stod(next());
    } else if (arg == "--terrain-hard-height-m") {
      cfg.terrain_hard_height_m = std::stod(next());
    } else if (arg == "--terrain-soft-slope-deg") {
      cfg.terrain_soft_slope_deg = std::stod(next());
    } else if (arg == "--terrain-hard-slope-deg") {
      cfg.terrain_hard_slope_deg = std::stod(next());
    } else if (arg == "--sensor-offset-x-m") {
      cfg.sensor_offset_x_m = std::stod(next());
    } else if (arg == "--sensor-offset-y-m") {
      cfg.sensor_offset_y_m = std::stod(next());
    } else if (arg == "--sensor-offset-z-m") {
      cfg.sensor_offset_z_m = std::stod(next());
    } else if (arg == "--terrain-decay-s") {
      cfg.terrain_decay_s = std::stod(next());
    } else if (arg == "--terrain-no-decay-radius") {
      cfg.terrain_no_decay_radius = std::stod(next());
    } else if (arg == "--terrain-min-block-points") {
      cfg.terrain_min_block_points = std::stoi(next());
    } else if (arg == "--terrain-quantile") {
      cfg.terrain_quantile = std::stod(next());
    } else if (arg == "--vehicle-height") {
      cfg.vehicle_height = std::stod(next());
    } else if (arg == "--terrain-cache-max-points") {
      cfg.terrain_cache_max_points = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--terrain-clear-dy-obs") {
      cfg.terrain_clear_dy_obs = parseBool(next());
    } else if (arg == "--terrain-min-dy-obs-dis") {
      cfg.terrain_min_dy_obs_dis = std::stod(next());
    } else if (arg == "--terrain-min-dy-obs-angle") {
      cfg.terrain_min_dy_obs_angle = std::stod(next());
    } else if (arg == "--terrain-min-dy-obs-rel-z") {
      cfg.terrain_min_dy_obs_rel_z = std::stod(next());
    } else if (arg == "--terrain-abs-dy-obs-rel-z-thre") {
      cfg.terrain_abs_dy_obs_rel_z_thre = std::stod(next());
    } else if (arg == "--terrain-min-dy-obs-vfov") {
      cfg.terrain_min_dy_obs_vfov = std::stod(next());
    } else if (arg == "--terrain-max-dy-obs-vfov") {
      cfg.terrain_max_dy_obs_vfov = std::stod(next());
    } else if (arg == "--terrain-min-dy-obs-point-num") {
      cfg.terrain_min_dy_obs_point_num = std::stoi(next());
    } else if (arg == "--terrain-no-data-obstacle") {
      cfg.terrain_no_data_obstacle = parseBool(next());
    } else if (arg == "--terrain-no-data-block-skip-num") {
      cfg.terrain_no_data_block_skip_num = std::stoi(next());
    } else if (arg == "--dynamic-clear") {
      cfg.dynamic_clear = parseBool(next());
    } else if (arg == "--dynamic-clear-voxel-size") {
      cfg.dynamic_clear_voxel_size = std::stod(next());
    } else if (arg == "--dynamic-clear-weak-ttl-s") {
      cfg.dynamic_clear_weak_ttl_s = std::stod(next());
    } else if (arg == "--dynamic-clear-static-ttl-s") {
      cfg.dynamic_clear_static_ttl_s = std::stod(next());
    } else if (arg == "--dynamic-clear-static-min-hits") {
      cfg.dynamic_clear_static_min_hits = std::stoi(next());
    } else if (arg == "--dynamic-clear-static-min-frames") {
      cfg.dynamic_clear_static_min_frames = std::stoi(next());
    } else if (arg == "--dynamic-clear-raycast") {
      cfg.dynamic_clear_raycast = parseBool(next());
    } else if (arg == "--dynamic-clear-raycast-min-frames") {
      cfg.dynamic_clear_raycast_min_frames = std::stoi(next());
    } else if (arg == "--dynamic-clear-raycast-max-range") {
      cfg.dynamic_clear_raycast_max_range = std::stod(next());
    } else if (arg == "--max-points") {
      cfg.max_points = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_traversability_dds [--domain-id N] [--publish-hz HZ] "
          "[--slow-hz HZ] [--tf-max-age-s S] [--cloud-pose-max-gap-s S] "
          "[--resolution M] [--radius M] [--max-points N] "
          "[--observed-free-ttl-s S] "
          "[--terrain-soft-height-m M] [--terrain-hard-height-m M] "
          "[--terrain-soft-slope-deg DEG] [--terrain-hard-slope-deg DEG] "
          "[--sensor-offset-x-m M] [--sensor-offset-y-m M] [--sensor-offset-z-m M] "
          "[--terrain-decay-s S] [--terrain-min-block-points N] "
          "[--terrain-quantile Q] [--terrain-cache-max-points N] "
          "[--terrain-clear-dy-obs BOOL] [--terrain-no-data-obstacle BOOL] "
          "[--dynamic-clear BOOL] [--dynamic-clear-weak-ttl-s S] "
          "[--dynamic-clear-raycast BOOL] "
          "[--status-file PATH]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.publish_hz = std::max(0.2, cfg.publish_hz);
  cfg.slow_hz = std::max(0.1, cfg.slow_hz);
  cfg.tick_hz = std::max(1.0, cfg.tick_hz);
  cfg.cloud_pose_max_gap_s = std::max(0.0, cfg.cloud_pose_max_gap_s);
  cfg.resolution = std::max(0.05, cfg.resolution);
  cfg.radius = std::max(1.0, cfg.radius);
  cfg.observed_free_ttl_s = std::max(0.1, cfg.observed_free_ttl_s);
  cfg.terrain_soft_height_m = std::max(0.0, cfg.terrain_soft_height_m);
  cfg.terrain_hard_height_m = std::max(
      cfg.terrain_soft_height_m + 0.01,
      cfg.terrain_hard_height_m);
  cfg.terrain_soft_slope_deg = std::max(0.0, cfg.terrain_soft_slope_deg);
  cfg.terrain_hard_slope_deg = std::max(
      cfg.terrain_soft_slope_deg + 1.0,
      cfg.terrain_hard_slope_deg);
  cfg.terrain_decay_s = std::max(0.1, cfg.terrain_decay_s);
  cfg.terrain_no_decay_radius = std::max(0.0, cfg.terrain_no_decay_radius);
  cfg.terrain_quantile = std::max(0.0, std::min(1.0, cfg.terrain_quantile));
  cfg.vehicle_height = std::max(0.1, cfg.vehicle_height);
  cfg.terrain_min_block_points = std::max(1, cfg.terrain_min_block_points);
  cfg.terrain_min_dy_obs_dis = std::max(0.0, cfg.terrain_min_dy_obs_dis);
  cfg.terrain_min_dy_obs_point_num = std::max(1, cfg.terrain_min_dy_obs_point_num);
  cfg.terrain_no_data_block_skip_num = std::max(0, cfg.terrain_no_data_block_skip_num);
  cfg.dynamic_clear_voxel_size = std::max(0.05, cfg.dynamic_clear_voxel_size);
  cfg.dynamic_clear_weak_ttl_s = std::max(0.05, cfg.dynamic_clear_weak_ttl_s);
  cfg.dynamic_clear_static_ttl_s =
      std::max(cfg.dynamic_clear_weak_ttl_s, cfg.dynamic_clear_static_ttl_s);
  cfg.dynamic_clear_static_min_hits = std::max(1, cfg.dynamic_clear_static_min_hits);
  cfg.dynamic_clear_static_min_frames = std::max(1, cfg.dynamic_clear_static_min_frames);
  cfg.dynamic_clear_raycast_min_frames = std::max(1, cfg.dynamic_clear_raycast_min_frames);
  cfg.dynamic_clear_raycast_max_range = std::max(0.0, cfg.dynamic_clear_raycast_max_range);
  cfg.terrain_cache_max_points = std::max<std::size_t>(cfg.max_points, cfg.terrain_cache_max_points);
  return cfg;
}

nav_kernel::TerrainParams terrainParamsFromConfig(const CliConfig& cfg) {
  nav_kernel::TerrainParams params;
  params.decayTime = cfg.terrain_decay_s;
  params.noDecayDis = cfg.terrain_no_decay_radius;
  params.terrainVoxelSize = 1.0;
  params.terrainVoxelHalfWidth =
      std::max(1, static_cast<int>(std::ceil(cfg.radius / params.terrainVoxelSize)));
  constexpr std::size_t kMergedTerrainVoxelCount = 11u * 11u;
  params.maxPointsPerVoxel = std::max<std::size_t>(
      32, cfg.terrain_cache_max_points / kMergedTerrainVoxelCount);
  params.maxStoredPoints = cfg.terrain_cache_max_points;
  params.planarVoxelSize = cfg.resolution;
  params.planarVoxelHalfWidth =
      std::max(1, static_cast<int>(std::ceil(cfg.radius / params.planarVoxelSize)));
  params.quantileZ = cfg.terrain_quantile;
  params.minBlockPointNum = cfg.terrain_min_block_points;
  params.vehicleHeight = cfg.vehicle_height;
  params.minRelZ = cfg.z_min;
  params.maxRelZ = cfg.z_max;
  params.disRatioZ = cfg.terrain_dis_ratio_z;
  params.clearDyObs = cfg.terrain_clear_dy_obs;
  params.minDyObsDis = cfg.terrain_min_dy_obs_dis;
  params.minDyObsAngle = cfg.terrain_min_dy_obs_angle;
  params.minDyObsRelZ = cfg.terrain_min_dy_obs_rel_z;
  params.absDyObsRelZThre = cfg.terrain_abs_dy_obs_rel_z_thre;
  params.minDyObsVFOV = cfg.terrain_min_dy_obs_vfov;
  params.maxDyObsVFOV = cfg.terrain_max_dy_obs_vfov;
  params.minDyObsPointNum = cfg.terrain_min_dy_obs_point_num;
  params.noDataObstacle = cfg.terrain_no_data_obstacle;
  params.noDataBlockSkipNum = cfg.terrain_no_data_block_skip_num;
  return params;
}

nav_kernel::DynamicClearParams dynamicClearParamsFromConfig(const CliConfig& cfg) {
  nav_kernel::DynamicClearParams params;
  params.enabled = cfg.dynamic_clear;
  params.voxelSize = cfg.dynamic_clear_voxel_size;
  params.weakTtlS = cfg.dynamic_clear_weak_ttl_s;
  params.staticTtlS = cfg.dynamic_clear_static_ttl_s;
  params.staticMinHits = static_cast<std::uint32_t>(cfg.dynamic_clear_static_min_hits);
  params.staticMinFrames = static_cast<std::uint32_t>(cfg.dynamic_clear_static_min_frames);
  params.raycastClearing = cfg.dynamic_clear_raycast;
  params.raycastClearMinFrames =
      static_cast<std::uint32_t>(cfg.dynamic_clear_raycast_min_frames);
  params.raycastMaxRange = cfg.dynamic_clear_raycast_max_range;
  params.maxRayCount = std::min<std::size_t>(512, cfg.max_points);
  return params;
}

std::vector<float> xyzToXyzi(const std::vector<float>& xyz) {
  std::vector<float> out;
  const std::size_t n = xyz.size() / 3;
  out.reserve(n * 4);
  for (std::size_t i = 0; i < n; ++i) {
    out.push_back(xyz[i * 3 + 0]);
    out.push_back(xyz[i * 3 + 1]);
    out.push_back(xyz[i * 3 + 2]);
    out.push_back(0.0f);
  }
  return out;
}

std::vector<float> limitXyzi(const std::vector<float>& xyzi, std::size_t max_points) {
  const std::size_t n = xyzi.size() / 4;
  if (max_points == 0 || n <= max_points) {
    return xyzi;
  }
  const std::size_t stride =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
                                  static_cast<double>(n) / static_cast<double>(max_points))));
  std::vector<float> out;
  out.reserve(max_points * 4);
  for (std::size_t i = 0; i < n && out.size() / 4 < max_points; i += stride) {
    out.push_back(xyzi[i * 4 + 0]);
    out.push_back(xyzi[i * 4 + 1]);
    out.push_back(xyzi[i * 4 + 2]);
    out.push_back(xyzi[i * 4 + 3]);
  }
  return out;
}

std::vector<float> limitXyz(const std::vector<float>& xyz, std::size_t max_points) {
  const std::size_t count = xyz.size() / 3;
  if (max_points == 0 || count <= max_points) {
    return xyz;
  }
  const std::size_t stride = std::max<std::size_t>(
      1,
      static_cast<std::size_t>(std::ceil(
          static_cast<double>(count) / static_cast<double>(max_points))));
  std::vector<float> out;
  out.reserve(max_points * 3);
  for (std::size_t i = 0; i < count && out.size() / 3 < max_points; i += stride) {
    out.push_back(xyz[i * 3 + 0]);
    out.push_back(xyz[i * 3 + 1]);
    out.push_back(xyz[i * 3 + 2]);
  }
  return out;
}

std::vector<float> combineTerrainExt(
    const std::vector<float>& kept_xyzi,
    const std::vector<float>& dynamic_xyzi,
    std::size_t max_points) {
  std::vector<float> combined;
  if (max_points > 0 && kept_xyzi.size() / 4 >= max_points) {
    return limitXyzi(kept_xyzi, max_points);
  }
  const std::size_t kept_points = kept_xyzi.size() / 4;
  const std::size_t dynamic_budget =
      max_points == 0 ? 0 : max_points - kept_points;
  const std::vector<float> dynamic_limited =
      max_points == 0 ? dynamic_xyzi : limitXyzi(dynamic_xyzi, dynamic_budget);
  combined.reserve(kept_xyzi.size() + dynamic_limited.size());
  combined.insert(combined.end(), kept_xyzi.begin(), kept_xyzi.end());
  combined.insert(combined.end(), dynamic_limited.begin(), dynamic_limited.end());
  return combined;
}

void inflateGridCost(
    map_layers::Grid2D& grid,
    int source_row,
    int source_col,
    float cost,
    double radius_m) {
  const int inflate = nav_endpoint::inflationSearchRadiusCells(
      radius_m,
      grid.resolution);
  for (int dr = -inflate; dr <= inflate; ++dr) {
    for (int dc = -inflate; dc <= inflate; ++dc) {
      const int row = source_row + dr;
      const int col = source_col + dc;
      if (row < 0 || row >= grid.rows || col < 0 || col >= grid.cols) {
        continue;
      }
      if (!nav_endpoint::cellCenterWithinInflationRadius(
              dr,
              dc,
              grid.resolution,
              radius_m)) {
        continue;
      }
      float& value = grid.data[static_cast<std::size_t>(grid.index(row, col))];
      value = std::max(value, cost);
    }
  }
}

map_layers::Grid2D buildOccupancyGrid(
    const std::vector<float>& xyz,
    const nav_endpoint::TraversabilityPose& pose,
    const nav_kernel::Vec3& sensor_origin,
    double stamp_s,
    nav_endpoint::ObservedFreeCache& observed_free,
    const CliConfig& cfg,
    SafetyGridDiagnostics& diagnostics,
    std::vector<std::uint8_t>& observed_before_overlays,
    map_layers::Grid2D& occupancy_source_output) {
  const int size = static_cast<int>(std::floor((2.0 * cfg.radius) / cfg.resolution));
  const double origin_x = nav_endpoint::snappedSafetyGridOrigin(
      pose.x,
      cfg.radius,
      cfg.resolution);
  const double origin_y = nav_endpoint::snappedSafetyGridOrigin(
      pose.y,
      cfg.radius,
      cfg.resolution);
  map_layers::Grid2D grid = nav_endpoint::makeUnknownSafetyGrid(
      size,
      size,
      cfg.resolution,
      origin_x,
      origin_y,
      100.0F);
  const std::size_t n = xyz.size() / 3;
  for (std::size_t i = 0; i < n; ++i) {
    const float x = xyz[i * 3 + 0];
    const float y = xyz[i * 3 + 1];
    const float z = xyz[i * 3 + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    observed_free.observeRay(
        sensor_origin.x,
        sensor_origin.y,
        x,
        y,
        stamp_s,
        cfg.radius * 2.0);
  }
  diagnostics = {};
  diagnostics.origin_x = grid.originX;
  diagnostics.origin_y = grid.originY;
  diagnostics.alignment_residual_x = nav_endpoint::safetyGridAlignmentResidual(
      grid.originX,
      grid.resolution);
  diagnostics.alignment_residual_y = nav_endpoint::safetyGridAlignmentResidual(
      grid.originY,
      grid.resolution);
  diagnostics.total_cells = grid.data.size();
  diagnostics.observed_free_cache_cells = observed_free.size();
  diagnostics.observed_free_applied_cells = observed_free.apply(
      grid,
      stamp_s,
      cfg.observed_free_ttl_s);
  diagnostics.observed_free_cache_cells = observed_free.size();
  diagnostics.observed_before_overlays_cells = static_cast<std::size_t>(
      std::count_if(
          grid.data.begin(),
          grid.data.end(),
          [](float value) { return value < 100.0F; }));
  diagnostics.unknown_before_overlays_cells =
      diagnostics.total_cells - diagnostics.observed_before_overlays_cells;
  observed_before_overlays.resize(grid.data.size());
  std::transform(
      grid.data.begin(),
      grid.data.end(),
      observed_before_overlays.begin(),
      [](float value) -> std::uint8_t { return value < 100.0F ? 1U : 0U; });
  map_layers::Grid2D occupancy_source = map_layers::makeGrid2D(
      size,
      size,
      cfg.resolution,
      origin_x,
      origin_y,
      0.0F);
  for (std::size_t i = 0; i < n; ++i) {
    const float x = xyz[i * 3 + 0];
    const float y = xyz[i * 3 + 1];
    const float z = xyz[i * 3 + 2];
    const double rel_z = static_cast<double>(z) - pose.z;
    if (rel_z < cfg.obstacle_min_z || rel_z > cfg.z_max) {
      continue;
    }
    const int col = static_cast<int>(std::floor((x - grid.originX) / grid.resolution));
    const int row = static_cast<int>(std::floor((y - grid.originY) / grid.resolution));
    if (row < 0 || row >= grid.rows || col < 0 || col >= grid.cols) {
      continue;
    }
    inflateGridCost(occupancy_source, row, col, 100.0F, cfg.robot_radius);
  }
  for (std::size_t i = 0; i < grid.data.size(); ++i) {
    if (occupancy_source.data[i] <= 0.0F) {
      continue;
    }
    grid.data[i] = std::max(grid.data[i], occupancy_source.data[i]);
    ++diagnostics.occupancy_source_cells;
  }
  occupancy_source_output = std::move(occupancy_source);
  return grid;
}

map_layers::Grid2D buildTerrainRiskGrid(
    const std::vector<float>& terrain_xyzi,
    const nav_endpoint::TraversabilityPose& pose,
    const CliConfig& cfg) {
  const int size = static_cast<int>(std::floor((2.0 * cfg.radius) / cfg.resolution));
  const double origin_x = nav_endpoint::snappedSafetyGridOrigin(
      pose.x,
      cfg.radius,
      cfg.resolution);
  const double origin_y = nav_endpoint::snappedSafetyGridOrigin(
      pose.y,
      cfg.radius,
      cfg.resolution);
  map_layers::Grid2D raw = map_layers::makeGrid2D(
      size, size, cfg.resolution, origin_x, origin_y, 0.0F);
  const std::size_t count = terrain_xyzi.size() / 4;
  for (std::size_t i = 0; i < count; ++i) {
    const float x = terrain_xyzi[i * 4 + 0];
    const float y = terrain_xyzi[i * 4 + 1];
    const float height = terrain_xyzi[i * 4 + 3];
    const float cost = nav_endpoint::terrainHeightRiskCost(
        height,
        cfg.terrain_soft_height_m,
        cfg.terrain_hard_height_m);
    if (cost <= 0.0F) {
      continue;
    }
    const int col = static_cast<int>(std::floor((x - raw.originX) / raw.resolution));
    const int row = static_cast<int>(std::floor((y - raw.originY) / raw.resolution));
    if (row < 0 || row >= raw.rows || col < 0 || col >= raw.cols) {
      continue;
    }
    float& cell = raw.data[static_cast<std::size_t>(raw.index(row, col))];
    cell = std::max(cell, cost);
  }
  map_layers::Grid2D inflated = map_layers::makeGrid2D(
      size, size, cfg.resolution, raw.originX, raw.originY, 0.0F);
  for (int row = 0; row < raw.rows; ++row) {
    for (int col = 0; col < raw.cols; ++col) {
      const float cost = raw.data[static_cast<std::size_t>(raw.index(row, col))];
      if (cost > 0.0F) {
        inflateGridCost(inflated, row, col, cost, cfg.robot_radius);
      }
    }
  }
  return inflated;
}

void mergeGridMax(
    map_layers::Grid2D& destination,
    const map_layers::Grid2D& source) {
  if (destination.resolution <= 0.0 || source.resolution <= 0.0) {
    return;
  }
  for (int row = 0; row < source.rows; ++row) {
    for (int col = 0; col < source.cols; ++col) {
      const float cost = source.data[static_cast<std::size_t>(source.index(row, col))];
      if (cost <= 0.0F) {
        continue;
      }
      const double x = source.originX + (static_cast<double>(col) + 0.5) * source.resolution;
      const double y = source.originY + (static_cast<double>(row) + 0.5) * source.resolution;
      const int destination_col = static_cast<int>(
          std::floor((x - destination.originX) / destination.resolution));
      const int destination_row = static_cast<int>(
          std::floor((y - destination.originY) / destination.resolution));
      if (destination_row < 0 || destination_row >= destination.rows ||
          destination_col < 0 || destination_col >= destination.cols) {
        continue;
      }
      float& destination_cost = destination.data[static_cast<std::size_t>(
          destination.index(destination_row, destination_col))];
      destination_cost = std::max(destination_cost, cost);
    }
  }
}

map_layers::Grid2D inflateRiskGrid(
    const map_layers::Grid2D& source,
    double radius_m) {
  map_layers::Grid2D inflated = map_layers::makeGrid2D(
      source.rows,
      source.cols,
      source.resolution,
      source.originX,
      source.originY,
      0.0F);
  for (int row = 0; row < source.rows; ++row) {
    for (int col = 0; col < source.cols; ++col) {
      const float cost = source.data[static_cast<std::size_t>(source.index(row, col))];
      if (cost > 0.0F) {
        inflateGridCost(inflated, row, col, cost, radius_m);
      }
    }
  }
  return inflated;
}

map_layers::Grid2D buildSurfaceTerrainRiskGrid(
    const std::vector<float>& terrain_xyzi,
    const nav_endpoint::TraversabilityPose& pose,
    const CliConfig& cfg) {
  std::vector<float> xyz;
  xyz.reserve((terrain_xyzi.size() / 4) * 3);
  for (std::size_t i = 0; i + 3 < terrain_xyzi.size(); i += 4) {
    xyz.push_back(terrain_xyzi[i + 0]);
    xyz.push_back(terrain_xyzi[i + 1]);
    xyz.push_back(terrain_xyzi[i + 2]);
  }
  const double origin_x = nav_endpoint::snappedSafetyGridOrigin(
      pose.x,
      cfg.radius,
      cfg.resolution);
  const double origin_y = nav_endpoint::snappedSafetyGridOrigin(
      pose.y,
      cfg.radius,
      cfg.resolution);
  const map_layers::ElevationMapResult elevation = map_layers::buildElevationMap(
      xyz,
      origin_x + cfg.radius,
      origin_y + cfg.radius,
      cfg.resolution,
      cfg.radius,
      pose.z + cfg.z_min,
      pose.z + cfg.z_max);
  const map_layers::TerrainRiskResult metrics =
      map_layers::computeTerrainRisk(elevation);
  map_layers::Grid2D graded = map_layers::makeGrid2D(
      metrics.risk.rows,
      metrics.risk.cols,
      metrics.risk.resolution,
      metrics.risk.originX,
      metrics.risk.originY,
      0.0F);
  for (std::size_t i = 0; i < graded.data.size(); ++i) {
    const float slope_cost = nav_endpoint::terrainSlopeRiskCost(
        metrics.slopeDeg.data[i],
        cfg.terrain_soft_slope_deg,
        cfg.terrain_hard_slope_deg);
    const float step_cost = nav_endpoint::terrainHeightRiskCost(
        metrics.stepHeight.data[i],
        cfg.terrain_soft_height_m,
        cfg.terrain_hard_height_m);
    const float roughness_cost = nav_endpoint::terrainHeightRiskCost(
        metrics.roughness.data[i],
        cfg.terrain_soft_height_m,
        cfg.terrain_hard_height_m);
    graded.data[i] = std::max(slope_cost, std::max(step_cost, roughness_cost));
  }
  return inflateRiskGrid(graded, cfg.robot_radius);
}

void updateFusedGridDiagnostics(
    const map_layers::Grid2D& grid,
    SafetyGridDiagnostics& diagnostics) {
  diagnostics.fused_zero_cells = 0;
  diagnostics.fused_soft_cells = 0;
  diagnostics.fused_hard_cells = 0;
  for (const float cost : grid.data) {
    if (cost >= 80.0F) {
      ++diagnostics.fused_hard_cells;
    } else if (cost > 0.0F) {
      ++diagnostics.fused_soft_cells;
    } else {
      ++diagnostics.fused_zero_cells;
    }
  }
}

std::pair<std::size_t, float> terrainRiskStats(
    const map_layers::Grid2D& grid) {
  std::size_t cells = 0;
  float max_cost = 0.0F;
  for (const float cost : grid.data) {
    if (cost > 0.0F) {
      ++cells;
      max_cost = std::max(max_cost, cost);
    }
  }
  return {cells, max_cost};
}

struct OccupancyMessage {
  lingtu_dds_OccupancyGrid msg{};
  std::vector<std::uint8_t> data;
};

struct PointCloudMessage {
  lingtu_dds_PointCloud2 msg{};
  std::array<lingtu_dds_PointField, 4> fields{};
  std::vector<std::uint8_t> data;
};

void fillPointFields(std::array<lingtu_dds_PointField, 4>& fields) {
  fields[0].name = const_cast<char*>("x");
  fields[0].offset = 0;
  fields[0].datatype = 7;
  fields[0].count = 1;
  fields[1].name = const_cast<char*>("y");
  fields[1].offset = 4;
  fields[1].datatype = 7;
  fields[1].count = 1;
  fields[2].name = const_cast<char*>("z");
  fields[2].offset = 8;
  fields[2].datatype = 7;
  fields[2].count = 1;
  fields[3].name = const_cast<char*>("intensity");
  fields[3].offset = 12;
  fields[3].datatype = 7;
  fields[3].count = 1;
}

OccupancyMessage toOccupancyMessage(
    const map_layers::Grid2D& grid,
    double stamp_s) {
  OccupancyMessage out;
  const double stamp =
      std::isfinite(stamp_s) && stamp_s > 0.0 ? stamp_s : nowSeconds();
  fillHeader(out.msg.header, stamp, "map");
  out.msg.info.map_load_time.sec = static_cast<std::int32_t>(stamp);
  out.msg.info.map_load_time.nanosec =
      static_cast<std::uint32_t>((stamp - static_cast<double>(out.msg.info.map_load_time.sec)) * 1e9);
  out.msg.info.resolution = static_cast<float>(grid.resolution);
  out.msg.info.width = static_cast<std::uint32_t>(grid.cols);
  out.msg.info.height = static_cast<std::uint32_t>(grid.rows);
  out.msg.info.origin.position.x = grid.originX;
  out.msg.info.origin.position.y = grid.originY;
  out.msg.info.origin.position.z = 0.0;
  out.msg.info.origin.orientation.w = 1.0;
  out.data.reserve(grid.data.size());
  for (const float v : grid.data) {
    const float cost = std::isfinite(v) ? map_layers::clampFloat(v, 0.0f, 100.0f) : 100.0f;
    out.data.push_back(static_cast<std::uint8_t>(std::lround(cost)));
  }
  out.msg.data._maximum = static_cast<std::uint32_t>(out.data.size());
  out.msg.data._length = static_cast<std::uint32_t>(out.data.size());
  out.msg.data._buffer = out.data.data();
  out.msg.data._release = false;
  return out;
}

PointCloudMessage toTerrainMapMessage(const std::vector<float>& xyzi, double stamp_s) {
  PointCloudMessage out;
  fillPointFields(out.fields);
  constexpr std::uint32_t point_step = 16;
  const auto width = static_cast<std::uint32_t>(xyzi.size() / 4);
  const auto row_step = point_step * width;
  out.data.resize(static_cast<std::size_t>(row_step));
  for (std::uint32_t i = 0; i < width; ++i) {
    const std::size_t base = static_cast<std::size_t>(i) * point_step;
    writeFloat(out.data, base + 0, xyzi[static_cast<std::size_t>(i) * 4 + 0]);
    writeFloat(out.data, base + 4, xyzi[static_cast<std::size_t>(i) * 4 + 1]);
    writeFloat(out.data, base + 8, xyzi[static_cast<std::size_t>(i) * 4 + 2]);
    writeFloat(out.data, base + 12, xyzi[static_cast<std::size_t>(i) * 4 + 3]);
  }
  fillHeader(out.msg.header, stamp_s, "map");
  out.msg.height = 1;
  out.msg.width = width;
  out.msg.fields._maximum = static_cast<std::uint32_t>(out.fields.size());
  out.msg.fields._length = static_cast<std::uint32_t>(out.fields.size());
  out.msg.fields._buffer = out.fields.data();
  out.msg.fields._release = false;
  out.msg.is_bigendian = false;
  out.msg.point_step = point_step;
  out.msg.row_step = row_step;
  out.msg.data._maximum = static_cast<std::uint32_t>(out.data.size());
  out.msg.data._length = static_cast<std::uint32_t>(out.data.size());
  out.msg.data._buffer = out.data.data();
  out.msg.data._release = false;
  out.msg.is_dense = false;
  return out;
}

void writeStatus(
    const CliConfig& cfg,
    std::uint64_t odom_count,
    std::uint64_t tf_count,
    std::uint64_t cloud_count,
    std::uint64_t publish_count,
    std::uint64_t slow_publish_count,
    std::uint64_t map_clearing_count,
    std::uint64_t cloud_clearing_count,
    std::uint64_t tf_reject_count,
    std::uint64_t odom_frame_reject_count,
    std::uint64_t cloud_frame_reject_count,
    std::uint64_t map_frame_jump_clear_count,
    std::uint64_t error_count,
    std::size_t last_points,
    std::size_t terrain_points,
    std::size_t terrain_ext_points,
    std::size_t dynamic_clear_points,
    std::size_t dynamic_clear_ray_points,
    std::size_t dynamic_clear_evidence,
    std::size_t dynamic_clear_free_voxels,
    std::size_t cache_points,
    std::size_t terrain_risk_cells,
    float terrain_risk_max_cost,
    bool has_odom,
    bool has_map_odom_tf,
    double map_odom_tf_age_s,
    double cloud_pose_gap_s,
    const std::string& odom_frame_id,
    const std::string& cloud_frame_id,
    const std::string& last_frame_error,
    const SafetyGridDiagnostics& grid_diagnostics,
    const nav_endpoint::SafetyGridForwardProbe& forward_probe,
    const TimingDiagnostics& timing) {
  if (cfg.status_file.empty()) {
    return;
  }
  const std::filesystem::path path(cfg.status_file);
  if (!path.parent_path().empty()) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
  }
  const std::filesystem::path tmp = path.string() + ".tmp";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out) {
    return;
  }
  out << std::fixed << std::setprecision(6);
  out << "{\n"
      << "  \"schema_version\": \"lingtu.traversability.status.v1\",\n"
      << "  \"endpoint\": \"lingtu_traversability_dds\",\n"
      << "  \"stamp_s\": " << nowSeconds() << ",\n"
      << "  \"domain_id\": " << cfg.domain_id << ",\n"
      << "  \"publish_hz\": " << cfg.publish_hz << ",\n"
      << "  \"slow_hz\": " << cfg.slow_hz << ",\n"
      << "  \"tick_hz\": " << cfg.tick_hz << ",\n"
      << "  \"resolution\": " << cfg.resolution << ",\n"
      << "  \"radius\": " << cfg.radius << ",\n"
      << "  \"observed_free\": {\"ttl_s\": " << cfg.observed_free_ttl_s << ", "
      << "\"cache_cells\": " << grid_diagnostics.observed_free_cache_cells << ", "
      << "\"applied_cells\": " << grid_diagnostics.observed_free_applied_cells << "},\n"
      << "  \"safety_grid\": {\"origin_x\": " << grid_diagnostics.origin_x << ", "
      << "\"origin_y\": " << grid_diagnostics.origin_y << ", "
      << "\"alignment_residual_x\": " << grid_diagnostics.alignment_residual_x << ", "
      << "\"alignment_residual_y\": " << grid_diagnostics.alignment_residual_y << ", "
      << "\"total_cells\": " << grid_diagnostics.total_cells << ", "
      << "\"observed_before_overlays_cells\": "
      << grid_diagnostics.observed_before_overlays_cells << ", "
      << "\"unknown_before_overlays_cells\": "
      << grid_diagnostics.unknown_before_overlays_cells << ", "
      << "\"occupancy_source_cells\": " << grid_diagnostics.occupancy_source_cells << ", "
      << "\"fused_zero_cells\": " << grid_diagnostics.fused_zero_cells << ", "
      << "\"fused_soft_cells\": " << grid_diagnostics.fused_soft_cells << ", "
      << "\"fused_hard_cells\": " << grid_diagnostics.fused_hard_cells << "},\n"
      << "  \"terrain_risk\": {\"source\": "
      << "\"height_plus_slope_step_roughness\", "
      << "\"soft_height_m\": " << cfg.terrain_soft_height_m << ", "
      << "\"hard_height_m\": " << cfg.terrain_hard_height_m << ", "
      << "\"soft_slope_deg\": " << cfg.terrain_soft_slope_deg << ", "
      << "\"hard_slope_deg\": " << cfg.terrain_hard_slope_deg << ", "
      << "\"soft_cost\": 40.0, \"hard_cost\": 100.0, "
      << "\"cells\": " << terrain_risk_cells << ", "
      << "\"max_cost\": " << terrain_risk_max_cost << "},\n"
      << "  \"forward_probe\": {\"grid_generation\": "
      << forward_probe.grid_generation << ", "
      << "\"terrain_generation\": " << forward_probe.terrain_generation << ", "
      << "\"source_stamp_s\": " << forward_probe.source_stamp_s << ", "
      << "\"terrain_source_stamp_s\": " << forward_probe.terrain_source_stamp_s << ", "
      << "\"horizon_m\": " << forward_probe.horizon_m << ", "
      << "\"step_m\": " << forward_probe.step_m << ", "
      << "\"pose\": {\"x\": " << forward_probe.pose.x << ", "
      << "\"y\": " << forward_probe.pose.y << ", "
      << "\"yaw\": " << forward_probe.pose.yaw << "}, "
      << "\"samples\": [";
  for (std::size_t i = 0; i < forward_probe.samples.size(); ++i) {
    const auto& sample = forward_probe.samples[i];
    if (i > 0) {
      out << ", ";
    }
    out << "{\"distance_m\": " << sample.distance_m << ", "
        << "\"map_x\": " << sample.map_x << ", "
        << "\"map_y\": " << sample.map_y << ", "
        << "\"row\": " << sample.row << ", "
        << "\"col\": " << sample.col << ", "
        << "\"used_by_teleop\": " << (sample.used_by_teleop ? "true" : "false") << ", "
        << "\"in_bounds\": " << (sample.in_bounds ? "true" : "false") << ", "
        << "\"observed_before_overlays\": "
        << (sample.observed_before_overlays ? "true" : "false") << ", "
        << "\"unknown_before_overlays\": "
        << (sample.unknown_before_overlays ? "true" : "false") << ", "
        << "\"occupancy_cost\": " << sample.occupancy_cost << ", "
        << "\"height_risk_cost\": " << sample.height_risk_cost << ", "
        << "\"surface_risk_cost\": " << sample.surface_risk_cost << ", "
        << "\"fused_cost\": " << sample.fused_cost << "}";
  }
  out << "]},\n"
      << "  \"cloud_pose_max_gap_s\": " << cfg.cloud_pose_max_gap_s << ",\n"
      << "  \"sensor_offset_m\": [" << cfg.sensor_offset_x_m << ", "
      << cfg.sensor_offset_y_m << ", " << cfg.sensor_offset_z_m << "],\n"
      << "  \"terrain_clear_dy_obs\": " << (cfg.terrain_clear_dy_obs ? "true" : "false") << ",\n"
      << "  \"terrain_no_data_obstacle\": " << (cfg.terrain_no_data_obstacle ? "true" : "false") << ",\n"
      << "  \"dynamic_clear\": " << (cfg.dynamic_clear ? "true" : "false") << ",\n"
      << "  \"dynamic_clear_raycast\": " << (cfg.dynamic_clear_raycast ? "true" : "false") << ",\n"
      << "  \"has_odom\": " << (has_odom ? "true" : "false") << ",\n"
      << "  \"has_map_odom_tf\": " << (has_map_odom_tf ? "true" : "false") << ",\n"
      << "  \"frame_contract\": {\n"
      << "    \"odom_input_frame\": \"" << jsonEscape(odom_frame_id) << "\",\n"
      << "    \"cloud_input_frame\": \"" << jsonEscape(cloud_frame_id) << "\",\n"
      << "    \"geometry_frame\": \"map\",\n"
      << "    \"header_frame\": \"map\",\n"
      << "    \"map_odom_tf_age_s\": " << map_odom_tf_age_s << ",\n"
      << "    \"cloud_pose_gap_s\": " << cloud_pose_gap_s << ",\n"
      << "    \"last_error\": \"" << jsonEscape(last_frame_error) << "\"\n"
      << "  },\n"
      << "  \"last_points\": " << last_points << ",\n"
      << "  \"terrain_points\": " << terrain_points << ",\n"
      << "  \"terrain_map_ext_points\": " << terrain_ext_points << ",\n"
      << "  \"dynamic_clear_points\": " << dynamic_clear_points << ",\n"
      << "  \"dynamic_clear_ray_points\": " << dynamic_clear_ray_points << ",\n"
      << "  \"dynamic_clear_evidence_voxels\": " << dynamic_clear_evidence << ",\n"
      << "  \"dynamic_clear_free_voxels\": " << dynamic_clear_free_voxels << ",\n"
      << "  \"cache_points\": " << cache_points << ",\n"
      << "  \"timing_ms\": {\n"
      << "    \"loop\": " << timing.loop_ms << ",\n"
      << "    \"input_callbacks\": " << timing.input_callbacks_ms << ",\n"
      << "    \"cloud_convert\": " << timing.cloud_convert_ms << ",\n"
      << "    \"fast_occupancy\": " << timing.fast_occupancy_ms << ",\n"
      << "    \"slow_terrain\": " << timing.slow_terrain_ms << ",\n"
      << "    \"terrain_core\": " << timing.terrain_core_ms << ",\n"
      << "    \"dynamic_clear\": " << timing.dynamic_clear_ms << ",\n"
      << "    \"terrain_pack\": " << timing.terrain_pack_ms << ",\n"
      << "    \"dds_write\": " << timing.dds_write_ms << ",\n"
      << "    \"sleep\": " << timing.sleep_ms << ",\n"
      << "    \"overrun\": " << timing.overrun_ms << "\n"
      << "  },\n"
      << "  \"counters\": {\n"
      << "    \"odom\": " << odom_count << ",\n"
      << "    \"tf\": " << tf_count << ",\n"
      << "    \"registered_clouds\": " << cloud_count << ",\n"
      << "    \"published\": " << publish_count << ",\n"
      << "    \"slow_published\": " << slow_publish_count << ",\n"
      << "    \"map_clearing\": " << map_clearing_count << ",\n"
      << "    \"cloud_clearing\": " << cloud_clearing_count << ",\n"
      << "    \"tf_rejected\": " << tf_reject_count << ",\n"
      << "    \"odom_frame_rejected\": " << odom_frame_reject_count << ",\n"
      << "    \"cloud_frame_rejected\": " << cloud_frame_reject_count << ",\n"
      << "    \"map_frame_jump_clears\": " << map_frame_jump_clear_count << ",\n"
      << "    \"errors\": " << error_count << "\n"
      << "  }\n"
      << "}\n";
  out.close();
  std::error_code ec;
  if (!lingtu::message::replaceSnapshotFile(tmp, path, &ec)) {
    std::fprintf(
        stderr,
        "traversability_dds: failed to replace status snapshot %s: %s\n",
        path.string().c_str(),
        ec.message().c_str());
  }
}

class DdsRuntime {
 public:
  explicit DdsRuntime(int domain_id) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    subscriber_ = checked(dds_create_subscriber(participant_, nullptr, nullptr),
                          "dds_create_subscriber");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher");
    odom_reader_ = reader(
        lingtu::message::kSlamOdometry.dds_topic.data(), &lingtu_dds_Odometry_desc, "odom");
    tf_reader_ = reader(
        lingtu::message::kTf.dds_topic.data(), &lingtu_dds_TFMessage_desc, "tf");
    cloud_reader_ = reader(
        lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "registered_cloud");
    map_clearing_reader_ = reader(
        lingtu::message::kNavMapClearing.dds_topic.data(),
        &lingtu_dds_Bool_desc,
        "map_clearing");
    cloud_clearing_reader_ = reader(
        lingtu::message::kNavCloudClearing.dds_topic.data(),
        &lingtu_dds_Bool_desc,
        "cloud_clearing");
    traversability_writer_ = writer(
        lingtu::message::kNavTraversability.dds_topic.data(),
        &lingtu_dds_OccupancyGrid_desc,
        "traversability");
    terrain_map_writer_ = writer(
        lingtu::message::kNavTerrainMap.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "terrain_map");
    terrain_map_ext_writer_ = writer(
        lingtu::message::kNavTerrainMapExt.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "terrain_map_ext");
  }

  ~DdsRuntime() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  template <typename Handler>
  void drainOdometry(Handler&& handler) {
    drainReader<lingtu_dds_Odometry>(
        odom_reader_, lingtu_dds_Odometry_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTf(Handler&& handler) {
    drainReader<lingtu_dds_TFMessage>(
        tf_reader_,
        lingtu_dds_TFMessage_desc,
        std::forward<Handler>(handler),
        nav_endpoint::DdsDrainBudget{8, 16});
  }

  template <typename Handler>
  void drainCloud(Handler&& handler) {
    drainReader<lingtu_dds_PointCloud2>(
        cloud_reader_, lingtu_dds_PointCloud2_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainMapClearing(Handler&& handler) {
    drainReader<lingtu_dds_Bool>(
        map_clearing_reader_, lingtu_dds_Bool_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCloudClearing(Handler&& handler) {
    drainReader<lingtu_dds_Bool>(
        cloud_clearing_reader_, lingtu_dds_Bool_desc, std::forward<Handler>(handler));
  }

  void writeTraversability(
      const map_layers::Grid2D& grid,
      double source_stamp_s) {
    OccupancyMessage msg = toOccupancyMessage(grid, source_stamp_s);
    logDdsError(dds_write(traversability_writer_, &msg.msg), "dds_write(traversability)");
  }

  void writeTerrainMap(const std::vector<float>& xyzi, double stamp_s) {
    PointCloudMessage msg = toTerrainMapMessage(xyzi, stamp_s);
    logDdsError(dds_write(terrain_map_writer_, &msg.msg), "dds_write(terrain_map)");
  }

  void writeTerrainMapExt(const std::vector<float>& xyzi, double stamp_s) {
    PointCloudMessage msg = toTerrainMapMessage(xyzi, stamp_s);
    logDdsError(dds_write(terrain_map_ext_writer_, &msg.msg), "dds_write(terrain_map_ext)");
  }

 private:
  dds_entity_t reader(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    const dds_entity_t entity = checked(
        dds_create_reader(subscriber_, topic, qos.get(), nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
    return entity;
  }

  dds_entity_t writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(topic_name));
    return checked(
        dds_create_writer(publisher_, topic, qos.get(), nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t odom_reader_{0};
  dds_entity_t tf_reader_{0};
  dds_entity_t cloud_reader_{0};
  dds_entity_t map_clearing_reader_{0};
  dds_entity_t cloud_clearing_reader_{0};
  dds_entity_t traversability_writer_{0};
  dds_entity_t terrain_map_writer_{0};
  dds_entity_t terrain_map_ext_writer_{0};
};

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    const CliConfig cfg = parseArgs(argc, argv);
    DdsRuntime dds(cfg.domain_id);

    std::optional<nav_endpoint::RigidTransform> odom_body;
    std::optional<nav_endpoint::RigidTransform> map_odom;
    std::optional<nav_endpoint::RigidTransform> map_body;
    nav_endpoint::TransformBuffer map_body_buffer(2.0);
    nav_endpoint::TransformBuffer map_odom_buffer(2.0);
    std::string odom_frame_id;
    std::string cloud_frame_id;
    std::string last_frame_error{"none"};
    std::uint64_t odom_count = 0;
    std::uint64_t tf_count = 0;
    std::uint64_t cloud_count = 0;
    std::uint64_t publish_count = 0;
    std::uint64_t slow_publish_count = 0;
    std::uint64_t map_clearing_count = 0;
    std::uint64_t cloud_clearing_count = 0;
    std::uint64_t tf_reject_count = 0;
    std::uint64_t odom_frame_reject_count = 0;
    std::uint64_t cloud_frame_reject_count = 0;
    std::uint64_t map_frame_jump_clear_count = 0;
    std::uint64_t error_count = 0;
    std::size_t last_points = 0;
    std::size_t last_terrain_points = 0;
    std::size_t last_terrain_ext_points = 0;
    std::size_t last_dynamic_clear_points = 0;
    std::size_t last_dynamic_clear_ray_points = 0;
    std::size_t last_dynamic_clear_evidence = 0;
    std::size_t last_dynamic_clear_free_voxels = 0;
    std::size_t last_terrain_risk_cells = 0;
    float last_terrain_risk_max_cost = 0.0F;
    SafetyGridDiagnostics last_grid_diagnostics;
    double last_cloud_pose_gap_s = -1.0;
    std::size_t cache_points = 0;
    std::optional<map_layers::Grid2D> terrain_risk_grid;
    std::optional<map_layers::Grid2D> height_risk_grid;
    std::optional<map_layers::Grid2D> surface_risk_grid;
    map_layers::Grid2D occupancy_source_grid;
    std::vector<std::uint8_t> observed_before_overlays;
    nav_endpoint::SafetyGridForwardProbe last_forward_probe;
    double last_terrain_risk_stamp_s = 0.0;
    nav_kernel::TerrainAnalysisCore terrain_core(terrainParamsFromConfig(cfg));
    nav_kernel::DynamicClearCore dynamic_clear(dynamicClearParamsFromConfig(cfg));
    nav_endpoint::ObservedFreeCache observed_free(cfg.resolution);
    auto clear_runtime_maps = [&]() {
      terrain_core.clear();
      cache_points = 0;
      last_terrain_points = 0;
      last_terrain_ext_points = 0;
      last_dynamic_clear_points = 0;
      last_dynamic_clear_ray_points = 0;
      last_dynamic_clear_evidence = 0;
      last_dynamic_clear_free_voxels = 0;
      last_terrain_risk_cells = 0;
      last_terrain_risk_max_cost = 0.0F;
      last_grid_diagnostics = {};
      terrain_risk_grid.reset();
      height_risk_grid.reset();
      surface_risk_grid.reset();
      occupancy_source_grid = {};
      observed_before_overlays.clear();
      last_forward_probe = {};
      last_terrain_risk_stamp_s = 0.0;
      dynamic_clear.reset();
      observed_free.clear();
    };
    auto refresh_map_body = [&]() {
      map_body = odom_body
          ? mapBodyTransform(*odom_body, odom_frame_id, map_odom)
          : std::optional<nav_endpoint::RigidTransform>{};
    };
    double last_publish = 0.0;
    double next_slow_publish = 0.0;
    double next_status = nowSeconds() + 5.0;
    TimingDiagnostics last_timing;
    const auto tick_period = std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(1.0 / std::max(1.0, cfg.tick_hz)));
    auto next_tick = SteadyClock::now();

    std::fprintf(
        stderr,
        "lingtu_traversability_dds: domain=%d publish_hz=%.2f slow_hz=%.2f resolution=%.2f radius=%.1f\n",
        cfg.domain_id,
        cfg.publish_hz,
        cfg.slow_hz,
        cfg.resolution,
        cfg.radius);

    while (g_running) {
      const auto loop_start = SteadyClock::now();
      next_tick += tick_period;
      TimingDiagnostics timing;
      const auto input_start = SteadyClock::now();
      dds.drainTf([&](const lingtu_dds_TFMessage& msg) {
        ++tf_count;
        const auto next_map_odom = nav_endpoint::mapOdomTransformFromTf(msg);
        if (!next_map_odom || !next_map_odom->valid) {
          ++tf_reject_count;
          last_frame_error = "map_odom_tf_missing_in_message";
          return;
        }
        if (map_odom) {
          const auto correction = nav_endpoint::composeTransforms(
              *next_map_odom,
              nav_endpoint::inverseTransform(*map_odom));
          const double translation_jump = std::sqrt(
              correction.translation.x * correction.translation.x +
              correction.translation.y * correction.translation.y +
              correction.translation.z * correction.translation.z);
          if (translation_jump > 0.50 || std::abs(correction.yaw) > 0.25) {
            clear_runtime_maps();
            map_body_buffer.clear();
            map_odom_buffer.clear();
            ++map_frame_jump_clear_count;
          }
        }
        map_odom = next_map_odom;
        map_odom_buffer.push(next_map_odom->stamp_s, *next_map_odom);
        refresh_map_body();
        if (map_body && odom_body) {
          map_body_buffer.push(odom_body->stamp_s, *map_body);
        }
        last_frame_error = "none";
      });
      dds.drainOdometry([&](const lingtu_dds_Odometry& msg) {
        ++odom_count;
        const std::string frame = headerFrameId(msg.header);
        if (frame != "odom" && frame != "map") {
          ++odom_frame_reject_count;
          last_frame_error = frame.empty() ? "odom_frame_empty" : "odom_frame_unsupported";
          return;
        }
        const auto next_odom_body = transformFromOdometry(msg);
        if (!next_odom_body.valid) {
          ++odom_frame_reject_count;
          last_frame_error = "odom_pose_nonfinite";
          return;
        }
        odom_frame_id = frame;
        odom_body = next_odom_body;
        refresh_map_body();
        if (map_body) {
          map_body_buffer.push(next_odom_body.stamp_s, *map_body);
        }
      });
      dds.drainMapClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        clear_runtime_maps();
        ++map_clearing_count;
      });
      dds.drainCloudClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        clear_runtime_maps();
        ++cloud_clearing_count;
      });
      dds.drainCloud([&](const lingtu_dds_PointCloud2& msg) {
        ++cloud_count;
        const double now = nowSeconds();
        const double publish_period_s = 1.0 / cfg.publish_hz;
        if (last_publish > 0.0 && now - last_publish < publish_period_s * 0.75) {
          return;
        }
        const double cloud_stamp_s = nav_endpoint::ddsStampSeconds(msg.header.stamp);
        if (!std::isfinite(cloud_stamp_s) || cloud_stamp_s <= 0.0) {
          ++cloud_frame_reject_count;
          last_frame_error = "cloud_stamp_invalid";
          return;
        }
        last_cloud_pose_gap_s = map_body_buffer.nearestGap(cloud_stamp_s);
        const auto cloud_map_body =
            map_body_buffer.sample(cloud_stamp_s, cfg.cloud_pose_max_gap_s);
        if (!cloud_map_body) {
          ++cloud_frame_reject_count;
          last_frame_error = "cloud_pose_gap_exceeded";
          return;
        }
        const auto cloud_pose = nav_endpoint::traversabilityPose(*cloud_map_body);
        const auto cloud_sensor_origin = cloud_pose
            ? nav_endpoint::traversabilitySensorOrigin(
                  *cloud_pose,
                  {
                      cfg.sensor_offset_x_m,
                      cfg.sensor_offset_y_m,
                      cfg.sensor_offset_z_m,
                  })
            : std::optional<nav_kernel::Vec3>{};
        if (!cloud_pose || !cloud_sensor_origin) {
          ++cloud_frame_reject_count;
          last_frame_error = "cloud_pose_invalid_6dof";
          return;
        }
        const std::string source_frame = headerFrameId(msg.header);
        std::optional<nav_endpoint::RigidTransform> cloud_map_odom;
        if (source_frame == "odom") {
          const double tf_gap_s = cfg.tf_max_age_s > 0.0
              ? std::min(cfg.cloud_pose_max_gap_s, cfg.tf_max_age_s)
              : cfg.cloud_pose_max_gap_s;
          cloud_map_odom =
              map_odom_buffer.sample(cloud_stamp_s, tf_gap_s);
          if (!cloud_map_odom) {
            ++tf_reject_count;
            last_frame_error = "cloud_tf_gap_exceeded";
            return;
          }
        }
        last_publish = now;
        try {
          const auto convert_start = SteadyClock::now();
          MapCloudResult cloud = cloudToMapXyz(
              msg,
              cloud_map_body,
              cloud_map_odom,
              cfg.terrain_cache_max_points);
          timing.cloud_convert_ms += elapsedMs(convert_start);
          cloud_frame_id = cloud.input_frame;
          if (!cloud.accepted) {
            ++cloud_frame_reject_count;
            last_frame_error = cloud.reason;
            return;
          }
          last_frame_error = "none";
          const std::vector<float>& xyz = cloud.xyz;
          last_points = xyz.size() / 3;
          if (!xyz.empty()) {
            const nav_endpoint::TraversabilityPose& pose = *cloud_pose;
            const std::vector<float> fast_xyz = limitXyz(xyz, cfg.max_points);
            const auto occupancy_start = SteadyClock::now();
            map_layers::Grid2D occupancy = buildOccupancyGrid(
                fast_xyz,
                pose,
                *cloud_sensor_origin,
                cloud_stamp_s,
                observed_free,
                cfg,
                last_grid_diagnostics,
                observed_before_overlays,
                occupancy_source_grid);
            timing.fast_occupancy_ms += elapsedMs(occupancy_start);

            const double slow_period_s = 1.0 / cfg.slow_hz;
            const double slow_tolerance_s = 0.5 * publish_period_s;
            if (next_slow_publish <= 0.0) {
              next_slow_publish = now;
            }
            if (now + slow_tolerance_s >= next_slow_publish) {
              do {
                next_slow_publish += slow_period_s;
              } while (next_slow_publish <= now - slow_period_s);
              const auto slow_start = SteadyClock::now();
              const std::vector<float> scan_xyzi = xyzToXyzi(xyz);
              terrain_core.updateVehicle(
                  pose.x,
                  pose.y,
                  pose.z,
                  pose.roll,
                  pose.pitch,
                  pose.yaw);
              const auto terrain_core_start = SteadyClock::now();
              const nav_kernel::TerrainResult terrain_result =
                  terrain_core.process(
                      scan_xyzi.data(),
                      static_cast<int>(scan_xyzi.size() / 4),
                      cloud_stamp_s);
              timing.terrain_core_ms += elapsedMs(terrain_core_start);
              cache_points = terrain_core.storedPointCount();
              const nav_kernel::DynamicClearOrigin clear_origin{
                  cloud_sensor_origin->x,
                  cloud_sensor_origin->y,
                  cloud_sensor_origin->z,
                  true,
              };
              const auto dynamic_clear_start = SteadyClock::now();
              const nav_kernel::DynamicClearResult clear_result =
                  dynamic_clear.filter(
                      terrain_result.terrain_points,
                      xyz,
                      clear_origin,
                      cloud_stamp_s);
              timing.dynamic_clear_ms += elapsedMs(dynamic_clear_start);
              const auto terrain_pack_start = SteadyClock::now();
              const std::vector<float> terrain_xyzi =
                  limitXyzi(clear_result.kept_xyzi, cfg.max_points);
              const std::vector<float> terrain_ext_xyzi =
                  combineTerrainExt(
                      terrain_xyzi,
                      clear_result.dynamic_xyzi,
                      cfg.max_points);
              height_risk_grid = buildTerrainRiskGrid(
                  clear_result.kept_xyzi,
                  pose,
                  cfg);
              surface_risk_grid = buildSurfaceTerrainRiskGrid(
                      clear_result.kept_xyzi,
                      pose,
                      cfg);
              terrain_risk_grid = *height_risk_grid;
              mergeGridMax(*terrain_risk_grid, *surface_risk_grid);
              last_terrain_risk_stamp_s = cloud_stamp_s;
              const auto [risk_cells, risk_max_cost] =
                  terrainRiskStats(*terrain_risk_grid);
              last_terrain_risk_cells = risk_cells;
              last_terrain_risk_max_cost = risk_max_cost;
              last_terrain_points = terrain_xyzi.size() / 4;
              last_dynamic_clear_points = clear_result.stats.dynamic_points;
              last_dynamic_clear_ray_points = clear_result.stats.ray_cleared_points;
              last_dynamic_clear_evidence = clear_result.stats.evidence_voxels;
              last_dynamic_clear_free_voxels = clear_result.stats.free_voxels;
              timing.terrain_pack_ms += elapsedMs(terrain_pack_start);
              if (!terrain_xyzi.empty() || !terrain_ext_xyzi.empty()) {
                const auto terrain_write_start = SteadyClock::now();
                dds.writeTerrainMap(terrain_xyzi, cloud_stamp_s);
                dds.writeTerrainMapExt(terrain_ext_xyzi, cloud_stamp_s);
                timing.dds_write_ms += elapsedMs(terrain_write_start);
              }
              last_terrain_ext_points = terrain_ext_xyzi.size() / 4;
              timing.slow_terrain_ms += elapsedMs(slow_start);
              ++slow_publish_count;
            }
            if (terrain_risk_grid) {
              mergeGridMax(occupancy, *terrain_risk_grid);
            }
            updateFusedGridDiagnostics(occupancy, last_grid_diagnostics);
            const nav_endpoint::SafetyGridProbeLayers probe_layers{
                &occupancy,
                &observed_before_overlays,
                &occupancy_source_grid,
                height_risk_grid ? &*height_risk_grid : nullptr,
                surface_risk_grid ? &*surface_risk_grid : nullptr,
            };
            last_forward_probe =
                nav_endpoint::buildStraightForwardSafetyGridProbe(
                    {pose.x, pose.y, pose.yaw},
                    probe_layers,
                    kNativeTeleopStraightProbeHorizonM,
                    kNativeTeleopTerrainMinimumStepM,
                    publish_count + 1,
                    slow_publish_count,
                    cloud_stamp_s,
                    last_terrain_risk_stamp_s);
            const auto write_start = SteadyClock::now();
            dds.writeTraversability(occupancy, cloud_stamp_s);
            timing.dds_write_ms += elapsedMs(write_start);
            ++publish_count;
          }
        } catch (const std::exception& exc) {
          ++error_count;
          std::fprintf(stderr, "traversability_dds: publish failed: %s\n", exc.what());
        }
      });
      timing.input_callbacks_ms = elapsedMs(input_start);

      const double now = nowSeconds();
      if (now >= next_status) {
        next_status = now + 5.0;
        std::fprintf(
            stderr,
            "traversability_dds: odom=%llu registered_clouds=%llu published=%llu slow=%llu errors=%llu points=%zu timing(loop=%.2f input=%.2f convert=%.2f occ=%.2f slow=%.2f core=%.2f clear=%.2f pack=%.2f write=%.2f overrun=%.2f)\n",
            static_cast<unsigned long long>(odom_count),
            static_cast<unsigned long long>(cloud_count),
            static_cast<unsigned long long>(publish_count),
            static_cast<unsigned long long>(slow_publish_count),
            static_cast<unsigned long long>(error_count),
            last_points,
            last_timing.loop_ms,
            last_timing.input_callbacks_ms,
            last_timing.cloud_convert_ms,
            last_timing.fast_occupancy_ms,
            last_timing.slow_terrain_ms,
            last_timing.terrain_core_ms,
            last_timing.dynamic_clear_ms,
            last_timing.terrain_pack_ms,
            last_timing.dds_write_ms,
            last_timing.overrun_ms);
        writeStatus(
            cfg,
            odom_count,
            tf_count,
            cloud_count,
            publish_count,
            slow_publish_count,
            map_clearing_count,
            cloud_clearing_count,
            tf_reject_count,
            odom_frame_reject_count,
            cloud_frame_reject_count,
            map_frame_jump_clear_count,
            error_count,
            last_points,
            last_terrain_points,
            last_terrain_ext_points,
            last_dynamic_clear_points,
            last_dynamic_clear_ray_points,
            last_dynamic_clear_evidence,
            last_dynamic_clear_free_voxels,
            cache_points,
            last_terrain_risk_cells,
            last_terrain_risk_max_cost,
            map_body.has_value(),
            map_odom.has_value(),
            map_odom && map_odom->stamp_s > 0.0 ? now - map_odom->stamp_s : -1.0,
            last_cloud_pose_gap_s,
            odom_frame_id,
            cloud_frame_id,
            last_frame_error,
            last_grid_diagnostics,
            last_forward_probe,
            last_timing);
      }

      const auto before_sleep = SteadyClock::now();
      if (before_sleep < next_tick) {
        timing.sleep_ms = elapsedMs(before_sleep, next_tick);
        std::this_thread::sleep_until(next_tick);
      } else {
        timing.overrun_ms = elapsedMs(next_tick, before_sleep);
        next_tick = before_sleep;
      }
      timing.loop_ms = elapsedMs(loop_start);
      if (timing.cloud_convert_ms > 0.0 || timing.fast_occupancy_ms > 0.0 ||
          timing.slow_terrain_ms > 0.0 || timing.terrain_core_ms > 0.0 ||
          timing.dynamic_clear_ms > 0.0 || timing.terrain_pack_ms > 0.0 ||
          timing.dds_write_ms > 0.0 ||
          timing.overrun_ms > 0.0) {
        last_timing = timing;
      }
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_traversability_dds failed: %s\n", exc.what());
    return 1;
  }
}
