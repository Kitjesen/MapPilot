#include "message/cpp/dds_topics.hpp"
#include "nav_kernel/map_layers_core.hpp"
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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
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

double yawFromQuaternion(const lingtu_dds_Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
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
    Handler&& handler) {
  constexpr std::size_t kMaxSamples = 8;
  void* samples[kMaxSamples];
  dds_sample_info_t infos[kMaxSamples];
  for (auto& sample : samples) {
    sample = dds_alloc(sizeof(T));
    std::memset(sample, 0, sizeof(T));
  }
  const dds_return_t count = dds_take(reader, samples, infos, kMaxSamples, kMaxSamples);
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
}

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

Pose2D toPose(const lingtu_dds_Odometry& msg) {
  Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;
  pose.z = msg.pose.pose.position.z;
  pose.yaw = yawFromQuaternion(msg.pose.pose.orientation);
  return pose;
}

struct FieldOffsets {
  int x{-1};
  int y{-1};
  int z{-1};
};

FieldOffsets fieldOffsets(const lingtu_dds_PointCloud2& msg) {
  FieldOffsets offsets;
  for (std::uint32_t i = 0; i < msg.fields._length; ++i) {
    const auto& field = msg.fields._buffer[i];
    const std::string name = field.name ? field.name : "";
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

std::vector<float> cloudToMapXyz(
    const lingtu_dds_PointCloud2& msg,
    const Pose2D& pose,
    std::size_t max_points) {
  std::vector<float> out;
  const FieldOffsets offsets = fieldOffsets(msg);
  if (offsets.x < 0 || offsets.y < 0 || offsets.z < 0 ||
      msg.point_step < 12 || msg.data._buffer == nullptr) {
    return out;
  }
  const std::size_t count =
      std::min<std::size_t>(msg.width * std::max<std::uint32_t>(1, msg.height),
                            msg.data._length / std::max<std::uint32_t>(1, msg.point_step));
  if (count == 0) {
    return out;
  }
  const std::size_t stride = max_points > 0 && count > max_points
      ? static_cast<std::size_t>(std::ceil(static_cast<double>(count) / max_points))
      : 1;
  const std::string frame_id = msg.header.frame_id ? msg.header.frame_id : "";
  const bool fixed_frame = frame_id == "map" || frame_id == "odom";
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  out.reserve((count / stride + 1) * 3);
  for (std::size_t i = 0; i < count; i += stride) {
    const auto* base = msg.data._buffer + i * msg.point_step;
    const float x = readFloat(base + offsets.x);
    const float y = readFloat(base + offsets.y);
    const float z = readFloat(base + offsets.z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    if (fixed_frame) {
      out.push_back(x);
      out.push_back(y);
      out.push_back(z);
    } else {
      out.push_back(static_cast<float>(pose.x + c * x - s * y));
      out.push_back(static_cast<float>(pose.y + s * x + c * y));
      out.push_back(static_cast<float>(pose.z + z));
    }
  }
  return out;
}

struct CliConfig {
  int domain_id{0};
  double publish_hz{10.0};
  double tick_hz{50.0};
  double resolution{0.2};
  double radius{15.0};
  double z_min{-1.5};
  double z_max{1.2};
  double obstacle_min_z{0.10};
  double robot_radius{0.45};
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
  std::size_t max_points{20000};
  std::size_t terrain_cache_max_points{80000};
  std::string status_file;
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
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(next());
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
    } else if (arg == "--max-points") {
      cfg.max_points = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_traversability_dds [--domain-id N] [--publish-hz HZ] "
          "[--resolution M] [--radius M] [--max-points N] "
          "[--terrain-decay-s S] [--terrain-min-block-points N] "
          "[--terrain-quantile Q] [--terrain-cache-max-points N] "
          "[--terrain-clear-dy-obs BOOL] [--terrain-no-data-obstacle BOOL] "
          "[--status-file PATH]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.publish_hz = std::max(0.2, cfg.publish_hz);
  cfg.tick_hz = std::max(1.0, cfg.tick_hz);
  cfg.resolution = std::max(0.05, cfg.resolution);
  cfg.radius = std::max(1.0, cfg.radius);
  cfg.terrain_decay_s = std::max(0.1, cfg.terrain_decay_s);
  cfg.terrain_no_decay_radius = std::max(0.0, cfg.terrain_no_decay_radius);
  cfg.terrain_quantile = std::max(0.0, std::min(1.0, cfg.terrain_quantile));
  cfg.vehicle_height = std::max(0.1, cfg.vehicle_height);
  cfg.terrain_min_block_points = std::max(1, cfg.terrain_min_block_points);
  cfg.terrain_min_dy_obs_dis = std::max(0.0, cfg.terrain_min_dy_obs_dis);
  cfg.terrain_min_dy_obs_point_num = std::max(1, cfg.terrain_min_dy_obs_point_num);
  cfg.terrain_no_data_block_skip_num = std::max(0, cfg.terrain_no_data_block_skip_num);
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

struct TerrainCachePoint {
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  double stamp_s{0.0};
};

void updateTerrainCache(
    std::vector<TerrainCachePoint>& cache,
    const std::vector<float>& xyz,
    const Pose2D& pose,
    const CliConfig& cfg,
    double stamp_s) {
  const std::size_t n = xyz.size() / 3;
  cache.reserve(std::min<std::size_t>(cfg.terrain_cache_max_points, cache.size() + n));
  for (std::size_t i = 0; i < n; ++i) {
    const float x = xyz[i * 3 + 0];
    const float y = xyz[i * 3 + 1];
    const float z = xyz[i * 3 + 2];
    const double dis = std::hypot(static_cast<double>(x) - pose.x, static_cast<double>(y) - pose.y);
    const double rel_z = static_cast<double>(z) - pose.z;
    if (rel_z > cfg.z_min - cfg.terrain_dis_ratio_z * dis &&
        rel_z < cfg.z_max + cfg.terrain_dis_ratio_z * dis &&
        dis < cfg.radius) {
      cache.push_back({x, y, z, stamp_s});
    }
  }

  cache.erase(
      std::remove_if(
          cache.begin(),
          cache.end(),
          [&](const TerrainCachePoint& p) {
            const double dis =
                std::hypot(static_cast<double>(p.x) - pose.x, static_cast<double>(p.y) - pose.y);
            return stamp_s - p.stamp_s > cfg.terrain_decay_s &&
                   dis >= cfg.terrain_no_decay_radius;
          }),
      cache.end());

  if (cache.size() > cfg.terrain_cache_max_points) {
    cache.erase(cache.begin(), cache.begin() + static_cast<std::ptrdiff_t>(
                                     cache.size() - cfg.terrain_cache_max_points));
  }
}

std::vector<float> cacheToXyz(
    const std::vector<TerrainCachePoint>& cache,
    std::size_t max_points) {
  std::vector<float> out;
  if (cache.empty()) {
    return out;
  }
  const std::size_t stride = max_points > 0 && cache.size() > max_points
      ? static_cast<std::size_t>(std::ceil(static_cast<double>(cache.size()) / max_points))
      : 1;
  out.reserve((cache.size() / stride + 1) * 3);
  for (std::size_t i = 0; i < cache.size(); i += stride) {
    out.push_back(cache[i].x);
    out.push_back(cache[i].y);
    out.push_back(cache[i].z);
  }
  return out;
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

nav_kernel::Grid2D buildOccupancyGrid(
    const std::vector<float>& xyz,
    const Pose2D& pose,
    const CliConfig& cfg) {
  const int size = static_cast<int>(std::floor((2.0 * cfg.radius) / cfg.resolution));
  nav_kernel::Grid2D grid = nav_kernel::makeGrid2D(
      size, size, cfg.resolution, pose.x - cfg.radius, pose.y - cfg.radius, 0.0f);
  const int inflate = std::max(0, static_cast<int>(std::ceil(cfg.robot_radius / cfg.resolution)));
  const std::size_t n = xyz.size() / 3;
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
    for (int dr = -inflate; dr <= inflate; ++dr) {
      for (int dc = -inflate; dc <= inflate; ++dc) {
        const int rr = row + dr;
        const int cc = col + dc;
        if (rr < 0 || rr >= grid.rows || cc < 0 || cc >= grid.cols) {
          continue;
        }
        if (dr * dr + dc * dc > inflate * inflate) {
          continue;
        }
        grid.data[static_cast<std::size_t>(grid.index(rr, cc))] = 100.0f;
      }
    }
  }
  return grid;
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

OccupancyMessage toOccupancyMessage(const nav_kernel::Grid2D& grid) {
  OccupancyMessage out;
  const double stamp = nowSeconds();
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
    const float cost = std::isfinite(v) ? nav_kernel::clampFloat(v, 0.0f, 100.0f) : 100.0f;
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
    std::uint64_t cloud_count,
    std::uint64_t publish_count,
    std::uint64_t map_clearing_count,
    std::uint64_t cloud_clearing_count,
    std::uint64_t error_count,
    std::size_t last_points,
    std::size_t terrain_points,
    std::size_t terrain_ext_points,
    std::size_t cache_points,
    bool has_odom) {
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
  out << "{\n"
      << "  \"schema_version\": \"lingtu.traversability.status.v1\",\n"
      << "  \"endpoint\": \"lingtu_traversability_dds\",\n"
      << "  \"stamp_s\": " << nowSeconds() << ",\n"
      << "  \"domain_id\": " << cfg.domain_id << ",\n"
      << "  \"publish_hz\": " << cfg.publish_hz << ",\n"
      << "  \"resolution\": " << cfg.resolution << ",\n"
      << "  \"radius\": " << cfg.radius << ",\n"
      << "  \"terrain_clear_dy_obs\": " << (cfg.terrain_clear_dy_obs ? "true" : "false") << ",\n"
      << "  \"terrain_no_data_obstacle\": " << (cfg.terrain_no_data_obstacle ? "true" : "false") << ",\n"
      << "  \"has_odom\": " << (has_odom ? "true" : "false") << ",\n"
      << "  \"last_points\": " << last_points << ",\n"
      << "  \"terrain_points\": " << terrain_points << ",\n"
      << "  \"terrain_map_ext_points\": " << terrain_ext_points << ",\n"
      << "  \"cache_points\": " << cache_points << ",\n"
      << "  \"counters\": {\n"
      << "    \"odom\": " << odom_count << ",\n"
      << "    \"registered_clouds\": " << cloud_count << ",\n"
      << "    \"published\": " << publish_count << ",\n"
      << "    \"map_clearing\": " << map_clearing_count << ",\n"
      << "    \"cloud_clearing\": " << cloud_clearing_count << ",\n"
      << "    \"errors\": " << error_count << "\n"
      << "  }\n"
      << "}\n";
  out.close();
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
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
    cloud_reader_ = reader(
        lingtu::message::kSlamRegisteredCloud.dds_topic.data(),
        &lingtu_dds_PointCloud2_desc,
        "registered_cloud");
    map_clearing_reader_ = reader(
        lingtu::message::kNavMapClearing.dds_topic.data(),
        &lingtu_dds_Bool_desc,
        "map_clearing",
        true);
    cloud_clearing_reader_ = reader(
        lingtu::message::kNavCloudClearing.dds_topic.data(),
        &lingtu_dds_Bool_desc,
        "cloud_clearing",
        true);
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

  void writeTraversability(const nav_kernel::Grid2D& grid) {
    OccupancyMessage msg = toOccupancyMessage(grid);
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
      const char* label,
      bool reliable = false) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    dds_qos_t* qos = nullptr;
    if (reliable) {
      qos = dds_create_qos();
      dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(1));
    }
    const dds_entity_t entity = checked(
        dds_create_reader(subscriber_, topic, qos, nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
    if (qos != nullptr) {
      dds_delete_qos(qos);
    }
    return entity;
  }

  dds_entity_t writer(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    return checked(
        dds_create_writer(publisher_, topic, nullptr, nullptr),
        (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t odom_reader_{0};
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

    std::optional<Pose2D> pose;
    std::uint64_t odom_count = 0;
    std::uint64_t cloud_count = 0;
    std::uint64_t publish_count = 0;
    std::uint64_t map_clearing_count = 0;
    std::uint64_t cloud_clearing_count = 0;
    std::uint64_t error_count = 0;
    std::size_t last_points = 0;
    std::size_t last_terrain_points = 0;
    std::size_t last_terrain_ext_points = 0;
    std::size_t cache_points = 0;
    std::vector<TerrainCachePoint> terrain_cache;
    nav_kernel::TerrainAnalysisCore terrain_core(terrainParamsFromConfig(cfg));
    double next_publish = 0.0;
    double next_status = nowSeconds() + 5.0;

    std::fprintf(
        stderr,
        "lingtu_traversability_dds: domain=%d publish_hz=%.2f resolution=%.2f radius=%.1f\n",
        cfg.domain_id,
        cfg.publish_hz,
        cfg.resolution,
        cfg.radius);

    while (g_running) {
      dds.drainOdometry([&](const lingtu_dds_Odometry& msg) {
        pose = toPose(msg);
        ++odom_count;
      });
      dds.drainMapClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        terrain_cache.clear();
        terrain_core.clear();
        cache_points = 0;
        last_terrain_points = 0;
        last_terrain_ext_points = 0;
        ++map_clearing_count;
      });
      dds.drainCloudClearing([&](const lingtu_dds_Bool& msg) {
        if (!msg.data) {
          return;
        }
        terrain_cache.clear();
        terrain_core.clear();
        cache_points = 0;
        last_terrain_points = 0;
        last_terrain_ext_points = 0;
        ++cloud_clearing_count;
      });
      dds.drainCloud([&](const lingtu_dds_PointCloud2& msg) {
        ++cloud_count;
        const double now = nowSeconds();
        if (!pose || now < next_publish) {
          return;
        }
        next_publish = now + 1.0 / cfg.publish_hz;
        try {
          const std::vector<float> xyz = cloudToMapXyz(msg, *pose, cfg.max_points);
          last_points = xyz.size() / 3;
          if (!xyz.empty()) {
            updateTerrainCache(terrain_cache, xyz, *pose, cfg, now);
            cache_points = terrain_cache.size();
            const std::vector<float> terrain_xyz = cacheToXyz(terrain_cache, cfg.max_points);
            nav_kernel::Grid2D occupancy = buildOccupancyGrid(terrain_xyz, *pose, cfg);
            auto elevation = nav_kernel::buildElevationMap(
                terrain_xyz, pose->x, pose->y, cfg.resolution, cfg.radius, cfg.z_min, cfg.z_max);
            auto esdf = nav_kernel::computeEsdf(occupancy);
            auto risk = nav_kernel::computeTerrainRisk(elevation);
            nav_kernel::TraversabilityParams params;
            nav_kernel::Grid2D fused = nav_kernel::fuseTraversabilityCost(
                occupancy, risk.slopeDeg, esdf.distance, risk.risk, params);
            const std::vector<float> scan_xyzi = xyzToXyzi(xyz);
            terrain_core.updateVehicle(pose->x, pose->y, pose->z, 0.0, 0.0, pose->yaw);
            const nav_kernel::TerrainResult terrain_result =
                terrain_core.process(scan_xyzi.data(), static_cast<int>(scan_xyzi.size() / 4), now);
            const std::vector<float> terrain_xyzi =
                limitXyzi(terrain_result.terrain_points, cfg.max_points);
            last_terrain_points = terrain_xyzi.size() / 4;
            if (!terrain_xyzi.empty()) {
              dds.writeTerrainMap(terrain_xyzi, now);
              dds.writeTerrainMapExt(terrain_xyzi, now);
            }
            last_terrain_ext_points = last_terrain_points;
            dds.writeTraversability(fused);
            ++publish_count;
          }
        } catch (const std::exception& exc) {
          ++error_count;
          std::fprintf(stderr, "traversability_dds: publish failed: %s\n", exc.what());
        }
      });

      const double now = nowSeconds();
      if (now >= next_status) {
        next_status = now + 5.0;
        std::fprintf(
            stderr,
            "traversability_dds: odom=%llu registered_clouds=%llu published=%llu errors=%llu points=%zu\n",
            static_cast<unsigned long long>(odom_count),
            static_cast<unsigned long long>(cloud_count),
            static_cast<unsigned long long>(publish_count),
            static_cast<unsigned long long>(error_count),
            last_points);
        writeStatus(
            cfg,
            odom_count,
            cloud_count,
            publish_count,
            map_clearing_count,
            cloud_clearing_count,
            error_count,
            last_points,
            last_terrain_points,
            last_terrain_ext_points,
            cache_points,
            pose.has_value());
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(static_cast<int>(1000.0 / cfg.tick_hz)));
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_traversability_dds failed: %s\n", exc.what());
    return 1;
  }
}
