#include "message/cpp/dds_topics.hpp"
#include "nav_kernel/map_layers_core.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <algorithm>
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
  double publish_hz{2.0};
  double tick_hz{20.0};
  double resolution{0.2};
  double radius{15.0};
  double z_min{-1.5};
  double z_max{1.2};
  double obstacle_min_z{0.10};
  double robot_radius{0.45};
  std::size_t max_points{20000};
  std::string status_file;
};

std::string envOrEmpty(const char* name) {
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
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
    } else if (arg == "--max-points") {
      cfg.max_points = static_cast<std::size_t>(std::stoull(next()));
    } else if (arg == "--status-file") {
      cfg.status_file = next();
    } else if (arg == "--help" || arg == "-h") {
      throw std::runtime_error(
          "usage: lingtu_traversability_dds [--domain-id N] [--publish-hz HZ] "
          "[--resolution M] [--radius M] [--max-points N] [--status-file PATH]");
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.publish_hz = std::max(0.2, cfg.publish_hz);
  cfg.tick_hz = std::max(1.0, cfg.tick_hz);
  cfg.resolution = std::max(0.05, cfg.resolution);
  cfg.radius = std::max(1.0, cfg.radius);
  return cfg;
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

void writeStatus(
    const CliConfig& cfg,
    std::uint64_t odom_count,
    std::uint64_t cloud_count,
    std::uint64_t publish_count,
    std::uint64_t error_count,
    std::size_t last_points,
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
      << "  \"has_odom\": " << (has_odom ? "true" : "false") << ",\n"
      << "  \"last_points\": " << last_points << ",\n"
      << "  \"counters\": {\n"
      << "    \"odom\": " << odom_count << ",\n"
      << "    \"registered_clouds\": " << cloud_count << ",\n"
      << "    \"published\": " << publish_count << ",\n"
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
    traversability_writer_ = writer(
        lingtu::message::kNavTraversability.dds_topic.data(),
        &lingtu_dds_OccupancyGrid_desc,
        "traversability");
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

  void writeTraversability(const nav_kernel::Grid2D& grid) {
    OccupancyMessage msg = toOccupancyMessage(grid);
    logDdsError(dds_write(traversability_writer_, &msg.msg), "dds_write(traversability)");
  }

 private:
  dds_entity_t reader(
      const char* topic_name,
      const dds_topic_descriptor_t* desc,
      const char* label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, desc, topic_name, nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    return checked(
        dds_create_reader(subscriber_, topic, nullptr, nullptr),
        (std::string("dds_create_reader(") + label + ")").c_str());
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
  dds_entity_t traversability_writer_{0};
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
    std::uint64_t error_count = 0;
    std::size_t last_points = 0;
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
            nav_kernel::Grid2D occupancy = buildOccupancyGrid(xyz, *pose, cfg);
            auto elevation = nav_kernel::buildElevationMap(
                xyz, pose->x, pose->y, cfg.resolution, cfg.radius, cfg.z_min, cfg.z_max);
            auto esdf = nav_kernel::computeEsdf(occupancy);
            auto risk = nav_kernel::computeTerrainRisk(elevation);
            nav_kernel::TraversabilityParams params;
            nav_kernel::Grid2D fused = nav_kernel::fuseTraversabilityCost(
                occupancy, risk.slopeDeg, esdf.distance, risk.risk, params);
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
            error_count,
            last_points,
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
