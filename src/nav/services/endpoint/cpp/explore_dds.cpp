#include "message/cpp/dds_topics.hpp"
#include "nav/commands/cpp/client.hpp"
#include "tare_policy.hpp"

#include "dds/dds.h"
#include "lingtu_slam.h"

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <memory>
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
  constexpr std::size_t kMaxSamples = 16;
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

struct Config {
  int domain_id = 232;
  double tick_hz = 2.0;
  double publish_period_s = 3.0;
  double min_republish_distance_m = 0.75;
  bool publish_waypoint = false;
  lingtu::explore::TarePolicyConfig policy;
};

Config parseArgs(int argc, char** argv) {
  Config cfg;
  if (const char* env = std::getenv("CYCLONEDDS_DOMAIN_ID")) {
    cfg.domain_id = std::atoi(env);
  }
  if (const char* env = std::getenv("LINGTU_EXPLORE_OUTPUT")) {
    cfg.publish_waypoint = std::string(env) == "way_point";
  }
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string(name) + " requires a value");
      }
      return argv[++i];
    };
    if (arg == "--domain") {
      cfg.domain_id = std::stoi(need_value("--domain"));
    } else if (arg == "--tick-hz") {
      cfg.tick_hz = std::stod(need_value("--tick-hz"));
    } else if (arg == "--publish-period") {
      cfg.publish_period_s = std::stod(need_value("--publish-period"));
    } else if (arg == "--output") {
      const std::string out = need_value("--output");
      if (out == "command") {
        cfg.publish_waypoint = false;
      } else if (out == "way_point") {
        cfg.publish_waypoint = true;
      } else {
        throw std::runtime_error("--output must be command or way_point");
      }
    } else if (arg == "--min-frontier-size") {
      cfg.policy.min_frontier_size = std::stoi(need_value("--min-frontier-size"));
    } else if (arg == "--sensor-range") {
      cfg.policy.sensor_range_m = std::stod(need_value("--sensor-range"));
    } else if (arg == "--candidate-radius") {
      cfg.policy.candidate_radius_m = std::stod(need_value("--candidate-radius"));
    } else if (arg == "--min-goal-distance") {
      cfg.policy.min_goal_distance_m = std::stod(need_value("--min-goal-distance"));
    } else if (arg == "--novelty-radius") {
      cfg.policy.novelty_radius_m = std::stod(need_value("--novelty-radius"));
    } else if (arg == "--max-candidates") {
      cfg.policy.max_candidates = std::stoi(need_value("--max-candidates"));
    } else if (arg == "--help" || arg == "-h") {
      std::printf(
          "usage: lingtu_explore_dds [--domain N] [--output command|way_point] "
          "[--tick-hz HZ] [--publish-period SEC]\n");
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }
  cfg.tick_hz = std::max(0.2, cfg.tick_hz);
  cfg.publish_period_s = std::max(0.1, cfg.publish_period_s);
  cfg.min_republish_distance_m = std::max(0.1, cfg.min_republish_distance_m);
  cfg.policy.min_frontier_size = std::max(1, cfg.policy.min_frontier_size);
  cfg.policy.max_candidates = std::max(1, cfg.policy.max_candidates);
  return cfg;
}

lingtu::explore::Pose2D toPose2D(const lingtu_dds_Odometry& msg) {
  lingtu::explore::Pose2D out;
  out.x = msg.pose.pose.position.x;
  out.y = msg.pose.pose.position.y;
  out.yaw = yawFromQuaternion(msg.pose.pose.orientation);
  return out;
}

lingtu::explore::Grid2D toExploreGrid(const lingtu_dds_OccupancyGrid& msg) {
  lingtu::explore::Grid2D grid;
  grid.width = static_cast<int>(msg.info.width);
  grid.height = static_cast<int>(msg.info.height);
  grid.resolution = static_cast<double>(msg.info.resolution);
  grid.origin_x = msg.info.origin.position.x;
  grid.origin_y = msg.info.origin.position.y;
  const std::size_t count = static_cast<std::size_t>(grid.width) *
                            static_cast<std::size_t>(grid.height);
  if (grid.width <= 0 || grid.height <= 0 || grid.resolution <= 0.0 ||
      msg.data._buffer == nullptr || msg.data._length < count) {
    return {};
  }
  grid.cells.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto raw = static_cast<unsigned int>(msg.data._buffer[i]);
    if (raw == 255u || raw > 100u) {
      grid.cells.push_back(lingtu::explore::kUnknown);
    } else if (raw >= 65u) {
      grid.cells.push_back(lingtu::explore::kOccupied);
    } else {
      grid.cells.push_back(lingtu::explore::kFree);
    }
  }
  return grid;
}

lingtu_dds_PoseStamped toGoalPose(double x, double y, double z) {
  lingtu_dds_PoseStamped out{};
  fillHeader(out.header, nowSeconds(), "map");
  out.pose.position.x = x;
  out.pose.position.y = y;
  out.pose.position.z = z;
  out.pose.orientation.w = 1.0;
  return out;
}

class DdsRuntime {
 public:
  explicit DdsRuntime(int domain_id, bool publish_waypoint) {
    participant_ = checked(
        dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
        "dds_create_participant");
    subscriber_ = checked(dds_create_subscriber(participant_, nullptr, nullptr),
                          "dds_create_subscriber");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher");

    odom_reader_ = reader(
        lingtu::message::kSlamOdometry.dds_topic.data(), &lingtu_dds_Odometry_desc, "odom");
    grid_reader_ = reader(
        lingtu::message::kNavExplorationGrid.dds_topic.data(),
        &lingtu_dds_OccupancyGrid_desc,
        "exploration_grid");
    if (publish_waypoint) {
      waypoint_writer_ = writer(
          lingtu::message::kNavWayPoint.dds_topic.data(),
          &lingtu_dds_PoseStamped_desc,
          "way_point");
    }
  }

  ~DdsRuntime() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  DdsRuntime(const DdsRuntime&) = delete;
  DdsRuntime& operator=(const DdsRuntime&) = delete;

  template <typename Handler>
  void drainOdometry(Handler&& handler) {
    drainReader<lingtu_dds_Odometry>(
        odom_reader_, lingtu_dds_Odometry_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainGrid(Handler&& handler) {
    drainReader<lingtu_dds_OccupancyGrid>(
        grid_reader_, lingtu_dds_OccupancyGrid_desc, std::forward<Handler>(handler));
  }

  void writeWayPoint(const lingtu_dds_PoseStamped& msg) {
    logDdsError(dds_write(waypoint_writer_, &msg), "dds_write(exploration_way_point)");
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
  dds_entity_t grid_reader_{0};
  dds_entity_t waypoint_writer_{0};
};

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, stopSignal);
  std::signal(SIGTERM, stopSignal);
  try {
    const Config cfg = parseArgs(argc, argv);
    DdsRuntime dds(cfg.domain_id, cfg.publish_waypoint);
    std::unique_ptr<lingtu::nav::commands::Client> command_client;
    if (!cfg.publish_waypoint) {
      command_client = std::make_unique<lingtu::nav::commands::Client>(cfg.domain_id);
    }
    lingtu::explore::TarePolicy policy(cfg.policy);

    std::optional<lingtu::explore::Pose2D> robot;
    std::optional<lingtu::explore::Grid2D> grid;
    std::vector<lingtu::explore::Pose2D> visited;
    double last_goal_s = -1.0;
    std::optional<lingtu::explore::Pose2D> last_goal;
    std::uint64_t odom_count = 0;
    std::uint64_t grid_count = 0;
    std::uint64_t goal_count = 0;
    std::string last_reason = "starting";

    std::fprintf(
        stderr,
        "lingtu_explore_dds: domain=%d tick_hz=%.1f output=%s\n",
        cfg.domain_id,
        cfg.tick_hz,
        cfg.publish_waypoint ? "way_point" : "typed_command_request_ack");

    const auto period = std::chrono::duration<double>(1.0 / cfg.tick_hz);
    while (g_running) {
      const auto loop_start = std::chrono::steady_clock::now();
      dds.drainOdometry([&](const lingtu_dds_Odometry& msg) {
        robot = toPose2D(msg);
        ++odom_count;
      });
      dds.drainGrid([&](const lingtu_dds_OccupancyGrid& msg) {
        auto parsed = toExploreGrid(msg);
        if (parsed.valid()) {
          grid = std::move(parsed);
          ++grid_count;
        }
      });

      if (robot.has_value() && grid.has_value()) {
        const auto decision = policy.select(*grid, *robot, visited);
        last_reason = decision.reason;
        if (decision.has_goal) {
          const double now = nowSeconds();
          const bool enough_time = last_goal_s < 0.0 || now - last_goal_s >= cfg.publish_period_s;
          const bool moved_goal = !last_goal.has_value() ||
              std::hypot(decision.goal_x - last_goal->x, decision.goal_y - last_goal->y) >=
                  cfg.min_republish_distance_m;
          if (enough_time && moved_goal) {
            const auto msg = toGoalPose(decision.goal_x, decision.goal_y, decision.goal_z);
            bool accepted = false;
            if (cfg.publish_waypoint) {
              dds.writeWayPoint(msg);
              accepted = true;
            } else {
              try {
                command_client->navigation().sendGoal(
                    decision.goal_x,
                    decision.goal_y,
                    decision.goal_z,
                    0.0,
                    10000);
                accepted = true;
              } catch (const std::exception& exc) {
                last_reason = std::string("navigation_rejected:") + exc.what();
                std::fprintf(
                    stderr,
                    "explore_dds: navigation command rejected: %s\n",
                    exc.what());
              }
            }
            if (!accepted) {
              last_goal_s = now;
            } else {
              last_goal = lingtu::explore::Pose2D{decision.goal_x, decision.goal_y, 0.0};
              last_goal_s = now;
              visited.push_back(*last_goal);
              ++goal_count;
            }
          }
        }
      } else {
        last_reason = robot.has_value() ? "waiting_for_exploration_grid" : "waiting_for_odometry";
      }

      static double last_log_s = 0.0;
      const double now = nowSeconds();
      if (now - last_log_s > 5.0) {
        last_log_s = now;
        std::fprintf(
            stderr,
            "explore_dds: odom=%llu grids=%llu goals=%llu reason=%s\n",
            static_cast<unsigned long long>(odom_count),
            static_cast<unsigned long long>(grid_count),
            static_cast<unsigned long long>(goal_count),
            last_reason.c_str());
      }

      const auto elapsed = std::chrono::steady_clock::now() - loop_start;
      if (elapsed < period) {
        std::this_thread::sleep_for(period - elapsed);
      }
    }
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_explore_dds failed: %s\n", exc.what());
    return 1;
  }
}
