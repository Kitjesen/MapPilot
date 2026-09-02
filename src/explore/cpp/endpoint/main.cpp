#include <algorithm>
#include <atomic>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dds/dds.h"
#include "endpoint/directed_intent.hpp"
#include "endpoint/run_event_outbox.hpp"
#include "endpoint/run_lifecycle.hpp"
#include "endpoint/segment_lifecycle.hpp"
#include "endpoint/control.hpp"
#include "endpoint/goal_command_lane.hpp"
#include "endpoint/goal_lifecycle.hpp"
#include "endpoint/input_gate.hpp"
#include "endpoint/status_identity.hpp"
#include "endpoint/route.hpp"
#include "endpoint/saved_coverage_grid.hpp"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "message/cpp/exploration_command.hpp"
#include "message/cpp/navigation_command.hpp"
#include "nav/cpp/platform/runtime.hpp"
#include "input/active/occupancy.hpp"
#include "status/status_snapshot_file_writer.hpp"
#include "tare_policy.hpp"

namespace {

using lingtu::explore::ExploreDecision;
using lingtu::explore::ExploreDiagnostics;
using lingtu::explore::ExploreMapIdentity;
using lingtu::explore::Pose2D;
using lingtu::nav::endpoint::DirectedExplorationIntent;
using lingtu::nav::endpoint::ExplorationSnapshot;
using lingtu::nav::endpoint::RigidTransform;
using lingtu::nav::endpoint::Route;
using lingtu::nav::endpoint::TimedMapPose;

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double nowSeconds() {
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

void fillHeader(lingtu_dds_Header &header, double stamp_s, const char *frame_id) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(header.stamp.sec)) * 1e9);
  header.frame_id = const_cast<char *>(frame_id);
}

double stampSeconds(const lingtu_dds_Time &stamp) {
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string stringValue(const char *value) {
  return value == nullptr ? std::string{} : std::string(value);
}

dds_entity_t checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

void logDdsError(dds_return_t value, const char *what) {
  if (value < 0) {
    std::fprintf(stderr, "%s: %s\n", what, dds_strretcode(-value));
  }
}

template <typename T, typename Handler>
void drainReader(dds_entity_t reader, const dds_topic_descriptor_t &, Handler &&handler) {
  constexpr std::size_t kMaxSamples = 64U;
  void *samples[kMaxSamples]{};
  dds_sample_info_t infos[kMaxSamples]{};
  const dds_return_t count = dds_take(reader, samples, infos, kMaxSamples, kMaxSamples);
  if (count < 0) {
    logDdsError(count, "dds_take(explore)");
    return;
  }

  try {
    for (dds_return_t index = 0; index < count; ++index) {
      if (infos[index].valid_data) {
        handler(*static_cast<T *>(samples[index]));
      }
    }
  } catch (...) {
    if (count > 0) {
      logDdsError(dds_return_loan(reader, samples, count), "dds_return_loan(explore)");
    }
    throw;
  }
  if (count > 0) {
    checked(dds_return_loan(reader, samples, count), "dds_return_loan(explore)");
  }
}

double monotonicSeconds() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

bool parseBool(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (value == "1" || value == "true" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::runtime_error("expected boolean value, got: " + value);
}

std::string envString(const char *name) {
  const char *value = std::getenv(name);
  return value == nullptr ? std::string{} : std::string(value);
}

void envDouble(const char *name, double &target) {
  const std::string value = envString(name);
  if (!value.empty()) {
    target = std::stod(value);
  }
}

void envInt(const char *name, int &target) {
  const std::string value = envString(name);
  if (!value.empty()) {
    target = std::stoi(value);
  }
}

void envSize(const char *name, std::size_t &target) {
  const std::string value = envString(name);
  if (!value.empty()) {
    target = static_cast<std::size_t>(std::stoull(value));
  }
}

void envBool(const char *name, bool &target) {
  const std::string value = envString(name);
  if (!value.empty()) {
    target = parseBool(value);
  }
}

std::int64_t positiveInt64Env(const char *name) {
  const std::string value = envString(name);
  std::int64_t parsed_value = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), parsed_value);
  if (value.empty() || parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
      parsed_value <= 0) {
    throw std::runtime_error(std::string(name) + " must be a positive integer");
  }
  return parsed_value;
}

std::string makeExploreBootId() {
  return lingtu::nav::platform::producerBootId(lingtu::nav::platform::hostBootId(),
                                               lingtu::nav::platform::bootTimeNanoseconds());
}

struct Config {
  int domain_id{0};
  std::optional<Route> route;
  double tick_hz{2.0};
  double command_retry_s{3.0};
  double arrival_tolerance_m{0.65};
  double goal_timeout_s{180.0};
  double status_period_s{2.0};
  double control_max_age_s{2.0};
  int command_timeout_ms{10000};
  double directed_target_max_ttl_s{3600.0};
  double directed_progress_weight{4.0};
  std::string status_file;
  std::string product;
  std::string product_session_id;
  std::string expected_map_id;
  std::int64_t expected_map_content_epoch{0};
  std::string expected_map_frame;
  std::filesystem::path occupancy_path;
  lingtu::nav::endpoint::ExploreInputGateConfig input_gate;
  lingtu::explore::TarePolicyConfig policy;
};

Config parseArgs(int argc, char **argv) {
  Config config;
  if (const char *domain = std::getenv("CYCLONEDDS_DOMAIN_ID")) {
    config.domain_id = std::stoi(domain);
  }
  config.product = envString("LINGTU_PRODUCT");
  config.status_file = envString("LINGTU_EXPLORE_STATUS_FILE");
  if (config.status_file.empty()) {
    const std::string session_root = envString("LINGTU_SESSION_ROOT");
    if (!session_root.empty()) {
      config.status_file = (std::filesystem::path(session_root) / "explore.status.json").string();
    }
  }
  config.product_session_id = envString("LINGTU_PRODUCT_SESSION_ID");
  config.occupancy_path = envString("EXPLORE_OCCUPANCY_PATH");
  const std::string route_env = envString("LINGTU_EXPLORE_ROUTE");
  if (!route_env.empty()) {
    const auto route = lingtu::nav::endpoint::parseRoute(route_env);
    if (!route.has_value()) {
      throw std::runtime_error("LINGTU_EXPLORE_ROUTE must be map or live");
    }
    config.route = *route;
  }

  envDouble("LINGTU_EXPLORE_TICK_HZ", config.tick_hz);
  envDouble("LINGTU_EXPLORE_COMMAND_RETRY_S", config.command_retry_s);
  envInt("LINGTU_EXPLORE_COMMAND_TIMEOUT_MS", config.command_timeout_ms);
  envDouble("LINGTU_EXPLORE_ARRIVAL_TOLERANCE_M", config.arrival_tolerance_m);
  envDouble("LINGTU_EXPLORE_GOAL_TIMEOUT_S", config.goal_timeout_s);
  envDouble("LINGTU_EXPLORE_STATUS_PERIOD_S", config.status_period_s);
  envDouble("LINGTU_EXPLORE_CONTROL_MAX_AGE_S", config.control_max_age_s);
  envDouble("LINGTU_EXPLORE_ODOM_MAX_AGE_S", config.input_gate.odometry_max_age_s);
  envDouble("LINGTU_EXPLORE_SNAPSHOT_MAX_AGE_S", config.input_gate.snapshot_max_age_s);
  envDouble("LINGTU_EXPLORE_TF_MAX_AGE_S", config.input_gate.transform_max_age_s);
  envDouble("LINGTU_EXPLORE_FUTURE_TOLERANCE_S", config.input_gate.future_tolerance_s);
  envInt("LINGTU_EXPLORE_MIN_FRONTIER_SIZE", config.policy.min_frontier_size);
  envDouble("LINGTU_EXPLORE_SENSOR_RANGE_M", config.policy.sensor_range_m);
  envDouble("LINGTU_EXPLORE_CANDIDATE_RADIUS_M", config.policy.candidate_radius_m);
  envDouble("LINGTU_EXPLORE_MIN_GOAL_DISTANCE_M", config.policy.min_goal_distance_m);
  envDouble("LINGTU_EXPLORE_NOVELTY_RADIUS_M", config.policy.novelty_radius_m);
  envInt("LINGTU_EXPLORE_MAX_CANDIDATES", config.policy.max_candidates);
  envDouble("LINGTU_EXPLORE_COVERAGE_RESOLUTION_M", config.policy.coverage_resolution_m);
  envDouble("LINGTU_EXPLORE_LOCAL_ROUTE_RADIUS_M", config.policy.local_route_radius_m);
  envDouble("LINGTU_EXPLORE_RETURN_HOME_DISTANCE_M", config.policy.return_home_distance_m);
  envDouble("LINGTU_EXPLORE_KEYPOSE_MIN_DISTANCE_M", config.policy.keypose_min_distance_m);
  envDouble("LINGTU_EXPLORE_KEYPOSE_CONNECT_DISTANCE_M", config.policy.keypose_connect_distance_m);
  envDouble("LINGTU_EXPLORE_GAIN_WEIGHT", config.policy.gain_weight);
  envDouble("LINGTU_EXPLORE_TRAVEL_WEIGHT", config.policy.travel_weight);
  envDouble("LINGTU_EXPLORE_MOMENTUM_WEIGHT", config.policy.momentum_weight);
  envDouble("LINGTU_EXPLORE_REVISIT_WEIGHT", config.policy.revisit_weight);
  envDouble("LINGTU_EXPLORE_MAX_PLAN_TIME_MS", config.policy.max_plan_time_ms);
  envInt("LINGTU_EXPLORE_ROUTE_2OPT_ITERATIONS", config.policy.route_2opt_iterations);
  envSize("LINGTU_EXPLORE_MAX_GRID_CELLS", config.policy.max_grid_cells);
  config.input_gate.max_grid_cells = config.policy.max_grid_cells;
  envSize("LINGTU_EXPLORE_MAX_FRONTIER_CELLS", config.policy.max_frontier_cells);
  envSize("LINGTU_EXPLORE_MAX_FRONTIER_CLUSTERS", config.policy.max_frontier_clusters);
  envSize("LINGTU_EXPLORE_MAX_COVERAGE_CELLS", config.policy.max_coverage_cells);
  envSize("LINGTU_EXPLORE_MAX_KEYPOSES", config.policy.max_keyposes);
  envSize("LINGTU_EXPLORE_MAX_KEYPOSE_EDGES", config.policy.max_keypose_edges);
  envSize("LINGTU_EXPLORE_MAX_KEYPOSE_NEIGHBOR_LINKS", config.policy.max_keypose_neighbor_links);
  envSize("LINGTU_EXPLORE_MAX_ROUTE_TARGETS", config.policy.max_route_targets);
  envBool("LINGTU_EXPLORE_RETURN_HOME", config.policy.return_home_when_done);

  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    auto value = [&]() -> std::string {
      if (index + 1 >= argc) {
        throw std::runtime_error(arg + " requires a value");
      }
      return argv[++index];
    };
    if (arg == "--domain") {
      config.domain_id = std::stoi(value());
    } else if (arg == "--route") {
      const std::string route_value = value();
      const auto route = lingtu::nav::endpoint::parseRoute(route_value);
      if (!route.has_value()) {
        throw std::runtime_error("--route must be map or live");
      }
      config.route = *route;
    } else if (arg == "--tick-hz") {
      config.tick_hz = std::stod(value());
    } else if (arg == "--command-retry-s") {
      config.command_retry_s = std::stod(value());
    } else if (arg == "--arrival-tolerance") {
      config.arrival_tolerance_m = std::stod(value());
    } else if (arg == "--goal-timeout") {
      config.goal_timeout_s = std::stod(value());
    } else if (arg == "--status-period") {
      config.status_period_s = std::stod(value());
    } else if (arg == "--status-file") {
      config.status_file = value();
    } else if (arg == "--command-timeout-ms") {
      config.command_timeout_ms = std::stoi(value());
    } else if (arg == "--control-max-age") {
      config.control_max_age_s = std::stod(value());
    } else if (arg == "--directed-target-max-ttl") {
      config.directed_target_max_ttl_s = std::stod(value());
    } else if (arg == "--directed-progress-weight") {
      config.directed_progress_weight = std::stod(value());
    } else if (arg == "--odom-max-age") {
      config.input_gate.odometry_max_age_s = std::stod(value());
    } else if (arg == "--snapshot-max-age") {
      config.input_gate.snapshot_max_age_s = std::stod(value());
    } else if (arg == "--tf-max-age") {
      config.input_gate.transform_max_age_s = std::stod(value());
    } else if (arg == "--future-tolerance") {
      config.input_gate.future_tolerance_s = std::stod(value());
    } else if (arg == "--min-frontier-size") {
      config.policy.min_frontier_size = std::stoi(value());
    } else if (arg == "--sensor-range") {
      config.policy.sensor_range_m = std::stod(value());
    } else if (arg == "--candidate-radius") {
      config.policy.candidate_radius_m = std::stod(value());
    } else if (arg == "--min-goal-distance") {
      config.policy.min_goal_distance_m = std::stod(value());
    } else if (arg == "--novelty-radius") {
      config.policy.novelty_radius_m = std::stod(value());
    } else if (arg == "--max-candidates") {
      config.policy.max_candidates = std::stoi(value());
    } else if (arg == "--coverage-resolution") {
      config.policy.coverage_resolution_m = std::stod(value());
    } else if (arg == "--local-route-radius") {
      config.policy.local_route_radius_m = std::stod(value());
    } else if (arg == "--return-home-distance") {
      config.policy.return_home_distance_m = std::stod(value());
    } else if (arg == "--keypose-min-distance") {
      config.policy.keypose_min_distance_m = std::stod(value());
    } else if (arg == "--keypose-connect-distance") {
      config.policy.keypose_connect_distance_m = std::stod(value());
    } else if (arg == "--gain-weight") {
      config.policy.gain_weight = std::stod(value());
    } else if (arg == "--travel-weight") {
      config.policy.travel_weight = std::stod(value());
    } else if (arg == "--momentum-weight") {
      config.policy.momentum_weight = std::stod(value());
    } else if (arg == "--revisit-weight") {
      config.policy.revisit_weight = std::stod(value());
    } else if (arg == "--max-plan-time-ms") {
      config.policy.max_plan_time_ms = std::stod(value());
    } else if (arg == "--route-2opt-iterations") {
      config.policy.route_2opt_iterations = std::stoi(value());
    } else if (arg == "--max-grid-cells") {
      config.policy.max_grid_cells = static_cast<std::size_t>(std::stoull(value()));
      config.input_gate.max_grid_cells = config.policy.max_grid_cells;
    } else if (arg == "--max-frontier-cells") {
      config.policy.max_frontier_cells = static_cast<std::size_t>(std::stoull(value()));
    } else if (arg == "--max-frontier-clusters") {
      config.policy.max_frontier_clusters = static_cast<std::size_t>(std::stoull(value()));
    } else if (arg == "--max-coverage-cells") {
      config.policy.max_coverage_cells = static_cast<std::size_t>(std::stoull(value()));
    } else if (arg == "--max-keyposes") {
      config.policy.max_keyposes = static_cast<std::size_t>(std::stoull(value()));
    } else if (arg == "--max-keypose-edges") {
      config.policy.max_keypose_edges = static_cast<std::size_t>(std::stoull(value()));
    } else if (arg == "--max-keypose-neighbor-links") {
      config.policy.max_keypose_neighbor_links = static_cast<std::size_t>(std::stoull(value()));
    } else if (arg == "--max-route-targets") {
      config.policy.max_route_targets = static_cast<std::size_t>(std::stoull(value()));
    } else if (arg == "--return-home") {
      config.policy.return_home_when_done = parseBool(value());
    } else if (arg == "--help" || arg == "-h") {
      std::printf(
          "usage: lingtu_explore_dds [--domain N] [--route map|live] [--tick-hz HZ] "
          "[--status-file PATH] [--arrival-tolerance M] "
          "[--goal-timeout SEC] [TARE policy options]\n");
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + arg);
    }
  }

  if (!config.route.has_value()) {
    throw std::runtime_error("LINGTU_EXPLORE_ROUTE or --route is required");
  }
  const std::string runtime_env = envString("LINGTU_ENV");
  if (runtime_env == "sim") {
    if (config.product.empty() ||
        std::any_of(config.product.begin(), config.product.end(),
                    [](unsigned char ch) { return std::isspace(ch) != 0; })) {
      throw std::runtime_error(
          "LINGTU_PRODUCT must be non-empty text without whitespace in simulation");
    }
    if (config.product_session_id.empty()) {
      throw std::runtime_error("LINGTU_PRODUCT_SESSION_ID is required in simulation");
    }
  }

  auto positive = [](double value, const char *name) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::runtime_error(std::string(name) + " must be positive");
    }
  };
  positive(config.tick_hz, "tick_hz");
  positive(config.command_retry_s, "command_retry_s");
  positive(config.arrival_tolerance_m, "arrival_tolerance_m");
  positive(config.goal_timeout_s, "goal_timeout_s");
  positive(config.status_period_s, "status_period_s");
  positive(config.control_max_age_s, "control_max_age_s");
  positive(config.directed_target_max_ttl_s, "directed_target_max_ttl_s");
  if (config.directed_target_max_ttl_s >
      lingtu::nav::endpoint::DirectedExplorationIntentStore::kMaxTtlSeconds) {
    throw std::runtime_error("directed_target_max_ttl_s exceeds hard limit");
  }
  if (!std::isfinite(config.directed_progress_weight) || config.directed_progress_weight < 0.0) {
    throw std::runtime_error("directed_progress_weight must be non-negative");
  }
  config.policy.directed_progress_weight = config.directed_progress_weight;
  positive(config.input_gate.odometry_max_age_s, "odometry_max_age_s");
  positive(config.input_gate.snapshot_max_age_s, "snapshot_max_age_s");
  positive(config.input_gate.transform_max_age_s, "transform_max_age_s");
  if (!std::isfinite(config.input_gate.future_tolerance_s) ||
      config.input_gate.future_tolerance_s < 0.0) {
    throw std::runtime_error("future_tolerance_s must be non-negative");
  }
  if (config.command_timeout_ms <= 0) {
    throw std::runtime_error("command_timeout_ms must be positive");
  }
  if (config.input_gate.max_grid_cells == 0U) {
    throw std::runtime_error("max_grid_cells must be positive");
  }
  if (config.product_session_id.empty() || config.product_session_id.size() > 128U) {
    throw std::runtime_error("LINGTU_PRODUCT_SESSION_ID is required for exploration control");
  }
  if (*config.route == Route::Map && config.occupancy_path.empty()) {
    throw std::runtime_error("EXPLORE_OCCUPANCY_PATH is required for the map route");
  }
  if (*config.route == Route::Map) {
    config.expected_map_id = envString("LINGTU_MAP_ID");
    if (config.expected_map_id.empty()) {
      throw std::runtime_error("LINGTU_MAP_ID is required for the map route");
    }
    config.expected_map_content_epoch = positiveInt64Env("LINGTU_MAP_CONTENT_EPOCH");
    config.expected_map_frame = envString("LINGTU_MAP_FRAME");
    if (config.expected_map_frame.empty()) {
      throw std::runtime_error("LINGTU_MAP_FRAME is required for the map route");
    }
  }
  return config;
}

class DdsRuntime {
 public:
  explicit DdsRuntime(int domain_id) {
    participant_ =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(explore)");
    subscriber_ = checked(dds_create_subscriber(participant_, nullptr, nullptr),
                          "dds_create_subscriber(explore)");
    publisher_ = checked(dds_create_publisher(participant_, nullptr, nullptr),
                         "dds_create_publisher(explore)");
    odometry_reader_ =
        reader(lingtu::message::kSlamOdometry, &lingtu_dds_Odometry_desc, "odometry");
    tf_reader_ = reader(lingtu::message::kTf, &lingtu_dds_TFMessage_desc, "tf");
    tf_static_reader_ = reader(lingtu::message::kTfStatic, &lingtu_dds_TFMessage_desc, "tf_static");
    snapshot_reader_ = reader(lingtu::message::kNavExplorationSnapshot,
                              &lingtu_dds_ExplorationGrid_desc, "exploration_snapshot");
    goal_status_reader_ = reader(lingtu::message::kNavGoalStatus,
                                 &lingtu_dds_NavigationGoalStatus_desc, "nav_goal_status");
    command_ack_reader_ = reader(lingtu::message::kNavCommandAck,
                                 &lingtu_dds_NavigationCommandAck_desc, "nav_command_ack");
    segment_ack_reader_ = reader(lingtu::message::kNavExplorationSegmentAck,
                                 &lingtu_dds_ExplorationSegmentAck_desc, "exploration_segment_ack");
    segment_status_reader_ =
        reader(lingtu::message::kNavExplorationSegmentStatus,
               &lingtu_dds_ExplorationSegmentStatus_desc, "exploration_segment_status");
    control_reader_ = reader(lingtu::message::kNavExplorationCommand,
                             &lingtu_dds_ExplorationCommandRequest_desc, "exploration_command");
    control_ack_writer_ = writer(lingtu::message::kNavExplorationAck,
                                 &lingtu_dds_ExplorationCommandAck_desc, "exploration_ack");
    run_event_writer_ = writer(lingtu::message::kNavExplorationRunEvent,
                               &lingtu_dds_ExplorationRunEvent_desc, "exploration_run_event");
    command_request_writer_ =
        writer(lingtu::message::kNavCommandRequest, &lingtu_dds_NavigationCommandRequest_desc,
               "nav_command_request");
    segment_request_writer_ =
        writer(lingtu::message::kNavExplorationSegmentRequest,
               &lingtu_dds_ExplorationSegmentRequest_desc, "exploration_segment_request");
  }

  ~DdsRuntime() {
    if (participant_ > 0) {
      dds_delete(participant_);
    }
  }

  DdsRuntime(const DdsRuntime &) = delete;
  DdsRuntime &operator=(const DdsRuntime &) = delete;

  template <typename Handler>
  void drainOdometry(Handler &&handler) {
    drainReader<lingtu_dds_Odometry>(odometry_reader_, lingtu_dds_Odometry_desc,
                                     std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTf(Handler &&handler) {
    drainReader<lingtu_dds_TFMessage>(tf_reader_, lingtu_dds_TFMessage_desc,
                                      std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainTfStatic(Handler &&handler) {
    drainReader<lingtu_dds_TFMessage>(tf_static_reader_, lingtu_dds_TFMessage_desc,
                                      std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainSnapshot(Handler &&handler) {
    drainReader<lingtu_dds_ExplorationGrid>(snapshot_reader_, lingtu_dds_ExplorationGrid_desc,
                                            std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainGoalStatus(Handler &&handler) {
    drainReader<lingtu_dds_NavigationGoalStatus>(
        goal_status_reader_, lingtu_dds_NavigationGoalStatus_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainCommandAck(Handler &&handler) {
    drainReader<lingtu_dds_NavigationCommandAck>(
        command_ack_reader_, lingtu_dds_NavigationCommandAck_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainSegmentAck(Handler &&handler) {
    drainReader<lingtu_dds_ExplorationSegmentAck>(
        segment_ack_reader_, lingtu_dds_ExplorationSegmentAck_desc, std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainSegmentStatus(Handler &&handler) {
    drainReader<lingtu_dds_ExplorationSegmentStatus>(segment_status_reader_,
                                                     lingtu_dds_ExplorationSegmentStatus_desc,
                                                     std::forward<Handler>(handler));
  }

  template <typename Handler>
  void drainControl(Handler &&handler) {
    drainReader<lingtu_dds_ExplorationCommandRequest>(
        control_reader_, lingtu_dds_ExplorationCommandRequest_desc, std::forward<Handler>(handler));
  }

  void writeControlAck(const std::string &request_id, const std::string &exploration_run_id,
                       std::int32_t kind, bool accepted, bool duplicate, const std::string &reason,
                       const std::string &product_session_id, std::uint64_t intent_revision) {
    lingtu_dds_ExplorationCommandAck message{};
    fillHeader(message.header, nowSeconds(), "map");
    message.request_id = const_cast<char *>(request_id.c_str());
    message.exploration_run_id = const_cast<char *>(exploration_run_id.c_str());
    message.kind = kind;
    message.accepted = accepted;
    message.duplicate = duplicate;
    message.reason = const_cast<char *>(reason.c_str());
    message.product_session_id = const_cast<char *>(product_session_id.c_str());
    message.intent_revision = intent_revision;
    logDdsError(dds_write(control_ack_writer_, static_cast<const void *>(&message)),
                "dds_write(exploration_ack)");
  }

  bool
  writeExplorationRunEvent(const lingtu::nav::endpoint::ExplorationRunEventEnvelope &envelope) {
    const auto &event = envelope.event;
    lingtu_dds_ExplorationRunEvent message{};
    message.timestamp_s = event.timestamp_s;
    message.frame_id = const_cast<char *>(event.frame_id.c_str());
    message.boot_id = const_cast<char *>(envelope.boot_id.c_str());
    message.event_sequence = envelope.event_sequence;
    message.kind = static_cast<std::int32_t>(event.kind);
    message.exploration_run_id = const_cast<char *>(event.exploration_run_id.c_str());
    message.start_request_id = const_cast<char *>(event.start_request_id.c_str());
    message.command_request_id = const_cast<char *>(event.command_request_id.c_str());
    message.product_session_id = const_cast<char *>(event.product_session_id.c_str());
    message.state = static_cast<std::int32_t>(event.state);
    message.route = const_cast<char *>(event.route.c_str());
    message.map_id = const_cast<char *>(event.map_id.c_str());
    message.map_content_epoch = event.map_content_epoch;
    message.reason = const_cast<char *>(event.reason.c_str());
    message.motion_stop_confirmed = event.motion_stop_confirmed;
    message.motion_stop_reason = const_cast<char *>(event.motion_stop_reason.c_str());
    const dds_return_t result = dds_write(run_event_writer_, static_cast<const void *>(&message));
    logDdsError(result, "dds_write(exploration_run_event)");
    return result >= 0;
  }

  void writeExplorationSegmentRequest(
      const lingtu::nav::endpoint::ExplorationSegmentRequestBinding &binding, const Pose2D &target,
      lingtu::nav::endpoint::ExplorationSegmentCommandKind kind, const std::string &reason) {
    lingtu_dds_ExplorationSegmentRequest message{};
    fillHeader(message.header, nowSeconds(), "map");
    message.request_id = const_cast<char *>(binding.request_id.c_str());
    message.kind = static_cast<std::int32_t>(kind);
    message.session_id = const_cast<char *>(binding.session_id.c_str());
    message.reset_epoch = binding.reset_epoch;
    message.minimum_generation = binding.minimum_generation;
    message.target.position.x = target.x;
    message.target.position.y = target.y;
    message.target.position.z = 0.0;
    const double half_yaw = std::isfinite(target.yaw) ? target.yaw * 0.5 : 0.0;
    message.target.orientation.z = std::sin(half_yaw);
    message.target.orientation.w = std::cos(half_yaw);
    message.reason = const_cast<char *>(reason.c_str());
    logDdsError(dds_write(segment_request_writer_, static_cast<const void *>(&message)),
                "dds_write(exploration_segment_request)");
  }

  [[nodiscard]] bool commandTransportReady() const {
    const dds_return_t readers = dds_get_matched_subscriptions(command_request_writer_, nullptr, 0);
    if (readers < 0) {
      logDdsError(readers, "dds_get_matched_subscriptions(nav_command_request)");
      return false;
    }
    const dds_return_t writers = dds_get_matched_publications(command_ack_reader_, nullptr, 0);
    if (writers < 0) {
      logDdsError(writers, "dds_get_matched_publications(nav_command_ack)");
      return false;
    }
    return readers > 0 && writers > 0;
  }

  bool writeNavigationCommand(const lingtu::nav::endpoint::ExploreGoalCommandWrite &write,
                              const std::string &client_id) {
    lingtu_dds_NavigationCommandRequest message{};
    fillHeader(message.header, nowSeconds(), "map");
    message.client_id = const_cast<char *>(client_id.c_str());
    message.task_id = const_cast<char *>(write.task_id.c_str());
    message.request_id = const_cast<char *>(write.request_id.c_str());
    message.kind = static_cast<std::int32_t>(write.kind);
    message.reason = const_cast<char *>(write.reason.c_str());
    if (write.kind == lingtu::message::NavigationCommandKind::Goal) {
      message.goal.position.x = write.target.x;
      message.goal.position.y = write.target.y;
      message.goal.position.z = write.target.z;
      const double half_yaw = write.target.yaw * 0.5;
      message.goal.orientation.z = std::sin(half_yaw);
      message.goal.orientation.w = std::cos(half_yaw);
    }
    const dds_return_t result =
        dds_write(command_request_writer_, static_cast<const void *>(&message));
    logDdsError(result, "dds_write(nav_command_request)");
    return result >= 0;
  }

 private:
  dds_entity_t writer(const lingtu::message::TopicContract &contract,
                      const dds_topic_descriptor_t *descriptor, const char *label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, descriptor, contract.dds_topic.data(), nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(dds_create_writer(publisher_, topic, qos.get(), nullptr),
                   (std::string("dds_create_writer(") + label + ")").c_str());
  }

  dds_entity_t reader(const lingtu::message::TopicContract &contract,
                      const dds_topic_descriptor_t *descriptor, const char *label) {
    const dds_entity_t topic = checked(
        dds_create_topic(participant_, descriptor, contract.dds_topic.data(), nullptr, nullptr),
        (std::string("dds_create_topic(") + label + ")").c_str());
    auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
    return checked(dds_create_reader(subscriber_, topic, qos.get(), nullptr),
                   (std::string("dds_create_reader(") + label + ")").c_str());
  }

  dds_entity_t participant_{0};
  dds_entity_t subscriber_{0};
  dds_entity_t publisher_{0};
  dds_entity_t odometry_reader_{0};
  dds_entity_t tf_reader_{0};
  dds_entity_t tf_static_reader_{0};
  dds_entity_t snapshot_reader_{0};
  dds_entity_t goal_status_reader_{0};
  dds_entity_t command_ack_reader_{0};
  dds_entity_t segment_ack_reader_{0};
  dds_entity_t segment_status_reader_{0};
  dds_entity_t control_reader_{0};
  dds_entity_t control_ack_writer_{0};
  dds_entity_t run_event_writer_{0};
  dds_entity_t command_request_writer_{0};
  dds_entity_t segment_request_writer_{0};
};

struct PendingGoal {
  Pose2D target;
  ExploreMapIdentity map;
  std::string task_id;
  std::string request_id;
  std::string cancel_request_id;
  double accepted_s{0.0};
  bool start_acknowledged{false};
  bool cancellation_acknowledged{false};
};

struct PendingMotionStopEvidence {
  double timestamp_s{0.0};
  std::string reason;
};

struct QueuedGoal {
  Pose2D target;
  ExploreMapIdentity map;
};

enum class PendingSegmentPhase {
  AwaitingExecuteAck,
  Executing,
  CancelRequested,
};

struct PendingSegment {
  Pose2D target;
  ExploreMapIdentity map;
  lingtu::nav::endpoint::ExplorationSegmentRequestBinding binding;
  PendingSegmentPhase phase{PendingSegmentPhase::AwaitingExecuteAck};
  std::uint64_t execution_generation{0U};
  double phase_started_s{0.0};
  bool deadline_reported{false};
  bool cancel_acknowledged{false};
};

struct SegmentReplanBarrier {
  ExploreMapIdentity map;
  lingtu::nav::endpoint::ExplorationSegmentReplanBarrier barrier;
};

struct Counters {
  std::uint64_t tf_messages{0U};
  std::uint64_t odometry_messages{0U};
  std::uint64_t odometry_rejected{0U};
  std::uint64_t snapshot_messages{0U};
  std::uint64_t snapshot_rejected{0U};
  std::uint64_t stale_generations{0U};
  std::uint64_t plans{0U};
  std::uint64_t plan_rejected{0U};
  std::uint64_t goals_accepted{0U};
  std::uint64_t goals_rejected{0U};
  std::uint64_t goals_reached{0U};
  std::uint64_t goals_timed_out{0U};
  std::uint64_t goal_status_messages{0U};
  std::uint64_t goal_failures{0U};
  std::uint64_t cancels_accepted{0U};
  std::uint64_t cancels_rejected{0U};
  std::uint64_t map_resets{0U};
  std::uint64_t directed_targets_set{0U};
  std::uint64_t directed_targets_cleared{0U};
  std::uint64_t directed_targets_expired{0U};
  std::uint64_t control_messages{0U};
  std::uint64_t control_accepted{0U};
  std::uint64_t control_rejected{0U};
  std::uint64_t control_duplicates{0U};
  std::uint64_t segment_requests{0U};
  std::uint64_t segment_ack_messages{0U};
  std::uint64_t segment_status_messages{0U};
  std::uint64_t segment_terminals{0U};
};

const char *segmentPhaseName(PendingSegmentPhase phase) noexcept {
  switch (phase) {
    case PendingSegmentPhase::AwaitingExecuteAck:
      return "awaiting_ack";
    case PendingSegmentPhase::Executing:
      return "executing";
    case PendingSegmentPhase::CancelRequested:
      return "cancelling";
  }
  return "unknown";
}

bool sameMapEpoch(const ExploreMapIdentity &left, const ExploreMapIdentity &right) {
  return left.sameSource(right) && left.reset_epoch == right.reset_epoch;
}

lingtu::nav::endpoint::ExplorationSegmentIdentity
segmentIdentityFromMap(const ExploreMapIdentity &identity) {
  return {
      identity.frame_id,   identity.session_id, identity.reset_epoch,
      identity.generation, identity.live,
  };
}

lingtu::nav::endpoint::ExplorationSegmentIdentity
segmentIdentityFromWire(const lingtu_dds_Header &header, const char *session_id,
                        std::uint64_t reset_epoch, std::uint64_t generation, bool live) {
  return {
      stringValue(header.frame_id), stringValue(session_id), reset_epoch, generation, live,
  };
}

std::string jsonEscape(const std::string &value) {
  std::string result;
  result.reserve(value.size());
  for (const char ch : value) {
    switch (ch) {
      case '\\':
        result += "\\\\";
        break;
      case '"':
        result += "\\\"";
        break;
      case '\n':
        result += "\\n";
        break;
      case '\r':
        result += "\\r";
        break;
      case '\t':
        result += "\\t";
        break;
      default:
        result += ch;
        break;
    }
  }
  return result;
}

std::string statusSnapshot(
    const Config &config, const std::string &endpoint_boot_id, const std::string &state,
    const std::string &reason, const lingtu::nav::endpoint::ExploreControl &control,
    const TimedMapPose *robot, const ExplorationSnapshot *snapshot,
    const std::optional<PendingGoal> &pending, const std::optional<PendingSegment> &pending_segment,
    const std::optional<QueuedGoal> &queued, const std::optional<std::string> &cancel_reason,
    const std::optional<DirectedExplorationIntent> &directed_target,
    const ExploreDecision &decision,
    const lingtu::nav::endpoint::ExplorationRunEventOutboxDiagnostics &run_events,
    const Counters &counters, double now_s) {
  const double odometry_age_s = robot != nullptr ? now_s - robot->stamp_s : -1.0;
  const double snapshot_age_s = snapshot != nullptr ? now_s - snapshot->stamp_s : -1.0;
  std::ostringstream output;
  output << std::fixed << std::setprecision(6);
  output << "{\n";
  lingtu::nav::endpoint::writeExploreStatusIdentity(output, endpoint_boot_id);
  output << "  \"product\": \"" << jsonEscape(config.product) << "\",\n"
         << "  \"product_session_id\": \"" << jsonEscape(config.product_session_id) << "\",\n"
         << "  \"stamp_s\": " << now_s << ",\n"
         << "  \"domain_id\": " << config.domain_id << ",\n"
         << "  \"route\": \"" << lingtu::nav::endpoint::routeName(*config.route) << "\",\n"
         << "  \"state\": \"" << jsonEscape(state) << "\",\n"
         << "  \"reason\": \"" << jsonEscape(reason) << "\",\n"
         << "  \"active\": " << (control.active() ? "true" : "false") << ",\n"
         << "  \"paused\": " << (control.paused() ? "true" : "false") << ",\n"
         << "  \"exploration_run_id\": \"" << jsonEscape(control.exploration_run_id()) << "\",\n"
         << "  \"ready\": " << (robot != nullptr && snapshot != nullptr ? "true" : "false") << ",\n"
         << "  \"input\": {\"odometry_age_s\": " << odometry_age_s
         << ", \"snapshot_age_s\": " << snapshot_age_s << "},\n";
  if (snapshot != nullptr) {
    const auto &map = snapshot->identity;
    output << "  \"map\": {\"frame_id\": \"" << jsonEscape(map.frame_id) << "\", \"map_id\": \""
           << jsonEscape(map.map_id) << "\", \"map_content_epoch\": " << map.map_content_epoch
           << ", \"reset_epoch\": " << map.reset_epoch << ", \"generation\": " << map.generation
           << ", \"live\": " << (map.live ? "true" : "false") << "},\n";
  } else {
    output << "  \"map\": null,\n";
  }
  if (pending.has_value()) {
    output << "  \"pending_goal\": {\"x\": " << pending->target.x
           << ", \"y\": " << pending->target.y << ", \"task_id\": \""
           << jsonEscape(pending->task_id) << "\", \"request_id\": \""
           << jsonEscape(pending->request_id) << "\", \"cancel_request_id\": \""
           << jsonEscape(pending->cancel_request_id) << "\""
           << ", \"cancellation_acknowledged\": "
           << (pending->cancellation_acknowledged ? "true" : "false")
           << ", \"age_s\": " << now_s - pending->accepted_s << "},\n";
  } else if (queued.has_value()) {
    output << "  \"pending_goal\": {\"x\": " << queued->target.x << ", \"y\": " << queued->target.y
           << ", \"age_s\": -1.0},\n";
  } else {
    output << "  \"pending_goal\": null,\n";
  }
  if (pending_segment.has_value()) {
    output << "  \"pending_segment\": {\"x\": " << pending_segment->target.x
           << ", \"y\": " << pending_segment->target.y << ", \"request_id\": \""
           << jsonEscape(pending_segment->binding.request_id)
           << "\", \"reset_epoch\": " << pending_segment->binding.reset_epoch
           << ", \"minimum_generation\": " << pending_segment->binding.minimum_generation
           << ", \"execution_generation\": " << pending_segment->execution_generation
           << ", \"phase\": \"" << segmentPhaseName(pending_segment->phase) << "\"},\n";
  } else {
    output << "  \"pending_segment\": null,\n";
  }
  if (directed_target.has_value()) {
    output << "  \"directed_target\": {\"x\": " << directed_target->target.x
           << ", \"y\": " << directed_target->target.y
           << ", \"revision\": " << directed_target->revision
           << ", \"ttl_remaining_s\": " << std::max(0.0, directed_target->expires_at_s - now_s)
           << "},\n";
  } else {
    output << "  \"directed_target\": null,\n";
  }
  output << "  \"cancel_reason\": ";
  if (cancel_reason.has_value()) {
    output << "\"" << jsonEscape(*cancel_reason) << "\"";
  } else {
    output << "null";
  }
  const ExploreDiagnostics &diagnostics = decision.diagnostics;
  output << ",\n"
         << "  \"planner\": {\"phase\": \"" << jsonEscape(diagnostics.phase)
         << "\", \"accepted_generation\": " << diagnostics.accepted_generation
         << ", \"accepted_intent_revision\": " << diagnostics.accepted_intent_revision
         << ", \"reachable_free_cells\": " << diagnostics.reachable_free_cells
         << ", \"frontier_cells\": " << diagnostics.frontier_cells
         << ", \"frontier_clusters\": " << diagnostics.frontier_clusters
         << ", \"keypose_nodes\": " << diagnostics.keypose_nodes
         << ", \"keypose_edges\": " << diagnostics.keypose_edges
         << ", \"covered_cells\": " << diagnostics.covered_cells
         << ", \"route_targets\": " << diagnostics.route_targets
         << ", \"route_length_m\": " << diagnostics.route_length_m
         << ", \"planning_time_ms\": " << diagnostics.planning_time_ms
         << ", \"state_committed\": " << (diagnostics.state_committed ? "true" : "false") << "},\n"
         << "  \"run_event_outbox\": {\"accepted\": " << run_events.accepted
         << ", \"rejected_invalid\": " << run_events.rejected_invalid
         << ", \"rejected_backpressure\": " << run_events.rejected_backpressure
         << ", \"delivered\": " << run_events.delivered
         << ", \"delivery_failures\": " << run_events.delivery_failures
         << ", \"pending\": " << run_events.pending << "},\n"
         << "  \"counters\": {"
         << "\"tf_messages\": " << counters.tf_messages
         << ", \"odometry_messages\": " << counters.odometry_messages
         << ", \"odometry_rejected\": " << counters.odometry_rejected
         << ", \"snapshot_messages\": " << counters.snapshot_messages
         << ", \"snapshot_rejected\": " << counters.snapshot_rejected
         << ", \"stale_generations\": " << counters.stale_generations
         << ", \"plans\": " << counters.plans << ", \"plan_rejected\": " << counters.plan_rejected
         << ", \"goals_accepted\": " << counters.goals_accepted
         << ", \"goals_rejected\": " << counters.goals_rejected
         << ", \"goals_reached\": " << counters.goals_reached
         << ", \"goals_timed_out\": " << counters.goals_timed_out
         << ", \"goal_status_messages\": " << counters.goal_status_messages
         << ", \"goal_failures\": " << counters.goal_failures
         << ", \"cancels_accepted\": " << counters.cancels_accepted
         << ", \"cancels_rejected\": " << counters.cancels_rejected
         << ", \"map_resets\": " << counters.map_resets
         << ", \"directed_targets_set\": " << counters.directed_targets_set
         << ", \"directed_targets_cleared\": " << counters.directed_targets_cleared
         << ", \"directed_targets_expired\": " << counters.directed_targets_expired
         << ", \"control_messages\": " << counters.control_messages
         << ", \"control_accepted\": " << counters.control_accepted
         << ", \"control_rejected\": " << counters.control_rejected
         << ", \"control_duplicates\": " << counters.control_duplicates
         << ", \"segment_requests\": " << counters.segment_requests
         << ", \"segment_ack_messages\": " << counters.segment_ack_messages
         << ", \"segment_status_messages\": " << counters.segment_status_messages
         << ", \"segment_terminals\": " << counters.segment_terminals << "}\n"
         << "}\n";
  return output.str();
}

}  // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, stopSignal);
  std::signal(SIGTERM, stopSignal);

  try {
    const Config config = parseArgs(argc, argv);
    std::optional<lingtu::explore::Grid2D> saved_coverage_grid;
    if (*config.route == Route::Map) {
      lingtu::nav::plan::MapIdentity map_identity;
      map_identity.map_id = config.expected_map_id;
      map_identity.content_epoch = config.expected_map_content_epoch;
      map_identity.frame_id = config.expected_map_frame;
      const lingtu::nav::endpoint::ActiveOccupancyGate saved_coverage_gate(map_identity);
      const auto prepared = saved_coverage_gate.prepare(config.occupancy_path);
      if (!prepared.ok()) {
        throw std::runtime_error("saved_map_occupancy_not_ready: " + prepared.reason);
      }
      if (prepared.artifact->map().CellCount() > config.policy.max_grid_cells) {
        throw std::runtime_error("saved_map_occupancy_exceeds_max_grid_cells");
      }
      ExploreMapIdentity expected_identity;
      expected_identity.session_id = config.product_session_id;
      expected_identity.map_id = config.expected_map_id;
      expected_identity.map_content_epoch = config.expected_map_content_epoch;
      expected_identity.reset_epoch = 1U;
      expected_identity.generation = 1U;
      expected_identity.live = false;
      auto coverage = lingtu::nav::endpoint::buildSavedCoverageGrid(prepared.artifact->map(),
                                                                   expected_identity);
      if (!coverage.ok()) {
        throw std::runtime_error(coverage.reason);
      }
      saved_coverage_grid = std::move(*coverage.grid);
    }
    DdsRuntime dds(config.domain_id);
    const std::string endpoint_boot_id = makeExploreBootId();
    lingtu::nav::endpoint::ExplorationRunEventOutbox run_event_outbox(
        endpoint_boot_id, [&dds](const lingtu::nav::endpoint::ExplorationRunEventEnvelope &event) {
          return dds.writeExplorationRunEvent(event);
        });
    lingtu::nav::endpoint::ExplorationRunLifecycle run_lifecycle(run_event_outbox);
    const double command_timeout_s = static_cast<double>(config.command_timeout_ms) / 1000.0;
    lingtu::nav::endpoint::ExploreGoalCommandLane goal_command_lane(config.command_retry_s,
                                                                    command_timeout_s);
    const std::string navigation_client_id = "explore:" + endpoint_boot_id;
    lingtu::explore::TarePolicy policy(config.policy);
    lingtu::nav::endpoint::StatusSnapshotFileWriter status_writer(config.status_file);

    std::optional<RigidTransform> map_odom;
    double map_odom_receive_s{-1.0};
    std::optional<TimedMapPose> robot;
    std::optional<ExplorationSnapshot> snapshot;
    std::optional<PendingGoal> pending;
    std::optional<PendingSegment> pending_segment;
    std::optional<SegmentReplanBarrier> segment_replan_barrier;
    std::optional<QueuedGoal> queued;
    std::optional<std::string> cancel_reason;
    std::optional<PendingMotionStopEvidence> pending_motion_stop_evidence;
    std::vector<lingtu::nav::endpoint::ExploreGoalCommandAck> command_acks;
    std::vector<Pose2D> visited;
    ExploreDecision last_decision;
    Counters counters;
    lingtu::nav::endpoint::ExploreControl exploration_control;
    lingtu::nav::endpoint::DirectedExplorationIntentStore directed_intent;
    std::string last_reason{"waiting_for_start"};
    std::string runtime_state{"idle"};
    double last_segment_command_attempt_s{-1.0};
    double next_goal_dispatch_s{0.0};
    double next_status_s{0.0};
    const std::string goal_request_prefix =
        "explore-" + std::to_string(static_cast<std::uint64_t>(nowSeconds() * 1000000.0));
    std::uint64_t goal_request_sequence{0U};
    const std::string segment_request_prefix = goal_request_prefix + "-segment";
    std::uint64_t segment_request_sequence{0U};
    auto markVisited = [&](const Pose2D &target) {
      visited.push_back(target);
      if (visited.size() > 4096U) {
        visited.erase(visited.begin(), visited.begin() + 2048);
      }
    };
    auto writeSegmentRequest = [&](const PendingSegment &segment,
                                   lingtu::nav::endpoint::ExplorationSegmentCommandKind kind,
                                   const std::string &reason) {
      dds.writeExplorationSegmentRequest(segment.binding, segment.target, kind, reason);
      ++counters.segment_requests;
    };
    auto beginSegment = [&](const Pose2D &target, const ExploreMapIdentity &map, double attempt_s,
                            const std::string &reason) {
      if (!map.valid() || !map.live) {
        throw std::runtime_error("live_route_requires_live_map");
      }
      lingtu::nav::endpoint::ExplorationSegmentRequestBinding binding{
          segment_request_prefix + "-" + std::to_string(++segment_request_sequence),
          map.session_id,
          map.reset_epoch,
          map.generation,
      };
      pending_segment = PendingSegment{
          target,    map,  std::move(binding), PendingSegmentPhase::AwaitingExecuteAck, 0U,
          attempt_s, false, false};
      last_segment_command_attempt_s = attempt_s;
      writeSegmentRequest(*pending_segment,
                          lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute, reason);
    };
    auto requestSegmentCancel = [&](const std::string &reason, double command_now_s) {
      if (!pending_segment.has_value() ||
          pending_segment->phase == PendingSegmentPhase::CancelRequested) {
        return;
      }
      pending_segment->phase = PendingSegmentPhase::CancelRequested;
      pending_segment->phase_started_s = command_now_s;
      pending_segment->deadline_reported = false;
      pending_segment->cancel_acknowledged = false;
      last_segment_command_attempt_s = command_now_s;
      writeSegmentRequest(*pending_segment,
                          lingtu::nav::endpoint::ExplorationSegmentCommandKind::kCancel, reason);
    };
    auto finishSegmentTerminal = [&](const PendingSegment &segment,
                                     std::uint64_t terminal_generation, const std::string &reason) {
      std::uint64_t observed_snapshot_generation{0U};
      const bool map_is_current =
          snapshot.has_value() && sameMapEpoch(segment.map, snapshot->identity);
      if (map_is_current) {
        observed_snapshot_generation = snapshot->identity.generation;
        segment_replan_barrier = SegmentReplanBarrier{
            segment.map,
            lingtu::nav::endpoint::makeExplorationSegmentReplanBarrier(
                segment.binding, terminal_generation, observed_snapshot_generation),
        };
      } else {
        segment_replan_barrier.reset();
      }
      ++counters.segment_terminals;
      pending_segment.reset();
      queued.reset();
      last_segment_command_attempt_s = -1.0;
      last_reason =
          "exploration_segment_terminal:" + (reason.empty() ? std::string{"unspecified"} : reason);
    };
    auto releaseSegmentReplanBarrier = [&]() {
      if (!segment_replan_barrier.has_value() || !snapshot.has_value() ||
          !sameMapEpoch(segment_replan_barrier->map, snapshot->identity)) {
        return;
      }
      if (lingtu::nav::endpoint::hasNewerExplorationSegmentSnapshot(
              segment_replan_barrier->barrier, segmentIdentityFromMap(snapshot->identity))) {
        segment_replan_barrier.reset();
        last_decision = ExploreDecision{};
        last_reason = "exploration_segment_replan_ready";
      }
    };
    auto observeMotionStopped = [&](double timestamp_s, const std::string &reason) {
      if (!run_lifecycle.stopConfirmationPending()) {
        return;
      }
      const std::string evidence_reason =
          reason.empty() ? std::string{"post_stop_terminal"} : reason;
      if (run_lifecycle.confirmMotionStop(timestamp_s, evidence_reason)) {
        pending_motion_stop_evidence.reset();
      } else {
        pending_motion_stop_evidence = PendingMotionStopEvidence{timestamp_s, evidence_reason};
      }
    };
    auto handleCommandAcks = [&](double ack_s, bool allow_requeue) {
      for (const auto &ack : command_acks) {
        if (!pending.has_value() || ack.task_id != pending->task_id) {
          continue;
        }
        if (ack.kind == lingtu::message::NavigationCommandKind::Goal &&
            ack.request_id == pending->request_id) {
          if (ack.accepted) {
            if (!pending->start_acknowledged) {
              pending->start_acknowledged = true;
              pending->accepted_s = ack_s;
              ++counters.goals_accepted;
            }
            if (!cancel_reason.has_value()) {
              last_reason = "exploration_goal_accepted";
            }
          } else {
            ++counters.goals_rejected;
            last_reason = "exploration_goal_rejected:" + ack.reason;
            const QueuedGoal rejected_goal{pending->target, pending->map};
            const auto rejection = lingtu::nav::endpoint::reactToRejectedExploreStart(
                cancel_reason.has_value() || !allow_requeue,
                allow_requeue && exploration_control.running());
            goal_command_lane.finishGoal();
            pending.reset();
            if (rejection.confirm_no_motion) {
              cancel_reason.reset();
              observeMotionStopped(ack_s, "navigation_goal_rejected_before_motion");
            } else if (rejection.requeue_goal) {
              queued = rejected_goal;
              next_goal_dispatch_s = ack_s + config.command_retry_s;
            }
          }
        } else if (ack.kind == lingtu::message::NavigationCommandKind::TaskCancel &&
                   !pending->cancel_request_id.empty() &&
                   ack.request_id == pending->cancel_request_id) {
          if (ack.accepted) {
            if (!pending->cancellation_acknowledged) {
              ++counters.cancels_accepted;
            }
            pending->cancellation_acknowledged = true;
            last_reason = "exploration_cancel_acknowledged";
          } else {
            ++counters.cancels_rejected;
            last_reason = "exploration_cancel_rejected:" + ack.reason;
            (void)run_lifecycle.recordStopConfirmationFailure(ack_s, last_reason);
          }
        }
      }
    };

    std::fprintf(stderr,
                 "lingtu_explore_dds: domain=%d route=%s tick_hz=%.2f input=%s command=%s\n",
                 config.domain_id, lingtu::nav::endpoint::routeName(*config.route).data(),
                 config.tick_hz, lingtu::message::kNavExplorationSnapshot.topic.data(),
                 lingtu::message::kNavCommandRequest.topic.data());

    const auto period = std::chrono::duration<double>(1.0 / config.tick_hz);
    std::exception_ptr loop_error;
    try {
      while (g_running.load(std::memory_order_relaxed)) {
        const auto loop_started = std::chrono::steady_clock::now();
        const double loop_now_s = nowSeconds();
        const double loop_command_s = monotonicSeconds();

        (void)run_event_outbox.flush();
        if (pending_motion_stop_evidence.has_value()) {
          if (!run_lifecycle.stopConfirmationPending() ||
              run_lifecycle.confirmMotionStop(pending_motion_stop_evidence->timestamp_s,
                                              pending_motion_stop_evidence->reason)) {
            pending_motion_stop_evidence.reset();
          }
        }

        command_acks.clear();
        dds.drainCommandAck([&](const lingtu_dds_NavigationCommandAck &message) {
          if (!lingtu::message::isKnownNavigationCommandKind(message.kind)) {
            return;
          }
          command_acks.push_back({
              stringValue(message.task_id),
              stringValue(message.request_id),
              static_cast<lingtu::message::NavigationCommandKind>(message.kind),
              message.accepted,
              stringValue(message.reason),
          });
        });

        auto acceptTf = [&](const lingtu_dds_TFMessage &message) {
          ++counters.tf_messages;
          const auto transform = lingtu::nav::endpoint::mapOdomTransformFromTf(message);
          if (transform.has_value()) {
            map_odom = transform;
            map_odom_receive_s = loop_now_s;
          }
        };
        dds.drainTf(acceptTf);
        dds.drainTfStatic(acceptTf);

        dds.drainOdometry([&](const lingtu_dds_Odometry &message) {
          ++counters.odometry_messages;
          std::string reason;
          auto parsed = lingtu::nav::endpoint::mapPoseFromOdometry(
              message, map_odom, map_odom_receive_s, loop_now_s, config.input_gate, &reason);
          if (parsed.has_value()) {
            robot = std::move(parsed);
          } else {
            ++counters.odometry_rejected;
            last_reason = std::move(reason);
          }
        });

        dds.drainSnapshot([&](const lingtu_dds_ExplorationGrid &message) {
          ++counters.snapshot_messages;
          std::string reason;
          auto parsed =
              lingtu::nav::endpoint::parseExplorationSnapshot(message, config.input_gate, &reason);
          if (!parsed.has_value() || !lingtu::nav::endpoint::snapshotFresh(
                                         *parsed, loop_now_s, config.input_gate, &reason)) {
            ++counters.snapshot_rejected;
            last_reason = std::move(reason);
            return;
          }

          if (!lingtu::nav::endpoint::isCanonicalExploreSnapshotBinding(
                  *config.route, config.product_session_id, config.expected_map_id,
                  config.expected_map_content_epoch, parsed->identity.session_id,
                  parsed->identity.live, parsed->identity.map_id,
                  parsed->identity.map_content_epoch)) {
            ++counters.snapshot_rejected;
            last_reason = parsed->identity.session_id == config.product_session_id
                              ? "exploration_snapshot_route_mismatch"
                              : "exploration_snapshot_product_session_mismatch";
            return;
          }

          if (snapshot.has_value() && snapshot->identity.sameSource(parsed->identity)) {
            if (parsed->identity.reset_epoch < snapshot->identity.reset_epoch ||
                (parsed->identity.reset_epoch == snapshot->identity.reset_epoch &&
                 parsed->identity.generation <= snapshot->identity.generation)) {
              ++counters.stale_generations;
              return;
            }
          }

          const bool map_epoch_changed =
              snapshot.has_value() && !sameMapEpoch(snapshot->identity, parsed->identity);
          if (map_epoch_changed) {
            ++counters.map_resets;
            const bool motion_pending = pending.has_value() || pending_segment.has_value();
            if (run_lifecycle.active() && !run_lifecycle.stopConfirmationPending()) {
              if (!run_lifecycle.fail(loop_now_s, "exploration_map_identity_changed",
                                      motion_pending)) {
                last_reason = "exploration_map_identity_change_event_backpressure";
              }
            }
            if (exploration_control.active()) {
              static_cast<void>(exploration_control.Complete());
            }
            queued.reset();
            visited.clear();
            policy.reset();
            directed_intent.Reset();
            last_decision = ExploreDecision{};
            if (pending_segment.has_value()) {
              requestSegmentCancel("exploration_map_identity_changed", loop_command_s);
            } else {
              segment_replan_barrier.reset();
              last_segment_command_attempt_s = -1.0;
            }
            if (pending.has_value()) {
              cancel_reason = "exploration_map_identity_changed";
            }
          }
          snapshot = std::move(parsed);
        });

        const bool robot_fresh = robot.has_value() && lingtu::nav::endpoint::sourceStampFresh(
                                                          robot->stamp_s, loop_now_s,
                                                          config.input_gate.odometry_max_age_s,
                                                          config.input_gate.future_tolerance_s);
        std::string freshness_reason;
        const bool snapshot_fresh =
            snapshot.has_value() &&
            lingtu::nav::endpoint::snapshotFresh(*snapshot, loop_now_s, config.input_gate,
                                                 &freshness_reason);
        const bool snapshot_route_bound =
            snapshot_fresh && lingtu::nav::endpoint::isCanonicalExploreSnapshotBinding(
                                  *config.route, config.product_session_id, config.expected_map_id,
                                  config.expected_map_content_epoch, snapshot->identity.session_id,
                                  snapshot->identity.live, snapshot->identity.map_id,
                                  snapshot->identity.map_content_epoch);
        if (snapshot_fresh && !snapshot_route_bound) {
          freshness_reason = snapshot->identity.session_id == config.product_session_id
                                 ? "exploration_snapshot_route_mismatch"
                                 : "exploration_snapshot_product_session_mismatch";
        }

        if (directed_intent.Expire(loop_now_s)) {
          ++counters.directed_targets_expired;
          queued.reset();
          last_decision = ExploreDecision{};
          if (pending_segment.has_value()) {
            requestSegmentCancel("directed_exploration_target_expired", loop_command_s);
          } else if (pending.has_value()) {
            cancel_reason = "directed_exploration_target_expired";
          }
          last_reason = "directed_exploration_target_expired";
        }

        dds.drainControl([&](const lingtu_dds_ExplorationCommandRequest &message) {
          ++counters.control_messages;
          lingtu::nav::endpoint::ExplorationControlRequest request;
          request.request_id = stringValue(message.request_id);
          request.exploration_run_id = stringValue(message.exploration_run_id);
          request.kind = message.kind;
          request.product_session_id = stringValue(message.product_session_id);
          request.expected_product_session_id = config.product_session_id;
          request.reason = stringValue(message.reason);
          request.frame_id = stringValue(message.header.frame_id);
          request.stamp_s = stampSeconds(message.header.stamp);
          request.now_s = loop_now_s;
          request.max_age_s = config.control_max_age_s;
          request.future_tolerance_s = config.input_gate.future_tolerance_s;
          request.inputs_ready = robot_fresh && snapshot_route_bound;
          request.snapshot_ready = snapshot_route_bound;
          request.goal_pending = pending.has_value() || pending_segment.has_value();
          request.cancellation_pending =
              cancel_reason.has_value() ||
              (pending_segment.has_value() &&
               pending_segment->phase == PendingSegmentPhase::CancelRequested);
          const bool lifecycle_command =
              request.kind ==
                  static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kStart) ||
              request.kind ==
                  static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kPause) ||
              request.kind ==
                  static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kResume) ||
              request.kind ==
                  static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kStop);
          const bool motion_stop_command =
              request.kind ==
                  static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kPause) ||
              request.kind ==
                  static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kStop);
          const bool motion_stop_transition_pending = run_lifecycle.stopConfirmationPending();
          std::size_t required_event_slots = 1U;
          if (request.kind ==
              static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kStart)) {
            required_event_slots = 2U;
          } else if (motion_stop_command && motion_stop_transition_pending) {
            required_event_slots = 0U;
          } else if (motion_stop_command && request.goal_pending) {
            required_event_slots =
                lingtu::nav::endpoint::ExplorationRunEventOutbox::kMotionStopReserve;
          }
          request.event_capacity_ready =
              !lifecycle_command ||
              (motion_stop_command ? run_event_outbox.canRecordMotionStop(required_event_slots)
                                   : run_event_outbox.canRecord(required_event_slots));
          request.has_directed_target = message.has_directed_target;
          request.directed_target_x = message.directed_target_x;
          request.directed_target_y = message.directed_target_y;
          request.directed_target_ttl_s = message.directed_target_ttl_s;
          request.max_directed_target_ttl_s = config.directed_target_max_ttl_s;

          const bool motion_pending_before = pending.has_value() || pending_segment.has_value();
          auto result = exploration_control.Apply(request);
          if (!result.duplicate && result.accepted && result.set_directed_target) {
            if (!snapshot.has_value()) {
              result.accepted = false;
              result.reason = "exploration_snapshot_not_ready";
              result.set_directed_target = false;
              result.clear_queue = false;
              result.request_cancel = false;
              exploration_control.RecordIntentOutcome(request.request_id, false, result.reason,
                                                      directed_intent.revision());
            } else {
              const auto intent_result = directed_intent.Set(
                  result.product_session_id, snapshot->identity, request.directed_target_x,
                  request.directed_target_y, request.directed_target_ttl_s, loop_now_s);
              if (!intent_result.accepted) {
                result.accepted = false;
                result.reason = intent_result.reason;
                result.set_directed_target = false;
                result.clear_queue = false;
                result.request_cancel = false;
                exploration_control.RecordIntentOutcome(request.request_id, false, result.reason,
                                                        directed_intent.revision());
              } else {
                result.reason = intent_result.reason;
                result.intent_revision = intent_result.revision;
                exploration_control.RecordIntentOutcome(request.request_id, true, result.reason,
                                                        result.intent_revision);
                last_decision = ExploreDecision{};
                ++counters.directed_targets_set;
              }
            }
          } else if (!result.duplicate && result.accepted && result.clear_directed_target) {
            if (request.kind ==
                static_cast<std::int32_t>(
                    lingtu::message::ExplorationCommandKind::kClearDirectedTarget)) {
              const auto intent_result = directed_intent.Clear();
              result.reason = intent_result.reason;
              result.intent_revision = intent_result.revision;
              exploration_control.RecordIntentOutcome(request.request_id, true, result.reason,
                                                      result.intent_revision);
              if (intent_result.changed) {
                last_decision = ExploreDecision{};
                ++counters.directed_targets_cleared;
              } else {
                result.clear_queue = false;
                result.request_cancel = false;
              }
            } else {
              directed_intent.Reset();
              result.intent_revision = 0U;
              exploration_control.RecordIntentRevision(request.request_id, result.intent_revision);
            }
          }
          if (!result.duplicate && result.accepted) {
            const auto kind = static_cast<lingtu::message::ExplorationCommandKind>(request.kind);
            const std::string command_reason =
                request.reason.empty() ? result.reason : request.reason;
            bool lifecycle_recorded = true;
            if (kind == lingtu::message::ExplorationCommandKind::kStart) {
              if (!snapshot.has_value()) {
                throw std::runtime_error("accepted exploration START has no map binding");
              }
              lifecycle_recorded = run_lifecycle.start(
                  lingtu::nav::endpoint::ExplorationRunBinding{
                      request.exploration_run_id,
                      request.request_id,
                      config.product_session_id,
                      std::string(lingtu::nav::endpoint::routeName(*config.route)),
                      snapshot->identity.map_id,
                      snapshot->identity.map_content_epoch,
                  },
                  loop_now_s, result.reason);
            } else if (kind == lingtu::message::ExplorationCommandKind::kPause &&
                       result.reason == "exploration_pause_admitted") {
              lifecycle_recorded = run_lifecycle.pause(request.request_id, loop_now_s,
                                                       command_reason, motion_pending_before);
            } else if (kind == lingtu::message::ExplorationCommandKind::kResume &&
                       result.reason == "exploration_resume_admitted") {
              lifecycle_recorded =
                  run_lifecycle.resume(request.request_id, loop_now_s, command_reason);
            } else if (kind == lingtu::message::ExplorationCommandKind::kStop &&
                       result.reason == "exploration_stop_admitted") {
              lifecycle_recorded = run_lifecycle.cancel(request.request_id, loop_now_s,
                                                        command_reason, motion_pending_before);
            }
            if (!lifecycle_recorded) {
              throw std::runtime_error("exploration lifecycle fact could not be recorded");
            }
          }
          if (result.clear_queue) {
            queued.reset();
          }
          if (result.clear_history) {
            visited.clear();
          }
          if (result.reset_planner) {
            policy.reset();
            last_decision = ExploreDecision{};
          }
          if (result.request_cancel) {
            const std::string requested_cancel_reason =
                result.cancel_reason.empty() ? "exploration_control_cancel" : result.cancel_reason;
            if (pending_segment.has_value()) {
              requestSegmentCancel(requested_cancel_reason, loop_command_s);
            } else if (pending.has_value()) {
              cancel_reason = requested_cancel_reason;
            }
          }
          dds.writeControlAck(request.request_id, result.exploration_run_id, request.kind,
                              result.accepted, result.duplicate, result.reason,
                              result.product_session_id, result.intent_revision);
          if (result.duplicate) {
            ++counters.control_duplicates;
          } else if (result.accepted) {
            ++counters.control_accepted;
          } else {
            ++counters.control_rejected;
          }
          last_reason = result.reason;
        });
        dds.drainGoalStatus([&](const lingtu_dds_NavigationGoalStatus &message) {
          ++counters.goal_status_messages;
          if (!lingtu::message::isKnownNavigationGoalState(message.state) || !pending.has_value()) {
            return;
          }
          if (stringValue(message.task_id) != pending->task_id) {
            return;
          }
          const auto state = static_cast<lingtu::message::NavigationGoalState>(message.state);
          const std::string reason = stringValue(message.reason);
          lingtu::nav::endpoint::PendingExploreGoalLifecycle lifecycle{
              pending->request_id,
              pending->cancel_request_id,
          };
          const auto reaction = lingtu::nav::endpoint::reactToNavigationGoalLifecycle(
              &lifecycle, {
                              stringValue(message.request_id),
                              state,
                              reason,
                          });
          if (!reaction.matched) {
            return;
          }

          const bool pending_map_is_current =
              snapshot.has_value() && sameMapEpoch(pending->map, snapshot->identity);
          const std::string status_reason = reaction.reason.empty()
                                                ? lingtu::message::navigationGoalStateName(state)
                                                : reaction.reason;
          if (reaction.clear_pending) {
            goal_command_lane.finishGoal();
            observeMotionStopped(loop_now_s,
                                 "navigation_goal_terminal_after_stop:" + status_reason);
          }
          if (lingtu::nav::endpoint::shouldMarkExploreGoalVisited(reaction,
                                                                  pending_map_is_current)) {
            markVisited(pending->target);
          }
          if (reaction.clear_pending) {
            pending.reset();
            cancel_reason.reset();
          }
          if (reaction.request_replan) {
            queued.reset();
            last_decision = ExploreDecision{};
            ++counters.goal_failures;
            last_reason = "exploration_goal_failed:" + status_reason;
          } else if (state == lingtu::message::NavigationGoalState::Reached) {
            ++counters.goals_reached;
            last_reason = "exploration_goal_reached";
          } else if (state == lingtu::message::NavigationGoalState::Cancelled) {
            last_reason = "exploration_goal_cancelled:" + status_reason;
          } else {
            last_reason = "exploration_goal_" + status_reason;
          }
        });
        dds.drainSegmentAck([&](const lingtu_dds_ExplorationSegmentAck &message) {
          ++counters.segment_ack_messages;
          if (!pending_segment.has_value()) {
            return;
          }
          const lingtu::nav::endpoint::ExplorationSegmentAckEvent event{
              stringValue(message.request_id),
              message.kind,
              message.accepted,
              segmentIdentityFromWire(message.header, message.session_id, message.reset_epoch,
                                      message.generation, message.live),
              stringValue(message.reason),
          };
          const PendingSegmentPhase phase = pending_segment->phase;
          lingtu::nav::endpoint::ExplorationSegmentCommandKind matched_kind =
              phase == PendingSegmentPhase::CancelRequested
                  ? lingtu::nav::endpoint::ExplorationSegmentCommandKind::kCancel
                  : lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute;
          auto reaction = lingtu::nav::endpoint::reactToExplorationSegmentAck(
              &pending_segment->binding, matched_kind, pending_segment->execution_generation,
              event);
          if (!reaction.matched && phase == PendingSegmentPhase::CancelRequested &&
              event.kind == static_cast<std::int32_t>(
                                lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute)) {
            matched_kind = lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute;
            reaction = lingtu::nav::endpoint::reactToExplorationSegmentAck(
                &pending_segment->binding, matched_kind, pending_segment->execution_generation,
                event);
          }
          if (!reaction.matched) {
            return;
          }
          if (pending_segment->execution_generation == 0U &&
              (matched_kind == lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute ||
               event.accepted)) {
            pending_segment->execution_generation = reaction.observed_generation;
          }
          if (reaction.terminal) {
            const PendingSegment terminal_segment = *pending_segment;
            observeMotionStopped(
                loop_now_s,
                "exploration_segment_execute_rejected_no_motion:" +
                    (reaction.reason.empty() ? std::string{"unspecified"} : reaction.reason));
            finishSegmentTerminal(terminal_segment, reaction.terminal_generation, reaction.reason);
            return;
          }
          if (phase == PendingSegmentPhase::CancelRequested &&
              matched_kind == lingtu::nav::endpoint::ExplorationSegmentCommandKind::kCancel &&
              event.accepted) {
            pending_segment->cancel_acknowledged = true;
          }
          if (phase == PendingSegmentPhase::AwaitingExecuteAck && event.accepted) {
            pending_segment->phase = PendingSegmentPhase::Executing;
            pending_segment->phase_started_s = loop_command_s;
            pending_segment->deadline_reported = false;
          }
          const std::string ack_reason =
              reaction.reason.empty() ? std::string{"unspecified"} : reaction.reason;
          if (phase == PendingSegmentPhase::CancelRequested && !event.accepted) {
            (void)run_lifecycle.recordStopConfirmationFailure(
                loop_now_s, "exploration_segment_cancel_rejected:" + ack_reason);
          }
          last_reason = event.accepted ? "exploration_segment_acknowledged:" + ack_reason
                                       : "exploration_segment_cancel_rejected:" + ack_reason;
        });
        dds.drainSegmentStatus([&](const lingtu_dds_ExplorationSegmentStatus &message) {
          ++counters.segment_status_messages;
          if (!pending_segment.has_value()) {
            return;
          }
          const lingtu::nav::endpoint::ExplorationSegmentStatusEvent event{
              stringValue(message.request_id),
              message.state,
              segmentIdentityFromWire(message.header, message.session_id, message.reset_epoch,
                                      message.generation, message.live),
              stringValue(message.reason),
          };
          const auto reaction = lingtu::nav::endpoint::reactToExplorationSegmentStatus(
              &pending_segment->binding, pending_segment->execution_generation, event);
          if (!reaction.matched) {
            return;
          }
          if (pending_segment->execution_generation == 0U) {
            pending_segment->execution_generation = reaction.observed_generation;
          }
          if (reaction.terminal) {
            const PendingSegment terminal_segment = *pending_segment;
            observeMotionStopped(
                loop_now_s,
                "exploration_segment_terminal_after_stop:" +
                    (reaction.reason.empty() ? std::string{"unspecified"} : reaction.reason));
            finishSegmentTerminal(terminal_segment, reaction.terminal_generation, reaction.reason);
            return;
          }
          if (pending_segment->phase == PendingSegmentPhase::AwaitingExecuteAck) {
            pending_segment->phase = PendingSegmentPhase::Executing;
            pending_segment->phase_started_s = loop_command_s;
            pending_segment->deadline_reported = false;
          }
          const std::string status_reason =
              reaction.reason.empty() ? std::string{"unspecified"} : reaction.reason;
          last_reason = "exploration_segment_status:" + status_reason;
        });
        releaseSegmentReplanBarrier();

        if (pending_segment.has_value()) {
          const double phase_timeout_s = pending_segment->phase == PendingSegmentPhase::Executing
                                             ? config.goal_timeout_s
                                             : command_timeout_s;
          const bool phase_expired =
              loop_command_s - pending_segment->phase_started_s >= phase_timeout_s;
          if (phase_expired && pending_segment->phase != PendingSegmentPhase::CancelRequested) {
            const std::string timeout_reason =
                pending_segment->phase == PendingSegmentPhase::AwaitingExecuteAck
                    ? "exploration_segment_execute_ack_timeout"
                    : "exploration_segment_execution_timeout";
            if (run_lifecycle.active() && !run_lifecycle.stopConfirmationPending()) {
              (void)run_lifecycle.fail(loop_now_s, timeout_reason, true);
            }
            if (exploration_control.active()) {
              static_cast<void>(exploration_control.Complete());
            }
            requestSegmentCancel(timeout_reason, loop_command_s);
            last_reason = timeout_reason;
          } else if (phase_expired && !pending_segment->deadline_reported) {
            pending_segment->deadline_reported = true;
            last_reason = "exploration_segment_cancel_motion_stop_unconfirmed";
            (void)run_lifecycle.recordStopConfirmationFailure(loop_now_s, last_reason);
          }
        }

        if (pending_segment.has_value() &&
            (pending_segment->phase == PendingSegmentPhase::AwaitingExecuteAck ||
             pending_segment->phase == PendingSegmentPhase::CancelRequested) &&
            !pending_segment->deadline_reported &&
            !(pending_segment->phase == PendingSegmentPhase::CancelRequested &&
              pending_segment->cancel_acknowledged) &&
            (last_segment_command_attempt_s < 0.0 ||
             loop_command_s - last_segment_command_attempt_s >= config.command_retry_s)) {
          const bool retry_cancel = pending_segment->phase == PendingSegmentPhase::CancelRequested;
          last_segment_command_attempt_s = loop_command_s;
          writeSegmentRequest(
              *pending_segment,
              retry_cancel ? lingtu::nav::endpoint::ExplorationSegmentCommandKind::kCancel
                           : lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute,
              retry_cancel ? "exploration_segment_cancel_retry"
                           : std::string(lingtu::nav::endpoint::kGoalOutsideStaticMapReason));
          last_reason = retry_cancel ? "exploration_segment_cancel_retry"
                                     : "exploration_segment_execute_retry";
        }

        if (pending.has_value() && !cancel_reason.has_value() && robot_fresh &&
            snapshot_route_bound && sameMapEpoch(pending->map, snapshot->identity)) {
          const double distance =
              std::hypot(pending->target.x - robot->pose.x, pending->target.y - robot->pose.y);
          if (distance <= config.arrival_tolerance_m) {
            // Odometry proximity is progress only. NavigationGoalStatus is
            // published after navd's MotionStopBarrier parking gate and is
            // the sole authority allowed to clear this pending motion.
            last_reason = "exploration_goal_arrival_observed_waiting_terminal";
          }
        }

        if (pending.has_value() && !cancel_reason.has_value()) {
          if (!robot_fresh) {
            cancel_reason = "exploration_odometry_stale";
          } else if (!snapshot_route_bound) {
            cancel_reason = "exploration_snapshot_stale";
          } else if (pending->start_acknowledged &&
                     loop_now_s - pending->accepted_s > config.goal_timeout_s) {
            cancel_reason = "exploration_goal_timeout";
            ++counters.goals_timed_out;
          }
        }

        if (exploration_control.running() && !pending.has_value() && !pending_segment.has_value() &&
            !segment_replan_barrier.has_value() && !cancel_reason.has_value() &&
            queued.has_value() && snapshot_route_bound && robot_fresh &&
            loop_now_s >= next_goal_dispatch_s) {
          if (!sameMapEpoch(queued->map, snapshot->identity)) {
            queued.reset();
            last_reason = "queued_goal_map_epoch_changed";
          } else {
            const std::uint64_t goal_sequence = ++goal_request_sequence;
            const std::string requested_task_id =
                goal_request_prefix + "-task-" + std::to_string(goal_sequence);
            const std::string requested_id =
                goal_request_prefix + "-request-" + std::to_string(goal_sequence);
            goal_command_lane.beginGoal(
                requested_task_id, requested_id,
                {queued->target.x, queued->target.y, 0.0, queued->target.yaw}, loop_command_s);
            pending = PendingGoal{
                queued->target, queued->map, requested_task_id, requested_id, {}, loop_now_s,
                false,          false,
            };
            queued.reset();
            last_reason = "exploration_goal_dispatching";
          }
        }

        if (pending.has_value() && cancel_reason.has_value() &&
            !pending->cancellation_acknowledged) {
          if (goal_command_lane.requestCancel(*cancel_reason, loop_command_s)) {
            pending->cancel_request_id = goal_command_lane.cancelRequestId();
          } else {
            pending.reset();
            cancel_reason.reset();
            observeMotionStopped(loop_now_s, "navigation_goal_not_dispatched");
            last_reason = "exploration_goal_cancelled_before_dispatch";
          }
        }

        handleCommandAcks(loop_now_s, true);

        for (const auto &write :
             goal_command_lane.advance(loop_command_s, dds.commandTransportReady(), command_acks)) {
          const bool written = dds.writeNavigationCommand(write, navigation_client_id);
          goal_command_lane.recordWriteResult(write.request_id, written);
          if (!written) {
            last_reason = "exploration_navigation_command_write_failed";
          }
        }
        if (const auto deadline = goal_command_lane.takeDeadline(); deadline.has_value()) {
          if (*deadline == lingtu::nav::endpoint::ExploreGoalCommandDeadline::StartAck) {
            constexpr const char *kReason = "exploration_goal_start_ack_timeout";
            if (run_lifecycle.active() && !run_lifecycle.stopConfirmationPending()) {
              (void)run_lifecycle.fail(loop_now_s, kReason, true);
            }
            if (exploration_control.active()) {
              static_cast<void>(exploration_control.Complete());
            }
            cancel_reason = kReason;
            if (pending.has_value()) {
              if (goal_command_lane.requestCancel(kReason, loop_command_s)) {
                pending->cancel_request_id = goal_command_lane.cancelRequestId();
                for (const auto &write :
                     goal_command_lane.advance(loop_command_s, dds.commandTransportReady(), {})) {
                  const bool written = dds.writeNavigationCommand(write, navigation_client_id);
                  goal_command_lane.recordWriteResult(write.request_id, written);
                }
              } else {
                pending.reset();
                cancel_reason.reset();
                observeMotionStopped(loop_now_s, "navigation_goal_start_not_written");
              }
            }
            last_reason = kReason;
          } else {
            constexpr const char *kReason = "exploration_goal_cancel_motion_stop_unconfirmed";
            (void)run_lifecycle.recordStopConfirmationFailure(loop_now_s, kReason);
            last_reason = kReason;
          }
        }

        if (exploration_control.running() && !pending.has_value() && !pending_segment.has_value() &&
            !segment_replan_barrier.has_value() && !queued.has_value() &&
            !cancel_reason.has_value() && robot_fresh && snapshot_route_bound) {
          const auto &previous = last_decision.diagnostics;
          const bool already_committed =
              previous.state_committed && previous.session_id == snapshot->identity.session_id &&
              previous.map_id == snapshot->identity.map_id &&
              previous.map_content_epoch == snapshot->identity.map_content_epoch &&
              previous.reset_epoch == snapshot->identity.reset_epoch &&
              previous.accepted_generation == snapshot->identity.generation &&
              previous.accepted_intent_revision == directed_intent.revision();
          if (!already_committed) {
            lingtu::explore::ExploreInput input;
            bool borrowed_saved_coverage = false;
            ++counters.plans;
            try {
              if (*config.route == Route::Map) {
                if (!saved_coverage_grid.has_value()) {
                  throw std::runtime_error("saved coverage grid is unavailable");
                }
                input.exploration_grid = std::move(*saved_coverage_grid);
                borrowed_saved_coverage = true;
                input.live_observation_grid = snapshot->grid;
              } else {
                input.exploration_grid = snapshot->grid;
              }
              input.robot_pose = robot->pose;
              input.visited_goals = visited;
              input.stamp_s = snapshot->stamp_s;
              input.map_frame = "map";
              input.map = snapshot->identity;
              input.directed_intent_revision = directed_intent.revision();
              if (const auto directed_target = directed_intent.current(
                      exploration_control.product_session_id(), snapshot->identity, loop_now_s);
                  directed_target.has_value()) {
                input.directed_target = lingtu::explore::DirectedTarget{
                    directed_target->target.x,
                    directed_target->target.y,
                };
              }
              last_decision =
                  policy.plan(input, []() { return !g_running.load(std::memory_order_relaxed); });
              if (borrowed_saved_coverage) {
                *saved_coverage_grid = std::move(input.exploration_grid);
                borrowed_saved_coverage = false;
              }
              last_reason = last_decision.reason;
              if (!last_decision.diagnostics.state_committed) {
                ++counters.plan_rejected;
              } else if (last_decision.has_goal) {
                const Pose2D target{
                    last_decision.goal_x,
                    last_decision.goal_y,
                    last_decision.goal_yaw,
                };
                if (lingtu::nav::endpoint::usesLiveSegment(*config.route)) {
                  beginSegment(target, snapshot->identity, loop_command_s, "live_route");
                  last_reason = "live_route_requested";
                } else {
                  queued = QueuedGoal{target, snapshot->identity};
                }
              }
            } catch (const std::exception &error) {
              if (borrowed_saved_coverage) {
                *saved_coverage_grid = std::move(input.exploration_grid);
              }
              ++counters.plan_rejected;
              last_reason = std::string("exploration_plan_exception:") + error.what();
            }
          }
        }

        if (exploration_control.running() && last_decision.done && !pending.has_value() &&
            !pending_segment.has_value() && !segment_replan_barrier.has_value() &&
            !queued.has_value() && !cancel_reason.has_value()) {
          const std::string completion_reason =
              last_decision.reason.empty() ? "exploration_completed" : last_decision.reason;
          if (run_lifecycle.complete(loop_now_s, completion_reason) &&
              exploration_control.Complete()) {
            directed_intent.Reset();
            last_reason = completion_reason;
          }
        }

        if (cancel_reason.has_value() ||
            (pending_segment.has_value() &&
             pending_segment->phase == PendingSegmentPhase::CancelRequested)) {
          runtime_state = "cancelling";
        } else if (!exploration_control.active()) {
          runtime_state = "idle";
        } else if (exploration_control.paused()) {
          runtime_state = "paused";
        } else if (pending_segment.has_value()) {
          runtime_state = pending_segment->phase == PendingSegmentPhase::AwaitingExecuteAck
                              ? "segment_dispatching"
                              : "segment_executing";
        } else if (pending.has_value()) {
          runtime_state = pending->start_acknowledged ? "executing" : "dispatching";
        } else if (queued.has_value()) {
          runtime_state = "dispatching";
        } else if (segment_replan_barrier.has_value()) {
          runtime_state = "waiting_segment_snapshot";
        } else if (!robot_fresh) {
          runtime_state = "waiting_odometry";
          if (last_reason.empty() || last_reason == "exploration_goal_reached") {
            last_reason = "odometry_missing_or_stale";
          }
        } else if (!snapshot_route_bound) {
          runtime_state = "waiting_map";
          last_reason =
              freshness_reason.empty() ? "exploration_snapshot_missing" : freshness_reason;
        } else if (last_decision.done) {
          runtime_state = "complete";
        } else {
          runtime_state = "planning";
        }
        std::optional<DirectedExplorationIntent> active_directed_target;
        if (snapshot_route_bound && exploration_control.active()) {
          active_directed_target = directed_intent.current(exploration_control.product_session_id(),
                                                           snapshot->identity, loop_now_s);
        }

        if (loop_now_s >= next_status_s) {
          next_status_s = loop_now_s + config.status_period_s;
          status_writer.submit(statusSnapshot(
              config, run_event_outbox.bootId(), runtime_state, last_reason, exploration_control,
              robot_fresh ? &*robot : nullptr, snapshot_route_bound ? &*snapshot : nullptr, pending,
              pending_segment, queued, cancel_reason, active_directed_target, last_decision,
              run_event_outbox.diagnostics(), counters, loop_now_s));
          std::fprintf(stderr,
                       "explore_dds: state=%s reason=%s odom=%llu snapshots=%llu "
                       "plans=%llu goals=%llu/%llu reached=%llu map_gen=%llu\n",
                       runtime_state.c_str(), last_reason.c_str(),
                       static_cast<unsigned long long>(counters.odometry_messages),
                       static_cast<unsigned long long>(counters.snapshot_messages),
                       static_cast<unsigned long long>(counters.plans),
                       static_cast<unsigned long long>(counters.goals_accepted),
                       static_cast<unsigned long long>(counters.goals_rejected),
                       static_cast<unsigned long long>(counters.goals_reached),
                       static_cast<unsigned long long>(
                           snapshot.has_value() ? snapshot->identity.generation : 0U));
        }

        (void)run_event_outbox.flush();

        const auto elapsed = std::chrono::steady_clock::now() - loop_started;
        if (elapsed < period) {
          std::this_thread::sleep_for(period - elapsed);
        }
      }
    } catch (...) {
      loop_error = std::current_exception();
    }

    constexpr const char *kShutdownReason = "exploration_endpoint_stopped";
    constexpr const char *kExceptionReason = "exploration_endpoint_exception";
    const char *cleanup_reason = loop_error ? kExceptionReason : kShutdownReason;
    const double shutdown_now_s = nowSeconds();
    const double shutdown_command_s = monotonicSeconds();
    bool goal_cancel_pending = false;
    if (pending.has_value()) {
      if (!cancel_reason.has_value()) {
        cancel_reason = cleanup_reason;
      }
      if (!goal_command_lane.requestCancel(*cancel_reason, shutdown_command_s)) {
        pending.reset();
      } else {
        pending->cancel_request_id = goal_command_lane.cancelRequestId();
        goal_cancel_pending = true;
      }
    }
    if (pending_segment.has_value()) {
      requestSegmentCancel(cleanup_reason, shutdown_command_s);
    }
    const bool motion_pending = goal_cancel_pending || pending_segment.has_value();
    if (run_lifecycle.active() && !run_lifecycle.stopConfirmationPending() &&
        !run_lifecycle.fail(shutdown_now_s, cleanup_reason, motion_pending)) {
      std::fprintf(stderr, "explore_dds: failed to record shutdown run transition\n");
    }
    if (!motion_pending) {
      observeMotionStopped(shutdown_now_s, "exploration_shutdown_no_pending_motion");
    }

    if (motion_pending) {
      const auto shutdown_deadline =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(config.command_timeout_ms);
      double last_goal_cleanup_attempt_s{-1.0};
      double last_segment_cleanup_attempt_s{-1.0};
      while ((pending.has_value() || pending_segment.has_value()) &&
             std::chrono::steady_clock::now() < shutdown_deadline) {
        const double shutdown_tick_s = nowSeconds();
        const double shutdown_tick_command_s = monotonicSeconds();
        command_acks.clear();
        dds.drainCommandAck([&](const lingtu_dds_NavigationCommandAck &message) {
          if (!lingtu::message::isKnownNavigationCommandKind(message.kind)) {
            return;
          }
          command_acks.push_back({
              stringValue(message.task_id),
              stringValue(message.request_id),
              static_cast<lingtu::message::NavigationCommandKind>(message.kind),
              message.accepted,
              stringValue(message.reason),
          });
        });
        handleCommandAcks(shutdown_tick_s, false);
        if (!pending.has_value()) {
          goal_cancel_pending = false;
        }
        dds.drainGoalStatus([&](const lingtu_dds_NavigationGoalStatus &message) {
          if (!pending.has_value() || !lingtu::message::isKnownNavigationGoalState(message.state) ||
              stringValue(message.task_id) != pending->task_id) {
            return;
          }
          const auto state = static_cast<lingtu::message::NavigationGoalState>(message.state);
          lingtu::nav::endpoint::PendingExploreGoalLifecycle lifecycle{
              pending->request_id,
              pending->cancel_request_id,
          };
          const auto reaction = lingtu::nav::endpoint::reactToNavigationGoalLifecycle(
              &lifecycle, {
                              stringValue(message.request_id),
                              state,
                              stringValue(message.reason),
                          });
          if (!reaction.matched || !reaction.clear_pending) {
            return;
          }
          const std::string status_reason = reaction.reason.empty()
                                                ? lingtu::message::navigationGoalStateName(state)
                                                : reaction.reason;
          goal_command_lane.finishGoal();
          observeMotionStopped(shutdown_tick_s,
                               "navigation_goal_terminal_during_shutdown:" + status_reason);
          pending.reset();
          cancel_reason.reset();
        });
        dds.drainSegmentAck([&](const lingtu_dds_ExplorationSegmentAck &message) {
          if (!pending_segment.has_value()) {
            return;
          }
          const lingtu::nav::endpoint::ExplorationSegmentAckEvent event{
              stringValue(message.request_id),
              message.kind,
              message.accepted,
              segmentIdentityFromWire(message.header, message.session_id, message.reset_epoch,
                                      message.generation, message.live),
              stringValue(message.reason),
          };
          auto matched_kind = lingtu::nav::endpoint::ExplorationSegmentCommandKind::kCancel;
          auto reaction = lingtu::nav::endpoint::reactToExplorationSegmentAck(
              &pending_segment->binding, matched_kind, pending_segment->execution_generation, event);
          if (!reaction.matched &&
              event.kind == static_cast<std::int32_t>(
                                lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute)) {
            matched_kind = lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute;
            reaction = lingtu::nav::endpoint::reactToExplorationSegmentAck(
                &pending_segment->binding, matched_kind, pending_segment->execution_generation,
                event);
          }
          if (!reaction.matched) {
            return;
          }
          if (pending_segment->execution_generation == 0U &&
              (matched_kind == lingtu::nav::endpoint::ExplorationSegmentCommandKind::kExecute ||
               event.accepted)) {
            pending_segment->execution_generation = reaction.observed_generation;
          }
          if (reaction.terminal) {
            const PendingSegment terminal_segment = *pending_segment;
            observeMotionStopped(shutdown_tick_s,
                                 "exploration_segment_terminal_during_shutdown:" + reaction.reason);
            finishSegmentTerminal(terminal_segment, reaction.terminal_generation, reaction.reason);
            return;
          }
          if (matched_kind == lingtu::nav::endpoint::ExplorationSegmentCommandKind::kCancel &&
              event.accepted) {
            pending_segment->cancel_acknowledged = true;
          }
        });
        dds.drainSegmentStatus([&](const lingtu_dds_ExplorationSegmentStatus &message) {
          if (!pending_segment.has_value()) {
            return;
          }
          const lingtu::nav::endpoint::ExplorationSegmentStatusEvent event{
              stringValue(message.request_id),
              message.state,
              segmentIdentityFromWire(message.header, message.session_id, message.reset_epoch,
                                      message.generation, message.live),
              stringValue(message.reason),
          };
          const auto reaction = lingtu::nav::endpoint::reactToExplorationSegmentStatus(
              &pending_segment->binding, pending_segment->execution_generation, event);
          if (!reaction.matched || !reaction.terminal) {
            return;
          }
          const PendingSegment terminal_segment = *pending_segment;
          observeMotionStopped(shutdown_tick_s,
                               "exploration_segment_terminal_during_shutdown:" + reaction.reason);
          finishSegmentTerminal(terminal_segment, reaction.terminal_generation, reaction.reason);
        });
        if (pending.has_value() && !pending->cancel_request_id.empty() &&
            !pending->cancellation_acknowledged &&
            (last_goal_cleanup_attempt_s < 0.0 ||
             shutdown_tick_command_s - last_goal_cleanup_attempt_s >= config.command_retry_s)) {
          const lingtu::nav::endpoint::ExploreGoalCommandWrite write{
              pending->task_id,
              pending->cancel_request_id,
              lingtu::message::NavigationCommandKind::TaskCancel,
              {},
              cancel_reason.value_or(cleanup_reason),
          };
          const bool written = dds.writeNavigationCommand(write, navigation_client_id);
          goal_command_lane.recordWriteResult(write.request_id, written);
          last_goal_cleanup_attempt_s = shutdown_tick_command_s;
        }
        if (pending_segment.has_value() && !pending_segment->cancel_acknowledged &&
            (last_segment_cleanup_attempt_s < 0.0 ||
             shutdown_tick_command_s - last_segment_cleanup_attempt_s >= config.command_retry_s)) {
          writeSegmentRequest(*pending_segment,
                              lingtu::nav::endpoint::ExplorationSegmentCommandKind::kCancel,
                              cleanup_reason);
          last_segment_cleanup_attempt_s = shutdown_tick_command_s;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      if (pending.has_value()) {
        constexpr const char *kReason = "exploration_shutdown_motion_stop_unconfirmed";
        (void)run_lifecycle.recordStopConfirmationFailure(nowSeconds(), kReason);
        std::fprintf(stderr,
                     "explore_dds: shutdown motion terminal not confirmed before deadline\n");
      }
      if (pending_segment.has_value()) {
        constexpr const char *kReason = "exploration_shutdown_segment_motion_stop_unconfirmed";
        (void)run_lifecycle.recordStopConfirmationFailure(nowSeconds(), kReason);
        std::fprintf(stderr,
                     "explore_dds: shutdown segment terminal not confirmed before deadline\n");
      }
    }
    (void)run_event_outbox.flush();
    status_writer.flush();
    if (loop_error) {
      std::rethrow_exception(loop_error);
    }
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "lingtu_explore_dds failed: %s\n", error.what());
    return 1;
  }
}
