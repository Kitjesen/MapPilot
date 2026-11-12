#include <algorithm>
#include "lingtu/maps/io/json.hpp"
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>

#include "dds.hpp"
#include "query.hpp"
#include "server.hpp"
#include "lingtu/maps/mapd/save.hpp"
#include "lingtu/maps/service.hpp"
#include "native/snapshot_file.hpp"

namespace {

namespace map_defaults = lingtu::settings::maps::maps;

using lingtu::maps::JsonEscape;
using lingtu::maps::JsonString;

using lingtu::maps::MapsServiceConfig;
using lingtu::maps::MapsServiceCore;
using lingtu::maps::MapStoreConfig;
using lingtu::maps::mapd::ActivationCoordinator;
using lingtu::maps::mapd::ActivationRequest;
using lingtu::maps::mapd::ActivationResult;
using lingtu::maps::mapd::Config;
using lingtu::maps::mapd::Dds;
using lingtu::maps::mapd::DdsInputState;
using lingtu::maps::mapd::DdsLimits;
using lingtu::maps::mapd::DdsOutputState;
using lingtu::maps::mapd::LiveMapEngine;
using lingtu::maps::mapd::PublicationCursor;
using lingtu::maps::mapd::PublicationProgress;
using lingtu::maps::mapd::SaveCoordinator;
using lingtu::maps::mapd::SaveCoordinatorConfig;
using lingtu::maps::mapd::SnapshotDetail;
using lingtu::maps::mapd::State;
using lingtu::maps::mapd::query::DefaultQuerySocketPath;
using lingtu::maps::mapd::query::MapQueryCore;
using lingtu::maps::mapd::query::QueryServer;
using lingtu::maps::mapd::query::QueryServerConfig;

std::atomic_bool g_running{true};

void StopSignal(int) {
  g_running = false;
}

void AdvanceDeadline(std::chrono::steady_clock::time_point &deadline,
                     std::chrono::steady_clock::duration period,
                     std::chrono::steady_clock::time_point now) {
  do {
    deadline += period;
  } while (deadline <= now);
}

struct Options {
  int domain_id{0};
  std::string product;
  std::string product_session_id;
  std::filesystem::path status_file;
  double state_hz{map_defaults::publish::state_hz};
  double cloud_hz{map_defaults::publish::cloud_hz};
  double map_hz{map_defaults::publish::map_hz};
  double scene_hz{map_defaults::publish::scene_hz};
  std::filesystem::path map_root;
  bool query_enabled{true};
  std::filesystem::path query_socket;
  std::size_t query_max_json_bytes{map_defaults::capacity::query_json_bytes};
  Config engine;
  DdsLimits dds;
  SaveCoordinatorConfig save;
};

std::string EnvOr(const char *name, std::string fallback) {
  const char *value = std::getenv(name);
  return value == nullptr || value[0] == '\0' ? std::move(fallback) : std::string{value};
}

bool EnvEnabled(const char *name, bool fallback) {
  std::string value = EnvOr(name, fallback ? "1" : "0");
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  if (value == "1" || value == "true" || value == "yes" || value == "on")
    return true;
  if (value.empty() || value == "0" || value == "false" || value == "no" || value == "off") {
    return false;
  }
  throw std::invalid_argument(std::string(name) + " is invalid");
}

double ParseDouble(const std::string &value, const char *name) {
  std::size_t consumed = 0U;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(std::string(name) + " is invalid");
  }
  return parsed;
}

std::uint64_t ParseUnsigned(const std::string &value, const char *name) {
  std::size_t consumed = 0U;
  const unsigned long long parsed = std::stoull(value, &consumed);
  if (consumed != value.size()) {
    throw std::invalid_argument(std::string(name) + " is invalid");
  }
  return static_cast<std::uint64_t>(parsed);
}

std::size_t ParseSize(const std::string &value, const char *name) {
  const std::uint64_t parsed = ParseUnsigned(value, name);
  if (parsed > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(std::string(name) + " exceeds size_t");
  }
  return static_cast<std::size_t>(parsed);
}

template <typename Value, typename Parser>
auto EnvNumber(const char* name, Value fallback, Parser parse) -> decltype(parse("", name)) {
  const auto value = EnvOr(name, "");
  return value.empty() ? fallback : parse(value, name);
}

std::filesystem::path ResolveMapRoot() {
  const char *configured = std::getenv("NAV_MAP_DIR");
  if (configured != nullptr && configured[0] != '\0') {
    return std::filesystem::absolute(configured).lexically_normal();
  }
  const char *home = std::getenv("HOME");
  if (home == nullptr || home[0] == '\0') {
    home = std::getenv("USERPROFILE");
  }
  if (home == nullptr || home[0] == '\0') {
    return (std::filesystem::current_path() / "maps").lexically_normal();
  }
  return (std::filesystem::path(home) / "data" / "lingtu" / "maps").lexically_normal();
}

int ParseInt(const std::string &value, const char *name) {
  std::size_t consumed = 0U;
  const long parsed = std::stol(value, &consumed);
  if (consumed != value.size() || parsed < std::numeric_limits<int>::min() ||
      parsed > std::numeric_limits<int>::max()) {
    throw std::invalid_argument(std::string(name) + " is invalid");
  }
  return static_cast<int>(parsed);
}

double LogOdds(double probability, const char *name) {
  if (!(probability > 0.0) || !(probability < 1.0)) {
    throw std::invalid_argument(std::string(name) + " must be in (0, 1)");
  }
  return std::log(probability / (1.0 - probability));
}

int RollMargin(int size, double resolution, double threshold) {
  if (size <= 0 || !(resolution > 0.0) || !(threshold > 0.0)) {
    throw std::invalid_argument("mapd occupancy geometry is invalid");
  }
  const int threshold_cells = std::max(1, static_cast<int>(std::ceil(threshold / resolution)));
  return std::clamp(size / 2 - threshold_cells, 0, (size - 1) / 2);
}

void ConfigureOccupancy(Config *engine) {
  if (engine == nullptr) {
    throw std::invalid_argument("mapd occupancy engine is required");
  }
  auto &occupancy = engine->occupancy;
  occupancy.resolution_m = EnvNumber("LINGTU_MAPD_OCCUPANCY_RESOLUTION_M", occupancy.resolution_m, ParseDouble);
  occupancy.size_x =
      EnvNumber("LINGTU_MAPD_OCCUPANCY_SIZE_X", occupancy.size_x, ParseInt);
  occupancy.size_y =
      EnvNumber("LINGTU_MAPD_OCCUPANCY_SIZE_Y", occupancy.size_y, ParseInt);
  occupancy.size_z =
      EnvNumber("LINGTU_MAPD_OCCUPANCY_SIZE_Z", occupancy.size_z, ParseInt);
  const double slide =
      EnvNumber("LINGTU_MAPD_OCCUPANCY_SLIDE_M", map_defaults::occupancy::slide_m, ParseDouble);
  occupancy.roll_margin_x = RollMargin(occupancy.size_x, occupancy.resolution_m, slide);
  occupancy.roll_margin_y = RollMargin(occupancy.size_y, occupancy.resolution_m, slide);
  occupancy.roll_margin_z = RollMargin(occupancy.size_z, occupancy.resolution_m, slide);
  occupancy.max_ray_range_m = EnvNumber("LINGTU_MAPD_OCCUPANCY_RAY_M", occupancy.max_ray_range_m, ParseDouble);
  occupancy.local_update_range_x_m = 0.5 * occupancy.size_x * occupancy.resolution_m;
  occupancy.local_update_range_y_m = 0.5 * occupancy.size_y * occupancy.resolution_m;
  occupancy.local_update_range_z_m = 0.5 * occupancy.size_z * occupancy.resolution_m;

  const double hit_probability = EnvNumber("LINGTU_MAPD_OCCUPANCY_P_HIT", map_defaults::occupancy::p_hit, ParseDouble);
  const double miss_probability = EnvNumber("LINGTU_MAPD_OCCUPANCY_P_MISS", map_defaults::occupancy::p_miss, ParseDouble);
  const double min_probability = EnvNumber("LINGTU_MAPD_OCCUPANCY_P_MIN", map_defaults::occupancy::p_min, ParseDouble);
  const double max_probability = EnvNumber("LINGTU_MAPD_OCCUPANCY_P_MAX", map_defaults::occupancy::p_max, ParseDouble);
  occupancy.hit_log_odds = LogOdds(hit_probability, "LINGTU_MAPD_OCCUPANCY_P_HIT");
  occupancy.miss_log_odds = -LogOdds(miss_probability, "LINGTU_MAPD_OCCUPANCY_P_MISS");
  occupancy.min_log_odds = LogOdds(min_probability, "LINGTU_MAPD_OCCUPANCY_P_MIN");
  occupancy.max_log_odds = LogOdds(max_probability, "LINGTU_MAPD_OCCUPANCY_P_MAX");
  occupancy.occupied_probability = EnvNumber("LINGTU_MAPD_OCCUPANCY_P_OCC", occupancy.occupied_probability, ParseDouble);
  const double decay_after_s = EnvNumber("LINGTU_MAPD_OCCUPANCY_DECAY_AFTER_S", static_cast<double>(occupancy.decay_after_ns) * 1e-9, ParseDouble);
  if (decay_after_s < 0.0 ||
      decay_after_s > static_cast<double>(std::numeric_limits<std::int64_t>::max()) * 1e-9) {
    throw std::invalid_argument("LINGTU_MAPD_OCCUPANCY_DECAY_AFTER_S is invalid");
  }
  occupancy.decay_after_ns =
      static_cast<std::int64_t>(std::llround(decay_after_s * 1e9));
  occupancy.inflation_radius_m = EnvNumber("LINGTU_MAPD_INFLATION_RADIUS_M", occupancy.inflation_radius_m, ParseDouble);
  occupancy.inflation_z_up_m = EnvNumber("LINGTU_MAPD_INFLATION_Z_UP_M", occupancy.inflation_z_up_m, ParseDouble);
  occupancy.inflation_z_down_m = EnvNumber("LINGTU_MAPD_INFLATION_Z_DOWN_M", occupancy.inflation_z_down_m, ParseDouble);
}

void PrintUsage() {
  std::cout << "mapd [options]\n"
            << "  --domain-id N       CycloneDDS domain id\n"
            << "  --status-file PATH  atomic readiness/status snapshot\n"
            << "  --map-root PATH     map asset root directory\n"
            << "  --state-hz HZ       /maps/state publish rate\n"
            << "  --cloud-hz HZ       live/voxel/local-collision rate limit\n"
            << "  --map-hz HZ         accumulated/grid layer rate limit\n"
            << "  --scene-hz HZ       coherent /maps/scene publish rate\n"
            << "  --query-socket PATH local AF_UNIX map management socket\n"
            << "  --disable-query     disable the local query endpoint explicitly\n"
            << "  --query-max-json-bytes N maximum UDS request/response JSON bytes\n"
            << "  --max-points N      maximum points in one MapObservation\n"
            << "  --max-cloud-bytes N maximum PointCloud2 payload bytes\n"
            << "  --max-fields N      maximum PointCloud2 field descriptors\n"
            << "  --max-point-step N  maximum PointCloud2 bytes per point\n"
            << "  --max-string-bytes N maximum DDS string bytes\n"
            << "  --max-scene-bytes N maximum serialized MapScene payload bytes\n"
            << "  --max-voxel-snapshot-points N bounded scene voxel points\n"
            << "  --voxel-snapshot-radius M local voxel scene radius\n"
            << "  --surface-residual-m M maximum ground fit residual\n"
            << "  --surface-min-columns N minimum distinct ground XY samples\n"
            << "  --max-voxels N     live voxel runtime hard limit\n"
            << "  --max-accumulated-cells N accumulated runtime cell hard limit\n"
            << "  --max-accumulated-blocks N accumulated runtime block hard limit\n"
            << "  --min-range M       near-point filter\n"
            << "  --max-range M       far-point filter\n"
            << "  --decay-ms MS       independent decay timer period\n"
            << "  --stale-ms MS       live input timeout\n";
}

Options ParseOptions(int argc, char **argv) {
  Options options;
  options.domain_id = ParseInt(EnvOr("LINGTU_DDS_DOMAIN_ID", "0"), "LINGTU_DDS_DOMAIN_ID");
  options.product = EnvOr("LINGTU_PRODUCT", "");
  options.save.product = options.product;
  options.product_session_id = EnvOr("LINGTU_PRODUCT_SESSION_ID", "");
  options.save.product_session_id = options.product_session_id;
  options.state_hz = EnvNumber("LINGTU_MAPD_STATE_HZ", options.state_hz, ParseDouble);
  options.cloud_hz = EnvNumber("LINGTU_MAPD_CLOUD_HZ", options.cloud_hz, ParseDouble);
  options.map_hz = EnvNumber("LINGTU_MAPD_MAP_HZ", options.map_hz, ParseDouble);
  options.scene_hz = EnvNumber("LINGTU_MAPD_SCENE_HZ", options.scene_hz, ParseDouble);
  options.engine.max_points_per_observation = EnvNumber("LINGTU_MAPD_MAX_POINTS", options.engine.max_points_per_observation, ParseSize);
  options.engine.min_range_m = static_cast<float>(EnvNumber("LINGTU_MAPD_MIN_RANGE_M", options.engine.min_range_m, ParseDouble));
  options.engine.max_range_m = static_cast<float>(EnvNumber("LINGTU_MAPD_MAX_RANGE_M", options.engine.max_range_m, ParseDouble));
  options.engine.decay_period = std::chrono::milliseconds(EnvNumber("LINGTU_MAPD_DECAY_MS", options.engine.decay_period.count(), ParseUnsigned));
  options.engine.stale_after = std::chrono::milliseconds(EnvNumber("LINGTU_MAPD_STALE_MS", options.engine.stale_after.count(), ParseUnsigned));
  options.engine.ground_surface.max_residual_m = EnvNumber("LINGTU_MAPD_SURFACE_RESIDUAL_M", options.engine.ground_surface.max_residual_m, ParseDouble);
  options.engine.ground_surface.min_support_columns = EnvNumber("LINGTU_MAPD_SURFACE_MIN_COLUMNS", options.engine.ground_surface.min_support_columns, ParseInt);
  options.engine.build_extended_layers = EnvEnabled("LINGTU_MAPD_EXTENDED_LAYERS", true);
  options.save.save_patches = EnvEnabled("LINGTU_MAP_SAVE_PATCHES", map_defaults::save::patches);
  auto &save_request = options.save.request_defaults;
  const bool build_octomap = EnvEnabled("LINGTU_MAP_SAVE_BUILD_OCTOMAP", map_defaults::save::build_octomap);
  save_request.require.occupancy = false;
  save_request.require.octomap = build_octomap;
  save_request.require.esdf = false;
  save_request.require.traversability = false;
  save_request.require.semantic = false;
  save_request.source.dynamic_filter_enabled = EnvEnabled("LINGTU_SAVE_DYNAMIC_FILTER", map_defaults::save::dynamic_filter);
  save_request.source.dynamic_filter_required =
      EnvEnabled("LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED", map_defaults::save::dynamic_filter_required);
  if (const char* command = std::getenv("LINGTU_SAVE_DYNAMIC_FILTER_COMMAND")) {
    save_request.source.dynamic_filter_command = command;
  }
  save_request.source.dynamic_filter_timeout_sec =
      EnvNumber("LINGTU_MAP_SAVE_DYNAMIC_FILTER_TIMEOUT_SEC", map_defaults::save::dynamic_filter_timeout_s, ParseDouble);
  save_request.octomap.resolution = EnvNumber("LINGTU_MAP_SAVE_OCTOMAP_RESOLUTION", save_request.octomap.resolution, ParseDouble);
  save_request.octomap.source_profile = EnvOr("LINGTU_PROFILE", std::string{map_defaults::save::profile});
  save_request.octomap.data_source = EnvOr("LINGTU_DATA_SOURCE", std::string{map_defaults::save::data_source});
  save_request.octomap.slam_source = "native_dds";
  save_request.octomap.localization_source = "native_dds";
  save_request.octomap.mapping_source = "save_map_product_chain";
  if (EnvOr("LINGTU_ENV", "") == "sim" && options.product_session_id.empty()) {
    throw std::invalid_argument("LINGTU_PRODUCT_SESSION_ID is required in simulation");
  }
#if defined(_WIN32)
  const std::string platform_default_status = "mapd_status.json";
#else
  const std::string platform_default_status = "/dev/shm/lingtu/mapd_status.json";
#endif
  const std::string session_root = EnvOr("LINGTU_SESSION_ROOT", "");
  const std::string default_status =
      session_root.empty() ? platform_default_status
                           : (std::filesystem::path(session_root) / "mapd.status.json").string();
  options.status_file = EnvOr("LINGTU_MAPD_STATUS_FILE", default_status);
  options.map_root = ResolveMapRoot();
  options.query_socket = DefaultQuerySocketPath();
  options.query_max_json_bytes = EnvNumber("LINGTU_MAPD_QUERY_MAX_JSON_BYTES", options.query_max_json_bytes, ParseSize);
  options.dds.max_cloud_bytes =
      EnvNumber("LINGTU_MAPD_MAX_CLOUD_BYTES", map_defaults::capacity::cloud_bytes, ParseSize);
  options.dds.max_point_fields =
      EnvNumber("LINGTU_MAPD_MAX_POINT_FIELDS", map_defaults::capacity::point_fields, ParseSize);
  options.dds.max_point_step =
      EnvNumber("LINGTU_MAPD_MAX_POINT_STEP", map_defaults::capacity::point_step, ParseSize);
  options.dds.max_string_bytes =
      EnvNumber("LINGTU_MAPD_MAX_STRING_BYTES", map_defaults::capacity::string_bytes, ParseSize);
  options.dds.max_scene_bytes =
      EnvNumber("LINGTU_MAPD_MAX_SCENE_BYTES", map_defaults::capacity::scene_bytes, ParseSize);
  options.engine.max_voxel_snapshot_points =
      EnvNumber("LINGTU_MAPD_MAX_VOXEL_SNAPSHOT_POINTS", map_defaults::capacity::voxel_snapshot_points, ParseSize);
  options.engine.voxel_snapshot_radius_m = static_cast<float>(EnvNumber("LINGTU_MAPD_VOXEL_SNAPSHOT_RADIUS_M", map_defaults::capacity::voxel_snapshot_radius_m, ParseDouble));
  options.engine.voxel.max_voxels =
      EnvNumber("LINGTU_MAPD_MAX_VOXELS", map_defaults::capacity::voxels, ParseSize);
  options.engine.accumulated.max_runtime_cells = EnvNumber("LINGTU_MAPD_MAX_ACCUMULATED_CELLS", map_defaults::capacity::accumulated_cells, ParseUnsigned);
  options.engine.accumulated.max_runtime_blocks = EnvNumber("LINGTU_MAPD_MAX_ACCUMULATED_BLOCKS", map_defaults::capacity::accumulated_blocks, ParseUnsigned);
  ConfigureOccupancy(&options.engine);

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto next = [&]() -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument(argument + " requires a value");
      }
      return argv[index];
    };
    if (argument == "--help" || argument == "-h") {
      PrintUsage();
      std::exit(0);
    }
    if (argument == "--domain-id") {
      options.domain_id = ParseInt(next(), "--domain-id");
    } else if (argument == "--status-file") {
      options.status_file = next();
    } else if (argument == "--map-root") {
      options.map_root = next();
    } else if (argument == "--state-hz") {
      options.state_hz = ParseDouble(next(), "--state-hz");
    } else if (argument == "--cloud-hz") {
      options.cloud_hz = ParseDouble(next(), "--cloud-hz");
    } else if (argument == "--map-hz" || argument == "--layers-hz") {
      options.map_hz = ParseDouble(next(), argument.c_str());
    } else if (argument == "--scene-hz") {
      options.scene_hz = ParseDouble(next(), "--scene-hz");
    } else if (argument == "--query-socket") {
      options.query_socket = next();
    } else if (argument == "--disable-query") {
      options.query_enabled = false;
    } else if (argument == "--query-max-json-bytes") {
      options.query_max_json_bytes = ParseSize(next(), "--query-max-json-bytes");
    } else if (argument == "--max-points") {
      options.engine.max_points_per_observation = ParseSize(next(), "--max-points");
    } else if (argument == "--max-cloud-bytes") {
      options.dds.max_cloud_bytes = ParseSize(next(), "--max-cloud-bytes");
    } else if (argument == "--max-fields") {
      options.dds.max_point_fields = ParseSize(next(), "--max-fields");
    } else if (argument == "--max-point-step") {
      options.dds.max_point_step = ParseSize(next(), "--max-point-step");
    } else if (argument == "--max-string-bytes") {
      options.dds.max_string_bytes = ParseSize(next(), "--max-string-bytes");
    } else if (argument == "--max-scene-bytes") {
      options.dds.max_scene_bytes = ParseSize(next(), "--max-scene-bytes");
    } else if (argument == "--max-voxel-snapshot-points") {
      options.engine.max_voxel_snapshot_points = ParseSize(next(), "--max-voxel-snapshot-points");
    } else if (argument == "--voxel-snapshot-radius") {
      options.engine.voxel_snapshot_radius_m =
          static_cast<float>(ParseDouble(next(), "--voxel-snapshot-radius"));
    } else if (argument == "--surface-residual-m") {
      options.engine.ground_surface.max_residual_m = ParseDouble(next(), "--surface-residual-m");
    } else if (argument == "--surface-min-columns") {
      options.engine.ground_surface.min_support_columns = ParseInt(next(), "--surface-min-columns");
    } else if (argument == "--max-voxels") {
      options.engine.voxel.max_voxels = ParseSize(next(), "--max-voxels");
    } else if (argument == "--max-accumulated-cells") {
      options.engine.accumulated.max_runtime_cells =
          ParseUnsigned(next(), "--max-accumulated-cells");
    } else if (argument == "--max-accumulated-blocks") {
      options.engine.accumulated.max_runtime_blocks =
          ParseUnsigned(next(), "--max-accumulated-blocks");
    } else if (argument == "--min-range") {
      options.engine.min_range_m = static_cast<float>(ParseDouble(next(), "--min-range"));
    } else if (argument == "--max-range") {
      options.engine.max_range_m = static_cast<float>(ParseDouble(next(), "--max-range"));
    } else if (argument == "--decay-ms") {
      options.engine.decay_period = std::chrono::milliseconds(ParseUnsigned(next(), "--decay-ms"));
    } else if (argument == "--stale-ms") {
      options.engine.stale_after = std::chrono::milliseconds(ParseUnsigned(next(), "--stale-ms"));
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (!(options.engine.ground_surface.max_residual_m > 0.0)) {
    throw std::invalid_argument("surface residual must be positive");
  }
  if (options.engine.ground_surface.min_support_columns < 3) {
    throw std::invalid_argument("surface min columns must be at least 3");
  }
  if (options.domain_id < 0 || options.state_hz <= 0.0 || options.cloud_hz <= 0.0 ||
      options.map_hz <= 0.0 || options.scene_hz <= 0.0 || options.status_file.empty() ||
      (options.query_enabled && options.query_socket.empty()) ||
      options.query_max_json_bytes == 0U) {
    throw std::invalid_argument("mapd options are invalid");
  }
  options.dds.max_points_per_observation = options.engine.max_points_per_observation;
  return options;
}

std::string TimingJson(const lingtu::maps::mapd::Timing &timing) {
  return "{\"samples\":" + std::to_string(timing.samples) +
         ",\"last_ms\":" + std::to_string(timing.last_ms) +
         ",\"mean_ms\":" + std::to_string(timing.samples > 0U ? timing.total_ms / timing.samples : 0.0) +
         ",\"max_ms\":" + std::to_string(timing.max_ms) +
         ",\"max_sequence\":" + std::to_string(timing.max_sequence) +
         ",\"max_generation\":" + std::to_string(timing.max_generation) + "}";
}

std::string StatusJson(const State &state, const DdsInputState &input, const DdsOutputState &output,
                       const PublicationProgress &publications, const std::string &producer_boot_id,
                       const Options &options) {
  const bool publications_ready = publications.BootComplete();
  const bool current_generation_published = publications.CurrentGenerationPublished(state);
  const auto readiness = lingtu::maps::mapd::EvaluateReadiness(state, output, publications);
  return std::string{"{"} + "\"schema_version\":\"lingtu.maps.runtime.v1\"," +
         "\"process\":\"mapd\"," + "\"native_product\":{\"product\":\"" +
         JsonEscape(options.product) + "\",\"product_session_id\":\"" +
         JsonEscape(options.product_session_id) + "\"}," +
         "\"producer_boot_id\":\"" + JsonEscape(producer_boot_id) + "\"," + "\"status\":\"" +
         readiness.status + "\"," + "\"ready\":" + (readiness.ready ? "true" : "false") + "," +
         "\"running\":" + (state.running ? "true" : "false") + "," +
         "\"live\":" + (state.live ? "true" : "false") + "," +
         "\"config\":{" +
         "\"occupancy_resolution_m\":" +
         std::to_string(options.engine.occupancy.resolution_m) + "," +
         "\"occupancy_size_x\":" + std::to_string(options.engine.occupancy.size_x) + "," +
         "\"occupancy_size_y\":" + std::to_string(options.engine.occupancy.size_y) + "," +
         "\"occupancy_size_z\":" + std::to_string(options.engine.occupancy.size_z) + "," +
         "\"occupancy_ray_m\":" +
         std::to_string(options.engine.occupancy.max_ray_range_m) + "," +
         "\"inflation_radius_m\":" +
         std::to_string(options.engine.occupancy.inflation_radius_m) + "," +
         "\"inflation_z_up_m\":" +
         std::to_string(options.engine.occupancy.inflation_z_up_m) + "," +
         "\"inflation_z_down_m\":" +
         std::to_string(options.engine.occupancy.inflation_z_down_m) + "}," +
         "\"extended_layers_enabled\":" +
         (state.extended_layers_enabled ? "true" : "false") + "," +
         "\"reset_epoch\":" + std::to_string(state.reset_epoch) + "," +
         "\"observation_sequence\":" + std::to_string(state.sequence) + "," +
         "\"generation\":" + std::to_string(state.generation) + "," +
         "\"realtime_snapshot_generation\":" + std::to_string(state.realtime_snapshot_generation) +
         "," +
         "\"complete_snapshot_generation\":" + std::to_string(state.complete_snapshot_generation) +
         "," + "\"realtime_snapshot_builds\":" + std::to_string(state.realtime_snapshot_builds) +
         "," + "\"complete_snapshot_builds\":" + std::to_string(state.complete_snapshot_builds) +
         "," + "\"queue_depth\":" + std::to_string(state.queue_depth) + "," +
         "\"live_points\":" + std::to_string(state.live_points) + "," +
         "\"voxel_points\":" + std::to_string(state.voxel_points) + "," +
         "\"voxel_cells\":" + std::to_string(state.voxel_cells) + "," +
         "\"voxel_snapshot_omitted_cells\":" + std::to_string(state.voxel_snapshot_omitted_cells) +
         "," + "\"voxel_capacity_rejections\":" + std::to_string(state.voxel_capacity_rejections) +
         "," + "\"accumulated_cells\":" + std::to_string(state.accumulated_cells) + "," +
         "\"accumulated_snapshot_cells\":" + std::to_string(state.accumulated_snapshot_cells) +
         "," + "\"accumulated_capacity_rejections\":" +
         std::to_string(state.accumulated_capacity_rejections) + "," +
         "\"capacity_limited\":" + (state.capacity_limited ? "true" : "false") + "," +
         "\"pose_quality\":" + std::to_string(state.pose_quality) + "," + "\"pose_state\":\"" +
         JsonEscape(state.pose_state) + "\"," + "\"pose_reason\":\"" +
         JsonEscape(state.pose_reason) + "\"," +
         "\"accepted_observations\":" + std::to_string(state.accepted_observations) + "," +
         "\"processed_observations\":" + std::to_string(state.processed_observations) + "," +
         "\"replaced_observations\":" + std::to_string(state.replaced_observations) + "," +
         "\"stale_observations\":" + std::to_string(state.stale_observations) + "," +
         "\"invalid_observations\":" +
         std::to_string(state.invalid_observations + input.rejected_samples) + "," +
         "\"dds_received\":" + std::to_string(input.received_samples) + "," +
         "\"dds_decoded\":" + std::to_string(input.decoded_samples) + "," +
         "\"dds_rejected\":" + std::to_string(input.rejected_samples) + "," +
         "\"dds_write_attempts\":" + std::to_string(output.write_attempts) + "," +
         "\"dds_write_failures\":" + std::to_string(output.write_failures) + "," +
         "\"dds_serialization_rejections\":" + std::to_string(output.serialization_rejections) +
         "," +
         "\"dds_scene_oversize_rejections\":" + std::to_string(output.scene_oversize_rejections) +
         "," + "\"dds_unhealthy_writers\":" + std::to_string(output.unhealthy_writers) + "," +
         "\"required_publications_ready\":" + (publications_ready ? "true" : "false") + "," +
         "\"current_generation_published\":" + (current_generation_published ? "true" : "false") +
         "," + "\"state_published_generation\":" + std::to_string(publications.state.generation) +
         "," + "\"realtime_clouds_published_generation\":" +
         std::to_string(publications.realtime_clouds.generation) + "," +
         "\"map_layers_published_generation\":" +
         std::to_string(publications.map_layers.generation) + "," +
         "\"scene_published_generation\":" + std::to_string(publications.scene.generation) + "," +
         "\"timing\":{\"process\":" + TimingJson(state.process_timing) +
         ",\"occupancy_update\":" + TimingJson(state.occupancy_update_timing) +
         ",\"view\":" + TimingJson(state.view_timing) +
         ",\"collision_build\":" + TimingJson(state.collision_build_timing) +
         ",\"realtime_publish\":" + TimingJson(output.realtime_publish_timing) +
         ",\"realtime_prepare\":" + TimingJson(output.realtime_prepare_timing) +
         ",\"live_cloud_write\":" + TimingJson(output.live_cloud_write_timing) +
         ",\"voxel_cloud_write\":" + TimingJson(output.voxel_cloud_write_timing) +
         ",\"collision_write\":" + TimingJson(output.collision_write_timing) + "}," +
         "\"collision_publish\":{\"cloud_data_bytes\":" + std::to_string(output.realtime_cloud_bytes) +
         ",\"bitmap_bytes\":" + std::to_string(output.collision_bitmap_bytes) +
         ",\"hit_bounds\":" + std::to_string(output.collision_hit_bounds) +
         ",\"payload_native_bytes\":" + std::to_string(output.collision_payload_native_bytes) +
         ",\"max_payload_native_bytes\":" + std::to_string(output.collision_max_payload_native_bytes) +
         ",\"writer_statistics_available\":" +
         (output.collision_writer_statistics_available ? "true" : "false") +
         ",\"throttle_count\":" + std::to_string(output.collision_throttle_count) +
         ",\"throttle_ns\":" + std::to_string(output.collision_throttle_ns) +
         ",\"rexmit_bytes\":" + std::to_string(output.collision_rexmit_bytes) + "}," +
         "\"engine_error\":\"" + JsonEscape(state.last_error) + "\"," + "\"input_error\":\"" +
         JsonEscape(input.last_error) + "\"," + "\"output_error\":\"" +
         JsonEscape(output.last_error) + "\"}\n";
}

bool WriteStatus(const std::filesystem::path &path, const std::string &payload,
                 std::string *error) noexcept {
  try {
    std::error_code ec;
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path(), ec);
      if (ec) {
        throw std::filesystem::filesystem_error("failed to create mapd status directory",
                                                path.parent_path(), ec);
      }
    }
    const std::filesystem::path temporary = path.string() + ".tmp";
    {
      std::ofstream output(temporary, std::ios::out | std::ios::binary | std::ios::trunc);
      if (!output) {
        throw std::runtime_error("failed to open mapd status temporary file");
      }
      output.write(payload.data(), static_cast<std::streamsize>(payload.size()));
      output.flush();
      if (!output) {
        throw std::runtime_error("failed to write mapd status temporary file");
      }
    }
    if (!lingtu::native::replaceSnapshotFile(temporary, path, &ec)) {
      throw std::filesystem::filesystem_error("failed to publish mapd status", temporary, path, ec);
    }
    if (error != nullptr) {
      error->clear();
    }
    return true;
  } catch (const std::exception &exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }
}

std::chrono::steady_clock::duration Period(double hz) {
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / hz));
}

}  // namespace

int main(int argc, char **argv) {
  std::signal(SIGINT, StopSignal);
  std::signal(SIGTERM, StopSignal);
  try {
    const Options options = ParseOptions(argc, argv);
    LiveMapEngine engine(options.engine);
    Dds dds(options.domain_id, options.dds);
    MapsServiceCore maps_service(MapsServiceConfig{MapStoreConfig{options.map_root}});
    ActivationCoordinator activation(maps_service.Store());
    SaveCoordinator save_coordinator(maps_service, dds, options.save);
    MapQueryCore query_core(maps_service, &save_coordinator);
    std::unique_ptr<QueryServer> query_server;
    if (options.query_enabled) {
      query_server = std::make_unique<QueryServer>(
          query_core, QueryServerConfig{options.query_socket, options.query_max_json_bytes});
    }
    engine.Start();
    auto reference_identity = activation.ActiveIdentity();
    engine.SetReferenceMap(reference_identity);
    if (query_server) {
      query_server->Start();
    }

    auto next_state = std::chrono::steady_clock::now();
    auto next_cloud = next_state;
    auto next_map = next_state;
    auto next_scene = next_state;
    PublicationProgress publications;
    std::string status_error;
    std::chrono::steady_clock::time_point next_status_error_log{};
    struct CachedActivationResult {
      ActivationRequest request;
      ActivationResult result;
    };
    std::unordered_map<std::string, CachedActivationResult> activation_results;
    std::deque<std::string> activation_result_order;
    constexpr std::size_t kActivationResultCacheSize = 128U;
    while (g_running) {
      for (auto& geometry : dds.TakeGeometrySnapshots())
        static_cast<void>(engine.SubmitGeometry(std::move(geometry)));
      for (const auto& ack : engine.TakeGeometryAcks())
        static_cast<void>(dds.PublishGeometryApplied(ack));
      if (auto observation = dds.TakeLatestObservation(); observation.has_value()) {
        static_cast<void>(engine.Submit(std::move(*observation)));
      }
      save_coordinator.Poll();
      for (const auto &request : dds.TakeActivationRequests()) {
        const auto cached = activation_results.find(request.request_id);
        if (cached != activation_results.end()) {
          if (cached->second.request == request) {
            static_cast<void>(dds.PublishActivationAck(cached->second.result));
          } else {
            ActivationResult conflict{request.request_id,
                                      request.operation,
                                      false,
                                      "request_id_conflict",
                                      false,
                                      request.target,
                                      request.previous,
                                      activation.ActiveIdentity(),
                                      dds.ProducerBootId()};
            static_cast<void>(dds.PublishActivationAck(conflict));
          }
          continue;
        }
        ActivationResult result = activation.Execute(request);
        const auto active_identity = activation.ActiveIdentity();
        if (active_identity != reference_identity) {
          reference_identity = active_identity;
          engine.SetReferenceMap(reference_identity);
        }
        result.producer_boot_id = dds.ProducerBootId();
        activation_results.emplace(request.request_id, CachedActivationResult{request, result});
        activation_result_order.push_back(request.request_id);
        if (activation_result_order.size() > kActivationResultCacheSize) {
          activation_results.erase(activation_result_order.front());
          activation_result_order.pop_front();
        }
        static_cast<void>(dds.PublishActivationAck(result));
      }

      const auto now = std::chrono::steady_clock::now();
      State state = engine.GetState();
      std::optional<lingtu::maps::mapd::EngineView> realtime_view;
      std::optional<lingtu::maps::mapd::EngineView> complete_view;
      const auto current_view =
          [&](SnapshotDetail detail) -> const lingtu::maps::mapd::EngineView & {
        auto &view = detail == SnapshotDetail::kRealtime ? realtime_view : complete_view;
        if (!view.has_value()) {
          view = engine.GetView(detail);
        }
        return *view;
      };
      const bool realtime_pending = publications.realtime_clouds.Pending(state);
      const bool map_pending = publications.map_layers.Pending(state);
      const bool scene_pending = publications.scene.Pending(state);
      const bool complete_due = (map_pending && now >= next_map) ||
                                (scene_pending && now >= next_scene) || now >= next_state;
      if (realtime_pending && now >= next_cloud) {
        const auto &value =
            current_view(complete_due ? SnapshotDetail::kComplete : SnapshotDetail::kRealtime);
        publications.realtime_clouds.Complete(
            dds.PublishRealtimeClouds(value.state, value.snapshot), value.snapshot.generation,
            value.state.live);
        AdvanceDeadline(next_cloud, Period(options.cloud_hz), now);
      }
      if (map_pending && now >= next_map) {
        const auto &value = current_view(SnapshotDetail::kComplete);
        publications.map_layers.Complete(dds.PublishMapLayers(value.state, value.snapshot),
                                         value.snapshot.generation, value.state.live);
        AdvanceDeadline(next_map, Period(options.map_hz), now);
      }
      if (scene_pending && now >= next_scene) {
        const auto &value = current_view(SnapshotDetail::kComplete);
        publications.scene.Complete(dds.PublishScene(value.state, value.snapshot),
                                    value.snapshot.generation, value.state.live);
        AdvanceDeadline(next_scene, Period(options.scene_hz), now);
      }
      if (now >= next_state) {
        if (complete_view || !publications.CurrentGenerationPublishedWhenStateArrives(state)) {
          const auto &value = current_view(SnapshotDetail::kComplete);
          state = value.state;
          if (!publications.realtime_clouds.PublishedFor(state)) {
            publications.realtime_clouds.Complete(dds.PublishRealtimeClouds(state, value.snapshot),
                                                  value.snapshot.generation, state.live);
            AdvanceDeadline(next_cloud, Period(options.cloud_hz), now);
          }
          if (!publications.map_layers.PublishedFor(state)) {
            publications.map_layers.Complete(dds.PublishMapLayers(state, value.snapshot),
                                             value.snapshot.generation, state.live);
            AdvanceDeadline(next_map, Period(options.map_hz), now);
          }
          if (!publications.scene.PublishedFor(state)) {
            publications.scene.Complete(dds.PublishScene(state, value.snapshot),
                                        value.snapshot.generation, state.live);
            AdvanceDeadline(next_scene, Period(options.scene_hz), now);
          }
        }
        publications.state.Complete(
            dds.PublishState(state, publications, activation.ActiveIdentity()), state.generation,
            state.live);
        const bool status_written = WriteStatus(
            options.status_file,
            StatusJson(state, dds.GetInputState(), dds.GetOutputState(), publications,
                       dds.ProducerBootId(), options),
            &status_error);
        if (!status_written && now >= next_status_error_log) {
          std::cerr << "mapd: status snapshot unavailable: " << status_error << '\n';
          next_status_error_log = now + std::chrono::seconds(10);
        }
        AdvanceDeadline(next_state, Period(options.state_hz), now);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    engine.Stop();
    if (query_server) {
      query_server->Stop();
    }
    const State final_state = engine.GetState();
    publications.state.Complete(
        dds.PublishState(final_state, publications, activation.ActiveIdentity()),
        final_state.generation, final_state.live);
    if (!WriteStatus(options.status_file,
                     StatusJson(final_state, dds.GetInputState(), dds.GetOutputState(),
                                publications, dds.ProducerBootId(), options),
                     &status_error)) {
      std::cerr << "mapd: final status snapshot unavailable: " << status_error << '\n';
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "mapd: " << error.what() << '\n';
    return 1;
  }
}
