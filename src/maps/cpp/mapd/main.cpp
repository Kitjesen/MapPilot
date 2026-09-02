#include <algorithm>
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
#include "lingtu/maps/mapd/query_core.hpp"
#include "lingtu/maps/mapd/query_server.hpp"
#include "lingtu/maps/mapd/save_coordinator.hpp"
#include "lingtu/maps/service.hpp"
#include "native/snapshot_file.hpp"

namespace {

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
  double state_hz{2.0};
  double cloud_hz{10.0};
  double map_hz{2.0};
  double scene_hz{2.0};
  std::filesystem::path map_root;
  bool query_enabled{true};
  std::filesystem::path query_socket;
  std::size_t query_max_json_bytes{lingtu::maps::mapd::query::kDefaultMaxJsonBytes};
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

std::string FirstConfigured(std::initializer_list<const char *> names) {
  for (const char *name : names) {
    const std::string value = EnvOr(name, "");
    if (!value.empty())
      return value;
  }
  return {};
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

double Probability(double log_odds) {
  return 1.0 / (1.0 + std::exp(-log_odds));
}

float LogOdds(double probability, const char *name) {
  if (!(probability > 0.0) || !(probability < 1.0)) {
    throw std::invalid_argument(std::string(name) + " must be in (0, 1)");
  }
  return static_cast<float>(std::log(probability / (1.0 - probability)));
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
  const double default_slide =
      static_cast<double>(occupancy.size_x / 2 - occupancy.roll_margin_x) * occupancy.resolution_m;
  occupancy.resolution_m = static_cast<float>(ParseDouble(
      EnvOr("LINGTU_MAPD_OCCUPANCY_RESOLUTION_M", std::to_string(occupancy.resolution_m)),
      "LINGTU_MAPD_OCCUPANCY_RESOLUTION_M"));
  occupancy.size_x =
      ParseInt(EnvOr("LINGTU_MAPD_OCCUPANCY_SIZE_X", std::to_string(occupancy.size_x)),
               "LINGTU_MAPD_OCCUPANCY_SIZE_X");
  occupancy.size_y =
      ParseInt(EnvOr("LINGTU_MAPD_OCCUPANCY_SIZE_Y", std::to_string(occupancy.size_y)),
               "LINGTU_MAPD_OCCUPANCY_SIZE_Y");
  occupancy.size_z =
      ParseInt(EnvOr("LINGTU_MAPD_OCCUPANCY_SIZE_Z", std::to_string(occupancy.size_z)),
               "LINGTU_MAPD_OCCUPANCY_SIZE_Z");
  const double slide =
      ParseDouble(EnvOr("LINGTU_MAPD_OCCUPANCY_SLIDE_M", std::to_string(default_slide)),
                  "LINGTU_MAPD_OCCUPANCY_SLIDE_M");
  occupancy.roll_margin_x = RollMargin(occupancy.size_x, occupancy.resolution_m, slide);
  occupancy.roll_margin_y = RollMargin(occupancy.size_y, occupancy.resolution_m, slide);
  occupancy.roll_margin_z = RollMargin(occupancy.size_z, occupancy.resolution_m, slide);
  occupancy.max_ray_range_m = static_cast<float>(
      ParseDouble(EnvOr("LINGTU_MAPD_OCCUPANCY_RAY_M", std::to_string(occupancy.max_ray_range_m)),
                  "LINGTU_MAPD_OCCUPANCY_RAY_M"));

  const double hit_probability = ParseDouble(
      EnvOr("LINGTU_MAPD_OCCUPANCY_P_HIT", std::to_string(Probability(occupancy.hit_log_odds))),
      "LINGTU_MAPD_OCCUPANCY_P_HIT");
  const double miss_probability = ParseDouble(
      EnvOr("LINGTU_MAPD_OCCUPANCY_P_MISS", std::to_string(Probability(-occupancy.miss_log_odds))),
      "LINGTU_MAPD_OCCUPANCY_P_MISS");
  const double min_probability = ParseDouble(
      EnvOr("LINGTU_MAPD_OCCUPANCY_P_MIN", std::to_string(Probability(occupancy.min_log_odds))),
      "LINGTU_MAPD_OCCUPANCY_P_MIN");
  const double max_probability = ParseDouble(
      EnvOr("LINGTU_MAPD_OCCUPANCY_P_MAX", std::to_string(Probability(occupancy.max_log_odds))),
      "LINGTU_MAPD_OCCUPANCY_P_MAX");
  occupancy.hit_log_odds = LogOdds(hit_probability, "LINGTU_MAPD_OCCUPANCY_P_HIT");
  occupancy.miss_log_odds = -LogOdds(miss_probability, "LINGTU_MAPD_OCCUPANCY_P_MISS");
  occupancy.min_log_odds = LogOdds(min_probability, "LINGTU_MAPD_OCCUPANCY_P_MIN");
  occupancy.max_log_odds = LogOdds(max_probability, "LINGTU_MAPD_OCCUPANCY_P_MAX");
  occupancy.occupied_probability = static_cast<float>(ParseDouble(
      EnvOr("LINGTU_MAPD_OCCUPANCY_P_OCC", std::to_string(occupancy.occupied_probability)),
      "LINGTU_MAPD_OCCUPANCY_P_OCC"));
  occupancy.inflation_radius_m = static_cast<float>(ParseDouble(
      EnvOr("LINGTU_MAPD_INFLATION_RADIUS_M", std::to_string(occupancy.inflation_radius_m)),
      "LINGTU_MAPD_INFLATION_RADIUS_M"));
  occupancy.inflation_z_up_m = static_cast<float>(ParseDouble(
      EnvOr("LINGTU_MAPD_INFLATION_Z_UP_M", std::to_string(occupancy.inflation_z_up_m)),
      "LINGTU_MAPD_INFLATION_Z_UP_M"));
  occupancy.inflation_z_down_m = static_cast<float>(ParseDouble(
      EnvOr("LINGTU_MAPD_INFLATION_Z_DOWN_M", std::to_string(occupancy.inflation_z_down_m)),
      "LINGTU_MAPD_INFLATION_Z_DOWN_M"));
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
            << "  --max-voxels N     live voxel runtime hard limit\n"
            << "  --max-accumulated-cells N accumulated runtime cell hard limit\n"
            << "  --max-accumulated-blocks N accumulated runtime block hard limit\n"
            << "  --min-range M       near-point filter\n"
            << "  --max-range M       far-point filter\n"
            << "  --carve-min-z M     column clearing lower sensor-relative Z\n"
            << "  --carve-max-z M     column clearing upper sensor-relative Z\n"
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
  options.engine.build_extended_layers = EnvEnabled("LINGTU_MAPD_EXTENDED_LAYERS", true);
  options.save.save_patches = EnvEnabled("LINGTU_MAP_SAVE_PATCHES", true);
  auto &save_request = options.save.request_defaults;
  const bool build_octomap = EnvEnabled("LINGTU_MAP_SAVE_BUILD_OCTOMAP", true);
  save_request.require.occupancy = true;
  save_request.require.octomap = build_octomap;
  save_request.require.esdf = build_octomap;
  save_request.require.traversability = build_octomap;
  save_request.require.semantic = false;
  save_request.source.dynamic_filter_enabled = EnvEnabled("LINGTU_SAVE_DYNAMIC_FILTER", true);
  save_request.source.dynamic_filter_required =
      EnvEnabled("LINGTU_SAVE_DYNAMIC_FILTER_REQUIRED", true);
  save_request.source.dynamic_filter_command = EnvOr("LINGTU_SAVE_DYNAMIC_FILTER_COMMAND", "");
  save_request.source.dynamic_filter_timeout_sec =
      ParseDouble(EnvOr("LINGTU_MAP_SAVE_DYNAMIC_FILTER_TIMEOUT_SEC", "300"),
                  "LINGTU_MAP_SAVE_DYNAMIC_FILTER_TIMEOUT_SEC");
  save_request.octomap.converter_command =
      FirstConfigured({"LINGTU_MAP_ARTIFACT_CONVERTER", "LINGTU_OCTOPLANNER3D_PCD_CONVERTER",
                       "LINGTU_OCTOMAP_CONVERTER"});
  save_request.octomap.build_mode =
      EnvOr("LINGTU_MAP_SAVE_OCTOMAP_BUILD_MODE", "external_pcl_converter");
  save_request.octomap.resolution = ParseDouble(EnvOr("LINGTU_MAP_SAVE_OCTOMAP_RESOLUTION", "0.20"),
                                                "LINGTU_MAP_SAVE_OCTOMAP_RESOLUTION");
  save_request.octomap.support_dilation_cells =
      ParseInt(EnvOr("LINGTU_MAP_SAVE_OCTOMAP_SUPPORT_DILATION_CELLS", "1"),
               "LINGTU_MAP_SAVE_OCTOMAP_SUPPORT_DILATION_CELLS");
  save_request.octomap.free_layers_above =
      ParseInt(EnvOr("LINGTU_MAP_SAVE_OCTOMAP_FREE_LAYERS_ABOVE", "3"),
               "LINGTU_MAP_SAVE_OCTOMAP_FREE_LAYERS_ABOVE");
  save_request.octomap.free_dilation_cells =
      ParseInt(EnvOr("LINGTU_MAP_SAVE_OCTOMAP_FREE_DILATION_CELLS", "1"),
               "LINGTU_MAP_SAVE_OCTOMAP_FREE_DILATION_CELLS");
  save_request.octomap.frame_id = EnvOr("LINGTU_MAP_FRAME", "map");
  save_request.octomap.source_profile = EnvOr("LINGTU_PROFILE", "native_dds");
  save_request.octomap.data_source = EnvOr("LINGTU_DATA_SOURCE", "field");
  save_request.octomap.slam_source = "native_dds";
  save_request.octomap.localization_source = "native_dds";
  save_request.octomap.mapping_source = "save_map_product_chain";
  save_request.octomap.timeout_sec = ParseDouble(EnvOr("LINGTU_MAP_SAVE_OCTOMAP_TIMEOUT_SEC", "60"),
                                                 "LINGTU_MAP_SAVE_OCTOMAP_TIMEOUT_SEC");
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
  options.query_max_json_bytes = ParseSize(EnvOr("LINGTU_MAPD_QUERY_MAX_JSON_BYTES", "1048576"),
                                           "LINGTU_MAPD_QUERY_MAX_JSON_BYTES");
  options.dds.max_cloud_bytes =
      ParseSize(EnvOr("LINGTU_MAPD_MAX_CLOUD_BYTES", "16777216"), "LINGTU_MAPD_MAX_CLOUD_BYTES");
  options.dds.max_point_fields =
      ParseSize(EnvOr("LINGTU_MAPD_MAX_POINT_FIELDS", "16"), "LINGTU_MAPD_MAX_POINT_FIELDS");
  options.dds.max_point_step =
      ParseSize(EnvOr("LINGTU_MAPD_MAX_POINT_STEP", "64"), "LINGTU_MAPD_MAX_POINT_STEP");
  options.dds.max_string_bytes =
      ParseSize(EnvOr("LINGTU_MAPD_MAX_STRING_BYTES", "4096"), "LINGTU_MAPD_MAX_STRING_BYTES");
  options.dds.max_scene_bytes =
      ParseSize(EnvOr("LINGTU_MAPD_MAX_SCENE_BYTES", "33554432"), "LINGTU_MAPD_MAX_SCENE_BYTES");
  options.engine.max_voxel_snapshot_points =
      ParseSize(EnvOr("LINGTU_MAPD_MAX_VOXEL_SNAPSHOT_POINTS", "200000"),
                "LINGTU_MAPD_MAX_VOXEL_SNAPSHOT_POINTS");
  options.engine.voxel_snapshot_radius_m = static_cast<float>(ParseDouble(
      EnvOr("LINGTU_MAPD_VOXEL_SNAPSHOT_RADIUS_M", "30"), "LINGTU_MAPD_VOXEL_SNAPSHOT_RADIUS_M"));
  options.engine.voxel.max_voxels =
      ParseSize(EnvOr("LINGTU_MAPD_MAX_VOXELS", "500000"), "LINGTU_MAPD_MAX_VOXELS");
  options.engine.accumulated.max_runtime_cells = ParseUnsigned(
      EnvOr("LINGTU_MAPD_MAX_ACCUMULATED_CELLS", "2000000"), "LINGTU_MAPD_MAX_ACCUMULATED_CELLS");
  options.engine.accumulated.max_runtime_blocks = ParseUnsigned(
      EnvOr("LINGTU_MAPD_MAX_ACCUMULATED_BLOCKS", "4096"), "LINGTU_MAPD_MAX_ACCUMULATED_BLOCKS");
  options.engine.column_carving_min_height_from_sensor_m = static_cast<float>(
      ParseDouble(EnvOr("LINGTU_MAPD_CARVE_MIN_Z_M", "-0.7"), "LINGTU_MAPD_CARVE_MIN_Z_M"));
  options.engine.column_carving_max_height_from_sensor_m = static_cast<float>(
      ParseDouble(EnvOr("LINGTU_MAPD_CARVE_MAX_Z_M", "1.8"), "LINGTU_MAPD_CARVE_MAX_Z_M"));
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
    } else if (argument == "--carve-min-z") {
      options.engine.column_carving_min_height_from_sensor_m =
          static_cast<float>(ParseDouble(next(), "--carve-min-z"));
    } else if (argument == "--carve-max-z") {
      options.engine.column_carving_max_height_from_sensor_m =
          static_cast<float>(ParseDouble(next(), "--carve-max-z"));
    } else if (argument == "--decay-ms") {
      options.engine.decay_period = std::chrono::milliseconds(ParseUnsigned(next(), "--decay-ms"));
    } else if (argument == "--stale-ms") {
      options.engine.stale_after = std::chrono::milliseconds(ParseUnsigned(next(), "--stale-ms"));
    } else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
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

std::string JsonEscape(const std::string &value) {
  std::string escaped;
  escaped.reserve(value.size() + 8U);
  for (const char character : value) {
    switch (character) {
      case '\\':
        escaped += "\\\\";
        break;
      case '"':
        escaped += "\\\"";
        break;
      case '\n':
        escaped += "\\n";
        break;
      case '\r':
        escaped += "\\r";
        break;
      case '\t':
        escaped += "\\t";
        break;
      default:
        escaped += character;
        break;
    }
  }
  return escaped;
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
