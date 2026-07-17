#include "lingtu/maps/service.hpp"

#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/map_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>
#include <vector>

namespace lingtu::maps {
namespace {

std::string JsonEscape(const std::string& value) {
  std::ostringstream stream;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        stream << "\\\"";
        break;
      case '\\':
        stream << "\\\\";
        break;
      case '\b':
        stream << "\\b";
        break;
      case '\f':
        stream << "\\f";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (ch < 0x20U) {
          constexpr char kHex[] = "0123456789abcdef";
          stream << "\\u00" << kHex[(ch >> 4U) & 0x0FU] << kHex[ch & 0x0FU];
        } else {
          stream << static_cast<char>(ch);
        }
    }
  }
  return stream.str();
}

std::string JsonString(const std::string& value) {
  return "\"" + JsonEscape(value) + "\"";
}

std::string StateName(MapState state) {
  switch (state) {
    case MapState::kDraft:
      return "CREATED";
    case MapState::kStale:
      return "STALE";
    case MapState::kValidated:
      return "READY";
    case MapState::kActive:
      return "ACTIVE";
    case MapState::kRetired:
      return "RETIRED";
    case MapState::kFailed:
      return "FAILED";
  }
  return "FAILED";
}

std::string ArtifactTypeName(ArtifactType type) {
  switch (type) {
    case ArtifactType::kPointCloud:
      return "POINTCLOUD";
    case ArtifactType::kOccupancy2D:
      return "OCCUPANCY_2D";
    case ArtifactType::kOctomap3D:
      return "OCTOMAP_3D";
    case ArtifactType::kEsdf:
      return "ESDF";
    case ArtifactType::kTraversability:
      return "TRAVERSABILITY";
    case ArtifactType::kSemantic:
      return "SEMANTIC";
  }
  return "UNKNOWN";
}

std::string ConfigValue(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key) {
  const auto it = values.find(key);
  return it == values.end() ? std::string{} : it->second;
}

ArtifactType ArtifactTypeForCapability(const std::string& capability, bool* ok) {
  if (ok != nullptr) {
    *ok = true;
  }
  if (capability == "source_pointcloud" || capability == "visualization") {
    return ArtifactType::kPointCloud;
  }
  if (capability == "path_planning_2d" || capability == "path_planning" ||
      capability == "global_2d_planning") {
    return ArtifactType::kOccupancy2D;
  }
  if (capability == "terrain_reasoning" || capability == "global_planning_2_5d") {
    return ArtifactType::kTraversability;
  }
  if (capability == "navigation_safety_3d" || capability == "global_planning_3d" ||
      capability == "collision_3d") {
    return ArtifactType::kOctomap3D;
  }
  if (capability == "trajectory_optimization" || capability == "esdf") {
    return ArtifactType::kEsdf;
  }
  if (capability == "traversability" || capability == "navigation_cost" ||
      capability == "local_planning_cost") {
    return ArtifactType::kTraversability;
  }
  if (capability == "semantic_query" || capability == "semantic") {
    return ArtifactType::kSemantic;
  }
  if (ok != nullptr) {
    *ok = false;
  }
  return ArtifactType::kPointCloud;
}

std::string BoolJson(bool value) {
  return value ? "true" : "false";
}

double UnixSecondsNow() {
  return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

double FileUnixSeconds(const std::filesystem::path& path) {
  try {
    const auto file_time = std::filesystem::last_write_time(path);
    const auto system_time = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
        file_time - decltype(file_time)::clock::now() + std::chrono::system_clock::now());
    return std::chrono::duration<double>(system_time.time_since_epoch()).count();
  } catch (const std::exception&) {
    return 0.0;
  }
}

bool HasPlanningArtifact(const MapRecord& record) {
  return std::any_of(record.artifacts.begin(), record.artifacts.end(), [](const auto& artifact) {
    return artifact.type == ArtifactType::kOccupancy2D ||
        artifact.type == ArtifactType::kOctomap3D ||
        artifact.type == ArtifactType::kEsdf ||
        artifact.type == ArtifactType::kTraversability;
  });
}

std::filesystem::path ExchangeRoot(
    const std::filesystem::path& map_root,
    const char* env_name,
    const char* leaf) {
  const char* configured = std::getenv(env_name);
  if (configured != nullptr && *configured != '\0') {
    return std::filesystem::absolute(configured).lexically_normal();
  }
  return std::filesystem::absolute(map_root / ".exchange" / leaf).lexically_normal();
}

std::filesystem::path BoundaryPath(const std::filesystem::path& path) {
  std::error_code ec;
  auto resolved = std::filesystem::weakly_canonical(path, ec);
  if (!ec) {
    return resolved.lexically_normal();
  }
  return std::filesystem::absolute(path).lexically_normal();
}

bool PathWithin(const std::filesystem::path& candidate, const std::filesystem::path& root) {
  const auto relative = BoundaryPath(candidate).lexically_relative(BoundaryPath(root));
  if (relative.empty() || relative.is_absolute() || relative == ".") {
    return false;
  }
  return std::none_of(relative.begin(), relative.end(), [](const auto& part) {
    return part == "..";
  });
}

std::string StringArrayJson(const std::vector<std::string>& values) {
  std::ostringstream out;
  out << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i != 0U) {
      out << ",";
    }
    out << JsonString(values[i]);
  }
  out << "]";
  return out.str();
}

std::string MetricJson(const health::MetricValue& metric) {
  const bool known = metric.state == health::MetricState::kOk;
  return "{"
      "\"state\":" + JsonString(health::ToString(metric.state)) + "," +
      "\"score\":" + (known ? std::to_string(metric.score) : "null") + "," +
      "\"reason\":" + JsonString(metric.reason) + "," +
      "\"provenance\":" + StringArrayJson(metric.provenance) +
      "}";
}

std::string ExistingPathJson(const std::filesystem::path& path) {
  return std::filesystem::is_regular_file(path) ? JsonString(path.string()) : "null";
}

bool SafeTsvField(const std::string& value) {
  return value.find('\t') == std::string::npos &&
      value.find('\n') == std::string::npos &&
      value.find('\r') == std::string::npos;
}

std::vector<std::string> SplitTab(const std::string& line) {
  std::vector<std::string> out;
  std::string item;
  std::istringstream stream(line);
  while (std::getline(stream, item, '\t')) {
    out.push_back(item);
  }
  return out;
}

std::string EncodeGraphMetadataValue(const std::string& value) {
  std::ostringstream out;
  out << std::uppercase << std::hex;
  for (const unsigned char ch : value) {
    const bool plain =
        (ch >= 'A' && ch <= 'Z') ||
        (ch >= 'a' && ch <= 'z') ||
        (ch >= '0' && ch <= '9') ||
        ch == '_' || ch == '-' || ch == '.' || ch == '/';
    if (plain) {
      out << static_cast<char>(ch);
    } else {
      out << '%' << std::setw(2) << std::setfill('0') << static_cast<int>(ch);
    }
  }
  return out.str();
}

int HexDigit(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'a' && ch <= 'f') return 10 + ch - 'a';
  if (ch >= 'A' && ch <= 'F') return 10 + ch - 'A';
  return -1;
}

std::string DecodeGraphValue(const std::string& value) {
  std::string out;
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] != '%' || i + 2U >= value.size()) {
      out.push_back(value[i]);
      continue;
    }
    const int hi = HexDigit(value[i + 1U]);
    const int lo = HexDigit(value[i + 2U]);
    if (hi < 0 || lo < 0) {
      out.push_back(value[i]);
      continue;
    }
    out.push_back(static_cast<char>((hi << 4) | lo));
    i += 2U;
  }
  return out;
}

std::unordered_map<std::string, std::string> ParseGraphFields(const std::string& line) {
  std::unordered_map<std::string, std::string> fields;
  std::size_t start = 0U;
  while (start < line.size()) {
    const std::size_t end = line.find(' ', start);
    const std::string token = line.substr(
        start,
        end == std::string::npos ? std::string::npos : end - start);
    const std::size_t sep = token.find('=');
    if (sep != std::string::npos) {
      fields[DecodeGraphValue(token.substr(0, sep))] = DecodeGraphValue(token.substr(sep + 1U));
    }
    if (end == std::string::npos) break;
    start = end + 1U;
  }
  return fields;
}

std::string GraphNodeId(const std::string& map_id) {
  return "map:" + map_id;
}

std::string GraphEdgeId(const std::string& from_map_id, const std::string& to_map_id) {
  return "map:" + from_map_id + "->" + to_map_id;
}

std::string MapIdFromGraphNodeId(const std::string& node_id) {
  constexpr const char* kPrefix = "map:";
  return node_id.rfind(kPrefix, 0) == 0 ? node_id.substr(4U) : node_id;
}

std::string JsonObjectOrEmpty(const std::string& value) {
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "{}";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  const std::string trimmed = value.substr(begin, end - begin + 1U);
  if (trimmed.size() >= 2U && trimmed.front() == '{' && trimmed.back() == '}') {
    return trimmed;
  }
  return "{}";
}

bool BuildRunningForMap(const std::filesystem::path& map_dir) {
  return std::filesystem::exists(map_dir / ".build_lock");
}

std::string NativeOctomapEmbeddedJson() {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  return "true";
#else
  return "false";
#endif
}

std::string SupportedOctomapBuildModesJson() {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  return "[\"native_octomap\",\"external_pcl_converter\"]";
#else
  return "[\"external_pcl_converter\"]";
#endif
}

std::string OctomapAlgorithmName() {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  return "native_octomap_octree_or_external_converter";
#else
  return "external_octomap_converter";
#endif
}

}  // namespace

MapsServiceCore::MapsServiceCore(MapsServiceConfig config)
    : store_(std::move(config.store)),
      pipeline_(store_),
      save_map_(store_),
      artifact_jobs_(
          ArtifactJobWorkerConfig{
              store_.RootDir() / ".jobs" / "artifacts",
              64U,
              std::chrono::seconds(30),
              std::chrono::seconds(1),
              false},
          [this](const ArtifactJobRequest& request, ArtifactJobContext& context) {
            return RunArtifactJob(request, context);
          }),
      maintenance_(MapMaintenanceConfig{store_.RootDir()}),
      active_slots_(ActiveSlotsPath(), [this](const std::string& map_id) {
        if (!MapStore::IsValidMapId(map_id)) {
          return false;
        }
        const auto record = store_.GetMapRecord(map_id);
        return record.has_value() && record->state != MapState::kRetired;
      }) {
  artifact_jobs_.Recover();
  artifact_jobs_.Start();
  const std::string active = store_.ActiveMapId();
  if (!active.empty() && active_slots_.Get("navigation").empty()) {
    (void)active_slots_.Set("navigation", active);
  }
}

std::string MapsServiceCore::GetMapTypesJson() const {
  constexpr const char* kStates =
      "[\"EMPTY\",\"CREATED\",\"BUILDING\",\"READY\",\"ACTIVE\",\"STALE\",\"FAILED\",\"RETIRED\"]";
  constexpr const char* kClasses =
      "[\"saved_point_cloud\",\"static_2d_occupancy\",\"portable_2d_occupancy\","
      "\"global_3d_occupancy\",\"esdf\",\"traversability\","
      "\"semantic\",\"map_metadata\"]";
  constexpr const char* kArtifacts =
      "{"
      "\"map_pcd\":{\"filename\":\"map.pcd\",\"map_class\":\"saved_point_cloud\","
      "\"role\":\"raw_slam_map_source\",\"type\":\"POINTCLOUD\",\"capability\":\"source_pointcloud\"},"
      "\"occupancy_grid\":{\"filename\":\"occupancy.npz\",\"map_class\":\"static_2d_occupancy\","
      "\"role\":\"static_grid_helper\",\"type\":\"OCCUPANCY_2D\",\"capability\":\"path_planning_2d\"},"
      "\"octomap\":{\"filename\":\"octomap.ot\",\"map_class\":\"global_3d_occupancy\","
      "\"role\":\"octoplanner3d_global_planning\",\"type\":\"OCTOMAP_3D\",\"capability\":\"navigation_safety_3d\"},"
      "\"map_yaml\":{\"filename\":\"map.yaml\",\"map_class\":\"portable_2d_occupancy\","
      "\"role\":\"occupancy_yaml_compatibility\",\"type\":\"OCCUPANCY_YAML\",\"capability\":\"occupancy_yaml\"},"
      "\"map_pgm\":{\"filename\":\"map.pgm\",\"map_class\":\"portable_2d_occupancy\","
      "\"role\":\"occupancy_image_compatibility\",\"type\":\"OCCUPANCY_PGM\",\"capability\":\"occupancy_image\"},"
      "\"metadata\":{\"filename\":\"metadata.json\",\"map_class\":\"map_metadata\","
      "\"role\":\"artifact_provenance\",\"type\":\"MAP_METADATA\",\"capability\":\"artifact_provenance\"},"
      "\"esdf\":{\"filename\":\"esdf.npz\",\"map_class\":\"esdf\","
      "\"role\":\"future_local_planner_constraint\",\"type\":\"ESDF\",\"capability\":\"trajectory_optimization\"},"
      "\"traversability\":{\"filename\":\"traversability.npz\",\"map_class\":\"traversability\","
      "\"role\":\"navigation_cost_layer\",\"type\":\"TRAVERSABILITY\",\"capability\":\"traversability\"},"
      "\"semantic\":{\"filename\":\"semantic_map.bin\",\"map_class\":\"semantic\","
      "\"role\":\"semantic_query\",\"type\":\"SEMANTIC\",\"capability\":\"semantic_query\"}"
      "}";
  constexpr const char* kAliases = "{\"occupancy\":\"occupancy_grid\"}";
  constexpr const char* kCapabilities =
      "{"
      "\"artifact_provenance\":\"MAP_METADATA\","
      "\"global_planning_2_5d\":\"TRAVERSABILITY\","
      "\"global_planning_3d\":\"OCTOMAP_3D\","
      "\"navigation_safety_3d\":\"OCTOMAP_3D\","
      "\"occupancy_image\":\"OCCUPANCY_PGM\","
      "\"occupancy_yaml\":\"OCCUPANCY_YAML\","
      "\"path_planning\":\"OCCUPANCY_2D\","
      "\"path_planning_2d\":\"OCCUPANCY_2D\","
      "\"semantic_query\":\"SEMANTIC\","
      "\"source_pointcloud\":\"POINTCLOUD\","
      "\"terrain_reasoning\":\"TRAVERSABILITY\","
      "\"trajectory_optimization\":\"ESDF\","
      "\"traversability\":\"TRAVERSABILITY\""
      "}";
  const std::string builders =
      "{"
      "\"occupancy_grid\":{\"native\":true,\"algorithm\":\"pcd_projection_snapshot\","
      "\"transactional_in_navigation_package\":true},"
      "\"esdf\":{\"native\":true,\"algorithm\":\"distance_transform_from_occupancy\","
      "\"transactional_in_navigation_package\":true},"
      "\"traversability\":{\"native\":true,\"algorithm\":\"cost_from_occupancy_esdf\","
      "\"transactional_in_navigation_package\":true},"
      "\"octomap\":{\"native_orchestration\":true,\"algorithm_embedded\":" +
      NativeOctomapEmbeddedJson() + ","
      "\"algorithm\":" + JsonString(OctomapAlgorithmName()) + ","
      "\"supported_build_modes\":" + SupportedOctomapBuildModesJson() + ","
      "\"transactional_in_navigation_package\":true}"
      "}";
  const std::string catalog = "{"
      "\"schema_version\":\"map.types\","
      "\"record_schema_version\":\"map.record\","
      "\"bundle_schema_version\":\"map.bundle\","
      "\"states\":" + std::string(kStates) + ","
      "\"classes\":" + std::string(kClasses) + ","
      "\"artifacts\":" + std::string(kArtifacts) + ","
      "\"aliases\":" + std::string(kAliases) + ","
      "\"capabilities\":" + std::string(kCapabilities) + ","
      "\"builders\":" + builders +
      "}";
  return "{"
      "\"action\":\"get_map_types\","
      "\"success\":true,"
      "\"schema_version\":\"map.types\","
      "\"record_schema_version\":\"map.record\","
      "\"bundle_schema_version\":\"map.bundle\","
      "\"catalog\":" + catalog + ","
      "\"states\":" + std::string(kStates) + ","
      "\"classes\":" + std::string(kClasses) + ","
      "\"artifacts\":" + std::string(kArtifacts) + ","
      "\"aliases\":" + std::string(kAliases) + ","
      "\"capabilities\":" + std::string(kCapabilities) + ","
      "\"builders\":" + builders +
      "}";
}

std::string MapsServiceCore::ListMapsJson() const {
  std::ostringstream maps;
  maps << "[";
  const auto ids = store_.ListMapIds();
  bool first = true;
  for (const auto& id : ids) {
    const auto record = store_.GetMapRecord(id);
    if (!record.has_value()) {
      continue;
    }
    const auto dir = store_.ContentPath(id);
    const auto pcd_path = dir / "map.pcd";
    const bool has_pcd = std::filesystem::is_regular_file(pcd_path);
    const bool has_occupancy = std::filesystem::is_regular_file(dir / "occupancy.npz");
    const bool has_octomap =
        std::filesystem::is_regular_file(dir / "octomap.ot") ||
        std::filesystem::is_regular_file(dir / "octomap.bt");
    const bool has_esdf = std::filesystem::is_regular_file(dir / "esdf.npz");
    const bool has_traversability = std::filesystem::is_regular_file(dir / "traversability.npz");
    const bool navigation_ready = has_pcd &&
        (has_occupancy || has_octomap || has_esdf || has_traversability);
    const auto patches_dir = dir / "patches";
    size_t patch_count = 0U;
    if (std::filesystem::is_directory(patches_dir)) {
      for (const auto& patch : std::filesystem::directory_iterator(patches_dir)) {
        if (patch.is_regular_file() && patch.path().extension() == ".pcd") {
          ++patch_count;
        }
      }
    }
    double size_mb = 0.0;
    if (has_pcd) {
      size_mb = static_cast<double>(std::filesystem::file_size(pcd_path)) / (1024.0 * 1024.0);
    }
    if (!first) {
      maps << ",";
    }
    first = false;
    maps
        << "{"
        << "\"name\":" << JsonString(id) << ","
        << "\"map_dir\":" << JsonString(dir.string()) << ","
        << "\"has_pcd\":" << BoolJson(has_pcd) << ","
        << "\"has_occupancy\":" << BoolJson(has_occupancy) << ","
        << "\"has_octomap\":" << BoolJson(has_octomap) << ","
        << "\"has_esdf\":" << BoolJson(has_esdf) << ","
        << "\"has_traversability\":" << BoolJson(has_traversability) << ","
        << "\"navigation_ready\":" << BoolJson(navigation_ready) << ","
        << "\"is_active\":" << BoolJson(record->state == MapState::kActive) << ","
        << "\"size_mb\":" << size_mb << ","
        << "\"patch_count\":" << patch_count << ","
        << "\"record\":" << RecordJson(*record) << ","
        << "\"state\":" << JsonString(StateName(record->state))
        << "}";
  }
  maps << "]";
  return "{"
      "\"action\":\"list\","
      "\"success\":true,"
      "\"maps\":" + maps.str() + ","
      "\"active\":" + JsonString(store_.ActiveMapId()) +
      "}";
}

std::string MapsServiceCore::GetRecordJson(const std::string& map_id) const {
  try {
    const auto record = store_.GetMapRecord(map_id);
    if (!record.has_value()) {
      return FailureJson("get_record", "map not found: " + map_id, "map_not_found");
    }
    return "{"
        "\"action\":\"get_record\","
        "\"success\":true,"
        "\"record\":" + RecordJson(*record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("get_record", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::GetActiveMapJson() const {
  const std::string active = store_.ActiveMapId();
  if (active.empty()) {
    return FailureJson("get_active", "no active map", "map_not_found");
  }
  const auto record = store_.GetActiveMap();
  if (!record.has_value()) {
    return FailureJson("get_active", "active map not found: " + active, "map_not_found");
  }
  return "{"
      "\"action\":\"get_active\","
      "\"success\":true,"
      "\"active\":" + JsonString(active) + ","
      "\"artifacts\":" + ActiveArtifactsJson(active) + ","
      "\"record\":" + RecordJson(*record) +
      "}";
}

std::string MapsServiceCore::GetHealthJson(const std::string& map_id) const {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("get_health", "missing map name", "missing_map_name");
  }
  try {
    const auto record = store_.GetMapRecord(resolved);
    if (!record.has_value()) {
      return FailureJson("get_health", "map not found: " + resolved, "map_not_found");
    }
    return "{"
        "\"action\":\"get_health\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(resolved) + ","
        "\"health\":" + HealthJson(*record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("get_health", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::ValidateArtifactsJson(
    const std::string& map_id,
    bool require_octomap,
    bool require_occupancy,
    const std::string& expected_frame_id) const {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    ArtifactValidationOptions options;
    options.require_octomap = require_octomap;
    options.require_occupancy = require_occupancy;
    options.expected_frame_id = expected_frame_id;
    const auto gate = store_.ValidateArtifacts(id, options);
    if (!gate.map_found) {
      return FailureJson("validate_artifacts", "map not found: " + id, "map_not_found");
    }
    const auto artifact_json = [&](
        const ArtifactCheck& artifact,
        bool derived) {
      if (artifact.path.empty()) {
        return std::string("{\"exists\":false,\"sha256_ok\":false,\"path\":null") +
            (derived ? ",\"source_map_sha256_matches_map\":false}" : "}");
      }
      return "{\"exists\":" + BoolJson(artifact.exists) +
          ",\"sha256_ok\":" + BoolJson(artifact.sha256_ok) +
          ",\"path\":" + JsonString(artifact.path.string()) +
          ",\"sha256\":" + JsonString(artifact.sha256) +
          (derived
              ? ",\"source_map_sha256_matches_map\":" +
                  BoolJson(artifact.source_map_sha256_matches_map)
              : std::string{}) + "}";
    };
    return "{"
        "\"action\":\"validate_artifacts\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"map_dir\":" + JsonString(gate.map_dir.string()) + ","
        "\"gate\":{"
        "\"schema_version\":\"lingtu.saved_map_artifacts.gate.v2\","
        "\"ok\":" + BoolJson(gate.ok) + ","
        "\"checked_frame_id\":" + JsonString(gate.checked_frame_id) + ","
        "\"expected_frame_id\":" + JsonString(gate.expected_frame_id) + ","
        "\"version_integrity_ok\":" + BoolJson(gate.version_integrity_ok) + ","
        "\"version_integrity_message\":" + JsonString(gate.version_integrity_message) + ","
        "\"metadata_ok\":" + BoolJson(gate.metadata_ok) + ","
        "\"artifacts\":{"
        "\"map_pcd\":" + artifact_json(gate.map_pcd, false) + ","
        "\"octomap\":" + artifact_json(gate.octomap, true) + ","
        "\"occupancy_grid\":" +
            artifact_json(gate.occupancy_grid, true) + "},"
        "\"blockers\":" + StringArrayJson(gate.blockers) +
        "}}";
  } catch (const std::exception& exc) {
    return FailureJson("validate_artifacts", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::IngestLocalizationHealthJson(
    const std::string& map_id,
    double timestamp_s,
    bool localized,
    double position_error_m,
    double covariance_trace,
    double quality,
    const std::string& source) {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  const auto record = store_.GetMapRecord(resolved);
  if (!record.has_value()) {
    return FailureJson("ingest_localization_health", "map not found: " + resolved, "map_not_found");
  }
  auto* model = HealthModelFor(*record);
  model->IngestLocalizationSample(health::LocalizationSample{
      timestamp_s > 0.0 ? timestamp_s : UnixSecondsNow(),
      localized,
      std::max(0.0, position_error_m),
      std::max(0.0, covariance_trace),
      std::max(0.0, std::min(1.0, quality)),
      source});
  return "{\"action\":\"ingest_localization_health\",\"success\":true,\"map_id\":" +
      JsonString(resolved) + ",\"health\":" + HealthJson(*record) + "}";
}

std::string MapsServiceCore::IngestPlanningOutcomeJson(
    const std::string& map_id,
    double timestamp_s,
    bool success,
    const std::string& planner,
    const std::string& reason) {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  const auto record = store_.GetMapRecord(resolved);
  if (!record.has_value()) {
    return FailureJson("ingest_planning_outcome", "map not found: " + resolved, "map_not_found");
  }
  HealthModelFor(*record)->IngestPlanningOutcome(health::PlanningOutcome{
      timestamp_s > 0.0 ? timestamp_s : UnixSecondsNow(), success, planner, reason});
  return "{\"action\":\"ingest_planning_outcome\",\"success\":true,\"map_id\":" +
      JsonString(resolved) + ",\"health\":" + HealthJson(*record) + "}";
}

std::string MapsServiceCore::IngestCollisionEventJson(
    const std::string& map_id,
    double timestamp_s,
    double severity,
    const std::string& source,
    const std::string& reason) {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  const auto record = store_.GetMapRecord(resolved);
  if (!record.has_value()) {
    return FailureJson("ingest_collision_event", "map not found: " + resolved, "map_not_found");
  }
  HealthModelFor(*record)->IngestCollisionEvent(health::CollisionEvent{
      timestamp_s > 0.0 ? timestamp_s : UnixSecondsNow(),
      std::max(0.0, severity),
      source,
      reason});
  return "{\"action\":\"ingest_collision_event\",\"success\":true,\"map_id\":" +
      JsonString(resolved) + ",\"health\":" + HealthJson(*record) + "}";
}

std::string MapsServiceCore::GetBundleJson(
    const std::string& map_id,
    const std::string& capability) const {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("get_map_bundle", "missing map name", "missing_map_name");
  }
  if (capability.empty()) {
    return FailureJson("get_map_bundle", "missing capability", "missing_capability");
  }
  try {
    const auto record = store_.GetMapRecord(resolved);
    if (!record.has_value()) {
      return FailureJson("get_map_bundle", "map not found: " + resolved, "map_not_found");
    }
    const auto* artifact = FindArtifactForCapability(*record, capability);
    if (artifact == nullptr) {
      return "{"
          "\"action\":\"get_map_bundle\","
          "\"success\":false,"
          "\"schema_version\":\"map.bundle\","
          "\"reason_code\":\"missing_capability\","
          "\"map_id\":" + JsonString(resolved) + ","
          "\"version_id\":" + JsonString(record->lineage_id + ":v" + std::to_string(record->version)) + ","
          "\"state\":" + JsonString(StateName(record->state)) + ","
          "\"capability\":" + JsonString(capability) + ","
          "\"message\":" + JsonString("capability unavailable: " + capability) + ","
          "\"available_capabilities\":" + CapabilitiesJson(*record) + ","
          "\"health\":" + HealthJson(*record) +
          "}";
    }
    const auto dir = store_.ContentPath(resolved);
    return "{"
        "\"action\":\"get_map_bundle\","
        "\"success\":true,"
        "\"schema_version\":\"map.bundle\","
        "\"map_id\":" + JsonString(record->map_id) + ","
        "\"version_id\":" + JsonString(record->lineage_id + ":v" + std::to_string(record->version)) + ","
        "\"state\":" + JsonString(StateName(record->state)) + ","
        "\"frame_id\":" + JsonString(record->scope.frame_id) + ","
        "\"map_dir\":" + JsonString(dir.string()) + ","
        "\"capability\":" + JsonString(capability) + ","
        "\"artifact\":" + ArtifactJson(*artifact) + ","
        "\"artifacts\":" + ArtifactsJson(*record) + ","
        "\"available_capabilities\":" + CapabilitiesJson(*record) + ","
        "\"health\":" + HealthJson(*record) + ","
        "\"record\":" + RecordJson(*record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("get_map_bundle", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::GetMapPointsJson(
    const std::string& map_id,
    std::uint64_t max_points) const {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("get_map_points", "missing map name", "missing_map_name");
  }
  try {
    const std::string id = MapStore::NormalizeMapId(resolved);
    auto map_lock = MapLock::TryAcquire(store_.RootDir(), id, "get-map-points");
    if (!map_lock.has_value()) {
      return FailureJson(
          "get_map_points",
          "map write in progress: " + id,
          "map_write_in_progress");
    }
    const auto map_dir = store_.MapPath(id);
    if (BuildRunningForMap(map_dir)) {
      return FailureJson(
          "get_map_points",
          "map build is running: " + id,
          "map_build_in_progress");
    }
    const auto record = store_.GetMapRecord(id);
    if (!record.has_value()) {
      return FailureJson("get_map_points", "map not found: " + id, "map_not_found");
    }
    const auto pcd_path = store_.ContentPath(id) / "map.pcd";
    if (!std::filesystem::is_regular_file(pcd_path)) {
      return FailureJson(
          "get_map_points",
          "map.pcd not found: " + pcd_path.string(),
          "map_pcd_not_found");
    }
    const auto pointcloud = std::find_if(
        record->artifacts.begin(),
        record->artifacts.end(),
        [](const auto& artifact) { return artifact.type == ArtifactType::kPointCloud; });
    if (pointcloud == record->artifacts.end() || pointcloud->sha256.size() != 64U) {
      return FailureJson(
          "get_map_points",
          "map.pcd sha256 is unavailable: " + id,
          "map_pcd_hash_unavailable");
    }
    const std::string map_pcd_sha256 = pointcloud->sha256;
    auto loaded = LoadPcdXyz(pcd_path);
    if (!loaded.ok) {
      return FailureJson("get_map_points", loaded.message, "map_pcd_unreadable");
    }
    if (Sha256File(pcd_path) != map_pcd_sha256) {
      return FailureJson(
          "get_map_points",
          "map.pcd changed while points were being read: " + id,
          "map_changed_during_read");
    }
    if (BuildRunningForMap(map_dir)) {
      return FailureJson(
          "get_map_points",
          "map build is running: " + id,
          "map_build_in_progress");
    }
    const auto total = loaded.points.size();
    const std::uint64_t limit =
        max_points == 0U ? static_cast<std::uint64_t>(total) : max_points;
    const std::uint64_t stride = total <= limit || limit == 0U
        ? 1U
        : static_cast<std::uint64_t>((total + static_cast<size_t>(limit) - 1U) / limit);
    std::ostringstream points;
    points << std::setprecision(7) << "[";
    size_t emitted = 0U;
    for (size_t i = 0U; i < total; i += static_cast<size_t>(std::max<std::uint64_t>(stride, 1U))) {
      if (limit != 0U && emitted >= limit) {
        break;
      }
      const auto& point = loaded.points[i];
      if (emitted > 0U) {
        points << ",";
      }
      points << "[" << point.x << "," << point.y << "," << point.z << "]";
      ++emitted;
    }
    points << "]";
    return "{"
        "\"action\":\"get_map_points\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"version_id\":" +
            JsonString(record->lineage_id + ":v" + std::to_string(record->version)) + ","
        "\"frame_id\":" + JsonString(record->scope.frame_id) + ","
        "\"map_pcd_sha256\":" + JsonString(map_pcd_sha256) + ","
        "\"pcd\":" + JsonString(pcd_path.string()) + ","
        "\"point_count\":" + std::to_string(total) + ","
        "\"returned\":" + std::to_string(emitted) + ","
        "\"stride\":" + std::to_string(stride) + ","
        "\"points\":" + points.str() +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("get_map_points", exc.what(), "invalid_map_name");
  }
}

std::filesystem::path MapsServiceCore::PoiPath(const std::string& map_id) const {
  return store_.MapPath(map_id) / "pois.tsv";
}

std::filesystem::path MapsServiceCore::GraphPath() const {
  return store_.RootDir() / "map_graph.ltg";
}

std::filesystem::path MapsServiceCore::ActiveSlotsPath() const {
  return store_.RootDir() / "active_slots.lts";
}

std::filesystem::path MapsServiceCore::ActiveHistoryPath() const {
  return store_.RootDir() / "active_history.tsv";
}

std::filesystem::path MapsServiceCore::BuildQueuePath() const {
  return store_.RootDir() / "build_queue.tsv";
}

void MapsServiceCore::AppendActiveHistory(
    const std::string& old_active,
    const std::string& new_active) const {
  if (old_active == new_active) {
    return;
  }
  std::ofstream file(ActiveHistoryPath(), std::ios::app);
  if (file) {
    file << old_active << "\t" << new_active << "\n";
  }
}

std::string MapsServiceCore::ListPoiJson(const std::string& map_id) const {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("poi_list", "missing map name", "missing_map_name");
  }
  try {
    if (!store_.GetMapRecord(resolved).has_value()) {
      return FailureJson("poi_list", "map not found: " + resolved, "map_not_found");
    }
    std::ifstream file(PoiPath(resolved));
    std::ostringstream pois;
    pois << "{";
    bool first = true;
    std::string line;
    while (std::getline(file, line)) {
      const auto fields = SplitTab(line);
      if (fields.size() < 7U) {
        continue;
      }
      if (!first) {
        pois << ",";
      }
      first = false;
      const bool has_yaw = fields[5] == "1";
      pois << JsonString(fields[0]) << ":{"
           << "\"frame_id\":" << JsonString(fields[1]) << ","
           << "\"x\":" << fields[2] << ","
           << "\"y\":" << fields[3] << ","
           << "\"z\":" << fields[4] << ","
           << "\"yaw\":" << (has_yaw ? fields[6] : "null") << ","
           << "\"tags\":" << (fields.size() >= 8U ? JsonObjectOrEmpty(fields[7]) : "{}")
           << "}";
    }
    pois << "}";
    return "{"
        "\"action\":\"poi_list\","
        "\"success\":true,"
        "\"schema_version\":\"map.poi.v1\","
        "\"map_id\":" + JsonString(resolved) + ","
        "\"pois\":" + pois.str() +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("poi_list", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::SetPoiJson(
    const std::string& map_id,
    const std::string& name,
    double x_m,
    double y_m,
    double z_m,
    double yaw_rad,
    bool has_yaw,
    const std::string& frame_id,
    const std::string& tags_json) {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("poi_set", "missing map name", "missing_map_name");
  }
  if (name.empty() || !SafeTsvField(name) || !SafeTsvField(frame_id) ||
      !SafeTsvField(tags_json)) {
    return FailureJson("poi_set", "invalid POI field", "invalid_poi");
  }
  try {
    if (!store_.GetMapRecord(resolved).has_value()) {
      return FailureJson("poi_set", "map not found: " + resolved, "map_not_found");
    }
    std::vector<std::string> kept;
    {
      std::ifstream in(PoiPath(resolved));
      std::string line;
      while (std::getline(in, line)) {
        const auto fields = SplitTab(line);
        if (fields.empty() || fields[0] != name) {
          kept.push_back(line);
        }
      }
    }
    std::filesystem::create_directories(store_.MapPath(resolved));
    std::ofstream out(PoiPath(resolved), std::ios::trunc);
    if (!out) {
      return FailureJson("poi_set", "failed to write POI store", "poi_store_write_failed");
    }
    for (const auto& line : kept) {
      out << line << "\n";
    }
    out << name << "\t" << (frame_id.empty() ? "map" : frame_id) << "\t"
        << std::setprecision(12) << x_m << "\t" << y_m << "\t" << z_m << "\t"
        << (has_yaw ? "1" : "0") << "\t" << yaw_rad << "\t"
        << JsonObjectOrEmpty(tags_json) << "\n";
    return "{"
        "\"action\":\"poi_set\","
        "\"success\":true,"
        "\"schema_version\":\"map.poi.v1\","
        "\"map_id\":" + JsonString(resolved) + ","
        "\"name\":" + JsonString(name) + ","
        "\"poi\":{"
        "\"frame_id\":" + JsonString(frame_id.empty() ? "map" : frame_id) + ","
        "\"x\":" + std::to_string(x_m) + ","
        "\"y\":" + std::to_string(y_m) + ","
        "\"z\":" + std::to_string(z_m) + ","
        "\"yaw\":" + (has_yaw ? std::to_string(yaw_rad) : "null") + ","
        "\"tags\":" + JsonObjectOrEmpty(tags_json) +
        "}}";
  } catch (const std::exception& exc) {
    return FailureJson("poi_set", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::DeletePoiJson(
    const std::string& map_id,
    const std::string& name) {
  const std::string resolved = map_id.empty() ? store_.ActiveMapId() : map_id;
  if (resolved.empty()) {
    return FailureJson("poi_delete", "missing map name", "missing_map_name");
  }
  try {
    std::vector<std::string> kept;
    bool removed = false;
    std::ifstream in(PoiPath(resolved));
    std::string line;
    while (std::getline(in, line)) {
      const auto fields = SplitTab(line);
      if (!fields.empty() && fields[0] == name) {
        removed = true;
      } else {
        kept.push_back(line);
      }
    }
    if (!removed) {
      return FailureJson("poi_delete", "POI not found: " + name, "poi_not_found");
    }
    std::ofstream out(PoiPath(resolved), std::ios::trunc);
    for (const auto& item : kept) {
      out << item << "\n";
    }
    return "{"
        "\"action\":\"poi_delete\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(resolved) + ","
        "\"name\":" + JsonString(name) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("poi_delete", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::ListMapGraphJson() const {
  std::ifstream file(GraphPath(), std::ios::binary);
  std::unordered_map<std::string, std::string> node_maps;
  struct EdgeView {
    std::string from;
    std::string to;
    std::string type;
    bool bidirectional{false};
  };
  std::vector<EdgeView> views;
  std::string line;
  if (std::getline(file, line) && line == "LINGTU_MAP_GRAPH 1") {
    while (std::getline(file, line)) {
      const std::size_t sep = line.find(' ');
      if (sep == std::string::npos) {
        continue;
      }
      const std::string kind = line.substr(0, sep);
      const auto fields = ParseGraphFields(line.substr(sep + 1U));
      if (kind == "node") {
        const auto id = fields.find("id");
        const auto map = fields.find("map");
        if (id != fields.end() && map != fields.end()) {
          node_maps[id->second] = map->second;
        }
      } else if (kind == "edge") {
        const auto enabled = fields.find("enabled");
        if (enabled != fields.end() && enabled->second == "0") {
          continue;
        }
        const auto from = fields.find("from");
        const auto to = fields.find("to");
        if (from == fields.end() || to == fields.end()) {
          continue;
        }
        std::string edge_type = "link";
        const auto transition = fields.find("transition");
        if (transition != fields.end()) {
          std::size_t start = 0U;
          while (start <= transition->second.size()) {
            const std::size_t end = transition->second.find(',', start);
            const std::string item = transition->second.substr(
                start,
                end == std::string::npos ? std::string::npos : end - start);
            const std::size_t meta_sep = item.find('=');
            if (meta_sep != std::string::npos &&
                DecodeGraphValue(item.substr(0, meta_sep)) == "type") {
              edge_type = DecodeGraphValue(item.substr(meta_sep + 1U));
              break;
            }
            if (end == std::string::npos) break;
            start = end + 1U;
          }
        }
        views.push_back(EdgeView{
            from->second,
            to->second,
            edge_type,
            fields.count("direction") != 0 && fields.at("direction") == "bidirectional"});
      }
    }
  }
  std::ostringstream edges;
  edges << "[";
  for (std::size_t i = 0; i < views.size(); ++i) {
    if (i != 0U) {
      edges << ",";
    }
    const auto from = node_maps.count(views[i].from) != 0
        ? node_maps[views[i].from]
        : MapIdFromGraphNodeId(views[i].from);
    const auto to = node_maps.count(views[i].to) != 0
        ? node_maps[views[i].to]
        : MapIdFromGraphNodeId(views[i].to);
    edges << "{"
          << "\"from\":" << JsonString(from) << ","
          << "\"to\":" << JsonString(to) << ","
          << "\"type\":" << JsonString(views[i].type) << ","
          << "\"bidirectional\":" << BoolJson(views[i].bidirectional)
          << "}";
  }
  edges << "]";
  return "{"
      "\"action\":\"map_graph\","
      "\"success\":true,"
      "\"schema_version\":\"map.graph.v1\","
      "\"storage\":\"map_graph.ltg\","
      "\"edges\":" + edges.str() +
      "}";
}

std::string MapsServiceCore::SetMapEdgeJson(
    const std::string& from_map_id,
    const std::string& to_map_id,
    const std::string& edge_type,
    bool bidirectional) {
  try {
    const std::string from = MapStore::NormalizeMapId(from_map_id);
    const std::string to = MapStore::NormalizeMapId(to_map_id);
    if (!store_.GetMapRecord(from).has_value() || !store_.GetMapRecord(to).has_value()) {
      return FailureJson("map_edge_set", "map edge endpoint not found", "map_not_found");
    }
    if (edge_type.find('\n') != std::string::npos || edge_type.find('\r') != std::string::npos) {
      return FailureJson("map_edge_set", "invalid edge type", "invalid_edge");
    }
    MapGraph graph([this](const std::string& id) {
      return MapStore::IsValidMapId(id) && store_.GetMapRecord(id).has_value();
    });
    if (std::filesystem::is_regular_file(GraphPath())) {
      const auto loaded = graph.Load(GraphPath());
      if (!loaded.ok && loaded.corrupt) {
        return FailureJson("map_edge_set", loaded.message, "graph_corrupt");
      }
    }
    MapGraphNode from_node;
    from_node.node_id = GraphNodeId(from);
    from_node.map_id = from;
    from_node.type = MapGraphNodeType::kPortal;
    from_node.label = from;
    from_node.metadata["map_id"] = from;
    if (graph.FindNode(from_node.node_id) == nullptr) {
      const auto added = graph.AddNode(std::move(from_node));
      if (!added.ok) {
        return FailureJson("map_edge_set", added.message, "invalid_edge");
      }
    }
    MapGraphNode to_node;
    to_node.node_id = GraphNodeId(to);
    to_node.map_id = to;
    to_node.type = MapGraphNodeType::kPortal;
    to_node.label = to;
    to_node.metadata["map_id"] = to;
    if (graph.FindNode(to_node.node_id) == nullptr) {
      const auto added = graph.AddNode(std::move(to_node));
      if (!added.ok) {
        return FailureJson("map_edge_set", added.message, "invalid_edge");
      }
    }
    MapGraphEdge edge;
    edge.edge_id = GraphEdgeId(from, to);
    edge.from_node_id = GraphNodeId(from);
    edge.to_node_id = GraphNodeId(to);
    edge.direction = bidirectional
        ? MapGraphEdgeDirection::kBidirectional
        : MapGraphEdgeDirection::kForward;
    edge.transition_metadata["type"] = edge_type.empty() ? "link" : edge_type;
    const auto added = graph.AddEdge(std::move(edge));
    if (!added.ok) {
      const auto enabled = graph.SetEdgeEnabled(GraphEdgeId(from, to), true);
      if (!enabled.ok) {
        return FailureJson("map_edge_set", added.message, "invalid_edge");
      }
    }
    const auto saved = graph.Save(GraphPath());
    if (!saved.ok) {
      return FailureJson("map_edge_set", saved.message, "graph_store_write_failed");
    }
    return "{"
        "\"action\":\"map_edge_set\","
        "\"success\":true,"
        "\"schema_version\":\"map.graph.v1\","
        "\"storage\":\"map_graph.ltg\","
        "\"edge\":{\"from\":" + JsonString(from) + ",\"to\":" + JsonString(to) +
        ",\"type\":" + JsonString(edge_type.empty() ? "link" : edge_type) +
        ",\"bidirectional\":" + BoolJson(bidirectional) + "}"
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("map_edge_set", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::DeleteMapEdgeJson(
    const std::string& from_map_id,
    const std::string& to_map_id) {
  try {
    const std::string from = MapStore::NormalizeMapId(from_map_id);
    const std::string to = MapStore::NormalizeMapId(to_map_id);
    MapGraph graph([this](const std::string& id) {
      return MapStore::IsValidMapId(id) && store_.GetMapRecord(id).has_value();
    });
    const auto loaded = graph.Load(GraphPath());
    if (!loaded.ok) {
      return FailureJson(
          "map_edge_delete",
          loaded.message,
          loaded.corrupt ? "graph_corrupt" : "edge_not_found");
    }
    const auto disabled = graph.SetEdgeEnabled(GraphEdgeId(from, to), false);
    if (!disabled.ok) {
      return FailureJson("map_edge_delete", "map edge not found", "edge_not_found");
    }
    const auto saved = graph.Save(GraphPath());
    if (!saved.ok) {
      return FailureJson("map_edge_delete", saved.message, "graph_store_write_failed");
    }
    return "{"
        "\"action\":\"map_edge_delete\","
        "\"success\":true,"
        "\"from\":" + JsonString(from) + ","
        "\"to\":" + JsonString(to) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("map_edge_delete", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::ShortestRouteJson(
    const std::string& start_map_id,
    const std::string& goal_map_id) const {
  try {
    const std::string start = MapStore::NormalizeMapId(start_map_id);
    const std::string goal = MapStore::NormalizeMapId(goal_map_id);
    MapGraph graph([this](const std::string& id) {
      return MapStore::IsValidMapId(id) && store_.GetMapRecord(id).has_value();
    });
    const auto loaded = graph.Load(GraphPath());
    if (!loaded.ok) {
      return FailureJson(
          "shortest_route",
          loaded.message,
          loaded.corrupt ? "graph_corrupt" : "route_not_found");
    }
    const auto route = graph.ShortestRoute(GraphNodeId(start), GraphNodeId(goal));
    std::ostringstream nodes;
    nodes << "[";
    for (std::size_t i = 0; i < route.node_ids.size(); ++i) {
      if (i != 0U) nodes << ",";
      nodes << JsonString(MapIdFromGraphNodeId(route.node_ids[i]));
    }
    nodes << "]";
    std::ostringstream edges;
    edges << "[";
    for (std::size_t i = 0; i < route.edge_ids.size(); ++i) {
      if (i != 0U) edges << ",";
      edges << JsonString(route.edge_ids[i]);
    }
    edges << "]";
    std::ostringstream transitions;
    transitions << "[";
    for (std::size_t i = 0; i < route.transitions.size(); ++i) {
      if (i != 0U) transitions << ",";
      const auto& transition = route.transitions[i];
      transitions << "{"
                  << "\"edge_id\":" << JsonString(transition.edge_id) << ","
                  << "\"from_map_id\":" << JsonString(transition.from_map_id) << ","
                  << "\"to_map_id\":" << JsonString(transition.to_map_id)
                  << "}";
    }
    transitions << "]";
    return "{"
        "\"action\":\"shortest_route\","
        "\"success\":" + BoolJson(route.found) + ","
        "\"schema_version\":\"map.graph.route.v1\","
        "\"from\":" + JsonString(start) + ","
        "\"to\":" + JsonString(goal) + ","
        "\"found\":" + BoolJson(route.found) + ","
        "\"total_cost\":" + std::to_string(route.total_cost) + ","
        "\"nodes\":" + nodes.str() + ","
        "\"edges\":" + edges.str() + ","
        "\"transitions\":" + transitions.str() +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("shortest_route", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::RollbackActiveMapJson() {
  std::ifstream in(ActiveHistoryPath());
  std::string line;
  std::string previous;
  while (std::getline(in, line)) {
    const auto fields = SplitTab(line);
    if (fields.size() >= 2U && !fields[0].empty() && fields[1] == store_.ActiveMapId()) {
      previous = fields[0];
    }
  }
  if (previous.empty()) {
    return FailureJson("rollback_active", "no previous active map", "active_rollback_unavailable");
  }
  const std::string current = store_.ActiveMapId();
  auto result = store_.SetActiveMap(previous, false);
  if (!result.ok) {
    return FailureJson("rollback_active", result.message, "active_rollback_failed");
  }
  (void)active_slots_.Set("navigation", previous);
  AppendActiveHistory(current, previous);
  return "{"
      "\"action\":\"rollback_active\","
      "\"success\":true,"
      "\"previous_active\":" + JsonString(current) + ","
      "\"active\":" + JsonString(previous) + ","
      "\"record\":" + RecordJson(*result.record) +
      "}";
}

std::string MapsServiceCore::ActiveSlotEntryJson(const ActiveSlotEntry& entry) const {
  return "{"
      "\"slot\":" + JsonString(entry.slot) + "," +
      "\"map_id\":" + JsonString(entry.map_id) + "," +
      "\"active\":" + JsonString(entry.map_id) +
      "}";
}

std::string MapsServiceCore::ListActiveSlotsJson() const {
  std::ostringstream slots;
  slots << "[";
  const auto entries = active_slots_.List();
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (i != 0U) slots << ",";
    slots << ActiveSlotEntryJson(entries[i]);
  }
  slots << "]";
  return "{"
      "\"action\":\"list_active_slots\","
      "\"success\":true,"
      "\"schema_version\":\"map.active_slots.v1\","
      "\"primary\":\"navigation\","
      "\"active\":" + JsonString(store_.ActiveMapId()) + ","
      "\"slots\":" + slots.str() +
      "}";
}

std::string MapsServiceCore::GetActiveSlotJson(const std::string& slot) const {
  if (!ActiveSlots::IsValidSlotName(slot)) {
    return FailureJson("get_active_slot", "invalid active slot: " + slot, "invalid_slot");
  }
  const ActiveSlotEntry entry{slot, active_slots_.Get(slot)};
  return "{"
      "\"action\":\"get_active_slot\","
      "\"success\":true,"
      "\"schema_version\":\"map.active_slots.v1\","
      "\"slot\":" + ActiveSlotEntryJson(entry) +
      "}";
}

std::string MapsServiceCore::SetActiveSlotJson(
    const std::string& slot,
    const std::string& map_id,
    bool strict) {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    const auto record = store_.GetMapRecord(id);
    if (!record.has_value()) {
      return FailureJson("set_active_slot", "map not found: " + id, "map_not_found");
    }
    if (record->state == MapState::kRetired) {
      return FailureJson("set_active_slot", "map is retired: " + id, "map_retired");
    }
    if (slot == "navigation") {
      const auto dir = store_.MapPath(id);
      if (BuildRunningForMap(dir)) {
        return FailureJson(
            "set_active_slot",
            "map build is running: " + id,
            "map_build_in_progress");
      }
      const std::string previous = store_.ActiveMapId();
      auto active = store_.SetActiveMap(id, strict);
      if (!active.ok) {
        return FailureJson("set_active_slot", active.message, "artifact_gate_failed");
      }
      AppendActiveHistory(previous, id);
    } else if (strict) {
      const auto probe = store_.SetActiveMap(id, true);
      if (!probe.ok) {
        return FailureJson("set_active_slot", probe.message, "artifact_gate_failed");
      }
      const std::string primary = active_slots_.Get("navigation");
      if (!primary.empty()) {
        (void)store_.SetActiveMap(primary, false);
      } else {
        store_.ClearActiveMap();
      }
    }
    const auto result = active_slots_.Set(slot, id);
    if (!result.ok || !result.entry.has_value()) {
      return FailureJson("set_active_slot", result.message, "active_slot_write_failed");
    }
    return "{"
        "\"action\":\"set_active_slot\","
        "\"success\":true,"
        "\"schema_version\":\"map.active_slots.v1\","
        "\"slot\":" + ActiveSlotEntryJson(*result.entry) + ","
        "\"active\":" + JsonString(store_.ActiveMapId()) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("set_active_slot", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::ClearActiveSlotJson(const std::string& slot) {
  const auto result = active_slots_.Clear(slot);
  if (!result.ok || !result.entry.has_value()) {
    return FailureJson("clear_active_slot", result.message, "invalid_slot");
  }
  if (slot == "navigation") {
    store_.ClearActiveMap();
  }
  return "{"
      "\"action\":\"clear_active_slot\","
      "\"success\":true,"
      "\"schema_version\":\"map.active_slots.v1\","
      "\"slot\":" + ActiveSlotEntryJson(*result.entry) + ","
      "\"active\":" + JsonString(store_.ActiveMapId()) +
      "}";
}

std::string MapsServiceCore::GetBuildQueueJson() const {
  std::ostringstream items;
  items << "[";
  const auto statuses = artifact_jobs_.ListStatuses(100U);
  for (std::size_t i = 0; i < statuses.size(); ++i) {
    if (i != 0U) {
      items << ",";
    }
    items << ArtifactJobStatusJson(statuses[i]);
  }
  items << "]";
  return "{"
      "\"action\":\"build_queue\","
      "\"success\":true,"
      "\"schema_version\":\"map.artifact_jobs.v2\","
      "\"worker_owned\":true,"
      "\"items\":" + items.str() +
      "}";
}

std::string MapsServiceCore::EnqueueBuildJson(
    const std::string& map_id,
    const std::string& artifact_type) {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    if (!store_.GetMapRecord(id).has_value()) {
      return FailureJson("enqueue_build", "map not found: " + id, "map_not_found");
    }
    if (artifact_type.empty() || !SafeTsvField(artifact_type)) {
      return FailureJson("enqueue_build", "missing artifact type", "missing_artifact_type");
    }
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    ArtifactJobRequest request;
    request.request_id = id + "-" + artifact_type + "-" + std::to_string(stamp);
    request.map_id = id;
    request.artifact_type = artifact_type;
    const auto submitted = artifact_jobs_.Submit(request);
    if (!submitted.accepted) {
      return FailureJson(
          "enqueue_build",
          submitted.status.message.empty() ? submitted.reason_code : submitted.status.message,
          submitted.reason_code.empty() ? "queue_rejected" : submitted.reason_code);
    }
    return "{"
        "\"action\":\"enqueue_build\","
        "\"success\":true,"
        "\"job\":" + ArtifactJobStatusJson(submitted.status) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("enqueue_build", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::PopBuildQueueJson() {
  return FailureJson(
      "pop_build_queue",
      "artifact queue is owned and consumed by the native worker",
      "worker_owned_queue");
}

std::string MapsServiceCore::GetArtifactJobJson(const std::string& request_id) const {
  const auto status = artifact_jobs_.GetStatus(request_id);
  if (!status.has_value()) {
    return FailureJson("get_artifact_job", "artifact job not found", "job_not_found");
  }
  return "{\"action\":\"get_artifact_job\",\"success\":true,\"job\":" +
      ArtifactJobStatusJson(*status) + "}";
}

std::string MapsServiceCore::CancelArtifactJobJson(const std::string& request_id) {
  const auto result = artifact_jobs_.Cancel(request_id);
  if (!result.accepted) {
    return FailureJson("cancel_artifact_job", result.status.message, result.reason_code);
  }
  return "{\"action\":\"cancel_artifact_job\",\"success\":true,\"job\":" +
      ArtifactJobStatusJson(result.status) + "}";
}

std::string MapsServiceCore::RetryArtifactJobJson(const std::string& request_id) {
  const auto result = artifact_jobs_.Retry(request_id);
  if (!result.accepted) {
    return FailureJson("retry_artifact_job", result.status.message, result.reason_code);
  }
  return "{\"action\":\"retry_artifact_job\",\"success\":true,\"job\":" +
      ArtifactJobStatusJson(result.status) + "}";
}

ArtifactBuildResult MapsServiceCore::RunArtifactJob(
    const ArtifactJobRequest& request,
    ArtifactJobContext& context) {
  context.UpdateProgress(0.05, "validating map and artifact request");
  if (!store_.GetMapRecord(request.map_id).has_value()) {
    return {false, false, "map not found: " + request.map_id, "map_not_found", {}};
  }
  std::string kind = request.artifact_type;
  std::transform(kind.begin(), kind.end(), kind.begin(), [](unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  context.UpdateProgress(0.20, "building artifact in native pipeline");
  std::string result;
  if (kind == "occupancy" || kind == "occupancy_2d" || kind == "occupancy_grid") {
    result = pipeline_.BuildOccupancySnapshotJson(request.map_id);
  } else if (kind == "esdf") {
    result = pipeline_.BuildEsdfArtifactJson(request.map_id);
  } else if (kind == "traversability") {
    result = pipeline_.BuildTraversabilityArtifactJson(request.map_id);
  } else if (kind == "semantic") {
    result = BuildSemanticArtifactJson(request.map_id);
  } else if (kind == "octomap" || kind == "octomap_3d") {
    OctomapBuildOptions options;
    const auto mode = request.parameters.find("build_mode");
    if (mode != request.parameters.end()) options.build_mode = mode->second;
    const auto converter = request.parameters.find("converter_command");
    if (converter != request.parameters.end()) options.converter_command = converter->second;
    options.cancel_requested = [&context]() { return context.IsCancelRequested(); };
    result = pipeline_.BuildOctomapArtifactJson(request.map_id, options);
  } else if (kind == "navigation_package") {
    OctomapBuildOptions options;
    options.cancel_requested = [&context]() { return context.IsCancelRequested(); };
    result = pipeline_.BuildNavigationPackageJson(request.map_id, options, true, true);
  } else {
    return {false, false, "unsupported artifact type: " + request.artifact_type,
            "unsupported_artifact_type", {}};
  }
  if (context.IsCancelRequested()) {
    return {false, true, "artifact build cancelled", "cancelled", {}};
  }
  const bool success = result.find("\"success\":true") != std::string::npos;
  if (!success) {
    return {false, true, result, "artifact_build_failed", {}};
  }
  context.UpdateProgress(0.95, "artifact committed and indexed");
  std::vector<std::filesystem::path> artifacts;
  const auto record = store_.GetMapRecord(request.map_id);
  if (record.has_value()) {
    for (const auto& artifact : record->artifacts) {
      artifacts.emplace_back(artifact.uri);
    }
  }
  return {true, false, "artifact build committed", "", artifacts};
}

std::string MapsServiceCore::ArtifactJobStatusJson(const ArtifactJobStatus& status) const {
  std::ostringstream artifacts;
  artifacts << "[";
  for (std::size_t i = 0; i < status.artifacts.size(); ++i) {
    if (i != 0U) artifacts << ",";
    artifacts << JsonString(status.artifacts[i].string());
  }
  artifacts << "]";
  return "{"
      "\"job_id\":" + JsonString(status.job_id) + "," +
      "\"request_id\":" + JsonString(status.request_id) + "," +
      "\"map_id\":" + JsonString(status.map_id) + "," +
      "\"artifact_type\":" + JsonString(status.artifact_type) + "," +
      "\"state\":" + JsonString(ArtifactJobStateName(status.state)) + "," +
      "\"progress\":" + std::to_string(status.progress) + "," +
      "\"attempt\":" + std::to_string(status.attempt) + "," +
      "\"message\":" + JsonString(status.message) + "," +
      "\"reason_code\":" + JsonString(status.reason_code) + "," +
      "\"recovered\":" + BoolJson(status.recovered) + "," +
      "\"replayed\":" + BoolJson(status.replayed) + "," +
      "\"artifacts\":" + artifacts.str() +
      "}";
}

std::string MapsServiceCore::CreateMapJson(const std::string& map_id) {
  try {
    auto result = store_.CreateMap(map_id);
    if (!result.ok) {
      const std::string reason = result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : "map_exists";
      return FailureJson("create", result.message, reason);
    }
    return "{"
        "\"action\":\"create\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(MapStore::NormalizeMapId(map_id)) + ","
        "\"map_dir\":" + JsonString(store_.MapPath(map_id).string()) + ","
        "\"record\":" + RecordJson(*result.record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("create", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::DeleteMapJson(const std::string& map_id) {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    auto result = store_.DeleteMap(id);
    if (!result.ok) {
      const std::string reason = result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : "map_not_found";
      return FailureJson("delete", result.message, reason);
    }
    const auto slots = active_slots_.ClearMapId(id);
    return "{"
        "\"action\":\"delete\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"message\":" + JsonString("deleted: " + id) + ","
        "\"slot_cleanup_ok\":" + BoolJson(slots.ok) + ","
        "\"slot_cleanup_message\":" + JsonString(slots.message) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("delete", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::RenameMapJson(const std::string& map_id, const std::string& new_map_id) {
  try {
    const std::string old_id = MapStore::NormalizeMapId(map_id);
    const std::string new_id = MapStore::NormalizeMapId(new_map_id);
    auto result = store_.RenameMap(old_id, new_id);
    if (!result.ok) {
      const std::string reason = result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : (result.message.find("not found") != std::string::npos
                 ? "map_not_found"
                 : "target_exists");
      return FailureJson("rename", result.message, reason);
    }
    const auto slots = result.record->state == MapState::kRetired
        ? active_slots_.ClearMapId(old_id)
        : active_slots_.ReplaceMapId(old_id, new_id);
    if (!slots.ok) {
      const auto rollback = store_.RenameMap(new_id, old_id);
      return FailureJson(
          "rename",
          rollback.ok
              ? slots.message
              : slots.message + "; map rename rollback failed: " + rollback.message,
          "active_slot_write_failed");
    }
    return "{"
        "\"action\":\"rename\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(new_id) + ","
        "\"old_map_id\":" + JsonString(old_id) + ","
        "\"message\":" + JsonString(old_id + " -> " + new_id) + ","
        "\"record\":" + RecordJson(*result.record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("rename", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::RetireMapJson(const std::string& map_id) {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    auto result = store_.RetireMap(id);
    if (!result.ok) {
      const std::string reason = result.message.find("write in progress") != std::string::npos
          ? "map_write_in_progress"
          : "map_not_found";
      return FailureJson("retire", result.message, reason);
    }
    const auto slots = active_slots_.ClearMapId(id);
    return "{"
        "\"action\":\"retire\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"message\":" + JsonString("retired: " + id) + ","
        "\"slot_cleanup_ok\":" + BoolJson(slots.ok) + ","
        "\"slot_cleanup_message\":" + JsonString(slots.message) + ","
        "\"record\":" + RecordJson(*result.record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("retire", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::SetActiveMapJson(const std::string& map_id, bool strict) {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    const auto dir = store_.MapPath(id);
    if (BuildRunningForMap(dir)) {
      return FailureJson(
          "set_active",
          "map build is running: " + id,
          "map_build_in_progress");
    }
    const std::string previous_active = store_.ActiveMapId();
    auto result = store_.SetActiveMap(id, strict);
    if (!result.ok) {
      const std::string reason = result.message.find("retired") != std::string::npos
          ? "map_retired"
          : "artifact_gate_failed";
      return FailureJson("set_active", result.message, reason);
    }
    const auto slot_result = active_slots_.Set("navigation", id);
    if (!slot_result.ok) {
      if (previous_active.empty()) {
        store_.ClearActiveMap();
      } else {
        (void)store_.SetActiveMap(previous_active, false);
      }
      return FailureJson("set_active", slot_result.message, "active_slot_write_failed");
    }
    AppendActiveHistory(previous_active, id);
    const auto content = store_.ContentPath(id);
    return "{"
        "\"action\":\"set_active\","
        "\"success\":true,"
        "\"active\":" + JsonString(id) + ","
        "\"previous_active\":" + JsonString(previous_active) + ","
        "\"octomap\":" + ExistingPathJson(std::filesystem::is_regular_file(content / "octomap.ot") ? content / "octomap.ot" : content / "octomap.bt") + ","
        "\"occupancy\":" + ExistingPathJson(content / "occupancy.npz") + ","
        "\"record\":" + RecordJson(*result.record) +
        "}";
  } catch (const std::exception& exc) {
    return FailureJson("set_active", exc.what(), "invalid_map_name");
  }
}

std::string MapsServiceCore::ClearActiveMapJson() {
  const auto slot_result = active_slots_.Clear("navigation");
  if (!slot_result.ok) {
    return FailureJson("clear_active", slot_result.message, "active_slot_write_failed");
  }
  store_.ClearActiveMap();
  return "{\"action\":\"clear_active\",\"success\":true,\"active\":\"\"}";
}

std::string MapsServiceCore::BeginBuildJson(
    const std::string& map_id,
    const std::string& artifact_type) {
  return pipeline_.BeginBuildJson(map_id, artifact_type);
}

std::string MapsServiceCore::FinishBuildJson(
    const std::string& map_id,
    const std::string& build_id,
    bool success,
    const std::string& message) {
  return pipeline_.FinishBuildJson(map_id, build_id, success, message);
}

std::string MapsServiceCore::GetBuildStatusJson(const std::string& map_id) const {
  return pipeline_.GetBuildStatusJson(map_id);
}

std::string MapsServiceCore::ImportPcdJson(
    const std::string& map_id,
    const std::filesystem::path& source_path,
    double voxel_size,
    const PcdBounds& bounds) {
  return pipeline_.ImportPcdJson(map_id, source_path, voxel_size, bounds);
}

std::string MapsServiceCore::CommitSavedSourceJson(
    const std::string& map_id,
    const std::filesystem::path& source_dir,
    const SourceCommitOptions& options) {
  return pipeline_.CommitSavedSourceJson(map_id, source_dir, options);
}

std::string MapsServiceCore::CropPcdJson(
    const std::string& map_id,
    const PcdBounds& bounds,
    bool invert,
    double voxel_size) {
  return pipeline_.CropPcdJson(map_id, bounds, invert, voxel_size);
}

std::string MapsServiceCore::RestoreSourceBackupJson(const std::string& map_id) {
  return pipeline_.RestoreSourceBackupJson(map_id);
}

std::string MapsServiceCore::BuildOccupancySnapshotJson(const std::string& map_id) {
  return pipeline_.BuildOccupancySnapshotJson(map_id);
}

std::string MapsServiceCore::BuildOctomapArtifactJson(
    const std::string& map_id,
    const OctomapBuildOptions& options) {
  return pipeline_.BuildOctomapArtifactJson(map_id, options);
}

std::string MapsServiceCore::GetVoxelEditsJson(const std::string& map_id) const {
  return pipeline_.GetVoxelEditsJson(map_id);
}

std::string MapsServiceCore::EditOctomapVoxelsJson(
    const std::string& map_id,
    const OctomapEditOptions& options) {
  return pipeline_.EditOctomapVoxelsJson(map_id, options);
}

std::string MapsServiceCore::BuildNavigationPackageJson(
    const std::string& map_id,
    const OctomapBuildOptions& options,
    bool include_esdf,
    bool include_traversability) {
  return pipeline_.BuildNavigationPackageJson(
      map_id,
      options,
      include_esdf,
      include_traversability);
}

std::string MapsServiceCore::BuildEsdfArtifactJson(const std::string& map_id) {
  return pipeline_.BuildEsdfArtifactJson(map_id);
}

std::string MapsServiceCore::BuildTraversabilityArtifactJson(const std::string& map_id) {
  return pipeline_.BuildTraversabilityArtifactJson(map_id);
}

std::string MapsServiceCore::BuildSemanticArtifactJson(const std::string& map_id) {
  return pipeline_.BuildSemanticArtifactJson(map_id);
}

std::string MapsServiceCore::BeginSaveMapJson(const SaveMapRequest& request) {
  return save_map_.BeginJson(request);
}

std::string MapsServiceCore::ProvideSaveMapSnapshotJson(
    const std::string& job_id,
    const MapSnapshot& snapshot) {
  return save_map_.ProvideSnapshotJson(job_id, snapshot);
}

std::string MapsServiceCore::GetSaveMapStatusJson(const std::string& job_id) const {
  return save_map_.GetStatusJson(job_id);
}

std::string MapsServiceCore::ListSaveMapJobsJson(std::size_t limit) const {
  return save_map_.ListStatusesJson(limit);
}

std::string MapsServiceCore::CancelSaveMapJson(const std::string& job_id) {
  return save_map_.CancelJson(job_id);
}

std::string MapsServiceCore::RetrySaveMapJson(const std::string& job_id) {
  return save_map_.RetryJson(job_id);
}

std::string MapsServiceCore::ListMapVersionsJson(const std::string& map_id) {
  return save_map_.ListVersionsJson(map_id);
}

std::string MapsServiceCore::RollbackMapVersionJson(
    const std::string& map_id,
    std::int64_t version) {
  return save_map_.RollbackVersionJson(map_id, version);
}

std::string MapsServiceCore::AuditMapVersionsJson(bool dry_run) const {
  return MaintenanceReportJson(
      "audit_map_versions",
      maintenance_.AuditImmutableVersions(dry_run));
}

std::string MapsServiceCore::QuarantineCorruptVersionsJson(bool dry_run) {
  return MaintenanceReportJson(
      "quarantine_corrupt_versions",
      maintenance_.QuarantineCorruptVersions(dry_run));
}

std::string MapsServiceCore::GarbageCollectVersionsJson(bool dry_run) {
  return MaintenanceReportJson(
      "garbage_collect_versions",
      maintenance_.GarbageCollectVersions(dry_run));
}

std::string MapsServiceCore::ExportMapVersionJson(
    const std::string& map_id,
    std::int64_t version,
    const std::filesystem::path& package_dir,
    bool dry_run) const {
  const auto export_root = ExchangeRoot(store_.RootDir(), "LINGTU_MAP_EXPORT_DIR", "export");
  if (!PathWithin(package_dir, export_root)) {
    return FailureJson(
        "export_map_version",
        "package path escapes configured map export root: " + export_root.string(),
        "unsafe_exchange_path");
  }
  return MaintenanceReportJson(
      "export_map_version",
      maintenance_.ExportMapVersion(map_id, version, package_dir, dry_run));
}

std::string MapsServiceCore::ImportMapPackageJson(
    const std::filesystem::path& package_dir,
    const std::string& requested_map_id,
    bool dry_run) {
  const auto import_root = ExchangeRoot(store_.RootDir(), "LINGTU_MAP_IMPORT_DIR", "import");
  if (!PathWithin(package_dir, import_root) || !std::filesystem::is_directory(package_dir)) {
    return FailureJson(
        "import_map_package",
        "package path must be an existing directory under: " + import_root.string(),
        "unsafe_exchange_path");
  }
  const auto imported = maintenance_.ImportMapPackage(package_dir, requested_map_id, dry_run);
  std::string payload = MaintenanceReportJson("import_map_package", imported.report);
  if (payload.size() >= 1U && payload.back() == '}') {
    payload.pop_back();
    payload += ",\"map_id\":" + JsonString(imported.map_id) +
        ",\"version\":" + std::to_string(imported.version) + "}";
  }
  return payload;
}

std::string MapsServiceCore::MigrateMapSchemasJson(bool dry_run) {
  return MaintenanceReportJson(
      "migrate_map_schemas",
      maintenance_.MigrateSchemas(dry_run));
}

std::string MapsServiceCore::MaintenanceReportJson(
    const std::string& action,
    const MaintenanceReport& report) const {
  std::ostringstream issues;
  issues << "[";
  for (std::size_t i = 0; i < report.issues.size(); ++i) {
    if (i != 0U) issues << ",";
    const auto& issue = report.issues[i];
    issues << "{"
           << "\"code\":" << JsonString(issue.code) << ","
           << "\"map_id\":" << JsonString(issue.map_id) << ","
           << "\"version\":" << issue.version << ","
           << "\"path\":" << JsonString(issue.path.string()) << ","
           << "\"message\":" << JsonString(issue.message)
           << "}";
  }
  issues << "]";
  std::ostringstream changes;
  changes << "[";
  for (std::size_t i = 0; i < report.changes.size(); ++i) {
    if (i != 0U) changes << ",";
    const auto& change = report.changes[i];
    changes << "{"
            << "\"map_id\":" << JsonString(change.map_id) << ","
            << "\"version\":" << change.version << ","
            << "\"path\":" << JsonString(change.path.string()) << ","
            << "\"target_path\":" << JsonString(change.target_path.string()) << ","
            << "\"message\":" << JsonString(change.message)
            << "}";
  }
  changes << "]";
  return "{"
      "\"action\":" + JsonString(action) + "," +
      "\"success\":" + BoolJson(report.ok) + "," +
      "\"schema_version\":\"map.maintenance.v1\"," +
      "\"dry_run\":" + BoolJson(report.dry_run) + "," +
      "\"issue_count\":" + std::to_string(report.issues.size()) + "," +
      "\"change_count\":" + std::to_string(report.changes.size()) + "," +
      "\"issues\":" + issues.str() + "," +
      "\"changes\":" + changes.str() +
      "}";
}

std::string MapsServiceCore::RecordJson(const MapRecord& record) const {
  const auto dir = store_.ContentPath(record.map_id);
  std::ostringstream out;
  out
      << "{"
      << "\"schema_version\":\"map.record\","
      << "\"map_id\":" << JsonString(record.map_id) << ","
      << "\"lineage_id\":" << JsonString(record.lineage_id) << ","
      << "\"version\":" << record.version << ","
      << "\"version_id\":" << JsonString(record.lineage_id + ":v" + std::to_string(record.version)) << ","
      << "\"state\":" << JsonString(StateName(record.state)) << ","
      << "\"scope\":{"
      << "\"map_dir\":" << JsonString(dir.string()) << ","
      << "\"frame_id\":" << JsonString(record.scope.frame_id)
      << "},"
      << "\"artifacts\":" << ArtifactsJson(record) << ","
      << "\"metadata\":{},"
      << "\"health\":" << HealthJson(record) << ","
      << "\"lifecycle\":{"
      << "\"state\":" << JsonString(StateName(record.state))
      << "},"
      << "\"capabilities\":" << CapabilitiesJson(record)
      << "}";
  return out.str();
}

std::string MapsServiceCore::ArtifactJson(const MapArtifact& artifact) const {
  const std::string capability = ConfigValue(artifact.build_config, "capability");
  const std::string map_class = ConfigValue(artifact.build_config, "map_class");
  return "{"
      "\"type\":" + JsonString(ArtifactTypeName(artifact.type)) + ","
      "\"uri\":" + JsonString(artifact.uri) + ","
      "\"hash\":" + JsonString(artifact.sha256) + ","
      "\"source_map_id\":" + JsonString(artifact.source_map_id) + ","
      "\"generator\":" + JsonString(artifact.generator) + ","
      "\"name\":" + JsonString(ConfigValue(artifact.build_config, "name")) + ","
      "\"map_class\":" + JsonString(map_class) + ","
      "\"capability\":" + JsonString(capability) + ","
      "\"role\":" + JsonString(ConfigValue(artifact.build_config, "role")) + ","
      "\"build_config\":{}"
      "}";
}

std::string MapsServiceCore::ArtifactsJson(const MapRecord& record) const {
  std::ostringstream artifacts;
  artifacts << "[";
  for (size_t i = 0U; i < record.artifacts.size(); ++i) {
    if (i > 0U) {
      artifacts << ",";
    }
    artifacts << ArtifactJson(record.artifacts[i]);
  }
  artifacts << "]";
  return artifacts.str();
}

std::string MapsServiceCore::CapabilitiesJson(const MapRecord& record) const {
  std::set<std::string> capabilities;
  for (const auto& artifact : record.artifacts) {
    const std::string capability = ConfigValue(artifact.build_config, "capability");
    if (!capability.empty()) {
      capabilities.insert(capability);
    }
  }
  std::ostringstream caps;
  caps << "[";
  size_t cap_index = 0U;
  for (const auto& capability : capabilities) {
    if (cap_index++ > 0U) {
      caps << ",";
    }
    caps << JsonString(capability);
  }
  caps << "]";
  return caps.str();
}

std::string MapsServiceCore::HealthJson(const MapRecord& record) const {
  const auto snapshot = HealthModelFor(record)->Snapshot(UnixSecondsNow());
  const bool has_planning = HasPlanningArtifact(record);
  const bool localization_blocks = snapshot.localization.state == health::MetricState::kOk &&
      snapshot.localization.score < 0.35;
  const bool collision_blocks = snapshot.collision.state == health::MetricState::kOk &&
      snapshot.collision.score < 0.35;
  const bool artifacts_block = snapshot.artifact_validation.state == health::MetricState::kOk &&
      snapshot.artifact_validation.score < 0.999;
  const bool active_allowed = has_planning && !localization_blocks && !collision_blocks && !artifacts_block;
  std::vector<std::string> blockers;
  if (!has_planning) blockers.push_back("no_runtime_planning_artifact");
  if (localization_blocks) blockers.push_back("localization_health_below_threshold");
  if (collision_blocks) blockers.push_back("collision_health_below_threshold");
  if (artifacts_block) blockers.push_back("artifact_validation_failed");
  std::ostringstream out;
  out << "{"
      << "\"schema_version\":\"map.health.v2\","
      << "\"state\":" << JsonString(health::ToString(snapshot.overall.state)) << ","
      << "\"localization_stability\":"
      << (snapshot.localization.state == health::MetricState::kOk ? std::to_string(snapshot.localization.score) : "null") << ","
      << "\"planning_success_rate\":"
      << (snapshot.planning.state == health::MetricState::kOk ? std::to_string(snapshot.planning.score) : "null") << ","
      << "\"collision_score\":"
      << (snapshot.collision.state == health::MetricState::kOk ? std::to_string(snapshot.collision.score) : "null") << ","
      << "\"map_freshness\":"
      << (snapshot.freshness.state == health::MetricState::kOk ? std::to_string(snapshot.freshness.score) : "null") << ","
      << "\"artifact_validity\":"
      << (snapshot.artifact_validation.state == health::MetricState::kOk ? std::to_string(snapshot.artifact_validation.score) : "null") << ","
      << "\"overall_score\":"
      << (snapshot.overall.state == health::MetricState::kUnknown ? "null" : std::to_string(snapshot.overall.score)) << ","
      << "\"metrics\":{"
      << "\"localization\":" << MetricJson(snapshot.localization) << ","
      << "\"planning\":" << MetricJson(snapshot.planning) << ","
      << "\"collision\":" << MetricJson(snapshot.collision) << ","
      << "\"freshness\":" << MetricJson(snapshot.freshness) << ","
      << "\"artifact_validation\":" << MetricJson(snapshot.artifact_validation)
      << "},"
      << "\"sample_counts\":{"
      << "\"localization\":" << snapshot.localization_sample_count << ","
      << "\"planning\":" << snapshot.planning_outcome_count << ","
      << "\"collision\":" << snapshot.collision_event_count << ","
      << "\"artifact_validation\":" << snapshot.artifact_validation_count
      << "},"
      << "\"active_allowed\":" << BoolJson(active_allowed) << ","
      << "\"blockers\":" << StringArrayJson(blockers)
      << "}";
  return out.str();
}

health::MapHealthModel* MapsServiceCore::HealthModelFor(const MapRecord& record) const {
  std::string artifact_signature;
  for (const auto& artifact : record.artifacts) {
    artifact_signature += std::to_string(static_cast<int>(artifact.type));
    artifact_signature += ":" + artifact.sha256 + ";";
  }
  std::lock_guard<std::mutex> lock(health_mutex_);
  auto& entry = health_models_[record.map_id];
  if (entry.model && entry.version == record.version &&
      entry.artifact_signature == artifact_signature) {
    return entry.model.get();
  }
  entry.version = record.version;
  entry.artifact_signature = artifact_signature;
  entry.model = std::make_unique<health::MapHealthModel>();
  double latest = 0.0;
  std::string source_uri;
  for (const auto& artifact : record.artifacts) {
    const std::filesystem::path path(artifact.uri);
    const double timestamp = FileUnixSeconds(path);
    latest = std::max(latest, timestamp);
    const bool planning = artifact.type == ArtifactType::kOccupancy2D ||
        artifact.type == ArtifactType::kOctomap3D ||
        artifact.type == ArtifactType::kEsdf ||
        artifact.type == ArtifactType::kTraversability;
    const bool required = artifact.type == ArtifactType::kPointCloud || planning;
    const bool valid = std::filesystem::is_regular_file(path) && artifact.sha256.size() == 64U;
    entry.model->IngestArtifactValidation(health::ArtifactValidation{
        timestamp,
        ConfigValue(artifact.build_config, "name"),
        required,
        valid,
        "lingtu_maps_store_sha256",
        valid ? "artifact hash present" : "artifact missing or unverified"});
    if (artifact.type == ArtifactType::kPointCloud) {
      source_uri = artifact.uri;
    }
  }
  if (latest > 0.0) {
    entry.model->SetSourceBuildTimestamps(health::SourceBuildTimestamps{
        latest, 0.0, latest, source_uri, "lingtu_maps_pipeline"});
  }
  return entry.model.get();
}

const MapArtifact* MapsServiceCore::FindArtifactForCapability(
    const MapRecord& record,
    const std::string& capability) const {
  bool known = false;
  const ArtifactType wanted = ArtifactTypeForCapability(capability, &known);
  if (!known) {
    return nullptr;
  }
  for (const auto& artifact : record.artifacts) {
    if (artifact.type == wanted) {
      return &artifact;
    }
  }
  return nullptr;
}

std::string MapsServiceCore::ActiveArtifactsJson(const std::string& map_id) const {
  const auto dir = store_.ContentPath(map_id);
  return "{"
      "\"map_dir\":" + JsonString(dir.string()) + ","
      "\"map_pcd\":" + ExistingPathJson(dir / "map.pcd") + ","
      "\"octomap\":" + ExistingPathJson(std::filesystem::is_regular_file(dir / "octomap.ot") ? dir / "octomap.ot" : dir / "octomap.bt") + ","
      "\"occupancy\":" + ExistingPathJson(dir / "occupancy.npz") + ","
      "\"esdf\":" + ExistingPathJson(dir / "esdf.npz") + ","
      "\"traversability\":" + ExistingPathJson(dir / "traversability.npz") + ","
      "\"metadata\":" + ExistingPathJson(dir / "metadata.json") +
      "}";
}

std::string MapsServiceCore::FailureJson(
    const std::string& action,
    const std::string& message,
    const std::string& reason_code) const {
  return "{"
      "\"action\":" + JsonString(action) + ","
      "\"success\":false,"
      "\"reason_code\":" + JsonString(reason_code) + ","
      "\"message\":" + JsonString(message) +
      "}";
}

}  // namespace lingtu::maps
