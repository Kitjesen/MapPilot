#include "lingtu/maps/service.hpp"

#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/json.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/map_graph.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>
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

std::string BoolJson(bool value) {
  return value ? "true" : "false";
}

bool IsNonEmptyRegularFile(const std::filesystem::path& path) {
  return std::filesystem::is_regular_file(path) && std::filesystem::file_size(path) > 0U;
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
      maintenance_(MapMaintenanceConfig{store_.RootDir()}) {}

SaveMapEngine& MapsServiceCore::SaveEngine() {
  std::call_once(save_engine_once_, [this]() {
    save_engine_ = std::make_unique<SaveMapEngine>(store_);
  });
  return *save_engine_;
}

const SaveMapEngine& MapsServiceCore::SaveEngine() const {
  return const_cast<MapsServiceCore*>(this)->SaveEngine();
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
  constexpr int kSnapshotAttempts = 3;
  for (int attempt = 0; attempt < kSnapshotAttempts; ++attempt) {
    const auto ids = store_.ListMapIds();
    std::string snapshot_active;
    bool have_snapshot_active = false;
    bool retry = false;
    std::ostringstream maps;
    maps << "[";
    bool first = true;
    for (const auto& id : ids) {
      auto map_lock = MapLock::TryAcquire(store_.RootDir(), id, "list-maps-snapshot");
      if (!map_lock.has_value()) {
        retry = true;
        break;
      }
      std::string active_error;
      if (!store_.ValidateActiveState(&active_error)) {
        return FailureJson("list", active_error, "active_map_state_invalid");
      }
      const std::string active = store_.ActiveMapId();
      if (have_snapshot_active && active != snapshot_active) {
        retry = true;
        break;
      }
      snapshot_active = active;
      have_snapshot_active = true;

      const auto record = store_.GetMapRecord(id);
      if (!record.has_value()) {
        continue;
      }
      const auto dir = store_.ContentPath(id);
      const auto pcd_path = dir / "map.pcd";
      const bool has_pcd = IsNonEmptyRegularFile(pcd_path);
      const bool has_occupancy = IsNonEmptyRegularFile(dir / "occupancy.npz");
      const bool has_octomap = IsNonEmptyRegularFile(dir / "octomap.ot");
      const bool has_esdf = IsNonEmptyRegularFile(dir / "esdf.npz");
      const bool has_traversability = IsNonEmptyRegularFile(dir / "traversability.npz");
      const bool activation_ready =
          store_.CheckMapActivationWhileLocked(id, *map_lock).ok;
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
        size_mb =
            static_cast<double>(std::filesystem::file_size(pcd_path)) / (1024.0 * 1024.0);
      }
      const std::string record_json = RecordJson(*record);
      if (!store_.ValidateActiveState(&active_error)) {
        return FailureJson("list", active_error, "active_map_state_invalid");
      }
      if (store_.ActiveMapId() != snapshot_active) {
        retry = true;
        break;
      }
      if (!first) {
        maps << ",";
      }
      first = false;
      maps
          << "{"
          << "\"name\":" << JsonString(id) << ","
          << "\"has_pcd\":" << BoolJson(has_pcd) << ","
          << "\"has_occupancy\":" << BoolJson(has_occupancy) << ","
          << "\"has_octomap\":" << BoolJson(has_octomap) << ","
          << "\"has_esdf\":" << BoolJson(has_esdf) << ","
          << "\"has_traversability\":" << BoolJson(has_traversability) << ","
          << "\"can_activate\":" << BoolJson(activation_ready) << ","
          << "\"is_active\":" << BoolJson(record->state == MapState::kActive) << ","
          << "\"size_mb\":" << size_mb << ","
          << "\"patch_count\":" << patch_count << ","
          << "\"record\":" << record_json << ","
          << "\"state\":" << JsonString(StateName(record->state))
          << "}";
    }
    if (retry) {
      continue;
    }
    std::string active_error;
    if (!store_.ValidateActiveState(&active_error)) {
      return FailureJson("list", active_error, "active_map_state_invalid");
    }
    const std::string final_active = store_.ActiveMapId();
    if ((have_snapshot_active && final_active != snapshot_active) ||
        store_.ListMapIds() != ids) {
      continue;
    }
    snapshot_active = final_active;
    maps << "]";
    return "{"
        "\"action\":\"list\","
        "\"success\":true,"
        "\"maps\":" + maps.str() + ","
        "\"active\":" + JsonString(snapshot_active) +
        "}";
  }
  return FailureJson("list", "map state changed while reading", "map_snapshot_busy");
}

std::string MapsServiceCore::GetRecordJson(const std::string& map_id) const {
  try {
    std::string active_error;
    if (!store_.ValidateActiveState(&active_error)) {
      return FailureJson("get_record", active_error, "active_map_state_invalid");
    }
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
  constexpr int kSnapshotAttempts = 3;
  for (int attempt = 0; attempt < kSnapshotAttempts; ++attempt) {
    std::string active_error;
    if (!store_.ValidateActiveState(&active_error)) {
      return FailureJson("get_active", active_error, "active_map_state_invalid");
    }
    const std::string active = store_.ActiveMapId();
    if (active.empty()) {
      return FailureJson("get_active", "no active map", "map_not_found");
    }
    auto map_lock = MapLock::TryAcquire(store_.RootDir(), active, "get-active-map-snapshot");
    if (!map_lock.has_value()) {
      continue;
    }
    if (!store_.ValidateActiveState(&active_error)) {
      return FailureJson("get_active", active_error, "active_map_state_invalid");
    }
    if (store_.ActiveMapId() != active) {
      continue;
    }
    const auto record = store_.GetMapRecord(active);
    if (!record.has_value()) {
      return FailureJson(
          "get_active", "active map not found: " + active, "map_not_found");
    }
    const std::string artifacts_json = ActiveArtifactsJson(active);
    const std::string record_json = RecordJson(*record);
    if (!store_.ValidateActiveState(&active_error)) {
      return FailureJson("get_active", active_error, "active_map_state_invalid");
    }
    if (store_.ActiveMapId() != active) {
      continue;
    }
    return "{"
        "\"action\":\"get_active\","
        "\"success\":true,"
        "\"active\":" + JsonString(active) + ","
        "\"artifacts\":" + artifacts_json + ","
        "\"record\":" + record_json +
        "}";
  }
  return FailureJson(
      "get_active", "active map changed while reading", "map_snapshot_busy");
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
    const std::string& expected_frame_id,
    const std::string& expected_data_source,
    const std::string& expected_source_profile) const {
  try {
    const std::string id = MapStore::NormalizeMapId(map_id);
    ArtifactValidationOptions options;
    options.require_octomap = require_octomap;
    options.require_occupancy = require_occupancy;
    options.validate_metadata_identity = true;
    options.expected_frame_id = expected_frame_id;
    options.expected_data_source = expected_data_source;
    options.expected_source_profile = expected_source_profile;
    const auto gate = store_.ValidateArtifacts(id, options);
    if (!gate.map_found) {
      return FailureJson("validate_artifacts", "map not found: " + id, "map_not_found");
    }
    const auto artifact_json = [](const ArtifactCheck& artifact) {
      if (artifact.path.empty()) {
        return std::string{"{\"exists\":false,\"format_ok\":false,\"path\":null}"};
      }
      return "{\"exists\":" + BoolJson(artifact.exists) +
          ",\"format_ok\":" + BoolJson(artifact.format_ok) +
          ",\"path\":" + JsonString(artifact.path.string()) + "}";
    };
    return "{"
        "\"action\":\"validate_artifacts\","
        "\"success\":true,"
        "\"map_id\":" + JsonString(id) + ","
        "\"gate\":{"
        "\"schema_version\":\"lingtu.saved_map_artifacts.gate.v2\","
        "\"ok\":" + BoolJson(gate.ok) + ","
        "\"checked_frame_id\":" + JsonString(gate.checked_frame_id) + ","
        "\"expected_frame_id\":" + JsonString(gate.expected_frame_id) + ","
        "\"metadata_ok\":" + BoolJson(gate.metadata_ok) + ","
        "\"checked_required_artifacts\":[\"map_pcd\"" +
            (require_octomap ? ",\"octomap\"" : "") +
            (require_occupancy ? ",\"occupancy_grid\"" : "") + "],"
        "\"checked_allowed_frame_ids\":[\"map\",\"odom\"],"
        "\"checked_expected\":{"
            "\"data_source\":" + JsonString(expected_data_source) + ","
            "\"source_profile\":" + JsonString(expected_source_profile) + ","
            "\"frame_id\":" + JsonString(expected_frame_id) + "},"
        "\"metadata\":{\"exists\":" + BoolJson(gate.metadata_ok) + "},"
        "\"metadata_validation\":{\"ok\":" + BoolJson(gate.metadata_identity_ok) +
            ",\"checked_data_source\":" + JsonString(gate.checked_data_source) +
            ",\"checked_source_profile\":" + JsonString(gate.checked_source_profile) +
            ",\"blockers\":" + StringArrayJson(gate.metadata_blockers) + "},"
        "\"artifacts\":{"
        "\"map_pcd\":" + artifact_json(gate.map_pcd) + ","
        "\"octomap\":" + artifact_json(gate.octomap) + ","
        "\"occupancy_grid\":" +
            artifact_json(gate.occupancy_grid) + "},"
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
          "\"content_epoch\":" + std::to_string(record->content_epoch) + ","
          "\"state\":" + JsonString(StateName(record->state)) + ","
          "\"capability\":" + JsonString(capability) + ","
          "\"message\":" + JsonString("capability unavailable: " + capability) + ","
          "\"available_capabilities\":" + CapabilitiesJson(*record) + ","
          "\"health\":" + HealthJson(*record) +
          "}";
    }
    return "{"
        "\"action\":\"get_map_bundle\","
        "\"success\":true,"
        "\"schema_version\":\"map.bundle\","
        "\"map_id\":" + JsonString(record->map_id) + ","
        "\"content_epoch\":" + std::to_string(record->content_epoch) + ","
        "\"state\":" + JsonString(StateName(record->state)) + ","
        "\"frame_id\":" + JsonString(record->scope.frame_id) + ","
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
    auto loaded = LoadPcdXyz(pcd_path);
    if (!loaded.ok) {
      return FailureJson("get_map_points", loaded.message, "map_pcd_unreadable");
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
        "\"content_epoch\":" + std::to_string(record->content_epoch) + ","
        "\"frame_id\":" + JsonString(record->scope.frame_id) + ","
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

std::string MapsServiceCore::RecordJson(const MapRecord& record) const {
  std::ostringstream out;
  out
      << "{"
      << "\"schema_version\":\"map.record\","
      << "\"map_id\":" << JsonString(record.map_id) << ","
      << "\"lineage_id\":" << JsonString(record.lineage_id) << ","
      << "\"content_epoch\":" << record.content_epoch << ","
      << "\"state\":" << JsonString(StateName(record.state)) << ","
      << "\"scope\":{"
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
  const bool planning_blocks = snapshot.planning.state == health::MetricState::kOk &&
      snapshot.planning.score < 0.35;
  const bool collision_blocks = snapshot.collision.state == health::MetricState::kOk &&
      snapshot.collision.score < 0.35;
  const bool artifacts_block = snapshot.artifact_validation.state == health::MetricState::kOk &&
      snapshot.artifact_validation.score < 0.999;
  const bool activation_ready = store_.CheckMapActivation(record.map_id).ok;
  const bool runtime_health_blocked =
      localization_blocks || planning_blocks || collision_blocks;
  const bool runtime_health_complete =
      snapshot.localization.state == health::MetricState::kOk &&
      snapshot.planning.state == health::MetricState::kOk;
  const char* runtime_health_state = runtime_health_blocked
      ? "blocked"
      : (runtime_health_complete ? "ready" : "unknown");
  std::vector<std::string> blockers;
  if (!has_planning) blockers.push_back("no_runtime_planning_artifact");
  if (localization_blocks) blockers.push_back("localization_health_below_threshold");
  if (planning_blocks) blockers.push_back("planning_health_below_threshold");
  if (collision_blocks) blockers.push_back("collision_health_below_threshold");
  if (artifacts_block) blockers.push_back("artifact_validation_failed");
  std::ostringstream out;
  out << "{"
      << "\"schema_version\":\"map.health.v3\","
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
      << "\"activation_ready\":" << BoolJson(activation_ready) << ","
      << "\"runtime_health_state\":" << JsonString(runtime_health_state) << ","
      << "\"blockers\":" << StringArrayJson(blockers)
      << "}";
  return out.str();
}

health::MapHealthModel* MapsServiceCore::HealthModelFor(const MapRecord& record) const {
  std::string artifact_signature;
  for (const auto& artifact : record.artifacts) {
    artifact_signature += std::to_string(static_cast<int>(artifact.type));
    artifact_signature += ":" + artifact.uri + ";";
  }
  std::lock_guard<std::mutex> lock(health_mutex_);
  auto& entry = health_models_[record.map_id];
  if (entry.model && entry.content_epoch == record.content_epoch &&
      entry.artifact_signature == artifact_signature) {
    return entry.model.get();
  }
  entry.content_epoch = record.content_epoch;
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
    const bool valid = std::filesystem::is_regular_file(path);
    entry.model->IngestArtifactValidation(health::ArtifactValidation{
        timestamp,
        ConfigValue(artifact.build_config, "name"),
        required,
        valid,
        "lingtu_maps_store",
        valid ? "artifact is readable" : "artifact is missing"});
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
  const auto wanted = ArtifactTypeForCapability(capability);
  if (!wanted.has_value()) {
    return nullptr;
  }
  for (const auto& artifact : record.artifacts) {
    if (artifact.type == *wanted) {
      return &artifact;
    }
  }
  return nullptr;
}

std::string MapsServiceCore::ActiveArtifactsJson(const std::string& map_id) const {
  const auto dir = store_.ContentPath(map_id);
  return "{"
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
