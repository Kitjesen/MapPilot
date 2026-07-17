#include "lingtu/maps/c_api/store.h"

#include "lingtu/maps/store.hpp"

#include <algorithm>
#include <cstring>
#include <exception>
#include <memory>
#include <set>
#include <sstream>
#include <string>

using lingtu::maps::MapStore;
using lingtu::maps::MapStoreConfig;

struct LingtuMapsStoreHandle {
  explicit LingtuMapsStoreHandle(MapStoreConfig config) : store(std::move(config)) {}
  MapStore store;
};

namespace {

template <typename Fn>
int32_t Protect(Fn&& fn) {
  try {
    return fn();
  } catch (const std::invalid_argument&) {
    return -4;
  } catch (const std::exception&) {
    return -2;
  } catch (...) {
    return -3;
  }
}

int32_t WriteString(const std::string& value, char* out, uint64_t capacity, uint64_t* out_size) {
  const uint64_t required = static_cast<uint64_t>(value.size() + 1U);
  if (out_size != nullptr) {
    *out_size = required;
  }
  if (out == nullptr || capacity == 0U) {
    return required == 1U ? 0 : 1;
  }
  if (capacity < required) {
    const uint64_t copy_count = capacity - 1U;
    std::memcpy(out, value.data(), static_cast<size_t>(copy_count));
    out[copy_count] = '\0';
    return 1;
  }
  std::memcpy(out, value.c_str(), static_cast<size_t>(required));
  return 0;
}

std::string JoinLines(const std::vector<std::string>& values) {
  std::ostringstream stream;
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      stream << '\n';
    }
    stream << values[i];
  }
  return stream.str();
}

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
          stream << "\\u";
          constexpr char kHex[] = "0123456789abcdef";
          stream << "00" << kHex[(ch >> 4U) & 0x0FU] << kHex[ch & 0x0FU];
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

std::string BoolJson(bool value) {
  return value ? "true" : "false";
}

std::string StringArrayJson(const std::vector<std::string>& values) {
  std::ostringstream out;
  out << "[";
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {
      out << ",";
    }
    out << JsonString(values[index]);
  }
  out << "]";
  return out.str();
}

std::string ValidationArtifactJson(
    const lingtu::maps::ArtifactCheck& artifact,
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
}

std::string ArtifactValidationJson(
    const lingtu::maps::ArtifactValidationResult& gate) {
  if (!gate.map_found) {
    return "{"
        "\"action\":\"validate_artifacts\","
        "\"success\":false,"
        "\"reason_code\":\"map_not_found\","
        "\"message\":" + JsonString("map not found: " + gate.map_id) +
        "}";
  }
  return "{"
      "\"action\":\"validate_artifacts\","
      "\"success\":true,"
      "\"map_id\":" + JsonString(gate.map_id) + ","
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
      "\"map_pcd\":" + ValidationArtifactJson(gate.map_pcd, false) + ","
      "\"octomap\":" + ValidationArtifactJson(gate.octomap, true) + ","
      "\"occupancy_grid\":" + ValidationArtifactJson(gate.occupancy_grid, true) + "},"
      "\"blockers\":" + StringArrayJson(gate.blockers) +
      "}}";
}

std::string StateName(lingtu::maps::MapState state) {
  switch (state) {
    case lingtu::maps::MapState::kDraft:
      return "CREATED";
    case lingtu::maps::MapState::kStale:
      return "STALE";
    case lingtu::maps::MapState::kValidated:
      return "READY";
    case lingtu::maps::MapState::kActive:
      return "ACTIVE";
    case lingtu::maps::MapState::kRetired:
      return "RETIRED";
    case lingtu::maps::MapState::kFailed:
      return "FAILED";
  }
  return "FAILED";
}

std::string ArtifactTypeName(lingtu::maps::ArtifactType type) {
  switch (type) {
    case lingtu::maps::ArtifactType::kPointCloud:
      return "POINTCLOUD";
    case lingtu::maps::ArtifactType::kOccupancy2D:
      return "OCCUPANCY_2D";
    case lingtu::maps::ArtifactType::kOctomap3D:
      return "OCTOMAP_3D";
    case lingtu::maps::ArtifactType::kEsdf:
      return "ESDF";
    case lingtu::maps::ArtifactType::kTraversability:
      return "TRAVERSABILITY";
    case lingtu::maps::ArtifactType::kSemantic:
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

bool ArtifactReadinessAllowed(const std::vector<lingtu::maps::MapArtifact>& artifacts) {
  return std::any_of(artifacts.begin(), artifacts.end(), [](const auto& item) {
    return item.type == lingtu::maps::ArtifactType::kOccupancy2D ||
        item.type == lingtu::maps::ArtifactType::kOctomap3D ||
        item.type == lingtu::maps::ArtifactType::kEsdf ||
        item.type == lingtu::maps::ArtifactType::kTraversability;
  });
}

std::string ArtifactJson(const lingtu::maps::MapArtifact& artifact) {
  const std::string capability = ConfigValue(artifact.build_config, "capability");
  return "{"
      "\"type\":" + JsonString(ArtifactTypeName(artifact.type)) + "," +
      "\"uri\":" + JsonString(artifact.uri) + "," +
      "\"hash\":" + JsonString(artifact.sha256) + "," +
      "\"source_map_id\":" + JsonString(artifact.source_map_id) + "," +
      "\"generator\":" + JsonString(artifact.generator) + "," +
      "\"name\":" + JsonString(ConfigValue(artifact.build_config, "name")) + "," +
      "\"map_class\":" + JsonString(ConfigValue(artifact.build_config, "map_class")) + "," +
      "\"capability\":" + JsonString(capability) + "," +
      "\"role\":" + JsonString(ConfigValue(artifact.build_config, "role")) + "," +
      "\"build_config\":{" +
      "\"name\":" + JsonString(ConfigValue(artifact.build_config, "name")) + "," +
      "\"map_class\":" + JsonString(ConfigValue(artifact.build_config, "map_class")) + "," +
      "\"capability\":" + JsonString(capability) + "," +
      "\"role\":" + JsonString(ConfigValue(artifact.build_config, "role")) +
      "}}";
}

std::string CapabilitiesJson(const lingtu::maps::MapRecord& record) {
  std::set<std::string> capabilities;
  for (const auto& artifact : record.artifacts) {
    const std::string capability = ConfigValue(artifact.build_config, "capability");
    if (!capability.empty()) {
      capabilities.insert(capability);
    }
  }
  std::ostringstream out;
  out << "[";
  size_t index = 0U;
  for (const auto& capability : capabilities) {
    if (index++ > 0U) {
      out << ",";
    }
    out << JsonString(capability);
  }
  out << "]";
  return out.str();
}

std::string RecordJson(const lingtu::maps::MapRecord& record) {
  std::ostringstream artifacts;
  artifacts << "[";
  for (size_t i = 0; i < record.artifacts.size(); ++i) {
    const auto& artifact = record.artifacts[i];
    if (i > 0U) {
      artifacts << ",";
    }
    artifacts << ArtifactJson(artifact);
  }
  artifacts << "]";

  const bool active_allowed = ArtifactReadinessAllowed(record.artifacts);
  std::ostringstream out;
  out
      << "{"
      << "\"schema_version\":\"map.record.native\","
      << "\"map_id\":" << JsonString(record.map_id) << ","
      << "\"lineage_id\":" << JsonString(record.lineage_id) << ","
      << "\"version\":" << record.version << ","
      << "\"version_id\":" << JsonString(record.lineage_id + ":v" + std::to_string(record.version)) << ","
      << "\"state\":" << JsonString(StateName(record.state)) << ","
      << "\"scope\":{"
      << "\"map_dir\":" << JsonString("") << ","
      << "\"frame_id\":" << JsonString(record.scope.frame_id)
      << "},"
      << "\"artifacts\":" << artifacts.str() << ","
      << "\"metadata\":{},"
      << "\"health\":{"
      << "\"localization_stability\":null,"
      << "\"planning_success_rate\":null,"
      << "\"collision_rate\":null,"
      << "\"map_freshness\":null,"
      << "\"overall_score\":null,"
      << "\"status\":\"unknown\","
      << "\"reason_code\":\"not_enough_data\","
      << "\"active_allowed\":" << (active_allowed ? "true" : "false") << ","
      << "\"blockers\":[]"
      << "},"
      << "\"lifecycle\":{"
      << "\"state\":" << JsonString(StateName(record.state))
      << "},"
      << "\"capabilities\":" << CapabilitiesJson(record)
      << "}";
  return out.str();
}

std::string BundleJson(
    MapStore& store,
    const std::string& map_id,
    const std::string& capability) {
  const auto record = store.GetMapRecord(map_id);
  if (!record.has_value()) {
    return "{\"action\":\"get_map_bundle\",\"success\":false,"
        "\"reason_code\":\"map_not_found\",\"message\":" +
        JsonString("map not found: " + map_id) + "}";
  }
  const auto artifact = std::find_if(
      record->artifacts.begin(),
      record->artifacts.end(),
      [&](const auto& candidate) {
        return ConfigValue(candidate.build_config, "capability") == capability;
      });
  if (artifact == record->artifacts.end()) {
    return "{\"action\":\"get_map_bundle\",\"success\":false,"
        "\"schema_version\":\"map.bundle\","
        "\"reason_code\":\"missing_capability\","
        "\"map_id\":" + JsonString(map_id) + ","
        "\"capability\":" + JsonString(capability) + ","
        "\"available_capabilities\":" + CapabilitiesJson(*record) + "}";
  }
  return "{"
      "\"action\":\"get_map_bundle\","
      "\"success\":true,"
      "\"schema_version\":\"map.bundle\","
      "\"map_id\":" + JsonString(record->map_id) + ","
      "\"version_id\":" + JsonString(
          record->lineage_id + ":v" + std::to_string(record->version)) + ","
      "\"state\":" + JsonString(StateName(record->state)) + ","
      "\"frame_id\":" + JsonString(record->scope.frame_id) + ","
      "\"map_dir\":" + JsonString(ConfigValue(record->metadata, "content_dir")) + ","
      "\"capability\":" + JsonString(capability) + ","
      "\"artifact\":" + ArtifactJson(*artifact) + ","
      "\"available_capabilities\":" + CapabilitiesJson(*record) + ","
      "\"record\":" + RecordJson(*record) +
      "}";
}

std::string ListRecordsJson(MapStore& store) {
  std::ostringstream out;
  out << "[";
  const auto ids = store.ListMapIds();
  bool first = true;
  for (const auto& id : ids) {
    const auto record = store.GetMapRecord(id);
    if (!record.has_value()) {
      continue;
    }
    if (!first) {
      out << ",";
    }
    first = false;
    out << RecordJson(*record);
  }
  out << "]";
  return out.str();
}

bool EmptyOrNull(const char* value) {
  return value == nullptr || value[0] == '\0';
}

}  // namespace

extern "C" {

LingtuMapsStoreHandle* lingtu_maps_store_create(
    const char* root_dir,
    const char* active_state_filename) {
  if (EmptyOrNull(root_dir)) {
    return nullptr;
  }
  try {
    MapStoreConfig config;
    config.root_dir = root_dir;
    if (!EmptyOrNull(active_state_filename)) {
      config.active_state_filename = active_state_filename;
    }
    return new LingtuMapsStoreHandle(std::move(config));
  } catch (...) {
    return nullptr;
  }
}

void lingtu_maps_store_destroy(LingtuMapsStoreHandle* handle) {
  delete handle;
}

int32_t lingtu_maps_store_validate_map_id(const char* map_id) {
  if (EmptyOrNull(map_id)) {
    return 0;
  }
  return MapStore::IsValidMapId(map_id) ? 1 : 0;
}

int32_t lingtu_maps_store_list_map_ids(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([&]() {
    return WriteString(JoinLines(handle->store.ListMapIds()), out, capacity, out_size);
  });
}

int32_t lingtu_maps_store_active_map_id(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([&]() {
    return WriteString(handle->store.ActiveMapId(), out, capacity, out_size);
  });
}

int32_t lingtu_maps_store_list_records_json(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([&]() {
    return WriteString(ListRecordsJson(handle->store), out, capacity, out_size);
  });
}

int32_t lingtu_maps_store_record_json(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return Protect([&]() {
    const auto record = handle->store.GetMapRecord(map_id);
    if (!record.has_value()) {
      return 2;
    }
    return WriteString(RecordJson(*record), out, capacity, out_size);
  });
}

int32_t lingtu_maps_store_active_record_json(
    LingtuMapsStoreHandle* handle,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([&]() {
    const auto record = handle->store.GetActiveMap();
    if (!record.has_value()) {
      return 2;
    }
    return WriteString(RecordJson(*record), out, capacity, out_size);
  });
}

int32_t lingtu_maps_store_bundle_json(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    const char* capability,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(capability)) {
    return -1;
  }
  return Protect([&]() {
    return WriteString(
        BundleJson(handle->store, map_id, capability),
        out,
        capacity,
        out_size);
  });
}

int32_t lingtu_maps_store_validate_artifacts_json(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    uint8_t require_octomap,
    uint8_t require_occupancy,
    const char* expected_frame_id,
    char* out,
    uint64_t capacity,
    uint64_t* out_size) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return Protect([&]() {
    lingtu::maps::ArtifactValidationOptions options;
    options.require_octomap = require_octomap != 0U;
    options.require_occupancy = require_occupancy != 0U;
    options.expected_frame_id = EmptyOrNull(expected_frame_id) ? "" : expected_frame_id;
    return WriteString(
        ArtifactValidationJson(handle->store.ValidateArtifacts(map_id, options)),
        out,
        capacity,
        out_size);
  });
}

int32_t lingtu_maps_store_create_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return Protect([&]() {
    return handle->store.CreateMap(map_id).ok ? 0 : 2;
  });
}

int32_t lingtu_maps_store_delete_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return Protect([&]() {
    return handle->store.DeleteMap(map_id).ok ? 0 : 2;
  });
}

int32_t lingtu_maps_store_rename_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    const char* new_map_id) {
  if (handle == nullptr || EmptyOrNull(map_id) || EmptyOrNull(new_map_id)) {
    return -1;
  }
  return Protect([&]() {
    return handle->store.RenameMap(map_id, new_map_id).ok ? 0 : 2;
  });
}

int32_t lingtu_maps_store_set_active_map(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    uint8_t strict) {
  if (handle == nullptr || EmptyOrNull(map_id)) {
    return -1;
  }
  return Protect([&]() {
    return handle->store.SetActiveMap(map_id, strict != 0U).ok ? 0 : 2;
  });
}

int32_t lingtu_maps_store_clear_active_map(LingtuMapsStoreHandle* handle) {
  if (handle == nullptr) {
    return -1;
  }
  return Protect([&]() {
    handle->store.ClearActiveMap();
    return 0;
  });
}

int32_t lingtu_maps_store_artifact_count(
    LingtuMapsStoreHandle* handle,
    const char* map_id,
    uint64_t* out_count) {
  if (handle == nullptr || EmptyOrNull(map_id) || out_count == nullptr) {
    return -1;
  }
  return Protect([&]() {
    auto record = handle->store.GetMapRecord(map_id);
    if (!record.has_value()) {
      *out_count = 0U;
      return 2;
    }
    *out_count = static_cast<uint64_t>(record->artifacts.size());
    return 0;
  });
}

}  // extern "C"
