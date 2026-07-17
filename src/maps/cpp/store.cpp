#include "lingtu/maps/store.hpp"

#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/version.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <fstream>
#include <stdexcept>
#include <unordered_map>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <fcntl.h>
#  include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

struct ArtifactSpec {
  const char* name;
  const char* filename;
  ArtifactType type;
  const char* capability;
  const char* role;
};

constexpr ArtifactSpec kArtifactSpecs[] = {
    {"map_pcd", "map.pcd", ArtifactType::kPointCloud, "source_pointcloud", "raw_slam_map_source"},
    {"occupancy_grid", "occupancy.npz", ArtifactType::kOccupancy2D, "path_planning_2d", "static_grid_helper"},
    {"octomap", "octomap.ot", ArtifactType::kOctomap3D, "navigation_safety_3d", "octoplanner3d_global_planning"},
    {"octomap", "octomap.bt", ArtifactType::kOctomap3D, "navigation_safety_3d", "octomap_compatibility"},
    {"esdf", "esdf.npz", ArtifactType::kEsdf, "trajectory_optimization", "distance_field"},
    {"traversability", "traversability.npz", ArtifactType::kTraversability, "traversability", "navigation_cost_layer"},
    {"semantic", "semantic_map.bin", ArtifactType::kSemantic, "semantic_query", "semantic_layer"},
};

constexpr const char* kLifecycleStateFilename = "lifecycle_state.txt";

std::string Trim(const std::string& value) {
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch) != 0;
  }).base();
  if (begin >= end) {
    return {};
  }
  return std::string(begin, end);
}

void SyncPath(const std::filesystem::path& path, bool directory) {
#if defined(_WIN32)
  const DWORD flags = directory ? FILE_FLAG_BACKUP_SEMANTICS : FILE_ATTRIBUTE_NORMAL;
  HANDLE handle = CreateFileW(
      path.wstring().c_str(),
      GENERIC_READ,
      FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
      nullptr,
      OPEN_EXISTING,
      flags,
      nullptr);
  if (handle != INVALID_HANDLE_VALUE) {
    FlushFileBuffers(handle);
    CloseHandle(handle);
  }
#else
  const int flags = directory ? (O_RDONLY | O_DIRECTORY) : O_RDONLY;
  const int fd = open(path.c_str(), flags);
  if (fd >= 0) {
    fsync(fd);
    close(fd);
  }
#endif
}

void AtomicReplace(const std::filesystem::path& source, const std::filesystem::path& target) {
#if defined(_WIN32)
  if (MoveFileExW(
          source.wstring().c_str(),
          target.wstring().c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) == 0) {
    throw std::runtime_error("failed to atomically replace active map state");
  }
#else
  if (::rename(source.c_str(), target.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace active map state");
  }
#endif
  SyncPath(target.parent_path(), true);
}

void WriteTextAtomic(const std::filesystem::path& path, const std::string& value) {
  std::filesystem::create_directories(path.parent_path());
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto temp = path.parent_path() /
      (path.filename().string() + ".tmp-" + std::to_string(stamp));
  {
    std::ofstream file(temp, std::ios::binary | std::ios::trunc);
    if (!file) throw std::runtime_error("failed to write active map state");
    file.write(value.data(), static_cast<std::streamsize>(value.size()));
    file.flush();
    if (!file) throw std::runtime_error("failed to flush active map state");
  }
  SyncPath(temp, false);
  AtomicReplace(temp, path);
}

MapHealth UnknownHealth() {
  MapHealth health;
  return health;
}

bool IsRetiredMap(const std::filesystem::path& map_dir) {
  std::ifstream file(map_dir / kLifecycleStateFilename, std::ios::binary);
  if (!file) {
    return false;
  }
  std::string value;
  std::getline(file, value);
  return Trim(value) == "RETIRED";
}

std::size_t JsonValueStart(const std::string& json, const std::string& key) {
  int object_depth = 0;
  int array_depth = 0;
  for (std::size_t index = 0; index < json.size();) {
    const char ch = json[index];
    if (ch == '{') {
      ++object_depth;
      ++index;
      continue;
    }
    if (ch == '}') {
      --object_depth;
      ++index;
      continue;
    }
    if (ch == '[') {
      ++array_depth;
      ++index;
      continue;
    }
    if (ch == ']') {
      --array_depth;
      ++index;
      continue;
    }
    if (ch != '"') {
      ++index;
      continue;
    }

    std::string token;
    bool escaped = false;
    std::size_t cursor = index + 1U;
    for (; cursor < json.size(); ++cursor) {
      const char token_ch = json[cursor];
      if (escaped) {
        escaped = false;
        token.push_back(token_ch);
      } else if (token_ch == '\\') {
        escaped = true;
      } else if (token_ch == '"') {
        break;
      } else {
        token.push_back(token_ch);
      }
    }
    if (cursor >= json.size()) {
      return std::string::npos;
    }
    ++cursor;
    if (object_depth == 1 && array_depth == 0 && token == key) {
      while (cursor < json.size() &&
             std::isspace(static_cast<unsigned char>(json[cursor]))) {
        ++cursor;
      }
      if (cursor < json.size() && json[cursor] == ':') {
        ++cursor;
        while (cursor < json.size() &&
               std::isspace(static_cast<unsigned char>(json[cursor]))) {
          ++cursor;
        }
        return cursor;
      }
    }
    index = cursor;
  }
  return std::string::npos;
}

std::string JsonObjectField(const std::string& json, const std::string& key) {
  const std::size_t start = JsonValueStart(json, key);
  if (start == std::string::npos || start >= json.size() || json[start] != '{') {
    return {};
  }
  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (std::size_t index = start; index < json.size(); ++index) {
    const char ch = json[index];
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        in_string = false;
      }
      continue;
    }
    if (ch == '"') {
      in_string = true;
    } else if (ch == '{') {
      ++depth;
    } else if (ch == '}' && --depth == 0) {
      return json.substr(start, index - start + 1U);
    }
  }
  return {};
}

std::string JsonStringField(const std::string& json, const std::string& key) {
  const std::size_t start = JsonValueStart(json, key);
  if (start == std::string::npos || start >= json.size() || json[start] != '"') {
    return {};
  }
  std::string value;
  bool escaped = false;
  for (std::size_t index = start + 1U; index < json.size(); ++index) {
    const char ch = json[index];
    if (escaped) {
      return {};
    }
    if (ch == '\\') {
      escaped = true;
    } else if (ch == '"') {
      return value;
    } else {
      value.push_back(ch);
    }
  }
  return {};
}

std::string ReadSmallTextFile(const std::filesystem::path& path) {
  std::error_code error;
  const auto size = std::filesystem::file_size(path, error);
  if (error || size > 4U * 1024U * 1024U) {
    return {};
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return {};
  }
  return std::string(
      std::istreambuf_iterator<char>(stream),
      std::istreambuf_iterator<char>());
}

std::string MetadataFrameId(const std::filesystem::path& map_dir) {
  const auto metadata_path = map_dir / "metadata.json";
  if (!std::filesystem::is_regular_file(metadata_path)) {
    return {};
  }
  return JsonStringField(ReadSmallTextFile(metadata_path), "frame_id");
}

std::string JoinBlockers(const std::vector<std::string>& blockers) {
  std::string message;
  for (const auto& blocker : blockers) {
    if (!message.empty()) {
      message += "; ";
    }
    message += blocker;
  }
  return message;
}

}  // namespace

MapStore::MapStore(MapStoreConfig config)
    : config_(std::move(config)), root_dir_(config_.root_dir) {
  if (root_dir_.empty()) {
    throw std::invalid_argument("MapStore root_dir is required");
  }
  std::filesystem::create_directories(root_dir_);
}

bool MapStore::IsValidMapId(const std::string& map_id) {
  const std::string value = Trim(map_id);
  if (value.empty() || value.size() != map_id.size()) {
    return false;
  }
  if (value.front() == '.' || value.front() == '-') {
    return false;
  }
  if (value == "." || value == ".." || value.find("..") != std::string::npos) {
    return false;
  }
  for (const unsigned char ch : value) {
    const bool ok = std::isalnum(ch) != 0 || ch == '_' || ch == '-' || ch == '.';
    if (!ok) {
      return false;
    }
  }
  std::filesystem::path path(value);
  return path.is_relative() && path.filename() == path && path.parent_path().empty();
}

std::string MapStore::NormalizeMapId(const std::string& map_id) {
  const std::string value = Trim(map_id);
  if (!IsValidMapId(value)) {
    throw std::invalid_argument("invalid map id: " + map_id);
  }
  return value;
}

std::filesystem::path MapStore::MapPath(const std::string& map_id) const {
  return root_dir_ / NormalizeMapId(map_id);
}

std::filesystem::path MapStore::ContentPath(const std::string& map_id) const {
  const auto map_dir = MapPath(map_id);
  std::ifstream file(map_dir / "current_version.txt");
  std::string version;
  if (file) {
    std::getline(file, version);
    version = Trim(version);
  }
  if (!version.empty() &&
      std::all_of(version.begin(), version.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0;
      })) {
    const auto content = map_dir / ".versions" / version;
    if (std::filesystem::is_directory(content) && VerifyMapVersion(content)) return content;
    return map_dir / ".versions" / ".invalid-current-version";
  }
  return map_dir;
}

std::int64_t MapStore::CurrentVersion(const std::string& map_id) const {
  const auto content = ContentPath(map_id);
  if (content == MapPath(map_id)) {
    return std::filesystem::is_directory(content) ? 1 : 0;
  }
  try {
    return std::stoll(content.filename().string());
  } catch (...) {
    return 0;
  }
}

std::filesystem::path MapStore::ActiveStatePath() const {
  return root_dir_ / config_.active_state_filename;
}

std::vector<std::string> MapStore::ListMapIds() const {
  std::vector<std::string> out;
  if (!std::filesystem::is_directory(root_dir_)) {
    return out;
  }
  for (const auto& entry : std::filesystem::directory_iterator(root_dir_)) {
    if (!entry.is_directory()) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (name == "active") {
      continue;
    }
    if (IsValidMapId(name)) {
      out.push_back(name);
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

std::optional<MapRecord> MapStore::GetMapRecord(const std::string& map_id) const {
  const std::string id = NormalizeMapId(map_id);
  const auto dir = root_dir_ / id;
  if (!std::filesystem::is_directory(dir)) {
    return std::nullopt;
  }
  const auto content = ContentPath(id);
  MapState state = MapState::kDraft;
  if (!std::filesystem::is_directory(content)) {
    state = MapState::kFailed;
  } else if (IsRetiredMap(dir)) {
    state = MapState::kRetired;
  } else if (ActiveMapId() == id) {
    state = MapState::kActive;
  } else if (HasRuntimePlanningArtifact(content)) {
    state = MapState::kValidated;
  } else if (std::filesystem::is_regular_file(content / "map.pcd")) {
    state = MapState::kStale;
  }
  return ScanRecord(id, state);
}

std::optional<MapRecord> MapStore::GetActiveMap() const {
  const std::string active = ActiveMapId();
  if (active.empty()) {
    return std::nullopt;
  }
  return GetMapRecord(active);
}

std::string MapStore::ActiveMapId() const {
  std::ifstream file(ActiveStatePath());
  if (!file) {
    return {};
  }
  std::string value;
  std::getline(file, value);
  value = Trim(value);
  return IsValidMapId(value) ? value : std::string{};
}

MapStoreResult MapStore::CreateMap(const std::string& map_id) {
  const std::string id = NormalizeMapId(map_id);
  auto map_lock = MapLock::TryAcquire(root_dir_, id, "create-map");
  if (!map_lock.has_value()) {
    return {false, "map write in progress: " + id, std::nullopt};
  }
  const auto dir = root_dir_ / id;
  if (std::filesystem::exists(dir)) {
    return {false, "map exists: " + id, std::nullopt};
  }
  std::filesystem::create_directories(dir);
  return {true, "created", ScanRecord(id, MapState::kDraft)};
}

MapStoreResult MapStore::DeleteMap(const std::string& map_id) {
  const std::string id = NormalizeMapId(map_id);
  auto map_lock = MapLock::TryAcquire(root_dir_, id, "delete-map");
  if (!map_lock.has_value()) {
    return {false, "map write in progress: " + id, std::nullopt};
  }
  const auto dir = root_dir_ / id;
  if (!std::filesystem::is_directory(dir)) {
    return {false, "map not found: " + id, std::nullopt};
  }
  std::filesystem::remove_all(dir);
  if (ActiveMapId() == id) {
    WriteActiveMapId("");
  }
  return {true, "deleted", std::nullopt};
}

MapStoreResult MapStore::RenameMap(const std::string& map_id, const std::string& new_map_id) {
  const std::string src_id = NormalizeMapId(map_id);
  const std::string dst_id = NormalizeMapId(new_map_id);
  auto source_lock = MapLock::TryAcquire(root_dir_, src_id, "rename-map-source");
  if (!source_lock.has_value()) {
    return {false, "map write in progress: " + src_id, std::nullopt};
  }
  auto target_lock = MapLock::TryAcquire(root_dir_, dst_id, "rename-map-target");
  if (!target_lock.has_value()) {
    return {false, "map write in progress: " + dst_id, std::nullopt};
  }
  const auto src = root_dir_ / src_id;
  const auto dst = root_dir_ / dst_id;
  if (!std::filesystem::is_directory(src)) {
    return {false, "map not found: " + src_id, std::nullopt};
  }
  if (std::filesystem::exists(dst)) {
    return {false, "target exists: " + dst_id, std::nullopt};
  }
  const bool was_active = ActiveMapId() == src_id;
  std::filesystem::rename(src, dst);
  if (was_active) {
    WriteActiveMapId(dst_id);
  }
  const auto content = ContentPath(dst_id);
  const MapState renamed_state = IsRetiredMap(dst)
      ? MapState::kRetired
      : was_active
      ? MapState::kActive
      : (HasRuntimePlanningArtifact(content) ? MapState::kValidated
                                         : (std::filesystem::is_regular_file(content / "map.pcd")
                                                ? MapState::kStale
                                                : MapState::kDraft));
  return {true, "renamed", ScanRecord(dst_id, renamed_state)};
}

MapStoreResult MapStore::RetireMap(const std::string& map_id) {
  const std::string id = NormalizeMapId(map_id);
  auto map_lock = MapLock::TryAcquire(root_dir_, id, "retire-map");
  if (!map_lock.has_value()) {
    return {false, "map write in progress: " + id, std::nullopt};
  }
  const auto dir = root_dir_ / id;
  if (!std::filesystem::is_directory(dir)) {
    return {false, "map not found: " + id, std::nullopt};
  }
  if (ActiveMapId() == id) {
    WriteActiveMapId("");
  }
  WriteTextAtomic(dir / kLifecycleStateFilename, "RETIRED\n");
  return {true, "retired", ScanRecord(id, MapState::kRetired)};
}

ArtifactValidationResult MapStore::ValidateArtifacts(
    const std::string& map_id,
    const ArtifactValidationOptions& options) const {
  const std::string id = NormalizeMapId(map_id);
  auto map_lock = MapLock::TryAcquire(root_dir_, id, "validate-artifacts");
  if (!map_lock.has_value()) {
    ArtifactValidationResult result;
    result.map_id = id;
    result.map_found = std::filesystem::is_directory(root_dir_ / id);
    result.map_dir = result.map_found ? ContentPath(id) : root_dir_ / id;
    result.expected_frame_id = options.expected_frame_id;
    result.blockers.push_back("map write in progress: " + id);
    return result;
  }
  return ValidateArtifactsUnlocked(id, options);
}

ArtifactValidationResult MapStore::ValidateArtifactsUnlocked(
    const std::string& map_id,
    const ArtifactValidationOptions& options) const {
  const std::string id = NormalizeMapId(map_id);
  ArtifactValidationResult result;
  result.map_id = id;
  result.expected_frame_id = options.expected_frame_id;
  const auto dir = root_dir_ / id;
  result.map_found = std::filesystem::is_directory(dir);
  result.map_dir = result.map_found ? ContentPath(id) : dir;
  if (!result.map_found) {
    result.blockers.push_back("map not found: " + id);
    return result;
  }

  const auto record = GetMapRecord(id);
  if (!record.has_value()) {
    result.blockers.push_back("map not found: " + id);
    return result;
  }
  const auto content = result.map_dir;
  result.checked_frame_id = MetadataFrameId(content);
  const auto find_artifact = [&](ArtifactType type) -> const MapArtifact* {
    const auto it = std::find_if(
        record->artifacts.begin(),
        record->artifacts.end(),
        [type](const auto& artifact) { return artifact.type == type; });
    return it == record->artifacts.end() ? nullptr : &*it;
  };
  const auto* pointcloud = find_artifact(ArtifactType::kPointCloud);
  const auto* octomap = find_artifact(ArtifactType::kOctomap3D);
  const auto* occupancy = find_artifact(ArtifactType::kOccupancy2D);
  const auto metadata_path = content / "metadata.json";
  result.metadata_ok = std::filesystem::is_regular_file(metadata_path);
  const std::string metadata =
      result.metadata_ok ? ReadSmallTextFile(metadata_path) : std::string{};
  const std::string metadata_artifacts = JsonObjectField(metadata, "artifacts");
  const auto metadata_artifact = [&](const std::string& name) {
    return JsonObjectField(metadata_artifacts, name);
  };
  const auto fill_check = [&](
      const MapArtifact* artifact,
      const std::string& name,
      bool derived,
      ArtifactCheck* check) {
    if (artifact == nullptr) {
      return;
    }
    check->path = artifact->uri;
    check->exists = std::filesystem::is_regular_file(artifact->uri);
    check->sha256 = artifact->sha256;
    const std::string metadata_entry = metadata_artifact(name);
    const std::string expected_sha = JsonStringField(metadata_entry, "sha256");
    check->sha256_ok = check->exists && expected_sha.size() == 64U &&
        expected_sha == check->sha256;
    if (derived) {
      const std::string source_sha = JsonStringField(metadata_entry, "source_map_sha256");
      check->source_map_sha256_matches_map = source_sha.size() == 64U &&
          pointcloud != nullptr && source_sha == pointcloud->sha256;
    }
  };
  fill_check(pointcloud, "map_pcd", false, &result.map_pcd);
  fill_check(octomap, "octomap", true, &result.octomap);
  fill_check(occupancy, "occupancy_grid", true, &result.occupancy_grid);

  if (record->state == MapState::kRetired) result.blockers.push_back("map_is_retired");
  if (record->state == MapState::kFailed) result.blockers.push_back("map_record_failed");
  if (pointcloud == nullptr) result.blockers.push_back("map.pcd is missing");
  if (!result.metadata_ok) result.blockers.push_back("metadata.json is missing");
  if (options.require_runtime_planning_artifact && !HasRuntimePlanningArtifact(content)) {
    result.blockers.push_back("runtime planning artifact is missing");
  }
  if (options.require_octomap && octomap == nullptr) {
    result.blockers.push_back("octomap artifact is missing");
  }
  if (options.require_occupancy && occupancy == nullptr) {
    result.blockers.push_back("occupancy artifact is missing");
  }
  if (pointcloud != nullptr && !result.map_pcd.sha256_ok) {
    result.blockers.push_back("map.pcd sha256 does not match metadata");
  }
  if (octomap != nullptr && !result.octomap.sha256_ok) {
    result.blockers.push_back("octomap sha256 does not match metadata");
  }
  if (octomap != nullptr && !result.octomap.source_map_sha256_matches_map) {
    result.blockers.push_back(
        "octomap source_map_sha256 does not match current map_pcd sha256");
  }
  if (occupancy != nullptr && !result.occupancy_grid.sha256_ok) {
    result.blockers.push_back("occupancy_grid sha256 does not match metadata");
  }
  if (occupancy != nullptr && !result.occupancy_grid.source_map_sha256_matches_map) {
    result.blockers.push_back(
        "occupancy_grid source_map_sha256 does not match current map_pcd sha256");
  }
  if (!options.expected_frame_id.empty() &&
      result.checked_frame_id != options.expected_frame_id) {
    result.blockers.push_back(
        "frame mismatch: expected " + options.expected_frame_id + ", got " +
        result.checked_frame_id);
  }
  if (std::filesystem::is_regular_file(dir / "current_version.txt")) {
    result.version_integrity_ok = VerifyMapVersion(content, &result.version_integrity_message);
    if (!result.version_integrity_ok) {
      result.blockers.push_back(
          "version integrity failed: " + result.version_integrity_message);
    }
  }
  result.ok = result.blockers.empty();
  return result;
}

MapStoreResult MapStore::SetActiveMap(const std::string& map_id, bool strict) {
  const std::string id = NormalizeMapId(map_id);
  auto map_lock = MapLock::TryAcquire(root_dir_, id, "set-active-map");
  if (!map_lock.has_value()) {
    return {false, "map write in progress: " + id, std::nullopt};
  }
  return SetActiveMapUnlocked(id, strict);
}

MapStoreResult MapStore::SetActiveMapWhileLocked(
    const std::string& map_id,
    bool strict,
    const MapLock& map_lock) {
  const std::string id = NormalizeMapId(map_id);
  if (!LockProtectsMap(map_lock, id)) {
    return {false, "map lock does not protect map: " + id, std::nullopt};
  }
  return SetActiveMapUnlocked(id, strict);
}

MapStoreResult MapStore::SetActiveMapUnlocked(const std::string& map_id, bool strict) {
  const std::string id = NormalizeMapId(map_id);
  const auto dir = root_dir_ / id;
  if (!std::filesystem::is_directory(dir)) {
    return {false, "map not found: " + id, std::nullopt};
  }
  if (IsRetiredMap(dir)) {
    return {false, "map is retired: " + id, ScanRecord(id, MapState::kRetired)};
  }
  if (strict) {
    ArtifactValidationOptions options;
    options.require_runtime_planning_artifact = true;
    options.expected_frame_id = "map";
    const auto gate = ValidateArtifactsUnlocked(id, options);
    if (!gate.ok) {
      return {
          false,
          "saved map artifact gate failed: " + JoinBlockers(gate.blockers),
          ScanRecord(id, MapState::kFailed)};
    }
  }
  WriteActiveMapId(id);
  return {true, "active map changed", ScanRecord(id, MapState::kActive)};
}

bool MapStore::LockProtectsMap(const MapLock& map_lock, const std::string& map_id) const {
  std::error_code error;
  const auto expected = std::filesystem::absolute(root_dir_ / ".map_locks" / map_id, error)
                            .lexically_normal();
  if (error) {
    return false;
  }
  const auto actual = std::filesystem::absolute(map_lock.path(), error).lexically_normal();
  return !error && actual == expected && std::filesystem::is_directory(actual);
}

void MapStore::ClearActiveMap() {
  WriteActiveMapId("");
}

MapRecord MapStore::ScanRecord(const std::string& map_id, MapState state) const {
  const auto content = ContentPath(map_id);
  const auto artifacts = ScanArtifacts(content, map_id);
  MapRecord record;
  record.map_id = map_id;
  record.lineage_id = map_id;
  record.version = CurrentVersion(map_id);
  record.state = state;
  record.scope.frame_id = MetadataFrameId(content);
  record.artifacts = artifacts;
  record.health = UnknownHealth();
  record.metadata["content_dir"] = content.string();
  record.metadata["version_id"] = map_id + ":v" + std::to_string(record.version);
  return record;
}

std::vector<MapArtifact> MapStore::ScanArtifacts(
    const std::filesystem::path& map_dir,
    const std::string& map_id) const {
  std::vector<MapArtifact> out;
  for (const auto& spec : kArtifactSpecs) {
    const auto path = map_dir / spec.filename;
    if (!std::filesystem::is_regular_file(path)) {
      continue;
    }
    MapArtifact artifact;
    artifact.type = spec.type;
    artifact.uri = path.string();
    try {
      artifact.sha256 = Sha256File(path);
    } catch (const std::exception&) {
      artifact.sha256.clear();
    }
    artifact.source_map_id = map_id;
    artifact.generator = "lingtu_maps_store";
    artifact.build_config["name"] = spec.name;
    artifact.build_config["capability"] = spec.capability;
    artifact.build_config["role"] = spec.role;
    switch (spec.type) {
      case ArtifactType::kPointCloud:
        artifact.build_config["map_class"] = "saved_point_cloud";
        break;
      case ArtifactType::kOccupancy2D:
        artifact.build_config["map_class"] = "static_2d_occupancy";
        break;
      case ArtifactType::kOctomap3D:
        artifact.build_config["map_class"] = "global_3d_occupancy";
        break;
      case ArtifactType::kEsdf:
        artifact.build_config["map_class"] = "esdf";
        break;
      case ArtifactType::kTraversability:
        artifact.build_config["map_class"] = "traversability";
        break;
      case ArtifactType::kSemantic:
        artifact.build_config["map_class"] = "semantic";
        break;
    }
    out.push_back(std::move(artifact));
  }
  return out;
}

bool MapStore::HasRuntimePlanningArtifact(const std::filesystem::path& map_dir) const {
  return std::filesystem::is_regular_file(map_dir / "octomap.ot") ||
      std::filesystem::is_regular_file(map_dir / "octomap.bt") ||
      std::filesystem::is_regular_file(map_dir / "occupancy.npz") ||
      std::filesystem::is_regular_file(map_dir / "esdf.npz") ||
      std::filesystem::is_regular_file(map_dir / "traversability.npz");
}

void MapStore::WriteActiveMapId(const std::string& map_id) const {
  WriteTextAtomic(ActiveStatePath(), map_id + "\n");
}

}  // namespace lingtu::maps
