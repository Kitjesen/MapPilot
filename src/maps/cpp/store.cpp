#include "lingtu/maps/store.hpp"

#include "lingtu/maps/build/grid_artifacts.hpp"
#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/json.hpp"
#include "lingtu/maps/lock.hpp"

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <unordered_map>

#if defined(LINGTU_MAPS_HAS_OCTOMAP)
#  include <octomap/OcTree.h>
#endif

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
constexpr const char* kActiveMapLockId = "__active_map__";
constexpr const char* kContentEpochLockId = "__content_epoch__";
constexpr const char* kDeletedMapsDirectory = ".deleted";
constexpr const char* kContentEpochCounterFilename = ".content_epoch_counter";
constexpr std::int64_t kMaxExactJsonInteger = 9'007'199'254'740'991LL;

bool IsValidPcd(const std::filesystem::path& path) {
  return LoadPcdXyz(path).ok;
}

bool IsValidOccupancy(const std::filesystem::path& path) {
  try {
    const auto occupancy = LoadOccupancyArtifact(path);
    return occupancy.rows > 0 && occupancy.cols > 0 && !occupancy.grid.empty();
  } catch (const std::exception&) {
    return false;
  }
}

bool HasOctomapHeader(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  std::string line;
  if (!std::getline(file, line) || line.rfind("# Octomap OcTree", 0U) != 0U) {
    return false;
  }
  for (int index = 0; index < 8 && std::getline(file, line); ++index) {
    if (line == "data" || line == "data\r") {
      return file.peek() != std::char_traits<char>::eof();
    }
  }
  return false;
}

bool IsValidOctomap(const std::filesystem::path& path) {
#if defined(LINGTU_MAPS_HAS_OCTOMAP)
  octomap::OcTree tree(0.1);
  return tree.readBinary(path.string());
#else
  return HasOctomapHeader(path);
#endif
}

bool IsValidArtifact(const MapArtifact& artifact) {
  switch (artifact.type) {
    case ArtifactType::kPointCloud:
      return IsValidPcd(artifact.uri);
    case ArtifactType::kOccupancy2D:
      return IsValidOccupancy(artifact.uri);
    case ArtifactType::kOctomap3D:
      return IsValidOctomap(artifact.uri);
    default:
      return std::filesystem::is_regular_file(artifact.uri) &&
          std::filesystem::file_size(artifact.uri) > 0U;
  }
}

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

std::filesystem::path UniqueDeletionStage(
    const std::filesystem::path& root_dir,
    const std::string& map_id) {
  const auto trash = root_dir / kDeletedMapsDirectory;
  std::filesystem::create_directories(trash);
  const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
  for (std::uint32_t attempt = 0; attempt < 1024U; ++attempt) {
    const auto candidate = trash /
        (map_id + "-" + std::to_string(stamp) + "-" + std::to_string(attempt));
    if (!std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  throw std::runtime_error("failed to allocate map deletion stage");
}

MapHealth UnknownHealth() {
  MapHealth health;
  return health;
}

enum class LifecycleMarker {
  kMissing,
  kRetired,
  kCorrupt,
};

LifecycleMarker ReadLifecycleMarker(const std::filesystem::path& map_dir) {
  const auto path = map_dir / kLifecycleStateFilename;
  std::error_code status_error;
  if (!std::filesystem::exists(path, status_error)) {
    return status_error ? LifecycleMarker::kCorrupt : LifecycleMarker::kMissing;
  }
  if (!std::filesystem::is_regular_file(path, status_error) || status_error) {
    return LifecycleMarker::kCorrupt;
  }
  std::ifstream file(path, std::ios::binary);
  std::string value;
  if (!file || !std::getline(file, value) || Trim(value) != "RETIRED") {
    return LifecycleMarker::kCorrupt;
  }
  std::string trailing;
  if (std::getline(file, trailing) || file.bad()) {
    return LifecycleMarker::kCorrupt;
  }
  return LifecycleMarker::kRetired;
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
  const auto begin = std::istreambuf_iterator<char>(stream);
  const auto end = std::istreambuf_iterator<char>();
  std::string text(begin, end);
  return stream.bad() ? std::string{} : text;
}


bool IsSafeDeclaredRelativePath(const std::filesystem::path& path) {
  if (path.empty() || path.is_absolute() || path.has_root_name() || path.has_root_directory()) {
    return false;
  }
  for (const auto& part : path) {
    if (part == "..") {
      return false;
    }
  }
  return true;
}

bool IsPathWithinDirectory(const std::filesystem::path& child,
                           const std::filesystem::path& directory) {
  auto child_it = child.begin();
  const auto child_end = child.end();
  auto dir_it = directory.begin();
  const auto dir_end = directory.end();
  for (; dir_it != dir_end; ++dir_it, ++child_it) {
    if (child_it == child_end || *child_it != *dir_it) {
      return false;
    }
  }
  return child_it != child_end;
}

const ArtifactSpec* ArtifactSpecForType(ArtifactType type) {
  for (const auto& spec : kArtifactSpecs) {
    if (spec.type == type) {
      return &spec;
    }
  }
  return nullptr;
}

std::optional<std::int64_t> ReadPersistedContentEpoch(
    const std::filesystem::path& map_dir) {
  const auto path = map_dir / MapStore::ContentEpochFilename();
  std::error_code error;
  if (!std::filesystem::exists(path, error)) {
    return error ? std::optional<std::int64_t>{0} : std::nullopt;
  }
  if (!std::filesystem::is_regular_file(path, error) || error) {
    return 0;
  }
  const std::string text = Trim(ReadSmallTextFile(path));
  std::int64_t value = 0;
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
  if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
      value <= 0 || value > kMaxExactJsonInteger) {
    return 0;
  }
  return value;
}

std::int64_t FileContentEpoch(const std::filesystem::path& map_dir) {
  if (!std::filesystem::is_directory(map_dir)) {
    return 0;
  }
  const auto persisted = ReadPersistedContentEpoch(map_dir);
  return persisted.has_value() ? *persisted : 1;
}

std::filesystem::path CheapContentPath(
    const std::filesystem::path& map_dir,
    std::int64_t* content_epoch) {
  if (content_epoch != nullptr) {
    *content_epoch = FileContentEpoch(map_dir);
  }
  return std::filesystem::is_directory(map_dir) ? map_dir : std::filesystem::path{};
}


std::string MetadataFrameId(const std::filesystem::path& map_dir) {
  const auto metadata_path = map_dir / "metadata.json";
  if (!std::filesystem::is_regular_file(metadata_path)) {
    return {};
  }
  const auto metadata = ReadSmallTextFile(metadata_path);
  const auto frame_id = JsonObjectStringAtPath(metadata, {"frame_id"});
  return frame_id.has_value() ? *frame_id : std::string{};
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
  return MapPath(map_id);
}

std::int64_t MapStore::ContentEpoch(const std::string& map_id) const {
  return FileContentEpoch(MapPath(map_id));
}

std::int64_t MapStore::AllocateContentEpoch() const {
  auto counter_lock = MapLock::TryAcquire(
      root_dir_, kContentEpochLockId, "allocate-content-epoch");
  if (!counter_lock.has_value()) {
    throw std::runtime_error("map content epoch allocator is busy");
  }

  std::int64_t floor = 0;
  const auto counter_path = root_dir_ / kContentEpochCounterFilename;
  std::error_code error;
  if (std::filesystem::exists(counter_path, error)) {
    const std::string text = Trim(ReadSmallTextFile(counter_path));
    const auto parsed = std::from_chars(text.data(), text.data() + text.size(), floor);
    if (text.empty() || parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
        floor <= 0 || floor >= kMaxExactJsonInteger) {
      throw std::runtime_error("map content epoch counter is invalid");
    }
  } else if (error) {
    throw std::runtime_error("map content epoch counter is unreadable");
  }

  for (const auto& entry : std::filesystem::directory_iterator(root_dir_)) {
    if (!entry.is_directory()) {
      continue;
    }
    const std::string name = entry.path().filename().string();
    if (!IsValidMapId(name)) {
      continue;
    }
    floor = std::max(floor, FileContentEpoch(entry.path()));
  }
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  floor = std::max(
      floor,
      std::clamp<std::int64_t>(
          static_cast<std::int64_t>(now_ms), 1, kMaxExactJsonInteger - 1));
  if (floor >= kMaxExactJsonInteger) {
    throw std::runtime_error("map content epoch counter is exhausted");
  }
  const std::int64_t next = floor + 1;
  WriteTextAtomic(counter_path, std::to_string(next) + "\n");
  return next;
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
  const auto build_lock = dir / ".build_lock";
  if (std::filesystem::exists(build_lock)) {
    return std::nullopt;
  }
  const std::int64_t epoch_before = FileContentEpoch(dir);
  if (epoch_before <= 0) {
    return std::nullopt;
  }
  const auto content = ContentPath(id);
  const auto lifecycle = ReadLifecycleMarker(dir);
  MapState state = MapState::kDraft;
  if (!std::filesystem::is_directory(content) || lifecycle == LifecycleMarker::kCorrupt) {
    state = MapState::kFailed;
  } else if (lifecycle == LifecycleMarker::kRetired) {
    state = MapState::kRetired;
  } else if (ActiveMapId() == id) {
    state = MapState::kActive;
  } else if (HasNavigationArtifacts(content)) {
    state = MapState::kValidated;
  } else if (std::filesystem::is_regular_file(content / "map.pcd")) {
    state = MapState::kStale;
  }
  auto record = ScanRecord(id, state);
  if (std::filesystem::exists(build_lock) ||
      FileContentEpoch(dir) != epoch_before ||
      record.content_epoch != epoch_before) {
    return std::nullopt;
  }
  return record;
}

std::optional<MapRecord> MapStore::GetActiveMap() const {
  const std::string active = ActiveMapId();
  if (active.empty()) {
    return std::nullopt;
  }
  return GetMapRecord(active);
}

std::string MapStore::ActiveMapId() const {
  const auto active = ReadActiveMapIdStrict(nullptr);
  return active.has_value() ? *active : std::string{};
}

bool MapStore::ValidateActiveState(std::string* error) const {
  return ReadActiveMapIdStrict(error).has_value();
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
  try {
    std::filesystem::create_directories(dir);
    WriteTextAtomic(
        dir / ContentEpochFilename(),
        std::to_string(AllocateContentEpoch()) + "\n");
    return {true, "created", ScanRecord(id, MapState::kDraft)};
  } catch (const std::exception& exc) {
    std::error_code ignored;
    std::filesystem::remove_all(dir, ignored);
    return {false, exc.what(), std::nullopt};
  }
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
  auto active_lock = MapLock::TryAcquire(root_dir_, kActiveMapLockId, "delete-map-active-state");
  if (!active_lock.has_value()) {
    return {false, "active map write in progress", std::nullopt};
  }
  std::string active_error;
  const auto active = ReadActiveMapIdStrict(&active_error);
  if (!active.has_value()) {
    return {false, active_error, std::nullopt};
  }
  const bool was_active = *active == id;
  try {
    const auto staged = UniqueDeletionStage(root_dir_, id);
    if (was_active) {
      // Clearing first prevents a crash from leaving active_map.txt pointing at
      // a directory that has already been moved or deleted.
      WriteActiveMapId("");
    }
    try {
      std::filesystem::rename(dir, staged);
    } catch (...) {
      if (was_active) {
        try {
          WriteActiveMapId(id);
        } catch (...) {
          return {
              false,
              "map deletion failed and active map restore failed: " + id,
              std::nullopt,
              *active};
        }
      }
      throw;
    }
    std::error_code cleanup_error;
    std::filesystem::remove_all(staged, cleanup_error);
    if (!cleanup_error) {
      std::error_code ignored;
      std::filesystem::remove(root_dir_ / kDeletedMapsDirectory, ignored);
    }
    return {
        true,
        cleanup_error ? "deleted; staged cleanup pending" : "deleted",
        std::nullopt,
        *active};
  } catch (const std::exception& exc) {
    return {false, exc.what(), std::nullopt, *active};
  }
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
  auto active_lock = MapLock::TryAcquire(root_dir_, kActiveMapLockId, "rename-map-active-state");
  if (!active_lock.has_value()) {
    return {false, "active map write in progress", std::nullopt};
  }
  std::string active_error;
  const auto active = ReadActiveMapIdStrict(&active_error);
  if (!active.has_value()) {
    return {false, active_error, std::nullopt};
  }
  const bool was_active = *active == src_id;
  try {
    if (was_active) {
      // Keep every crash-visible intermediate state non-dangling. The active
      // map is briefly empty while the directory identity changes.
      WriteActiveMapId("");
    }
    try {
      std::filesystem::rename(src, dst);
    } catch (...) {
      if (was_active) {
        try {
          WriteActiveMapId(src_id);
        } catch (...) {
          return {
              false,
              "map rename failed and active map restore failed: " + src_id,
              std::nullopt,
              *active};
        }
      }
      throw;
    }
    if (was_active) {
      try {
        WriteActiveMapId(dst_id);
      } catch (const std::exception& exc) {
        std::error_code rollback_error;
        std::filesystem::rename(dst, src, rollback_error);
        if (!rollback_error) {
          try {
            WriteActiveMapId(src_id);
          } catch (...) {
            return {
                false,
                std::string(exc.what()) + "; active map restore failed",
                std::nullopt,
                *active};
          }
          return {
              false,
              std::string(exc.what()) + "; map rename rolled back",
              std::nullopt,
              *active};
        }
        return {
            false,
            std::string(exc.what()) + "; map rename rollback failed: " +
                rollback_error.message(),
            std::nullopt,
            *active};
      }
    }
  } catch (const std::exception& exc) {
    return {false, exc.what(), std::nullopt, *active};
  }
  const auto content = ContentPath(dst_id);
  const auto lifecycle = ReadLifecycleMarker(dst);
  const MapState renamed_state = lifecycle == LifecycleMarker::kCorrupt
      ? MapState::kFailed
      : lifecycle == LifecycleMarker::kRetired
      ? MapState::kRetired
      : was_active
      ? MapState::kActive
      : (HasNavigationArtifacts(content) ? MapState::kValidated
                                         : (std::filesystem::is_regular_file(content / "map.pcd")
                                                ? MapState::kStale
                                                : MapState::kDraft));
  return {
      true,
      "renamed",
      ScanRecord(dst_id, renamed_state),
      *active};
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
  auto active_lock = MapLock::TryAcquire(root_dir_, kActiveMapLockId, "retire-map-active-state");
  if (!active_lock.has_value()) {
    return {false, "active map write in progress", std::nullopt};
  }
  std::string active_error;
  const auto active = ReadActiveMapIdStrict(&active_error);
  if (!active.has_value()) {
    return {false, active_error, std::nullopt};
  }
  const bool was_active = *active == id;
  try {
    if (was_active) {
      WriteActiveMapId("");
    }
    try {
      WriteTextAtomic(dir / kLifecycleStateFilename, "RETIRED\n");
    } catch (...) {
      if (was_active) {
        try {
          WriteActiveMapId(id);
        } catch (...) {
          return {
              false,
              "map retirement failed and active map restore failed: " + id,
              std::nullopt,
              *active};
        }
      }
      throw;
    }
  } catch (const std::exception& exc) {
    return {false, exc.what(), std::nullopt, *active};
  }
  return {
      true,
      "retired",
      ScanRecord(id, MapState::kRetired),
      *active};
}

DeclaredArtifactIdentityResult MapStore::ReadDeclaredArtifactIdentity(
    const std::string& map_id,
    ArtifactType type,
    const std::string& expected_frame_id) const {
  const std::string id = NormalizeMapId(map_id);
  const auto* spec = ArtifactSpecForType(type);
  if (spec == nullptr) {
    return {std::nullopt, "unsupported artifact type"};
  }
  const auto map_dir = root_dir_ / id;
  const auto build_lock = map_dir / ".build_lock";
  if (std::filesystem::exists(build_lock)) {
    return {std::nullopt, "map write in progress: " + id};
  }
  std::int64_t content_epoch = 0;
  const auto content = CheapContentPath(map_dir, &content_epoch);
  if (content.empty()) {
    return {std::nullopt, "map content unavailable: " + id};
  }
  const auto metadata_path = content / "metadata.json";
  if (!std::filesystem::is_regular_file(metadata_path)) {
    return {std::nullopt, "metadata.json is missing"};
  }
  const std::string metadata = ReadSmallTextFile(metadata_path);
  if (metadata.empty() || !IsValidJsonObject(metadata)) {
    return {std::nullopt, "metadata.json is unreadable or invalid"};
  }
  const std::string frame_id =
      JsonObjectStringAtPath(metadata, {"frame_id"}).value_or(std::string{});
  if (frame_id.empty()) {
    return {std::nullopt, "metadata frame_id is missing"};
  }
  if (!expected_frame_id.empty() && frame_id != expected_frame_id) {
    return {std::nullopt, "frame mismatch: expected " + expected_frame_id + ", got " + frame_id};
  }
  const auto declared_path_value =
      JsonObjectStringAtPath(metadata, {"artifacts", spec->name, "path"});
  if (!declared_path_value.has_value()) {
    return {std::nullopt, std::string(spec->name) + " metadata is missing"};
  }
  const std::string& declared_path = *declared_path_value;
  if (declared_path.empty()) {
    return {std::nullopt, std::string(spec->name) + " path is missing"};
  }
  const std::filesystem::path declared_relative_path(declared_path);
  if (!IsSafeDeclaredRelativePath(declared_relative_path)) {
    return {std::nullopt, std::string(spec->name) + " path escapes map content"};
  }
  std::error_code ec;
  const auto canonical_content = std::filesystem::canonical(content, ec);
  if (ec) {
    return {std::nullopt, "map content unavailable: " + id};
  }
  ec.clear();
  const auto artifact_path = std::filesystem::weakly_canonical(content / declared_relative_path, ec);
  if (ec || !IsPathWithinDirectory(artifact_path, canonical_content)) {
    return {std::nullopt, std::string(spec->name) + " path escapes map content"};
  }
  if (!std::filesystem::is_regular_file(artifact_path)) {
    return {std::nullopt, std::string(spec->name) + " artifact is missing"};
  }
  DeclaredArtifactIdentity identity;
  identity.map_id = id;
  identity.content_epoch = content_epoch;
  identity.type = type;
  identity.map_dir = content;
  identity.artifact_path = artifact_path;
  identity.frame_id = frame_id;
  if (!identity.valid()) {
    return {std::nullopt, "declared artifact identity is incomplete"};
  }
  if (std::filesystem::exists(build_lock) || FileContentEpoch(map_dir) != content_epoch) {
    return {std::nullopt, "map content changed while reading: " + id};
  }
  return {std::move(identity), {}};
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

ArtifactValidationResult MapStore::CheckMapActivation(const std::string& map_id) const {
  const std::string id = NormalizeMapId(map_id);
  auto map_lock = MapLock::TryAcquire(root_dir_, id, "check-map-activation");
  if (!map_lock.has_value()) {
    ArtifactValidationResult result;
    result.map_id = id;
    result.map_found = std::filesystem::is_directory(root_dir_ / id);
    result.map_dir = result.map_found ? ContentPath(id) : root_dir_ / id;
    result.content_epoch = result.map_found ? ContentEpoch(id) : 0;
    result.expected_frame_id = "map";
    result.blockers.push_back("map write in progress: " + id);
    return result;
  }
  return CheckMapActivationUnlocked(id);
}

ArtifactValidationResult MapStore::CheckMapActivationWhileLocked(
    const std::string& map_id,
    const MapLock& map_lock) const {
  const std::string id = NormalizeMapId(map_id);
  if (!LockProtectsMap(map_lock, id)) {
    ArtifactValidationResult result;
    result.map_id = id;
    result.expected_frame_id = "map";
    result.blockers.push_back("map lock does not protect map: " + id);
    return result;
  }
  return CheckMapActivationUnlocked(id);
}

ArtifactValidationResult MapStore::CheckMapActivationUnlocked(const std::string& map_id) const {
  ArtifactValidationOptions options;
  options.require_octomap = true;
  options.require_occupancy = false;
  options.expected_frame_id = "map";
  return ValidateArtifactsUnlocked(map_id, options);
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
  result.content_epoch = record->content_epoch;
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
  result.metadata_ok = result.metadata_ok && IsValidJsonObject(metadata);
  result.metadata_identity_ok = result.metadata_ok;
  if (options.validate_metadata_identity) {
    const auto add_metadata_blocker = [&](const std::string& blocker) {
      result.metadata_identity_ok = false;
      result.metadata_blockers.push_back(blocker);
      result.blockers.push_back(blocker);
    };
    if (!result.metadata_ok) {
      result.metadata_identity_ok = false;
      result.metadata_blockers.push_back("metadata.json is missing or invalid");
    } else {
      result.checked_data_source =
          JsonObjectStringAtPath(metadata, {"data_source"}).value_or("");
      result.checked_source_profile =
          JsonObjectStringAtPath(metadata, {"source_profile"}).value_or("");
      if (result.checked_data_source.empty()) {
        add_metadata_blocker("metadata.data_source missing");
      }
      if (result.checked_source_profile.empty()) {
        add_metadata_blocker("metadata.source_profile missing");
      }
      if (!options.expected_data_source.empty() &&
          result.checked_data_source != options.expected_data_source) {
        add_metadata_blocker("metadata.data_source does not match expected data source");
      }
      if (!options.expected_source_profile.empty() &&
          result.checked_source_profile != options.expected_source_profile) {
        add_metadata_blocker("metadata.source_profile does not match expected source profile");
      }
      if (result.checked_frame_id != "map" && result.checked_frame_id != "odom") {
        add_metadata_blocker("metadata.frame_id is not supported");
      }
      if (!options.expected_frame_id.empty() &&
          result.checked_frame_id != options.expected_frame_id) {
        add_metadata_blocker(
            "frame mismatch: expected " + options.expected_frame_id + ", got " +
            result.checked_frame_id);
      }
      for (const char* artifact_name : {"map_pcd", "octomap", "occupancy_grid"}) {
        if (!JsonObjectHasPath(metadata, {"artifacts", artifact_name})) continue;
        const auto artifact_data_source =
            JsonObjectStringAtPath(metadata, {"artifacts", artifact_name, "data_source"});
        const auto artifact_source_profile =
            JsonObjectStringAtPath(metadata, {"artifacts", artifact_name, "source_profile"});
        const auto artifact_frame =
            JsonObjectStringAtPath(metadata, {"artifacts", artifact_name, "frame_id"});
        const std::string prefix = std::string("metadata.artifacts.") + artifact_name;
        if (!artifact_data_source.has_value() || artifact_data_source->empty()) {
          add_metadata_blocker(prefix + ".data_source missing");
        } else if (*artifact_data_source != result.checked_data_source) {
          add_metadata_blocker(prefix + ".data_source does not match metadata.data_source");
        }
        if (!artifact_source_profile.has_value() || artifact_source_profile->empty()) {
          add_metadata_blocker(prefix + ".source_profile missing");
        } else if (*artifact_source_profile != result.checked_source_profile) {
          add_metadata_blocker(prefix + ".source_profile does not match metadata.source_profile");
        }
        if (!artifact_frame.has_value() || artifact_frame->empty()) {
          add_metadata_blocker(prefix + ".frame_id missing");
        } else if (*artifact_frame != result.checked_frame_id) {
          add_metadata_blocker(prefix + ".frame_id does not match metadata.frame_id");
        }
      }
    }
  }
  const auto fill_check = [&](const MapArtifact* artifact, ArtifactCheck* check) {
    if (artifact == nullptr) {
      return;
    }
    check->path = artifact->uri;
    check->exists = std::filesystem::is_regular_file(artifact->uri);
    check->format_ok = check->exists && IsValidArtifact(*artifact);
  };
  fill_check(pointcloud, &result.map_pcd);
  fill_check(octomap, &result.octomap);
  fill_check(occupancy, &result.occupancy_grid);

  if (record->state == MapState::kRetired) result.blockers.push_back("map_is_retired");
  if (record->state == MapState::kFailed) result.blockers.push_back("map_record_failed");
  if (pointcloud == nullptr) result.blockers.push_back("map.pcd is missing");
  if (pointcloud != nullptr && !result.map_pcd.format_ok) {
    result.blockers.push_back("map.pcd is empty or unreadable");
  }
  if (!result.metadata_ok) result.blockers.push_back("metadata.json is missing or invalid");
  if (options.require_octomap && octomap == nullptr) {
    result.blockers.push_back("octomap artifact is missing");
  }
  if (options.require_octomap && octomap != nullptr && !result.octomap.format_ok) {
    result.blockers.push_back("octomap artifact is empty or unreadable");
  }
  if (options.require_occupancy && occupancy == nullptr) {
    result.blockers.push_back("occupancy artifact is missing");
  }
  if (options.require_occupancy && occupancy != nullptr && !result.occupancy_grid.format_ok) {
    result.blockers.push_back("occupancy artifact is empty or unreadable");
  }
  if (!options.validate_metadata_identity && !options.expected_frame_id.empty() &&
      result.checked_frame_id != options.expected_frame_id) {
    result.blockers.push_back(
        "frame mismatch: expected " + options.expected_frame_id + ", got " +
        result.checked_frame_id);
  }
  result.ok = result.blockers.empty();
  return result;
}

MapStoreResult MapStore::SetActiveMap(
    const std::string& map_id,
    bool strict,
    const std::optional<std::string>& expected_active_map_id) {
  const std::string id = NormalizeMapId(map_id);
  auto map_lock = MapLock::TryAcquire(root_dir_, id, "set-active-map");
  if (!map_lock.has_value()) {
    return {false, "map write in progress: " + id, std::nullopt};
  }
  return SetActiveMapUnlocked(id, strict, expected_active_map_id);
}

MapStoreResult MapStore::SetActiveMapWhileLocked(
    const std::string& map_id,
    bool strict,
    const MapLock& map_lock,
    const std::optional<std::string>& expected_active_map_id) {
  const std::string id = NormalizeMapId(map_id);
  if (!LockProtectsMap(map_lock, id)) {
    return {false, "map lock does not protect map: " + id, std::nullopt};
  }
  return SetActiveMapUnlocked(id, strict, expected_active_map_id);
}

MapStoreResult MapStore::SetActiveMapUnlocked(
    const std::string& map_id,
    bool strict,
    const std::optional<std::string>& expected_active_map_id) {
  const std::string id = NormalizeMapId(map_id);
  const auto dir = root_dir_ / id;
  if (!std::filesystem::is_directory(dir)) {
    return {false, "map not found: " + id, std::nullopt};
  }
  const auto lifecycle = ReadLifecycleMarker(dir);
  if (lifecycle == LifecycleMarker::kCorrupt) {
    return {false, "map lifecycle state is corrupt: " + id,
            ScanRecord(id, MapState::kFailed)};
  }
  if (lifecycle == LifecycleMarker::kRetired) {
    return {false, "map is retired: " + id, ScanRecord(id, MapState::kRetired)};
  }
  if (strict) {
    const auto gate = CheckMapActivationUnlocked(id);
    if (!gate.ok) {
      return {
          false,
          "saved map artifact gate failed: " + JoinBlockers(gate.blockers),
          ScanRecord(id, MapState::kFailed)};
    }
  }
  auto active_lock = MapLock::TryAcquire(root_dir_, kActiveMapLockId, "set-active-map-state");
  if (!active_lock.has_value()) {
    return {false, "active map write in progress", std::nullopt};
  }
  std::string active_error;
  const auto active = ReadActiveMapIdStrict(&active_error);
  if (!active.has_value()) {
    return {false, active_error, std::nullopt};
  }
  const std::string previous_active = *active;
  if (expected_active_map_id.has_value() && previous_active != *expected_active_map_id) {
    return {
        false,
        "active map changed: expected " + *expected_active_map_id + ", got " + previous_active,
        std::nullopt,
        previous_active};
  }
  try {
    WriteActiveMapId(id);
  } catch (const std::exception& exc) {
    return {false, exc.what(), std::nullopt, previous_active};
  }
  return {
      true,
      "active map changed",
      ScanRecord(id, MapState::kActive),
      previous_active};
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

MapStoreResult MapStore::ClearActiveMap(
    const std::optional<std::string>& expected_active_map_id) {
  auto active_lock = MapLock::TryAcquire(root_dir_, kActiveMapLockId, "clear-active-map-state");
  if (!active_lock.has_value()) {
    return {false, "active map write in progress", std::nullopt};
  }
  std::string active_error;
  const auto active = ReadActiveMapIdStrict(&active_error);
  if (!active.has_value()) {
    return {false, active_error, std::nullopt};
  }
  const std::string previous_active = *active;
  if (expected_active_map_id.has_value() && previous_active != *expected_active_map_id) {
    return {
        false,
        "active map changed: expected " + *expected_active_map_id + ", got " + previous_active,
        std::nullopt,
        previous_active};
  }
  try {
    WriteActiveMapId("");
  } catch (const std::exception& exc) {
    return {false, exc.what(), std::nullopt, previous_active};
  }
  return {
      true,
      "active map cleared",
      std::nullopt,
      previous_active};
}

MapRecord MapStore::ScanRecord(const std::string& map_id, MapState state) const {
  const auto content = ContentPath(map_id);
  const auto artifacts = ScanArtifacts(content, map_id);
  MapRecord record;
  record.map_id = map_id;
  record.lineage_id = map_id;
  record.content_epoch = ContentEpoch(map_id);
  record.state = state;
  record.scope.frame_id = MetadataFrameId(content);
  record.artifacts = artifacts;
  record.health = UnknownHealth();
  record.metadata["content_dir"] = content.string();
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

bool MapStore::HasNavigationArtifacts(const std::filesystem::path& map_dir) const {
  return std::filesystem::is_regular_file(map_dir / "octomap.ot") ||
      std::filesystem::is_regular_file(map_dir / "octomap.bt") ||
      std::filesystem::is_regular_file(map_dir / "occupancy.npz") ||
      std::filesystem::is_regular_file(map_dir / "esdf.npz") ||
      std::filesystem::is_regular_file(map_dir / "traversability.npz");
}

void MapStore::WriteActiveMapId(const std::string& map_id) const {
  WriteTextAtomic(ActiveStatePath(), map_id + "\n");
}

std::optional<std::string> MapStore::ReadActiveMapIdStrict(std::string* error) const {
  const auto path = ActiveStatePath();
  if (!std::filesystem::exists(path)) {
    return std::string{};
  }
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    if (error != nullptr) {
      *error = "active map state is unreadable";
    }
    return std::nullopt;
  }
  std::string value;
  if (!std::getline(file, value)) {
    if (error != nullptr) {
      *error = "active map state is corrupt";
    }
    return std::nullopt;
  }
  value = Trim(value);
  if (!value.empty() && !IsValidMapId(value)) {
    if (error != nullptr) {
      *error = "active map state is corrupt";
    }
    return std::nullopt;
  }
  std::string trailing;
  while (std::getline(file, trailing)) {
    if (!Trim(trailing).empty()) {
      if (error != nullptr) {
        *error = "active map state is corrupt";
      }
      return std::nullopt;
    }
  }
  if (file.bad()) {
    if (error != nullptr) {
      *error = "active map state is unreadable";
    }
    return std::nullopt;
  }
  return value;
}

}  // namespace lingtu::maps
