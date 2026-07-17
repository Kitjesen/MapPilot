#include "lingtu/maps/save.hpp"

#include "lingtu/maps/build/pcd.hpp"
#include "lingtu/maps/hash.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/semantic_map_persistence.hpp"
#include "lingtu/maps/version.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cctype>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <cerrno>
#  include <csignal>
#  include <fcntl.h>
#  include <sys/types.h>
#  include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

using Clock = std::chrono::system_clock;

constexpr const char* kJobSchemaVersion = "1";
constexpr const char* kJobsDirectoryName = ".save_jobs";
constexpr const char* kVersionsDirectoryName = ".versions";
constexpr const char* kCurrentVersionFilename = "current_version.txt";
constexpr const char* kManifestFilename = "save_manifest.json";
constexpr const char* kManifestHashFilename = "save_manifest.sha256";

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      Clock::now().time_since_epoch()).count();
}

std::uint64_t CurrentProcessId() {
#if defined(_WIN32)
  return static_cast<std::uint64_t>(GetCurrentProcessId());
#else
  return static_cast<std::uint64_t>(getpid());
#endif
}

bool ProcessAlive(std::uint64_t pid) {
  if (pid == 0U) {
    return false;
  }
#if defined(_WIN32)
  HANDLE process = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, static_cast<DWORD>(pid));
  if (process == nullptr) {
    return false;
  }
  DWORD exit_code = 0;
  const bool alive = GetExitCodeProcess(process, &exit_code) != 0 && exit_code == STILL_ACTIVE;
  CloseHandle(process);
  return alive;
#else
  if (kill(static_cast<pid_t>(pid), 0) == 0) {
    return true;
  }
  return errno == EPERM;
#endif
}

std::string JsonEscape(const std::string& value) {
  std::ostringstream out;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (ch < 0x20U) {
          constexpr char kHex[] = "0123456789abcdef";
          out << "\\u00" << kHex[(ch >> 4U) & 0x0FU] << kHex[ch & 0x0FU];
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  return out.str();
}

std::string JsonString(const std::string& value) {
  return "\"" + JsonEscape(value) + "\"";
}

std::string PercentEncode(const std::string& value) {
  constexpr char kHex[] = "0123456789ABCDEF";
  std::string out;
  out.reserve(value.size());
  for (const unsigned char ch : value) {
    const bool safe = std::isalnum(ch) != 0 || ch == '-' || ch == '_' || ch == '.' || ch == '/';
    if (safe) {
      out.push_back(static_cast<char>(ch));
    } else {
      out.push_back('%');
      out.push_back(kHex[(ch >> 4U) & 0x0FU]);
      out.push_back(kHex[ch & 0x0FU]);
    }
  }
  return out;
}

int HexValue(char ch) {
  if (ch >= '0' && ch <= '9') return ch - '0';
  if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
  return -1;
}

std::string PercentDecode(const std::string& value) {
  std::string out;
  out.reserve(value.size());
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] == '%' && i + 2U < value.size()) {
      const int high = HexValue(value[i + 1U]);
      const int low = HexValue(value[i + 2U]);
      if (high >= 0 && low >= 0) {
        out.push_back(static_cast<char>((high << 4) | low));
        i += 2U;
        continue;
      }
    }
    out.push_back(value[i]);
  }
  return out;
}

bool ParseBool(const std::string& value) {
  return value == "1" || value == "true";
}

template <typename T>
T ParseInteger(const std::string& value, T fallback = 0) {
  try {
    if constexpr (std::is_signed<T>::value) {
      return static_cast<T>(std::stoll(value));
    }
    return static_cast<T>(std::stoull(value));
  } catch (...) {
    return fallback;
  }
}

double ParseDouble(const std::string& value, double fallback = 0.0) {
  try {
    return std::stod(value);
  } catch (...) {
    return fallback;
  }
}

bool IsTerminal(SaveJobState state) {
  return state == SaveJobState::kSucceeded || state == SaveJobState::kFailed ||
      state == SaveJobState::kCancelled;
}

SaveJobState ParseState(const std::string& value) {
  if (value == "WAITING_SNAPSHOT") return SaveJobState::kWaitingSnapshot;
  if (value == "QUEUED") return SaveJobState::kQueued;
  if (value == "RUNNING") return SaveJobState::kRunning;
  if (value == "SUCCEEDED") return SaveJobState::kSucceeded;
  if (value == "FAILED") return SaveJobState::kFailed;
  if (value == "CANCELLED") return SaveJobState::kCancelled;
  return SaveJobState::kFailed;
}

SavePhase ParsePhase(const std::string& value) {
  if (value == "CAPTURE") return SavePhase::kCapture;
  if (value == "VALIDATE") return SavePhase::kValidate;
  if (value == "PROCESS_SOURCE") return SavePhase::kProcessSource;
  if (value == "BUILD_ARTIFACTS") return SavePhase::kBuildArtifacts;
  if (value == "VERIFY") return SavePhase::kVerify;
  if (value == "COMMIT") return SavePhase::kCommit;
  return SavePhase::kDone;
}

bool IsValidJobId(const std::string& value) {
  if (value.empty() || value.size() > 128U || value.front() == '.' || value.front() == '-') {
    return false;
  }
  for (const unsigned char ch : value) {
    if (std::isalnum(ch) == 0 && ch != '-' && ch != '_') {
      return false;
    }
  }
  return true;
}

std::string RequestCanonical(const SaveMapRequest& request) {
  std::ostringstream out;
  out << request.request_id << '\n' << request.map_id << '\n'
      << request.require.occupancy << request.require.octomap << request.require.esdf
      << request.require.traversability << request.require.semantic << '\n'
      << std::setprecision(17) << request.source.voxel_size << '\n'
      << request.source.dynamic_filter_enabled << request.source.dynamic_filter_required << '\n'
      << request.source.dynamic_filter_command << '\n'
      << request.source.dynamic_filter_timeout_sec << '\n'
      << request.source.optimizer_strategy << '\n'
      << request.source.optimizer_required << '\n'
      << request.source.optimizer_command << '\n'
      << request.source.optimizer_timeout_sec << '\n'
      << request.octomap.converter_command << '\n'
      << request.octomap.build_mode << '\n'
      << request.octomap.resolution << '\n'
      << request.octomap.support_dilation_cells << '\n'
      << request.octomap.free_layers_above << '\n'
      << request.octomap.free_dilation_cells << '\n'
      << request.octomap.frame_id << '\n'
      << request.octomap.source_profile << '\n'
      << request.octomap.data_source << '\n'
      << request.octomap.slam_source << '\n'
      << request.octomap.localization_source << '\n'
      << request.octomap.mapping_source << '\n'
      << request.octomap.timeout_sec << '\n'
      << request.activate_on_success << request.require_slam_healthy << '\n'
      << request.minimum_point_count;
  return out.str();
}

std::string SnapshotCanonical(const MapSnapshot& snapshot) {
  std::ostringstream out;
  out << snapshot.snapshot_id << '\n' << snapshot.frame_id << '\n'
      << snapshot.captured_at_ns << '\n' << snapshot.first_sequence << '\n'
      << snapshot.last_sequence << '\n' << snapshot.source_sha256 << '\n'
      << snapshot.slam_healthy << '\n' << snapshot.health_message;
  return out.str();
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
    throw std::runtime_error("failed to atomically replace " + target.string());
  }
#else
  if (::rename(source.c_str(), target.c_str()) != 0) {
    throw std::runtime_error("failed to atomically replace " + target.string());
  }
#endif
  SyncPath(target.parent_path(), true);
}

void WriteTextAtomic(const std::filesystem::path& path, const std::string& value) {
  std::filesystem::create_directories(path.parent_path());
  const auto temp = path.parent_path() /
      (path.filename().string() + ".tmp-" + std::to_string(CurrentProcessId()) + "-" +
       std::to_string(NowNs()));
  {
    std::ofstream file(temp, std::ios::binary | std::ios::trunc);
    if (!file) {
      throw std::runtime_error("failed to write " + temp.string());
    }
    file.write(value.data(), static_cast<std::streamsize>(value.size()));
    file.flush();
    if (!file) {
      throw std::runtime_error("failed to flush " + temp.string());
    }
  }
  SyncPath(temp, false);
  AtomicReplace(temp, path);
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return {};
  }
  return std::string(
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>());
}

std::unordered_map<std::string, std::string> ReadKeyValues(const std::filesystem::path& path) {
  std::unordered_map<std::string, std::string> values;
  std::ifstream file(path, std::ios::binary);
  std::string line;
  while (std::getline(file, line)) {
    const auto equals = line.find('=');
    if (equals == std::string::npos) {
      continue;
    }
    values.emplace(line.substr(0, equals), PercentDecode(line.substr(equals + 1U)));
  }
  return values;
}

std::string GetValue(
    const std::unordered_map<std::string, std::string>& values,
    const std::string& key,
    const std::string& fallback = {}) {
  const auto found = values.find(key);
  return found == values.end() ? fallback : found->second;
}

void AppendKey(std::ostringstream& out, const std::string& key, const std::string& value) {
  out << key << '=' << PercentEncode(value) << '\n';
}

void CopyTree(
    const std::filesystem::path& source,
    const std::filesystem::path& target,
    bool exclude_control_dirs) {
  if (!std::filesystem::is_directory(source)) {
    throw std::runtime_error("source directory not found: " + source.string());
  }
  std::filesystem::create_directories(target);
  for (std::filesystem::recursive_directory_iterator it(source), end; it != end; ++it) {
    const auto relative = std::filesystem::relative(it->path(), source);
    const auto first = relative.begin() == relative.end() ? std::string{} : relative.begin()->string();
    if (exclude_control_dirs &&
        (first == ".builds" || first == ".save_lock" || first == kVersionsDirectoryName)) {
      if (it->is_directory()) {
        it.disable_recursion_pending();
      }
      continue;
    }
    if (it->is_symlink()) {
      throw std::runtime_error("symlinks are not allowed in map snapshots: " + it->path().string());
    }
    const auto destination = target / relative;
    if (it->is_directory()) {
      std::filesystem::create_directories(destination);
    } else if (it->is_regular_file()) {
      std::filesystem::create_directories(destination.parent_path());
      std::filesystem::copy_file(
          it->path(),
          destination,
          std::filesystem::copy_options::overwrite_existing);
    }
  }
}

void SyncTree(const std::filesystem::path& root) {
  std::vector<std::filesystem::path> directories;
  directories.push_back(root);
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      SyncPath(entry.path(), false);
    } else if (entry.is_directory()) {
      directories.push_back(entry.path());
    }
  }
  std::sort(
      directories.begin(),
      directories.end(),
      [](const auto& left, const auto& right) {
        return left.native().size() > right.native().size();
      });
  for (const auto& directory : directories) {
    SyncPath(directory, true);
  }
}

std::uint64_t TreeBytes(const std::filesystem::path& root) {
  std::uint64_t bytes = 0U;
  if (!std::filesystem::is_directory(root)) {
    return bytes;
  }
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      bytes += static_cast<std::uint64_t>(entry.file_size());
    }
  }
  return bytes;
}

bool JsonSucceeded(const std::string& value) {
  return value.find("\"success\":true") != std::string::npos;
}

std::string JsonReason(const std::string& value, const std::string& fallback) {
  const std::string marker = "\"reason_code\":\"";
  const auto begin = value.find(marker);
  if (begin == std::string::npos) {
    return fallback;
  }
  const auto start = begin + marker.size();
  const auto end = value.find('"', start);
  return end == std::string::npos ? fallback : value.substr(start, end - start);
}

std::string VersionName(std::int64_t version) {
  std::ostringstream out;
  out << std::setw(20) << std::setfill('0') << version;
  return out.str();
}

std::int64_t ParseVersionName(const std::string& value) {
  if (value.empty() || !std::all_of(value.begin(), value.end(), [](unsigned char ch) {
        return std::isdigit(ch) != 0;
      })) {
    return 0;
  }
  return ParseInteger<std::int64_t>(value, 0);
}

std::int64_t NextVersion(const std::filesystem::path& map_dir) {
  std::string current_text = ReadText(map_dir / kCurrentVersionFilename);
  while (!current_text.empty() &&
         std::isspace(static_cast<unsigned char>(current_text.back())) != 0) {
    current_text.pop_back();
  }
  std::int64_t current = ParseVersionName(current_text);
  const auto versions = map_dir / kVersionsDirectoryName;
  if (std::filesystem::is_directory(versions)) {
    for (const auto& entry : std::filesystem::directory_iterator(versions)) {
      if (entry.is_directory()) {
        current = std::max(current, ParseVersionName(entry.path().filename().string()));
      }
    }
  }
  return current + 1;
}

std::vector<std::pair<std::string, std::string>> HashArtifacts(
    const std::filesystem::path& version_dir) {
  std::vector<std::pair<std::string, std::string>> hashes;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(version_dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto relative = std::filesystem::relative(entry.path(), version_dir).generic_string();
    if (relative == kManifestFilename || relative == kManifestHashFilename ||
        relative == kArtifactChecksumsFilename) {
      continue;
    }
    hashes.emplace_back(relative, Sha256File(entry.path()));
  }
  std::sort(hashes.begin(), hashes.end());
  return hashes;
}

void VerifyRequiredArtifacts(
    const std::filesystem::path& dir,
    const SaveMapRequest& request,
    std::uint64_t* point_count) {
  const auto pcd = dir / "map.pcd";
  const auto loaded = LoadPcdXyz(pcd);
  if (!loaded.ok || loaded.points.size() < request.minimum_point_count) {
    throw std::runtime_error("map.pcd failed point-count validation");
  }
  if (point_count != nullptr) {
    *point_count = static_cast<std::uint64_t>(loaded.points.size());
  }
  const auto require_file = [&](const char* filename) {
    if (!std::filesystem::is_regular_file(dir / filename)) {
      throw std::runtime_error(std::string("required artifact missing: ") + filename);
    }
  };
  if (request.require.occupancy || request.require.octomap) require_file("occupancy.npz");
  if (request.require.octomap) {
    if (!std::filesystem::is_regular_file(dir / "octomap.ot") &&
        !std::filesystem::is_regular_file(dir / "octomap.bt")) {
      throw std::runtime_error("required artifact missing: octomap.ot or octomap.bt");
    }
    require_file("metadata.json");
  }
  if (request.require.esdf || request.require.traversability) require_file("esdf.npz");
  if (request.require.traversability) require_file("traversability.npz");
  if (request.require.semantic) {
    std::string error;
    if (!ValidateSemanticMapBinary(dir / kSemanticMapArtifactFilename, &error)) {
      throw std::runtime_error(
          error.empty() ? "semantic_map.bin validation failed" : error);
    }
  }
}

std::string BuildManifest(
    const SaveMapRequest& request,
    const MapSnapshot& snapshot,
    const SaveMapStatus& status,
    std::uint64_t point_count,
    const std::vector<std::pair<std::string, std::string>>& hashes) {
  std::string map_pcd_sha256;
  for (const auto& item : hashes) {
    if (item.first == "map.pcd") {
      map_pcd_sha256 = item.second;
      break;
    }
  }
  std::ostringstream out;
  out << "{\n"
      << "  \"schema_version\":2,\n"
      << "  \"product\":\"lingtu_save_map\",\n"
      << "  \"artifact_index\":" << JsonString(kArtifactChecksumsFilename) << ",\n"
      << "  \"job_id\":" << JsonString(status.job_id) << ",\n"
      << "  \"request_id\":" << JsonString(status.request_id) << ",\n"
      << "  \"map_id\":" << JsonString(status.map_id) << ",\n"
      << "  \"lineage_id\":" << JsonString(status.map_id) << ",\n"
      << "  \"version\":" << status.version << ",\n"
      << "  \"created_at_ns\":" << NowNs() << ",\n"
      << "  \"snapshot\":{\"snapshot_id\":" << JsonString(snapshot.snapshot_id)
      << ",\"frame_id\":" << JsonString(snapshot.frame_id)
      << ",\"captured_at_ns\":" << snapshot.captured_at_ns
      << ",\"first_sequence\":" << snapshot.first_sequence
      << ",\"last_sequence\":" << snapshot.last_sequence
      << ",\"source_sha256\":" << JsonString(snapshot.source_sha256)
      << ",\"point_count\":" << point_count << "},\n"
      << "  \"requirements\":{\"occupancy\":" << (request.require.occupancy ? "true" : "false")
      << ",\"octomap\":" << (request.require.octomap ? "true" : "false")
      << ",\"esdf\":" << (request.require.esdf ? "true" : "false")
      << ",\"traversability\":" << (request.require.traversability ? "true" : "false")
      << ",\"semantic\":" << (request.require.semantic ? "true" : "false") << "},\n"
      << "  \"processing\":{"
      << "\"dynamic_filter_enabled\":"
      << (request.source.dynamic_filter_enabled ? "true" : "false")
      << ",\"dynamic_filter_required\":"
      << (request.source.dynamic_filter_required ? "true" : "false")
      << ",\"dynamic_filter_command_sha256\":"
      << JsonString(request.source.dynamic_filter_command.empty()
              ? std::string{}
              : Sha256Text(request.source.dynamic_filter_command))
      << ",\"optimizer_strategy\":" << JsonString(request.source.optimizer_strategy)
      << ",\"optimizer_required\":"
      << (request.source.optimizer_required ? "true" : "false")
      << ",\"optimizer_command_sha256\":"
      << JsonString(request.source.optimizer_command.empty()
              ? std::string{}
              : Sha256Text(request.source.optimizer_command))
      << ",\"octomap_build_mode\":" << JsonString(request.octomap.build_mode)
      << ",\"octomap_converter_command_sha256\":"
      << JsonString(request.octomap.converter_command.empty()
              ? std::string{}
              : Sha256Text(request.octomap.converter_command)) << "},\n"
      << "  \"source_report\":"
      << (status.source_report_json.empty() ? "{}" : status.source_report_json) << ",\n"
      << "  \"artifact_report\":"
      << (status.artifact_report_json.empty() ? "{}" : status.artifact_report_json) << ",\n"
      << "  \"artifacts\":[\n";
  for (std::size_t i = 0; i < hashes.size(); ++i) {
    out << "    {\"path\":" << JsonString(hashes[i].first)
        << ",\"sha256\":" << JsonString(hashes[i].second)
        << ",\"source_sha256\":"
        << JsonString(hashes[i].first == "map.pcd"
                ? snapshot.source_sha256
                : map_pcd_sha256) << "}";
    out << (i + 1U == hashes.size() ? "\n" : ",\n");
  }
  out << "  ]\n}\n";
  return out.str();
}

std::string CurrentVersion(const std::filesystem::path& map_dir) {
  std::string value = ReadText(map_dir / kCurrentVersionFilename);
  while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
    value.pop_back();
  }
  return value;
}

bool ManifestValid(const std::filesystem::path& version_dir) {
  return VerifyMapVersion(version_dir);
}

void SyncCompatibilityView(
    const std::filesystem::path& map_dir,
    const std::filesystem::path& version_dir) {
  static constexpr const char* kManagedNames[] = {
      "map.pcd", "occupancy.npz", "map.pgm", "map.yaml", "octomap.ot", "octomap.bt",
      "esdf.npz", "traversability.npz", "semantic_map.bin", "metadata.json", "poses.txt",
      "trajectory.txt", "patches", "map.raw.pcd", "map.clean.pcd", "map.removed.pcd",
      "map.pcd.preclean", "map_optimization.json", kManifestFilename,
      kManifestHashFilename, kArtifactChecksumsFilename,
  };
  const auto stage = map_dir /
      (".compatibility-stage-" + version_dir.filename().string() + "-" +
       std::to_string(NowNs()));
  std::filesystem::remove_all(stage);
  std::filesystem::create_directories(stage);
  try {
    for (const char* name : kManagedNames) {
      const auto source = version_dir / name;
      const auto staged = stage / name;
      if (!std::filesystem::exists(source)) continue;
      if (std::filesystem::is_directory(source)) {
        CopyTree(source, staged, false);
      } else {
        std::filesystem::copy_file(
            source, staged, std::filesystem::copy_options::overwrite_existing);
      }
    }
    SyncTree(stage);

    for (const char* name : kManagedNames) {
      const auto staged = stage / name;
      const auto target = map_dir / name;
      if (!std::filesystem::exists(staged)) {
        std::error_code ignored;
        std::filesystem::remove_all(target, ignored);
        continue;
      }
      const auto temp = map_dir /
          (std::string(".compat-") + name + "-" + std::to_string(NowNs()));
      std::filesystem::remove_all(temp);
      if (std::filesystem::is_directory(staged)) {
        std::filesystem::rename(staged, temp);
        std::filesystem::remove_all(target);
        std::filesystem::rename(temp, target);
      } else {
        std::filesystem::copy_file(
            staged, temp, std::filesystem::copy_options::overwrite_existing);
        SyncPath(temp, false);
        AtomicReplace(temp, target);
      }
    }
    WriteTextAtomic(
        map_dir / "compatibility_version.txt",
        version_dir.filename().string() + "\n");
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove_all(stage, ignored);
    throw;
  }
  std::error_code ignored;
  std::filesystem::remove_all(stage, ignored);
}

}  // namespace

const char* SaveJobStateName(SaveJobState state) {
  switch (state) {
    case SaveJobState::kWaitingSnapshot: return "WAITING_SNAPSHOT";
    case SaveJobState::kQueued: return "QUEUED";
    case SaveJobState::kRunning: return "RUNNING";
    case SaveJobState::kSucceeded: return "SUCCEEDED";
    case SaveJobState::kFailed: return "FAILED";
    case SaveJobState::kCancelled: return "CANCELLED";
  }
  return "FAILED";
}

const char* SavePhaseName(SavePhase phase) {
  switch (phase) {
    case SavePhase::kCapture: return "CAPTURE";
    case SavePhase::kValidate: return "VALIDATE";
    case SavePhase::kProcessSource: return "PROCESS_SOURCE";
    case SavePhase::kBuildArtifacts: return "BUILD_ARTIFACTS";
    case SavePhase::kVerify: return "VERIFY";
    case SavePhase::kCommit: return "COMMIT";
    case SavePhase::kDone: return "DONE";
  }
  return "DONE";
}

std::string SaveMapStatusJson(const SaveMapStatus& status) {
  return "{"
      "\"job_id\":" + JsonString(status.job_id) + ","
      "\"request_id\":" + JsonString(status.request_id) + ","
      "\"map_id\":" + JsonString(status.map_id) + ","
      "\"state\":" + JsonString(SaveJobStateName(status.state)) + ","
      "\"phase\":" + JsonString(SavePhaseName(status.phase)) + ","
      "\"progress\":" + std::to_string(status.progress) + ","
      "\"message\":" + JsonString(status.message) + ","
      "\"reason_code\":" + JsonString(status.reason_code) + ","
      "\"capture_dir\":" + JsonString(status.capture_dir.string()) + ","
      "\"version_dir\":" + JsonString(status.version_dir.string()) + ","
      "\"manifest_path\":" + JsonString(status.manifest_path.string()) + ","
      "\"source_report\":" +
          (status.source_report_json.empty() ? "{}" : status.source_report_json) + ","
      "\"artifact_report\":" +
          (status.artifact_report_json.empty() ? "{}" : status.artifact_report_json) + ","
      "\"created_at_ns\":" + std::to_string(status.created_at_ns) + ","
      "\"updated_at_ns\":" + std::to_string(status.updated_at_ns) + ","
      "\"completed_at_ns\":" + std::to_string(status.completed_at_ns) + ","
      "\"version\":" + std::to_string(status.version) + ","
      "\"cancel_requested\":" + (status.cancel_requested ? "true" : "false") + ","
      "\"compatibility_ready\":" +
          (status.compatibility_ready ? "true" : "false") + ","
      "\"compatibility_message\":" + JsonString(status.compatibility_message) + ","
      "\"recovered\":" + (status.recovered ? "true" : "false") + ","
      "\"replayed\":" + (status.replayed ? "true" : "false") +
      "}";
}

class SaveMapEngine::Impl {
 public:
  Impl(MapStore& store, SaveMapHooks hooks)
      : store_(store),
        hooks_(std::move(hooks)),
        jobs_root_(store.RootDir() / kJobsDirectoryName),
        engine_lock_path_(store.RootDir() / ".save_engine_lock") {
    AcquireEngineLock();
    try {
      std::filesystem::create_directories(jobs_root_);
      RecoverInternal();
      worker_ = std::thread([this]() { WorkerLoop(); });
    } catch (...) {
      ReleaseEngineLock();
      throw;
    }
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
    ReleaseEngineLock();
  }

  struct Job {
    SaveMapRequest request;
    std::optional<MapSnapshot> snapshot;
    SaveMapStatus status;
    std::string request_fingerprint;
    std::string snapshot_fingerprint;
    bool accepting_snapshot{false};
  };

  SaveMapResult Begin(const SaveMapRequest& raw_request) {
    SaveMapRequest request = raw_request;
    request.map_id = MapStore::NormalizeMapId(request.map_id);
    if (!IsValidJobId(request.request_id)) {
      throw std::invalid_argument("request_id must contain only letters, digits, '_' or '-'");
    }
    if (request.minimum_point_count == 0U) {
      throw std::invalid_argument("minimum_point_count must be positive");
    }
    if (request.require.traversability) {
      request.require.esdf = true;
    }
    const std::string fingerprint = Sha256Text(RequestCanonical(request));

    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = jobs_.find(request.request_id);
    if (existing != jobs_.end()) {
      if (existing->second->request_fingerprint != fingerprint) {
        SaveMapStatus conflict = existing->second->status;
        conflict.state = SaveJobState::kFailed;
        conflict.reason_code = "idempotency_conflict";
        conflict.message = "request_id already belongs to a different SaveMap request";
        conflict.replayed = true;
        return {false, true, "idempotency_conflict", conflict};
      }
      SaveMapStatus replay = existing->second->status;
      replay.replayed = true;
      return {true, true, {}, replay};
    }

    auto job = std::make_shared<Job>();
    job->request = request;
    job->request_fingerprint = fingerprint;
    job->status.job_id = request.request_id;
    job->status.request_id = request.request_id;
    job->status.map_id = request.map_id;
    job->status.state = SaveJobState::kWaitingSnapshot;
    job->status.phase = SavePhase::kCapture;
    job->status.message = "waiting for immutable map snapshot";
    job->status.created_at_ns = NowNs();
    job->status.updated_at_ns = job->status.created_at_ns;
    job->status.capture_dir = JobDir(request.request_id) / "capture";
    std::filesystem::create_directories(job->status.capture_dir);
    jobs_.emplace(request.request_id, job);
    PersistJobLocked(*job);
    AppendEventLocked(*job, "REQUESTED");
    return {true, false, {}, job->status};
  }

  SaveMapResult ProvideSnapshot(const std::string& job_id, const MapSnapshot& raw_snapshot) {
    std::shared_ptr<Job> job;
    SaveMapStatus queued_status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = jobs_.find(job_id);
      if (found == jobs_.end()) {
        return {false, false, "job_not_found", MissingStatus(job_id, "job_not_found")};
      }
      job = found->second;
      if (job->snapshot.has_value()) {
        const bool id_conflict = !raw_snapshot.snapshot_id.empty() &&
            raw_snapshot.snapshot_id != job->snapshot->snapshot_id;
        const bool hash_conflict = !raw_snapshot.source_sha256.empty() &&
            raw_snapshot.source_sha256 != job->snapshot->source_sha256;
        if (id_conflict || hash_conflict) {
          SaveMapStatus conflict = job->status;
          conflict.reason_code = "snapshot_idempotency_conflict";
          conflict.message = "job already owns a different immutable snapshot";
          return {false, true, conflict.reason_code, conflict};
        }
        SaveMapStatus replay = job->status;
        replay.replayed = true;
        return {true, true, {}, replay};
      }
      if (IsTerminal(job->status.state)) {
        SaveMapStatus replay = job->status;
        replay.replayed = true;
        return {job->status.state == SaveJobState::kSucceeded, true, job->status.reason_code, replay};
      }
      if (job->accepting_snapshot) {
        SaveMapStatus busy = job->status;
        busy.reason_code = "snapshot_capture_in_progress";
        busy.message = "another caller is capturing this SaveMap snapshot";
        return {false, false, busy.reason_code, busy};
      }
      job->accepting_snapshot = true;
    }

    struct CaptureReservation {
      std::mutex& mutex;
      std::shared_ptr<Job> job;
      ~CaptureReservation() {
        std::lock_guard<std::mutex> lock(mutex);
        job->accepting_snapshot = false;
      }
    } capture_reservation{mutex_, job};

    MapSnapshot snapshot = raw_snapshot;
    if (snapshot.snapshot_id.empty()) {
      snapshot.snapshot_id = job_id;
    }
    if (snapshot.frame_id.empty()) {
      snapshot.frame_id = "map";
    }
    if (snapshot.captured_at_ns == 0) {
      snapshot.captured_at_ns = NowNs();
    }
    if (job->request.require_slam_healthy && !snapshot.slam_healthy) {
      return FailBeforeQueue(*job, "slam_unhealthy", snapshot.health_message.empty()
          ? "SLAM health gate rejected the snapshot" : snapshot.health_message);
    }
    if (snapshot.frame_id != job->request.octomap.frame_id) {
      return FailBeforeQueue(*job, "snapshot_frame_mismatch", "snapshot frame does not match map frame");
    }
    if (!std::filesystem::is_directory(snapshot.source_dir)) {
      return FailBeforeQueue(*job, "snapshot_source_missing", "snapshot source directory not found");
    }

    const auto capture_dir = job->status.capture_dir;
    try {
      const auto canonical_source = std::filesystem::weakly_canonical(snapshot.source_dir);
      const auto canonical_capture = std::filesystem::weakly_canonical(capture_dir);
      if (canonical_source != canonical_capture) {
        const auto temp_capture = JobDir(job_id) / ("capture-staging-" + std::to_string(NowNs()));
        std::filesystem::remove_all(temp_capture);
        CopyTree(snapshot.source_dir, temp_capture, true);
        std::filesystem::remove_all(capture_dir);
        std::filesystem::rename(temp_capture, capture_dir);
      }
      const auto pcd_path = capture_dir / "map.pcd";
      if (!std::filesystem::is_regular_file(pcd_path)) {
        return FailBeforeQueue(*job, "snapshot_pcd_missing", "snapshot does not contain map.pcd");
      }
      const auto source_sha = Sha256File(pcd_path);
      if (!snapshot.source_sha256.empty() && snapshot.source_sha256 != source_sha) {
        return FailBeforeQueue(*job, "snapshot_hash_mismatch", "snapshot source hash does not match map.pcd");
      }
      snapshot.source_sha256 = source_sha;
      snapshot.source_dir = capture_dir;
      const auto loaded = LoadPcdXyz(pcd_path);
      if (!loaded.ok || loaded.points.size() < job->request.minimum_point_count) {
        return FailBeforeQueue(*job, "snapshot_pcd_invalid", "snapshot map.pcd failed point-count validation");
      }
      if (job->request.require.semantic) {
        std::string semantic_error;
        if (!ValidateSemanticMapBinary(capture_dir / kSemanticMapArtifactFilename, &semantic_error)) {
          return FailBeforeQueue(
              *job,
              "semantic_snapshot_invalid",
              semantic_error.empty() ? "required semantic snapshot is missing or invalid" : semantic_error);
        }
      }
      const auto bytes = TreeBytes(capture_dir);
      const auto available = std::filesystem::space(store_.RootDir()).available;
      const std::uint64_t reserve = 16ULL * 1024ULL * 1024ULL;
      if (available < bytes * 3U + reserve) {
        return FailBeforeQueue(*job, "insufficient_disk_space", "not enough disk space for staged SaveMap transaction");
      }
      SyncTree(capture_dir);
    } catch (const std::exception& exc) {
      return FailBeforeQueue(*job, "snapshot_capture_failed", exc.what());
    }

    const std::string fingerprint = Sha256Text(SnapshotCanonical(snapshot));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      job->snapshot = snapshot;
      job->snapshot_fingerprint = fingerprint;
      job->status.state = SaveJobState::kQueued;
      job->status.phase = SavePhase::kValidate;
      job->status.progress = 0.05;
      job->status.message = "snapshot accepted and queued";
      job->status.updated_at_ns = NowNs();
      PersistJobLocked(*job);
      AppendEventLocked(*job, "SNAPSHOT_ACCEPTED");
      queue_.push_back(job_id);
      queued_status = job->status;
    }
    cv_.notify_all();
    return {true, false, {}, queued_status};
  }

  SaveMapResult Cancel(const std::string& job_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    if (found == jobs_.end()) {
      return {false, false, "job_not_found", MissingStatus(job_id, "job_not_found")};
    }
    auto& job = *found->second;
    if (IsTerminal(job.status.state)) {
      SaveMapStatus replay = job.status;
      replay.replayed = true;
      return {job.status.state == SaveJobState::kSucceeded, true, job.status.reason_code, replay};
    }
    job.status.cancel_requested = true;
    job.status.updated_at_ns = NowNs();
    job.status.message = "cancellation requested";
    if (job.status.state == SaveJobState::kWaitingSnapshot || job.status.state == SaveJobState::kQueued) {
      job.status.state = SaveJobState::kCancelled;
      job.status.phase = SavePhase::kDone;
      job.status.progress = 1.0;
      job.status.reason_code = "cancelled";
      job.status.completed_at_ns = job.status.updated_at_ns;
    }
    PersistJobLocked(job);
    AppendEventLocked(job, job.status.state == SaveJobState::kCancelled
        ? "CANCELLED"
        : "CANCEL_REQUESTED");
    cv_.notify_all();
    return {true, false, {}, job.status};
  }

  SaveMapResult Retry(const std::string& job_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    if (found == jobs_.end()) {
      return {false, false, "job_not_found", MissingStatus(job_id, "job_not_found")};
    }
    auto& job = *found->second;
    if (job.status.state == SaveJobState::kSucceeded) {
      SaveMapStatus replay = job.status;
      replay.replayed = true;
      return {true, true, {}, replay};
    }
    if (!IsTerminal(job.status.state)) {
      return {false, false, "job_not_terminal", job.status};
    }
    job.status.cancel_requested = false;
    job.status.reason_code.clear();
    job.status.completed_at_ns = 0;
    job.status.version = 0;
    job.status.version_dir.clear();
    job.status.manifest_path.clear();
    job.status.artifact_report_json.clear();
    job.status.compatibility_ready = false;
    job.status.compatibility_message.clear();
    if (job.snapshot.has_value() &&
        std::filesystem::is_regular_file(job.status.capture_dir / "map.pcd")) {
      job.status.state = SaveJobState::kQueued;
      job.status.phase = SavePhase::kValidate;
      job.status.progress = 0.05;
      job.status.message = "SaveMap retry queued";
      queue_.push_back(job_id);
    } else {
      job.snapshot.reset();
      job.snapshot_fingerprint.clear();
      job.status.state = SaveJobState::kWaitingSnapshot;
      job.status.phase = SavePhase::kCapture;
      job.status.progress = 0.0;
      job.status.message = "SaveMap retry requires a new snapshot";
      std::filesystem::create_directories(job.status.capture_dir);
    }
    job.status.updated_at_ns = NowNs();
    PersistJobLocked(job);
    AppendEventLocked(job, "RETRY_REQUESTED");
    cv_.notify_all();
    return {true, false, {}, job.status};
  }

  std::optional<SaveMapStatus> GetStatus(const std::string& job_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    return found == jobs_.end() ? std::nullopt : std::optional<SaveMapStatus>(found->second->status);
  }

  std::vector<SaveMapStatus> ListStatuses(std::size_t limit) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SaveMapStatus> statuses;
    statuses.reserve(jobs_.size());
    for (const auto& item : jobs_) {
      statuses.push_back(item.second->status);
    }
    std::sort(statuses.begin(), statuses.end(), [](const auto& left, const auto& right) {
      return left.created_at_ns > right.created_at_ns;
    });
    if (limit > 0U && statuses.size() > limit) {
      statuses.resize(limit);
    }
    return statuses;
  }

  std::optional<SaveMapStatus> Wait(
      const std::string& job_id,
      std::chrono::milliseconds timeout) const {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    if (found == jobs_.end()) {
      return std::nullopt;
    }
    cv_.wait_for(lock, timeout, [&]() {
      const auto current = jobs_.find(job_id);
      return current == jobs_.end() || IsTerminal(current->second->status.state);
    });
    const auto current = jobs_.find(job_id);
    return current == jobs_.end() ? std::nullopt : std::optional<SaveMapStatus>(current->second->status);
  }

  void Recover() {
    std::lock_guard<std::mutex> lock(mutex_);
    RecoverInternalLocked();
    cv_.notify_all();
  }

  std::string ListVersionsJson(const std::string& map_id) {
    const std::string id = MapStore::NormalizeMapId(map_id);
    auto map_lock = MapLock::TryAcquire(store_.RootDir(), id, "list-versions");
    if (!map_lock.has_value()) {
      return "{\"action\":\"list_map_versions\",\"success\":false,"
          "\"reason_code\":\"map_write_in_progress\",\"map_id\":" +
          JsonString(id) + "}";
    }
    const auto map_dir = store_.MapPath(id);
    const auto versions_dir = map_dir / kVersionsDirectoryName;
    const std::string current = CurrentVersion(map_dir);
    std::vector<std::pair<std::int64_t, std::filesystem::path>> versions;
    if (std::filesystem::is_directory(versions_dir)) {
      for (const auto& entry : std::filesystem::directory_iterator(versions_dir)) {
        const auto version = ParseVersionName(entry.path().filename().string());
        if (entry.is_directory() && version > 0 && ManifestValid(entry.path())) {
          versions.emplace_back(version, entry.path());
        }
      }
    }
    std::sort(versions.begin(), versions.end(), [](const auto& left, const auto& right) {
      return left.first > right.first;
    });
    std::ostringstream out;
    out << "{\"action\":\"list_map_versions\",\"success\":true,"
        << "\"map_id\":" << JsonString(id) << ",\"versions\":[";
    for (std::size_t i = 0; i < versions.size(); ++i) {
      if (i > 0U) out << ',';
      out << "{\"version\":" << versions[i].first
          << ",\"current\":"
          << (versions[i].second.filename().string() == current ? "true" : "false")
          << ",\"path\":" << JsonString(versions[i].second.string())
          << ",\"manifest\":"
          << JsonString((versions[i].second / kManifestFilename).string())
          << ",\"manifest_sha256\":"
          << JsonString(Sha256File(versions[i].second / kManifestFilename)) << "}";
    }
    out << "],\"count\":" << versions.size() << "}";
    return out.str();
  }

  std::string RollbackVersionJson(const std::string& map_id, std::int64_t version) {
    const std::string id = MapStore::NormalizeMapId(map_id);
    if (version <= 0) {
      return "{\"action\":\"rollback_map_version\",\"success\":false,"
          "\"reason_code\":\"invalid_version\",\"map_id\":" + JsonString(id) + "}";
    }
    auto map_lock = MapLock::TryAcquire(
        store_.RootDir(), id, "rollback-v" + std::to_string(version));
    if (!map_lock.has_value()) {
      return "{\"action\":\"rollback_map_version\",\"success\":false,"
          "\"reason_code\":\"map_save_in_progress\",\"map_id\":" + JsonString(id) + "}";
    }
    const auto map_dir = store_.MapPath(id);
    const auto target = map_dir / kVersionsDirectoryName / VersionName(version);
    std::string integrity_error;
    if (!VerifyMapVersion(target, &integrity_error)) {
      return "{\"action\":\"rollback_map_version\",\"success\":false,"
          "\"reason_code\":\"version_not_found_or_invalid\",\"map_id\":" +
          JsonString(id) + ",\"version\":" + std::to_string(version) +
          ",\"message\":" + JsonString(integrity_error) + "}";
    }
    const std::string previous = CurrentVersion(map_dir);
    WriteTextAtomic(map_dir / kCurrentVersionFilename, VersionName(version) + "\n");
    std::string warning;
    try {
      SyncCompatibilityView(map_dir, target);
    } catch (const std::exception& exc) {
      warning = exc.what();
    }
    const auto event_path = map_dir / "version_events.jsonl";
    {
      std::ofstream event(event_path, std::ios::binary | std::ios::app);
      event << "{\"event\":\"VERSION_ROLLED_BACK\",\"timestamp_ns\":" << NowNs()
            << ",\"from\":" << JsonString(previous)
            << ",\"to\":" << JsonString(VersionName(version)) << "}\n";
      event.flush();
    }
    SyncPath(event_path, false);
    return "{\"action\":\"rollback_map_version\",\"success\":true,"
        "\"map_id\":" + JsonString(id) + ",\"version\":" +
        std::to_string(version) + ",\"previous_version\":" + JsonString(previous) +
        ",\"compatibility_ready\":" + (warning.empty() ? "true" : "false") +
        ",\"degraded\":" + (warning.empty() ? "false" : "true") +
        ",\"warning\":" + JsonString(warning) + "}";
  }

 private:
  std::filesystem::path JobDir(const std::string& job_id) const {
    return jobs_root_ / job_id;
  }

  std::filesystem::path JobStatePath(const std::string& job_id) const {
    return JobDir(job_id) / "job.state";
  }

  std::filesystem::path JobJournalPath(const std::string& job_id) const {
    return JobDir(job_id) / "events.jsonl";
  }

  std::filesystem::path WorkRoot(const std::string& job_id) const {
#if defined(_WIN32)
    const auto key = Sha256Text(store_.RootDir().string() + "\n" + job_id).substr(0, 16);
    return std::filesystem::temp_directory_path() / "lt_maps" / key;
#else
    return JobDir(job_id) / "work";
#endif
  }

  void AcquireEngineLock() {
    std::filesystem::create_directories(engine_lock_path_.parent_path());
    if (!std::filesystem::create_directory(engine_lock_path_)) {
      const auto owner = ReadKeyValues(engine_lock_path_ / "owner.state");
      const auto owner_pid = ParseInteger<std::uint64_t>(GetValue(owner, "pid"));
      if (ProcessAlive(owner_pid)) {
        throw std::runtime_error(
            "another SaveMap engine owns map root: " + store_.RootDir().string());
      }
      std::filesystem::remove_all(engine_lock_path_);
      if (!std::filesystem::create_directory(engine_lock_path_)) {
        throw std::runtime_error("failed to acquire SaveMap engine lock");
      }
    }
    std::ostringstream owner;
    AppendKey(owner, "pid", std::to_string(CurrentProcessId()));
    AppendKey(owner, "created_at_ns", std::to_string(NowNs()));
    WriteTextAtomic(engine_lock_path_ / "owner.state", owner.str());
    owns_engine_lock_ = true;
  }

  void ReleaseEngineLock() noexcept {
    if (!owns_engine_lock_) {
      return;
    }
    std::error_code ignored;
    std::filesystem::remove_all(engine_lock_path_, ignored);
    owns_engine_lock_ = false;
  }

  SaveMapStatus MissingStatus(const std::string& job_id, const std::string& reason) const {
    SaveMapStatus status;
    status.job_id = job_id;
    status.state = SaveJobState::kFailed;
    status.phase = SavePhase::kDone;
    status.reason_code = reason;
    status.message = "SaveMap job not found";
    return status;
  }

  SaveMapResult FailBeforeQueue(Job& job, const std::string& reason, const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex_);
    job.status.state = SaveJobState::kFailed;
    job.status.phase = SavePhase::kDone;
    job.status.progress = 1.0;
    job.status.reason_code = reason;
    job.status.message = message;
    job.status.updated_at_ns = NowNs();
    job.status.completed_at_ns = job.status.updated_at_ns;
    PersistJobLocked(job);
    AppendEventLocked(job, "FAILED");
    cv_.notify_all();
    return {false, false, reason, job.status};
  }

  void PersistJobLocked(const Job& job) const {
    std::ostringstream out;
    AppendKey(out, "schema", kJobSchemaVersion);
    AppendKey(out, "job_id", job.status.job_id);
    AppendKey(out, "request_id", job.request.request_id);
    AppendKey(out, "map_id", job.request.map_id);
    AppendKey(out, "request_fingerprint", job.request_fingerprint);
    AppendKey(out, "snapshot_fingerprint", job.snapshot_fingerprint);
    AppendKey(out, "state", SaveJobStateName(job.status.state));
    AppendKey(out, "phase", SavePhaseName(job.status.phase));
    AppendKey(out, "progress", std::to_string(job.status.progress));
    AppendKey(out, "message", job.status.message);
    AppendKey(out, "reason_code", job.status.reason_code);
    AppendKey(out, "capture_dir", job.status.capture_dir.string());
    AppendKey(out, "version_dir", job.status.version_dir.string());
    AppendKey(out, "manifest_path", job.status.manifest_path.string());
    AppendKey(out, "source_report_json", job.status.source_report_json);
    AppendKey(out, "artifact_report_json", job.status.artifact_report_json);
    AppendKey(out, "created_at_ns", std::to_string(job.status.created_at_ns));
    AppendKey(out, "updated_at_ns", std::to_string(job.status.updated_at_ns));
    AppendKey(out, "completed_at_ns", std::to_string(job.status.completed_at_ns));
    AppendKey(out, "version", std::to_string(job.status.version));
    AppendKey(out, "cancel_requested", job.status.cancel_requested ? "1" : "0");
    AppendKey(out, "compatibility_ready", job.status.compatibility_ready ? "1" : "0");
    AppendKey(out, "compatibility_message", job.status.compatibility_message);
    AppendKey(out, "recovered", job.status.recovered ? "1" : "0");
    AppendKey(out, "require_occupancy", job.request.require.occupancy ? "1" : "0");
    AppendKey(out, "require_octomap", job.request.require.octomap ? "1" : "0");
    AppendKey(out, "require_esdf", job.request.require.esdf ? "1" : "0");
    AppendKey(out, "require_traversability", job.request.require.traversability ? "1" : "0");
    AppendKey(out, "require_semantic", job.request.require.semantic ? "1" : "0");
    AppendKey(out, "activate_on_success", job.request.activate_on_success ? "1" : "0");
    AppendKey(out, "require_slam_healthy", job.request.require_slam_healthy ? "1" : "0");
    AppendKey(out, "minimum_point_count", std::to_string(job.request.minimum_point_count));
    AppendKey(out, "source_voxel_size", std::to_string(job.request.source.voxel_size));
    AppendKey(out, "dynamic_filter_enabled", job.request.source.dynamic_filter_enabled ? "1" : "0");
    AppendKey(out, "dynamic_filter_required", job.request.source.dynamic_filter_required ? "1" : "0");
    AppendKey(out, "dynamic_filter_command", job.request.source.dynamic_filter_command);
    AppendKey(out, "dynamic_filter_timeout_sec", std::to_string(job.request.source.dynamic_filter_timeout_sec));
    AppendKey(out, "optimizer_strategy", job.request.source.optimizer_strategy);
    AppendKey(out, "optimizer_required", job.request.source.optimizer_required ? "1" : "0");
    AppendKey(out, "optimizer_command", job.request.source.optimizer_command);
    AppendKey(out, "optimizer_timeout_sec", std::to_string(job.request.source.optimizer_timeout_sec));
    AppendKey(out, "octomap_converter_command", job.request.octomap.converter_command);
    AppendKey(out, "octomap_build_mode", job.request.octomap.build_mode);
    AppendKey(out, "octomap_resolution", std::to_string(job.request.octomap.resolution));
    AppendKey(out, "octomap_support_dilation_cells", std::to_string(job.request.octomap.support_dilation_cells));
    AppendKey(out, "octomap_free_layers_above", std::to_string(job.request.octomap.free_layers_above));
    AppendKey(out, "octomap_free_dilation_cells", std::to_string(job.request.octomap.free_dilation_cells));
    AppendKey(out, "octomap_frame_id", job.request.octomap.frame_id);
    AppendKey(out, "octomap_source_profile", job.request.octomap.source_profile);
    AppendKey(out, "octomap_data_source", job.request.octomap.data_source);
    AppendKey(out, "octomap_slam_source", job.request.octomap.slam_source);
    AppendKey(out, "octomap_localization_source", job.request.octomap.localization_source);
    AppendKey(out, "octomap_mapping_source", job.request.octomap.mapping_source);
    AppendKey(out, "octomap_timeout_sec", std::to_string(job.request.octomap.timeout_sec));
    if (job.snapshot.has_value()) {
      AppendKey(out, "snapshot_id", job.snapshot->snapshot_id);
      AppendKey(out, "snapshot_source_dir", job.snapshot->source_dir.string());
      AppendKey(out, "snapshot_frame_id", job.snapshot->frame_id);
      AppendKey(out, "snapshot_captured_at_ns", std::to_string(job.snapshot->captured_at_ns));
      AppendKey(out, "snapshot_first_sequence", std::to_string(job.snapshot->first_sequence));
      AppendKey(out, "snapshot_last_sequence", std::to_string(job.snapshot->last_sequence));
      AppendKey(out, "snapshot_source_sha256", job.snapshot->source_sha256);
      AppendKey(out, "snapshot_slam_healthy", job.snapshot->slam_healthy ? "1" : "0");
      AppendKey(out, "snapshot_health_message", job.snapshot->health_message);
    }
    WriteTextAtomic(JobStatePath(job.status.job_id), out.str());
  }

  void AppendEventLocked(const Job& job, const std::string& event) const {
    const auto path = JobJournalPath(job.status.job_id);
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary | std::ios::app);
    if (!file) {
      throw std::runtime_error("failed to append SaveMap journal: " + path.string());
    }
    file << "{\"schema_version\":1,\"event\":" << JsonString(event)
         << ",\"timestamp_ns\":" << NowNs()
         << ",\"job_id\":" << JsonString(job.status.job_id)
         << ",\"map_id\":" << JsonString(job.status.map_id)
         << ",\"state\":" << JsonString(SaveJobStateName(job.status.state))
         << ",\"phase\":" << JsonString(SavePhaseName(job.status.phase))
         << ",\"progress\":" << job.status.progress
         << ",\"reason_code\":" << JsonString(job.status.reason_code)
         << ",\"message\":" << JsonString(job.status.message) << "}\n";
    file.flush();
    if (!file) {
      throw std::runtime_error("failed to flush SaveMap journal: " + path.string());
    }
    SyncPath(path, false);
  }

  void AppendEvent(const std::shared_ptr<Job>& job, const std::string& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    AppendEventLocked(*job, event);
  }

  std::shared_ptr<Job> LoadJob(const std::filesystem::path& state_path) const {
    const auto values = ReadKeyValues(state_path);
    if (GetValue(values, "schema") != kJobSchemaVersion) {
      return nullptr;
    }
    auto job = std::make_shared<Job>();
    job->request.request_id = GetValue(values, "request_id");
    job->request.map_id = GetValue(values, "map_id");
    if (!IsValidJobId(job->request.request_id) || !MapStore::IsValidMapId(job->request.map_id)) {
      return nullptr;
    }
    job->request_fingerprint = GetValue(values, "request_fingerprint");
    job->snapshot_fingerprint = GetValue(values, "snapshot_fingerprint");
    job->request.require.occupancy = ParseBool(GetValue(values, "require_occupancy", "1"));
    job->request.require.octomap = ParseBool(GetValue(values, "require_octomap", "1"));
    job->request.require.esdf = ParseBool(GetValue(values, "require_esdf", "1"));
    job->request.require.traversability = ParseBool(GetValue(values, "require_traversability", "1"));
    job->request.require.semantic = ParseBool(GetValue(values, "require_semantic"));
    job->request.activate_on_success = ParseBool(GetValue(values, "activate_on_success"));
    job->request.require_slam_healthy = ParseBool(GetValue(values, "require_slam_healthy", "1"));
    job->request.minimum_point_count = ParseInteger<std::uint64_t>(GetValue(values, "minimum_point_count"), 1U);
    job->request.source.voxel_size = ParseDouble(GetValue(values, "source_voxel_size"));
    job->request.source.dynamic_filter_enabled = ParseBool(GetValue(values, "dynamic_filter_enabled", "1"));
    job->request.source.dynamic_filter_required = ParseBool(GetValue(values, "dynamic_filter_required"));
    job->request.source.dynamic_filter_command = GetValue(values, "dynamic_filter_command");
    job->request.source.dynamic_filter_timeout_sec = ParseDouble(GetValue(values, "dynamic_filter_timeout_sec"), 300.0);
    job->request.source.optimizer_strategy = GetValue(values, "optimizer_strategy", "pgo");
    job->request.source.optimizer_required = ParseBool(GetValue(values, "optimizer_required"));
    job->request.source.optimizer_command = GetValue(values, "optimizer_command");
    job->request.source.optimizer_timeout_sec = ParseDouble(GetValue(values, "optimizer_timeout_sec"), 120.0);
    job->request.octomap.converter_command = GetValue(values, "octomap_converter_command");
    job->request.octomap.build_mode = GetValue(values, "octomap_build_mode", "external_pcl_converter");
    job->request.octomap.resolution = ParseDouble(GetValue(values, "octomap_resolution"), 0.20);
    job->request.octomap.support_dilation_cells = ParseInteger<int>(GetValue(values, "octomap_support_dilation_cells"), 1);
    job->request.octomap.free_layers_above = ParseInteger<int>(GetValue(values, "octomap_free_layers_above"), 3);
    job->request.octomap.free_dilation_cells = ParseInteger<int>(GetValue(values, "octomap_free_dilation_cells"), 1);
    job->request.octomap.frame_id = GetValue(values, "octomap_frame_id", "map");
    job->request.octomap.source_profile = GetValue(values, "octomap_source_profile", "map_pipeline");
    job->request.octomap.data_source = GetValue(values, "octomap_data_source", "map_pipeline");
    job->request.octomap.slam_source = GetValue(values, "octomap_slam_source", "unknown");
    job->request.octomap.localization_source = GetValue(values, "octomap_localization_source", "unknown");
    job->request.octomap.mapping_source = GetValue(values, "octomap_mapping_source", "lingtu_maps_pipeline");
    job->request.octomap.timeout_sec = ParseDouble(GetValue(values, "octomap_timeout_sec"), 60.0);

    job->status.job_id = GetValue(values, "job_id", job->request.request_id);
    job->status.request_id = job->request.request_id;
    job->status.map_id = job->request.map_id;
    job->status.state = ParseState(GetValue(values, "state"));
    job->status.phase = ParsePhase(GetValue(values, "phase"));
    job->status.progress = ParseDouble(GetValue(values, "progress"));
    job->status.message = GetValue(values, "message");
    job->status.reason_code = GetValue(values, "reason_code");
    job->status.capture_dir = GetValue(values, "capture_dir");
    job->status.version_dir = GetValue(values, "version_dir");
    job->status.manifest_path = GetValue(values, "manifest_path");
    job->status.source_report_json = GetValue(values, "source_report_json");
    job->status.artifact_report_json = GetValue(values, "artifact_report_json");
    job->status.created_at_ns = ParseInteger<std::int64_t>(GetValue(values, "created_at_ns"));
    job->status.updated_at_ns = ParseInteger<std::int64_t>(GetValue(values, "updated_at_ns"));
    job->status.completed_at_ns = ParseInteger<std::int64_t>(GetValue(values, "completed_at_ns"));
    job->status.version = ParseInteger<std::int64_t>(GetValue(values, "version"));
    job->status.cancel_requested = ParseBool(GetValue(values, "cancel_requested"));
    job->status.compatibility_ready = ParseBool(GetValue(values, "compatibility_ready"));
    job->status.compatibility_message = GetValue(values, "compatibility_message");
    job->status.recovered = ParseBool(GetValue(values, "recovered"));

    if (!GetValue(values, "snapshot_id").empty()) {
      MapSnapshot snapshot;
      snapshot.snapshot_id = GetValue(values, "snapshot_id");
      snapshot.source_dir = GetValue(values, "snapshot_source_dir");
      snapshot.frame_id = GetValue(values, "snapshot_frame_id", "map");
      snapshot.captured_at_ns = ParseInteger<std::int64_t>(GetValue(values, "snapshot_captured_at_ns"));
      snapshot.first_sequence = ParseInteger<std::uint64_t>(GetValue(values, "snapshot_first_sequence"));
      snapshot.last_sequence = ParseInteger<std::uint64_t>(GetValue(values, "snapshot_last_sequence"));
      snapshot.source_sha256 = GetValue(values, "snapshot_source_sha256");
      snapshot.slam_healthy = ParseBool(GetValue(values, "snapshot_slam_healthy", "1"));
      snapshot.health_message = GetValue(values, "snapshot_health_message");
      job->snapshot = std::move(snapshot);
    }
    return job;
  }

  void RecoverInternal() {
    std::lock_guard<std::mutex> lock(mutex_);
    RecoverInternalLocked();
  }

  void RecoverInternalLocked() {
    if (!std::filesystem::is_directory(jobs_root_)) {
      return;
    }
    for (const auto& entry : std::filesystem::directory_iterator(jobs_root_)) {
      if (!entry.is_directory()) {
        continue;
      }
      auto job = LoadJob(entry.path() / "job.state");
      if (!job) {
        continue;
      }
      if (jobs_.find(job->status.job_id) != jobs_.end()) {
        continue;
      }
      const auto map_dir = store_.MapPath(job->request.map_id);
      const std::string current = CurrentVersion(map_dir);
      if (!IsTerminal(job->status.state)) {
        job->status.recovered = true;
        job->status.cancel_requested = false;
        if (job->status.version > 0 && current == VersionName(job->status.version) &&
            ManifestValid(job->status.version_dir)) {
          try {
            SyncCompatibilityView(map_dir, job->status.version_dir);
            job->status.compatibility_ready = true;
            job->status.compatibility_message.clear();
          } catch (const std::exception& exc) {
            job->status.compatibility_ready = false;
            job->status.compatibility_message = exc.what();
          }
          job->status.state = SaveJobState::kSucceeded;
          job->status.phase = SavePhase::kDone;
          job->status.progress = 1.0;
          job->status.message = "recovered committed SaveMap version";
          job->status.completed_at_ns = NowNs();
          std::error_code cleanup_error;
          std::filesystem::remove_all(WorkRoot(job->status.job_id), cleanup_error);
          std::filesystem::remove_all(job->status.capture_dir, cleanup_error);
        } else if (job->snapshot.has_value() &&
                   std::filesystem::is_regular_file(job->status.capture_dir / "map.pcd")) {
          job->status.state = SaveJobState::kQueued;
          job->status.phase = SavePhase::kValidate;
          job->status.progress = 0.05;
          job->status.message = "recovered interrupted SaveMap job";
          queue_.push_back(job->status.job_id);
        } else {
          job->status.state = SaveJobState::kWaitingSnapshot;
          job->status.phase = SavePhase::kCapture;
          job->status.progress = 0.0;
          job->status.message = "recovered job is waiting for snapshot";
        }
        const auto version_stage =
            map_dir / kVersionsDirectoryName / (".staging-" + job->status.job_id);
        std::error_code cleanup_error;
        std::filesystem::remove_all(version_stage, cleanup_error);
        if (job->status.version > 0 &&
            current != VersionName(job->status.version) &&
            std::filesystem::is_directory(job->status.version_dir)) {
          std::filesystem::remove_all(job->status.version_dir, cleanup_error);
          job->status.version = 0;
          job->status.version_dir.clear();
          job->status.manifest_path.clear();
        }
        job->status.updated_at_ns = NowNs();
      } else if (
          job->status.state == SaveJobState::kSucceeded && job->status.version > 0 &&
          current == VersionName(job->status.version) &&
          ManifestValid(job->status.version_dir) &&
          (!job->status.compatibility_ready ||
           ReadText(map_dir / "compatibility_version.txt") != current + "\n")) {
        job->status.recovered = true;
        try {
          SyncCompatibilityView(map_dir, job->status.version_dir);
          job->status.compatibility_ready = true;
          job->status.compatibility_message.clear();
          job->status.message = "recovered compatibility view for committed SaveMap version";
        } catch (const std::exception& exc) {
          job->status.compatibility_ready = false;
          job->status.compatibility_message = exc.what();
          job->status.message = "committed version is valid; compatibility recovery failed";
        }
        job->status.updated_at_ns = NowNs();
      }
      jobs_.emplace(job->status.job_id, job);
      PersistJobLocked(*job);
      if (job->status.recovered) {
        AppendEventLocked(*job, "RECOVERED");
      }
    }
  }

  bool CancelRequested(const std::string& job_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    return stopping_ || (found != jobs_.end() && found->second->status.cancel_requested);
  }

  bool StopRequested() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stopping_;
  }

  void SetStatus(
      const std::shared_ptr<Job>& job,
      SaveJobState state,
      SavePhase phase,
      double progress,
      const std::string& message,
      const std::string& reason = {}) {
    std::lock_guard<std::mutex> lock(mutex_);
    job->status.state = state;
    job->status.phase = phase;
    job->status.progress = progress;
    job->status.message = message;
    job->status.reason_code = reason;
    job->status.updated_at_ns = NowNs();
    if (IsTerminal(state)) {
      job->status.completed_at_ns = job->status.updated_at_ns;
    }
    PersistJobLocked(*job);
    AppendEventLocked(*job, std::string(SaveJobStateName(state)) + ":" + SavePhaseName(phase));
    cv_.notify_all();
  }

  void SetReports(
      const std::shared_ptr<Job>& job,
      const std::string* source_report,
      const std::string* artifact_report) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (source_report != nullptr) {
      job->status.source_report_json = *source_report;
    }
    if (artifact_report != nullptr) {
      job->status.artifact_report_json = *artifact_report;
    }
    job->status.updated_at_ns = NowNs();
    PersistJobLocked(*job);
  }

  bool BeforePhase(const std::shared_ptr<Job>& job, SavePhase phase) {
    if (hooks_.before_phase) {
      hooks_.before_phase(job->status.job_id, phase);
    }
    if (hooks_.forced_failure) {
      const auto failure = hooks_.forced_failure(job->status.job_id, phase);
      if (failure.has_value()) {
        SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0, *failure, "forced_stage_failure");
        return false;
      }
    }
    if (CancelRequested(job->status.job_id)) {
      if (StopRequested() && !job->status.cancel_requested) {
        SetStatus(job, SaveJobState::kQueued, phase, job->status.progress,
                  "SaveMap service stopped; job will recover on restart");
      } else {
        SetStatus(job, SaveJobState::kCancelled, SavePhase::kDone, 1.0, "SaveMap cancelled", "cancelled");
      }
      return false;
    }
    return true;
  }

  void WorkerLoop() {
    for (;;) {
      std::shared_ptr<Job> job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [&]() { return stopping_ || !queue_.empty(); });
        if (stopping_) {
          return;
        }
        const std::string job_id = queue_.front();
        queue_.pop_front();
        const auto found = jobs_.find(job_id);
        if (found == jobs_.end() || found->second->status.state != SaveJobState::kQueued) {
          continue;
        }
        job = found->second;
      }
      try {
        ProcessJob(job);
      } catch (const std::exception& exc) {
        std::lock_guard<std::mutex> lock(mutex_);
        job->status.state = SaveJobState::kFailed;
        job->status.phase = SavePhase::kDone;
        job->status.progress = 1.0;
        job->status.reason_code = "save_worker_failure";
        job->status.message = exc.what();
        job->status.updated_at_ns = NowNs();
        job->status.completed_at_ns = job->status.updated_at_ns;
        try {
          PersistJobLocked(*job);
          AppendEventLocked(*job, "WORKER_FAILURE");
        } catch (...) {
        }
        cv_.notify_all();
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        job->status.state = SaveJobState::kFailed;
        job->status.phase = SavePhase::kDone;
        job->status.progress = 1.0;
        job->status.reason_code = "save_worker_failure";
        job->status.message = "unknown SaveMap worker failure";
        job->status.updated_at_ns = NowNs();
        job->status.completed_at_ns = job->status.updated_at_ns;
        cv_.notify_all();
      }
    }
  }

  void ProcessJob(const std::shared_ptr<Job>& job) {
    if (!job->snapshot.has_value()) {
      SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                "queued SaveMap job has no snapshot", "snapshot_missing");
      return;
    }
    auto map_lock = MapLock::TryAcquire(
        store_.RootDir(), job->request.map_id, job->status.job_id);
    if (!map_lock.has_value()) {
      SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                "another process is saving this map", "map_save_in_progress");
      return;
    }

    try {
      SetStatus(job, SaveJobState::kRunning, SavePhase::kValidate, 0.10, "validating snapshot");
      if (!BeforePhase(job, SavePhase::kValidate)) return;
      const auto capture_pcd = job->status.capture_dir / "map.pcd";
      if (Sha256File(capture_pcd) != job->snapshot->source_sha256) {
        SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                  "captured map.pcd changed after acceptance", "snapshot_changed");
        return;
      }

      SetStatus(job, SaveJobState::kRunning, SavePhase::kProcessSource, 0.25,
                "filtering and optimizing source map");
      if (!BeforePhase(job, SavePhase::kProcessSource)) return;
      const auto work_root = WorkRoot(job->status.job_id);
      const auto source_work = work_root / "i";
      const auto store_root = work_root / "s";
      std::filesystem::remove_all(work_root);
      CopyTree(job->status.capture_dir, source_work, true);
      MapStore staging_store(MapStoreConfig{store_root});
      MapPipelineCore pipeline(staging_store);
      SourceCommitOptions source_options = job->request.source;
      source_options.cancel_requested = [this, id = job->status.job_id]() {
        return CancelRequested(id);
      };
      const auto source_result = pipeline.CommitSavedSourceJson(
          job->request.map_id, source_work, source_options);
      SetReports(job, &source_result, nullptr);
      if (!JsonSucceeded(source_result)) {
        if (CancelRequested(job->status.job_id)) {
          BeforePhase(job, SavePhase::kProcessSource);
        } else {
          SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                    "source processing failed", JsonReason(source_result, "source_processing_failed"));
        }
        return;
      }
      const auto staging_map = staging_store.MapPath(job->request.map_id);
      const auto semantic_source = job->status.capture_dir / kSemanticMapArtifactFilename;
      if (std::filesystem::is_regular_file(semantic_source)) {
        std::filesystem::copy_file(
            semantic_source,
            staging_map / kSemanticMapArtifactFilename,
            std::filesystem::copy_options::overwrite_existing);
      }

      SetStatus(job, SaveJobState::kRunning, SavePhase::kBuildArtifacts, 0.50,
                "building required map artifacts");
      if (!BeforePhase(job, SavePhase::kBuildArtifacts)) return;
      OctomapBuildOptions octomap_options = job->request.octomap;
      octomap_options.cancel_requested = [this, id = job->status.job_id]() {
        return CancelRequested(id);
      };
      std::string build_result;
      if (job->request.require.octomap) {
        build_result = pipeline.BuildNavigationPackageJson(
            job->request.map_id,
            octomap_options,
            job->request.require.esdf,
            job->request.require.traversability);
        SetReports(job, nullptr, &build_result);
        if (!JsonSucceeded(build_result)) {
          if (CancelRequested(job->status.job_id)) {
            BeforePhase(job, SavePhase::kBuildArtifacts);
          } else {
            SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                      "navigation package build failed",
                      JsonReason(build_result, "navigation_package_failed"));
          }
          return;
        }
      } else {
        if (job->request.require.occupancy) {
          build_result = pipeline.BuildOccupancySnapshotJson(job->request.map_id);
          SetReports(job, nullptr, &build_result);
          if (!JsonSucceeded(build_result)) {
            SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                      "occupancy build failed", JsonReason(build_result, "occupancy_build_failed"));
            return;
          }
        }
        if (job->request.require.esdf) {
          build_result = pipeline.BuildEsdfArtifactJson(job->request.map_id);
          SetReports(job, nullptr, &build_result);
          if (!JsonSucceeded(build_result)) {
            SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                      "ESDF build failed", JsonReason(build_result, "esdf_build_failed"));
            return;
          }
        }
        if (job->request.require.traversability) {
          build_result = pipeline.BuildTraversabilityArtifactJson(job->request.map_id);
          SetReports(job, nullptr, &build_result);
          if (!JsonSucceeded(build_result)) {
            SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                      "traversability build failed",
                      JsonReason(build_result, "traversability_build_failed"));
            return;
          }
        }
      }

      SetStatus(job, SaveJobState::kRunning, SavePhase::kVerify, 0.75,
                "verifying artifacts and hashes");
      if (!BeforePhase(job, SavePhase::kVerify)) return;
      const auto map_dir = store_.MapPath(job->request.map_id);
      const auto versions_dir = map_dir / kVersionsDirectoryName;
      std::filesystem::create_directories(versions_dir);
      const std::int64_t version = NextVersion(map_dir);
      const std::string version_name = VersionName(version);
      const auto version_stage = versions_dir / (".staging-" + job->status.job_id);
      const auto version_dir = versions_dir / version_name;
      std::filesystem::remove_all(version_stage);
      CopyTree(staging_map, version_stage, true);
      std::uint64_t point_count = 0U;
      VerifyRequiredArtifacts(version_stage, job->request, &point_count);
      auto hashes = HashArtifacts(version_stage);
      job->status.version = version;
      job->status.version_dir = version_dir;
      job->status.manifest_path = version_dir / kManifestFilename;
      WriteTextAtomic(
          version_stage / kArtifactChecksumsFilename,
          BuildArtifactChecksums(hashes));
      const auto manifest = BuildManifest(
          job->request, *job->snapshot, job->status, point_count, hashes);
      WriteTextAtomic(version_stage / kManifestFilename, manifest);
      WriteTextAtomic(
          version_stage / kManifestHashFilename,
          Sha256File(version_stage / kManifestFilename) + "\n" +
              Sha256File(version_stage / kArtifactChecksumsFilename) + "\n");
      for (const auto& hash : hashes) {
        if (Sha256File(version_stage / hash.first) != hash.second) {
          throw std::runtime_error("artifact changed during verification: " + hash.first);
        }
      }
      std::string integrity_error;
      if (!VerifyMapVersion(version_stage, &integrity_error)) {
        throw std::runtime_error("map version integrity check failed: " + integrity_error);
      }
      SyncTree(version_stage);

      SetStatus(job, SaveJobState::kRunning, SavePhase::kCommit, 0.90,
                "publishing immutable map version");
      if (!BeforePhase(job, SavePhase::kCommit)) {
        std::filesystem::remove_all(version_stage);
        return;
      }
      if (std::filesystem::exists(version_dir)) {
        throw std::runtime_error("target map version already exists");
      }
      AppendEvent(job, "VERSION_PREPARED");
      std::filesystem::rename(version_stage, version_dir);
      SyncPath(versions_dir, true);
      WriteTextAtomic(map_dir / kCurrentVersionFilename, version_name + "\n");
      AppendEvent(job, "VERSION_COMMITTED");
      try {
        SyncCompatibilityView(map_dir, version_dir);
        job->status.compatibility_ready = true;
        job->status.compatibility_message.clear();
      } catch (const std::exception& exc) {
        job->status.compatibility_ready = false;
        job->status.compatibility_message = exc.what();
      }
      if (job->request.activate_on_success) {
        const auto activated =
            store_.SetActiveMapWhileLocked(job->request.map_id, true, *map_lock);
        if (!activated.ok) {
          throw std::runtime_error("version committed but activation failed: " + activated.message);
        }
      }
      std::error_code cleanup_error;
      std::filesystem::remove_all(WorkRoot(job->status.job_id), cleanup_error);
      std::filesystem::remove_all(job->status.capture_dir, cleanup_error);
      SetStatus(job, SaveJobState::kSucceeded, SavePhase::kDone, 1.0,
                job->status.compatibility_ready
                    ? "SaveMap version committed and verified"
                    : "SaveMap version committed; compatibility view is degraded");
    } catch (const std::exception& exc) {
      const auto map_dir = store_.MapPath(job->request.map_id);
      if (job->status.version > 0 &&
          CurrentVersion(map_dir) == VersionName(job->status.version) &&
          ManifestValid(job->status.version_dir)) {
        try {
          SyncCompatibilityView(map_dir, job->status.version_dir);
          job->status.compatibility_ready = true;
          job->status.compatibility_message.clear();
        } catch (const std::exception& compatibility_error) {
          job->status.compatibility_ready = false;
          job->status.compatibility_message = compatibility_error.what();
        }
        std::error_code cleanup_error;
        std::filesystem::remove_all(WorkRoot(job->status.job_id), cleanup_error);
        std::filesystem::remove_all(job->status.capture_dir, cleanup_error);
        if (job->request.activate_on_success &&
            store_.ActiveMapId() != job->request.map_id) {
          SetStatus(
              job,
              SaveJobState::kFailed,
              SavePhase::kDone,
              1.0,
              std::string("map version committed but requested activation failed: ") +
                  exc.what(),
              "activation_failed_after_commit");
        } else {
          SetStatus(
              job,
              SaveJobState::kSucceeded,
              SavePhase::kDone,
              1.0,
              std::string("recovered committed version after finalization error: ") +
                  exc.what());
        }
      } else if (CancelRequested(job->status.job_id)) {
        BeforePhase(job, job->status.phase);
      } else {
        SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                  exc.what(), "save_map_failed");
      }
    }
  }

  MapStore& store_;
  SaveMapHooks hooks_;
  std::filesystem::path jobs_root_;
  std::filesystem::path engine_lock_path_;
  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
  std::unordered_map<std::string, std::shared_ptr<Job>> jobs_;
  std::deque<std::string> queue_;
  bool stopping_{false};
  bool owns_engine_lock_{false};
  std::thread worker_;
};

SaveMapEngine::SaveMapEngine(MapStore& store, SaveMapHooks hooks)
    : impl_(std::make_unique<Impl>(store, std::move(hooks))) {}

SaveMapEngine::~SaveMapEngine() = default;

SaveMapResult SaveMapEngine::Begin(const SaveMapRequest& request) {
  return impl_->Begin(request);
}

SaveMapResult SaveMapEngine::ProvideSnapshot(
    const std::string& job_id,
    const MapSnapshot& snapshot) {
  return impl_->ProvideSnapshot(job_id, snapshot);
}

SaveMapResult SaveMapEngine::Cancel(const std::string& job_id) {
  return impl_->Cancel(job_id);
}

SaveMapResult SaveMapEngine::Retry(const std::string& job_id) {
  return impl_->Retry(job_id);
}

std::optional<SaveMapStatus> SaveMapEngine::GetStatus(const std::string& job_id) const {
  return impl_->GetStatus(job_id);
}

std::vector<SaveMapStatus> SaveMapEngine::ListStatuses(std::size_t limit) const {
  return impl_->ListStatuses(limit);
}

std::optional<SaveMapStatus> SaveMapEngine::Wait(
    const std::string& job_id,
    std::chrono::milliseconds timeout) const {
  return impl_->Wait(job_id, timeout);
}

void SaveMapEngine::Recover() {
  impl_->Recover();
}

std::string SaveMapEngine::BeginJson(const SaveMapRequest& request) {
  const auto result = Begin(request);
  return "{\"action\":\"save_map_begin\",\"accepted\":" +
      std::string(result.accepted ? "true" : "false") +
      ",\"replayed\":" + (result.replayed ? "true" : "false") +
      ",\"reason_code\":" + JsonString(result.reason_code) +
      ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::ProvideSnapshotJson(
    const std::string& job_id,
    const MapSnapshot& snapshot) {
  const auto result = ProvideSnapshot(job_id, snapshot);
  return "{\"action\":\"save_map_snapshot\",\"accepted\":" +
      std::string(result.accepted ? "true" : "false") +
      ",\"replayed\":" + (result.replayed ? "true" : "false") +
      ",\"reason_code\":" + JsonString(result.reason_code) +
      ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::CancelJson(const std::string& job_id) {
  const auto result = Cancel(job_id);
  return "{\"action\":\"save_map_cancel\",\"accepted\":" +
      std::string(result.accepted ? "true" : "false") +
      ",\"replayed\":" + (result.replayed ? "true" : "false") +
      ",\"reason_code\":" + JsonString(result.reason_code) +
      ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::RetryJson(const std::string& job_id) {
  const auto result = Retry(job_id);
  return "{\"action\":\"save_map_retry\",\"accepted\":" +
      std::string(result.accepted ? "true" : "false") +
      ",\"replayed\":" + (result.replayed ? "true" : "false") +
      ",\"reason_code\":" + JsonString(result.reason_code) +
      ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::GetStatusJson(const std::string& job_id) const {
  const auto status = GetStatus(job_id);
  if (!status.has_value()) {
    return "{\"action\":\"save_map_status\",\"success\":false,"
        "\"reason_code\":\"job_not_found\",\"job_id\":" + JsonString(job_id) + "}";
  }
  return "{\"action\":\"save_map_status\",\"success\":true,\"status\":" +
      SaveMapStatusJson(*status) + "}";
}

std::string SaveMapEngine::ListStatusesJson(std::size_t limit) const {
  const auto statuses = ListStatuses(limit);
  std::ostringstream out;
  out << "{\"action\":\"list_save_map_jobs\",\"success\":true,\"jobs\":[";
  for (std::size_t i = 0; i < statuses.size(); ++i) {
    if (i > 0U) {
      out << ',';
    }
    auto summary = statuses[i];
    summary.source_report_json.clear();
    summary.artifact_report_json.clear();
    out << SaveMapStatusJson(summary);
  }
  out << "],\"count\":" << statuses.size() << "}";
  return out.str();
}

std::string SaveMapEngine::ListVersionsJson(const std::string& map_id) {
  return impl_->ListVersionsJson(map_id);
}

std::string SaveMapEngine::RollbackVersionJson(
    const std::string& map_id,
    std::int64_t version) {
  return impl_->RollbackVersionJson(map_id, version);
}

}  // namespace lingtu::maps
