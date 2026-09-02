#include "lingtu/maps/save.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "lingtu/maps/build/process.hpp"
#include "lingtu/maps/json.hpp"
#include "lingtu/maps/lock.hpp"
#include "lingtu/maps/semantic_map_persistence.hpp"

#if defined(_WIN32)
#define NOMINMAX
#include <windows.h>
#else
#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

using Clock = std::chrono::system_clock;

constexpr const char *kJobSchemaVersion = "4";
constexpr const char *kJobsDirectoryName = ".save_jobs";

std::int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch())
      .count();
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

std::string JsonEscape(const std::string &value) {
  std::ostringstream out;
  for (const unsigned char ch : value) {
    switch (ch) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\b':
        out << "\\b";
        break;
      case '\f':
        out << "\\f";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
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

std::string JsonString(const std::string &value) {
  return "\"" + JsonEscape(value) + "\"";
}

std::string PercentEncode(const std::string &value) {
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
  if (ch >= '0' && ch <= '9')
    return ch - '0';
  if (ch >= 'A' && ch <= 'F')
    return ch - 'A' + 10;
  if (ch >= 'a' && ch <= 'f')
    return ch - 'a' + 10;
  return -1;
}

std::string PercentDecode(const std::string &value) {
  std::string out;
  out.reserve(value.size());
  for (std::size_t i = 0; i < value.size(); ++i) {
    if (value[i] == '%') {
      if (i + 2U >= value.size()) {
        throw std::runtime_error("truncated percent encoding in persisted state");
      }
      const int high = HexValue(value[i + 1U]);
      const int low = HexValue(value[i + 2U]);
      if (high < 0 || low < 0) {
        throw std::runtime_error("invalid percent encoding in persisted state");
      }
      out.push_back(static_cast<char>((high << 4) | low));
      i += 2U;
      continue;
    }
    out.push_back(value[i]);
  }
  return out;
}

bool ParseBool(const std::string &value) {
  if (value == "1" || value == "true") {
    return true;
  }
  if (value.empty() || value == "0" || value == "false") {
    return false;
  }
  throw std::runtime_error("invalid persisted boolean");
}

double ParseDouble(const std::string &value, double fallback = 0.0) {
  if (value.empty()) {
    return fallback;
  }
  try {
    std::size_t consumed = 0U;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size() || !std::isfinite(parsed)) {
      throw std::runtime_error("invalid persisted floating-point value");
    }
    return parsed;
  } catch (...) {
    throw std::runtime_error("invalid persisted floating-point value");
  }
}

template <typename T>
T ParseIntegerStrict(const std::string &value, T fallback = 0) {
  if (value.empty()) {
    return fallback;
  }
  T parsed{};
  const auto result = std::from_chars(value.data(), value.data() + value.size(), parsed);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) {
    throw std::runtime_error("invalid persisted integer");
  }
  return parsed;
}

bool IsTerminal(SaveJobState state) {
  return state == SaveJobState::kSucceeded || state == SaveJobState::kFailed ||
         state == SaveJobState::kCancelled;
}

SaveJobState ParseState(const std::string &value) {
  if (value == "WAITING_SNAPSHOT")
    return SaveJobState::kWaitingSnapshot;
  if (value == "QUEUED")
    return SaveJobState::kQueued;
  if (value == "RUNNING")
    return SaveJobState::kRunning;
  if (value == "SUCCEEDED")
    return SaveJobState::kSucceeded;
  if (value == "FAILED")
    return SaveJobState::kFailed;
  if (value == "CANCELLED")
    return SaveJobState::kCancelled;
  throw std::runtime_error("invalid persisted SaveMap state");
}

SavePhase ParsePhase(const std::string &value) {
  if (value == "CAPTURE")
    return SavePhase::kCapture;
  if (value == "VALIDATE")
    return SavePhase::kValidate;
  if (value == "OPTIMIZE_SOURCE")
    return SavePhase::kOptimizeSource;
  if (value == "PROCESS_SOURCE")
    return SavePhase::kProcessSource;
  if (value == "BUILD_ARTIFACTS")
    return SavePhase::kBuildArtifacts;
  if (value == "VERIFY")
    return SavePhase::kVerify;
  if (value == "COMMIT")
    return SavePhase::kCommit;
  if (value == "DONE")
    return SavePhase::kDone;
  throw std::runtime_error("invalid persisted SaveMap phase");
}

bool IsBasename(const std::string &value) {
  if (value.empty() || value == "." || value == "..") {
    return false;
  }
  const std::filesystem::path path(value);
  return path == path.filename();
}

std::string ShellQuote(const std::string &value) {
#if defined(_WIN32)
  std::string out = "\"";
  for (const char ch : value) {
    out += ch == '"' ? "\\\"" : std::string(1, ch);
  }
  return out + '"';
#else
  std::string out = "'";
  for (const char ch : value) {
    out += ch == '\'' ? "'\\''" : std::string(1, ch);
  }
  return out + '\'';
#endif
}

std::string NormalizeShellCommand(std::string command) {
#if defined(_WIN32)
  if (!command.empty() && command.front() == '"') {
    return "\"" + command + "\"";
  }
#endif
  return command;
}

std::string ResolvePgoExecutable(const PgoOptions &options) {
  if (!options.executable.empty()) {
    return options.executable;
  }
  if (const char *env = std::getenv("LINGTU_PGO_BIN"); env != nullptr && *env != '\0') {
    return env;
  }
  const auto cwd = std::filesystem::current_path();
  for (const auto &candidate : {
           cwd / "build" / "map_opt" / "Release" / "lt_pgo.exe",
           cwd / "build" / "map_opt" / "lt_pgo.exe",
           cwd / "build" / "map_opt" / "lt_pgo",
           std::filesystem::path("/opt/lingtu/current/bin/lt_pgo"),
       }) {
    std::error_code error;
    if (std::filesystem::is_regular_file(candidate, error) && !error) {
      return candidate.string();
    }
  }
  return "lt_pgo";
}

std::string BuildPgoCommand(const PgoOptions &options, const std::filesystem::path &source,
                            const std::filesystem::path &output,
                            const std::optional<std::filesystem::path> &constraints) {
  std::string command = ShellQuote(ResolvePgoExecutable(options)) + " --map " +
                        ShellQuote(std::filesystem::absolute(source).string()) + " --out " +
                        ShellQuote(std::filesystem::absolute(output).string());
  if (constraints.has_value()) {
    command += " --constraints " + ShellQuote(std::filesystem::absolute(*constraints).string());
  } else {
    command += " --auto-constraints";
  }
  return NormalizeShellCommand(std::move(command));
}

bool IsValidJobId(const std::string &value) {
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

bool IsValidProductSessionId(const std::string &value) {
  if (value.empty() || value.size() > 63U ||
      !((value.front() >= 'a' && value.front() <= 'z') ||
        (value.front() >= 'A' && value.front() <= 'Z') ||
        (value.front() >= '0' && value.front() <= '9'))) {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](const unsigned char ch) {
    return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') ||
           ch == '_' || ch == '-' || ch == '.';
  });
}

std::string RequestCanonical(const SaveMapRequest &request) {
  std::ostringstream out;
  out << request.request_id << '\n'
      << request.map_id << '\n'
      << request.require.occupancy << request.require.octomap << request.require.esdf
      << request.require.traversability << request.require.semantic << '\n'
      << request.pgo.executable << '\n'
      << request.pgo.constraints_file << '\n'
      << std::setprecision(17) << request.pgo.timeout_sec << '\n'
      << request.source.voxel_size << '\n'
      << request.source.dynamic_filter_enabled << request.source.dynamic_filter_required << '\n'
      << request.source.dynamic_filter_command << '\n'
      << request.source.dynamic_filter_timeout_sec << '\n'
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
      << request.activate_on_success << request.require_slam_healthy
      << request.allow_unverified_snapshot << '\n'
      << request.minimum_point_count;
  if (!request.product_session_id.empty()) {
    out << '\n' << request.product_session_id;
  }
  return out.str();
}

bool HasSnapshotReceipt(const MapSnapshot &snapshot) {
  return !snapshot.slam_boot_id.empty() || !snapshot.product_session_id.empty() ||
         snapshot.reset_epoch != 0U || snapshot.observation_sequence != 0U ||
         snapshot.source_point_count != 0U;
}

bool HasCompleteSnapshotReceipt(const MapSnapshot &snapshot) {
  return !snapshot.slam_boot_id.empty() && snapshot.reset_epoch > 0U &&
         snapshot.observation_sequence > 0U && snapshot.source_point_count > 0U;
}

void SyncPath(const std::filesystem::path &path, bool directory) {
#if defined(_WIN32)
  const DWORD flags = directory ? FILE_FLAG_BACKUP_SEMANTICS : FILE_ATTRIBUTE_NORMAL;
  HANDLE handle = CreateFileW(path.wstring().c_str(), GENERIC_READ,
                              FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                              OPEN_EXISTING, flags, nullptr);
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

void AtomicReplace(const std::filesystem::path &source, const std::filesystem::path &target) {
#if defined(_WIN32)
  if (MoveFileExW(source.wstring().c_str(), target.wstring().c_str(),
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

void WriteTextAtomic(const std::filesystem::path &path, const std::string &value) {
  std::filesystem::create_directories(path.parent_path());
  const auto temp =
      path.parent_path() / (path.filename().string() + ".tmp-" +
                            std::to_string(CurrentProcessId()) + "-" + std::to_string(NowNs()));
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

std::string ReadText(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return {};
  }
  return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

std::unordered_map<std::string, std::string> ReadKeyValues(const std::filesystem::path &path) {
  std::unordered_map<std::string, std::string> values;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    throw std::runtime_error("failed to read persisted state: " + path.string());
  }
  std::string line;
  while (std::getline(file, line)) {
    const auto equals = line.find('=');
    if (equals == std::string::npos || equals == 0U) {
      throw std::runtime_error("malformed persisted state: " + path.string());
    }
    const auto key = line.substr(0, equals);
    const auto inserted = values.emplace(key, PercentDecode(line.substr(equals + 1U)));
    if (!inserted.second) {
      throw std::runtime_error("duplicate persisted state key: " + key);
    }
  }
  if (file.bad() || values.empty()) {
    throw std::runtime_error("failed to read complete persisted state: " + path.string());
  }
  return values;
}

bool SameNormalizedPath(const std::filesystem::path &left, const std::filesystem::path &right) {
  return std::filesystem::absolute(left).lexically_normal() ==
         std::filesystem::absolute(right).lexically_normal();
}

std::string GetValue(const std::unordered_map<std::string, std::string> &values,
                     const std::string &key, const std::string &fallback = {}) {
  const auto found = values.find(key);
  return found == values.end() ? fallback : found->second;
}

void AppendKey(std::ostringstream &out, const std::string &key, const std::string &value) {
  out << key << '=' << PercentEncode(value) << '\n';
}

std::string PreciseDouble(double value) {
  std::ostringstream out;
  out << std::setprecision(17) << value;
  return out.str();
}

void CopyTree(const std::filesystem::path &source, const std::filesystem::path &target,
              bool exclude_control_dirs) {
  if (!std::filesystem::is_directory(source)) {
    throw std::runtime_error("source directory not found: " + source.string());
  }
  std::filesystem::create_directories(target);
  for (std::filesystem::recursive_directory_iterator it(source), end; it != end; ++it) {
    const auto relative = std::filesystem::relative(it->path(), source);
    const auto first =
        relative.begin() == relative.end() ? std::string{} : relative.begin()->string();
    if (exclude_control_dirs && (first == ".builds" || first == ".save_lock")) {
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
      std::filesystem::copy_file(it->path(), destination,
                                 std::filesystem::copy_options::overwrite_existing);
    }
  }
}

void SyncTree(const std::filesystem::path &root) {
  std::vector<std::filesystem::path> directories;
  directories.push_back(root);
  for (const auto &entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      SyncPath(entry.path(), false);
    } else if (entry.is_directory()) {
      directories.push_back(entry.path());
    }
  }
  std::sort(directories.begin(), directories.end(), [](const auto &left, const auto &right) {
    return left.native().size() > right.native().size();
  });
  for (const auto &directory : directories) {
    SyncPath(directory, true);
  }
}

std::uint64_t TreeBytes(const std::filesystem::path &root) {
  std::uint64_t bytes = 0U;
  if (!std::filesystem::is_directory(root)) {
    return bytes;
  }
  for (const auto &entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file()) {
      bytes += static_cast<std::uint64_t>(entry.file_size());
    }
  }
  return bytes;
}

bool JsonSucceeded(const std::string &value) {
  return JsonObjectBoolAtPath(value, {"success"}) == true;
}

std::string JsonReason(const std::string &value, const std::string &fallback) {
  const auto reason = JsonObjectStringAtPath(value, {"reason_code"});
  return reason.has_value() ? *reason : fallback;
}

bool IsNonEmptyRegularFile(const std::filesystem::path &path) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error) || error) {
    return false;
  }
  return std::filesystem::file_size(path, error) > 0U && !error;
}

bool JsonCount(const std::string &json, const char *key, std::size_t *value,
               bool require_positive) {
  const auto number = JsonObjectNumberAtPath(json, {key});
  if (!number.has_value() || !std::isfinite(*number) || *number < 0.0 ||
      std::floor(*number) != *number ||
      *number > static_cast<double>(std::numeric_limits<std::size_t>::max()) ||
      (require_positive && *number == 0.0)) {
    return false;
  }
  *value = static_cast<std::size_t>(*number);
  return true;
}

bool ReadPatchManifest(const std::filesystem::path &path, std::size_t *patch_count) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }
  const std::array<const char *, 6> expected_keys = {
      "LINGTU_PATCH_BUNDLE_V1", "complete",      "dropped_count",
      "first_sequence",         "last_sequence", "patch_count",
  };
  std::array<std::uint64_t, 5> values{};
  std::string line;
  if (!std::getline(input, line)) {
    return false;
  }
  if (!line.empty() && line.back() == '\r')
    line.pop_back();
  if (line != expected_keys[0]) {
    return false;
  }
  for (std::size_t index = 1; index < expected_keys.size(); ++index) {
    if (!std::getline(input, line)) {
      return false;
    }
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    std::istringstream row(line);
    std::string key;
    std::string token;
    std::string extra;
    std::uint64_t parsed = 0U;
    if (!(row >> key >> token) || row >> extra || key != expected_keys[index] || token.empty() ||
        token.front() == '-') {
      return false;
    }
    const auto result = std::from_chars(token.data(), token.data() + token.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != token.data() + token.size()) {
      return false;
    }
    values[index - 1] = parsed;
  }
  if (std::getline(input, line)) {
    return false;
  }
  if (values[0] != 1U || values[1] != 0U || values[4] == 0U || values[2] != 0U ||
      values[3] != values[4] - 1U || values[4] > std::numeric_limits<std::size_t>::max()) {
    return false;
  }
  *patch_count = static_cast<std::size_t>(values[4]);
  return true;
}

bool HasCompletePatchBundle(const std::filesystem::path &source) {
  const auto patches = source / "patches";
  if (!IsNonEmptyRegularFile(source / "map.pcd") || !IsNonEmptyRegularFile(source / "poses.txt") ||
      !IsNonEmptyRegularFile(source / "patch_bundle.manifest") ||
      !std::filesystem::is_directory(patches)) {
    return false;
  }
  std::size_t manifest_patch_count = 0U;
  if (!ReadPatchManifest(source / "patch_bundle.manifest", &manifest_patch_count)) {
    return false;
  }
  std::set<std::string> disk_patches;
  for (const auto &entry : std::filesystem::directory_iterator(patches)) {
    if (entry.is_regular_file() && entry.path().extension() == ".pcd" &&
        IsNonEmptyRegularFile(entry.path())) {
      disk_patches.insert(entry.path().filename().string());
    }
  }
  std::set<std::string> pose_patches;
  std::ifstream poses(source / "poses.txt", std::ios::binary);
  std::string line;
  while (std::getline(poses, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    std::istringstream row(line);
    std::string filename;
    std::array<double, 7> pose{};
    std::string extra;
    if (!(row >> filename) || !IsBasename(filename) ||
        std::filesystem::path(filename).extension() != ".pcd") {
      return false;
    }
    for (double &value : pose) {
      if (!(row >> value) || !std::isfinite(value))
        return false;
    }
    if (row >> extra || !pose_patches.insert(filename).second)
      return false;
  }
  return !pose_patches.empty() && pose_patches == disk_patches &&
         pose_patches.size() == manifest_patch_count;
}

struct PgoCliResult {
  bool ok{false};
  bool performed{false};
  std::string code;
  std::string message;
  std::size_t pose_count{0U};
  std::size_t factor_count{0U};
  std::size_t sequential_count{0U};
  std::size_t loop_count{0U};
};

std::optional<PgoCliResult> ParsePgoCliResult(const std::string &value) {
  PgoCliResult result;
  const auto ok = JsonObjectBoolAtPath(value, {"ok"});
  const auto performed = JsonObjectBoolAtPath(value, {"performed"});
  const auto code = JsonObjectStringAtPath(value, {"code"});
  const auto message = JsonObjectStringAtPath(value, {"message"});
  if (!IsValidJsonObject(value) || !ok.has_value() || !performed.has_value() || !code.has_value() ||
      code->empty() || !message.has_value() ||
      !JsonCount(value, "pose_count", &result.pose_count, false) ||
      !JsonCount(value, "factor_count", &result.factor_count, false) ||
      !JsonCount(value, "sequential_count", &result.sequential_count, false) ||
      !JsonCount(value, "loop_count", &result.loop_count, false) ||
      result.factor_count != result.sequential_count + result.loop_count) {
    return std::nullopt;
  }
  result.ok = *ok;
  result.performed = *performed;
  result.code = *code;
  result.message = *message;
  return result;
}

void WritePgoNotPerformedReport(const std::filesystem::path &source, const PgoCliResult &result) {
  WriteTextAtomic(source / "map_optimization.json",
                  "{\"schema\":\"lingtu.map_optimization.v1\",\"success\":true,"
                  "\"performed\":false,\"code\":" +
                      JsonString(result.code) + ",\"message\":" + JsonString(result.message) +
                      ",\"pose_count\":" + std::to_string(result.pose_count) +
                      ",\"factor_count\":" + std::to_string(result.factor_count) +
                      ",\"sequential_count\":" + std::to_string(result.sequential_count) +
                      ",\"loop_count\":" + std::to_string(result.loop_count) + "}\n");
}

bool IsAllowedAutoPgoSkip(const PgoCliResult &result) {
  if (!result.ok || result.performed || result.loop_count != 0U)
    return false;
  if (result.code == "insufficient_keyframes") {
    return result.pose_count < 2U && result.sequential_count == 0U && result.factor_count == 0U;
  }
  if (result.code == "sequential_chain_incomplete") {
    return result.pose_count >= 2U && result.sequential_count + 1U < result.pose_count &&
           result.factor_count == result.sequential_count;
  }
  if (result.code == "no_verified_loops") {
    return result.pose_count >= 2U && result.sequential_count + 1U == result.pose_count &&
           result.factor_count == result.sequential_count;
  }
  return false;
}

bool IsValidPerformedPgoResult(const PgoCliResult &result, bool automatic) {
  if (!result.ok || !result.performed || result.code != "optimized" || result.pose_count == 0U ||
      result.factor_count == 0U) {
    return false;
  }
  if (!automatic)
    return true;
  return result.pose_count >= 2U && result.sequential_count + 1U == result.pose_count &&
         result.loop_count >= 1U;
}

bool HasPgoOutputBundle(const std::filesystem::path &output, const PgoCliResult &cli) {
  const auto patches = output / "patches";
  if (!IsNonEmptyRegularFile(output / "map.pcd") || !IsNonEmptyRegularFile(output / "poses.txt") ||
      !IsNonEmptyRegularFile(output / "patch_bundle.manifest") ||
      !IsNonEmptyRegularFile(output / "map_optimization.json") ||
      !std::filesystem::is_directory(patches)) {
    return false;
  }
  const auto report = ReadText(output / "map_optimization.json");
  std::size_t report_pose_count = 0U;
  std::size_t report_patch_count = 0U;
  std::size_t report_factor_count = 0U;
  if (!IsValidJsonObject(report) ||
      JsonObjectStringAtPath(report, {"schema"}) != "lingtu.map_optimization.v1" ||
      JsonObjectBoolAtPath(report, {"success"}) != true ||
      JsonObjectStringAtPath(report, {"code"}) != "optimized" ||
      JsonObjectBoolAtPath(report, {"converged"}) != true ||
      !JsonCount(report, "pose_count", &report_pose_count, true) ||
      !JsonCount(report, "patch_count", &report_patch_count, true) ||
      !JsonCount(report, "factor_count", &report_factor_count, true)) {
    return false;
  }

  std::size_t manifest_patch_count = 0U;
  if (!ReadPatchManifest(output / "patch_bundle.manifest", &manifest_patch_count)) {
    return false;
  }

  std::set<std::string> disk_patches;
  for (const auto &entry : std::filesystem::directory_iterator(patches)) {
    if (entry.is_regular_file() && entry.path().extension() == ".pcd" &&
        IsNonEmptyRegularFile(entry.path())) {
      disk_patches.insert(entry.path().filename().string());
    }
  }
  std::set<std::string> pose_patches;
  std::ifstream poses(output / "poses.txt", std::ios::binary);
  std::string line;
  std::size_t pose_count = 0U;
  while (std::getline(poses, line)) {
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    if (line.empty()) {
      return false;
    }
    std::istringstream row(line);
    std::string filename;
    std::array<double, 7> pose{};
    std::string extra;
    if (!(row >> filename) || !IsBasename(filename) ||
        std::filesystem::path(filename).extension() != ".pcd") {
      return false;
    }
    for (double &value : pose) {
      if (!(row >> value) || !std::isfinite(value)) {
        return false;
      }
    }
    if (row >> extra || !pose_patches.insert(filename).second) {
      return false;
    }
    ++pose_count;
  }
  return pose_count > 0U && pose_patches == disk_patches && pose_count == disk_patches.size() &&
         pose_count == manifest_patch_count && pose_count == report_pose_count &&
         pose_count == report_patch_count && pose_count == cli.pose_count &&
         report_factor_count == cli.factor_count;
}

void VerifyRequiredArtifacts(const std::filesystem::path &dir, const SaveMapRequest &request) {
  const auto require_file = [&](const char *filename) {
    const auto path = dir / filename;
    if (!IsNonEmptyRegularFile(path)) {
      throw std::runtime_error(std::string("required artifact missing: ") + filename);
    }
  };
  require_file("map.pcd");
  if (request.require.occupancy || request.require.octomap)
    require_file("occupancy.npz");
  if (request.require.octomap) {
    require_file("octomap.ot");
    require_file("metadata.json");
    const auto metadata = ReadText(dir / "metadata.json");
    const auto frame_id = JsonObjectStringAtPath(metadata, {"frame_id"});
    if (!IsValidJsonObject(metadata) || !frame_id.has_value() || *frame_id != "map") {
      throw std::runtime_error("metadata.json is invalid or does not declare frame_id map");
    }
  }
  if (request.require.esdf || request.require.traversability)
    require_file("esdf.npz");
  if (request.require.traversability)
    require_file("traversability.npz");
  if (request.require.semantic) {
    std::string error;
    if (!ValidateSemanticMapBinary(dir / kSemanticMapArtifactFilename, &error)) {
      throw std::runtime_error(error.empty() ? "semantic_map.bin validation failed" : error);
    }
  }
}

std::filesystem::path CommitStage(const std::filesystem::path &root, const std::string &job_id) {
  return root / (".save-staging-" + job_id);
}

std::filesystem::path CommitBackup(const std::filesystem::path &root, const std::string &map_id,
                                   const std::string &job_id) {
  return root / (".save-backup-" + map_id + "-" + job_id);
}

}  // namespace

const char *SaveJobStateName(SaveJobState state) {
  switch (state) {
    case SaveJobState::kWaitingSnapshot:
      return "WAITING_SNAPSHOT";
    case SaveJobState::kQueued:
      return "QUEUED";
    case SaveJobState::kRunning:
      return "RUNNING";
    case SaveJobState::kSucceeded:
      return "SUCCEEDED";
    case SaveJobState::kFailed:
      return "FAILED";
    case SaveJobState::kCancelled:
      return "CANCELLED";
  }
  return "FAILED";
}

const char *SavePhaseName(SavePhase phase) {
  switch (phase) {
    case SavePhase::kCapture:
      return "CAPTURE";
    case SavePhase::kValidate:
      return "VALIDATE";
    case SavePhase::kOptimizeSource:
      return "OPTIMIZE_SOURCE";
    case SavePhase::kProcessSource:
      return "PROCESS_SOURCE";
    case SavePhase::kBuildArtifacts:
      return "BUILD_ARTIFACTS";
    case SavePhase::kVerify:
      return "VERIFY";
    case SavePhase::kCommit:
      return "COMMIT";
    case SavePhase::kDone:
      return "DONE";
  }
  return "DONE";
}

std::string SaveMapStatusJson(const SaveMapStatus &status) {
  return "{"
         "\"job_id\":" +
         JsonString(status.job_id) +
         ","
         "\"request_id\":" +
         JsonString(status.request_id) +
         ","
         "\"map_id\":" +
         JsonString(status.map_id) +
         ","
         "\"product_session_id\":" +
         JsonString(status.product_session_id) +
         ","
         "\"state\":" +
         JsonString(SaveJobStateName(status.state)) +
         ","
         "\"phase\":" +
         JsonString(SavePhaseName(status.phase)) +
         ","
         "\"progress\":" +
         std::to_string(status.progress) +
         ","
         "\"message\":" +
         JsonString(status.message) +
         ","
         "\"reason_code\":" +
         JsonString(status.reason_code) +
         ","
         "\"capture_dir\":" +
         JsonString(status.capture_dir.string()) +
         ","
         "\"map_dir\":" +
         JsonString(status.map_dir.string()) +
         ","
         "\"source_report\":" +
         (status.source_report_json.empty() ? "{}" : status.source_report_json) +
         ","
         "\"artifact_report\":" +
         (status.artifact_report_json.empty() ? "{}" : status.artifact_report_json) +
         ","
         "\"created_at_ns\":" +
         std::to_string(status.created_at_ns) +
         ","
         "\"updated_at_ns\":" +
         std::to_string(status.updated_at_ns) +
         ","
         "\"completed_at_ns\":" +
         std::to_string(status.completed_at_ns) +
         ","
         "\"activation_requested\":" +
         (status.activation_requested ? "true" : "false") +
         ","
         "\"activation_succeeded\":" +
         (status.activation_succeeded ? "true" : "false") +
         ","
         "\"cancel_requested\":" +
         (status.cancel_requested ? "true" : "false") +
         ","
         "\"recovered\":" +
         (status.recovered ? "true" : "false") +
         ","
         "\"replayed\":" +
         (status.replayed ? "true" : "false") + "}";
}

class SaveMapEngine::Impl {
 public:
  Impl(MapStore &store, SaveMapHooks hooks)
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
    bool map_committed{false};
    bool accepting_snapshot{false};
    std::string loaded_schema{kJobSchemaVersion};
  };

  SaveMapResult Begin(const SaveMapRequest &raw_request) {
    SaveMapRequest request = raw_request;
    request.map_id = MapStore::NormalizeMapId(request.map_id);
    if (!IsValidJobId(request.request_id)) {
      throw std::invalid_argument("request_id must contain only letters, digits, '_' or '-'");
    }
    if (request.minimum_point_count == 0U) {
      throw std::invalid_argument("minimum_point_count must be positive");
    }
    if (!IsBasename(request.pgo.constraints_file)) {
      throw std::invalid_argument("pgo constraints_file must be a basename");
    }
    if (!std::isfinite(request.pgo.timeout_sec) || request.pgo.timeout_sec <= 0.0) {
      throw std::invalid_argument("pgo timeout_sec must be positive and finite");
    }
    if (!request.product_session_id.empty() &&
        !IsValidProductSessionId(request.product_session_id)) {
      throw std::invalid_argument("product_session_id is invalid");
    }
    if (request.allow_unverified_snapshot && request.octomap.slam_source == "native_dds") {
      throw std::invalid_argument("native_dds SaveMap cannot allow an unverified snapshot");
    }
    if (request.require.traversability) {
      request.require.esdf = true;
    }
    const std::string canonical = RequestCanonical(request);

    std::lock_guard<std::mutex> lock(mutex_);
    const auto existing = jobs_.find(request.request_id);
    if (existing != jobs_.end()) {
      if (existing->second->status.reason_code == "journal_corrupt") {
        SaveMapStatus corrupt = existing->second->status;
        corrupt.replayed = true;
        return {false, true, "journal_corrupt", corrupt};
      }
      if (RequestCanonical(existing->second->request) != canonical) {
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
    job->status.job_id = request.request_id;
    job->status.request_id = request.request_id;
    job->status.map_id = request.map_id;
    job->status.product_session_id = request.product_session_id;
    job->status.state = SaveJobState::kWaitingSnapshot;
    job->status.phase = SavePhase::kCapture;
    job->status.message = "waiting for map snapshot";
    job->status.activation_requested = request.activate_on_success;
    job->status.created_at_ns = NowNs();
    job->status.updated_at_ns = job->status.created_at_ns;
    job->status.capture_dir = JobDir(request.request_id) / "capture";
    std::filesystem::create_directories(job->status.capture_dir);
    jobs_.emplace(request.request_id, job);
    PersistJobLocked(*job);
    AppendEventLocked(*job, "REQUESTED");
    return {true, false, {}, job->status};
  }

  SaveMapResult ProvideSnapshot(const std::string &job_id, const MapSnapshot &raw_snapshot) {
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
        if (id_conflict) {
          SaveMapStatus conflict = job->status;
          conflict.reason_code = "snapshot_idempotency_conflict";
          conflict.message = "job already owns a different map snapshot";
          return {false, true, conflict.reason_code, conflict};
        }
        SaveMapStatus replay = job->status;
        replay.replayed = true;
        return {true, true, {}, replay};
      }
      if (IsTerminal(job->status.state)) {
        SaveMapStatus replay = job->status;
        replay.replayed = true;
        return {job->status.state == SaveJobState::kSucceeded, true, job->status.reason_code,
                replay};
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
      std::mutex &mutex;
      std::shared_ptr<Job> job;
      ~CaptureReservation() {
        std::lock_guard<std::mutex> lock(mutex);
        job->accepting_snapshot = false;
      }
    } capture_reservation{mutex_, job};

    MapSnapshot snapshot = raw_snapshot;
    if (!job->request.product_session_id.empty() &&
        snapshot.product_session_id != job->request.product_session_id) {
      SaveMapStatus mismatch = job->status;
      mismatch.reason_code = "snapshot_product_session_mismatch";
      mismatch.message = "snapshot does not belong to the SaveMap Product session";
      return {false, false, mismatch.reason_code, mismatch};
    }
    if (snapshot.snapshot_id.empty()) {
      snapshot.snapshot_id = job_id;
    }
    if (snapshot.frame_id.empty()) {
      snapshot.frame_id = "map";
    }
    if (snapshot.captured_at_ns == 0) {
      snapshot.captured_at_ns = NowNs();
    }
    if (snapshot.first_sequence > 0U && snapshot.last_sequence > 0U &&
        snapshot.first_sequence > snapshot.last_sequence) {
      return FailBeforeQueue(*job, "snapshot_sequence_invalid",
                             "snapshot first_sequence is greater than last_sequence");
    }
    if (HasSnapshotReceipt(snapshot) && !HasCompleteSnapshotReceipt(snapshot)) {
      return FailBeforeQueue(
          *job, "snapshot_receipt_incomplete",
          "snapshot receipt is missing SLAM boot, epoch, sequence, or point count");
    }
    if (!HasCompleteSnapshotReceipt(snapshot) && !job->request.allow_unverified_snapshot) {
      return FailBeforeQueue(*job, "snapshot_receipt_required",
                             "SaveMap requires a SLAM snapshot receipt");
    }
    if (HasCompleteSnapshotReceipt(snapshot) &&
        (snapshot.first_sequence != snapshot.observation_sequence ||
         snapshot.last_sequence != snapshot.observation_sequence)) {
      return FailBeforeQueue(*job, "snapshot_receipt_sequence_mismatch",
                             "snapshot sequence range does not match the atomic SLAM receipt");
    }
    if (job->request.require_slam_healthy && !snapshot.slam_healthy) {
      return FailBeforeQueue(*job, "slam_unhealthy",
                             snapshot.health_message.empty()
                                 ? "SLAM health gate rejected the snapshot"
                                 : snapshot.health_message);
    }
    if (snapshot.frame_id != job->request.octomap.frame_id) {
      return FailBeforeQueue(*job, "snapshot_frame_mismatch",
                             "snapshot frame does not match map frame");
    }
    if (!std::filesystem::is_directory(snapshot.source_dir)) {
      return FailBeforeQueue(*job, "snapshot_source_missing",
                             "snapshot source directory not found");
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
      if (!IsNonEmptyRegularFile(pcd_path)) {
        return FailBeforeQueue(*job, "snapshot_pcd_missing",
                               "snapshot does not contain a non-empty map.pcd");
      }
      snapshot.source_dir = capture_dir;
      if (HasCompleteSnapshotReceipt(snapshot) &&
          snapshot.source_point_count < job->request.minimum_point_count) {
        return FailBeforeQueue(*job, "snapshot_receipt_point_count_too_small",
                               "snapshot receipt point count is below the SaveMap minimum");
      }
      if (job->request.require.semantic) {
        std::string semantic_error;
        if (!ValidateSemanticMapBinary(capture_dir / kSemanticMapArtifactFilename,
                                       &semantic_error)) {
          return FailBeforeQueue(*job, "semantic_snapshot_invalid",
                                 semantic_error.empty()
                                     ? "required semantic snapshot is missing or invalid"
                                     : semantic_error);
        }
      }
      const auto bytes = TreeBytes(capture_dir);
      const auto available = std::filesystem::space(store_.RootDir()).available;
      const std::uint64_t reserve = 16ULL * 1024ULL * 1024ULL;
      if (available < bytes * 3U + reserve) {
        return FailBeforeQueue(*job, "insufficient_disk_space",
                               "not enough disk space for staged SaveMap transaction");
      }
      SyncTree(capture_dir);
      if (hooks_.after_snapshot_copied) {
        hooks_.after_snapshot_copied(job_id);
      }
    } catch (const std::exception &exc) {
      return FailBeforeQueue(*job, "snapshot_capture_failed", exc.what());
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!job->accepting_snapshot || job->status.state != SaveJobState::kWaitingSnapshot ||
          IsTerminal(job->status.state) || job->status.cancel_requested) {
        SaveMapStatus rejected = job->status;
        rejected.replayed = true;
        const std::string reason = rejected.reason_code.empty()
                                       ? "snapshot_no_longer_accepted"
                                       : rejected.reason_code;
        return {false, true, reason, rejected};
      }
      job->snapshot = snapshot;
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

  SaveMapResult RejectSnapshot(const std::string &job_id, const std::string &reason_code,
                               const std::string &message) {
    std::shared_ptr<Job> job;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = jobs_.find(job_id);
      if (found == jobs_.end()) {
        return {false, false, "job_not_found", MissingStatus(job_id, "job_not_found")};
      }
      job = found->second;
      if (IsTerminal(job->status.state)) {
        SaveMapStatus replay = job->status;
        replay.replayed = true;
        return {false, true, replay.reason_code, replay};
      }
    }
    return FailBeforeQueue(*job, reason_code, message);
  }

  SaveMapResult Cancel(const std::string &job_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    if (found == jobs_.end()) {
      return {false, false, "job_not_found", MissingStatus(job_id, "job_not_found")};
    }
    auto &job = *found->second;
    if (IsTerminal(job.status.state)) {
      SaveMapStatus replay = job.status;
      replay.replayed = true;
      return {job.status.state == SaveJobState::kSucceeded, true, job.status.reason_code, replay};
    }
    job.status.cancel_requested = true;
    job.status.updated_at_ns = NowNs();
    job.status.message = "cancellation requested";
    if (job.status.state == SaveJobState::kWaitingSnapshot ||
        job.status.state == SaveJobState::kQueued) {
      job.status.state = SaveJobState::kCancelled;
      job.status.phase = SavePhase::kDone;
      job.status.progress = 1.0;
      job.status.reason_code = "cancelled";
      job.status.completed_at_ns = job.status.updated_at_ns;
    }
    PersistJobLocked(job);
    AppendEventLocked(job, job.status.state == SaveJobState::kCancelled ? "CANCELLED"
                                                                        : "CANCEL_REQUESTED");
    cv_.notify_all();
    return {true, false, {}, job.status};
  }

  SaveMapResult Retry(const std::string &job_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    if (found == jobs_.end()) {
      return {false, false, "job_not_found", MissingStatus(job_id, "job_not_found")};
    }
    auto &job = *found->second;
    if (job.status.reason_code == "journal_corrupt") {
      return {false, false, "journal_corrupt", job.status};
    }
    if (job.status.reason_code == "legacy_schema_requires_resubmit") {
      return {false, false, "legacy_schema_requires_resubmit", job.status};
    }
    if (job.accepting_snapshot) {
      return {false, false, "snapshot_capture_in_progress", job.status};
    }
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
    job.status.activation_succeeded = false;
    job.status.map_dir.clear();
    job.map_committed = false;
    job.status.artifact_report_json.clear();
    if (job.snapshot.has_value() && IsNonEmptyRegularFile(job.status.capture_dir / "map.pcd")) {
      job.status.state = SaveJobState::kQueued;
      job.status.phase = SavePhase::kValidate;
      job.status.progress = 0.05;
      job.status.message = "SaveMap retry queued";
      queue_.push_back(job_id);
    } else {
      job.snapshot.reset();
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

  std::optional<SaveMapStatus> GetStatus(const std::string &job_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    return found == jobs_.end() ? std::nullopt
                                : std::optional<SaveMapStatus>(found->second->status);
  }

  std::vector<SaveMapStatus> ListStatuses(std::size_t limit) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SaveMapStatus> statuses;
    statuses.reserve(jobs_.size());
    for (const auto &item : jobs_) {
      statuses.push_back(item.second->status);
    }
    std::sort(statuses.begin(), statuses.end(), [](const auto &left, const auto &right) {
      return left.created_at_ns > right.created_at_ns;
    });
    if (limit > 0U && statuses.size() > limit) {
      statuses.resize(limit);
    }
    return statuses;
  }

  std::optional<SaveMapStatus> Wait(const std::string &job_id,
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
    return current == jobs_.end() ? std::nullopt
                                  : std::optional<SaveMapStatus>(current->second->status);
  }

  void Recover() {
    std::lock_guard<std::mutex> lock(mutex_);
    RecoverInternalLocked();
    cv_.notify_all();
  }

 private:
  std::filesystem::path JobDir(const std::string &job_id) const { return jobs_root_ / job_id; }

  std::filesystem::path JobStatePath(const std::string &job_id) const {
    return JobDir(job_id) / "job.state";
  }

  std::filesystem::path JobJournalPath(const std::string &job_id) const {
    return JobDir(job_id) / "events.jsonl";
  }

  std::filesystem::path WorkRoot(const std::string &job_id) const {
#if defined(_WIN32)
    return std::filesystem::temp_directory_path() / "lt_maps" / job_id;
#else
    return JobDir(job_id) / "work";
#endif
  }

  void AcquireEngineLock() {
    std::filesystem::create_directories(engine_lock_path_.parent_path());
    if (!std::filesystem::create_directory(engine_lock_path_)) {
      const auto owner = ReadKeyValues(engine_lock_path_ / "owner.state");
      const auto owner_pid = ParseIntegerStrict<std::uint64_t>(GetValue(owner, "pid"));
      if (owner_pid == 0U) {
        throw std::runtime_error("SaveMap engine lock owner state is invalid");
      }
      if (ProcessAlive(owner_pid)) {
        throw std::runtime_error("another SaveMap engine owns map root: " +
                                 store_.RootDir().string());
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

  SaveMapStatus MissingStatus(const std::string &job_id, const std::string &reason) const {
    SaveMapStatus status;
    status.job_id = job_id;
    status.state = SaveJobState::kFailed;
    status.phase = SavePhase::kDone;
    status.reason_code = reason;
    status.message = "SaveMap job not found";
    return status;
  }

  SaveMapResult FailBeforeQueue(Job &job, const std::string &reason, const std::string &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (IsTerminal(job.status.state) || job.status.cancel_requested) {
      SaveMapStatus replay = job.status;
      replay.replayed = true;
      const std::string current_reason =
          replay.reason_code.empty()
              ? (job.status.cancel_requested ? "cancel_requested" : "snapshot_no_longer_accepted")
              : replay.reason_code;
      return {false, true, current_reason, replay};
    }
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

  void PersistJobLocked(const Job &job) const {
    std::ostringstream out;
    AppendKey(out, "schema", kJobSchemaVersion);
    AppendKey(out, "job_id", job.status.job_id);
    AppendKey(out, "request_id", job.request.request_id);
    AppendKey(out, "map_id", job.request.map_id);
    AppendKey(out, "product_session_id", job.request.product_session_id);
    AppendKey(out, "state", SaveJobStateName(job.status.state));
    AppendKey(out, "phase", SavePhaseName(job.status.phase));
    AppendKey(out, "progress", PreciseDouble(job.status.progress));
    AppendKey(out, "message", job.status.message);
    AppendKey(out, "reason_code", job.status.reason_code);
    AppendKey(out, "capture_dir", job.status.capture_dir.string());
    AppendKey(out, "map_dir", job.status.map_dir.string());
    AppendKey(out, "source_report_json", job.status.source_report_json);
    AppendKey(out, "artifact_report_json", job.status.artifact_report_json);
    AppendKey(out, "created_at_ns", std::to_string(job.status.created_at_ns));
    AppendKey(out, "updated_at_ns", std::to_string(job.status.updated_at_ns));
    AppendKey(out, "completed_at_ns", std::to_string(job.status.completed_at_ns));
    AppendKey(out, "map_committed", job.map_committed ? "1" : "0");
    AppendKey(out, "activation_requested", job.status.activation_requested ? "1" : "0");
    AppendKey(out, "activation_succeeded", job.status.activation_succeeded ? "1" : "0");
    AppendKey(out, "cancel_requested", job.status.cancel_requested ? "1" : "0");
    AppendKey(out, "recovered", job.status.recovered ? "1" : "0");
    AppendKey(out, "require_occupancy", job.request.require.occupancy ? "1" : "0");
    AppendKey(out, "require_octomap", job.request.require.octomap ? "1" : "0");
    AppendKey(out, "require_esdf", job.request.require.esdf ? "1" : "0");
    AppendKey(out, "require_traversability", job.request.require.traversability ? "1" : "0");
    AppendKey(out, "require_semantic", job.request.require.semantic ? "1" : "0");
    AppendKey(out, "activate_on_success", job.request.activate_on_success ? "1" : "0");
    AppendKey(out, "require_slam_healthy", job.request.require_slam_healthy ? "1" : "0");
    AppendKey(out, "allow_unverified_snapshot", job.request.allow_unverified_snapshot ? "1" : "0");
    AppendKey(out, "minimum_point_count", std::to_string(job.request.minimum_point_count));
    AppendKey(out, "pgo_executable", job.request.pgo.executable);
    AppendKey(out, "pgo_constraints_file", job.request.pgo.constraints_file);
    AppendKey(out, "pgo_timeout_sec", PreciseDouble(job.request.pgo.timeout_sec));
    AppendKey(out, "source_voxel_size", PreciseDouble(job.request.source.voxel_size));
    AppendKey(out, "dynamic_filter_enabled", job.request.source.dynamic_filter_enabled ? "1" : "0");
    AppendKey(out, "dynamic_filter_required",
              job.request.source.dynamic_filter_required ? "1" : "0");
    AppendKey(out, "dynamic_filter_command", job.request.source.dynamic_filter_command);
    AppendKey(out, "dynamic_filter_timeout_sec",
              PreciseDouble(job.request.source.dynamic_filter_timeout_sec));
    AppendKey(out, "octomap_converter_command", job.request.octomap.converter_command);
    AppendKey(out, "octomap_build_mode", job.request.octomap.build_mode);
    AppendKey(out, "octomap_resolution", PreciseDouble(job.request.octomap.resolution));
    AppendKey(out, "octomap_support_dilation_cells",
              std::to_string(job.request.octomap.support_dilation_cells));
    AppendKey(out, "octomap_free_layers_above",
              std::to_string(job.request.octomap.free_layers_above));
    AppendKey(out, "octomap_free_dilation_cells",
              std::to_string(job.request.octomap.free_dilation_cells));
    AppendKey(out, "octomap_frame_id", job.request.octomap.frame_id);
    AppendKey(out, "octomap_source_profile", job.request.octomap.source_profile);
    AppendKey(out, "octomap_data_source", job.request.octomap.data_source);
    AppendKey(out, "octomap_slam_source", job.request.octomap.slam_source);
    AppendKey(out, "octomap_localization_source", job.request.octomap.localization_source);
    AppendKey(out, "octomap_mapping_source", job.request.octomap.mapping_source);
    AppendKey(out, "octomap_timeout_sec", PreciseDouble(job.request.octomap.timeout_sec));
    if (job.snapshot.has_value()) {
      AppendKey(out, "snapshot_id", job.snapshot->snapshot_id);
      AppendKey(out, "snapshot_source_dir", job.snapshot->source_dir.string());
      AppendKey(out, "snapshot_frame_id", job.snapshot->frame_id);
      AppendKey(out, "snapshot_captured_at_ns", std::to_string(job.snapshot->captured_at_ns));
      AppendKey(out, "snapshot_first_sequence", std::to_string(job.snapshot->first_sequence));
      AppendKey(out, "snapshot_last_sequence", std::to_string(job.snapshot->last_sequence));
      AppendKey(out, "snapshot_slam_boot_id", job.snapshot->slam_boot_id);
      AppendKey(out, "snapshot_product_session_id", job.snapshot->product_session_id);
      AppendKey(out, "snapshot_reset_epoch", std::to_string(job.snapshot->reset_epoch));
      AppendKey(out, "snapshot_observation_sequence",
                std::to_string(job.snapshot->observation_sequence));
      AppendKey(out, "snapshot_source_point_count",
                std::to_string(job.snapshot->source_point_count));
      AppendKey(out, "snapshot_slam_healthy", job.snapshot->slam_healthy ? "1" : "0");
      AppendKey(out, "snapshot_health_message", job.snapshot->health_message);
    }
    WriteTextAtomic(JobStatePath(job.status.job_id), out.str());
  }

  void AppendEventLocked(const Job &job, const std::string &event) const {
    const auto path = JobJournalPath(job.status.job_id);
    std::filesystem::create_directories(path.parent_path());
    std::ofstream file(path, std::ios::binary | std::ios::app);
    if (!file) {
      throw std::runtime_error("failed to append SaveMap journal: " + path.string());
    }
    file << "{\"schema_version\":1,\"event\":" << JsonString(event)
         << ",\"timestamp_ns\":" << NowNs() << ",\"job_id\":" << JsonString(job.status.job_id)
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

  void AppendEvent(const std::shared_ptr<Job> &job, const std::string &event) {
    std::lock_guard<std::mutex> lock(mutex_);
    AppendEventLocked(*job, event);
  }

  std::shared_ptr<Job> LoadJob(const std::filesystem::path &state_path) const {
    const auto values = ReadKeyValues(state_path);
    const auto schema = GetValue(values, "schema");
    if (schema != "3" && schema != kJobSchemaVersion) {
      throw std::runtime_error("unsupported SaveMap job schema");
    }
    const std::string directory_job_id = state_path.parent_path().filename().string();
    if (!IsValidJobId(directory_job_id)) {
      throw std::runtime_error("invalid SaveMap job directory identity");
    }
    auto job = std::make_shared<Job>();
    job->loaded_schema = schema;
    job->request.request_id = GetValue(values, "request_id");
    job->request.map_id = GetValue(values, "map_id");
    job->request.product_session_id = GetValue(values, "product_session_id");
    if (job->request.request_id != directory_job_id ||
        !MapStore::IsValidMapId(job->request.map_id)) {
      throw std::runtime_error("SaveMap job identity is inconsistent");
    }
    job->request.require.occupancy = ParseBool(GetValue(values, "require_occupancy", "1"));
    job->request.require.octomap = ParseBool(GetValue(values, "require_octomap", "1"));
    job->request.require.esdf = ParseBool(GetValue(values, "require_esdf", "1"));
    job->request.require.traversability =
        ParseBool(GetValue(values, "require_traversability", "1"));
    job->request.require.semantic = ParseBool(GetValue(values, "require_semantic"));
    job->request.activate_on_success = ParseBool(GetValue(values, "activate_on_success"));
    job->request.require_slam_healthy = ParseBool(GetValue(values, "require_slam_healthy", "1"));
    job->request.allow_unverified_snapshot =
        ParseBool(GetValue(values, "allow_unverified_snapshot"));
    job->request.minimum_point_count =
        ParseIntegerStrict<std::uint64_t>(GetValue(values, "minimum_point_count"), 1U);
    job->request.pgo.executable = GetValue(values, "pgo_executable");
    job->request.pgo.constraints_file =
        GetValue(values, "pgo_constraints_file", "pose_graph.constraints");
    job->request.pgo.timeout_sec = ParseDouble(GetValue(values, "pgo_timeout_sec"), 300.0);
    job->request.source.voxel_size = ParseDouble(GetValue(values, "source_voxel_size"));
    job->request.source.dynamic_filter_enabled =
        ParseBool(GetValue(values, "dynamic_filter_enabled", "1"));
    job->request.source.dynamic_filter_required =
        ParseBool(GetValue(values, "dynamic_filter_required"));
    job->request.source.dynamic_filter_command = GetValue(values, "dynamic_filter_command");
    job->request.source.dynamic_filter_timeout_sec =
        ParseDouble(GetValue(values, "dynamic_filter_timeout_sec"), 300.0);
    job->request.octomap.converter_command = GetValue(values, "octomap_converter_command");
    job->request.octomap.build_mode =
        GetValue(values, "octomap_build_mode", "external_pcl_converter");
    job->request.octomap.resolution = ParseDouble(GetValue(values, "octomap_resolution"), 0.20);
    job->request.octomap.support_dilation_cells =
        ParseIntegerStrict<int>(GetValue(values, "octomap_support_dilation_cells"), 1);
    job->request.octomap.free_layers_above =
        ParseIntegerStrict<int>(GetValue(values, "octomap_free_layers_above"), 3);
    job->request.octomap.free_dilation_cells =
        ParseIntegerStrict<int>(GetValue(values, "octomap_free_dilation_cells"), 1);
    job->request.octomap.frame_id = GetValue(values, "octomap_frame_id", "map");
    job->request.octomap.source_profile =
        GetValue(values, "octomap_source_profile", "map_pipeline");
    job->request.octomap.data_source = GetValue(values, "octomap_data_source", "map_pipeline");
    job->request.octomap.slam_source = GetValue(values, "octomap_slam_source", "unknown");
    job->request.octomap.localization_source =
        GetValue(values, "octomap_localization_source", "unknown");
    job->request.octomap.mapping_source =
        GetValue(values, "octomap_mapping_source", "lingtu_maps_pipeline");
    job->request.octomap.timeout_sec = ParseDouble(GetValue(values, "octomap_timeout_sec"), 60.0);

    job->status.job_id = GetValue(values, "job_id", job->request.request_id);
    job->status.request_id = job->request.request_id;
    job->status.map_id = job->request.map_id;
    job->status.product_session_id = job->request.product_session_id;
    job->status.state = ParseState(GetValue(values, "state"));
    job->status.phase = ParsePhase(GetValue(values, "phase"));
    job->status.progress = ParseDouble(GetValue(values, "progress"));
    job->status.message = GetValue(values, "message");
    job->status.reason_code = GetValue(values, "reason_code");
    job->status.capture_dir = GetValue(values, "capture_dir");
    job->status.map_dir = GetValue(values, "map_dir");
    job->status.source_report_json = GetValue(values, "source_report_json");
    job->status.artifact_report_json = GetValue(values, "artifact_report_json");
    job->status.created_at_ns = ParseIntegerStrict<std::int64_t>(GetValue(values, "created_at_ns"));
    job->status.updated_at_ns = ParseIntegerStrict<std::int64_t>(GetValue(values, "updated_at_ns"));
    job->status.completed_at_ns =
        ParseIntegerStrict<std::int64_t>(GetValue(values, "completed_at_ns"));
    job->map_committed = ParseBool(GetValue(values, "map_committed"));
    job->status.activation_requested = ParseBool(
        GetValue(values, "activation_requested", job->request.activate_on_success ? "1" : "0"));
    job->status.activation_succeeded = ParseBool(GetValue(values, "activation_succeeded"));
    job->status.cancel_requested = ParseBool(GetValue(values, "cancel_requested"));
    job->status.recovered = ParseBool(GetValue(values, "recovered"));

    if (!GetValue(values, "snapshot_id").empty()) {
      MapSnapshot snapshot;
      snapshot.snapshot_id = GetValue(values, "snapshot_id");
      snapshot.source_dir = GetValue(values, "snapshot_source_dir");
      snapshot.frame_id = GetValue(values, "snapshot_frame_id", "map");
      snapshot.captured_at_ns =
          ParseIntegerStrict<std::int64_t>(GetValue(values, "snapshot_captured_at_ns"));
      snapshot.first_sequence =
          ParseIntegerStrict<std::uint64_t>(GetValue(values, "snapshot_first_sequence"));
      snapshot.last_sequence =
          ParseIntegerStrict<std::uint64_t>(GetValue(values, "snapshot_last_sequence"));
      snapshot.slam_boot_id = GetValue(values, "snapshot_slam_boot_id");
      snapshot.product_session_id = GetValue(values, "snapshot_product_session_id");
      snapshot.reset_epoch =
          ParseIntegerStrict<std::uint64_t>(GetValue(values, "snapshot_reset_epoch"));
      snapshot.observation_sequence =
          ParseIntegerStrict<std::uint64_t>(GetValue(values, "snapshot_observation_sequence"));
      snapshot.source_point_count =
          ParseIntegerStrict<std::uint64_t>(GetValue(values, "snapshot_source_point_count"));
      snapshot.slam_healthy = ParseBool(GetValue(values, "snapshot_slam_healthy", "1"));
      snapshot.health_message = GetValue(values, "snapshot_health_message");
      job->snapshot = std::move(snapshot);
    }

    if (job->status.job_id != directory_job_id ||
        (!job->request.product_session_id.empty() &&
         !IsValidProductSessionId(job->request.product_session_id)) ||
        job->request.minimum_point_count == 0U || !IsBasename(job->request.pgo.constraints_file) ||
        !std::isfinite(job->request.pgo.timeout_sec) || job->request.pgo.timeout_sec <= 0.0 ||
        job->status.progress < 0.0 || job->status.progress > 1.0 ||
        job->status.created_at_ns <= 0 || job->status.updated_at_ns <= 0 ||
        job->status.completed_at_ns < 0) {
      throw std::runtime_error("SaveMap job state failed integrity validation");
    }
    const auto expected_capture = state_path.parent_path() / "capture";
    if (!SameNormalizedPath(job->status.capture_dir, expected_capture)) {
      throw std::runtime_error("SaveMap capture path escaped its job directory");
    }
    if (!job->status.map_dir.empty() &&
        !SameNormalizedPath(job->status.map_dir, store_.MapPath(job->request.map_id))) {
      throw std::runtime_error("SaveMap map path failed integrity validation");
    }
    if (job->snapshot.has_value()) {
      if (!SameNormalizedPath(job->snapshot->source_dir, expected_capture) ||
          (HasSnapshotReceipt(*job->snapshot) && !HasCompleteSnapshotReceipt(*job->snapshot)) ||
          (!job->request.allow_unverified_snapshot &&
           !HasCompleteSnapshotReceipt(*job->snapshot))) {
        throw std::runtime_error("SaveMap snapshot state failed integrity validation");
      }
    }
    return job;
  }

  std::shared_ptr<Job> CorruptJob(const std::filesystem::path &job_dir) const {
    const std::string job_id = job_dir.filename().string();
    if (!IsValidJobId(job_id)) {
      return nullptr;
    }
    auto job = std::make_shared<Job>();
    job->request.request_id = job_id;
    job->status.job_id = job_id;
    job->status.request_id = job_id;
    job->status.state = SaveJobState::kFailed;
    job->status.phase = SavePhase::kDone;
    job->status.progress = 1.0;
    job->status.reason_code = "journal_corrupt";
    job->status.message = "SaveMap job journal is corrupt";
    job->status.created_at_ns = NowNs();
    job->status.updated_at_ns = job->status.created_at_ns;
    job->status.completed_at_ns = job->status.created_at_ns;
    job->status.recovered = true;
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
    for (const auto &entry : std::filesystem::directory_iterator(jobs_root_)) {
      if (!entry.is_directory()) {
        continue;
      }
      std::shared_ptr<Job> job;
      try {
        job = LoadJob(entry.path() / "job.state");
      } catch (const std::exception &) {
        job = CorruptJob(entry.path());
      }
      if (!job) {
        continue;
      }
      if (jobs_.find(job->status.job_id) != jobs_.end()) {
        continue;
      }
      if (job->status.reason_code == "journal_corrupt") {
        jobs_.emplace(job->status.job_id, job);
        continue;
      }
      if (job->loaded_schema == "3" && !IsTerminal(job->status.state)) {
        job->status.state = SaveJobState::kFailed;
        job->status.phase = SavePhase::kDone;
        job->status.progress = 1.0;
        job->status.reason_code = "legacy_schema_requires_resubmit";
        job->status.message = "legacy SaveMap job requires a new request";
        job->status.recovered = true;
        job->status.cancel_requested = false;
        job->status.updated_at_ns = NowNs();
        job->status.completed_at_ns = job->status.updated_at_ns;
        jobs_.emplace(job->status.job_id, job);
        PersistJobLocked(*job);
        AppendEventLocked(*job, "RECOVERED_LEGACY_FAILED");
        continue;
      }
      const auto map_dir = store_.MapPath(job->request.map_id);
      const auto commit_stage = CommitStage(store_.RootDir(), job->status.job_id);
      const auto commit_backup =
          CommitBackup(store_.RootDir(), job->request.map_id, job->status.job_id);
      const bool terminal = IsTerminal(job->status.state);
      if (!job->map_committed && std::filesystem::is_directory(commit_backup) &&
          !std::filesystem::exists(map_dir)) {
        std::error_code restore_error;
        std::filesystem::rename(commit_backup, map_dir, restore_error);
        if (!restore_error) {
          SyncPath(store_.RootDir(), true);
        }
      }
      bool committed_map = job->map_committed && std::filesystem::is_directory(map_dir);
      if (!terminal && !committed_map && std::filesystem::is_directory(commit_backup) &&
          std::filesystem::is_directory(map_dir) && !std::filesystem::exists(commit_stage)) {
        try {
          VerifyRequiredArtifacts(map_dir, job->request);
          committed_map = true;
          job->map_committed = true;
        } catch (const std::exception &) {}
      }
      if (committed_map && !terminal) {
        job->status.map_dir = map_dir;
        job->status.activation_succeeded =
            job->status.activation_requested && store_.ActiveMapId() == job->request.map_id;
      }
      if (!terminal) {
        job->status.recovered = true;
        job->status.cancel_requested = false;
        if (committed_map) {
          job->status.state = job->status.activation_requested && !job->status.activation_succeeded
                                  ? SaveJobState::kFailed
                                  : SaveJobState::kSucceeded;
          job->status.phase = SavePhase::kDone;
          job->status.progress = 1.0;
          job->status.reason_code =
              job->status.state == SaveJobState::kFailed ? "activation_failed_after_commit" : "";
          job->status.message = job->status.state == SaveJobState::kFailed
                                    ? "recovered committed map without requested activation"
                                    : "recovered committed map";
          job->status.completed_at_ns = NowNs();
          std::error_code cleanup_error;
          std::filesystem::remove_all(WorkRoot(job->status.job_id), cleanup_error);
          std::filesystem::remove_all(job->status.capture_dir, cleanup_error);
          std::filesystem::remove_all(commit_backup, cleanup_error);
        } else if (job->snapshot.has_value() &&
                   IsNonEmptyRegularFile(job->status.capture_dir / "map.pcd")) {
          if (std::filesystem::is_directory(commit_backup) && !std::filesystem::exists(map_dir)) {
            std::error_code restore_error;
            std::filesystem::rename(commit_backup, map_dir, restore_error);
            if (restore_error) {
              job->status.state = SaveJobState::kFailed;
              job->status.phase = SavePhase::kDone;
              job->status.progress = 1.0;
              job->status.reason_code = "map_restore_failed";
              job->status.message = "failed to restore map interrupted during commit";
              job->status.completed_at_ns = NowNs();
              job->status.updated_at_ns = job->status.completed_at_ns;
              jobs_.emplace(job->status.job_id, job);
              PersistJobLocked(*job);
              AppendEventLocked(*job, "RECOVERED");
              continue;
            }
          }
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
        std::error_code cleanup_error;
        std::filesystem::remove_all(commit_stage, cleanup_error);
        job->status.updated_at_ns = NowNs();
      }
      jobs_.emplace(job->status.job_id, job);
      PersistJobLocked(*job);
      if (job->status.recovered) {
        AppendEventLocked(*job, "RECOVERED");
      }
    }
  }

  bool CancelRequested(const std::string &job_id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = jobs_.find(job_id);
    return stopping_ || (found != jobs_.end() && found->second->status.cancel_requested);
  }

  void SetStatus(const std::shared_ptr<Job> &job, SaveJobState state, SavePhase phase,
                 double progress, const std::string &message, const std::string &reason = {}) {
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

  void SetReports(const std::shared_ptr<Job> &job, const std::string *source_report,
                  const std::string *artifact_report) {
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

  SaveMapStatus SetPreparedMap(const std::shared_ptr<Job> &job,
                               const std::filesystem::path &map_dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    job->status.map_dir = map_dir;
    job->status.updated_at_ns = NowNs();
    PersistJobLocked(*job);
    return job->status;
  }

  void UpdateDurableEvidence(const std::shared_ptr<Job> &job, std::optional<bool> map_committed,
                             std::optional<bool> activation_succeeded,
                             const std::string &event = {}) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (map_committed.has_value()) {
      job->map_committed = *map_committed;
    }
    if (activation_succeeded.has_value()) {
      job->status.activation_succeeded = *activation_succeeded;
    }
    job->status.updated_at_ns = NowNs();
    PersistJobLocked(*job);
    if (!event.empty()) {
      AppendEventLocked(*job, event);
    }
    cv_.notify_all();
  }

  bool BeforePhase(const std::shared_ptr<Job> &job, SavePhase phase,
                   std::optional<MapLock> *map_lock = nullptr,
                   const std::function<void()> &before_unlock = {}) {
    if (hooks_.before_phase) {
      hooks_.before_phase(job->status.job_id, phase);
    }
    if (hooks_.forced_failure) {
      const auto failure = hooks_.forced_failure(job->status.job_id, phase);
      if (failure.has_value()) {
        if (before_unlock) {
          before_unlock();
        }
        if (map_lock != nullptr) {
          map_lock->reset();
        }
        SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0, *failure,
                  "forced_stage_failure");
        return false;
      }
    }
    bool stop_requested = false;
    bool cancel_requested = false;
    double progress = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested = stopping_;
      const auto found = jobs_.find(job->status.job_id);
      cancel_requested = found != jobs_.end() && found->second->status.cancel_requested;
      progress = job->status.progress;
    }
    if (stop_requested || cancel_requested) {
      if (before_unlock) {
        before_unlock();
      }
      if (map_lock != nullptr) {
        map_lock->reset();
      }
      if (stop_requested && !cancel_requested) {
        SetStatus(job, SaveJobState::kQueued, phase, progress,
                  "SaveMap service stopped; job will recover on restart");
      } else {
        SetStatus(job, SaveJobState::kCancelled, SavePhase::kDone, 1.0, "SaveMap cancelled",
                  "cancelled");
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
      } catch (const std::exception &exc) {
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
        } catch (...) {}
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

  void ProcessJob(const std::shared_ptr<Job> &job) {
    if (!job->snapshot.has_value()) {
      SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                "queued SaveMap job has no snapshot", "snapshot_missing");
      return;
    }
    auto map_lock = MapLock::TryAcquire(store_.RootDir(), job->request.map_id, job->status.job_id);
    if (!map_lock.has_value()) {
      SetStatus(job, SaveJobState::kFailed, SavePhase::kDone, 1.0,
                "another process is saving this map", "map_save_in_progress");
      return;
    }
    const auto finish = [&map_lock, this, &job](SaveJobState state, const std::string &message,
                                                const std::string &reason) {
      map_lock.reset();
      SetStatus(job, state, SavePhase::kDone, 1.0, message, reason);
    };

    try {
      SetStatus(job, SaveJobState::kRunning, SavePhase::kValidate, 0.10,
                "checking required snapshot inputs");
      if (!BeforePhase(job, SavePhase::kValidate, &map_lock))
        return;
      const auto capture_pcd = job->status.capture_dir / "map.pcd";
      if (!IsNonEmptyRegularFile(capture_pcd)) {
        finish(SaveJobState::kFailed, "captured map.pcd is missing or empty", "snapshot_invalid");
        return;
      }

      const auto work_root = WorkRoot(job->status.job_id);
      const auto source_work = work_root / "i";
      const auto optimized_work = work_root / "o";
      const auto store_root = work_root / "s";
      std::filesystem::remove_all(work_root);
      CopyTree(job->status.capture_dir, source_work, true);

      SetStatus(job, SaveJobState::kRunning, SavePhase::kOptimizeSource, 0.20,
                "optimizing saved map source");
      if (!BeforePhase(job, SavePhase::kOptimizeSource, &map_lock))
        return;
      const auto constraints = source_work / job->request.pgo.constraints_file;
      std::filesystem::path processed_source = source_work;
      const bool explicit_constraints = std::filesystem::is_regular_file(constraints);
      const bool auto_constraints = !explicit_constraints && HasCompletePatchBundle(source_work);
      if (explicit_constraints || auto_constraints) {
        ProcessRunOptions options;
        options.cwd = source_work;
        options.timeout_sec = job->request.pgo.timeout_sec;
        options.cancel_requested = [this, id = job->status.job_id]() {
          return CancelRequested(id);
        };
        const auto result = RunShellCommand(
            BuildPgoCommand(job->request.pgo, source_work, optimized_work,
                            explicit_constraints ? std::optional<std::filesystem::path>(constraints)
                                                 : std::nullopt),
            options);
        if (result.cancelled || CancelRequested(job->status.job_id)) {
          BeforePhase(job, SavePhase::kOptimizeSource, &map_lock);
          return;
        }
        if (result.timed_out) {
          if (explicit_constraints) {
            finish(SaveJobState::kFailed, "pose graph optimization timed out", "pgo_timeout");
            return;
          }
          WritePgoNotPerformedReport(source_work,
                                     PgoCliResult{true, false, "pgo_timeout",
                                                  "automatic pose graph optimization timed out"});
        } else {
          const auto cli = ParsePgoCliResult(result.stdout_text);
          if (result.launch_failed || result.exit_code != 0) {
            finish(SaveJobState::kFailed,
                   cli.has_value()
                       ? cli->message
                       : "pose graph optimization failed: " + result.error + result.stderr_text,
                   cli.has_value() ? cli->code : "pgo_failed");
            return;
          }
          if (!cli.has_value()) {
            finish(SaveJobState::kFailed,
                   "pose graph optimizer returned an invalid result contract",
                   "pgo_output_invalid");
            return;
          }
          if (!cli->ok) {
            finish(SaveJobState::kFailed, cli->message, cli->code);
            return;
          }
          if (!cli->performed) {
            if (explicit_constraints || !IsAllowedAutoPgoSkip(*cli)) {
              finish(SaveJobState::kFailed,
                     "pose graph optimizer returned an invalid not-performed result",
                     "pgo_output_invalid");
              return;
            }
            WritePgoNotPerformedReport(source_work, *cli);
          } else {
            if (!IsValidPerformedPgoResult(*cli, auto_constraints) ||
                !HasPgoOutputBundle(optimized_work, *cli)) {
              finish(SaveJobState::kFailed,
                     "pose graph optimizer did not produce a complete map bundle",
                     "pgo_output_invalid");
              return;
            }
            processed_source = optimized_work;
          }
        }
      } else {
        WritePgoNotPerformedReport(
            source_work,
            PgoCliResult{true, false, "patch_bundle_incomplete",
                         "automatic pose graph optimization requires a complete patch bundle"});
      }

      SetStatus(job, SaveJobState::kRunning, SavePhase::kProcessSource, 0.30,
                "filtering source map");
      if (!BeforePhase(job, SavePhase::kProcessSource, &map_lock))
        return;
      MapStore staging_store(MapStoreConfig{store_root});
      MapPipelineCore pipeline(staging_store);
      SourceCommitOptions source_options = job->request.source;
      source_options.cancel_requested = [this, id = job->status.job_id]() {
        return CancelRequested(id);
      };
      const auto source_result =
          pipeline.CommitSavedSourceJson(job->request.map_id, processed_source, source_options);
      SetReports(job, &source_result, nullptr);
      if (!JsonSucceeded(source_result)) {
        if (CancelRequested(job->status.job_id)) {
          BeforePhase(job, SavePhase::kProcessSource, &map_lock);
        } else {
          finish(SaveJobState::kFailed, "source processing failed",
                 JsonReason(source_result, "source_processing_failed"));
        }
        return;
      }
      const auto staging_map = staging_store.MapPath(job->request.map_id);
      const auto semantic_source = job->status.capture_dir / kSemanticMapArtifactFilename;
      if (std::filesystem::is_regular_file(semantic_source)) {
        std::filesystem::copy_file(semantic_source, staging_map / kSemanticMapArtifactFilename,
                                   std::filesystem::copy_options::overwrite_existing);
      }

      SetStatus(job, SaveJobState::kRunning, SavePhase::kBuildArtifacts, 0.50,
                "building required map artifacts");
      if (!BeforePhase(job, SavePhase::kBuildArtifacts, &map_lock))
        return;
      OctomapBuildOptions octomap_options = job->request.octomap;
      octomap_options.cancel_requested = [this, id = job->status.job_id]() {
        return CancelRequested(id);
      };
      std::string build_result;
      if (job->request.require.octomap) {
        build_result = pipeline.BuildNavigationPackageJson(job->request.map_id, octomap_options,
                                                           job->request.require.esdf,
                                                           job->request.require.traversability);
        SetReports(job, nullptr, &build_result);
        if (!JsonSucceeded(build_result)) {
          if (CancelRequested(job->status.job_id)) {
            BeforePhase(job, SavePhase::kBuildArtifacts, &map_lock);
          } else {
            finish(SaveJobState::kFailed, "navigation package build failed",
                   JsonReason(build_result, "navigation_package_failed"));
          }
          return;
        }
      } else {
        if (job->request.require.occupancy) {
          build_result = pipeline.BuildOccupancySnapshotJson(job->request.map_id);
          SetReports(job, nullptr, &build_result);
          if (!JsonSucceeded(build_result)) {
            finish(SaveJobState::kFailed, "occupancy build failed",
                   JsonReason(build_result, "occupancy_build_failed"));
            return;
          }
        }
        if (job->request.require.esdf) {
          build_result = pipeline.BuildEsdfArtifactJson(job->request.map_id);
          SetReports(job, nullptr, &build_result);
          if (!JsonSucceeded(build_result)) {
            finish(SaveJobState::kFailed, "ESDF build failed",
                   JsonReason(build_result, "esdf_build_failed"));
            return;
          }
        }
        if (job->request.require.traversability) {
          build_result = pipeline.BuildTraversabilityArtifactJson(job->request.map_id);
          SetReports(job, nullptr, &build_result);
          if (!JsonSucceeded(build_result)) {
            finish(SaveJobState::kFailed, "traversability build failed",
                   JsonReason(build_result, "traversability_build_failed"));
            return;
          }
        }
      }

      SetStatus(job, SaveJobState::kRunning, SavePhase::kVerify, 0.75,
                "checking required artifacts");
      if (!BeforePhase(job, SavePhase::kVerify, &map_lock))
        return;
      const auto map_dir = store_.MapPath(job->request.map_id);
      const auto commit_stage = CommitStage(store_.RootDir(), job->status.job_id);
      const auto commit_backup =
          CommitBackup(store_.RootDir(), job->request.map_id, job->status.job_id);
      std::filesystem::remove_all(commit_stage);
      std::filesystem::remove_all(commit_backup);
      CopyTree(staging_map, commit_stage, true);
      VerifyRequiredArtifacts(commit_stage, job->request);
      WriteTextAtomic(commit_stage / MapStore::ContentEpochFilename(),
                      std::to_string(store_.AllocateContentEpoch()) + "\n");
      SetPreparedMap(job, map_dir);
      SyncTree(commit_stage);

      SetStatus(job, SaveJobState::kRunning, SavePhase::kCommit, 0.90, "committing map");
      if (!BeforePhase(job, SavePhase::kCommit, &map_lock,
                       [&commit_stage]() { std::filesystem::remove_all(commit_stage); })) {
        return;
      }
      const bool had_map = std::filesystem::exists(map_dir);
      if (had_map) {
        std::filesystem::rename(map_dir, commit_backup);
      }
      try {
        std::filesystem::rename(commit_stage, map_dir);
        SyncPath(store_.RootDir(), true);
      } catch (...) {
        std::error_code restore_error;
        if (std::filesystem::exists(map_dir)) {
          std::filesystem::remove_all(map_dir, restore_error);
          restore_error.clear();
        }
        if (had_map) {
          std::filesystem::rename(commit_backup, map_dir, restore_error);
        } else {
          std::filesystem::remove_all(commit_backup, restore_error);
        }
        if (restore_error) {
          throw std::runtime_error("map commit failed and previous map restore failed: " +
                                   restore_error.message());
        }
        throw;
      }
      UpdateDurableEvidence(job, true, std::nullopt, "MAP_COMMITTED");
      std::filesystem::remove_all(commit_backup);
      if (job->request.activate_on_success) {
        const auto activated = store_.SetActiveMapWhileLocked(job->request.map_id, true, *map_lock);
        if (!activated.ok) {
          throw std::runtime_error("map committed but activation failed: " + activated.message);
        }
        UpdateDurableEvidence(job, std::nullopt, true, "ACTIVATION_SUCCEEDED");
      }
      std::error_code cleanup_error;
      std::filesystem::remove_all(WorkRoot(job->status.job_id), cleanup_error);
      std::filesystem::remove_all(job->status.capture_dir, cleanup_error);
      finish(SaveJobState::kSucceeded, "SaveMap saved", "");
    } catch (const std::exception &exc) {
      const auto map_dir = store_.MapPath(job->request.map_id);
      const auto commit_stage = CommitStage(store_.RootDir(), job->status.job_id);
      const auto commit_backup =
          CommitBackup(store_.RootDir(), job->request.map_id, job->status.job_id);
      const bool committed = job->map_committed && std::filesystem::is_directory(map_dir);
      if (committed) {
        const bool activation_succeeded =
            job->status.activation_requested && store_.ActiveMapId() == job->request.map_id;
        UpdateDurableEvidence(job, std::nullopt, activation_succeeded);
        std::error_code cleanup_error;
        std::filesystem::remove_all(commit_backup, cleanup_error);
        std::filesystem::remove_all(commit_stage, cleanup_error);
        std::filesystem::remove_all(WorkRoot(job->status.job_id), cleanup_error);
        std::filesystem::remove_all(job->status.capture_dir, cleanup_error);
        if (job->status.activation_requested && !activation_succeeded) {
          finish(SaveJobState::kFailed,
                 std::string("map committed but requested activation failed: ") + exc.what(),
                 "activation_failed_after_commit");
        } else {
          finish(SaveJobState::kSucceeded,
                 std::string("recovered committed map after finalization error: ") + exc.what(),
                 "");
        }
      } else if (CancelRequested(job->status.job_id)) {
        BeforePhase(job, job->status.phase, &map_lock);
      } else {
        std::error_code cleanup_error;
        std::filesystem::remove_all(commit_stage, cleanup_error);
        if (std::filesystem::is_directory(commit_backup) && !std::filesystem::exists(map_dir)) {
          std::filesystem::rename(commit_backup, map_dir, cleanup_error);
        }
        finish(SaveJobState::kFailed, exc.what(), "save_map_failed");
      }
    }
  }

  MapStore &store_;
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

SaveMapEngine::SaveMapEngine(MapStore &store, SaveMapHooks hooks)
    : impl_(std::make_unique<Impl>(store, std::move(hooks))) {}

SaveMapEngine::~SaveMapEngine() = default;

SaveMapResult SaveMapEngine::Begin(const SaveMapRequest &request) {
  return impl_->Begin(request);
}

SaveMapResult SaveMapEngine::ProvideSnapshot(const std::string &job_id,
                                             const MapSnapshot &snapshot) {
  return impl_->ProvideSnapshot(job_id, snapshot);
}

SaveMapResult SaveMapEngine::RejectSnapshot(const std::string &job_id,
                                            const std::string &reason_code,
                                            const std::string &message) {
  return impl_->RejectSnapshot(job_id, reason_code, message);
}

SaveMapResult SaveMapEngine::Cancel(const std::string &job_id) {
  return impl_->Cancel(job_id);
}

SaveMapResult SaveMapEngine::Retry(const std::string &job_id) {
  return impl_->Retry(job_id);
}

std::optional<SaveMapStatus> SaveMapEngine::GetStatus(const std::string &job_id) const {
  return impl_->GetStatus(job_id);
}

std::vector<SaveMapStatus> SaveMapEngine::ListStatuses(std::size_t limit) const {
  return impl_->ListStatuses(limit);
}

std::optional<SaveMapStatus> SaveMapEngine::Wait(const std::string &job_id,
                                                 std::chrono::milliseconds timeout) const {
  return impl_->Wait(job_id, timeout);
}

void SaveMapEngine::Recover() {
  impl_->Recover();
}

std::string SaveMapEngine::BeginJson(const SaveMapRequest &request) {
  const auto result = Begin(request);
  return "{\"action\":\"save_map_begin\",\"accepted\":" +
         std::string(result.accepted ? "true" : "false") +
         ",\"replayed\":" + (result.replayed ? "true" : "false") +
         ",\"reason_code\":" + JsonString(result.reason_code) +
         ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::ProvideSnapshotJson(const std::string &job_id,
                                               const MapSnapshot &snapshot) {
  const auto result = ProvideSnapshot(job_id, snapshot);
  return "{\"action\":\"save_map_snapshot\",\"accepted\":" +
         std::string(result.accepted ? "true" : "false") +
         ",\"replayed\":" + (result.replayed ? "true" : "false") +
         ",\"reason_code\":" + JsonString(result.reason_code) +
         ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::RejectSnapshotJson(const std::string &job_id,
                                              const std::string &reason_code,
                                              const std::string &message) {
  const auto result = RejectSnapshot(job_id, reason_code, message);
  return "{\"action\":\"save_map_snapshot\",\"accepted\":" +
         std::string(result.accepted ? "true" : "false") +
         ",\"replayed\":" + (result.replayed ? "true" : "false") +
         ",\"reason_code\":" + JsonString(result.reason_code) +
         ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::CancelJson(const std::string &job_id) {
  const auto result = Cancel(job_id);
  return "{\"action\":\"save_map_cancel\",\"accepted\":" +
         std::string(result.accepted ? "true" : "false") +
         ",\"replayed\":" + (result.replayed ? "true" : "false") +
         ",\"reason_code\":" + JsonString(result.reason_code) +
         ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::RetryJson(const std::string &job_id) {
  const auto result = Retry(job_id);
  return "{\"action\":\"save_map_retry\",\"accepted\":" +
         std::string(result.accepted ? "true" : "false") +
         ",\"replayed\":" + (result.replayed ? "true" : "false") +
         ",\"reason_code\":" + JsonString(result.reason_code) +
         ",\"status\":" + SaveMapStatusJson(result.status) + "}";
}

std::string SaveMapEngine::GetStatusJson(const std::string &job_id) const {
  const auto status = GetStatus(job_id);
  if (!status.has_value()) {
    return "{\"action\":\"save_map_status\",\"success\":false,"
           "\"reason_code\":\"job_not_found\",\"job_id\":" +
           JsonString(job_id) + "}";
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

}  // namespace lingtu::maps
