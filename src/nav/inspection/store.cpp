#include "store.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#endif

namespace lingtu::nav::inspection {
namespace {

constexpr const char* kHeader = "LINGTU_INSPECTION_ROUTE\t1";
constexpr const char* kCheckpointHeader =
    "LINGTU_INSPECTION_TASK_EVENT_CHECKPOINT\t1\n";
constexpr std::uintmax_t kMaxCheckpointBytes = 64U * 1024U;

std::vector<std::string> SplitTab(const std::string& line) {
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  for (;;) {
    const auto end = line.find('\t', begin);
    fields.push_back(line.substr(begin, end - begin));
    if (end == std::string::npos) break;
    begin = end + 1U;
  }
  return fields;
}

bool SafeField(const std::string& value) {
  return value.find_first_of("\t\r\n") == std::string::npos &&
      value.find('\0') == std::string::npos;
}

bool SafeIdentifier(const std::string& value) {
  if (value.empty() || value.size() > 128U || value.front() == '.' ||
      value.front() == '-' || value.find("..") != std::string::npos) {
    return false;
  }
  for (const unsigned char ch : value) {
    if (!std::isalnum(ch) && ch != '-' && ch != '_' && ch != '.') return false;
  }
  return true;
}

std::string EscapeJson(const std::string& value) {
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
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<unsigned int>(ch) << std::dec;
        } else {
          out << static_cast<char>(ch);
        }
    }
  }
  return out.str();
}

FailurePolicy ParsePolicy(const std::string& value) {
  if (value == "retry") return FailurePolicy::kRetry;
  if (value == "skip") return FailurePolicy::kSkip;
  return FailurePolicy::kStop;
}

template <typename T>
T ParseNumber(const std::string& value) {
  std::istringstream in(value);
  T parsed{};
  in >> parsed;
  if (!in || !in.eof()) throw std::runtime_error("invalid_numeric_field");
  return parsed;
}

std::filesystem::path TemporaryPath(const std::filesystem::path& path) {
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  return path.string() + ".tmp." + std::to_string(tick);
}

class RouteFileLock {
 public:
  explicit RouteFileLock(const std::filesystem::path& path) {
#ifdef _WIN32
    handle_ = CreateFileW(
        path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) return;
    OVERLAPPED overlapped{};
    if (LockFileEx(handle_, LOCKFILE_EXCLUSIVE_LOCK, 0, MAXDWORD, MAXDWORD, &overlapped) ==
        FALSE) {
      CloseHandle(handle_);
      handle_ = INVALID_HANDLE_VALUE;
      return;
    }
    locked_ = true;
#else
    descriptor_ = ::open(path.c_str(), O_CREAT | O_RDWR, 0600);
    if (descriptor_ < 0) return;
    if (::flock(descriptor_, LOCK_EX) != 0) {
      ::close(descriptor_);
      descriptor_ = -1;
      return;
    }
    locked_ = true;
#endif
  }

  RouteFileLock(const RouteFileLock&) = delete;
  RouteFileLock& operator=(const RouteFileLock&) = delete;

  ~RouteFileLock() {
#ifdef _WIN32
    if (handle_ == INVALID_HANDLE_VALUE) return;
    if (locked_) {
      OVERLAPPED overlapped{};
      UnlockFileEx(handle_, 0, MAXDWORD, MAXDWORD, &overlapped);
    }
    CloseHandle(handle_);
#else
    if (descriptor_ < 0) return;
    if (locked_) ::flock(descriptor_, LOCK_UN);
    ::close(descriptor_);
#endif
  }

  bool locked() const noexcept { return locked_; }

 private:
  bool locked_{false};
#ifdef _WIN32
  HANDLE handle_{INVALID_HANDLE_VALUE};
#else
  int descriptor_{-1};
#endif
};

std::filesystem::path RouteLockPath(const std::filesystem::path& target) {
  return target.string() + ".lock";
}

std::string HexEncode(const std::string& value) {
  static constexpr char kDigits[] = "0123456789abcdef";
  std::string encoded;
  encoded.reserve(value.size() * 2U);
  for (const unsigned char byte : value) {
    encoded.push_back(kDigits[byte >> 4U]);
    encoded.push_back(kDigits[byte & 0x0fU]);
  }
  return encoded;
}

bool HexDecode(std::string_view encoded, std::string* value) {
  if (value == nullptr || encoded.size() % 2U != 0U) return false;
  auto nibble = [](char ch) -> int {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    return -1;
  };
  value->clear();
  value->reserve(encoded.size() / 2U);
  for (std::size_t index = 0U; index < encoded.size(); index += 2U) {
    const int high = nibble(encoded[index]);
    const int low = nibble(encoded[index + 1U]);
    if (high < 0 || low < 0) return false;
    value->push_back(static_cast<char>((high << 4U) | low));
  }
  return true;
}

std::string DoubleField(double value) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17) << value;
  return out.str();
}

void AppendField(std::string* payload, std::string_view name, std::string value) {
  payload->append(name.data(), name.size());
  payload->push_back('\t');
  payload->append(std::move(value));
  payload->push_back('\n');
}

std::string SerializeCheckpointPayload(const TaskEvent& event) {
  const auto& status = event.status;
  std::string payload{kCheckpointHeader};
  AppendField(&payload, "sequence", std::to_string(event.sequence));
  AppendField(&payload, "timestamp", DoubleField(event.timestamp_s));
  AppendField(&payload, "kind", std::to_string(static_cast<std::int32_t>(event.kind)));
  AppendField(&payload, "event_request_id", HexEncode(event.request_id));
  AppendField(&payload, "state", std::to_string(static_cast<std::int32_t>(status.state)));
  AppendField(&payload, "task_id", HexEncode(status.task_id));
  AppendField(&payload, "run_id", HexEncode(status.run_id));
  AppendField(&payload, "command_request_id", HexEncode(status.request_id));
  AppendField(&payload, "map_id", HexEncode(status.map_id));
  AppendField(&payload, "map_content_epoch", std::to_string(status.map_content_epoch));
  AppendField(&payload, "route_id", HexEncode(status.route_id));
  AppendField(&payload, "route_revision", std::to_string(status.route_revision));
  AppendField(&payload, "point_index", std::to_string(status.point_index));
  AppendField(&payload, "point_count", std::to_string(status.point_count));
  AppendField(&payload, "loop_index", std::to_string(status.loop_index));
  AppendField(&payload, "retry_count", std::to_string(status.retry_count));
  AppendField(&payload, "point_id", HexEncode(status.point_id));
  AppendField(&payload, "action", HexEncode(status.action));
  AppendField(&payload, "action_request_id", HexEncode(status.action_request_id));
  AppendField(&payload, "evidence_id", HexEncode(status.evidence_id));
  AppendField(&payload, "phase_started_at", DoubleField(status.phase_started_at_s));
  AppendField(&payload, "stable_since", DoubleField(status.stable_since_s));
  AppendField(&payload, "deadline", DoubleField(status.deadline_s));
  AppendField(&payload, "reason", HexEncode(status.reason));
  return payload;
}

template <typename T>
bool ParseInteger(std::string_view value, T* parsed) {
  if (parsed == nullptr || value.empty()) return false;
  T candidate{};
  const auto result = std::from_chars(value.data(), value.data() + value.size(), candidate);
  if (result.ec != std::errc{} || result.ptr != value.data() + value.size()) return false;
  *parsed = candidate;
  return true;
}

bool ParseDouble(std::string_view value, double* parsed) {
  if (parsed == nullptr || value.empty()) return false;
  std::istringstream in{std::string(value)};
  in.imbue(std::locale::classic());
  double candidate = 0.0;
  in >> std::noskipws >> candidate;
  if (!in || !in.eof() || !std::isfinite(candidate)) return false;
  *parsed = candidate;
  return true;
}

bool ParseCheckpointPayload(
    const std::string& payload,
    TaskEvent* event,
    std::string* error) {
  static constexpr std::array<std::string_view, 24U> kFields{
      "sequence",          "timestamp",       "kind",        "event_request_id",
      "state",             "task_id",         "run_id",      "command_request_id",
      "map_id",            "map_content_epoch",     "route_id",    "route_revision",
      "point_index",       "point_count",     "loop_index",  "retry_count",
      "point_id",          "action",          "action_request_id",
      "evidence_id",       "phase_started_at", "stable_since", "deadline",
      "reason",
  };
  if (event == nullptr || payload.compare(0U, std::char_traits<char>::length(kCheckpointHeader),
                                          kCheckpointHeader) != 0) {
    if (error != nullptr) *error = "checkpoint_header_invalid";
    return false;
  }

  std::vector<std::string_view> values;
  values.reserve(kFields.size());
  std::size_t begin = std::char_traits<char>::length(kCheckpointHeader);
  for (std::size_t index = 0U; index < kFields.size(); ++index) {
    const auto end = payload.find('\n', begin);
    if (end == std::string::npos) {
      if (error != nullptr) *error = "checkpoint_field_missing";
      return false;
    }
    const std::string_view line(payload.data() + begin, end - begin);
    const auto separator = line.find('\t');
    if (separator == std::string_view::npos ||
        line.find('\t', separator + 1U) != std::string_view::npos ||
        line.substr(0U, separator) != kFields[index]) {
      if (error != nullptr) *error = "checkpoint_field_invalid:" + std::string(kFields[index]);
      return false;
    }
    values.push_back(line.substr(separator + 1U));
    begin = end + 1U;
  }
  if (begin != payload.size()) {
    if (error != nullptr) *error = "checkpoint_extra_fields";
    return false;
  }

  TaskEvent parsed;
  std::int32_t kind = 0;
  std::int32_t state = 0;
  std::uint64_t point_index = 0U;
  std::uint64_t point_count = 0U;
  if (!ParseInteger(values[0], &parsed.sequence) ||
      !ParseDouble(values[1], &parsed.timestamp_s) ||
      !ParseInteger(values[2], &kind) ||
      !HexDecode(values[3], &parsed.request_id) ||
      !ParseInteger(values[4], &state) ||
      !HexDecode(values[5], &parsed.status.task_id) ||
      !HexDecode(values[6], &parsed.status.run_id) ||
      !HexDecode(values[7], &parsed.status.request_id) ||
      !HexDecode(values[8], &parsed.status.map_id) ||
      !ParseInteger(values[9], &parsed.status.map_content_epoch) ||
      !HexDecode(values[10], &parsed.status.route_id) ||
      !ParseInteger(values[11], &parsed.status.route_revision) ||
      !ParseInteger(values[12], &point_index) ||
      !ParseInteger(values[13], &point_count) ||
      !ParseInteger(values[14], &parsed.status.loop_index) ||
      !ParseInteger(values[15], &parsed.status.retry_count) ||
      !HexDecode(values[16], &parsed.status.point_id) ||
      !HexDecode(values[17], &parsed.status.action) ||
      !HexDecode(values[18], &parsed.status.action_request_id) ||
      !HexDecode(values[19], &parsed.status.evidence_id) ||
      !ParseDouble(values[20], &parsed.status.phase_started_at_s) ||
      !ParseDouble(values[21], &parsed.status.stable_since_s) ||
      !ParseDouble(values[22], &parsed.status.deadline_s) ||
      !HexDecode(values[23], &parsed.status.reason) ||
      point_index > std::numeric_limits<std::size_t>::max() ||
      point_count > std::numeric_limits<std::size_t>::max()) {
    if (error != nullptr) *error = "checkpoint_value_invalid";
    return false;
  }
  parsed.kind = static_cast<TaskEventKind>(kind);
  parsed.status.state = static_cast<RunState>(state);
  parsed.status.point_index = static_cast<std::size_t>(point_index);
  parsed.status.point_count = static_cast<std::size_t>(point_count);
  const auto validation = ValidateTaskEvent(parsed);
  if (!validation.ok) {
    if (error != nullptr) *error = "checkpoint_event_invalid:" + validation.reason;
    return false;
  }
  if (SerializeCheckpointPayload(parsed) != payload) {
    if (error != nullptr) *error = "checkpoint_not_canonical";
    return false;
  }
  *event = std::move(parsed);
  return true;
}

}  // namespace

namespace detail {

bool AtomicPublishFile(
    const std::filesystem::path& temporary,
    const std::filesystem::path& target,
    std::string* error) {
  std::error_code ec;
#ifdef _WIN32
  if (MoveFileExW(
          temporary.c_str(),
          target.c_str(),
          MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH) != FALSE) {
    return true;
  }
  ec = std::error_code(static_cast<int>(GetLastError()), std::system_category());
#else
  std::filesystem::rename(temporary, target, ec);
  if (!ec) return true;
#endif
  if (error != nullptr) *error = "route_publish_failed:" + ec.message();
  std::error_code cleanup_ec;
  std::filesystem::remove(temporary, cleanup_ec);
  return false;
}

}  // namespace detail

Store::Store(std::filesystem::path data_dir)
    : data_dir_(std::filesystem::absolute(std::move(data_dir))) {}

StoreResult Store::Put(const Route& route) {
  const auto validation = ValidateRoute(route);
  if (!validation.ok) return {false, validation.reason};
  if (!SafeField(route.name)) return {false, "invalid_route_name"};
  for (const auto& point : route.points) {
    if (!SafeField(point.action)) return {false, "invalid_point_action"};
  }
  const auto directory = RouteDirectory(route.map_id);
  const auto target = RoutePath(route.map_id, route.id);
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) return {false, "route_directory_create_failed:" + ec.message()};

  RouteFileLock lock(RouteLockPath(target));
  if (!lock.locked()) return {false, "route_lock_failed"};

  const auto existing = Get(route.map_id, route.id);
  if (!existing && route.revision != 1U) {
    return {false, "new_route_revision_must_be_one"};
  }
  if (existing && route.revision != existing->revision + 1U) {
    return {false, "route_revision_must_increment_by_one"};
  }

  const auto temporary = TemporaryPath(target);

  std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
  if (!out) return {false, "route_temporary_open_failed"};
  out << kHeader << "\n";
  out << "R\t" << route.id << "\t" << route.name << "\t" << route.map_id << "\t"
      << route.map_content_epoch << "\t" << route.revision << "\t" << route.loop_count << "\t"
      << FailurePolicyName(route.failure_policy) << "\t" << route.max_retries << "\n";
  out << std::setprecision(17);
  for (const auto& point : route.points) {
    out << "P\t" << point.id << "\t" << point.frame_id << "\t"
        << point.x_m << "\t" << point.y_m << "\t" << point.z_m << "\t"
        << (point.has_yaw ? 1 : 0) << "\t" << point.yaw_rad << "\t"
        << point.position_tolerance_m << "\t" << point.yaw_tolerance_rad << "\t"
        << point.dwell_s << "\t" << (point.enabled ? 1 : 0) << "\t"
        << point.action << "\n";
  }
  out.flush();
  if (!out) {
    out.close();
    std::filesystem::remove(temporary, ec);
    return {false, "route_temporary_write_failed"};
  }
  out.close();
  std::string error;
  if (!detail::AtomicPublishFile(temporary, target, &error)) return {false, error};
  return {true, "route_saved"};
}

std::optional<Route> Store::Get(
    const std::string& map_id,
    const std::string& route_id) const {
  if (!SafeIdentifier(map_id) || !SafeIdentifier(route_id)) return std::nullopt;
  std::ifstream in(RoutePath(map_id, route_id), std::ios::binary);
  if (!in) return std::nullopt;
  std::string line;
  if (!std::getline(in, line) || line != kHeader) return std::nullopt;
  if (!std::getline(in, line)) return std::nullopt;
  const auto route_fields = SplitTab(line);
  if (route_fields.size() != 9U || route_fields[0] != "R") return std::nullopt;
  try {
    Route route;
    route.id = route_fields[1];
    route.name = route_fields[2];
    route.map_id = route_fields[3];
    route.map_content_epoch = ParseNumber<std::int64_t>(route_fields[4]);
    route.revision = ParseNumber<std::uint64_t>(route_fields[5]);
    route.loop_count = ParseNumber<std::uint32_t>(route_fields[6]);
    route.failure_policy = ParsePolicy(route_fields[7]);
    route.max_retries = ParseNumber<std::uint32_t>(route_fields[8]);
    while (std::getline(in, line)) {
      const auto fields = SplitTab(line);
      if (fields.size() != 13U || fields[0] != "P") return std::nullopt;
      Point point;
      point.id = fields[1];
      point.frame_id = fields[2];
      point.x_m = ParseNumber<double>(fields[3]);
      point.y_m = ParseNumber<double>(fields[4]);
      point.z_m = ParseNumber<double>(fields[5]);
      point.has_yaw = ParseNumber<int>(fields[6]) != 0;
      point.yaw_rad = ParseNumber<double>(fields[7]);
      point.position_tolerance_m = ParseNumber<double>(fields[8]);
      point.yaw_tolerance_rad = ParseNumber<double>(fields[9]);
      point.dwell_s = ParseNumber<double>(fields[10]);
      point.enabled = ParseNumber<int>(fields[11]) != 0;
      point.action = fields[12];
      route.points.push_back(std::move(point));
    }
    if (route.map_id != map_id || route.id != route_id || !ValidateRoute(route).ok) {
      return std::nullopt;
    }
    return route;
  } catch (...) {
    return std::nullopt;
  }
}

StoreResult Store::Delete(const std::string& map_id, const std::string& route_id) {
  if (!SafeIdentifier(map_id) || !SafeIdentifier(route_id)) {
    return {false, "invalid_route_identifier"};
  }
  const auto path = RoutePath(map_id, route_id);
  std::error_code ec;
  if (!std::filesystem::exists(path.parent_path(), ec) || ec) {
    return {false, "route_not_found"};
  }
  RouteFileLock lock(RouteLockPath(path));
  if (!lock.locked()) return {false, "route_lock_failed"};
  if (!std::filesystem::exists(path, ec)) return {false, "route_not_found"};
  if (!std::filesystem::remove(path, ec) || ec) {
    return {false, "route_delete_failed:" + ec.message()};
  }
  return {true, "route_deleted"};
}

std::vector<RouteSummary> Store::List(const std::string& map_id) const {
  std::vector<RouteSummary> routes;
  if (!SafeIdentifier(map_id)) return routes;
  std::error_code ec;
  const auto directory = RouteDirectory(map_id);
  if (!std::filesystem::exists(directory, ec)) return routes;
  for (const auto& entry : std::filesystem::directory_iterator(directory, ec)) {
    if (ec || !entry.is_regular_file() || entry.path().extension() != ".ltroute") continue;
    const auto route = Get(map_id, entry.path().stem().string());
    if (!route) continue;
    routes.push_back({
        route->id,
        route->name,
        route->map_id,
        route->map_content_epoch,
        route->revision,
        route->points.size(),
    });
  }
  std::sort(routes.begin(), routes.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.id < rhs.id;
  });
  return routes;
}

std::string Store::StatusJson() const {
  const auto path = data_dir_ / "run_status.json";
  std::ifstream in(path, std::ios::binary);
  if (!in) return RunStatusToJson({});
  std::ostringstream out;
  out << in.rdbuf();
  const auto value = out.str();
  return value.empty() ? RunStatusToJson({}) : value;
}

StoreResult Store::PutTaskEventCheckpoint(const TaskEvent& event) {
  const auto validation = ValidateTaskEvent(event);
  if (!validation.ok) return {false, "checkpoint_invalid:" + validation.reason};

  const auto target = TaskEventCheckpointPath();
  const auto directory = target.parent_path();
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) return {false, "checkpoint_directory_create_failed:" + ec.message()};

  const auto existing = LoadTaskEventCheckpoint();
  if (existing.state == TaskEventCheckpointLoadState::kCorrupt) {
    return {false, "checkpoint_existing_corrupt:" + existing.reason};
  }
  if (existing.state == TaskEventCheckpointLoadState::kIoError) {
    return {false, "checkpoint_existing_io_error:" + existing.reason};
  }
  if (existing.state == TaskEventCheckpointLoadState::kNotFound) {
    if (event.sequence != 1U) {
      return {false, "checkpoint_first_sequence_must_be_one"};
    }
  } else if (existing.loaded()) {
    const auto& current = *existing.event;
    if (event.sequence < current.sequence) {
      return {false, "checkpoint_sequence_regression"};
    }
    if (event.sequence == current.sequence) {
      if (SerializeCheckpointPayload(event) != SerializeCheckpointPayload(current)) {
        return {false, "checkpoint_sequence_conflict"};
      }
      return {true, "checkpoint_unchanged"};
    }
    if (current.sequence == std::numeric_limits<std::uint64_t>::max() ||
        event.sequence != current.sequence + 1U) {
      return {false, "checkpoint_sequence_gap"};
    }
  }

  const auto temporary = TemporaryPath(target);
  std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
  if (!out) return {false, "checkpoint_temporary_open_failed"};
  out << SerializeCheckpointPayload(event);
  out.flush();
  if (!out) {
    out.close();
    std::filesystem::remove(temporary, ec);
    return {false, "checkpoint_temporary_write_failed"};
  }
  out.close();
  std::string error;
  if (!detail::AtomicPublishFile(temporary, target, &error)) {
    return {false, error};
  }
  return {true, "checkpoint_saved"};
}

TaskEventCheckpointLoadResult Store::LoadTaskEventCheckpoint() const {
  const auto path = TaskEventCheckpointPath();
  std::error_code ec;
  const bool exists = std::filesystem::exists(path, ec);
  if (ec) {
    return {TaskEventCheckpointLoadState::kIoError, std::nullopt,
            "checkpoint_stat_failed:" + ec.message()};
  }
  if (!exists) {
    return {TaskEventCheckpointLoadState::kNotFound, std::nullopt,
            "checkpoint_not_found"};
  }
  const auto size = std::filesystem::file_size(path, ec);
  if (ec) {
    return {TaskEventCheckpointLoadState::kIoError, std::nullopt,
            "checkpoint_size_failed:" + ec.message()};
  }
  if (size == 0U || size > kMaxCheckpointBytes) {
    return {TaskEventCheckpointLoadState::kCorrupt, std::nullopt,
            "checkpoint_size_invalid"};
  }

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return {TaskEventCheckpointLoadState::kIoError, std::nullopt,
            "checkpoint_open_failed"};
  }
  std::string serialized{
      std::istreambuf_iterator<char>{in},
      std::istreambuf_iterator<char>{}};
  if (in.bad()) {
    return {TaskEventCheckpointLoadState::kIoError, std::nullopt,
            "checkpoint_read_failed"};
  }

  TaskEvent event;
  std::string error;
  if (!ParseCheckpointPayload(serialized, &event, &error)) {
    return {TaskEventCheckpointLoadState::kCorrupt, std::nullopt, std::move(error)};
  }
  return {TaskEventCheckpointLoadState::kLoaded, std::move(event), "checkpoint_loaded"};
}

std::filesystem::path Store::TaskEventCheckpointPath() const {
  return data_dir_ / "task_event_checkpoint.v1";
}

std::filesystem::path Store::RouteDirectory(const std::string& map_id) const {
  return data_dir_ / "routes" / map_id;
}

std::filesystem::path Store::RoutePath(
    const std::string& map_id,
    const std::string& route_id) const {
  return RouteDirectory(map_id) / (route_id + ".ltroute");
}

std::string RouteToJson(const Route& route) {
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"schema_version\":\"nav.inspection.route.v1\","
      << "\"id\":\"" << EscapeJson(route.id) << "\","
      << "\"name\":\"" << EscapeJson(route.name) << "\","
      << "\"map_id\":\"" << EscapeJson(route.map_id) << "\","
      << "\"map_content_epoch\":" << route.map_content_epoch << ","
      << "\"revision\":" << route.revision << ","
      << "\"loop_count\":" << route.loop_count << ","
      << "\"failure_policy\":\"" << FailurePolicyName(route.failure_policy) << "\","
      << "\"max_retries\":" << route.max_retries << ",\"points\":[";
  for (std::size_t i = 0; i < route.points.size(); ++i) {
    if (i > 0U) out << ',';
    const auto& point = route.points[i];
    out << "{\"id\":\"" << EscapeJson(point.id) << "\","
        << "\"frame_id\":\"" << EscapeJson(point.frame_id) << "\","
        << "\"x\":" << point.x_m << ",\"y\":" << point.y_m
        << ",\"z\":" << point.z_m << ",\"yaw\":";
    if (point.has_yaw) out << point.yaw_rad; else out << "null";
    out << ",\"position_tolerance_m\":" << point.position_tolerance_m
        << ",\"yaw_tolerance_rad\":" << point.yaw_tolerance_rad
        << ",\"dwell_s\":" << point.dwell_s
        << ",\"action\":\"" << EscapeJson(point.action) << "\","
        << "\"enabled\":" << (point.enabled ? "true" : "false") << '}';
  }
  out << "]}";
  return out.str();
}

std::string RouteListToJson(const std::vector<RouteSummary>& routes) {
  std::ostringstream out;
  out << "{\"schema_version\":\"nav.inspection.route-list.v1\",\"routes\":[";
  for (std::size_t i = 0; i < routes.size(); ++i) {
    if (i > 0U) out << ',';
    const auto& route = routes[i];
    out << "{\"id\":\"" << EscapeJson(route.id) << "\","
        << "\"name\":\"" << EscapeJson(route.name) << "\","
        << "\"map_id\":\"" << EscapeJson(route.map_id) << "\","
        << "\"map_content_epoch\":" << route.map_content_epoch << ','
        << "\"revision\":" << route.revision << ','
        << "\"point_count\":" << route.point_count << '}';
  }
  out << "]}";
  return out.str();
}

std::string RunStatusToJson(const RunStatus& status) {
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"schema_version\":\"nav.inspection.status.v1\","
      << "\"task_id\":\"" << EscapeJson(status.task_id) << "\","
      << "\"run_id\":\"" << EscapeJson(status.run_id) << "\","
      << "\"request_id\":\"" << EscapeJson(status.request_id) << "\","
      << "\"map_id\":\"" << EscapeJson(status.map_id) << "\","
      << "\"map_content_epoch\":" << status.map_content_epoch << ','
      << "\"route_id\":\"" << EscapeJson(status.route_id) << "\","
      << "\"route_revision\":" << status.route_revision << ','
      << "\"state\":\"" << RunStateName(status.state) << "\","
      << "\"state_code\":" << static_cast<std::int32_t>(status.state) << ','
      << "\"point_index\":" << status.point_index << ','
      << "\"point_count\":" << status.point_count << ','
      << "\"loop_index\":" << status.loop_index << ','
      << "\"retry_count\":" << status.retry_count << ','
      << "\"point_id\":\"" << EscapeJson(status.point_id) << "\","
      << "\"action\":\"" << EscapeJson(status.action) << "\","
      << "\"action_request_id\":\"" << EscapeJson(status.action_request_id) << "\","
      << "\"evidence_id\":\"" << EscapeJson(status.evidence_id) << "\","
      << "\"phase_started_at\":" << status.phase_started_at_s << ','
      << "\"stable_since\":" << status.stable_since_s << ','
      << "\"deadline\":" << status.deadline_s << ','
      << "\"reason\":\"" << EscapeJson(status.reason) << "\"}";
  return out.str();
}

std::string TaskEventToJson(const TaskEvent& event) {
  const RunStatus& status = event.status;
  std::ostringstream out;
  out << std::setprecision(17)
      << "{\"schema_version\":\"nav.inspection.task-event.v1\","
      << "\"sequence\":" << event.sequence << ','
      << "\"timestamp\":" << event.timestamp_s << ','
      << "\"kind\":\"" << TaskEventKindName(event.kind) << "\","
      << "\"task_id\":\"" << EscapeJson(status.task_id) << "\","
      << "\"request_id\":\"" << EscapeJson(event.request_id) << "\","
      << "\"command_request_id\":\"" << EscapeJson(status.request_id) << "\","
      << "\"map_id\":\"" << EscapeJson(status.map_id) << "\","
      << "\"map_content_epoch\":" << status.map_content_epoch << ','
      << "\"route_id\":\"" << EscapeJson(status.route_id) << "\","
      << "\"route_revision\":" << status.route_revision << ','
      << "\"state\":\"" << RunStateName(status.state) << "\","
      << "\"state_code\":" << static_cast<std::int32_t>(status.state) << ','
      << "\"point_index\":" << status.point_index << ','
      << "\"point_count\":" << status.point_count << ','
      << "\"loop_index\":" << status.loop_index << ','
      << "\"retry_count\":" << status.retry_count << ','
      << "\"point_id\":\"" << EscapeJson(status.point_id) << "\","
      << "\"action\":\"" << EscapeJson(status.action) << "\","
      << "\"action_request_id\":\"" << EscapeJson(status.action_request_id) << "\","
      << "\"evidence_id\":\"" << EscapeJson(status.evidence_id) << "\","
      << "\"reason\":\"" << EscapeJson(status.reason) << "\"}";
  return out.str();
}

}  // namespace lingtu::nav::inspection
