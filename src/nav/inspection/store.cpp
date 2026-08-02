#include "store.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace lingtu::nav::inspection {
namespace {

constexpr const char* kHeader = "LINGTU_INSPECTION_ROUTE\t1";

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

Store::Store(std::filesystem::path map_root)
    : map_root_(std::filesystem::absolute(std::move(map_root))) {}

StoreResult Store::Put(const Route& route) {
  const auto validation = ValidateRoute(route);
  if (!validation.ok) return {false, validation.reason};
  if (!SafeField(route.name)) return {false, "invalid_route_name"};
  for (const auto& point : route.points) {
    if (!SafeField(point.action)) return {false, "invalid_point_action"};
  }
  const auto existing = Get(route.map_id, route.id);
  if (!existing && route.revision != 1U) {
    return {false, "new_route_revision_must_be_one"};
  }
  if (existing && route.revision != existing->revision + 1U) {
    return {false, "route_revision_must_increment_by_one"};
  }

  const auto directory = RouteDirectory(route.map_id);
  const auto target = RoutePath(route.map_id, route.id);
  const auto temporary = TemporaryPath(target);
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) return {false, "route_directory_create_failed:" + ec.message()};

  std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
  if (!out) return {false, "route_temporary_open_failed"};
  out << kHeader << "\n";
  out << "R\t" << route.id << "\t" << route.name << "\t" << route.map_id << "\t"
      << route.map_version << "\t" << route.revision << "\t" << route.loop_count << "\t"
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
    route.map_version = ParseNumber<std::int64_t>(route_fields[4]);
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
        route->map_version,
        route->revision,
        route->points.size(),
    });
  }
  std::sort(routes.begin(), routes.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.id < rhs.id;
  });
  return routes;
}

StoreResult Store::PutStatus(const RunStatus& status) {
  const auto directory = map_root_ / ".inspection";
  const auto target = directory / "run_status.json";
  const auto temporary = TemporaryPath(target);
  std::error_code ec;
  std::filesystem::create_directories(directory, ec);
  if (ec) return {false, "status_directory_create_failed:" + ec.message()};
  std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
  if (!out) return {false, "status_temporary_open_failed"};
  out << RunStatusToJson(status) << "\n";
  out.flush();
  if (!out) {
    out.close();
    std::filesystem::remove(temporary, ec);
    return {false, "status_temporary_write_failed"};
  }
  out.close();
  std::string error;
  if (!detail::AtomicPublishFile(temporary, target, &error)) return {false, error};
  return {true, "status_saved"};
}

std::string Store::StatusJson() const {
  const auto path = map_root_ / ".inspection" / "run_status.json";
  std::ifstream in(path, std::ios::binary);
  if (!in) return RunStatusToJson({});
  std::ostringstream out;
  out << in.rdbuf();
  const auto value = out.str();
  return value.empty() ? RunStatusToJson({}) : value;
}

std::filesystem::path Store::RouteDirectory(const std::string& map_id) const {
  return map_root_ / map_id / "inspection" / "routes";
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
      << "\"map_version\":" << route.map_version << ","
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
        << "\"map_version\":" << route.map_version << ','
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
      << "\"map_version\":" << status.map_version << ','
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
      << "\"map_version\":" << status.map_version << ','
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
