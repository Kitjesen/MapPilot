#include "octoplanner3d_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <streambuf>
#include <string>
#include <vector>

#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

namespace {

using octoplanner3d::runtime::PlanRequest;
using octoplanner3d::runtime::PlannerOptions;
using octoplanner3d::runtime::Point;

struct BenchmarkOptions {
  int samples{0};
  std::string cache_directory;
};

struct HeadlessRequest {
  std::string map_path;
  PlanRequest plan;
  std::optional<BenchmarkOptions> benchmark;
};

struct BenchmarkResult {
  double cold_prepare_ms{0.0};
  double warmup_ms{0.0};
  std::vector<double> query_ms;
  double cache_restore_ms{0.0};
  double restored_query_ms{0.0};
  std::size_t prepare_count{0};
  std::size_t cache_load_count{0};
  std::size_t map_load_count{0};
  std::size_t restored_prepare_count{0};
  std::size_t restored_cache_load_count{0};
  bool same_result{true};
};

class RedirectStdoutToStderr {
public:
  RedirectStdoutToStderr()
  : original_cout_(std::cout.rdbuf(std::cerr.rdbuf())),
    original_stdout_fd_(duplicateFd(fileNo(stdout)))
  {
    std::fflush(stdout);
    if (original_stdout_fd_ >= 0) {
      duplicateTo(fileNo(stderr), fileNo(stdout));
    }
  }

  ~RedirectStdoutToStderr()
  {
    std::fflush(stdout);
    if (original_stdout_fd_ >= 0) {
      duplicateTo(original_stdout_fd_, fileNo(stdout));
      closeFd(original_stdout_fd_);
    }
    std::cout.rdbuf(original_cout_);
  }

private:
  static int fileNo(FILE * file)
  {
#ifdef _WIN32
    return _fileno(file);
#else
    return fileno(file);
#endif
  }

  static int duplicateFd(int fd)
  {
#ifdef _WIN32
    return _dup(fd);
#else
    return dup(fd);
#endif
  }

  static void duplicateTo(int from, int to)
  {
#ifdef _WIN32
    (void)_dup2(from, to);
#else
    (void)dup2(from, to);
#endif
  }

  static void closeFd(int fd)
  {
#ifdef _WIN32
    (void)_close(fd);
#else
    (void)close(fd);
#endif
  }

  std::streambuf * original_cout_;
  int original_stdout_fd_;
};

std::string readStdin()
{
  std::ostringstream buffer;
  buffer << std::cin.rdbuf();
  return buffer.str();
}

std::string jsonEscape(const std::string & value)
{
  std::ostringstream out;
  for (const char ch : value) {
    switch (ch) {
      case '\\': out << "\\\\"; break;
      case '"': out << "\\\""; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default: out << ch; break;
    }
  }
  return out.str();
}

std::string keyToken(const std::string & key)
{
  return "\"" + key + "\"";
}

std::string extractString(const std::string & json, const std::string & key)
{
  const auto key_pos = json.find(keyToken(key));
  if (key_pos == std::string::npos) {
    throw std::runtime_error("missing string field: " + key);
  }
  const auto colon = json.find(':', key_pos);
  if (colon == std::string::npos) {
    throw std::runtime_error("missing ':' after field: " + key);
  }
  const auto quote = json.find('"', colon + 1);
  if (quote == std::string::npos) {
    throw std::runtime_error("missing opening quote for field: " + key);
  }

  std::string value;
  bool escaped = false;
  for (std::size_t i = quote + 1; i < json.size(); ++i) {
    const char ch = json[i];
    if (escaped) {
      switch (ch) {
        case 'n': value.push_back('\n'); break;
        case 'r': value.push_back('\r'); break;
        case 't': value.push_back('\t'); break;
        default: value.push_back(ch); break;
      }
      escaped = false;
      continue;
    }
    if (ch == '\\') {
      escaped = true;
      continue;
    }
    if (ch == '"') {
      return value;
    }
    value.push_back(ch);
  }
  throw std::runtime_error("unterminated string field: " + key);
}

std::vector<double> extractNumberArray(const std::string & json, const std::string & key)
{
  const auto key_pos = json.find(keyToken(key));
  if (key_pos == std::string::npos) {
    throw std::runtime_error("missing numeric array field: " + key);
  }
  const auto open = json.find('[', key_pos);
  if (open == std::string::npos) {
    throw std::runtime_error("missing '[' for field: " + key);
  }
  int depth = 0;
  std::size_t close = std::string::npos;
  for (std::size_t i = open; i < json.size(); ++i) {
    if (json[i] == '[') {
      ++depth;
    } else if (json[i] == ']') {
      --depth;
      if (depth == 0) {
        close = i;
        break;
      }
    }
  }
  if (close == std::string::npos) {
    throw std::runtime_error("unterminated numeric array field: " + key);
  }

  std::vector<double> values;
  std::string body = json.substr(open + 1, close - open - 1);
  const char * cursor = body.c_str();
  char * end = nullptr;
  while (*cursor != '\0') {
    while (*cursor == ' ' || *cursor == '\n' || *cursor == '\r' ||
           *cursor == '\t' || *cursor == ',') {
      ++cursor;
    }
    if (*cursor == '\0') {
      break;
    }
    const double value = std::strtod(cursor, &end);
    if (end == cursor) {
      throw std::runtime_error("invalid number in field: " + key);
    }
    values.push_back(value);
    cursor = end;
  }
  if (values.size() < 2) {
    throw std::runtime_error("field must contain at least x and y: " + key);
  }
  return values;
}

Point toPoint(const std::vector<double> & values)
{
  Point point;
  point.x = values[0];
  point.y = values[1];
  point.z = values.size() >= 3 ? values[2] : 0.0;
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
    throw std::runtime_error("point contains non-finite value");
  }
  return point;
}

std::string extractObjectBody(const std::string & json, const std::string & key)
{
  const auto key_pos = json.find(keyToken(key));
  if (key_pos == std::string::npos) {
    return "";
  }
  const auto colon = json.find(':', key_pos);
  if (colon == std::string::npos) {
    throw std::runtime_error("missing ':' after field: " + key);
  }
  const auto open = json.find('{', colon + 1);
  if (open == std::string::npos) {
    throw std::runtime_error("missing object for field: " + key);
  }

  int depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (std::size_t i = open; i < json.size(); ++i) {
    const char ch = json[i];
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
    } else if (ch == '}') {
      --depth;
      if (depth == 0) {
        return json.substr(open + 1, i - open - 1);
      }
    }
  }
  throw std::runtime_error("unterminated object field: " + key);
}

const char * skipJsonWhitespace(const char * cursor)
{
  while (*cursor == ' ' || *cursor == '\n' || *cursor == '\r' || *cursor == '\t') {
    ++cursor;
  }
  return cursor;
}

const char * optionalValueCursor(const std::string & json, const std::string & key)
{
  const auto key_pos = json.find(keyToken(key));
  if (key_pos == std::string::npos) {
    return nullptr;
  }
  const auto colon = json.find(':', key_pos);
  if (colon == std::string::npos) {
    throw std::runtime_error("missing ':' after field: " + key);
  }
  return skipJsonWhitespace(json.c_str() + colon + 1);
}

double optionalDouble(
  const std::string & json,
  const std::string & key,
  double default_value)
{
  const char * cursor = optionalValueCursor(json, key);
  if (cursor == nullptr) {
    return default_value;
  }
  char * end = nullptr;
  const double value = std::strtod(cursor, &end);
  if (end == cursor || !std::isfinite(value)) {
    throw std::runtime_error("invalid numeric option: " + key);
  }
  return value;
}

int optionalInt(const std::string & json, const std::string & key, int default_value)
{
  const char * cursor = optionalValueCursor(json, key);
  if (cursor == nullptr) {
    return default_value;
  }
  char * end = nullptr;
  const long value = std::strtol(cursor, &end, 10);
  if (end == cursor) {
    throw std::runtime_error("invalid integer option: " + key);
  }
  return static_cast<int>(value);
}

bool optionalBool(
  const std::string & json,
  const std::string & key,
  bool default_value)
{
  const char * cursor = optionalValueCursor(json, key);
  if (cursor == nullptr) {
    return default_value;
  }
  if (std::string(cursor).rfind("true", 0) == 0) {
    return true;
  }
  if (std::string(cursor).rfind("false", 0) == 0) {
    return false;
  }
  throw std::runtime_error("invalid boolean option: " + key);
}

void applyOptionsFromJson(PlannerOptions & options, const std::string & json)
{
  const std::string body = extractObjectBody(json, "options");
  if (body.empty()) {
    return;
  }
  options.robot_radius = optionalDouble(body, "robot_radius", options.robot_radius);
  options.body_offset_m = optionalDouble(body, "body_offset_m", options.body_offset_m);
  options.body_clearance_below_m = optionalDouble(
    body,
    "body_clearance_below_m",
    options.body_clearance_below_m);
  options.body_clearance_above_m = optionalDouble(
    body,
    "body_clearance_above_m",
    options.body_clearance_above_m);
  options.max_iterations = optionalInt(body, "max_iterations", options.max_iterations);
  options.snap_search_radius_cells = optionalInt(
    body,
    "snap_search_radius_cells",
    options.snap_search_radius_cells);
  options.ground_support = optionalBool(
    body,
    "ground_support",
    options.ground_support);
  options.direct_support_only = optionalBool(
    body,
    "direct_support_only",
    options.direct_support_only);
  options.support_depth_cells = optionalInt(
    body,
    "support_depth_cells",
    options.support_depth_cells);
  options.search_resolution_m = optionalDouble(body, "search_resolution_m", options.search_resolution_m);
  options.surface_closing_radius_m = optionalDouble(
    body, "surface_closing_radius_m", options.surface_closing_radius_m);
  options.surface_max_hole_area_m2 = optionalDouble(
    body, "surface_max_hole_area_m2", options.surface_max_hole_area_m2);
  options.surface_max_slope = optionalDouble(body, "surface_max_slope", options.surface_max_slope);
  options.support_height_m = optionalDouble(
    body,
    "support_height_m",
    options.support_height_m);
  options.support_height_tolerance_m = optionalDouble(
    body,
    "support_height_tolerance_m",
    options.support_height_tolerance_m);
  options.support_patch_radius_cells = optionalInt(
    body,
    "support_patch_radius_cells",
    options.support_patch_radius_cells);
  options.support_patch_min_samples = optionalInt(
    body,
    "support_patch_min_samples",
    options.support_patch_min_samples);
  options.enable_preblocked_costmap = optionalBool(
    body,
    "enable_preblocked_costmap",
    options.enable_preblocked_costmap);
  options.preblocked_costmap_radius_cells = optionalInt(
    body,
    "preblocked_costmap_radius_cells",
    options.preblocked_costmap_radius_cells);
  options.preblocked_costmap_weight = optionalDouble(
    body,
    "preblocked_costmap_weight",
    options.preblocked_costmap_weight);
  options.lowest_traversable_only = optionalBool(
    body,
    "lowest_traversable_only",
    options.lowest_traversable_only);
  options.floor_change_penalty = optionalDouble(
    body,
    "floor_change_penalty",
    options.floor_change_penalty);
  options.max_step_height = optionalDouble(
    body,
    "max_step_height",
    options.max_step_height);
  options.max_slope = optionalDouble(
    body,
    "max_slope",
    options.max_slope);
  options.same_floor_preference = optionalBool(
    body,
    "same_floor_preference",
    options.same_floor_preference);
  options.same_floor_z_tolerance = optionalDouble(
    body,
    "same_floor_z_tolerance",
    options.same_floor_z_tolerance);
  options.obstacle_clearance_radius_cells = optionalInt(
    body,
    "obstacle_clearance_radius_cells",
    options.obstacle_clearance_radius_cells);
  options.obstacle_clearance_weight = optionalDouble(
    body,
    "obstacle_clearance_weight",
    options.obstacle_clearance_weight);
  options.terminal_goal_tolerance_m = optionalDouble(
    body,
    "terminal_goal_tolerance_m",
    options.terminal_goal_tolerance_m);
  options.terminal_goal_xy_tolerance_m = optionalDouble(
    body,
    "terminal_goal_xy_tolerance_m",
    options.terminal_goal_xy_tolerance_m);
  options.terminal_goal_z_tolerance_m = optionalDouble(
    body,
    "terminal_goal_z_tolerance_m",
    options.terminal_goal_z_tolerance_m);
}

HeadlessRequest parseRequest(const std::string & json)
{
  HeadlessRequest request;
  request.map_path = extractString(json, "map_path");
  request.plan.start = toPoint(extractNumberArray(json, "start"));
  request.plan.goal = toPoint(extractNumberArray(json, "goal"));
  if (optionalValueCursor(json, "start_yaw")) request.plan.start.yaw = optionalDouble(json, "start_yaw", 0.0);
  if (optionalValueCursor(json, "goal_yaw")) request.plan.goal.yaw = optionalDouble(json, "goal_yaw", 0.0);
  applyOptionsFromJson(request.plan.options, json);
  const auto benchmark = extractObjectBody(json, "benchmark");
  if (optionalValueCursor(json, "benchmark") != nullptr) {
    const double samples = optionalDouble(benchmark, "samples", 20);
    if (samples < 1 || samples > std::numeric_limits<int>::max() ||
        std::floor(samples) != samples) {
      throw std::runtime_error("benchmark samples must be a positive integer");
    }
    request.benchmark = BenchmarkOptions{
      static_cast<int>(samples), extractString(benchmark, "cache_directory")};
    if (request.benchmark->cache_directory.empty()) {
      throw std::runtime_error("benchmark cache_directory must not be empty");
    }
  }
  if (request.map_path.empty()) {
    throw std::runtime_error("map_path must not be empty");
  }
  return request;
}

void emitConstraints(const PlannerOptions & options)
{
  std::cout << "\"constraints\":{"
            << "\"robot_radius\":" << options.robot_radius << ','
            << "\"body_offset_m\":" << options.body_offset_m << ','
            << "\"body_clearance_below_m\":" << options.body_clearance_below_m << ','
            << "\"body_clearance_above_m\":" << options.body_clearance_above_m << ','
            << "\"max_iterations\":" << options.max_iterations << ','
            << "\"snap_search_radius_cells\":" << options.snap_search_radius_cells << ','
            << "\"ground_support\":"
            << (options.ground_support ? "true" : "false") << ','
            << "\"direct_support_only\":"
            << (options.direct_support_only ? "true" : "false") << ','
            << "\"support_depth_cells\":"
            << options.support_depth_cells << ','
            << "\"search_resolution_m\":" << options.search_resolution_m << ','
            << "\"surface_closing_radius_m\":" << options.surface_closing_radius_m << ','
            << "\"surface_max_hole_area_m2\":" << options.surface_max_hole_area_m2 << ','
            << "\"surface_max_slope\":" << options.surface_max_slope << ','
            << "\"support_height_m\":" << options.support_height_m << ','
            << "\"support_height_tolerance_m\":"
            << options.support_height_tolerance_m << ','
            << "\"support_patch_radius_cells\":"
            << options.support_patch_radius_cells << ','
            << "\"support_patch_min_samples\":"
            << options.support_patch_min_samples << ','
            << "\"enable_preblocked_costmap\":"
            << (options.enable_preblocked_costmap ? "true" : "false") << ','
            << "\"preblocked_costmap_radius_cells\":"
            << options.preblocked_costmap_radius_cells << ','
            << "\"preblocked_costmap_weight\":"
            << options.preblocked_costmap_weight << ','
            << "\"lowest_traversable_only\":"
            << (options.lowest_traversable_only ? "true" : "false") << ','
            << "\"floor_change_penalty\":" << options.floor_change_penalty << ','
            << "\"max_step_height\":" << options.max_step_height << ','
            << "\"max_slope\":" << options.max_slope << ','
            << "\"same_floor_preference\":"
            << (options.same_floor_preference ? "true" : "false") << ','
            << "\"same_floor_z_tolerance\":" << options.same_floor_z_tolerance << ','
            << "\"obstacle_clearance_radius_cells\":"
            << options.obstacle_clearance_radius_cells << ','
            << "\"obstacle_clearance_weight\":" << options.obstacle_clearance_weight << ','
            << "\"terminal_goal_tolerance_m\":" << options.terminal_goal_tolerance_m << ','
            << "\"terminal_goal_xy_tolerance_m\":" << options.terminal_goal_xy_tolerance_m << ','
            << "\"terminal_goal_z_tolerance_m\":" << options.terminal_goal_z_tolerance_m
            << '}';
}

void emitError(const std::string & message)
{
  PlannerOptions options;
  std::cout << "{\"planner\":\"octoplanner3d\",\"protocol_version\":1,"
            << "\"ok\":false,\"error\":\"" << jsonEscape(message)
            << "\",\"diagnostics\":{"
            << "\"source\":\"octoplanner3d_headless\","
            << "\"ros2_required\":false,"
            << "\"pcd_conversion\":"
            << (octoplanner3d::runtime::pcdConversionEnabled() ? "true" : "false") << ','
            << "\"octomap_file\":true,";
  emitConstraints(options);
  std::cout << "}}" << std::endl;
}

void emitSuccess(const octoplanner3d::runtime::PlanResult & result,
                 const std::optional<BenchmarkResult> & benchmark = {})
{
  std::cout.precision(std::numeric_limits<double>::max_digits10);
  std::cout << "{\"planner\":\"octoplanner3d\",\"protocol_version\":1,"
            << "\"ok\":" << (result.ok ? "true" : "false") << ",";
  if (!result.ok) {
    const std::string error = result.failure_reason.empty() ? "empty_path" : result.failure_reason;
    std::cout << "\"error\":\"" << error << "\",";
  }
  std::cout << "\"path\":[";
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    if (i > 0) {
      std::cout << ',';
    }
    std::cout << '[' << result.path[i].x << ',' << result.path[i].y << ',' << result.path[i].z << ']';
  }
  std::cout << "],\"reached_goal\":" << (result.reached_goal ? "true" : "false")
            << ",\"diagnostics\":{"
            << "\"source\":\"octoplanner3d_headless\","
            << "\"ros2_required\":false,"
            << "\"pcd_conversion\":"
            << (octoplanner3d::runtime::pcdConversionEnabled() ? "true" : "false") << ','
            << "\"octomap_file\":true,"
            << "\"path_points\":" << result.path.size() << ','
            << "\"used_surface_graph\":"
            << (result.used_surface_graph ? "true" : "false") << ','
            << "\"used_rust_graph\":"
            << (result.used_rust_graph ? "true" : "false") << ','
            << "\"rust_fallback\":"
            << (result.rust_fallback ? "true" : "false") << ','
            << "\"rust_failure_kind\":\"" << result.rust_failure_kind << "\","
            << "\"rust_failure\":\"" << result.rust_failure << "\","
            << "\"rust_query_failures\":" << result.rust_query_failures << ','
            << "\"rust_geometry_rejections\":" << result.rust_geometry_rejections << ','
            << "\"rust_other_failures\":" << result.rust_other_failures << ','
            << "\"goal_error_m\":" << result.goal_error_m << ','
            << "\"goal_xy_error_m\":" << result.goal_xy_error_m << ','
            << "\"goal_z_error_m\":" << result.goal_z_error_m << ','
            << "\"elapsed_ms\":" << result.elapsed_ms << ',';
  emitConstraints(result.options);
  std::cout << '}';
  if (benchmark) {
    auto sorted = benchmark->query_ms;
    std::sort(sorted.begin(), sorted.end());
    const auto middle = sorted.size() / 2;
    const double median = sorted.size() % 2 == 0
      ? (sorted[middle - 1] + sorted[middle]) / 2 : sorted[middle];
    const auto p95 = static_cast<std::size_t>(std::ceil(sorted.size() * .95)) - 1;
    std::cout << ",\"benchmark\":{\"samples\":" << sorted.size()
              << ",\"cold_prepare_ms\":" << benchmark->cold_prepare_ms
              << ",\"warmup_ms\":" << benchmark->warmup_ms
              << ",\"warm_query_median_ms\":" << median
              << ",\"warm_query_p95_ms\":" << sorted[p95]
              << ",\"cache_restore_ms\":" << benchmark->cache_restore_ms
              << ",\"restored_query_ms\":" << benchmark->restored_query_ms
              << ",\"prepare_count\":" << benchmark->prepare_count
              << ",\"cache_load_count\":" << benchmark->cache_load_count
              << ",\"map_load_count\":" << benchmark->map_load_count
              << ",\"restored_prepare_count\":" << benchmark->restored_prepare_count
              << ",\"restored_cache_load_count\":" << benchmark->restored_cache_load_count
              << ",\"same_result\":" << (benchmark->same_result ? "true" : "false")
              << ",\"query_ms\":[";
    for (std::size_t i = 0; i < benchmark->query_ms.size(); ++i) {
      if (i > 0) std::cout << ',';
      std::cout << benchmark->query_ms[i];
    }
    std::cout << "]}";
  }
  std::cout << '}' << std::endl;
}

bool sameResult(const octoplanner3d::runtime::PlanResult & a,
                const octoplanner3d::runtime::PlanResult & b)
{
  if (a.ok != b.ok || a.reached_goal != b.reached_goal ||
      a.failure_reason != b.failure_reason || a.path.size() != b.path.size() ||
      a.used_rust_graph != b.used_rust_graph || a.rust_fallback != b.rust_fallback) {
    return false;
  }
  for (std::size_t i = 0; i < a.path.size(); ++i) {
    if (a.path[i].x != b.path[i].x || a.path[i].y != b.path[i].y ||
        a.path[i].z != b.path[i].z) return false;
  }
  return true;
}

octoplanner3d::runtime::PlanResult runBenchmark(
    const HeadlessRequest & request, BenchmarkResult & benchmark)
{
  using Clock = std::chrono::steady_clock;
  const auto & options = *request.benchmark;
  const std::filesystem::path cache(options.cache_directory);
  if (std::filesystem::exists(cache) && !std::filesystem::is_empty(cache)) {
    throw std::runtime_error("benchmark cache_directory must be empty for a cold preparation");
  }
  const lingtu::nav::plan::MapIdentity identity{request.map_path, 1, "map"};
  octoplanner3d::runtime::PlannerSession session(cache);
  auto started = Clock::now();
  session.prepare(request.map_path, identity, request.plan.options);
  benchmark.cold_prepare_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  started = Clock::now();
  auto result = session.run(request.map_path, identity, request.plan);
  benchmark.warmup_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  benchmark.query_ms.reserve(options.samples);
  for (int i = 0; i < options.samples; ++i) {
    started = Clock::now();
    const auto next = session.run(request.map_path, identity, request.plan);
    benchmark.query_ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - started).count());
    benchmark.same_result = sameResult(result, next) && benchmark.same_result;
  }
  benchmark.prepare_count = session.prepareCount();
  benchmark.cache_load_count = session.cacheLoadCount();
  benchmark.map_load_count = session.mapLoadCount();
  octoplanner3d::runtime::PlannerSession restored(cache);
  started = Clock::now();
  restored.prepare(request.map_path, identity, request.plan.options);
  benchmark.cache_restore_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  started = Clock::now();
  const auto restored_result = restored.run(request.map_path, identity, request.plan);
  benchmark.restored_query_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  benchmark.same_result = sameResult(result, restored_result) && benchmark.same_result;
  benchmark.restored_prepare_count = restored.prepareCount();
  benchmark.restored_cache_load_count = restored.cacheLoadCount();
  return result;
}

}  // namespace

int main()
{
  try {
    const HeadlessRequest request = parseRequest(readStdin());
    octoplanner3d::runtime::PlanResult result;
    std::optional<BenchmarkResult> benchmark;
    {
      RedirectStdoutToStderr redirect_logs;
      if (request.benchmark) {
        benchmark.emplace();
        result = runBenchmark(request, *benchmark);
      } else {
        result = octoplanner3d::runtime::runPlan(request.map_path, request.plan);
      }
    }

    emitSuccess(result, benchmark);
    return result.ok ? 0 : 2;
  } catch (const std::exception & exc) {
    emitError(exc.what());
    return 1;
  }
}
