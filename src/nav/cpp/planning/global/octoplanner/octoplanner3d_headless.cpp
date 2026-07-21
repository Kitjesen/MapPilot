#include "octoplanner3d_core.hpp"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <iostream>
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

struct HeadlessRequest {
  std::string map_path;
  PlanRequest plan;
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
  options.require_ground_support = optionalBool(
    body,
    "require_ground_support",
    options.require_ground_support);
  options.strict_direct_ground_support = optionalBool(
    body,
    "strict_direct_ground_support",
    options.strict_direct_ground_support);
  options.ground_support_xy_radius_cells = optionalInt(
    body,
    "ground_support_xy_radius_cells",
    options.ground_support_xy_radius_cells);
  options.ground_support_depth_cells = optionalInt(
    body,
    "ground_support_depth_cells",
    options.ground_support_depth_cells);
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
  options.max_same_floor_z_excursion = optionalDouble(
    body,
    "max_same_floor_z_excursion",
    options.max_same_floor_z_excursion);
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
  applyOptionsFromJson(request.plan.options, json);
  if (request.map_path.empty()) {
    throw std::runtime_error("map_path must not be empty");
  }
  return request;
}

void emitConstraints(const PlannerOptions & options)
{
  std::cout << "\"constraints\":{"
            << "\"robot_radius\":" << options.robot_radius << ','
            << "\"body_clearance_below_m\":" << options.body_clearance_below_m << ','
            << "\"body_clearance_above_m\":" << options.body_clearance_above_m << ','
            << "\"max_iterations\":" << options.max_iterations << ','
            << "\"snap_search_radius_cells\":" << options.snap_search_radius_cells << ','
            << "\"require_ground_support\":"
            << (options.require_ground_support ? "true" : "false") << ','
            << "\"strict_direct_ground_support\":"
            << (options.strict_direct_ground_support ? "true" : "false") << ','
            << "\"ground_support_xy_radius_cells\":"
            << options.ground_support_xy_radius_cells << ','
            << "\"ground_support_depth_cells\":"
            << options.ground_support_depth_cells << ','
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
            << "\"max_same_floor_z_excursion\":"
            << options.max_same_floor_z_excursion << ','
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

void emitSuccess(const octoplanner3d::runtime::PlanResult & result)
{
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
            << "\"goal_error_m\":" << result.goal_error_m << ','
            << "\"goal_xy_error_m\":" << result.goal_xy_error_m << ','
            << "\"goal_z_error_m\":" << result.goal_z_error_m << ','
            << "\"elapsed_ms\":" << result.elapsed_ms << ',';
  emitConstraints(result.options);
  std::cout << "}}" << std::endl;
}

}  // namespace

int main()
{
  try {
    const HeadlessRequest request = parseRequest(readStdin());
    octoplanner3d::runtime::PlanResult result;
    {
      RedirectStdoutToStderr redirect_logs;
      result = octoplanner3d::runtime::runPlan(request.map_path, request.plan);
    }

    emitSuccess(result);
    return result.ok ? 0 : 2;
  } catch (const std::exception & exc) {
    emitError(exc.what());
    return 1;
  }
}
