#include "global_planner.h"

#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
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

using global_planner::OctoPlanner3D;
using global_planner::PlannerConfig;
using global_planner::PointPose;

struct Args
{
  std::string map_path;
  std::string path_path;
  bool self_test{false};
  PlannerConfig config;
  std::optional<PointPose> planning_anchor;

  Args()
  {
    config.robot_radius = 0.155;
    config.body_clearance_below_m = 0.10;
    config.body_clearance_above_m = 0.10;
    config.ground_support = true;
    config.direct_support_only = false;

    config.support_depth_cells = 2;
    config.support_height_m = 0.35;
    config.support_height_tolerance_m = 0.05;
    config.support_patch_radius_cells = 0;
    config.support_patch_min_samples = 0;
    config.lowest_traversable_only = false;
    config.max_step_height = 0.45;
    config.max_slope = 0.57735;
  }
};

struct Verification
{
  bool valid{false};
  bool route_valid{false};
  std::string route_reason{"not_checked"};
  std::string reason;
  std::size_t point_index{0};
  std::size_t segment_index{0};
  std::size_t checked_points{0};
  std::size_t checked_segments{0};
  double resolution{0.0};
  double planning_z_resolution{0.0};
  bool sweep_valid{false};
  bool motion_diagnostic_valid{false};
  struct SegmentFailure
  {
    std::size_t index;
    std::string reason;
    PointPose from;
    PointPose to;
    std::optional<PointPose> at;
    double horizontal_m;
    double rise_m;
  };
  std::vector<SegmentFailure> sweep_failures;
  std::vector<SegmentFailure> motion_failures;
};

class RedirectStdoutToStderr
{
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

double parseDouble(const std::string & value, const std::string & option)
{
  char * end = nullptr;
  const double result = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(result)) {
    throw std::runtime_error("invalid value for " + option + ": " + value);
  }
  return result;
}

int parseInt(const std::string & value, const std::string & option)
{
  char * end = nullptr;
  const long result = std::strtol(value.c_str(), &end, 10);
  if (end == value.c_str() || *end != '\0') {
    throw std::runtime_error("invalid value for " + option + ": " + value);
  }
  return static_cast<int>(result);
}

bool parseBool(const std::string & value, const std::string & option)
{
  if (value == "true" || value == "1") return true;
  if (value == "false" || value == "0") return false;
  throw std::runtime_error("invalid value for " + option + ": " + value);
}

Args parseArgs(int argc, char ** argv)
{
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) throw std::runtime_error("missing value for " + option);
      return argv[++i];
    };
    if (option == "--map") args.map_path = next();
    else if (option == "--path") args.path_path = next();
    else if (option == "--planning-anchor") {
      args.planning_anchor = PointPose{
        parseDouble(next(), option),
        parseDouble(next(), option),
        parseDouble(next(), option)};
    }
    else if (option == "--self-test") args.self_test = true;
    else if (option == "--robot-radius") args.config.robot_radius = parseDouble(next(), option);
    else if (option == "--body-clearance-below")
      args.config.body_clearance_below_m = parseDouble(next(), option);
    else if (option == "--body-clearance-above")
      args.config.body_clearance_above_m = parseDouble(next(), option);
    else if (option == "--direct-support-only")
      args.config.direct_support_only = parseBool(next(), option);
    else if (option == "--support-depth-cells")
      args.config.support_depth_cells = parseInt(next(), option);
    else if (option == "--search-resolution")
      args.config.search_resolution_m = parseDouble(next(), option);
    else if (option == "--surface-closing-radius")
      args.config.surface_closing_radius_m = parseDouble(next(), option);
    else if (option == "--support-height")
      args.config.support_height_m = parseDouble(next(), option);
    else if (option == "--support-height-tolerance")
      args.config.support_height_tolerance_m = parseDouble(next(), option);
    else if (option == "--support-patch-radius-cells")
      args.config.support_patch_radius_cells = parseInt(next(), option);
    else if (option == "--support-patch-min-samples")
      args.config.support_patch_min_samples = parseInt(next(), option);
    else if (option == "--lowest-traversable-only")
      args.config.lowest_traversable_only = parseBool(next(), option);
    else if (option == "--max-step-height")
      args.config.max_step_height = parseDouble(next(), option);
    else if (option == "--max-slope") args.config.max_slope = parseDouble(next(), option);
    else if (option == "--help" || option == "-h") {
      throw std::runtime_error(
        "usage: octoplanner3d_verify_path --map map.bt --path path.xyz "
        "[--planning-anchor x y z] "
        "[planner profile options]; path.xyz contains one 'x y z' point per line");
    } else {
      throw std::runtime_error("unknown argument: " + option);
    }
  }
  if (args.self_test) return args;
  if (args.map_path.empty()) throw std::runtime_error("missing --map");
  if (args.path_path.empty()) throw std::runtime_error("missing --path");
  return args;
}

std::shared_ptr<octomap::OcTree> loadTree(const std::string & path)
{
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(path));
  if (!raw) {
    auto tree = std::make_shared<octomap::OcTree>(0.2);
    if (tree->readBinary(path)) return tree;
    throw std::runtime_error("failed to read OctoMap: " + path);
  }
  auto * tree = dynamic_cast<octomap::OcTree *>(raw.release());
  if (tree == nullptr) throw std::runtime_error("OctoMap is not an OcTree: " + path);
  return std::shared_ptr<octomap::OcTree>(tree);
}

std::vector<PointPose> loadPath(const std::string & path)
{
  std::ifstream input(path);
  if (!input) throw std::runtime_error("failed to open path: " + path);
  std::vector<PointPose> points;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) line.resize(comment);
    std::istringstream row(line);
    row >> std::ws;
    if (row.eof()) continue;
    PointPose point;
    std::string extra;
    if (!(row >> point.x >> point.y >> point.z) || (row >> extra) ||
        !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      throw std::runtime_error("invalid path point at line " + std::to_string(line_number));
    }
    points.push_back(point);
  }
  if (points.size() < 2U) throw std::runtime_error("path must contain at least two points");
  return points;
}

std::string jsonEscape(const std::string & value)
{
  std::ostringstream out;
  for (const char ch : value) {
    if (ch == '\\' || ch == '"') out << '\\';
    if (ch == '\n') out << "\\n";
    else if (ch == '\r') out << "\\r";
    else if (ch == '\t') out << "\\t";
    else out << ch;
  }
  return out.str();
}

std::shared_ptr<octomap::OcTree> makeSelfTestMap(bool wall)
{
  constexpr double resolution = 0.05;
  auto tree = std::make_shared<octomap::OcTree>(resolution);
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      tree->updateNode(octomap::point3d(
        (x + 0.5) * resolution, (y + 0.5) * resolution, -0.325), true);
      for (int z = -6; z <= 6; ++z) {
        tree->updateNode(octomap::point3d(
          (x + 0.5) * resolution, (y + 0.5) * resolution,
          (z + 0.5) * resolution), false);
      }
    }
  }
  if (wall) {
    for (int y = -3; y <= 3; ++y) {
      for (int z = -2; z <= 4; ++z) {
        tree->updateNode(octomap::point3d(
          0.025, (y + 0.5) * resolution, (z + 0.5) * resolution), true);
      }
    }
  }
  tree->updateInnerOccupancy();
  return tree;
}

}  // namespace

namespace global_planner {

struct OctoPlannerGridQueryTest
{
  static const char * failureName(OctoPlanner3D::TraversabilityFailure failure)
  {
    using Failure = OctoPlanner3D::TraversabilityFailure;
    switch (failure) {
      case Failure::None: return "none";
      case Failure::OutsideBounds: return "outside_bounds";
      case Failure::UnknownSpace: return "unknown_space";
      case Failure::GroundSupport: return "ground_support";
      case Failure::ExternalPreblockedBelow: return "external_preblocked_below";
      case Failure::ExternalPreblockedBody: return "external_preblocked_body";
      case Failure::OccupiedBody: return "occupied_body";
    }
    return "unknown_failure";
  }

  static Verification verify(
    const std::shared_ptr<octomap::OcTree> & tree,
    const std::vector<PointPose> & path,
    const PlannerConfig & config,
    const PointPose * planning_anchor = nullptr)
  {
    Verification result;
    result.resolution = tree->getResolution();
    OctoPlanner3D planner;
    planner.setConfig(config);
    planner.setOctomap(tree);
    result.planning_z_resolution = planner.planningZResolution();

    // A planner may snap the first output waypoint away from the requested
    // start.  When the request pose is available, anchor the verification
    // lattice at that pose so it uses the same planning offset as planning.
    // Falling back to the first waypoint preserves the standalone path-file
    // contract for callers that have no request metadata.
    const auto & first = planning_anchor != nullptr ? *planning_anchor : path.front();
    planner.start_point_=first;
    planner.has_start_=true;
    planner.anchorPlanningLattice(first);

    if (planning_anchor && !planner.isStartConnectionAllowed(
            octomap::point3d(planning_anchor->x, planning_anchor->y, planning_anchor->z),
            octomap::point3d(path.front().x, path.front().y, path.front().z))) {
      result.reason = result.route_reason = "start_connection_failed";
      result.point_index = 0;
      return result;
    }

    std::vector<GridIndex> cells;
    const bool body_centre=config.support_height_m>0 || config.body_clearance_below_m>0 ||
        config.body_clearance_above_m>0;
    cells.reserve(path.size());
    for (std::size_t i = 0; i < path.size(); ++i) {
      const auto & point = path[i];
      const auto cell = planner.worldToPlanningGrid(point.x, point.y, point.z);
      OctoPlanner3D::TraversabilityFailure failure;
      const bool traversable=body_centre ?
        (failure=planner.queryWorld(octomap::point3d(point.x,point.y,point.z),config.robot_radius,
            config.ground_support&&i!=0,i!=0))==
            OctoPlanner3D::TraversabilityFailure::None :
        planner.isPlanningCellTraversableDetailed(
            cell, config.robot_radius, config.ground_support,
            config.direct_support_only,
            config.support_depth_cells, &failure);
      if (!traversable) {
        result.reason = std::string("point_") + failureName(failure);
        result.route_reason = result.reason;
        result.point_index = i;
        return result;
      }
      ++result.checked_points;
      cells.push_back(cell);
    }
    // Legacy paths retain the upstream adjacent-cell contract; metric paths may be sparse.
    for (std::size_t i = 0; i + 1U < cells.size(); ++i) {
      const auto & a = cells[i];
      const auto & b = cells[i + 1U];
      const int span = std::max({std::abs(a.x-b.x), std::abs(a.y-b.y), std::abs(a.z-b.z)});
      if (!body_centre && span != 1) {
        result.reason = "non_adjacent_route";
        result.route_reason = result.reason;
        result.segment_index = i;
        return result;
      }
    }
    result.route_valid = true;
    result.route_reason = "ok";

    for (std::size_t i = 0; i + 1U < path.size(); ++i) {
      // Metric routes retain their actual coordinates; legacy routes use the planning lattice.
      const auto from = body_centre ? octomap::point3d(path[i].x,path[i].y,path[i].z) :
          planner.planningPoint(cells[i]);
      const auto to = body_centre ? octomap::point3d(path[i+1].x,path[i+1].y,path[i+1].z) :
          planner.planningPoint(cells[i + 1U]);
      const double horizontal = std::hypot(to.x() - from.x(), to.y() - from.y());
      const double rise = std::abs(to.z() - from.z());
      OctoPlanner3D::TraversabilityFailure sweep_failure;
      octomap::point3d failed_point;
      if (!planner.sweptRouteFree(
            from,
            to, &sweep_failure, &failed_point,true,i==0)) {
        result.sweep_failures.push_back({i, failureName(sweep_failure),
          {from.x(), from.y(), from.z()}, {to.x(), to.y(), to.z()},
          PointPose{failed_point.x(), failed_point.y(), failed_point.z()}, horizontal, rise});
      }
      // Match the same metric step/slope gate used during body-mode search.
      // This remains a route check, not a SCAN or locomotion validation.
      if (!(body_centre || i==0 ? planner.metricConnectionWithinLimits(
              {from.x(),from.y(),from.z()},{to.x(),to.y(),to.z()}) :
              planner.isMotionAllowed(cells[i], cells[i + 1U]))) {
        const GridIndex step{cells[i+1U].x-cells[i].x,
          cells[i+1U].y-cells[i].y, cells[i+1U].z-cells[i].z};
        const double grid_rise = std::abs(step.z) * planner.planningZResolution();
        const double grid_horizontal = std::hypot(step.x, step.y) * result.resolution;
        const char * reason = "sweep";
        if (config.max_step_height > 0.0 && grid_rise > config.max_step_height)
          reason = "step_height_limit";
        else if (grid_rise > 0.0 && grid_horizontal <= 1e-9)
          reason = "vertical_transition";
        else if (config.max_slope > 0.0 && grid_rise > 0.0 &&
                 grid_rise / grid_horizontal > config.max_slope)
          reason = "slope_limit";
        result.motion_failures.push_back({i, reason,
          {from.x(), from.y(), from.z()}, {to.x(), to.y(), to.z()}, {},
          grid_horizontal, grid_rise});
      }
      ++result.checked_segments;
    }
    result.sweep_valid = result.sweep_failures.empty();
    result.motion_diagnostic_valid = result.motion_failures.empty();
    result.valid = result.sweep_valid && result.motion_diagnostic_valid;
    result.reason = !result.sweep_valid ? "segment_sweep_failed" :
      (!result.motion_diagnostic_valid ? "segment_motion_failed" : "ok");
    if (!result.sweep_valid) result.segment_index = result.sweep_failures.front().index;
    else if (!result.motion_diagnostic_valid) result.segment_index = result.motion_failures.front().index;
    return result;
  }
};

}  // namespace global_planner

namespace {

void writeSegmentFailures(const std::vector<Verification::SegmentFailure> & failures)
{
  const auto point = [](const PointPose & p) {
    std::cout << '[' << p.x << ',' << p.y << ',' << p.z << ']';
  };
  std::cout << '[';
  bool first = true;
  for (const auto & failure : failures) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"segment_index\":" << failure.index
      << ",\"reason\":\"" << jsonEscape(failure.reason) << "\",\"from\":";
    point(failure.from);
    std::cout << ",\"to\":";
    point(failure.to);
    std::cout << ",\"at\":";
    if (failure.at) point(*failure.at);
    else std::cout << "null";
    std::cout << ",\"horizontal_m\":" << failure.horizontal_m
      << ",\"rise_m\":" << failure.rise_m << ",\"slope\":";
    if (failure.horizontal_m > 1e-9) std::cout << failure.rise_m / failure.horizontal_m;
    else std::cout << "null";
    std::cout << '}';
  }
  std::cout << ']';
}

bool runSelfTest(const PlannerConfig & base_config)
{
  PlannerConfig config = base_config;
  config.robot_radius = 0.10;
  std::vector<PointPose> clear_path, blocked_path;
  for (int x = -8; x <= 7; ++x) {
    clear_path.push_back({(x+0.5)*0.05, 0.325, 0.025});
    blocked_path.push_back({(x+0.5)*0.05, 0.025, 0.025});
  }
  const auto clear = global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(false), clear_path, config);
  const auto blocked = global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(true), blocked_path, config);

  auto ramp = std::make_shared<octomap::OcTree>(0.05);
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      for (int z = x - 7; z <= x + 8; ++z) {
        ramp->updateNode(octomap::point3d(
          (x + 0.5) * 0.05, (y + 0.5) * 0.05, (z + 0.5) * 0.05),
          z == x - 7);
      }
    }
  }
  ramp->updateInnerOccupancy();
  config.max_slope = 0.57735;
  std::vector<PointPose> ramp_path;
  for (int x = -8; x <= 7; ++x)
    ramp_path.push_back({(x+0.5)*0.05, 0.025, (x+0.5)*0.05});
  const auto rising = global_planner::OctoPlannerGridQueryTest::verify(ramp, ramp_path, config);
  const auto shortcut = global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(false), {clear_path.front(), clear_path.back()}, config);

  // An early forbidden vertical transition must not hide a later collision.
  auto corner = makeSelfTestMap(false);
  corner->updateNode(octomap::point3d(-.175, -.025, .075), true);
  corner->updateInnerOccupancy();
  config.robot_radius = .01;
  config.body_clearance_below_m = config.body_clearance_above_m = .01;
  config.ground_support = false;
  const auto sparse_clear=global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(false),{clear_path.front(),clear_path.back()},config);
  const auto sparse_wall=global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(true),{blocked_path.front(),blocked_path.back()},config);
  const std::vector<PointPose> corner_path{
    {-.175, -.075, .025}, {-.175, -.075, .05}, {-.125, -.025, .05}};
  const auto hidden = global_planner::OctoPlannerGridQueryTest::verify(corner, corner_path, config);
  OctoPlanner3D planner;
  planner.setConfig(config);
  planner.setOctomap(corner);
  planner.makePlan(corner_path[1], corner_path[2]);
  std::vector<PointPose> detour;
  planner.getPlannerResults(detour);
  if (detour.size() < 2U) return false;
  const auto routed = global_planner::OctoPlannerGridQueryTest::verify(corner, detour, config);
  // A clear route is still unusable when its original start is blocked or
  // needs a forbidden vertical connection to its first returned waypoint.
  config.robot_radius = .10;
  config.body_clearance_below_m = config.body_clearance_above_m = .01;
  const PointPose blocked_anchor{.025, .025, .025};
  const auto bad_entry = global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(true), clear_path, config, &blocked_anchor);
  const PointPose high_anchor{clear_path.front().x, clear_path.front().y, .075};
  const auto vertical_entry = global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(false), clear_path, config, &high_anchor);
  const PointPose free_anchor{clear_path.front().x+.01, clear_path.front().y, .025};
  const auto free_entry = global_planner::OctoPlannerGridQueryTest::verify(
    makeSelfTestMap(false), clear_path, config, &free_anchor);
  return clear.valid && clear.route_valid && !blocked.route_valid &&
    rising.route_valid && !rising.valid && !rising.motion_diagnostic_valid &&
    sparse_clear.valid && sparse_wall.route_valid && !sparse_wall.sweep_valid &&
    hidden.route_valid && !hidden.sweep_valid && hidden.checked_segments == 2U &&
    hidden.sweep_failures.front().index == 1U &&
    hidden.sweep_failures.front().reason == "occupied_body" &&
    !hidden.motion_failures.empty() && hidden.motion_failures.front().index == 0U &&
    hidden.reason == "segment_sweep_failed" &&
    detour.size() > 2U && routed.route_valid && routed.sweep_valid &&
    shortcut.valid &&
    !bad_entry.valid && bad_entry.reason == "start_connection_failed" &&
    !vertical_entry.valid && vertical_entry.reason == "start_connection_failed" &&
    free_entry.valid;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Args args = parseArgs(argc, argv);
    Verification result;
    {
      RedirectStdoutToStderr redirect;
      if (args.self_test) {
        const bool passed = runSelfTest(args.config);
        result.valid = passed;
        result.route_valid = passed;
        result.route_reason = passed ? "self_test_passed" : "self_test_failed";
        result.reason = passed ? "self_test_passed" : "self_test_failed";
        result.sweep_valid = result.motion_diagnostic_valid = passed;
      } else {
        result = global_planner::OctoPlannerGridQueryTest::verify(
          loadTree(args.map_path), loadPath(args.path_path), args.config,
          args.planning_anchor ? &*args.planning_anchor : nullptr);
      }
    }
    std::cout
      << "{\"route_valid\":" << (result.route_valid ? "true" : "false")
      << ",\"route_reason\":\"" << jsonEscape(result.route_reason) << "\""
      << ",\"valid\":" << (result.valid ? "true" : "false")
      << ",\"reason\":\"" << jsonEscape(result.reason) << "\""
      << ",\"point_index\":" << result.point_index
      << ",\"segment_index\":" << result.segment_index
      << ",\"checked_points\":" << result.checked_points
      << ",\"checked_segments\":" << result.checked_segments
      << ",\"resolution\":" << result.resolution
      << ",\"planning_z_resolution\":" << result.planning_z_resolution
      << ",\"sweep_valid\":" << (result.sweep_valid ? "true" : "false")
      << ",\"motion_diagnostic_valid\":" << (result.motion_diagnostic_valid ? "true" : "false")
      << ",\"max_step_height_m\":" << args.config.max_step_height
      << ",\"max_slope\":" << args.config.max_slope
      << ",\"sweep_failures\":";
    writeSegmentFailures(result.sweep_failures);
    std::cout << ",\"motion_failures\":";
    writeSegmentFailures(result.motion_failures);
    std::cout << "}\n";
    return result.valid ? 0 : 2;
  } catch (const std::exception & error) {
    std::cout << "{\"valid\":false,\"error\":\""
              << jsonEscape(error.what()) << "\"}\n";
    return 1;
  }
}
