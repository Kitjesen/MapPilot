// Offline saved-map component experiment. No sensor, DDS, driver or physics loop.
#include "app/config/config.hpp"
#include "collision/body.hpp"
#include "collision_bitmap.hpp"
#include "lingtu/maps/octomap/io.hpp"
#include "navigation/executor.hpp"
#include "planning/global/octoplanner/octoplanner3d_core.hpp"
#include "planning/local/scan/backend.hpp"
#include "planning/local/scan/grid.hpp"
#include "trajectory/spline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;
using nav_kernel::Vec3;
namespace fs = std::filesystem;

double milliseconds(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

std::string quoted(const std::string& text) {
  std::string result = "\"";
  for (char value : text) {
    if (value == '"' || value == '\\') result += '\\';
    if (value == '\n') result += "\\n";
    else if (value == '\r') result += "\\r";
    else result += value;
  }
  return result + '"';
}

std::ofstream output(const fs::path& path) {
  std::ofstream stream(path);
  if (!stream) throw std::runtime_error("Cannot write " + path.string());
  stream << std::setprecision(17);
  return stream;
}

void point(std::ostream& out, const Vec3& p) {
  out << '[' << p.x << ',' << p.y << ',' << p.z << ']';
}

void routeCsv(const fs::path& path, const std::vector<Vec3>& route,
              const std::vector<lingtu::nav::navigation::PoseRoutePoint>* poses = nullptr) {
  auto stream = output(path);
  stream << "index,x_m,y_m,z_m" << (poses ? ",yaw_rad\n" : "\n");
  for (std::size_t index = 0; index < route.size(); ++index) {
    const auto& p = route[index];
    stream << index << ',' << p.x << ',' << p.y << ',' << p.z;
    if (poses) stream << ',' << (*poses)[index].yaw;
    stream << '\n';
  }
}

// This fixture samples every SCAN-resolution cell, not sparse leaf centres.
// OcTree::search resolves coarse leaves over their entire represented cube.
// Occupied, observed-free and unknown cells remain separate evidence classes.
class StaticCollision {
 public:
  StaticCollision(const octomap::OcTree& tree, const std::vector<Vec3>& route,
                  const lingtu::nav::endpoint::CliConfig& config,
                  const nav_kernel::Pose& initial_pose,
                  std::shared_ptr<const lingtu::nav::collision::Body> body)
      : resolution_(config.scan_planner.voxelResolution),
        source_resolution_(tree.getResolution()), start_pose_(initial_pose), body_(std::move(body)) {
    Vec3 low = route.front(), high = low;
    const auto body_bounds = lingtu::nav::collision::bounds(*body_, {{},0.0});
    const double body_radius = std::hypot(std::abs(body_bounds.center.x)+body_bounds.half_size.x,
                                        std::abs(body_bounds.center.y)+body_bounds.half_size.y);
    const double margin = config.scan_planner.planningHorizon + body_radius + 1.0;
    for (const auto& p : route) {
      low.x = std::min(low.x, p.x); low.y = std::min(low.y, p.y); low.z = std::min(low.z, p.z);
      high.x = std::max(high.x, p.x); high.y = std::max(high.y, p.y); high.z = std::max(high.z, p.z);
    }
    minimum_ = {std::floor((low.x-margin)/resolution_)*resolution_,
                std::floor((low.y-margin)/resolution_)*resolution_,
                std::floor((low.z-2.0)/resolution_)*resolution_};
    maximum_ = {std::ceil((high.x+margin)/resolution_)*resolution_,
                std::ceil((high.y+margin)/resolution_)*resolution_,
                std::ceil((high.z+2.0)/resolution_)*resolution_};
    occupied_ = std::make_unique<lingtu::nav::tests::CollisionBitmap>(minimum_, maximum_, resolution_);
    free_ = std::make_unique<lingtu::nav::tests::CollisionBitmap>(minimum_, maximum_, resolution_);
    const auto size = occupied_->view();
    for (int z = 0; z < size.sizeZ; ++z) {
      for (int y = 0; y < size.sizeY; ++y) {
        for (int x = 0; x < size.sizeX; ++x) {
          Vec3 center{minimum_.x + (x+.5)*resolution_,
                      minimum_.y + (y+.5)*resolution_,
                      minimum_.z + (z+.5)*resolution_};
          const auto* node = tree.search(center.x, center.y, center.z);
          if (node == nullptr) { ++unknown_count_; continue; }
          if (tree.isNodeOccupied(node)) {
            occupied_->occupy(center);
            ++occupied_count_;
          } else {
            free_->occupy(center);
            ++free_count_;
          }
        }
      }
    }
    const auto occupied = occupied_->view();
    occupied_storage_ = std::make_shared<const std::vector<std::uint8_t>>(
        occupied.inflatedBits, occupied.inflatedBits + occupied.inflatedBytes);
    const auto known_free = free_->view();
    free_storage_ = std::make_shared<const std::vector<std::uint8_t>>(
        known_free.inflatedBits, known_free.inflatedBits + known_free.inflatedBytes);
  }

  nav_kernel::LocalCollisionMapView view(double now) const {
    auto result = occupied_->view(now, 1);
    result.measuredOccupiedStorage = occupied_storage_;
    result.knownFreeStorage = free_storage_;
    result.live = false;
    return result;
  }

  void write(const fs::path& path, const nav_kernel::LocalPlannerParams& params) const {
    auto out = output(path);
    out << "{\"source\":\"same_saved_octomap\",\"mapd_live\":false,"
        << "\"source_octomap_resolution_m\":" << source_resolution_
        << ",\"fixture_resolution_m\":" << resolution_ << ",\"aabb_min\":";
    point(out, minimum_); out << ",\"aabb_max\":"; point(out, maximum_);
    out << ",\"occupied_cells\":" << occupied_count_
        << ",\"observed_free_cells\":" << free_count_
        << ",\"unknown_cells\":" << unknown_count_;
    nav_kernel::LocalPlanRequest diagnostic;
    diagnostic.body = body_;
    diagnostic.robot.pose = start_pose_;
    diagnostic.environment.collision = view(1);
    const nav_kernel::local::scan::Grid grid(params,diagnostic);
    out << ",\"start_body_collision_state\":"
        << grid.inflatedOccupancy(start_pose_.position,start_pose_.yaw)
        << ",\"start_position\":";
    point(out,start_pose_.position);
    out << ",\"start_yaw_rad\":" << start_pose_.yaw
        << ",\"coarse_leaf_volume_preserved\":true,\"unknown_marked_free\":false,"
        << "\"construction\":\"Exhaustive SCAN-resolution lookup into the unchanged OctoMap. Single OBB queries raw measured occupied and known-free; no cylinder preinflation and no floor removal.\","
        << "\"limitation\":\"Static saved-map fixture, not live mapd. Unknown or outside body volume is rejected. Sensor/ray transport and driver are absent.\"}\n";
  }

 private:
  double resolution_, source_resolution_;
  Vec3 minimum_{}, maximum_{};
  nav_kernel::Pose start_pose_{};
  std::shared_ptr<const lingtu::nav::collision::Body> body_;
  std::uint64_t occupied_count_{0}, free_count_{0}, unknown_count_{0};
  std::unique_ptr<lingtu::nav::tests::CollisionBitmap> occupied_, free_;
  std::shared_ptr<const std::vector<std::uint8_t>> occupied_storage_, free_storage_;
};

void saveSpline(const fs::path& folder, const nav_kernel::LocalPlan& plan) {
  const auto* spline = std::get_if<nav_kernel::SplineTarget>(&plan.target());
  if (!spline) return;
  auto out = output(folder / "first-spline.json");
  out << "{\"order\":" << spline->order << ",\"trajectory_id\":" << spline->trajectoryId
      << ",\"start_time_s\":" << spline->startTimeS
      << ",\"max_linear_speed_mps\":" << spline->maxLinearSpeedMps << ",\"controls\":[";
  for (std::size_t i = 0; i < spline->controls.size(); ++i) {
    if (i) out << ',';
    point(out, spline->controls[i]);
  }
  out << "],\"knots\":[";
  for (std::size_t i = 0; i < spline->knots.size(); ++i) {
    if (i) out << ',';
    out << spline->knots[i];
  }
  out << "]}\n";
  nav_kernel::SplineView view(*spline);
  if (!view.valid()) throw std::runtime_error("Published spline is invalid");
  auto samples = output(folder / "first-spline-samples.csv");
  samples << "time_from_start_s,x_m,y_m,z_m,vx_world_mps,vy_world_mps,vz_world_mps\n";
  for (double time = 0; time <= view.duration()+1e-9; time += .02) {
    const auto p = view.position(time), v = view.velocity(time);
    samples << time << ',' << p.x << ',' << p.y << ',' << p.z << ','
            << v.x << ',' << v.y << ',' << v.z << '\n';
  }
  routeCsv(folder / "first-preview.csv", plan.previewPath());
}

int run(int argc, char** argv) {
  if (argc != 10) throw std::runtime_error(
      "Usage: benchmark_saved_route map.ot sx sy sz gx gy gz out_dir feedback_seconds; native nav configuration is read from the inherited RunPlan environment");
  const fs::path map_path = argv[1], folder = argv[8];
  const double feedback_seconds = std::stod(argv[9]);
  if (!std::isfinite(feedback_seconds) || feedback_seconds < 0)
    throw std::runtime_error("feedback_seconds must be finite and nonnegative");
  fs::create_directories(folder);
  char name[] = "navd";
  char* config_argv[] = {name};
  const auto config = lingtu::nav::endpoint::parseArgs(1, config_argv);
  const auto params = lingtu::nav::endpoint::buildLocalPlannerParams(config);
  const auto executor_config = lingtu::nav::endpoint::buildExecutorConfig(config);
  const auto body_geometry = executor_config.body;
  if (params.backend != nav_kernel::LocalPlannerBackend::Scan)
    throw std::runtime_error("RunPlan must explicitly select the scan backend");
  if (!body_geometry || body_geometry->parts.size() != 1U ||
      body_geometry->parts.front().shape != lingtu::nav::collision::Shape::Box)
    throw std::runtime_error("RunPlan must provide the single navigation OBB");
  const auto& box = body_geometry->parts.front();
  const auto safety_config = lingtu::nav::endpoint::commandSafetyConfig(config);
  const double final_reaction_s = std::max(safety_config.cmd_max_age_s, 1.0/config.tick_hz);
  const double final_linear_deceleration = std::min(
      config.velocity_smoother.x.deceleration, config.velocity_smoother.y.deceleration);
  const double final_yaw_deceleration = config.velocity_smoother.yaw.deceleration;
  auto configuration = output(folder / "resolved-config.json");
  configuration << "{\"source\":\"native parseArgs(1) of inherited RunPlan environment\","
      << "\"robot_radius_m\":" << config.octoplanner_options.robot_radius
      << ",\"body_clearance_below_m\":" << config.octoplanner_options.body_clearance_below_m
      << ",\"body_clearance_above_m\":" << config.octoplanner_options.body_clearance_above_m
      << ",\"support_height_m\":" << config.octoplanner_options.support_height_m
      << ",\"collision_radius_m\":" << config.collision_cylinder_radius_m
      << ",\"collision_offset_m\":" << config.collision_cylinder_offset_m
      << ",\"collision_below_m\":" << config.collision_clearance_below_m
      << ",\"collision_above_m\":" << config.collision_clearance_above_m
      << ",\"scan_resolution_m\":" << params.scan.voxelResolution
      << ",\"speed_limit_mps\":" << executor_config.max_speed
      << ",\"collision_shape\":\"single_obb\",\"body_parts\":" << body_geometry->parts.size()
      << ",\"body_margin_m\":" << body_geometry->margin_m << ",\"body_center_m\":";
  point(configuration,{box.center.x,box.center.y,box.center.z});
  configuration << ",\"body_half_size_m\":";
  point(configuration,{box.half_size.x,box.half_size.y,box.half_size.z});
  configuration << ",\"final_reaction_s\":" << final_reaction_s
      << ",\"final_linear_deceleration_mps2\":" << final_linear_deceleration
      << ",\"final_yaw_deceleration_radps2\":" << final_yaw_deceleration << "}\n";
  configuration.close();

  auto summary = output(folder / "summary.json");
  summary << "{\"qualification\":\"ideal_planar_component_feedback\","
      << "\"physical_simulation\":false,\"sensor_feedback\":false,\"z_evolved\":false,"
      << "\"final_geometry_braking_included\":true,\"final_safety_and_driver_included\":false,"
      << "\"dds_lifecycle_included\":false,\"collision_shape\":\"single_obb\",\"map\":" << quoted(map_path.string());
  lingtu::nav::plan::GlobalPlanRequest request;
  request.start = {std::stod(argv[2]),std::stod(argv[3]),std::stod(argv[4])};
  request.start.yaw = 0.0;
  request.goal = {std::stod(argv[5]),std::stod(argv[6]),std::stod(argv[7])};
  request.options = config.octoplanner_options;
  request.map_identity = {"offline-saved-route", 1, "map"};
  octoplanner3d::runtime::PlannerSession session;
  auto started = Clock::now();
  const auto cold = session.run(map_path, request.map_identity, request);
  const double cold_request_ms = milliseconds(started);
  started = Clock::now();
  const auto result = session.run(map_path, request.map_identity, request);
  const double cached_request_ms = milliseconds(started);
  summary << ",\"cold_request_ms\":" << cold_request_ms
      << ",\"cached_request_ms\":" << cached_request_ms
      << ",\"map_load_count\":" << session.mapLoadCount()
      << ",\"prepare_count\":" << session.prepareCount()
      << ",\"global_ok\":" << (result.ok ? "true" : "false")
      << ",\"global_reason\":" << quoted(result.failure_reason)
      << ",\"global_cold_ok\":" << (cold.ok ? "true" : "false")
      << ",\"global_point_count\":" << result.path.size()
      << ",\"global_goal_error_m\":" << result.goal_error_m;
  // Activation preparation is charged separately from command response. The
  // three goals share one session, as navd does, and alternate out/back/out.
  octoplanner3d::runtime::PlannerSession activated_session;
  started = Clock::now();
  activated_session.prepare(map_path, request.map_identity, request.options);
  summary << ",\"activation_prepare_ms\":" << milliseconds(started)
      << ",\"sequential_planning_only\":true,\"sequential_goals\":[";
  auto sequential = request;
  for (int index = 0; index < 3; ++index) {
    if (index != 0) summary << ',';
    const auto loads_before = activated_session.mapLoadCount();
    const auto prepares_before = activated_session.prepareCount();
    started = Clock::now();
    const auto planned = activated_session.run(map_path, request.map_identity, sequential);
    summary << "{\"ordinal\":" << index + 1
        << ",\"request_ms\":" << milliseconds(started)
        << ",\"map_load_delta\":" << activated_session.mapLoadCount() - loads_before
        << ",\"prepare_delta\":" << activated_session.prepareCount() - prepares_before
        << ",\"ok\":" << (planned.ok ? "true" : "false")
        << ",\"reason\":" << quoted(planned.failure_reason)
        << ",\"point_count\":" << planned.path.size() << ",\"start\":";
    point(summary, {sequential.start.x, sequential.start.y, sequential.start.z});
    summary << ",\"goal\":";
    point(summary, {sequential.goal.x, sequential.goal.y, sequential.goal.z});
    summary << '}';
    std::swap(sequential.start, sequential.goal);
    // Planning-only return starts at the previous requested goal.
  }
  summary << "]";
  if (!result.ok || result.path.empty()) {
    summary << ",\"local_attempted\":false}\n";
    return 2;
  }
  std::vector<Vec3> route;
  for (const auto& p : result.path) route.push_back({p.x,p.y,p.z});
  summary << ",\"global_route_kind\":\"xyz\"";
  routeCsv(folder / "global-route.csv", route);
  // Feedback starts at the requested pose; snapping never relocates the body.
  nav_kernel::Pose body{{request.start.x,request.start.y,request.start.z},*request.start.yaw};
  summary << ",\"feedback_initial_position\":";
  point(summary,body.position);
  summary << ",\"feedback_initialized_at_request_start\":true"
      << ",\"route_start_offset_m\":" << std::sqrt(
          std::pow(route.front().x-body.position.x,2)+
          std::pow(route.front().y-body.position.y,2)+
          std::pow(route.front().z-body.position.z,2));
  started = Clock::now();
  const auto tree = lingtu::maps::LoadOctomapTree(map_path);
  const double fixture_load_ms = milliseconds(started);
  if (!tree) throw std::runtime_error("Cannot load static collision source");
  started = Clock::now();
  StaticCollision collision(*tree, route, config, body, body_geometry);
  const double fixture_ms = milliseconds(started);
  collision.write(folder / "collision-fixture.json", params);
  summary << ",\"fixture_map_load_ms\":" << fixture_load_ms
      << ",\"fixture_build_ms\":" << fixture_ms << ",\"local_attempted\":true";
  nav_kernel::local::scan::Backend backend(params);
  nav_kernel::LocalPlanRequest local;
  local.body = body_geometry;
  local.bodyHeading = body.yaw;
  local.robot.pose = body;
  local.objective = nav_kernel::RouteTarget{{route.data(),static_cast<int>(route.size()),1}};
  local.reference = {route.data(),static_cast<int>(route.size()),1};
  local.identity = {1,1,0};
  local.clock.mode = nav_kernel::PlanClockMode::External;
  local.maxLinearSpeedMps = executor_config.max_speed;
  auto ticks = output(folder / "local-planning.csv");
  ticks << "tick,time_s,tick_ms,search_ms,spline_ms,status,search_reason,attempt_stage,attempt_reason\n";
  nav_kernel::LocalPlan first;
  started = Clock::now();
  for (int tick = 0; tick < 500; ++tick) {
    local.clock.timestampS = 100 + tick*.01;
    local.environment.collision = collision.view(local.clock.timestampS);
    const auto tick_started = Clock::now();
    first = backend.tick(local);
    const double tick_ms = milliseconds(tick_started);
    const auto debug = backend.debugSnapshot();
    ticks << tick << ',' << local.clock.timestampS << ',' << tick_ms << ','
          << debug.searchTimeMs << ',' << debug.splineTimeMs << ','
          << nav_kernel::localPlanStatusName(first.status()) << ',' << debug.searchReason
          << ',' << debug.scanAttempt.stage << ',' << debug.scanAttempt.reason << '\n';
    if (first.ready()) break;
  }
  summary << ",\"first_local_plan_ms\":" << milliseconds(started)
      << ",\"local_status\":" << quoted(nav_kernel::localPlanStatusName(first.status()))
      << ",\"local_search_reason\":" << quoted(backend.debugSnapshot().searchReason)
      << ",\"local_attempt_stage\":" << quoted(backend.debugSnapshot().scanAttempt.stage)
      << ",\"local_attempt_reason\":" << quoted(backend.debugSnapshot().scanAttempt.reason)
      << ",\"local_ready\":" << (first.ready() ? "true" : "false");
  if (first.ready()) {
    saveSpline(folder, first);
    nav_kernel::Follower follower;
    nav_kernel::FollowerState state;
    state.vehicleRelative = body.position;
    state.vehicleYawRelative = body.yaw;
    state.currentTime = local.clock.timestampS;
    state.params = executor_config.follower;
    started = Clock::now();
    const auto follow = follower.follow(first, state);
    summary << ",\"first_follower_ms\":" << milliseconds(started)
        << ",\"first_command\": [" << follow.cmd.vx << ',' << follow.cmd.vy << ',' << follow.cmd.wz << ']';
  }
  bool reached = false;
  bool cancel_checked = false, cancel_zero = false;
  nav_kernel::Twist cancel_command{};
  std::string cancel_reason;
  int collision_state_count = 0, invalid_collision_query_count = 0;
  int final_blocked_ticks = 0, final_limited_ticks = 0;
  int raw_motion_ticks = 0, applied_motion_ticks = 0;
  double first_collision_s = -1.0;
  double traveled = 0;
  std::string reason = "feedback_not_requested";
  if (feedback_seconds > 0) {
    nav_kernel::LocalPlanner planner(params);
    if (!planner.configure("")) throw std::runtime_error("Cannot configure SCAN executor");
    lingtu::nav::navigation::Executor executor(executor_config, std::move(planner));
    lingtu::nav::navigation::Route active;
    active.points = route;
    active.body = body_geometry;
    active.finalYaw = request.goal.yaw;
    if (request.options.ground_support) {
      active.supportHeightM = request.options.support_height_m;
      active.supportHeightToleranceM = request.options.support_height_tolerance_m;
    }
    executor.setRoute(std::move(active));
    nav_kernel::Twist measured{};
    auto feedback = output(folder / "feedback.csv");
    feedback << "tick,time_s,x_m,y_m,z_m,yaw_rad,vx_mps,vy_mps,wz_rps,tick_ms,trajectory_id,tracking_active,position_error_m,goal_reached,reason,collision_state,raw_vx_mps,raw_vy_mps,raw_wz_rps,final_braking_scale,final_geometry_reason,executor_ms,final_geometry_ms,pose_stage_index,pose_stage_count\n";
    auto paths = output(folder / "feedback-previews.csv");
    paths << "tick,trajectory_id,index,x_m,y_m,z_m\n";
    const double dt = .01;
    const auto wall_start = Clock::now();
    lingtu::nav::navigation::ExecutionInput last_input;
    double last_time = 200.0;
    for (int tick = 0; tick < static_cast<int>(feedback_seconds/dt); ++tick) {
      const double now = 200 + tick*dt;
      // Re-observe the unchanged synthetic world at 5 Hz. Content generation
      // remains fixed; observation sequence and timestamps advance together.
      const auto observation_sequence = static_cast<std::uint64_t>(1 + tick/20);
      const double observation_time = 200 + (tick/20)*.2;
      lingtu::nav::navigation::ExecutionInput input;
      input.mapBody = input.odomBody = body;
      input.timestampS = now;
      input.observation.frame_epoch = 1;
      input.observation.cloud_generation = observation_sequence;
      input.observation.odom_stamp_s = now;
      input.observation.cloud_stamp_s = observation_time;
      input.observation.clock_mode = nav_kernel::PlanClockMode::External;
      input.observation.body_velocity_valid = true;
      input.observation.body_linear_velocity = {measured.vx,measured.vy,0};
      input.observation.body_yaw_rate = measured.wz;
      input.observation.collision = collision.view(observation_time);
      input.observation.collision.observationSequence = observation_sequence;
      last_input = input;
      last_time = now;
      nav_kernel::LocalPlanRequest diagnostic;
      diagnostic.body = body_geometry;
      diagnostic.robot.pose = body;
      diagnostic.clock.timestampS = now;
      diagnostic.environment.collision = input.observation.collision;
      const nav_kernel::local::scan::Grid diagnostic_grid(params, diagnostic);
      const int collision_state = diagnostic_grid.inflatedOccupancy(body.position, body.yaw);
      if (collision_state == 1) {
        ++collision_state_count;
        if (first_collision_s < 0) first_collision_s = now;
      } else if (collision_state < 0) {
        ++invalid_collision_query_count;
      }
      const auto tick_start = Clock::now();
      const auto out = executor.tick(input);
      const double executor_ms = milliseconds(tick_start);
      const auto final_start = Clock::now();
      const auto raw_command = out.cmd_vel;
      const bool measured_blocked = diagnostic_grid.brakingOccupancy(
          body, measured, final_reaction_s, final_linear_deceleration,
          final_yaw_deceleration) != 0;
      const double final_scale = measured_blocked ? 0.0 : diagnostic_grid.brakingScale(
          body, raw_command, final_reaction_s, final_linear_deceleration,
          final_yaw_deceleration);
      auto applied_command = raw_command;
      std::string final_reason = "accepted";
      if (final_scale <= 0.0) {
        applied_command = {};
        final_reason = "scan_actual_motion_blocked";
        ++final_blocked_ticks;
      } else if (final_scale < 1.0) {
        applied_command.vx *= final_scale;
        applied_command.vy *= final_scale;
        applied_command.wz *= final_scale;
        final_reason = "scan_actual_motion_limited";
        ++final_limited_ticks;
      }
      const double final_geometry_ms = milliseconds(final_start);
      if (raw_command.vx != 0 || raw_command.vy != 0 || raw_command.wz != 0) ++raw_motion_ticks;
      if (applied_command.vx != 0 || applied_command.vy != 0 || applied_command.wz != 0)
        ++applied_motion_ticks;
      const double tick_ms = milliseconds(tick_start);
      feedback << tick << ',' << now << ',' << body.position.x << ',' << body.position.y << ','
          << body.position.z << ',' << body.yaw << ',' << applied_command.vx << ','
          << applied_command.vy << ',' << applied_command.wz << ',' << tick_ms << ','
          << out.tracking.trajectoryId << ',' << out.tracking.active << ','
          << out.tracking.positionErrorM << ',' << out.goal_reached << ',' << out.reason
          << ',' << collision_state << ',' << raw_command.vx << ',' << raw_command.vy << ','
          << raw_command.wz << ',' << final_scale << ',' << final_reason << ','
          << executor_ms << ',' << final_geometry_ms << ',' << out.pose_stage_index << ','
          << out.pose_stage_count << '\n';
      for (std::size_t index=0; index<out.local_path_map.size(); ++index) {
        const auto& p = out.local_path_map[index];
        paths << tick << ',' << out.tracking.trajectoryId << ',' << index << ','
              << p.x << ',' << p.y << ',' << p.z << '\n';
      }
      reason = final_scale < 1.0 ? final_reason : out.reason;
      measured = applied_command;
      const double c = std::cos(body.yaw), s = std::sin(body.yaw);
      body.position.x += (c*measured.vx-s*measured.vy)*dt;
      body.position.y += (s*measured.vx+c*measured.vy)*dt;
      body.yaw = nav_kernel::normalizeAngle(body.yaw+measured.wz*dt);
      traveled += std::hypot(measured.vx,measured.vy)*dt;
      if (out.goal_reached && collision_state == 0 && final_scale > 0.0) {
        reached = true;
        break;
      }
      std::this_thread::sleep_until(wall_start + std::chrono::microseconds((tick+1)*10000));
    }
    executor.clear();
    last_input.mapBody = last_input.odomBody = body;
    last_input.timestampS = last_time + dt;
    last_input.observation.odom_stamp_s = last_input.timestampS;
    const auto cancelled = executor.tick(last_input);
    cancel_checked = true;
    cancel_command = cancelled.cmd_vel;
    cancel_reason = cancelled.reason;
    cancel_zero = cancel_command.vx == 0 && cancel_command.vy == 0 && cancel_command.wz == 0;
  }
  summary << ",\"feedback_seconds_requested\":" << feedback_seconds
      << ",\"feedback_goal_reached\":" << (reached ? "true" : "false")
      << ",\"feedback_final_reason\":" << quoted(reason)
      << ",\"collision_state_count\":" << collision_state_count
      << ",\"invalid_collision_query_count\":" << invalid_collision_query_count
      << ",\"final_blocked_ticks\":" << final_blocked_ticks
      << ",\"final_limited_ticks\":" << final_limited_ticks
      << ",\"raw_motion_ticks\":" << raw_motion_ticks
      << ",\"applied_motion_ticks\":" << applied_motion_ticks
      << ",\"first_collision_time_s\":";
  if (first_collision_s < 0) summary << "null";
  else summary << first_collision_s;
  summary << ",\"component_cancel_checked\":" << (cancel_checked ? "true" : "false")
      << ",\"component_cancel_zero\":" << (cancel_zero ? "true" : "false")
      << ",\"component_cancel_reason\":" << quoted(cancel_reason)
      << ",\"component_cancel_command\":[" << cancel_command.vx << ','
      << cancel_command.vy << ',' << cancel_command.wz << ']'
      << ",\"physical_stop_verified\":false"
      << ",\"feedback_planar_distance_m\":" << traveled << ",\"final_position\":";
  point(summary, body.position);
  summary << "}\n";
  return first.ready() ? 0 : 3;
}
}  // namespace

int main(int argc, char** argv) {
  try { return run(argc, argv); }
  catch (const std::exception& error) {
    std::cerr << "saved_route_component: " << error.what() << '\n';
    return 1;
  }
}
