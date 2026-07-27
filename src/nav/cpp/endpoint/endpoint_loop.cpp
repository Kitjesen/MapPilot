#include "endpoint_loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "endpoint_state.hpp"
#include "endpoint_time.hpp"
#include "input/nav_input_state_projector.hpp"
#include "inspection/inspection_command_coordinator.hpp"
#include "inspection/inspection_runtime_controller.hpp"
#include "lingtu_slam.h"
#include "motion/autonomy_tick_controller.hpp"
#include "motion/command_identity.hpp"
#include "motion/command_ingress_controller.hpp"
#include "motion/control_authority.hpp"
#include "motion/control_loop_runtime_guard.hpp"
#include "motion/motion_stop_coordinator.hpp"
#include "motion/operator_motion_authority.hpp"
#include "motion/operator_motion_output_evidence.hpp"
#include "motion/stop_confirmation.hpp"
#include "motion/teleop_admission_controller.hpp"
#include "motion/teleop_safety.hpp"
#include "motion/teleop_tick_controller.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav_dds_runtime.hpp"
#include "nav_endpoint_config.hpp"
#include "nav_endpoint_messages.hpp"
#include "nav_loop.hpp"
#include "plan/goal_plan_controller.hpp"
#include "plan/input_gate.hpp"
#include "plan/live_obstacle_layer.hpp"
#include "plan/planner_inputs.hpp"
#include "plan/rolling_segment_effect_coordinator.hpp"
#include "plan/rolling_segment_lifecycle.hpp"
#include "status/control_loop_health.hpp"
#include "status/inspection_status_file_writer.hpp"
#include "status/nav_status_endpoint_adapter.hpp"
#include "status/nav_status_publisher.hpp"
#include "status/navigation_state.hpp"

namespace {

using CommandKind = lingtu::message::NavigationCommandKind;
using GoalState = lingtu::message::NavigationGoalState;
using OperatorMotionAction = lingtu::message::OperatorMotionAction;

using lingtu::nav::endpoint::commandIngressRequestFromDds;
using lingtu::nav::endpoint::decodeGoal;
using lingtu::nav::endpoint::decodePath;
using lingtu::nav::endpoint::decodeTwist;
using lingtu::nav::endpoint::elapsedMs;
using lingtu::nav::endpoint::headerFrameId;
using lingtu::nav::endpoint::headerStampSeconds;
using lingtu::nav::endpoint::nowSeconds;
using lingtu::nav::endpoint::sourceStampError;
using lingtu::nav::endpoint::sourceStampPredates;
using lingtu::nav::endpoint::steadySeconds;
using lingtu::nav::endpoint::stringValue;
using lingtu::nav::endpoint::textData;

bool inspectionPostArrivalState(lingtu::nav::inspection::RunState state) noexcept {
  using RunState = lingtu::nav::inspection::RunState;
  return state == RunState::kSettling || state == RunState::kDwelling ||
         state == RunState::kActionPending;
}

bool localizationGateBlocked(const lingtu::nav::endpoint::InputGateState &state,
                             const lingtu::nav::endpoint::InputGateConfig &config) noexcept {
  if (!config.require_localization_health)
    return false;
  const double age_s = state.localization_health_age_s;
  return !std::isfinite(age_s) || age_s < -config.future_tolerance_s ||
         (config.localization_health_max_age_s > 0.0 &&
          age_s > config.localization_health_max_age_s) ||
         !state.localization_healthy || state.localization_state.empty() ||
         !lingtu::nav::endpoint::isHealthyLocalizationState(state.localization_state) ||
         lingtu::nav::endpoint::isCatastrophicLocalizationReason(state.localization_reason);
}

lingtu::nav::endpoint::RollingSegmentExecutionGrid
rollingSegmentGridFromDds(const lingtu::nav::endpoint::ExplorationExecutionGridView &grid) {
  lingtu::nav::endpoint::RollingSegmentExecutionGrid result;
  result.stamp_s = grid.stamp_s;
  result.frame_id = grid.frame_id;
  result.width = grid.width;
  result.height = grid.height;
  result.resolution = grid.resolution;
  result.origin_x = grid.origin_x;
  result.origin_y = grid.origin_y;
  result.origin_z = grid.origin_z;
  result.origin_qx = grid.origin_qx;
  result.origin_qy = grid.origin_qy;
  result.origin_qz = grid.origin_qz;
  result.origin_qw = grid.origin_qw;
  result.occupancy = grid.occupancy;
  result.terrain_cost = grid.terrain_cost;
  result.session_id = grid.session_id;
  result.reset_epoch = grid.reset_epoch;
  result.generation = grid.generation;
  result.live = grid.live;
  result.terrain_risk_stamp_s = grid.terrain_risk_stamp_s;
  result.terrain_risk_ready = grid.terrain_risk_ready;
  result.payload_complete = grid.payload_complete;
  return result;
}

lingtu::nav::endpoint::RollingSegmentCommand
rollingSegmentCommandFromDds(const lingtu::nav::endpoint::ExplorationSegmentRequestView &request) {
  lingtu::nav::endpoint::RollingSegmentCommand command;
  command.stamp_s = request.stamp_s;
  command.frame_id = request.frame_id;
  command.request_id = request.request_id;
  command.kind = request.kind;
  command.session_id = request.session_id;
  command.reset_epoch = request.reset_epoch;
  command.minimum_generation = request.minimum_generation;
  command.target = {
      request.target.x,  request.target.y,  request.target.z,  request.target.qx,
      request.target.qy, request.target.qz, request.target.qw,
  };
  command.reason = request.reason;
  return command;
}

lingtu_dds_PoseStamped inspectionGoalMessage(const lingtu::nav::inspection::Point &point,
                                             double stamp_s) {
  lingtu_dds_PoseStamped goal{};
  goal.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  goal.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<double>(goal.header.stamp.sec)) * 1e9);
  goal.header.frame_id = const_cast<char *>("map");
  goal.pose.position.x = point.x_m;
  goal.pose.position.y = point.y_m;
  goal.pose.position.z = point.z_m;
  const double half_yaw = point.has_yaw ? point.yaw_rad * 0.5 : 0.0;
  goal.pose.orientation.z = std::sin(half_yaw);
  goal.pose.orientation.w = std::cos(half_yaw);
  return goal;
}

}  // namespace

namespace lingtu::nav::endpoint {

int runEndpointLoop(EndpointLoopContext &ctx, const std::atomic_bool &running) {
  auto &cfg = ctx.cfg;
  auto &gate_cfg = ctx.gate_cfg;
  auto &obstacle_merge_config = ctx.obstacle_merge_config;
  auto &safety_config = ctx.safety_config;
  auto &operator_motion_interface_enabled = ctx.operator_motion_interface_enabled;
  auto &dds = ctx.dds;
  auto &state = ctx.state;
  auto &nav = ctx.nav;
  auto &live_obstacles = ctx.live_obstacles;
  auto &inspection_status_writer = ctx.inspection_status_writer;
  auto &input_projector = ctx.input_projector;
  auto &goal_plan = ctx.goal_plan;
  auto &motion_stop = ctx.motion_stop;
  auto &inspection_runtime = ctx.inspection_runtime;
  auto &inspection_command_coordinator = ctx.inspection_command_coordinator;
  auto &teleop_admission = ctx.teleop_admission;
  auto &teleop_tick = ctx.teleop_tick;
  auto &autonomy_tick = ctx.autonomy_tick;
  auto &rolling_segment = ctx.rolling_segment;
  auto &rolling_segment_effect_coordinator = ctx.rolling_segment_effect_coordinator;
  auto &nav_status = ctx.nav_status;
  auto &control_loop_health = ctx.control_loop_health;
  auto &control_loop_guard = ctx.control_loop_guard;
  auto &operator_motion_authority = ctx.operator_motion_authority;
  auto &command_ingress = ctx.command_ingress;
  auto &navigation_state = ctx.navigation_state;
  auto &active_map_identity = ctx.active_map_identity;
  auto &current_map_identity = ctx.current_map_identity;
  auto &clear_motion_outputs = ctx.clear_motion_outputs;
  auto &sync_goal_plan_diagnostics = ctx.sync_goal_plan_diagnostics;
  auto &control_loop_guard_latched = ctx.control_loop_guard_latched;
  auto &current_timing = ctx.current_timing;
  auto &operator_motion_admitted_sequence = ctx.operator_motion_admitted_sequence;
  auto &operator_motion_final_output_sequence = ctx.operator_motion_final_output_sequence;
  auto &navigation_map_identity = ctx.navigation_map_identity;
  auto &last_navigation_map_identity_refresh_s = ctx.last_navigation_map_identity_refresh_s;

  auto &odom_body = state.odom_body;
  auto &map_body = state.map_body;
  auto &map_odom_tf = state.map_odom_tf;
  auto &obstacle_xyzh = state.obstacle_xyzh;
  auto &terrain_xyzh = state.terrain_xyzh;
  auto &terrain_ext_xyzh = state.terrain_ext_xyzh;
  auto &planner_terrain_xyzh = state.planner_terrain_xyzh;
  auto &latest_dynamic_clusters = state.latest_dynamic_clusters;
  auto &traversability_grid = state.traversability_grid;
  auto &last_terrain_map_receive_s = state.last_terrain_map_receive_s;
  auto &last_terrain_ext_receive_s = state.last_terrain_ext_receive_s;
  auto &last_traversability_receive_s = state.last_traversability_receive_s;
  auto &last_odom_s = state.last_odom_s;
  auto &last_odom_linear_speed_mps = state.last_odom_linear_speed_mps;
  auto &last_odom_angular_speed_radps = state.last_odom_angular_speed_radps;
  auto &driver_accepted_producer_boot_id = state.driver_accepted_producer_boot_id;
  auto &driver_accepted_output_sequence = state.driver_accepted_output_sequence;
  auto &driver_last_command_accepted = state.driver_last_command_accepted;
  auto &driver_authority_previous = state.driver_authority_previous;
  auto &teleop_receive_time = state.teleop_receive_time;
  auto &teleop_received = state.teleop_received;
  auto &input_gate_state = state.input_gate_state;
  auto &last_plan = state.last_plan;
  auto &last_local = state.last_local;
  auto &last_teleop = state.last_teleop;
  auto &frames = state.frames;
  auto &operator_motion_transport = state.operator_motion_transport;
  auto &last_global_path = state.last_global_path;
  auto &last_local_path = state.last_local_path;
  auto &last_local_planner_debug = state.last_local_planner_debug;
  auto &control_authority = state.control_authority;
  auto &inspection_executor = state.inspection_executor;
  auto &operator_resume_required = state.operator_resume_required;
  auto &path_echo = state.path_echo;
  auto &odom_generation = state.odom_generation;
  auto &frame_epoch = state.frame_epoch;
  auto &goal_count = state.goal_count;
  auto &cancel_count = state.cancel_count;
  auto &teleop_cmd_count = state.teleop_cmd_count;
  auto &teleop_output_count = state.teleop_output_count;
  auto &teleop_stop_count = state.teleop_stop_count;
  auto &teleop_slow_count = state.teleop_slow_count;
  auto &teleop_limited_count = state.teleop_limited_count;
  auto &path_count = state.path_count;
  auto &plan_fail_count = state.plan_fail_count;
  auto &output_count = state.output_count;
  auto &cmd_vel_count = state.cmd_vel_count;
  auto &autonomy_request_not_before_s = state.autonomy_request_not_before_s;

  const auto tick_period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / std::max(1.0, cfg.tick_hz)));
  auto next_tick = SteadyClock::now();
  TimingDiagnostics last_timing;

  auto record_final_output_failure = [&](const std::string &reason) {
    control_authority.holdOperatorTakeover();
    operator_resume_required = true;
    last_teleop.published = false;
    frames.last_error = reason;
    nav_status.requestImmediate();
  };
  auto record_zero_publish_failure = [&](const std::string &reason) {
    record_final_output_failure(reason + ":zero_publish_failed");
  };
  auto fail_closed_after_cmd_vel_write = [&](const std::string &reason) {
    record_final_output_failure(reason);
    if (!motion_stop.keepZeroFresh()) {
      record_zero_publish_failure(reason);
    }
  };

  while (running) {
    const auto loop_start = SteadyClock::now();
    next_tick += tick_period;
    TimingDiagnostics timing;
    current_timing = &timing;
    timing.motion_update_last_ms = last_timing.motion_update_last_ms;
    timing.obstacle_snapshot_last_ms = last_timing.obstacle_snapshot_last_ms;
    const auto loop_guard_decision = control_loop_guard.observe(control_loop_health.snapshot());
    const std::string control_loop_hold_reason =
        "control_loop_unhealthy:" + loop_guard_decision.reason;
    if (loop_guard_decision.hold_motion) {
      control_authority.holdOperatorTakeover();
      operator_resume_required = true;
      if (loop_guard_decision.clear_motion) {
        (void)inspection_executor.Pause(control_loop_hold_reason);
        inspection_runtime.requestStatus();
        if (!motion_stop.clearEndpointMotion(control_loop_hold_reason)) {
          record_zero_publish_failure(control_loop_hold_reason);
        }
      }
    }
    if (!loop_guard_decision.clear_motion && loop_guard_decision.hold_motion) {
      if (!motion_stop.keepZeroFresh()) {
        record_zero_publish_failure(control_loop_hold_reason);
      }
    }
    (void)rolling_segment_effect_coordinator.apply(
        rolling_segment.step(RollingSegmentBeginTick{nowSeconds()}));
    const auto input_start = SteadyClock::now();
    dds.drainTf(
        [&](const lingtu_dds_TFMessage &msg) { input_projector.projectTf(msg, steadySeconds()); });
    dds.drainOdometry([&](const lingtu_dds_Odometry &msg) {
      input_projector.projectOdometry(msg, steadySeconds());
    });
    dds.drainDriverControlState([&](const lingtu_dds_DriverControlState &msg) {
      input_projector.projectDriverControl(msg, SteadyClock::now());
    });
    dds.drainExplorationExecutionGrids([&](const ExplorationExecutionGridView &grid) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentObserveExecutionGrid{
              rollingSegmentGridFromDds(grid),
          }));
      const auto rolling_state = rolling_segment.snapshot();
      if (!rolling_state.has_execution_grid && !rolling_state.input_error.empty()) {
        frames.last_error = rolling_state.input_error;
      }
    });
    auto submit_goal = [&](const lingtu_dds_PoseStamped &msg, const std::string &request_id,
                           GoalPlanOrigin origin) -> std::pair<bool, std::string> {
      ++goal_count;
      GoalPlanRequest request;
      request.request_id = request_id;
      request.origin = origin;
      request.source_stamp_s = headerStampSeconds(msg.header);
      const auto decoded = decodeGoal(msg, map_odom_tf);
      if (decoded.ok()) {
        request.target = GoalPlanTarget{decoded.value.position, decoded.value.yaw};
      } else {
        request.decode_error = decoded.error;
      }
      GoalPlanAdmissionContext admission;
      admission.motion_allowed = control_authority.motionAllowed();
      admission.operator_takeover_latched =
          control_authority.operatorTakeoverLatched() || control_loop_guard_latched();
      admission.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
      admission.control_mode_name = controlModeName(cfg.control_mode);
      admission.driver_control_blocker = input_projector.driverControlBlocker(SteadyClock::now());
      admission.autonomy_request_not_before_s = autonomy_request_not_before_s;
      if (map_body) {
        admission.map_position = map_body->position;
      }
      admission.odometry_ready = odom_body.has_value();
      admission.planner_map_configured = !cfg.map_path.empty();
      admission.planner_map_missing_reason = cfg.global_planner == GlobalPlannerBackend::Far
                                                 ? "active_occupancy_not_configured"
                                                 : "active_octomap_not_configured";
      admission.frame_epoch = frame_epoch;
      admission.rolling_segment_active = rolling_segment.snapshot().active;
      admission.planner_options = cfg.octoplanner_options;
      const auto result = goal_plan.submit(request, admission);
      sync_goal_plan_diagnostics();
      if (result.counted_failure) {
        ++plan_fail_count;
      }
      if (result.count_frame_rejection) {
        ++frames.goal_rejected;
      }
      if (result.record_frame_error) {
        frames.last_error = result.reason;
      }
      if (!result.accepted) {
        std::fprintf(stderr, "nav_native: reject goal, %s\n", result.reason.c_str());
      } else {
        std::fprintf(stderr, "nav_native: %s planning started\n",
                     globalPlannerBackendName(cfg.global_planner));
      }
      return {result.accepted, result.reason};
    };
    dds.drainLegacyGoals([&](const lingtu_dds_PoseStamped &msg) {
      (void)submit_goal(msg, {}, GoalPlanOrigin::kExternal);
    });
    dds.drainCloud([&](const lingtu_dds_PointCloud2 &msg) {
      input_projector.projectCloud(msg, steadySeconds(), nowSeconds(), timing);
    });
    dds.drainTerrainMap([&](const lingtu_dds_PointCloud2 &msg) {
      input_projector.projectTerrainMap(msg, steadySeconds(), timing);
    });
    dds.drainTerrainMapExt([&](const lingtu_dds_PointCloud2 &msg) {
      input_projector.projectTerrainMapExt(msg, steadySeconds(), timing);
    });
    dds.drainMapClearing([&](const lingtu_dds_Bool &msg) {
      input_projector.clearPlannerInputs(msg, PlannerInputClearSource::kMap);
    });
    dds.drainCloudClearing([&](const lingtu_dds_Bool &msg) {
      input_projector.clearPlannerInputs(msg, PlannerInputClearSource::kCloud);
    });
    dds.drainLegacyGlobalPath([&](const lingtu_dds_Path &msg) {
      if (!control_authority.motionAllowed()) {
        ++frames.path_rejected;
        frames.last_error = "estop_latched";
        return;
      }
      if (control_authority.operatorTakeoverLatched() || control_loop_guard_latched()) {
        ++frames.path_rejected;
        frames.last_error = "operator_takeover_resume_required";
        return;
      }
      if (cfg.control_mode != ControlMode::Autonomy) {
        ++frames.path_rejected;
        frames.last_error = std::string("path_not_allowed_in_") + controlModeName(cfg.control_mode);
        return;
      }
      const std::string path_driver_blocker =
          input_projector.driverControlBlocker(SteadyClock::now());
      if (!path_driver_blocker.empty()) {
        ++frames.path_rejected;
        frames.last_error = path_driver_blocker;
        return;
      }
      const double path_stamp_s = headerStampSeconds(msg.header);
      if (sourceStampPredates(path_stamp_s, autonomy_request_not_before_s)) {
        ++frames.path_rejected;
        frames.last_error = "path_predates_autonomy_resume";
        return;
      }
      const auto decoded = decodePath(msg, map_odom_tf);
      if (!decoded.ok()) {
        ++frames.path_rejected;
        frames.last_error = decoded.error;
        std::fprintf(stderr, "nav_native: reject global path, %s\n", decoded.error.c_str());
        return;
      }
      const auto &path = decoded.value;
      if (path_echo.take(path, path_stamp_s, nowSeconds())) {
        return;
      }
      if (path.size() < 2) {
        ++frames.path_rejected;
        frames.last_error = "path_too_short";
        return;
      }
      if (rolling_segment.snapshot().active) {
        control_authority.stop();
        if (!motion_stop.clearEndpointMotion("superseded_by_generic_path")) {
          ++frames.path_rejected;
          frames.last_error = "segment_preempt_zero_publish_failed";
          return;
        }
      }
      nav.setGlobalPath(path);
      last_global_path = path;
      last_local_path.clear();
      last_local_planner_debug = {};
      (void)control_authority.activatePath();
      last_plan = PlanDiagnostics{};
      last_plan.seen = true;
      last_plan.accepted = true;
      last_plan.reached_goal = false;
      last_plan.reason = "external_global_path";
      last_plan.waypoints = path.size();
      if (map_body) {
        last_plan.start = map_body->position;
      }
      last_plan.goal = path.back();
      last_local = LocalDiagnostics{};
      ++path_count;
    });
    dds.drainTraversability([&](const lingtu_dds_OccupancyGrid &msg) {
      input_projector.projectTraversability(msg, steadySeconds());
    });
    dds.drainLocalizationHealth([&](const lingtu_dds_Text &msg) {
      input_projector.projectLocalizationHealth(msg, steadySeconds());
    });
    auto handle_cancel = [&](const std::string &) -> std::pair<bool, std::string> {
      ++cancel_count;
      const auto result = motion_stop.cancel();
      return {result.accepted, result.reason};
    };
    auto handle_stop = [&](const std::string &) -> std::pair<bool, std::string> {
      const auto result = motion_stop.stop();
      return {result.accepted, result.reason};
    };
    auto handle_estop = [&](const std::string &reason) -> std::pair<bool, std::string> {
      const auto result = motion_stop.estop(reason);
      return {result.accepted, result.reason};
    };
    auto handle_clear_estop = [&](const lingtu_dds_Header &header) -> std::pair<bool, std::string> {
      const std::string stamp_error =
          sourceStampError("clear_estop", headerStampSeconds(header), nowSeconds(),
                           cfg.teleop_cmd_max_age_s, cfg.input_future_tolerance_s);
      const auto result = motion_stop.clearEstop(stamp_error);
      return {result.accepted, result.reason};
    };
    auto handle_resume_autonomy =
        [&](const lingtu_dds_Header &header) -> std::pair<bool, std::string> {
      const auto guard_resume = control_loop_guard.requestResume();
      const bool runtime_guard_latched = control_loop_guard_latched();
      ResumeAutonomyRequest request;
      if (cfg.control_mode != ControlMode::Autonomy) {
        request.precondition_error =
            std::string("resume_not_allowed_in_") + controlModeName(cfg.control_mode);
      } else if (!control_authority.motionAllowed()) {
        request.precondition_error = "estop_latched";
      } else {
        request.source_stamp_s = headerStampSeconds(header);
        request.precondition_error =
            sourceStampError("resume_autonomy", request.source_stamp_s, nowSeconds(),
                             cfg.teleop_cmd_max_age_s, cfg.input_future_tolerance_s);
        if (request.precondition_error.empty() && runtime_guard_latched &&
            !guard_resume.resume_allowed) {
          request.precondition_error = "control_loop_recovery_pending";
        }
      }
      request.operator_takeover_latched =
          control_authority.operatorTakeoverLatched() || runtime_guard_latched;
      const auto result = motion_stop.resumeAutonomy(request);
      const auto resume_result = control_loop_guard.completeResume(result.accepted);
      if (resume_result.resume_completed) {
        operator_resume_required = false;
      }
      return {result.accepted, result.reason};
    };
    dds.drainLegacyCancel([&](const lingtu_dds_Text &msg) { (void)handle_cancel(textData(msg)); });
    auto handle_teleop = [&](const lingtu_dds_TwistStamped &msg) -> std::pair<bool, std::string> {
      ++teleop_cmd_count;
      TeleopAdmissionContext context;
      context.motion_allowed = control_authority.motionAllowed();
      context.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
      context.allow_takeover = cfg.allow_teleop_takeover;
      context.operator_takeover_latched =
          control_authority.operatorTakeoverLatched() || control_loop_guard_latched();
      context.has_active_teleop = control_authority.teleopRequest().has_value();
      context.max_age_s = cfg.teleop_cmd_max_age_s;
      context.future_tolerance_s = cfg.input_future_tolerance_s;
      context.min_motion = cfg.teleop_min_motion_speed_mps;
      context.publish_cmd_vel = cfg.publish_cmd_vel;
      TeleopAdmissionRequest request;
      if (context.motion_allowed) {
        if (control_loop_guard_latched()) {
          request.decode_error = "control_loop_runtime_hold";
        } else {
          const auto decoded = decodeTwist(msg);
          if (!decoded.ok()) {
            request.decode_error = decoded.error;
          } else {
            request.command = decoded.value;
            request.source_stamp_s = headerStampSeconds(msg.header);
            context.receive_s = nowSeconds();
          }
        }
      }
      const auto admission = teleop_admission.admit(request, context);
      frames.teleop_rejected += admission.delta.frame_rejected;
      teleop_output_count += admission.delta.output_count;
      teleop_stop_count += admission.delta.stop_count;
      teleop_limited_count += admission.delta.limited_count;
      if (admission.delta.frame_error) {
        frames.last_error = *admission.delta.frame_error;
      }
      const auto &observation = admission.observation;
      if (observation.seen)
        last_teleop.seen = *observation.seen;
      if (observation.fresh)
        last_teleop.fresh = *observation.fresh;
      if (observation.published) {
        last_teleop.published = *observation.published;
      }
      if (observation.stopped)
        last_teleop.stopped = *observation.stopped;
      if (observation.limited)
        last_teleop.limited = *observation.limited;
      if (observation.reason)
        last_teleop.reason = *observation.reason;
      if (observation.request)
        last_teleop.request = *observation.request;
      if (observation.output)
        last_teleop.output = *observation.output;
      if (observation.age_s)
        last_teleop.age_s = *observation.age_s;
      if (admission.reason.rfind("teleop_source_stamp_", 0) == 0) {
        std::fprintf(stderr, "nav_native: reject teleop source stamp, reason=%s age_s=%.6f\n",
                     admission.reason.c_str(), observation.age_s.value_or(0.0));
      }
      if (admission.update_receive_timestamp) {
        teleop_receive_time = SteadyClock::now();
        teleop_received = true;
      }
      return {admission.accepted, admission.reason};
    };
    dds.drainLegacyTeleopCmdVel(
        [&](const lingtu_dds_TwistStamped &msg) { (void)handle_teleop(msg); });
    auto complete_operator_zero_barrier =
        [&](const std::string &reason) -> std::pair<OperatorMotionReceipt, std::uint64_t> {
      if (cfg.control_mode == ControlMode::Autonomy) {
        control_authority.holdOperatorTakeover();
        operator_resume_required = true;
      } else {
        control_authority.stop();
      }
      const std::uint64_t output_before = dds.lastOutputSequence();
      const bool cleared = motion_stop.clearEndpointMotion(reason);
      const std::uint64_t output_after = dds.lastOutputSequence();
      const auto &final_command = dds.lastOutputCommand();
      const bool old_motion_cleared =
          !control_authority.teleopRequest().has_value() && !control_authority.pathActive();
      const bool final_is_zero =
          final_command.vx == 0.0 && final_command.vy == 0.0 && final_command.wz == 0.0;
      const bool zero_published = cfg.publish_cmd_vel && cleared && old_motion_cleared &&
                                  output_after > output_before && final_is_zero;
      auto completion = operator_motion_authority.completeZeroBarrier(zero_published);
      const auto final_output_sequence = updateOperatorMotionOutputEvidence(
          completion, operator_motion_authority.snapshot(), output_after, final_command,
          operator_motion_admitted_sequence, operator_motion_final_output_sequence);
      if (!zero_published) {
        frames.last_error = reason + ":operator_zero_barrier_failed";
        return {std::move(completion), 0U};
      }
      return {std::move(completion), final_output_sequence};
    };
    auto write_operator_motion_ack = [&](const lingtu_dds_OperatorMotionControl &message,
                                         const OperatorMotionReceipt &receipt,
                                         std::uint64_t final_output_sequence) {
      OperatorMotionAckSample ack;
      ack.source_id = stringValue(message.source_id);
      ack.source_epoch = message.source_epoch;
      ack.sequence = message.source_sequence;
      ack.request_id = stringValue(message.request_id);
      ack.action = message.action;
      ack.accepted = receipt.accepted;
      ack.reason = receipt.reason;
      ack.accepted_sequence = receipt.accepted ? message.source_sequence : 0U;
      ack.final_output_sequence = final_output_sequence;
      const bool ack_published = dds.writeOperatorMotionAck(ack);
      operator_motion_transport.last_ack = OperatorMotionAckDiagnostics{true,
                                                                        ack_published,
                                                                        ack.source_id,
                                                                        ack.source_epoch,
                                                                        ack.sequence,
                                                                        ack.request_id,
                                                                        ack.action,
                                                                        ack.accepted,
                                                                        ack.reason,
                                                                        ack.accepted_sequence,
                                                                        ack.final_output_sequence};
      if (ack_published) {
        ++operator_motion_transport.ack_sent;
      } else {
        ++operator_motion_transport.ack_publish_failed;
        frames.last_error = "operator_motion_ack_publish_failed";
        nav_status.requestImmediate();
      }
    };
    dds.drainOperatorMotionControls([&](const lingtu_dds_OperatorMotionControl &message) {
      OperatorMotionReceipt receipt;
      std::uint64_t final_output_sequence{0U};
      const auto raw_action = message.action;
      const bool known_action = lingtu::message::isKnownOperatorMotionAction(raw_action);
      const auto action = known_action ? static_cast<OperatorMotionAction>(raw_action)
                                       : OperatorMotionAction::Claim;
      if (stringValue(message.request_id).empty()) {
        receipt = {false, "request_id_empty", false, false};
      } else if (!known_action) {
        receipt = {false, "action_unknown", false, false};
      } else if (action == OperatorMotionAction::Claim && !operator_motion_interface_enabled) {
        receipt = {false,
                   std::string("operator_motion_not_allowed_in_") +
                       controlModeName(cfg.control_mode),
                   false, false};
      } else if (action == OperatorMotionAction::Claim) {
        receipt = operator_motion_authority.claim(OperatorMotionClaim{
            stringValue(message.source_id), message.source_epoch, message.source_sequence,
            steadySeconds(), static_cast<double>(message.lease_ttl_ms) * 1e-3});
        if (receipt.accepted && receipt.authority_changed) {
          operator_motion_admitted_sequence = 0U;
          operator_motion_final_output_sequence = 0U;
        }
      } else {
        const OperatorMotionRelease control{stringValue(message.source_id), message.source_epoch,
                                            message.source_sequence};
        receipt = action == OperatorMotionAction::Release
                      ? operator_motion_authority.release(control)
                      : operator_motion_authority.hold(control);
      }
      const bool accepted_zero_action =
          receipt.accepted &&
          (action == OperatorMotionAction::Release || action == OperatorMotionAction::Hold);
      if (operator_motion_authority.snapshot().zero_barrier_pending) {
        auto zero_result = complete_operator_zero_barrier(
            action == OperatorMotionAction::Release
                ? "operator_motion_release"
                : (action == OperatorMotionAction::Hold ? "operator_motion_hold"
                                                        : "operator_motion_epoch_transition"));
        if (accepted_zero_action) {
          receipt = std::move(zero_result.first);
          if (receipt.accepted) {
            receipt.reason = action == OperatorMotionAction::Release ? "release_zero_published"
                                                                     : "hold_zero_published";
            final_output_sequence = zero_result.second;
          }
        }
      }
      write_operator_motion_ack(message, receipt, final_output_sequence);
    });
    dds.drainOperatorMotionSamples([&](const lingtu_dds_OperatorMotionSample &message) {
      const double source_stamp_s = static_cast<double>(message.source_stamp_ns) * 1e-9;
      OperatorMotionSample sample;
      sample.source_id = stringValue(message.source_id);
      sample.epoch = message.source_epoch;
      sample.sequence = message.source_sequence;
      sample.source_stamp_wall_s = source_stamp_s;
      sample.deadline_wall_s =
          source_stamp_s + static_cast<double>(message.freshness_budget_ms) * 1e-3;
      sample.deadman = message.deadman;
      sample.command = {message.velocity.linear.x, message.velocity.linear.y,
                        message.velocity.angular.z};
      const auto receipt = operator_motion_authority.submit(sample, steadySeconds(), nowSeconds());
      if (!receipt.accepted) {
        frames.last_error = "operator_motion_sample_" + receipt.reason;
        return;
      }
      const auto authority_snapshot = operator_motion_authority.snapshot();
      if (authority_snapshot.zero_barrier_pending) {
        (void)complete_operator_zero_barrier(message.deadman ? "operator_motion_lease_expired"
                                                             : "operator_motion_deadman_hold");
        return;
      }
      if (!authority_snapshot.has_active_sample ||
          authority_snapshot.last_sample_sequence != message.source_sequence) {
        return;
      }
      lingtu_dds_TwistStamped teleop{};
      teleop.header = message.header;
      if (source_stamp_s > 0.0 &&
          source_stamp_s < static_cast<double>(std::numeric_limits<std::int32_t>::max())) {
        teleop.header.stamp.sec = static_cast<std::int32_t>(source_stamp_s);
        teleop.header.stamp.nanosec = static_cast<std::uint32_t>(
            (source_stamp_s - static_cast<double>(teleop.header.stamp.sec)) * 1e9);
      }
      teleop.twist = message.velocity;
      const auto admission = handle_teleop(teleop);
      if (admission.first) {
        operator_motion_admitted_sequence = message.source_sequence;
        // Admission and final DDS publication are separate facts. Never carry
        // an older output sequence across a newly admitted sample.
        operator_motion_final_output_sequence = 0U;
      } else {
        operator_motion_admitted_sequence = 0U;
        operator_motion_final_output_sequence = 0U;
        frames.last_error = "operator_motion_admission_" + admission.second;
      }
    });
    if (!operator_motion_authority.snapshot().zero_barrier_pending) {
      (void)operator_motion_authority.tick(steadySeconds());
    }
    if (operator_motion_authority.snapshot().zero_barrier_pending) {
      (void)complete_operator_zero_barrier("operator_motion_lease_expired");
    }
    dds.drainCommandRequests([&](const lingtu_dds_NavigationCommandRequest &msg) {
      const auto ingress_request = commandIngressRequestFromDds(msg);
      const auto ingress_result = command_ingress.handle(
          ingress_request, [&](CommandKind kind, const CommandPayload &payload) -> CommandAck {
            std::pair<bool, std::string> result;
            if (kind == CommandKind::Goal) {
              lingtu_dds_PoseStamped goal{};
              goal.header = msg.header;
              goal.pose = msg.goal;
              result = submit_goal(goal, ingress_request.request_id, GoalPlanOrigin::kExternal);
            } else if (kind == CommandKind::Cancel) {
              result = handle_cancel(payload.reason);
            } else if (kind == CommandKind::Teleop) {
              if (!cfg.allow_legacy_motion_inputs) {
                result = {false, "legacy_teleop_disabled_use_operator_motion"};
              } else {
                lingtu_dds_TwistStamped teleop{};
                teleop.header = msg.header;
                teleop.twist = msg.velocity;
                result = handle_teleop(teleop);
              }
            } else if (kind == CommandKind::Stop) {
              result = payload.reason == "client_clock_sync"
                           ? std::pair<bool, std::string>{true, "clock_synchronized"}
                           : handle_stop(payload.reason);
            } else if (kind == CommandKind::Estop) {
              result = handle_estop(payload.reason);
            } else if (kind == CommandKind::ClearEstop) {
              result = handle_clear_estop(msg.header);
            } else {
              result = handle_resume_autonomy(msg.header);
            }
            return {result.first, result.second};
          });
      const bool ack_published =
          dds.writeCommandAck(ingress_result.request_id.c_str(), ingress_result.kind,
                              ingress_result.ack.accepted, ingress_result.ack.reason.c_str());
      command_ingress.recordAckPublication(ack_published);
      if (!ack_published) {
        frames.last_error = "command_ack_publish_failed";
        nav_status.requestImmediate();
      }
    });
    dds.drainInspectionCommands([&](const lingtu_dds_InspectionCommandRequest &msg) {
      InspectionCommandRequest request;
      request.request_id = stringValue(msg.request_id);
      request.raw_kind = msg.kind;
      request.route_id = stringValue(msg.route_id);
      request.route_revision = msg.route_revision;
      request.reason = stringValue(msg.reason);
      (void)inspection_command_coordinator.handle(request);
    });
    input_gate_state = input_projector.evaluateInputGate(steadySeconds(), SteadyClock::now());
    auto rolling_segment_context = [&]() {
      RollingSegmentRuntimeContext context;
      context.now_s = nowSeconds();
      context.input_ready = input_gate_state.ready;
      context.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
      context.motion_allowed = control_authority.motionAllowed() && !control_loop_guard_latched();
      context.operator_takeover_latched =
          control_authority.operatorTakeoverLatched() || control_loop_guard_latched();
      context.driver_control_ready =
          input_projector.driverControlBlocker(SteadyClock::now()).empty();
      const auto goal_snapshot = goal_plan.snapshot();
      context.generic_navigation_active =
          inspection_executor.active() || goal_snapshot.busy || control_authority.pathActive() ||
          !goal_snapshot.planning_request_id.empty() || !goal_snapshot.active_request_id.empty();
      if (map_body) {
        context.robot_pose =
            lingtu::explore::Pose2D{map_body->position.x, map_body->position.y, map_body->yaw};
        context.map_z = map_body->position.z;
      }
      return context;
    };
    dds.drainExplorationSegmentRequests([&](const ExplorationSegmentRequestView &request) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentCommandEvent{rollingSegmentCommandFromDds(request),
                                                          rolling_segment_context()}));
    });
    if (inspectionPostArrivalState(inspection_executor.status().state) &&
        localizationGateBlocked(input_gate_state, gate_cfg)) {
      if (inspection_executor.Pause("inspection_localization_health_blocked")) {
        if (!motion_stop.clearEndpointMotion("inspection_post_arrival_localization_pause")) {
          record_zero_publish_failure("inspection_post_arrival_localization_pause");
        }
        inspection_runtime.requestStatus();
      }
    }
    GoalPlanAdvanceContext advance_context;
    advance_context.frame_epoch = frame_epoch;
    advance_context.operator_takeover_latched =
        control_authority.operatorTakeoverLatched() || control_loop_guard_latched();
    advance_context.now_s = nowSeconds();
    auto advance_result = goal_plan.advance(advance_context);
    if (advance_result.completion_consumed) {
      sync_goal_plan_diagnostics();
      timing.global_plan_ms = advance_result.elapsed_ms;
      if (advance_result.counted_failure) {
        ++plan_fail_count;
      }
      if (advance_result.record_frame_error) {
        frames.last_error = last_plan.reason;
      }
      if (advance_result.terminal_after_stop.has_value()) {
        auto terminal_after_stop = std::move(*advance_result.terminal_after_stop);
        const auto terminal_result = motion_stop.commitGoalTerminalAfterStop(
            terminal_after_stop.reason, std::move(terminal_after_stop.commit));
        if (terminal_result.accepted) {
          sync_goal_plan_diagnostics();
        } else if (inspection_executor.active()) {
          (void)inspection_executor.Pause("planning_terminal_stop_unconfirmed:" +
                                          terminal_result.reason);
          inspection_runtime.requestStatus();
        }
      }
      if (advance_result.path_activated) {
        ++path_count;
        std::fprintf(stderr,
                     "nav_native: %s path accepted waypoints=%zu elapsed_ms=%.1f reached=%d\n",
                     globalPlannerBackendName(cfg.global_planner), last_plan.waypoints,
                     last_plan.elapsed_ms, last_plan.reached_goal ? 1 : 0);
      } else if (!last_plan.reason.empty()) {
        std::fprintf(
            stderr, "nav_native: %s plan completed reason=%s map_check=%s elapsed_ms=%.1f\n",
            globalPlannerBackendName(cfg.global_planner), last_plan.reason.c_str(),
            advance_result.map_identity_error.empty() ? "ok"
                                                      : advance_result.map_identity_error.c_str(),
            last_plan.elapsed_ms);
      }
      if (advance_result.inspection_status_changed) {
        inspection_runtime.requestStatus();
      }
    }
    const double inspection_now = nowSeconds();
    std::vector<InspectionRuntimeEvidenceResult> inspection_evidence_results;
    dds.drainInspectionEvidenceResults([&](const lingtu_dds_InspectionEvidenceResult &result) {
      inspection_evidence_results.push_back({stringValue(result.request_id), result.persisted,
                                             stringValue(result.evidence_id),
                                             stringValue(result.reason)});
    });
    InspectionRuntimeTickInput inspection_tick_input;
    inspection_tick_input.now_s = inspection_now;
    inspection_tick_input.odom_generation = odom_generation;
    inspection_tick_input.arrival_sample = lingtu::nav::inspection::ArrivalSample{
        last_odom_s, last_odom_linear_speed_mps, last_odom_angular_speed_radps};
    if (map_body) {
      inspection_tick_input.robot_position =
          InspectionRuntimeRobotPosition{map_body->position.x, map_body->position.y};
    }
    inspection_tick_input.path_active = control_authority.pathActive();
    inspection_tick_input.goal_plan_busy = goal_plan.snapshot().busy;
    if (const auto active_map = active_map_identity()) {
      inspection_tick_input.active_map =
          InspectionRuntimeMapIdentity{active_map->first, active_map->second};
    }
    inspection_tick_input.evidence_worker_matched = dds.inspectionEvidenceWorkerMatched();
    inspection_tick_input.evidence_results = std::move(inspection_evidence_results);
    const auto inspection_result = inspection_runtime.tick(inspection_tick_input);
    for (const auto &intent : inspection_result.ordered_intents) {
      if (intent.kind == InspectionRuntimeIntentKind::kStopControlAuthority) {
        control_authority.stop();
      } else {
        if (!motion_stop.clearEndpointMotion(intent.reason)) {
          record_zero_publish_failure(intent.reason);
        }
      }
    }
    if (inspection_result.goal_dispatch) {
      const auto &point = inspection_result.goal_dispatch->point;
      const auto dispatch_result = submit_goal(inspectionGoalMessage(point, inspection_now), {},
                                               GoalPlanOrigin::kInspection);
      const auto completion = inspection_runtime.completeGoalDispatch(
          dispatch_result.first, dispatch_result.second, inspection_now);
      if (completion.clear_motion_reason) {
        if (!motion_stop.clearEndpointMotion(*completion.clear_motion_reason)) {
          record_zero_publish_failure(*completion.clear_motion_reason);
        }
      }
    }
    timing.input_callbacks_ms = elapsedMs(input_start);
    if (inspection_result.evidence_dispatch) {
      const auto &dispatch = *inspection_result.evidence_dispatch;
      const bool published = dds.writeInspectionEvidenceRequest(
          dispatch.action, dispatch.map_id, dispatch.map_version, dispatch.deadline_s);
      const auto completion = inspection_runtime.completeEvidenceDispatch(
          dispatch.action.request_id, published, inspection_now);
      if (completion.clear_motion_reason) {
        if (!motion_stop.clearEndpointMotion(*completion.clear_motion_reason)) {
          record_zero_publish_failure(*completion.clear_motion_reason);
        }
      }
    }
    const std::string driver_blocker = input_projector.driverControlBlocker(SteadyClock::now());
    const bool driver_authority_now = driver_blocker.empty();
    if (driver_authority_previous && !driver_authority_now) {
      if (!motion_stop.driverAuthorityLost(driver_blocker)) {
        record_zero_publish_failure("driver_control_lost:" + driver_blocker);
      }
    }
    if (!driver_authority_now && !motion_stop.keepZeroFresh()) {
      record_zero_publish_failure("driver_control_blocked:" + driver_blocker);
    }
    driver_authority_previous = driver_authority_now;
    (void)input_projector.materializeLiveObstacleSnapshot(timing);
    if (control_authority.estopLatched()) {
      last_local.seen = true;
      last_local.active = false;
      last_local.near_field_stop = true;
      last_local.reason = "estop_latched";
      last_local.cmd_vel = {};
      last_teleop.fresh = false;
      last_teleop.published = cfg.publish_cmd_vel;
      last_teleop.stopped = true;
      last_teleop.output = {};
      last_teleop.reason = "estop_latched";
      if (!motion_stop.keepZeroFresh()) {
        record_zero_publish_failure("estop_latched");
      }
    }
    const auto &active_teleop_request = control_authority.teleopRequest();
    const bool path_active = control_authority.pathActive();
    auto teleop_result = teleop_tick.tick(TeleopTickInput{
        cfg, safety_config, map_body, input_gate_state,
        active_teleop_request ? &*active_teleop_request : nullptr, path_active, obstacle_xyzh,
        traversability_grid, last_traversability_receive_s, last_teleop, timing});
    if (teleop_result.handled) {
      last_teleop = std::move(teleop_result.teleop);
      if (teleop_result.local) {
        const auto local_write_start = SteadyClock::now();
        if (teleop_result.publish.local_path) {
          dds.writeLocalPath(teleop_result.local_path);
        }
        if (teleop_result.publish.waypoint) {
          dds.writeWayPoint(teleop_result.waypoint);
        }
        timing.dds_write_ms += elapsedMs(local_write_start);
        last_local = std::move(*teleop_result.local);
        last_local_path = std::move(teleop_result.local_path);
        last_local_planner_debug = std::move(teleop_result.local_planner_debug);
      }
      bool cmd_vel_published = false;
      if (teleop_result.publish.cmd_vel) {
        const auto write_start = SteadyClock::now();
        const auto receipt = dds.writeCmdVelSequenced(teleop_result.publish.command);
        cmd_vel_published = receipt.has_value();
        timing.dds_write_ms += elapsedMs(write_start);
        if (cmd_vel_published) {
          operator_motion_final_output_sequence = receipt->output_sequence;
        } else {
          operator_motion_final_output_sequence = 0U;
          fail_closed_after_cmd_vel_write("teleop_cmd_vel_publish_failed");
        }
      } else if (operator_motion_admitted_sequence != 0U) {
        operator_motion_final_output_sequence = 0U;
      }
      if (cmd_vel_published) {
        cmd_vel_count += teleop_result.delta.cmd_vel_count;
      }
      teleop_output_count += teleop_result.delta.teleop_output_count;
      teleop_stop_count += teleop_result.delta.teleop_stop_count;
      teleop_slow_count += teleop_result.delta.teleop_slow_count;
      teleop_limited_count += teleop_result.delta.teleop_limited_count;
      output_count += teleop_result.delta.output_count;
      if (teleop_result.timing.nav_tick_measured) {
        timing.nav_tick_ms = teleop_result.timing.nav_tick_ms;
      }
    }
    timing.teleop_gate_ms = teleop_result.timing.teleop_gate_ms;
    if (rolling_segment.snapshot().active) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentRevalidate{rolling_segment_context()}));
    }
    const bool path_active_for_tick = control_authority.pathActive();
    auto autonomy_result = autonomy_tick.tick(
        AutonomyTickInput{safety_config, map_body, input_gate_state, path_active_for_tick,
                          control_authority.motionAllowed() && !control_loop_guard_latched(),
                          rolling_segment.snapshot().active, cfg.publish_cmd_vel,
                          traversability_grid, last_local, timing});
    if (autonomy_result.handled) {
      if (autonomy_result.local) {
        last_local = std::move(*autonomy_result.local);
      }
      if (autonomy_result.timing.nav_tick_measured) {
        timing.nav_tick_ms = autonomy_result.timing.nav_tick_ms;
      }
      if (autonomy_result.output) {
        auto &out = *autonomy_result.output;
        if (autonomy_result.publish.local_path) {
          const auto local_write_start = SteadyClock::now();
          dds.writeLocalPath(out.local_path_map);
          timing.dds_write_ms += elapsedMs(local_write_start);
        }
        last_local_path = std::move(out.local_path_map);
        last_local_planner_debug = std::move(out.local_planner_debug);
        if (autonomy_result.publish.waypoint) {
          const auto waypoint_write_start = SteadyClock::now();
          dds.writeWayPoint(out.target);
          timing.dds_write_ms += elapsedMs(waypoint_write_start);
        }
      }
      bool cmd_vel_published = false;
      if (autonomy_result.publish.cmd_vel) {
        const auto cmd_write_start = SteadyClock::now();
        cmd_vel_published = dds.writeCmdVel(autonomy_result.publish.command);
        timing.dds_write_ms += elapsedMs(cmd_write_start);
        if (!cmd_vel_published) {
          fail_closed_after_cmd_vel_write("autonomy_cmd_vel_publish_failed");
        }
      }
      if (autonomy_result.clear_local_path) {
        last_local_path.clear();
      }
      if (autonomy_result.clear_local_planner_debug) {
        last_local_planner_debug = {};
      }
      if (cmd_vel_published) {
        cmd_vel_count += autonomy_result.delta.cmd_vel_count;
      }
      output_count += autonomy_result.delta.output_count;
      switch (autonomy_result.outcome.kind) {
        case AutonomyTickOutcomeKind::kRollingFinalSafetyStopped:
          (void)rolling_segment_effect_coordinator.apply(rolling_segment.step(
              RollingSegmentMotionOutcome{RollingSegmentMotionOutcomeKind::kFinalSafetyStopped,
                                          autonomy_result.outcome.reason}));
          break;
        case AutonomyTickOutcomeKind::kRollingRecoveryExhausted:
          (void)rolling_segment_effect_coordinator.apply(rolling_segment.step(
              RollingSegmentMotionOutcome{RollingSegmentMotionOutcomeKind::kRecoveryExhausted,
                                          autonomy_result.outcome.reason}));
          break;
        case AutonomyTickOutcomeKind::kRollingReached:
          (void)rolling_segment_effect_coordinator.apply(
              rolling_segment.step(RollingSegmentMotionOutcome{
                  RollingSegmentMotionOutcomeKind::kReached, autonomy_result.outcome.reason}));
          break;
        case AutonomyTickOutcomeKind::kGoalFailed: {
          const bool inspection_navigating =
              inspection_executor.status().state == lingtu::nav::inspection::RunState::kNavigating;
          auto commit_failed = goal_plan.deferFailure(autonomy_result.outcome.reason);
          sync_goal_plan_diagnostics();
          const std::string stop_reason = inspection_navigating
                                              ? "inspection_local_recovery_exhausted"
                                              : autonomy_result.outcome.reason;
          const auto terminal_result =
              motion_stop.commitGoalTerminalAfterStop(stop_reason, std::move(commit_failed));
          if (!terminal_result.accepted) {
            if (inspection_navigating) {
              (void)inspection_executor.Pause("inspection_local_recovery_stop_unconfirmed:" +
                                              terminal_result.reason);
              inspection_runtime.requestStatus();
            }
          } else {
            sync_goal_plan_diagnostics();
            if (inspection_navigating) {
              (void)inspection_executor.OnNavigationFailed(autonomy_result.outcome.reason,
                                                           nowSeconds());
              inspection_runtime.requestStatus();
            }
          }
          break;
        }
        case AutonomyTickOutcomeKind::kGoalReached: {
          const bool inspection_arrival =
              autonomy_result.outcome.inspection_arrival_intent && inspection_executor.active();
          auto commit_reached =
              goal_plan.deferActiveTerminal(GoalState::Reached, autonomy_result.outcome.reason);
          const auto terminal_result = motion_stop.commitGoalTerminalAfterStop(
              autonomy_result.outcome.reason, std::move(commit_reached));
          if (!terminal_result.accepted) {
            if (inspection_arrival) {
              (void)inspection_executor.Pause("inspection_arrival_stop_unconfirmed:" +
                                              terminal_result.reason);
              inspection_runtime.requestStatus();
            }
          } else {
            sync_goal_plan_diagnostics();
            if (inspection_arrival) {
              inspection_runtime.onGoalReached(nowSeconds());
            }
          }
          break;
        }
        case AutonomyTickOutcomeKind::kNone:
          break;
      }
    }
    const double now = nowSeconds();
    if (inspection_executor.status().state == lingtu::nav::inspection::RunState::kNavigating &&
        path_active_for_tick && !input_gate_state.ready) {
      if (inspection_executor.Pause(std::string("input_gate_") + input_gate_state.reason)) {
        if (!motion_stop.clearEndpointMotion("inspection_input_gate_pause")) {
          record_zero_publish_failure("inspection_input_gate_pause");
        }
        inspection_runtime.requestStatus();
      }
    }
    if (inspection_runtime.takeStatusDue(now)) {
      dds.writeInspectionStatus(inspection_executor.status());
      inspection_status_writer.submit(inspection_executor.status());
    }
    const auto operator_motion_snapshot = operator_motion_authority.snapshot();
    const OperatorMotionStatusSample operator_status{operator_motion_snapshot.active_source_id,
                                                     operator_motion_snapshot.active_epoch,
                                                     operator_motion_snapshot.has_active_authority,
                                                     operator_motion_snapshot.holding,
                                                     operator_motion_snapshot.has_active_sample,
                                                     operator_motion_snapshot.last_sample_sequence,
                                                     operator_motion_admitted_sequence,
                                                     operator_motion_final_output_sequence,
                                                     operator_motion_snapshot.last_reason,
                                                     input_gate_state.reason,
                                                     last_teleop.output,
                                                     dds.lastOutputCommand()};
    const bool operator_status_published = dds.writeOperatorMotionStatus(operator_status);
    operator_motion_transport.status =
        OperatorMotionStatusDiagnostics{true,
                                        operator_status_published,
                                        operator_status.active_source_id,
                                        operator_status.active_source_epoch,
                                        operator_status.has_active_authority,
                                        operator_status.holding,
                                        operator_status.has_active_sample,
                                        operator_status.last_sample_sequence,
                                        operator_status.admitted_sequence,
                                        operator_status.final_output_sequence,
                                        operator_status.authority_reason,
                                        operator_status.input_gate_reason,
                                        operator_status.teleop_output,
                                        operator_status.final_cmd_vel};
    if (operator_status_published) {
      ++operator_motion_transport.status_sent;
    } else {
      ++operator_motion_transport.status_publish_failed;
      frames.last_error = "operator_motion_status_publish_failed";
      nav_status.requestImmediate();
    }
    if (now - last_navigation_map_identity_refresh_s >= 1.0) {
      last_navigation_map_identity_refresh_s = now;
      const auto identity = current_map_identity();
      if (identity.identity.has_value()) {
        navigation_map_identity =
            NavigationMapIdentity{identity.identity->map_id, identity.identity->version,
                                  identity.identity->artifact_sha256};
      } else {
        navigation_map_identity.reset();
      }
    }
    std::string navigation_authority = "none";
    if (control_authority.estopLatched()) {
      navigation_authority = "estop";
    } else if (operator_motion_snapshot.has_active_authority ||
               control_authority.operatorTakeoverLatched()) {
      navigation_authority = "operator";
    } else if (control_authority.pathActive()) {
      navigation_authority = "autonomy";
    }
    const auto navigation_state_write_start = SteadyClock::now();
    (void)dds.writeNavigationState(navigation_state.sample(NavigationStateContext{
        navigation_map_identity, control_authority.pathActive(), input_gate_state.ready,
        input_gate_state.reason, control_authority.estopLatched(), control_authority.estopReason(),
        control_authority.operatorTakeoverLatched(), last_local.recovery_state != 0,
        navigation_authority}));
    timing.dds_write_ms += elapsedMs(navigation_state_write_start);
    auto status_runtime_state = statusRuntimeStateFromEndpoint(state);
    status_runtime_state.final_output.producer_boot_id = dds.producerBootId();
    status_runtime_state.final_output.output_sequence = dds.lastOutputSequence();
    (void)nav_status.publishIfDue(status_runtime_state, command_ingress.diagnostics(), last_timing,
                                  timing);
    const auto before_sleep = SteadyClock::now();
    if (before_sleep < next_tick) {
      timing.sleep_ms = elapsedMs(before_sleep, next_tick);
      std::this_thread::sleep_until(next_tick);
    } else {
      timing.overrun_ms = elapsedMs(next_tick, before_sleep);
      next_tick = before_sleep;
    }
    timing.loop_ms = elapsedMs(loop_start);
    (void)control_loop_health.observe(
        ControlLoopSample{timing.loop_ms, timing.sleep_ms, timing.overrun_ms});
    last_timing = timing;
  }

  current_timing = nullptr;
  // -- Post-loop shutdown ---------------------------------------------------
  if (rolling_segment.snapshot().active) {
    (void)rolling_segment_effect_coordinator.apply(rolling_segment.step(RollingSegmentShutdown{}));
  }
  constexpr int kShutdownSegmentStatusRetryLimit = 3;
  for (int attempt = 0; attempt < kShutdownSegmentStatusRetryLimit &&
                        rolling_segment.snapshot().terminal_delivery_pending;
       ++attempt) {
    (void)rolling_segment_effect_coordinator.apply(
        rolling_segment.step(RollingSegmentBeginTick{nowSeconds()}));
  }
  if (rolling_segment.snapshot().terminal_delivery_pending) {
    std::fprintf(stderr,
                 "navd shutdown: rolling segment terminal status remained "
                 "undelivered after %d retries\n",
                 kShutdownSegmentStatusRetryLimit);
  }
  const auto shutdown_result = motion_stop.finalShutdown();
  if (!shutdown_result.success) {
    if (!shutdown_result.confirmation_state) {
      std::fputs("navd shutdown: failed to publish final sequenced zero\n", stderr);
    } else {
      std::fprintf(stderr, "navd shutdown: final zero was not confirmed (state=%d)\n",
                   static_cast<int>(*shutdown_result.confirmation_state));
    }
    return 1;
  }
  return 0;
}

}  // namespace lingtu::nav::endpoint
