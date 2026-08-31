#include "runtime/loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "command/cancel.hpp"
#include "command/goal.hpp"
#include "command/ingress.hpp"
#include "command/terminal.hpp"
#include "runtime/config/config.hpp"
#include "control/admission.hpp"
#include "control/authority.hpp"
#include "control/autonomy.hpp"
#include "control/guard.hpp"
#include "control/operator.hpp"
#include "control/teleop.hpp"
#include "dds/runtime.hpp"
#include "input/gate.hpp"
#include "input/projector.hpp"
#include "runtime/inspection/inspection_command_coordinator.hpp"
#include "runtime/inspection/inspection_runtime_controller.hpp"
#include "message/cpp/geofence.hpp"
#include "nav/cpp/platform/runtime.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav/inspection/store.hpp"
#include "navigation/executor.hpp"
#include "runtime/goal/blockage.hpp"
#include "runtime/goal/plan.hpp"
#include "runtime/goal/runtime.hpp"
#include "runtime/rolling/effects.hpp"
#include "runtime/rolling/lifecycle.hpp"
#include "runtime/navigation.hpp"
#include "runtime/state.hpp"
#include "runtime/time.hpp"
#include "safety/command.hpp"
#include "safety/geofence.hpp"
#include "safety/stop.hpp"
#include "status/control_loop_health.hpp"
#include "status/goal_terminal_status_delivery.hpp"
#include "status/goal_terminal_transaction.hpp"
#include "status/inspection_status_file_writer.hpp"
#include "status/nav_status_endpoint_adapter.hpp"
#include "status/nav_status_publisher.hpp"
#include "status/navigation_goal_status_outbox.hpp"
#include "status/navigation_state.hpp"

namespace {

using CommandKind = lingtu::message::NavigationCommandKind;
using OperatorMotionAction = lingtu::message::OperatorMotionAction;

using lingtu::nav::endpoint::elapsedMs;
using lingtu::nav::endpoint::nowSeconds;
using lingtu::nav::endpoint::sourceStampError;
using lingtu::nav::endpoint::steadySeconds;

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

lingtu::nav::endpoint::GoalSample inspectionGoal(const lingtu::nav::inspection::Point &point,
                                                 double stamp_s) {
  lingtu::nav::endpoint::GoalSample goal;
  goal.stamp_s = stamp_s;
  goal.frame_id = "map";
  goal.position = {point.x_m, point.y_m, point.z_m};
  const double half_yaw = point.has_yaw ? point.yaw_rad * 0.5 : 0.0;
  goal.orientation.z = std::sin(half_yaw);
  goal.orientation.w = std::cos(half_yaw);
  goal.has_orientation = point.has_yaw;
  return goal;
}

}  // namespace

namespace lingtu::nav::endpoint {

int runEndpointLoop(EndpointLoopContext &ctx, const std::atomic_bool &running) {
  auto &cfg = ctx.cfg;
  auto &gate_cfg = ctx.gate_cfg;
  auto &safety_config = ctx.safety_config;
  auto &operator_motion_interface_enabled = ctx.operator_motion_interface_enabled;
  auto &dds = ctx.dds;
  auto &dds_status = ctx.dds_status;
  auto &state = ctx.state;
  auto &executor = ctx.executor;
  auto &geofence = ctx.geofence;
  auto &inspection_status_writer = ctx.inspection_status_writer;
  auto *inspection_checkpoint_store = ctx.inspection_checkpoint_store;
  lingtu::nav::inspection::InspectionTaskEventOutbox &inspection_task_event_outbox =
      ctx.inspection_task_event_outbox;
  auto &inputs = ctx.inputs;
  auto &plan_preview = ctx.plan_preview;
  auto &goal_plan = ctx.goal_plan;
  auto &goal_replan_runtime = ctx.goal_replan_runtime;
  auto &active_path_blockage_policy = ctx.active_path_blockage_policy;
  auto &motion_stop = ctx.motion_stop;
  auto &goal_status_outbox = ctx.goal_status_outbox;
  auto &goal_terminal_delivery = ctx.goal_terminal_delivery;
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
  auto &current_map_identity = ctx.current_map_identity;
  auto &sync_goal_plan_diagnostics = ctx.sync_goal_plan_diagnostics;
  auto &control_loop_guard_latched = ctx.control_loop_guard_latched;
  auto &current_timing = ctx.current_timing;
  std::uint64_t operator_motion_admitted_sequence{0U};
  std::uint64_t operator_motion_final_output_sequence{0U};
  std::optional<NavigationMapIdentity> navigation_map_identity;
  double last_navigation_map_identity_refresh_s{0.0};

  auto &odom_body = state.odom_body;
  auto &map_body = state.map_body;
  auto &map_odom_tf = state.map_odom_tf;
  auto &obstacle_xyzh = state.obstacle_xyzh;
  auto &traversability_grid = state.traversability_grid;
  auto &last_traversability_receive_s = state.last_traversability_receive_s;
  auto &last_odom_s = state.last_odom_s;
  auto &last_odom_linear_speed_mps = state.last_odom_linear_speed_mps;
  auto &last_odom_angular_speed_radps = state.last_odom_angular_speed_radps;
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
  auto &odom_generation = state.odom_generation;
  auto &frame_epoch = state.frame_epoch;
  auto &cloud_generation = state.cloud_generation;
  auto &traversability_generation = state.traversability_generation;
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
  auto &teleop_request_not_before_s = state.teleop_request_not_before_s;

  auto evaluate_input_gate = [&]() {
    auto evaluated = inputs.evaluateGate(steadySeconds(), SteadyClock::now());
    if (map_body) {
      if (const auto zone = geofence.intrusion(map_body->position.x, map_body->position.y)) {
        evaluated.ready = false;
        evaluated.recovering = false;
        evaluated.reason = "geofence_restricted_zone:" + *zone;
      }
    }
    return evaluated;
  };

  const auto tick_period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / std::max(1.0, cfg.tick_hz)));
  lingtu::nav::platform::DeadlineSleeper deadline_sleeper;
  auto next_tick = SteadyClock::now();
  TimingDiagnostics last_timing;
  std::optional<std::string> last_geofence_intrusion;

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

  std::optional<InspectionGoalDispatchIntent> staged_inspection_goal;
  auto terminal_delivery_error = [&](const std::string &reason) {
    frames.last_error = reason;
    nav_status.requestImmediate();
  };
  GoalTerminalTransaction goal_terminal_transaction(goal_replan_runtime, motion_stop,
                                                    goal_terminal_delivery,
                                                    GoalTerminalTransactionActions{
                                                        terminal_delivery_error,
                                                        [&](const std::string &reason) {
                                                          if (inspection_executor.active()) {
                                                            (void)inspection_executor.Pause(reason);
                                                            inspection_runtime.requestStatus();
                                                          }
                                                        },
                                                        sync_goal_plan_diagnostics,
                                                    });
  NavigationRuntimeController navigation_runtime_controller(goal_plan, goal_replan_runtime,
                                                            goal_terminal_transaction);
  auto terminal_ingress = [&](GoalTerminalIngressKind kind) {
    return evaluateGoalTerminalIngress(navigation_runtime_controller.terminalPending()
                                           ? GoalTerminalBarrierState::kTerminalPending
                                           : GoalTerminalBarrierState::kOpen,
                                       kind);
  };
  GoalTaskCancelRouter task_cancel_router(
      goal_plan, goal_replan_runtime,
      [&](const GoalReplanRuntimeResult &runtime_result) {
        const GoalTerminalTransactionResult serviced =
            navigation_runtime_controller.completeTerminal(runtime_result);
        return GoalTaskCancelTerminalServiceResult{serviced.action_committed, serviced.reason};
      },
      [&] { nav_status.requestImmediate(); });

  auto next_internal_goal_identity = [&](const std::string &source) {
    const std::string prefix =
        dds_status.producer_boot_id + ":" + source + ":" +
        std::to_string(goal_count + 1U);
    return std::pair<std::string, std::string>{prefix + ":task", prefix + ":request"};
  };
  struct PendingPreview {
    std::string request_id;
    nav_kernel::Vec3 start;
    nav_kernel::Vec3 goal;
  };
  std::optional<PendingPreview> pending_preview;
  const std::string preview_planner = globalPlannerBackendName(cfg.global_planner);
  auto write_preview = [&](PlanResultSample result) {
    if (!dds.publish(OutputEvent{std::move(result)}).published) {
      frames.last_error = "plan_result_publish_failed";
      nav_status.requestImmediate();
    }
  };
  auto cancel_preview = [&]() {
    if (!pending_preview) {
      return;
    }
    write_preview({pending_preview->request_id,
                   false,
                   true,
                   "preview_preempted",
                   0.0,
                   preview_planner,
                   pending_preview->start,
                   pending_preview->goal,
                   {}});
    pending_preview.reset();
    plan_preview.cancel();
  };
  auto submit_goal = [&](const GoalSample &goal, const std::string &task_id,
                         const std::string &request_id,
                         GoalPlanOrigin origin) -> std::pair<bool, std::string> {
    GoalPlanRequest request;
    request.task_id = task_id;
    request.request_id = request_id;
    request.origin = origin;
    request.source_stamp_s = goal.stamp_s;
    const auto decoded = parseGoal(goal, map_odom_tf);
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
    admission.driver_control_blocker = inputs.driverBlocker(SteadyClock::now());
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
    const auto submission =
        navigation_runtime_controller.submitGoal(request, admission, steadySeconds());
    const auto &result = submission.plan_result;
    if (submission.preemption_terminal || result.reason == "goal_terminal_pending") {
      nav_status.requestImmediate();
      return {false, "goal_terminal_pending"};
    }
    if (result.accepted) {
      cancel_preview();
    }
    ++goal_count;
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
  auto task_resume_context = [&]() {
    input_gate_state = evaluate_input_gate();
    GoalPlanAdmissionContext context;
    context.motion_allowed = control_authority.motionAllowed();
    context.operator_takeover_latched =
        control_authority.operatorTakeoverLatched() || control_loop_guard_latched();
    context.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
    context.control_mode_name = controlModeName(cfg.control_mode);
    context.driver_control_blocker = inputs.driverBlocker(SteadyClock::now());
    context.autonomy_request_not_before_s = autonomy_request_not_before_s;
    context.input_ready = input_gate_state.ready;
    context.input_gate_reason = input_gate_state.reason;
    if (map_body) {
      context.map_position = map_body->position;
    }
    context.odometry_ready = odom_body.has_value();
    context.planner_map_configured = !cfg.map_path.empty();
    context.retained_path_ready = executor.hasRoute() && !last_global_path.empty();
    context.retained_path_reason = "retained_global_path_missing";
    context.planner_map_missing_reason = cfg.global_planner == GlobalPlannerBackend::Far
                                             ? "active_occupancy_not_configured"
                                             : "active_octomap_not_configured";
    context.frame_epoch = frame_epoch;
    context.rolling_segment_active = rolling_segment.snapshot().active;
    context.planner_options = cfg.octoplanner_options;
    return context;
  };
  auto handle_task_pause = [&](const std::string &task_id, const std::string &request_id,
                               const std::string &reason) -> std::pair<bool, std::string> {
    auto transition = goal_plan.deferPause(task_id, request_id, reason);
    if (!transition.accepted) {
      return {false, transition.reason};
    }
    const auto result = motion_stop.pauseTask(std::move(transition.commit));
    sync_goal_plan_diagnostics();
    if (result.accepted) {
      nav_status.requestImmediate();
    }
    return {result.accepted, result.reason};
  };
  auto handle_task_resume = [&](const std::string &task_id,
                                const std::string &request_id) -> std::pair<bool, std::string> {
    if (control_loop_guard_latched()) {
      return {false, "control_loop_recovery_pending"};
    }
    auto transition = goal_plan.deferResume(task_id, request_id, task_resume_context());
    if (!transition.accepted) {
      return {false, transition.reason};
    }
    if (!control_authority.activatePath()) {
      return {false, "path_resume_authority_rejected"};
    }
    transition.commit();
    sync_goal_plan_diagnostics();
    nav_status.requestImmediate();
    return {true, "resume_requested"};
  };
  auto handle_stop = [&](const std::string &) -> std::pair<bool, std::string> {
    const auto interruption = navigation_runtime_controller.interrupt(
        GoalReplanRuntimeInterruption::kStop, steadySeconds());
    if (interruption.terminal_transaction) {
      const auto &terminal_result = *interruption.terminal_transaction;
      return {terminal_result.action_committed,
              terminal_result.action_committed ? "stopped" : terminal_result.reason};
    }
    const auto physical_stop = motion_stop.stopWithoutTerminalCommit();
    return {physical_stop.accepted, physical_stop.accepted ? "stopped" : physical_stop.reason};
  };
  auto handle_estop = [&](const std::string &reason) -> std::pair<bool, std::string> {
    const auto interruption = navigation_runtime_controller.interrupt(
        GoalReplanRuntimeInterruption::kEstop, steadySeconds(), reason);
    if (interruption.terminal_transaction) {
      const auto &terminal_result = *interruption.terminal_transaction;
      return {terminal_result.action_committed,
              terminal_result.action_committed ? "estop_latched" : terminal_result.reason};
    }
    const auto physical_estop = motion_stop.estopWithoutTerminalCommit(reason);
    return {physical_estop.accepted, physical_estop.reason};
  };
  auto handle_clear_estop = [&](double source_stamp_s) -> std::pair<bool, std::string> {
    const std::string stamp_error =
        sourceStampError("clear_estop", source_stamp_s, nowSeconds(),
                         cfg.teleop_cmd_max_age_s, cfg.input_future_tolerance_s);
    const auto result = motion_stop.clearEstop(stamp_error);
    return {result.accepted, result.reason};
  };
  auto handle_resume_motion = [&](double source_stamp_s) -> std::pair<bool, std::string> {
    const auto guard_resume = control_loop_guard.requestResume();
    const bool runtime_guard_latched = control_loop_guard_latched();
    MotionStopResult result;
    if (cfg.control_mode == ControlMode::Autonomy) {
      ResumeAutonomyRequest request;
      if (!control_authority.motionAllowed()) {
        request.precondition_error = "estop_latched";
      } else {
        request.source_stamp_s = source_stamp_s;
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
      result = motion_stop.resumeAutonomy(request);
    } else {
      ResumeTeleopRequest request;
      if (!control_authority.motionAllowed()) {
        request.precondition_error = "estop_latched";
      } else {
        request.source_stamp_s = source_stamp_s;
        request.precondition_error =
            sourceStampError("resume_autonomy", source_stamp_s, nowSeconds(),
                             cfg.teleop_cmd_max_age_s, cfg.input_future_tolerance_s);
        if (request.precondition_error.empty()) {
          input_gate_state = evaluate_input_gate();
          if (!input_gate_state.ready) {
            request.precondition_error =
                std::string("input_gate_") +
                (input_gate_state.reason.empty() ? "blocked" : input_gate_state.reason);
          } else if (runtime_guard_latched && !guard_resume.resume_allowed) {
            request.precondition_error = "control_loop_recovery_pending";
          }
        }
      }
      request.motion_hold_latched =
          control_authority.operatorTakeoverLatched() || runtime_guard_latched;
      result = motion_stop.resumeTeleop(request);
      if (result.accepted && request.motion_hold_latched) {
        input_gate_state = evaluate_input_gate();
        if (!input_gate_state.ready) {
          control_authority.holdOperatorTakeover();
          operator_resume_required = true;
          result = {
              false,
              std::string("input_gate_") +
                  (input_gate_state.reason.empty() ? "blocked" : input_gate_state.reason),
          };
        }
      }
    }
    const auto resume_result = control_loop_guard.completeResume(result.accepted);
    if (resume_result.resume_completed) {
      operator_resume_required = false;
    }
    return {result.accepted, result.reason};
  };
  auto handle_teleop = [&](const OperatorMotionInputSample &sample,
                           bool manual_mode) -> std::pair<bool, std::string> {
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
    context.request_not_before_s = teleop_request_not_before_s;
    context.min_motion = cfg.teleop_min_motion_speed_mps;
    context.publish_cmd_vel = cfg.publish_cmd_vel;
    TeleopAdmissionRequest request;
    if (context.motion_allowed) {
      if (control_loop_guard_latched()) {
        request.decode_error = "control_loop_runtime_hold";
      } else {
        const auto decoded = parseMotion(sample);
        if (!decoded.ok()) {
          request.decode_error = decoded.error;
        } else {
          request.command = decoded.value;
          const double explicit_source_stamp_s =
              static_cast<double>(sample.source_stamp_ns) * 1e-9;
          request.source_stamp_s =
              explicit_source_stamp_s > 0.0 &&
                      explicit_source_stamp_s <
                          static_cast<double>(std::numeric_limits<std::int32_t>::max())
                  ? explicit_source_stamp_s
                  : sample.stamp_s;
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
    if (admission.accepted) {
      control_authority.setTeleopManualMode(manual_mode);
    }
    return {admission.accepted, admission.reason};
  };
  auto complete_operator_zero_barrier =
      [&](const std::string &reason) -> std::pair<OperatorMotionReceipt, std::uint64_t> {
    if (cfg.control_mode == ControlMode::Autonomy) {
      control_authority.holdOperatorTakeover();
      operator_resume_required = true;
    } else {
      control_authority.stop();
    }
    const std::uint64_t output_before = dds_status.final_output_sequence;
    const bool cleared = motion_stop.clearEndpointMotion(reason);
    const std::uint64_t output_after = dds_status.final_output_sequence;
    const auto &final_command = dds_status.final_output_command;
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
  auto write_operator_motion_ack = [&](const OperatorMotionControlSample &message,
                                       const OperatorMotionReceipt &receipt,
                                       std::uint64_t final_output_sequence) {
    OperatorMotionAckSample ack;
    ack.source_id = message.source_id;
    ack.source_epoch = message.source_epoch;
    ack.sequence = message.source_sequence;
    ack.request_id = message.request_id;
    ack.action = message.action;
    ack.accepted = receipt.accepted;
    ack.reason = receipt.reason;
    ack.accepted_sequence = receipt.accepted ? message.source_sequence : 0U;
    ack.final_output_sequence = final_output_sequence;
    const bool ack_published = dds.publish(OutputEvent{ack}).published;
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
  auto rolling_segment_context = [&]() {
    RollingSegmentRuntimeContext context;
    context.now_s = nowSeconds();
    context.input_ready = input_gate_state.ready;
    context.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
    context.motion_allowed = control_authority.motionAllowed() && !control_loop_guard_latched();
    context.operator_takeover_latched =
        control_authority.operatorTakeoverLatched() || control_loop_guard_latched();
    context.driver_control_ready = inputs.driverBlocker(SteadyClock::now()).empty();
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
  auto publish_state = [&](double now, TimingDiagnostics &timing) {
    if (inspection_runtime.takeStatusDue(now)) {
      const auto &inspection_status = inspection_executor.status();
      (void)dds.publish(OutputEvent{InspectionStatusOutput{inspection_status}});
      inspection_status_writer.submit(inspection_status);
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
                                                     dds_status.final_output_command};
    const bool operator_status_published = dds.publish(OutputEvent{operator_status}).published;
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
            NavigationMapIdentity{identity.identity->map_id, identity.identity->content_epoch};
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
    (void)dds.publish(OutputEvent{navigation_state.sample(NavigationStateContext{
        navigation_map_identity, control_authority.pathActive(), input_gate_state.ready,
        input_gate_state.reason, control_authority.estopLatched(), control_authority.estopReason(),
        control_authority.operatorTakeoverLatched(), last_local.recovery_state != 0,
        navigation_authority})});
    timing.dds_write_ms += elapsedMs(navigation_state_write_start);
    auto status_runtime_state = statusRuntimeStateFromEndpoint(state);
    status_runtime_state.final_output.producer_boot_id = dds_status.producer_boot_id;
    status_runtime_state.final_output.output_sequence = dds_status.final_output_sequence;
    (void)nav_status.publishIfDue(status_runtime_state, command_ingress.diagnostics(), last_timing,
                                  timing);
  };
  auto shutdown = [&]() -> int {
    if (rolling_segment.snapshot().active) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentShutdown{}));
    }
    constexpr int kSegmentStatusRetryLimit = 3;
    for (int attempt = 0;
         attempt < kSegmentStatusRetryLimit && rolling_segment.snapshot().terminal_delivery_pending;
         ++attempt) {
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentBeginTick{nowSeconds()}));
    }
    if (rolling_segment.snapshot().terminal_delivery_pending) {
      std::fprintf(stderr,
                   "navd shutdown: rolling segment terminal status remained "
                   "undelivered after %d retries\n",
                   kSegmentStatusRetryLimit);
    }
    constexpr auto kPendingLogInterval = std::chrono::seconds(5);
    auto next_pending_log = SteadyClock::now();
    while (true) {
      const ShutdownTransactionResult transaction = advanceShutdownTransaction(
          goal_replan_runtime, motion_stop, goal_terminal_delivery, steadySeconds());
      if (transaction.decision.allow_exit) {
        return 0;
      }
      const auto log_now = SteadyClock::now();
      if (log_now >= next_pending_log) {
        std::fprintf(stderr,
                     "navd shutdown pending: %s "
                     "(stop_confirmed=%d terminal_required=%d terminal_pending=%d "
                     "delivery_acknowledged=%d terminal_flush=%d)\n",
                     transaction.reason.c_str(), transaction.stop_confirmed ? 1 : 0,
                     transaction.terminal_required ? 1 : 0,
                     navigation_runtime_controller.terminalPending() ? 1 : 0,
                     transaction.delivery_acknowledged ? 1 : 0,
                     static_cast<int>(transaction.terminal_flush));
        next_pending_log = log_now + kPendingLogInterval;
      }
      (void)motion_stop.keepZeroFresh();
      std::this_thread::sleep_for(tick_period);
    }
  };
  auto drain_sensors = [&](TimingDiagnostics &timing) {
    inputs.apply(dds.takeSensors(steadySeconds()), timing);
  };
  CommandBatch command_batch;
  auto drain_operator_motion = [&]() {
    auto process_control = [&](const OperatorMotionControlSample &message) {
      OperatorMotionReceipt receipt;
      std::uint64_t final_output_sequence{0U};
      const auto raw_action = message.action;
      const bool known_action = lingtu::message::isKnownOperatorMotionAction(raw_action);
      const auto action = known_action ? static_cast<OperatorMotionAction>(raw_action)
                                       : OperatorMotionAction::Claim;
      if (known_action) {
        GoalTerminalIngressKind ingress_kind = GoalTerminalIngressKind::kOperatorClaim;
        if (action == OperatorMotionAction::Hold) {
          ingress_kind = GoalTerminalIngressKind::kOperatorHold;
        } else if (action == OperatorMotionAction::Release) {
          ingress_kind = GoalTerminalIngressKind::kOperatorRelease;
        }
        const auto verdict = terminal_ingress(ingress_kind);
        if (verdict.decision != GoalTerminalIngressDecision::kAllow) {
          receipt = {false, std::string(verdict.reason), false, false};
          write_operator_motion_ack(message, receipt, final_output_sequence);
          return;
        }
      }
      if (message.request_id.empty()) {
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
            message.source_id, message.source_epoch, message.source_sequence,
            steadySeconds(), static_cast<double>(message.lease_ttl_ms) * 1e-3});
        if (receipt.accepted && receipt.authority_changed) {
          operator_motion_admitted_sequence = 0U;
          operator_motion_final_output_sequence = 0U;
          (void)navigation_runtime_controller.interrupt(
              GoalReplanRuntimeInterruption::kOperatorTakeover, steadySeconds());
        }
      } else {
        const OperatorMotionRelease control{message.source_id, message.source_epoch,
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
    };
    auto process_sample = [&](const OperatorMotionInputSample &message) {
      const auto verdict = terminal_ingress(GoalTerminalIngressKind::kOperatorMotionSample);
      if (verdict.decision != GoalTerminalIngressDecision::kAllow) {
        frames.last_error = verdict.reason;
        nav_status.requestImmediate();
        return;
      }
      const double source_stamp_s = static_cast<double>(message.source_stamp_ns) * 1e-9;
      OperatorMotionSample sample;
      sample.source_id = message.source_id;
      sample.epoch = message.source_epoch;
      sample.sequence = message.source_sequence;
      sample.source_stamp_wall_s = source_stamp_s;
      sample.deadline_wall_s =
          source_stamp_s + static_cast<double>(message.freshness_budget_ms) * 1e-3;
      sample.deadman = message.deadman;
      sample.manual_mode = message.manual_mode;
      sample.command = message.velocity;
      if (sample.manual_mode && cfg.control_mode != ControlMode::TeleopAvoid) {
        frames.last_error = "operator_motion_manual_mode_requires_teleop_avoid";
        return;
      }
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
      const auto admission =
          handle_teleop(message, authority_snapshot.active_sample_manual_mode);
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
    };
    for (const auto &event : command_batch.ordered) {
      if (const auto *control = std::get_if<OperatorMotionControlSample>(&event)) {
        process_control(*control);
      } else if (const auto *sample = std::get_if<OperatorMotionInputSample>(&event)) {
        process_sample(*sample);
      }
    }
    auto operator_snapshot = operator_motion_authority.snapshot();
    if (!operator_snapshot.zero_barrier_pending) {
      (void)operator_motion_authority.tick(steadySeconds());
      operator_snapshot = operator_motion_authority.snapshot();
    }
    if (operator_snapshot.zero_barrier_pending) {
      (void)complete_operator_zero_barrier("operator_motion_lease_expired");
    }
  };
  auto drain_commands = [&]() {
    if (auto completion = plan_preview.poll()) {
      if (pending_preview && pending_preview->request_id == completion->context.request_id) {
        std::vector<nav_kernel::Vec3> path;
        std::string reason;
        if (!completion->error.empty()) {
          reason = "planner_error";
        } else if (completion->result.cancelled) {
          reason = "preview_cancelled";
        } else {
          const auto map = current_map_identity();
          reason = globalPlanStaleReason(*completion, completion->context.goal_epoch, frame_epoch,
                                         map.identity);
          if (reason.empty() && !completion->result.ok) {
            reason = completion->result.failure_reason.empty() ? "global_planner_failed"
                                                               : completion->result.failure_reason;
          } else if (reason.empty() && !completion->result.reached_goal) {
            reason = "goal_not_reached";
          } else if (reason.empty()) {
            path = globalPlanPath(*completion);
            if (path.empty()) {
              reason = "empty_path";
            }
          }
        }
        PlanResultSample result{pending_preview->request_id,
                                reason.empty(),
                                true,
                                reason,
                                completion->result.elapsed_ms,
                                preview_planner,
                                pending_preview->start,
                                pending_preview->goal,
                                {}};
        if (result.feasible) {
          result.path = std::move(path);
        }
        write_preview(std::move(result));
        pending_preview.reset();
      }
    }
    auto process_navigation_command = [&](const NavigationCommandSample &sample) {
      const auto &ingress_request = sample.ingress;
      const auto ingress_result = command_ingress.handle(
          ingress_request, [&](CommandKind kind, const CommandPayload &payload) -> CommandAck {
            GoalTerminalIngressKind ingress_kind = GoalTerminalIngressKind::kUnknown;
            switch (kind) {
              case CommandKind::Goal:
                ingress_kind = GoalTerminalIngressKind::kTypedGoal;
                break;
              case CommandKind::TaskCancel:
                ingress_kind = GoalTerminalIngressKind::kTypedTaskCancel;
                break;
              case CommandKind::TaskPause:
                ingress_kind = GoalTerminalIngressKind::kTypedTaskPause;
                break;
              case CommandKind::TaskResume:
                ingress_kind = GoalTerminalIngressKind::kTypedTaskResume;
                break;
              case CommandKind::Stop:
                ingress_kind = payload.reason == "client_clock_sync"
                                   ? GoalTerminalIngressKind::kTypedClientClockSync
                                   : GoalTerminalIngressKind::kTypedStop;
                break;
              case CommandKind::Estop:
                ingress_kind = GoalTerminalIngressKind::kTypedEstop;
                break;
              case CommandKind::ClearEstop:
                ingress_kind = GoalTerminalIngressKind::kTypedClearEstop;
                break;
              case CommandKind::ResumeAutonomy:
                ingress_kind = GoalTerminalIngressKind::kTypedResumeAutonomy;
                break;
            }
            const auto terminal_verdict = terminal_ingress(ingress_kind);
            if (terminal_verdict.decision == GoalTerminalIngressDecision::kReject) {
              return {false, std::string(terminal_verdict.reason)};
            }
            if (terminal_verdict.decision == GoalTerminalIngressDecision::kSafetyStop) {
              const auto preserved = navigation_runtime_controller.stopWhileTerminalPending();
              return {preserved.accepted, preserved.reason};
            }
            if (terminal_verdict.decision == GoalTerminalIngressDecision::kSafetyEstop) {
              const auto preserved =
                  navigation_runtime_controller.estopWhileTerminalPending(payload.reason);
              return {preserved.accepted, preserved.reason};
            }
            std::pair<bool, std::string> result;
            if (kind == CommandKind::Goal) {
              result = submit_goal(sample.goal, ingress_request.task_id,
                                   ingress_request.request_id,
                                   GoalPlanOrigin::kExternal);
            } else if (kind == CommandKind::TaskCancel) {
              ++cancel_count;
              const CommandAck cancel_result = task_cancel_router.handle(
                  GoalTaskCancelRequest{ingress_request.task_id, ingress_request.request_id,
                                        payload.reason, steadySeconds()});
              result = {cancel_result.accepted, cancel_result.reason};
            } else if (kind == CommandKind::TaskPause) {
              result = handle_task_pause(ingress_request.task_id, ingress_request.request_id,
                                         payload.reason);
            } else if (kind == CommandKind::TaskResume) {
              result = handle_task_resume(ingress_request.task_id, ingress_request.request_id);
            } else if (kind == CommandKind::Stop) {
              result = payload.reason == "client_clock_sync"
                           ? std::pair<bool, std::string>{true, "clock_synchronized"}
                           : handle_stop(payload.reason);
            } else if (kind == CommandKind::Estop) {
              result = handle_estop(payload.reason);
            } else if (kind == CommandKind::ClearEstop) {
              result = handle_clear_estop(sample.goal.stamp_s);
            } else if (kind == CommandKind::ResumeAutonomy) {
              result = handle_resume_motion(sample.goal.stamp_s);
            } else {
              result = {false, "unknown_command_kind"};
            }
            return {result.first, result.second};
          });
      const bool ack_published =
          dds.publish(OutputEvent{CommandAckOutput{
                          ingress_result.task_id, ingress_result.request_id, ingress_result.kind,
                          ingress_result.ack.accepted, ingress_result.ack.reason}})
              .published;
      command_ingress.recordAckPublication(ack_published);
      if (!ack_published) {
        frames.last_error = "command_ack_publish_failed";
        nav_status.requestImmediate();
      }
    };
    auto process_plan_request = [&](const PlanPreviewRequest &request) {
      const std::string &request_id = request.request_id;
      const nav_kernel::Vec3 &goal = request.goal;
      const bool start_valid = map_body.has_value();
      const nav_kernel::Vec3 start = start_valid ? map_body->position : nav_kernel::Vec3{};
      auto reject = [&](const char *reason) {
        write_preview(
            {request_id, false, start_valid, reason, 0.0, preview_planner, start, goal, {}});
      };
      if (request_id.empty()) {
        return;
      }
      if (request.frame_id != "map") {
        reject("frame_not_map");
        return;
      }
      if (!std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.z)) {
        reject("invalid_goal");
        return;
      }
      if (!start_valid) {
        reject(state.odom_body ? "map_odom_tf_not_ready" : "odometry_not_ready");
        return;
      }
      if (cfg.map_path.empty()) {
        reject(cfg.global_planner == GlobalPlannerBackend::Far ? "active_occupancy_not_configured"
                                                               : "active_octomap_not_configured");
        return;
      }
      const auto active_goal = goal_plan.snapshot();
      if (active_goal.busy || active_goal.pending_plan_queued ||
          !active_goal.active_task_id.empty()) {
        reject("navigation_busy");
        return;
      }
      if (plan_preview.busy() || pending_preview) {
        reject("planner_busy");
        return;
      }
      GlobalPlanContext context;
      context.request_id = request_id;
      context.start = start;
      context.goal = goal;
      context.frame_epoch = frame_epoch;
      context.request.start = {start.x, start.y, start.z};
      context.request.goal = {goal.x, goal.y, goal.z};
      context.request.options = cfg.octoplanner_options;
      if (!plan_preview.start(std::move(context))) {
        reject("planner_busy");
        return;
      }
      pending_preview = PendingPreview{request_id, start, goal};
    };
    auto process_inspection_command = [&](const InspectionCommandRequest &request) {
      const auto verdict = terminal_ingress(GoalTerminalIngressKind::kInspectionCommand);
      if (verdict.decision != GoalTerminalIngressDecision::kAllow) {
        (void)inspection_command_coordinator.reject(request, std::string(verdict.reason));
        return;
      }
      (void)inspection_command_coordinator.handle(request);
    };
    auto process_geofence_command = [&](const GeofenceCommandView &request) {
      GeofenceCommandResult result;
      if (request.request_id.empty()) {
        result.reason = "missing_request_id";
        result.revision = geofence.revision();
        result.zones = geofence.summaries();
      } else if (!lingtu::message::isKnownGeofenceAction(request.action)) {
        result.reason = "unknown_action";
        result.revision = geofence.revision();
        result.zones = geofence.summaries();
      } else {
        result = geofence.apply({static_cast<lingtu::message::GeofenceAction>(request.action),
                                 request.name, request.polygon});
      }
      if (!dds.publish(OutputEvent{GeofenceCommandAckSample{
                           request.request_id, request.action, result.accepted, result.reason,
                           result.revision, std::move(result.zones)}})
               .published) {
        frames.last_error = "geofence_response_publish_failed";
        nav_status.requestImmediate();
      }
    };
    for (const auto &event : command_batch.ordered) {
      if (const auto *command = std::get_if<NavigationCommandSample>(&event)) {
        process_navigation_command(*command);
      } else if (const auto *request = std::get_if<PlanPreviewRequest>(&event)) {
        process_plan_request(*request);
      } else if (const auto *request = std::get_if<InspectionCommandRequest>(&event)) {
        process_inspection_command(*request);
      } else if (const auto *request = std::get_if<GeofenceCommandView>(&event)) {
        process_geofence_command(*request);
      }
    }
    const std::optional<std::string> current_geofence_intrusion =
        map_body ? geofence.intrusion(map_body->position.x, map_body->position.y) : std::nullopt;
    if (current_geofence_intrusion != last_geofence_intrusion) {
      const bool active = current_geofence_intrusion.has_value();
      const GeofenceAlertSample alert{
          active,
          active ? *current_geofence_intrusion : std::string{},
          map_body ? map_body->position.x : 0.0,
          map_body ? map_body->position.y : 0.0,
          map_body ? map_body->position.z : 0.0,
          geofence.revision(),
          active ? "restricted_zone_intrusion" : "restricted_zone_clear",
      };
      if (!dds.publish(OutputEvent{alert}).published) {
        frames.last_error = "geofence_alert_publish_failed";
        nav_status.requestImmediate();
      }
      last_geofence_intrusion = current_geofence_intrusion;
    }
    input_gate_state = evaluate_input_gate();
    auto process_rolling_command = [&](const RollingSegmentCommand &request) {
      const auto context = rolling_segment_context();
      auto command = request;
      const auto verdict = terminal_ingress(GoalTerminalIngressKind::kRollingCommand);
      if (verdict.decision != GoalTerminalIngressDecision::kAllow) {
        (void)rolling_segment_effect_coordinator.apply(
            rolling_segment.step(RollingSegmentIngressRejected{std::move(command), context,
                                                               std::string(verdict.reason)}));
        return;
      }
      (void)rolling_segment_effect_coordinator.apply(
          rolling_segment.step(RollingSegmentCommandEvent{std::move(command), context}));
    };
    for (const auto &event : command_batch.ordered) {
      if (const auto *request = std::get_if<RollingSegmentCommand>(&event)) {
        process_rolling_command(*request);
      }
    }
  };

  auto advance_runtime = [&](const SteadyClock::time_point &input_start,
                             TimingDiagnostics &timing) -> double {
    if (inspectionPostArrivalState(inspection_executor.status().state) &&
        localizationGateBlocked(input_gate_state, gate_cfg)) {
      if (inspection_executor.Pause("inspection_localization_health_blocked")) {
        if (!motion_stop.clearEndpointMotion("inspection_post_arrival_localization_pause")) {
          record_zero_publish_failure("inspection_post_arrival_localization_pause");
        }
        inspection_runtime.requestStatus();
      }
    }
    const std::string driver_blocker = inputs.driverBlocker(SteadyClock::now());
    const bool driver_authority_now = driver_blocker.empty();
    if (driver_authority_previous && !driver_authority_now) {
      (void)goal_replan_runtime.interrupt(
          GoalReplanRuntimeInterruption::kDriverAuthorityLost, steadySeconds());
      if (!motion_stop.clearEndpointMotion("driver_control_lost:" + driver_blocker)) {
        record_zero_publish_failure("driver_control_lost:" + driver_blocker);
      }
    }
    if (!driver_authority_now && !motion_stop.keepZeroFresh()) {
      record_zero_publish_failure("driver_control_blocked:" + driver_blocker);
    }
    driver_authority_previous = driver_authority_now;
    if (staged_inspection_goal && !navigation_runtime_controller.terminalPending()) {
      const auto verdict = terminal_ingress(GoalTerminalIngressKind::kInspectionGoalDispatch);
      if (verdict.decision == GoalTerminalIngressDecision::kAllow) {
        const auto identity = next_internal_goal_identity("inspection-leg");
        const auto dispatch_result =
            submit_goal(inspectionGoal(staged_inspection_goal->point, nowSeconds()),
                        identity.first, identity.second, GoalPlanOrigin::kInspection);
        staged_inspection_goal.reset();
        const auto completion = inspection_runtime.completeGoalDispatch(
            dispatch_result.first, dispatch_result.second, nowSeconds());
        if (completion.clear_motion_reason) {
          if (!motion_stop.clearEndpointMotion(*completion.clear_motion_reason)) {
            record_zero_publish_failure(*completion.clear_motion_reason);
          }
        }
      }
    }
    GoalReplanRuntimeFrameInput runtime_frame;
    runtime_frame.steady_now_s = steadySeconds();
    runtime_frame.wall_now_s = nowSeconds();
    runtime_frame.fresh_admission = task_resume_context();
    runtime_frame.inspection_active = inspection_executor.active();
    runtime_frame.rolling_segment_active = rolling_segment.snapshot().active;
    runtime_frame.control_hold = control_loop_guard_latched();
    bool path_active_for_tick = false;
    AutonomyTickResult autonomy_result;
    NavigationRuntimeFrameActions runtime_actions;
    runtime_actions.complete_endpoint_work_before_autonomy =
        [&](const GoalReplanRuntimeResult &runtime_advance_result) {
          const auto &plan_advance = runtime_advance_result.plan_advance;
          if (plan_advance.completion_consumed) {
            sync_goal_plan_diagnostics();
            timing.global_plan_ms = plan_advance.elapsed_ms;
            if (plan_advance.counted_failure) {
              ++plan_fail_count;
            }
            if (plan_advance.record_frame_error) {
              frames.last_error = last_plan.reason;
            }
            if (plan_advance.path_activated) {
              ++path_count;
              std::fprintf(
                  stderr, "nav_native: %s path accepted waypoints=%zu elapsed_ms=%.1f reached=%d\n",
                  globalPlannerBackendName(cfg.global_planner), last_plan.waypoints,
                  last_plan.elapsed_ms, last_plan.reached_goal ? 1 : 0);
            } else if (!last_plan.reason.empty()) {
              std::fprintf(
                  stderr, "nav_native: %s plan completed reason=%s map_check=%s elapsed_ms=%.1f\n",
                  globalPlannerBackendName(cfg.global_planner), last_plan.reason.c_str(),
                  plan_advance.map_identity_error.empty() ? "ok"
                                                          : plan_advance.map_identity_error.c_str(),
                  last_plan.elapsed_ms);
            }
            if (plan_advance.inspection_status_changed) {
              inspection_runtime.requestStatus();
            }
          }
          const double inspection_now = nowSeconds();
          std::vector<InspectionRuntimeEvidenceResult> inspection_evidence_results;
          for (const auto &event : command_batch.ordered) {
            if (const auto *result = std::get_if<InspectionEvidenceResultSample>(&event)) {
              inspection_evidence_results.push_back(
                  {result->request_id, result->persisted, result->evidence_id, result->reason});
            }
          }
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
          inspection_tick_input.goal_plan_busy =
              goal_plan.snapshot().busy || navigation_runtime_controller.terminalPending();
          if (cfg.map_identity.valid()) {
            inspection_tick_input.active_map = InspectionRuntimeMapIdentity{
                cfg.map_identity.map_id, cfg.map_identity.content_epoch};
          }
          inspection_tick_input.evidence_worker_matched =
              dds_status.inspection_evidence_worker_matched;
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
          if (inspection_result.goal_dispatch && !staged_inspection_goal) {
            staged_inspection_goal = *inspection_result.goal_dispatch;
          }
          timing.input_callbacks_ms = elapsedMs(input_start);
          if (inspection_result.evidence_dispatch) {
            const auto &dispatch = *inspection_result.evidence_dispatch;
            const bool published =
                dds.publish(OutputEvent{InspectionEvidenceRequestOutput{
                                dispatch.action, dispatch.map_id, dispatch.map_content_epoch,
                                dispatch.deadline_s}})
                    .published;
            const auto completion = inspection_runtime.completeEvidenceDispatch(
                dispatch.action.request_id, published, inspection_now);
            if (completion.clear_motion_reason) {
              if (!motion_stop.clearEndpointMotion(*completion.clear_motion_reason)) {
                record_zero_publish_failure(*completion.clear_motion_reason);
              }
            }
          }
          (void)inputs.materializeObstacles(timing);
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
              traversability_grid, last_traversability_receive_s, last_teleop, timing,
              control_authority.teleopManualMode()});
          if (teleop_result.handled) {
            last_teleop = std::move(teleop_result.teleop);
            if (teleop_result.local) {
              const auto local_write_start = SteadyClock::now();
              if (teleop_result.publish.local_path) {
                (void)dds.publish(OutputEvent{LocalPathOutput{teleop_result.local_path}});
              }
              if (teleop_result.publish.waypoint) {
                (void)dds.publish(OutputEvent{WaypointOutput{teleop_result.waypoint}});
              }
              timing.dds_write_ms += elapsedMs(local_write_start);
              last_local = std::move(*teleop_result.local);
              last_local_path = std::move(teleop_result.local_path);
              last_local_planner_debug = std::move(teleop_result.local_planner_debug);
            }
            bool cmd_vel_published = false;
            if (teleop_result.publish.cmd_vel) {
              const bool command_is_nonzero = teleop_result.publish.command.vx != 0.0 ||
                                              teleop_result.publish.command.vy != 0.0 ||
                                              teleop_result.publish.command.wz != 0.0;
              PostPlanningInputReadinessResult post_planning_readiness;
              if (command_is_nonzero) {
                input_gate_state = evaluate_input_gate();
                post_planning_readiness = enforcePostPlanningInputReadiness(
                    teleop_result.publish.command, input_gate_state,
                    [&](const std::string &reason) {
                      return motion_stop.clearEndpointMotion(reason);
                    },
                    teleop_result.teleop.manual_mode);
              }
              if (post_planning_readiness.stop_required) {
                operator_motion_final_output_sequence = 0U;
                last_teleop.output = {};
                last_teleop.published = false;
                last_teleop.stopped = true;
                last_teleop.reason = post_planning_readiness.reason;
                ++teleop_stop_count;
                if (!post_planning_readiness.stop_succeeded) {
                  record_zero_publish_failure(post_planning_readiness.reason);
                }
              } else {
                const auto write_start = SteadyClock::now();
                const auto receipt =
                    dds.publish(OutputEvent{FinalVelocityOutput{teleop_result.publish.command}});
                cmd_vel_published = receipt.final_velocity.has_value();
                timing.dds_write_ms += elapsedMs(write_start);
                if (cmd_vel_published) {
                  operator_motion_final_output_sequence =
                      receipt.final_velocity->output_sequence;
                } else {
                  operator_motion_final_output_sequence = 0U;
                  fail_closed_after_cmd_vel_write("teleop_cmd_vel_publish_failed");
                }
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
          path_active_for_tick = control_authority.pathActive();
        };
    runtime_actions.run_autonomy = [&](const GoalPlanSnapshot &goal_snapshot_for_tick) {
      const bool rolling_active = rolling_segment.snapshot().active;
      std::optional<GoalReplanIdentity> active_goal_identity;
      if (goal_snapshot_for_tick.active_origin &&
          *goal_snapshot_for_tick.active_origin == GoalPlanOrigin::kExternal &&
          !goal_snapshot_for_tick.active_task_id.empty() &&
          !goal_snapshot_for_tick.active_request_id.empty() &&
          goal_snapshot_for_tick.active_goal_epoch != 0U &&
          goal_snapshot_for_tick.active_map_identity &&
          goal_snapshot_for_tick.active_map_identity->valid()) {
        active_goal_identity = GoalReplanIdentity{
            goal_snapshot_for_tick.active_task_id,
            goal_snapshot_for_tick.active_request_id,
            goal_snapshot_for_tick.active_goal_epoch,
            *goal_snapshot_for_tick.active_map_identity,
        };
      }
      ActivePathBlockageObservation blockage_observation;
      blockage_observation.now_s = steadySeconds();
      blockage_observation.external_active_goal =
          cfg.global_planner == GlobalPlannerBackend::OctoPlanner3D && cfg.check_obstacle &&
          path_active_for_tick && active_goal_identity.has_value() && map_body.has_value() &&
          input_gate_state.ready && control_authority.motionAllowed() &&
          goalPlanAcceptsReplanTrigger(goal_snapshot_for_tick) && !control_loop_guard_latched() &&
          !inspection_executor.active() && !rolling_active;
      blockage_observation.local_path_viable =
          last_local.active && last_local.path_found && !last_local.near_field_stop &&
          last_local.local_path_points >= 2U && last_local.recovery_state == 0 &&
          !last_local.recovery_exhausted;
      if (active_goal_identity) {
        blockage_observation.goal = *active_goal_identity;
      }
      blockage_observation.frame_epoch = frame_epoch;
      if (map_body) {
        blockage_observation.robot_position = map_body->position;
      }
      blockage_observation.active_global_path = &last_global_path;
      blockage_observation.live_obstacles_xyzh = &obstacle_xyzh;
      blockage_observation.cloud_generation = cloud_generation;
      blockage_observation.traversability_generation = traversability_generation;
      auto obstruction_trigger = active_path_blockage_policy.observe(blockage_observation);
      autonomy_result = autonomy_tick.tick(
          AutonomyTickInput{safety_config, map_body, input_gate_state, path_active_for_tick,
                            goal_snapshot_for_tick.active_map_identity,
                            control_authority.motionAllowed() && !control_loop_guard_latched(),
                            rolling_active, cfg.publish_cmd_vel, traversability_grid, last_local,
                            timing, active_goal_identity, obstruction_trigger});
      GoalReplanRuntimeFrameInput autonomy_frame = runtime_frame;
      autonomy_frame.steady_now_s = steadySeconds();
      autonomy_frame.wall_now_s = nowSeconds();
      autonomy_frame.inspection_active = inspection_executor.active();
      autonomy_frame.rolling_segment_active = rolling_active;
      autonomy_frame.control_hold = control_loop_guard_latched();
      autonomy_frame.map_drift =
          autonomy_result.outcome.reason == "active_path_map_identity_missing" ||
          autonomy_result.outcome.reason == "active_map_unavailable_during_navigation" ||
          autonomy_result.outcome.reason == "active_map_changed_during_navigation";
      return NavigationRuntimeAutonomyObservation{autonomy_frame, autonomy_result.outcome,
                                                  autonomy_result.handled,
                                                  inspection_executor.active(), rolling_active};
    };
    runtime_actions.apply_autonomy_outputs = [&](const GoalReplanRuntimeResult &) {
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
            (void)dds.publish(OutputEvent{LocalPathOutput{out.local_path_map}});
            timing.dds_write_ms += elapsedMs(local_write_start);
          }
          last_local_path = std::move(out.local_path_map);
          last_local_planner_debug = std::move(out.local_planner_debug);
          if (autonomy_result.publish.waypoint) {
            const auto waypoint_write_start = SteadyClock::now();
            (void)dds.publish(OutputEvent{WaypointOutput{out.target}});
            timing.dds_write_ms += elapsedMs(waypoint_write_start);
          }
        }
        bool cmd_vel_published = false;
        if (autonomy_result.publish.cmd_vel) {
          const auto cmd_write_start = SteadyClock::now();
          cmd_vel_published =
              dds.publish(OutputEvent{FinalVelocityOutput{autonomy_result.publish.command}})
                  .published;
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
          last_local_planner_debug.backend = cfg.local_planner_backend;
        }
        if (cmd_vel_published) {
          cmd_vel_count += autonomy_result.delta.cmd_vel_count;
        }
        output_count += autonomy_result.delta.output_count;
        switch (autonomy_result.outcome.kind) {
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
          case AutonomyTickOutcomeKind::kGoalFailed:
          case AutonomyTickOutcomeKind::kGoalReached:
          case AutonomyTickOutcomeKind::kNone:
            break;
        }
      }
      return NavigationRuntimePostAutonomyState{inspection_executor.active(),
                                                inspection_executor.status().state ==
                                                    lingtu::nav::inspection::RunState::kNavigating};
    };
    const NavigationRuntimeFrameResult runtime_frame_result =
        navigation_runtime_controller.advanceFrame(runtime_frame, runtime_actions);
    if (runtime_frame_result.inspection_completion) {
      if (runtime_frame_result.inspection_completion->kind ==
          AutonomyTickOutcomeKind::kGoalFailed) {
        (void)inspection_executor.OnNavigationFailed(
            runtime_frame_result.inspection_completion->reason, nowSeconds());
      } else if (runtime_frame_result.inspection_completion->kind ==
                 AutonomyTickOutcomeKind::kGoalReached) {
        inspection_runtime.onGoalReached(nowSeconds());
      }
      inspection_runtime.requestStatus();
    }
    if (runtime_frame_result.pending_cycle_advanced &&
        runtime_frame_result.pending_result.pending_resumed) {
      sync_goal_plan_diagnostics();
      nav_status.requestImmediate();
    }
    if (!navigation_runtime_controller.terminalPending()) {
      (void)goal_status_outbox.flush();
    }
    const double publish_now = nowSeconds();
    if (inspection_executor.status().state == lingtu::nav::inspection::RunState::kNavigating &&
        path_active_for_tick && !input_gate_state.ready) {
      if (inspection_executor.Pause(std::string("input_gate_") + input_gate_state.reason)) {
        if (!motion_stop.clearEndpointMotion("inspection_input_gate_pause")) {
          record_zero_publish_failure("inspection_input_gate_pause");
        }
        inspection_runtime.requestStatus();
      }
    }
    auto inspection_event_record_result =
        lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted;
    std::string inspection_checkpoint_error;
    (void)inspection_executor.FlushTaskEvents([&](const lingtu::nav::inspection::TaskEvent &event) {
      if (inspection_checkpoint_store == nullptr) {
        inspection_checkpoint_error = "store_unavailable";
        return false;
      }
      const auto checkpoint = inspection_checkpoint_store->PutTaskEventCheckpoint(event);
      if (!checkpoint.ok) {
        inspection_checkpoint_error = checkpoint.reason;
        return false;
      }
      inspection_event_record_result = inspection_task_event_outbox.Record(event);
      return inspection_event_record_result ==
             lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted;
    });
    (void)inspection_task_event_outbox.Flush();
    const auto inspection_event_diagnostics = inspection_task_event_outbox.diagnostics();
    if (!inspection_checkpoint_error.empty()) {
      frames.last_error = "inspection_task_event_checkpoint_" + inspection_checkpoint_error;
      nav_status.requestImmediate();
    } else if (inspection_event_record_result !=
               lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted) {
      frames.last_error =
          std::string("inspection_task_event_") +
          lingtu::nav::inspection::TaskEventOutboxRecordResultName(inspection_event_record_result);
      nav_status.requestImmediate();
    } else if (inspection_event_diagnostics.pending != 0U &&
               inspection_event_diagnostics.delivery_failures != 0U) {
      frames.last_error = "inspection_task_event_delivery_pending";
      nav_status.requestImmediate();
    }
    return publish_now;
  };

  while (running) {
    const auto loop_start = SteadyClock::now();
    // A late scheduler wake-up belongs to the previous sleep, not to this
    // frame's control work. Rebase before assigning the next deadline so one
    // Windows wake-up delay cannot manufacture an overrun on the following
    // otherwise-fast frame.
    if (loop_start > next_tick) {
      next_tick = loop_start;
    }
    next_tick += tick_period;
    TimingDiagnostics timing;
    current_timing = &timing;
    timing.motion_update_last_ms = last_timing.motion_update_last_ms;
    timing.obstacle_snapshot_last_ms = last_timing.obstacle_snapshot_last_ms;
    const auto loop_guard_decision = control_loop_guard.observe(control_loop_health.snapshot());
    if (loop_guard_decision.hold_motion) {
      const std::string hold_reason = "control_loop_unhealthy:" + loop_guard_decision.reason;
      if (cfg.control_mode == ControlMode::Autonomy) {
        control_authority.holdOperatorTakeover();
        operator_resume_required = true;
      }
      if (loop_guard_decision.clear_motion) {
        (void)inspection_executor.Pause(hold_reason);
        inspection_runtime.requestStatus();
        if (!motion_stop.clearEndpointMotion(hold_reason)) {
          record_zero_publish_failure(hold_reason);
        }
      } else if (!motion_stop.keepZeroFresh()) {
        record_zero_publish_failure(hold_reason);
      }
    }
    (void)rolling_segment_effect_coordinator.apply(
        rolling_segment.step(RollingSegmentBeginTick{nowSeconds()}));
    const auto input_start = SteadyClock::now();
    const auto sensor_drain_start = SteadyClock::now();
    drain_sensors(timing);
    timing.sensor_drain_ms = elapsedMs(sensor_drain_start);
    const auto operator_drain_start = SteadyClock::now();
    command_batch = dds.takeCommands(steadySeconds());
    drain_operator_motion();
    timing.operator_drain_ms = elapsedMs(operator_drain_start);
    const auto command_drain_start = SteadyClock::now();
    drain_commands();
    timing.command_drain_ms = elapsedMs(command_drain_start);
    const auto runtime_start = SteadyClock::now();
    const double publish_now = advance_runtime(input_start, timing);
    timing.runtime_ms = elapsedMs(runtime_start);
    const auto publish_start = SteadyClock::now();
    publish_state(publish_now, timing);
    timing.publish_ms = elapsedMs(publish_start);
    const auto before_sleep = SteadyClock::now();
    if (before_sleep < next_tick) {
      const auto sleep_start = before_sleep;
      deadline_sleeper.sleepUntil(next_tick);
      // Record the actual time yielded to the scheduler, not the planned
      // sleep budget. ControlLoopHealth derives work_ms as loop_ms-sleep_ms;
      // using the planned sleep time incorrectly classifies OS wake-up jitter
      // as control-thread work and can trip the runtime hold gate even when
      // the hot path finished before its deadline.
      timing.sleep_ms = elapsedMs(sleep_start);
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
  return shutdown();
}

}  // namespace lingtu::nav::endpoint
