#include "app/loop.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "command/cancel.hpp"
#include "command/goal.hpp"
#include "command/ingress.hpp"
#include "command/terminal.hpp"
#include "app/config/config.hpp"
#include "app/planner.hpp"
#include "control/admission.hpp"
#include "control/authority.hpp"
#include "control/autonomy.hpp"
#include "control/operator.hpp"
#include "control/teleop.hpp"
#include "dds/runtime.hpp"
#include "input/gate.hpp"
#include "input/projector.hpp"
#include "app/inspection/commands.hpp"
#include "app/inspection/runtime.hpp"
#include "message/protocol/geofence.hpp"
#include "nav/cpp/platform/runtime.hpp"
#include "nav/inspection/inspection.hpp"
#include "nav/inspection/store.hpp"
#include "navigation/executor.hpp"
#include "app/goal/plan.hpp"
#include "app/goal/runtime.hpp"
#include "app/rolling/effects.hpp"
#include "app/rolling/lifecycle.hpp"
#include "app/state.hpp"
#include "app/semantic/view_query.hpp"
#include "app/time.hpp"
#include "safety/command.hpp"
#include "safety/geofence.hpp"
#include "safety/stop.hpp"
#include "status/control_loop_health.hpp"
#include "status/goal_terminal_status_delivery.hpp"
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

bool pathsMateriallyDiffer(const std::vector<nav_kernel::Vec3> &left,
                           const std::vector<nav_kernel::Vec3> &right) noexcept {
  if (left.size() != right.size())
    return true;
  constexpr double kPositionToleranceM = 0.01;
  constexpr double kPositionToleranceSquared =
      kPositionToleranceM * kPositionToleranceM;
  for (std::size_t index = 0; index < left.size(); ++index) {
    const double dx = left[index].x - right[index].x;
    const double dy = left[index].y - right[index].y;
    const double dz = left[index].z - right[index].z;
    if (dx * dx + dy * dy + dz * dz > kPositionToleranceSquared)
      return true;
  }
  return false;
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
  auto &safety_config = ctx.safety_config;
  auto &operator_enabled = ctx.operator_motion_interface_enabled;
  auto &dds = ctx.dds;
  auto &dds_status = ctx.dds_status;
  auto &state = ctx.state;
  auto &executor = ctx.executor;
  auto &geofence = ctx.geofence;
  auto &inspection_writer = ctx.inspection_status_writer;
  auto *checkpoint_store = ctx.inspection_checkpoint_store;
  lingtu::nav::inspection::InspectionTaskEventOutbox &inspection_outbox =
      ctx.inspection_task_event_outbox;
  auto &inputs = ctx.inputs;
  auto &plan_preview = ctx.plan_preview;
  auto &goal_plan = ctx.goal_plan;
  auto &replan = ctx.goal_replan_runtime;
  auto &motion_stop = ctx.motion_stop;
  auto &goal_outbox = ctx.goal_status_outbox;
  auto &runtime = replan;
  auto &terminal_delivery = ctx.goal_terminal_delivery;
  auto &inspection_runtime = ctx.inspection_runtime;
  auto &inspection_commands = ctx.inspection_command_coordinator;

  auto &teleop_tick = ctx.teleop_tick;
  auto &autonomy_tick = ctx.autonomy_tick;
  auto &rolling_segment = ctx.rolling_segment;
  auto &rolling_effects = ctx.rolling_segment_effect_coordinator;
  auto &nav_status = ctx.nav_status;
  auto &loop_health = ctx.control_loop_health;
  auto &operator_authority = ctx.operator_motion_authority;
  auto &command_ingress = ctx.command_ingress;
  auto &nav_state = ctx.navigation_state;
  auto &map_identity = ctx.current_map_identity;
  auto &sync_goal_diagnostics = ctx.sync_goal_plan_diagnostics;
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
  auto &input_gate_state = state.input_gate_state;
  auto &last_plan = state.last_plan;
  auto &last_local = state.last_local;
  auto &last_teleop = state.last_teleop;
  auto &frames = state.frames;
  auto &operator_transport = state.operator_motion_transport;
  auto &last_global_path = state.last_global_path;
  auto &last_local_path = state.last_local_path;
  auto &last_local_planner_debug = state.last_local_planner_debug;
  auto &control_authority = state.control_authority;
  auto &inspection_executor = state.inspection_executor;
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
  auto &autonomy_retry_after_s = state.autonomy_request_not_before_s;

  auto evaluate_input_gate = [&]() {
    auto evaluated = inputs.evaluateGate(steadySeconds(), SteadyClock::now());
    if (map_body) {
      if (const auto zone = geofence.intrusion(map_body->position.x, map_body->position.y)) {
        evaluated.ready = false;
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
  constexpr auto kLocalPathRefreshPeriod = std::chrono::milliseconds(100);
  SteadyClock::time_point last_local_path_publish_time{};
  bool local_path_published{false};

  auto publish_local_path = [&](const std::vector<nav_kernel::Vec3> &path,
                                TimingDiagnostics &timing) {
    const auto now = SteadyClock::now();
    const bool refresh_due =
        !local_path_published || now - last_local_path_publish_time >= kLocalPathRefreshPeriod;
    if (!refresh_due && !pathsMateriallyDiffer(last_local_path, path))
      return;
    const auto write_start = SteadyClock::now();
    (void)dds.publish(OutputEvent{LocalPathOutput{path}});
    timing.dds_write_ms += elapsedMs(write_start);
    last_local_path_publish_time = now;
    local_path_published = true;
  };

  const auto &hold = ctx.hold;
  auto fail_output = [&](const std::string &reason) {
    frames.last_error = reason;
    std::fprintf(stderr, "navd: %s\n", frames.last_error.c_str());
    nav_status.requestImmediate();
  };

  auto publish_final_velocity = [&](const nav_kernel::Twist &command,
                                    TimingDiagnostics &timing,
                                    const char *failure_reason) -> std::optional<std::uint64_t> {
    const auto write_start = SteadyClock::now();
    const auto receipt = dds.publish(OutputEvent{FinalVelocityOutput{command}});
    timing.dds_write_ms += elapsedMs(write_start);
    if (!receipt.final_velocity) {
      fail_output(failure_reason);
      return std::nullopt;
    }
    state.motion_stop_evidence.observePublishedFinalOutput(
        receipt.final_velocity->output_sequence, command,
        static_cast<double>(receipt.final_velocity->source_wall_ns) / 1e9);
    return receipt.final_velocity->output_sequence;
  };

  std::optional<InspectionGoalDispatchIntent> staged_inspection_goal;
  PlannerSelection planner_selection;
  struct PendingPlanner {
    CommandIngressRequest ingress;
    std::string stop_key;
    bool stopped{false};
    std::optional<PreparedPlanner> prepared;
  };
  std::optional<PendingPlanner> pending_planner;
  auto finish_planner = [&](bool accepted, const std::string &reason) {
    if (!pending_planner) return;
    const auto receipt = command_ingress.complete(pending_planner->ingress, {accepted, reason});
    command_ingress.recordAckPublication(dds.publish(OutputEvent{CommandAckOutput{
        receipt.task_id, receipt.request_id, receipt.kind, receipt.ack.accepted,
        receipt.ack.reason}}).published);
    if (pending_planner->prepared)
      planner_selection.retire(std::move(pending_planner->prepared->executor));
    pending_planner.reset();
    nav_status.requestImmediate();
  };
  auto terminal_ingress = [&](GoalTerminalIngressKind kind) {
    if ((pending_planner || planner_selection.busy()) &&
        kind != GoalTerminalIngressKind::kTypedStop &&
        kind != GoalTerminalIngressKind::kTypedEstop &&
        kind != GoalTerminalIngressKind::kOperatorHold &&
        kind != GoalTerminalIngressKind::kOperatorRelease &&
        kind != GoalTerminalIngressKind::kTypedClientClockSync)
      return GoalTerminalIngressVerdict{GoalTerminalIngressDecision::kReject, "planner_switch_pending"};
    return evaluateGoalTerminalIngress(runtime.terminalPending()
                                           ? GoalTerminalBarrierState::kTerminalPending
                                           : GoalTerminalBarrierState::kOpen,
                                       kind);
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
    if (ctx.semantic_view_query) ctx.semantic_view_query->cancel();
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
      request.target = GoalPlanTarget{decoded.value.position, decoded.value.yaw,
                                      decoded.value.max_speed_mps, decoded.value.acceptance_radius_m};
    } else {
      request.decode_error = decoded.error;
    }
    GoalPlanAdmissionContext admission;
    if (cfg.global_planner == GlobalPlannerBackend::OctoPlanner3D) {
      admission.body = executor.body();
    }
    admission.motion_allowed = control_authority.motionAllowed();
    admission.hold_reason = control_authority.holdReason();
    admission.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
    admission.control_mode_name = controlModeName(cfg.control_mode);
    admission.autonomy_request_not_before_s = autonomy_retry_after_s;
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
        runtime.submitGoal(request, admission, steadySeconds());
    const auto &result = submission.plan_result;
    if (!result.accepted &&
        (submission.preemption_terminal || result.reason == "goal_terminal_pending")) {
      nav_status.requestImmediate();
      return {false, "goal_terminal_pending"};
    }
    if (result.accepted) {
      cancel_preview();
    }
    ++goal_count;
    sync_goal_diagnostics();
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
    if (cfg.global_planner == GlobalPlannerBackend::OctoPlanner3D) {
      context.body = executor.body();
    }
    context.motion_allowed = control_authority.motionAllowed();
    context.hold_reason = control_authority.holdReason();
    context.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
    context.control_mode_name = controlModeName(cfg.control_mode);
    context.autonomy_request_not_before_s = autonomy_retry_after_s;
    context.input_ready = input_gate_state.ready;
    context.input_gate_reason = input_gate_state.reason;
    if (map_body) {
      context.map_position = map_body->position;
    }
    context.odometry_ready = odom_body.has_value();
    context.planner_map_configured = !cfg.map_path.empty();
    context.retained_path_ready = executor.hasRoute() && !last_global_path.empty();
    context.retained_path_reason = "retained_global_path_missing";
    context.replan_pending = replan.replanPending();
    context.planner_map_missing_reason = cfg.global_planner == GlobalPlannerBackend::Far
                                             ? "active_occupancy_not_configured"
                                             : "active_octomap_not_configured";
    context.frame_epoch = frame_epoch;
    context.rolling_segment_active = rolling_segment.snapshot().active;
    context.planner_options = cfg.octoplanner_options;
    return context;
  };
  std::optional<CommandIngressRequest> deferred_control_ack;
  struct PendingPosture {
    CommandIngressRequest ingress;
    lingtu::message::PostureRequest request;
    SteadyClock::time_point deadline;
    SteadyClock::time_point retry_at;
    std::string stop_key;
    bool waiting_stop{false};
  };
  std::optional<PendingPosture> pending_posture;
  bool external_selected = false;
  bool external_active = false;
  std::optional<lingtu::message::PostureRequest> external_stop_request;
  auto external_stop_retry = SteadyClock::now();
  auto finish_posture = [&](bool accepted, const std::string &reason) {
    if (!pending_posture) return;
    if (accepted && pending_posture->request.action == static_cast<int>(CommandKind::EnterExternal)) {
      external_selected = true;
      external_active = true;
    }
    if (accepted && pending_posture->request.action == static_cast<int>(CommandKind::ExitExternal)) {
      external_selected = false;
      external_active = false;
    }
    const auto receipt = command_ingress.complete(pending_posture->ingress, {accepted, reason});
    command_ingress.recordAckPublication(dds.publish(OutputEvent{CommandAckOutput{
        receipt.task_id, receipt.request_id, receipt.kind, receipt.ack.accepted,
        receipt.ack.reason}}).published);
    pending_posture.reset();
    nav_status.requestImmediate();
  };
  auto send_posture = [&]() {
    if (!pending_posture) return;
    if (!dds.publish(OutputEvent{pending_posture->request}).published) {
      finish_posture(false, "posture_publish_failed");
      return;
    }
    pending_posture->retry_at = SteadyClock::now() + std::chrono::milliseconds(100);
  };
  auto cancel_posture = [&](const std::string &reason) {
    if (!pending_posture) return;
    auto request = pending_posture->request;
    request.phase = 2;
    (void)dds.publish(OutputEvent{request});
    finish_posture(false, reason);
  };
  std::string deferred_control_key;
  auto finish_deferred_control_ack = [&](CommandAck ack) {
    if (!deferred_control_ack) return;
    const auto receipt = command_ingress.complete(*deferred_control_ack, std::move(ack));
    const bool published = dds.publish(OutputEvent{CommandAckOutput{
        receipt.task_id, receipt.request_id, receipt.kind, receipt.ack.accepted,
        receipt.ack.reason}}).published;
    command_ingress.recordAckPublication(published);
    deferred_control_ack.reset();
    deferred_control_key.clear();
  };
  auto resume_blocker = [&]() -> std::string {
    const auto operator_state = operator_authority.snapshot();
    const auto &manual = operator_state.active_sample;
    if (operator_state.has_active_sample && !operator_state.holding &&
        (manual.vx != 0.0 || manual.vy != 0.0 || manual.wz != 0.0))
      return "operator_authority_active";
    return {};
  };
  const GoalResumeActions resume_actions{
      [&] { operator_authority.releaseAfterStop(); },
      [&] {
        if (executor.hasRoute()) {
          executor.resumeRoute();
          (void)control_authority.activatePath();
        }
      },
      [&] { sync_goal_diagnostics(); nav_status.requestImmediate(); }};
  auto handle_task_pause = [&](const std::string &task_id, const std::string &request_id,
                               const std::string &reason) {
    const auto result = runtime.pauseTask(task_id, request_id, reason);
    if (result.first) finish_deferred_control_ack({false, "task_pause_pending"});
    sync_goal_diagnostics();
    nav_status.requestImmediate();
    return result;
  };
  auto handle_task_resume = [&](const std::string &task_id, const std::string &request_id,
                                double source_stamp_s) {
    return runtime.resumeTask(task_id, request_id, source_stamp_s,
        task_resume_context(), resume_actions, resume_blocker());
  };
  auto handle_stop = [&](const std::string &) -> std::pair<bool, std::string> {
    runtime.cancelResume();
    const auto interruption = runtime.interruptAndService(
        GoalReplanRuntimeInterruption::kStop, steadySeconds());
    if (interruption.terminal) {
      const auto &terminal_result = *interruption.terminal;
      return {terminal_result.action_committed || motion_stop.confirmationPending(),
              motion_stop.confirmationPending() ? "stop_requested" : terminal_result.reason};
    }
    const auto physical_stop = motion_stop.stopWithoutTerminalCommit();
    return {physical_stop.accepted || physical_stop.pending,
            physical_stop.pending ? "stop_requested" : physical_stop.reason};
  };
  auto handle_estop = [&](const std::string &reason) -> std::pair<bool, std::string> {
    runtime.cancelResume();
    const auto interruption = runtime.interruptAndService(
        GoalReplanRuntimeInterruption::kEstop, steadySeconds(), reason);
    if (interruption.terminal) {
      const auto &terminal_result = *interruption.terminal;
      return {terminal_result.action_committed || motion_stop.confirmationPending(),
              motion_stop.confirmationPending() ? "estop_requested" : terminal_result.reason};
    }
    const auto physical_estop = motion_stop.estopWithoutTerminalCommit(reason);
    return {physical_estop.accepted || physical_estop.pending, physical_estop.pending ? "estop_requested" : physical_estop.reason};
  };
  auto handle_clear_estop = [&](double source_stamp_s) -> std::pair<bool, std::string> {
    const std::string stamp_error =
        sourceStampError("clear_estop", source_stamp_s, nowSeconds(),
                         cfg.teleop_cmd_max_age_s, cfg.input_future_tolerance_s);
    const auto result = motion_stop.clearEstop(stamp_error);
    return {result.accepted || result.pending,
            result.pending ? "clear_estop_requested" : result.reason};
  };
  auto handle_resume_motion = [&](double source_stamp_s) -> std::pair<bool, std::string> {
    if (operator_authority.snapshot().has_active_authority) {
      return {false, "operator_authority_active"};
    }
    const bool autonomy = cfg.control_mode == ControlMode::Autonomy;
    ResumeRequest request;
    request.held = control_authority.held();
    request.source_stamp_s = source_stamp_s;
    request.precondition_error = !control_authority.motionAllowed()
        ? "estop_latched"
        : sourceStampError("resume_autonomy", source_stamp_s, nowSeconds(),
                           cfg.teleop_cmd_max_age_s, cfg.input_future_tolerance_s);
    auto result = autonomy ? motion_stop.resumeAutonomy(request) : motion_stop.resumeTeleop(request);
    return {result.accepted || result.pending,
            result.pending ? "resume_requested" : result.reason};
  };
  auto handle_teleop = [&](const OperatorMotionInputSample &sample,
                           bool manual_mode) -> std::pair<bool, std::string> {
    ++teleop_cmd_count;
    const auto sample_receive_time = SteadyClock::now();
    TeleopAdmissionRequest request;
    const auto decoded = parseMotion(sample);
    if (!decoded.ok()) {
      request.decode_error = decoded.error;
    } else {
      request.command = decoded.value;
      const double explicit_source_stamp_s = static_cast<double>(sample.source_stamp_ns) * 1e-9;
      request.source_stamp_s = explicit_source_stamp_s > 0.0 &&
          explicit_source_stamp_s < static_cast<double>(std::numeric_limits<std::int32_t>::max())
          ? explicit_source_stamp_s : sample.stamp_s;
    }
    const auto admission = admitTeleop(request);
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
    if (admission.update_receive_timestamp &&
        (request.command.vx != 0.0 || request.command.vy != 0.0 || request.command.wz != 0.0)) {
      motion_stop.cancelPendingResume();
      runtime.cancelResume();
      if (deferred_control_ack &&
          deferred_control_ack->raw_kind == static_cast<int>(CommandKind::ResumeAutonomy)) {
        finish_deferred_control_ack({false, "takeover_active"});
      }
    }
    if (admission.update_receive_timestamp) {
      // Preserve time already spent in transport and admission. Subsequent
      // checks use the steady clock, so wall-clock adjustments cannot renew it.
      state.teleop_freshness.record(sample_receive_time,
                                   nowSeconds() - request.source_stamp_s,
                                   static_cast<double>(sample.freshness_budget_ms) * 1e-3);
    }
    if (admission.accepted) {
      control_authority.setTeleop(request.command);
      control_authority.setTeleopManualMode(manual_mode);
    }
    return {admission.accepted, admission.reason};
  };
  auto complete_operator_zero_barrier =
      [&](const std::string &reason) -> std::pair<OperatorMotionReceipt, std::uint64_t> {
    const bool owns_motion = cfg.control_mode != ControlMode::Autonomy ||
                             control_authority.operatorHeld();
    if (owns_motion && cfg.control_mode == ControlMode::Autonomy) {
      control_authority.holdOperator();
    } else if (owns_motion && !control_authority.held()) {
      control_authority.stop();
    }
    const auto output_before = dds_status.final_output_sequence;
    const bool cleared = owns_motion ? motion_stop.holdEndpointMotion(reason)
                                      : motion_stop.keepZeroFresh();
    const auto output_after = dds_status.final_output_sequence;
    const auto &final_command = dds_status.final_output_command;
    const bool old_motion_cleared = !control_authority.teleopRequest().has_value() &&
                                    (!owns_motion || !control_authority.pathActive());
    const bool final_is_zero = final_command.vx == 0.0 && final_command.vy == 0.0 && final_command.wz == 0.0;
    const bool zero_published = cfg.publish_cmd_vel && cleared && old_motion_cleared &&
                                output_after > output_before && final_is_zero;
    auto completion = operator_authority.completeZeroBarrier(zero_published);
    const auto final_output_sequence = updateOperatorMotionOutputEvidence(
        completion, operator_authority.snapshot(), output_after, final_command,
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
    operator_transport.last_ack = OperatorMotionAckDiagnostics{true,
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
      ++operator_transport.ack_sent;
    } else {
      ++operator_transport.ack_publish_failed;
      frames.last_error = "operator_motion_ack_publish_failed";
      nav_status.requestImmediate();
    }
  };
  auto rolling_segment_context = [&]() {
    RollingSegmentRuntimeContext context;
    context.now_s = nowSeconds();
    context.input_ready = input_gate_state.ready;
    context.autonomy_mode = cfg.control_mode == ControlMode::Autonomy;
    context.motion_allowed = control_authority.motionAllowed();
    context.operator_takeover_latched =
        control_authority.held();
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
      inspection_writer.submit(inspection_status);
    }
    const auto operator_motion_snapshot = operator_authority.snapshot();
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
    operator_transport.status =
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
      ++operator_transport.status_sent;
    } else {
      ++operator_transport.status_publish_failed;
      frames.last_error = "operator_motion_status_publish_failed";
      nav_status.requestImmediate();
    }
    if (now - last_navigation_map_identity_refresh_s >= 1.0) {
      last_navigation_map_identity_refresh_s = now;
      const auto identity = map_identity();
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
               control_authority.operatorHeld()) {
      navigation_authority = "operator";
    } else if (control_authority.pathActive()) {
      navigation_authority = "autonomy";
    }
    const auto nav_state_write_start = SteadyClock::now();
    const auto navigation_sample = nav_state.sample(NavigationStateContext{
        navigation_map_identity, control_authority.pathActive(), input_gate_state.ready,
        input_gate_state.reason, control_authority.estopLatched(), control_authority.estopReason(),
        control_authority.operatorHeld(), last_local.recovery_state != 0,
        navigation_authority, control_authority.holdReason()});
    (void)nav_state.publishIfDue(navigation_sample, steadySeconds(), [&](const auto &sample) {
      return dds.publish(OutputEvent{sample}).published;
    });
    timing.dds_write_ms += elapsedMs(nav_state_write_start);
    auto status_runtime_state = statusRuntimeStateFromEndpoint(state);
    status_runtime_state.final_output.producer_boot_id = dds_status.producer_boot_id;
    status_runtime_state.final_output.output_sequence = dds_status.final_output_sequence;
    (void)nav_status.publishIfDue(status_runtime_state, command_ingress.diagnostics(), last_timing,
                                  timing);
  };

  auto flush_inspection_events = [&]() {
    auto record_result = lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted;
    std::string checkpoint_error;
    (void)inspection_executor.FlushTaskEvents(
        [&](const lingtu::nav::inspection::TaskEvent &event) {
          if (checkpoint_store == nullptr) {
            checkpoint_error = "store_unavailable";
            return false;
          }
          const auto checkpoint = checkpoint_store->PutTaskEventCheckpoint(event);
          if (!checkpoint.ok) {
            checkpoint_error = checkpoint.reason;
            return false;
          }
          record_result = inspection_outbox.Record(event);
          if (record_result ==
              lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted) {
            nav_state.observeInspection(event.status);
          }
          return record_result ==
                 lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted;
        });
    (void)inspection_outbox.Flush();
    const auto diagnostics = inspection_outbox.diagnostics();
    if (!checkpoint_error.empty()) {
      frames.last_error = "inspection_task_event_checkpoint_" + checkpoint_error;
      nav_status.requestImmediate();
    } else if (record_result !=
               lingtu::nav::inspection::TaskEventOutboxRecordResult::kAccepted) {
      frames.last_error =
          std::string("inspection_task_event_") +
          lingtu::nav::inspection::TaskEventOutboxRecordResultName(record_result);
      nav_status.requestImmediate();
    } else if (diagnostics.pending != 0U && diagnostics.delivery_failures != 0U) {
      frames.last_error = "inspection_task_event_delivery_pending";
      nav_status.requestImmediate();
    }
  };
  std::function<void()> process_shutdown_commands;
  auto shutdown = [&]() -> int {
    if (rolling_segment.snapshot().active) {
      (void)rolling_effects.apply(
          rolling_segment.step(RollingSegmentShutdown{}));
    }
    constexpr int kSegmentStatusRetryLimit = 3;
    for (int attempt = 0;
         attempt < kSegmentStatusRetryLimit && rolling_segment.snapshot().terminal_delivery_pending;
         ++attempt) {
      (void)rolling_effects.apply(
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
      auto batch = dds.takeSensors(steadySeconds());
      if (ctx.observe_stop_inputs) ctx.observe_stop_inputs(batch);
      TimingDiagnostics shutdown_timing;
      inputs.apply(std::move(batch), shutdown_timing);
      if (process_shutdown_commands) process_shutdown_commands();
      const ShutdownTransactionResult transaction = advanceShutdownTransaction(
          replan, motion_stop, terminal_delivery, steadySeconds());
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
                     runtime.terminalPending() ? 1 : 0,
                     transaction.delivery_acknowledged ? 1 : 0,
                     static_cast<int>(transaction.terminal_flush));
        next_pending_log = log_now + kPendingLogInterval;
      }
      (void)motion_stop.keepZeroFresh();
      std::this_thread::sleep_for(tick_period);
    }
  };
  auto drain_sensors = [&](TimingDiagnostics &timing) {
    const auto take_start = SteadyClock::now();
    auto batch = dds.takeSensors(steadySeconds());
    timing.sensor_take_ms += elapsedMs(take_start);
    const auto apply_start = SteadyClock::now();
    if (ctx.observe_stop_inputs) ctx.observe_stop_inputs(batch);
    inputs.apply(std::move(batch), timing);
    timing.sensor_apply_ms += elapsedMs(apply_start);
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
      if (external_selected && action == OperatorMotionAction::Claim) {
        write_operator_motion_ack(message, {false, "external_control_selected", false, false}, 0U);
        return;
      }
      if (pending_posture && action == OperatorMotionAction::Claim) {
        write_operator_motion_ack(message, {false, "posture_pending", false, false}, 0U);
        return;
      }
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
      } else if (action == OperatorMotionAction::Claim && !operator_enabled) {
        receipt = {false,
                   std::string("operator_motion_not_allowed_in_") +
                       controlModeName(cfg.control_mode),
                   false, false};
      } else if (action == OperatorMotionAction::Claim) {
        receipt = operator_authority.claim(OperatorMotionClaim{
            message.source_id, message.source_epoch, message.source_sequence,
            steadySeconds(), static_cast<double>(message.lease_ttl_ms) * 1e-3});
        if (receipt.accepted && receipt.authority_changed) {
          operator_motion_admitted_sequence = 0U;
          operator_motion_final_output_sequence = 0U;

        }
      } else {
        const OperatorMotionRelease control{message.source_id, message.source_epoch,
                                            message.source_sequence};
        receipt = action == OperatorMotionAction::Release
                      ? operator_authority.release(control)
                      : operator_authority.hold(control);
      }
      const bool accepted_zero_action =
          receipt.accepted &&
          (action == OperatorMotionAction::Release || action == OperatorMotionAction::Hold);
      if (operator_authority.snapshot().zero_barrier_pending) {
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
      if (pending_posture || external_selected) return;
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
      const auto receipt = operator_authority.submit(sample, steadySeconds(), nowSeconds());
      if (!receipt.accepted) {
        frames.last_error = "operator_motion_sample_" + receipt.reason;
        return;
      }
      const auto authority_snapshot = operator_authority.snapshot();
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
    auto operator_snapshot = operator_authority.snapshot();
    if (!operator_snapshot.zero_barrier_pending) {
      (void)operator_authority.tick(steadySeconds());
      operator_snapshot = operator_authority.snapshot();
    }
    if (operator_snapshot.zero_barrier_pending) {
      (void)complete_operator_zero_barrier("operator_motion_lease_expired");
    }
  };
  auto drain_commands = [&]() {
    if (ctx.semantic_view_query) {
      if (auto result = ctx.semantic_view_query->poll()) {
        const auto identity = map_identity();
        if (!map_body || !input_gate_state.ready ||
            result->frame_epoch != frame_epoch || !identity.identity ||
            !plan::sameMapIdentity(result->map, *identity.identity)) {
          result->available = false;
          result->geometry_exhausted = false;
          result->candidates.clear();
          result->reason = "semantic_search_context_changed";
        }
        if (!dds.publish(OutputEvent{std::move(*result)}).published) {
          frames.last_error = "semantic_view_result_publish_failed";
          nav_status.requestImmediate();
        }
      }
    }
    if (auto completion = plan_preview.poll()) {
      if (pending_preview && pending_preview->request_id == completion->context.request_id) {
        std::vector<nav_kernel::Vec3> path;
        std::string reason;
        if (!completion->error.empty()) {
          reason = "planner_error";
        } else if (completion->result.cancelled) {
          reason = completion->timed_out ? "preview_timeout" : "preview_cancelled";
        } else {
          const auto map = map_identity();
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
      if (pending_posture &&
          pending_posture->ingress.client_id == ingress_request.client_id &&
          pending_posture->ingress.request_id == ingress_request.request_id &&
          pending_posture->ingress.raw_kind == ingress_request.raw_kind &&
          pending_posture->ingress.payload.canonical() == ingress_request.payload.canonical()) return;
      if (deferred_control_ack && deferred_control_ack->client_id == ingress_request.client_id &&
          deferred_control_ack->request_id == ingress_request.request_id &&
          deferred_control_ack->raw_kind == ingress_request.raw_kind &&
          deferred_control_ack->payload.canonical() == ingress_request.payload.canonical()) return;
      if (pending_planner && pending_planner->ingress.client_id == ingress_request.client_id &&
          pending_planner->ingress.request_id == ingress_request.request_id &&
          pending_planner->ingress.raw_kind == ingress_request.raw_kind &&
          pending_planner->ingress.payload.canonical() == ingress_request.payload.canonical()) return;
      const auto ingress_result = command_ingress.handle(
          ingress_request, [&](CommandKind kind, const CommandPayload &payload) -> CommandAck {
            if (kind == CommandKind::Stop || kind == CommandKind::Estop) {
              if (kind == CommandKind::Estop || payload.reason != "client_clock_sync")
                finish_planner(false, "planner_switch_cancelled");
              cancel_posture("posture_cancel_requested");
              if (external_selected) {
                external_stop_request = lingtu::message::PostureRequest{
                    dds_status.producer_boot_id, ingress_request.client_id,
                    "external-stop-" + ingress_request.request_id,
                    static_cast<int>(CommandKind::DisableMotors), 0, {}, {}};
                (void)dds.publish(OutputEvent{*external_stop_request});
                external_stop_retry = SteadyClock::now() + std::chrono::milliseconds(100);
              }
            } else if (pending_posture) {
              return {false, "posture_pending"};
            } else if (pending_planner || planner_selection.busy()) {
              return {false, "planner_switch_pending"};
            }
            if (external_selected && kind != CommandKind::Stop && kind != CommandKind::Estop &&
                kind != CommandKind::DisableMotors && kind != CommandKind::ExitExternal &&
                kind != CommandKind::EnableExternal && kind != CommandKind::ClearEstop &&
                !(kind == CommandKind::EnterExternal && !external_active)) {
              return {false, "external_control_selected"};
            }
            if (external_stop_request && kind == CommandKind::EnableExternal) {
              return {false, "stop_confirmation_pending"};
            }
            if (kind == CommandKind::Stand || kind == CommandKind::Sit ||
                kind == CommandKind::EnableMotors || kind == CommandKind::DisableMotors ||
                kind == CommandKind::SwitchProfile || kind == CommandKind::EnterExternal ||
                kind == CommandKind::ExitExternal || kind == CommandKind::EnableExternal) {
              const bool disabling = kind == CommandKind::DisableMotors || kind == CommandKind::ExitExternal;
              const bool disabled_hold = disabling && !control_authority.motionAllowed();
              const auto stamp_error = sourceStampError("posture", sample.goal.stamp_s,
                  nowSeconds(), cfg.teleop_cmd_max_age_s, cfg.input_future_tolerance_s);
              if (!stamp_error.empty()) return {false, stamp_error};
              if (!disabling && !control_authority.motionAllowed()) return {false, "estop_latched"};
              if (!disabling && (runtime.terminalPending() ||
                  motion_stop.confirmationPending())) return {false, "stop_confirmation_pending"};
              if (!disabled_hold && (goal_plan.snapshot().busy || inspection_executor.active() ||
                  rolling_segment.snapshot().active || control_authority.pathActive())) {
                return {false, "task_active"};
              }
              if (operator_authority.snapshot().has_active_authority) {
                return {false, "operator_authority_active"};
              }
              if (kind == CommandKind::SwitchProfile && payload.profile_name.empty()) {
                return {false, "profile_name_required"};
              }
              hold("posture_requested");

              if (!disabling && !motion_stop.holdEndpointMotion("posture_requested")) {
                return {false, "zero_publish_failed"};
              }
              pending_posture = PendingPosture{ingress_request,
                  {dds_status.producer_boot_id, ingress_request.client_id,
                   ingress_request.request_id, static_cast<int>(kind), 0, payload.profile_name,
                   payload.external_session_id},
                  SteadyClock::now() + std::chrono::seconds(8), SteadyClock::now(), {}, false};
              send_posture();
              return {true, "posture_pending", true};
            }
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
              case CommandKind::TaskSpeedLimit:
                ingress_kind = GoalTerminalIngressKind::kTypedTaskSpeedLimit;
                break;
              case CommandKind::SelectPlanner:
                ingress_kind = GoalTerminalIngressKind::kTypedSelectPlanner;
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
              case CommandKind::Stand:
              case CommandKind::Sit:
              case CommandKind::EnableMotors:
              case CommandKind::DisableMotors:
              case CommandKind::SwitchProfile:
              case CommandKind::EnterExternal:
              case CommandKind::ExitExternal:
              case CommandKind::EnableExternal:
                break;
            }
            const auto current_goal = goal_plan.snapshot();
            const bool owns_cancel_target = kind == CommandKind::TaskCancel &&
                (current_goal.active_task_id == ingress_request.task_id ||
                 current_goal.planning_task_id == ingress_request.task_id ||
                 current_goal.pending_task_id == ingress_request.task_id ||
                 replan.replayPendingTerminal().terminal_task_id == ingress_request.task_id);
            if (owns_cancel_target || kind == CommandKind::Estop ||
                (kind == CommandKind::Stop && payload.reason != "client_clock_sync")) {
              runtime.cancelResume();
              finish_deferred_control_ack({false, kind == CommandKind::Estop ? "estop_latched" :
                                                        kind == CommandKind::TaskCancel ? "cancelled" : "stopped"});
            }
            const auto terminal_verdict = terminal_ingress(ingress_kind);
            if (terminal_verdict.decision == GoalTerminalIngressDecision::kReject) {
              return {false, std::string(terminal_verdict.reason)};
            }
            if (terminal_verdict.decision == GoalTerminalIngressDecision::kSafetyStop) {
              if (kind == CommandKind::TaskCancel &&
                  replan.replayPendingTerminal().terminal_task_id != ingress_request.task_id) {
                return {false, "task_not_active"};
              }
              const auto preserved = runtime.stopWhileTerminalPending();
              return {preserved.accepted || preserved.pending,
                      preserved.pending ? "stop_requested" : preserved.reason};
            }
            if (terminal_verdict.decision == GoalTerminalIngressDecision::kSafetyEstop) {
              const auto preserved =
                  runtime.estopWhileTerminalPending(payload.reason);
              return {preserved.accepted || preserved.pending,
                      preserved.pending ? "estop_requested" : preserved.reason};
            }
            std::pair<bool, std::string> result;
            if (kind == CommandKind::SelectPlanner) {
              if (!current_goal.active_task_id.empty() || current_goal.busy ||
                  current_goal.pending_plan_queued || executor.hasRoute() ||
                  inspection_executor.active() || rolling_segment.snapshot().active ||
                  staged_inspection_goal) return {false, "task_active"};
              if (operator_authority.snapshot().has_active_authority)
                return {false, "operator_authority_active"};
              if (motion_stop.confirmationPending()) return {false, "stop_confirmation_pending"};
              if (payload.local_planner == nav_kernel::localPlannerBackendName(cfg.local_planner_backend))
                return {true, "planner_unchanged"};
              try {
                const auto error = planner_selection.start(cfg, payload.local_planner);
                if (!error.empty()) return {false, error};
              } catch (const std::exception &error) {
                return {false, error.what()};
              }
              const auto stop = motion_stop.stopWithoutTerminalCommit();
              if (!stop.accepted && !stop.pending) return {false, stop.reason};
              pending_planner.emplace(PendingPlanner{ingress_request,
                  motion_stop.confirmationKey(), !stop.pending, std::nullopt});
              return {true, "planner_switch_pending", true};
            } else if (kind == CommandKind::Goal) {
              result = submit_goal(sample.goal, ingress_request.task_id,
                                   ingress_request.request_id,
                                   GoalPlanOrigin::kExternal);
            } else if (kind == CommandKind::TaskCancel) {
              ++cancel_count;
              const CommandAck cancel_result = runtime.cancelTask(
                  GoalTaskCancelRequest{ingress_request.task_id, ingress_request.request_id,
                                        payload.reason, steadySeconds()});
              result = {cancel_result.accepted, cancel_result.reason};
              nav_status.requestImmediate();
            } else if (kind == CommandKind::TaskPause) {
              result = handle_task_pause(ingress_request.task_id, ingress_request.request_id,
                                         payload.reason);
            } else if (kind == CommandKind::TaskResume) {
              result = handle_task_resume(ingress_request.task_id, ingress_request.request_id, sample.goal.stamp_s);
            } else if (kind == CommandKind::TaskSpeedLimit) {
              result = goal_plan.setTaskSpeedLimit(ingress_request.task_id, payload.goal[7],
                                                   cfg.path_follower_max_speed_mps);
              if (result.first) {
                executor.setSpeedLimit(payload.goal[7]);
                nav_status.requestImmediate();
              }
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
            const bool pending_control =
                (kind == CommandKind::ResumeAutonomy || kind == CommandKind::ClearEstop) &&
                motion_stop.confirmationPending() && result.first;
            return {result.first, result.second, pending_control};
          });
      if (ingress_result.ack.pending) {
        if (ingress_result.kind == CommandKind::SelectPlanner) return;
        if (ingress_result.kind == CommandKind::Stand || ingress_result.kind == CommandKind::Sit ||
            ingress_result.kind == CommandKind::EnableMotors ||
            ingress_result.kind == CommandKind::DisableMotors ||
            ingress_result.kind == CommandKind::SwitchProfile ||
            ingress_result.kind == CommandKind::EnterExternal ||
            ingress_result.kind == CommandKind::ExitExternal ||
            ingress_result.kind == CommandKind::EnableExternal) return;
        if (deferred_control_ack &&
            deferred_control_ack->request_id != ingress_request.request_id) {
          finish_deferred_control_ack({false, "stop_confirmation_pending"});
        }
        deferred_control_ack = ingress_request;
        deferred_control_key = motion_stop.confirmationKey();
        return;
      }
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
      if (!std::isfinite(request.acceptance_radius_m) || request.acceptance_radius_m < 0.0) {
        reject("goal_constraints_invalid");
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
      if (active_goal.busy || active_goal.pending_plan_queued) {
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
      if (request.acceptance_radius_m > 0.0) {
        context.request.options.terminal_goal_xy_tolerance_m = std::min(
            context.request.options.terminal_goal_xy_tolerance_m, request.acceptance_radius_m);
      }
      // Keep preview searches shorter than the client's 10-second wait.
      context.timeout = std::chrono::seconds(7);
      if (!plan_preview.start(std::move(context))) {
        reject("planner_busy");
        return;
      }
      pending_preview = PendingPreview{request_id, start, goal};
    };
    auto process_semantic_view_query = [&](const semantic::ViewQuery &request) {
      if (request.request_id.empty()) return;
      const auto identity = map_identity();
      auto reject = [&](const std::string &reason) {
        semantic::ViewResult result;
        result.request_id = request.request_id;
        result.boot_id = dds_status.producer_boot_id;
        result.map = identity.identity.value_or(plan::MapIdentity{});
        result.frame_epoch = frame_epoch;
        result.timestamp_s = nowSeconds();
        result.reason = reason;
        (void)dds.publish(OutputEvent{std::move(result)});
      };
      if (!ctx.semantic_view_query || cfg.global_planner != GlobalPlannerBackend::OctoPlanner3D) {
        reject("semantic_search_backend_unavailable");
        return;
      }
      if (!map_body || !input_gate_state.ready) {
        reject("semantic_search_inputs_not_ready");
        return;
      }
      if (!identity.identity) {
        reject("semantic_search_map_unavailable");
        return;
      }
      const auto active = goal_plan.snapshot();
      if (active.busy || active.pending_plan_queued) {
        reject("navigation_busy");
        return;
      }
      SemanticViewContext context;
      context.query = request;
      context.boot_id = dds_status.producer_boot_id;
      context.map = *identity.identity;
      context.frame_epoch = frame_epoch;
      context.robot = {map_body->position.x, map_body->position.y, map_body->position.z};
      context.yaw = map_body->yaw;
      context.timestamp_s = nowSeconds();
      if (!ctx.semantic_view_query->start(std::move(context))) reject("semantic_search_query_busy");
    };
    auto process_inspection_command = [&](const InspectionCommandRequest &request) {
      if (pending_posture) {
        (void)inspection_commands.reject(request, "posture_pending");
        return;
      }
      const auto verdict = terminal_ingress(GoalTerminalIngressKind::kInspectionCommand);
      if (verdict.decision != GoalTerminalIngressDecision::kAllow) {
        (void)inspection_commands.reject(request, std::string(verdict.reason));
        return;
      }
      (void)inspection_commands.handle(request);
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
      if (const auto *state = std::get_if<lingtu::message::ExternalState>(&event)) {
        if (state->fresh) {
          external_selected = state->selected;
          external_active = state->active;
          if (state->motor_disable_confirmed && !state->motors_enabled) external_stop_request.reset();
        }
      } else if (const auto *ack = std::get_if<lingtu::message::PostureAck>(&event)) {
        if (!pending_posture ||
            !lingtu::message::samePostureRequest(pending_posture->request, ack->request) ||
            pending_posture->request.action != ack->request.action ||
            pending_posture->request.profile_name != ack->request.profile_name ||
            pending_posture->request.external_session_id != ack->request.external_session_id ||
            pending_posture->request.phase != ack->request.phase ||
            pending_posture->waiting_stop) continue;
        if (ack->final_result) {
          finish_posture(ack->accepted, ack->reason);
        } else if (ack->reason == "stop_required" && ack->request.phase == 0) {
          const auto stop = motion_stop.stopWithoutTerminalCommit();
          if (stop.pending) {
            pending_posture->waiting_stop = true;
            pending_posture->stop_key = motion_stop.confirmationKey();
          } else if (stop.accepted) {
            pending_posture->request.phase = 1;
            send_posture();
          } else {
            finish_posture(false, stop.reason);
          }
        }
      } else if (const auto *command = std::get_if<NavigationCommandSample>(&event)) {
        process_navigation_command(*command);
      } else if (const auto *preview = std::get_if<PlanPreviewRequest>(&event)) {
        process_plan_request(*preview);
      } else if (const auto *query = std::get_if<semantic::ViewQuery>(&event)) {
        process_semantic_view_query(*query);
      } else if (const auto *inspection = std::get_if<InspectionCommandRequest>(&event)) {
        process_inspection_command(*inspection);
      } else if (const auto *geofence_command = std::get_if<GeofenceCommandView>(&event)) {
        process_geofence_command(*geofence_command);
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
      if (pending_posture) {
        (void)rolling_effects.apply(rolling_segment.step(
            RollingSegmentIngressRejected{std::move(command), context, "posture_pending"}));
        return;
      }
      const auto verdict = terminal_ingress(GoalTerminalIngressKind::kRollingCommand);
      if (verdict.decision != GoalTerminalIngressDecision::kAllow) {
        (void)rolling_effects.apply(
            rolling_segment.step(RollingSegmentIngressRejected{std::move(command), context,
                                                               std::string(verdict.reason)}));
        return;
      }
      (void)rolling_effects.apply(
          rolling_segment.step(RollingSegmentCommandEvent{std::move(command), context}));
    };
    for (const auto &event : command_batch.ordered) {
      if (const auto *request = std::get_if<RollingSegmentCommand>(&event)) {
        process_rolling_command(*request);
      }
    }
  };

  process_shutdown_commands = [&] {
    command_batch = dds.takeCommands(steadySeconds());
    auto &events = command_batch.ordered;
    events.erase(std::remove_if(events.begin(), events.end(), [](const auto &event) {
      const auto *command = std::get_if<NavigationCommandSample>(&event);
      if (!command) return true;
      const auto kind = static_cast<CommandKind>(command->ingress.raw_kind);
      return kind != CommandKind::Stop && kind != CommandKind::Estop && kind != CommandKind::TaskCancel;
    }), events.end());
    drain_commands();
  };
  auto advance_runtime = [&](const SteadyClock::time_point &input_start,
                             TimingDiagnostics &timing) -> double {
    if (pending_posture || pending_planner || planner_selection.busy() || motion_stop.confirmationPending()) {
      (void)motion_stop.keepZeroFresh();
      if (runtime.terminalPending()) {
        (void)runtime.completeTerminal(replan.replayPendingTerminal());
      }
      return nowSeconds();
    }
    // Keep the inspection phase across a transient localization gap. The
    // navigation admission gate will hold the next motion command until the
    // required inputs recover; a temporary input gap must not rewrite the
    // inspection task state.
    if (staged_inspection_goal &&
        (inspection_executor.status().state != lingtu::nav::inspection::RunState::kPlanning ||
         inspection_executor.status().task_id != staged_inspection_goal->task_id ||
         inspection_executor.status().request_id != staged_inspection_goal->request_id)) {
      staged_inspection_goal.reset();
      inspection_runtime.clearActivePoint();
    }
    if (staged_inspection_goal && !runtime.terminalPending()) {
      const auto verdict = terminal_ingress(GoalTerminalIngressKind::kInspectionGoalDispatch);
      if (verdict.decision == GoalTerminalIngressDecision::kAllow) {
        const auto dispatch_result =
            submit_goal(inspectionGoal(staged_inspection_goal->point, nowSeconds()),
                        staged_inspection_goal->task_id, staged_inspection_goal->request_id,
                        GoalPlanOrigin::kInspection);
        staged_inspection_goal.reset();
        const auto completion = inspection_runtime.completeGoalDispatch(
            dispatch_result.first, dispatch_result.second, nowSeconds());
        if (completion.clear_motion_reason) {
          if (!motion_stop.clearEndpointMotion(*completion.clear_motion_reason)) {
            hold(*completion.clear_motion_reason + std::string(":zero_publish_failed"));
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
    bool path_active_for_tick = false;
    AutonomyTickResult autonomy_result;
    NavigationRuntimeFrameActions runtime_actions;
    runtime_actions.complete_endpoint_work_before_autonomy =
        [&](const GoalReplanRuntimeResult &runtime_advance_result) {
          const auto &plan_advance = runtime_advance_result.plan_advance;
          if (plan_advance.completion_consumed || plan_advance.path_activated) {
            sync_goal_diagnostics();
            if (plan_advance.completion_consumed) {
              timing.global_plan_ms = plan_advance.elapsed_ms;
            }
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
          inspection_tick_input.goal_plan_busy =
              goal_plan.snapshot().busy || runtime.terminalPending();
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
                hold(intent.reason + std::string(":zero_publish_failed"));
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
                hold(*completion.clear_motion_reason + std::string(":zero_publish_failed"));
              }
            }
          }
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
              hold("estop_latched" + std::string(":zero_publish_failed"));
            }
          }
          const auto &active_teleop_request = control_authority.teleopRequest();
          const bool path_active = control_authority.pathActive();
          auto teleop_result = teleop_tick.tick(TeleopTickInput{
              cfg, safety_config, map_body, input_gate_state,
              active_teleop_request ? &*active_teleop_request : nullptr, path_active, obstacle_xyzh,
              traversability_grid, last_traversability_receive_s, last_teleop, timing,
              control_authority.teleopManualMode(), state.teleop_freshness.budget_s});
          if (teleop_result.handled) {
            last_teleop = std::move(teleop_result.teleop);
            if (teleop_result.local) {
              if (teleop_result.publish.local_path) {
                publish_local_path(teleop_result.local_path, timing);
              }
              if (teleop_result.publish.waypoint) {
                const auto waypoint_write_start = SteadyClock::now();
                (void)dds.publish(OutputEvent{WaypointOutput{teleop_result.waypoint}});
                timing.dds_write_ms += elapsedMs(waypoint_write_start);
              }
              last_local = std::move(*teleop_result.local);
              last_local_path = std::move(teleop_result.local_path);
              last_local_planner_debug = std::move(teleop_result.local_planner_debug);
            }
            bool cmd_vel_published = false;
            if (teleop_result.publish.cmd_vel) {
              const auto output_sequence = publish_final_velocity(
                  teleop_result.publish.command, timing, "teleop_cmd_vel_publish_failed");
              cmd_vel_published = output_sequence.has_value();
              if (output_sequence) {
                operator_motion_final_output_sequence = *output_sequence;
              } else {
                operator_motion_final_output_sequence = 0U;
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
            timing.final_control_ms += teleop_result.timing.final_control_ms;
          }
          timing.teleop_gate_ms = teleop_result.timing.teleop_gate_ms;
          if (rolling_segment.snapshot().active) {
            (void)rolling_effects.apply(
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
      autonomy_result = autonomy_tick.tick(
          AutonomyTickInput{safety_config, map_body, input_gate_state, path_active_for_tick,
                            goal_snapshot_for_tick.active_map_identity,
                            control_authority.motionAllowed(),
                            rolling_active, cfg.publish_cmd_vel, traversability_grid, last_local,
                            timing, active_goal_identity});
      GoalReplanRuntimeFrameInput autonomy_frame = runtime_frame;
      autonomy_frame.steady_now_s = steadySeconds();
      autonomy_frame.wall_now_s = nowSeconds();
      autonomy_frame.inspection_active = inspection_executor.active();
      autonomy_frame.rolling_segment_active = rolling_active;
      autonomy_frame.map_drift =
          autonomy_result.outcome.reason == "active_path_map_identity_missing" ||
          autonomy_result.outcome.reason == "active_map_unavailable_during_navigation" ||
          autonomy_result.outcome.reason == "active_map_changed_during_navigation";
      return NavigationRuntimeAutonomyObservation{autonomy_frame, autonomy_result.outcome,
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
        timing.final_control_ms += autonomy_result.timing.final_control_ms;
        if (autonomy_result.output) {
          auto &out = *autonomy_result.output;
          if (autonomy_result.publish.local_path) {
            publish_local_path(out.local_path_map, timing);
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
          cmd_vel_published = publish_final_velocity(
                                  autonomy_result.publish.command, timing,
                                  "autonomy_cmd_vel_publish_failed")
                                  .has_value();
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
            (void)rolling_effects.apply(rolling_segment.step(
                RollingSegmentMotionOutcome{RollingSegmentMotionOutcomeKind::kRecoveryExhausted,
                                            autonomy_result.outcome.reason}));
            break;
          case AutonomyTickOutcomeKind::kRollingReached:
            (void)rolling_effects.apply(
                rolling_segment.step(RollingSegmentMotionOutcome{
                    RollingSegmentMotionOutcomeKind::kReached, autonomy_result.outcome.reason}));
            break;
          case AutonomyTickOutcomeKind::kGoalFailed:
          case AutonomyTickOutcomeKind::kGoalReached:
          case AutonomyTickOutcomeKind::kNone:
            break;
        }
      }
    };
    const std::uint64_t output_before_runtime = dds_status.final_output_sequence;
    const NavigationRuntimeFrameResult runtime_frame_result =
        runtime.advanceFrame(runtime_frame, runtime_actions);
    const bool runtime_requests_zero =
        runtime_frame_result.planning_result.zero_kept_fresh ||
        runtime_frame_result.pending_result.zero_kept_fresh ||
        (runtime_frame_result.autonomy_result &&
         runtime_frame_result.autonomy_result->zero_kept_fresh);
    if (runtime_requests_zero && cfg.publish_cmd_vel &&
        dds_status.final_output_sequence == output_before_runtime) {
      (void)publish_final_velocity({}, timing, "runtime_zero_publish_failed");
    }
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
      sync_goal_diagnostics();
      nav_status.requestImmediate();
    }
    // Idle command expiry belongs to the driver watchdog. Navigation only
    // publishes zero when a controller or explicit stop transaction requests it.
    if (!runtime.terminalPending()) {
      (void)goal_outbox.flush();
    }
    const double publish_now = nowSeconds();
    // Input freshness is a motion hold, not a task transition.  AutonomyTick
    // and the post-planning readiness check already publish a checked zero;
    // keeping the inspection leg in Navigating lets it resume when the next
    // valid odometry/collision/localization frame arrives.
    flush_inspection_events();
    return publish_now;
  };

  while (running) {
    const auto loop_start = SteadyClock::now();
    const auto cpu_start = lingtu::nav::platform::threadCpuNanoseconds();
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
    (void)rolling_effects.apply(
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
    const std::string completing_stop_key = motion_stop.confirmationKey();
    if (deferred_control_ack && completing_stop_key != deferred_control_key) {
      finish_deferred_control_ack({false, "stop_confirmation_pending"});
    }
    if (auto stop_result = motion_stop.poll(); stop_result && !stop_result->pending) {
      if (pending_planner && !pending_planner->stopped) {
        if (completing_stop_key != pending_planner->stop_key || !stop_result->accepted)
          finish_planner(false, stop_result->reason);
        else
          pending_planner->stopped = true;
      }
      if (pending_posture && pending_posture->waiting_stop) {
        if (completing_stop_key != pending_posture->stop_key || !stop_result->accepted) {
          finish_posture(false, stop_result->reason);
        } else {
          pending_posture->waiting_stop = false;
          pending_posture->request.phase = 1;
          send_posture();
        }
      }
      finish_deferred_control_ack({stop_result->accepted, stop_result->reason});
      sync_goal_diagnostics();
      nav_status.requestImmediate();
    }
    try {
      if (auto prepared = planner_selection.poll()) {
        if (pending_planner)
          pending_planner->prepared = std::move(prepared);
        else
          planner_selection.retire(std::move(prepared->executor));
      }
      if (pending_planner && pending_planner->stopped && pending_planner->prepared) {
        ctx.apply_planner(*pending_planner->prepared);
        last_local_path.clear();
        last_local_planner_debug = {};
        last_local_planner_debug.backend = cfg.local_planner_backend;
        input_gate_state = evaluate_input_gate();
        finish_planner(true, "planner_selected");
      }
    } catch (const std::exception &error) {
      finish_planner(false, error.what());
    }
    if (external_stop_request && SteadyClock::now() >= external_stop_retry) {
      (void)dds.publish(OutputEvent{*external_stop_request});
      external_stop_retry = SteadyClock::now() + std::chrono::milliseconds(100);
    }
    if (pending_posture) {
      if (SteadyClock::now() >= pending_posture->deadline) {
        cancel_posture("posture_ack_timeout");
      } else if (!pending_posture->waiting_stop &&
                 SteadyClock::now() >= pending_posture->retry_at) {
        send_posture();
      }
    }
    runtime.retryResume(task_resume_context, resume_actions, resume_blocker);
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
    timing.thread_cpu_ms = static_cast<double>(
        lingtu::nav::platform::threadCpuNanoseconds() - cpu_start) / 1e6;
    (void)loop_health.observe(
        ControlLoopSample{timing.loop_ms, timing.sleep_ms, timing.overrun_ms,
                          {timing.sensor_drain_ms, timing.operator_drain_ms,
                           timing.command_drain_ms, timing.runtime_ms, timing.publish_ms},
                          timing.nav_tick_ms, timing.final_control_ms,
                          timing.sensor_take_ms, timing.sensor_apply_ms, timing.thread_cpu_ms});
    last_timing = timing;
  }

  current_timing = nullptr;
  return shutdown();
}

}  // namespace lingtu::nav::endpoint
