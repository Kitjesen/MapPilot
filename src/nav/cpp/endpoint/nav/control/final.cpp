#include "control/final.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "app/config/config.hpp"
#include "app/state.hpp"
#include "app/time.hpp"
#include "input/planner.hpp"
#include "planning/local/scan/grid.hpp"

namespace lingtu::nav::endpoint {
namespace {

bool isZeroCommand(const nav_kernel::Twist &command) {
  return linearSpeed(command) <= 1e-6 && std::abs(command.wz) <= 1e-6;
}

bool isTeleop(FinalMode mode) {
  return mode == FinalMode::kTeleopCommand || mode == FinalMode::kTeleopPath;
}

CommandSafetyDecision stoppedDecision(const std::string &reason) {
  CommandSafetyDecision decision;
  decision.should_publish = true;
  decision.stopped = true;
  decision.limited = true;
  decision.reason = reason;
  return decision;
}

}  // namespace

CommandSafetyDecision evaluateFinalSafety(const FinalSafetyInput &input,
                                        const CommandSafetyConfig &base_safety,
                                        const nav_kernel::Twist &command) {
  const auto &cfg = input.config;
  const auto &body = input.collision.robot.pose;
  auto safety = base_safety;
  if (cfg.control_mode == ControlMode::Autonomy && input.has_route &&
      !input.operator_takeover) {
    safety.max_speed_mps = std::min(safety.max_speed_mps, input.active_max_speed_mps);
  }
  auto decision = evaluateCommandSafety(safety, command);
  if (decision.stopped || input.planner.backend != nav_kernel::LocalPlannerBackend::Scan ||
      (!input.planner.checkObstacle && !input.collision.environment.no_go_regions) || input.manual_mode ||
      (cfg.control_mode != ControlMode::Autonomy && !cfg.teleop_local_planner)) return decision;
  nav_kernel::local::scan::Grid grid(input.planner, input.collision);
  const double deceleration = std::min(
      cfg.velocity_smoother.x.deceleration, cfg.velocity_smoother.y.deceleration);
  // Preserve the braking horizon independently of command expiry.
  const double reaction = std::max(cfg.teleop_cmd_max_age_s, 1.0 / cfg.tick_hz);
  if (input.has_pose && safety.verified_recovery_translation &&
      grid.inflatedOccupancy(body.position, body.yaw) == 1 &&
      input.measured_velocity_valid &&
      grid.boundaryDepartureMotionFree(body, decision.cmd, input.measured,
          std::min(cfg.recovery_translation_speed_mps, safety.max_speed_mps),
          reaction, deceleration, cfg.velocity_smoother.yaw.deceleration)) {
    decision.reason = "scan_boundary_departure";
    return decision;
  }
  const bool measured_blocked = !input.has_pose ||
      (input.measured_velocity_valid &&
       grid.brakingOccupancy(body, input.measured, reaction, deceleration,
                             cfg.velocity_smoother.yaw.deceleration) != 0);
  const double braking_scale = measured_blocked ? 0.0 :
      grid.brakingScale(body, decision.cmd, reaction, deceleration,
                        cfg.velocity_smoother.yaw.deceleration);
  if (braking_scale <= 0.0) {
    decision.cmd = {};
    decision.stopped = true;
    decision.limited = true;
    decision.reason = measured_blocked ? "brake_blocked"
                                       : "cmd_blocked";
  } else if (braking_scale < 1.0) {
    decision.cmd.vx *= braking_scale;
    decision.cmd.vy *= braking_scale;
    decision.cmd.wz *= braking_scale;
    decision.slowed = true;
    decision.limited = true;
    decision.reason = "cmd_limited";
  }
  return decision;
}

FinalControl::FinalControl(FinalActions actions) : actions_(std::move(actions)) {
  if (!actions_.command_safety || !actions_.shape || !actions_.commit || !actions_.stop) {
    throw std::invalid_argument("final control actions must all be configured");
  }
}

namespace {

FinalActions runtimeActions(const CliConfig &config,
                            const nav_kernel::LocalPlannerParams &planner,
                            const EndpointState &state, navigation::Executor &executor) {
  FinalActions actions;
  actions.command_safety = [&config, &planner, &state, &executor](
      const CommandSafetyConfig &safety, const nav_kernel::Twist &command) {
    nav_kernel::LocalPlanRequest collision;
    collision.body = executor.activeBody();
    collision.observedBody = state.observed_body;
    if (state.map_body) collision.robot.pose = *state.map_body;
    collision.environment.collision = state.local_collision_map.view();
    collision.environment.no_go_regions = state.no_go_regions;
    collision.clock.timestampS = steadySeconds();
    collision.environment.predictions = makePredictionView(
        state.predicted_obstacle_volumes, state.prediction_source_receive_s,
        collision.clock.timestampS, config.cloud_max_age_s);
    return evaluateFinalSafety(
        FinalSafetyInput{config, planner, collision, state.map_body.has_value(),
                         {state.odom_linear_velocity_body.x, state.odom_linear_velocity_body.y,
                          state.odom_yaw_rate},
                         state.odom_velocity_valid, executor.hasRoute(),
                         state.control_authority.operatorHeld(),
                         state.control_authority.teleopManualMode(), executor.activeMaxSpeedMps()},
        safety, command);
  };
  auto smoother = config.velocity_smoother_enabled
                      ? std::make_shared<nav_kernel::VelocitySmoother>(config.velocity_smoother)
                      : nullptr;
  actions.shape = [smoother](const nav_kernel::Twist &target, double timestamp_s) {
    if (!smoother) {
      nav_kernel::VelocitySmootherOutput output;
      output.command = target;
      output.valid = true;
      output.reason = "disabled";
      return output;
    }
    std::string error;
    if (!smoother->SetTarget(target, timestamp_s, &error)) {
      nav_kernel::VelocitySmootherOutput output;
      output.reason = error.empty() ? "velocity_smoother_set_target_failed" : error;
      return output;
    }
    return smoother->Step(timestamp_s);
  };
  actions.commit = [smoother, &executor](const nav_kernel::Twist &applied, double timestamp_s) {
    if (smoother && !smoother->CommitApplied(applied, timestamp_s)) return false;
    executor.commitApplied(applied);
    return true;
  };
  actions.stop = [smoother, &executor](double timestamp_s, const std::string &reason) {
    executor.commitApplied({});
    if (smoother) (void)smoother->Stop(timestamp_s, reason);
  };
  return actions;
}

}  // namespace

FinalControl::FinalControl(const CliConfig &config,
                           const nav_kernel::LocalPlannerParams &planner,
                           const EndpointState &state, navigation::Executor &executor)
    : FinalControl(runtimeActions(config, planner, state, executor)) {}

FinalResult FinalControl::finalize(const FinalInput &input) {
  FinalResult result;
  const auto shaped = actions_.shape(input.candidate, input.now_s);
  if (!shaped.valid || shaped.timed_out) {
    result.reason = shaped.reason.empty() ? "velocity_smoother_invalid" : shaped.reason;
    result.decision = stoppedDecision(result.reason);
    stop(input.now_s, result.reason);
    result.smoother_stopped = true;
    return result;
  }

  if (isZeroCommand(shaped.command)) {
    applied_yaw_rate_ = 0.0;
    applied_yaw_stamp_s_ = input.now_s;
    result.decision.should_publish = true;
    result.decision.reason = "zero";
    return result;
  }

  auto candidate = shaped.command;
  bool yaw_limited = false;
  if (input.mode == FinalMode::kTeleopCommand &&
      input.direct_yaw_accel_rad_s2 > 0.0 &&
      linearSpeed(input.candidate) <= 1e-6 &&
      std::abs(input.candidate.wz) > 1e-6) {
    constexpr double kMaxYawUpdateGapS = 0.2;
    double dt = applied_yaw_stamp_s_ ? input.now_s - *applied_yaw_stamp_s_
                                    : input.nominal_dt_s;
    // A late tick re-anchors time without spending the missed acceleration budget.
    if (!std::isfinite(dt) || dt < 0.0 || dt > kMaxYawUpdateGapS) dt = 0.0;
    const double step = input.direct_yaw_accel_rad_s2 * dt;
    candidate.wz = std::clamp(candidate.wz, applied_yaw_rate_ - step,
                              applied_yaw_rate_ + step);
    yaw_limited = candidate.wz != shaped.command.wz;
  }

  result.decision = actions_.command_safety(input.safety, candidate);
  result.safety_applied = true;

  if (isTeleop(input.mode)) {
    result.decision.limited =
        result.decision.limited || input.limited_before_final || shaped.limited || yaw_limited;
    if (input.mode == FinalMode::kTeleopCommand && !result.decision.stopped &&
        result.decision.reason == "accepted" && input.limited_before_final) {
      result.decision.reason = "limited";
    }
  }
  if (result.decision.stopped) {
    result.reason = result.decision.reason;
    stop(input.now_s, result.reason);
    result.smoother_stopped = true;
  }

  if (input.publish && !isZeroCommand(result.decision.cmd) &&
      !actions_.commit(result.decision.cmd, input.now_s)) {
    result.decision.cmd = {};
    result.decision.stopped = true;
    result.decision.reason = "velocity_smoother_commit_failed";
    if (isTeleop(input.mode)) {
      result.decision.slowed = false;
      result.decision.limited = true;
    }
    result.reason = result.decision.reason;
    stop(input.now_s, result.reason);
    result.smoother_stopped = true;
  }
  if (input.publish && result.decision.should_publish && !result.decision.stopped) {
    applied_yaw_rate_ = result.decision.cmd.wz;
    applied_yaw_stamp_s_ = applied_yaw_stamp_s_
                               ? std::max(*applied_yaw_stamp_s_, input.now_s)
                               : input.now_s;
  }
  return result;
}

void FinalControl::stop(double now_s, const std::string &reason) {
  applied_yaw_rate_ = 0.0;
  // Retain the clock anchor so a repeated tick cannot spend another first step.
  applied_yaw_stamp_s_ = now_s;
  actions_.stop(now_s, reason);
}

}  // namespace lingtu::nav::endpoint
