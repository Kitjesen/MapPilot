#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <exception>
#include <string>
#include <thread>

#include "body.hpp"
#include "cmd_vel_writer_gate.hpp"
#include "config.hpp"
#include "core.hpp"
#include "dds.hpp"
#include "posture.hpp"
#include "message/protocol/navigation.hpp"
#include "status.hpp"

namespace {

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double epochSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

bool isZeroVelocity(const lingtu::driver::Velocity &velocity) noexcept {
  return std::abs(velocity.vx_mps) <= 1e-9 &&
         std::abs(velocity.vy_mps) <= 1e-9 &&
         std::abs(velocity.yaw_rps) <= 1e-9;
}

const char *outputKind(const lingtu::driver::Action &action) noexcept {
  using lingtu::driver::ActionReason;
  switch (action.reason) {
    case ActionReason::Command:
      return isZeroVelocity(action.velocity) ? "commanded_zero" : "motion_command";
    case ActionReason::Watchdog:
      return "watchdog_zero";
    case ActionReason::InvalidFrame:
      return "invalid_frame_zero";
    case ActionReason::NonFinite:
      return "nonfinite_zero";
    case ActionReason::ConnectProbe:
      return "probe_zero";
    case ActionReason::Fault:
      return "fault_stop";
    case ActionReason::Shutdown:
      return "shutdown_stop";
  }
  return "unknown";
}

}  // namespace

int main(int argc, char **argv) {
  using lingtu::driver::Action;
  using lingtu::driver::ActionReason;
  using lingtu::driver::Clock;
  using lingtu::driver::kLeaseRefreshPeriod;
  using lingtu::driver::Result;

  try {
    const lingtu::driver::Config config = lingtu::driver::loadConfig(argc, argv);
    if (config.help) {
      std::fputs(lingtu::driver::usage(), stdout);
      return 0;
    }

    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    // Go2's ChannelFactory must select the robot-facing NIC before another
    // participant is created in the same CycloneDDS runtime.
    auto body = lingtu::driver::makeBody(config);
    lingtu::driver::DdsReader dds(config.domain_id);
    lingtu::driver::CmdVelWriterGate cmd_vel_writer_gate;
    lingtu::driver::Core core(config.limits, dds.hostBootId());
    lingtu::driver::ProfileSwitch profile_switch(config.stairs_up_profile, config.stairs_down_profile);
    lingtu::driver::PostureControl posture_control;
    auto last_zero_command = Clock::time_point{};
    std::string zero_producer;
    lingtu::driver::RuntimeStats stats;
    stats.capabilities = body->capabilities();

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.poll_hz));
    auto next_tick = Clock::now();
    auto next_probe = next_tick;
    auto next_status = next_tick;
    auto next_control_state = next_tick;
    auto next_joint_state = next_tick;
    auto next_external_state = next_tick;
    double last_joint_stamp_s = 0.0;
    constexpr auto kControlStatePeriod = std::chrono::milliseconds(50);
    constexpr auto kJointStatePeriod = std::chrono::milliseconds(34);
    bool cmd_vel_writer_fault_active = false;

    auto publish_control_state = [&]() {
      return dds.writeControlState(stats.control, stats.zero_output, epochSeconds());
    };

    auto apply_result = [&](const Result &result) {
      stats.connected = result.transport_ok && result.state.connected;
      stats.control = result.state;
      stats.body = body->state();
      stats.health = body->health();
      stats.control.connected = stats.connected;
      stats.ready = result.state.ready;
      if (!result.transport_ok) {
        stats.control.ready = false;
        stats.control.control_assured = false;
        stats.control.lease_valid = false;
        if (stats.control.reason.empty()) stats.control.reason = "transport_error";
      }
      stats.last_error = result.error;
    };

    auto stop_and_release = [&](const std::string &success_reason,
                                const std::string &failure_prefix) noexcept {
      stats.zero_output = false;
      const Result result = body->stop();
      stats.backend_calls += result.calls;
      stats.ready = false;
      stats.connected = result.transport_ok;
      stats.control = result.state;
      stats.body = body->state();
      stats.health = body->health();
      stats.control.connected = result.transport_ok;
      stats.control.ready = false;
      const bool acknowledged = result.confirmsStop();
      if (acknowledged) {
        stats.zero_output = true;
        ++stats.zeros_sent;
        lingtu::driver::recordMotionOutput(
            stats, {}, stats.body, stats.last_output_kind, epochSeconds());
        stats.control.control_assured = false;
        stats.control.lease_valid = false;
        stats.control.reason = success_reason;
        stats.last_error.clear();
        return true;
      }
      if (!result.transport_ok) {
        ++stats.backend_errors;
      } else {
        ++stats.command_rejections;
      }
      const std::string detail = !result.state.reason.empty()
                                     ? result.state.reason
                                     : (!result.error.empty() ? result.error : "unknown");
      stats.control.reason = failure_prefix + ":" + detail;
      stats.last_error = !result.error.empty() ? result.error : detail;
      return false;
    };

    auto stop_output = [&](std::string reason,
                           const char *operation, double call_ms) {
      // Preserve the triggering result before attempting zero output.
      std::fprintf(stderr,
                   "lingtu_driver: stop_output operation=%s call_ms=%.1f reason=%s backend_reason=%s "
                   "backend_error=%s body_age_ms=%.1f ready=%d connected=%d last_output=%s\n",
                   operation, call_ms, reason.c_str(), stats.control.reason.c_str(), stats.last_error.c_str(),
                   stats.body.observed_at == Clock::time_point{} ? -1.0 :
                       std::chrono::duration<double, std::milli>(Clock::now() - stats.body.observed_at).count(),
                   stats.ready ? 1 : 0, stats.connected ? 1 : 0,
                   stats.last_output_kind.c_str());
      stats.last_output_kind = "control_loss_stop";
      // Stop this output without releasing the adapter or discarding newer input.
      const auto stopped = body->move({});
      stats.backend_calls += stopped.calls;
      stats.zero_output = stopped.transport_ok && stopped.accepted;
      if (stats.zero_output) {
        ++stats.zeros_sent;
        stats.last_send_s = epochSeconds();
        lingtu::driver::recordMotionOutput(
            stats, {}, body->state(), stats.last_output_kind, stats.last_send_s);
      }
      stats.last_error = reason;
      (void)publish_control_state();
      next_control_state = Clock::now() + kControlStatePeriod;
    };

    auto send = [&](const Action &action, Clock::time_point now) {
      const auto started = Clock::now();
      stats.zero_output = false;
      const Result result = body->move(action.velocity);
      const auto returned = Clock::now();
      const double call_ms = std::chrono::duration<double, std::milli>(returned - started).count();
      stats.backend_calls += result.calls;
      stats.last_reason = lingtu::driver::reasonName(action.reason);
      apply_result(result);
      if (!result.transport_ok) {
        ++stats.backend_errors;
        stop_output(stats.control.reason, isZeroVelocity(action.velocity) ? "zero" : "move", call_ms);
        next_probe = now + config.reconnect_delay;
        return;
      }
      if (!result.accepted) {
        ++stats.command_rejections;
        stop_output(result.state.reason,
                    isZeroVelocity(action.velocity) ? "zero" : "move", call_ms);
        next_probe = now + config.reconnect_delay;
        return;
      }
      stats.last_error.clear();
      stats.zero_output = isZeroVelocity(action.velocity);
      stats.last_send_s = epochSeconds();
      stats.last_output_kind = outputKind(action);
      lingtu::driver::recordMotionOutput(
          stats, action.velocity, stats.body, stats.last_output_kind, stats.last_send_s);
      if (action.reason == ActionReason::Command) {
        ++stats.commands_sent;
      } else {
        ++stats.zeros_sent;
      }
    };

    auto refresh_control = [&](Clock::time_point now) {
      ++stats.lease_refreshes;
      const auto started = Clock::now();
      const Result result = body->refresh();
      const double call_ms = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
      stats.backend_calls += result.calls;
      apply_result(result);
      if (!result.transport_ok) {
        ++stats.backend_errors;
        stop_output(stats.control.reason, "refresh", call_ms);
        next_probe = now + config.reconnect_delay;
        return;
      }
      if (!result.accepted) {
        ++stats.lease_rejections;
        stop_output(result.state.reason, "refresh", call_ms);
        next_probe = now + config.reconnect_delay;
        return;
      }
      stats.last_error.clear();
      next_probe = now + kLeaseRefreshPeriod;
    };

    auto update_cmd_vel_writer_gate = [&]() {
      const auto decision = cmd_vel_writer_gate.update(dds.matchedCommandWriters());
      stats.matched_cmd_vel_writers = decision.matched_writers;
      stats.cmd_vel_writer_ready = decision.ready;
      stats.cmd_vel_writer_reason = decision.reason;
      return decision;
    };

    auto fail_cmd_vel_writer_gate = [&](const lingtu::driver::CmdVelWriterDecision &decision) {
      ++stats.cmd_vel_writer_faults;
      stats.last_output_kind = "writer_gate_stop";
      (void)stop_and_release(decision.reason, decision.reason + ":stop_unconfirmed");

      core.reset();
      cmd_vel_writer_fault_active = true;
      (void)publish_control_state();
      next_control_state = Clock::now() + kControlStatePeriod;
      next_probe = Clock::now() + config.reconnect_delay;
    };

    auto best_effort_stop = [&](ActionReason reason) noexcept {
      stats.last_reason = lingtu::driver::reasonName(reason);
      stats.last_output_kind =
          reason == ActionReason::Shutdown ? "shutdown_stop" : "fault_stop";
      try {
        (void)core.forceStop(reason);
        (void)stop_and_release("stop_confirmed", "stop_unconfirmed");

        (void)publish_control_state();
        next_control_state = Clock::now() + kControlStatePeriod;
      } catch (const std::exception &exc) {
        stats.connected = false;
        stats.last_error = exc.what();
      } catch (...) {
        stats.connected = false;
        stats.last_error = "unknown stop error";
      }
      try {
        lingtu::driver::writeStatus(config, core, stats, body->diagnostics(), epochSeconds());
      } catch (...) {}
    };

    const auto adapter = body->diagnostics();
    std::fprintf(stderr,
                 "lingtu_driver: domain=%d topic=rt/nav/cmd_vel robot=%s target=%s poll=%.1fHz "
                 "watchdog=%lldms lease_refresh=%lldms\n",
                 config.domain_id, adapter.name.c_str(), adapter.target.c_str(), config.poll_hz,
                 static_cast<long long>(config.limits.command_timeout.count()),
                 static_cast<long long>(kLeaseRefreshPeriod.count()));

    try {
      while (g_running) {
        const auto now = Clock::now();
        if (now >= next_joint_state) {
          next_joint_state = now + kJointStatePeriod;
          const auto joints = body->jointState();
          if (joints && joints->stamp_s != last_joint_stamp_s && dds.writeJointState(*joints)) {
            last_joint_stamp_s = joints->stamp_s;
          }
        }
        const auto writer_decision = update_cmd_vel_writer_gate();
        if (!writer_decision.ready) {
          if (!cmd_vel_writer_fault_active || writer_decision.requires_stop) {
            fail_cmd_vel_writer_gate(writer_decision);
          }
          if (now >= next_status) {
            lingtu::driver::writeStatus(config, core, stats, body->diagnostics(), epochSeconds());
            next_status = now + std::chrono::seconds(1);
          }
          next_tick += period;
          if (next_tick < Clock::now()) {
            next_tick = Clock::now();
          }
          std::this_thread::sleep_until(next_tick);
          continue;
        }
        cmd_vel_writer_fault_active = false;

        auto receive_command = [&]() {
          auto latest = dds.takeLatest();
          stats.dds_samples += latest.valid_samples;
          if (!latest.latest) return;
          stats.last_receive_s = epochSeconds();
          const auto &sample = *latest.latest;
          const auto command = core.accept(sample.freshnessInput(), "body",
                                           sample.vx, sample.vy, sample.wz, sample.received_at);
          lingtu::driver::recordFreshnessDecision(stats, command.freshness);
          if (!command.freshness.accepted) return;
          if (command.action &&
              command.action->reason == ActionReason::Command &&
              isZeroVelocity(command.action->velocity)) {
            last_zero_command = sample.received_at;
            zero_producer = sample.producer_boot_id;
          } else {
            last_zero_command = Clock::time_point{};
          }
        };
        receive_command();
        if (const auto request = dds.takeProfileRequest(epochSeconds())) {
          if (!posture_control.holdsMotion()) profile_switch.request(*request, now);
        }
        if (const auto request = dds.takePostureRequest()) {
          const auto reply = posture_control.request(*request, *body, zero_producer,
              last_zero_command != Clock::time_point{} &&
                  now - last_zero_command <= config.limits.command_timeout,
              profile_switch.holdsMotion() && !profile_switch.needsCommand());
          const bool disable = request->action ==
              static_cast<int>(lingtu::message::NavigationCommandKind::DisableMotors);
          if (disable && reply.result && reply.result->calls) {
            profile_switch.cancel("operator_motor_disable");
          } else if (reply.new_request && !disable &&
                     (!reply.ack.final_result || reply.ack.accepted)) {
            profile_switch.resetBlocked();
          }
          if (reply.result && reply.result->calls) {
            stats.backend_calls += reply.result->calls;
            apply_result(*reply.result);
            stats.ready = reply.result->state.ready;
          }
          (void)dds.writePostureAck(reply.ack, epochSeconds());
          next_control_state = now;
        }
        const bool held_before = profile_switch.holdsMotion();
        if (!body->externalSelected()) {
          profile_switch.step(*body, now, last_zero_command != Clock::time_point{} &&
                              now - last_zero_command <= config.limits.command_timeout);
        }
        const bool profile_hold = profile_switch.holdsMotion();
        const auto external = dds.takeExternal();
        if (body->externalSelected()) {
          const auto selected_session = body->externalState().session_id;
          if (external.heartbeat_session == selected_session) {
            const auto result = body->externalHeartbeat();
            stats.backend_calls += result.calls;
            apply_result(result);
          }
          if (external.frame && external.frame->session_id == selected_session) {
            const auto result = body->externalFrame(*external.frame);
            stats.backend_calls += result.calls;
          }
          const auto state = body->externalState();
          stats.ready = false;
          stats.control.ready = false;
          stats.control.control_assured = false;
          stats.control.lease_valid = false;
          stats.control.motors_enabled = state.motors_enabled;
          stats.control.owner = "external";
          stats.control.reason = state.reason;
          stats.zero_output = state.fresh && !state.motors_enabled && state.motor_disable_confirmed;
          core.reset();
        } else if (posture_control.holdsMotion()) {
          if (now >= next_probe) {
            const auto inspection = body->inspectControl();
            stats.backend_calls += inspection.calls;
            if (inspection.calls) apply_result(inspection);
            posture_control.observeBody(body->state());
            next_probe = now + std::chrono::milliseconds(100);
          }
          stats.ready = false;
          stats.control.ready = false;
          if (const auto action = core.take(Clock::now()); action && isZeroVelocity(action->velocity)) {
            (void)stop_and_release("posture_hold_stop", "posture_hold_stop_unconfirmed");
          }
          posture_control.observeBody(body->state());
          core.reset();
        } else if (profile_hold) {
          if (!held_before) {
            stats.zero_output = false;
            stats.zero_output = body->stop().confirmsStop();
          }
          // A planned body/profile handoff has no velocity output and does not
          // acquire a walking lease while the platform is Grounded.
          stats.ready = false;
          stats.control.ready = false;
          stats.control.control_assured = false;
          stats.control.lease_valid = false;
          stats.control.reason = "profile_transition";
          stats.body = body->state();

          core.reset();
        } else {
          if (held_before) next_probe = now;
          if (now >= next_probe) {
            refresh_control(now);
          }
          // SDK maintenance may block; use the newest command and recheck expiry afterward.
          receive_command();
          const auto send_time = Clock::now();
          if (const auto action = core.take(send_time)) {
            send(*action, send_time);
            next_control_state = send_time;
          }
        }

        if (now >= next_external_state) {
          (void)dds.writeExternalState(body->externalState(), epochSeconds());
          next_external_state = now + std::chrono::milliseconds(20);
        }
        if (now >= next_control_state) {
          (void)publish_control_state();
          (void)dds.writeProfileState(profile_switch.status(), epochSeconds());
          next_control_state = now + kControlStatePeriod;
        }
        if (now >= next_status) {
          lingtu::driver::writeStatus(config, core, stats, body->diagnostics(), epochSeconds());
          next_status = now + std::chrono::seconds(1);
        }

        next_tick += period;
        if (next_tick < Clock::now()) {
          next_tick = Clock::now();
        }
        std::this_thread::sleep_until(next_tick);
      }
    } catch (...) {
      best_effort_stop(ActionReason::Fault);
      throw;
    }

    best_effort_stop(ActionReason::Shutdown);
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "lingtu_driver: fatal: %s\n", exc.what());
    return 1;
  }
}
