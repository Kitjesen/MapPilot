#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <exception>
#include <optional>
#include <string>
#include <thread>

#include "body.hpp"
#include "cmd_vel_writer_gate.hpp"
#include "config.hpp"
#include "core.hpp"
#include "dds.hpp"
#include "status.hpp"

namespace {

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double epochSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
struct SequencedOutputIdentity {
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
};

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
    lingtu::driver::RuntimeStats stats;
    stats.capabilities = body->capabilities();

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.poll_hz));
    auto next_tick = Clock::now();
    auto next_probe = next_tick;
    auto next_status = next_tick;
    auto next_control_state = next_tick;
    auto next_lease_refresh = next_tick;
    constexpr auto kControlStatePeriod = std::chrono::milliseconds(50);
    std::uint64_t safety_stop_sequence = 0;
    bool readiness_announced = false;
    bool cmd_vel_writer_fault_active = false;

    auto publish_control_state = [&]() {
      const auto now = Clock::now();
      stats.output_ack.expire(now);
      const auto output_ack = stats.output_ack.current(now);
      const bool published =
          dds.writeControlState(stats.control, output_ack.accepted(), output_ack.producerBootId(),
                                output_ack.outputSequence(), epochSeconds());
      return published;
    };

    auto apply_result = [&](const Result &result) {
      stats.connected = result.transport_ok;
      stats.control = result.state;
      stats.body = body->state();
      stats.health = body->health();
      stats.control.connected = result.transport_ok;
      stats.ready = result.ok && stats.cmd_vel_writer_ready;
      if (!result.transport_ok) {
        stats.control.ready = false;
        stats.control.control_assured = false;
        stats.control.lease_valid = false;
        stats.control.reason = "transport_error";
      }
      if (!stats.cmd_vel_writer_ready) {
        stats.ready = false;
        stats.control.ready = false;
        stats.control.control_assured = false;
        stats.control.lease_valid = false;
        stats.control.reason = stats.cmd_vel_writer_reason;
      }
      stats.last_error = result.error;
    };

    auto publish_stop = [&](const std::string &reason) {
      ++safety_stop_sequence;
      if (dds.writeNavigationStop(reason, safety_stop_sequence, epochSeconds())) {
        ++stats.safety_stops;
      }
    };

    auto stop_and_release = [&](const std::string &success_reason,
                                const std::string &failure_prefix) noexcept {
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

    auto fail_closed = [&](const std::string &reason, bool preserve_current_rejection) {
      stats.last_output_kind = "control_loss_stop";
      const bool stopped = stop_and_release(reason, reason + ":stop_unconfirmed");
      core.reset();
      stats.ready = false;
      if (!preserve_current_rejection) {
        stats.output_ack.invalidate();
      }
      stats.control.ready = false;
      if (stopped && !reason.empty()) {
        stats.control.reason = reason;
      }
      if (readiness_announced) {
        publish_stop(std::string("driver_control_lost:") + stats.control.reason);
      }
      readiness_announced = false;
      (void)publish_control_state();
      next_control_state = Clock::now() + kControlStatePeriod;
    };

    auto send = [&](const Action &action, Clock::time_point now,
                    std::optional<SequencedOutputIdentity> identity) -> bool {
      const Result result = body->move(action.velocity);
      stats.backend_calls += result.calls;
      stats.last_reason = lingtu::driver::reasonName(action.reason);
      apply_result(result);
      const bool has_current_identity =
          action.reason == ActionReason::Command && identity.has_value() &&
          !identity->producer_boot_id.empty() && identity->output_sequence != 0;
      const bool accepted = result.accepted;
      if (has_current_identity && result.transport_ok) {
        stats.output_ack.record(identity->producer_boot_id, identity->output_sequence, accepted,
                                now);
      } else {
        stats.output_ack.invalidate();
      }
      if (!result.transport_ok) {
        ++stats.backend_errors;
        fail_closed("transport_error", false);
        next_probe = now + config.reconnect_delay;
        return false;
      }
      if (!result.accepted || !result.ok) {
        ++stats.command_rejections;
        fail_closed(result.state.reason, has_current_identity && !result.accepted);
        next_probe = now + config.reconnect_delay;
        return false;
      }
      readiness_announced = true;
      stats.last_error.clear();
      stats.last_send_s = epochSeconds();
      stats.last_output_kind = outputKind(action);
      lingtu::driver::recordMotionOutput(
          stats, action.velocity, stats.body, stats.last_output_kind, stats.last_send_s);
      if (action.reason == ActionReason::Command) {
        ++stats.commands_sent;
      } else {
        ++stats.zeros_sent;
      }
      return true;
    };

    auto refresh_control = [&](Clock::time_point now) {
      if (!stats.cmd_vel_writer_ready) {
        return;
      }
      const bool acquiring = !stats.ready;
      if (acquiring) {
        // The acquisition zero proves control readiness, not nav output delivery.
        stats.output_ack.invalidate();
      }
      ++stats.lease_refreshes;
      const Result result = body->refresh();
      stats.backend_calls += result.calls;
      apply_result(result);
      if (!result.transport_ok) {
        ++stats.backend_errors;
        fail_closed("transport_error", false);
        next_probe = now + config.reconnect_delay;
        return;
      }
      if (!result.accepted || !result.ok) {
        ++stats.lease_rejections;
        fail_closed(result.state.reason, false);
        next_probe = now + config.reconnect_delay;
        return;
      }
      readiness_announced = true;
      stats.last_error.clear();
      if (acquiring) {
        stats.last_output_kind = "initial_zero";
      }
      next_lease_refresh = now + kLeaseRefreshPeriod;
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
      stats.ready = false;
      stats.output_ack.invalidate();
      stats.control.ready = false;
      stats.control.reason = decision.reason;
      stats.last_output_kind = "writer_gate_stop";
      (void)stop_and_release(decision.reason, decision.reason + ":stop_unconfirmed");
      stats.output_ack.invalidate();
      core.reset();
      stats.control.ready = false;
      if (readiness_announced) {
        publish_stop(std::string("cmd_vel_writer_gate:") + decision.reason);
      }
      readiness_announced = false;
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
        stats.output_ack.invalidate();
        stats.ready = false;
        stats.control.ready = false;
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

        if ((!stats.ready && now >= next_probe) || (stats.ready && now >= next_lease_refresh)) {
          refresh_control(now);
        }

        const auto read = dds.takeLatest();
        stats.dds_samples += read.valid_samples;
        if (read.latest) {
          stats.last_receive_s = epochSeconds();
          const auto command = core.accept(read.latest->freshnessInput(), read.latest->frame,
                                           read.latest->vx, read.latest->vy, read.latest->wz);
          lingtu::driver::recordFreshnessDecision(stats, command.freshness);
          if (command.action) {
            if (stats.ready) {
              if (send(*command.action, now,
                       SequencedOutputIdentity{read.latest->producer_boot_id,
                                               read.latest->output_seq})) {
                next_control_state = now;
              }
            } else {
              ++stats.dropped_disconnected;
              core.reset();
              stats.output_ack.invalidate();
              next_control_state = now;
            }
          } else {
            stats.output_ack.invalidate();
            next_control_state = now;
          }
        }

        if (stats.ready) {
          if (const auto stop = core.poll(now)) {
            (void)send(*stop, now, std::nullopt);
            stats.output_ack.invalidate();
            next_control_state = now;
          }
        }

        if (now >= next_control_state) {
          (void)publish_control_state();
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
