#include "brainstem.hpp"
#include "cmd_vel_writer_gate.hpp"
#include "config.hpp"
#include "core.hpp"
#include "dds.hpp"
#include "status.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <optional>
#include <string>
#include <thread>

namespace {

std::atomic_bool g_running{true};

void stopSignal(int) {
  g_running = false;
}

double epochSeconds() {
  return std::chrono::duration<double>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
struct SequencedOutputIdentity {
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
};


}  // namespace

int main(int argc, char** argv) {
  using lingtu::driver::Action;
  using lingtu::driver::ActionReason;
  using lingtu::driver::Clock;
  using lingtu::driver::RpcResult;

  try {
    const lingtu::driver::Config config = lingtu::driver::loadConfig(argc, argv);
    if (config.help) {
      std::fputs(lingtu::driver::usage(), stdout);
      return 0;
    }

    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);

    lingtu::driver::DdsReader dds(config.domain_id);
    lingtu::driver::CmdVelWriterGate cmd_vel_writer_gate;
    lingtu::driver::Core core(config.limits, dds.hostBootId());
    lingtu::driver::Brainstem brainstem(
        config.host, config.port, config.rpc_timeout, config.brainstem_tls);
    lingtu::driver::RuntimeStats stats;

    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(1.0 / config.poll_hz));
    constexpr auto kLeaseRefreshPeriod = std::chrono::milliseconds(100);
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
      lingtu::driver::expireCurrentOutputEvidence(stats);
      const auto output_ack = lingtu::driver::currentOutputAck(stats);
      const bool published = dds.writeControlState(
          stats.control,
          output_ack.accepted,
          output_ack.producer_boot_id,
          output_ack.output_sequence,
          epochSeconds());
      return published;
    };

    auto apply_result = [&](const RpcResult& result) {
      stats.connected = result.transport_ok;
      stats.control = result.state;
      stats.control.connected = result.transport_ok;
      stats.ready = result.ok && stats.cmd_vel_writer_ready;
      if (!result.transport_ok) {
        stats.control.ready = false;
        stats.control.lease_valid = false;
        stats.control.reason = "transport_error";
      }
      if (!stats.cmd_vel_writer_ready) {
        stats.ready = false;
        stats.control.ready = false;
        stats.control.lease_valid = false;
        stats.control.reason = stats.cmd_vel_writer_reason;
      }
      stats.last_error = result.error;
    };

    auto publish_stop = [&](const std::string& reason) {
      ++safety_stop_sequence;
      if (dds.writeNavigationStop(
              reason, safety_stop_sequence, epochSeconds())) {
        ++stats.safety_stops;
      }
    };

    auto fail_closed = [&](
                           const std::string& reason,
                           bool preserve_current_rejection) {
      core.reset();
      stats.ready = false;
      if (!preserve_current_rejection) {
        lingtu::driver::invalidateCurrentOutputAck(stats);
      }
      stats.control.ready = false;
      if (!reason.empty()) {
        stats.control.reason = reason;
      }
      if (readiness_announced) {
        publish_stop(
            std::string("brainstem_control_lost:") + stats.control.reason);
      }
      readiness_announced = false;
      (void)publish_control_state();
      next_control_state = Clock::now() + kControlStatePeriod;
    };

    auto send = [&](
                    const Action& action,
                    Clock::time_point now,
                    std::optional<SequencedOutputIdentity> identity) -> bool {
      const RpcResult result = brainstem.send(action.walk);
      stats.rpc_attempts += result.rpc_calls;
      stats.last_reason = lingtu::driver::reasonName(action.reason);
      apply_result(result);
      const bool has_current_identity =
          action.reason == ActionReason::Command && identity.has_value() &&
          !identity->producer_boot_id.empty() &&
          identity->output_sequence != 0;
      const bool accepted = result.accepted;
      if (has_current_identity && result.transport_ok) {
        lingtu::driver::recordCurrentOutputResult(
            stats,
            identity->producer_boot_id,
            identity->output_sequence,
            accepted,
            now);
      } else {
        lingtu::driver::invalidateCurrentOutputAck(stats);
      }
      if (!result.transport_ok) {
        ++stats.rpc_errors;
        fail_closed("transport_error", false);
        next_probe = now + config.reconnect_delay;
        return false;
      }
      if (!result.accepted || !result.ok) {
        ++stats.command_rejections;
        fail_closed(
            result.state.reason, has_current_identity && !result.accepted);
        next_probe = now + config.reconnect_delay;
        return false;
      }
      readiness_announced = true;
      stats.last_error.clear();
      stats.last_send_s = epochSeconds();
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
      if (!stats.ready) {
        // The acquisition zero proves control readiness, not nav output delivery.
        lingtu::driver::invalidateCurrentOutputAck(stats);
      }
      ++stats.lease_refreshes;
      const RpcResult result = brainstem.refreshControl();
      stats.rpc_attempts += result.rpc_calls;
      apply_result(result);
      if (!result.transport_ok) {
        ++stats.rpc_errors;
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
      next_lease_refresh = now + kLeaseRefreshPeriod;
    };

    auto update_cmd_vel_writer_gate = [&]() {
      const auto decision =
          cmd_vel_writer_gate.update(dds.matchedCommandWriters());
      stats.matched_cmd_vel_writers = decision.matched_writers;
      stats.cmd_vel_writer_ready = decision.ready;
      stats.cmd_vel_writer_reason = decision.reason;
      return decision;
    };

    auto fail_cmd_vel_writer_gate = [&](
                                        const lingtu::driver::
                                            CmdVelWriterDecision& decision,
                                        Clock::time_point now) {
      ++stats.cmd_vel_writer_faults;
      stats.ready = false;
      lingtu::driver::invalidateCurrentOutputAck(stats);
      stats.control.ready = false;
      stats.control.reason = decision.reason;
      if (decision.requires_stop && stats.control.lease_valid) {
        (void)send(core.forceStop(ActionReason::Fault), now, std::nullopt);
      }
      brainstem.release();
      lingtu::driver::invalidateCurrentOutputAck(stats);
      core.reset();
      stats.control.ready = false;
      stats.control.lease_valid = false;
      stats.control.reason = decision.reason;
      if (readiness_announced) {
        publish_stop(std::string("cmd_vel_writer_gate:") + decision.reason);
      }
      readiness_announced = false;
      cmd_vel_writer_fault_active = true;
      (void)publish_control_state();
      next_control_state = Clock::now() + kControlStatePeriod;
      next_probe = now + config.reconnect_delay;
    };

    auto best_effort_stop = [&](ActionReason reason) noexcept {
      stats.last_reason = lingtu::driver::reasonName(reason);
      try {
        if (stats.ready) {
          (void)send(core.forceStop(reason), Clock::now(), std::nullopt);
        }
        brainstem.release();
        lingtu::driver::invalidateCurrentOutputAck(stats);
        stats.ready = false;
        stats.control.ready = false;
        stats.control.lease_valid = false;
        stats.control.reason = "released";
        (void)publish_control_state();
        next_control_state = Clock::now() + kControlStatePeriod;
      } catch (const std::exception& exc) {
        stats.connected = false;
        stats.last_error = exc.what();
      } catch (...) {
        stats.connected = false;
        stats.last_error = "unknown stop error";
      }
      try {
        lingtu::driver::writeStatus(
            config, core, stats, brainstem.target(), epochSeconds());
      } catch (...) {
      }
    };

    std::fprintf(
        stderr,
        "lingtu_driver: domain=%d topic=rt/nav/cmd_vel brainstem=%s poll=%.1fHz "
        "watchdog=%lldms lease_refresh=%lldms\n",
        config.domain_id,
        brainstem.target().c_str(),
        config.poll_hz,
        static_cast<long long>(config.limits.command_timeout.count()),
        static_cast<long long>(kLeaseRefreshPeriod.count()));

    try {
      while (g_running) {
        const auto now = Clock::now();
        const auto writer_decision = update_cmd_vel_writer_gate();
        if (!writer_decision.ready) {
          if (!cmd_vel_writer_fault_active || writer_decision.requires_stop) {
            fail_cmd_vel_writer_gate(writer_decision, now);
          }
          if (now >= next_status) {
            lingtu::driver::writeStatus(
                config, core, stats, brainstem.target(), epochSeconds());
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

        if ((!stats.ready && now >= next_probe) ||
            (stats.ready && now >= next_lease_refresh)) {
          refresh_control(now);
        }

        const auto read = dds.takeLatest();
        stats.dds_samples += read.valid_samples;
        if (read.latest) {
          stats.last_receive_s = epochSeconds();
          const auto command = core.accept(
              read.latest->freshnessInput(),
              read.latest->frame,
              read.latest->vx,
              read.latest->vy,
              read.latest->wz);
          lingtu::driver::recordFreshnessDecision(
              stats, command.freshness);
          if (command.action) {
            if (stats.ready) {
              if (send(
                      *command.action,
                      now,
                      SequencedOutputIdentity{
                          read.latest->producer_boot_id,
                          read.latest->output_seq})) {
                stats.current_output_evidence_valid = true;
                stats.last_command_accepted = true;
                stats.accepted_producer_boot_id = read.latest->producer_boot_id;
                stats.accepted_output_sequence = read.latest->output_seq;
                next_control_state = now;
              }
            } else {
              ++stats.dropped_disconnected;
              core.reset();
              lingtu::driver::invalidateCurrentOutputAck(stats);
              next_control_state = now;
            }
          } else {
            lingtu::driver::invalidateCurrentOutputAck(stats);
            next_control_state = now;
          }
        }

        if (stats.ready) {
          if (const auto stop = core.poll(now)) {
            (void)send(*stop, now, std::nullopt);
            lingtu::driver::invalidateCurrentOutputAck(stats);
            next_control_state = now;
          }
        }

        if (now >= next_control_state) {
          (void)publish_control_state();
          next_control_state = now + kControlStatePeriod;
        }
        if (now >= next_status) {
          lingtu::driver::writeStatus(
              config, core, stats, brainstem.target(), epochSeconds());
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
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "lingtu_driver: fatal: %s\n", exc.what());
    return 1;
  }
}
