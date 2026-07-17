#include "brainstem.hpp"
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
    std::uint64_t safety_cancel_sequence = 0;
    bool readiness_announced = false;

    auto apply_result = [&](const RpcResult& result, bool command) {
      stats.connected = result.transport_ok;
      stats.control = result.state;
      stats.control.connected = result.transport_ok;
      stats.ready = result.ok;
      if (!result.transport_ok) {
        stats.control.ready = false;
        stats.control.lease_valid = false;
        stats.control.reason = "transport_error";
      }
      if (command) {
        stats.last_command_accepted = result.accepted;
      }
      stats.last_error = result.error;
    };

    auto publish_cancel = [&](const std::string& reason) {
      ++safety_cancel_sequence;
      if (dds.writeNavigationCancel(
              reason, safety_cancel_sequence, epochSeconds())) {
        ++stats.safety_cancels;
      }
    };

    auto fail_closed = [&](const std::string& reason) {
      core.reset();
      stats.ready = false;
      stats.last_command_accepted = false;
      stats.control.ready = false;
      if (!reason.empty()) {
        stats.control.reason = reason;
      }
      if (readiness_announced) {
        publish_cancel(
            std::string("brainstem_control_lost:") + stats.control.reason);
      }
      readiness_announced = false;
      dds.writeControlState(
          stats.control, stats.last_command_accepted, epochSeconds());
      next_control_state = Clock::now() + kControlStatePeriod;
    };

    auto send = [&](const Action& action, Clock::time_point now) -> bool {
      const RpcResult result = brainstem.send(action.walk);
      stats.rpc_attempts += result.rpc_calls;
      stats.last_reason = lingtu::driver::reasonName(action.reason);
      apply_result(result, true);
      if (!result.transport_ok) {
        ++stats.rpc_errors;
        fail_closed("transport_error");
        next_probe = now + config.reconnect_delay;
        return false;
      }
      if (!result.accepted || !result.ok) {
        ++stats.command_rejections;
        fail_closed(result.state.reason);
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
      ++stats.lease_refreshes;
      const RpcResult result = brainstem.refreshControl();
      stats.rpc_attempts += result.rpc_calls;
      apply_result(result, false);
      if (!result.transport_ok) {
        ++stats.rpc_errors;
        fail_closed("transport_error");
        next_probe = now + config.reconnect_delay;
        return;
      }
      if (!result.accepted || !result.ok) {
        ++stats.lease_rejections;
        fail_closed(result.state.reason);
        next_probe = now + config.reconnect_delay;
        return;
      }
      readiness_announced = true;
      stats.last_error.clear();
      next_lease_refresh = now + kLeaseRefreshPeriod;
    };

    auto best_effort_stop = [&](ActionReason reason) noexcept {
      stats.last_reason = lingtu::driver::reasonName(reason);
      try {
        if (stats.ready) {
          (void)send(core.forceStop(reason), Clock::now());
        }
        brainstem.release();
        stats.ready = false;
        stats.control.ready = false;
        stats.control.lease_valid = false;
        stats.control.reason = "released";
        dds.writeControlState(
            stats.control, stats.last_command_accepted, epochSeconds());
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
              (void)send(*command.action, now);
            } else {
              ++stats.dropped_disconnected;
              core.reset();
            }
          }
        }

        if (stats.ready) {
          if (const auto stop = core.poll(now)) {
            (void)send(*stop, now);
          }
        }

        if (now >= next_control_state) {
          dds.writeControlState(
              stats.control, stats.last_command_accepted, epochSeconds());
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
