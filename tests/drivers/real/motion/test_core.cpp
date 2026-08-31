#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "body.hpp"
#include "cmd_vel_writer_gate.hpp"
#include "command_freshness_gate.hpp"
#include "config.hpp"
#include "core.hpp"

namespace {

using namespace std::chrono_literals;
using lingtu::driver::ActionReason;
using lingtu::driver::BodyAction;
using lingtu::driver::Capabilities;
using lingtu::driver::Clock;
using lingtu::driver::CmdVelWriterGate;
using lingtu::driver::CommandFreshnessGate;
using lingtu::driver::CommandFreshnessInput;
using lingtu::driver::CommandFreshnessReason;
using lingtu::driver::Core;
using lingtu::driver::Limits;
using lingtu::driver::Result;

void setEnvironment(const char *name, const char *value) {
#ifdef _WIN32
  _putenv_s(name, value == nullptr ? "" : value);
#else
  if (value == nullptr) {
    unsetenv(name);
  } else {
    setenv(name, value, 1);
  }
#endif
}

class ScopedEnvironment {
 public:
  ScopedEnvironment(const char *name, const char *value) : name_(name) {
    if (const char *previous = std::getenv(name)) {
      had_previous_ = true;
      previous_ = previous;
    }
    setEnvironment(name, value);
  }

  ~ScopedEnvironment() {
    setEnvironment(name_.c_str(), had_previous_ ? previous_.c_str() : nullptr);
  }

 private:
  std::string name_;
  std::string previous_;
  bool had_previous_{false};
};

void check(bool value, const char *message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

void close(double actual, double expected, const char *message) {
  if (std::abs(actual - expected) > 1e-9) {
    throw std::runtime_error(message);
  }
}

void testFreshCommandIsAccepted() {
  CommandFreshnessGate gate(200ms, "host-boot");
  const auto decision = gate.evaluate(CommandFreshnessInput{
      "host-boot",
      "producer-a",
      1,
      1000ms,
      1050ms,
  });

  check(decision.accepted, "fresh command must be accepted");
  check(decision.reason == CommandFreshnessReason::Accepted, "fresh command reason");
}

void testFreshnessRejectionsAreExplicitAndDoNotAdvanceSequence() {
  CommandFreshnessGate gate(200ms, "host-boot");

  auto decision =
      gate.evaluate(CommandFreshnessInput{"other-host-boot", "producer-a", 1, 1000ms, 1050ms});
  check(!decision.accepted, "boot mismatch must be rejected");
  check(decision.reason == CommandFreshnessReason::BootMismatch, "boot mismatch reason");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "", 1, 1000ms, 1050ms});
  check(!decision.accepted, "missing producer boot id must be rejected");
  check(decision.reason == CommandFreshnessReason::ProducerBootMissing,
        "missing producer boot id reason");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 1, 1000ms, 1050ms});
  check(decision.accepted, "boot mismatch must not consume sequence");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 1, 1060ms, 1070ms});
  check(!decision.accepted, "duplicate sequence must be rejected");
  check(decision.reason == CommandFreshnessReason::DuplicateSequence, "duplicate sequence reason");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 0, 1060ms, 1070ms});
  check(!decision.accepted, "sequence rollback must be rejected");
  check(decision.reason == CommandFreshnessReason::SequenceRollback, "sequence rollback reason");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 2, 1100ms, 1099ms});
  check(!decision.accepted, "future command must be rejected");
  check(decision.reason == CommandFreshnessReason::Future, "future command reason");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 2, 1000ms, 1200ms});
  check(!decision.accepted, "command at max age must be expired");
  check(decision.reason == CommandFreshnessReason::Expired, "expired command reason");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 2, 1180ms, 1200ms});
  check(decision.accepted, "rejected timestamps must not consume sequence");
}

void testProducerRestartAdvancesBySourceBoottime() {
  CommandFreshnessGate gate(200ms, "host-boot");
  auto decision =
      gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 10, 1000ms, 1010ms});
  check(decision.accepted, "first producer command must be accepted");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-b", 1, 1100ms, 1110ms});
  check(decision.accepted, "newer producer session must reset sequence");

  decision = gate.evaluate(CommandFreshnessInput{"host-boot", "producer-a", 11, 1150ms, 1160ms});
  check(!decision.accepted, "older producer session must not flow back");
  check(decision.reason == CommandFreshnessReason::ProducerRollback,
        "older producer session reason");
}

void testPhysicalVelocityLimitsAndFrames() {
  Core core(Limits{0.5, 2.0, 200ms});
  const auto now = Clock::now();
  const auto action = core.accept("body", 0.25, -1.0, 4.0, now);
  check(action.has_value(), "body command must be accepted");
  check(action->reason == ActionReason::Command, "accepted command reason");
  close(action->velocity.vx_mps, 0.25, "linear x must remain m/s");
  close(action->velocity.vy_mps, -0.5, "linear y must clamp in m/s");
  close(action->velocity.yaw_rps, 2.0, "yaw must clamp in rad/s");

  const auto base_link = core.accept("base_link", 0.0, 0.0, 0.0, now + 1ms);
  check(base_link.has_value(), "base_link compatibility frame must be accepted");
  check(core.counters().accepted == 2, "accepted counter");
}

void testInvalidInputStopsActiveMotion() {
  Core core(Limits{1.0, 1.0, 200ms});
  const auto now = Clock::now();
  check(core.accept("body", 0.4, 0.0, 0.0, now).has_value(), "seed command");

  const auto frame_stop = core.accept("map", 0.4, 0.0, 0.0, now + 1ms);
  check(frame_stop.has_value(), "invalid frame must stop active motion");
  check(frame_stop->reason == ActionReason::InvalidFrame, "invalid frame reason");
  close(frame_stop->velocity.vx_mps, 0.0, "invalid frame zero x");
  check(!core.active(), "invalid frame clears active state");

  check(core.accept("body", 0.4, 0.0, 0.0, now + 2ms).has_value(), "resume command");
  const auto nonfinite_stop =
      core.accept("body", std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, now + 3ms);
  check(nonfinite_stop.has_value(), "nonfinite input must stop active motion");
  check(nonfinite_stop->reason == ActionReason::NonFinite, "nonfinite reason");
  check(core.counters().rejected_frame == 1, "frame reject counter");
  check(core.counters().rejected_nonfinite == 1, "nonfinite reject counter");
  check(core.counters().invalid_stops == 2, "invalid stop counter");
}

void testWatchdogUsesElapsedTime() {
  Core core(Limits{1.0, 1.0, 200ms});
  const auto now = Clock::now();
  check(core.accept("body", 0.5, 0.0, 0.0, now).has_value(), "seed command");
  check(!core.poll(now + 199ms).has_value(), "watchdog must not stop early");

  const auto stop = core.poll(now + 200ms);
  check(stop.has_value(), "watchdog must stop at timeout");
  check(stop->reason == ActionReason::Watchdog, "watchdog reason");
  check(!core.poll(now + 400ms).has_value(), "watchdog zero must be sent once");
  check(core.counters().watchdog_stops == 1, "watchdog counter");

  check(core.accept("body", 0.2, 0.0, 0.0, now + 401ms).has_value(), "new command resumes");
  check(core.active(), "new command restores active state");
}

void testRejectedFreshnessDoesNotRefreshWatchdog() {
  Core core(Limits{1.0, 1.0, 200ms}, "host-boot");
  const auto accepted = core.accept(
      CommandFreshnessInput{"host-boot", "producer-a", 1, 1000ms, 1000ms}, "body", 0.5, 0.0, 0.0);
  check(accepted.freshness.accepted, "fresh Core command must be admitted");
  check(accepted.action.has_value(), "fresh Core command must produce an action");

  const auto duplicate = core.accept(
      CommandFreshnessInput{"host-boot", "producer-a", 1, 1150ms, 1150ms}, "body", 0.5, 0.0, 0.0);
  check(!duplicate.freshness.accepted, "duplicate Core command must be rejected");
  check(!duplicate.action.has_value(), "rejected freshness must not produce an action");

  const auto before_timeout =
      lingtu::driver::TimePoint{std::chrono::duration_cast<Clock::duration>(1199ms)};
  check(!core.poll(before_timeout).has_value(),
        "watchdog must remain active before the accepted command timeout");

  const auto timeout =
      lingtu::driver::TimePoint{std::chrono::duration_cast<Clock::duration>(1200ms)};
  const auto stop = core.poll(timeout);
  check(stop.has_value(), "rejected freshness must not postpone watchdog stop");
  check(stop->reason == ActionReason::Watchdog, "freshness watchdog reason");
}

void testDisconnectDropsStaleCommand() {
  Core core(Limits{1.0, 1.0, 200ms});
  const auto now = Clock::now();
  check(core.accept("body", 0.5, 0.0, 0.0, now).has_value(), "seed command");
  core.reset();
  check(!core.active(), "reset clears active state");
  check(!core.poll(now + 1s).has_value(), "reset command must never replay");
}

void testInvalidLimitsFailClosed() {
  bool threw = false;
  try {
    Core core(Limits{0.0, 1.0, 200ms});
    (void)core;
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  check(threw, "zero speed limit must be rejected");
}

void testFaultStopIsExplicit() {
  Core core(Limits{1.0, 1.0, 200ms});
  const auto now = Clock::now();
  check(core.accept("body", 0.5, 0.0, 0.0, now).has_value(), "seed command");
  const auto stop = core.forceStop(ActionReason::Fault);
  check(stop.reason == ActionReason::Fault, "fault stop reason");
  close(stop.velocity.vx_mps, 0.0, "fault stop zero x");
  check(!core.active(), "fault stop clears active state");
  check(std::string(lingtu::driver::reasonName(stop.reason)) == "fault", "fault status name");
}

void testRemoteBrainstemRequiresCompleteTlsConfiguration() {
  ScopedEnvironment backend("LINGTU_DRIVER_BACKEND", "doso");
  ScopedEnvironment host("LINGTU_BRAINSTEM_HOST", "192.168.114.10");
  ScopedEnvironment ca("LINGTU_BRAINSTEM_TLS_CA_FILE", nullptr);
  ScopedEnvironment cert("LINGTU_BRAINSTEM_TLS_CERT_FILE", nullptr);
  ScopedEnvironment key("LINGTU_BRAINSTEM_TLS_KEY_FILE", nullptr);
  char program[] = "test_driver_core";
  char *argv[] = {program};

  bool threw = false;
  try {
    (void)lingtu::driver::loadConfig(1, argv);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "remote Brainstem without mTLS must fail closed");
}

void testBodyCapabilitiesAreExplicit() {
  const Capabilities go2{true, true, true, true};
  check(go2.supports(BodyAction::Stand), "Go2 stand capability");
  check(go2.supports(BodyAction::Sit), "Go2 sit capability");
  check(go2.supports(BodyAction::Recover), "Go2 recover capability");
  check(go2.supports(BodyAction::Damp), "Go2 damp capability");

  const Capabilities doso{true, true, false, false};
  check(doso.supports(BodyAction::Stand), "Doso stand capability");
  check(doso.supports(BodyAction::Sit), "Doso sit capability");
  check(!doso.supports(BodyAction::Damp), "Doso damp must remain unavailable");
  check(std::string(lingtu::driver::bodyActionName(BodyAction::Damp)) == "damp",
        "body action status name");
}

void testStopConfirmationRequiresTheWholeCommonContract() {
  Result result;
  result.ok = true;
  result.transport_ok = true;
  result.accepted = true;
  result.stop_confirmed = true;
  result.state.ready = false;
  result.state.control_assured = false;
  result.state.lease_valid = false;
  check(result.confirmsStop(), "acknowledged stop with invalidated readiness must confirm");

  result.state.lease_valid = true;
  check(!result.confirmsStop(), "retained motion control must not confirm stop");
  result.state.lease_valid = false;
  result.state.control_assured = true;
  check(!result.confirmsStop(), "retained control assurance must not confirm stop");
  result.state.control_assured = false;
  result.stop_confirmed = false;
  check(!result.confirmsStop(), "generic command acceptance must not masquerade as stop");
}

void testRobotSelectionIsExplicit() {
  ScopedEnvironment robot("LINGTU_DRIVER_BACKEND", nullptr);
  char program[] = "test_driver_core";
  char *argv[] = {program};

  bool threw = false;
  try {
    (void)lingtu::driver::loadConfig(1, argv);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "missing driver robot must be rejected");
}

void testLoopbackBrainstemMayUseInsecureTestTransport() {
  ScopedEnvironment backend("LINGTU_DRIVER_BACKEND", "doso");
  ScopedEnvironment host("LINGTU_BRAINSTEM_HOST", "127.0.0.1");
  ScopedEnvironment ca("LINGTU_BRAINSTEM_TLS_CA_FILE", nullptr);
  ScopedEnvironment cert("LINGTU_BRAINSTEM_TLS_CERT_FILE", nullptr);
  ScopedEnvironment key("LINGTU_BRAINSTEM_TLS_KEY_FILE", nullptr);
  char program[] = "test_driver_core";
  char *argv[] = {program};

  const auto config = lingtu::driver::loadConfig(1, argv);
  check(!config.brainstem_tls.enabled(), "loopback test transport may remain insecure");
}

void testPollRateCanHonorLeaseRefreshCadence() {
  ScopedEnvironment robot("LINGTU_DRIVER_BACKEND", "doso");
  ScopedEnvironment host("LINGTU_BRAINSTEM_HOST", "127.0.0.1");
  ScopedEnvironment ca("LINGTU_BRAINSTEM_TLS_CA_FILE", nullptr);
  ScopedEnvironment cert("LINGTU_BRAINSTEM_TLS_CERT_FILE", nullptr);
  ScopedEnvironment key("LINGTU_BRAINSTEM_TLS_KEY_FILE", nullptr);
  char program[] = "test_driver_core";
  char option[] = "--poll-hz";
  char too_slow[] = "9";
  char *slow_argv[] = {program, option, too_slow};

  bool threw = false;
  try {
    (void)lingtu::driver::loadConfig(3, slow_argv);
  } catch (const std::runtime_error &) {
    threw = true;
  }
  check(threw, "poll rate below the 10 Hz lease refresh cadence must be rejected");

  char minimum[] = "10";
  char *minimum_argv[] = {program, option, minimum};
  const auto config = lingtu::driver::loadConfig(3, minimum_argv);
  close(config.poll_hz, 10.0, "10 Hz poll rate must remain supported");
}

void testCmdVelWriterGateRequiresExactlyOneWriter() {
  CmdVelWriterGate gate;

  auto decision = gate.update(0);
  check(!decision.ready, "missing cmd_vel writer must not be ready");
  check(!decision.requires_stop, "initial missing writer must not require duplicate stop");
  check(decision.reason == "missing_cmd_vel_writer", "missing cmd_vel writer reason");

  decision = gate.update(1);
  check(decision.ready, "exactly one cmd_vel writer must be ready");
  check(!decision.requires_stop, "single writer recovery must not force stop");
  check(decision.reason == "single_cmd_vel_writer", "single cmd_vel writer reason");

  decision = gate.update(2);
  check(!decision.ready, "multiple cmd_vel writers must fail closed");
  check(decision.requires_stop, "single-to-multiple transition must request stop");
  check(decision.reason == "ambiguous_cmd_vel_writers", "ambiguous cmd_vel writer reason");

  decision = gate.update(1);
  check(decision.ready, "exactly one writer must recover readiness");
  check(!decision.requires_stop, "ambiguous-to-single recovery must not force stop");

  decision = gate.update(0);
  check(!decision.ready, "writer loss must fail closed");
  check(decision.requires_stop, "single-to-zero transition must request stop");
}

}  // namespace

int main() {
  try {
    testFreshCommandIsAccepted();
    testFreshnessRejectionsAreExplicitAndDoNotAdvanceSequence();
    testProducerRestartAdvancesBySourceBoottime();
    testPhysicalVelocityLimitsAndFrames();
    testBodyCapabilitiesAreExplicit();
    testStopConfirmationRequiresTheWholeCommonContract();
    testInvalidInputStopsActiveMotion();
    testWatchdogUsesElapsedTime();
    testRejectedFreshnessDoesNotRefreshWatchdog();
    testDisconnectDropsStaleCommand();
    testInvalidLimitsFailClosed();
    testFaultStopIsExplicit();
    testRobotSelectionIsExplicit();
    testRemoteBrainstemRequiresCompleteTlsConfiguration();
    testLoopbackBrainstemMayUseInsecureTestTransport();
    testPollRateCanHonorLeaseRefreshCadence();
    testCmdVelWriterGateRequiresExactlyOneWriter();
    std::cout << "test_driver_core: PASS\n";
    return 0;
  } catch (const std::exception &exc) {
    std::cerr << "test_driver_core: FAIL: " << exc.what() << '\n';
    return 1;
  }
}
