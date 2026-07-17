#include "core.hpp"
#include "command_freshness_gate.hpp"
#include "config.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

using namespace std::chrono_literals;
using lingtu::driver::ActionReason;
using lingtu::driver::Clock;
using lingtu::driver::CommandFreshnessGate;
using lingtu::driver::CommandFreshnessInput;
using lingtu::driver::CommandFreshnessReason;
using lingtu::driver::Core;
using lingtu::driver::Limits;

void setEnvironment(const char* name, const char* value) {
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
  ScopedEnvironment(const char* name, const char* value) : name_(name) {
    if (const char* previous = std::getenv(name)) {
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

void check(bool value, const char* message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

void close(double actual, double expected, const char* message) {
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
  check(
      decision.reason == CommandFreshnessReason::Accepted,
      "fresh command reason");
}

void testFreshnessRejectionsAreExplicitAndDoNotAdvanceSequence() {
  CommandFreshnessGate gate(200ms, "host-boot");

  auto decision = gate.evaluate(CommandFreshnessInput{
      "other-host-boot", "producer-a", 1, 1000ms, 1050ms});
  check(!decision.accepted, "boot mismatch must be rejected");
  check(
      decision.reason == CommandFreshnessReason::BootMismatch,
      "boot mismatch reason");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "", 1, 1000ms, 1050ms});
  check(!decision.accepted, "missing producer boot id must be rejected");
  check(
      decision.reason == CommandFreshnessReason::ProducerBootMissing,
      "missing producer boot id reason");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 1, 1000ms, 1050ms});
  check(decision.accepted, "boot mismatch must not consume sequence");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 1, 1060ms, 1070ms});
  check(!decision.accepted, "duplicate sequence must be rejected");
  check(
      decision.reason == CommandFreshnessReason::DuplicateSequence,
      "duplicate sequence reason");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 0, 1060ms, 1070ms});
  check(!decision.accepted, "sequence rollback must be rejected");
  check(
      decision.reason == CommandFreshnessReason::SequenceRollback,
      "sequence rollback reason");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 2, 1100ms, 1099ms});
  check(!decision.accepted, "future command must be rejected");
  check(
      decision.reason == CommandFreshnessReason::Future,
      "future command reason");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 2, 1000ms, 1200ms});
  check(!decision.accepted, "command at max age must be expired");
  check(
      decision.reason == CommandFreshnessReason::Expired,
      "expired command reason");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 2, 1180ms, 1200ms});
  check(decision.accepted, "rejected timestamps must not consume sequence");
}

void testProducerRestartAdvancesBySourceBoottime() {
  CommandFreshnessGate gate(200ms, "host-boot");
  auto decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 10, 1000ms, 1010ms});
  check(decision.accepted, "first producer command must be accepted");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-b", 1, 1100ms, 1110ms});
  check(decision.accepted, "newer producer session must reset sequence");

  decision = gate.evaluate(CommandFreshnessInput{
      "host-boot", "producer-a", 11, 1150ms, 1160ms});
  check(!decision.accepted, "older producer session must not flow back");
  check(
      decision.reason == CommandFreshnessReason::ProducerRollback,
      "older producer session reason");
}

void testNormalizationAndFrames() {
  Core core(Limits{0.5, 2.0, 200ms});
  const auto now = Clock::now();
  const auto action = core.accept("body", 0.25, -1.0, 4.0, now);
  check(action.has_value(), "body command must be accepted");
  check(action->reason == ActionReason::Command, "accepted command reason");
  close(action->walk.x, 0.5, "linear x normalization");
  close(action->walk.y, -1.0, "linear y clamp");
  close(action->walk.z, 1.0, "angular clamp");

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
  close(frame_stop->walk.x, 0.0, "invalid frame zero x");
  check(!core.active(), "invalid frame clears active state");

  check(core.accept("body", 0.4, 0.0, 0.0, now + 2ms).has_value(), "resume command");
  const auto nonfinite_stop = core.accept(
      "body", std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, now + 3ms);
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
      CommandFreshnessInput{
          "host-boot", "producer-a", 1, 1000ms, 1000ms},
      "body",
      0.5,
      0.0,
      0.0);
  check(accepted.freshness.accepted, "fresh Core command must be admitted");
  check(accepted.action.has_value(), "fresh Core command must produce an action");

  const auto duplicate = core.accept(
      CommandFreshnessInput{
          "host-boot", "producer-a", 1, 1150ms, 1150ms},
      "body",
      0.5,
      0.0,
      0.0);
  check(!duplicate.freshness.accepted, "duplicate Core command must be rejected");
  check(!duplicate.action.has_value(), "rejected freshness must not produce an action");

  const auto before_timeout = lingtu::driver::TimePoint{
      std::chrono::duration_cast<Clock::duration>(1199ms)};
  check(
      !core.poll(before_timeout).has_value(),
      "watchdog must remain active before the accepted command timeout");

  const auto timeout = lingtu::driver::TimePoint{
      std::chrono::duration_cast<Clock::duration>(1200ms)};
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
  } catch (const std::invalid_argument&) {
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
  close(stop.walk.x, 0.0, "fault stop zero x");
  check(!core.active(), "fault stop clears active state");
  check(std::string(lingtu::driver::reasonName(stop.reason)) == "fault", "fault status name");
}

void testRemoteBrainstemRequiresCompleteTlsConfiguration() {
  ScopedEnvironment host("LINGTU_BRAINSTEM_HOST", "192.168.114.10");
  ScopedEnvironment ca("LINGTU_BRAINSTEM_TLS_CA_FILE", nullptr);
  ScopedEnvironment cert("LINGTU_BRAINSTEM_TLS_CERT_FILE", nullptr);
  ScopedEnvironment key("LINGTU_BRAINSTEM_TLS_KEY_FILE", nullptr);
  char program[] = "test_driver_core";
  char* argv[] = {program};

  bool threw = false;
  try {
    (void)lingtu::driver::loadConfig(1, argv);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  check(threw, "remote Brainstem without mTLS must fail closed");
}

void testLoopbackBrainstemMayUseInsecureTestTransport() {
  ScopedEnvironment host("LINGTU_BRAINSTEM_HOST", "127.0.0.1");
  ScopedEnvironment ca("LINGTU_BRAINSTEM_TLS_CA_FILE", nullptr);
  ScopedEnvironment cert("LINGTU_BRAINSTEM_TLS_CERT_FILE", nullptr);
  ScopedEnvironment key("LINGTU_BRAINSTEM_TLS_KEY_FILE", nullptr);
  char program[] = "test_driver_core";
  char* argv[] = {program};

  const auto config = lingtu::driver::loadConfig(1, argv);
  check(!config.brainstem_tls.enabled(), "loopback test transport may remain insecure");
}

}  // namespace

int main() {
  try {
    testFreshCommandIsAccepted();
    testFreshnessRejectionsAreExplicitAndDoNotAdvanceSequence();
    testProducerRestartAdvancesBySourceBoottime();
    testNormalizationAndFrames();
    testInvalidInputStopsActiveMotion();
    testWatchdogUsesElapsedTime();
    testRejectedFreshnessDoesNotRefreshWatchdog();
    testDisconnectDropsStaleCommand();
    testInvalidLimitsFailClosed();
    testFaultStopIsExplicit();
    testRemoteBrainstemRequiresCompleteTlsConfiguration();
    testLoopbackBrainstemMayUseInsecureTestTransport();
    std::cout << "test_driver_core: PASS\n";
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "test_driver_core: FAIL: " << exc.what() << '\n';
    return 1;
  }
}
