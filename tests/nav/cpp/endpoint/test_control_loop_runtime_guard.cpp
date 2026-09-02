#include <cstdio>
#include <stdexcept>
#include <string>

#include "control/guard.hpp"
#include "status/control_loop_health.hpp"

namespace {

using lingtu::nav::endpoint::ControlLoopHealthSnapshot;
using lingtu::nav::endpoint::ControlLoopRuntimeGuard;
using lingtu::nav::endpoint::ControlLoopRuntimeGuardConfig;
using lingtu::nav::endpoint::ControlLoopRuntimeGuardDecision;
using lingtu::nav::endpoint::ControlLoopRuntimeGuardState;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

ControlLoopHealthSnapshot warmup() {
  ControlLoopHealthSnapshot snapshot;
  snapshot.ready = false;
  snapshot.healthy = false;
  snapshot.reason = "warming_up";
  return snapshot;
}

ControlLoopHealthSnapshot matureHealthy() {
  ControlLoopHealthSnapshot snapshot;
  snapshot.ready = true;
  snapshot.healthy = true;
  snapshot.reason = "healthy";
  return snapshot;
}

ControlLoopHealthSnapshot matureUnhealthy(std::string reason,
                                          std::size_t current_miss_streak = 0) {
  ControlLoopHealthSnapshot snapshot;
  snapshot.ready = true;
  snapshot.healthy = false;
  snapshot.reason = std::move(reason);
  snapshot.current_miss_streak = current_miss_streak;
  return snapshot;
}

ControlLoopRuntimeGuardConfig config() {
  ControlLoopRuntimeGuardConfig cfg;
  cfg.unhealthy_confirmation_samples = 2;
  cfg.recovery_confirmation_samples = 20;
  cfg.latch_miss_streak = 20;
  return cfg;
}

void requireState(const ControlLoopRuntimeGuardDecision &decision,
                  ControlLoopRuntimeGuardState expected, const char *message) {
  require(decision.state == expected, message);
}

void requireInvalidConfig(ControlLoopRuntimeGuardConfig cfg, const char *message) {
  try {
    ControlLoopRuntimeGuard guard(cfg);
    (void)guard;
  } catch (const std::invalid_argument &) {
    return;
  }
  require(false, message);
}

void testRejectsInvalidConfiguration() {
  ControlLoopRuntimeGuardConfig invalid = config();
  invalid.unhealthy_confirmation_samples = 0;
  requireInvalidConfig(invalid, "zero unhealthy confirmation count must be rejected");

  invalid = config();
  invalid.recovery_confirmation_samples = 0;
  requireInvalidConfig(invalid, "zero recovery confirmation count must be rejected");

  invalid = config();
  invalid.latch_miss_streak = 0;
  requireInvalidConfig(invalid, "zero latch miss streak must be rejected");
}

void testWarmupSnapshotDoesNotTripGuard() {
  ControlLoopRuntimeGuard guard(config());

  const auto decision = guard.observe(warmup());

  requireState(decision, ControlLoopRuntimeGuardState::kMonitoring,
               "warmup must remain monitoring");
  require(!decision.clear_motion, "warmup must not clear motion");
  require(!decision.hold_motion, "warmup must not hold motion");
  require(!decision.resume_allowed, "warmup must not allow resume");
}

void testHighUtilizationWarningDoesNotHoldMotion() {
  ControlLoopRuntimeGuard guard(config());

  for (int sample = 0; sample < 5; ++sample) {
    const auto decision = guard.observe(matureUnhealthy("p95_utilization_high"));

    requireState(decision, ControlLoopRuntimeGuardState::kMonitoring,
                 "high utilization without deadline loss must remain monitoring");
    require(decision.reason == "p95_utilization_high",
            "monitoring must preserve the high-utilization diagnostic");
    require(!decision.clear_motion, "high utilization must not clear motion");
    require(!decision.hold_motion, "high utilization must not hold motion");
    require(!decision.resume_allowed, "high utilization must not create a resume boundary");
  }
}

void testRollingDeadlineRatioWarningDoesNotHoldMotion() {
  ControlLoopRuntimeGuard guard(config());

  for (int sample = 0; sample < 5; ++sample) {
    const auto decision = guard.observe(matureUnhealthy("deadline_miss_ratio_high"));

    requireState(decision, ControlLoopRuntimeGuardState::kMonitoring,
                 "rolling deadline ratio without a current streak must remain monitoring");
    require(!decision.clear_motion, "rolling deadline ratio must not clear motion");
    require(!decision.hold_motion, "rolling deadline ratio must not hold motion");
  }
}

void testSingleGenericUnhealthySnapshotOnlySuspects() {
  ControlLoopRuntimeGuard guard(config());

  const auto decision = guard.observe(matureUnhealthy("control_loop_unhealthy"));

  requireState(decision, ControlLoopRuntimeGuardState::kSuspect,
               "first mature unhealthy sample must only enter suspect state");
  require(decision.reason == "control_loop_unhealthy",
          "suspect state must preserve the health reason");
  require(!decision.clear_motion, "suspect state must not clear motion");
  require(!decision.hold_motion, "suspect state must not hold motion");
  require(!decision.resume_allowed, "suspect state must not allow resume");
}

void testHealthySampleBetweenUnhealthySamplesResetsConfirmation() {
  ControlLoopRuntimeGuard guard(config());

  guard.observe(matureUnhealthy("control_loop_unhealthy"));
  guard.observe(matureHealthy());
  const auto decision = guard.observe(matureUnhealthy("control_loop_unhealthy"));

  requireState(decision, ControlLoopRuntimeGuardState::kSuspect,
               "non-consecutive mature unhealthy samples must not trip");
  require(!decision.clear_motion, "reset confirmation must not clear motion");
  require(!decision.hold_motion, "reset confirmation must not hold motion");
}

void testSecondConsecutiveMatureUnhealthySnapshotTripsGuard() {
  ControlLoopRuntimeGuard guard(config());

  guard.observe(matureUnhealthy("control_loop_unhealthy"));
  const auto decision = guard.observe(matureUnhealthy("control_loop_unhealthy"));

  requireState(decision, ControlLoopRuntimeGuardState::kLatched,
               "second consecutive mature unhealthy sample must trip");
  require(decision.clear_motion, "first trip must request clear motion");
  require(decision.hold_motion, "first trip must request hold motion");
  require(!decision.resume_allowed, "first trip must not allow resume");
  require(decision.reason == "control_loop_unhealthy",
          "trip decision must preserve the health reason");
}

void testShortDeadlineMissStreakDoesNotLatch() {
  ControlLoopRuntimeGuard guard(config());

  const auto decision = guard.observe(matureUnhealthy("consecutive_deadline_misses", 3));

  requireState(decision, ControlLoopRuntimeGuardState::kSuspect,
               "short deadline miss streak must remain recoverable without a latch");
  require(!decision.clear_motion, "short streak must not clear and latch motion");
  require(!decision.hold_motion, "short streak must not hold motion");
  require(!decision.resume_allowed, "short streak must not create a resume boundary");
}

void testSustainedDeadlineMissStreakTripsImmediately() {
  ControlLoopRuntimeGuard guard(config());

  const auto decision = guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));

  requireState(decision, ControlLoopRuntimeGuardState::kLatched,
               "sustained deadline misses must trip immediately");
  require(decision.clear_motion, "immediate trip must request clear motion");
  require(decision.hold_motion, "immediate trip must request hold motion");
  require(!decision.resume_allowed, "immediate trip must not allow resume");
}

void testLatchedGuardHoldsWithoutClearingAgain() {
  ControlLoopRuntimeGuard guard(config());

  guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));
  const auto decision = guard.observe(matureUnhealthy("deadline_miss_ratio_high"));

  requireState(decision, ControlLoopRuntimeGuardState::kLatched, "latched guard must stay latched");
  require(!decision.clear_motion, "latched guard must not clear motion more than once");
  require(decision.hold_motion, "latched guard must keep holding motion");
  require(!decision.resume_allowed, "latched guard must not allow resume before recovery");
  require(decision.reason == "deadline_miss_ratio_high",
          "latched guard must report the latest unhealthy reason");
}

void testResumeIsBlockedBeforeRecoveryConfirmation() {
  ControlLoopRuntimeGuard guard(config());
  guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));

  for (int sample = 0; sample < 19; ++sample) {
    guard.observe(matureHealthy());
  }
  const auto decision = guard.requestResume();

  requireState(decision, ControlLoopRuntimeGuardState::kLatched,
               "resume must stay blocked before recovery confirmation");
  require(!decision.resume_allowed, "resume must not be allowed before recovery");
  require(!decision.resume_completed, "blocked resume must not be completed");
}

void testResumeIsPermittedAfterRecoveryConfirmation() {
  ControlLoopRuntimeGuard guard(config());
  guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));

  for (int sample = 0; sample < 20; ++sample) {
    guard.observe(matureHealthy());
  }
  const auto decision = guard.requestResume();

  requireState(decision, ControlLoopRuntimeGuardState::kRecovered,
               "twenty mature healthy samples must enter recovered state");
  require(decision.resume_allowed, "resume must be allowed after recovery");
  require(!decision.resume_completed, "permitting resume must not complete it implicitly");
}

void testOperatorModeAutoResumesAfterRecoveryConfirmation() {
  auto cfg = config();
  cfg.auto_resume = true;
  ControlLoopRuntimeGuard guard(cfg);
  guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));

  ControlLoopRuntimeGuardDecision decision;
  for (int sample = 0; sample < 20; ++sample) {
    decision = guard.observe(matureHealthy());
  }

  requireState(decision, ControlLoopRuntimeGuardState::kMonitoring,
               "operator control must return to monitoring after healthy confirmation");
  require(!decision.hold_motion && decision.resume_completed,
          "operator control recovery must release its transient runtime hold automatically");
}

void testRollingWarningCanConfirmRecoveryAfterHardDeadlineFault() {
  ControlLoopRuntimeGuard guard(config());
  guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));

  for (int sample = 0; sample < 19; ++sample) {
    const auto decision =
        guard.observe(matureUnhealthy("deadline_miss_ratio_high", 0));
    requireState(decision, ControlLoopRuntimeGuardState::kLatched,
                 "rolling warning must not unlock before recovery confirmation");
    require(decision.hold_motion, "recovery confirmation must keep motion held");
  }
  const auto recovered = guard.observe(matureUnhealthy("deadline_miss_ratio_high", 0));

  requireState(recovered, ControlLoopRuntimeGuardState::kRecovered,
               "current deadline recovery must not wait for the rolling window to clear");
  require(recovered.resume_allowed,
          "explicit resume must be allowed after current deadline recovery");
  require(recovered.hold_motion, "recovered state must stay held until explicit resume");
}

void testFailedResumeCompletionKeepsGuardLatched() {
  ControlLoopRuntimeGuard guard(config());
  guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));
  for (int sample = 0; sample < 20; ++sample) {
    guard.observe(matureHealthy());
  }

  guard.requestResume();
  const auto decision = guard.completeResume(false);

  requireState(decision, ControlLoopRuntimeGuardState::kLatched,
               "failed resume completion must re-latch");
  require(!decision.resume_allowed, "failed resume completion must block resume");
  require(!decision.resume_completed, "failed resume completion must remain incomplete");
}

void testSuccessfulResumeCompletionResetsGuard() {
  ControlLoopRuntimeGuard guard(config());
  guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));
  for (int sample = 0; sample < 20; ++sample) {
    guard.observe(matureHealthy());
  }

  guard.requestResume();
  const auto decision = guard.completeResume(true);

  requireState(decision, ControlLoopRuntimeGuardState::kMonitoring,
               "successful resume completion must reset monitoring");
  require(decision.resume_completed, "successful completion must be visible");
  require(!decision.clear_motion, "successful completion must not clear motion");
  require(!decision.hold_motion, "successful completion must not hold motion");
}

void testResumeCompletionOutsideRecoveredDoesNotChangeHoldOrUnlock() {
  {
    ControlLoopRuntimeGuard guard(config());
    const auto failed = guard.completeResume(false);
    requireState(failed, ControlLoopRuntimeGuardState::kMonitoring,
                 "failed completion in monitoring must stay monitoring");
    require(!failed.clear_motion, "failed completion in monitoring must not clear motion");
    require(!failed.hold_motion, "failed completion in monitoring must not create hold");
    require(!failed.resume_allowed, "failed completion in monitoring must not allow resume");
    require(!failed.resume_completed, "failed completion in monitoring must not complete resume");

    const auto succeeded = guard.completeResume(true);
    requireState(succeeded, ControlLoopRuntimeGuardState::kMonitoring,
                 "successful completion in monitoring must stay monitoring");
    require(!succeeded.resume_allowed,
            "successful completion in monitoring must not unlock autonomy");
    require(!succeeded.resume_completed,
            "successful completion in monitoring must be ignored as stale");
  }

  {
    ControlLoopRuntimeGuard guard(config());
    guard.observe(matureUnhealthy("control_loop_unhealthy"));
    const auto failed = guard.completeResume(false);
    requireState(failed, ControlLoopRuntimeGuardState::kSuspect,
                 "failed completion in suspect must stay suspect");
    require(!failed.clear_motion, "failed completion in suspect must not clear motion");
    require(!failed.hold_motion, "failed completion in suspect must not create hold");
    require(!failed.resume_allowed, "failed completion in suspect must not allow resume");

    const auto succeeded = guard.completeResume(true);
    requireState(succeeded, ControlLoopRuntimeGuardState::kSuspect,
                 "successful completion in suspect must stay suspect");
    require(!succeeded.resume_allowed, "successful completion in suspect must not unlock");
    require(!succeeded.resume_completed,
            "successful completion in suspect must be ignored as stale");
  }

  {
    ControlLoopRuntimeGuard guard(config());
    guard.observe(matureUnhealthy("consecutive_deadline_misses", 20));
    const auto failed = guard.completeResume(false);
    requireState(failed, ControlLoopRuntimeGuardState::kLatched,
                 "failed completion in latched must stay latched");
    require(!failed.clear_motion, "failed completion in latched must not clear again");
    require(failed.hold_motion, "failed completion in latched must preserve hold");
    require(!failed.resume_allowed, "failed completion in latched must not allow resume");

    const auto succeeded = guard.completeResume(true);
    requireState(succeeded, ControlLoopRuntimeGuardState::kLatched,
                 "successful completion in latched must stay latched");
    require(!succeeded.clear_motion, "successful completion in latched must not clear again");
    require(succeeded.hold_motion, "successful completion in latched must preserve hold");
    require(!succeeded.resume_allowed, "successful completion in latched must not unlock");
    require(!succeeded.resume_completed,
            "successful completion in latched must be ignored before recovery");
  }
}
}  // namespace

int main() {
  try {
    testRejectsInvalidConfiguration();
    testWarmupSnapshotDoesNotTripGuard();
    testHighUtilizationWarningDoesNotHoldMotion();
    testRollingDeadlineRatioWarningDoesNotHoldMotion();
    testSingleGenericUnhealthySnapshotOnlySuspects();
    testHealthySampleBetweenUnhealthySamplesResetsConfirmation();
    testSecondConsecutiveMatureUnhealthySnapshotTripsGuard();
    testShortDeadlineMissStreakDoesNotLatch();
    testSustainedDeadlineMissStreakTripsImmediately();
    testLatchedGuardHoldsWithoutClearingAgain();
    testResumeIsBlockedBeforeRecoveryConfirmation();
    testResumeIsPermittedAfterRecoveryConfirmation();
    testOperatorModeAutoResumesAfterRecoveryConfirmation();
    testRollingWarningCanConfirmRecoveryAfterHardDeadlineFault();
    testFailedResumeCompletionKeepsGuardLatched();
    testSuccessfulResumeCompletionResetsGuard();
    testResumeCompletionOutsideRecoveredDoesNotChangeHoldOrUnlock();
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_control_loop_runtime_guard: FAIL: %s\n", error.what());
    return 1;
  }
  return 0;
}
