#include <cmath>
#include <iostream>
#include <limits>
#include <string>

#include "nav_kernel/velocity_smoother.hpp"

namespace {

int failures = 0;

void Expect(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void ExpectNear(double actual, double expected, double tolerance, const std::string &message) {
  Expect(std::abs(actual - expected) <= tolerance,
         message + " actual=" + std::to_string(actual) + " expected=" + std::to_string(expected));
}

void TestConfigurationAndInputValidation() {
  nav_kernel::VelocitySmoother smoother;
  nav_kernel::VelocitySmootherConfig invalid = smoother.config();
  invalid.x.minimum = 0.1;
  std::string error;
  Expect(!smoother.Configure(invalid, &error), "limits that exclude zero must be rejected");
  Expect(error == "x_limits_must_contain_zero", "configuration rejection must be diagnosable");

  Expect(!smoother.SetTarget({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 0.0, &error),
         "non-finite targets must be rejected");
  Expect(smoother.SetTarget({0.2, 0.0, 0.0}, 1.0, &error), "valid target must be accepted");
  Expect(!smoother.SetTarget({0.2, 0.0, 0.0}, 0.9, &error), "out-of-order target must be rejected");
}

void TestAccelerationDecelerationAndReversal() {
  nav_kernel::VelocitySmootherConfig config;
  config.x.acceleration = 1.0;
  config.x.deceleration = 2.0;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "forward target accepted");
  ExpectNear(smoother.Step(0.0).command.vx, 0.0, 1e-9, "first tick initializes from zero");
  ExpectNear(smoother.Step(0.1).command.vx, 0.1, 1e-9, "acceleration is time bounded");
  ExpectNear(smoother.Step(0.2).command.vx, 0.2, 1e-9,
             "acceleration accumulates deterministically");

  Expect(smoother.SetTarget({-1.0, 0.0, 0.0}, 0.2), "reverse target accepted");
  ExpectNear(smoother.Step(0.3).command.vx, 0.0, 1e-9, "reversal decelerates to zero first");
  ExpectNear(smoother.Step(0.4).command.vx, -0.1, 1e-9,
             "reverse acceleration starts on a later tick");
}

void TestProportionalScaling() {
  nav_kernel::VelocitySmootherConfig config;
  config.scale_velocities = true;
  config.x = {-1.0, 1.0, 1.0, 2.0, 0.0};
  config.y = {-1.0, 1.0, 0.5, 2.0, 0.0};
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 1.0, 0.0}, 0.0), "diagonal target accepted");
  (void)smoother.Step(0.0);
  const auto output = smoother.Step(0.1);
  // y (accel 0.5) is the most restricted axis: its natural delta is 0.05 and
  // it dictates the common fraction, so x (natural delta 0.1) is scaled down.
  ExpectNear(output.command.vx, 0.05, 1e-9,
             "most restricted axis dictates the common fraction; faster x axis is scaled down "
             "to match");
  ExpectNear(output.command.vy, 0.05, 1e-9,
             "most restricted y axis keeps its natural rate-limited delta");
}

void TestTimeoutAndHardStop() {
  nav_kernel::VelocitySmootherConfig config;
  config.target_timeout_s = 0.25;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "target accepted");
  (void)smoother.Step(0.0);
  Expect(smoother.Step(0.1).command.vx > 0.0, "command moves before timeout");

  const auto timeout = smoother.Step(0.3);
  Expect(timeout.valid && timeout.timed_out, "stale target produces an explicit timeout");
  ExpectNear(timeout.command.vx, 0.0, 1e-9, "timeout immediately fails closed");

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.31), "new target accepted after timeout");
  (void)smoother.Step(0.31);
  const auto stopped = smoother.Stop(0.32, "estop");
  Expect(stopped.valid && stopped.reason == "estop", "hard stop preserves its safety reason");
  ExpectNear(stopped.command.vx, 0.0, 1e-9, "hard stop bypasses smoothing");
  Expect(smoother.Step(0.33).timed_out, "hard stop clears the previous target");
}

void TestClosedLoopFeedbackFreshness() {
  nav_kernel::VelocitySmootherConfig config;
  config.feedback_mode = nav_kernel::VelocityFeedbackMode::kClosedLoop;
  config.feedback_timeout_s = 0.1;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "closed-loop target accepted");
  const auto missing = smoother.Step(0.0);
  Expect(!missing.valid && missing.reason == "feedback_missing",
         "closed loop fails closed without feedback");

  Expect(smoother.SetFeedback({0.2, 0.0, 0.0}, 0.01), "feedback accepted");
  const auto fresh = smoother.Step(0.01);
  Expect(fresh.valid && fresh.feedback_used, "fresh feedback is used");
  ExpectNear(fresh.command.vx, 0.21, 1e-9, "closed-loop rate limit starts from measured velocity");

  const auto stale = smoother.Step(0.2);
  Expect(!stale.valid && stale.reason == "feedback_timeout", "stale feedback fails closed");
  ExpectNear(stale.command.vx, 0.0, 1e-9, "stale feedback cannot leave motion active");
}

void TestDeadbandAndTimestampFailures() {
  nav_kernel::VelocitySmootherConfig config;
  config.x.deadband = 0.15;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({0.1, 0.0, 0.0}, 0.0), "small target accepted");
  (void)smoother.Step(0.0);
  const auto deadband = smoother.Step(0.1);
  ExpectNear(deadband.command.vx, 0.0, 1e-9, "deadband suppresses tiny output");

  nav_kernel::VelocitySmoother future(config);
  Expect(future.SetTarget({0.2, 0.0, 0.0}, 1.0), "future target can be queued");
  const auto rejected = future.Step(0.0);
  Expect(!rejected.valid && rejected.reason == "target_timestamp_future",
         "excessively future target fails closed");
}

void TestTimeoutBoundaryFailsClosed() {
  nav_kernel::VelocitySmootherConfig config;
  config.target_timeout_s = 0.25;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "target accepted");
  (void)smoother.Step(0.0);
  Expect(smoother.Step(0.1).command.vx > 0.0, "command ramps before the boundary");

  // Age exactly equal to the timeout must fail closed under the >= rule.
  const auto boundary = smoother.Step(0.25);
  Expect(boundary.valid && boundary.timed_out,
         "target age exactly at timeout must fail closed immediately");
  ExpectNear(boundary.command.vx, 0.0, 1e-9, "boundary timeout output is zero");
}

void TestDtClampingBoundsStepGrowth() {
  nav_kernel::VelocitySmootherConfig config;  // x accel 1.0, max_step_s 0.2
  config.target_timeout_s = 1.0;              // keep the target fresh across the gap
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "target accepted");
  const auto init = smoother.Step(0.0);
  Expect(!init.dt_clamped, "first step has no clamp diagnostic");

  const auto gap = smoother.Step(0.5);
  Expect(gap.dt_clamped, "large step interval must report dt_clamped");
  ExpectNear(gap.command.vx, 0.2, 1e-9,
             "single-step growth is bounded by acceleration times max_step_s");
  Expect(gap.command.vx <= 1.0 * config.max_step_s + 1e-9,
         "clamped step cannot exceed rate times max_step_s");

  const auto normal = smoother.Step(0.6);
  Expect(!normal.dt_clamped, "small interval must not report dt_clamped");
  ExpectNear(normal.command.vx, 0.3, 1e-9, "normal interval uses the real dt");
}

void TestMultiStepReversalSequence() {
  nav_kernel::VelocitySmootherConfig config;  // x accel 1.0, decel 2.0
  config.target_timeout_s = 2.0;              // keep the target fresh through the sequence
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({0.5, 0.0, 0.0}, 0.0), "forward target accepted");
  (void)smoother.Step(0.0);
  (void)smoother.Step(0.2);
  (void)smoother.Step(0.4);
  ExpectNear(smoother.Step(0.5).command.vx, 0.5, 1e-9, "reached forward hold velocity");

  Expect(smoother.SetTarget({-0.5, 0.0, 0.0}, 0.5), "reverse target accepted");
  double previous = 0.5;
  bool reached_zero = false;
  bool crossed_without_zero = false;
  int steps = 0;
  double t = 0.5;
  while (steps < 20) {
    t += 0.1;
    const auto out = smoother.Step(t);
    Expect(out.valid, "reversal step stays valid");
    const double vx = out.command.vx;
    Expect(vx <= previous + 1e-9, "reversal sequence is monotonically non-increasing");
    if (std::abs(vx) <= 1e-9) {
      reached_zero = true;
    } else if (vx < 0.0 && !reached_zero) {
      crossed_without_zero = true;
    }
    previous = vx;
    ++steps;
    if (std::abs(vx + 0.5) <= 1e-9) {
      break;
    }
  }
  Expect(steps >= 3, "reversal from 0.5 to -0.5 needs more than two steps");
  Expect(reached_zero, "reversal passes through zero explicitly");
  Expect(!crossed_without_zero, "no single step may jump across zero");
  ExpectNear(previous, -0.5, 1e-9, "reversal eventually reaches the target");

  // Retarget again in the middle of a reversal.
  Expect(smoother.SetTarget({0.5, 0.0, 0.0}, t), "retarget back forward accepted");
  double retarget_prev = previous;
  for (int i = 0; i < 20; ++i) {
    t += 0.1;
    const auto out = smoother.Step(t);
    Expect(out.valid, "retarget step stays valid");
    const double vx = out.command.vx;
    Expect(vx >= retarget_prev - 1e-9, "retarget sequence is monotonically non-decreasing");
    retarget_prev = vx;
    if (std::abs(vx - 0.5) <= 1e-9) {
      break;
    }
  }
  ExpectNear(retarget_prev, 0.5, 1e-9, "mid-reversal retarget reaches the new target");
}

void TestFeedbackValidationAndRecovery() {
  nav_kernel::VelocitySmootherConfig config;
  config.feedback_mode = nav_kernel::VelocityFeedbackMode::kClosedLoop;
  config.feedback_timeout_s = 0.1;
  std::string error;

  // a) Out-of-order feedback timestamps are rejected.
  nav_kernel::VelocitySmoother out_of_order(config);
  Expect(out_of_order.SetFeedback({0.1, 0.0, 0.0}, 1.0, &error), "first feedback accepted");
  Expect(!out_of_order.SetFeedback({0.1, 0.0, 0.0}, 0.9, &error),
         "out-of-order feedback must be rejected");
  Expect(error == "feedback_out_of_order", "out-of-order feedback is diagnosable");

  // b) Feedback beyond the future tolerance window fails the step closed.
  nav_kernel::VelocitySmoother future(config);
  Expect(future.SetTarget({1.0, 0.0, 0.0}, 0.0), "closed-loop target accepted");
  (void)future.Step(0.0);
  Expect(future.SetFeedback({0.0, 0.0, 0.0}, 0.2), "future-stamped feedback is stored");
  const auto rejected = future.Step(0.0);
  Expect(!rejected.valid && rejected.timed_out && rejected.reason == "feedback_timestamp_future",
         "feedback outside the future window fails closed with timed_out");
  ExpectNear(rejected.command.vx, 0.0, 1e-9, "future feedback cannot drive motion");

  // c) Timeout zeroes the output, then fresh feedback resumes the ramp.
  nav_kernel::VelocitySmoother recovery(config);
  Expect(recovery.SetTarget({1.0, 0.0, 0.0}, 0.0), "recovery target accepted");
  Expect(recovery.SetFeedback({0.0, 0.0, 0.0}, 0.0), "initial feedback accepted");
  (void)recovery.Step(0.0);
  ExpectNear(recovery.Step(0.05).command.vx, 0.05, 1e-9, "closed-loop ramp in progress");
  const auto stale = recovery.Step(0.3);
  Expect(!stale.valid && stale.timed_out && stale.reason == "feedback_timeout",
         "stale feedback fails closed");
  ExpectNear(stale.command.vx, 0.0, 1e-9, "timeout zeroes the command");

  Expect(recovery.SetFeedback({0.0, 0.0, 0.0}, 0.3), "fresh feedback accepted after timeout");
  const auto resume = recovery.Step(0.35);
  Expect(resume.valid && resume.feedback_used, "fresh feedback resumes control");
  ExpectNear(resume.command.vx, 0.05, 1e-9, "ramp restarts from the fresh measurement");
  Expect(recovery.SetFeedback({0.05, 0.0, 0.0}, 0.4), "measurement feedback accepted");
  ExpectNear(recovery.Step(0.45).command.vx, 0.15, 1e-9,
             "ramp keeps accumulating from the latest measurement");
}

void TestStopRecoveryRampsFromZero() {
  nav_kernel::VelocitySmoother smoother;
  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "target accepted");
  (void)smoother.Step(0.0);
  ExpectNear(smoother.Step(0.1).command.vx, 0.1, 1e-9, "ramp in progress before stop");

  (void)smoother.Stop(0.15, "estop");
  Expect(smoother.SetTarget({0.5, 0.0, 0.0}, 0.15), "fresh target accepted after stop");
  const auto resumed = smoother.Step(0.25);
  Expect(resumed.valid, "post-stop step is valid");
  ExpectNear(resumed.command.vx, 0.1, 1e-9, "post-stop command ramps from zero, not from history");
  ExpectNear(smoother.Step(0.35).command.vx, 0.2, 1e-9, "post-stop ramp keeps accelerating");
}

void TestConfigureFailurePathsAndReconfigure() {
  nav_kernel::VelocitySmoother smoother;
  std::string error;

  auto expect_rejected = [&smoother, &error](
                             const nav_kernel::VelocitySmootherConfig &candidate,
                             const std::string &expected_error, const std::string &message) {
    error.clear();
    Expect(!smoother.Configure(candidate, &error), message);
    Expect(error == expected_error, message + " error=" + error + " expected=" + expected_error);
  };

  nav_kernel::VelocitySmootherConfig bad = smoother.config();
  bad.x.acceleration = std::numeric_limits<double>::quiet_NaN();
  expect_rejected(bad, "x_limits_nonfinite", "non-finite axis limit must be rejected");

  bad = smoother.config();
  bad.y.deceleration = 0.0;
  expect_rejected(bad, "y_rate_limit_nonpositive", "non-positive rate limit must be rejected");

  bad = smoother.config();
  bad.yaw.acceleration = -1.0;
  expect_rejected(bad, "yaw_rate_limit_nonpositive", "negative acceleration must be rejected");

  bad = smoother.config();
  bad.x.deadband = 1.5;
  expect_rejected(bad, "x_deadband_invalid", "deadband beyond axis magnitude must be rejected");

  bad = smoother.config();
  bad.x.deadband = -0.1;
  expect_rejected(bad, "x_deadband_invalid", "negative deadband must be rejected");

  bad = smoother.config();
  bad.x.minimum = 0.2;
  expect_rejected(bad, "x_limits_must_contain_zero", "limits excluding zero must be rejected");

  bad = smoother.config();
  bad.y.minimum = -0.1;
  bad.y.maximum = -0.5;
  expect_rejected(bad, "y_limits_must_contain_zero", "minimum above maximum must be rejected");

  bad = smoother.config();
  bad.target_timeout_s = 0.0;
  expect_rejected(bad, "target_timeout_invalid", "non-positive target timeout must be rejected");

  bad = smoother.config();
  bad.feedback_timeout_s = std::numeric_limits<double>::quiet_NaN();
  expect_rejected(bad, "feedback_timeout_invalid", "non-finite feedback timeout must be rejected");

  bad = smoother.config();
  bad.max_step_s = -0.1;
  expect_rejected(bad, "max_step_invalid", "non-positive max step must be rejected");

  bad = smoother.config();
  bad.future_tolerance_s = -0.01;
  expect_rejected(bad, "future_tolerance_invalid", "negative future tolerance must be rejected");

  // A valid reconfigure clears all runtime state.
  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "target accepted before reconfigure");
  (void)smoother.Step(0.0);
  ExpectNear(smoother.Step(0.1).command.vx, 0.1, 1e-9, "ramp in progress before reconfigure");
  error.clear();
  Expect(smoother.Configure(smoother.config(), &error), "valid reconfigure succeeds");
  const auto cleared = smoother.Step(0.2);
  Expect(cleared.valid && cleared.timed_out && cleared.reason == "target_missing",
         "reconfigure clears the previous target");
  ExpectNear(cleared.command.vx, 0.0, 1e-9, "reconfigure output is zero");
  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.2), "new target accepted after reconfigure");
  ExpectNear(smoother.Step(0.3).command.vx, 0.1, 1e-9, "ramp restarts from zero after reconfigure");
}

void TestYawAccelerationAtFiftyHertz() {
  nav_kernel::VelocitySmootherConfig config;
  config.yaw.acceleration = 1.2;
  config.yaw.deceleration = 2.4;
  config.target_timeout_s = 2.0;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({0.0, 0.0, 0.6}, 0.0), "yaw target accepted");
  auto previous = smoother.Step(0.0).command.wz;
  for (int tick = 1; tick <= 25; ++tick) {
    const double now_s = 0.02 * static_cast<double>(tick);
    const double current = smoother.Step(now_s).command.wz;
    Expect(current >= previous - 1e-9, "yaw ramp must be monotonic");
    Expect(current - previous <= config.yaw.acceleration * 0.02 + 1e-9,
           "50 Hz yaw acceleration must stay within rad/s^2 limit");
    previous = current;
  }
  ExpectNear(previous, 0.6, 1e-9, "50 Hz yaw ramp reaches the requested rate");

  Expect(smoother.SetTarget({0.0, 0.0, 0.0}, 0.5), "zero yaw target accepted");
  const double slowing = smoother.Step(0.52).command.wz;
  ExpectNear(slowing, 0.6 - config.yaw.deceleration * 0.02, 1e-9,
             "50 Hz yaw stop uses the configured deceleration");

  const auto stopped = smoother.Stop(0.53, "rotation_safety_stop");
  ExpectNear(stopped.command.wz, 0.0, 1e-9,
             "rotation safety stop bypasses gradual deceleration");
}

void TestCommitAppliedRebasesNextStepWithoutChangingTarget() {
  nav_kernel::VelocitySmootherConfig config;
  config.target_timeout_s = 1.0;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "target accepted");
  (void)smoother.Step(0.0);
  ExpectNear(smoother.Step(0.2).command.vx, 0.2, 1e-9, "smoother ramps before safety slowdown");
  Expect(smoother.CommitApplied({0.05, 0.0, 0.0}, 0.2),
         "actual safety-limited publication is accepted");

  const auto released = smoother.Step(0.3);
  Expect(released.valid, "step after safety release is valid");
  ExpectNear(released.command.vx, 0.15, 1e-9,
             "step after safety release ramps from the actual published command");
  ExpectNear(smoother.Step(0.4).command.vx, 0.25, 1e-9,
             "original target remains active after applied-state commit");
}

void TestCommitAppliedRejectsInvalidStateWithoutMutation() {
  auto expect_rejected = [](const nav_kernel::Twist &applied, double timestamp_s,
                            const std::string &expected_error, const std::string &message) {
    nav_kernel::VelocitySmootherConfig config;
    config.target_timeout_s = 1.0;
    nav_kernel::VelocitySmoother smoother(config);
    Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), message + " setup target accepted");
    (void)smoother.Step(0.0);
    (void)smoother.Step(0.2);
    Expect(smoother.CommitApplied({0.05, 0.0, 0.0}, 0.2),
           message + " setup applied state accepted");

    std::string error;
    Expect(!smoother.CommitApplied(applied, timestamp_s, &error), message + " must be rejected");
    Expect(error == expected_error,
           message + " error=" + error + " expected=" + expected_error);
    ExpectNear(smoother.last_output().vx, 0.05, 1e-9,
               message + " must preserve the previous applied command");
    ExpectNear(smoother.Step(0.3).command.vx, 0.15, 1e-9,
               message + " must preserve the previous applied timestamp");
  };

  expect_rejected({std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, 0.25,
                  "applied_invalid", "non-finite applied command");
  expect_rejected({0.05, 0.0, 0.0}, std::numeric_limits<double>::quiet_NaN(),
                  "applied_invalid", "non-finite applied timestamp");
  expect_rejected({0.05, 0.0, 0.0}, -0.1, "applied_invalid",
                  "negative applied timestamp");
  expect_rejected({0.05, 0.0, 0.0}, 0.19, "applied_out_of_order",
                  "out-of-order applied timestamp");
  expect_rejected({1.01, 0.0, 0.0}, 0.25, "applied_out_of_bounds",
                  "x-axis applied command outside limits");
  expect_rejected({0.05, -0.51, 0.0}, 0.25, "applied_out_of_bounds",
                  "y-axis applied command outside limits");
  expect_rejected({0.05, 0.0, 1.51}, 0.25, "applied_out_of_bounds",
                  "yaw-axis applied command outside limits");
}

void TestDeadbandDoesNotDiscardRateLimitedProgress() {
  nav_kernel::VelocitySmootherConfig config;
  config.x.acceleration = 0.1;
  config.x.deadband = 0.025;
  config.target_timeout_s = 1.0;
  nav_kernel::VelocitySmoother smoother(config);

  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.0), "low-rate target accepted");
  (void)smoother.Step(0.0);
  ExpectNear(smoother.Step(0.1).command.vx, 0.0, 1e-9,
             "first sub-deadband step publishes zero");
  ExpectNear(smoother.Step(0.2).command.vx, 0.0, 1e-9,
             "second sub-deadband step still publishes zero");
  const auto crossed = smoother.Step(0.3);
  ExpectNear(crossed.command.vx, 0.03, 1e-9,
             "accumulated rate-limited state eventually crosses the deadband");
  Expect(crossed.command.vx <= config.x.acceleration * 0.3 + 1e-9,
         "published motion cannot exceed rate times total elapsed time");

  (void)smoother.Stop(0.31, "raw_stop");
  Expect(smoother.SetTarget({1.0, 0.0, 0.0}, 0.31), "target accepted after raw stop");
  ExpectNear(smoother.Step(0.31).command.vx, 0.0, 1e-9,
             "raw stop clears accumulated pre-deadband state");
  ExpectNear(smoother.Step(0.41).command.vx, 0.0, 1e-9,
             "post-stop ramp restarts below the deadband");
  ExpectNear(smoother.Step(0.51).command.vx, 0.0, 1e-9,
             "post-stop ramp must accumulate again from zero");
  ExpectNear(smoother.Step(0.61).command.vx, 0.03, 1e-9,
             "post-stop ramp crosses the deadband only after rebuilding history");
}

}  // namespace

int main() {
  TestConfigurationAndInputValidation();
  TestAccelerationDecelerationAndReversal();
  TestYawAccelerationAtFiftyHertz();
  TestProportionalScaling();
  TestTimeoutAndHardStop();
  TestClosedLoopFeedbackFreshness();
  TestDeadbandAndTimestampFailures();
  TestTimeoutBoundaryFailsClosed();
  TestDtClampingBoundsStepGrowth();
  TestMultiStepReversalSequence();
  TestFeedbackValidationAndRecovery();
  TestStopRecoveryRampsFromZero();
  TestConfigureFailurePathsAndReconfigure();
  TestCommitAppliedRebasesNextStepWithoutChangingTarget();
  TestCommitAppliedRejectsInvalidStateWithoutMutation();
  TestDeadbandDoesNotDiscardRateLimitedProgress();

  if (failures != 0) {
    std::cerr << failures << " velocity smoother checks failed\n";
    return 1;
  }
  std::cout << "velocity smoother checks passed\n";
  return 0;
}
