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
  ExpectNear(output.command.vx, 0.05, 1e-9, "fast axis is scaled to slowest axis");
  ExpectNear(output.command.vy, 0.05, 1e-9, "slow axis retains its allowed delta");
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

}  // namespace

int main() {
  TestConfigurationAndInputValidation();
  TestAccelerationDecelerationAndReversal();
  TestProportionalScaling();
  TestTimeoutAndHardStop();
  TestClosedLoopFeedbackFreshness();
  TestDeadbandAndTimestampFailures();

  if (failures != 0) {
    std::cerr << failures << " velocity smoother checks failed\n";
    return 1;
  }
  std::cout << "velocity smoother checks passed\n";
  return 0;
}
