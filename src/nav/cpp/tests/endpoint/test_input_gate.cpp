#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "input/gate.hpp"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

lingtu::nav::endpoint::InputSnapshot input(double now_s, double odom_s, double tf_s,
                                           double cloud_s, bool require_tf,
                                           std::uint64_t generation) {
  lingtu::nav::endpoint::InputSnapshot out;
  out.now_s = now_s;
  out.odom_stamp_s = odom_s;
  out.tf_stamp_s = tf_s;
  out.cloud_stamp_s = cloud_s;
  out.odom_requires_tf = require_tf;
  out.odom_generation = generation;
  out.tf_generation = generation;
  out.cloud_generation = generation;
  return out;
}

void testRequiresConsecutiveFreshFrames() {
  lingtu::nav::endpoint::InputGate gate({0.25, 0.25, 0.35, 3});
  auto state = gate.evaluate(input(1.0, 1.0, 1.0, 1.0, true, 1));
  require(state.recovering, "first fresh frame must remain in recovery");
  state = gate.evaluate(input(1.05, 1.05, 1.05, 1.05, true, 2));
  require(state.recovering, "second fresh frame must remain in recovery");
  state = gate.evaluate(input(1.10, 1.10, 1.10, 1.10, true, 3));
  require(state.ready, "third consecutive fresh frame must open the gate");
}

void testRecoveryAdvancesOnlyAfterEveryRequiredInputHasANewGeneration() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 2;
  cfg.require_traversability = true;
  cfg.require_localization_health = true;
  lingtu::nav::endpoint::InputGate gate(cfg);

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 10.0;
  inputs.odom_stamp_s = 10.0;
  inputs.tf_stamp_s = 10.0;
  inputs.cloud_stamp_s = 10.0;
  inputs.traversability_stamp_s = 10.0;
  inputs.localization_health_stamp_s = 10.0;
  inputs.odom_generation = 1;
  inputs.tf_generation = 1;
  inputs.cloud_generation = 1;
  inputs.traversability_generation = 1;
  inputs.localization_health_generation = 1;
  inputs.localization_healthy = true;

  auto state = gate.evaluate(inputs);
  require(state.recovering && state.fresh_frames == 1,
          "first complete DDS generation set must start recovery");

  inputs.now_s = 10.01;
  state = gate.evaluate(inputs);
  require(state.recovering && state.fresh_frames == 1,
          "re-evaluating the same DDS generations must not advance recovery");

  ++inputs.odom_generation;
  inputs.odom_stamp_s = inputs.now_s;
  state = gate.evaluate(inputs);
  require(state.recovering && state.fresh_frames == 1,
          "one faster DDS input must not advance recovery by itself");

  ++inputs.tf_generation;
  ++inputs.cloud_generation;
  ++inputs.traversability_generation;
  ++inputs.localization_health_generation;
  inputs.tf_stamp_s = inputs.now_s;
  inputs.cloud_stamp_s = inputs.now_s;
  inputs.traversability_stamp_s = inputs.now_s;
  inputs.localization_health_stamp_s = inputs.now_s;
  state = gate.evaluate(inputs);
  require(state.ready && state.fresh_frames == 2,
          "all required DDS inputs advancing must complete recovery");

  inputs.now_s = 10.02;
  state = gate.evaluate(inputs);
  require(state.ready, "an open gate must stay open while its inputs remain fresh");
}

void testCoordinateEpochForcesFreshDdsRecovery() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 2;
  lingtu::nav::endpoint::InputGate gate(cfg);

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 10.0;
  inputs.odom_stamp_s = 10.0;
  inputs.tf_stamp_s = 10.0;
  inputs.cloud_stamp_s = 10.0;
  inputs.odom_generation = 1;
  inputs.tf_generation = 1;
  inputs.cloud_generation = 1;
  require(gate.evaluate(inputs).recovering, "first input set must start recovery");
  ++inputs.odom_generation;
  ++inputs.tf_generation;
  ++inputs.cloud_generation;
  require(gate.evaluate(inputs).ready, "second input set must open the gate");

  gate.beginRecoveryFrom(inputs);
  auto state = gate.evaluate(inputs);
  require(state.recovering && state.fresh_frames == 0,
          "an epoch boundary must immediately close an already-open gate");

  ++inputs.tf_generation;
  state = gate.evaluate(inputs);
  require(state.recovering && state.fresh_frames == 0,
          "a new map transform alone must not count as a complete recovery set");

  ++inputs.odom_generation;
  ++inputs.cloud_generation;
  state = gate.evaluate(inputs);
  require(state.recovering && state.fresh_frames == 1,
          "all required post-epoch inputs must start recovery from frame one");
}

void testStaleCloudStopsImmediatelyAndRequiresRecoveryAgain() {
  lingtu::nav::endpoint::InputGate gate({0.25, 0.25, 0.35, 2});
  gate.evaluate(input(1.0, 1.0, 1.0, 1.0, true, 1));
  require(gate.evaluate(input(1.1, 1.1, 1.1, 1.1, true, 2)).ready,
          "gate must become ready");

  const auto stale = gate.evaluate(input(1.6, 1.6, 1.6, 1.1, true, 3));
  require(!stale.ready && !stale.recovering, "stale cloud must close the gate immediately");
  require(stale.reason == "cloud_stale", "stale cloud reason must be explicit");

  const auto recovering = gate.evaluate(input(1.7, 1.7, 1.7, 1.7, true, 4));
  require(recovering.recovering, "one fresh frame after a stop must not reopen motion");
}

void testMapFrameOdometryDoesNotRequireMapOdomTf() {
  lingtu::nav::endpoint::InputGate gate({0.25, 0.25, 0.35, 1});
  const auto state = gate.evaluate(input(1.0, 1.0, 0.0, 1.0, false, 1));
  require(state.ready, "map-frame odometry must not require map-to-odom TF");
}

void testFutureInputsCloseTheGate() {
  lingtu::nav::endpoint::InputGateConfig cfg{0.25, 0.25, 0.35, 1};
  cfg.future_tolerance_s = 0.05;

  lingtu::nav::endpoint::InputGate odom_gate(cfg);
  auto state = odom_gate.evaluate(input(10.0, 10.1, 10.0, 10.0, true, 1));
  require(!state.ready && state.reason == "odom_future", "future odometry must close the gate");

  lingtu::nav::endpoint::InputGate tf_gate(cfg);
  state = tf_gate.evaluate(input(10.0, 10.0, 10.1, 10.0, true, 1));
  require(!state.ready && state.reason == "tf_future", "future TF must close the gate");

  lingtu::nav::endpoint::InputGate cloud_gate(cfg);
  state = cloud_gate.evaluate(input(10.0, 10.0, 10.0, 10.1, true, 1));
  require(!state.ready && state.reason == "cloud_future", "future cloud must close the gate");

  lingtu::nav::endpoint::InputGate tolerated_gate(cfg);
  state = tolerated_gate.evaluate(input(10.0, 10.04, 10.04, 10.04, true, 1));
  require(state.ready, "small timestamp jitter inside the tolerance must remain fresh");
}

void testCloudCanBeOptional() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_cloud = false;
  lingtu::nav::endpoint::InputGate gate(cfg);

  const auto state = gate.evaluate(input(10.0, 9.9, 9.9, 0.0, true, 1));
  require(state.ready, "optional cloud must not block an obstacle-disabled endpoint");
  require(state.reason == "ready", "optional cloud gate must become ready");
}

void testOdometryAndCloudCanBeOptionalForPureTeleop() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_odom = false;
  cfg.require_cloud = false;
  lingtu::nav::endpoint::InputGate gate(cfg);

  const auto state = gate.evaluate(input(10.0, 0.0, 0.0, 0.0, true, 1));
  require(state.ready, "pure teleop must not require localization or cloud input");
  require(state.odom_age_s < 0.0, "optional odometry must be reported as not required");
  require(state.cloud_age_s < 0.0, "optional cloud must be reported as not required");
}

void testTeleopAvoidRequiresFreshTraversabilityAndHealthyLocalization() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_traversability = true;
  cfg.require_localization_health = true;
  cfg.traversability_max_age_s = 0.5;
  cfg.localization_health_max_age_s = 0.25;
  lingtu::nav::endpoint::InputGate gate(cfg);

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 10.0;
  inputs.odom_stamp_s = 9.9;
  inputs.tf_stamp_s = 9.9;
  inputs.cloud_stamp_s = 9.9;
  inputs.traversability_stamp_s = 9.8;
  inputs.localization_health_stamp_s = 9.9;
  inputs.odom_generation = 1;
  inputs.tf_generation = 1;
  inputs.cloud_generation = 1;
  inputs.traversability_generation = 1;
  inputs.localization_health_generation = 1;
  inputs.localization_healthy = true;
  require(gate.evaluate(inputs).ready, "fresh healthy teleop_avoid inputs must open the gate");

  inputs.traversability_stamp_s = 9.0;
  auto state = gate.evaluate(inputs);
  require(!state.ready && state.reason == "traversability_stale",
          "stale traversability must close the gate");

  inputs.traversability_stamp_s = 9.8;
  inputs.localization_healthy = false;
  state = gate.evaluate(inputs);
  require(!state.ready && state.reason == "localization_unhealthy",
          "unhealthy localization must close the gate");
}

void testDivergentOdometryVelocityClosesTheGate() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  require(speed_monitor.observe(10.000, "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) == 0.0,
          "the first stationary odometry sample must establish the speed baseline");
  double derived_speed = 0.0;
  for (std::size_t index = 1; index < 9; ++index) {
    derived_speed =
        std::max(derived_speed, speed_monitor.observe(10.000 + 0.010 * static_cast<double>(index),
                                                      "odom", 0.20, 0.0, 0.0, 0.0, 0.0, 0.0));
  }
  require(std::abs(derived_speed - 4.0) < 1e-9,
          "a persistent pose jump must close the gate within the temporal window "
          "when DDS twist is zero");

  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_cloud = false;
  cfg.odom_max_speed_mps = 3.0;

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 10.080;
  inputs.odom_stamp_s = 10.080;
  inputs.odom_generation = 1;
  inputs.odom_requires_tf = false;
  inputs.odom_linear_speed_mps = derived_speed;

  auto state = lingtu::nav::endpoint::InputGate(cfg).evaluate(inputs);
  require(!state.ready && state.reason == "odom_velocity_out_of_bounds",
          "an implausible SLAM velocity must close the gate even when odometry is fresh");

  inputs.odom_linear_speed_mps = std::numeric_limits<double>::infinity();
  state = lingtu::nav::endpoint::InputGate(cfg).evaluate(inputs);
  require(!state.ready && state.reason == "odom_velocity_nonfinite",
          "a non-finite SLAM velocity must close the gate");
}

void testSingleSampleOdometryPoseImpulseDoesNotCloseTheGate() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  const double baseline_speed = speed_monitor.observe(10.00, "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  require(baseline_speed == 0.0, "the first stationary sample must establish the pose baseline");

  const double impulse_speed =
      speed_monitor.observe(10.005, "odom", 0.08, 0.0, 0.04, 0.0, 0.0, 0.0);
  const double recovered_speed =
      speed_monitor.observe(10.010, "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const double stable_after_recovery_speed =
      speed_monitor.observe(10.015, "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  require(impulse_speed <= 3.0 && recovered_speed <= 3.0 && stable_after_recovery_speed <= 3.0,
          "one MuJoCo contact-tick pose impulse must not become an odometry overspeed");

  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_cloud = false;
  cfg.odom_max_speed_mps = 3.0;
  lingtu::nav::endpoint::InputGate gate(cfg);

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.odom_requires_tf = false;
  inputs.now_s = 10.00;
  inputs.odom_stamp_s = 10.00;
  inputs.odom_generation = 1;
  inputs.odom_linear_speed_mps = baseline_speed;
  require(gate.evaluate(inputs).ready, "the stationary baseline must open the gate");

  inputs.now_s = 10.005;
  inputs.odom_stamp_s = 10.005;
  ++inputs.odom_generation;
  inputs.odom_linear_speed_mps = impulse_speed;
  require(gate.evaluate(inputs).ready, "one contact pose impulse must leave the gate open");

  inputs.now_s = 10.010;
  inputs.odom_stamp_s = 10.010;
  ++inputs.odom_generation;
  inputs.odom_linear_speed_mps = recovered_speed;
  require(gate.evaluate(inputs).ready, "recovery from one pose impulse must leave the gate open");

  inputs.now_s = 10.015;
  inputs.odom_stamp_s = 10.015;
  ++inputs.odom_generation;
  inputs.odom_linear_speed_mps = stable_after_recovery_speed;
  require(gate.evaluate(inputs).ready, "the impulse return edge must not close the gate later");
}

void testTwoSampleOdometryPoseImpulseDoesNotCloseTheGate() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  const std::array<double, 6> positions{0.0, 0.08, 0.08, 0.0, 0.0, 0.0};
  for (std::size_t index = 0; index < positions.size(); ++index) {
    const double speed =
        speed_monitor.observe(10.0 + 0.005 * static_cast<double>(index), "odom", positions[index],
                              0.0, 0.04 * (positions[index] > 0.0 ? 1.0 : 0.0), 0.0, 0.0, 0.0);
    require(speed <= 3.0,
            "a two-tick contact-solver displacement that returns to the pose trend "
            "must not become an odometry overspeed");
  }
}

void testPersistentOdometryTeleportClosesWithinBoundedWindow() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  double max_speed = 0.0;
  for (std::size_t index = 0; index < 9; ++index) {
    max_speed = std::max(max_speed,
                         speed_monitor.observe(10.0 + 0.010 * static_cast<double>(index), "odom",
                                               index == 0 ? 0.0 : 0.20, 0.0, 0.0, 0.0, 0.0, 0.0));
  }
  require(max_speed > 3.0,
          "a persistent pose teleport with a falsely quiet DDS twist must close "
          "the gate within the bounded temporal window");
}

void testLeggedBodyHeaveDoesNotTripPlanarPoseSpeedConsistency() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  const std::array<double, 6> z_positions{0.0, 0.08, 0.08, 0.08, 0.08, 0.0};
  for (std::size_t index = 0; index < z_positions.size(); ++index) {
    const double speed = speed_monitor.observe(10.0 + 0.005 * static_cast<double>(index), "odom",
                                               0.0, 0.0, z_positions[index], 0.0, 0.0, 0.0);
    require(speed <= 3.0,
            "short-window quadruped body heave must not be classified as planar "
            "navigation overspeed");
  }
}

void testLeggedPlanarGaitOscillationUsesTemporalChordSpeed() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  const std::array<double, 12> x_positions{0.0,   0.035,  0.05, 0.035, 0.0,  -0.035,
                                           -0.05, -0.035, 0.0,  0.035, 0.05, 0.035};
  for (std::size_t index = 0; index < x_positions.size(); ++index) {
    const double speed = speed_monitor.observe(10.0 + 0.010 * static_cast<double>(index), "odom",
                                               x_positions[index], 0.0, 0.0, 0.0, 0.0, 0.0);
    require(speed <= 3.0,
            "bounded gait oscillation must use a temporal chord instead of an "
            "adjacent filtered-pose derivative");
  }
}

void testPersistentMultiAxisOdometryJumpsCloseTheGate() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  (void)speed_monitor.observe(10.000, "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  double confirmed_speed = 0.0;
  for (std::size_t index = 1; index < 9; ++index) {
    confirmed_speed =
        std::max(confirmed_speed, speed_monitor.observe(10.000 + 0.010 * static_cast<double>(index),
                                                        "odom", 0.20, 0.20, 0.0, 0.0, 0.0, 0.0));
  }
  require(confirmed_speed > 3.0,
          "persistent multi-axis pose jumps must close the gate within the bounded window");
}

void testRepeatedPoseImpulsesCloseTheGateBeforeRearming() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  double repeated_impulse_speed = 0.0;
  const std::array<double, 12> positions{0.0, 0.20, 0.20, 0.0, 0.20, 0.20,
                                         0.0, 0.20, 0.20, 0.0, 0.20, 0.20};
  for (std::size_t index = 0; index < positions.size(); ++index) {
    repeated_impulse_speed =
        std::max(repeated_impulse_speed,
                 speed_monitor.observe(10.000 + 0.010 * static_cast<double>(index), "odom",
                                       positions[index], 0.0, 0.0, 0.0, 0.0, 0.0));
  }
  require(repeated_impulse_speed > 3.0,
          "pose impulses occupying most of a robust window must close the gate");
}

void testOdometryMonitorHardFailuresRemainImmediate() {
  lingtu::nav::endpoint::OdometrySpeedMonitor speed_monitor;
  require(std::abs(speed_monitor.observe(10.000, "odom", 0.0, 0.0, 0.0, 3.01, 0.0, 0.0) - 3.01) <
              1e-12,
          "reported DDS twist overspeed must remain immediate");
  require(std::isinf(speed_monitor.observe(10.000, "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
          "duplicate odometry timestamps must remain fail-closed");

  speed_monitor.reset();
  require(std::isinf(speed_monitor.observe(10.005, "odom", std::numeric_limits<double>::quiet_NaN(),
                                           0.0, 0.0, 0.0, 0.0, 0.0)),
          "non-finite odometry poses must remain fail-closed");

  speed_monitor.reset();
  require(std::abs(speed_monitor.observe(10.010, "odom", 0.0, 0.0, 0.0, 0.0, 0.0, 3.01) - 3.01) <
              1e-12,
          "reported vertical DDS twist overspeed must remain immediate");
}

void testStopEvidenceUsesFreshPlanarPoseInsteadOfBiasedVelocityState() {
  using lingtu::nav::endpoint::OdometrySpeedEvidence;
  using lingtu::nav::endpoint::OdometrySpeedMonitor;

  OdometrySpeedMonitor normal;
  OdometrySpeedMonitor stop_evidence(OdometrySpeedEvidence::PoseDerivedPlanar);
  double normal_speed = 0.0;
  double stop_speed = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < 12; ++index) {
    const double stamp_s = 10.0 + 0.02 * static_cast<double>(index);
    normal_speed = normal.observe(stamp_s, "odom", 1.0, -2.0, 0.4, 0.24, 0.03, -0.08);
    stop_speed = stop_evidence.observe(stamp_s, "odom", 1.0, -2.0, 0.4, 0.24, 0.03, -0.08);
  }

  require(normal_speed > 0.20,
          "normal input evidence must preserve a suspicious reported velocity");
  require(std::isfinite(stop_speed) && stop_speed < 1e-9,
          "post-ACK stop evidence must accept a fresh, stationary planar pose even "
          "when the estimator velocity state is biased");

  double moving_speed = 0.0;
  for (std::size_t index = 0; index < 8; ++index) {
    moving_speed = stop_evidence.observe(10.24 + 0.02 * static_cast<double>(index), "odom",
                                         1.02 + 0.02 * static_cast<double>(index), -2.0, 0.4, 0.24,
                                         0.03, -0.08);
  }
  require(std::isfinite(moving_speed) && moving_speed > 0.03,
          "post-ACK stop evidence must still reject sustained planar displacement");

  const double invalid_speed = stop_evidence.observe(
      10.42, "odom", 1.10, -2.0, 0.4, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  require(std::isinf(invalid_speed),
          "non-finite odometry must remain fail-closed in pose-derived stop evidence");
}

void testLocalizationStateAndCatastrophicReasonCloseTheGate() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_odom = false;
  cfg.require_cloud = false;
  cfg.require_localization_health = true;

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 10.0;
  inputs.localization_health_stamp_s = 10.0;
  inputs.localization_health_generation = 1;
  inputs.localization_healthy = true;
  inputs.localization_state = "LOST";

  auto state = lingtu::nav::endpoint::InputGate(cfg).evaluate(inputs);
  require(!state.ready && state.reason == "localization_not_tracking",
          "a non-tracking localization state must close the gate even if a stale healthy flag says "
          "true");

  inputs.localization_state = "TRACKING";
  inputs.localization_reason = "fastlio_velocity_out_of_bounds";
  state = lingtu::nav::endpoint::InputGate(cfg).evaluate(inputs);
  require(!state.ready && state.reason == "localization_catastrophic",
          "a catastrophic Fast-LIO reason must override a TRACKING state");
}

void testSafetyInputsUseSourceTimestamps() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_odom = false;
  cfg.require_cloud = false;
  cfg.require_traversability = true;
  cfg.require_localization_health = true;
  cfg.traversability_max_age_s = 0.5;
  cfg.localization_health_max_age_s = 0.5;
  cfg.future_tolerance_s = 0.05;

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 20.0;
  inputs.traversability_stamp_s = 20.1;
  inputs.localization_health_stamp_s = 20.0;
  inputs.localization_healthy = true;
  auto state = lingtu::nav::endpoint::InputGate(cfg).evaluate(inputs);
  require(state.reason == "traversability_future",
          "future traversability source stamp must fail closed");

  inputs.traversability_stamp_s = 20.0;
  inputs.localization_health_stamp_s = 20.1;
  state = lingtu::nav::endpoint::InputGate(cfg).evaluate(inputs);
  require(state.reason == "localization_health_future",
          "future health source stamp must fail closed");

  inputs.localization_health_stamp_s = 19.0;
  state = lingtu::nav::endpoint::InputGate(cfg).evaluate(inputs);
  require(state.reason == "localization_health_stale",
          "stale health source stamp must fail closed");
}

void testDriverControlMustBeFreshAndReady() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.require_odom = false;
  cfg.require_cloud = false;
  cfg.require_driver_control = true;
  cfg.driver_control_max_age_s = 0.35;
  cfg.recovery_frames = 1;
  lingtu::nav::endpoint::InputGate gate(cfg);

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 10.0;
  auto state = gate.evaluate(inputs);
  require(!state.ready && state.reason == "driver_control_missing",
          "navigation must not start before lingtu-driver publishes control state");

  inputs.driver_control_stamp_s = 9.0;
  inputs.driver_control_generation = 1;
  inputs.driver_control_ready = true;
  state = gate.evaluate(inputs);
  require(!state.ready && state.reason == "driver_control_stale",
          "stale Brainstem readiness must close the motion gate");

  inputs.driver_control_stamp_s = 10.0;
  inputs.driver_control_ready = false;
  inputs.driver_control_reason = "preempted";
  state = gate.evaluate(inputs);
  require(!state.ready && state.reason == "driver_control_rejected",
          "lease loss or command rejection must close the motion gate");

  ++inputs.driver_control_generation;
  inputs.driver_control_ready = true;
  inputs.driver_control_reason = "none";
  state = gate.evaluate(inputs);
  require(state.ready, "fresh accepted LingTu control ownership must open the gate");
}

void testSourceOrderIsIndependentOfReceiverWallClock() {
  using lingtu::nav::endpoint::classifySourceOrder;
  using lingtu::nav::endpoint::SourceStampDecision;

  require(classifySourceOrder(100.0, 100.1, 0.35) == SourceStampDecision::kAccept,
          "monotonic source timestamps must be accepted without a receiver wall-clock comparison");
  require(classifySourceOrder(100.0, 99.9, 0.35) == SourceStampDecision::kReject,
          "small DDS reordering must remain rejected");
  require(classifySourceOrder(100.95, 100.01, 0.35) == SourceStampDecision::kClockRebase,
          "a large source-clock rollback must open a new source epoch");
}

void testInputFreshnessUsesReceiverSteadyClock() {
  lingtu::nav::endpoint::InputGateConfig cfg;
  cfg.recovery_frames = 1;
  cfg.require_odom = true;
  cfg.require_cloud = true;
  cfg.require_traversability = true;
  cfg.require_localization_health = true;
  cfg.require_driver_control = true;
  lingtu::nav::endpoint::InputGate gate(cfg);

  lingtu::nav::endpoint::InputSnapshot inputs;
  inputs.now_s = 1000.0;
  inputs.odom_stamp_s = 10.0;
  inputs.tf_stamp_s = 10.0;
  inputs.cloud_stamp_s = 10.0;
  inputs.traversability_stamp_s = 10.0;
  inputs.localization_health_stamp_s = 10.0;
  inputs.driver_control_stamp_s = 10.0;
  inputs.odom_receive_s = 999.95;
  inputs.tf_receive_s = 999.95;
  inputs.cloud_receive_s = 999.95;
  inputs.traversability_receive_s = 999.95;
  inputs.localization_health_receive_s = 999.95;
  inputs.driver_control_receive_s = 999.95;
  inputs.odom_generation = 1;
  inputs.tf_generation = 1;
  inputs.cloud_generation = 1;
  inputs.traversability_generation = 1;
  inputs.localization_health_generation = 1;
  inputs.driver_control_generation = 1;
  inputs.localization_healthy = true;
  inputs.localization_state = "TRACKING";
  inputs.driver_control_ready = true;

  const auto state = gate.evaluate(inputs);
  require(state.ready,
          "fresh local DDS receipts must keep the motion gate open despite producer clock offset");
  require(std::abs(state.odom_age_s - 0.05) < 1e-9 && std::abs(state.cloud_age_s - 0.05) < 1e-9,
          "reported input ages must use receiver steady-clock receipt time");
}

void testLocalizationHealthJsonCarriesSourceStateAndStamp() {
  const auto sample = lingtu::nav::endpoint::decodeLocalizationHealth(
      R"({"state":"TRACKING","confidence":0.98,"ts":42.125})");
  require(sample.valid, "valid localization health JSON must decode");
  require(sample.healthy, "TRACKING localization must be healthy");
  require(sample.state == "TRACKING", "health state must be preserved");
  require(std::abs(sample.stamp_s - 42.125) < 1e-12, "health source timestamp must decode");

  const auto catastrophic = lingtu::nav::endpoint::decodeLocalizationHealth(
      R"({"state":"TRACKING","reason":"fastlio_state_nonfinite","ts":42.126})");
  require(catastrophic.valid, "catastrophic health JSON must still decode");
  require(!catastrophic.healthy, "catastrophic reason must override TRACKING health");
  require(catastrophic.reason == "fastlio_state_nonfinite", "health reason must be preserved");

  const auto missing_stamp =
      lingtu::nav::endpoint::decodeLocalizationHealth(R"({"state":"TRACKING"})");
  require(!missing_stamp.valid, "health without a source timestamp must be rejected");

  const auto nested_stamp_only = lingtu::nav::endpoint::decodeLocalizationHealth(
      R"({"state":"TRACKING","map_odom_tf":{"valid":true,"ts":42.125}})");
  require(!nested_stamp_only.valid,
          "a nested transform timestamp must not masquerade as the health source timestamp");
}

}  // namespace

int main() {
  testRequiresConsecutiveFreshFrames();
  testRecoveryAdvancesOnlyAfterEveryRequiredInputHasANewGeneration();
  testCoordinateEpochForcesFreshDdsRecovery();
  testStaleCloudStopsImmediatelyAndRequiresRecoveryAgain();
  testMapFrameOdometryDoesNotRequireMapOdomTf();
  testFutureInputsCloseTheGate();
  testCloudCanBeOptional();
  testOdometryAndCloudCanBeOptionalForPureTeleop();
  testTeleopAvoidRequiresFreshTraversabilityAndHealthyLocalization();
  testDivergentOdometryVelocityClosesTheGate();
  testSingleSampleOdometryPoseImpulseDoesNotCloseTheGate();
  testTwoSampleOdometryPoseImpulseDoesNotCloseTheGate();
  testPersistentOdometryTeleportClosesWithinBoundedWindow();
  testLeggedBodyHeaveDoesNotTripPlanarPoseSpeedConsistency();
  testLeggedPlanarGaitOscillationUsesTemporalChordSpeed();
  testPersistentMultiAxisOdometryJumpsCloseTheGate();
  testRepeatedPoseImpulsesCloseTheGateBeforeRearming();
  testOdometryMonitorHardFailuresRemainImmediate();
  testStopEvidenceUsesFreshPlanarPoseInsteadOfBiasedVelocityState();
  testLocalizationStateAndCatastrophicReasonCloseTheGate();
  testSafetyInputsUseSourceTimestamps();
  testDriverControlMustBeFreshAndReady();
  testSourceOrderIsIndependentOfReceiverWallClock();
  testInputFreshnessUsesReceiverSteadyClock();
  testLocalizationHealthJsonCarriesSourceStateAndStamp();
  std::cout << "test_input_gate passed\n";
  return 0;
}
