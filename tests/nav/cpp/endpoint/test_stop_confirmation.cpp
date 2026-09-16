#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "safety/stop.hpp"

namespace {

using lingtu::nav::endpoint::StopConfirmation;
using lingtu::nav::endpoint::StopConfirmationConfig;
using lingtu::nav::endpoint::StopConfirmationEvidencePolicy;
using lingtu::nav::endpoint::StopConfirmationState;

constexpr std::uint64_t stampNs(std::uint64_t sec, std::uint64_t nanosec = 0U) {
  return sec * 1000000000ULL + nanosec;
}

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void observeQuietStop(StopConfirmation &confirmation, double first_stamp_s) {
  confirmation.observeQuietOdometry(first_stamp_s, 0.0, 0.0);
  confirmation.observeQuietOdometry(first_stamp_s + 0.1, 0.0, 0.0);
  confirmation.observeQuietOdometry(first_stamp_s + 0.2, 0.0, 0.0);
}

void testRejectsMismatchedAndStaleDriverAcks() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{10s};
  StopConfirmation wrong_producer("endpoint-a", 42, stampNs(100), start, {1000ms, 0.03, 0.08, 3});
  wrong_producer.observeDriverAck("endpoint-b", 42, true, stampNs(101));
  observeQuietStop(wrong_producer, 101.1);
  require(wrong_producer.state(start + 100ms) == StopConfirmationState::Pending,
          "an ACK from another producer must not confirm this stop");

  StopConfirmation wrong_sequence("endpoint-a", 42, stampNs(100), start, {1000ms, 0.03, 0.08, 3});
  wrong_sequence.observeDriverAck("endpoint-a", 43, true, stampNs(101));
  observeQuietStop(wrong_sequence, 101.1);
  require(wrong_sequence.state(start + 100ms) == StopConfirmationState::Pending,
          "an ACK for another output sequence must not confirm this stop");

  StopConfirmation stale_ack("endpoint-a", 42, stampNs(100), start, {1000ms, 0.03, 0.08, 3});
  stale_ack.observeDriverAck("endpoint-a", 42, true, stampNs(99, 900000000));
  observeQuietStop(stale_ack, 100.1);
  require(stale_ack.state(start + 100ms) == StopConfirmationState::Pending,
          "an ACK stamped before the zero publication must not confirm this stop");
}

void testUsesExactZeroCommandStampNotLaterWallRead() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{15s};
  StopConfirmation confirmation("endpoint-a", 42, stampNs(100), start, {1000ms, 0.03, 0.08, 3});

  confirmation.observeDriverAck("endpoint-a", 42, true, stampNs(100, 100000000));
  observeQuietStop(confirmation, 100.2);
  require(confirmation.state(start + 300ms) == StopConfirmationState::Confirmed,
          "ACK after the exact zero command source stamp must confirm even if a later wall read "
          "would exceed it");

  StopConfirmation post_write_wall_read("endpoint-a", 42, stampNs(100, 200000000), start,
                                        {1000ms, 0.03, 0.08, 3});
  post_write_wall_read.observeDriverAck("endpoint-a", 42, true, stampNs(100, 100000000));
  observeQuietStop(post_write_wall_read, 100.3);
  require(post_write_wall_read.state(start + 300ms) == StopConfirmationState::Pending,
          "using a post-write wall read as the zero stamp reproduces the lost valid ACK");
}

void testRequiresMatchingDriverAckFollowedByQuietOdometry() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{20s};
  StopConfirmation confirmation("endpoint-a", 42, stampNs(100), start, {1000ms, 0.03, 0.08, 3});

  observeQuietStop(confirmation, 100.1);
  confirmation.observeDriverAck("endpoint-a", 42, true, stampNs(101));
  require(confirmation.state(start + 200ms) == StopConfirmationState::Pending,
          "quiet odometry before the matching ACK must not confirm stop");
  observeQuietStop(confirmation, 101.1);
  require(confirmation.state(start + 300ms) == StopConfirmationState::Confirmed,
          "matching Brainstem ACK followed by quiet odometry must confirm stop");
}

void testOdometryClockMayLagTheDriverWallClock() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{25s};
  StopConfirmation confirmation("endpoint-a", 42, stampNs(1000), start,
                                {1000ms, 0.03, 0.08, 3});

  confirmation.observeDriverAck("endpoint-a", 42, true, stampNs(1001));
  observeQuietStop(confirmation, 12.0);
  require(confirmation.state(start + 300ms) == StopConfirmationState::Confirmed,
          "post-ACK receive order must not compare an odometry clock with the driver wall clock");
}

void testMotionResetsQuietSampleWindow() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{30s};
  StopConfirmation confirmation("endpoint-a", 7, stampNs(200), start, {1000ms, 0.03, 0.08, 2});
  confirmation.observeDriverAck("endpoint-a", 7, true, stampNs(201));
  confirmation.observeQuietOdometry(201.1, 0.01, 0.01);
  confirmation.observeQuietOdometry(201.2, 0.2, 0.01);
  confirmation.observeQuietOdometry(201.3, 0.01, 0.01);
  require(confirmation.state(start + 100ms) == StopConfirmationState::Pending,
          "motion must reset consecutive quiet samples");
  confirmation.observeQuietOdometry(201.4, 0.01, 0.01);
  require(confirmation.state(start + 200ms) == StopConfirmationState::Confirmed,
          "the complete quiet window must be observed after motion");
  const auto diagnostics = confirmation.diagnostics();
  require(diagnostics.driver_ack_observed && diagnostics.driver_accepted,
          "diagnostics must preserve matching driver ACK evidence");
  require(diagnostics.odometry_samples_observed == 4U &&
              diagnostics.post_ack_odometry_samples == 4U,
          "diagnostics must count observed and post-ACK odometry");
  require(diagnostics.moving_odometry_samples == 1U && diagnostics.quiet_odometry_samples == 2U,
          "diagnostics must expose motion resets and final quiet evidence");
}

void testDriverRejectionAndTimeoutFailClosed() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{40s};
  StopConfirmation rejected("endpoint-a", 9, stampNs(300), start, {500ms, 0.03, 0.08, 1});
  rejected.observeDriverAck("endpoint-a", 9, false, stampNs(301));
  require(rejected.state(start + 10ms) == StopConfirmationState::DriverRejected,
          "matching driver rejection must fail immediately");

  StopConfirmation timed_out("endpoint-a", 10, stampNs(300), start, {500ms, 0.03, 0.08, 1});
  timed_out.observeDriverAck("endpoint-a", 10, true, stampNs(301));
  require(timed_out.state(start + 501ms) == StopConfirmationState::TimedOut,
          "missing odometry confirmation must time out");

  StopConfirmation late("endpoint-a", 11, stampNs(300), start, {500ms, 0.03, 0.08, 1});
  late.observeDriverAck("endpoint-a", 11, true, stampNs(301));
  late.observeQuietOdometry(301.1, 0.0, 0.0);
  require(late.state(start + 501ms) == StopConfirmationState::TimedOut,
          "confirmation evidence observed after the deadline must fail closed");
}

void testDriverAckOnlyPolicyIsExplicitAndStillRequiresFreshExactAcceptance() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{50s};
  const StopConfirmationConfig config{
      500ms,
      0.03,
      0.08,
      8,
      StopConfirmationEvidencePolicy::DriverAckOnly,
  };
  StopConfirmation confirmation("endpoint-a", 12, stampNs(400), start, config);
  require(confirmation.state(start + 10ms) == StopConfirmationState::Pending,
          "ACK-only policy must not confirm before a matching driver result");
  confirmation.observeDriverAck("endpoint-b", 12, true, stampNs(401));
  confirmation.observeDriverAck("endpoint-a", 13, true, stampNs(401));
  confirmation.observeDriverAck("endpoint-a", 12, true, stampNs(399));
  require(confirmation.state(start + 20ms) == StopConfirmationState::Pending,
          "ACK-only policy accepted stale or mismatched driver evidence");
  confirmation.observeDriverAck("endpoint-a", 12, true, stampNs(401));
  require(confirmation.state(start + 30ms) == StopConfirmationState::Confirmed,
          "fresh exact driver acceptance must confirm map-free teleop stop without odometry");
  const auto diagnostics = confirmation.diagnostics();
  require(!diagnostics.quiet_odometry_required &&
              diagnostics.required_quiet_odometry_samples == 0U &&
              diagnostics.odometry_samples_observed == 0U,
          "ACK-only diagnostics must expose that odometry was not part of the proof");

  StopConfirmation rejected("endpoint-a", 13, stampNs(400), start, config);
  rejected.observeDriverAck("endpoint-a", 13, false, stampNs(401));
  require(rejected.state(start + 30ms) == StopConfirmationState::DriverRejected,
          "ACK-only policy must still fail immediately on exact driver rejection");
}

void testFreshZerosConfirmAfterDriverDropsOutputsDuringReconnect() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{60s};
  const StopConfirmationConfig config{
      4000ms, 0.03, 0.08, 8, StopConfirmationEvidencePolicy::DriverAckOnly};
  StopConfirmation confirmation("endpoint-a", 100, stampNs(500), start, config);
  StopConfirmation original_single_zero("endpoint-a", 100, stampNs(500), start, config);

  // The driver consumes and drops outputs while its three-second reconnect
  // delay runs. Its acquisition zero has no nav output identity and is no ACK.
  for (std::uint64_t tick = 1; tick <= 150; ++tick) {
    confirmation.observePublishedZero(100 + tick, stampNs(500, tick * 20000000));
    confirmation.observeDriverAck("", 0, false, stampNs(500, tick * 20000000 + 1000));
    require(confirmation.state(start + 20ms * tick) == StopConfirmationState::Pending,
            "a disconnected driver or acquisition zero must not confirm the stop");
  }
  confirmation.observePublishedZero(251, stampNs(503, 20000000));
  confirmation.observeDriverAck("endpoint-a", 251, true, stampNs(503, 21000000));
  require(confirmation.state(start + 3021ms) == StopConfirmationState::Confirmed,
          "the next fresh zero must confirm after driver reconnect drops the first zero");
  original_single_zero.observeDriverAck("endpoint-a", 251, true, stampNs(503, 21000000));
  require(original_single_zero.state(start + 4000ms) == StopConfirmationState::TimedOut,
          "the original single-zero behavior must reproduce the missing exact ACK timeout");
  const auto diagnostics = confirmation.diagnostics();
  require(diagnostics.driver_ack_output_sequence == 251U &&
              diagnostics.zero_published_source_wall_ns == stampNs(503, 20000000),
          "stop evidence must identify the exact zero acknowledged after reconnect");
}

void testOnlyRegisteredZerosCanConfirmAndRefreshCannotExtendDeadline() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{70s};
  const StopConfirmationConfig config{
      500ms, 0.03, 0.08, 8, StopConfirmationEvidencePolicy::DriverAckOnly};
  StopConfirmation confirmation("endpoint-a", 42, stampNs(600), start, config);
  confirmation.observePublishedZero(44, stampNs(600, 20000000));
  confirmation.observeDriverAck("endpoint-a", 43, true, stampNs(600, 30000000));
  confirmation.observeDriverAck("endpoint-a", 45, true, stampNs(600, 30000000));
  confirmation.observeDriverAck("endpoint-b", 44, true, stampNs(600, 30000000));
  confirmation.observeDriverAck("endpoint-a", 44, true, stampNs(600, 10000000));
  require(confirmation.state(start + 30ms) == StopConfirmationState::Pending,
          "nonzero/unpublished tokens, wrong producers and pre-publication ACKs must not confirm");

  confirmation.observePublishedZero(46, stampNs(600, 40000000));
  confirmation.observeDriverAck("endpoint-a", 44, true, stampNs(600, 30000000));
  require(confirmation.state(start + 50ms) == StopConfirmationState::Confirmed,
          "a delayed exact ACK must remain valid when the next zero has already been published");
  confirmation.observePublishedZero(47, stampNs(600, 490000000));
  confirmation.observeDriverAck("endpoint-a", 47, true, stampNs(600, 495000000));
  require(confirmation.state(start + 500ms) == StopConfirmationState::TimedOut,
          "publishing fresh zeros must never extend the original confirmation deadline");

  StopConfirmation rejected("endpoint-a", 42, stampNs(600), start, config);
  rejected.observePublishedZero(44, stampNs(600, 20000000));
  rejected.observeDriverAck("endpoint-a", 44, false, stampNs(600, 30000000));
  require(rejected.state(start + 30ms) == StopConfirmationState::DriverRejected,
          "an exact rejection for a refreshed zero must still fail closed immediately");
}

void testRefreshedZeroAcksPreservePostAckQuietWindow() {
  using namespace std::chrono_literals;
  const auto start = StopConfirmation::Clock::time_point{80s};
  StopConfirmation confirmation("endpoint-a", 42, stampNs(700), start,
                                {1000ms, 0.03, 0.08, 3});
  observeQuietStop(confirmation, 10.0);
  confirmation.observePublishedZero(43, stampNs(700, 20000000));
  confirmation.observeDriverAck("endpoint-a", 43, true, stampNs(700, 21000000));
  require(confirmation.state(start + 30ms) == StopConfirmationState::Pending,
          "a refreshed zero ACK must not reuse odometry sampled before ACK");
  for (std::uint64_t tick = 1; tick <= 3; ++tick) {
    confirmation.observePublishedZero(43 + tick, stampNs(700, (tick + 1) * 20000000));
    confirmation.observeDriverAck("endpoint-a", 43 + tick, true,
                                  stampNs(700, (tick + 1) * 20000000 + 1000));
    confirmation.observeQuietOdometry(11.0 + 0.02 * tick, 0.0, 0.0);
  }
  require(confirmation.state(start + 90ms) == StopConfirmationState::Confirmed,
          "continued zero acceptance must not reset the post-ACK quiet sample window");
}

}  // namespace

int main() {
  try {
    testRejectsMismatchedAndStaleDriverAcks();
    testUsesExactZeroCommandStampNotLaterWallRead();
    testRequiresMatchingDriverAckFollowedByQuietOdometry();
    testOdometryClockMayLagTheDriverWallClock();
    testMotionResetsQuietSampleWindow();
    testDriverRejectionAndTimeoutFailClosed();
    testDriverAckOnlyPolicyIsExplicitAndStillRequiresFreshExactAcceptance();
    testFreshZerosConfirmAfterDriverDropsOutputsDuringReconnect();
    testOnlyRegisteredZerosCanConfirmAndRefreshCannotExtendDeadline();
    testRefreshedZeroAcksPreservePostAckQuietWindow();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_stop_confirmation: FAIL: %s\n", exc.what());
    return 1;
  }
}
