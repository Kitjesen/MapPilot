#include <cstdio>
#include <stdexcept>
#include <string>

#include "motion/control_authority.hpp"
#include "motion/operator_motion_authority.hpp"

namespace {
using lingtu::nav::endpoint::ControlAuthority;
using lingtu::nav::endpoint::OperatorMotionAuthority;
using lingtu::nav::endpoint::OperatorMotionClaim;
using lingtu::nav::endpoint::OperatorMotionReceipt;
using lingtu::nav::endpoint::OperatorMotionSample;

void require(bool ok, const char *message) {
  if (!ok) {
    throw std::runtime_error(message);
  }
}

OperatorMotionClaim claim(std::string source_id, std::uint64_t epoch, std::uint64_t sequence = 1) {
  return {std::move(source_id), epoch, sequence, 10.0, 1.0};
}

OperatorMotionSample sample(std::string source_id, std::uint64_t epoch, std::uint64_t sequence) {
  OperatorMotionSample out;
  out.source_id = std::move(source_id);
  out.epoch = epoch;
  out.sequence = sequence;
  out.source_stamp_wall_s = 10.01;
  out.deadline_wall_s = 10.20;
  out.deadman = true;
  out.command = {0.2, 0.0, 0.1};
  return out;
}

OperatorMotionSample freshSample(std::string source_id, std::uint64_t epoch, std::uint64_t sequence,
                                 double receive_wall_s) {
  auto out = sample(std::move(source_id), epoch, sequence);
  out.source_stamp_wall_s = receive_wall_s;
  out.deadline_wall_s = receive_wall_s + 0.20;
  return out;
}

OperatorMotionReceipt submitSample(OperatorMotionAuthority &authority,
                                   const OperatorMotionSample &input, double receive_s) {
  return authority.submit(input, receive_s, receive_s);
}

void testOnlyActiveSourceAndEpochMaySubmitSamples() {
  OperatorMotionAuthority authority;

  const auto claimed = authority.claim(claim("board", 7));
  require(claimed.accepted, "initial claim must be accepted");

  const auto wrong_source = submitSample(authority, sample("websocket", 7, 2), 10.02);
  require(!wrong_source.accepted, "different source must not submit");
  require(wrong_source.reason == "not_active_source", "different source reason mismatch");

  const auto wrong_epoch = submitSample(authority, sample("board", 6, 2), 10.02);
  require(!wrong_epoch.accepted, "stale epoch must not submit");
  require(wrong_epoch.reason == "not_active_epoch", "stale epoch reason mismatch");

  const auto accepted = submitSample(authority, sample("board", 7, 2), 10.02);
  require(accepted.accepted, "active source+epoch sample must be accepted");

  const auto snapshot = authority.snapshot();
  require(snapshot.active_source_id == "board", "snapshot active source mismatch");
  require(snapshot.active_epoch == 7, "snapshot epoch mismatch");
  require(snapshot.last_sequence == 2, "snapshot sequence mismatch");
  require(snapshot.last_sample_sequence == 2, "snapshot sample sequence mismatch");
  require(snapshot.has_active_sample, "snapshot must expose active sample");
}

void testSamplesMustBeSequenceMonotonic() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");

  require(submitSample(authority, sample("board", 1, 2), 10.02).accepted,
          "first sample must be accepted");
  const auto replay = submitSample(authority, sample("board", 1, 2), 10.03);
  require(!replay.accepted, "same sequence must reject");
  require(replay.reason == "non_monotonic_sequence", "same sequence reason mismatch");
  const auto stale = submitSample(authority, sample("board", 1, 1), 10.03);
  require(!stale.accepted, "lower sequence must reject");
  require(stale.reason == "non_monotonic_sequence", "lower sequence reason mismatch");

  require(submitSample(authority, sample("board", 1, 3), 10.04).accepted,
          "next sequence must be accepted");
  require(authority.snapshot().last_sequence == 3, "latest sequence mismatch");
}

void testSamplesMustBeFreshAndInDeadline() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");

  auto stale = sample("board", 1, 2);
  stale.source_stamp_wall_s = 9.0;
  const auto stale_result = submitSample(authority, stale, 10.0);
  require(!stale_result.accepted, "stale sample must reject");
  require(stale_result.reason == "sample_stamp_stale", "stale reason mismatch");

  auto future = sample("board", 1, 2);
  future.source_stamp_wall_s = 10.20;
  const auto future_result = submitSample(authority, future, 10.0);
  require(!future_result.accepted, "future sample must reject");
  require(future_result.reason == "sample_stamp_future", "future reason mismatch");

  auto expired = sample("board", 1, 2);
  expired.source_stamp_wall_s = 10.0;
  expired.deadline_wall_s = 9.99;
  const auto expired_result = submitSample(authority, expired, 10.0);
  require(!expired_result.accepted, "deadline-expired sample must reject");
  require(expired_result.reason == "sample_deadline_expired", "deadline reason mismatch");

  require(!authority.snapshot().has_active_sample, "rejected samples must not update output");
}

void testDeadmanFalseIsExplicitHold() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");
  require(submitSample(authority, sample("board", 1, 2), 10.02).accepted, "moving sample failed");

  auto hold = sample("board", 1, 3);
  hold.deadman = false;
  const auto result = submitSample(authority, hold, 10.03);
  require(result.accepted, "deadman release must be processed");
  require(result.reason == "deadman_released_hold", "deadman reason mismatch");
  const auto snapshot = authority.snapshot();
  require(!snapshot.has_active_sample, "deadman release must clear active sample");
  require(snapshot.holding, "deadman release must expose hold state");
  require(snapshot.zero_required, "deadman release must require a zero barrier");
  require(snapshot.zero_barrier_pending, "deadman release must block source switches");
  require(!submitSample(authority, sample("board", 1, 4), 10.04).accepted,
          "pending zero barrier must reject new samples");
  require(authority.completeZeroBarrier(true).accepted, "confirmed zero barrier must complete");
  require(authority.snapshot().has_active_authority, "hold zero barrier must retain authority");
  require(authority.snapshot().holding, "hold zero barrier must retain holding state");
  auto resumed = sample("board", 1, 4);
  resumed.source_stamp_wall_s = 10.05;
  resumed.deadline_wall_s = 10.30;
  require(authority.submit(resumed, 10.05, 10.05).accepted,
          "same source epoch must resume after hold zero confirmation");
  require(authority.snapshot().has_active_sample, "resumed sample must become active");
}

void testReleaseRequiresZeroBarrierBeforeSourceSwitch() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");
  require(!authority.claim(claim("websocket", 1)).accepted,
          "second source must not steal authority while lease is live");

  const auto wrong_release = authority.release({"websocket", 1, 2});
  require(!wrong_release.accepted, "wrong source release must reject");
  require(wrong_release.reason == "not_active_source", "wrong release reason mismatch");

  const auto release = authority.release({"board", 1, 2});
  require(release.accepted, "active source release failed");
  require(release.reason == "released_zero_required", "release reason mismatch");
  require(authority.snapshot().has_active_authority,
          "release must keep authority until zero is confirmed");
  require(!authority.claim(claim("websocket", 1)).accepted,
          "source switch before zero confirmation must reject");
  require(!authority.completeZeroBarrier(false).accepted,
          "failed zero publication must keep barrier pending");
  require(!authority.claim(claim("websocket", 1)).accepted,
          "source switch after failed zero must reject");
  require(authority.completeZeroBarrier(true).accepted, "zero confirmation must complete release");
  require(!authority.snapshot().has_active_authority, "zero confirmation must clear authority");
  require(authority.claim(claim("websocket", 1)).accepted,
          "released authority must allow source switch");
  require(submitSample(authority, sample("websocket", 1, 2), 10.02).accepted,
          "new source sample failed");
  require(authority.snapshot().active_source_id == "websocket", "source switch mismatch");
}

void testTickExpiresSilentAuthorityIntoZeroBarrier() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");
  require(authority.tick(10.50).accepted, "live authority tick failed");

  const auto expired = authority.tick(11.61);
  require(expired.accepted, "expired authority must enter safe hold");
  require(expired.reason == "authority_lease_expired", "expiry reason mismatch");
  require(authority.snapshot().zero_required, "expiry must require zero");
  require(!authority.claim(claim("websocket", 1)).accepted,
          "expired authority must not switch source before zero");
  require(authority.completeZeroBarrier(true).accepted, "expiry zero barrier confirmation failed");
  require(authority.claim(claim("websocket", 1)).accepted,
          "source switch after expiry zero confirmation failed");
}

void testClaimChangesCannotBypassZeroBarrier() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");
  require(submitSample(authority, sample("board", 1, 2), 10.02).accepted, "sample failed");

  const auto contender = authority.claim(claim("websocket", 1));
  require(!contender.accepted, "different live source must reject");
  require(contender.reason == "authority_busy", "different source reason mismatch");
  require(!authority.snapshot().zero_barrier_pending,
          "different live source must not force active operator into zero barrier");
  require(authority.snapshot().has_active_sample,
          "different live source must not clear active sample");

  const auto stale_same_source = authority.claim(claim("board", 0));
  require(!stale_same_source.accepted, "zero epoch must reject");
  require(!authority.snapshot().zero_barrier_pending,
          "invalid same-source epoch must not force zero barrier");

  const auto same_source_new_epoch = authority.claim(claim("board", 2));
  require(!same_source_new_epoch.accepted, "epoch switch must not bypass zero");
  require(same_source_new_epoch.reason == "authority_change_requires_zero_barrier",
          "epoch switch reason mismatch");
  require(authority.snapshot().zero_barrier_pending, "epoch switch must require zero barrier");
  require(!authority.claim(claim("websocket", 1)).accepted,
          "different source must remain blocked while zero is pending");
  require(authority.completeZeroBarrier(true).accepted, "zero completion failed");
  require(authority.claim(claim("board", 2)).accepted,
          "new epoch may claim only after zero completion");
  const auto lower_same_source = authority.claim(claim("board", 1));
  require(!lower_same_source.accepted, "same-source lower epoch must reject");
  require(lower_same_source.reason == "stale_epoch", "lower epoch reason mismatch");
  require(!authority.snapshot().zero_barrier_pending,
          "same-source lower epoch must not force zero barrier");
}

void testExpiredLeaseClaimCannotBypassTick() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");

  auto late = claim("websocket", 1);
  late.now_monotonic_s = 11.01;
  const auto result = authority.claim(late);
  require(!result.accepted, "claim after expired lease must not steal authority");
  require(result.reason == "authority_lease_expired", "expired claim reason mismatch");
  require(authority.snapshot().zero_required, "expired claim must require zero");
  require(!authority.claim(late).accepted, "zero-pending expired claim must still reject");
  require(authority.completeZeroBarrier(true).accepted, "zero completion failed");
  require(authority.claim(late).accepted, "claim after zero completion failed");
}

void testLeaseTtlIsBoundedAndEpochCannotReplayAfterRelease() {
  OperatorMotionAuthority authority;
  auto too_long = claim("board", 1);
  too_long.lease_ttl_s = 5.0;
  const auto rejected_ttl = authority.claim(too_long);
  require(!rejected_ttl.accepted, "overlong lease must reject");
  require(rejected_ttl.reason == "lease_ttl_too_long", "overlong lease reason mismatch");

  require(authority.claim(claim("board", 1)).accepted, "claim failed");
  require(authority.release({"board", 1, 2}).accepted, "release failed");
  require(authority.completeZeroBarrier(true).accepted, "zero completion failed");
  const auto replay = authority.claim(claim("board", 1));
  require(!replay.accepted, "released source epoch must not replay");
  require(replay.reason == "stale_epoch", "released epoch replay reason mismatch");
  require(authority.claim(claim("board", 2)).accepted,
          "new source epoch after release must be accepted");
}

void testPerSourceEpochHighWaterSurvivesSourceSwitches() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "board claim failed");
  require(authority.release({"board", 1, 2}).accepted, "board release failed");
  require(authority.completeZeroBarrier(true).accepted, "board zero failed");
  require(authority.claim(claim("websocket", 1)).accepted, "websocket claim failed");
  require(authority.release({"websocket", 1, 2}).accepted, "websocket release failed");
  require(authority.completeZeroBarrier(true).accepted, "websocket zero failed");

  const auto old_board = authority.claim(claim("board", 1));
  require(!old_board.accepted, "old board epoch must not replay after A->B->A");
  require(old_board.reason == "stale_epoch", "old board reason mismatch");
  require(authority.claim(claim("board", 2)).accepted,
          "new board epoch must be accepted after source switch");
}

void testSourceIdLengthIsBounded() {
  OperatorMotionAuthority authority;
  const auto overlong = authority.claim(claim(std::string(129, 'x'), 1));
  require(!overlong.accepted, "overlong source ID must reject");
  require(overlong.reason == "source_id_too_long", "overlong source ID reason mismatch");

  const auto maximum_length = authority.claim(claim(std::string(128, 'x'), 1));
  require(maximum_length.accepted, "source ID at the maximum length must be accepted");

  const auto overlong_sample = submitSample(authority, sample(std::string(129, 'x'), 1, 2), 10.02);
  require(!overlong_sample.accepted, "overlong sample source ID must reject");
  require(overlong_sample.reason == "source_id_too_long",
          "overlong sample source ID reason mismatch");

  const auto overlong_release = authority.release({std::string(129, 'x'), 1, 2});
  require(!overlong_release.accepted, "overlong control source ID must reject");
  require(overlong_release.reason == "source_id_too_long",
          "overlong control source ID reason mismatch");
  const auto overlong_hold = authority.hold({std::string(129, 'x'), 1, 2});
  require(!overlong_hold.accepted, "overlong hold source ID must reject");
  require(overlong_hold.reason == "source_id_too_long", "overlong hold source ID reason mismatch");

  const auto maximum_sample = submitSample(authority, sample(std::string(128, 'x'), 1, 2), 10.02);
  require(maximum_sample.accepted, "maximum-length sample source ID must be accepted");
  require(authority.release({std::string(128, 'x'), 1, 3}).accepted,
          "maximum-length control source ID must be accepted");
}

void testRetainedSourceIdentityCapacityFailsClosed() {
  lingtu::nav::endpoint::OperatorMotionAuthorityOptions options;
  options.max_retained_source_identities = 2;
  OperatorMotionAuthority authority(options);

  require(authority.claim(claim("alpha", 1)).accepted, "first source claim failed");
  require(authority.release({"alpha", 1, 2}).accepted, "first source release failed");
  require(authority.completeZeroBarrier(true).accepted, "first source zero failed");
  require(authority.claim(claim("beta", 1)).accepted, "second source claim failed");
  require(authority.release({"beta", 1, 2}).accepted, "second source release failed");
  require(authority.completeZeroBarrier(true).accepted, "second source zero failed");

  const auto replay = authority.claim(claim("alpha", 1));
  require(!replay.accepted, "retained source epoch must remain replay-protected");
  require(replay.reason == "stale_epoch", "retained source replay reason mismatch");
  require(authority.claim(claim("alpha", 2)).accepted,
          "retained source with a new epoch must still be accepted");
  require(authority.claim(claim("alpha", 2, 2)).accepted,
          "active retained source must still refresh at capacity");
  require(authority.release({"alpha", 2, 3}).accepted, "retained source release failed");
  require(authority.completeZeroBarrier(true).accepted, "retained source zero failed");

  const auto new_source = authority.claim(claim("gamma", 1));
  require(!new_source.accepted, "new source must reject at identity capacity");
  require(new_source.reason == "source_identity_capacity_exhausted",
          "identity capacity rejection reason mismatch");
  require(authority.claim(claim("beta", 2)).accepted,
          "existing source must remain accepted after capacity rejection");

  auto expired_contender = claim("gamma", 1);
  expired_contender.now_monotonic_s = 11.01;
  const auto expiry = authority.claim(expired_contender);
  require(!expiry.accepted, "expired authority must not be bypassed by a capacity rejection");
  require(expiry.reason == "authority_lease_expired",
          "authority expiry must take precedence once the lease is stale");
  require(authority.snapshot().zero_barrier_pending,
          "expired authority must still enter the zero barrier at capacity");
  require(authority.completeZeroBarrier(true).accepted, "expired authority zero completion failed");
  const auto still_full = authority.claim(expired_contender);
  require(!still_full.accepted, "capacity must remain full after an existing source expires");
  require(still_full.reason == "source_identity_capacity_exhausted",
          "post-expiry capacity rejection reason mismatch");
}

void testWallClockStepsDoNotChangeLeaseLiveness() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");

  auto forward_wall = sample("board", 1, 2);
  forward_wall.source_stamp_wall_s = 1000.0;
  forward_wall.deadline_wall_s = 1000.2;
  require(authority.submit(forward_wall, 10.02, 1000.01).accepted,
          "valid wall-adjusted sample after wall step must be accepted");
  require(authority.tick(10.50).accepted, "wall step must not expire live lease");

  auto backward_wall = sample("board", 1, 3);
  backward_wall.source_stamp_wall_s = 5.0;
  backward_wall.deadline_wall_s = 5.2;
  require(authority.submit(backward_wall, 10.60, 5.01).accepted,
          "valid wall-adjusted sample after backward wall step must be accepted");

  const auto expired = authority.tick(11.61);
  require(expired.accepted, "steady timeout must enter zero barrier");
  require(expired.reason == "authority_lease_expired", "steady timeout reason mismatch");
  require(authority.snapshot().zero_barrier_pending,
          "steady timeout must require zero regardless of wall time");
}

void testFreshSamplesRefreshLeasePastInitialClaimDeadline() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");
  require(authority.snapshot().lease_until_monotonic_s == 11.0, "initial lease mismatch");

  require(authority.submit(freshSample("board", 1, 2, 100.0), 10.80, 100.0).accepted,
          "fresh sample before initial deadline failed");
  require(authority.snapshot().lease_until_monotonic_s == 11.80, "first sample must refresh lease");
  require(authority.submit(freshSample("board", 1, 3, 101.0), 11.60, 101.0).accepted,
          "fresh sample after initial deadline must still be accepted");
  require(authority.snapshot().lease_until_monotonic_s == 12.60,
          "second sample must refresh lease");

  require(authority.tick(12.50).accepted, "lease should still be live");
  require(!authority.snapshot().zero_barrier_pending, "live refreshed lease must not require zero");
  const auto expired = authority.tick(12.61);
  require(expired.accepted, "expired refreshed lease must enter zero barrier");
  require(expired.reason == "authority_lease_expired", "refreshed expiry reason mismatch");
  require(authority.snapshot().zero_barrier_pending,
          "expired refreshed lease must require zero barrier");
}

void testDeadmanFalseRefreshesLeaseBeforeHolding() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1)).accepted, "claim failed");
  auto hold = freshSample("board", 1, 2, 100.0);
  hold.deadman = false;
  require(authority.submit(hold, 10.80, 100.0).accepted, "fresh hold sample failed");
  require(authority.snapshot().lease_until_monotonic_s == 11.80,
          "hold sample must refresh lease before zero barrier");
  require(authority.completeZeroBarrier(true).accepted, "hold zero completion failed");
  require(authority.tick(11.70).accepted, "held authority should still be live");
  const auto resumed = authority.submit(freshSample("board", 1, 3, 101.0), 11.75, 101.0);
  require(resumed.accepted, "held authority must resume with fresh sample");
}

void testControlAndSamplesShareOneMonotonicSequence() {
  OperatorMotionAuthority authority;
  const auto zero_release_without_authority = authority.release({"board", 1, 0});
  require(!zero_release_without_authority.accepted,
          "zero-sequence release must reject before authority lookup");
  require(zero_release_without_authority.reason == "sequence_zero", "zero release reason mismatch");
  const auto zero_sequence = authority.claim(claim("board", 1, 0));
  require(!zero_sequence.accepted, "zero-sequence claim must reject");
  require(zero_sequence.reason == "sequence_zero", "zero claim reason mismatch");

  require(authority.claim(claim("board", 1, 1)).accepted, "initial claim failed");
  const auto repeated_claim = authority.claim(claim("board", 1, 1));
  require(!repeated_claim.accepted, "replayed claim sequence must reject");
  require(repeated_claim.reason == "non_monotonic_sequence", "replayed claim reason mismatch");
  require(authority.claim(claim("board", 1, 2)).accepted,
          "same-epoch lease refresh with a new sequence failed");
  require(!submitSample(authority, sample("board", 1, 2), 10.02).accepted,
          "sample must not reuse a control sequence");
  require(submitSample(authority, sample("board", 1, 3), 10.02).accepted,
          "sample after control high-water failed");

  const auto zero_hold = authority.hold({"board", 1, 0});
  require(!zero_hold.accepted, "zero-sequence hold must reject");
  require(zero_hold.reason == "sequence_zero", "zero hold reason mismatch");
  const auto replayed_release = authority.release({"board", 1, 3});
  require(!replayed_release.accepted, "release must not reuse a sample sequence");
  require(replayed_release.reason == "non_monotonic_sequence", "replayed release reason mismatch");
}

void testPendingBarrierDispositionCannotBeOverwritten() {
  OperatorMotionAuthority authority;
  require(authority.claim(claim("board", 1, 1)).accepted, "claim failed");
  require(authority.hold({"board", 1, 2}).accepted, "hold failed");
  const auto release_while_hold_pending = authority.release({"board", 1, 3});
  require(!release_while_hold_pending.accepted,
          "release must not overwrite a pending hold barrier");
  require(release_while_hold_pending.reason == "zero_barrier_pending",
          "pending hold override reason mismatch");
  require(authority.completeZeroBarrier(true).accepted, "hold zero failed");
  require(authority.snapshot().has_active_authority,
          "completed hold must retain authority despite rejected release");

  require(authority.release({"board", 1, 3}).accepted, "release failed");
  const auto hold_while_release_pending = authority.hold({"board", 1, 4});
  require(!hold_while_release_pending.accepted,
          "hold must not overwrite a pending release barrier");
  require(hold_while_release_pending.reason == "zero_barrier_pending",
          "pending release override reason mismatch");
  require(authority.completeZeroBarrier(true).accepted, "release zero failed");
  require(!authority.snapshot().has_active_authority,
          "completed release must clear authority despite rejected hold");
}

void testInnerTeleopRequestIsClearedBeforeOuterZeroCompletes() {
  OperatorMotionAuthority outer;
  ControlAuthority inner;
  require(outer.claim(claim("board", 1, 1)).accepted, "outer claim failed");
  require(submitSample(outer, sample("board", 1, 2), 10.02).accepted, "outer sample failed");
  require(inner.acceptTeleop({0.3, 0.0, 0.1}, 10.01), "inner teleop admission failed");
  require(inner.teleopRequest().has_value(), "inner teleop request missing");

  require(outer.hold({"board", 1, 3}).accepted, "outer hold failed");
  inner.holdOperatorTakeover();
  require(!inner.teleopRequest().has_value(),
          "inner request must be cleared before publishing the zero barrier");
  require(!inner.pathActive(), "inner path must be inactive before publishing the zero barrier");
  require(outer.completeZeroBarrier(true).accepted, "outer zero completion failed");
  require(!inner.teleopRequest().has_value(),
          "old teleop request must not revive after zero completion");
}

void testPassiveTicksDoNotOverwriteAdmissionObservability() {
  OperatorMotionAuthority authority;
  const auto idle_before = authority.snapshot();
  require(authority.tick(1.0).accepted, "idle tick failed");
  const auto idle_after = authority.snapshot();
  require(idle_after.accepted == idle_before.accepted,
          "idle ticks must not increment accepted commands");
  require(idle_after.last_reason == idle_before.last_reason,
          "idle ticks must not overwrite the last command reason");

  require(authority.claim(claim("board", 1, 1)).accepted, "claim failed");
  const auto claimed = authority.snapshot();
  require(claimed.last_reason == "claimed", "claim reason missing");
  require(authority.tick(10.5).accepted, "live tick failed");
  const auto live = authority.snapshot();
  require(live.accepted == claimed.accepted, "live ticks must not increment accepted commands");
  require(live.last_reason == "claimed", "live ticks must preserve the last admission reason");
}

}  // namespace

int main() {
  try {
    testOnlyActiveSourceAndEpochMaySubmitSamples();
    testSamplesMustBeSequenceMonotonic();
    testSamplesMustBeFreshAndInDeadline();
    testDeadmanFalseIsExplicitHold();
    testReleaseRequiresZeroBarrierBeforeSourceSwitch();
    testTickExpiresSilentAuthorityIntoZeroBarrier();
    testClaimChangesCannotBypassZeroBarrier();
    testExpiredLeaseClaimCannotBypassTick();
    testLeaseTtlIsBoundedAndEpochCannotReplayAfterRelease();
    testPerSourceEpochHighWaterSurvivesSourceSwitches();
    testSourceIdLengthIsBounded();
    testRetainedSourceIdentityCapacityFailsClosed();
    testWallClockStepsDoNotChangeLeaseLiveness();
    testFreshSamplesRefreshLeasePastInitialClaimDeadline();
    testDeadmanFalseRefreshesLeaseBeforeHolding();
    testControlAndSamplesShareOneMonotonicSequence();
    testPendingBarrierDispositionCannotBeOverwritten();
    testInnerTeleopRequestIsClearedBeforeOuterZeroCompletes();
    testPassiveTicksDoNotOverwriteAdmissionObservability();
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_operator_motion_authority: FAIL: %s\n", exc.what());
    return 1;
  }
  return 0;
}
