#include <cstdint>
#include <cstdio>
#include <stdexcept>

#include "control/operator.hpp"

namespace {

using lingtu::nav::endpoint::OperatorMotionReceipt;
using lingtu::nav::endpoint::OperatorMotionSnapshot;
using lingtu::nav::endpoint::updateOperatorMotionOutputEvidence;

void require(bool ok, const char *message) {
  if (!ok) {
    throw std::runtime_error(message);
  }
}

void testHoldZeroSequenceBecomesDurableStatusEvidence() {
  constexpr std::uint64_t moving_output_sequence = 41U;
  constexpr std::uint64_t hold_zero_sequence = moving_output_sequence + 1U;

  std::uint64_t admitted_sequence = 7U;
  std::uint64_t status_final_output_sequence = moving_output_sequence;
  OperatorMotionReceipt completion{true, "hold_zero_published", false, true};
  OperatorMotionSnapshot snapshot;
  snapshot.has_active_authority = true;
  snapshot.holding = true;
  const nav_kernel::Twist final_command{};

  const auto ack_final_output_sequence =
      updateOperatorMotionOutputEvidence(completion, snapshot, hold_zero_sequence, final_command,
                                         admitted_sequence, status_final_output_sequence);

  require(ack_final_output_sequence == hold_zero_sequence,
          "hold ACK must report the sequenced zero output");
  require(status_final_output_sequence == ack_final_output_sequence,
          "durable status evidence must match the hold ACK sequence");
  require(status_final_output_sequence == moving_output_sequence + 1U,
          "hold zero must supersede the moving output sequence");
  require(admitted_sequence == 7U,
          "authority-retaining hold must preserve admitted sample evidence");
  require(final_command.vx == 0.0 && final_command.vy == 0.0 && final_command.wz == 0.0,
          "hold status must expose zero final velocity");
}

void testClearedAuthorityResetsDurableEvidence() {
  std::uint64_t admitted_sequence = 7U;
  std::uint64_t status_final_output_sequence = 41U;
  OperatorMotionReceipt completion{true, "release_zero_published", true, true};
  OperatorMotionSnapshot snapshot;
  snapshot.has_active_authority = false;

  const auto ack_final_output_sequence =
      updateOperatorMotionOutputEvidence(completion, snapshot, 42U, nav_kernel::Twist{},
                                         admitted_sequence, status_final_output_sequence);

  require(ack_final_output_sequence == 42U, "release ACK must retain its sequenced zero output");
  require(admitted_sequence == 0U, "authority-clearing zero barrier must reset admitted evidence");
  require(status_final_output_sequence == 0U,
          "authority-clearing zero barrier must reset durable output evidence");
}

void testFailedBarrierPreservesPriorEvidence() {
  std::uint64_t admitted_sequence = 7U;
  std::uint64_t status_final_output_sequence = 41U;
  OperatorMotionReceipt completion{false, "zero_publish_failed", false, false};
  OperatorMotionSnapshot snapshot;
  snapshot.has_active_authority = true;

  const auto ack_final_output_sequence =
      updateOperatorMotionOutputEvidence(completion, snapshot, 42U, nav_kernel::Twist{},
                                         admitted_sequence, status_final_output_sequence);

  require(ack_final_output_sequence == 0U,
          "failed zero barrier must not claim an ACK output sequence");
  require(admitted_sequence == 7U, "failed zero barrier must preserve admitted evidence");
  require(status_final_output_sequence == 41U,
          "failed zero barrier must preserve prior durable output evidence");
}

}  // namespace

int main() {
  try {
    testHoldZeroSequenceBecomesDurableStatusEvidence();
    testClearedAuthorityResetsDurableEvidence();
    testFailedBarrierPreservesPriorEvidence();
    std::puts("test_operator_motion_output_evidence: PASS");
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_operator_motion_output_evidence: FAIL: %s\n", exc.what());
    return 1;
  }
}
