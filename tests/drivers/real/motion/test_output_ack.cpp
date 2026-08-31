#include "output_ack.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace {

using namespace std::chrono_literals;
using lingtu::driver::OutputAckState;

void check(bool value, const char* message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

void checkEmpty(
    const OutputAckState& state,
    OutputAckState::TimePoint now,
    const char* message) {
  const auto evidence = state.current(now);
  check(!evidence.accepted(), message);
  check(evidence.producerBootId().empty(), message);
  check(evidence.outputSequence() == 0, message);
}

void testDefaultIsEmpty() {
  const OutputAckState state;
  checkEmpty(state, OutputAckState::TimePoint{}, "default ACK must be empty");
}

void testAcceptedIdentityPersistsUntilInvalidated() {
  OutputAckState state;
  const auto now = OutputAckState::TimePoint{} + 1s;
  state.record("producer-a", 42, true, now);

  const auto evidence = state.current(now + 24h);
  check(evidence.accepted(), "accepted ACK must remain accepted");
  check(
      evidence.producerBootId() == "producer-a",
      "accepted ACK must preserve producer identity");
  check(
      evidence.outputSequence() == 42,
      "accepted ACK must preserve output sequence");

  state.expire(now + 48h);
  check(
      state.current(now + 48h).accepted(),
      "expire must not clear accepted ACK evidence");
  state.invalidate();
  checkEmpty(state, now + 48h, "invalidate must clear accepted ACK evidence");
}

void testRejectedIdentityExpiresAtExactBoundary() {
  OutputAckState state;
  const auto now = OutputAckState::TimePoint{} + 2s;
  state.record("producer-rejected", 73, false, now);

  const auto before = state.current(
      now + OutputAckState::kRejectedEvidenceLifetime - 1ms);
  check(!before.accepted(), "rejected ACK must remain rejected");
  check(
      before.producerBootId() == "producer-rejected",
      "rejected ACK must preserve producer identity before expiry");
  check(
      before.outputSequence() == 73,
      "rejected ACK must preserve output sequence before expiry");

  const auto boundary = now + OutputAckState::kRejectedEvidenceLifetime;
  checkEmpty(state, boundary, "rejected ACK must be empty at expiry boundary");
  state.expire(boundary);
  checkEmpty(state, boundary, "expire must clear rejected ACK state");
}

void testInvalidIdentityClearsPreviousEvidence() {
  OutputAckState state;
  const auto now = OutputAckState::TimePoint{} + 3s;
  state.record("producer-a", 1, true, now);
  state.record("", 2, false, now + 1ms);
  checkEmpty(state, now + 1ms, "empty producer must clear previous ACK");

  state.record("producer-b", 2, true, now + 2ms);
  state.record("producer-c", 0, false, now + 3ms);
  checkEmpty(state, now + 3ms, "zero sequence must clear previous ACK");
}

void testNewRecordAtomicallyReplacesIdentity() {
  OutputAckState state;
  const auto now = OutputAckState::TimePoint{} + 4s;
  state.record("producer-old", 10, true, now);
  state.record("producer-new", 11, false, now + 1ms);

  const auto evidence = state.current(now + 2ms);
  check(!evidence.accepted(), "replacement rejection must remain rejected");
  check(
      evidence.producerBootId() == "producer-new",
      "new record must replace old producer identity");
  check(
      evidence.outputSequence() == 11,
      "new record must replace old output sequence");
}

}  // namespace

int main() {
  try {
    testDefaultIsEmpty();
    testAcceptedIdentityPersistsUntilInvalidated();
    testRejectedIdentityExpiresAtExactBoundary();
    testInvalidIdentityClearsPreviousEvidence();
    testNewRecordAtomicallyReplacesIdentity();
    std::cout << "test_output_ack: PASS\n";
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "test_output_ack: FAIL: " << exc.what() << '\n';
    return 1;
  }
}
