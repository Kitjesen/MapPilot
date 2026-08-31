#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav/inspection/inspection.hpp"

namespace {

using lingtu::nav::inspection::RunState;
using lingtu::nav::inspection::TaskEvent;
using lingtu::nav::inspection::TaskEventEnvelope;
using lingtu::nav::inspection::TaskEventKind;
using lingtu::nav::inspection::InspectionTaskEventOutbox;
using lingtu::nav::inspection::TaskEventOutboxRecordResult;

void require(bool condition, const char* message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

TaskEvent event(std::uint64_t sequence, std::string task_id, std::string request_id,
                TaskEventKind kind = TaskEventKind::kStateChanged,
                RunState state = RunState::kPlanning) {
  TaskEvent value;
  value.sequence = sequence;
  value.timestamp_s = 100.0 + static_cast<double>(sequence);
  value.kind = kind;
  value.request_id = std::move(request_id);
  value.status.task_id = std::move(task_id);
  value.status.run_id = value.status.task_id;
  value.status.request_id = value.request_id;
  value.status.map_id = "factory";
  value.status.map_content_epoch = 7;
  value.status.route_id = "round-a";
  value.status.route_revision = 2U;
  value.status.state = state;
  return value;
}

void testDeliveryRetainsFailedHeadAndPreservesEventIdentity() {
  bool writable = false;
  std::vector<TaskEventEnvelope> writes;
  InspectionTaskEventOutbox outbox(
      "endpoint-boot-42",
      [&](const TaskEventEnvelope& value) {
        writes.push_back(value);
        return writable;
      });

  require(outbox.Record(event(1U, "inspection-task-42", "request-start-42")) ==
              TaskEventOutboxRecordResult::kAccepted,
          "first event must be accepted");
  require(outbox.Record(event(2U, "inspection-task-42", "request-pause-42")) ==
              TaskEventOutboxRecordResult::kAccepted,
          "second event must be accepted");

  require(outbox.Flush() == 0U, "failed DDS head write must remain pending");
  require(writes.size() == 1U && writes.front().sequence == 1U,
          "failed head event must be attempted first");
  require(outbox.diagnostics().pending == 2U,
          "failed head must block later event delivery");

  writable = true;
  require(outbox.Flush() == 2U, "retry must deliver both retained events");
  require(writes.size() == 3U, "retry must retry unchanged head before later event");
  require(writes[0].sequence == 1U && writes[1].sequence == 1U &&
              writes[2].sequence == 2U,
          "DDS delivery must preserve strict event order across retry");
  for (const auto& write : writes) {
    require(write.boot_id == "endpoint-boot-42", "event envelope must retain boot identity");
    require(write.event.status.task_id == "inspection-task-42",
            "event envelope must retain task identity");
  }
  require(outbox.diagnostics().pending == 0U, "all delivered events must leave the outbox");
}

void testOutboxFailsClosedOnInvalidOutOfOrderAndBackpressureEvents() {
  InspectionTaskEventOutbox outbox(
      "endpoint-boot-99", [](const TaskEventEnvelope&) { return true; }, 2U);

  auto invalid = event(1U, "", "request-a");
  require(outbox.Record(invalid) == TaskEventOutboxRecordResult::kInvalid,
          "missing task identity must fail closed");
  auto nonfinite = event(1U, "task-a", "request-a");
  nonfinite.timestamp_s = std::numeric_limits<double>::quiet_NaN();
  require(outbox.Record(nonfinite) == TaskEventOutboxRecordResult::kInvalid,
          "non-finite event time must fail closed");

  require(outbox.Record(event(1U, "task-a", "request-a")) ==
              TaskEventOutboxRecordResult::kAccepted,
          "first well-formed event must be accepted");
  require(outbox.Record(event(1U, "task-a", "request-b")) ==
              TaskEventOutboxRecordResult::kOutOfOrder,
          "duplicate sequence must never be silently merged");
  require(outbox.Record(event(3U, "task-a", "request-c")) ==
              TaskEventOutboxRecordResult::kOutOfOrder,
          "a sequence gap must not hide a missing lifecycle event");
  require(outbox.Record(event(2U, "task-a", "request-d")) ==
              TaskEventOutboxRecordResult::kAccepted,
          "second ordered event must be accepted");
  require(outbox.Record(event(3U, "task-a", "request-e")) ==
              TaskEventOutboxRecordResult::kBackpressure,
          "bounded outbox must report backpressure instead of dropping an event");

  const auto diagnostics = outbox.diagnostics();
  require(diagnostics.rejected_invalid == 2U, "invalid events must be counted");
  require(diagnostics.rejected_out_of_order == 2U, "ordering violations must be counted");
  require(diagnostics.rejected_backpressure == 1U, "backpressure must be observable");
}

void testRecoveryBootCanInitializeItsFirstSequenceExactlyOnce() {
  std::vector<TaskEventEnvelope> writes;
  InspectionTaskEventOutbox outbox(
      "endpoint-recovery-boot",
      [&](const TaskEventEnvelope& value) {
        writes.push_back(value);
        return true;
      });

  require(!outbox.InitializeNextSequence(0U), "zero recovery sequence must be rejected");
  require(outbox.InitializeNextSequence(41U),
          "new boot must accept an explicit recovery start sequence");
  require(!outbox.InitializeNextSequence(40U),
          "recovery cursor must reject reinitialization and rollback");
  require(outbox.Record(event(40U, "task-recovery", "request-recovery")) ==
              TaskEventOutboxRecordResult::kOutOfOrder,
          "record before the recovery cursor must fail closed");
  require(outbox.Record(event(41U, "task-recovery", "request-recovery")) ==
              TaskEventOutboxRecordResult::kAccepted,
          "replayed recovery event must match the initialized cursor");
  require(outbox.Record(event(42U, "task-recovery", "request-next")) ==
              TaskEventOutboxRecordResult::kAccepted,
          "ordinary records after recovery must remain contiguous");
  require(outbox.Record(event(44U, "task-recovery", "request-gap")) ==
              TaskEventOutboxRecordResult::kOutOfOrder,
          "ordinary records must still reject a sequence gap");
  require(outbox.Flush() == 2U && writes.size() == 2U,
          "recovery and next event must flush in order");
  require(writes[0].sequence == 41U && writes[1].sequence == 42U,
          "recovery boot sequence order mismatch");
}

}  // namespace

int main() {
  try {
    testDeliveryRetainsFailedHeadAndPreservesEventIdentity();
    testOutboxFailsClosedOnInvalidOutOfOrderAndBackpressureEvents();
    testRecoveryBootCanInitializeItsFirstSequenceExactlyOnce();
    return 0;
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "test_inspection_task_event_outbox: FAIL: %s\\n", exc.what());
    return 1;
  }
}
