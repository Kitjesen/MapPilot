#include "inspection/inspection_task_event_outbox.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

const char* InspectionTaskEventOutboxRecordResultName(
    InspectionTaskEventOutboxRecordResult result) noexcept {
  switch (result) {
    case InspectionTaskEventOutboxRecordResult::kAccepted:
      return "accepted";
    case InspectionTaskEventOutboxRecordResult::kInvalid:
      return "invalid";
    case InspectionTaskEventOutboxRecordResult::kOutOfOrder:
      return "out_of_order";
    case InspectionTaskEventOutboxRecordResult::kBackpressure:
      return "backpressure";
  }
  return "unknown";
}

InspectionTaskEventOutbox::InspectionTaskEventOutbox(std::string boot_id, WriteCallback write,
                                                     std::size_t capacity)
    : boot_id_(std::move(boot_id)), write_(std::move(write)), capacity_(capacity) {
  if (boot_id_.empty()) {
    throw std::invalid_argument("inspection task event outbox boot_id is required");
  }
  if (!write_) {
    throw std::invalid_argument("inspection task event outbox write callback is required");
  }
  if (capacity_ == 0U) {
    throw std::invalid_argument("inspection task event outbox capacity must be positive");
  }
}

InspectionTaskEventOutboxRecordResult InspectionTaskEventOutbox::record(
    const lingtu::nav::inspection::TaskEvent& event) {
  if (!valid(event)) {
    ++diagnostics_.rejected_invalid;
    return InspectionTaskEventOutboxRecordResult::kInvalid;
  }
  if (event.sequence != last_accepted_sequence_ + 1U) {
    ++diagnostics_.rejected_out_of_order;
    return InspectionTaskEventOutboxRecordResult::kOutOfOrder;
  }
  if (pending_.size() >= capacity_) {
    ++diagnostics_.rejected_backpressure;
    return InspectionTaskEventOutboxRecordResult::kBackpressure;
  }

  pending_.push_back(InspectionTaskEventEnvelope{boot_id_, event.sequence, event});
  last_accepted_sequence_ = event.sequence;
  ++diagnostics_.accepted;
  diagnostics_.pending = pending_.size();
  return InspectionTaskEventOutboxRecordResult::kAccepted;
}

std::size_t InspectionTaskEventOutbox::flush(std::size_t max_events) {
  std::size_t delivered = 0U;
  while (delivered < max_events && !pending_.empty()) {
    bool wrote = false;
    try {
      wrote = write_(pending_.front());
    } catch (...) {
      wrote = false;
    }
    if (!wrote) {
      ++diagnostics_.delivery_failures;
      break;
    }
    pending_.pop_front();
    ++delivered;
    ++diagnostics_.delivered;
  }
  diagnostics_.pending = pending_.size();
  return delivered;
}

InspectionTaskEventOutboxDiagnostics InspectionTaskEventOutbox::diagnostics() const noexcept {
  return diagnostics_;
}

bool InspectionTaskEventOutbox::valid(const lingtu::nav::inspection::TaskEvent& event) noexcept {
  const auto& status = event.status;
  return event.sequence != 0U && std::isfinite(event.timestamp_s) && !event.request_id.empty() &&
      !status.task_id.empty() && !status.run_id.empty() && !status.request_id.empty() &&
      !status.map_id.empty() && !status.route_id.empty() && status.route_revision != 0U &&
      validKind(event.kind) && validState(status.state);
}

bool InspectionTaskEventOutbox::validKind(
    lingtu::nav::inspection::TaskEventKind kind) noexcept {
  using lingtu::nav::inspection::TaskEventKind;
  switch (kind) {
    case TaskEventKind::kTaskAccepted:
    case TaskEventKind::kStateChanged:
    case TaskEventKind::kMilestone:
    case TaskEventKind::kStopConfirmationFailed:
    case TaskEventKind::kEvidenceRecorded:
      return true;
  }
  return false;
}

bool InspectionTaskEventOutbox::validState(
    lingtu::nav::inspection::RunState state) noexcept {
  using lingtu::nav::inspection::RunState;
  switch (state) {
    case RunState::kIdle:
    case RunState::kValidating:
    case RunState::kPlanning:
    case RunState::kNavigating:
    case RunState::kDwelling:
    case RunState::kPaused:
    case RunState::kRecovering:
    case RunState::kSucceeded:
    case RunState::kFailed:
    case RunState::kCancelled:
    case RunState::kSettling:
    case RunState::kActionPending:
    case RunState::kPausing:
    case RunState::kCancelling:
      return true;
  }
  return false;
}

}  // namespace lingtu::nav::endpoint
