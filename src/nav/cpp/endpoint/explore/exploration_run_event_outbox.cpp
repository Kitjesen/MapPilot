#include "explore/exploration_run_event_outbox.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

bool validRouteBinding(const ExplorationRunEventRecord &event) noexcept {
  if (event.route == "live") {
    return event.map_version >= 0;
  }
  return event.route == "map" && !event.map_id.empty() && event.map_version > 0 &&
      !event.artifact_hash.empty();
}

}  // namespace

const char *ExplorationRunEventOutboxRecordResultName(
    ExplorationRunEventOutboxRecordResult result) noexcept {
  switch (result) {
    case ExplorationRunEventOutboxRecordResult::kAccepted:
      return "accepted";
    case ExplorationRunEventOutboxRecordResult::kInvalid:
      return "invalid";
    case ExplorationRunEventOutboxRecordResult::kBackpressure:
      return "backpressure";
  }
  return "unknown";
}

ExplorationRunEventOutbox::ExplorationRunEventOutbox(std::string boot_id, WriteCallback write,
                                                     std::size_t capacity)
    : boot_id_(std::move(boot_id)), write_(std::move(write)), capacity_(capacity) {
  if (boot_id_.empty()) {
    throw std::invalid_argument("exploration run event outbox boot_id is required");
  }
  if (!write_) {
    throw std::invalid_argument("exploration run event outbox write callback is required");
  }
  if (capacity_ == 0U) {
    throw std::invalid_argument("exploration run event outbox capacity must be positive");
  }
}

bool ExplorationRunEventOutbox::canRecord(std::size_t count) const noexcept {
  return count <= capacity_ - pending_.size();
}

ExplorationRunEventOutboxRecordResult ExplorationRunEventOutbox::record(
    const ExplorationRunEventRecord &event) {
  if (!valid(event) || next_sequence_ == 0U) {
    ++diagnostics_.rejected_invalid;
    return ExplorationRunEventOutboxRecordResult::kInvalid;
  }
  if (!canRecord()) {
    ++diagnostics_.rejected_backpressure;
    return ExplorationRunEventOutboxRecordResult::kBackpressure;
  }

  pending_.push_back(ExplorationRunEventEnvelope{boot_id_, next_sequence_, event});
  if (next_sequence_ == std::numeric_limits<std::uint64_t>::max()) {
    next_sequence_ = 0U;
  } else {
    ++next_sequence_;
  }
  ++diagnostics_.accepted;
  diagnostics_.pending = pending_.size();
  return ExplorationRunEventOutboxRecordResult::kAccepted;
}

std::size_t ExplorationRunEventOutbox::flush(std::size_t max_events) {
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

ExplorationRunEventOutboxDiagnostics ExplorationRunEventOutbox::diagnostics() const noexcept {
  return diagnostics_;
}

bool ExplorationRunEventOutbox::valid(const ExplorationRunEventRecord &event) noexcept {
  const auto kind_value = static_cast<std::int32_t>(event.kind);
  const auto state_value = static_cast<std::int32_t>(event.state);
  if (!std::isfinite(event.timestamp_s) || event.timestamp_s <= 0.0 || event.frame_id != "map" ||
      !lingtu::message::isKnownExplorationRunEventKind(kind_value) ||
      !lingtu::message::isKnownExplorationRunState(state_value) ||
      !lingtu::message::isValidExplorationRunId(event.exploration_run_id) ||
      event.start_request_id.empty() || event.command_request_id.empty() ||
      event.product_session_id.empty() || event.reason.empty() || !validRouteBinding(event)) {
    return false;
  }

  const bool admitted_kind = event.kind == lingtu::message::ExplorationRunEventKind::kAdmitted;
  const bool admitted_state = event.state == lingtu::message::ExplorationRunState::kAdmitted;
  if (admitted_kind != admitted_state) {
    return false;
  }
  if (event.kind == lingtu::message::ExplorationRunEventKind::kStopConfirmationFailed &&
      event.state != lingtu::message::ExplorationRunState::kPausing &&
      event.state != lingtu::message::ExplorationRunState::kCancelling) {
    return false;
  }
  if (event.motion_stop_confirmed && event.motion_stop_reason.empty()) {
    return false;
  }
  if (event.kind == lingtu::message::ExplorationRunEventKind::kStopConfirmationFailed) {
    if (event.motion_stop_confirmed || event.motion_stop_reason.empty()) {
      return false;
    }
  } else if (!event.motion_stop_confirmed && !event.motion_stop_reason.empty()) {
    return false;
  }
  if (lingtu::message::explorationRunStateRequiresMotionStop(event.state) &&
      !event.motion_stop_confirmed) {
    return false;
  }
  if ((event.state == lingtu::message::ExplorationRunState::kPausing ||
       event.state == lingtu::message::ExplorationRunState::kCancelling) &&
      event.motion_stop_confirmed) {
    return false;
  }
  return true;
}

}  // namespace lingtu::nav::endpoint
