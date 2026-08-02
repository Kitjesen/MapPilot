#include "explore/exploration_run_lifecycle.hpp"

#include <utility>

namespace lingtu::nav::endpoint {

bool ExplorationRunBinding::valid() const noexcept {
  if (!lingtu::message::isValidExplorationRunId(exploration_run_id) || start_request_id.empty() ||
      product_session_id.empty()) {
    return false;
  }
  if (route == "live") {
    return map_version >= 0;
  }
  return route == "map" && !map_id.empty() && map_version > 0 && !artifact_hash.empty();
}

bool ExplorationRunLifecycle::record(const ExplorationRunEventRecord &value) {
  return outbox_.record(value) == ExplorationRunEventOutboxRecordResult::kAccepted;
}

ExplorationRunEventRecord ExplorationRunLifecycle::event(
    lingtu::message::ExplorationRunEventKind kind, lingtu::message::ExplorationRunState state,
    const std::string &command_request_id, double timestamp_s, const std::string &reason,
    bool motion_stop_confirmed, std::string motion_stop_reason) const {
  return {
      timestamp_s,
      "map",
      kind,
      binding_.exploration_run_id,
      binding_.start_request_id,
      command_request_id,
      binding_.product_session_id,
      state,
      binding_.route,
      binding_.map_id,
      binding_.map_version,
      binding_.artifact_hash,
      reason,
      motion_stop_confirmed,
      std::move(motion_stop_reason),
  };
}

bool ExplorationRunLifecycle::start(const ExplorationRunBinding &binding, double timestamp_s,
                                    const std::string &reason) {
  if (active_ || !binding.valid() || !outbox_.canRecord(2U)) {
    return false;
  }
  binding_ = binding;
  const auto admitted = event(lingtu::message::ExplorationRunEventKind::kAdmitted,
                              lingtu::message::ExplorationRunState::kAdmitted,
                              binding.start_request_id, timestamp_s, reason);
  const auto running = event(lingtu::message::ExplorationRunEventKind::kStateChanged,
                             lingtu::message::ExplorationRunState::kRunning,
                             binding.start_request_id, timestamp_s, "exploration_running");
  if (!record(admitted) || !record(running)) {
    return false;
  }
  active_ = true;
  state_ = lingtu::message::ExplorationRunState::kRunning;
  pending_terminal_ = PendingTerminal::kNone;
  pending_command_request_id_.clear();
  pending_reason_.clear();
  return true;
}

bool ExplorationRunLifecycle::finishWithoutMotion(lingtu::message::ExplorationRunState state,
                                                  const std::string &command_request_id,
                                                  double timestamp_s,
                                                  const std::string &reason) {
  const auto terminal = event(lingtu::message::ExplorationRunEventKind::kStateChanged, state,
                              command_request_id, timestamp_s, reason, true,
                              "no_pending_motion");
  if (!record(terminal)) {
    return false;
  }
  state_ = state;
  pending_terminal_ = PendingTerminal::kNone;
  pending_command_request_id_.clear();
  pending_reason_.clear();
  if (lingtu::message::isTerminalExplorationRunState(state)) {
    active_ = false;
  }
  return true;
}

bool ExplorationRunLifecycle::pause(const std::string &command_request_id, double timestamp_s,
                                    const std::string &reason, bool motion_pending) {
  if (!active_ || state_ != lingtu::message::ExplorationRunState::kRunning ||
      command_request_id.empty()) {
    return false;
  }
  if (!motion_pending) {
    return finishWithoutMotion(lingtu::message::ExplorationRunState::kPaused, command_request_id,
                               timestamp_s, reason);
  }
  const auto pausing = event(lingtu::message::ExplorationRunEventKind::kStateChanged,
                             lingtu::message::ExplorationRunState::kPausing, command_request_id,
                             timestamp_s, reason);
  if (!record(pausing)) {
    return false;
  }
  state_ = lingtu::message::ExplorationRunState::kPausing;
  pending_terminal_ = PendingTerminal::kPaused;
  pending_command_request_id_ = command_request_id;
  pending_reason_ = reason;
  return true;
}

bool ExplorationRunLifecycle::resume(const std::string &command_request_id, double timestamp_s,
                                     const std::string &reason) {
  if (!active_ || state_ != lingtu::message::ExplorationRunState::kPaused ||
      stopConfirmationPending() || command_request_id.empty()) {
    return false;
  }
  const auto running = event(lingtu::message::ExplorationRunEventKind::kStateChanged,
                             lingtu::message::ExplorationRunState::kRunning, command_request_id,
                             timestamp_s, reason);
  if (!record(running)) {
    return false;
  }
  state_ = lingtu::message::ExplorationRunState::kRunning;
  return true;
}

bool ExplorationRunLifecycle::cancel(const std::string &command_request_id, double timestamp_s,
                                     const std::string &reason, bool motion_pending) {
  if (!active_ || command_request_id.empty()) {
    return false;
  }
  if (!motion_pending) {
    return finishWithoutMotion(lingtu::message::ExplorationRunState::kCancelled,
                               command_request_id, timestamp_s, reason);
  }
  const auto cancelling = event(lingtu::message::ExplorationRunEventKind::kStateChanged,
                                lingtu::message::ExplorationRunState::kCancelling,
                                command_request_id, timestamp_s, reason);
  if (!record(cancelling)) {
    return false;
  }
  state_ = lingtu::message::ExplorationRunState::kCancelling;
  pending_terminal_ = PendingTerminal::kCancelled;
  pending_command_request_id_ = command_request_id;
  pending_reason_ = reason;
  return true;
}

bool ExplorationRunLifecycle::complete(double timestamp_s, const std::string &reason) {
  if (!active_ || state_ != lingtu::message::ExplorationRunState::kRunning ||
      stopConfirmationPending()) {
    return false;
  }
  return finishWithoutMotion(lingtu::message::ExplorationRunState::kCompleted,
                             binding_.start_request_id, timestamp_s, reason);
}

bool ExplorationRunLifecycle::fail(double timestamp_s, const std::string &reason,
                                   bool motion_pending) {
  if (!active_) {
    return false;
  }
  if (!motion_pending) {
    return finishWithoutMotion(lingtu::message::ExplorationRunState::kFailed,
                               binding_.start_request_id, timestamp_s, reason);
  }
  const auto cancelling = event(lingtu::message::ExplorationRunEventKind::kStateChanged,
                                lingtu::message::ExplorationRunState::kCancelling,
                                binding_.start_request_id, timestamp_s, reason);
  if (!record(cancelling)) {
    return false;
  }
  state_ = lingtu::message::ExplorationRunState::kCancelling;
  pending_terminal_ = PendingTerminal::kFailed;
  pending_command_request_id_ = binding_.start_request_id;
  pending_reason_ = reason;
  return true;
}

bool ExplorationRunLifecycle::confirmMotionStop(double timestamp_s,
                                                const std::string &motion_stop_reason) {
  if (!active_ || pending_terminal_ == PendingTerminal::kNone || motion_stop_reason.empty()) {
    return false;
  }
  lingtu::message::ExplorationRunState target = lingtu::message::ExplorationRunState::kPaused;
  if (pending_terminal_ == PendingTerminal::kCancelled) {
    target = lingtu::message::ExplorationRunState::kCancelled;
  } else if (pending_terminal_ == PendingTerminal::kFailed) {
    target = lingtu::message::ExplorationRunState::kFailed;
  }
  const auto terminal = event(lingtu::message::ExplorationRunEventKind::kStateChanged, target,
                              pending_command_request_id_, timestamp_s, pending_reason_, true,
                              motion_stop_reason);
  if (!record(terminal)) {
    return false;
  }
  state_ = target;
  pending_terminal_ = PendingTerminal::kNone;
  pending_command_request_id_.clear();
  pending_reason_.clear();
  if (lingtu::message::isTerminalExplorationRunState(target)) {
    active_ = false;
  }
  return true;
}

bool ExplorationRunLifecycle::recordStopConfirmationFailure(double timestamp_s,
                                                            const std::string &reason) {
  if (!active_ || pending_terminal_ == PendingTerminal::kNone || reason.empty()) {
    return false;
  }
  auto failure = event(lingtu::message::ExplorationRunEventKind::kStopConfirmationFailed, state_,
                       pending_command_request_id_, timestamp_s, reason);
  failure.motion_stop_reason = reason;
  return record(failure);
}

}  // namespace lingtu::nav::endpoint
