#include "endpoint/goal_command_lane.hpp"

#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

ExploreGoalCommandLane::ExploreGoalCommandLane(double retry_s, double command_timeout_s)
    : retry_s_(retry_s), command_timeout_s_(command_timeout_s) {
  if (retry_s_ <= 0.0 || command_timeout_s_ <= 0.0) {
    throw std::invalid_argument("explore goal command timing must be positive");
  }
}

void ExploreGoalCommandLane::beginGoal(std::string task_id, std::string start_request_id,
                                       ExploreGoalTarget target, double now_s) {
  if (task_id.empty() || start_request_id.empty()) {
    throw std::invalid_argument("explore goal task and start request ids are required");
  }
  binding_ = ExploreGoalCommandBinding{std::move(task_id), std::move(start_request_id), target};
  cancel_request_id_.clear();
  cancel_reason_.clear();
  last_start_write_s_.reset();
  last_cancel_write_s_.reset();
  phase_started_s_ = now_s;
  pending_deadline_.reset();
  start_written_ = false;
  start_ack_received_ = false;
  cancel_acked_ = false;
  deadline_expired_ = false;
}

bool ExploreGoalCommandLane::requestCancel(std::string reason, double now_s) {
  if (!binding_.has_value()) {
    throw std::logic_error("cannot cancel without an explore goal");
  }
  if (!cancel_request_id_.empty()) {
    return true;
  }
  if (!start_written_) {
    finishGoal();
    return false;
  }
  cancel_request_id_ = binding_->start_request_id + "-cancel";
  cancel_reason_ = reason.empty() ? "cancel" : std::move(reason);
  phase_started_s_ = now_s;
  pending_deadline_.reset();
  deadline_expired_ = false;
  return true;
}

void ExploreGoalCommandLane::finishGoal() noexcept {
  binding_.reset();
  cancel_request_id_.clear();
  cancel_reason_.clear();
  last_start_write_s_.reset();
  last_cancel_write_s_.reset();
  phase_started_s_ = 0.0;
  pending_deadline_.reset();
  start_written_ = false;
  start_ack_received_ = false;
  cancel_acked_ = false;
  deadline_expired_ = false;
}

std::vector<ExploreGoalCommandWrite>
ExploreGoalCommandLane::advance(double now_s, bool transport_ready,
                                const std::vector<ExploreGoalCommandAck> &acks) {
  std::vector<ExploreGoalCommandWrite> writes;
  if (!binding_.has_value()) {
    return writes;
  }

  for (const auto &ack : acks) {
    if (ack.kind == lingtu::message::NavigationCommandKind::Goal &&
        ack.task_id == binding_->task_id && ack.request_id == binding_->start_request_id) {
      start_ack_received_ = true;
    } else if (ack.kind == lingtu::message::NavigationCommandKind::TaskCancel &&
               ack.task_id == binding_->task_id && ack.request_id == cancel_request_id_) {
      cancel_acked_ = ack.accepted;
    }
  }
  const bool deadline_due = now_s - phase_started_s_ >= command_timeout_s_;
  if (!deadline_expired_ && ((cancel_request_id_.empty() && !start_ack_received_ && deadline_due) ||
                             (!cancel_request_id_.empty() && deadline_due))) {
    deadline_expired_ = true;
    pending_deadline_ = cancel_request_id_.empty() ? ExploreGoalCommandDeadline::StartAck
                                                   : ExploreGoalCommandDeadline::CancelTerminal;
  }
  if (deadline_expired_) {
    return writes;
  }
  if (!transport_ready) {
    return writes;
  }

  if (cancel_request_id_.empty()) {
    const bool start_due = !start_ack_received_ && (!last_start_write_s_.has_value() ||
                                                    now_s - *last_start_write_s_ >= retry_s_);
    if (start_due) {
      writes.push_back({binding_->task_id,
                        binding_->start_request_id,
                        lingtu::message::NavigationCommandKind::Goal,
                        binding_->target,
                        {}});
      last_start_write_s_ = now_s;
    }
    return writes;
  }

  const bool cancel_due = !cancel_acked_ && (!last_cancel_write_s_.has_value() ||
                                             now_s - *last_cancel_write_s_ >= retry_s_);
  if (cancel_due) {
    writes.push_back({binding_->task_id,
                      cancel_request_id_,
                      lingtu::message::NavigationCommandKind::TaskCancel,
                      {},
                      cancel_reason_});
    last_cancel_write_s_ = now_s;
  }
  return writes;
}

std::optional<ExploreGoalCommandDeadline> ExploreGoalCommandLane::takeDeadline() noexcept {
  auto deadline = pending_deadline_;
  pending_deadline_.reset();
  return deadline;
}

void ExploreGoalCommandLane::recordWriteResult(const std::string &request_id,
                                               bool written) noexcept {
  if (!binding_.has_value()) {
    return;
  }
  if (request_id == binding_->start_request_id) {
    if (written) {
      start_written_ = true;
    } else {
      last_start_write_s_.reset();
    }
  } else if (!cancel_request_id_.empty() && request_id == cancel_request_id_ && !written) {
    last_cancel_write_s_.reset();
  }
}

const std::optional<ExploreGoalCommandBinding> &ExploreGoalCommandLane::binding() const noexcept {
  return binding_;
}

const std::string &ExploreGoalCommandLane::cancelRequestId() const noexcept {
  return cancel_request_id_;
}

}  // namespace lingtu::nav::endpoint
