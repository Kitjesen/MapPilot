#include "explore/explore_control.hpp"

#include <cmath>
#include <utility>

namespace lingtu::nav::endpoint {

ExploreControl::ExploreControl(std::size_t cache_limit)
    : cache_limit_(cache_limit == 0U ? 1U : cache_limit) {}

void ExploreControl::Remember(const std::string &request_id, const AckRecord &record) {
  if (request_id.empty()) {
    return;
  }
  if (ack_cache_.find(request_id) == ack_cache_.end()) {
    ack_order_.push_back(request_id);
  }
  ack_cache_[request_id] = record;
  while (ack_order_.size() > cache_limit_) {
    ack_cache_.erase(ack_order_.front());
    ack_order_.pop_front();
  }
}

void ExploreControl::RecordIntentRevision(const std::string &request_id, std::uint64_t revision) {
  const auto cached = ack_cache_.find(request_id);
  if (cached != ack_cache_.end()) {
    cached->second.intent_revision = revision;
  }
}
void ExploreControl::RecordIntentOutcome(const std::string &request_id, bool accepted,
                                         const std::string &reason, std::uint64_t revision) {
  const auto cached = ack_cache_.find(request_id);
  if (cached != ack_cache_.end()) {
    cached->second.accepted = accepted;
    cached->second.reason = reason;
    cached->second.intent_revision = revision;
  }
}

ExplorationControlResult ExploreControl::Finish(const ExplorationControlRequest &request,
                                                bool accepted, std::string reason,
                                                ExplorationControlResult actions) {
  actions.accepted = accepted;
  actions.reason = std::move(reason);
  actions.session_id = session_id_;
  Remember(request.request_id, AckRecord{
                                   request.kind,
                                   actions.accepted,
                                   actions.reason,
                                   actions.session_id,
                                   actions.intent_revision,
                               });
  return actions;
}

ExplorationControlResult ExploreControl::Apply(const ExplorationControlRequest &request) {
  if (request.request_id.empty()) {
    return Finish(request, false, "exploration_request_id_empty");
  }

  const auto cached = ack_cache_.find(request.request_id);
  if (cached != ack_cache_.end()) {
    ExplorationControlResult duplicate;
    duplicate.duplicate = true;
    duplicate.session_id = cached->second.session_id;
    duplicate.intent_revision = cached->second.intent_revision;
    if (cached->second.kind != request.kind) {
      duplicate.accepted = false;
      duplicate.reason = "duplicate_request_id_kind_mismatch";
    } else {
      duplicate.accepted = cached->second.accepted;
      duplicate.reason = cached->second.reason;
    }
    return duplicate;
  }

  if (!lingtu::message::isKnownExplorationCommandKind(request.kind)) {
    return Finish(request, false, "unknown_exploration_command");
  }
  if (request.frame_id != "map") {
    return Finish(request, false, "exploration_command_frame_must_be_map");
  }
  if (!std::isfinite(request.stamp_s) || !std::isfinite(request.now_s) ||
      !std::isfinite(request.max_age_s) || !std::isfinite(request.future_tolerance_s) ||
      request.max_age_s <= 0.0 || request.future_tolerance_s < 0.0 ||
      request.now_s - request.stamp_s > request.max_age_s ||
      request.stamp_s - request.now_s > request.future_tolerance_s) {
    return Finish(request, false, "exploration_command_stamp_stale");
  }
  if (request.request_id.size() > 128U || request.session_id.size() > 128U ||
      request.reason.size() > 256U) {
    return Finish(request, false, "exploration_command_field_too_long");
  }

  const auto kind = static_cast<lingtu::message::ExplorationCommandKind>(request.kind);
  switch (kind) {
    case lingtu::message::ExplorationCommandKind::kStart: {
      if (active_) {
        if (!request.session_id.empty() && request.session_id != session_id_) {
          return Finish(request, false, "exploration_session_conflict");
        }
        return Finish(request, true, "exploration_already_active");
      }
      if (request.goal_pending || request.cancellation_pending) {
        return Finish(request, false, "exploration_stop_in_progress");
      }
      if (!request.inputs_ready) {
        return Finish(request, false, "exploration_inputs_not_ready");
      }
      active_ = true;
      paused_ = false;
      session_id_ = request.session_id.empty() ? request.request_id : request.session_id;
      ExplorationControlResult actions;
      actions.reset_planner = true;
      actions.clear_queue = true;
      actions.clear_history = true;
      actions.clear_directed_target = true;
      return Finish(request, true, "exploration_started", std::move(actions));
    }
    case lingtu::message::ExplorationCommandKind::kPause: {
      if (!active_) {
        return Finish(request, false, "exploration_not_active");
      }
      if (paused_) {
        return Finish(request, true, "exploration_already_paused");
      }
      paused_ = true;
      ExplorationControlResult actions;
      actions.clear_queue = true;
      actions.request_cancel = request.goal_pending;
      actions.cancel_reason = "exploration_paused";
      return Finish(request, true, "exploration_paused", std::move(actions));
    }
    case lingtu::message::ExplorationCommandKind::kResume:
      if (!active_) {
        return Finish(request, false, "exploration_not_active");
      }
      if (!paused_) {
        return Finish(request, true, "exploration_already_running");
      }
      if (!request.inputs_ready) {
        return Finish(request, false, "exploration_inputs_not_ready");
      }
      paused_ = false;
      return Finish(request, true, "exploration_resumed");
    case lingtu::message::ExplorationCommandKind::kStop: {
      if (!active_ && !paused_ && !request.goal_pending && !request.cancellation_pending) {
        return Finish(request, true, "exploration_already_stopped");
      }
      active_ = false;
      paused_ = false;
      ExplorationControlResult actions;
      actions.reset_planner = true;
      actions.clear_queue = true;
      actions.clear_history = true;
      actions.clear_directed_target = true;
      actions.request_cancel = request.goal_pending;
      actions.cancel_reason = request.reason.empty() ? "exploration_stopped" : request.reason;
      const std::string reason = request.goal_pending || request.cancellation_pending
                                     ? "exploration_stop_in_progress"
                                     : "exploration_stopped";
      return Finish(request, true, reason, std::move(actions));
    }
    case lingtu::message::ExplorationCommandKind::kSetDirectedTarget: {
      if (!active_) {
        return Finish(request, false, "exploration_not_active");
      }
      if (request.session_id.empty() || request.session_id != session_id_) {
        return Finish(request, false, "directed_target_session_mismatch");
      }
      if (!request.snapshot_ready) {
        return Finish(request, false, "exploration_snapshot_not_ready");
      }
      if (!request.has_directed_target || !std::isfinite(request.directed_target_x) ||
          !std::isfinite(request.directed_target_y) ||
          std::abs(request.directed_target_x) > 1'000'000.0 ||
          std::abs(request.directed_target_y) > 1'000'000.0 ||
          !std::isfinite(request.directed_target_ttl_s) ||
          !std::isfinite(request.max_directed_target_ttl_s) ||
          request.directed_target_ttl_s <= 0.0 || request.max_directed_target_ttl_s <= 0.0 ||
          request.directed_target_ttl_s > request.max_directed_target_ttl_s) {
        return Finish(request, false, "directed_target_invalid");
      }
      ExplorationControlResult actions;
      actions.set_directed_target = true;
      actions.clear_queue = true;
      actions.request_cancel = request.goal_pending;
      actions.cancel_reason = "directed_exploration_retarget";
      return Finish(request, true, "directed_exploration_target_set", std::move(actions));
    }
    case lingtu::message::ExplorationCommandKind::kClearDirectedTarget: {
      if (!active_) {
        return Finish(request, false, "exploration_not_active");
      }
      if (request.session_id.empty() || request.session_id != session_id_) {
        return Finish(request, false, "directed_target_session_mismatch");
      }
      ExplorationControlResult actions;
      actions.clear_directed_target = true;
      actions.clear_queue = true;
      actions.request_cancel = request.goal_pending;
      actions.cancel_reason = "directed_exploration_target_cleared";
      return Finish(request, true, "directed_exploration_target_cleared", std::move(actions));
    }
  }
  return Finish(request, false, "unknown_exploration_command");
}

}  // namespace lingtu::nav::endpoint
