#include "endpoint/control.hpp"

#include <cmath>
#include <utility>

namespace lingtu::nav::endpoint {

namespace {

bool sameSemanticDouble(double lhs, double rhs) {
  return lhs == rhs || (std::isnan(lhs) && std::isnan(rhs));
}

}  // namespace

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
  actions.exploration_run_id = request.exploration_run_id;
  actions.product_session_id = request.product_session_id;
  Remember(request.request_id, AckRecord{
                                   request.kind,
                                   request.exploration_run_id,
                                   request.product_session_id,
                                   request.has_directed_target,
                                   request.directed_target_x,
                                   request.directed_target_y,
                                   request.directed_target_ttl_s,
                                   actions.accepted,
                                   actions.reason,
                                   actions.product_session_id,
                                   actions.intent_revision,
                               });
  return actions;
}

bool ExploreControl::Complete() {
  if (!active_) {
    return false;
  }
  active_ = false;
  paused_ = false;
  return true;
}

ExplorationControlResult ExploreControl::Apply(const ExplorationControlRequest &request) {
  if (request.request_id.empty()) {
    return Finish(request, false, "exploration_request_id_empty");
  }

  const auto start_session_rejection = [&request]() -> const char * {
    if (request.product_session_id.empty()) {
      return "exploration_product_session_id_empty";
    }
    if (request.expected_product_session_id.empty()) {
      return "exploration_product_session_unverified";
    }
    if (request.product_session_id != request.expected_product_session_id) {
      return "exploration_product_session_mismatch";
    }
    return nullptr;
  };

  const auto cached = ack_cache_.find(request.request_id);
  if (cached != ack_cache_.end()) {
    ExplorationControlResult duplicate;
    duplicate.duplicate = true;
    duplicate.exploration_run_id = request.exploration_run_id;
    duplicate.product_session_id = request.product_session_id;
    duplicate.intent_revision = cached->second.intent_revision;
    if (cached->second.kind != request.kind) {
      duplicate.accepted = false;
      duplicate.reason = "duplicate_request_id_kind_mismatch";
    } else if (request.exploration_run_id != cached->second.request_exploration_run_id) {
      duplicate.accepted = false;
      duplicate.reason = "duplicate_request_id_run_mismatch";
    } else if (request.kind ==
               static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kStart)) {
      if (const char *binding_rejection = start_session_rejection(); binding_rejection != nullptr) {
        duplicate.accepted = false;
        duplicate.reason = binding_rejection;
      } else if (request.product_session_id != cached->second.request_product_session_id) {
        duplicate.accepted = false;
        duplicate.reason = "duplicate_request_id_product_session_mismatch";
      } else {
        duplicate.accepted = cached->second.accepted;
        duplicate.reason = cached->second.reason;
      }
    } else if (request.product_session_id != cached->second.request_product_session_id) {
      duplicate.accepted = false;
      duplicate.reason = "duplicate_request_id_product_session_mismatch";
    } else if (request.kind == static_cast<std::int32_t>(
                                   lingtu::message::ExplorationCommandKind::kSetDirectedTarget) &&
               (request.has_directed_target != cached->second.request_has_directed_target ||
                !sameSemanticDouble(request.directed_target_x,
                                    cached->second.request_directed_target_x) ||
                !sameSemanticDouble(request.directed_target_y,
                                    cached->second.request_directed_target_y) ||
                !sameSemanticDouble(request.directed_target_ttl_s,
                                    cached->second.request_directed_target_ttl_s))) {
      duplicate.accepted = false;
      duplicate.reason = "duplicate_request_id_directed_target_mismatch";
    } else {
      duplicate.accepted = cached->second.accepted;
      duplicate.reason = cached->second.reason;
    }
    return duplicate;
  }

  if (request.kind ==
          static_cast<std::int32_t>(lingtu::message::ExplorationCommandKind::kStart) &&
      !last_start_request_id_.empty() && request.request_id == last_start_request_id_) {
    ExplorationControlResult duplicate;
    duplicate.duplicate = true;
    duplicate.exploration_run_id = request.exploration_run_id;
    duplicate.product_session_id = request.product_session_id;
    if (const char *binding_rejection = start_session_rejection(); binding_rejection != nullptr) {
      duplicate.accepted = false;
      duplicate.reason = binding_rejection;
    } else if (request.exploration_run_id != last_start_exploration_run_id_) {
      duplicate.accepted = false;
      duplicate.reason = "duplicate_request_id_run_mismatch";
    } else if (request.product_session_id != last_start_product_session_id_) {
      duplicate.accepted = false;
      duplicate.reason = "duplicate_request_id_product_session_mismatch";
    } else {
      duplicate.accepted = true;
      duplicate.reason = "exploration_start_admitted";
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
  if (!lingtu::message::isValidExplorationRunId(request.exploration_run_id)) {
    return Finish(request, false, "exploration_run_id_invalid");
  }
  if (request.exploration_run_id == request.request_id) {
    return Finish(request, false, "exploration_run_id_matches_request_id");
  }
  if (request.request_id.size() > 128U || request.product_session_id.size() > 128U ||
      request.expected_product_session_id.size() > 128U || request.reason.size() > 256U) {
    return Finish(request, false, "exploration_command_field_too_long");
  }

  const auto kind = static_cast<lingtu::message::ExplorationCommandKind>(request.kind);
  if (!request.event_capacity_ready &&
      (kind == lingtu::message::ExplorationCommandKind::kStart ||
       kind == lingtu::message::ExplorationCommandKind::kPause ||
       kind == lingtu::message::ExplorationCommandKind::kResume ||
       kind == lingtu::message::ExplorationCommandKind::kStop)) {
    ExplorationControlResult backpressure;
    backpressure.reason = "exploration_event_outbox_backpressure";
    backpressure.exploration_run_id = request.exploration_run_id;
    backpressure.product_session_id = request.product_session_id;
    return backpressure;
  }
  const auto active_session_rejection = [this, &request]() -> const char * {
    if (request.exploration_run_id != exploration_run_id_) {
      return "exploration_run_mismatch";
    }
    if (request.product_session_id.empty()) {
      return "exploration_product_session_id_empty";
    }
    if (request.product_session_id != product_session_id_) {
      return "exploration_product_session_mismatch";
    }
    return nullptr;
  };
  switch (kind) {
    case lingtu::message::ExplorationCommandKind::kStart: {
      if (const char *binding_rejection = start_session_rejection(); binding_rejection != nullptr) {
        return Finish(request, false, binding_rejection);
      }
      if (active_) {
        if (request.product_session_id != product_session_id_) {
          return Finish(request, false, "exploration_product_session_conflict");
        }
        return Finish(request, false, "exploration_start_conflict");
      }
      if (used_exploration_run_ids_.find(request.exploration_run_id) !=
          used_exploration_run_ids_.end()) {
        return Finish(request, false, "exploration_run_id_reuse");
      }
      if (request.goal_pending || request.cancellation_pending) {
        return Finish(request, false, "exploration_stop_in_progress");
      }
      if (!request.inputs_ready) {
        return Finish(request, false, "exploration_inputs_not_ready");
      }
      active_ = true;
      paused_ = false;
      product_session_id_ = request.product_session_id;
      exploration_run_id_ = request.exploration_run_id;
      last_start_request_id_ = request.request_id;
      last_start_product_session_id_ = request.product_session_id;
      last_start_exploration_run_id_ = request.exploration_run_id;
      used_exploration_run_ids_.insert(request.exploration_run_id);
      ExplorationControlResult actions;
      actions.reset_planner = true;
      actions.clear_queue = true;
      actions.clear_history = true;
      actions.clear_directed_target = true;
      return Finish(request, true, "exploration_start_admitted", std::move(actions));
    }
    case lingtu::message::ExplorationCommandKind::kPause: {
      if (!active_) {
        return Finish(request, false, "exploration_not_active");
      }
      if (const char *binding_rejection = active_session_rejection();
          binding_rejection != nullptr) {
        return Finish(request, false, binding_rejection);
      }
      if (paused_) {
        return Finish(request, true, "exploration_already_paused");
      }
      paused_ = true;
      ExplorationControlResult actions;
      actions.clear_queue = true;
      actions.request_cancel = request.goal_pending;
      actions.cancel_reason = "exploration_paused";
      return Finish(request, true, "exploration_pause_admitted", std::move(actions));
    }
    case lingtu::message::ExplorationCommandKind::kResume:
      if (!active_) {
        return Finish(request, false, "exploration_not_active");
      }
      if (const char *binding_rejection = active_session_rejection();
          binding_rejection != nullptr) {
        return Finish(request, false, binding_rejection);
      }
      if (!paused_) {
        return Finish(request, true, "exploration_already_running");
      }
      if (request.goal_pending || request.cancellation_pending) {
        return Finish(request, false, "exploration_pause_in_progress");
      }
      if (!request.inputs_ready) {
        return Finish(request, false, "exploration_inputs_not_ready");
      }
      paused_ = false;
      return Finish(request, true, "exploration_resume_admitted");
    case lingtu::message::ExplorationCommandKind::kStop: {
      if (active_ || request.goal_pending || request.cancellation_pending || !product_session_id_.empty()) {
        if (const char *binding_rejection = active_session_rejection();
            binding_rejection != nullptr) {
          return Finish(request, false, binding_rejection);
        }
      }
      if (!active_ && (request.goal_pending || request.cancellation_pending)) {
        return Finish(request, true, "exploration_stop_in_progress");
      }
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
      return Finish(request, true, "exploration_stop_admitted", std::move(actions));
    }
    case lingtu::message::ExplorationCommandKind::kSetDirectedTarget: {
      if (!active_) {
        return Finish(request, false, "exploration_not_active");
      }
      if (request.exploration_run_id != exploration_run_id_) {
        return Finish(request, false, "directed_target_run_mismatch");
      }
      if (request.product_session_id.empty() || request.product_session_id != product_session_id_) {
        return Finish(request, false, "directed_target_product_session_mismatch");
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
      if (request.exploration_run_id != exploration_run_id_) {
        return Finish(request, false, "directed_target_run_mismatch");
      }
      if (request.product_session_id.empty() || request.product_session_id != product_session_id_) {
        return Finish(request, false, "directed_target_product_session_mismatch");
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
