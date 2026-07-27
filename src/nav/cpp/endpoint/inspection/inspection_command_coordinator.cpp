#include "inspection/inspection_command_coordinator.hpp"

#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {

InspectionCommandCoordinator::InspectionCommandCoordinator(
    lingtu::nav::inspection::Executor &executor, InspectionCommandActions actions,
    std::size_t cache_limit)
    : executor_(executor), actions_(std::move(actions)), cache_limit_(cache_limit) {
  validateConfiguration();
}

InspectionCommandResult
InspectionCommandCoordinator::handle(const InspectionCommandRequest &request) {
  using lingtu::nav::inspection::CommandKind;

  const auto incoming_kind = static_cast<CommandKind>(request.raw_kind);
  if (request.request_id.empty()) {
    return publishOnly(
        InspectionCommandAck{
            request.request_id,
            incoming_kind,
            false,
            "inspection_request_id_empty",
            "",
        },
        false);
  }
  if (!lingtu::nav::inspection::IsKnownCommandKind(request.raw_kind)) {
    return publishOnly(
        InspectionCommandAck{
            request.request_id,
            incoming_kind,
            false,
            "unknown_inspection_command",
            "",
        },
        false);
  }

  const auto cached = ack_cache_.find(request.request_id);
  if (cached != ack_cache_.end()) {
    const bool kind_matches = cached->second.kind == incoming_kind;
    return publishOnly(
        InspectionCommandAck{
            request.request_id,
            incoming_kind,
            kind_matches && cached->second.accepted,
            kind_matches ? cached->second.reason : "duplicate_request_id_kind_mismatch",
            kind_matches ? cached->second.run_id : "",
        },
        true);
  }

  bool accepted = false;
  std::string reason;
  std::string run_id = executor_.status().run_id;
  const auto active_map = actions_.active_map();

  switch (incoming_kind) {
    case CommandKind::kStart: {
      if (!actions_.route_source_available() || !active_map) {
        reason = "active_map_unavailable";
      } else if (executor_.active()) {
        reason = "inspection_run_active";
      } else {
        auto route = actions_.load_route(active_map->map_id, request.route_id);
        if (!route) {
          reason = "inspection_route_not_found";
        } else if (request.route_revision != 0U && request.route_revision != route->revision) {
          reason = "inspection_route_revision_mismatch";
        } else {
          run_id = request.request_id;
          const double now_s = actions_.now_s();
          accepted = executor_.Start(std::move(*route), run_id, active_map->map_id,
                                     active_map->version, now_s, &reason);
          if (accepted) {
            reason = "inspection_route_accepted";
          }
        }
      }
      break;
    }
    case CommandKind::kPause:
      accepted = executor_.Pause(request.reason);
      reason = accepted ? "inspection_paused" : "inspection_pause_not_allowed";
      if (accepted && !actions_.clear_motion("inspection_paused")) {
        accepted = false;
        reason = "inspection_pause_zero_publish_failed";
      }
      break;
    case CommandKind::kResume:
      if (actions_.operator_takeover_latched()) {
        reason = "inspection_resume_requires_autonomy";
      } else if (active_map) {
        const double now_s = actions_.now_s();
        accepted = executor_.Resume(active_map->map_id, active_map->version, now_s);
        reason = accepted ? "inspection_resumed" : "inspection_resume_not_allowed";
      } else {
        reason = "inspection_resume_not_allowed";
      }
      break;
    case CommandKind::kCancel:
      accepted = executor_.Cancel(request.reason);
      reason = accepted ? "inspection_cancelled" : "inspection_cancel_not_allowed";
      if (accepted && !actions_.clear_motion("inspection_cancelled")) {
        accepted = false;
        reason = "inspection_cancel_zero_publish_failed";
      }
      break;
  }

  InspectionCommandResult result;
  result.ack = InspectionCommandAck{
      request.request_id, incoming_kind, accepted, std::move(reason), std::move(run_id),
  };
  remember(result.ack);
  result.ack_published = actions_.publish_ack(result.ack);
  actions_.request_status();
  result.status_requested = true;
  return result;
}

InspectionCommandResult InspectionCommandCoordinator::publishOnly(InspectionCommandAck ack,
                                                                  bool replayed) {
  InspectionCommandResult result;
  result.ack = std::move(ack);
  result.replayed = replayed;
  result.ack_published = actions_.publish_ack(result.ack);
  return result;
}

void InspectionCommandCoordinator::remember(const InspectionCommandAck &ack) {
  if (ack_cache_.find(ack.request_id) == ack_cache_.end()) {
    ack_order_.push_back(ack.request_id);
  }
  ack_cache_[ack.request_id] = AckRecord{ack.kind, ack.accepted, ack.reason, ack.run_id};
  while (ack_order_.size() > cache_limit_) {
    ack_cache_.erase(ack_order_.front());
    ack_order_.pop_front();
  }
}

void InspectionCommandCoordinator::validateConfiguration() const {
  if (cache_limit_ == 0U) {
    throw std::invalid_argument("inspection command coordinator cache limit must be positive");
  }
  if (!actions_.route_source_available || !actions_.active_map || !actions_.load_route ||
      !actions_.operator_takeover_latched || !actions_.clear_motion || !actions_.publish_ack ||
      !actions_.request_status || !actions_.now_s) {
    throw std::invalid_argument("inspection command coordinator actions are incomplete");
  }
}

}  // namespace lingtu::nav::endpoint
