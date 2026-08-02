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

  if (auto result = validateOrReplay(request)) {
    return std::move(*result);
  }

  const auto incoming_kind = static_cast<CommandKind>(request.raw_kind);
  bool accepted = false;
  std::string reason;
  std::string task_id = request.task_id;
  std::string run_id = executor_.status().run_id;

  switch (incoming_kind) {
    case CommandKind::kStart: {
      const auto active_map = actions_.active_map();
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
          task_id = request.task_id;
          const double now_s = actions_.now_s();
          accepted = executor_.Start(std::move(*route), task_id, request.request_id,
                                     active_map->map_id, active_map->version, now_s, &reason);
          if (accepted) {
            reason = "inspection_route_accepted";
            task_id = executor_.status().task_id;
            run_id = executor_.status().run_id;
          }
        }
      }
      break;
    }
    case CommandKind::kPause: {
      if (request.task_id != executor_.status().task_id) {
        reason = "inspection_task_id_mismatch";
        break;
      }
      run_id = executor_.status().run_id;
      if (!executor_.RequestPause(request.reason, request.request_id, actions_.now_s())) {
        reason = "inspection_pause_not_allowed";
        break;
      }
      const InspectionStopBarrierResult stop = actions_.stop_and_commit(
          "inspection_pause_requested", [this]() { return executor_.CommitPause(actions_.now_s()); });
      if (!stop.confirmed) {
        executor_.MarkStopConfirmationFailed(stop.reason, actions_.now_s());
      }
      accepted = true;
      reason = "pause_requested";
      break;
    }
    case CommandKind::kResume: {
      const auto active_map = actions_.active_map();
      if (request.task_id != executor_.status().task_id) {
        reason = "inspection_task_id_mismatch";
      } else if (actions_.operator_takeover_latched()) {
        reason = "inspection_resume_requires_autonomy";
      } else if (active_map) {
        run_id = executor_.status().run_id;
        accepted = executor_.Resume(
            active_map->map_id,
            active_map->version,
            actions_.now_s(),
            request.request_id);
        reason = accepted ? "inspection_resumed" : "inspection_resume_not_allowed";
      } else {
        reason = "inspection_resume_not_allowed";
      }
      break;
    }
    case CommandKind::kCancel: {
      if (request.task_id != executor_.status().task_id) {
        reason = "inspection_task_id_mismatch";
        break;
      }
      run_id = executor_.status().run_id;
      if (!executor_.RequestCancel(request.reason, request.request_id, actions_.now_s())) {
        reason = "inspection_cancel_not_allowed";
        break;
      }
      const InspectionStopBarrierResult stop = actions_.stop_and_commit(
          "inspection_cancel_requested", [this]() { return executor_.CommitCancel(actions_.now_s()); });
      if (!stop.confirmed) {
        executor_.MarkStopConfirmationFailed(stop.reason, actions_.now_s());
      }
      accepted = true;
      reason = "cancel_requested";
      break;
    }
  }

  InspectionCommandResult result;
  result.ack = InspectionCommandAck{
      std::move(task_id), request.request_id, incoming_kind, accepted, std::move(reason),
      std::move(run_id),
  };
  remember(request, result.ack);
  result.ack_published = actions_.publish_ack(result.ack);
  actions_.request_status();
  result.status_requested = true;
  return result;
}

InspectionCommandResult
InspectionCommandCoordinator::reject(const InspectionCommandRequest &request,
                                     const std::string &reason) {
  using lingtu::nav::inspection::CommandKind;

  if (auto result = validateOrReplay(request)) {
    return std::move(*result);
  }

  const auto incoming_kind = static_cast<CommandKind>(request.raw_kind);
  InspectionCommandResult result;
  result.ack = InspectionCommandAck{
      request.task_id,
      request.request_id,
      incoming_kind,
      false,
      reason.empty() ? "inspection_command_rejected" : reason,
      executor_.status().run_id,
  };
  remember(request, result.ack);
  result.ack_published = actions_.publish_ack(result.ack);
  return result;
}

std::optional<InspectionCommandResult>
InspectionCommandCoordinator::validateOrReplay(const InspectionCommandRequest &request) {
  using lingtu::nav::inspection::CommandKind;

  const auto incoming_kind = static_cast<CommandKind>(request.raw_kind);
  if (request.request_id.empty()) {
    return publishOnly(
        InspectionCommandAck{
            request.task_id,
            request.request_id,
            incoming_kind,
            false,
            "inspection_request_id_empty",
            "",
        },
        false);
  }
  if (request.task_id.empty()) {
    return publishOnly(
        InspectionCommandAck{
            request.task_id,
            request.request_id,
            incoming_kind,
            false,
            "inspection_task_id_empty",
            "",
        },
        false);
  }
  if (!lingtu::nav::inspection::IsKnownCommandKind(request.raw_kind)) {
    return publishOnly(
        InspectionCommandAck{
            request.task_id,
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
    const bool payload_matches = kind_matches &&
        cached->second.request_task_id == request.task_id &&
        cached->second.route_id == request.route_id &&
        cached->second.route_revision == request.route_revision &&
        cached->second.reason_input == request.reason;
    return publishOnly(
        InspectionCommandAck{
            payload_matches ? cached->second.task_id : "",
            request.request_id,
            incoming_kind,
            payload_matches && cached->second.accepted,
            !kind_matches ? "duplicate_request_id_kind_mismatch"
                          : (payload_matches ? cached->second.reason
                                             : "duplicate_request_id_payload_mismatch"),
            payload_matches ? cached->second.run_id : "",
        },
        true);
  }

  return std::nullopt;
}

InspectionCommandResult InspectionCommandCoordinator::publishOnly(InspectionCommandAck ack,
                                                                  bool replayed) {
  InspectionCommandResult result;
  result.ack = std::move(ack);
  result.replayed = replayed;
  result.ack_published = actions_.publish_ack(result.ack);
  return result;
}

void InspectionCommandCoordinator::remember(
    const InspectionCommandRequest &request,
    const InspectionCommandAck &ack) {
  if (ack_cache_.find(ack.request_id) == ack_cache_.end()) {
    ack_order_.push_back(ack.request_id);
  }
  ack_cache_[ack.request_id] = AckRecord{
      ack.kind,
      request.task_id,
      ack.task_id,
      request.route_id,
      request.route_revision,
      request.reason,
      ack.accepted,
      ack.reason,
      ack.run_id,
  };
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
      !actions_.operator_takeover_latched || !actions_.stop_and_commit || !actions_.publish_ack ||
      !actions_.request_status || !actions_.now_s) {
    throw std::invalid_argument("inspection command coordinator actions are incomplete");
  }
}

}  // namespace lingtu::nav::endpoint
