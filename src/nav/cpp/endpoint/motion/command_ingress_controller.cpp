#include "motion/command_ingress_controller.hpp"

#include <chrono>
#include <cstddef>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

constexpr std::size_t kBusinessJournalCapacity = 512U;
constexpr auto kBusinessJournalTtl = std::chrono::hours(24);

}  // namespace

CommandIngressController::CommandIngressController()
    : business_journal_(kBusinessJournalCapacity, kBusinessJournalTtl) {}

CommandIngressResult CommandIngressController::handle(const CommandIngressRequest &request,
                                                      const Dispatcher &dispatch,
                                                      std::optional<Clock::time_point> now) {
  ++diagnostics_.received;

  const auto kind = static_cast<CommandKind>(request.raw_kind);
  if (request.request_id.empty()) {
    return finish(request.task_id, {}, kind, {false, "command_request_id_empty"}, false, false);
  }
  if (request.client_id.empty()) {
    return finish(request.task_id, request.request_id, kind, {false, "command_client_id_empty"},
                  false, false);
  }
  if (!lingtu::message::isKnownNavigationCommandKind(request.raw_kind)) {
    return finish(request.task_id, request.request_id, kind, {false, "unknown_command_kind"},
                  false, false);
  }
  const bool task_id_required =
      kind == CommandKind::Goal || kind == CommandKind::TaskCancel ||
      kind == CommandKind::TaskPause || kind == CommandKind::TaskResume;
  if (task_id_required && request.task_id.empty()) {
    return finish({}, request.request_id, kind, {false, "command_task_id_empty"}, false, false);
  }

  CommandIdentity identity{
      request.client_id,
      request.request_id,
      request.task_id,
      kind,
      request.payload.canonical(),
  };
  const auto timestamp = now ? *now : Clock::now();
  const auto cached = business_journal_.lookup(identity, timestamp);
  if (cached.disposition == JournalDisposition::Conflict) {
    return finish(request.task_id, request.request_id, kind, {false, "idempotency_conflict"},
                  false, false);
  }
  if (cached.disposition == JournalDisposition::Replay && cached.ack) {
    ++diagnostics_.replayed;
    return finish(request.task_id, request.request_id, kind, *cached.ack, true, false);
  }

  CommandAck ack = dispatch(kind, request.payload);
  const bool cacheable = ack.accepted;
  if (cacheable) {
    business_journal_.remember(identity, ack, timestamp);
  }
  return finish(request.task_id, request.request_id, kind, std::move(ack), false, true);
}

CommandIngressResult CommandIngressController::finish(std::string task_id, std::string request_id,
                                                      CommandKind kind, CommandAck ack,
                                                      bool replayed, bool dispatched) {
  if (!ack.accepted) {
    ++diagnostics_.rejected;
  }
  diagnostics_.last_task_id = task_id;
  diagnostics_.last_request_id = request_id;
  diagnostics_.last_kind = lingtu::message::navigationCommandKindName(kind);
  diagnostics_.last_accepted = ack.accepted;
  diagnostics_.last_reason = ack.reason;
  return {
      std::move(task_id), std::move(request_id), kind, std::move(ack), replayed, dispatched,
  };
}

void CommandIngressController::recordAckPublication(bool published) noexcept {
  if (published) {
    ++diagnostics_.ack_sent;
  } else {
    ++diagnostics_.ack_publish_failed;
  }
}

}  // namespace lingtu::nav::endpoint
