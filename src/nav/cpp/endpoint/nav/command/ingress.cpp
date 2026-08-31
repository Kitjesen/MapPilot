#include "command/ingress.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace lingtu::nav::endpoint {
namespace {

constexpr std::size_t kBusinessJournalCapacity = 512U;
constexpr auto kBusinessJournalTtl = std::chrono::hours(24);

void appendUint64(std::string &output, std::uint64_t value) {
  for (int shift = 56; shift >= 0; shift -= 8) {
    output.push_back(static_cast<char>((value >> shift) & 0xffU));
  }
}

void appendString(std::string &output, const std::string &value) {
  appendUint64(output, static_cast<std::uint64_t>(value.size()));
  output.append(value);
}

void appendDouble(std::string &output, double value) {
  if (value == 0.0) {
    value = 0.0;
  }
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  appendUint64(output, bits);
}

}  // namespace

std::string CommandPayload::canonical() const {
  std::string output;
  output.reserve(sizeof(std::uint64_t) * (2U + goal.size()) + frame_id.size() + reason.size());
  appendString(output, frame_id);
  for (const double value : goal) {
    appendDouble(output, value);
  }
  appendString(output, reason);
  return output;
}

CommandAckJournal::CommandAckJournal(std::size_t capacity, Clock::duration ttl)
    : capacity_(capacity), ttl_(ttl) {
  if (capacity_ == 0U) {
    throw std::invalid_argument("command ACK journal capacity must be positive");
  }
  if (ttl_ <= Clock::duration::zero()) {
    throw std::invalid_argument("command ACK journal TTL must be positive");
  }
}

JournalLookup CommandAckJournal::lookup(const CommandIdentity &identity, Clock::time_point now) {
  prune(now);
  const Key key = keyFor(identity);
  for (auto &[entry_key, entry] : entries_) {
    if (entry_key.client_id == identity.client_id && entry_key.kind == identity.kind &&
        std::find(entry.request_ids.begin(), entry.request_ids.end(), identity.request_id) !=
            entry.request_ids.end() &&
        (!(entry_key == key) || entry.identity.task_id != identity.task_id ||
         entry.identity.canonical_payload != identity.canonical_payload)) {
      return {JournalDisposition::Conflict, std::nullopt};
    }
  }
  const auto found = entries_.find(key);
  if (found == entries_.end()) {
    return {};
  }
  if (found->second.identity.task_id != identity.task_id ||
      found->second.identity.canonical_payload != identity.canonical_payload) {
    return {JournalDisposition::Conflict, std::nullopt};
  }
  if (std::find(found->second.request_ids.begin(), found->second.request_ids.end(),
                identity.request_id) == found->second.request_ids.end()) {
    found->second.request_ids.push_back(identity.request_id);
  }
  return {JournalDisposition::Replay, found->second.ack};
}

void CommandAckJournal::remember(CommandIdentity identity, CommandAck ack, Clock::time_point now) {
  prune(now);
  Key key = keyFor(identity);
  for (const auto &[entry_key, entry] : entries_) {
    if (entry_key.client_id == identity.client_id && entry_key.kind == identity.kind &&
        std::find(entry.request_ids.begin(), entry.request_ids.end(), identity.request_id) !=
            entry.request_ids.end() &&
        (!(entry_key == key) || entry.identity.task_id != identity.task_id ||
         entry.identity.canonical_payload != identity.canonical_payload)) {
      throw std::logic_error("cannot alias one request ID to conflicting task identity");
    }
  }
  const auto found = entries_.find(key);
  if (found != entries_.end()) {
    if (found->second.identity.task_id != identity.task_id ||
        found->second.identity.canonical_payload != identity.canonical_payload) {
      throw std::logic_error("cannot overwrite command ACK with a conflicting identity");
    }
    found->second.ack = std::move(ack);
    if (std::find(found->second.request_ids.begin(), found->second.request_ids.end(),
                  identity.request_id) == found->second.request_ids.end()) {
      found->second.request_ids.push_back(identity.request_id);
    }
    return;
  }

  order_.push_back(key);
  std::vector<std::string> request_ids{identity.request_id};
  entries_.emplace(std::move(key),
                   Entry{std::move(identity), std::move(ack), now, std::move(request_ids)});
  while (entries_.size() > capacity_ && !order_.empty()) {
    entries_.erase(order_.front());
    order_.pop_front();
  }
}

bool CommandAckJournal::Key::operator==(const Key &other) const noexcept {
  return client_id == other.client_id && logical_id == other.logical_id && kind == other.kind;
}

std::size_t CommandAckJournal::KeyHash::operator()(const Key &key) const noexcept {
  const std::size_t client_hash = std::hash<std::string>{}(key.client_id);
  const std::size_t request_hash = std::hash<std::string>{}(key.logical_id);
  const std::size_t request_key_hash =
      client_hash ^ (request_hash + static_cast<std::size_t>(0x9e3779b9U) +
                     (client_hash << 6U) + (client_hash >> 2U));
  const auto kind_hash = std::hash<std::int32_t>{}(static_cast<std::int32_t>(key.kind));
  return request_key_hash ^ (kind_hash + (request_key_hash << 6U));
}

CommandAckJournal::Key CommandAckJournal::keyFor(const CommandIdentity &identity) {
  const bool retry_is_task_scoped =
      identity.kind == lingtu::message::NavigationCommandKind::Goal ||
      identity.kind == lingtu::message::NavigationCommandKind::TaskCancel;
  return {
      identity.client_id,
      retry_is_task_scoped && !identity.task_id.empty() ? identity.task_id : identity.request_id,
      identity.kind,
  };
}

void CommandAckJournal::prune(Clock::time_point now) {
  while (!order_.empty()) {
    const auto found = entries_.find(order_.front());
    if (found == entries_.end()) {
      order_.pop_front();
      continue;
    }
    if (now - found->second.recorded_at <= ttl_) {
      break;
    }
    entries_.erase(found);
    order_.pop_front();
  }
}

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
