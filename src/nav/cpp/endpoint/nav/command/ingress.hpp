#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "message/cpp/navigation_command.hpp"

namespace lingtu::nav::endpoint {

struct CommandPayload {
  std::string frame_id;
  std::array<double, 7> goal{};
  std::string reason;

  [[nodiscard]] std::string canonical() const;
};

struct CommandIdentity {
  std::string client_id;
  std::string request_id;
  std::string task_id;
  lingtu::message::NavigationCommandKind kind{lingtu::message::NavigationCommandKind::Goal};
  std::string canonical_payload;
};

struct CommandAck {
  bool accepted{false};
  std::string reason;
};

enum class JournalDisposition {
  Miss,
  Replay,
  Conflict,
};

struct JournalLookup {
  JournalDisposition disposition{JournalDisposition::Miss};
  std::optional<CommandAck> ack;
};

class CommandAckJournal {
 public:
  using Clock = std::chrono::steady_clock;

  CommandAckJournal(std::size_t capacity, Clock::duration ttl);

  JournalLookup lookup(const CommandIdentity &identity, Clock::time_point now = Clock::now());
  void remember(CommandIdentity identity, CommandAck ack,
                Clock::time_point now = Clock::now());

 private:
  struct Key {
    std::string client_id;
    std::string logical_id;
    lingtu::message::NavigationCommandKind kind{lingtu::message::NavigationCommandKind::Goal};

    bool operator==(const Key &other) const noexcept;
  };

  struct KeyHash {
    std::size_t operator()(const Key &key) const noexcept;
  };

  struct Entry {
    CommandIdentity identity;
    CommandAck ack;
    Clock::time_point recorded_at;
    std::vector<std::string> request_ids;
  };

  static Key keyFor(const CommandIdentity &identity);
  void prune(Clock::time_point now);

  std::size_t capacity_;
  Clock::duration ttl_;
  std::unordered_map<Key, Entry, KeyHash> entries_;
  std::deque<Key> order_;
};

struct CommandDiagnostics {
  std::uint64_t received{0};
  std::uint64_t ack_sent{0};
  std::uint64_t ack_publish_failed{0};
  std::uint64_t rejected{0};
  std::uint64_t replayed{0};
  std::string last_task_id;
  std::string last_request_id;
  std::string last_kind{"none"};
  bool last_accepted{false};
  std::string last_reason{"none"};
};

struct CommandIngressRequest {
  std::string client_id;
  std::string request_id;
  std::string task_id;
  std::int32_t raw_kind{0};
  CommandPayload payload;
};

struct CommandIngressResult {
  std::string task_id;
  std::string request_id;
  lingtu::message::NavigationCommandKind kind{lingtu::message::NavigationCommandKind::Goal};
  CommandAck ack;
  bool replayed{false};
  bool dispatched{false};
};

class CommandIngressController {
 public:
  using Clock = CommandAckJournal::Clock;
  using CommandKind = lingtu::message::NavigationCommandKind;
  using Dispatcher = std::function<CommandAck(CommandKind, const CommandPayload &)>;

  CommandIngressController();

  CommandIngressResult handle(const CommandIngressRequest &request, const Dispatcher &dispatch,
                              std::optional<Clock::time_point> now = std::nullopt);
  void recordAckPublication(bool published) noexcept;

  const CommandDiagnostics &diagnostics() const noexcept { return diagnostics_; }

 private:
  CommandIngressResult finish(std::string task_id, std::string request_id, CommandKind kind,
                              CommandAck ack, bool replayed, bool dispatched);

  CommandAckJournal business_journal_;
  CommandDiagnostics diagnostics_;
};

}  // namespace lingtu::nav::endpoint
