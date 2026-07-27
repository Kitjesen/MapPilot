#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "message/cpp/navigation_command.hpp"

namespace lingtu::nav::endpoint {
namespace command_journal_detail {

inline void appendUint64(std::string &output, std::uint64_t value) {
  for (int shift = 56; shift >= 0; shift -= 8) {
    output.push_back(static_cast<char>((value >> shift) & 0xffU));
  }
}

inline void appendString(std::string &output, const std::string &value) {
  appendUint64(output, static_cast<std::uint64_t>(value.size()));
  output.append(value);
}

inline void appendDouble(std::string &output, double value) {
  if (value == 0.0) {
    value = 0.0;
  }
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  appendUint64(output, bits);
}

}  // namespace command_journal_detail

struct CommandPayload {
  std::string frame_id;
  std::array<double, 7> goal{};
  std::array<double, 6> velocity{};
  std::string reason;

  std::string canonical() const {
    std::string output;
    output.reserve(sizeof(std::uint64_t) * (2U + goal.size() + velocity.size()) + frame_id.size() +
                   reason.size());
    command_journal_detail::appendString(output, frame_id);
    for (const double value : goal) {
      command_journal_detail::appendDouble(output, value);
    }
    for (const double value : velocity) {
      command_journal_detail::appendDouble(output, value);
    }
    command_journal_detail::appendString(output, reason);
    return output;
  }
};

struct CommandIdentity {
  std::string client_id;
  std::string request_id;
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

  CommandAckJournal(std::size_t capacity, Clock::duration ttl) : capacity_(capacity), ttl_(ttl) {
    if (capacity_ == 0U) {
      throw std::invalid_argument("command ACK journal capacity must be positive");
    }
    if (ttl_ <= Clock::duration::zero()) {
      throw std::invalid_argument("command ACK journal TTL must be positive");
    }
  }

  JournalLookup lookup(const CommandIdentity &identity, Clock::time_point now = Clock::now()) {
    prune(now);
    const Key key{
        identity.client_id,
        identity.request_id,
        identity.kind,
    };
    const auto found = entries_.find(key);
    if (found == entries_.end()) {
      return {};
    }
    if (found->second.identity.canonical_payload != identity.canonical_payload) {
      return {JournalDisposition::Conflict, std::nullopt};
    }
    return {JournalDisposition::Replay, found->second.ack};
  }

  void remember(CommandIdentity identity, CommandAck ack, Clock::time_point now = Clock::now()) {
    prune(now);
    Key key{
        identity.client_id,
        identity.request_id,
        identity.kind,
    };
    const auto found = entries_.find(key);
    if (found != entries_.end()) {
      if (found->second.identity.canonical_payload != identity.canonical_payload) {
        throw std::logic_error("cannot overwrite command ACK with a conflicting identity");
      }
      found->second.ack = std::move(ack);
      return;
    }

    order_.push_back(key);
    entries_.emplace(std::move(key), Entry{std::move(identity), std::move(ack), now});
    while (entries_.size() > capacity_ && !order_.empty()) {
      entries_.erase(order_.front());
      order_.pop_front();
    }
  }

 private:
  struct Key {
    std::string client_id;
    std::string request_id;
    lingtu::message::NavigationCommandKind kind{lingtu::message::NavigationCommandKind::Goal};

    bool operator==(const Key &other) const noexcept {
      return client_id == other.client_id && request_id == other.request_id && kind == other.kind;
    }
  };

  struct KeyHash {
    std::size_t operator()(const Key &key) const noexcept {
      const std::size_t client_hash = std::hash<std::string>{}(key.client_id);
      const std::size_t request_hash = std::hash<std::string>{}(key.request_id);
      const std::size_t request_key_hash =
          client_hash ^ (request_hash + static_cast<std::size_t>(0x9e3779b9U) +
                         (client_hash << 6U) + (client_hash >> 2U));
      const auto kind_hash = std::hash<std::int32_t>{}(static_cast<std::int32_t>(key.kind));
      return request_key_hash ^ (kind_hash + (request_key_hash << 6U));
    }
  };

  struct Entry {
    CommandIdentity identity;
    CommandAck ack;
    Clock::time_point recorded_at;
  };

  void prune(Clock::time_point now) {
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

  std::size_t capacity_;
  Clock::duration ttl_;
  std::unordered_map<Key, Entry, KeyHash> entries_;
  std::deque<Key> order_;
};

}  // namespace lingtu::nav::endpoint
