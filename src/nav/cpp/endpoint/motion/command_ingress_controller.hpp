#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "motion/command_ack_journal.hpp"

namespace lingtu::nav::endpoint {

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

  CommandAckJournal &journalFor(CommandKind kind) noexcept;

  CommandAckJournal business_journal_;
  CommandAckJournal teleop_journal_;
  CommandDiagnostics diagnostics_;
};

}  // namespace lingtu::nav::endpoint
