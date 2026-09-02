#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include "message/cpp/exploration_command.hpp"

namespace lingtu::nav::endpoint {

struct ExplorationRunEventRecord {
  double timestamp_s{0.0};
  std::string frame_id{"map"};
  lingtu::message::ExplorationRunEventKind kind{
      lingtu::message::ExplorationRunEventKind::kStateChanged};
  std::string exploration_run_id;
  std::string start_request_id;
  std::string command_request_id;
  std::string product_session_id;
  lingtu::message::ExplorationRunState state{lingtu::message::ExplorationRunState::kAdmitted};
  std::string route;
  std::string map_id;
  std::int64_t map_content_epoch{0};
  std::string reason;
  bool motion_stop_confirmed{false};
  std::string motion_stop_reason;
};

struct ExplorationRunEventEnvelope {
  std::string boot_id;
  std::uint64_t event_sequence{0U};
  ExplorationRunEventRecord event;
};

enum class ExplorationRunEventOutboxRecordResult : std::uint8_t {
  kAccepted,
  kInvalid,
  kBackpressure,
};

const char *ExplorationRunEventOutboxRecordResultName(
    ExplorationRunEventOutboxRecordResult result) noexcept;

struct ExplorationRunEventOutboxDiagnostics {
  std::uint64_t accepted{0U};
  std::uint64_t rejected_invalid{0U};
  std::uint64_t rejected_backpressure{0U};
  std::uint64_t delivered{0U};
  std::uint64_t delivery_failures{0U};
  std::size_t pending{0U};
};

// Ordered, retrying, boot-local delivery for immutable exploration facts.
// Accepted facts are never merged, reordered, or silently discarded.
// Normal capacity cannot consume the three slots needed for one motion-stop
// transition: cancelling/pausing, one deduplicated failure, and its terminal.
class ExplorationRunEventOutbox final {
 public:
  using WriteCallback = std::function<bool(const ExplorationRunEventEnvelope &)>;

  static constexpr std::size_t kDefaultCapacity = 1024U;
  static constexpr std::size_t kDefaultFlushLimit = 32U;
  static constexpr std::size_t kMotionStopReserve = 3U;

  ExplorationRunEventOutbox(std::string boot_id, WriteCallback write,
                            std::size_t capacity = kDefaultCapacity);

  ExplorationRunEventOutbox(const ExplorationRunEventOutbox &) = delete;
  ExplorationRunEventOutbox &operator=(const ExplorationRunEventOutbox &) = delete;

  [[nodiscard]] bool canRecord(std::size_t count = 1U) const noexcept;
  [[nodiscard]] bool canRecordMotionStop(std::size_t count = 1U) const noexcept;
  [[nodiscard]] const std::string &bootId() const noexcept { return boot_id_; }
  [[nodiscard]] ExplorationRunEventOutboxRecordResult record(
      const ExplorationRunEventRecord &event);
  [[nodiscard]] ExplorationRunEventOutboxRecordResult recordMotionStop(
      const ExplorationRunEventRecord &event);
  [[nodiscard]] std::size_t flush(std::size_t max_events = kDefaultFlushLimit);
  [[nodiscard]] ExplorationRunEventOutboxDiagnostics diagnostics() const noexcept;

 private:
  [[nodiscard]] static bool valid(const ExplorationRunEventRecord &event) noexcept;
  [[nodiscard]] ExplorationRunEventOutboxRecordResult recordImpl(
      const ExplorationRunEventRecord &event, bool motion_stop);

  std::string boot_id_;
  WriteCallback write_;
  std::size_t capacity_{kDefaultCapacity};
  std::uint64_t next_sequence_{1U};
  std::deque<ExplorationRunEventEnvelope> pending_;
  ExplorationRunEventOutboxDiagnostics diagnostics_;
};

}  // namespace lingtu::nav::endpoint
