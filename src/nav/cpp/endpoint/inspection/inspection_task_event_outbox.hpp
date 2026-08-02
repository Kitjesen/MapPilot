#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <string>

#include "nav/inspection/inspection.hpp"

namespace lingtu::nav::endpoint {

// Transport-neutral envelope for one immutable inspection task fact. The
// endpoint boot identity plus sequence lets downstream consumers detect a
// restart or a missing event without deriving lifecycle state from ACKs.
struct InspectionTaskEventEnvelope {
  std::string boot_id;
  std::uint64_t sequence{0U};
  lingtu::nav::inspection::TaskEvent event;
};

enum class InspectionTaskEventOutboxRecordResult : std::uint8_t {
  kAccepted,
  kInvalid,
  kOutOfOrder,
  kBackpressure,
};

const char* InspectionTaskEventOutboxRecordResultName(
    InspectionTaskEventOutboxRecordResult result) noexcept;

struct InspectionTaskEventOutboxDiagnostics {
  std::uint64_t accepted{0U};
  std::uint64_t rejected_invalid{0U};
  std::uint64_t rejected_out_of_order{0U};
  std::uint64_t rejected_backpressure{0U};
  std::uint64_t delivered{0U};
  std::uint64_t delivery_failures{0U};
  std::size_t pending{0U};
};

// The control loop records native task facts here and retries the oldest DDS
// write until it succeeds. It never merges, reorders, or silently drops an
// accepted fact. Restart persistence is intentionally a separate journal
// concern; this class protects delivery within one endpoint boot.
class InspectionTaskEventOutbox final {
 public:
  using WriteCallback = std::function<bool(const InspectionTaskEventEnvelope&)>;

  static constexpr std::size_t kDefaultCapacity = 1024U;
  static constexpr std::size_t kDefaultFlushLimit = 32U;

  InspectionTaskEventOutbox(std::string boot_id, WriteCallback write,
                            std::size_t capacity = kDefaultCapacity);

  InspectionTaskEventOutbox(const InspectionTaskEventOutbox&) = delete;
  InspectionTaskEventOutbox& operator=(const InspectionTaskEventOutbox&) = delete;

  [[nodiscard]] InspectionTaskEventOutboxRecordResult record(
      const lingtu::nav::inspection::TaskEvent& event);
  [[nodiscard]] std::size_t flush(std::size_t max_events = kDefaultFlushLimit);
  [[nodiscard]] InspectionTaskEventOutboxDiagnostics diagnostics() const noexcept;

 private:
  [[nodiscard]] static bool valid(const lingtu::nav::inspection::TaskEvent& event) noexcept;
  [[nodiscard]] static bool validKind(
      lingtu::nav::inspection::TaskEventKind kind) noexcept;
  [[nodiscard]] static bool validState(
      lingtu::nav::inspection::RunState state) noexcept;

  std::string boot_id_;
  WriteCallback write_;
  std::size_t capacity_;
  std::uint64_t last_accepted_sequence_{0U};
  std::deque<InspectionTaskEventEnvelope> pending_;
  InspectionTaskEventOutboxDiagnostics diagnostics_;
};

}  // namespace lingtu::nav::endpoint
