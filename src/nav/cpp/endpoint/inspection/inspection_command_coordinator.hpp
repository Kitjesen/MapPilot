#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>

#include "nav/inspection/inspection.hpp"

namespace lingtu::nav::endpoint {

struct InspectionActiveMap {
  std::string map_id;
  std::int64_t version{0};
};

// Owning command representation. DDS samples must be copied into this request
// before their loan is returned.
struct InspectionCommandRequest {
  std::string task_id;
  std::string request_id;
  std::int32_t raw_kind{0};
  std::string route_id;
  std::uint64_t route_revision{0U};
  std::string reason;
};

struct InspectionCommandAck {
  std::string task_id;
  std::string request_id;
  lingtu::nav::inspection::CommandKind kind{lingtu::nav::inspection::CommandKind::kStart};
  bool accepted{false};
  std::string reason;
  std::string run_id;
};

struct InspectionCommandResult {
  InspectionCommandAck ack;
  bool ack_published{false};
  bool replayed{false};
  bool status_requested{false};
};

using InspectionCommandCommit = std::function<bool()>;

struct InspectionStopBarrierResult {
  bool confirmed{false};
  std::string reason;
};

struct InspectionCommandActions {
  std::function<bool()> route_source_available;
  std::function<std::optional<InspectionActiveMap>()> active_map;
  std::function<std::optional<lingtu::nav::inspection::Route>(const std::string &map_id,
                                                              const std::string &route_id)>
      load_route;
  std::function<bool()> operator_takeover_latched;
  // The native adapter must clear route motion, confirm final zero output,
  // and only then invoke commit. It is the task-control physical-stop seam.
  std::function<InspectionStopBarrierResult(const std::string &reason,
                                            InspectionCommandCommit commit)>
      stop_and_commit;
  std::function<bool(const InspectionCommandAck &ack)> publish_ack;
  std::function<void()> request_status;
  std::function<double()> now_s;
};

// Transport- and file-I/O-free command admission, idempotency, and ordered
// effect coordinator for inspection runs.
class InspectionCommandCoordinator final {
 public:
  static constexpr std::size_t kDefaultCacheLimit = 128U;

  InspectionCommandCoordinator(lingtu::nav::inspection::Executor &executor,
                               InspectionCommandActions actions,
                               std::size_t cache_limit = kDefaultCacheLimit);

  InspectionCommandCoordinator(const InspectionCommandCoordinator &) = delete;
  InspectionCommandCoordinator &operator=(const InspectionCommandCoordinator &) = delete;

  [[nodiscard]] InspectionCommandResult handle(const InspectionCommandRequest &request);
  [[nodiscard]] InspectionCommandResult reject(const InspectionCommandRequest &request,
                                               const std::string &reason);

 private:
  struct AckRecord {
    lingtu::nav::inspection::CommandKind kind{lingtu::nav::inspection::CommandKind::kStart};
    std::string request_task_id;
    std::string task_id;
    std::string route_id;
    std::uint64_t route_revision{0U};
    std::string reason_input;
    bool accepted{false};
    std::string reason;
    std::string run_id;
  };

  [[nodiscard]] std::optional<InspectionCommandResult>
  validateOrReplay(const InspectionCommandRequest &request);
  [[nodiscard]] InspectionCommandResult publishOnly(InspectionCommandAck ack, bool replayed);
  void remember(const InspectionCommandRequest &request, const InspectionCommandAck &ack);
  void validateConfiguration() const;

  lingtu::nav::inspection::Executor &executor_;
  InspectionCommandActions actions_;
  std::size_t cache_limit_;
  std::unordered_map<std::string, AckRecord> ack_cache_;
  std::deque<std::string> ack_order_;
};

}  // namespace lingtu::nav::endpoint
