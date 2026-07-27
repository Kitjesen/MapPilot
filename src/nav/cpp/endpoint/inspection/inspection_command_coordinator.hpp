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
  std::string request_id;
  std::int32_t raw_kind{0};
  std::string route_id;
  std::uint64_t route_revision{0U};
  std::string reason;
};

struct InspectionCommandAck {
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

struct InspectionCommandActions {
  std::function<bool()> route_source_available;
  std::function<std::optional<InspectionActiveMap>()> active_map;
  std::function<std::optional<lingtu::nav::inspection::Route>(const std::string &map_id,
                                                              const std::string &route_id)>
      load_route;
  std::function<bool()> operator_takeover_latched;
  std::function<bool(const std::string &reason)> clear_motion;
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

 private:
  struct AckRecord {
    lingtu::nav::inspection::CommandKind kind{lingtu::nav::inspection::CommandKind::kStart};
    bool accepted{false};
    std::string reason;
    std::string run_id;
  };

  [[nodiscard]] InspectionCommandResult publishOnly(InspectionCommandAck ack, bool replayed);
  void remember(const InspectionCommandAck &ack);
  void validateConfiguration() const;

  lingtu::nav::inspection::Executor &executor_;
  InspectionCommandActions actions_;
  std::size_t cache_limit_;
  std::unordered_map<std::string, AckRecord> ack_cache_;
  std::deque<std::string> ack_order_;
};

}  // namespace lingtu::nav::endpoint
