#pragma once

#include "inspection.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::nav::inspection {

struct StoreResult {
  bool ok{false};
  std::string reason;
};

struct RouteSummary {
  std::string id;
  std::string name;
  std::string map_id;
  std::int64_t map_content_epoch{0};
  std::uint64_t revision{0};
  std::size_t point_count{0};
};

enum class TaskEventCheckpointLoadState : std::uint8_t {
  kNotFound,
  kLoaded,
  kCorrupt,
  kIoError,
};

struct TaskEventCheckpointLoadResult {
  TaskEventCheckpointLoadState state{TaskEventCheckpointLoadState::kNotFound};
  std::optional<TaskEvent> event;
  std::string reason;

  bool loaded() const noexcept {
    return state == TaskEventCheckpointLoadState::kLoaded && event.has_value();
  }
};

class Store {
 public:
  explicit Store(std::filesystem::path data_dir);

  StoreResult Put(const Route& route);
  std::optional<Route> Get(const std::string& map_id, const std::string& route_id) const;
  StoreResult Delete(const std::string& map_id, const std::string& route_id);
  std::vector<RouteSummary> List(const std::string& map_id) const;
  std::string StatusJson() const;
  StoreResult PutTaskEventCheckpoint(const TaskEvent& event);
  TaskEventCheckpointLoadResult LoadTaskEventCheckpoint() const;
  std::filesystem::path TaskEventCheckpointPath() const;

  std::filesystem::path RouteDirectory(const std::string& map_id) const;
  std::filesystem::path RoutePath(
      const std::string& map_id,
      const std::string& route_id) const;

 private:
  std::filesystem::path data_dir_;
};

std::string RouteToJson(const Route& route);
std::string RouteListToJson(const std::vector<RouteSummary>& routes);
std::string RunStatusToJson(const RunStatus& status);
std::string TaskEventToJson(const TaskEvent& event);

}  // namespace lingtu::nav::inspection
