#pragma once

#include <future>
#include <optional>
#include <string>

#include "app/config/config.hpp"

namespace lingtu::nav::endpoint {

struct PreparedPlanner {
  CliConfig config;
  navigation::Executor executor;
};

// Loading CMU's path library must not block the control/stop heartbeat.
class PlannerSelection {
 public:
  std::string start(const CliConfig &config, const std::string &name);
  [[nodiscard]] bool busy() const { return loading_.valid(); }
  std::optional<PreparedPlanner> poll();
  void retire(navigation::Executor executor);

 private:
  std::future<PreparedPlanner> loading_;
  std::vector<std::future<void>> retired_;
};

}  // namespace lingtu::nav::endpoint
