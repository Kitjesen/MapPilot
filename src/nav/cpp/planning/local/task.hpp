#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "planning/local/planner.hpp"

namespace nav_kernel::local {

struct LocalPlanUpdate {
  LocalPlan plan{};
  LocalPlannerDebugSnapshot debug{};
};

// Stateful latest-only planning task. It owns request copying, asynchronous
// completion identity, body-frame relocation, stale-map revalidation and
// refresh timing; callers only provide the current request and consume a plan.
class LocalPlanTask {
 public:
  explicit LocalPlanTask(const LocalPlannerParams &params);
  ~LocalPlanTask();

  LocalPlanTask(const LocalPlanTask &) = delete;
  LocalPlanTask &operator=(const LocalPlanTask &) = delete;

  bool configure(const std::string &pathLibraryDir = {});
  [[nodiscard]] bool configured() const;
  LocalPlanUpdate update(const LocalPlanRequest &request);
  void reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local
