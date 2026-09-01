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

// Latest-only SCAN worker. It snapshots each collision generation once and
// retains the last committed world-frame spline while the same guide replans.
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
