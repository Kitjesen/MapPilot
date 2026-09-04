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

// Serialized SCAN timer runtime. One worker owns the mutable upstream FSM and
// runs its 100 Hz state callback and independent 20 Hz collision callback.
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
