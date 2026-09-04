#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

struct Update {
  LocalPlan plan{};
  LocalPlannerDebugSnapshot debug{};
};

// Serialized SCAN timer runtime. One worker owns the mutable upstream FSM and
// runs its 100 Hz state callback and independent 20 Hz collision callback.
class Task {
 public:
  explicit Task(const LocalPlannerParams &params);
  ~Task();

  Task(const Task &) = delete;
  Task &operator=(const Task &) = delete;

  bool configure(const std::string &pathLibraryDir = {});
  [[nodiscard]] bool configured() const;
  Update update(const LocalPlanRequest &request);
  void reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local::scan
