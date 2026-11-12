#pragma once

#include <cstdint>
#include <memory>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

struct Update {
  LocalPlan plan{};
  LocalPlannerDebugSnapshot debug{};
};

// Runs SCAN on one worker thread while the control thread keeps following its plan.
// configure: start the worker and its 100 Hz FSM / 20 Hz collision timers.
// update: own the latest input, receive results, and accept a due trajectory handoff.
// pause: hold reference time; reset: cancel work and discard the accepted plan.
// Backend runs SCAN callbacks. Task owns input lifetime and result delivery.
class Task {
 public:
  explicit Task(const LocalPlannerParams &params);
  ~Task();

  Task(const Task &) = delete;
  Task &operator=(const Task &) = delete;

  bool configure();
  // Views are borrowed for this call. The worker owns immutable route/map
  // snapshots; replacing route geometry requires a new reference generation.
  Update update(const LocalPlanRequest &request);
  // Last accepted worker failure, independent of plan polling and resets.
  [[nodiscard]] std::shared_ptr<const ScanFailureSnapshot> lastScanFailure() const;
  void pause();
  void reset();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local::scan
