#pragma once

#include "planning/local/planner.hpp"

#include <memory>
#include <string>

namespace nav_kernel::local::cmu {

// The fixed path-library backend. This interface is private to Local Planner;
// callers select a backend through local::Planner.
class Backend {
 public:
  explicit Backend(const LocalPlannerParams& params);
  ~Backend();

  Backend(Backend&&) noexcept;
  Backend& operator=(Backend&&) noexcept;
  Backend(const Backend&) = delete;
  Backend& operator=(const Backend&) = delete;

  bool loadPaths(const std::string& pathsDir);
  bool pathsLoaded() const;
  LocalPlannerDebugSnapshot debugSnapshot() const;

  LocalPlan plan(const LocalPlanRequest& request);
  void reset();

  const LocalPlannerParams& params() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local::cmu
