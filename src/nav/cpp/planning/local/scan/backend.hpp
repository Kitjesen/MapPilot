#pragma once

#include <memory>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

class Backend {
 public:
  explicit Backend(const LocalPlannerParams &params);
  ~Backend();

  Backend(Backend &&) noexcept;
  Backend &operator=(Backend &&) noexcept;
  Backend(const Backend &) = delete;
  Backend &operator=(const Backend &) = delete;

  LocalPlanResult plan(const LocalPlanInput &input);
  LocalPlanResult planIntent(const LocalPlanInput &input, const LocalMotionIntent &intent);
  void reset();
  LocalPlannerDebugSnapshot debugSnapshot() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local::scan
