#pragma once

#include <memory>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

class Backend {
 public:
  static double fsmPeriodS() noexcept;
  static double collisionPeriodS() noexcept;

  explicit Backend(const LocalPlannerParams &params);
  ~Backend();

  Backend(Backend &&) noexcept;
  Backend &operator=(Backend &&) noexcept;
  Backend(const Backend &) = delete;
  Backend &operator=(const Backend &) = delete;

  LocalPlan tick(const LocalPlanRequest &request,
                 const LocalPlanCancel &cancel = {});
  LocalPlan checkCollision(const LocalPlanRequest &request,
                           const LocalPlanCancel &cancel = {});
  void reset();
  LocalPlannerDebugSnapshot debugSnapshot() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local::scan
