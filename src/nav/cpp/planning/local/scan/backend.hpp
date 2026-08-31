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

  LocalPlan plan(const LocalPlanRequest &request);
  LocalPlan plan(const LocalPlanRequest &request, const LocalPlanCancel &cancel);
  [[nodiscard]] bool pathSafe(const LocalPlanRequest &request,
                              const std::vector<Vec3> &planningPath) const;
  void reset();
  LocalPlannerDebugSnapshot debugSnapshot() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local::scan
