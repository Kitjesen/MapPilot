#pragma once

#include <memory>
#include <functional>

#include "planning/local/planner.hpp"

namespace nav_kernel::local::scan {

namespace upstream { struct TrajectoryHandoff; }

struct ExecutionFeedback {
  double nowS{0.0};
  std::int64_t trajectoryId{0};
  double startTimeS{0.0};
  bool frozen{false};
};

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
  void setExecutionFeedback(std::function<ExecutionFeedback()> feedback);
  std::shared_ptr<upstream::TrajectoryHandoff> handoff() const;
  void reset();
  LocalPlannerDebugSnapshot debugSnapshot() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace nav_kernel::local::scan
