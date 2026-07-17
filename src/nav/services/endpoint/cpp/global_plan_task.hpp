#pragma once

#include "global_planner_contract.hpp"
#include "nav_kernel/types.hpp"

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>

namespace lingtu::nav::endpoint {

struct GlobalPlanContext {
  std::string request_id;
  nav_kernel::Vec3 start{};
  nav_kernel::Vec3 goal{};
  std::optional<double> goal_yaw;
  lingtu::nav::plan::GlobalPlanRequest request{};
};

struct GlobalPlanCompletion {
  GlobalPlanContext context{};
  lingtu::nav::plan::GlobalPlanResult result{};
  std::string error;
};

class GlobalPlanTask {
 public:
  using Planner = lingtu::nav::plan::GlobalPlannerFunction;

  explicit GlobalPlanTask(Planner planner);
  ~GlobalPlanTask();

  GlobalPlanTask(const GlobalPlanTask&) = delete;
  GlobalPlanTask& operator=(const GlobalPlanTask&) = delete;

  bool start(GlobalPlanContext context);
  std::optional<GlobalPlanCompletion> poll();
  void cancel();
  bool busy() const;

 private:
  Planner planner_;
  std::optional<GlobalPlanContext> context_;
  std::future<lingtu::nav::plan::GlobalPlanResult> future_;
  std::shared_ptr<std::atomic_bool> cancel_requested_;
  bool discard_result_{false};
};

}  // namespace lingtu::nav::endpoint
