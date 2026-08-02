#pragma once

#include <functional>
#include <string>

#include "motion/command_ack_journal.hpp"

namespace lingtu::nav::endpoint {

class GoalPlanController;
class GoalReplanRuntimeCoordinator;
struct GoalReplanRuntimeResult;

struct GoalTaskCancelRequest {
  std::string task_id;
  std::string cancel_request_id;
  std::string reason;
  double steady_now_s{0.0};
};

struct GoalTaskCancelTerminalServiceResult {
  bool action_committed{false};
  std::string reason;
};

// Transport-free typed TaskCancel routing. GoalPlanController remains the sole
// owner of task identity; this router retains no lifecycle state.
class GoalTaskCancelRouter {
 public:
  using TerminalService = std::function<GoalTaskCancelTerminalServiceResult(
      const GoalReplanRuntimeResult &runtime_result)>;
  using StatusRequest = std::function<void()>;

  GoalTaskCancelRouter(GoalPlanController &goal_plan, GoalReplanRuntimeCoordinator &runtime,
                       TerminalService terminal_service, StatusRequest request_status);

  [[nodiscard]] CommandAck handle(const GoalTaskCancelRequest &request);

 private:
  GoalPlanController &goal_plan_;
  GoalReplanRuntimeCoordinator &runtime_;
  TerminalService terminal_service_;
  StatusRequest request_status_;
};

}  // namespace lingtu::nav::endpoint
