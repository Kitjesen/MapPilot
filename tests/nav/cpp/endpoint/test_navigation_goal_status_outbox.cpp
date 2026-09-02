#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "status/navigation_goal_status_outbox.hpp"

namespace {

using lingtu::message::NavigationGoalState;
using lingtu::nav::endpoint::GoalPlanStatus;
using lingtu::nav::endpoint::NavigationGoalStatusOutbox;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

GoalPlanStatus status(std::string task_id, std::string request_id, std::uint64_t goal_epoch,
                      NavigationGoalState state, std::string reason = "",
                      bool project_to_navigation_state = true) {
  GoalPlanStatus value;
  value.task_id = std::move(task_id);
  value.request_id = std::move(request_id);
  value.goal_epoch = goal_epoch;
  value.state = state;
  value.reason = std::move(reason);
  value.project_to_navigation_state = project_to_navigation_state;
  return value;
}

void testRecordObservesOnceAndFlushesInInsertionOrder() {
  std::vector<std::string> observed;
  std::vector<std::string> writes;
  NavigationGoalStatusOutbox outbox(
      [&](const GoalPlanStatus &value) {
        observed.push_back(value.task_id + ":" + value.request_id + ":" +
                           std::to_string(value.goal_epoch) + ":" +
                           std::to_string(static_cast<int>(value.state)));
      },
      [&](const GoalPlanStatus &value) {
        writes.push_back(value.reason);
        return true;
      });

  const auto first = status("task-a", "request-a", 1U, NavigationGoalState::Planning, "first");
  const auto duplicate_with_different_reason =
      status("task-a", "request-a", 1U, NavigationGoalState::Planning, "changed");
  const auto second = status("task-a", "request-a", 1U, NavigationGoalState::PathActive, "second");

  require(outbox.record(first), "first identity must be accepted");
  require(outbox.record(duplicate_with_different_reason),
          "reason is part of exact delivery identity");
  require(outbox.record(second), "state is part of identity");
  require(observed.size() == 3U, "observe must run exactly once per identity");

  require(outbox.flush() == 3U, "flush must report delivered count");
  require((writes == std::vector<std::string>{"first", "changed", "second"}),
          "flush must preserve insertion order and unchanged records");
  require(outbox.delivered(first), "delivered query must recognize delivered records");

  require(!outbox.record(first), "delivered duplicates must not be accepted");
  require(outbox.flush() == 0U, "delivered duplicates must not be re-written");
  require(observed.size() == 3U, "delivered duplicate must not re-observe");
  require(writes.size() == 3U, "delivered duplicate must not re-write");
}

void testDeliveredRequiresExactTerminalStatusIdentity() {
  NavigationGoalStatusOutbox outbox([](const GoalPlanStatus &) {},
                                    [](const GoalPlanStatus &) { return true; });

  const auto delivered =
      status("task-terminal", "request-terminal", 9U, NavigationGoalState::Cancelled,
             "operator_cancelled", true);
  const auto different_reason =
      status("task-terminal", "request-terminal", 9U, NavigationGoalState::Cancelled,
             "planner_cancelled", true);
  const auto different_projection =
      status("task-terminal", "request-terminal", 9U, NavigationGoalState::Cancelled,
             "operator_cancelled", false);

  require(outbox.record(delivered), "exact terminal status must be accepted");
  require(outbox.flush() == 1U, "exact terminal status must be delivered");

  require(!outbox.delivered(different_reason),
          "different reason must not be treated as delivered");
  require(!outbox.delivered(different_projection),
          "different navigation-state projection must not be treated as delivered");
  require(outbox.delivered(delivered), "identical terminal status must be treated as delivered");
}

void testFailedWriteBlocksLaterRecordsUntilRetried() {
  std::vector<std::string> writes;
  bool allow_write = false;
  NavigationGoalStatusOutbox outbox([](const GoalPlanStatus &) {},
                                    [&](const GoalPlanStatus &value) {
                                      writes.push_back(value.reason);
                                      return allow_write;
                                    });

  const auto planning =
      status("task-queue", "request-queue", 3U, NavigationGoalState::Planning, "planning");
  const auto path_active =
      status("task-queue", "request-queue", 3U, NavigationGoalState::PathActive, "path_active");
  require(outbox.record(planning), "planning record must be accepted");
  require(outbox.record(path_active), "path-active record must be accepted");

  require(outbox.flush() == 0U, "failed head write must deliver nothing");
  require((writes == std::vector<std::string>{"planning"}),
          "failed head record must block later status writes");
  require(!outbox.delivered(planning), "failed planning status must remain pending");
  require(!outbox.delivered(path_active), "blocked path-active status must remain pending");

  allow_write = true;
  require(outbox.flush() == 2U, "retry must deliver head before later records");
  require((writes == std::vector<std::string>{"planning", "planning", "path_active"}),
          "retry must preserve queue order after the blocked head succeeds");
  require(outbox.delivered(planning), "planning status must be delivered after retry");
  require(outbox.delivered(path_active), "path-active status must deliver only after planning");
}
void testFailedWriteIsRetriedUnchanged() {
  std::vector<std::string> writes;
  bool allow_write = false;
  NavigationGoalStatusOutbox outbox([](const GoalPlanStatus &) {},
                                    [&](const GoalPlanStatus &value) {
                                      writes.push_back(value.reason);
                                      return allow_write;
                                    });

  const auto failed = status("task-b", "request-b", 2U, NavigationGoalState::Failed, "blocked");
  require(outbox.record(failed), "record before failed write must be accepted");
  require(outbox.flush() == 0U, "failed write must remain undelivered");
  require(!outbox.delivered(failed), "failed write must not mark delivery");

  allow_write = true;
  require(outbox.flush() == 1U, "later flush must retry retained record");
  require((writes == std::vector<std::string>{"blocked", "blocked"}),
          "retry must use the original unchanged record");
  require(outbox.delivered(failed), "retry success must mark delivery");
}

void testMalformedIdentityFailsClosed() {
  int observed = 0;
  int writes = 0;
  NavigationGoalStatusOutbox outbox([&](const GoalPlanStatus &) { ++observed; },
                                    [&](const GoalPlanStatus &) {
                                      ++writes;
                                      return true;
                                    });

  require(!outbox.record(status("", "request", 1U, NavigationGoalState::Planning)),
          "empty task id must fail closed");
  require(!outbox.record(status("task", "", 1U, NavigationGoalState::Planning)),
          "empty request id must fail closed");
  require(!outbox.record(status("task", "request", 0U, NavigationGoalState::Planning)),
          "zero goal epoch must fail closed");
  require(!outbox.record(status("task", "request", 1U, static_cast<NavigationGoalState>(999))),
          "unknown state must fail closed");
  require(outbox.flush() == 0U, "malformed records must not be queued");
  require(observed == 0 && writes == 0, "malformed identities must not invoke callbacks");
}

}  // namespace

int main() {
  try {
    testRecordObservesOnceAndFlushesInInsertionOrder();
    testDeliveredRequiresExactTerminalStatusIdentity();
    testFailedWriteIsRetriedUnchanged();
    testFailedWriteBlocksLaterRecordsUntilRetried();
    testMalformedIdentityFailsClosed();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_navigation_goal_status_outbox: FAIL: %s\n", exc.what());
    return 1;
  }
}
