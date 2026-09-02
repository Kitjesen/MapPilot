#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>

#include "runtime/goal/retry.hpp"

namespace {

using lingtu::nav::endpoint::BoundedGoalReplanAction;
using lingtu::nav::endpoint::BoundedGoalReplanConfig;
using lingtu::nav::endpoint::BoundedGoalReplanController;
using lingtu::nav::endpoint::BoundedGoalReplanGoal;
using lingtu::nav::endpoint::BoundedGoalReplanState;

void require(bool condition, const char *message) {
  if (!condition) {
    std::fprintf(stderr, "test_bounded_goal_replan_controller: FAIL: %s\n", message);
    std::exit(1);
  }
}

lingtu::nav::plan::MapIdentity mapIdentity(std::string id = "map-a", std::int64_t version = 1,
                                           std::string frame = "map") {
  return {std::move(id), version, std::move(frame)};
}

BoundedGoalReplanGoal goal(std::string task = "task-a", std::string request = "req-a",
                           std::uint64_t epoch = 1U,
                           lingtu::nav::plan::MapIdentity map = mapIdentity()) {
  return {std::move(task), std::move(request), epoch, std::move(map)};
}

void expectAction(BoundedGoalReplanAction actual, BoundedGoalReplanAction expected,
                  const char *message) {
  require(actual == expected, message);
}

}  // namespace

int main() {
  BoundedGoalReplanController controller;
  const auto active = goal();
  expectAction(controller.observeGoal(active, 1.0).action, BoundedGoalReplanAction::kNone,
               "new goal observe did not stay passive");
  require(controller.snapshot().map_identity.has_value(),
          "snapshot did not expose typed map identity");
  auto armed = controller.armAfterConfirmedStop(active, 1.0, true);
  expectAction(armed.action, BoundedGoalReplanAction::kHold,
               "confirmed stop did not arm backoff");
  require(controller.snapshot().state == BoundedGoalReplanState::kBackoffPending,
          "arm did not enter backoff pending");
  require(controller.snapshot().budget_consumed, "arm did not consume budget");
  require(std::abs(controller.snapshot().deadline_s - 1.5) < 1e-9,
          "default backoff deadline is not exactly 0.5s");
  expectAction(controller.tick(active, 1.499).action, BoundedGoalReplanAction::kHold,
               "tick before deadline did not hold");
  expectAction(controller.tick(active, 1.5).action, BoundedGoalReplanAction::kStart,
               "tick at exact deadline did not start");
  expectAction(controller.tick(active, 1.6).action, BoundedGoalReplanAction::kNone,
               "repeated tick emitted a second start");
  require(controller.snapshot().start_emitted, "snapshot did not record emitted start");

  BoundedGoalReplanController zero_backoff({0.0});
  (void)zero_backoff.observeGoal(active, 1.0);
  (void)zero_backoff.armAfterConfirmedStop(active, 1.0, true);
  expectAction(zero_backoff.tick(active, 1.0).action, BoundedGoalReplanAction::kStart,
               "zero backoff did not start immediately at same timestamp");

  BoundedGoalReplanController repeat_arm;
  (void)repeat_arm.observeGoal(active, 2.0);
  (void)repeat_arm.armAfterConfirmedStop(active, 2.0, true);
  const double first_deadline = repeat_arm.snapshot().deadline_s;
  expectAction(repeat_arm.armAfterConfirmedStop(active, 2.2, true).action,
               BoundedGoalReplanAction::kHold,
               "repeat arm did not hold existing pending backoff");
  require(repeat_arm.snapshot().deadline_s == first_deadline,
          "repeat arm reset the pending deadline");

  BoundedGoalReplanController unconfirmed;
  (void)unconfirmed.observeGoal(active, 3.0);
  expectAction(unconfirmed.armAfterConfirmedStop(active, 3.0, false).action,
               BoundedGoalReplanAction::kConsume,
               "unconfirmed stop did not consume budget fail-closed");
  expectAction(unconfirmed.armAfterConfirmedStop(active, 3.1, true).action,
               BoundedGoalReplanAction::kReject,
               "consumed budget allowed a second arm");

  BoundedGoalReplanController epoch_budget;
  (void)epoch_budget.observeGoal(active, 4.0);
  (void)epoch_budget.armAfterConfirmedStop(active, 4.0, true);
  (void)epoch_budget.tick(active, 4.5);
  (void)epoch_budget.completeReplan(active, 4.6, true);
  const auto same_request_new_epoch = goal("task-a", "req-a", 2U, mapIdentity());
  expectAction(epoch_budget.observeGoal(same_request_new_epoch, 4.7).action,
               BoundedGoalReplanAction::kNone,
               "consumed same request epoch advance returned cancel");
  require(epoch_budget.snapshot().reason == "active_goal_epoch_advanced_budget_retained" &&
              epoch_budget.snapshot().budget_consumed,
          "consumed same request epoch advance did not retain budget state");
  expectAction(epoch_budget.armAfterConfirmedStop(same_request_new_epoch, 4.8, true).action,
               BoundedGoalReplanAction::kReject,
               "same task/request got a second replan budget after epoch advance");

  const auto other_request = goal("task-a", "req-b", 1U, mapIdentity());
  expectAction(epoch_budget.observeGoal(other_request, 5.0).action, BoundedGoalReplanAction::kNone,
               "new request after consumed did not observe passively");
  require(!epoch_budget.snapshot().budget_consumed,
          "new task/request did not restore budget");

  BoundedGoalReplanController pending_supersede;
  (void)pending_supersede.observeGoal(active, 5.5);
  (void)pending_supersede.armAfterConfirmedStop(active, 5.5, true);
  const auto other_task = goal("task-b", "req-c", 1U, mapIdentity("map-c", 1));
  expectAction(pending_supersede.observeGoal(other_task, 5.6).action,
               BoundedGoalReplanAction::kCancel,
               "new goal did not cancel pending old goal");
  require(pending_supersede.snapshot().task_id == "task-b" &&
              !pending_supersede.snapshot().budget_consumed,
          "new goal did not bind fresh budget after superseding pending");

  BoundedGoalReplanController inflight_supersede;
  (void)inflight_supersede.observeGoal(active, 5.7);
  (void)inflight_supersede.armAfterConfirmedStop(active, 5.7, true);
  (void)inflight_supersede.tick(active, 6.2);
  expectAction(inflight_supersede.observeGoal(other_task, 6.3).action,
               BoundedGoalReplanAction::kCancel,
               "new goal did not cancel in-flight old goal");
  require(inflight_supersede.snapshot().task_id == "task-b" &&
              !inflight_supersede.snapshot().budget_consumed,
          "new goal did not bind fresh budget after superseding in-flight");

  BoundedGoalReplanController stale_cases;
  (void)stale_cases.observeGoal(active, 7.0);
  (void)stale_cases.armAfterConfirmedStop(active, 7.0, true);
  const auto stale_epoch = goal("task-a", "req-a", 99U, mapIdentity());
  expectAction(stale_cases.tick(stale_epoch, 7.5).action, BoundedGoalReplanAction::kCancel,
               "stale epoch did not beat pending timer");
  require(stale_cases.snapshot().reason == "stale_active_goal_epoch",
          "stale epoch reason changed");

  BoundedGoalReplanController map_change;
  (void)map_change.observeGoal(active, 8.0);
  (void)map_change.armAfterConfirmedStop(active, 8.0, true);
  const auto changed_map = goal("task-a", "req-a", 1U, mapIdentity("map-b", 1));
  expectAction(map_change.tick(changed_map, 8.5).action, BoundedGoalReplanAction::kCancel,
               "map identity change did not beat pending timer");
  require(map_change.snapshot().reason == "stale_map_identity",
          "map identity stale reason changed");

  BoundedGoalReplanController old_completion;
  (void)old_completion.observeGoal(active, 9.0);
  (void)old_completion.armAfterConfirmedStop(active, 9.0, true);
  (void)old_completion.tick(active, 9.5);
  (void)old_completion.observeGoal(other_task, 9.6);
  expectAction(old_completion.completeReplan(active, 9.7, true).action,
               BoundedGoalReplanAction::kIgnore,
               "old completion polluted new goal");
  require(old_completion.snapshot().task_id == "task-b" &&
              old_completion.snapshot().request_id == "req-c" &&
              !old_completion.snapshot().budget_consumed,
          "old completion changed new goal snapshot");

  BoundedGoalReplanController cancel_case;
  (void)cancel_case.observeGoal(active, 10.0);
  (void)cancel_case.armAfterConfirmedStop(active, 10.0, true);
  expectAction(cancel_case.cancel(active, 10.1, "operator_cancelled").action,
               BoundedGoalReplanAction::kCancel,
               "cancel did not cancel pending replan");
  require(cancel_case.snapshot().state == BoundedGoalReplanState::kAttemptConsumed &&
              cancel_case.snapshot().reason == "operator_cancelled",
          "cancel did not consume attempt with reason");

  BoundedGoalReplanController invalid_goal;
  expectAction(invalid_goal.observeGoal(goal("", "req", 1U, mapIdentity()), 10.2).action,
               BoundedGoalReplanAction::kReject,
               "empty task id was accepted");
  expectAction(invalid_goal.observeGoal(goal("task", "", 1U, mapIdentity()), 10.3).action,
               BoundedGoalReplanAction::kReject,
               "empty request id was accepted");
  expectAction(invalid_goal.observeGoal(goal("task", "req", 1U,
                                             lingtu::nav::plan::MapIdentity{}),
                                        10.4)
                   .action,
               BoundedGoalReplanAction::kReject,
               "invalid map identity was accepted");

  BoundedGoalReplanController invalid_time;
  (void)invalid_time.observeGoal(active, 11.0);
  expectAction(invalid_time.armAfterConfirmedStop(active,
                                                  std::numeric_limits<double>::quiet_NaN(), true)
                   .action,
               BoundedGoalReplanAction::kReject,
               "NaN time did not fail closed");
  require(invalid_time.snapshot().budget_consumed,
          "NaN time did not consume current budget");

  BoundedGoalReplanController infinite_time;
  (void)infinite_time.observeGoal(active, 12.0);
  expectAction(infinite_time.tick(active, std::numeric_limits<double>::infinity()).action,
               BoundedGoalReplanAction::kReject,
               "infinite time did not fail closed");

  BoundedGoalReplanController clock_regression;
  (void)clock_regression.observeGoal(active, 13.0);
  expectAction(clock_regression.armAfterConfirmedStop(active, 12.9, true).action,
               BoundedGoalReplanAction::kReject,
               "clock regression did not fail closed");
  require(clock_regression.snapshot().reason == "clock_regression_budget_consumed",
          "clock regression reason changed");

  BoundedGoalReplanController overflow({1.0});
  (void)overflow.observeGoal(active, 14.0);
  expectAction(overflow.armAfterConfirmedStop(active, std::numeric_limits<double>::max(), true)
                   .action,
               BoundedGoalReplanAction::kReject,
               "deadline overflow did not fail closed");
  require(overflow.snapshot().reason == "deadline_overflow_budget_consumed",
          "deadline overflow reason changed");

  bool threw_negative = false;
  try {
    BoundedGoalReplanController negative({-0.1});
  } catch (const std::invalid_argument &) {
    threw_negative = true;
  }
  require(threw_negative, "negative backoff config was accepted");

  bool threw_nan = false;
  try {
    BoundedGoalReplanController nan_backoff(
        {std::numeric_limits<double>::quiet_NaN()});
  } catch (const std::invalid_argument &) {
    threw_nan = true;
  }
  require(threw_nan, "NaN backoff config was accepted");

  bool threw_inf = false;
  try {
    BoundedGoalReplanController inf_backoff({std::numeric_limits<double>::infinity()});
  } catch (const std::invalid_argument &) {
    threw_inf = true;
  }
  require(threw_inf, "+Inf backoff config was accepted");

  BoundedGoalReplanController reset_case;
  (void)reset_case.observeGoal(active, 15.0);
  (void)reset_case.armAfterConfirmedStop(active, 15.0, true);
  reset_case.reset();
  require(reset_case.snapshot().state == BoundedGoalReplanState::kIdle &&
              reset_case.snapshot().reason == "reset" &&
              reset_case.snapshot().task_id.empty() &&
              !reset_case.snapshot().map_identity.has_value(),
          "reset snapshot changed");

  return 0;
}
