#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "inspection/inspection_runtime_controller.hpp"

namespace {

using lingtu::nav::endpoint::InspectionRuntimeConfig;
using lingtu::nav::endpoint::InspectionRuntimeController;
using lingtu::nav::endpoint::InspectionRuntimeEvidenceResult;
using lingtu::nav::endpoint::InspectionRuntimeIntentKind;
using lingtu::nav::endpoint::InspectionRuntimeMapIdentity;
using lingtu::nav::endpoint::InspectionRuntimeRobotPosition;
using lingtu::nav::endpoint::InspectionRuntimeTickInput;
using lingtu::nav::inspection::Executor;
using lingtu::nav::inspection::FailurePolicy;
using lingtu::nav::inspection::Point;
using lingtu::nav::inspection::Route;
using lingtu::nav::inspection::RunState;

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

Route makeRoute(std::string action = {}, double dwell_s = 0.0) {
  Route route;
  route.id = "route-a";
  route.name = "Route A";
  route.map_id = "map-a";
  route.map_version = 7;
  route.revision = 3U;
  route.loop_count = 1U;
  route.failure_policy = FailurePolicy::kStop;

  Point point;
  point.id = "point-a";
  point.x_m = 3.0;
  point.y_m = 4.0;
  point.dwell_s = dwell_s;
  point.action = std::move(action);
  route.points.push_back(std::move(point));
  return route;
}

void start(Executor &executor, Route route, double now_s = 1.0) {
  std::string error;
  require(executor.Start(std::move(route), "run-a", "map-a", 7, now_s, &error),
          "route starts: " + error);
}

InspectionRuntimeTickInput inputAt(double now_s) {
  InspectionRuntimeTickInput input;
  input.now_s = now_s;
  input.active_map = InspectionRuntimeMapIdentity{"map-a", 7};
  return input;
}

void startNavigation(Executor &executor, InspectionRuntimeController &runtime, double now_s = 1.0) {
  auto input = inputAt(now_s);
  const auto tick = runtime.tick(input);
  require(tick.goal_dispatch.has_value(), "pending point becomes a goal intent");
  require(tick.goal_dispatch->point.id == "point-a", "goal intent preserves the active point");

  const auto duplicate = runtime.tick(input);
  require(!duplicate.goal_dispatch.has_value(), "outstanding goal dispatch is not duplicated");

  const auto completion = runtime.completeGoalDispatch(true, "accepted", now_s + 0.01);
  require(completion.consumed, "goal completion is consumed");
  require(!completion.clear_motion_reason.has_value(),
          "on-time accepted goal needs no stop intent");
  require(executor.status().reason == "planning_started",
          "accepted dispatch feeds planning-started state");

  const auto repeated = runtime.completeGoalDispatch(true, "accepted", now_s + 0.02);
  require(!repeated.consumed, "duplicate completion is ignored");
  require(executor.OnPlanReady(now_s + 0.03), "planner completion advances inspection navigation");
}

void reachActionPending(Executor &executor, InspectionRuntimeController &runtime,
                        bool worker_matched) {
  startNavigation(executor, runtime);
  auto moving = inputAt(1.1);
  moving.odom_generation = 10U;
  moving.path_active = true;
  moving.robot_position = InspectionRuntimeRobotPosition{0.0, 0.0};
  (void)runtime.tick(moving);

  runtime.onGoalReached(1.2);
  const double sample_times[] = {1.30, 1.55, 1.81};
  for (std::size_t index = 0U; index < 3U; ++index) {
    auto settling = inputAt(sample_times[index]);
    settling.odom_generation = 11U + index;
    settling.arrival_sample = lingtu::nav::inspection::ArrivalSample{sample_times[index], 0.0, 0.0};
    settling.evidence_worker_matched = worker_matched && index == 2U;
    const auto result = runtime.tick(settling);
    if (index == 2U && worker_matched) {
      require(result.evidence_dispatch.has_value(),
              "matched worker receives evidence dispatch intent");
      require(result.evidence_dispatch->map_id == "map-a" &&
                  result.evidence_dispatch->map_version == 7,
              "evidence intent carries immutable route map identity");
      require(result.evidence_dispatch->deadline_s == executor.status().deadline_s,
              "evidence intent carries the action deadline");
    }
  }
  require(executor.status().state == RunState::kActionPending,
          "arrival settlement advances into action pending");
}

void testStatusCadenceAndValidation() {
  Executor executor;
  InspectionRuntimeController runtime(executor, InspectionRuntimeConfig{1.0, 0.5});

  require(runtime.takeStatusDue(100.0), "first status is immediately due");
  require(!runtime.takeStatusDue(100.49), "periodic status waits for cadence");
  runtime.requestStatus();
  require(runtime.takeStatusDue(100.49), "explicit status request bypasses cadence");
  require(!runtime.takeStatusDue(100.98), "immediate publish resets cadence");
  require(runtime.takeStatusDue(100.99), "periodic status becomes due again");
  require(!runtime.takeStatusDue(std::numeric_limits<double>::quiet_NaN()),
          "invalid clock sample never emits status");

  bool rejected_map_interval = false;
  try {
    InspectionRuntimeController invalid(executor, InspectionRuntimeConfig{0.0, 0.5});
    (void)invalid;
  } catch (const std::invalid_argument &) {
    rejected_map_interval = true;
  }
  require(rejected_map_interval, "non-positive map cadence is rejected");

  bool rejected_status_interval = false;
  try {
    InspectionRuntimeController invalid(executor, InspectionRuntimeConfig{1.0, -1.0});
    (void)invalid;
  } catch (const std::invalid_argument &) {
    rejected_status_interval = true;
  }
  require(rejected_status_interval, "non-positive status cadence is rejected");
}

void testGoalDispatchAndProgress() {
  Executor executor;
  start(executor, makeRoute());
  InspectionRuntimeController runtime(executor);
  startNavigation(executor, runtime);

  require(runtime.activePoint().has_value(), "controller owns the active point");
  require(&runtime.executor() == &executor, "controller operates the endpoint-owned executor");

  const double initial_deadline = executor.status().deadline_s;
  auto progress = inputAt(2.0);
  progress.odom_generation = 4U;
  progress.path_active = true;
  progress.robot_position = InspectionRuntimeRobotPosition{0.0, 0.0};
  (void)runtime.tick(progress);
  require(executor.status().deadline_s > initial_deadline,
          "navigation progress refreshes the stall deadline");

  runtime.clearActivePoint();
  require(!runtime.activePoint().has_value(), "external goal clears active point");
}

void testGoalDispatchFailureFeedback() {
  {
    Executor executor;
    start(executor, makeRoute());
    InspectionRuntimeController runtime(executor);
    const auto tick = runtime.tick(inputAt(1.1));
    require(tick.goal_dispatch.has_value(), "goal intent emitted");
    const auto completion = runtime.completeGoalDispatch(false, "goal_rejected", 1.2);
    require(completion.consumed, "rejected dispatch completion consumed");
    require(!completion.clear_motion_reason.has_value(),
            "goal admission rejection does not add a motion clear");
    require(executor.status().state == RunState::kFailed &&
                executor.status().reason == "goal_rejected",
            "goal rejection feeds the established leg failure path");
  }

  {
    Executor executor;
    start(executor, makeRoute(), 1.0);
    InspectionRuntimeController runtime(executor);
    const auto tick = runtime.tick(inputAt(1.1));
    require(tick.goal_dispatch.has_value(), "late completion setup emits goal");
    const auto completion = runtime.completeGoalDispatch(true, "accepted", 31.0);
    require(completion.clear_motion_reason == std::optional<std::string>{"planning_timeout"},
            "late planning start requests fail-closed motion clear");
  }
}

void testOnlyPostArrivalOdometrySettles() {
  Executor executor;
  start(executor, makeRoute({}, 5.0));
  InspectionRuntimeController runtime(executor);
  startNavigation(executor, runtime);

  auto before_arrival = inputAt(1.1);
  before_arrival.odom_generation = 20U;
  before_arrival.arrival_sample = {1.1, 0.0, 0.0};
  (void)runtime.tick(before_arrival);
  runtime.onGoalReached(1.2);

  auto stale_generation = inputAt(1.3);
  stale_generation.odom_generation = 20U;
  stale_generation.arrival_sample = {1.3, 0.0, 0.0};
  (void)runtime.tick(stale_generation);
  require(executor.status().stable_since_s == 0.0,
          "odometry received before arrival is not reused for settling");

  auto first = inputAt(1.31);
  first.odom_generation = 21U;
  first.arrival_sample = {1.31, 0.0, 0.0};
  (void)runtime.tick(first);
  require(executor.status().stable_since_s == 1.31, "first new odometry starts settlement window");

  auto duplicate_generation = inputAt(1.56);
  duplicate_generation.odom_generation = 21U;
  duplicate_generation.arrival_sample = {1.56, 0.0, 0.0};
  (void)runtime.tick(duplicate_generation);

  auto second = inputAt(1.57);
  second.odom_generation = 22U;
  second.arrival_sample = {1.57, 0.0, 0.0};
  (void)runtime.tick(second);
  require(executor.status().state == RunState::kSettling,
          "duplicate odometry generation cannot count as another sample");

  auto third = inputAt(1.83);
  third.odom_generation = 23U;
  third.arrival_sample = {1.83, 0.0, 0.0};
  (void)runtime.tick(third);
  require(executor.status().state == RunState::kDwelling,
          "three fresh post-arrival generations complete settling");
}

void testEvidenceDispatchFailure() {
  Executor executor;
  start(executor, makeRoute("capture"));
  InspectionRuntimeController runtime(executor);
  reachActionPending(executor, runtime, true);

  const std::string request_id = executor.status().action_request_id;
  const auto wrong = runtime.completeEvidenceDispatch("wrong-request", false, 1.9);
  require(!wrong.consumed, "mismatched evidence completion is ignored");

  const auto completion = runtime.completeEvidenceDispatch(request_id, false, 1.9);
  require(completion.consumed, "evidence publication completion consumed");
  require(completion.clear_motion_reason ==
              std::optional<std::string>{"evidence_request_publish_failed"},
          "failed evidence publication requests established clear reason");
  require(executor.status().state == RunState::kFailed &&
              executor.status().reason == "evidence_request_publish_failed",
          "failed publication is fed back to executor");
}

void testEvidenceResultOrderingAndFallbackReason() {
  {
    Executor executor;
    start(executor, makeRoute("capture"));
    InspectionRuntimeController runtime(executor);
    reachActionPending(executor, runtime, true);
    const std::string request_id = executor.status().action_request_id;
    require(runtime.completeEvidenceDispatch(request_id, true, 1.9).consumed,
            "successful evidence publication completion consumed");

    auto evidence = inputAt(2.0);
    evidence.evidence_results.push_back(
        InspectionRuntimeEvidenceResult{request_id, true, "evidence-1", ""});
    const auto result = runtime.tick(evidence);
    require(result.ordered_intents.size() == 1U &&
                result.ordered_intents.front().kind == InspectionRuntimeIntentKind::kClearMotion &&
                result.ordered_intents.front().reason == "route_complete",
            "evidence transition clears motion before later dispatch intents");
    require(executor.status().state == RunState::kSucceeded,
            "persisted evidence completes single-point route");
  }

  {
    Executor executor;
    start(executor, makeRoute("capture"));
    InspectionRuntimeController runtime(executor);
    reachActionPending(executor, runtime, true);
    const std::string request_id = executor.status().action_request_id;
    require(runtime.completeEvidenceDispatch(request_id, true, 1.9).consumed,
            "fallback setup publishes evidence request");

    auto evidence = inputAt(2.0);
    evidence.evidence_results.push_back(InspectionRuntimeEvidenceResult{request_id, false, "", ""});
    const auto result = runtime.tick(evidence);
    require(executor.status().reason == "evidence_not_persisted",
            "empty negative result receives established fallback reason");
    require(!result.ordered_intents.empty() &&
                result.ordered_intents.front().reason == "evidence_not_persisted",
            "negative evidence result emits matching clear-motion intent");
  }
}

void testWorkerMatchGatesEvidenceDispatch() {
  Executor executor;
  start(executor, makeRoute("capture"));
  InspectionRuntimeController runtime(executor);
  reachActionPending(executor, runtime, false);

  auto unmatched = inputAt(1.9);
  unmatched.evidence_worker_matched = false;
  const auto blocked = runtime.tick(unmatched);
  require(!blocked.evidence_dispatch.has_value() &&
              executor.status().reason == "point_action_pending",
          "unmatched worker leaves action pending and undispatched");

  auto matched = inputAt(2.0);
  matched.evidence_worker_matched = true;
  const auto dispatched = runtime.tick(matched);
  require(dispatched.evidence_dispatch.has_value() &&
              executor.status().reason == "point_action_started",
          "matched worker starts and emits evidence action");
}

void testActionTimeoutReleasesEvidenceDispatchLatch() {
  Executor executor;
  auto route = makeRoute("capture");
  route.failure_policy = FailurePolicy::kRetry;
  route.max_retries = 1U;
  start(executor, std::move(route));
  InspectionRuntimeController runtime(executor);
  reachActionPending(executor, runtime, true);

  const std::string first_request_id = executor.status().action_request_id;
  require(runtime.completeEvidenceDispatch(first_request_id, true, 1.9).consumed,
          "first evidence request is published");
  const double action_deadline = executor.status().deadline_s;

  const auto timeout = runtime.tick(inputAt(action_deadline));
  require(executor.status().state == RunState::kPlanning && executor.status().retry_count == 1U,
          "action timeout enters configured retry");
  require(!timeout.ordered_intents.empty() &&
              timeout.ordered_intents.front().reason == "action_timeout",
          "action timeout preserves fail-closed motion reason");
  require(timeout.goal_dispatch.has_value(), "retry emits a new goal intent");

  require(runtime.completeGoalDispatch(true, "accepted", action_deadline + 0.01).consumed,
          "retry goal dispatch completes");
  require(executor.OnPlanReady(action_deadline + 0.02), "retry plan advances to navigation");
  runtime.onGoalReached(action_deadline + 0.03);

  std::optional<lingtu::nav::endpoint::InspectionEvidenceDispatchIntent> second_dispatch;
  const double sample_times[] = {
      action_deadline + 0.10,
      action_deadline + 0.35,
      action_deadline + 0.61,
  };
  for (std::size_t index = 0U; index < 3U; ++index) {
    auto settling = inputAt(sample_times[index]);
    settling.odom_generation = 100U + index;
    settling.arrival_sample = lingtu::nav::inspection::ArrivalSample{sample_times[index], 0.0, 0.0};
    settling.evidence_worker_matched = true;
    const auto result = runtime.tick(settling);
    if (result.evidence_dispatch) {
      second_dispatch = result.evidence_dispatch;
    }
  }
  require(second_dispatch.has_value(), "retry action is not blocked by stale latch");
  require(second_dispatch->action.request_id != first_request_id,
          "retry action receives a fresh request id");
}

void testTimeoutAndMapChangeIntents() {
  {
    Executor executor;
    start(executor, makeRoute(), 1.0);
    InspectionRuntimeController runtime(executor);
    auto timed_out = inputAt(31.0);
    const auto result = runtime.tick(timed_out);
    require(result.ordered_intents.size() == 1U &&
                result.ordered_intents.front().kind == InspectionRuntimeIntentKind::kClearMotion &&
                result.ordered_intents.front().reason == "planning_timeout",
            "planning timeout emits exact fail-closed reason");
  }

  {
    Executor executor;
    start(executor, makeRoute(), 1.0);
    InspectionRuntimeController runtime(executor, InspectionRuntimeConfig{1.0, 0.5});

    auto initial = inputAt(1.1);
    initial.goal_plan_busy = true;
    (void)runtime.tick(initial);

    auto before_due = inputAt(1.5);
    before_due.active_map = InspectionRuntimeMapIdentity{"different-map", 8};
    before_due.goal_plan_busy = true;
    const auto early = runtime.tick(before_due);
    require(early.ordered_intents.empty() && executor.active(),
            "map identity is checked only at configured cadence");

    auto due = inputAt(2.1);
    due.active_map = InspectionRuntimeMapIdentity{"different-map", 8};
    const auto changed = runtime.tick(due);
    require(changed.ordered_intents.size() == 2U,
            "map change emits authority stop and motion clear");
    require(changed.ordered_intents[0].kind == InspectionRuntimeIntentKind::kStopControlAuthority &&
                changed.ordered_intents[0].reason == "inspection_active_map_changed",
            "authority stop is ordered before motion clear");
    require(changed.ordered_intents[1].kind == InspectionRuntimeIntentKind::kClearMotion &&
                changed.ordered_intents[1].reason == "inspection_active_map_changed",
            "map change uses established clear-motion reason");
    require(executor.status().state == RunState::kFailed &&
                executor.status().reason == "active_map_changed",
            "executor retains canonical map failure reason");
  }
}

}  // namespace

int main() {
  testStatusCadenceAndValidation();
  testGoalDispatchAndProgress();
  testGoalDispatchFailureFeedback();
  testOnlyPostArrivalOdometrySettles();
  testEvidenceDispatchFailure();
  testEvidenceResultOrderingAndFallbackReason();
  testWorkerMatchGatesEvidenceDispatch();
  testActionTimeoutReleasesEvidenceDispatchLatch();
  testTimeoutAndMapChangeIntents();
  return 0;
}
