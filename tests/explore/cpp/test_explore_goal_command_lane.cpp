#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "endpoint/goal_command_lane.hpp"

namespace {

using lingtu::message::NavigationCommandKind;
using lingtu::nav::endpoint::ExploreGoalCommandDeadline;
using lingtu::nav::endpoint::ExploreGoalCommandLane;
using lingtu::nav::endpoint::ExploreGoalCommandWrite;
using lingtu::nav::endpoint::ExploreGoalTarget;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void requireSameCancel(const ExploreGoalCommandWrite &actual,
                       const ExploreGoalCommandWrite &expected) {
  require(actual.task_id == expected.task_id, "cancel retry changed task_id");
  require(actual.request_id == expected.request_id, "cancel retry changed request_id");
  require(actual.kind == expected.kind, "cancel retry changed kind");
  require(actual.reason == expected.reason, "cancel retry changed payload");
}

void testLostStartAckRetriesExactIdentity() {
  ExploreGoalCommandLane lane(1.0, 10.0);
  lane.beginGoal("explore-task-6", "explore-start-6", ExploreGoalTarget{1.0, 2.0, 0.3, 0.5}, 8.0);

  const auto first = lane.advance(8.0, true, {});
  require(first.size() == 1U, "first goal start was not written");
  require(first.front().task_id == "explore-task-6", "goal start changed task_id");
  require(first.front().request_id == "explore-start-6", "goal start changed request_id");
  require(first.front().kind == NavigationCommandKind::Goal, "first write was not goal start");
  require(first.front().reason.empty(), "goal start gained a reason payload");
  lane.recordWriteResult(first.front().request_id, true);

  require(lane.advance(8.5, true, {}).empty(), "goal start retried before retry interval");
  const auto retry = lane.advance(9.0, true, {});
  require(retry.size() == 1U, "lost goal ACK did not trigger retry");
  require(retry.front().task_id == first.front().task_id &&
              retry.front().request_id == first.front().request_id &&
              retry.front().kind == first.front().kind,
          "goal retry changed command identity");
}

void testLostCancelAckRetriesExactIdentity() {
  ExploreGoalCommandLane lane(1.0, 10.0);
  lane.beginGoal("explore-task-7", "explore-start-7", ExploreGoalTarget{1.0, 2.0, 0.3, 0.5}, 9.0);
  const auto start = lane.advance(9.0, true, {});
  require(start.size() == 1U, "goal start was not written before cancel");
  lane.recordWriteResult(start.front().request_id, true);
  (void)lane.requestCancel("exploration_stop", 10.0);

  const auto first = lane.advance(10.0, true, {});
  require(first.size() == 1U, "first cancel was not written");
  require(first.front().kind == NavigationCommandKind::TaskCancel,
          "first write was not task cancel");

  require(lane.advance(10.5, true, {}).empty(), "cancel retried before retry interval");

  const auto retry = lane.advance(11.0, true, {});
  require(retry.size() == 1U, "lost cancel ACK did not trigger retry");
  requireSameCancel(retry.front(), first.front());
}

void testGoalTerminalStopsCommandRetries() {
  ExploreGoalCommandLane lane(1.0, 10.0);
  lane.beginGoal("explore-task-8", "explore-start-8", ExploreGoalTarget{1.0, 2.0, 0.3, 0.5}, 20.0);
  require(lane.advance(20.0, true, {}).size() == 1U, "goal start was not written before terminal");
  lane.recordWriteResult("explore-start-8", true);
  (void)lane.requestCancel("exploration_stop", 20.1);
  require(lane.advance(20.1, true, {}).size() == 1U, "cancel was not written before terminal");

  lane.finishGoal();
  require(!lane.binding().has_value(), "goal terminal retained command binding");
  require(lane.advance(30.0, true, {}).empty(), "goal terminal allowed a later command retry");
}

void testCancelPreemptsUnacknowledgedStart() {
  ExploreGoalCommandLane lane(1.0, 10.0);
  lane.beginGoal("explore-task-9", "explore-start-9", ExploreGoalTarget{1.0, 2.0, 0.3, 0.5}, 40.0);
  require(lane.advance(40.0, true, {}).size() == 1U, "goal start was not written");
  lane.recordWriteResult("explore-start-9", true);

  (void)lane.requestCancel("exploration_stop", 40.1);
  const auto cancel = lane.advance(40.1, true, {});
  require(cancel.size() == 1U && cancel.front().kind == NavigationCommandKind::TaskCancel,
          "cancel waited for the outstanding start ACK");

  const std::vector<lingtu::nav::endpoint::ExploreGoalCommandAck> late_start_ack{
      {"explore-task-9", "explore-start-9", NavigationCommandKind::Goal, true, "planning_started"},
  };
  require(lane.advance(40.2, true, late_start_ack).empty(),
          "late start ACK produced another command before cancel retry was due");
  const auto cancel_retry = lane.advance(41.1, true, {});
  require(cancel_retry.size() == 1U &&
              cancel_retry.front().kind == NavigationCommandKind::TaskCancel &&
              cancel_retry.front().request_id == cancel.front().request_id,
          "late start ACK restored start or changed cancel identity");
}

void testCancelDropsGoalThatWasNeverWritten() {
  ExploreGoalCommandLane lane(1.0, 10.0);
  lane.beginGoal("explore-task-10", "explore-start-10", ExploreGoalTarget{1.0, 2.0, 0.3, 0.5},
                 50.0);
  require(lane.advance(50.0, false, {}).empty(),
          "unmatched transport unexpectedly wrote goal start");

  require(!lane.requestCancel("exploration_stop", 50.1),
          "never-written goal incorrectly required a nav cancel");
  require(!lane.binding().has_value(), "never-written cancelled goal retained command binding");
  require(lane.advance(60.0, true, {}).empty(),
          "never-written cancelled goal emitted a later command");
}

void testFailedStartWriteCanBeCancelledLocally() {
  ExploreGoalCommandLane lane(1.0, 10.0);
  lane.beginGoal("explore-task-11", "explore-start-11", ExploreGoalTarget{1.0, 2.0, 0.3, 0.5},
                 70.0);
  const auto writes = lane.advance(70.0, true, {});
  require(writes.size() == 1U, "goal start write was not attempted");
  lane.recordWriteResult(writes.front().request_id, false);

  require(!lane.requestCancel("exploration_stop", 70.1),
          "failed goal write incorrectly required a nav cancel");
  require(!lane.binding().has_value(),
          "failed goal write retained command binding after local cancel");
}

void testRejectedStartAckStopsSameIdentityRetry() {
  ExploreGoalCommandLane lane(1.0, 10.0);
  lane.beginGoal("explore-task-12", "explore-start-12", ExploreGoalTarget{1.0, 2.0, 0.3, 0.5},
                 80.0);
  const auto start = lane.advance(80.0, true, {});
  require(start.size() == 1U, "goal start was not written before rejection");
  lane.recordWriteResult(start.front().request_id, true);

  const std::vector<lingtu::nav::endpoint::ExploreGoalCommandAck> rejected_ack{
      {"explore-task-12", "explore-start-12", NavigationCommandKind::Goal, false,
       "goal_not_admitted"},
  };
  require(lane.advance(81.0, true, rejected_ack).empty(),
          "explicitly rejected goal retried the same command identity");
  require(lane.advance(90.0, true, {}).empty(),
          "explicitly rejected goal resumed retrying after the ACK tick");
}

void testStartDeadlineReportsOnceAndStopsRetries() {
  ExploreGoalCommandLane lane(1.0, 3.0);
  lane.beginGoal("task-deadline", "start-deadline", {}, 100.0);
  require(lane.advance(100.0, true, {}).size() == 1U, "start was not attempted");
  lane.recordWriteResult("start-deadline", true);

  require(lane.advance(103.0, true, {}).empty(), "start retried at its ACK deadline");
  require(lane.takeDeadline() == ExploreGoalCommandDeadline::StartAck,
          "start ACK deadline was not reported");
  require(!lane.takeDeadline().has_value(), "start ACK deadline was reported twice");
  require(lane.requestCancel("start_timeout", 103.0),
          "written timed-out start did not require cancellation");
  const auto cancel = lane.advance(103.0, true, {});
  require(cancel.size() == 1U && cancel.front().task_id == "task-deadline" &&
              cancel.front().request_id == "start-deadline-cancel" &&
              cancel.front().kind == NavigationCommandKind::TaskCancel,
          "start deadline did not produce a stable same-tick cancel");
}

void testAckWinsOnDeadlineTick() {
  ExploreGoalCommandLane lane(1.0, 3.0);
  lane.beginGoal("task-ack", "start-ack", {}, 200.0);
  require(lane.advance(200.0, true, {}).size() == 1U, "start was not attempted");
  lane.recordWriteResult("start-ack", true);
  const std::vector<lingtu::nav::endpoint::ExploreGoalCommandAck> acks{
      {"task-ack", "start-ack", NavigationCommandKind::Goal, true, "planning_started"},
  };
  require(lane.advance(203.0, true, acks).empty(), "ACK tick produced an extra start");
  require(!lane.takeDeadline().has_value(), "matching ACK lost to deadline check");
}

void testCancelTerminalDeadlineReportsOnceAndStopsRetries() {
  ExploreGoalCommandLane lane(1.0, 3.0);
  lane.beginGoal("task-cancel", "start-cancel", {}, 300.0);
  require(lane.advance(300.0, true, {}).size() == 1U, "start was not attempted");
  lane.recordWriteResult("start-cancel", true);
  require(lane.requestCancel("stop", 301.0), "written goal did not require cancel");
  require(lane.advance(301.0, true, {}).size() == 1U, "cancel was not attempted");

  require(lane.advance(304.0, true, {}).empty(), "cancel retried at terminal deadline");
  require(lane.takeDeadline() == ExploreGoalCommandDeadline::CancelTerminal,
          "cancel terminal deadline was not reported");
  require(!lane.takeDeadline().has_value(), "cancel terminal deadline was reported twice");
  require(lane.advance(310.0, true, {}).empty(), "expired cancel resumed retrying");
}

}  // namespace

int main() {
  try {
    testLostStartAckRetriesExactIdentity();
    testLostCancelAckRetriesExactIdentity();
    testGoalTerminalStopsCommandRetries();
    testCancelPreemptsUnacknowledgedStart();
    testCancelDropsGoalThatWasNeverWritten();
    testFailedStartWriteCanBeCancelledLocally();
    testRejectedStartAckStopsSameIdentityRetry();
    testStartDeadlineReportsOnceAndStopsRetries();
    testAckWinsOnDeadlineTick();
    testCancelTerminalDeadlineReportsOnceAndStopsRetries();
    return 0;
  } catch (const std::exception &exc) {
    std::fprintf(stderr, "test_explore_goal_command_lane: FAIL: %s\n", exc.what());
    return 1;
  }
}
