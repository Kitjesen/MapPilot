#include "lingtu/recording/inspection_timeline.hpp"

#include <stdexcept>
#include <string>
#include <vector>

using lingtu::recording::DriverControlFact;
using lingtu::recording::FinalOutputFact;
using lingtu::recording::InspectionTaskEventFact;
using lingtu::recording::verify_inspection_task_timeline;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

InspectionTaskEventFact event(std::uint64_t time, std::uint64_t sequence,
                             std::int32_t state, std::string boot = "nav-a",
                             std::string reason = {}, std::string request = "start-request") {
  InspectionTaskEventFact fact;
  fact.observed_time_ns = time;
  fact.event_sequence = sequence;
  fact.kind = sequence == 41U ? 1 : 2;
  fact.task_id = "task-a";
  fact.request_id = std::move(request);
  fact.boot_id = std::move(boot);
  fact.map_id = "map-a";
  fact.map_content_epoch = 5;
  fact.route_id = "route-a";
  fact.route_revision = 4;
  fact.state = state;
  fact.reason = std::move(reason);
  fact.point_count = 2;
  return fact;
}

std::vector<InspectionTaskEventFact> resumed_events(bool waiting = false) {
  return {event(100, 41, 1), event(200, 42, waiting ? 14 : 3),
          event(300, 43, 5, "nav-b", waiting ? "restart_loop_wait_resume_required" : "restart_resume_required"),
          event(400, 44, waiting ? 14 : 2, "nav-b",
                waiting ? "waiting_next_loop" : "resumed_replan_current_leg", "resume-request"),
          event(700, 45, 7, "nav-b", "route_complete", "resume-request")};
}

int main() {
  const std::vector<FinalOutputFact> new_zero{{600, 7, 0, 0, 0, 0, 0, 0, "nav-b"}};
  const std::vector<DriverControlFact> new_ack{{650, true, true, 7, "nav-b"}};
  for (const bool waiting : {false, true}) {
    require(verify_inspection_task_timeline("task-a", resumed_events(waiting), new_zero, new_ack).ok,
            "checkpoint pause and explicit resume did not qualify");
  }
  auto events = resumed_events();
  events[2].state = 3;
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "an arbitrary new-boot navigation event qualified as recovery");
  events = resumed_events();
  events[3].request_id = "start-request";
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "recovery resumed without an explicit operator request");
  events = resumed_events();
  events[3].state = 3;
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "navigation bypassed the recovery resume fact");
  events = resumed_events();
  ++events[2].event_sequence;
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "checkpoint recovery filled a missing event");
  events = resumed_events();
  events[2].map_content_epoch = 6;
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "recovery changed the map identity");
  events = resumed_events();
  require(!verify_inspection_task_timeline("task-a", events, new_zero,
                                          {{650, true, true, 7, "nav-a"}}).ok,
          "pre-restart driver acknowledgement qualified a new-boot stop");
  require(!verify_inspection_task_timeline("task-a", events,
                                          {{600, 7, 0.1, 0, 0, 0, 0, 0, "nav-b"}}, new_ack).ok,
          "nonzero final command qualified after recovery");
  require(!verify_inspection_task_timeline("task-a", events, new_zero,
                                          {{650, true, true, 6, "nav-b"}}).ok,
          "an older output acknowledgement qualified the final stop");

  auto unrelated = event(150, 150, 3, "another-robot");
  unrelated.task_id = "task-unrelated";
  events.insert(events.begin() + 1, unrelated);
  unrelated.event_sequence = 153;
  unrelated.observed_time_ns = 750;
  events.push_back(unrelated);
  require(verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "an unrelated robot's incomplete history invalidated this task");

  auto accepted = event(100, 41, 1);
  auto terminal = event(700, 44, 9);
  unrelated = event(200, 42, 3);
  unrelated.task_id = "task-unrelated";
  auto middle = event(300, 43, 3);
  require(verify_inspection_task_timeline("task-a", {accepted, unrelated, middle, terminal},
                                          {{600, 7, 0, 0, 0, 0, 0, 0, "nav-a"}},
                                          {{650, true, true, 7, "nav-a"}}).ok,
          "a captured intermediate writer cursor was ignored");
  auto replay = terminal;
  replay.boot_id = "nav-b";
  replay.observed_time_ns = 800;
  require(verify_inspection_task_timeline("task-a", {accepted, unrelated, middle, terminal, replay},
                                          {{600, 7, 0, 0, 0, 0, 0, 0, "nav-a"}},
                                          {{650, true, true, 7, "nav-a"}}).ok,
          "a terminal checkpoint replay was treated as a second completion");
  replay.point_index = 1;
  require(!verify_inspection_task_timeline("task-a", {accepted, unrelated, middle, terminal, replay},
                                           {{600, 7, 0, 0, 0, 0, 0, 0, "nav-a"}},
                                           {{650, true, true, 7, "nav-a"}}).ok,
          "a different terminal fact was accepted as an unchanged replay");

  events = {event(100, 41, 1), event(200, 42, 13, "nav-a", "cancel_requested", "cancel-request"),
            event(700, 43, 9, "nav-b", "cancelled_after_restart", "cancel-request")};
  require(verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "pending cancellation did not survive a checkpoint restart");

  auto completed = event(200, 42, 4, "nav-a", "point_completed");
  completed.kind = 3;
  completed.point_index = 1;
  auto restored_completion = event(700, 43, 7, "nav-b", "route_complete");
  restored_completion.point_index = restored_completion.point_count;
  events = {accepted, completed, restored_completion};
  require(verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "a persisted final point completion did not survive restart");
  events.back().point_index = 1;
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "a new-boot success did not advance to the terminal cursor");
  events = {accepted, completed, restored_completion};
  ++events.back().loop_index;
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "a new-boot success silently advanced the saved loop");
  events = {accepted, event(200, 42, 3), restored_completion};
  require(!verify_inspection_task_timeline("task-a", events, new_zero, new_ack).ok,
          "a new-boot success was accepted without a persisted point completion");
  return 0;
}
