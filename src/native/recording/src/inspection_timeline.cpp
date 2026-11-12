#include "lingtu/recording/inspection_timeline.hpp"
#include "message/generated/enums.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_set>
#include <utility>

namespace lingtu::recording {
namespace {

using TaskState = lingtu::message::InspectionTaskState;
using EventKind = lingtu::message::InspectionTaskEventKind;
constexpr auto kTaskAccepted = static_cast<std::int32_t>(EventKind::kTaskAccepted);
constexpr auto kStateChanged = static_cast<std::int32_t>(EventKind::kStateChanged);
constexpr auto kMilestone = static_cast<std::int32_t>(EventKind::kMilestone);
constexpr auto kEvidenceRecorded = static_cast<std::int32_t>(EventKind::kEvidenceRecorded);
constexpr auto kSucceeded = static_cast<std::int32_t>(TaskState::kSucceeded);
constexpr auto kFailed = static_cast<std::int32_t>(TaskState::kFailed);
constexpr auto kCancelled = static_cast<std::int32_t>(TaskState::kCancelled);
constexpr auto kPaused = static_cast<std::int32_t>(TaskState::kPaused);
constexpr auto kPlanning = static_cast<std::int32_t>(TaskState::kPlanning);
constexpr auto kWaitingNextLoop = static_cast<std::int32_t>(TaskState::kWaitingNextLoop);
constexpr auto kCancelling = static_cast<std::int32_t>(TaskState::kCancelling);
constexpr double kZeroVelocityTolerance = 1e-6;

bool terminal_state(std::int32_t state) {
  return state == kSucceeded || state == kFailed || state == kCancelled;
}

bool same_identity(const InspectionTaskEventFact &first, const InspectionTaskEventFact &event) {
  return event.task_id == first.task_id && event.map_id == first.map_id &&
      event.map_content_epoch == first.map_content_epoch && event.route_id == first.route_id &&
      event.route_revision == first.route_revision;
}

bool checkpoint_restart(const InspectionTaskEventFact &previous,
                        const InspectionTaskEventFact &event) {
  if (terminal_state(previous.state) || event.kind != kStateChanged || !same_identity(previous, event) ||
      event.event_sequence != previous.event_sequence + 1U) return false;
  if (event.state == kPaused) {
    return event.reason == "restart_resume_required" ||
        event.reason == "restart_loop_wait_resume_required";
  }
  if (event.state == kCancelled) {
    return previous.state == kCancelling && event.reason == "cancelled_after_restart";
  }
  return event.state == kSucceeded && event.reason == "route_complete" &&
      event.point_count == previous.point_count && event.point_index == event.point_count &&
      event.loop_index == previous.loop_index &&
      (previous.kind == kEvidenceRecorded || (previous.kind == kMilestone &&
          (previous.reason == "point_completed" ||
           previous.reason.rfind("point_skipped:", 0U) == 0U)));
}

bool same_terminal_fact(const InspectionTaskEventFact &first,
                        const InspectionTaskEventFact &event) {
  return terminal_state(first.state) && same_identity(first, event) &&
      first.event_sequence == event.event_sequence && first.kind == event.kind &&
      first.state == event.state && first.request_id == event.request_id &&
      first.point_index == event.point_index && first.point_count == event.point_count &&
      first.loop_index == event.loop_index && first.retry_count == event.retry_count &&
      first.action == event.action && first.action_request_id == event.action_request_id &&
      first.evidence_id == event.evidence_id && first.reason == event.reason;
}

bool zero_output(const FinalOutputFact &output) {
  return std::isfinite(output.linear_x) && std::isfinite(output.linear_y) &&
         std::isfinite(output.linear_z) && std::isfinite(output.angular_x) &&
         std::isfinite(output.angular_y) && std::isfinite(output.angular_z) &&
         std::abs(output.linear_x) <= kZeroVelocityTolerance &&
         std::abs(output.linear_y) <= kZeroVelocityTolerance &&
         std::abs(output.linear_z) <= kZeroVelocityTolerance &&
         std::abs(output.angular_x) <= kZeroVelocityTolerance &&
         std::abs(output.angular_y) <= kZeroVelocityTolerance &&
         std::abs(output.angular_z) <= kZeroVelocityTolerance;
}

void add_error(InspectionTimelineReport &report, std::string error) {
  report.errors.push_back(std::move(error));
}

}  // namespace

std::string InspectionTimelineReport::summary() const {
  if (ok) {
    return "inspection task timeline verified";
  }
  std::ostringstream output;
  for (std::size_t index = 0; index < errors.size(); ++index) {
    if (index != 0) {
      output << "; ";
    }
    output << errors[index];
  }
  return output.str();
}

InspectionTimelineReport
verify_inspection_task_timeline(const std::string &task_id,
                                const std::vector<InspectionTaskEventFact> &events,
                                const std::vector<FinalOutputFact> &outputs,
                                const std::vector<DriverControlFact> &driver_states) {
  InspectionTimelineReport report;
  if (task_id.empty()) {
    add_error(report, "inspection task_id is required");
    return report;
  }

  std::vector<const InspectionTaskEventFact *> task_events;
  for (const auto &event : events) {
    if (event.task_id == task_id) {
      task_events.push_back(&event);
    }
  }
  report.event_count = task_events.size();
  if (task_events.empty()) {
    add_error(report, "no inspection task events were captured for task_id " + task_id);
    return report;
  }

  const auto &identity = *task_events.front();
  if (identity.kind != kTaskAccepted) {
    add_error(report, "inspection timeline does not start with TASK_ACCEPTED");
  }
  if (identity.boot_id.empty() || identity.request_id.empty() || identity.map_id.empty() ||
      identity.route_id.empty() || identity.route_revision == 0) {
    add_error(report, "inspection TASK_ACCEPTED is missing immutable task identity");
  }

  const InspectionTaskEventFact *terminal = nullptr;
  const InspectionTaskEventFact *previous = nullptr;
  std::string current_boot;
  std::unordered_set<std::string> observed_boots;
  std::uint64_t last_sequence = 0U;
  bool resume_required = false;
  std::string restart_request_id;
  std::uint64_t previous_time_ns = 0;
  for (const auto &captured : events) {
    const auto *event = &captured;
    if (event->task_id != task_id) {
      // Other tasks on this writer can explain an intermediate global cursor.
      // Their own histories and other endpoint boots do not qualify this task.
      if (previous != nullptr && terminal == nullptr && event->boot_id == current_boot &&
          event->event_sequence != 0U) {
        if (event->event_sequence != last_sequence + 1U) {
          add_error(report, "inspection event sequence is not contiguous for boot " + current_boot);
        }
        last_sequence = event->event_sequence;
      }
      continue;
    }
    if (event->boot_id.empty() || event->event_sequence == 0U) {
      add_error(report, "inspection event has an invalid boot cursor");
    }
    if (event->request_id.empty()) {
      add_error(report, "inspection task event is missing request_id");
    }
    if (!same_identity(identity, *event)) {
      add_error(report, "inspection task identity changed during the timeline");
    }
    if (event->observed_time_ns < previous_time_ns) {
      add_error(report, "inspection task event timestamps moved backwards");
    }
    previous_time_ns = event->observed_time_ns;
    bool restarted = false;
    if (previous != nullptr && event->boot_id != current_boot) {
      if (terminal != nullptr && same_terminal_fact(*terminal, *event)) {
        continue;
      }
      if (!checkpoint_restart(*previous, *event) || observed_boots.count(event->boot_id) != 0U) {
        add_error(report, "inspection task crossed a native boot without checkpoint recovery");
      }
      current_boot = event->boot_id;
      observed_boots.insert(current_boot);
      last_sequence = event->event_sequence;
      restarted = true;
      resume_required = event->state == kPaused;
      restart_request_id = event->request_id;
    } else if (previous != nullptr) {
      if (event->event_sequence != last_sequence + 1U) {
        add_error(report, "inspection event sequence is not contiguous for boot " + current_boot);
      }
      last_sequence = event->event_sequence;
    } else {
      current_boot = event->boot_id;
      observed_boots.insert(current_boot);
      last_sequence = event->event_sequence;
    }
    if (resume_required && !restarted && event->state != kPaused) {
      const bool resumed = event->kind == kStateChanged && event->request_id != restart_request_id &&
          ((event->state == kPlanning && event->reason == "resumed_replan_current_leg") ||
           (event->state == kWaitingNextLoop && event->reason == "waiting_next_loop") ||
           event->state == kCancelling);
      if (!resumed) {
        add_error(report, "restarted inspection advanced without an explicit resume or cancel");
      } else {
        resume_required = false;
      }
    }
    if (terminal != nullptr) {
      add_error(report, "inspection task emitted an event after its terminal fact");
    }
    if (terminal_state(event->state)) {
      if (terminal != nullptr) {
        add_error(report, "inspection task emitted more than one terminal fact");
      } else {
        terminal = event;
        report.terminal_state = event->state;
      }
    }
    previous = event;
  }
  if (terminal == nullptr) {
    add_error(report, "inspection task has no captured terminal fact");
    report.ok = false;
    return report;
  }

  const FinalOutputFact *last_output = nullptr;
  for (const auto &output : outputs) {
    if (output.observed_time_ns >= identity.observed_time_ns &&
        output.observed_time_ns <= terminal->observed_time_ns &&
        (last_output == nullptr || output.observed_time_ns > last_output->observed_time_ns ||
         (output.observed_time_ns == last_output->observed_time_ns &&
          output.output_sequence > last_output->output_sequence))) {
      last_output = &output;
    }
  }
  if (last_output == nullptr) {
    add_error(report, "terminal task has no preceding final cmd_vel evidence");
  } else if (!zero_output(*last_output)) {
    add_error(report, "last final cmd_vel before terminal is not zero");
  } else if (last_output->producer_boot_id.empty() ||
             last_output->producer_boot_id != terminal->boot_id) {
    add_error(report, "final zero-output evidence belongs to a different native boot");
  } else {
    std::uint64_t last_motion_ns = identity.observed_time_ns;
    for (const auto &output : outputs) {
      if (output.observed_time_ns <= terminal->observed_time_ns && !zero_output(output)) {
        last_motion_ns = std::max(last_motion_ns, output.observed_time_ns);
      }
    }
    auto zero_started_ns = last_output->observed_time_ns;
    for (const auto &output : outputs) {
      if (output.observed_time_ns > last_motion_ns && zero_output(output) &&
          output.producer_boot_id == terminal->boot_id) {
        zero_started_ns = std::min(zero_started_ns, output.observed_time_ns);
      }
    }
    const DriverControlFact *confirmation = nullptr;
    for (const auto &state : driver_states) {
      if (state.observed_time_ns <= zero_started_ns ||
          state.observed_time_ns > terminal->observed_time_ns) {
        continue;
      }
      if (confirmation == nullptr || state.observed_time_ns > confirmation->observed_time_ns) {
        confirmation = &state;
      }
    }
    if (confirmation == nullptr || !confirmation->zero_output) {
      add_error(report, "driver zero-output state was not observed before terminal");
    } else {
      report.final_output_sequence = last_output->output_sequence;
    }
  }

  report.ok = report.errors.empty();
  return report;
}

}  // namespace lingtu::recording
