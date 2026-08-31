#include "lingtu/recording/inspection_timeline.hpp"

#include <cmath>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace lingtu::recording {
namespace {

constexpr std::int32_t kTaskAccepted = 1;
constexpr std::int32_t kSucceeded = 7;
constexpr std::int32_t kFailed = 8;
constexpr std::int32_t kCancelled = 9;
constexpr double kZeroVelocityTolerance = 1e-6;

bool terminal_state(std::int32_t state) {
  return state == kSucceeded || state == kFailed || state == kCancelled;
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

  std::unordered_map<std::string, std::uint64_t> last_sequence_by_boot;
  for (const auto &event : events) {
    if (event.boot_id.empty() || event.event_sequence == 0) {
      add_error(report, "inspection event has an invalid boot cursor");
      continue;
    }
    const auto [position, inserted] =
        last_sequence_by_boot.emplace(event.boot_id, event.event_sequence);
    if (!inserted) {
      if (event.event_sequence != position->second + 1) {
        add_error(report, "inspection event sequence is not contiguous for boot " + event.boot_id);
      }
      position->second = event.event_sequence;
    }
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
  std::uint64_t previous_time_ns = 0;
  for (const auto *event : task_events) {
    if (event->request_id.empty()) {
      add_error(report, "inspection task event is missing request_id");
    }
    if (event->boot_id != identity.boot_id) {
      add_error(report, "inspection task crossed a native endpoint boot boundary");
    }
    if (event->map_id != identity.map_id || event->map_content_epoch != identity.map_content_epoch ||
        event->route_id != identity.route_id || event->route_revision != identity.route_revision) {
      add_error(report, "inspection task identity changed during the timeline");
    }
    if (event->observed_time_ns < previous_time_ns) {
      add_error(report, "inspection task event timestamps moved backwards");
    }
    previous_time_ns = event->observed_time_ns;
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
    const DriverControlFact *confirmation = nullptr;
    for (const auto &state : driver_states) {
      if (state.observed_time_ns < identity.observed_time_ns ||
          state.observed_time_ns < last_output->observed_time_ns ||
          state.observed_time_ns > terminal->observed_time_ns || !state.connected ||
          !state.last_command_accepted ||
          state.accepted_producer_boot_id != last_output->producer_boot_id ||
          state.accepted_output_sequence < last_output->output_sequence) {
        continue;
      }
      if (confirmation == nullptr || state.observed_time_ns > confirmation->observed_time_ns) {
        confirmation = &state;
      }
    }
    if (confirmation == nullptr) {
      add_error(report, "driver did not confirm the final zero-output sequence before terminal");
    } else {
      report.confirmed_output_sequence = last_output->output_sequence;
    }
  }

  report.ok = report.errors.empty();
  return report;
}

}  // namespace lingtu::recording
