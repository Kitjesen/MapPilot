#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lingtu::recording {

struct InspectionTaskEventFact {
  std::uint64_t observed_time_ns{0};
  std::string boot_id;
  std::uint64_t event_sequence{0};
  std::int32_t kind{0};
  std::string task_id;
  std::string request_id;
  std::string map_id;
  std::int64_t map_content_epoch{0};
  std::string route_id;
  std::uint64_t route_revision{0};
  std::int32_t state{0};
};

struct FinalOutputFact {
  std::uint64_t observed_time_ns{0};
  std::uint64_t output_sequence{0};
  double linear_x{0.0};
  double linear_y{0.0};
  double linear_z{0.0};
  double angular_x{0.0};
  double angular_y{0.0};
  double angular_z{0.0};
  std::string producer_boot_id;
};

struct DriverControlFact {
  std::uint64_t observed_time_ns{0};
  bool connected{false};
  bool last_command_accepted{false};
  std::uint64_t accepted_output_sequence{0};
  std::string accepted_producer_boot_id;
};

struct InspectionTimelineReport {
  bool ok{false};
  std::size_t event_count{0};
  std::int32_t terminal_state{-1};
  std::uint64_t confirmed_output_sequence{0};
  std::vector<std::string> errors;

  [[nodiscard]] std::string summary() const;
};

InspectionTimelineReport
verify_inspection_task_timeline(const std::string &task_id,
                                const std::vector<InspectionTaskEventFact> &events,
                                const std::vector<FinalOutputFact> &outputs,
                                const std::vector<DriverControlFact> &driver_states);

}  // namespace lingtu::recording
