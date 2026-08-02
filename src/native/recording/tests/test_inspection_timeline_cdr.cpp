#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "dds/dds.h"
#include "lingtu/recording/cyclone_cdr.hpp"
#include "lingtu/recording/inspection_timeline_cdr.hpp"
#include "lingtu/recording/topic_catalog.hpp"
#include "lingtu_slam.h"

namespace {

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

char *text(const char *value) {
  return dds_string_dup(value);
}

void set_header(lingtu_dds_Header &header) {
  header.stamp.sec = 0;
  header.stamp.nanosec = 0;
  header.frame_id = text("map");
}

lingtu::recording::RecordedMessage capture_sample(dds_entity_t participant,
                                                  const lingtu::recording::TopicBinding &binding,
                                                  const void *sample, dds_time_t source_timestamp,
                                                  std::uint32_t &sequence) {
  const std::string wire_topic(binding.contract->dds_topic);
  const dds_entity_t topic =
      dds_create_topic(participant, binding.descriptor, wire_topic.c_str(), nullptr, nullptr);
  require(topic > 0, "failed to create timeline DDS topic");
  const dds_entity_t writer = dds_create_writer(participant, topic, nullptr, nullptr);
  const dds_entity_t reader = dds_create_reader(participant, topic, nullptr, nullptr);
  require(writer > 0 && reader > 0, "failed to create timeline DDS endpoints");
  require(dds_write_ts(writer, sample, source_timestamp) == DDS_RETCODE_OK,
          "failed to publish timeline DDS sample");

  std::vector<lingtu::recording::RecordedMessage> captured;
  for (int attempt = 0; attempt < 100 && captured.empty(); ++attempt) {
    captured = lingtu::recording::take_cdr_messages(reader, wire_topic, sequence, 1);
    if (captured.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  require(captured.size() == 1, "did not capture one timeline DDS sample");
  return std::move(captured.front());
}

lingtu_dds_InspectionTaskEvent *inspection_event(std::uint64_t event_sequence, std::int32_t kind,
                                                 std::int32_t state) {
  auto *event = lingtu_dds_InspectionTaskEvent__alloc();
  *event = {};
  set_header(event->header);
  event->boot_id = text("nav-boot-1");
  event->event_sequence = event_sequence;
  event->kind = kind;
  event->task_id = text("inspection-task-1");
  event->request_id = text("request-1");
  event->command_request_id = text("command-1");
  event->state = state;
  event->map_id = text("map-a");
  event->map_version = 4;
  event->route_id = text("route-a");
  event->route_revision = 9;
  event->point_id = text("");
  event->action = text("");
  event->action_request_id = text("");
  event->evidence_id = text("");
  event->reason = text("");
  return event;
}

}  // namespace

int main() {
  const auto *event_binding = lingtu::recording::find_recording_topic("/nav/inspection/task/event");
  const auto *output_binding = lingtu::recording::find_recording_topic("/nav/cmd_vel");
  const auto *driver_binding = lingtu::recording::find_recording_topic("/driver/control_state");
  require(event_binding != nullptr && output_binding != nullptr && driver_binding != nullptr,
          "inspection evidence bindings are incomplete");

  const dds_entity_t participant = dds_create_participant(202, nullptr, nullptr);
  require(participant > 0, "failed to create timeline DDS participant");
  std::uint32_t sequence = 0;
  lingtu::recording::InspectionTimelineCapture capture;

  auto *accepted = inspection_event(41, 1, 1);
  auto accepted_message = capture_sample(participant, *event_binding, accepted, 100, sequence);
  lingtu_dds_InspectionTaskEvent_free(accepted, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  capture.observe(*event_binding, accepted_message);

  auto *output = lingtu_dds_FinalVelocityCommand__alloc();
  *output = {};
  output->host_boot_id = text("host-boot-1");
  output->producer_boot_id = text("nav-boot-1");
  output->output_seq = 71;
  auto output_message = capture_sample(participant, *output_binding, output, 300, sequence);
  lingtu_dds_FinalVelocityCommand_free(output, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  capture.observe(*output_binding, output_message);

  auto *driver = lingtu_dds_DriverControlState__alloc();
  *driver = {};
  set_header(driver->header);
  driver->connected = true;
  driver->ready = true;
  driver->lease_valid = true;
  driver->accepted_producer_boot_id = text("nav-boot-1");
  driver->accepted_output_sequence = 71;
  driver->last_command_accepted = true;
  driver->fsm = text("ready");
  driver->owner = text("nav");
  driver->owner_id = text("nav-boot-1");
  driver->reason = text("");
  auto driver_message = capture_sample(participant, *driver_binding, driver, 350, sequence);
  lingtu_dds_DriverControlState_free(driver, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  capture.observe(*driver_binding, driver_message);

  auto *cancelled = inspection_event(42, 2, 9);
  auto cancelled_message = capture_sample(participant, *event_binding, cancelled, 400, sequence);
  lingtu_dds_InspectionTaskEvent_free(cancelled, static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  capture.observe(*event_binding, cancelled_message);

  const auto report = capture.verify("inspection-task-1");
  require(report.ok, "portable CDR timeline rejected valid task evidence");
  require(report.event_count == 2, "portable CDR timeline lost task events");
  require(report.terminal_state == 9, "portable CDR timeline lost terminal state");
  require(report.confirmed_output_sequence == 71,
          "portable CDR timeline lost driver stop confirmation");

  bool truncated_rejected = false;
  require(cancelled_message.payload.size() > 4, "captured event payload is too small");
  cancelled_message.payload.resize(cancelled_message.payload.size() - 4);
  try {
    lingtu::recording::InspectionTimelineCapture malformed;
    malformed.observe(*event_binding, cancelled_message);
  } catch (const std::runtime_error &) {
    truncated_rejected = true;
  }
  require(truncated_rejected, "portable CDR timeline accepted a truncated payload");

  dds_delete(participant);
  return 0;
}
