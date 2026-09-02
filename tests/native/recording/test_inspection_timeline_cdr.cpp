#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "dds/dds.h"
#include "dds/ddsi/ddsi_serdata.h"
#include "lingtu/recording/cyclone_cdr.hpp"
#include "lingtu/recording/inspection_timeline_cdr.hpp"
#include "lingtu/recording/topic_catalog.hpp"
#include "messages.h"

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
  event->map_content_epoch = 4;
  event->route_id = text("route-a");
  event->route_revision = 9;
  event->point_id = text("");
  event->action = text("");
  event->action_request_id = text("");
  event->evidence_id = text("");
  event->reason = text("");
  return event;
}

struct FixtureWriter {
  dds_entity_t topic{DDS_RETCODE_ERROR};
  dds_entity_t writer{DDS_RETCODE_ERROR};
};

FixtureWriter create_writer(dds_entity_t participant,
                            const lingtu::recording::TopicBinding &binding) {
  FixtureWriter result;
  const std::string wire_topic(binding.contract->dds_topic);
  result.topic =
      dds_create_topic(participant, binding.descriptor, wire_topic.c_str(), nullptr, nullptr);
  require(result.topic > 0, "failed to create drain fixture topic");
  auto qos = lingtu::dds::make_qos(binding.qos_profile);
  result.writer = dds_create_writer(participant, result.topic, qos.get(), nullptr);
  require(result.writer > 0, "failed to create drain fixture writer");
  return result;
}

void wait_for_reader(dds_entity_t writer, std::int32_t expected_count = 1) {
  for (int attempt = 0; attempt < 500; ++attempt) {
    dds_publication_matched_status_t status{};
    require(dds_get_publication_matched_status(writer, &status) == DDS_RETCODE_OK,
            "failed to read drain fixture publication status");
    if (status.current_count >= expected_count) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("drain fixture did not match recorder reader");
}

void wait_for_samples(dds_entity_t reader, dds_return_t expected_count) {
  for (int attempt = 0; attempt < 500; ++attempt) {
    std::array<ddsi_serdata *, 64> samples{};
    std::array<dds_sample_info_t, 64> infos{};
    const dds_return_t count =
        dds_readcdr(reader, samples.data(), samples.size(), infos.data(), DDS_ANY_STATE);
    require(count >= 0, "failed to inspect drain fixture backlog");
    for (dds_return_t index = 0; index < count; ++index) {
      if (samples[static_cast<std::size_t>(index)] != nullptr) {
        ddsi_serdata_unref(samples[static_cast<std::size_t>(index)]);
      }
    }
    if (count >= expected_count) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  throw std::runtime_error("drain fixture backlog did not become readable");
}

void write_checked(dds_entity_t writer, const void *sample, dds_time_t source_timestamp) {
  require(dds_write_ts(writer, sample, source_timestamp) == DDS_RETCODE_OK,
          "failed to publish drain fixture sample");
  require(dds_wait_for_acks(writer, DDS_SECS(1)) == DDS_RETCODE_OK,
          "drain fixture sample was not acknowledged");
}

int publish_drain_fixture(int domain_id, const std::filesystem::path &ready_path,
                          const std::filesystem::path &go_path) {
  const auto *event_binding = lingtu::recording::find_recording_topic("/nav/inspection/task/event");
  const auto *output_binding = lingtu::recording::find_recording_topic("/nav/cmd_vel");
  const auto *driver_binding = lingtu::recording::find_recording_topic("/driver/control_state");
  require(event_binding != nullptr && output_binding != nullptr && driver_binding != nullptr,
          "inspection evidence bindings are incomplete");

  const dds_entity_t participant = dds_create_participant(domain_id, nullptr, nullptr);
  require(participant > 0, "failed to create drain fixture participant");
  try {
    const auto event_writer = create_writer(participant, *event_binding);
    const auto output_writer = create_writer(participant, *output_binding);
    const auto driver_writer = create_writer(participant, *driver_binding);
    wait_for_reader(event_writer.writer);
    wait_for_reader(output_writer.writer);
    wait_for_reader(driver_writer.writer);

    auto *accepted = inspection_event(41, 1, 1);
    write_checked(event_writer.writer, accepted, 100);
    lingtu_dds_InspectionTaskEvent_free(accepted,
                                        static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));

    auto *output = lingtu_dds_FinalVelocityCommand__alloc();
    *output = {};
    output->host_boot_id = text("host-boot-1");
    output->producer_boot_id = text("nav-boot-1");
    output->output_seq = 71;
    write_checked(output_writer.writer, output, 300);
    lingtu_dds_FinalVelocityCommand_free(output,
                                         static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));

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
    write_checked(driver_writer.writer, driver, 350);
    lingtu_dds_DriverControlState_free(driver,
                                       static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));

    std::ofstream(ready_path).put('\n');
    for (int attempt = 0; attempt < 500 && !std::filesystem::exists(go_path); ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    require(std::filesystem::exists(go_path), "drain fixture timed out waiting for release");

    auto *cancelled = inspection_event(42, 2, 9);
    write_checked(event_writer.writer, cancelled, 400);
    lingtu_dds_InspectionTaskEvent_free(cancelled,
                                        static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  } catch (...) {
    dds_delete(participant);
    throw;
  }
  dds_delete(participant);
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc == 5 && std::string(argv[1]) == "--publish-drain-fixture") {
    return publish_drain_fixture(std::stoi(argv[2]), argv[3], argv[4]);
  }
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

  const std::string event_wire_topic(event_binding->contract->dds_topic);
  const dds_entity_t drain_topic = dds_create_topic(
      participant, event_binding->descriptor, event_wire_topic.c_str(), nullptr, nullptr);
  require(drain_topic > 0, "failed to create bounded drain topic");
  auto drain_qos = lingtu::dds::make_qos(event_binding->qos_profile);
  const dds_entity_t drain_writer =
      dds_create_writer(participant, drain_topic, drain_qos.get(), nullptr);
  const dds_entity_t drain_reader =
      dds_create_reader(participant, drain_topic, drain_qos.get(), nullptr);
  const dds_entity_t second_drain_reader =
      dds_create_reader(participant, drain_topic, drain_qos.get(), nullptr);
  require(drain_writer > 0 && drain_reader > 0 && second_drain_reader > 0,
          "failed to create bounded drain endpoints");
  wait_for_reader(drain_writer, 2);

  lingtu::recording::InspectionTimelineCapture delayed_capture;
  delayed_capture.observe(*event_binding, accepted_message);
  delayed_capture.observe(*output_binding, output_message);
  delayed_capture.observe(*driver_binding, driver_message);
  auto *terminal = inspection_event(42, 2, 9);
  write_checked(drain_writer, terminal, 400);
  lingtu_dds_InspectionTaskEvent_free(terminal,
                                      static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  auto *extra = inspection_event(43, 2, 3);
  write_checked(drain_writer, extra, 401);
  lingtu_dds_InspectionTaskEvent_free(extra,
                                      static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  wait_for_samples(drain_reader, 2);
  std::uint32_t drain_sequence = 0;
  std::uint32_t second_drain_sequence = 0;
  const std::vector<lingtu::recording::CdrDrainReader> drain_readers{
      {drain_reader, event_wire_topic, &drain_sequence},
      {second_drain_reader, event_wire_topic, &second_drain_sequence}};
  std::size_t second_reader_consumed = 0;
  const bool completed = lingtu::recording::bounded_drain_cdr_messages(
      drain_readers, std::chrono::milliseconds(250),
      [&] { return delayed_capture.verify("inspection-task-1").ok; },
      [&](std::size_t index, lingtu::recording::RecordedMessage &&message) {
        if (index != 0) {
          ++second_reader_consumed;
          return;
        }
        delayed_capture.observe(*event_binding, message);
      });
  require(completed, "bounded drain did not capture a delayed terminal task event");
  require(delayed_capture.verify("inspection-task-1").ok,
          "bounded drain returned without a complete task timeline");
  require(second_reader_consumed == 0,
          "bounded drain consumed a remaining reader after timeline completion");
  require(drain_sequence == 1,
          "bounded drain advanced sequence for an unconsumed same-reader fact");
  auto remaining_after_completion = lingtu::recording::take_cdr_messages(
      drain_reader, event_wire_topic, drain_sequence, 1);
  require(remaining_after_completion.size() == 1,
          "bounded drain removed a same-reader fact after timeline completion");
  require(remaining_after_completion.front().sequence == 1 && drain_sequence == 2,
          "same-reader fact did not retain the next recording sequence");

  constexpr std::size_t kBacklogSamples = 40;
  for (std::size_t index = 0; index < kBacklogSamples; ++index) {
    auto *backlog = inspection_event(100 + index, 2, 3);
    write_checked(drain_writer, backlog, 500 + static_cast<dds_time_t>(index));
    lingtu_dds_InspectionTaskEvent_free(backlog,
                                        static_cast<dds_free_op_t>(DDS_FREE_ALL_BIT));
  }
  wait_for_samples(second_drain_reader, static_cast<dds_return_t>(kBacklogSamples + 2));
  const std::vector<lingtu::recording::CdrDrainReader> timeout_readers{
      {second_drain_reader, event_wire_topic, &second_drain_sequence}};
  const auto timeout_started = std::chrono::steady_clock::now();
  std::size_t timeout_consumed = 0;
  const bool impossible_completion = lingtu::recording::bounded_drain_cdr_messages(
      timeout_readers, std::chrono::milliseconds(15), [] { return false; },
      [&](std::size_t, lingtu::recording::RecordedMessage &&) {
        ++timeout_consumed;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      });
  const auto timeout_elapsed = std::chrono::steady_clock::now() - timeout_started;
  require(!impossible_completion, "bounded drain fabricated task completion");
  require(timeout_consumed < kBacklogSamples,
          "bounded drain consumed an entire backlog after its deadline");
  require(second_drain_sequence == timeout_consumed,
          "bounded drain advanced sequence beyond its consumed timeout backlog");
  auto remaining_after_timeout = lingtu::recording::take_cdr_messages(
      second_drain_reader, event_wire_topic, second_drain_sequence, 1);
  require(remaining_after_timeout.size() == 1,
          "bounded drain removed a same-reader fact after timeout");
  require(remaining_after_timeout.front().sequence == timeout_consumed &&
              second_drain_sequence == timeout_consumed + 1,
          "post-timeout fact did not retain the next recording sequence");
  require(timeout_elapsed < std::chrono::milliseconds(100),
          "bounded drain exceeded its shutdown deadline");

  std::uint32_t completion_race_sequence = 0;
  bool completion_race = false;
  std::size_t completion_race_consumed = 0;
  const std::vector<lingtu::recording::CdrDrainReader> completion_race_readers{
      {1, "test/complete-during-take", &completion_race_sequence}};
  const bool completion_race_result = lingtu::recording::bounded_drain_cdr_messages(
      completion_race_readers, std::chrono::milliseconds(100),
      [&] { return completion_race; },
      [&](std::size_t, lingtu::recording::RecordedMessage &&message) {
        ++completion_race_consumed;
        require(message.sequence == 0, "completion-race message sequence changed");
      },
      [&](dds_entity_t, std::string_view, std::uint32_t &next_sequence) {
        completion_race = true;
        lingtu::recording::RecordedMessage message;
        message.sequence = next_sequence++;
        return std::vector<lingtu::recording::RecordedMessage>{std::move(message)};
      });
  require(completion_race_result, "external completion during take was not reported");
  require(completion_race_consumed == 1 && completion_race_sequence == 1,
          "message taken during external completion was not consumed exactly once");

  std::uint32_t deadline_race_sequence = 0;
  std::size_t deadline_race_consumed = 0;
  const std::vector<lingtu::recording::CdrDrainReader> deadline_race_readers{
      {1, "test/deadline-during-take", &deadline_race_sequence}};
  const bool deadline_race_result = lingtu::recording::bounded_drain_cdr_messages(
      deadline_race_readers, std::chrono::milliseconds(1), [] { return false; },
      [&](std::size_t, lingtu::recording::RecordedMessage &&message) {
        ++deadline_race_consumed;
        require(message.sequence == 0, "deadline-race message sequence changed");
      },
      [&](dds_entity_t, std::string_view, std::uint32_t &next_sequence) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        lingtu::recording::RecordedMessage message;
        message.sequence = next_sequence++;
        return std::vector<lingtu::recording::RecordedMessage>{std::move(message)};
      });
  require(!deadline_race_result, "deadline crossing during take fabricated completion");
  require(deadline_race_consumed == 1 && deadline_race_sequence == 1,
          "message taken while crossing deadline was not consumed exactly once");

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
