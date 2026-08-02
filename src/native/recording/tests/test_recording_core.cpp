#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "lingtu/recording/inspection_timeline.hpp"
#include "lingtu/recording/mcap_session.hpp"
#include "lingtu/recording/recording_core.hpp"
#include "mcap/reader.hpp"

namespace {

using lingtu::recording::BoundedMessageQueue;
using lingtu::recording::ChannelDefinition;
using lingtu::recording::DriverControlFact;
using lingtu::recording::FinalOutputFact;
using lingtu::recording::InspectionTaskEventFact;
using lingtu::recording::McapSessionWriter;
using lingtu::recording::RecordedMessage;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class ScopedEnvironment {
 public:
  explicit ScopedEnvironment(const char *name) : name_(name) {
    if (const char *value = std::getenv(name); value != nullptr) {
      original_ = value;
    }
  }

  ~ScopedEnvironment() {
    assign(original_);
  }

  void clear() {
    assign(std::nullopt);
  }

  void set(const std::filesystem::path &value) {
    assign(value.string());
  }

 private:
  void assign(const std::optional<std::string> &value) const {
#ifdef _WIN32
    _putenv_s(name_.c_str(), value ? value->c_str() : "");
#else
    if (value) {
      setenv(name_.c_str(), value->c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
#endif
  }

  std::string name_;
  std::optional<std::string> original_;
};

RecordedMessage message(std::size_t size) {
  RecordedMessage result;
  result.wire_topic = "rt/imu/raw";
  result.payload.resize(size, std::byte{0x2a});
  return result;
}

void test_bounded_queue_counts_bytes_and_drops() {
  BoundedMessageQueue queue(5);
  require(queue.try_push(message(4)), "queue rejected a message below capacity");
  require(!queue.try_push(message(2)), "queue accepted a message over capacity");
  require(queue.dropped_messages() == 1, "queue did not count an overflow drop");
  require(queue.high_watermark_bytes() == 4, "queue high watermark is incorrect");

  RecordedMessage taken;
  require(queue.pop(taken), "queue did not return its first message");
  require(taken.payload.size() == 4, "queue returned the wrong payload");
  require(queue.try_push(message(5)), "queue rejected an exact-capacity message");
  queue.close();
  require(!queue.try_push(message(1)), "closed queue accepted a message");
  require(queue.pop(taken), "closed queue did not drain its final message");
  require(!queue.pop(taken), "closed and drained queue returned a message");
  require(queue.dropped_messages() == 2, "closed queue drop was not counted");
}

void test_replay_safety_and_timing() {
  require(lingtu::recording::is_sensor_replay_topic("/imu/raw"),
          "IMU topic is missing from the replay allowlist");
  require(lingtu::recording::is_sensor_replay_topic("rt/slam/registered_cloud"),
          "wire point-cloud topic is missing from the replay allowlist");
  require(!lingtu::recording::is_sensor_replay_topic("/nav/cmd_vel"),
          "motion topic entered the replay allowlist");
  require(!lingtu::recording::is_sensor_replay_topic("/nav/goal_pose"),
          "goal topic entered the replay allowlist");
  require(lingtu::recording::validate_replay_domain(84, false).empty(),
          "isolated replay domain was rejected");
  require(!lingtu::recording::validate_replay_domain(0, false).empty(),
          "live replay domain was accepted without opt-in");
  require(lingtu::recording::validate_replay_domain(0, true).empty(),
          "explicit live replay domain opt-in was rejected");
  require(lingtu::recording::replay_offset_ns(100, 350, 2.0) == 125,
          "replay rate scaling is incorrect");
}

bool contains_topic(const std::vector<std::string> &topics, std::string_view expected) {
  for (const auto &topic : topics) {
    if (topic == expected) {
      return true;
    }
  }
  return false;
}

void test_generic_sensor_plan_includes_raw_lidar_without_duplicate_map_layers() {
  const auto plan = lingtu::recording::dds_recording_plan("generic-sensors-v1");
  require(contains_topic(plan.selected_topics, "/lidar/raw_frame"),
          "generic recording omitted the raw lidar stream");
  require(contains_topic(plan.required_topics, "/imu/raw"),
          "generic recording did not require the raw IMU stream");
  require(contains_topic(plan.required_topics, "/lidar/raw_frame"),
          "generic recording did not require the raw lidar stream");
  require(!contains_topic(plan.selected_topics, "/slam/map_observation"),
          "generic recording defaulted both raw lidar and MapObservation");
}

void test_generic_sensor_plan_preserves_explicit_required_sensor_selection() {
  const auto plan = lingtu::recording::dds_recording_plan(
      "generic-sensors-v1", std::vector<std::string>{"/imu/raw", "/lidar/raw_frame"});

  require(plan.selected_topics.size() == 2,
          "explicit selection cannot be completed when requested topics are missing");
  require(contains_topic(plan.selected_topics, "/imu/raw"),
          "explicit selection cannot be completed when /imu/raw is missing");
  require(contains_topic(plan.selected_topics, "/lidar/raw_frame"),
          "explicit selection cannot be completed when /lidar/raw_frame is missing");
  require(contains_topic(plan.required_topics, "/imu/raw"),
          "explicit selection cannot be completed when required /imu/raw is missing");
  require(contains_topic(plan.required_topics, "/lidar/raw_frame"),
          "explicit selection cannot be completed when required /lidar/raw_frame is missing");
}

void test_recording_idl_resolution_precedence() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto root = std::filesystem::temp_directory_path() /
                    ("lingtu-recording-idl-resolution-" + std::to_string(nonce));
  const auto executable = root / "build" / "native-recording" / "lingtu_dds_player";
  const auto executable_idl = root / "src" / "message" / "idl" / "lingtu_slam.idl";
  const auto repository = root / "repository";
  const auto repository_idl = repository / "src" / "message" / "idl" / "lingtu_slam.idl";
  const auto environment_idl = root / "environment" / "lingtu_slam.idl";
  const auto fallback_idl = root / "fallback" / "lingtu_slam.idl";
  for (const auto &path : {executable_idl, repository_idl, environment_idl, fallback_idl}) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path) << "module lingtu {};";
  }

  ScopedEnvironment idl_environment("LINGTU_RECORDING_IDL");
  ScopedEnvironment repository_environment("LINGTU_REPO");
  idl_environment.clear();
  repository_environment.clear();

  require(lingtu::recording::resolve_recording_idl(executable, fallback_idl) ==
              std::filesystem::weakly_canonical(executable_idl),
          "executable-relative recording IDL was not selected");

  repository_environment.set(repository);
  require(lingtu::recording::resolve_recording_idl(executable, fallback_idl) ==
              std::filesystem::weakly_canonical(repository_idl),
          "LINGTU_REPO did not override the executable-relative recording IDL");

  idl_environment.set(environment_idl);
  require(lingtu::recording::resolve_recording_idl(executable, fallback_idl) ==
              std::filesystem::weakly_canonical(environment_idl),
          "LINGTU_RECORDING_IDL did not take precedence");

  idl_environment.clear();
  repository_environment.clear();
  std::filesystem::remove(executable_idl);
  require(lingtu::recording::resolve_recording_idl(executable, fallback_idl) ==
              std::filesystem::weakly_canonical(fallback_idl),
          "compile-time recording IDL fallback was not selected");

  std::filesystem::remove_all(root);
}

void test_inspection_recording_plan_captures_a_safe_product_timeline() {
  const auto plan = lingtu::recording::dds_recording_plan("inspection-evidence-v1");

  require(contains_topic(plan.selected_topics, "/nav/inspection/task/event"),
          "inspection recording omitted the authoritative task event stream");
  require(contains_topic(plan.selected_topics, "/nav/global_path") &&
              contains_topic(plan.selected_topics, "/nav/local_path"),
          "inspection recording omitted path evidence");
  require(contains_topic(plan.selected_topics, "/nav/cmd_vel"),
          "inspection recording omitted logical motion evidence");
  require(contains_topic(plan.selected_topics, "/driver/control_state"),
          "inspection recording omitted driver acceptance evidence");

  require(contains_topic(plan.required_topics, "/nav/inspection/task/event"),
          "inspection completion does not require a native task fact");
  require(contains_topic(plan.required_topics, "/nav/state"),
          "inspection completion does not require native navigation state");
  require(contains_topic(plan.required_topics, "/driver/control_state"),
          "inspection completion does not require driver state");

  require(!lingtu::recording::is_sensor_replay_topic("/nav/inspection/task/event"),
          "task events entered the sensor replay allowlist");
  require(!lingtu::recording::is_sensor_replay_topic("/nav/global_path"),
          "global paths entered the sensor replay allowlist");
  require(!lingtu::recording::is_sensor_replay_topic("/nav/cmd_vel"),
          "recorded logical motion entered the sensor replay allowlist");
}

InspectionTaskEventFact inspection_event(std::uint64_t time_ns, std::uint64_t sequence,
                                         std::int32_t kind, std::int32_t state) {
  return {
      time_ns,
      "nav-boot-1",
      sequence,
      kind,
      "inspection-task-1",
      "request-" + std::to_string(sequence),
      "map-a",
      4,
      "route-a",
      9,
      state,
  };
}

void test_inspection_timeline_requires_terminal_stop_evidence() {
  const std::vector<InspectionTaskEventFact> events{
      inspection_event(100, 41, 1, 1),
      inspection_event(200, 42, 2, 3),
      inspection_event(500, 43, 2, 9),
  };
  const std::vector<FinalOutputFact> outputs{
      {300, 70, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"},
      {400, 71, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"},
  };
  const std::vector<DriverControlFact> driver_states{{450, true, true, 71, "nav-boot-1"}};

  const auto report = lingtu::recording::verify_inspection_task_timeline(
      "inspection-task-1", events, outputs, driver_states);
  require(report.ok, "valid inspection task evidence was rejected");
  require(report.terminal_state == 9, "inspection terminal state was not reported");
  require(report.confirmed_output_sequence == 71,
          "confirmed zero-output sequence was not reported");

  const auto missing_driver =
      lingtu::recording::verify_inspection_task_timeline("inspection-task-1", events, outputs, {});
  require(!missing_driver.ok, "terminal task passed without driver stop confirmation");
  require(missing_driver.summary().find("driver") != std::string::npos,
          "missing driver evidence did not produce an actionable error");

  const auto stale_driver = lingtu::recording::verify_inspection_task_timeline(
      "inspection-task-1", events, outputs, {{450, true, true, 71, "old-nav-boot"}});
  require(!stale_driver.ok, "terminal task accepted driver evidence from an old native boot");
}

void test_inspection_timeline_rejects_stop_evidence_before_task_acceptance() {
  const std::vector<InspectionTaskEventFact> events{
      inspection_event(200, 51, 1, 1),
      inspection_event(500, 52, 2, 9),
  };
  const std::vector<FinalOutputFact> stale_outputs{
      {100, 71, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"},
  };
  const std::vector<DriverControlFact> stale_driver_states{
      {150, true, true, 71, "nav-boot-1"},
  };

  const auto report = lingtu::recording::verify_inspection_task_timeline(
      "inspection-task-1", events, stale_outputs, stale_driver_states);
  require(!report.ok, "task accepted stop evidence captured before TASK_ACCEPTED");
  require(report.summary().find("final cmd_vel") != std::string::npos,
          "stale stop evidence did not produce an actionable error");
}

void test_inspection_timeline_rejects_gaps_identity_drift_and_false_zero() {
  const std::vector<InspectionTaskEventFact> gap_events{
      inspection_event(100, 41, 1, 1),
      inspection_event(200, 43, 2, 3),
      inspection_event(500, 44, 2, 7),
  };
  const std::vector<FinalOutputFact> nonzero_last_output{
      {300, 70, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"},
      {400, 71, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"},
  };
  const std::vector<DriverControlFact> driver_states{{450, true, true, 71, "nav-boot-1"}};

  auto report = lingtu::recording::verify_inspection_task_timeline(
      "inspection-task-1", gap_events, nonzero_last_output, driver_states);
  require(!report.ok, "task timeline passed with an event gap and nonzero final output");

  auto drift_events = std::vector<InspectionTaskEventFact>{
      inspection_event(100, 51, 1, 1),
      inspection_event(200, 52, 2, 3),
      inspection_event(500, 53, 2, 8),
  };
  drift_events[1].map_id = "map-b";
  report = lingtu::recording::verify_inspection_task_timeline(
      "inspection-task-1", drift_events, {{400, 80, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"}},
      {{450, true, true, 80, "nav-boot-1"}});
  require(!report.ok, "task timeline passed after immutable map identity changed");
}

void test_inspection_timeline_requires_acceptance_and_one_terminal() {
  auto events = std::vector<InspectionTaskEventFact>{
      inspection_event(200, 60, 2, 3),
      inspection_event(500, 61, 2, 7),
  };
  auto report = lingtu::recording::verify_inspection_task_timeline(
      "inspection-task-1", events, {{400, 90, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"}},
      {{450, true, true, 90, "nav-boot-1"}});
  require(!report.ok, "task timeline passed without a captured acceptance event");

  events = {
      inspection_event(100, 70, 1, 1),
      inspection_event(400, 71, 2, 7),
      inspection_event(500, 72, 2, 8),
  };
  report = lingtu::recording::verify_inspection_task_timeline(
      "inspection-task-1", events, {{300, 100, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "nav-boot-1"}},
      {{350, true, true, 100, "nav-boot-1"}});
  require(!report.ok, "task timeline passed with multiple terminal facts");
}

void test_mcap_round_trip_and_atomic_commit() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto path = std::filesystem::temp_directory_path() /
                    ("lingtu-recording-" + std::to_string(nonce) + ".mcap");
  const std::string idl = "module lingtu { module dds { struct Imu {}; }; };";
  const ChannelDefinition channel{"/imu/raw", "rt/imu/raw", "lingtu.dds.Imu", "sensor_stream"};

  {
    McapSessionWriter writer(path, idl, {channel}, 1024);
    auto recorded = message(4);
    recorded.log_time_ns = 1000;
    recorded.publish_time_ns = 900;
    recorded.sequence = 7;
    writer.write(recorded);
    writer.commit();
  }

  require(std::filesystem::exists(path), "committed MCAP file does not exist");
  require(!std::filesystem::exists(path.string() + ".tmp"),
          "committed MCAP temporary file still exists");

  mcap::McapReader reader;
  require(reader.open(path.string()).ok(), "failed to open committed MCAP file");
  require(reader.header().has_value(), "MCAP header is missing");
  require(reader.header()->profile == lingtu::recording::kMcapProfile, "MCAP profile is incorrect");
  std::size_t count = 0;
  for (const auto &view : reader.readMessages()) {
    ++count;
    require(view.channel->topic == "rt/imu/raw", "MCAP topic is incorrect");
    require(view.channel->messageEncoding == "cdr", "MCAP encoding is incorrect");
    require(view.channel->metadata.at("lingtu.canonical_topic") == "/imu/raw",
            "MCAP canonical topic metadata is incorrect");
    require(view.channel->metadata.at("lingtu.idl_type") == "lingtu.dds.Imu",
            "MCAP type metadata is incorrect");
    require(view.schema->name == "lingtu.dds.Imu", "MCAP schema name is incorrect");
    require(view.schema->encoding == "omgidl", "MCAP schema encoding is incorrect");
    require(view.message.logTime == 1000, "MCAP log time is incorrect");
    require(view.message.publishTime == 900, "MCAP publish time is incorrect");
    require(view.message.sequence == 7, "MCAP sequence is incorrect");
    require(view.message.dataSize == 4, "MCAP payload size is incorrect");
  }
  require(count == 1, "MCAP message count is incorrect");
  reader.close();
  std::filesystem::remove(path);
}

void test_mcap_rejects_conflicts_before_open_and_retains_uncommitted_temp() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto base = std::filesystem::temp_directory_path() /
                    ("lingtu-recording-failure-" + std::to_string(nonce));
  const std::string idl = "module lingtu { module dds { struct Imu {}; }; };";
  const ChannelDefinition channel{"/imu/raw", "rt/imu/raw", "lingtu.dds.Imu", "sensor_stream"};

  const auto duplicate_path = base.string() + "-duplicate.mcap";
  bool duplicate_rejected = false;
  try {
    McapSessionWriter writer(duplicate_path, idl, {channel, channel}, 1024);
  } catch (const std::invalid_argument &) {
    duplicate_rejected = true;
  }
  require(duplicate_rejected, "duplicate MCAP channel was accepted");
  require(!std::filesystem::exists(duplicate_path + ".tmp"),
          "duplicate channel created an MCAP temporary file");

  const auto existing_path = base.string() + "-existing.mcap";
  {
    std::ofstream marker(existing_path);
    marker << "occupied";
  }
  bool existing_rejected = false;
  try {
    McapSessionWriter writer(existing_path, idl, {channel}, 1024);
  } catch (const std::runtime_error &) {
    existing_rejected = true;
  }
  require(existing_rejected, "existing MCAP output was overwritten");
  require(!std::filesystem::exists(existing_path + ".tmp"),
          "existing output created an MCAP temporary file");
  std::filesystem::remove(existing_path);

  const auto uncommitted_path = base.string() + "-uncommitted.mcap";
  {
    McapSessionWriter writer(uncommitted_path, idl, {channel}, 1024);
    auto recorded = message(4);
    writer.write(recorded);
  }
  require(!std::filesystem::exists(uncommitted_path),
          "uncommitted MCAP session created a final file");
  require(std::filesystem::exists(uncommitted_path + ".tmp"),
          "uncommitted MCAP session did not preserve its temporary file");
  std::filesystem::remove(uncommitted_path + ".tmp");
}

}  // namespace

int main() {
  try {
    test_bounded_queue_counts_bytes_and_drops();
    test_replay_safety_and_timing();
    test_generic_sensor_plan_includes_raw_lidar_without_duplicate_map_layers();
    test_generic_sensor_plan_preserves_explicit_required_sensor_selection();
    test_recording_idl_resolution_precedence();
    test_inspection_recording_plan_captures_a_safe_product_timeline();
    test_inspection_timeline_requires_terminal_stop_evidence();
    test_inspection_timeline_rejects_stop_evidence_before_task_acceptance();
    test_inspection_timeline_rejects_gaps_identity_drift_and_false_zero();
    test_inspection_timeline_requires_acceptance_and_one_terminal();
    test_mcap_round_trip_and_atomic_commit();
    test_mcap_rejects_conflicts_before_open_and_retains_uncommitted_temp();
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
