#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "lingtu/recording/recording_manager.hpp"

namespace {

namespace recording = lingtu::recording;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::filesystem::path unique_directory(const std::string &name) {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("lingtu-recording-manager-" + name + "-" + std::to_string(nonce));
}

std::string read_text(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

void write_artifact(const std::filesystem::path &path) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  output << "recorded";
  if (!output) {
    throw std::runtime_error("failed to create fake recording artifact");
  }
}

class FakeProcess final : public recording::RecordingProcess {
 public:
  explicit FakeProcess(std::int64_t process_id) : process_id_(process_id) {}

  recording::RecordingProcessSnapshot snapshot() override {
    if (throw_after_snapshot_count && snapshot_count >= *throw_after_snapshot_count) {
      throw std::runtime_error("intentional snapshot failure");
    }
    ++snapshot_count;
    return {process_id_, running_, exit_code_};
  }

  void request_stop() override {
    stop_requested_ = true;
    if (throw_on_stop) {
      throw std::runtime_error("intentional stop request failure");
    }
  }

  int wait(std::chrono::milliseconds timeout) override {
    wait_timeouts.push_back(timeout);
    if (wait_delay.count() > 0) {
      std::this_thread::sleep_for(wait_delay);
    }
    running_ = false;
    exit_code_ = 0;
    return *exit_code_;
  }

  bool stop_requested() const noexcept { return stop_requested_; }

  void finish(int exit_code) {
    running_ = false;
    exit_code_ = exit_code;
  }

  std::chrono::milliseconds wait_delay{0};
  std::vector<std::chrono::milliseconds> wait_timeouts;
  std::optional<std::size_t> throw_after_snapshot_count;
  bool throw_on_stop{false};

 private:
  std::int64_t process_id_;
  std::size_t snapshot_count{0};
  bool running_{true};
  bool stop_requested_{false};
  std::optional<int> exit_code_;
};

class FakeProcessFactory final : public recording::RecordingProcessFactory {
 public:
  std::unique_ptr<recording::RecordingProcess>
  start(const recording::RecordingProcessSpec &spec) override {
    if (fail_on_call && started.size() == *fail_on_call) {
      throw std::runtime_error("intentional recorder start failure");
    }
    started.push_back(spec);
    auto process = std::make_unique<FakeProcess>(next_process_id++);
    process->throw_after_snapshot_count = throw_after_snapshot_count;
    processes.push_back(process.get());
    return process;
  }

  std::vector<recording::RecordingProcessSpec> started;
  std::vector<FakeProcess *> processes;
  std::int64_t next_process_id{1000};
  std::optional<std::size_t> fail_on_call;
  std::optional<std::size_t> throw_after_snapshot_count;
};

void test_start_creates_one_observable_session() {
  const auto root = unique_directory("start");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-001";
  spec.context.product = "nav";
  spec.context.product_session_id = "product-session-test";
  spec.context.robot_id = "s100p-01";
  spec.context.task_id = "inspection-task-1";
  spec.minimum_free_bytes = 1;
  spec.children = {
      {"dds", {"lingtu_dds_recorder", "--output", "dds/sensors.mcap"}, true, {"dds/sensors.mcap"}},
      {"camera_color",
       {"lingtu_camera_recorder", "--output-dir", ".", "--color-shm", "/lingtu_camera_color"},
       true,
       {"camera_color.mcap"}},
  };
  spec.children.front().selected_topics = {"/imu/raw", "/nav/inspection/task/event",
                                           "/nav/cmd_vel"};
  spec.children.front().required_topics = {"/nav/inspection/task/event"};

  const auto status = manager.start(spec);
  require(status.state == recording::RecordingState::kRecording,
          "manager did not enter recording state");
  require(status.session_id == "field-001", "manager changed the session id");
  require(status.minimum_free_bytes == 1,
          "manager did not expose the effective storage threshold");
  require(status.available_bytes_at_start >= status.minimum_free_bytes,
          "manager reported an admitted session without enough storage");
  require(status.children.size() == 2, "manager did not expose both recorders");
  require(status.manager_process_id > 0, "manager process id is not observable");
  require(factory.started.size() == 2, "manager did not start both recorders");
  require(factory.started.front().working_directory == root,
          "manager did not isolate recorder working directory to the session");
  require(factory.started.front().stdout_log == root / "logs" / "dds.stdout.log" &&
              factory.started.front().stderr_log == root / "logs" / "dds.stderr.log",
          "manager did not assign deterministic recorder log paths");
  require(std::filesystem::exists(root / "session.json"), "session manifest is missing");
  require(!std::filesystem::exists(root / "session.json.tmp"),
          "temporary manifest survived atomic promotion");

  const auto manifest = read_text(root / "session.json");
  require(manifest.find("\"session_id\":\"field-001\"") != std::string::npos,
          "manifest omitted the session id");
  require(manifest.find("\"manager_pid\":") != std::string::npos,
          "manifest omitted the manager process id");
  require(manifest.find("\"state\":\"recording\"") != std::string::npos,
          "manifest omitted the recording state");
  require(manifest.find("\"storage\":{\"minimum_free_bytes\":1,\"available_bytes_at_start\":") !=
              std::string::npos,
          "manifest omitted the storage admission evidence");
  require(manifest.find("\"product\":\"nav\"") != std::string::npos &&
              manifest.find("\"product_session_id\":\"product-session-test\"") != std::string::npos &&
              manifest.find("\"robot_id\":\"s100p-01\"") != std::string::npos &&
              manifest.find("\"task_id\":\"inspection-task-1\"") != std::string::npos,
          "manifest omitted the Product recording context");
  require(manifest.find("\"name\":\"dds\"") != std::string::npos,
          "manifest omitted the DDS recorder");
  require(manifest.find("\"name\":\"camera_color\"") != std::string::npos,
          "manifest omitted the camera recorder");
  require(manifest.find("\"argv\":[\"lingtu_dds_recorder\"") != std::string::npos &&
              manifest.find("/lingtu_camera_color") != std::string::npos,
          "manifest omitted the effective recorder configuration");
  require(
      manifest.find(
          "\"selected_topics\":[\"/imu/raw\",\"/nav/inspection/task/event\",\"/nav/cmd_vel\"]") !=
          std::string::npos,
      "manifest omitted the DDS capture plan");
  require(manifest.find("\"required_topics\":[\"/nav/inspection/task/event\"]") !=
              std::string::npos,
          "manifest omitted the required evidence topics");

  static_cast<void>(manager.stop(std::chrono::milliseconds(10)));
  std::filesystem::remove_all(root);
}

void test_start_rejects_insufficient_storage_before_session_or_children() {
  const auto root = unique_directory("insufficient-storage");
  FakeProcessFactory factory;
  recording::RecordingManager manager(
      factory, [](const std::filesystem::path &) { return std::uint64_t{9}; },
      std::chrono::milliseconds(0));

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-no-space";
  spec.minimum_free_bytes = 10;
  spec.children = {{"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}}};

  bool rejected = false;
  try {
    static_cast<void>(manager.start(spec));
  } catch (const std::runtime_error &error) {
    const std::string message(error.what());
    rejected = message.find("insufficient recording storage") != std::string::npos &&
               message.find("required=") != std::string::npos &&
               message.find("available=") != std::string::npos;
  }
  require(rejected, "manager did not explain the insufficient-storage rejection");
  require(factory.started.empty(), "manager started a recorder before storage admission");
  require(!std::filesystem::exists(root),
          "manager created the session directory before storage admission");
}

void test_runtime_storage_low_stops_children_and_persists_failed_state() {
  const auto root = unique_directory("runtime-storage-low");
  FakeProcessFactory factory;
  std::size_t storage_probe_count = 0;
  recording::RecordingManager manager(
      factory,
      [&](const std::filesystem::path &) {
        ++storage_probe_count;
        return storage_probe_count == 1 ? std::uint64_t{10} : std::uint64_t{9};
      },
      std::chrono::milliseconds(0));

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-runtime-storage-low";
  spec.minimum_free_bytes = 10;
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };
  static_cast<void>(manager.start(spec));
  write_artifact(root / "dds" / "sensors.mcap");
  write_artifact(root / "camera_color.mcap");
  factory.processes.front()->wait_delay = std::chrono::milliseconds(30);

  const auto status = manager.status();
  require(status.state == recording::RecordingState::kFailed,
          "runtime low storage did not fail the recording session");
  require(status.error.find("recording_storage_low") != std::string::npos,
          "runtime low-storage failure did not expose its stable reason");
  require(factory.processes.front()->stop_requested() && factory.processes.back()->stop_requested(),
          "runtime low storage did not stop every recorder");
  require(!factory.processes.front()->wait_timeouts.empty() &&
              !factory.processes.back()->wait_timeouts.empty() &&
              factory.processes.back()->wait_timeouts.front() <
                  factory.processes.front()->wait_timeouts.front(),
          "runtime low-storage shutdown did not use one shared grace deadline");
  require(!status.children.front().running && !status.children.back().running,
          "runtime low storage left a recorder running in status");
  require(status.ended_at_unix_ns >= status.started_at_unix_ns,
          "runtime low-storage failure did not persist a terminal time");

  const auto manifest = read_text(root / "session.json");
  require(manifest.find("\"state\":\"failed\"") != std::string::npos &&
              manifest.find("recording_storage_low") != std::string::npos,
          "runtime low-storage failure was not persisted in the manifest");
  const auto terminal = manager.status();
  require(terminal.state == recording::RecordingState::kFailed,
          "runtime low-storage session later claimed a non-failed state");
  require(storage_probe_count == 2,
          "terminal low-storage session continued probing the filesystem");

  std::filesystem::remove_all(root);
}

void test_runtime_storage_query_failure_stops_children_and_persists_failed_state() {
  const auto root = unique_directory("runtime-storage-unavailable");
  FakeProcessFactory factory;
  std::size_t storage_probe_count = 0;
  recording::RecordingManager manager(
      factory,
      [&](const std::filesystem::path &) -> std::uint64_t {
        ++storage_probe_count;
        if (storage_probe_count == 1) {
          return 100;
        }
        throw std::runtime_error("intentional storage query failure");
      },
      std::chrono::milliseconds(0));

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-runtime-storage-unavailable";
  spec.minimum_free_bytes = 10;
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };
  static_cast<void>(manager.start(spec));
  write_artifact(root / "dds" / "sensors.mcap");
  write_artifact(root / "camera_color.mcap");

  recording::RecordingStatus status;
  bool query_escaped = false;
  try {
    status = manager.status();
  } catch (const std::exception &) {
    query_escaped = true;
  }
  require(!query_escaped, "runtime storage query failure escaped the recording manager");
  require(status.state == recording::RecordingState::kFailed,
          "runtime storage query failure did not fail the recording session");
  require(status.error == "recording_storage_status_unavailable",
          "runtime storage query failure did not expose its stable reason");
  require(factory.processes.front()->stop_requested() && factory.processes.back()->stop_requested(),
          "runtime storage query failure did not stop every recorder");
  require(read_text(root / "session.json").find("recording_storage_status_unavailable") !=
              std::string::npos,
          "runtime storage query failure was not persisted in the manifest");

  std::filesystem::remove_all(root);
}

void test_runtime_storage_probe_is_throttled() {
  const auto root = unique_directory("runtime-storage-throttle");
  FakeProcessFactory factory;
  std::size_t storage_probe_count = 0;
  recording::RecordingManager manager(
      factory,
      [&](const std::filesystem::path &) {
        ++storage_probe_count;
        return std::uint64_t{100};
      },
      std::chrono::milliseconds(3600000));

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-runtime-storage-throttle";
  spec.minimum_free_bytes = 10;
  spec.children = {{"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}}};
  static_cast<void>(manager.start(spec));

  const auto first = manager.status();
  const auto second = manager.status();
  require(first.state == recording::RecordingState::kRecording &&
              second.state == recording::RecordingState::kRecording,
          "throttled storage checks changed a healthy recording state");
  require(storage_probe_count == 1,
          "runtime storage probe ignored its throttle interval");

  write_artifact(root / "dds" / "sensors.mcap");
  static_cast<void>(manager.stop(std::chrono::milliseconds(10)));
  std::filesystem::remove_all(root);
}

void test_required_recorder_exit_fails_session_and_stops_peers() {
  const auto root = unique_directory("unexpected-exit");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-failure";
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };
  static_cast<void>(manager.start(spec));

  factory.processes.front()->finish(17);
  const auto status = manager.status();
  require(status.state == recording::RecordingState::kFailed,
          "required recorder exit did not fail the session");
  require(status.error.find("dds") != std::string::npos,
          "failed status did not name the required recorder");
  require(factory.processes.back()->stop_requested(),
          "manager did not stop the remaining recorder after peer failure");
  require(read_text(root / "session.json").find("\"state\":\"failed\"") != std::string::npos,
          "failed state was not persisted");

  std::filesystem::remove_all(root);
}

void test_failed_start_rolls_back_started_recorders_and_persists_truth() {
  const auto root = unique_directory("start-rollback");
  FakeProcessFactory factory;
  factory.fail_on_call = 1;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-rollback";
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };

  bool rejected = false;
  try {
    static_cast<void>(manager.start(spec));
  } catch (const std::runtime_error &) {
    rejected = true;
  }
  require(rejected, "manager accepted a partially started session");
  require(factory.processes.size() == 1, "test did not reach the partial-start state");
  require(factory.processes.front()->stop_requested(),
          "manager did not stop the recorder started before failure");

  const auto status = manager.status();
  require(status.state == recording::RecordingState::kFailed,
          "failed start did not leave an inspectable failed session");
  require(!status.children.front().running,
          "rolled-back recorder still appears to be running in status");
  const auto manifest = read_text(root / "session.json");
  require(manifest.find("\"state\":\"failed\"") != std::string::npos,
          "failed start was not written to the manifest");
  require(manifest.find("\"state\":\"exited\"") != std::string::npos,
          "rollback manifest still claims that its child is running");

  std::filesystem::remove_all(root);
}

void test_failed_start_cleanup_does_not_snapshot_after_successful_wait() {
  const auto root = unique_directory("start-cleanup-snapshot");
  FakeProcessFactory factory;
  factory.fail_on_call = 1;
  factory.throw_after_snapshot_count = 1;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-cleanup-snapshot";
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };

  bool rejected = false;
  try {
    static_cast<void>(manager.start(spec));
  } catch (const std::runtime_error &error) {
    rejected = std::string(error.what()).find("start failure") != std::string::npos;
  }
  require(rejected, "manager lost the original partial-start failure");
  require(read_text(root / "session.json").find("\"state\":\"failed\"") != std::string::npos,
          "snapshot failure prevented the rollback manifest");

  std::filesystem::remove_all(root);
}

void test_stop_completes_only_after_required_artifacts_exist() {
  const auto root = unique_directory("complete");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-complete";
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };
  static_cast<void>(manager.start(spec));
  write_artifact(root / "dds" / "sensors.mcap");
  write_artifact(root / "camera_color.mcap");

  const auto status = manager.stop(std::chrono::milliseconds(10));
  require(status.state == recording::RecordingState::kCompleted,
          "valid recording artifacts did not complete the session");
  require(status.ended_at_unix_ns >= status.started_at_unix_ns,
          "completed session has an invalid time range");
  require(factory.processes.front()->stop_requested() && factory.processes.back()->stop_requested(),
          "stop did not reach every recorder");
  require(read_text(root / "session.json").find("\"state\":\"completed\"") != std::string::npos,
          "completed state was not persisted");

  std::filesystem::remove_all(root);
}

void test_stop_fails_when_required_artifact_is_missing() {
  const auto root = unique_directory("missing-artifact");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-incomplete";
  spec.children = {{"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}}};
  static_cast<void>(manager.start(spec));

  const auto status = manager.stop(std::chrono::milliseconds(10));
  require(status.state == recording::RecordingState::kFailed,
          "session completed without its required artifact");
  require(status.error.find("dds/sensors.mcap") != std::string::npos,
          "missing artifact was not identified");

  std::filesystem::remove_all(root);
}

void test_stop_fails_when_required_artifact_is_not_a_regular_file() {
  const auto root = unique_directory("non-regular-artifact");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-non-regular-artifact";
  spec.children = {{"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}}};
  static_cast<void>(manager.start(spec));
  std::filesystem::create_directories(root / "dds" / "sensors.mcap");

  const auto status = manager.stop(std::chrono::milliseconds(10));
  require(status.state == recording::RecordingState::kFailed,
          "non-regular required artifact completed the session");
  require(status.error.find("not a regular file") != std::string::npos,
          "non-regular required artifact failure is not diagnosable");

  std::filesystem::remove_all(root);
}

void test_optional_recorder_exit_is_visible_without_failing_required_recording() {
  const auto root = unique_directory("optional-exit");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-degraded";
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, false, {"camera_color.mcap"}},
  };
  static_cast<void>(manager.start(spec));
  factory.processes.back()->finish(7);

  const auto status = manager.status();
  require(status.state == recording::RecordingState::kRecording,
          "optional recorder exit incorrectly failed the session");
  require(!status.children.back().running && status.children.back().exit_code == 7,
          "optional recorder exit is missing from live status");
  const auto manifest = read_text(root / "session.json");
  const auto optional_child = manifest.find("\"name\":\"camera_color\"");
  require(optional_child != std::string::npos &&
              manifest.find("\"state\":\"exited\"", optional_child) != std::string::npos,
          "optional recorder exit was not persisted");

  write_artifact(root / "dds" / "sensors.mcap");
  static_cast<void>(manager.stop(std::chrono::milliseconds(10)));
  std::filesystem::remove_all(root);
}

void test_stop_grace_period_is_one_session_deadline() {
  const auto root = unique_directory("stop-deadline");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-deadline";
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };
  static_cast<void>(manager.start(spec));
  write_artifact(root / "dds" / "sensors.mcap");
  write_artifact(root / "camera_color.mcap");
  factory.processes.front()->wait_delay = std::chrono::milliseconds(30);

  static_cast<void>(manager.stop(std::chrono::milliseconds(100)));
  require(!factory.processes.front()->wait_timeouts.empty() &&
              !factory.processes.back()->wait_timeouts.empty(),
          "manager did not wait for every recorder");
  require(factory.processes.back()->wait_timeouts.front() <
              factory.processes.front()->wait_timeouts.front(),
          "stop grace period was restarted for each recorder");

  std::filesystem::remove_all(root);
}

void test_stop_continues_after_one_required_signal_failure() {
  const auto root = unique_directory("stop-signal-failure");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-stop-signal-failure";
  spec.children = {
      {"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}},
      {"camera_color", {"lingtu_camera_recorder"}, true, {"camera_color.mcap"}},
  };
  static_cast<void>(manager.start(spec));
  write_artifact(root / "dds" / "sensors.mcap");
  write_artifact(root / "camera_color.mcap");
  factory.processes.back()->throw_on_stop = true;

  const auto status = manager.stop(std::chrono::milliseconds(10));
  require(status.state == recording::RecordingState::kFailed,
          "required signal failure did not fail the session");
  require(factory.processes.front()->stop_requested(),
          "signal failure prevented another recorder from being stopped");
  require(status.error.find("camera_color") != std::string::npos,
          "signal failure did not identify the affected recorder");

  std::filesystem::remove_all(root);
}

void test_empty_required_artifact_cannot_complete_session() {
  const auto root = unique_directory("empty-artifact");
  FakeProcessFactory factory;
  recording::RecordingManager manager(factory);

  recording::RecordingSpec spec;
  spec.session_directory = root;
  spec.session_id = "field-empty";
  spec.children = {{"dds", {"lingtu_dds_recorder"}, true, {"dds/sensors.mcap"}}};
  static_cast<void>(manager.start(spec));
  std::filesystem::create_directories(root / "dds");
  std::ofstream(root / "dds" / "sensors.mcap", std::ios::binary).close();

  const auto status = manager.stop(std::chrono::milliseconds(10));
  require(status.state == recording::RecordingState::kFailed,
          "empty artifact completed the session");
  require(status.error.find("empty") != std::string::npos,
          "empty artifact failure is not diagnosable");

  std::filesystem::remove_all(root);
}

}  // namespace

int main() {
  try {
    test_start_creates_one_observable_session();
    test_start_rejects_insufficient_storage_before_session_or_children();
    test_runtime_storage_low_stops_children_and_persists_failed_state();
    test_runtime_storage_query_failure_stops_children_and_persists_failed_state();
    test_runtime_storage_probe_is_throttled();
    test_required_recorder_exit_fails_session_and_stops_peers();
    test_failed_start_rolls_back_started_recorders_and_persists_truth();
    test_failed_start_cleanup_does_not_snapshot_after_successful_wait();
    test_stop_completes_only_after_required_artifacts_exist();
    test_stop_fails_when_required_artifact_is_missing();
    test_stop_fails_when_required_artifact_is_not_a_regular_file();
    test_optional_recorder_exit_is_visible_without_failing_required_recording();
    test_stop_grace_period_is_one_session_deadline();
    test_stop_continues_after_one_required_signal_failure();
    test_empty_required_artifact_cannot_complete_session();
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_recording_manager: %s\n", error.what());
    return 1;
  }
}
