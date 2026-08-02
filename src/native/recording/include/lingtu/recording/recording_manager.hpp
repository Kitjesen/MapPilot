#pragma once

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace lingtu::recording {

enum class RecordingState {
  kIdle,
  kPreparing,
  kRecording,
  kStopping,
  kCompleted,
  kFailed,
};

const char *recording_state_name(RecordingState state) noexcept;

struct RecordingChildSpec {
  std::string name;
  std::vector<std::string> argv;
  bool required{true};
  std::vector<std::filesystem::path> artifacts;
  std::vector<std::string> selected_topics;
  std::vector<std::string> required_topics;
};

struct RecordingContext {
  std::string product;
  std::string run_plan_fingerprint;
  std::string robot_id;
  std::string task_id;
};

struct RecordingSpec {
  std::filesystem::path session_directory;
  std::string session_id;
  RecordingContext context;
  std::uint64_t minimum_free_bytes{0};
  std::vector<RecordingChildSpec> children;
};

struct RecordingProcessSnapshot {
  std::int64_t process_id{-1};
  bool running{false};
  std::optional<int> exit_code;
};

struct RecordingProcessSpec {
  std::string name;
  std::vector<std::string> argv;
  std::filesystem::path working_directory;
  std::filesystem::path stdout_log;
  std::filesystem::path stderr_log;
};

class RecordingProcess {
 public:
  virtual ~RecordingProcess() = default;

  virtual RecordingProcessSnapshot snapshot() = 0;
  virtual void request_stop() = 0;
  virtual int wait(std::chrono::milliseconds timeout) = 0;
};

class RecordingProcessFactory {
 public:
  virtual ~RecordingProcessFactory() = default;

  virtual std::unique_ptr<RecordingProcess> start(const RecordingProcessSpec &spec) = 0;
};

struct RecordingChildStatus {
  std::string name;
  bool required{true};
  std::int64_t process_id{-1};
  bool running{false};
  std::optional<int> exit_code;
  std::vector<std::string> argv;
  std::filesystem::path stdout_log;
  std::filesystem::path stderr_log;
  std::vector<std::filesystem::path> artifacts;
  std::vector<std::string> selected_topics;
  std::vector<std::string> required_topics;
};

struct RecordingStatus {
  std::string session_id;
  std::filesystem::path session_directory;
  RecordingContext context;
  std::int64_t manager_process_id{-1};
  RecordingState state{RecordingState::kIdle};
  std::uint64_t created_at_unix_ns{0};
  std::uint64_t started_at_unix_ns{0};
  std::uint64_t ended_at_unix_ns{0};
  std::uint64_t minimum_free_bytes{0};
  std::uint64_t available_bytes_at_start{0};
  std::vector<RecordingChildStatus> children;
  std::string error;
};

class RecordingManager {
 public:
  explicit RecordingManager(RecordingProcessFactory &process_factory);
  ~RecordingManager();

  RecordingManager(const RecordingManager &) = delete;
  RecordingManager &operator=(const RecordingManager &) = delete;

  RecordingStatus start(const RecordingSpec &spec);
  RecordingStatus status();
  RecordingStatus stop(std::chrono::milliseconds grace_period);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lingtu::recording
