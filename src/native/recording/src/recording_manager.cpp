#include "lingtu/recording/recording_manager.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <utility>

#if defined(_WIN32)
#include <Windows.h>
#include <io.h>
#include <process.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace lingtu::recording {
namespace {

std::uint64_t unix_time_ns() {
  const auto elapsed = std::chrono::system_clock::now().time_since_epoch();
  const auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count();
  return value < 0 ? 0 : static_cast<std::uint64_t>(value);
}

std::int64_t current_process_id() noexcept {
#if defined(_WIN32)
  return static_cast<std::int64_t>(::_getpid());
#else
  return static_cast<std::int64_t>(::getpid());
#endif
}

bool safe_name(std::string_view value) {
  if (value.empty() || value.size() > 128 || value == "." || value == "..") {
    return false;
  }
  return std::all_of(value.begin(), value.end(), [](unsigned char character) {
    return (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
           (character >= '0' && character <= '9') || character == '-' || character == '_' ||
           character == '.';
  });
}

bool relative_artifact_path(const std::filesystem::path &path) {
  if (path.empty() || path.is_absolute() || path.has_root_name() || path.has_root_directory()) {
    return false;
  }
  for (const auto &part : path) {
    if (part == "..") {
      return false;
    }
  }
  return true;
}

void validate_spec(const RecordingSpec &spec) {
  if (spec.session_directory.empty()) {
    throw std::invalid_argument("recording session directory must not be empty");
  }
  if (!safe_name(spec.session_id)) {
    throw std::invalid_argument("recording session id must use [A-Za-z0-9._-]");
  }
  if (spec.children.empty()) {
    throw std::invalid_argument("recording session must contain at least one recorder");
  }
  const auto valid_context_value = [](const std::string &value) {
    return value.size() <= 256 && value.find('\0') == std::string::npos;
  };
  if (!valid_context_value(spec.context.product) ||
      !valid_context_value(spec.context.run_plan_fingerprint) ||
      !valid_context_value(spec.context.robot_id) || !valid_context_value(spec.context.task_id)) {
    throw std::invalid_argument("recording context values must be at most 256 bytes");
  }
  std::set<std::string> names;
  for (const auto &child : spec.children) {
    if (!safe_name(child.name) || !names.insert(child.name).second) {
      throw std::invalid_argument("recording child names must be unique safe identifiers");
    }
    if (child.argv.empty() || child.argv.front().empty()) {
      throw std::invalid_argument("recording child argv must contain an executable");
    }
    for (const auto &argument : child.argv) {
      if (argument.find('\0') != std::string::npos) {
        throw std::invalid_argument("recording child argv contains a NUL byte");
      }
    }
    for (const auto &artifact : child.artifacts) {
      if (!relative_artifact_path(artifact)) {
        throw std::invalid_argument("recording artifact paths must stay inside the session");
      }
    }
    std::set<std::string> selected_topics;
    for (const auto &topic : child.selected_topics) {
      if (topic.empty() || topic.size() > 512 || topic.find('\0') != std::string::npos ||
          !selected_topics.insert(topic).second) {
        throw std::invalid_argument("selected recording topics must be unique non-empty names");
      }
    }
    std::set<std::string> required_topics;
    for (const auto &topic : child.required_topics) {
      if (!required_topics.insert(topic).second || selected_topics.count(topic) == 0) {
        throw std::invalid_argument(
            "required recording topics must be unique members of selected_topics");
      }
    }
  }
}

std::string json_string(std::string_view value) {
  std::ostringstream output;
  output << '"';
  static constexpr char kHex[] = "0123456789abcdef";
  for (const unsigned char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (character < 0x20) {
          output << "\\u00" << kHex[character >> 4U] << kHex[character & 0x0fU];
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
  return output.str();
}

void append_optional_number(std::ostringstream &output, const std::optional<int> &value) {
  if (value) {
    output << *value;
  } else {
    output << "null";
  }
}

std::string manifest_json(const RecordingStatus &status) {
  std::ostringstream output;
  output << "{\"version\":1,\"session_id\":" << json_string(status.session_id)
         << ",\"state\":" << json_string(recording_state_name(status.state))
         << ",\"session_directory\":" << json_string(status.session_directory.generic_string())
         << ",\"context\":{\"product\":" << json_string(status.context.product)
         << ",\"run_plan_fingerprint\":" << json_string(status.context.run_plan_fingerprint)
         << ",\"robot_id\":" << json_string(status.context.robot_id)
         << ",\"task_id\":" << json_string(status.context.task_id) << "}"
         << ",\"storage\":{\"minimum_free_bytes\":" << status.minimum_free_bytes
         << ",\"available_bytes_at_start\":" << status.available_bytes_at_start
         << "}"
         << ",\"manager_pid\":" << status.manager_process_id
         << ",\"created_at_unix_ns\":" << status.created_at_unix_ns << ",\"started_at_unix_ns\":";
  if (status.started_at_unix_ns == 0) {
    output << "null";
  } else {
    output << status.started_at_unix_ns;
  }
  output << ",\"ended_at_unix_ns\":";
  if (status.ended_at_unix_ns == 0) {
    output << "null";
  } else {
    output << status.ended_at_unix_ns;
  }
  output << ",\"error\":";
  if (status.error.empty()) {
    output << "null";
  } else {
    output << json_string(status.error);
  }
  output << ",\"children\":[";
  for (std::size_t index = 0; index < status.children.size(); ++index) {
    const auto &child = status.children[index];
    if (index != 0) {
      output << ',';
    }
    output << "{\"name\":" << json_string(child.name)
           << ",\"required\":" << (child.required ? "true" : "false")
           << ",\"pid\":" << child.process_id
           << ",\"state\":" << json_string(child.running ? "running" : "exited")
           << ",\"exit_code\":";
    append_optional_number(output, child.exit_code);
    output << ",\"argv\":[";
    for (std::size_t argument_index = 0; argument_index < child.argv.size(); ++argument_index) {
      if (argument_index != 0) {
        output << ',';
      }
      output << json_string(child.argv[argument_index]);
    }
    output << "],\"stdout_log\":" << json_string(child.stdout_log.generic_string())
           << ",\"stderr_log\":" << json_string(child.stderr_log.generic_string())
           << ",\"artifacts\":[";
    for (std::size_t artifact_index = 0; artifact_index < child.artifacts.size();
         ++artifact_index) {
      if (artifact_index != 0) {
        output << ',';
      }
      output << json_string(child.artifacts[artifact_index].generic_string());
    }
    output << "],\"selected_topics\":[";
    for (std::size_t topic_index = 0; topic_index < child.selected_topics.size(); ++topic_index) {
      if (topic_index != 0) {
        output << ',';
      }
      output << json_string(child.selected_topics[topic_index]);
    }
    output << "],\"required_topics\":[";
    for (std::size_t topic_index = 0; topic_index < child.required_topics.size(); ++topic_index) {
      if (topic_index != 0) {
        output << ',';
      }
      output << json_string(child.required_topics[topic_index]);
    }
    output << "]}";
  }
  output << "]}\n";
  return output.str();
}

void sync_file(std::FILE *file) {
  if (std::fflush(file) != 0) {
    throw std::runtime_error("failed to flush recording manifest");
  }
#if defined(_WIN32)
  if (_commit(_fileno(file)) != 0) {
    throw std::runtime_error("failed to commit recording manifest");
  }
#else
  if (::fdatasync(fileno(file)) != 0) {
    throw std::runtime_error("failed to sync recording manifest");
  }
#endif
}

void sync_directory(const std::filesystem::path &input) {
#if defined(_WIN32)
  static_cast<void>(input);
#else
  const auto directory = input.empty() ? std::filesystem::path(".") : input;
  const int fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (fd < 0) {
    throw std::runtime_error("failed to open recording directory for sync: " +
                             std::string(std::strerror(errno)));
  }
  const int result = ::fsync(fd);
  const int sync_error = errno;
  ::close(fd);
  if (result != 0) {
    throw std::runtime_error("failed to sync recording directory: " +
                             std::string(std::strerror(sync_error)));
  }
#endif
}

void replace_file(const std::filesystem::path &source, const std::filesystem::path &destination) {
#if defined(_WIN32)
  if (!::MoveFileExW(source.c_str(), destination.c_str(),
                     MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
    throw std::runtime_error("failed to promote recording manifest: Windows error " +
                             std::to_string(::GetLastError()));
  }
#else
  std::filesystem::rename(source, destination);
  sync_directory(destination.parent_path());
#endif
}

void write_manifest(const RecordingStatus &status) {
  const auto final_path = status.session_directory / "session.json";
  const auto temporary_path = status.session_directory / "session.json.tmp";
  const auto content = manifest_json(status);
#if defined(_WIN32)
  std::FILE *file = _wfopen(temporary_path.c_str(), L"wb");
#else
  std::FILE *file = std::fopen(temporary_path.c_str(), "wb");
#endif
  if (file == nullptr) {
    throw std::runtime_error("failed to open recording manifest temporary file: " +
                             std::string(std::strerror(errno)));
  }
  try {
    if (std::fwrite(content.data(), 1, content.size(), file) != content.size()) {
      throw std::runtime_error("failed to write recording manifest");
    }
    sync_file(file);
    if (std::fclose(file) != 0) {
      file = nullptr;
      throw std::runtime_error("failed to close recording manifest");
    }
    file = nullptr;
    replace_file(temporary_path, final_path);
  } catch (...) {
    if (file != nullptr) {
      std::fclose(file);
    }
    throw;
  }
}

}  // namespace

const char *recording_state_name(RecordingState state) noexcept {
  switch (state) {
    case RecordingState::kIdle:
      return "idle";
    case RecordingState::kPreparing:
      return "preparing";
    case RecordingState::kRecording:
      return "recording";
    case RecordingState::kStopping:
      return "stopping";
    case RecordingState::kCompleted:
      return "completed";
    case RecordingState::kFailed:
      return "failed";
  }
  return "failed";
}

class RecordingManager::Impl {
 public:
  explicit Impl(RecordingProcessFactory &input_factory) : process_factory(input_factory) {}

  ~Impl() {
    if (current.state == RecordingState::kRecording ||
        current.state == RecordingState::kPreparing || current.state == RecordingState::kStopping) {
      try {
        stop(std::chrono::seconds(2));
      } catch (...) {}
    }
  }

  RecordingStatus start(const RecordingSpec &spec) {
    if (current.state != RecordingState::kIdle) {
      throw std::logic_error("recording manager already owns a session");
    }
    validate_spec(spec);
    if (std::filesystem::exists(spec.session_directory)) {
      throw std::runtime_error("recording session directory already exists: " +
                               spec.session_directory.string());
    }
    const auto parent = spec.session_directory.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
    const auto storage_directory = parent.empty() ? std::filesystem::path(".") : parent;
    std::error_code storage_error;
    const auto storage = std::filesystem::space(storage_directory, storage_error);
    if (storage_error) {
      throw std::runtime_error("failed to query recording storage for " +
                               storage_directory.string() + ": " + storage_error.message());
    }
    const auto maximum_bytes = (std::numeric_limits<std::uint64_t>::max)();
    const auto available_bytes =
        storage.available > static_cast<std::uintmax_t>(maximum_bytes)
            ? maximum_bytes
            : static_cast<std::uint64_t>(storage.available);
    if (available_bytes < spec.minimum_free_bytes) {
      throw std::runtime_error("insufficient recording storage: required=" +
                               std::to_string(spec.minimum_free_bytes) + " available=" +
                               std::to_string(available_bytes));
    }
    if (!std::filesystem::create_directory(spec.session_directory)) {
      throw std::runtime_error("failed to create recording session directory");
    }
    std::filesystem::create_directory(spec.session_directory / "logs");
    sync_directory(spec.session_directory);
    sync_directory(parent);

    current.session_id = spec.session_id;
    current.session_directory = spec.session_directory;
    current.context = spec.context;
    current.manager_process_id = current_process_id();
    current.minimum_free_bytes = spec.minimum_free_bytes;
    current.available_bytes_at_start = available_bytes;
    current.state = RecordingState::kPreparing;
    current.created_at_unix_ns = unix_time_ns();
    current.children.reserve(spec.children.size());
    write_manifest(current);

    try {
      for (const auto &child_spec : spec.children) {
        const auto stdout_relative =
            std::filesystem::path("logs") / (child_spec.name + ".stdout.log");
        const auto stderr_relative =
            std::filesystem::path("logs") / (child_spec.name + ".stderr.log");
        RecordingProcessSpec process_spec;
        process_spec.name = child_spec.name;
        process_spec.argv = child_spec.argv;
        process_spec.working_directory = spec.session_directory;
        process_spec.stdout_log = spec.session_directory / stdout_relative;
        process_spec.stderr_log = spec.session_directory / stderr_relative;
        auto process = process_factory.start(process_spec);
        if (!process) {
          throw std::runtime_error("recording process factory returned an empty process");
        }
        const auto process_snapshot = process->snapshot();
        if (!process_snapshot.running) {
          throw std::runtime_error("recording child did not remain running: " + child_spec.name);
        }
        RecordingChildStatus child_status;
        child_status.name = child_spec.name;
        child_status.required = child_spec.required;
        child_status.process_id = process_snapshot.process_id;
        child_status.running = process_snapshot.running;
        child_status.exit_code = process_snapshot.exit_code;
        child_status.argv = child_spec.argv;
        child_status.stdout_log = stdout_relative;
        child_status.stderr_log = stderr_relative;
        child_status.artifacts = child_spec.artifacts;
        child_status.selected_topics = child_spec.selected_topics;
        child_status.required_topics = child_spec.required_topics;
        current.children.push_back(std::move(child_status));
        processes.push_back(std::move(process));
      }
      current.started_at_unix_ns = unix_time_ns();
      current.state = RecordingState::kRecording;
      write_manifest(current);
      return current;
    } catch (const std::exception &error) {
      fail_started_children(error.what());
      throw;
    }
  }

  RecordingStatus status() {
    std::optional<std::size_t> failed_required_child;
    bool changed = false;
    for (std::size_t index = 0; index < processes.size(); ++index) {
      const auto snapshot = processes[index]->snapshot();
      changed = changed || current.children[index].process_id != snapshot.process_id ||
                current.children[index].running != snapshot.running ||
                current.children[index].exit_code != snapshot.exit_code;
      current.children[index].process_id = snapshot.process_id;
      current.children[index].running = snapshot.running;
      current.children[index].exit_code = snapshot.exit_code;
      if (current.state == RecordingState::kRecording && current.children[index].required &&
          !snapshot.running) {
        failed_required_child = index;
        break;
      }
    }
    if (failed_required_child) {
      fail_unexpected_exit(*failed_required_child);
    } else if (changed) {
      write_manifest(current);
    }
    return current;
  }

  RecordingStatus stop(std::chrono::milliseconds grace_period) {
    if (grace_period.count() < 0) {
      throw std::invalid_argument("recording stop grace period must not be negative");
    }
    if (current.state == RecordingState::kIdle || current.state == RecordingState::kCompleted ||
        current.state == RecordingState::kFailed) {
      return current;
    }
    current.state = RecordingState::kStopping;
    write_manifest(current);

    bool required_failed = false;
    std::string error;
    for (std::size_t index = processes.size(); index-- > 0;) {
      try {
        processes[index]->request_stop();
      } catch (const std::exception &stop_error) {
        if (current.children[index].required) {
          required_failed = true;
          if (error.empty()) {
            error = "failed to stop required recorder " + current.children[index].name + ": " +
                    stop_error.what();
          }
        }
      }
    }

    const auto deadline = std::chrono::steady_clock::now() + grace_period;
    for (std::size_t index = 0; index < processes.size(); ++index) {
      try {
        const auto now = std::chrono::steady_clock::now();
        const auto remaining =
            now >= deadline ? std::chrono::milliseconds(0)
                            : std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
        const int exit_code = processes[index]->wait(remaining);
        current.children[index].running = false;
        current.children[index].exit_code = exit_code;
        if (current.children[index].required && exit_code != 0) {
          required_failed = true;
          error = "required recorder exited with code " + std::to_string(exit_code) + ": " +
                  current.children[index].name;
        }
      } catch (const std::exception &wait_error) {
        current.children[index].running = false;
        required_failed = required_failed || current.children[index].required;
        if (error.empty()) {
          error = wait_error.what();
        }
      }
    }
    for (const auto &child : current.children) {
      if (!child.required) {
        continue;
      }
      for (const auto &artifact : child.artifacts) {
        const auto artifact_path = current.session_directory / artifact;
        std::error_code artifact_error;
        const auto artifact_status = std::filesystem::symlink_status(artifact_path, artifact_error);
        if (artifact_error || !std::filesystem::is_regular_file(artifact_status)) {
          required_failed = true;
          error = "required recording artifact is missing or not a regular file: " +
                  artifact.generic_string();
        } else if (std::filesystem::file_size(artifact_path, artifact_error) == 0 ||
                   artifact_error) {
          required_failed = true;
          error = "required recording artifact is empty: " + artifact.generic_string();
        }
      }
    }
    current.ended_at_unix_ns = unix_time_ns();
    current.error = std::move(error);
    current.state = required_failed ? RecordingState::kFailed : RecordingState::kCompleted;
    write_manifest(current);
    return current;
  }

 private:
  void fail_unexpected_exit(std::size_t failed_index) {
    const auto failed_name = current.children[failed_index].name;
    const auto failed_code = current.children[failed_index].exit_code;
    for (std::size_t index = 0; index < processes.size(); ++index) {
      if (index == failed_index) {
        continue;
      }
      const auto snapshot = processes[index]->snapshot();
      current.children[index].process_id = snapshot.process_id;
      current.children[index].running = snapshot.running;
      current.children[index].exit_code = snapshot.exit_code;
      if (snapshot.running) {
        processes[index]->request_stop();
      }
    }
    for (std::size_t index = 0; index < processes.size(); ++index) {
      if (index == failed_index || !current.children[index].running) {
        continue;
      }
      try {
        current.children[index].exit_code = processes[index]->wait(std::chrono::seconds(2));
      } catch (const std::exception &) {}
      current.children[index].running = false;
    }
    current.state = RecordingState::kFailed;
    current.ended_at_unix_ns = unix_time_ns();
    current.error = "required recorder exited unexpectedly: " + failed_name;
    if (failed_code) {
      current.error += " (exit code " + std::to_string(*failed_code) + ")";
    }
    write_manifest(current);
  }

  void fail_started_children(const std::string &message) noexcept {
    for (auto process = processes.rbegin(); process != processes.rend(); ++process) {
      try {
        (*process)->request_stop();
      } catch (...) {}
    }
    for (std::size_t index = 0; index < processes.size(); ++index) {
      try {
        const int exit_code = processes[index]->wait(std::chrono::seconds(2));
        current.children[index].running = false;
        current.children[index].exit_code = exit_code;
        continue;
      } catch (...) {}
      try {
        const auto snapshot = processes[index]->snapshot();
        current.children[index].process_id = snapshot.process_id;
        current.children[index].running = snapshot.running;
        if (snapshot.exit_code) {
          current.children[index].exit_code = snapshot.exit_code;
        }
      } catch (...) {}
    }
    current.state = RecordingState::kFailed;
    current.ended_at_unix_ns = unix_time_ns();
    current.error = message;
    try {
      write_manifest(current);
    } catch (...) {}
  }

  RecordingProcessFactory &process_factory;
  RecordingStatus current;
  std::vector<std::unique_ptr<RecordingProcess>> processes;
};

RecordingManager::RecordingManager(RecordingProcessFactory &process_factory)
    : impl_(std::make_unique<Impl>(process_factory)) {}

RecordingManager::~RecordingManager() = default;

RecordingStatus RecordingManager::start(const RecordingSpec &spec) {
  return impl_->start(spec);
}

RecordingStatus RecordingManager::status() {
  return impl_->status();
}

RecordingStatus RecordingManager::stop(std::chrono::milliseconds grace_period) {
  return impl_->stop(grace_period);
}

}  // namespace lingtu::recording
