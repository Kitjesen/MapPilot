#include "lingtu/recording/recording_windows.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace lingtu::recording {
namespace {

[[noreturn]] void windows_error(const std::string &operation, DWORD error = ::GetLastError()) {
  throw std::runtime_error(operation + " failed: Windows error " + std::to_string(error));
}

void close_handle(HANDLE &handle) noexcept {
  if (handle != nullptr && handle != INVALID_HANDLE_VALUE) {
    ::CloseHandle(handle);
    handle = nullptr;
  }
}

std::wstring stop_event_name(std::int64_t process_id, const std::string &session_id) {
  return L"Local\\LingTuRecordingStop_" + std::to_wstring(process_id) + L"_" +
         std::filesystem::path(session_id).generic_wstring();
}

HANDLE create_stop_event(std::int64_t process_id, const std::string &session_id) {
  const auto name = stop_event_name(process_id, session_id);
  HANDLE event = ::CreateEventW(nullptr, TRUE, FALSE, name.c_str());
  if (event == nullptr) {
    windows_error("creating recorder stop event");
  }
  return event;
}

std::wstring quote_argument(const std::string &argument) {
  // CreateProcess supplies one command line; preserve the CRT argv contract,
  // including spaces, embedded quotes, and trailing directory backslashes.
  const auto value = std::filesystem::path(argument).wstring();
  std::wstring quoted{L'"'};
  std::size_t slashes = 0;
  for (const auto character : value) {
    if (character == L'\\') {
      ++slashes;
      continue;
    }
    quoted.append(character == L'"' ? 2 * slashes + 1 : slashes, L'\\');
    quoted.push_back(character);
    slashes = 0;
  }
  quoted.append(2 * slashes, L'\\');
  quoted.push_back(L'"');
  return quoted;
}

HANDLE open_log(const std::filesystem::path &path) {
  SECURITY_ATTRIBUTES security{sizeof(security), nullptr, TRUE};
  // The controller promotes the manager log after its logs directory appears.
  HANDLE handle = ::CreateFileW(path.c_str(), GENERIC_WRITE,
                               FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                               &security, CREATE_NEW, FILE_ATTRIBUTE_NORMAL, nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    windows_error("opening recorder log " + path.string());
  }
  return handle;
}

class WindowsRecordingProcess final : public RecordingProcess {
 public:
  WindowsRecordingProcess(HANDLE process, DWORD process_id, HANDLE stop_event, HANDLE job,
                          bool owned)
      : process_(process), process_id_(process_id), stop_event_(stop_event), job_(job),
        owned_(owned) {}

  ~WindowsRecordingProcess() override {
    if (owned_) {
      try {
        if (snapshot().running) {
          request_stop();
          static_cast<void>(wait(std::chrono::seconds(2)));
        }
      } catch (...) {
        ::TerminateProcess(process_, 137);
        ::WaitForSingleObject(process_, INFINITE);
      }
    }
    close_handle(stop_event_);
    close_handle(process_);
    close_handle(job_);
  }

  RecordingProcessSnapshot snapshot() override {
    if (!exit_code_ && ::WaitForSingleObject(process_, 0) == WAIT_OBJECT_0) {
      DWORD code = 0;
      if (::GetExitCodeProcess(process_, &code) == 0) {
        windows_error("reading recorder exit status");
      }
      exit_code_ = static_cast<int>(code);
    }
    return {static_cast<std::int64_t>(process_id_), !exit_code_, exit_code_};
  }

  void request_stop() override {
    if (snapshot().running && ::SetEvent(stop_event_) == 0) {
      windows_error("requesting recorder stop");
    }
  }

  int wait(std::chrono::milliseconds timeout) override {
    if (timeout.count() < 0) {
      throw std::invalid_argument("recording process wait timeout must not be negative");
    }
    const auto milliseconds = static_cast<DWORD>(
        (std::min)(static_cast<std::uint64_t>(timeout.count()),
                   static_cast<std::uint64_t>(INFINITE - 1)));
    const auto result = ::WaitForSingleObject(process_, milliseconds);
    if (result == WAIT_TIMEOUT) {
      const BOOL stopped = job_ != nullptr ? ::TerminateJobObject(job_, 137)
                                           : ::TerminateProcess(process_, 137);
      if (stopped == 0) {
        windows_error("terminating recorder after stop timeout");
      }
      if (::WaitForSingleObject(process_, INFINITE) != WAIT_OBJECT_0) {
        windows_error("waiting for terminated recorder");
      }
    } else if (result != WAIT_OBJECT_0) {
      windows_error("waiting for recorder");
    }
    return *snapshot().exit_code;
  }

 private:
  HANDLE process_{nullptr};
  DWORD process_id_{0};
  HANDLE stop_event_{nullptr};
  HANDLE job_{nullptr};
  bool owned_{false};
  std::optional<int> exit_code_;
};

std::unique_ptr<RecordingProcess> launch(const RecordingProcessSpec &spec, bool owned,
                                        const std::string &session_id) {
  if (spec.name.empty() || spec.argv.empty() || spec.argv.front().empty() ||
      !std::filesystem::is_directory(spec.working_directory)) {
    throw std::invalid_argument("recording process requires a name, executable and working directory");
  }
  if (spec.stdout_log.empty() || spec.stderr_log.empty() ||
      (owned && spec.stdout_log == spec.stderr_log)) {
    throw std::invalid_argument("recording process requires distinct stdout and stderr logs");
  }
  std::wstring command;
  for (const auto &argument : spec.argv) {
    if (argument.find('\0') != std::string::npos) {
      throw std::invalid_argument("recording process argv contains a NUL byte");
    }
    if (!command.empty()) command.push_back(L' ');
    command += quote_argument(argument);
  }

  HANDLE stdout_log = nullptr;
  HANDLE stderr_log = nullptr;
  HANDLE null_input = nullptr;
  HANDLE job = nullptr;
  HANDLE event = nullptr;
  PROCESS_INFORMATION process{};
  try {
    stdout_log = open_log(spec.stdout_log);
    stderr_log = spec.stderr_log == spec.stdout_log ? stdout_log : open_log(spec.stderr_log);
    SECURITY_ATTRIBUTES security{sizeof(security), nullptr, TRUE};
    null_input = ::CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                              &security, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (null_input == INVALID_HANDLE_VALUE) windows_error("detaching recorder standard input");
    if (owned) {
      job = ::CreateJobObjectW(nullptr, nullptr);
      if (job == nullptr) windows_error("creating recorder process job");
      JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
      limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
      if (::SetInformationJobObject(job, JobObjectExtendedLimitInformation,
                                   &limits, sizeof(limits)) == 0) {
        windows_error("configuring recorder process job");
      }
    }
    STARTUPINFOEXW startup{};
    startup.StartupInfo.cb = sizeof(startup);
    startup.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    startup.StartupInfo.hStdInput = null_input;
    startup.StartupInfo.hStdOutput = stdout_log;
    startup.StartupInfo.hStdError = stderr_log;
    SIZE_T attribute_bytes = 0;
    ::InitializeProcThreadAttributeList(nullptr, 1, 0, &attribute_bytes);
    std::vector<unsigned char> attributes(attribute_bytes);
    startup.lpAttributeList = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attributes.data());
    if (::InitializeProcThreadAttributeList(startup.lpAttributeList, 1, 0,
                                            &attribute_bytes) == 0) {
      windows_error("initializing recorder inherited handles");
    }
    HANDLE inherited_handles[]{null_input, stdout_log, stderr_log};
    const auto handle_count = stderr_log == stdout_log ? 2U : 3U;
    if (::UpdateProcThreadAttribute(startup.lpAttributeList, 0,
                                    PROC_THREAD_ATTRIBUTE_HANDLE_LIST, inherited_handles,
                                    sizeof(HANDLE) * handle_count, nullptr, nullptr) == 0) {
      ::DeleteProcThreadAttributeList(startup.lpAttributeList);
      windows_error("selecting recorder inherited handles");
    }
    const auto executable = std::filesystem::path(spec.argv.front()).wstring();
    const auto created = ::CreateProcessW(
        executable.c_str(), command.data(), nullptr, nullptr, TRUE,
        CREATE_NO_WINDOW | CREATE_SUSPENDED | EXTENDED_STARTUPINFO_PRESENT, nullptr,
        spec.working_directory.c_str(), &startup.StartupInfo, &process);
    const auto creation_error = ::GetLastError();
    ::DeleteProcThreadAttributeList(startup.lpAttributeList);
    if (created == 0) windows_error("executing recorder", creation_error);
    if (job != nullptr && ::AssignProcessToJobObject(job, process.hProcess) == 0) {
      windows_error("assigning recorder process job");
    }
    event = create_stop_event(process.dwProcessId, session_id);
    if (::ResumeThread(process.hThread) == static_cast<DWORD>(-1)) {
      windows_error("resuming recorder");
    }
    ::CloseHandle(process.hThread);
    process.hThread = nullptr;
    if (stderr_log == stdout_log) stderr_log = nullptr;
    close_handle(stdout_log);
    close_handle(stderr_log);
    close_handle(null_input);
    return std::make_unique<WindowsRecordingProcess>(process.hProcess, process.dwProcessId,
                                                      event, job, owned);
  } catch (...) {
    if (process.hProcess != nullptr) {
      ::TerminateProcess(process.hProcess, 137);
      ::WaitForSingleObject(process.hProcess, INFINITE);
    }
    close_handle(process.hThread);
    close_handle(process.hProcess);
    close_handle(event);
    close_handle(job);
    if (stderr_log == stdout_log) stderr_log = nullptr;
    close_handle(stdout_log);
    close_handle(stderr_log);
    close_handle(null_input);
    throw;
  }
}

}  // namespace

WindowsRecordingStopEvent::WindowsRecordingStopEvent(const std::string &session_id)
    : event_(create_stop_event(::GetCurrentProcessId(), session_id)) {}

WindowsRecordingStopEvent::~WindowsRecordingStopEvent() { close_handle(event_); }

bool WindowsRecordingStopEvent::stop_requested() const {
  const auto result = ::WaitForSingleObject(event_, 0);
  if (result == WAIT_FAILED) windows_error("reading recorder stop event");
  return result == WAIT_OBJECT_0;
}

std::unique_ptr<RecordingProcess> WindowsRecordingProcessFactory::start(
    const RecordingProcessSpec &spec) {
  return launch(spec, true, {});
}

std::unique_ptr<RecordingProcess> start_windows_recording_manager(
    const std::vector<std::string> &command, const std::filesystem::path &manager_log,
    const std::string &session_id) {
  return launch({"manager", command, manager_log.parent_path(), manager_log, manager_log},
                false, session_id);
}

bool windows_recording_process_alive(std::int64_t process_id) {
  if (process_id <= 0 || process_id > (std::numeric_limits<DWORD>::max)()) return false;
  HANDLE process = ::OpenProcess(SYNCHRONIZE, FALSE, static_cast<DWORD>(process_id));
  if (process == nullptr) {
    if (::GetLastError() == ERROR_INVALID_PARAMETER) return false;
    windows_error("opening recording manager process");
  }
  const auto result = ::WaitForSingleObject(process, 0);
  ::CloseHandle(process);
  if (result == WAIT_FAILED) windows_error("observing recording manager");
  return result == WAIT_TIMEOUT;
}

std::optional<std::string> windows_recording_manager_identity_error(
    std::int64_t process_id, const std::filesystem::path &session_directory) {
  try {
    if (!windows_recording_process_alive(process_id)) {
      return "recording manager process is no longer running";
    }
    HANDLE process = ::OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE,
                                   static_cast<DWORD>(process_id));
    if (process == nullptr) windows_error("opening recording manager image");
    wchar_t executable[32768]{};
    DWORD length = 32768;
    const auto queried = ::QueryFullProcessImageNameW(process, 0, executable, &length);
    ::CloseHandle(process);
    if (queried == 0) windows_error("reading recording manager image");
    wchar_t own_executable[32768]{};
    if (::GetModuleFileNameW(nullptr, own_executable, 32768) == 0) {
      windows_error("reading recorder executable image");
    }
    std::error_code error;
    if (!std::filesystem::equivalent(executable, own_executable, error) || error) {
      return "manifest PID does not identify this lingtu_recorder executable";
    }
    const auto name = stop_event_name(
        process_id, std::filesystem::weakly_canonical(session_directory).generic_string());
    HANDLE event = ::OpenEventW(SYNCHRONIZE, FALSE, name.c_str());
    if (event == nullptr) return "manifest PID belongs to a different recording session";
    ::CloseHandle(event);
  } catch (const std::exception &error) {
    return error.what();
  }
  return std::nullopt;
}

void request_windows_recording_stop(std::int64_t process_id, const std::string &session_id) {
  const auto name = stop_event_name(process_id, session_id);
  HANDLE event = ::OpenEventW(EVENT_MODIFY_STATE, FALSE, name.c_str());
  if (event == nullptr) windows_error("opening recording manager stop event");
  const auto result = ::SetEvent(event);
  const auto error = ::GetLastError();
  ::CloseHandle(event);
  if (result == 0) windows_error("requesting recording manager stop", error);
}

}  // namespace lingtu::recording
