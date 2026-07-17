#include "lingtu/maps/build/process.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>
#include <vector>

#if defined(_WIN32)
#  define NOMINMAX
#  include <windows.h>
#else
#  include <cerrno>
#  include <cstring>
#  include <fcntl.h>
#  include <signal.h>
#  include <sys/types.h>
#  include <sys/wait.h>
#  include <unistd.h>
#endif

namespace lingtu::maps {
namespace {

void AppendBounded(
    std::string& target,
    bool& truncated,
    const char* data,
    std::size_t size,
    std::uint64_t max_output_bytes) {
  if (size == 0U || max_output_bytes == 0U) {
    truncated = truncated || size > 0U;
    return;
  }
  const std::uint64_t remaining =
      target.size() >= max_output_bytes ? 0U : max_output_bytes - target.size();
  if (remaining == 0U) {
    truncated = true;
    return;
  }
  const auto keep = static_cast<std::size_t>(
      std::min<std::uint64_t>(remaining, static_cast<std::uint64_t>(size)));
  target.append(data, keep);
  if (keep < size) {
    truncated = true;
  }
}

#if defined(_WIN32)

std::wstring Utf8ToWide(const std::string& value) {
  if (value.empty()) {
    return {};
  }
  const int size = MultiByteToWideChar(
      CP_UTF8,
      0,
      value.c_str(),
      static_cast<int>(value.size()),
      nullptr,
      0);
  if (size <= 0) {
    return {};
  }
  std::wstring out(static_cast<std::size_t>(size), L'\0');
  MultiByteToWideChar(
      CP_UTF8,
      0,
      value.c_str(),
      static_cast<int>(value.size()),
      out.data(),
      size);
  return out;
}

bool IsValidUtf8(const std::string& value) {
  if (value.empty()) {
    return true;
  }
  return MultiByteToWideChar(
             CP_UTF8,
             MB_ERR_INVALID_CHARS,
             value.data(),
             static_cast<int>(value.size()),
             nullptr,
             0) > 0;
}

std::string CodePageToUtf8(const std::string& value, UINT code_page) {
  if (value.empty()) {
    return {};
  }
  const int wide_size = MultiByteToWideChar(
      code_page,
      0,
      value.data(),
      static_cast<int>(value.size()),
      nullptr,
      0);
  if (wide_size <= 0) {
    return value;
  }
  std::wstring wide(static_cast<std::size_t>(wide_size), L'\0');
  MultiByteToWideChar(
      code_page,
      0,
      value.data(),
      static_cast<int>(value.size()),
      wide.data(),
      wide_size);
  const int utf8_size = WideCharToMultiByte(
      CP_UTF8,
      0,
      wide.data(),
      wide_size,
      nullptr,
      0,
      nullptr,
      nullptr);
  if (utf8_size <= 0) {
    return value;
  }
  std::string utf8(static_cast<std::size_t>(utf8_size), '\0');
  WideCharToMultiByte(
      CP_UTF8,
      0,
      wide.data(),
      wide_size,
      utf8.data(),
      utf8_size,
      nullptr,
      nullptr);
  return utf8;
}

std::string NormalizeProcessOutputEncoding(const std::string& value) {
  return IsValidUtf8(value) ? value : CodePageToUtf8(value, CP_OEMCP);
}

void DrainPipe(
    HANDLE read_pipe,
    std::string& target,
    bool& truncated,
    std::uint64_t max_output_bytes) {
  std::array<char, 4096> buffer{};
  for (;;) {
    DWORD available = 0;
    if (PeekNamedPipe(read_pipe, nullptr, 0, nullptr, &available, nullptr) == 0 ||
        available == 0) {
      break;
    }
    DWORD read_count = 0;
    const DWORD request = std::min<DWORD>(
        static_cast<DWORD>(buffer.size()),
        available);
    if (ReadFile(read_pipe, buffer.data(), request, &read_count, nullptr) == 0 ||
        read_count == 0) {
      break;
    }
    AppendBounded(target, truncated, buffer.data(), read_count, max_output_bytes);
  }
}

ProcessRunResult RunWindowsShellCommand(
    const std::string& command,
    const ProcessRunOptions& options) {
  ProcessRunResult result;

  SECURITY_ATTRIBUTES security{};
  security.nLength = sizeof(security);
  security.bInheritHandle = TRUE;

  HANDLE stdout_read = nullptr;
  HANDLE stdout_write = nullptr;
  HANDLE stderr_read = nullptr;
  HANDLE stderr_write = nullptr;
  if (CreatePipe(&stdout_read, &stdout_write, &security, 0) == 0 ||
      CreatePipe(&stderr_read, &stderr_write, &security, 0) == 0) {
    result.launch_failed = true;
    result.error = "failed to create process pipes";
    if (stdout_read != nullptr) CloseHandle(stdout_read);
    if (stdout_write != nullptr) CloseHandle(stdout_write);
    if (stderr_read != nullptr) CloseHandle(stderr_read);
    if (stderr_write != nullptr) CloseHandle(stderr_write);
    return result;
  }
  SetHandleInformation(stdout_read, HANDLE_FLAG_INHERIT, 0);
  SetHandleInformation(stderr_read, HANDLE_FLAG_INHERIT, 0);

  STARTUPINFOW startup{};
  startup.cb = sizeof(startup);
  startup.dwFlags = STARTF_USESTDHANDLES;
  startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  startup.hStdOutput = stdout_write;
  startup.hStdError = stderr_write;

  PROCESS_INFORMATION process{};
  std::wstring command_line = Utf8ToWide("cmd.exe /S /C " + command);
  std::vector<wchar_t> command_buffer(command_line.begin(), command_line.end());
  command_buffer.push_back(L'\0');
  std::wstring cwd = options.cwd.empty() ? std::wstring{} : options.cwd.wstring();
  HANDLE job = CreateJobObjectW(nullptr, nullptr);
  bool job_assigned = false;
  if (job != nullptr) {
    JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    if (SetInformationJobObject(
            job,
            JobObjectExtendedLimitInformation,
            &limits,
            sizeof(limits)) == 0) {
      CloseHandle(job);
      job = nullptr;
    }
  }
  const BOOL created = CreateProcessW(
      nullptr,
      command_buffer.data(),
      nullptr,
      nullptr,
      TRUE,
      CREATE_NO_WINDOW | CREATE_SUSPENDED,
      nullptr,
      cwd.empty() ? nullptr : cwd.c_str(),
      &startup,
      &process);

  CloseHandle(stdout_write);
  CloseHandle(stderr_write);

  if (created == 0) {
    if (job != nullptr) {
      CloseHandle(job);
    }
    CloseHandle(stdout_read);
    CloseHandle(stderr_read);
    result.launch_failed = true;
    result.error = "CreateProcessW failed";
    return result;
  }

  if (job != nullptr && AssignProcessToJobObject(job, process.hProcess) != 0) {
    job_assigned = true;
  }
  ResumeThread(process.hThread);

  const auto started = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(options.timeout_sec);
  for (;;) {
    DrainPipe(
        stdout_read,
        result.stdout_text,
        result.stdout_truncated,
        options.max_output_bytes);
    DrainPipe(
        stderr_read,
        result.stderr_text,
        result.stderr_truncated,
        options.max_output_bytes);

    const DWORD wait_ms = 10U;
    const DWORD wait = WaitForSingleObject(process.hProcess, wait_ms);
    if (wait == WAIT_OBJECT_0) {
      break;
    }
    if (options.timeout_sec > 0.0 &&
        std::chrono::steady_clock::now() - started >= timeout) {
      result.timed_out = true;
      if (job_assigned) {
        TerminateJobObject(job, 124U);
      } else {
        TerminateProcess(process.hProcess, 124U);
      }
      WaitForSingleObject(process.hProcess, 1000U);
      break;
    }
    if (options.cancel_requested && options.cancel_requested()) {
      result.cancelled = true;
      if (job_assigned) {
        TerminateJobObject(job, 125U);
      } else {
        TerminateProcess(process.hProcess, 125U);
      }
      WaitForSingleObject(process.hProcess, 1000U);
      break;
    }
  }

  DrainPipe(
      stdout_read,
      result.stdout_text,
      result.stdout_truncated,
      options.max_output_bytes);
  DrainPipe(
      stderr_read,
      result.stderr_text,
      result.stderr_truncated,
      options.max_output_bytes);

  DWORD exit_code = 1U;
  GetExitCodeProcess(process.hProcess, &exit_code);
  result.exit_code = result.cancelled ? 125 :
      (result.timed_out ? 124 : static_cast<int>(exit_code));
  CloseHandle(process.hThread);
  CloseHandle(process.hProcess);
  if (job != nullptr) {
    CloseHandle(job);
  }
  CloseHandle(stdout_read);
  CloseHandle(stderr_read);
  result.stdout_text = NormalizeProcessOutputEncoding(result.stdout_text);
  result.stderr_text = NormalizeProcessOutputEncoding(result.stderr_text);
  return result;
}

#else

bool SetNonBlocking(int fd) {
  const int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

void DrainFd(
    int fd,
    std::string& target,
    bool& truncated,
    std::uint64_t max_output_bytes) {
  std::array<char, 4096> buffer{};
  for (;;) {
    const ssize_t count = read(fd, buffer.data(), buffer.size());
    if (count > 0) {
      AppendBounded(
          target,
          truncated,
          buffer.data(),
          static_cast<std::size_t>(count),
          max_output_bytes);
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    break;
  }
}

ProcessRunResult RunPosixShellCommand(
    const std::string& command,
    const ProcessRunOptions& options) {
  ProcessRunResult result;
  int stdout_pipe[2] = {-1, -1};
  int stderr_pipe[2] = {-1, -1};
  if (pipe(stdout_pipe) != 0 || pipe(stderr_pipe) != 0) {
    result.launch_failed = true;
    result.error = std::strerror(errno);
    if (stdout_pipe[0] >= 0) close(stdout_pipe[0]);
    if (stdout_pipe[1] >= 0) close(stdout_pipe[1]);
    if (stderr_pipe[0] >= 0) close(stderr_pipe[0]);
    if (stderr_pipe[1] >= 0) close(stderr_pipe[1]);
    return result;
  }

  const pid_t pid = fork();
  if (pid < 0) {
    result.launch_failed = true;
    result.error = std::strerror(errno);
    close(stdout_pipe[0]);
    close(stdout_pipe[1]);
    close(stderr_pipe[0]);
    close(stderr_pipe[1]);
    return result;
  }

  if (pid == 0) {
    setpgid(0, 0);
    close(stdout_pipe[0]);
    close(stderr_pipe[0]);
    dup2(stdout_pipe[1], STDOUT_FILENO);
    dup2(stderr_pipe[1], STDERR_FILENO);
    close(stdout_pipe[1]);
    close(stderr_pipe[1]);
    if (!options.cwd.empty()) {
      chdir(options.cwd.string().c_str());
    }
    execl("/bin/sh", "sh", "-c", command.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }

  close(stdout_pipe[1]);
  close(stderr_pipe[1]);
  setpgid(pid, pid);
  SetNonBlocking(stdout_pipe[0]);
  SetNonBlocking(stderr_pipe[0]);

  const auto started = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(options.timeout_sec);
  int status = 0;
  bool exited = false;
  for (;;) {
    DrainFd(
        stdout_pipe[0],
        result.stdout_text,
        result.stdout_truncated,
        options.max_output_bytes);
    DrainFd(
        stderr_pipe[0],
        result.stderr_text,
        result.stderr_truncated,
        options.max_output_bytes);

    const pid_t wait_result = waitpid(pid, &status, WNOHANG);
    if (wait_result == pid) {
      exited = true;
      break;
    }
    if (options.timeout_sec > 0.0 &&
        std::chrono::steady_clock::now() - started >= timeout) {
      result.timed_out = true;
      if (kill(-pid, SIGKILL) != 0) {
        kill(pid, SIGKILL);
      }
      waitpid(pid, &status, 0);
      exited = true;
      break;
    }
    if (options.cancel_requested && options.cancel_requested()) {
      result.cancelled = true;
      if (kill(-pid, SIGKILL) != 0) {
        kill(pid, SIGKILL);
      }
      waitpid(pid, &status, 0);
      exited = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  DrainFd(
      stdout_pipe[0],
      result.stdout_text,
      result.stdout_truncated,
      options.max_output_bytes);
  DrainFd(
      stderr_pipe[0],
      result.stderr_text,
      result.stderr_truncated,
      options.max_output_bytes);
  close(stdout_pipe[0]);
  close(stderr_pipe[0]);

  if (result.cancelled) {
    result.exit_code = 125;
  } else if (result.timed_out) {
    result.exit_code = 124;
  } else if (exited && WIFEXITED(status)) {
    result.exit_code = WEXITSTATUS(status);
  } else if (exited && WIFSIGNALED(status)) {
    result.exit_code = 128 + WTERMSIG(status);
  } else {
    result.exit_code = -1;
  }
  return result;
}

#endif

}  // namespace

ProcessRunResult RunShellCommand(
    const std::string& command,
    const ProcessRunOptions& options) {
#if defined(_WIN32)
  return RunWindowsShellCommand(command, options);
#else
  return RunPosixShellCommand(command, options);
#endif
}

}  // namespace lingtu::maps
