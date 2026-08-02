#include "lingtu/recording/recording_linux.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace lingtu::recording {
namespace {

[[noreturn]] void throw_system_error(const std::string &operation, int error_number = errno) {
  throw std::runtime_error(operation + " failed: " + std::strerror(error_number));
}

void close_fd(int &fd) noexcept {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

void validate_process_spec(const RecordingProcessSpec &spec) {
  if (spec.name.empty() || spec.argv.empty() || spec.argv.front().empty()) {
    throw std::invalid_argument("recording process name and executable must not be empty");
  }
  if (spec.working_directory.empty() || !std::filesystem::is_directory(spec.working_directory)) {
    throw std::invalid_argument("recording process working directory must exist");
  }
  if (spec.stdout_log.empty() || spec.stderr_log.empty() || spec.stdout_log == spec.stderr_log) {
    throw std::invalid_argument("recording process requires distinct stdout and stderr logs");
  }
  for (const auto &argument : spec.argv) {
    if (argument.find('\0') != std::string::npos) {
      throw std::invalid_argument("recording process argv contains a NUL byte");
    }
  }
}

int open_log(const std::filesystem::path &path) {
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0640);
  if (fd < 0) {
    throw_system_error("opening recorder log " + path.string());
  }
  return fd;
}

int exit_code_from_status(int status) noexcept {
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  if (WIFSIGNALED(status)) {
    return 128 + WTERMSIG(status);
  }
  return 255;
}

void signal_process_group(pid_t process_id, int signal_number) {
  if (::kill(-process_id, signal_number) == 0) {
    return;
  }
  const int group_error = errno;
  if (group_error == ESRCH) {
    return;
  }
  throw_system_error("signalling recorder process group", group_error);
}

void signal_process(pid_t process_id, int signal_number) {
  if (::kill(process_id, signal_number) == 0 || errno == ESRCH) {
    return;
  }
  throw_system_error("signalling recorder process", errno);
}

class PosixRecordingProcess final : public RecordingProcess {
 public:
  explicit PosixRecordingProcess(pid_t process_id) : process_id_(process_id) {}

  ~PosixRecordingProcess() override {
    try {
      if (snapshot().running) {
        request_stop();
        static_cast<void>(wait(std::chrono::seconds(2)));
      }
    } catch (...) {}
  }

  RecordingProcessSnapshot snapshot() override {
    reap(false);
    return {static_cast<std::int64_t>(process_id_), !exit_code_.has_value(), exit_code_};
  }

  void request_stop() override {
    reap(false);
    if (!exit_code_) {
      signal_process(process_id_, SIGTERM);
    }
  }

  int wait(std::chrono::milliseconds timeout) override {
    if (timeout.count() < 0) {
      throw std::invalid_argument("recording process wait timeout must not be negative");
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!exit_code_) {
      reap(false);
      if (exit_code_) {
        break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        signal_process_group(process_id_, SIGKILL);
        reap(true);
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return *exit_code_;
  }

 private:
  void reap(bool blocking) {
    if (exit_code_) {
      return;
    }
    int status = 0;
    pid_t result;
    do {
      result = ::waitpid(process_id_, &status, blocking ? 0 : WNOHANG);
    } while (result < 0 && errno == EINTR);
    if (result == 0) {
      return;
    }
    if (result < 0) {
      throw_system_error("waiting for recorder process");
    }
    exit_code_ = exit_code_from_status(status);
  }

  pid_t process_id_;
  std::optional<int> exit_code_;
};

enum class ExecStage : std::uint8_t {
  kProcessGroup = 1,
  kWorkingDirectory = 2,
  kStdout = 3,
  kStderr = 4,
  kExecute = 5,
};

struct ExecFailure {
  ExecStage stage;
  int error_number;
};

const char *exec_stage_name(ExecStage stage) noexcept {
  switch (stage) {
    case ExecStage::kProcessGroup:
      return "creating recorder process group";
    case ExecStage::kWorkingDirectory:
      return "entering recorder working directory";
    case ExecStage::kStdout:
      return "redirecting recorder stdout";
    case ExecStage::kStderr:
      return "redirecting recorder stderr";
    case ExecStage::kExecute:
      return "executing recorder";
  }
  return "starting recorder";
}

[[noreturn]] void child_failure(int error_fd, ExecStage stage) noexcept {
  const ExecFailure failure{stage, errno};
  while (::write(error_fd, &failure, sizeof(failure)) < 0 && errno == EINTR) {}
  _exit(127);
}

}  // namespace

std::unique_ptr<RecordingProcess>
PosixRecordingProcessFactory::start(const RecordingProcessSpec &spec) {
  validate_process_spec(spec);

  int stdout_fd = -1;
  int stderr_fd = -1;
  int error_pipe[2]{-1, -1};
  try {
    stdout_fd = open_log(spec.stdout_log);
    stderr_fd = open_log(spec.stderr_log);
    if (::pipe2(error_pipe, O_CLOEXEC) != 0) {
      throw_system_error("creating recorder exec-status pipe");
    }

    std::vector<char *> argv;
    argv.reserve(spec.argv.size() + 1);
    for (const auto &argument : spec.argv) {
      argv.push_back(const_cast<char *>(argument.c_str()));
    }
    argv.push_back(nullptr);

    const pid_t child = ::fork();
    if (child < 0) {
      throw_system_error("forking recorder process");
    }
    if (child == 0) {
      close_fd(error_pipe[0]);
      if (::setpgid(0, 0) != 0) {
        child_failure(error_pipe[1], ExecStage::kProcessGroup);
      }
      if (::chdir(spec.working_directory.c_str()) != 0) {
        child_failure(error_pipe[1], ExecStage::kWorkingDirectory);
      }
      if (::dup2(stdout_fd, STDOUT_FILENO) < 0) {
        child_failure(error_pipe[1], ExecStage::kStdout);
      }
      if (::dup2(stderr_fd, STDERR_FILENO) < 0) {
        child_failure(error_pipe[1], ExecStage::kStderr);
      }
      close_fd(stdout_fd);
      close_fd(stderr_fd);
      ::execvp(argv.front(), argv.data());
      child_failure(error_pipe[1], ExecStage::kExecute);
    }

    close_fd(stdout_fd);
    close_fd(stderr_fd);
    close_fd(error_pipe[1]);
    if (::setpgid(child, child) != 0 && errno != EACCES && errno != ESRCH) {
      const int group_error = errno;
      ::kill(child, SIGKILL);
      int status = 0;
      while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
      close_fd(error_pipe[0]);
      throw_system_error("creating recorder process group", group_error);
    }

    ExecFailure failure{};
    ssize_t bytes;
    do {
      bytes = ::read(error_pipe[0], &failure, sizeof(failure));
    } while (bytes < 0 && errno == EINTR);
    const int read_error = errno;
    close_fd(error_pipe[0]);
    if (bytes == 0) {
      return std::make_unique<PosixRecordingProcess>(child);
    }

    int status = 0;
    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
    if (bytes == static_cast<ssize_t>(sizeof(failure))) {
      throw_system_error(exec_stage_name(failure.stage), failure.error_number);
    }
    throw_system_error("reading recorder exec status", bytes < 0 ? read_error : EIO);
  } catch (...) {
    close_fd(stdout_fd);
    close_fd(stderr_fd);
    close_fd(error_pipe[0]);
    close_fd(error_pipe[1]);
    throw;
  }
}

}  // namespace lingtu::recording
