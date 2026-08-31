#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <locale>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "dds/dds.h"
#include "messages.h"
#include "message/cpp/qos.hpp"
#include "message/cpp/topics.hpp"
#include "test_dds_domain.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <csignal>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;

constexpr char kBridgeBootId[] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
constexpr char kControllerBootId[] = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
constexpr char kHostBootId[] = "test-host-boot";
constexpr char kProducerBootId[] = "test-navd-boot";
constexpr std::uint64_t kOutputSequence = 77;
constexpr double kInputVx = 2.0;
constexpr double kInputVy = -0.5;
constexpr double kInputWz = 4.0;
constexpr double kExpectedWalkX = 1.0;
constexpr double kExpectedWalkY = -0.5;
constexpr double kExpectedWalkZ = 1.0;

void require(bool value, const std::string &message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

dds_entity_t checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

std::uint64_t platform_boot_time_ns() {
#ifdef _WIN32
  LARGE_INTEGER counter{};
  LARGE_INTEGER frequency{};
  require(QueryPerformanceCounter(&counter) != 0 && QueryPerformanceFrequency(&frequency) != 0 &&
              counter.QuadPart > 0 && frequency.QuadPart > 0,
          "QueryPerformanceCounter failed");
  const auto ticks = static_cast<std::uint64_t>(counter.QuadPart);
  const auto ticks_per_second = static_cast<std::uint64_t>(frequency.QuadPart);
  return ticks / ticks_per_second * 1'000'000'000ULL +
         ticks % ticks_per_second * 1'000'000'000ULL / ticks_per_second;
#elif defined(__linux__)
  timespec value{};
  require(clock_gettime(CLOCK_BOOTTIME, &value) == 0 && value.tv_sec >= 0 && value.tv_nsec >= 0,
          "clock_gettime(CLOCK_BOOTTIME) failed");
  return static_cast<std::uint64_t>(value.tv_sec) * 1'000'000'000ULL +
         static_cast<std::uint64_t>(value.tv_nsec);
#else
  const auto value =
      std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
  require(value > 0, "steady clock must be positive");
  return static_cast<std::uint64_t>(value);
#endif
}

int test_domain_id() {
  return lingtu::sim::dds_adapter::test::domain_id_from_environment();
}

bool close_enough(double left, double right) {
  return std::abs(left - right) < 1e-9;
}

Deadline bounded_deadline(Deadline overall, Clock::duration budget) {
  return std::min(overall, Clock::now() + budget);
}

dds_duration_t remaining_dds_time(Deadline deadline) {
  const auto now = Clock::now();
  if (now >= deadline) {
    return 0;
  }
  const auto remaining =
      std::chrono::duration_cast<std::chrono::nanoseconds>(deadline - now).count();
  return static_cast<dds_duration_t>(std::max<std::int64_t>(1, remaining));
}

std::vector<std::string> split_tabs(const std::string &line) {
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (true) {
    const auto tab = line.find('\t', begin);
    fields.emplace_back(
        line.substr(begin, tab == std::string::npos ? std::string::npos : tab - begin));
    if (tab == std::string::npos) {
      return fields;
    }
    begin = tab + 1;
  }
}

std::uint64_t parse_u64(const std::string &value, const char *field) {
  if (value.empty() ||
      !std::all_of(value.begin(), value.end(), [](char ch) { return ch >= '0' && ch <= '9'; })) {
    throw std::runtime_error(std::string(field) + " is not a decimal uint64: " + value);
  }
  std::size_t consumed = 0;
  std::uint64_t parsed = 0;
  try {
    parsed = std::stoull(value, &consumed, 10);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string(field) + " overflows uint64: " + value);
  }
  if (consumed != value.size()) {
    throw std::runtime_error(std::string(field) + " has trailing bytes: " + value);
  }
  return parsed;
}

double parse_finite_double(const std::string &value, const char *field) {
  std::istringstream input(value);
  input.imbue(std::locale::classic());
  double parsed = 0.0;
  input >> parsed;
  if (!input || input.peek() != std::char_traits<char>::eof() || !std::isfinite(parsed)) {
    throw std::runtime_error(std::string(field) + " is not a finite double: " + value);
  }
  return parsed;
}

std::string format_double(double value) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output.precision(std::numeric_limits<double>::max_digits10);
  output << value;
  return output.str();
}

struct ParsedBridgeCommand {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t bridge_command_seq{0};
  std::string kind;
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
  double walk_x{0.0};
  double walk_y{0.0};
  double walk_z{0.0};
};

ParsedBridgeCommand parse_bridge_command(const std::string &line) {
  const auto fields = split_tabs(line);
  require(fields.size() == 10 && fields[0] == "LT_DRIVER_COMMAND_V2",
          "malformed LT_DRIVER_COMMAND_V2: " + line);
  ParsedBridgeCommand command;
  command.bridge_boot_id = fields[1];
  command.controller_boot_id = fields[2];
  command.bridge_command_seq = parse_u64(fields[3], "bridge_command_seq");
  command.kind = fields[4];
  command.producer_boot_id = fields[5] == "-" ? std::string{} : fields[5];
  command.output_sequence = parse_u64(fields[6], "output_sequence");
  command.walk_x = parse_finite_double(fields[7], "walk_x");
  command.walk_y = parse_finite_double(fields[8], "walk_y");
  command.walk_z = parse_finite_double(fields[9], "walk_z");
  require(command.bridge_command_seq > 0, "bridge command sequence must be positive");
  return command;
}

struct ParsedReady {
  std::string bridge_boot_id;
  std::string controller_boot_id;
  std::uint64_t accepted_sequence{0};
  std::string producer_boot_id;
  std::uint64_t output_sequence{0};
};

ParsedReady parse_ready(const std::string &line) {
  const auto fields = split_tabs(line);
  require(fields.size() == 6 && fields[0] == "LT_DRIVER_READY_V2",
          "malformed LT_DRIVER_READY_V2: " + line);
  return {
      fields[1],
      fields[2],
      parse_u64(fields[3], "accepted_sequence"),
      fields[4] == "-" ? std::string{} : fields[4],
      parse_u64(fields[5], "accepted_output_sequence"),
  };
}

class LineCollector {
 public:
  void add_bytes(const char *data, std::size_t size) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < size; ++i) {
      const char ch = data[i];
      if (ch == '\n') {
        lines_.push_back(current_);
        current_.clear();
        cv_.notify_all();
      } else if (ch != '\r') {
        current_.push_back(ch);
      }
    }
  }

  void mark_closed() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_.empty()) {
      lines_.push_back(current_);
      current_.clear();
    }
    closed_ = true;
    cv_.notify_all();
  }

  std::optional<std::string> wait_for_next(Deadline deadline) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (cursor_ >= lines_.size() && !closed_) {
      if (cv_.wait_until(lock, deadline) == std::cv_status::timeout && cursor_ >= lines_.size()) {
        return std::nullopt;
      }
    }
    if (cursor_ >= lines_.size()) {
      return std::nullopt;
    }
    return lines_[cursor_++];
  }

  std::string joined_lines() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string joined;
    for (const auto &line : lines_) {
      joined += line;
      joined += '\n';
    }
    joined += current_;
    return joined;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::string> lines_;
  std::string current_;
  std::size_t cursor_{0};
  bool closed_{false};
};

class BridgeProcess {
 public:
  BridgeProcess(const std::filesystem::path &binary, const std::filesystem::path &ready_file,
                int domain_id) {
    start(binary, ready_file, domain_id);
  }

  BridgeProcess(const BridgeProcess &) = delete;
  BridgeProcess &operator=(const BridgeProcess &) = delete;

  ~BridgeProcess() { shutdown_noexcept(); }

  void write_line(const std::string &line) {
    const std::string framed = line + "\n";
#ifdef _WIN32
    if (stdin_write_ == nullptr) {
      throw std::runtime_error("bridge stdin is closed");
    }
    std::size_t offset = 0;
    while (offset < framed.size()) {
      const DWORD request = static_cast<DWORD>(std::min<std::size_t>(
          framed.size() - offset, static_cast<std::size_t>(std::numeric_limits<DWORD>::max())));
      DWORD written = 0;
      if (!WriteFile(stdin_write_, framed.data() + offset, request, &written, nullptr) ||
          written == 0) {
        throw std::runtime_error("failed to write bridge stdin");
      }
      offset += written;
    }
#else
    if (stdin_write_ < 0) {
      throw std::runtime_error("bridge stdin is closed");
    }
    std::size_t offset = 0;
    while (offset < framed.size()) {
      const ssize_t written = write(stdin_write_, framed.data() + offset, framed.size() - offset);
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written <= 0) {
        throw std::runtime_error("failed to write bridge stdin");
      }
      offset += static_cast<std::size_t>(written);
    }
#endif
  }

  void close_stdin() noexcept {
#ifdef _WIN32
    if (stdin_write_ != nullptr) {
      CloseHandle(stdin_write_);
      stdin_write_ = nullptr;
    }
#else
    if (stdin_write_ >= 0) {
      close(stdin_write_);
      stdin_write_ = -1;
    }
#endif
  }

  void request_graceful_stop() {
#ifdef _WIN32
    require(process_id_ != 0, "bridge process id is unavailable");
    require(GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, process_id_) != 0,
            "failed to deliver CTRL_BREAK to bridge process group");
#else
    require(pid_ > 0, "bridge pid is unavailable");
    require(kill(pid_, SIGTERM) == 0, "failed to deliver SIGTERM to bridge process");
#endif
  }

  std::optional<int> wait_until(Deadline deadline) {
    {
      std::unique_lock<std::mutex> lock(exit_mutex_);
      if (!exit_cv_.wait_until(lock, deadline, [this]() { return exit_code_.has_value(); })) {
        return std::nullopt;
      }
    }
    join_threads();
    std::lock_guard<std::mutex> lock(exit_mutex_);
    return exit_code_;
  }

  bool exited() const noexcept { return exited_.load(std::memory_order_acquire); }

  LineCollector &lines() noexcept { return lines_; }

  std::string stdout_text() const { return lines_.joined_lines(); }

 private:
#ifdef _WIN32
  static std::wstring quote(const std::wstring &value) { return L"\"" + value + L"\""; }

  static std::wstring widen(const std::string &value) {
    return std::wstring(value.begin(), value.end());
  }
#endif

  void start(const std::filesystem::path &binary, const std::filesystem::path &ready_file,
             int domain_id) {
#ifdef _WIN32
    SECURITY_ATTRIBUTES security{};
    security.nLength = sizeof(security);
    security.bInheritHandle = TRUE;

    HANDLE stdin_read = nullptr;
    HANDLE stdout_write = nullptr;
    if (!CreatePipe(&stdin_read, &stdin_write_, &security, 0)) {
      throw std::runtime_error("failed to create bridge stdin pipe");
    }
    if (!CreatePipe(&stdout_read_, &stdout_write, &security, 0)) {
      CloseHandle(stdin_read);
      close_stdin();
      throw std::runtime_error("failed to create bridge stdout pipe");
    }
    if (!SetHandleInformation(stdin_write_, HANDLE_FLAG_INHERIT, 0) ||
        !SetHandleInformation(stdout_read_, HANDLE_FLAG_INHERIT, 0)) {
      CloseHandle(stdin_read);
      CloseHandle(stdout_write);
      CloseHandle(stdout_read_);
      stdout_read_ = nullptr;
      close_stdin();
      throw std::runtime_error("failed to configure bridge pipe inheritance");
    }

    std::wstring command = quote(binary.wstring()) + L" --domain-id " + std::to_wstring(domain_id) +
                           L" --ready-file " + quote(ready_file.wstring()) + L" --bridge-boot-id " +
                           widen(kBridgeBootId) + L" --expected-host-boot-id " +
                           widen(kHostBootId) + L" --max-linear-mps 1 --max-angular-rps 2 " +
                           L"--command-timeout-ms 1000 --heartbeat-timeout-ms 1000 " +
                           L"--apply-timeout-ms 500";
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    startup.dwFlags = STARTF_USESTDHANDLES;
    startup.hStdInput = stdin_read;
    startup.hStdOutput = stdout_write;
    startup.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    PROCESS_INFORMATION process{};
    std::vector<wchar_t> mutable_command(command.begin(), command.end());
    mutable_command.push_back(L'\0');
    const BOOL launched =
        CreateProcessW(nullptr, mutable_command.data(), nullptr, nullptr, TRUE,
                       CREATE_NEW_PROCESS_GROUP, nullptr, nullptr, &startup, &process);
    CloseHandle(stdin_read);
    CloseHandle(stdout_write);
    if (!launched) {
      CloseHandle(stdout_read_);
      stdout_read_ = nullptr;
      close_stdin();
      throw std::runtime_error("failed to launch MuJoCo driver bridge process");
    }
    process_ = process.hProcess;
    process_thread_ = process.hThread;
    process_id_ = process.dwProcessId;
#else
    int stdin_pipe[2]{};
    int stdout_pipe[2]{};
    if (pipe(stdin_pipe) != 0) {
      throw std::runtime_error("failed to create bridge stdin pipe");
    }
    if (pipe(stdout_pipe) != 0) {
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
      throw std::runtime_error("failed to create bridge stdout pipe");
    }
    pid_ = fork();
    if (pid_ < 0) {
      close(stdin_pipe[0]);
      close(stdin_pipe[1]);
      close(stdout_pipe[0]);
      close(stdout_pipe[1]);
      throw std::runtime_error("failed to fork MuJoCo driver bridge process");
    }
    if (pid_ == 0) {
      close(stdin_pipe[1]);
      close(stdout_pipe[0]);
      if (dup2(stdin_pipe[0], STDIN_FILENO) < 0 || dup2(stdout_pipe[1], STDOUT_FILENO) < 0) {
        _exit(126);
      }
      close(stdin_pipe[0]);
      close(stdout_pipe[1]);
      const std::string domain = std::to_string(domain_id);
      execl(binary.c_str(), binary.c_str(), "--domain-id", domain.c_str(), "--ready-file",
            ready_file.c_str(), "--bridge-boot-id", kBridgeBootId, "--expected-host-boot-id",
            kHostBootId, "--max-linear-mps", "1", "--max-angular-rps", "2", "--command-timeout-ms",
            "1000", "--heartbeat-timeout-ms", "1000", "--apply-timeout-ms", "500",
            static_cast<char *>(nullptr));
      _exit(127);
    }
    close(stdin_pipe[0]);
    close(stdout_pipe[1]);
    stdin_write_ = stdin_pipe[1];
    stdout_read_ = stdout_pipe[0];
#endif
    reader_thread_ = std::thread([this]() { read_stdout(); });
    waiter_thread_ = std::thread([this]() { wait_for_child(); });
  }

  void read_stdout() noexcept {
    char buffer[512];
    while (true) {
#ifdef _WIN32
      DWORD read_count = 0;
      if (stdout_read_ == nullptr ||
          !ReadFile(stdout_read_, buffer, static_cast<DWORD>(sizeof(buffer)), &read_count,
                    nullptr) ||
          read_count == 0) {
        break;
      }
      lines_.add_bytes(buffer, static_cast<std::size_t>(read_count));
#else
      if (stdout_read_ < 0) {
        break;
      }
      const ssize_t read_count = read(stdout_read_, buffer, sizeof(buffer));
      if (read_count < 0 && errno == EINTR) {
        continue;
      }
      if (read_count <= 0) {
        break;
      }
      lines_.add_bytes(buffer, static_cast<std::size_t>(read_count));
#endif
    }
    lines_.mark_closed();
  }

  void wait_for_child() noexcept {
    int code = -1;
#ifdef _WIN32
    if (process_ != nullptr && WaitForSingleObject(process_, INFINITE) == WAIT_OBJECT_0) {
      DWORD child_code = 0;
      if (GetExitCodeProcess(process_, &child_code)) {
        code = static_cast<int>(child_code);
      }
    }
#else
    int status = 0;
    pid_t result = -1;
    do {
      result = waitpid(pid_, &status, 0);
    } while (result < 0 && errno == EINTR);
    if (result == pid_) {
      if (WIFEXITED(status)) {
        code = WEXITSTATUS(status);
      } else if (WIFSIGNALED(status)) {
        code = 128 + WTERMSIG(status);
      }
    }
#endif
    exited_.store(true, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lock(exit_mutex_);
      exit_code_ = code;
    }
    exit_cv_.notify_all();
  }

  void terminate_noexcept(bool force) noexcept {
    if (exited()) {
      return;
    }
#ifdef _WIN32
    if (process_ != nullptr) {
      TerminateProcess(process_, force ? 137U : 1U);
    }
#else
    if (pid_ > 0) {
      kill(pid_, force ? SIGKILL : SIGTERM);
    }
#endif
  }

  void close_native_handles() noexcept {
#ifdef _WIN32
    if (stdout_read_ != nullptr) {
      CloseHandle(stdout_read_);
      stdout_read_ = nullptr;
    }
    if (process_thread_ != nullptr) {
      CloseHandle(process_thread_);
      process_thread_ = nullptr;
    }
    if (process_ != nullptr) {
      CloseHandle(process_);
      process_ = nullptr;
    }
#else
    if (stdout_read_ >= 0) {
      close(stdout_read_);
      stdout_read_ = -1;
    }
#endif
  }

  void join_threads() noexcept {
    if (waiter_thread_.joinable()) {
      waiter_thread_.join();
    }
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    close_native_handles();
  }

  void shutdown_noexcept() noexcept {
    close_stdin();
    if (!exited()) {
      terminate_noexcept(false);
      if (!wait_until(Clock::now() + std::chrono::seconds(2)).has_value()) {
        terminate_noexcept(true);
        (void)wait_until(Clock::now() + std::chrono::seconds(2));
      }
    }
    if (exited()) {
      join_threads();
    } else {
      close_native_handles();
      if (waiter_thread_.joinable()) {
        waiter_thread_.detach();
      }
      if (reader_thread_.joinable()) {
        reader_thread_.detach();
      }
    }
  }

  LineCollector lines_;
  std::thread reader_thread_;
  std::thread waiter_thread_;
  std::atomic<bool> exited_{false};
  mutable std::mutex exit_mutex_;
  std::condition_variable exit_cv_;
  std::optional<int> exit_code_;
#ifdef _WIN32
  HANDLE process_{nullptr};
  HANDLE process_thread_{nullptr};
  HANDLE stdin_write_{nullptr};
  HANDLE stdout_read_{nullptr};
  DWORD process_id_{0};
#else
  pid_t pid_{-1};
  int stdin_write_{-1};
  int stdout_read_{-1};
#endif
};

std::string wait_for_protocol_line(BridgeProcess &process, std::string_view expected_name,
                                   Deadline deadline) {
  while (Clock::now() < deadline) {
    const auto line = process.lines().wait_for_next(deadline);
    if (!line.has_value()) {
      break;
    }
    const auto fields = split_tabs(*line);
    const std::string &name = fields.front();
    if (name == "LT_DRIVER_FAULT_V2") {
      throw std::runtime_error("bridge emitted FAULT while waiting for " +
                               std::string(expected_name) + ": " + *line);
    }
    if (name == expected_name) {
      return *line;
    }
  }
  throw std::runtime_error("timed out waiting for " + std::string(expected_name) +
                           "; bridge stdout:\n" + process.stdout_text());
}

ParsedBridgeCommand wait_for_command(BridgeProcess &process, std::string_view expected_kind,
                                     Deadline deadline) {
  const auto line = wait_for_protocol_line(process, "LT_DRIVER_COMMAND_V2", deadline);
  auto command = parse_bridge_command(line);
  require(command.kind == expected_kind, "expected bridge command kind " +
                                             std::string(expected_kind) + ", got " + command.kind +
                                             "; line=" + line);
  return command;
}

void check_command_identity(const ParsedBridgeCommand &command) {
  require(command.bridge_boot_id == kBridgeBootId, "bridge command boot id mismatch");
  require(command.controller_boot_id == kControllerBootId,
          "bridge command controller boot id mismatch");
}

std::string applied_line(const ParsedBridgeCommand &command, std::uint64_t applied_step_seq) {
  return "LT_DRIVER_APPLIED_V2\t" + command.bridge_boot_id + "\t" + command.controller_boot_id +
         "\t" + std::to_string(command.bridge_command_seq) + "\t" + command.kind + "\t" +
         (command.producer_boot_id.empty() ? std::string("-") : command.producer_boot_id) + "\t" +
         std::to_string(command.output_sequence) + "\t" + format_double(command.walk_x) + "\t" +
         format_double(command.walk_y) + "\t" + format_double(command.walk_z) + "\t" +
         std::to_string(applied_step_seq);
}

void wait_for_ready_file(BridgeProcess &process, const std::filesystem::path &path,
                         Deadline deadline) {
  std::mutex poll_mutex;
  std::condition_variable poll_cv;
  std::unique_lock<std::mutex> lock(poll_mutex);
  while (Clock::now() < deadline) {
    std::error_code ec;
    if (std::filesystem::exists(path, ec)) {
      std::ifstream input(path);
      std::string marker;
      std::getline(input, marker);
      if (input.is_open() && marker == "ready") {
        return;
      }
    }
    if (process.exited()) {
      throw std::runtime_error("bridge exited before writing ready marker; stdout:\n" +
                               process.stdout_text());
    }
    poll_cv.wait_until(lock, std::min(deadline, Clock::now() + std::chrono::milliseconds(5)));
  }
  throw std::runtime_error("bridge did not write ready marker: " + path.string());
}

bool writer_has_match(dds_entity_t writer) {
  const dds_return_t count = dds_get_matched_subscriptions(writer, nullptr, 0);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_get_matched_subscriptions: ") +
                             dds_strretcode(-count));
  }
  return count > 0;
}

bool reader_has_match(dds_entity_t reader) {
  const dds_return_t count = dds_get_matched_publications(reader, nullptr, 0);
  if (count < 0) {
    throw std::runtime_error(std::string("dds_get_matched_publications: ") +
                             dds_strretcode(-count));
  }
  return count > 0;
}

void wait_for_dds_match(dds_entity_t participant, dds_entity_t endpoint, bool endpoint_is_writer,
                        Deadline deadline) {
  if (endpoint_is_writer ? writer_has_match(endpoint) : reader_has_match(endpoint)) {
    return;
  }
  checked(dds_set_status_mask(endpoint, endpoint_is_writer ? DDS_PUBLICATION_MATCHED_STATUS
                                                           : DDS_SUBSCRIPTION_MATCHED_STATUS),
          "dds_set_status_mask(match)");
  const dds_entity_t waitset = checked(dds_create_waitset(participant), "dds_create_waitset");
  try {
    checked(dds_waitset_attach(waitset, endpoint, static_cast<dds_attach_t>(1)),
            "dds_waitset_attach(match)");
    while (Clock::now() < deadline) {
      if (endpoint_is_writer ? writer_has_match(endpoint) : reader_has_match(endpoint)) {
        dds_delete(waitset);
        return;
      }
      dds_attach_t triggered{};
      const dds_return_t count =
          dds_waitset_wait(waitset, &triggered, 1, remaining_dds_time(deadline));
      if (count < 0) {
        throw std::runtime_error(std::string("dds_waitset_wait(match): ") + dds_strretcode(-count));
      }
      if (count == 0) {
        break;
      }
    }
  } catch (...) {
    dds_delete(waitset);
    throw;
  }
  dds_delete(waitset);
  throw std::runtime_error(endpoint_is_writer
                               ? "FinalVelocityCommand writer did not discover bridge reader"
                               : "DriverControlState reader did not discover bridge writer");
}

struct ObservedControlState {
  bool connected{false};
  bool ready{false};
  bool motors_enabled{false};
  bool critical_fault{false};
  bool lease_valid{false};
  std::uint64_t accepted_sequence{0};
  std::string accepted_producer_boot_id;
  std::uint64_t accepted_output_sequence{0};
  bool last_command_accepted{false};
  std::string fsm;
  std::string owner;
  std::string owner_id;
  std::string reason;
};

std::string nullable_string(const char *value) {
  return value == nullptr ? std::string{} : std::string(value);
}

class ControlStateStream {
 public:
  ControlStateStream(dds_entity_t participant, dds_entity_t reader) : reader_(reader) {
    condition_ = checked(dds_create_readcondition(reader_, DDS_ANY_STATE),
                         "dds_create_readcondition(control_state)");
    waitset_ = checked(dds_create_waitset(participant), "dds_create_waitset(control_state)");
    checked(dds_waitset_attach(waitset_, condition_, static_cast<dds_attach_t>(1)),
            "dds_waitset_attach(control_state)");
  }

  ControlStateStream(const ControlStateStream &) = delete;
  ControlStateStream &operator=(const ControlStateStream &) = delete;

  ~ControlStateStream() = default;

  std::vector<ObservedControlState> drain() {
    std::vector<ObservedControlState> observed;
    while (true) {
      void *samples[32]{};
      dds_sample_info_t infos[32]{};
      const dds_return_t count = dds_take(reader_, samples, infos, 32, 32);
      if (count < 0) {
        throw std::runtime_error(std::string("dds_take(driver_control_state): ") +
                                 dds_strretcode(-count));
      }
      for (dds_return_t index = 0; index < count; ++index) {
        if (!infos[index].valid_data || samples[index] == nullptr) {
          continue;
        }
        const auto *message = static_cast<const lingtu_dds_DriverControlState *>(samples[index]);
        observed.push_back({
            static_cast<bool>(message->connected),
            static_cast<bool>(message->ready),
            static_cast<bool>(message->motors_enabled),
            static_cast<bool>(message->critical_fault),
            static_cast<bool>(message->lease_valid),
            message->accepted_sequence,
            nullable_string(message->accepted_producer_boot_id),
            message->accepted_output_sequence,
            static_cast<bool>(message->last_command_accepted),
            nullable_string(message->fsm),
            nullable_string(message->owner),
            nullable_string(message->owner_id),
            nullable_string(message->reason),
        });
      }
      if (count > 0) {
        checked(dds_return_loan(reader_, samples, count), "dds_return_loan(driver_control_state)");
      }
      if (count < 32) {
        return observed;
      }
    }
  }

  std::vector<ObservedControlState> wait_for_any(Deadline deadline) {
    auto observed = wait_until_any(deadline);
    if (!observed.empty()) {
      return observed;
    }
    throw std::runtime_error("timed out waiting for DriverControlState");
  }

  std::vector<ObservedControlState> wait_until_any(Deadline deadline) {
    while (Clock::now() < deadline) {
      auto observed = drain();
      if (!observed.empty()) {
        return observed;
      }
      dds_attach_t triggered{};
      const dds_return_t count =
          dds_waitset_wait(waitset_, &triggered, 1, remaining_dds_time(deadline));
      if (count < 0) {
        throw std::runtime_error(std::string("dds_waitset_wait(control_state): ") +
                                 dds_strretcode(-count));
      }
      if (count == 0) {
        break;
      }
    }
    return {};
  }

  template <typename Predicate>
  ObservedControlState wait_for(Predicate predicate, Deadline deadline) {
    while (Clock::now() < deadline) {
      for (auto &state : drain()) {
        if (predicate(state)) {
          return state;
        }
      }
      dds_attach_t triggered{};
      const dds_return_t count =
          dds_waitset_wait(waitset_, &triggered, 1, remaining_dds_time(deadline));
      if (count < 0) {
        throw std::runtime_error(std::string("dds_waitset_wait(control_state): ") +
                                 dds_strretcode(-count));
      }
      if (count == 0) {
        break;
      }
    }
    throw std::runtime_error("timed out waiting for matching DriverControlState");
  }

 private:
  dds_entity_t reader_{0};
  dds_entity_t condition_{0};
  dds_entity_t waitset_{0};
};

void publish_nav_command(dds_entity_t writer,
                         std::uint64_t output_sequence = kOutputSequence) {
  const auto source_boottime = platform_boot_time_ns();
  const auto source_wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::system_clock::now().time_since_epoch())
                               .count();
  require(source_boottime > 0 && source_wall > 0, "test clocks must be positive");

  lingtu_dds_FinalVelocityCommand message{};
  message.host_boot_id = const_cast<char *>(kHostBootId);
  message.producer_boot_id = const_cast<char *>(kProducerBootId);
  message.output_seq = output_sequence;
  message.source_boottime_ns = source_boottime;
  message.source_wall_ns = static_cast<std::uint64_t>(source_wall);
  message.twist.linear.x = kInputVx;
  message.twist.linear.y = kInputVy;
  message.twist.linear.z = 0.0;
  message.twist.angular.x = 0.0;
  message.twist.angular.y = 0.0;
  message.twist.angular.z = kInputWz;
  checked(dds_write(writer, &message), "dds_write(FinalVelocityCommand)");
}

dds_entity_t create_nav_writer(dds_entity_t participant) {
  const dds_entity_t publisher =
      checked(dds_create_publisher(participant, nullptr, nullptr), "dds_create_publisher");
  const auto &contract = lingtu::message::kNavCmdVel;
  const dds_entity_t topic =
      checked(dds_create_topic(participant, &lingtu_dds_FinalVelocityCommand_desc,
                               contract.dds_topic.data(), nullptr, nullptr),
              "dds_create_topic(FinalVelocityCommand)");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                 "dds_create_writer(FinalVelocityCommand)");
}

dds_entity_t create_control_state_reader(dds_entity_t participant) {
  const dds_entity_t subscriber =
      checked(dds_create_subscriber(participant, nullptr, nullptr), "dds_create_subscriber");
  const auto &contract = lingtu::message::kDriverControlState;
  const dds_entity_t topic =
      checked(dds_create_topic(participant, &lingtu_dds_DriverControlState_desc,
                               contract.dds_topic.data(), nullptr, nullptr),
              "dds_create_topic(DriverControlState)");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(dds_create_reader(subscriber, topic, qos.get(), nullptr),
                 "dds_create_reader(DriverControlState)");
}

bool exact_output_ack(const ObservedControlState &state,
                      std::uint64_t output_sequence = kOutputSequence) {
  return state.last_command_accepted && state.accepted_producer_boot_id == kProducerBootId &&
         state.accepted_output_sequence == output_sequence;
}

void check_internal_zero(const ParsedBridgeCommand &command, std::string_view expected_kind) {
  check_command_identity(command);
  require(command.kind == expected_kind, "internal zero kind mismatch");
  require(command.producer_boot_id.empty(), "internal zero must not carry a producer identity");
  require(command.output_sequence == 0, "internal zero must not carry an output sequence");
  require(command.walk_x == 0.0 && !std::signbit(command.walk_x) && command.walk_y == 0.0 &&
              !std::signbit(command.walk_y) && command.walk_z == 0.0 &&
              !std::signbit(command.walk_z),
          "internal zero must carry exact positive-zero walk values");
}

int run_writer_fixture(const std::filesystem::path &ready_path,
                       const std::filesystem::path &stop_path) {
  dds_entity_t participant = 0;
  try {
    const int domain_id = test_domain_id();
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(writer fixture)");
    const dds_entity_t nav_writer = create_nav_writer(participant);
    {
      std::ofstream output(ready_path, std::ios::binary | std::ios::trunc);
      require(static_cast<bool>(output), "failed to create writer fixture ready marker");
      output << "ready\n";
      require(static_cast<bool>(output), "failed to publish writer fixture ready marker");
    }
    const Deadline deadline = Clock::now() + std::chrono::seconds(20);
    auto publish_path = stop_path;
    publish_path += ".publish";
    bool published = false;
    while (Clock::now() < deadline && !std::filesystem::exists(stop_path)) {
      if (!published && std::filesystem::exists(publish_path)) {
        publish_nav_command(nav_writer);
        published = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    require(std::filesystem::exists(stop_path), "writer fixture timed out waiting for stop marker");
    checked(dds_delete(participant), "dds_delete(writer fixture)");
    return 0;
  } catch (const std::exception &error) {
    if (participant > 0) {
      (void)dds_delete(participant);
    }
    std::fprintf(stderr, "writer fixture failed: %s\n", error.what());
    return 1;
  }
}

}  // namespace

int main(int argc, const char *const argv[]) {
  if (argc == 4 && std::string_view(argv[1]) == "--writer-fixture") {
    return run_writer_fixture(argv[2], argv[3]);
  }
  if (argc != 2) {
    std::fprintf(stderr,
                 "usage: test_mujoco_driver_bridge_dds_process MUJOCO_DRIVER_BRIDGE_EXE\n"
                 "       test_mujoco_driver_bridge_dds_process --writer-fixture READY STOP\n");
    return 2;
  }

#ifndef _WIN32
  std::signal(SIGPIPE, SIG_IGN);
#endif

  dds_entity_t participant = 0;
  std::filesystem::path ready_file;
  try {
    const Deadline overall_deadline = Clock::now() + std::chrono::seconds(15);
    const int domain_id = test_domain_id();
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(test)");
    const dds_entity_t nav_writer = create_nav_writer(participant);
    const dds_entity_t control_state_reader = create_control_state_reader(participant);
    ControlStateStream control_states(participant, control_state_reader);

    ready_file = std::filesystem::temp_directory_path() /
                 ("lingtu_mujoco_driver_bridge_ready_" +
                  std::to_string(Clock::now().time_since_epoch().count()) + ".txt");
    std::filesystem::remove(ready_file);

    BridgeProcess bridge(argv[1], ready_file, domain_id);
    const Deadline discovery_deadline = bounded_deadline(overall_deadline, std::chrono::seconds(5));
    wait_for_dds_match(participant, nav_writer, true, discovery_deadline);
    wait_for_dds_match(participant, control_state_reader, false, discovery_deadline);
    wait_for_ready_file(bridge, ready_file, discovery_deadline);

    bridge.write_line(std::string("LT_DRIVER_ACTIVATE_V2\t") + kBridgeBootId + "\t" +
                      kControllerBootId + "\t1");
    const auto activation = wait_for_command(
        bridge, "activation_zero", bounded_deadline(overall_deadline, std::chrono::seconds(2)));
    check_internal_zero(activation, "activation_zero");

    bridge.write_line(applied_line(activation, 1));
    bridge.write_line(std::string("LT_DRIVER_HEARTBEAT_V2\t") + kBridgeBootId + "\t" +
                      kControllerBootId + "\t2\t2");
    const auto ready = parse_ready(wait_for_protocol_line(
        bridge, "LT_DRIVER_READY_V2", bounded_deadline(overall_deadline, std::chrono::seconds(2))));
    require(ready.bridge_boot_id == kBridgeBootId, "READY bridge boot id mismatch");
    require(ready.controller_boot_id == kControllerBootId, "READY controller boot id mismatch");
    require(ready.accepted_sequence == activation.bridge_command_seq,
            "activation READY must identify the physically applied zero");
    require(ready.producer_boot_id.empty() && ready.output_sequence == 0,
            "activation READY must not invent a nav output identity");

    (void)control_states.drain();
    publish_nav_command(nav_writer);
    const auto nav = wait_for_command(
        bridge, "nav", bounded_deadline(overall_deadline, std::chrono::milliseconds(150)));
    check_command_identity(nav);
    require(nav.bridge_command_seq > activation.bridge_command_seq,
            "nav bridge command sequence must advance after activation zero");
    require(nav.producer_boot_id == kProducerBootId,
            "nav command must preserve producer boot identity");
    require(nav.output_sequence == kOutputSequence,
            "nav command must preserve producer output sequence");
    require(close_enough(nav.walk_x, kExpectedWalkX) && close_enough(nav.walk_y, kExpectedWalkY) &&
                close_enough(nav.walk_z, kExpectedWalkZ),
            "nav command must carry Core-normalized walk 1/-0.5/1");

    const auto before_applied = control_states.wait_for(
        [&](const ObservedControlState &state) {
          return state.accepted_sequence == activation.bridge_command_seq &&
                 !state.last_command_accepted;
        },
        bounded_deadline(overall_deadline, std::chrono::milliseconds(150)));
    require(before_applied.connected && before_applied.ready && before_applied.lease_valid,
            "pending nav must not disturb the physically applied activation state");
    while (const auto line = bridge.lines().wait_for_next(Clock::now())) {
      const auto fields = split_tabs(*line);
      require(fields.front() != "LT_DRIVER_FAULT_V2",
              "bridge faulted before the physical Nav APPLIED: " + *line);
      if (fields.front() == "LT_DRIVER_READY_V2") {
        const auto early_ready = parse_ready(*line);
        require(!(early_ready.producer_boot_id == kProducerBootId &&
                  early_ready.output_sequence == kOutputSequence),
                "bridge emitted nav READY before matching physical APPLIED");
      }
    }

    bridge.write_line(applied_line(nav, 3));
    bridge.write_line(std::string("LT_DRIVER_HEARTBEAT_V2\t") + kBridgeBootId + "\t" +
                      kControllerBootId + "\t3\t4");
    const auto nav_ready = parse_ready(wait_for_protocol_line(
        bridge, "LT_DRIVER_READY_V2", bounded_deadline(overall_deadline, std::chrono::seconds(2))));
    require(nav_ready.bridge_boot_id == kBridgeBootId &&
                nav_ready.controller_boot_id == kControllerBootId,
            "nav READY identity mismatch");
    require(nav_ready.accepted_sequence == nav.bridge_command_seq &&
                nav_ready.producer_boot_id == kProducerBootId &&
                nav_ready.output_sequence == kOutputSequence,
            "nav READY must identify the exact physically applied output");
    const auto accepted = control_states.wait_for(
        [&](const ObservedControlState &state) {
          return exact_output_ack(state) && state.accepted_sequence == nav.bridge_command_seq;
        },
        bounded_deadline(overall_deadline, std::chrono::seconds(2)));
    require(accepted.connected && accepted.ready && accepted.motors_enabled &&
                !accepted.critical_fault && accepted.lease_valid,
            "physical nav ACK must be published only in a ready, leased control state");
    require(accepted.owner == "grpc" && accepted.owner_id == "lingtu-driver@robot",
            "ready DriverControlState must expose the canonical simulation driver principal");
    require(accepted.accepted_producer_boot_id == kProducerBootId &&
                accepted.accepted_output_sequence == kOutputSequence &&
                accepted.accepted_sequence == nav.bridge_command_seq,
            "DriverControlState must publish the exact physically applied producer/output");

    bridge.write_line(std::string("LT_DRIVER_DEACTIVATE_V2\t") + kBridgeBootId + "\t" +
                      kControllerBootId + "\t4");
    const auto deactivation = wait_for_command(
        bridge, "deactivate_zero", bounded_deadline(overall_deadline, std::chrono::seconds(2)));
    check_internal_zero(deactivation, "deactivate_zero");
    require(deactivation.bridge_command_seq > nav.bridge_command_seq,
            "deactivation zero command sequence must advance after nav");
    bridge.write_line(applied_line(deactivation, 5));
    const auto stopped = control_states.wait_for(
        [](const ObservedControlState &state) {
          return !state.connected && !state.ready && !state.lease_valid &&
                 !state.last_command_accepted && state.reason == "stopped";
        },
        bounded_deadline(overall_deadline, std::chrono::seconds(2)));
    require(!stopped.motors_enabled && !stopped.critical_fault &&
                stopped.accepted_producer_boot_id.empty() && stopped.accepted_output_sequence == 0,
            "physical deactivation must publish a clean terminal control state");
    bridge.close_stdin();

    const auto exit_code =
        bridge.wait_until(bounded_deadline(overall_deadline, std::chrono::seconds(3)));
    require(exit_code.has_value(), "bridge did not exit after deactivation APPLIED and stdin EOF");
    require(*exit_code == 0, "bridge exited nonzero after clean deactivation: " +
                                 std::to_string(*exit_code) + "; stdout:\n" + bridge.stdout_text());

    std::filesystem::remove(ready_file);
    ready_file = std::filesystem::temp_directory_path() /
                 ("lingtu_mujoco_driver_bridge_signal_ready_" +
                  std::to_string(Clock::now().time_since_epoch().count()) + ".txt");
    std::filesystem::remove(ready_file);
    (void)control_states.drain();

    BridgeProcess signal_bridge(argv[1], ready_file, domain_id);
    const Deadline signal_discovery_deadline =
        bounded_deadline(overall_deadline, std::chrono::seconds(5));
    wait_for_dds_match(participant, nav_writer, true, signal_discovery_deadline);
    wait_for_dds_match(participant, control_state_reader, false, signal_discovery_deadline);
    wait_for_ready_file(signal_bridge, ready_file, signal_discovery_deadline);

    signal_bridge.write_line(std::string("LT_DRIVER_ACTIVATE_V2\t") + kBridgeBootId + "\t" +
                             kControllerBootId + "\t1");
    const auto signal_activation =
        wait_for_command(signal_bridge, "activation_zero",
                         bounded_deadline(overall_deadline, std::chrono::seconds(2)));
    check_internal_zero(signal_activation, "activation_zero");
    signal_bridge.write_line(applied_line(signal_activation, 1));
    signal_bridge.write_line(std::string("LT_DRIVER_HEARTBEAT_V2\t") + kBridgeBootId + "\t" +
                             kControllerBootId + "\t2\t2");
    (void)parse_ready(
        wait_for_protocol_line(signal_bridge, "LT_DRIVER_READY_V2",
                               bounded_deadline(overall_deadline, std::chrono::seconds(2))));

    (void)control_states.drain();
    publish_nav_command(nav_writer);
    const auto pending_nav = wait_for_command(
        signal_bridge, "nav", bounded_deadline(overall_deadline, std::chrono::milliseconds(150)));
    check_command_identity(pending_nav);
    require(pending_nav.producer_boot_id == kProducerBootId &&
                pending_nav.output_sequence == kOutputSequence,
            "signal scenario nav command identity mismatch");

    signal_bridge.request_graceful_stop();
    const auto stopping = control_states.wait_for(
        [&](const ObservedControlState &state) {
          return !state.ready && state.accepted_sequence == signal_activation.bridge_command_seq &&
                 state.reason == "deactivating_zero";
        },
        bounded_deadline(overall_deadline, std::chrono::milliseconds(250)));
    require(!stopping.last_command_accepted,
            "signal stop must invalidate ACK while nav is still awaiting APPLIED");

    signal_bridge.write_line(applied_line(pending_nav, 3));
    const auto safety_zero =
        wait_for_command(signal_bridge, "safety_zero",
                         bounded_deadline(overall_deadline, std::chrono::milliseconds(250)));
    check_internal_zero(safety_zero, "safety_zero");
    require(safety_zero.bridge_command_seq > pending_nav.bridge_command_seq,
            "signal safety zero sequence must advance after the pending nav command");
    signal_bridge.write_line(applied_line(safety_zero, 4));

    const auto signal_exit =
        signal_bridge.wait_until(bounded_deadline(overall_deadline, std::chrono::seconds(3)));
    require(signal_exit.has_value(),
            "bridge did not exit after pending nav and signal safety zero were physically APPLIED");
    require(*signal_exit == 0, "bridge exited nonzero after signal safety zero APPLIED: " +
                                   std::to_string(*signal_exit) + "; stdout:\n" +
                                   signal_bridge.stdout_text());

    std::filesystem::remove(ready_file);
    ready_file.clear();
    dds_delete(participant);
    participant = 0;
    std::printf(
        "MuJoCo driver bridge DDS process passed: domain=%d cmd_topic=%s control_state_topic=%s\n",
        domain_id, lingtu::message::kNavCmdVel.dds_topic.data(),
        lingtu::message::kDriverControlState.dds_topic.data());
    return 0;
  } catch (const std::exception &exception) {
    if (!ready_file.empty()) {
      std::error_code ignored;
      std::filesystem::remove(ready_file, ignored);
    }
    if (participant > 0) {
      dds_delete(participant);
    }
    std::fprintf(stderr, "test_mujoco_driver_bridge_dds_process failed: %s\n", exception.what());
    return 1;
  }
}
