#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
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
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {

constexpr std::uint64_t kSourceWallNs = 2030123456789ULL;
constexpr double kSourceWallS = static_cast<double>(kSourceWallNs) * 1e-9;
constexpr double kVx = 0.31;
constexpr double kVy = -0.12;
constexpr double kWz = 0.44;

dds_entity_t checked(dds_return_t value, const char *what) {
  if (value < 0) {
    throw std::runtime_error(std::string(what) + ": " + dds_strretcode(-value));
  }
  return static_cast<dds_entity_t>(value);
}

int test_domain_id() {
  return lingtu::sim::dds_adapter::test::domain_id_from_environment();
}

bool close_enough(double left, double right) {
  return std::abs(left - right) < 1e-6;
}

void wait_for_ready_file(const std::filesystem::path &ready) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
  while (std::chrono::steady_clock::now() < deadline) {
    std::error_code ec;
    if (std::filesystem::exists(ready, ec)) {
      std::ifstream in(ready);
      std::string value;
      std::getline(in, value);
      if (value == "ready") {
        return;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error("cmd_vel tap did not write ready marker: " + ready.string());
}

struct ParsedCommand {
  std::uint64_t sequence{0};
  double source_wall_s{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

std::optional<ParsedCommand> parse_command_line(const std::string &line) {
  std::istringstream input(line);
  std::string prefix;
  ParsedCommand command;
  if (std::getline(input, prefix, '\t') && prefix == "LT_CMD_V1" && (input >> command.sequence)) {
    if (input.peek() == '\t') {
      input.get();
    }
    if (input >> command.source_wall_s) {
      if (input.peek() == '\t') {
        input.get();
      }
      if (input >> command.vx) {
        if (input.peek() == '\t') {
          input.get();
        }
        if (input >> command.vy) {
          if (input.peek() == '\t') {
            input.get();
          }
          if (input >> command.wz) {
            return command;
          }
        }
      }
    }
  }
  return std::nullopt;
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

  std::optional<ParsedCommand>
  wait_for_matching_command(std::chrono::steady_clock::time_point deadline) {
    std::unique_lock<std::mutex> lock(mutex_);
    while (std::chrono::steady_clock::now() < deadline) {
      for (const std::string &line : lines_) {
        auto parsed = parse_command_line(line);
        if (parsed.has_value() && parsed->sequence > 0 &&
            close_enough(parsed->source_wall_s, kSourceWallS) && close_enough(parsed->vx, kVx) &&
            close_enough(parsed->vy, kVy) && close_enough(parsed->wz, kWz)) {
          return parsed;
        }
      }
      cv_.wait_until(lock, deadline);
    }
    return std::nullopt;
  }

  std::string joined_lines() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string joined;
    for (const std::string &line : lines_) {
      joined += line;
      joined += "\n";
    }
    if (!current_.empty()) {
      joined += current_;
    }
    return joined;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::string> lines_;
  std::string current_;
};

class TapProcess {
 public:
  TapProcess(const std::filesystem::path &binary, const std::filesystem::path &ready,
             int domain_id) {
    start(binary, ready, domain_id);
  }

  TapProcess(const TapProcess &) = delete;
  TapProcess &operator=(const TapProcess &) = delete;

  ~TapProcess() {
    terminate();
    join_reader();
  }

  std::optional<ParsedCommand> wait_for_command(std::chrono::milliseconds timeout) {
    return lines_.wait_for_matching_command(std::chrono::steady_clock::now() + timeout);
  }

  bool terminate_and_wait(std::chrono::milliseconds timeout) {
    terminate();
    return wait(timeout);
  }

  std::string stdout_text() const { return lines_.joined_lines(); }

 private:
#ifdef _WIN32
  static std::wstring quote(const std::filesystem::path &value) {
    return L"\"" + value.wstring() + L"\"";
  }
#endif

  void start(const std::filesystem::path &binary, const std::filesystem::path &ready,
             int domain_id) {
#ifdef _WIN32
    SECURITY_ATTRIBUTES security{};
    security.nLength = sizeof(security);
    security.bInheritHandle = TRUE;
    HANDLE stdout_read = nullptr;
    HANDLE stdout_write = nullptr;
    if (!CreatePipe(&stdout_read, &stdout_write, &security, 0)) {
      throw std::runtime_error("failed to create stdout pipe");
    }
    if (!SetHandleInformation(stdout_read, HANDLE_FLAG_INHERIT, 0)) {
      CloseHandle(stdout_read);
      CloseHandle(stdout_write);
      throw std::runtime_error("failed to mark stdout read handle non-inheritable");
    }

    std::wstring command = quote(binary) + L" --domain-id " + std::to_wstring(domain_id) +
                           L" --ready-file " + quote(ready);
    STARTUPINFOW si{};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdOutput = stdout_write;
    si.hStdError = GetStdHandle(STD_ERROR_HANDLE);
    PROCESS_INFORMATION pi{};
    std::vector<wchar_t> mutable_command(command.begin(), command.end());
    mutable_command.push_back(L'\0');
    const BOOL ok = CreateProcessW(nullptr, mutable_command.data(), nullptr, nullptr, TRUE, 0,
                                   nullptr, nullptr, &si, &pi);
    CloseHandle(stdout_write);
    if (!ok) {
      CloseHandle(stdout_read);
      throw std::runtime_error("failed to launch cmd_vel tap process");
    }
    process_ = pi.hProcess;
    thread_ = pi.hThread;
    stdout_read_ = stdout_read;
#else
    int pipe_fds[2]{};
    if (pipe(pipe_fds) != 0) {
      throw std::runtime_error("failed to create stdout pipe");
    }
    pid_ = fork();
    if (pid_ < 0) {
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      throw std::runtime_error("failed to fork cmd_vel tap process");
    }
    if (pid_ == 0) {
      dup2(pipe_fds[1], STDOUT_FILENO);
      close(pipe_fds[0]);
      close(pipe_fds[1]);
      const std::string domain = std::to_string(domain_id);
      execl(binary.c_str(), binary.c_str(), "--domain-id", domain.c_str(), "--ready-file",
            ready.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
    close(pipe_fds[1]);
    stdout_fd_ = pipe_fds[0];
#endif
    reader_ = std::thread([this]() { read_stdout(); });
  }

  void read_stdout() {
    char buffer[256];
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
      if (stdout_fd_ < 0) {
        break;
      }
      const ssize_t read_count = read(stdout_fd_, buffer, sizeof(buffer));
      if (read_count <= 0) {
        break;
      }
      lines_.add_bytes(buffer, static_cast<std::size_t>(read_count));
#endif
    }
  }

  void terminate() {
#ifdef _WIN32
    if (process_ != nullptr) {
      TerminateProcess(process_, 0);
    }
#else
    if (pid_ > 0) {
      kill(pid_, SIGTERM);
    }
#endif
  }

  bool wait(std::chrono::milliseconds timeout) {
#ifdef _WIN32
    if (process_ == nullptr) {
      return true;
    }
    const DWORD rc = WaitForSingleObject(process_, static_cast<DWORD>(timeout.count()));
    if (rc != WAIT_OBJECT_0) {
      return false;
    }
    CloseHandle(process_);
    process_ = nullptr;
    if (thread_ != nullptr) {
      CloseHandle(thread_);
      thread_ = nullptr;
    }
    if (stdout_read_ != nullptr) {
      CloseHandle(stdout_read_);
      stdout_read_ = nullptr;
    }
    return true;
#else
    if (pid_ <= 0) {
      return true;
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      int status = 0;
      const pid_t rc = waitpid(pid_, &status, WNOHANG);
      if (rc == pid_) {
        pid_ = -1;
        if (stdout_fd_ >= 0) {
          close(stdout_fd_);
          stdout_fd_ = -1;
        }
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
#endif
  }

  void join_reader() {
    if (reader_.joinable()) {
      reader_.join();
    }
  }

  LineCollector lines_;
  std::thread reader_;
#ifdef _WIN32
  HANDLE process_{nullptr};
  HANDLE thread_{nullptr};
  HANDLE stdout_read_{nullptr};
#else
  pid_t pid_{-1};
  int stdout_fd_{-1};
#endif
};

void publish_command(dds_entity_t writer) {
  lingtu_dds_FinalVelocityCommand msg{};
  msg.host_boot_id = const_cast<char *>("test-host-boot");
  msg.producer_boot_id = const_cast<char *>("test-navd-boot");
  msg.output_seq = 77;
  msg.source_boottime_ns = 1000000ULL;
  msg.source_wall_ns = kSourceWallNs;
  msg.twist.linear.x = kVx;
  msg.twist.linear.y = kVy;
  msg.twist.linear.z = 0.0;
  msg.twist.angular.x = 0.0;
  msg.twist.angular.y = 0.0;
  msg.twist.angular.z = kWz;
  checked(dds_write(writer, &msg), "dds_write(cmd_vel)");
}

dds_entity_t create_writer(dds_entity_t participant) {
  const dds_entity_t publisher =
      checked(dds_create_publisher(participant, nullptr, nullptr), "dds_create_publisher");
  const auto &contract = lingtu::message::kNavCmdVel;
  const dds_entity_t topic =
      checked(dds_create_topic(participant, &lingtu_dds_FinalVelocityCommand_desc,
                               contract.dds_topic.data(), nullptr, nullptr),
              "dds_create_topic(cmd_vel)");
  auto qos = lingtu::dds::make_qos(lingtu::dds::qos_for_topic(contract.dds_topic));
  return checked(dds_create_writer(publisher, topic, qos.get(), nullptr),
                 "dds_create_writer(cmd_vel)");
}

}  // namespace

int main(int argc, const char *const argv[]) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: test_cmd_vel_tap_dds_process CMD_VEL_TAP_EXE\n");
    return 2;
  }

  dds_entity_t participant = 0;
  try {
    const int domain_id = test_domain_id();
    participant =
        checked(dds_create_participant(static_cast<dds_domainid_t>(domain_id), nullptr, nullptr),
                "dds_create_participant(writer)");
    const dds_entity_t writer = create_writer(participant);
    const auto ready =
        std::filesystem::temp_directory_path() /
        ("lingtu_cmd_vel_tap_ready_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".txt");
    std::filesystem::remove(ready);

    TapProcess tap(argv[1], ready, domain_id);
    wait_for_ready_file(ready);

    std::optional<ParsedCommand> received;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (std::chrono::steady_clock::now() < deadline && !received.has_value()) {
      publish_command(writer);
      received = tap.wait_for_command(std::chrono::milliseconds(50));
    }

    if (!received.has_value()) {
      throw std::runtime_error("cmd_vel tap did not emit matching LT_CMD_V1; stdout:\n" +
                               tap.stdout_text());
    }
    if (!tap.terminate_and_wait(std::chrono::seconds(3))) {
      throw std::runtime_error("cmd_vel tap did not exit after termination");
    }

    std::filesystem::remove(ready);
    dds_delete(participant);
    std::printf("cmd_vel tap DDS process passed: domain=%d topic=%s\n", domain_id,
                lingtu::message::kNavCmdVel.dds_topic.data());
    return 0;
  } catch (const std::exception &exc) {
    if (participant > 0) {
      dds_delete(participant);
    }
    std::fprintf(stderr, "test_cmd_vel_tap_dds_process failed: %s\n", exc.what());
    return 1;
  }
}
