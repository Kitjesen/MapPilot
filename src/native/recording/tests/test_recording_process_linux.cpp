#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "lingtu/recording/recording_linux.hpp"

namespace {

namespace recording = lingtu::recording;
volatile sig_atomic_t child_stop_requested = 0;

extern "C" void request_child_stop(int) {
  child_stop_requested = 1;
}

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

std::filesystem::path unique_directory() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  return std::filesystem::temp_directory_path() /
         ("lingtu-recording-process-" + std::to_string(nonce));
}

std::string read_text(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

int run_fake_recorder() {
  std::signal(SIGTERM, request_child_stop);
  std::cout << "fake recorder stdout" << std::endl;
  std::cerr << "fake recorder stderr" << std::endl;
  while (!child_stop_requested) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return 0;
}

int run_fake_recorder_with_encoder() {
  child_stop_requested = 0;
  std::signal(SIGTERM, request_child_stop);
  int input_pipe[2]{-1, -1};
  if (::pipe(input_pipe) != 0) {
    return 40;
  }
  const pid_t encoder = ::fork();
  if (encoder < 0) {
    ::close(input_pipe[0]);
    ::close(input_pipe[1]);
    return 41;
  }
  if (encoder == 0) {
    std::signal(SIGTERM, SIG_DFL);
    ::close(input_pipe[1]);
    char byte = 0;
    while (true) {
      const auto count = ::read(input_pipe[0], &byte, sizeof(byte));
      if (count == 0) {
        _exit(0);
      }
      if (count < 0 && errno == EINTR) {
        continue;
      }
      if (count < 0) {
        _exit(42);
      }
    }
  }

  ::close(input_pipe[0]);
  std::cout << "nested recorder ready" << std::endl;
  while (!child_stop_requested) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  ::close(input_pipe[1]);
  int status = 0;
  pid_t result;
  do {
    result = ::waitpid(encoder, &status, 0);
  } while (result < 0 && errno == EINTR);
  return result == encoder && WIFEXITED(status) && WEXITSTATUS(status) == 0 ? 0 : 43;
}

void test_process_uses_argv_working_directory_logs_and_graceful_stop(const char *self) {
  const auto directory = unique_directory();
  std::filesystem::create_directories(directory / "logs");

  recording::RecordingProcessSpec spec;
  spec.name = "fake";
  spec.argv = {self, "--fake-recorder"};
  spec.working_directory = directory;
  spec.stdout_log = directory / "logs" / "fake.stdout.log";
  spec.stderr_log = directory / "logs" / "fake.stderr.log";

  recording::PosixRecordingProcessFactory factory;
  auto process = factory.start(spec);
  const auto running = process->snapshot();
  require(running.process_id > 0 && running.running && !running.exit_code,
          "spawned recorder is not running");
  const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (read_text(spec.stdout_log).find("fake recorder stdout") == std::string::npos &&
         std::chrono::steady_clock::now() < ready_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  require(read_text(spec.stdout_log).find("fake recorder stdout") != std::string::npos,
          "recorder did not become ready");
  process->request_stop();
  require(process->wait(std::chrono::seconds(2)) == 0, "recorder did not stop cleanly on SIGTERM");
  const auto stopped = process->snapshot();
  require(!stopped.running && stopped.exit_code == 0, "stopped recorder status is inconsistent");
  require(read_text(spec.stdout_log).find("fake recorder stdout") != std::string::npos,
          "recorder stdout was not captured");
  require(read_text(spec.stderr_log).find("fake recorder stderr") != std::string::npos,
          "recorder stderr was not captured");

  std::filesystem::remove_all(directory);
}

void test_graceful_stop_allows_worker_to_finalize_its_encoder(const char *self) {
  const auto directory = unique_directory();
  std::filesystem::create_directories(directory / "logs");

  recording::RecordingProcessSpec spec;
  spec.name = "nested";
  spec.argv = {self, "--fake-recorder-with-encoder"};
  spec.working_directory = directory;
  spec.stdout_log = directory / "logs" / "nested.stdout.log";
  spec.stderr_log = directory / "logs" / "nested.stderr.log";

  recording::PosixRecordingProcessFactory factory;
  auto process = factory.start(spec);
  const auto ready_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (read_text(spec.stdout_log).find("nested recorder ready") == std::string::npos &&
         std::chrono::steady_clock::now() < ready_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  require(read_text(spec.stdout_log).find("nested recorder ready") != std::string::npos,
          "nested recorder did not become ready");
  process->request_stop();
  require(process->wait(std::chrono::seconds(2)) == 0,
          "manager interrupted the worker-owned encoder before graceful finalization");

  std::filesystem::remove_all(directory);
}

void test_missing_executable_fails_before_a_process_is_returned() {
  const auto directory = unique_directory();
  std::filesystem::create_directories(directory / "logs");
  recording::RecordingProcessSpec spec;
  spec.name = "missing";
  spec.argv = {"/definitely/missing/lingtu-recorder-worker"};
  spec.working_directory = directory;
  spec.stdout_log = directory / "logs" / "missing.stdout.log";
  spec.stderr_log = directory / "logs" / "missing.stderr.log";

  recording::PosixRecordingProcessFactory factory;
  bool rejected = false;
  try {
    static_cast<void>(factory.start(spec));
  } catch (const std::runtime_error &error) {
    rejected = std::string(error.what()).find("executing") != std::string::npos;
  }
  require(rejected, "missing recorder executable was not reported synchronously");
  std::filesystem::remove_all(directory);
}

}  // namespace

int main(int argc, char **argv) {
  if (argc > 1 && std::string_view(argv[1]) == "--fake-recorder") {
    return run_fake_recorder();
  }
  if (argc > 1 && std::string_view(argv[1]) == "--fake-recorder-with-encoder") {
    return run_fake_recorder_with_encoder();
  }
  try {
    test_process_uses_argv_working_directory_logs_and_graceful_stop(argv[0]);
    test_graceful_stop_allows_worker_to_finalize_its_encoder(argv[0]);
    test_missing_executable_fails_before_a_process_is_returned();
    return 0;
  } catch (const std::exception &error) {
    std::fprintf(stderr, "test_recording_process_linux: %s\n", error.what());
    return 1;
  }
}
