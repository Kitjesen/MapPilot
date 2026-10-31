#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string_view>
#include <thread>

#include "lingtu/recording/recording_windows.hpp"

namespace recording = lingtu::recording;

namespace {

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

std::string read_text(const std::filesystem::path &path) {
  std::ifstream input(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int worker(int argc, char **argv) {
  recording::WindowsRecordingStopEvent stop;
  require(argc == 5 && std::string(argv[2]) == "argument with spaces" &&
              std::string(argv[3]) == "quote\"inside" && std::string(argv[4]) == "C:\\trailing\\",
          "worker argv was changed");
  std::cout << "worker ready " << std::filesystem::current_path().filename().string() << std::endl;
  std::cerr << "worker stderr" << std::endl;
  while (!stop.stop_requested()) std::this_thread::sleep_for(std::chrono::milliseconds(2));
  std::ofstream("finalized.mcap") << "finalized before exit\n";
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc > 1 && std::string_view(argv[1]) == "--worker") return worker(argc, argv);
    const auto directory = std::filesystem::temp_directory_path() /
        ("lingtu recording process " + std::to_string(::GetCurrentProcessId()));
    std::filesystem::create_directories(directory / "logs");
    recording::RecordingProcessSpec spec{
        "worker", {std::filesystem::absolute(argv[0]).string(), "--worker",
                   "argument with spaces", "quote\"inside", "C:\\trailing\\"}, directory,
        directory / "logs" / "stdout.log", directory / "logs" / "stderr.log"};
    recording::WindowsRecordingProcessFactory factory;
    auto process = factory.start(spec);
    require(process->snapshot().running, "worker did not start");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (read_text(spec.stdout_log).find("worker ready") == std::string::npos &&
           std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    require(read_text(spec.stdout_log).find(directory.filename().string()) != std::string::npos,
            "worker argv or working directory was incorrect");
    require(read_text(spec.stderr_log).find("worker stderr") != std::string::npos,
            "worker stderr was not redirected");
    process->request_stop();
    require(process->wait(std::chrono::seconds(2)) == 0, "worker did not stop gracefully");
    require(!process->snapshot().running && process->snapshot().exit_code == 0,
            "worker terminal snapshot was inconsistent");
    require(read_text(directory / "finalized.mcap").find("finalized before exit") !=
                std::string::npos, "worker artifact was not finalized before exit");
    process.reset();

    spec.argv = {(directory / "missing-worker.exe").string()};
    spec.stdout_log = directory / "logs" / "missing.stdout.log";
    spec.stderr_log = directory / "logs" / "missing.stderr.log";
    bool rejected = false;
    try { static_cast<void>(factory.start(spec)); }
    catch (const std::runtime_error &error) {
      rejected = std::string(error.what()).find("executing recorder") != std::string::npos;
    }
    require(rejected, "missing executable was not reported synchronously");
    std::filesystem::remove_all(directory);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "test_recording_process_windows: " << error.what() << '\n';
    return 1;
  }
}
