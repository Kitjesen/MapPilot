#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include "lingtu/recording/camera_recording.hpp"

namespace {

volatile sig_atomic_t stop_requested = 0;

extern "C" void request_stop(int) {
  stop_requested = 1;
}

class OneFrameSource final : public lingtu::recording::CameraFrameSource {
 public:
  bool next(lingtu::recording::CameraFrame &frame) override {
    if (sent_) return false;
    sent_ = true;
    frame.sequence = 1;
    frame.timestamp_ns = 1;
    frame.width = frame.height = 2;
    frame.stride = 6;
    frame.encoding = "rgb8";
    frame.frame_id = "test-only-camera";
    frame.payload.assign(12, std::byte{0x42});
    return true;
  }
 private:
  bool sent_{false};
};

class FixtureEncoder final : public lingtu::recording::CameraSegmentEncoder {
 public:
  std::string codec() const override { return "test-only-raw"; }
  std::string container_extension() const override { return ".rgb"; }
  void begin(const std::filesystem::path &path,
             const lingtu::recording::CameraStreamDescription &) override {
    output_.open(path, std::ios::binary);
    if (!output_) throw std::runtime_error("cannot open test camera segment");
  }
  void write(const lingtu::recording::CameraFrame &frame) override {
    output_.write(reinterpret_cast<const char *>(frame.payload.data()), frame.payload.size());
  }
  void finish() override { output_.close(); }
  void abort() noexcept override { output_.close(); }
 private:
  std::ofstream output_;
};

}  // namespace

int main() {
  struct sigaction action{};
  action.sa_handler = request_stop;
  sigemptyset(&action.sa_mask);
  if (::sigaction(SIGTERM, &action, nullptr) != 0) {
    return 2;
  }

  {
    std::ofstream pid_file("worker.pid", std::ios::trunc);
    pid_file << ::getpid() << '\n';
  }
  {
    std::ofstream ready_file("worker.ready", std::ios::trunc);
    ready_file << "ready\n";
  }
  std::cout << "fake camera recorder ready" << std::endl;

  while (!stop_requested) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  try {
    OneFrameSource source;
    FixtureEncoder encoder;
    lingtu::recording::CameraRecordingOptions options;
    options.session_directory = ".";
    lingtu::recording::CameraSegmentRecorder(options, source, encoder).record();
  } catch (const std::exception &error) {
    std::cerr << "failed to finalize test camera artifact: " << error.what() << '\n';
    return 3;
  }
  return 0;
}
