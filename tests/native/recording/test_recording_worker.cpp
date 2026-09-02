#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <thread>
#include <unistd.h>

namespace {

volatile sig_atomic_t stop_requested = 0;

extern "C" void request_stop(int) {
  stop_requested = 1;
}

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

  std::ofstream artifact("camera_color.mcap", std::ios::binary | std::ios::trunc);
  artifact << "fake-camera-index\n";
  artifact.close();
  if (!artifact) {
    std::cerr << "failed to finalize fake camera artifact\n";
    return 3;
  }
  return 0;
}
