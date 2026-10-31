#include "lingtu/recording/live_observer.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> stopped{false};
void stop(int) {
  stopped.store(true);
}
}  // namespace

int main(int argc, char **argv) {
  try {
    int domain = 0, interval_ms = 500;
    double seconds = 0;
    std::vector<std::string> topics;
    for (int i = 1; i < argc; ++i) {
      const std::string option(argv[i]);
      const auto value = [&]() -> std::string {
        if (++i >= argc)
          throw std::invalid_argument("missing value for " + option);
        return argv[i];
      };
      if (option == "--domain")
        domain = std::stoi(value());
      else if (option == "--interval-ms")
        interval_ms = std::stoi(value());
      else if (option == "--seconds")
        seconds = std::stod(value());
      else if (option == "--topic")
        topics.push_back(value());
      else if (option == "--help" || option == "-h") {
        std::cout << "usage: lingtu_dds_observer [--domain N] [--interval-ms 500] [--seconds N] "
                     "[--topic TOPIC ...]\nRead-only observed DDS rates and bounded latest values "
                     "as JSON lines; no recording or commands.\n";
        return 0;
      } else
        throw std::invalid_argument("unknown option: " + option);
    }
    if (interval_ms < 100 || interval_ms > 5000)
      throw std::invalid_argument("--interval-ms must be 100..5000");
    if (!std::isfinite(seconds) || seconds < 0)
      throw std::invalid_argument("--seconds must be finite and nonnegative");
    lingtu::recording::LiveObserver observer(domain, topics);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    const auto started = std::chrono::steady_clock::now();
    auto next = started;
    while (!stopped.load()) {
      observer.poll();
      const auto now = std::chrono::steady_clock::now();
      if (now >= next) {
        std::cout << observer.snapshot() << std::endl;
        next = now + std::chrono::milliseconds(interval_ms);
      }
      if (seconds > 0 && std::chrono::duration<double>(now - started).count() >= seconds)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "lingtu_dds_observer: " << error.what() << '\n';
    return 2;
  }
}
