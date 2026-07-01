#include <chrono>
#include <iostream>
#include <string>

#include "wtrtk980/nmea_parser.hpp"
#include "wtrtk980/serial_detector.hpp"
#include "wtrtk980/serial_reader.hpp"

namespace {

struct Args {
  std::string device = "/dev/wtrtk980";
  int baud = 115200;
  double seconds = 0.0;
  double probe_seconds = 3.0;
  bool find = false;
  bool raw = false;
};

Args parseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    auto next = [&]() -> std::string { return i + 1 < argc ? std::string(argv[++i]) : std::string(); };
    if (a == "--device") args.device = next();
    else if (a == "--baud") args.baud = std::stoi(next());
    else if (a == "--seconds") args.seconds = std::stod(next());
    else if (a == "--probe-seconds") args.probe_seconds = std::stod(next());
    else if (a == "--find") args.find = true;
    else if (a == "--raw") args.raw = true;
    else if (a == "--help" || a == "-h") {
      std::cout << "Usage: wtrtk980_reader [--device /dev/wtrtk980|auto] [--seconds N] [--raw] [--find]\n";
      std::exit(0);
    }
  }
  return args;
}

}  // namespace

int main(int argc, char** argv) {
  auto args = parseArgs(argc, argv);
  try {
    if (args.find) {
      auto result = wtrtk980::findWtrtk980Device("/dev/wtrtk980", args.baud, args.probe_seconds);
      if (result.device.empty()) {
        std::cerr << "WTRTK-980 not found\n";
        for (const auto& item : result.checked) std::cerr << item << "\n";
        return 1;
      }
      std::cout << result.device << " score=" << result.score << " reasons=";
      for (const auto& reason : result.reasons) std::cout << reason << " ";
      std::cout << "\n";
      return 0;
    }

    wtrtk980::SerialReader reader(args.device, args.baud, 1000);
    reader.open();
    wtrtk980::GpsFix state;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(args.seconds * 1000));
    while (true) {
      if (args.seconds > 0 && std::chrono::steady_clock::now() >= deadline) return 0;
      auto raw = reader.readRawLine();
      if (!raw) continue;
      if (args.raw) std::cout << *raw << "\n";
      if (wtrtk980::parseNmeaLine(*raw, state)) std::cout << state.summary() << "\n";
    }
  } catch (const std::exception& exc) {
    std::cerr << "error: " << exc.what() << "\n";
    return 1;
  }
}
