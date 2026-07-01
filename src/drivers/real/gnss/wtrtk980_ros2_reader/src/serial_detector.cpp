#include "wtrtk980/serial_detector.hpp"

#include <dirent.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <filesystem>
#include <set>
#include <sstream>

#include "wtrtk980/serial_reader.hpp"

namespace wtrtk980 {
namespace {

std::string runCommand(const std::string& command) {
  std::array<char, 256> buffer{};
  std::string result;
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) return result;
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) result += buffer.data();
  pclose(pipe);
  return result;
}

bool startsWith(const std::string& text, const std::string& prefix) {
  return text.rfind(prefix, 0) == 0;
}

bool containsLine(const std::vector<std::string>& lines, const std::string& prefix) {
  return std::any_of(lines.begin(), lines.end(), [&](const auto& line) { return startsWith(line, prefix); });
}

}  // namespace

std::vector<std::string> listSerialDevices() {
  std::vector<std::string> devices;
  std::set<std::string> seen;
  DIR* dir = opendir("/dev");
  if (!dir) return devices;
  while (dirent* ent = readdir(dir)) {
    std::string name(ent->d_name);
    if (!startsWith(name, "ttyUSB") && !startsWith(name, "ttyACM")) continue;
    std::string path = "/dev/" + name;
    std::string real;
    try {
      real = std::filesystem::weakly_canonical(path).string();
    } catch (...) {
      real = path;
    }
    if (seen.insert(real).second) devices.push_back(path);
  }
  closedir(dir);
  std::sort(devices.begin(), devices.end());
  return devices;
}

bool isCh341Candidate(const std::string& device) {
  std::string output = runCommand("udevadm info -q property -n " + device + " 2>/dev/null");
  return output.find("ID_VENDOR_ID=1a86") != std::string::npos &&
         output.find("ID_MODEL_ID=7523") != std::string::npos &&
         output.find("ID_USB_DRIVER=ch341") != std::string::npos;
}

std::vector<std::string> probeLines(const std::string& device, int baud, double seconds) {
  std::vector<std::string> lines;
  try {
    SerialReader reader(device, baud, 500);
    reader.open();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(static_cast<int>(seconds * 1000));
    while (std::chrono::steady_clock::now() < deadline) {
      auto line = reader.readRawLine();
      if (line) lines.push_back(*line);
    }
  } catch (...) {
  }
  return lines;
}

std::pair<int, std::vector<std::string>> scoreWtrtk980Lines(const std::vector<std::string>& lines) {
  int score = 0;
  std::vector<std::string> reasons;
  if (containsLine(lines, "#BASEINFOA")) { score += 10; reasons.push_back("#BASEINFOA"); }
  if (containsLine(lines, "$GNRMC") || containsLine(lines, "$GPRMC")) { score += 3; reasons.push_back("RMC"); }
  if (containsLine(lines, "$GNGGA") || containsLine(lines, "$GPGGA")) { score += 3; reasons.push_back("GGA"); }
  if (containsLine(lines, "$GNGSA") || containsLine(lines, "$GPGSA")) { score += 1; reasons.push_back("GSA"); }
  if (containsLine(lines, "$GPGSV") || containsLine(lines, "$GNGSV")) { score += 1; reasons.push_back("GSV"); }
  return {score, reasons};
}

DetectionResult findWtrtk980Device(const std::string& prefer, int baud, double probe_seconds, int min_score) {
  std::vector<std::string> candidates;
  std::set<std::string> seen;
  if (!prefer.empty() && access(prefer.c_str(), F_OK) == 0) {
    candidates.push_back(prefer);
    try { seen.insert(std::filesystem::weakly_canonical(prefer).string()); } catch (...) { seen.insert(prefer); }
  }
  for (const auto& device : listSerialDevices()) {
    std::string real;
    try { real = std::filesystem::weakly_canonical(device).string(); } catch (...) { real = device; }
    if (seen.count(real) == 0 && isCh341Candidate(device)) {
      candidates.push_back(device);
      seen.insert(real);
    }
  }

  DetectionResult best;
  for (const auto& candidate : candidates) {
    auto lines = probeLines(candidate, baud, probe_seconds);
    auto [score, reasons] = scoreWtrtk980Lines(lines);
    std::ostringstream checked;
    checked << candidate << " score=" << score << " reasons=";
    for (const auto& reason : reasons) checked << reason << ",";
    best.checked.push_back(checked.str());
    if (score > best.score) {
      best.device = candidate;
      best.score = score;
      best.reasons = reasons;
    }
  }
  if (best.score < min_score) best.device.clear();
  return best;
}

}  // namespace wtrtk980
