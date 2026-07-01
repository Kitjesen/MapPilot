#pragma once

#include <string>
#include <utility>
#include <vector>

namespace wtrtk980 {

struct DetectionResult {
  std::string device;
  int score = -1;
  std::vector<std::string> reasons;
  std::vector<std::string> checked;
};

std::vector<std::string> listSerialDevices();
bool isCh341Candidate(const std::string& device);
std::vector<std::string> probeLines(const std::string& device, int baud = 115200, double seconds = 3.0);
std::pair<int, std::vector<std::string>> scoreWtrtk980Lines(const std::vector<std::string>& lines);
DetectionResult findWtrtk980Device(
    const std::string& prefer = "/dev/wtrtk980",
    int baud = 115200,
    double probe_seconds = 3.0,
    int min_score = 6);

}  // namespace wtrtk980
