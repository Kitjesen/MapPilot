#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>

namespace lingtu::maps {

struct ProcessRunOptions {
  std::filesystem::path cwd;
  double timeout_sec{60.0};
  std::uint64_t max_output_bytes{65536U};
  std::function<bool()> cancel_requested;
};

struct ProcessRunResult {
  int exit_code{-1};
  bool timed_out{false};
  bool cancelled{false};
  bool launch_failed{false};
  bool stdout_truncated{false};
  bool stderr_truncated{false};
  std::string stdout_text;
  std::string stderr_text;
  std::string error;
};

ProcessRunResult RunShellCommand(
    const std::string& command,
    const ProcessRunOptions& options);

}  // namespace lingtu::maps
