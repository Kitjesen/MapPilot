#include "lingtu/maps/build/process.hpp"

#include <cassert>
#include <filesystem>
#include <string>

int main() {
  lingtu::maps::ProcessRunOptions options;
  options.cwd = std::filesystem::temp_directory_path() /
      "lingtu_maps_process_missing_working_directory";

  std::error_code error;
  std::filesystem::remove_all(options.cwd, error);

  const auto result = lingtu::maps::RunShellCommand("exit 0", options);
  assert(result.launch_failed);
  assert(result.exit_code == -1);
  assert(!result.error.empty());
  return 0;
}
