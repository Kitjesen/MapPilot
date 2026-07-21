#pragma once

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#if defined(_WIN32)
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace lingtu::nav::endpoint {

// Persists the software E-stop across endpoint process restarts. An existing
// marker always means latched, even if a crash left it empty.
class EstopLatchStore {
 public:
  explicit EstopLatchStore(std::string path) : path_(std::move(path)) {}

  std::optional<std::string> load() const {
    if (path_.empty()) {
      return std::nullopt;
    }
    std::error_code error;
    const bool exists = std::filesystem::exists(path_, error);
    if (error) {
      return std::string("estop_latch_state_unreadable");
    }
    if (!exists) {
      return std::nullopt;
    }
    std::ifstream input(path_);
    std::string reason;
    if (input) {
      std::getline(input, reason);
    }
    return reason.empty() ? std::optional<std::string>("persisted_estop")
                          : std::optional<std::string>(reason);
  }

  bool persist(const std::string& reason) const {
    if (path_.empty()) {
      return true;
    }
    std::error_code error;
    const auto parent = path_.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent, error);
      if (error) {
        return false;
      }
    }
    const std::string path_text = path_.string();
    std::FILE* output = std::fopen(path_text.c_str(), "wb");
    if (output == nullptr) {
      return false;
    }
    std::string line = reason.empty() ? "estop" : reason;
    const auto newline = line.find_first_of("\r\n");
    if (newline != std::string::npos) {
      line.resize(newline);
    }
    line.push_back('\n');
    const bool written =
        std::fwrite(line.data(), 1, line.size(), output) == line.size();
    const bool synced = written && syncFile(output);
    const bool closed = std::fclose(output) == 0;
    return synced && closed && syncParentDirectory();
  }

  bool clear() const {
    if (path_.empty()) {
      return true;
    }
    std::error_code error;
    std::filesystem::remove(path_, error);
    if (error) {
      return false;
    }
    // Once deletion succeeds, treat the operator's explicit clear as
    // committed for this boot. A failed directory fsync may make the old
    // marker reappear after power loss, which fails safe by re-latching; it
    // must not leave memory latched while the marker is already absent.
    (void)syncParentDirectory();
    return true;
  }

 private:
  static bool syncFile(std::FILE* file) {
    if (std::fflush(file) != 0) {
      return false;
    }
#if defined(_WIN32)
    return ::_commit(::_fileno(file)) == 0;
#else
    return ::fsync(::fileno(file)) == 0;
#endif
  }

  bool syncParentDirectory() const {
#if defined(_WIN32)
    // FlushFileBuffers on a directory requires a platform-specific handle;
    // product deployment is Linux and is covered by the fsync branch below.
    return true;
#else
    const auto parent = path_.parent_path().empty()
                            ? std::filesystem::path(".")
                            : path_.parent_path();
    const int descriptor = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY);
    if (descriptor < 0) {
      return false;
    }
    const bool synced = ::fsync(descriptor) == 0;
    ::close(descriptor);
    return synced;
#endif
  }

  std::filesystem::path path_;
};

}  // namespace lingtu::nav::endpoint
