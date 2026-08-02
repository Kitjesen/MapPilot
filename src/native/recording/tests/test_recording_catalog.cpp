#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "lingtu/recording/recording_catalog.hpp"

namespace {

namespace recording = lingtu::recording;

void require(bool condition, const char *message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    path = std::filesystem::temp_directory_path() /
           ("lingtu-recording-catalog-" + std::to_string(suffix));
    std::filesystem::create_directories(path);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }

  std::filesystem::path path;
};

void write_manifest(const std::filesystem::path &session, const std::string &state,
                    std::int64_t manager_pid, const std::filesystem::path *declared = nullptr) {
  std::filesystem::create_directories(session);
  const auto declared_path =
      std::filesystem::absolute(declared == nullptr ? session : *declared).lexically_normal();
  std::ofstream output(session / "session.json", std::ios::binary);
  output << "{\"version\":1,\"session_id\":\"" << session.filename().string() << "\",\"state\":\""
         << state << "\",\"session_directory\":\"" << declared_path.generic_string()
         << "\",\"context\":{},\"manager_pid\":" << manager_pid
         << ",\"created_at_unix_ns\":1,\"started_at_unix_ns\":2,"
            "\"ended_at_unix_ns\":3,\"children\":[]}\n";
}

void test_empty_catalog_is_idle() {
  TemporaryDirectory temporary;
  const auto catalog = recording::inspect_recording_catalog(temporary.path);
  require(!catalog.selected.has_value(), "empty catalog selected a session");
  require(catalog.disk_total_bytes > 0, "empty catalog omitted disk capacity");
}

void test_active_session_wins_over_newer_history() {
  TemporaryDirectory temporary;
  const auto active = temporary.path / "active";
  const auto completed = temporary.path / "completed";
  write_manifest(active, "recording", 1001);
  write_manifest(completed, "completed", 1002);
  std::filesystem::last_write_time(active / "session.json",
                                   std::filesystem::file_time_type::clock::now() -
                                       std::chrono::seconds(10));

  const auto catalog = recording::inspect_recording_catalog(temporary.path);
  require(catalog.selected.has_value(), "catalog omitted active session");
  require(catalog.selected->session_id == "active", "history replaced active session");
}

void test_multiple_active_sessions_fail_closed() {
  TemporaryDirectory temporary;
  write_manifest(temporary.path / "one", "recording", 1001);
  write_manifest(temporary.path / "two", "stopping", 1002);
  try {
    static_cast<void>(recording::inspect_recording_catalog(temporary.path));
    throw std::runtime_error("catalog accepted multiple active sessions");
  } catch (const recording::RecordingCatalogError &error) {
    require(error.code() == "multiple_recordings_active", "wrong multiple-session error code");
  }
}

void test_manifest_cannot_redirect_session_identity() {
  TemporaryDirectory temporary;
  const auto outside = temporary.path.parent_path() / "outside";
  write_manifest(temporary.path / "forged", "completed", 1001, &outside);
  try {
    static_cast<void>(recording::inspect_recording_catalog(temporary.path));
    throw std::runtime_error("catalog accepted redirected session identity");
  } catch (const recording::RecordingCatalogError &error) {
    require(error.code() == "recording_manifest_invalid", "wrong manifest error code");
  }
}

void test_manifest_rejects_pid_outside_linux_process_range() {
  TemporaryDirectory temporary;
  write_manifest(temporary.path / "invalid-pid", "recording", 2147483648LL);
  try {
    static_cast<void>(recording::inspect_recording_catalog(temporary.path));
    throw std::runtime_error("catalog accepted an overflowing manager PID");
  } catch (const recording::RecordingCatalogError &error) {
    require(error.code() == "recording_manifest_invalid", "wrong PID error code");
  }
}

void test_size_scan_is_hard_bounded() {
  TemporaryDirectory temporary;
  const auto session = temporary.path / "completed";
  write_manifest(session, "completed", 1001);
  std::ofstream(session / "one.bin") << "one";
  std::ofstream(session / "two.bin") << "two";

  const auto catalog = recording::inspect_recording_catalog(temporary.path, 16, 1);
  require(catalog.selected_size_truncated, "catalog did not report a bounded size scan");
}

}  // namespace

int main() {
  try {
    test_empty_catalog_is_idle();
    test_active_session_wins_over_newer_history();
    test_multiple_active_sessions_fail_closed();
    test_manifest_cannot_redirect_session_identity();
    test_manifest_rejects_pid_outside_linux_process_range();
    test_size_scan_is_hard_bounded();
    std::cout << "native recording catalog tests passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "native recording catalog test failed: " << error.what() << '\n';
    return 1;
  }
}
