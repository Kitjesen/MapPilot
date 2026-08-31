#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace lingtu::recording {

class RecordingCatalogError final : public std::runtime_error {
 public:
  RecordingCatalogError(std::string code, std::string message);

  const std::string &code() const noexcept;

 private:
  std::string code_;
};

struct RecordingManifestSnapshot {
  std::filesystem::path session_directory;
  std::string manifest_json;
  std::string session_id;
  std::string state;
  std::int64_t manager_process_id{-1};
  std::filesystem::file_time_type modified_at{};
};

struct RecordingCatalogSnapshot {
  std::filesystem::path root;
  // All validated sessions, newest first.  The selected session remains the
  // active session (or newest terminal session) for the status command.
  std::vector<RecordingManifestSnapshot> sessions;
  std::optional<RecordingManifestSnapshot> selected;
  std::uint64_t selected_size_bytes{0};
  bool selected_size_truncated{false};
  std::uint64_t disk_free_bytes{0};
  std::uint64_t disk_total_bytes{0};
};

bool recording_state_is_active(std::string_view state) noexcept;
bool recording_state_is_terminal(std::string_view state) noexcept;

RecordingManifestSnapshot read_recording_manifest(const std::filesystem::path &session_directory);

RecordingCatalogSnapshot inspect_recording_catalog(const std::filesystem::path &root,
                                                   std::size_t maximum_sessions = 4096,
                                                   std::size_t maximum_files = 100000);

}  // namespace lingtu::recording
