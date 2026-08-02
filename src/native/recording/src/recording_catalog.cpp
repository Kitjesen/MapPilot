#include "lingtu/recording/recording_catalog.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <iterator>
#include <limits>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace lingtu::recording {
namespace {

constexpr std::uintmax_t kMaximumManifestBytes = 4U * 1024U * 1024U;

[[noreturn]] void invalid_manifest(const std::filesystem::path &path, const std::string &reason) {
  throw RecordingCatalogError("recording_manifest_invalid",
                              "invalid recording manifest " + path.string() + ": " + reason);
}

std::size_t json_field_value(const std::string &json, std::string_view field,
                             const std::filesystem::path &path) {
  std::optional<std::size_t> found;
  std::size_t depth = 0;
  bool expect_key = false;
  for (std::size_t index = 0; index < json.size();) {
    const char value = json[index];
    if (value == '"') {
      std::string token;
      bool escaped = false;
      std::size_t cursor = index + 1;
      for (; cursor < json.size(); ++cursor) {
        const char character = json[cursor];
        if (escaped) {
          token.push_back(character);
          escaped = false;
        } else if (character == '\\') {
          escaped = true;
        } else if (character == '"') {
          break;
        } else {
          token.push_back(character);
        }
      }
      if (cursor == json.size()) {
        invalid_manifest(path, "unterminated JSON string");
      }
      if (depth == 1 && expect_key) {
        auto colon = cursor + 1;
        while (colon < json.size() && (json[colon] == ' ' || json[colon] == '\t' ||
                                       json[colon] == '\r' || json[colon] == '\n')) {
          ++colon;
        }
        if (colon == json.size() || json[colon] != ':') {
          invalid_manifest(path, "top-level key has no value");
        }
        auto position = colon + 1;
        while (position < json.size() && (json[position] == ' ' || json[position] == '\t' ||
                                          json[position] == '\r' || json[position] == '\n')) {
          ++position;
        }
        if (token == field) {
          if (found) {
            invalid_manifest(path, "top-level field is duplicated: " + std::string(field));
          }
          found = position;
        }
        expect_key = false;
      }
      index = cursor + 1;
      continue;
    }
    if (value == '{' || value == '[') {
      ++depth;
      if (depth == 1 && value == '{') {
        expect_key = true;
      }
    } else if (value == '}' || value == ']') {
      if (depth == 0) {
        invalid_manifest(path, "JSON nesting is malformed");
      }
      --depth;
    } else if (value == ',' && depth == 1) {
      expect_key = true;
    }
    ++index;
  }
  if (depth != 0) {
    invalid_manifest(path, "JSON nesting is incomplete");
  }
  if (!found) {
    invalid_manifest(path, "top-level field is missing: " + std::string(field));
  }
  return *found;
}

std::string json_string_field(const std::string &json, std::string_view field,
                              const std::filesystem::path &path) {
  const auto position = json_field_value(json, field, path);
  if (position >= json.size() || json[position] != '"') {
    invalid_manifest(path, "field is not a string: " + std::string(field));
  }
  std::string result;
  bool escaped = false;
  for (std::size_t index = position + 1; index < json.size(); ++index) {
    const char value = json[index];
    if (escaped) {
      switch (value) {
        case '"':
        case '\\':
        case '/':
          result.push_back(value);
          break;
        case 'b':
          result.push_back('\b');
          break;
        case 'f':
          result.push_back('\f');
          break;
        case 'n':
          result.push_back('\n');
          break;
        case 'r':
          result.push_back('\r');
          break;
        case 't':
          result.push_back('\t');
          break;
        default:
          invalid_manifest(path, "unsupported string escape in field: " + std::string(field));
      }
      escaped = false;
      continue;
    }
    if (value == '\\') {
      escaped = true;
      continue;
    }
    if (value == '"') {
      return result;
    }
    if (static_cast<unsigned char>(value) < 0x20U) {
      invalid_manifest(path, "control character in field: " + std::string(field));
    }
    result.push_back(value);
  }
  invalid_manifest(path, "unterminated string field: " + std::string(field));
}

std::int64_t json_integer_field(const std::string &json, std::string_view field,
                                const std::filesystem::path &path) {
  const auto position = json_field_value(json, field, path);
  std::int64_t value = 0;
  const auto result = std::from_chars(json.data() + position, json.data() + json.size(), value);
  if (result.ec != std::errc{}) {
    invalid_manifest(path, "field is not an integer: " + std::string(field));
  }
  auto cursor = result.ptr;
  while (cursor != json.data() + json.size() &&
         (*cursor == ' ' || *cursor == '\t' || *cursor == '\r' || *cursor == '\n')) {
    ++cursor;
  }
  if (cursor == json.data() + json.size() || (*cursor != ',' && *cursor != '}')) {
    invalid_manifest(path, "integer field is malformed: " + std::string(field));
  }
  return value;
}

std::filesystem::path normalized_path(const std::filesystem::path &input,
                                      const std::filesystem::path &error_path) {
  std::error_code error;
  const auto normalized = std::filesystem::weakly_canonical(input, error);
  if (error) {
    invalid_manifest(error_path, "cannot normalize session directory");
  }
  return normalized;
}

std::pair<std::uint64_t, bool> directory_size(const std::filesystem::path &root,
                                              std::size_t maximum_files) {
  std::uint64_t total = 0;
  std::size_t files = 0;
  std::error_code error;
  std::filesystem::recursive_directory_iterator iterator(
      root, std::filesystem::directory_options::skip_permission_denied, error);
  const std::filesystem::recursive_directory_iterator end;
  if (error) {
    return {0, true};
  }
  for (; iterator != end; iterator.increment(error)) {
    if (error) {
      return {total, true};
    }
    const auto status = iterator->symlink_status(error);
    if (error) {
      return {total, true};
    }
    if (std::filesystem::is_symlink(status)) {
      iterator.disable_recursion_pending();
      continue;
    }
    if (!std::filesystem::is_regular_file(status)) {
      continue;
    }
    if (++files > maximum_files) {
      return {total, true};
    }
    const auto size = iterator->file_size(error);
    if (error || size > std::numeric_limits<std::uint64_t>::max() - total) {
      return {total, true};
    }
    total += static_cast<std::uint64_t>(size);
  }
  return {total, false};
}

std::filesystem::path existing_storage_path(std::filesystem::path path) {
  std::error_code error;
  while (!path.empty() && !std::filesystem::exists(path, error) && path != path.parent_path()) {
    error.clear();
    path = path.parent_path();
  }
  return path.empty() ? std::filesystem::path(".") : path;
}

}  // namespace

RecordingCatalogError::RecordingCatalogError(std::string code, std::string message)
    : std::runtime_error(std::move(message)), code_(std::move(code)) {}

const std::string &RecordingCatalogError::code() const noexcept {
  return code_;
}

bool recording_state_is_active(std::string_view state) noexcept {
  return state == "preparing" || state == "recording" || state == "stopping";
}

bool recording_state_is_terminal(std::string_view state) noexcept {
  return state == "completed" || state == "failed";
}

RecordingManifestSnapshot read_recording_manifest(const std::filesystem::path &session_directory) {
  const auto path = session_directory / "session.json";
  std::error_code error;
  const auto status = std::filesystem::symlink_status(path, error);
  if (error || !std::filesystem::is_regular_file(status) || std::filesystem::is_symlink(status)) {
    invalid_manifest(path, "file is missing, unsafe, or not regular");
  }
  const auto size = std::filesystem::file_size(path, error);
  if (error || size == 0 || size > kMaximumManifestBytes) {
    invalid_manifest(path, "file size is outside the accepted range");
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    invalid_manifest(path, "file cannot be opened");
  }

  RecordingManifestSnapshot snapshot;
  snapshot.session_directory = normalized_path(session_directory, path);
  snapshot.manifest_json.assign(std::istreambuf_iterator<char>(input),
                                std::istreambuf_iterator<char>());
  if (json_integer_field(snapshot.manifest_json, "version", path) != 1) {
    invalid_manifest(path, "unsupported manifest version");
  }
  snapshot.session_id = json_string_field(snapshot.manifest_json, "session_id", path);
  snapshot.state = json_string_field(snapshot.manifest_json, "state", path);
  snapshot.manager_process_id = json_integer_field(snapshot.manifest_json, "manager_pid", path);
  if (snapshot.session_id.empty() ||
      (!recording_state_is_active(snapshot.state) &&
       !recording_state_is_terminal(snapshot.state)) ||
      snapshot.manager_process_id <= 1 ||
      snapshot.manager_process_id > std::numeric_limits<std::int32_t>::max()) {
    invalid_manifest(path, "identity or state is invalid");
  }
  const auto declared = json_string_field(snapshot.manifest_json, "session_directory", path);
  if (declared.empty() || normalized_path(declared, path) != snapshot.session_directory) {
    invalid_manifest(path, "session_directory does not match its catalog entry");
  }
  snapshot.modified_at = std::filesystem::last_write_time(path, error);
  if (error) {
    invalid_manifest(path, "modification time cannot be read");
  }
  return snapshot;
}

RecordingCatalogSnapshot inspect_recording_catalog(const std::filesystem::path &input_root,
                                                   std::size_t maximum_sessions,
                                                   std::size_t maximum_files) {
  RecordingCatalogSnapshot catalog;
  catalog.root = std::filesystem::absolute(input_root).lexically_normal();
  std::error_code error;
  const auto root_status = std::filesystem::symlink_status(catalog.root, error);
  if (!error && std::filesystem::exists(root_status)) {
    if (std::filesystem::is_symlink(root_status) || !std::filesystem::is_directory(root_status)) {
      throw RecordingCatalogError("recording_catalog_unsafe",
                                  "recording root is not a safe directory: " +
                                      catalog.root.string());
    }
  } else if (error && error != std::errc::no_such_file_or_directory) {
    throw RecordingCatalogError("recording_catalog_unreadable",
                                "recording root cannot be inspected: " + catalog.root.string());
  }

  std::vector<RecordingManifestSnapshot> sessions;
  if (std::filesystem::is_directory(catalog.root, error) && !error) {
    std::size_t entries = 0;
    for (std::filesystem::directory_iterator iterator(catalog.root, error), end;
         !error && iterator != end; iterator.increment(error)) {
      if (++entries > maximum_sessions) {
        throw RecordingCatalogError("recording_catalog_too_large",
                                    "recording catalog exceeds its hard session limit");
      }
      const auto status = iterator->symlink_status(error);
      if (error) {
        break;
      }
      if (std::filesystem::is_symlink(status)) {
        throw RecordingCatalogError("recording_catalog_unsafe",
                                    "recording catalog contains a symbolic link: " +
                                        iterator->path().string());
      }
      if (!std::filesystem::is_directory(status)) {
        continue;
      }
      const auto manifest_path = iterator->path() / "session.json";
      const auto manifest_status = std::filesystem::symlink_status(manifest_path, error);
      if (error == std::errc::no_such_file_or_directory) {
        error.clear();
        continue;
      }
      if (error) {
        break;
      }
      if (!std::filesystem::exists(manifest_status)) {
        continue;
      }
      sessions.push_back(read_recording_manifest(iterator->path()));
    }
    if (error) {
      throw RecordingCatalogError("recording_catalog_unreadable",
                                  "recording catalog traversal failed: " + error.message());
    }
  }

  std::sort(sessions.begin(), sessions.end(), [](const auto &left, const auto &right) {
    return left.modified_at > right.modified_at;
  });
  std::vector<const RecordingManifestSnapshot *> active;
  for (const auto &session : sessions) {
    if (recording_state_is_active(session.state)) {
      active.push_back(&session);
    }
  }
  if (active.size() > 1) {
    throw RecordingCatalogError("multiple_recordings_active",
                                "recording catalog contains more than one active session");
  }
  if (!active.empty()) {
    catalog.selected = *active.front();
  } else if (!sessions.empty()) {
    catalog.selected = sessions.front();
  }

  if (catalog.selected) {
    const auto [size, truncated] =
        directory_size(catalog.selected->session_directory, maximum_files);
    catalog.selected_size_bytes = size;
    catalog.selected_size_truncated = truncated;
  }
  const auto storage = existing_storage_path(catalog.root);
  const auto space = std::filesystem::space(storage, error);
  if (!error) {
    catalog.disk_free_bytes = space.available;
    catalog.disk_total_bytes = space.capacity;
  }
  return catalog;
}

}  // namespace lingtu::recording
